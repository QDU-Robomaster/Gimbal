#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 双 yaw 联合 LQR 云台，支持小轴平滑回位 / Coupled dual-yaw gimbal with LQR and gradual small-axis centering
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "DoubleYaw.hpp"
#include "Motor.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

static constexpr double GIMBAL_PI = 3.14159265358979323846;
static constexpr float GIMBAL_MAX_SPEED =
    static_cast<float>(2.0 * GIMBAL_PI) * 2.0f;
enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_AUTOPATROL,
  SET_MODE_LOW_SENSITIVITY
};

// Bounds use the signed small-yaw encoder position in radians.
struct YawLimit {
  float min_position = 0;
  float max_position = 0;
};

struct GimbalSmallYawFeedforward {
  float J = 0;
  float B = 0;
  float Fc = 0.063f;
};

class Gimbal {
 public:
  /**
   * @brief 构造函数初始化数据成员
   *
   * @param cmd 命令模块实例
   * @param task_stack_depth 任务堆栈深度
   * @param pid_pit_angle Pitch轴角度环PID参数
   * @param pid_pit_omega Pitch轴角速度环PID参数
   * @param motor_pit Pitch轴电机指针，nullptr 表示不使用 Pitch
   * @param motor_yaw Yaw轴电机指针
   * @param pit_max_angle Pitch轴最大角度
   * @param pit_min_angle Pitch轴最小角度
   * @param pit_lc Pitch质心距离(m)(距离水平向上为+)*Pitch质心重力(N)
   * @param pit_theta Pitch质心与重力轴线夹角(rad 极性自己猜)
   * @param j_pit Pitch轴转动惯量
   * @param pit_zero Pitch轴零点
   * @param yaw_zero Yaw轴零点
   * @param reverse_flag Pitch轴反转标志
   */
  Gimbal(
      CMD& cmd, uint32_t task_stack_depth,
      LibXR::PID<float>::Param pid_pit_angle,
      LibXR::PID<float>::Param pid_pit_omega, Motor* motor_pit,
      Motor* motor_yaw, Motor* motor_yaw2, float pit_max_angle,
      float pit_min_angle, float pit_lc, float pit_theta, float j_pit,
      float pit_zero, float yaw_zero, float yaw2_zero, float patrol_range,
      float patrol_omega, bool reverse_flag,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
      YawLimit yaw_limit = {}, DoubleYaw::Param dual_yaw = {},
      GimbalSmallYawFeedforward small_yaw_feedforward = {})
      : cmd_(cmd),
        pid_pit_angle_(pid_pit_angle),
        pid_pit_omega_(pid_pit_omega),
        motor_yaw_(motor_yaw),
        motor_yaw2_(motor_yaw2),
        motor_pit_(motor_pit),
        pit_max_angle_(pit_max_angle),
        pit_min_angle_(pit_min_angle),
        pit_lc_(pit_lc),
        pit_theta_(pit_theta),
        j_pit_(j_pit),
        pit_zero_(pit_zero),
        yaw_zero_(yaw_zero),
        yaw2_zero_(yaw2_zero),
        patrol_range_(patrol_range),
        patrol_omega_(patrol_omega),
        reverse_flag_(reverse_flag ? 1.0f : -1.0f),
        yaw_limit_(yaw_limit),
        dual_yaw_lqr_(dual_yaw),
        small_yaw_feedforward_(small_yaw_feedforward) {
    dbg_yaw_min_position_ = yaw_limit.min_position;
    dbg_yaw_max_position_ = yaw_limit.max_position;
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->current_mode_ = static_cast<GimbalEvent>(event_id);
        },
        this);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON),
                           callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_LOW_SENSITIVITY), callback);
    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   thread_priority);
  }

  /**
   * @brief 线程函数
   *
   * @param gimbal Gimbal实例指针
   */
  static void ThreadFunc(Gimbal* gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "bmi088_gyro");
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    gimbal->last_update_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_mode = GimbalEvent::SET_MODE_RELAX;

    while (true) {
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        gimbal->euler_ = euler_suber.GetData();
        gimbal->euler_.Pitch() *= -1.0f;
        gimbal->dbg_imu_yaw_ = gimbal->euler_.Yaw();
        gimbal->last_euler_sample_us_ = LibXR::Timebase::GetMicroseconds();
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gimbal->gyro_data_.y() *= -1.0f;
        gimbal->last_gyro_sample_us_ = LibXR::Timebase::GetMicroseconds();
        gyro_suber.StartWaiting();
      }

      gimbal->Update();
      GimbalEvent mode = gimbal->current_mode_;
      if (mode != GimbalEvent::SET_MODE_COMMON &&
          mode != GimbalEvent::SET_MODE_RELAX && !gimbal->cmd_.Online()) {
        mode = GimbalEvent::SET_MODE_RELAX;
        gimbal->current_mode_ = mode;
      }
      if (mode != last_mode) {
        gimbal->ResetTargets();
        last_mode = mode;
      }
      gimbal->ParseCMD(mode);
      gimbal->Control(mode);
      LibXR::Thread::Sleep(2);
    }
  };

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    motor_yaw_->Update();
    motor_yaw2_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_yaw2_feedback_ = motor_yaw2_->GetFeedback();

    dual_yaw_lqr_.UpdateBigFeedback(
        motor_yaw2_feedback_.position,
        FeedbackSampleTime(motor_yaw2_feedback_, 0));
    if (motor_pit_ != nullptr) {
      motor_pit_->Update();
      motor_pit_feedback_ = motor_pit_->GetFeedback();
    }

    const auto NOW = LibXR::Timebase::GetMicroseconds();
    dt_ = (NOW - last_update_time_).ToSecondf();
    last_update_time_ = NOW;

    if (motor_pit_ != nullptr) {
      abs_angle_pit_ = motor_pit_feedback_.abs_angle - pit_zero_;
      topic_pit_angle_.Publish(abs_angle_pit_);
    }
    abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;
    abs_angle_yaw2_ = motor_yaw2_feedback_.abs_angle - yaw2_zero_;

    topic_yaw_angle_.Publish(abs_angle_yaw_);
    topic_yaw2_angle_.Publish(abs_angle_yaw2_);
  }

  /**
   * @brief 解析云台控制命令
   */
  void ParseCMD(GimbalEvent mode) {
    if (mode == GimbalEvent::SET_MODE_COMMON) {
      target_yaw_cmd_ = static_cast<float>(dbg_imu_yaw_target_);
      target_yaw_dot_ = target_yaw_ddot_ = 0;
      return;
    }
    if (mode == GimbalEvent::SET_MODE_RELAX) return;

    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      const float SPEED = mode == GimbalEvent::SET_MODE_LOW_SENSITIVITY
                              ? GIMBAL_MAX_SPEED * 0.1f
                              : GIMBAL_MAX_SPEED;
      target_yaw_dot_ = cmd_data_.yaw * SPEED;
      target_yaw_cmd_ += target_yaw_dot_ * dt_;
      target_pit_cmd_ += cmd_data_.pit * SPEED * dt_;
      target_yaw_ddot_ = target_pit_dot_ = target_pit_ddot_ = 0;
    } else if (cmd_.GetAIGimbalStatus()) {
      target_yaw_cmd_ = cmd_data_.yaw;
      target_yaw_dot_ = cmd_data_.yaw_dot;
      target_yaw_ddot_ = cmd_data_.yaw_ddot;
      target_pit_cmd_ = cmd_data_.pit;
      target_pit_dot_ = cmd_data_.pit_dot;
      target_pit_ddot_ = cmd_data_.pit_ddot;
    } else if (mode == GimbalEvent::SET_MODE_AUTOPATROL) {
      target_pit_cmd_ -=
          patrol_range_ * (2 / GIMBAL_PI) *
          asin(sin(patrol_omega_ *
                   (LibXR::Timebase::GetMilliseconds() - patrol_start_time_))) /
          1000.0f;
      target_yaw_cmd_ += dt_;
      target_yaw_dot_ = 1;
      target_yaw_ddot_ = 0;
    } else {
      target_yaw_cmd_ -= cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED;
      target_pit_cmd_ += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED;
      target_yaw_dot_ = target_yaw_ddot_ = 0;
      target_pit_dot_ = target_pit_ddot_ = 0;
    }
  }

  void Control(GimbalEvent mode) {
    const float MIN_POSITION = dbg_yaw_min_position_;
    const float MAX_POSITION = dbg_yaw_max_position_;
    dbg_yaw_limit_update_valid_ = std::isfinite(MIN_POSITION) &&
                                  std::isfinite(MAX_POSITION) &&
                                  MIN_POSITION < MAX_POSITION;
    if (dbg_yaw_limit_update_valid_) {
      yaw_limit_ = {MIN_POSITION, MAX_POSITION};
    }
    if (mode == GimbalEvent::SET_MODE_RELAX) {
      dual_yaw_lqr_.Reset();
      dual_yaw_last_enable_us_ = 0;
      dbg_yaw_e_ = dbg_yaw2_e_ = dbg_yaw_u_ = dbg_yaw2_u_ = 0;
      motor_yaw_->Relax();
      motor_yaw2_->Relax();
      if (motor_pit_) motor_pit_->Relax();
      return;
    }

    const uint64_t NOW = LibXR::Timebase::GetMicroseconds();
    const uint64_t SMALL_SAMPLE_US = FeedbackSampleTime(motor_yaw_feedback_, 0);
    const uint64_t BIG_SAMPLE_US = FeedbackSampleTime(motor_yaw2_feedback_, 0);
    const bool INPUTS_READY =
        last_euler_sample_us_ && NOW - last_euler_sample_us_ <= 50000 &&
        last_gyro_sample_us_ && NOW - last_gyro_sample_us_ <= 20000 &&
        SMALL_SAMPLE_US && NOW - SMALL_SAMPLE_US <= 50000 &&
        motor_yaw_feedback_.state == 1 &&
        motor_yaw_feedback_.temp < DUAL_YAW_TEMP_LIMIT &&
        motor_yaw2_feedback_.temp < DUAL_YAW_TEMP_LIMIT;
    const bool BIG_ONLINE = BIG_SAMPLE_US && NOW - BIG_SAMPLE_US <= 50000;
    const uint8_t BIG_STATE = motor_yaw2_feedback_.state;
    if (!INPUTS_READY || !BIG_ONLINE || BIG_STATE != 1 || !SolveDualYaw()) {
      dual_yaw_lqr_.Suspend();
      dbg_yaw_e_ = dbg_yaw2_e_ = dbg_yaw_u_ = dbg_yaw2_u_ = 0;
      motor_yaw_->Relax();
      if (!INPUTS_READY || BIG_STATE > 1) {
        motor_yaw2_->Relax();
      } else if (BIG_ONLINE && BIG_STATE == 1) {
        // Ask the enabled DM for feedback while tracking is paused.
        motor_yaw2_->Control(
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = 0});
      } else if (!dual_yaw_last_enable_us_ ||
                 NOW - dual_yaw_last_enable_us_ >= 200000) {
        motor_yaw2_->Enable();
        dual_yaw_last_enable_us_ = NOW;
      }
      return;
    }
    motor_yaw_->Control(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = dbg_yaw_u_});
    motor_yaw2_->Control(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = dbg_yaw2_u_});

    if (motor_pit_) {
      PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
                 pit_max_angle_, pit_min_angle_, reverse_flag_);
      if (motor_pit_feedback_.state == 0) {
        motor_pit_->Enable();
      } else if (motor_pit_feedback_.state == 1) {
        motor_pit_->Control(
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = SolvePitch()});
      } else {
        motor_pit_->ClearError();
      }
    }
  }

  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  CMD& cmd_;
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_pit_omega_;
  Motor* motor_yaw_;
  Motor* motor_yaw2_;
  Motor* motor_pit_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_yaw2_feedback_;
  Motor::Feedback motor_pit_feedback_;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  uint64_t last_gyro_sample_us_ = 0;
  uint64_t last_euler_sample_us_ = 0;
  LibXR::EulerAngle<float> euler_;

  float dbg_yaw_e_ = 0.0f;
  float dbg_yaw_u_ = 0.0f;
  float dbg_yaw2_e_ = 0.0f;
  float dbg_yaw2_u_ = 0.0f;

  // Ozone 可直接修改；初值来自构造参数，复位后恢复配置值。
  volatile float dbg_yaw_min_position_ = 0.0f;
  volatile float dbg_yaw_max_position_ = 0.0f;
  // min >= max 或非有限值时拒绝更新，继续使用上一组有效限位。
  volatile bool dbg_yaw_limit_update_valid_ = true;

  LibXR::Event gimbal_event_;
  volatile GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_yaw2_angle_ =
      LibXR::Topic::CreateTopic<float>("yaw2motor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");

  float pit_max_angle_ = 0.0f;
  float pit_min_angle_ = 0.0f;
  float pit_lc_ = 0.0f;
  float pit_theta_ = 0.0f;
  float target_yaw_dot_ = 0.0f;
  float target_yaw_ddot_ = 0.0f;
  float target_pit_dot_ = 0.0f;
  float target_pit_ddot_ = 0.0f;
  float j_pit_ = 0.0f;
  LibXR::CycleValue<float> pit_zero_ = 0.0f;
  LibXR::CycleValue<float> yaw_zero_ = 0.0f;
  LibXR::CycleValue<float> yaw2_zero_ = 0.0f;
  float patrol_range_ = 0.0f;
  float patrol_omega_ = 0.0f;
  float target_pit_cmd_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float abs_angle_yaw_ = 0.0f;
  float abs_angle_yaw2_ = 0.0f;
  float abs_angle_pit_ = 0.0f;
  float last_pit_omega_ = 0.0f;
  float reverse_flag_ = 1.0f;
  LibXR::MillisecondTimestamp patrol_start_time_{};
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_update_time_;
  YawLimit yaw_limit_;
  // Keep the Ozone paths for dual-yaw tracking and diagnostics.
  DoubleYaw dual_yaw_lqr_;
  GimbalSmallYawFeedforward small_yaw_feedforward_;
  volatile float dbg_imu_yaw_ = 0;
  volatile float dbg_imu_yaw_target_ = 0;
  static constexpr float DUAL_YAW_TEMP_LIMIT = 75.0f;
  uint64_t dual_yaw_last_enable_us_ = 0;
  LibXR::Thread thread_;

  /**
   * @brief Pitch轴角度限位
   *
   * @param target_pit 目标Pitch角度
   * @param now_eulr_angle 当前Pitch欧拉角
   * @param now_motor_angle 当前Pitch电机角度
   * @param motor_max 电机最大角度
   * @param motor_min 电机最小角度
   * @param sign 方向符号
   */
  void PitchLimit(float& target_pit, float now_eulr_angle,
                  float now_motor_angle, float motor_max, float motor_min,
                  float sign) {
    if ((motor_max == 0.0f) && (motor_min == 0.0f)) {
      return;
    };

    LibXR::CycleValue<float> cycle_motor_min(motor_min);
    LibXR::CycleValue<float> cycle_motor_max(motor_max);

    float diff_min = cycle_motor_min - now_motor_angle;
    float diff_max = cycle_motor_max - now_motor_angle;
    float pitch_bound_0 = now_eulr_angle + diff_min / sign;
    float pitch_bound_1 = now_eulr_angle + diff_max / sign;

    float upper_bound = std::max(pitch_bound_0, pitch_bound_1);
    float lower_bound = std::min(pitch_bound_0, pitch_bound_1);
    target_pit = std::clamp(target_pit, lower_bound, upper_bound);
  }

  float SolvePitch() {
    const float ERROR = target_pit_cmd_ - euler_.Pitch();
    const float TARGET_OMEGA =
        pid_pit_angle_.Calculate(ERROR, 0.0f, dt_) + target_pit_dot_;
    const float INERTIA_FF =
        j_pit_ * ((TARGET_OMEGA - last_pit_omega_) / dt_ + target_pit_ddot_);
    const float GRAVITY_FF = -pit_lc_ * sinf(euler_.Pitch() + pit_theta_);
    const float FEEDBACK =
        pid_pit_omega_.Calculate(TARGET_OMEGA, gyro_data_.y(), dt_);
    last_pit_omega_ = TARGET_OMEGA;
    return FEEDBACK + INERTIA_FF + GRAVITY_FF;
  }

  template <typename Feedback>
  static auto FeedbackSampleTime(const Feedback& feedback, int)
      -> decltype(feedback.sample_time_us) {
    return feedback.sample_time_us;
  }

  // A driver without a decode timestamp cannot provide fresh yaw feedback.
  static uint64_t FeedbackSampleTime(const Motor::Feedback&, long) { return 0; }

  bool SolveDualYaw() {
    const auto& FF = small_yaw_feedforward_;
    for (const float VALUE : {FF.J, FF.B, FF.Fc}) {
      if (!std::isfinite(VALUE) || VALUE < 0) return false;
    }

    // Gimbal stores negated gyro Y and pitch. Recover raw body Y for ZYX yaw
    // rate.
    const float ROLL = euler_.Roll();
    const DoubleYaw::Input INPUT{
        .aim_error =
            std::remainder(static_cast<float>(target_yaw_cmd_) - euler_.Yaw(),
                           static_cast<float>(2.0 * GIMBAL_PI)),
        .small_position = motor_yaw_feedback_.position,
        .aim_rate = (std::cos(ROLL) * gyro_data_.z() -
                     std::sin(ROLL) * gyro_data_.y()) /
                    std::cos(euler_.Pitch()),
        .min_position = yaw_limit_.min_position,
        .max_position = yaw_limit_.max_position,
        .target_rate = target_yaw_dot_,
        .target_accel = target_yaw_ddot_,
        .dt = dt_,
    };
    const auto SMALL_YAW_FF = [&FF](const DoubleYaw::SmallYawReference& ref) {
      return FF.J * ref.aim_accel + FF.B * ref.rate +
             FF.Fc * std::tanh((ref.rate + 3 * ref.angle_error) / 0.03f);
    };
    if (!dual_yaw_lqr_.Calculate(INPUT, SMALL_YAW_FF)) return false;

    dbg_yaw_e_ = dual_yaw_lqr_.aim_error_;
    dbg_yaw2_e_ = dual_yaw_lqr_.big_error_;
    dbg_yaw_u_ = dual_yaw_lqr_.small_torque_;
    dbg_yaw2_u_ =
        dual_yaw_lqr_.param_.big_direction * dual_yaw_lqr_.big_torque_;
    return true;
  }

  void ResetTargets() {
    target_pit_cmd_ = euler_.Pitch();
    target_yaw_cmd_ = euler_.Yaw();
    target_yaw_dot_ = target_yaw_ddot_ = 0;
    target_pit_dot_ = target_pit_ddot_ = 0;
    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    last_pit_omega_ = 0;
    patrol_start_time_ = LibXR::Timebase::GetMilliseconds();
  }
};
