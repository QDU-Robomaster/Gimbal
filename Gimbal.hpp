#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_yaw_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_yaw_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_pit_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_pit_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_pitch: '@&motor_pit'
  - motor_yaw: '@&motor_yaw'
  - pit_max_angle: 0.0
  - pit_min_angle: 0.0
  - j_pit: 0.0
  - j_yaw: 0.0
  - pit_zero: 0.0
  - yaw_zero: 0.0
  - pit_reverse_flag: false
template_args: []
required_hardware: []
depends:
  - qdu-future/CMD
  - qdu-future/Motor
  - qdu-future/BMI088
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(M_2PI) * 1.5f;
enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
};

class Gimbal : public LibXR::Application {
 public:
  /**
   * @brief 构造函数初始化数据成员
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
   * @param task_stack_depth 任务堆栈深度
   * @param pid_yaw_angle_param   Yaw轴角度环PID参数
   * @param pid_pit_angle_param Pitch轴角度环PID参数
   * @param pid_yaw_omega_param   Yaw轴角速度环PID参数
   * @param pid_pit_omega_param Pitch轴角速度环PID参数
   */
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
         uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
         LibXR::PID<float>::Param pid_yaw_omega,
         LibXR::PID<float>::Param pid_pit_angle,
         LibXR::PID<float>::Param pid_pit_omega, Motor *motor_pit,
         Motor *motor_yaw, float pit_max_angle, float pit_min_angle,
         float j_pit, float j_yaw, float pit_zero, float yaw_zero,
         bool reverse_flag)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_omega_(pid_yaw_omega),
        pid_pit_angle_(pid_pit_angle),
        pid_pit_omega_(pid_pit_omega),
        motor_yaw_(motor_yaw),
        motor_pit_(motor_pit),
        pit_max_angle_(pit_max_angle),
        pit_min_angle_(pit_min_angle),
        j_pit_(j_pit),
        j_yaw_(j_yaw),
        pit_zero_(pit_zero),
        yaw_zero_(yaw_zero),
        reverse_flag_(reverse_flag ? -1.0f : 1.0f) {
    UNUSED(hw);
    UNUSED(app);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->SetMode(static_cast<GimbalEvent>(event_id));
        },
        this);

    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd_.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON),
                           callback);
  };

  static void ThreadFunc(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "bmi088_gyro");
    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (true) {
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        gimbal->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gyro_suber.StartWaiting();
      }

      gimbal->Update();
      gimbal->ParseCMD();
      gimbal->Control();
      LibXR::Thread::SleepUntil(last_time, 2);
    }
  };

  void Update() {
    motor_yaw_->Update();
    motor_pit_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    /*神秘dt_实现*/
    if (dt_ < 0.001 or dt_ > 0.003) {
      dt_ = 0.002f;
    }

    abs_angle_pit_ = motor_pit_feedback_.abs_angle - pit_zero_;
    abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;

    topic_yaw_angle_.Publish(abs_angle_yaw_);
    topic_pit_angle_.Publish(abs_angle_pit_);
  }

  void ParseCMD() {
    if (current_mode_ != GimbalEvent::SET_MODE_COMMON) {
      return;
    };
    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      target_yaw_cmd_ += cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
    } else {
      if (cmd_.GetAIGimbalStatus()) {
        target_yaw_cmd_ = cmd_data_.yaw;
        target_pit_cmd_ = cmd_data_.pit;
      } else {
        target_yaw_cmd_ += cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      }
    }
  }

  void Control() {
    float out_pit = 0.0f;
    float out_yaw = 0.0f;
    PitchLimit(target_pit_cmd_, euler_.Pitch(), abs_angle_pit_,
               pit_max_angle_, pit_min_angle_, reverse_flag_);
    Solve(out_pit, out_yaw, target_pit_cmd_, target_yaw_cmd_, dt_);
    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_yaw});
    auto pit_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_pit});

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      motor_yaw_->Relax();
      motor_pit_->Relax();
      return;
    }

    auto motor_control = [&](Motor *motor, const Motor::Feedback &fb,
                              const Motor::MotorCmd &cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 and fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_pit_, motor_pit_feedback_, pit_motor_cmd);
    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  void OnMonitor() override {}

 private:
  CMD &cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_pit_omega_;

  Motor *motor_yaw_;
  Motor *motor_pit_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");

  float pit_max_angle_ = 0.0f;
  float pit_min_angle_ = 0.0f;
  float j_pit_ = 0.0f;
  float j_yaw_ = 0.0f;
  LibXR::CycleValue<float> pit_zero_ = 0.0f;
  LibXR::CycleValue<float> yaw_zero_ = 0.0f;
  LibXR::CycleValue<float> target_pit_cmd_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float abs_angle_yaw_ = 0.0f;
  float abs_angle_pit_ = 0.0f;
  float last_pit_omega_ = 0.0f;
  float last_yaw_omega_ = 0.0f;
  float reverse_flag_ = 1.0f;

  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;

  LibXR::Thread thread_;

  /*----------工具函数--------------------------------*/

  void PitchLimit(LibXR::CycleValue<float> &target_pit, float now_eulr_angle,
                  float now_motor_angle, float motor_max, float motor_min,
                  float sign) {
    if ((motor_max == 0.0f) && (motor_min == 0.0f)) {
      return;
    };
    float pitch_bound_0 = now_eulr_angle + (motor_min - now_motor_angle) / sign;
    float pitch_bound_1 = now_eulr_angle + (motor_max - now_motor_angle) / sign;

    float upper_bound = std::max(pitch_bound_0, pitch_bound_1);
    float lower_bound = std::min(pitch_bound_0, pitch_bound_1);
    target_pit =
        std::clamp(static_cast<float>(target_pit), lower_bound, upper_bound);
  }

  void Solve(float &pit_output, float &yaw_output, float target_pit_angle,
             float target_yaw_angle, float dt_) {
    float target_pit_omega =
        pid_pit_angle_.Calculate(target_pit_angle, euler_.Pitch(), dt_);
    float ff_pit = JFeedforward(target_pit_omega, last_pit_omega_, dt_, j_pit_);
    float fb_pit =
        pid_pit_omega_.Calculate(target_pit_omega, gyro_data_.y(), dt_);
    pit_output = ff_pit + fb_pit;
    last_pit_omega_ = target_pit_omega;

    float target_yaw_omega =
        pid_yaw_angle_.Calculate(target_yaw_angle, euler_.Yaw(), dt_);
    float ff_yaw = JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_);
    float fb_yaw =
        pid_yaw_omega_.Calculate(target_yaw_omega, gyro_data_.z(), dt_);
    yaw_output = ff_yaw + fb_yaw;
    last_yaw_omega_ = target_yaw_omega;
  }

  static float JFeedforward(float target_omega, float last_omega, float dt_,
                            float J) {
    float feedforward = 0.0f;
    float delta_omega = target_omega - last_omega;
    feedforward = (J * delta_omega / dt_);
    return feedforward;
  }

  void SetMode(GimbalEvent gimbal_event) {
    if (gimbal_event == current_mode_) {
      return;
    };
    current_mode_ = gimbal_event;

    switch (gimbal_event) {
      case GimbalEvent::SET_MODE_RELAX:
        motor_yaw_->Disable();
        motor_pit_->Disable();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_pit_cmd_ = 0.0f;
        target_yaw_cmd_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_COMMON:
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();

        last_pit_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        break;
      default:
        break;
    }
  }
};
