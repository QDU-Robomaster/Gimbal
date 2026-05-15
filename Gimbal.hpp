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
      cycle: false
  - pid_roll_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_roll_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_roll: '@&motor_roll'
  - motor_yaw: '@&motor_yaw'
  - roll_max_angle: 0.0
  - roll_min_angle: 0.0
  - roll_lc: 0.0
  - roll_theta: 0.0
  - yaw_k: 0.0
  - j_roll: 0.0
  - j_yaw: 0.0
  - roll_zero: 0.0
  - yaw_zero: 0.0
  - patrol_range: 0.0
  - patrol_omega: 0.0
  - roll_reverse_flag: false
  - referee: '@&referee'
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args: []
required_hardware: []
depends:
  - qdu-future/CMD
  - qdu-future/Motor
  - qdu-future/BMI088
=== END MANIFEST === */
// clang-format on

#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define UI_GIMBAL_LAYER 3

static constexpr float GIMBAL_MAX_SPEED =
    static_cast<float>(LibXR::TWO_PI) * 2.0f;
enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_AUTOPATROL,
  SET_MODE_LOW_SENSITIVITY
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
   * @param pid_yaw_angle Yaw轴角度环PID参数
   * @param pid_yaw_omega Yaw轴角速度环PID参数
   * @param pid_roll_angle Roll轴角度环PID参数
   * @param pid_roll_omega Roll轴角速度环PID参数
   * @param motor_roll Roll轴电机指针
   * @param motor_yaw Yaw轴电机指针
   * @param roll_max_angle Roll轴最大角度
   * @param roll_min_angle Roll轴最小角度
   * @param roll_lc Roll轴质心距离(m)(距离水平向上为+)*Roll轴质心重力(N)
   * @param roll_theta Roll轴质心与重力轴线夹角(rad 极性自己猜)
   * @param yaw_k Yaw轴阻力系数
   * @param j_roll Roll轴转动惯量
   * @param j_yaw Yaw轴转动惯量
   * @param roll_zero Roll轴零点
   * @param yaw_zero Yaw轴零点
   * @param reverse_flag Roll轴反转标志
   */
  Gimbal(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
      LibXR::PID<float>::Param pid_yaw_omega,
      LibXR::PID<float>::Param pid_roll_angle,
      LibXR::PID<float>::Param pid_roll_omega, Motor* motor_roll,
      Motor* motor_yaw, float roll_max_angle, float roll_min_angle,
      float roll_lc, float roll_theta, float yaw_k, float j_roll, float j_yaw,
      float roll_zero, float yaw_zero, float patrol_range, float patrol_omega,
      bool reverse_flag, Referee* referee,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_omega_(pid_yaw_omega),
        pid_roll_angle_(pid_roll_angle),
        pid_roll_omega_(pid_roll_omega),
        motor_yaw_(motor_yaw),
        motor_roll_(motor_roll),
        roll_max_angle_(roll_max_angle),
        roll_min_angle_(roll_min_angle),
        roll_lc_(roll_lc),
        roll_theta_(roll_theta),
        yaw_k_(yaw_k),
        j_roll_(j_roll),
        j_yaw_(j_yaw),
        roll_zero_(roll_zero),
        yaw_zero_(yaw_zero),
        patrol_range_(patrol_range),
        patrol_omega_(patrol_omega),
        reverse_flag_(reverse_flag ? 1.0f : -1.0f),
        referee_(referee) {
    UNUSED(app);
UNUSED(referee_);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
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
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_LOW_SENSITIVITY), callback);
  };

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

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

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
      LibXR::Thread::Sleep(2);
    }
  };

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    motor_yaw_->Update();
    motor_roll_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_roll_feedback_ = motor_roll_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    abs_angle_roll_ = motor_roll_feedback_.abs_angle - roll_zero_;
    abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;

    topic_yaw_angle_.Publish(abs_angle_yaw_);
    topic_roll_angle_.Publish(abs_angle_roll_);
  }

  /**
   * @brief 解析云台控制命令
   */
  void ParseCMD() {
    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      if (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY) {
        target_yaw_cmd_ -= cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_roll_cmd_ += cmd_data_.rol * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_roll_dot_ = 0.0f;
        target_roll_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
      } else {
        target_yaw_cmd_ -= cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_roll_cmd_ += cmd_data_.rol * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_roll_dot_ = 0.0f;
        target_roll_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
      }
    } else {
      if (cmd_.GetAIGimbalStatus()) {
        target_yaw_cmd_ = cmd_data_.yaw;
        target_roll_cmd_ = cmd_data_.rol;
        target_roll_dot_ = cmd_data_.rol_dot;
        target_roll_ddot_ = cmd_data_.rol_ddot;
        target_yaw_dot_ = cmd_data_.yaw_dot;
        target_yaw_ddot_ = cmd_data_.yaw_ddot;
      } else {
        if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
          target_roll_cmd_ -=
              patrol_range_ * (2 / static_cast<float>(M_PI)) *
              asin(sin(patrol_omega_ * (LibXR::Timebase::GetMilliseconds() -
                                        patrol_start_time))) /
              1000.0f;
          target_yaw_cmd_ += 1 * dt_;
        } else {
          target_yaw_cmd_ -=
              cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
          target_roll_cmd_ +=
              cmd_data_.rol * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
          target_roll_dot_ = 0.0f;
          target_roll_ddot_ = 0.0f;
          target_yaw_dot_ = 0.0f;
          target_yaw_ddot_ = 0.0f;
        }
      }
    }
  }

  /**
   * @brief 云台控制计算与输出
   */
  void Control() {
    /*仅用于调试极性()*/
    this->torque_ =
        -this->roll_lc_ * sinf(RollFeedbackAngle() + this->roll_theta_);
    float out_roll = 0.0f;
    float out_yaw = 0.0f;

    RollLimit(target_roll_cmd_, RollFeedbackAngle(),
              motor_roll_feedback_.abs_angle, roll_max_angle_, roll_min_angle_,
              reverse_flag_);
    Solve(out_roll, out_yaw, target_roll_cmd_, target_yaw_cmd_, dt_);
    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_yaw});
    auto roll_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_roll});

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      motor_yaw_->Relax();
      motor_roll_->Relax();
      return;
    }

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 and fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_roll_, motor_roll_feedback_, roll_motor_cmd);
    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  void OnMonitor() override {}

  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  CMD& cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_roll_angle_;
  LibXR::PID<float> pid_roll_omega_;
  Motor* motor_yaw_;
  Motor* motor_roll_;
  float torque_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_roll_feedback_;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_roll_angle_ =
      LibXR::Topic::CreateTopic<float>("rollmotor_angle");

  float roll_max_angle_ = 0.0f;
  float roll_min_angle_ = 0.0f;
  float roll_lc_ = 0.0f;
  float roll_theta_ = 0.0f;
  float yaw_k_ = 0.0f;
  float target_yaw_dot_ = 0.0f;
  float target_yaw_ddot_ = 0.0f;
  float target_roll_dot_ = 0.0f;
  float target_roll_ddot_ = 0.0f;
  float j_roll_ = 0.0f;
  float j_yaw_ = 0.0f;
  LibXR::CycleValue<float> roll_zero_ = 0.0f;
  LibXR::CycleValue<float> yaw_zero_ = 0.0f;
  float patrol_range_ = 0.0f;
  float patrol_omega_ = 0.0f;
  float target_roll_cmd_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float abs_angle_yaw_ = 0.0f;
  float abs_angle_roll_ = 0.0f;
  float last_roll_omega_ = 0.0f;
  float last_yaw_omega_ = 0.0f;
  float reverse_flag_ = 1.0f;
  LibXR::MillisecondTimestamp patrol_start_time = 0.0f;
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  Referee* referee_;
  LibXR::Thread thread_;

  /*----------工具函数--------------------------------*/
  /**
   * @brief Roll轴反馈角。
   *
   * 在 x右/y前/z上 的机体系中，Roll轴就是 X 轴。这里直接读取
   * LibXR EulerAngle::Roll()。
   */
  float RollFeedbackAngle() const { return euler_.Roll(); }

  /**
   * @brief Roll轴反馈角速度，对应机体系 X 轴角速度。
   */
  float RollFeedbackOmega() const { return gyro_data_.x(); }

  /**
   * @brief Roll轴角度限位
   *
   * @param target_roll 目标Roll角度
   * @param now_eulr_angle 当前Roll反馈角
   * @param now_motor_angle 当前Roll轴电机角度
   * @param motor_max 电机最大角度
   * @param motor_min 电机最小角度
   * @param sign 方向符号
   */
  void RollLimit(float& target_roll, float now_eulr_angle,
                 float now_motor_angle, float motor_max, float motor_min,
                 float sign) {
    if ((motor_max == 0.0f) && (motor_min == 0.0f)) {
      return;
    };

    LibXR::CycleValue<float> cycle_motor_min(motor_min);
    LibXR::CycleValue<float> cycle_motor_max(motor_max);

    float diff_min = cycle_motor_min - now_motor_angle;
    float diff_max = cycle_motor_max - now_motor_angle;
    float roll_bound_0 = now_eulr_angle + diff_min / sign;
    float roll_bound_1 = now_eulr_angle + diff_max / sign;

    float upper_bound = std::max(roll_bound_0, roll_bound_1);
    float lower_bound = std::min(roll_bound_0, roll_bound_1);
    target_roll = std::clamp(target_roll, lower_bound, upper_bound);
  }

  /**
   * @brief 解算PID控制输出
   *
   * @param roll_output Roll轴输出引用
   * @param yaw_output Yaw轴输出引用
   * @param target_roll_angle 目标Roll角度
   * @param target_yaw_angle 目标Yaw角度
   * @param dt_ 时间间隔
   */
  void Solve(float& roll_output, float& yaw_output, float target_roll_angle,
             const LibXR::CycleValue<float>& target_yaw_angle, float dt_) {
    const float roll_feedback = RollFeedbackAngle();
    float roll_error = target_roll_angle - roll_feedback;
    float target_roll_omega =
        pid_roll_angle_.Calculate(roll_error, 0.0f, dt_) + target_roll_dot_;
    float ff_roll =
        JFeedforward(target_roll_omega, last_roll_omega_, dt_, j_roll_) +
        j_roll_ * target_roll_ddot_;
    float gravity_ff_roll =
        -this->roll_lc_ * sinf(roll_feedback + this->roll_theta_);
    float fb_roll =
        pid_roll_omega_.Calculate(target_roll_omega, RollFeedbackOmega(), dt_);
    roll_output = ff_roll + fb_roll + gravity_ff_roll;
    last_roll_omega_ = target_roll_omega;
    float yaw_error = target_yaw_angle - euler_.Yaw();
    float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_) + target_yaw_dot_;
    float ff_yaw =
        JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_) +
        j_yaw_ * target_yaw_ddot_;
    float fb_yaw =
        pid_yaw_omega_.Calculate(target_yaw_omega, gyro_data_.z(), dt_);
    yaw_output = ff_yaw + fb_yaw + motor_yaw_feedback_.omega * this->yaw_k_;
    last_yaw_omega_ = target_yaw_omega;
  }

  /**
   * @brief 转动惯量前馈计算
   *
   * @param target_omega 目标角速度
   * @param last_omega 上一次角速度
   * @param dt_ 时间间隔
   * @param J 转动惯量 kg*m^2
   * @return float 前馈值
   */
  static float JFeedforward(float target_omega, float last_omega, float dt_,
                            float J) {
    float feedforward = 0.0f;
    float delta_omega = target_omega - last_omega;
    feedforward = (J * delta_omega / dt_);
    return feedforward;
  }

  /**
   * @brief 设置云台模式
   *
   * @param gimbal_event 云台事件类型
   */
  void SetMode(GimbalEvent gimbal_event) {
    if (gimbal_event == current_mode_) {
      return;
    };
    if ((current_mode_ == GimbalEvent::SET_MODE_COMMON &&
         gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY) ||
        (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY &&
         gimbal_event == GimbalEvent::SET_MODE_COMMON)) {
      current_mode_ = gimbal_event;
      return;
    }
    current_mode_ = gimbal_event;

    switch (gimbal_event) {
      case GimbalEvent::SET_MODE_RELAX:
        motor_yaw_->Disable();
        motor_roll_->Disable();
        pid_roll_angle_.Reset();
        pid_roll_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_roll_cmd_ = 0.0f;
        target_yaw_cmd_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_roll_dot_ = 0.0f;
        target_roll_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_COMMON:
        target_roll_cmd_ = RollFeedbackAngle();
        target_yaw_cmd_ = euler_.Yaw();
        pid_roll_angle_.Reset();
        pid_roll_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        last_roll_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_roll_dot_ = 0.0f;
        target_roll_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        patrol_start_time = LibXR::Timebase::GetMilliseconds();
        target_roll_cmd_ = RollFeedbackAngle();
        target_yaw_cmd_ = euler_.Yaw();
        pid_roll_angle_.Reset();
        pid_roll_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        last_roll_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        target_yaw_dot_ =  0.0f;
        target_yaw_ddot_ =  0.0f;
        target_roll_dot_ =  0.0f;
        target_roll_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_LOW_SENSITIVITY:
        target_roll_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_roll_angle_.Reset();
        pid_roll_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        last_roll_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_roll_dot_ = 0.0f;
        target_roll_ddot_ = 0.0f;
        break;
      default:
        break;
    }
  }
};
