#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 4096
  - pid_yaw_angle_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 50.0
      cycle: true
  - pid_pitch_angle_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 50.0
      cycle: true
  - pid_yaw_omega_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 3.0
      cycle: false
  - pid_pitch_omega_param:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 3.0
      cycle: false
  - motor_pitch: '@&motor_pit'
  - motor_yaw: '@&motor_yaw'
  - limit:
      max_pitch_angle_: 6.0
      min_pitch_angle_: 5.2
      max_yaw_angle_: 0.0
      min_yaw_angle_: 0.0
      reverse_pitch_limit_: false
      reverse_yaw_limit_: false
      J_pit_: 0.0
      J_yaw_: 0.0
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "DMMotor.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "pid.hpp"

static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(M_2PI) * 1.5f;
static constexpr float TORQUE_CONSTANT = 0.741f;

enum class GimbalEvent : uint8_t { SET_MODE_RELAX, SET_MODE_INDEPENDENT, SET_MODE_AUTOAIM };
enum class GimbalMode : uint8_t { RELAX, INDEPENDENT, AUTOAIM };

class Gimbal : public LibXR::Application {
 public:
  struct NowParam {
    float now_pit_angle_ = 0.0f;
    float now_pit_omega_ = 0.0f;
    float now_yaw_angle_ = 0.0f;
    float now_yaw_omega_ = 0.0f;
  };

  struct TarParam {
    float target_pit_angle_ = 0.0f;
    float target_pit_omega_ = 0.0f;
    float target_yaw_angle_ = 0.0f;
    float target_yaw_omega_ = 0.0f;
  };

  struct Limit {
    float max_pit_angle_ = 0.0f;
    float min_pit_angle_ = 0.0f;
    float max_yaw_angle_ = 0.0f;
    float min_yaw_angle_ = 0.0f;
    bool reverse_pit_ = false;
    bool reverse_yaw_ = false;
    float J_pit_ = 0.0f;
    float J_yaw_ = 0.0f;
  };

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
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param pid_yaw_angle_param,
         LibXR::PID<float>::Param pid_pit_angle_param,
         LibXR::PID<float>::Param pid_yaw_omega_param,
         LibXR::PID<float>::Param pid_pit_omega_param,
         Motor *motor_pit,
         Motor *motor_yaw,
         Limit limit)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle_param),
        pid_pit_angle_(pid_pit_angle_param),
        pid_yaw_omega_(pid_yaw_omega_param),
        pid_pit_omega_(pid_pit_omega_param),
        motor_yaw_(motor_yaw),
        motor_pit_(motor_pit),
        limit_(limit) {
    UNUSED(hw);
    UNUSED(app);

    motor_yaw_cmd_.mode = Motor::ControlMode::MODE_TORQUE;
    motor_yaw_cmd_.reduction_ratio = 1.0f;
    motor_yaw_cmd_.torque = 0.0f;
    motor_yaw_cmd_.position = 0.0f;
    motor_yaw_cmd_.velocity = 0.0f;
    motor_yaw_cmd_.kp = 0.0f;
    motor_yaw_cmd_.kd = 0.0f;

    motor_pit_cmd_.mode = Motor::ControlMode::MODE_TORQUE;
    motor_pit_cmd_.reduction_ratio = 1.0f;
    motor_pit_cmd_.torque = 0.0f;
    motor_pit_cmd_.position = 0.0f;
    motor_pit_cmd_.velocity = 0.0f;
    motor_pit_cmd_.kp = 0.0f;
    motor_pit_cmd_.kd = 0.0f;

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalMode::RELAX);
        },
        this);

    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->EventHandler(event_id);
        },
        this);

    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_INDEPENDENT),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOAIM),
                           callback);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 云台控制线程函数
   */
  static void ThreadFunc(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber("ahrs_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber("bmi088_gyro");

    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    while (true) {

      gimbal->mutex_.Lock();
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
      gimbal->SetpointFromCMD();
      gimbal->mutex_.Unlock();
      gimbal->OutputToDynamics();
      gimbal->thread_.Sleep(2);
    }
  }

  /**
   * @brief 更新函数
   */
  void Update() {
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    now_param_.now_yaw_angle_ = euler_.Yaw();
    now_param_.now_pit_angle_ = euler_.Pitch();

    now_param_.now_yaw_omega_ = gyro_data_.z();
    now_param_.now_pit_omega_ = gyro_data_.y();

    last_yaw_angle_ = now_param_.now_yaw_angle_;
    last_pit_angle_ = now_param_.now_pit_angle_;
    last_yaw_omega_ = now_param_.now_yaw_omega_;
    last_pit_omega_ = now_param_.now_pit_omega_;

  float yaw_angle = motor_yaw_feedback_.abs_angle;
  float pit_angle = motor_pit_feedback_.abs_angle;

    topic_yaw_angle_.Publish(yaw_angle);
    topic_pit_angle_.Publish(pit_angle);
  }

  /**
   * @brief 处理遥控器的数据
   */
  void SetpointFromCMD() {
    float gimbal_yaw_cmd = 0.0f;
    float gimbal_pit_cmd = 0.0f;
    /*操作员控制模式*/
    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
      gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      gimbal_pit_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      tar_param_.target_yaw_angle_ =
          tar_param_.target_yaw_angle_ + gimbal_yaw_cmd;
      tar_param_.target_pit_angle_ =
          tar_param_.target_pit_angle_ + gimbal_pit_cmd;
    }
    /*自动控制模式*/
    else {
      /*查看AI云台是否在线*/
      if (cmd_.GetAIGimbalStatus()) {
        tar_param_.target_yaw_angle_ = cmd_data_.yaw;
        tar_param_.target_pit_angle_ = cmd_data_.pit;
      }
      /*AI离线用遥控器数据*/
      else {
        gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        gimbal_pit_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        tar_param_.target_yaw_angle_ =
            tar_param_.target_yaw_angle_ + gimbal_yaw_cmd;
        tar_param_.target_pit_angle_ =
            tar_param_.target_pit_angle_ + gimbal_pit_cmd;
      }
    }
    /*pitch轴限位*/
    if (limit_.max_pit_angle_ != limit_.min_pit_angle_) {
      if (limit_.reverse_pit_ == true) {
        const float ENCODER_DELTA_MAX_PIT =
          LibXR::CycleValue(motor_pit_feedback_.abs_angle) -
          this->limit_.max_pit_angle_;
        const float ENCODER_DELTA_MIN_PIT =
          LibXR::CycleValue(motor_pit_feedback_.abs_angle) -
          this->limit_.min_pit_angle_;
        const float PIT_ERR =
          tar_param_.target_pit_angle_ - now_param_.now_pit_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
          tar_param_.target_pit_angle_ = std::clamp(
            tar_param_.target_pit_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      } else {
        const float ENCODER_DELTA_MAX_PIT =
          this->limit_.max_pit_angle_ -
          LibXR::CycleValue(motor_pit_feedback_.abs_angle);
        const float ENCODER_DELTA_MIN_PIT =
          this->limit_.min_pit_angle_ -
          LibXR::CycleValue(motor_pit_feedback_.abs_angle);
        const float PIT_ERR =
          tar_param_.target_pit_angle_ - now_param_.now_pit_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
          tar_param_.target_pit_angle_ = std::clamp(
            tar_param_.target_pit_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      }
    }
  }

  /**
   *@brief 控制函数
   *
   */
  void OutputToDynamics() {
    if ((this->enable_flag_ == true) && (this->dm_motor_flag_ == false)) {
      for (int i = 0; i < 2; i++) {
        this->dm_motor_flag_ = true;
        if (motor_yaw_cmd_.mode == Motor::ControlMode::MODE_MIT) {
          if (motor_yaw_->GetFeedback().state == 0) {
            motor_yaw_->Enable();
          } else if (motor_yaw_feedback_.state == 1) {
          } else {
            motor_yaw_->ClearError();
            motor_yaw_->Enable();
          }
        }
        if (motor_pit_cmd_.mode == Motor::ControlMode::MODE_MIT) {
          if (motor_pit_feedback_.state == 0) {
            motor_pit_->Enable();
          } else if (motor_pit_feedback_.state == 1) {
          } else {
            motor_pit_->ClearError();
            motor_pit_->Enable();
          }
        }
      }
    }
    switch (current_mode_) {
      case GimbalMode::RELAX: {
        if (motor_yaw_cmd_.mode == Motor::ControlMode::MODE_MIT) {
          motor_yaw_->Disable();
        } else {
          motor_yaw_->Relax();
        }

        if (motor_pit_cmd_.mode == Motor::ControlMode::MODE_MIT) {
          motor_pit_->Disable();
        } else {
          motor_pit_->Relax();
        }
        enable_flag_ = false;
        dm_motor_flag_ = false;
      } break;
      case GimbalMode::INDEPENDENT: {
        /*串级PID位置外环 角速度内环 + 力矩前馈*/
        /*位置环+前馈*/
        tar_param_.target_yaw_omega_ =
            pid_yaw_angle_.Calculate(
                tar_param_.target_yaw_angle_,
                now_param_.now_yaw_angle_,
                dt_);
        output_yaw_ =
            FeedforwardControl(
                tar_param_.target_yaw_omega_,
                now_param_.now_yaw_omega_,
                dt_,
                limit_.J_yaw_);

        tar_param_.target_pit_omega_ =
            pid_pit_angle_.Calculate(
                tar_param_.target_pit_angle_,
                now_param_.now_pit_angle_,
                dt_);
        output_pit_ =
            FeedforwardControl(
                tar_param_.target_pit_omega_,
                now_param_.now_pit_omega_,
                dt_,
                limit_.J_pit_);
        /*速度环计算*/
        output_yaw_ +=
            (pid_yaw_omega_.Calculate(
                tar_param_.target_yaw_omega_,
                gyro_data_.z(),
                dt_));
        output_pit_ +=
            (pid_pit_omega_.Calculate(
                tar_param_.target_pit_omega_,
                gyro_data_.y(),
                dt_));

        motor_yaw_cmd_.torque = output_yaw_;
        motor_yaw_->Control(motor_yaw_cmd_);
        motor_pit_cmd_.torque = output_pit_;
        motor_pit_->Control(motor_pit_cmd_);

        enable_flag_ = true;
      } break;
      case GimbalMode::AUTOAIM: {
        tar_param_.target_yaw_omega_ =
            pid_yaw_angle_.Calculate(
                tar_param_.target_yaw_angle_,
                now_param_.now_yaw_angle_,
                dt_);
        tar_param_.target_pit_omega_ =
            pid_pit_angle_.Calculate(
                tar_param_.target_pit_angle_,
                now_param_.now_pit_angle_,
                dt_);

        output_yaw_ =
            pid_yaw_omega_.Calculate(
                tar_param_.target_yaw_omega_,
                gyro_data_.z(),
                dt_);
    output_pit_ =
      pid_pit_omega_.Calculate(
        tar_param_.target_pit_omega_,
        gyro_data_.y(),
        dt_);

  motor_yaw_cmd_.torque = output_yaw_;
  motor_yaw_->Control(motor_yaw_cmd_);
  motor_pit_cmd_.torque = output_pit_;
  motor_pit_->Control(motor_pit_cmd_);
  enable_flag_ = true;
      } break;
      default:
        break;
    }
  }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) {
    switch (static_cast<GimbalEvent>(event_id)) {
      case GimbalEvent::SET_MODE_RELAX:
        SetMode(GimbalMode::RELAX);
        break;
      case GimbalEvent::SET_MODE_INDEPENDENT:
        SetMode(GimbalMode::INDEPENDENT);
        break;
      case GimbalEvent::SET_MODE_AUTOAIM:
        SetMode(GimbalMode::AUTOAIM);
      default:
        break;
    }
  }

  /**
   * @brief 设置云台模式
   * @param mode 云台模式
   */
  void SetMode(GimbalMode mode) {
    if (current_mode_ == GimbalMode::RELAX) {
      if (mode == GimbalMode::INDEPENDENT) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pit_angle_ = now_param_.now_pit_angle_;
      }
    } else if (current_mode_ == GimbalMode::INDEPENDENT) {
      if (mode == GimbalMode::AUTOAIM || mode == GimbalMode::RELAX) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pit_angle_ = now_param_.now_pit_angle_;
      }
    }
    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    mutex_.Lock();
    current_mode_ = mode;
    mutex_.Unlock();
  }

  LibXR::Event &GetEvent() { return gimbal_event_; }

  /**
   * @brief  前馈控制feedforward
   * @param J 电机转动惯量 (kg*m^2)
   * @note 用在速度环之前,位置环之后
   * @return 扭矩(N*m)
   */
  float FeedforwardControl(float target_omega, float now_omega, float dt_,
                           float J) {
    float out = 0.0f, delta_omega = 0.0f;
    delta_omega = target_omega - now_omega;
    out = (J * delta_omega / dt_);
    return out;  // N*m
  }

  void OnMonitor() override {}

 private:
  CMD &cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pit_omega_;

  Motor *motor_yaw_;
  Motor *motor_pit_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;

  Motor::MotorCmd motor_yaw_cmd_;
  Motor::MotorCmd motor_pit_cmd_;

  Limit limit_;
  NowParam now_param_;
  TarParam tar_param_;

  bool enable_flag_ = false;
  bool dm_motor_flag_ = false;

  float last_yaw_angle_ = 0.0f;
  float last_yaw_omega_ = 0.0f;
  float last_pit_angle_ = 0.0f;
  float last_pit_omega_ = 0.0f;

  GimbalMode current_mode_ = GimbalMode::RELAX;

  float output_yaw_ = 0.0f;
  float output_pit_ = 0.0f;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");

  CMD::GimbalCMD cmd_data_;
  LibXR::Thread thread_;
  LibXR::Event gimbal_event_;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  float dt_ = 0.0f;

  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Mutex mutex_;
};
