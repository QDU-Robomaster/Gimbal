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
      max_pitch_angle_: 0.0
      min_pitch_angle_: 0.0
      max_yaw_angle_: 0.0
      min_yaw_angle_: 0.0
      reverse_pitch_limit_: false
      reverse_yaw_limit_: false
      J_pitch_: 0.0
      J_yaw_: 0.0
  - zero_point: 0.0
  - gimbal_cmd_topic_name: gimbal_cmd
  - accl_topic_name: bmi088_accl
  - euler_topic_name: ahrs_euler
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "DMMotor.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "pid.hpp"

static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(M_2PI) * 1.5f;
static constexpr float TORQUE_CONSTANT = 0.741f;
static constexpr float MAX_CURRENT = 3.0f;
static constexpr float MAX_TORQUE = 10.0f;
static constexpr int FILTER_SIZE = 6;

template <typename MotorTypePitch, typename MotorTypeYaw>
class Gimbal : public LibXR::Application {
 public:
  enum class GimbalEvent : uint8_t {
    SET_MODE_RELAX,
    SET_MODE_INDEPENDENT,
    SET_MODE_AUTOAIM,
  };

  typedef enum : uint8_t { RELAX, INDEPENDENT, AUTOAIM } GimbalMode;

  struct NowParam {
    float now_pitch_angle_ = 0.0f;
    float now_yaw_angle_ = 0.0f;
    float now_pitch_omega_ = 0.0f;
    float now_yaw_omega_ = 0.0f;
    float now_pitch_current_ = 0.0f;
    float now_yaw_current_ = 0.0f;
  };

  struct TarParam {
    float target_yaw_angle_ = 0.0f;
    float target_pitch_angle_ = 0.0f;
    float target_yaw_omega_ = 0.0f;
    float target_pitch_omega_ = 0.0f;
    float target_yaw_current_ = 0.0f;
    float target_pitch_current_ = 0.0f;
  };

  struct Limit {
    float max_pitch_angle_ = 0.0f;
    float min_pitch_angle_ = 0.0f;
    float max_yaw_angle_ = 0.0f;
    float min_yaw_angle_ = 0.0f;
    bool reverse_pitch_limit_ = false;
    bool reverse_yaw_limit_ = false;
    float J_pitch_ = 0.0f;
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
   * @param pid_pitch_angle_param Pitch轴角度环PID参数
   * @param pid_yaw_omega_param   Yaw轴角速度环PID参数
   * @param pid_pitch_omega_param Pitch轴角速度环PID参数
   * @param gimbal_cmd_topic_name 云台命令Topic名称
   * @param accl_topic_name  加速度计数据Topic名称
   * @param euler_topic_name 欧拉角数据Topic名称
   */
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param pid_yaw_angle_param,
         LibXR::PID<float>::Param pid_pitch_angle_param,
         LibXR::PID<float>::Param pid_yaw_omega_param,
         LibXR::PID<float>::Param pid_pitch_omega_param,
         MotorTypePitch *motor_pitch, MotorTypeYaw *motor_yaw, Limit limit,
         float zero_point,
         const char *gimbal_cmd_topic_name, const char *accl_topic_name,
         const char *euler_topic_name)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle_param),
        pid_pitch_angle_(pid_pitch_angle_param),
        pid_yaw_omega_(pid_yaw_omega_param),
        pid_pitch_omega_(pid_pitch_omega_param),
        motor_yaw_(motor_yaw),
        motor_pitch_(motor_pitch),
        gimbal_cmd_name_(gimbal_cmd_topic_name),
        accl_name_(accl_topic_name),
        euler_name_(euler_topic_name),
        limit_(limit),
        zero_point_(zero_point) {
    UNUSED(hw);
    UNUSED(app);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(RELAX);
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
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_INDEPENDENT), callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOAIM),
                           callback);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 云台控制线程函数
   */
  static void ThreadFunc(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(
        gimbal->gimbal_cmd_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        gimbal->accl_name_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->euler_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "bmi088_gyro");

    cmd_suber.StartWaiting();
    accl_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    while (true) {
      auto last_time = LibXR::Timebase::GetMilliseconds();

      gimbal->mutex_.Lock();
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (accl_suber.Available()) {
        gimbal->accl_data_ = accl_suber.GetData();
        accl_suber.StartWaiting();
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
      gimbal->thread_.SleepUntil(last_time, 2.0f);
    }
  }

  /**
   * @brief 更新函数
   */
  void Update() {
    motor_yaw_->Update();
    motor_pitch_->Update();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    now_param_.now_yaw_angle_   = euler_.Yaw();
    now_param_.now_pitch_angle_ = euler_.Pitch();

    now_param_.now_yaw_omega_   = gyro_data_.z();
    now_param_.now_pitch_omega_ = gyro_data_.y();

    last_yaw_angle_ = now_param_.now_yaw_angle_;
    last_pitch_angle_ = now_param_.now_pitch_angle_;
    last_yaw_omega_ = now_param_.now_yaw_omega_;
    last_pitch_omega_ = now_param_.now_pitch_omega_;

    float yaw_angle = motor_yaw_->GetAngle();
    float pit_angle = motor_pitch_->GetAngle();
    float delta_angle = LibXR::CycleValue(motor_yaw_->GetAngle() - zero_point_);

    topic_yaw_angle_.Publish(yaw_angle);
    topic_pitch_angle_.Publish(pit_angle);
    topic_delta_yaw_.Publish(delta_angle);
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
      tar_param_.target_pitch_angle_ =
          tar_param_.target_pitch_angle_ + gimbal_pit_cmd;
    }
    /*自动控制模式*/
    else {
      /*查看AI云台是否在线*/
      if (cmd_.GetAIGimbalStatus()) {
        tar_param_.target_yaw_angle_ = cmd_data_.yaw;
        tar_param_.target_pitch_angle_ = cmd_data_.pit;
      }
      /*AI离线用遥控器数据*/
      else {
        gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        gimbal_pit_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        tar_param_.target_yaw_angle_ =
            tar_param_.target_yaw_angle_ + gimbal_yaw_cmd;
        tar_param_.target_pitch_angle_ =
            tar_param_.target_pitch_angle_ + gimbal_pit_cmd;
      }
    }
    /*pitch轴限位*/
    if (limit_.max_pitch_angle_ != limit_.min_pitch_angle_) {
      if (limit_.reverse_pitch_limit_ == true) {
        const float ENCODER_DELTA_MAX_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.max_pitch_angle_;
        const float ENCODER_DELTA_MIN_PIT =
            LibXR::CycleValue(motor_pitch_->GetAngle()) -
            this->limit_.min_pitch_angle_;
        const float PIT_ERR =
            tar_param_.target_pitch_angle_ - now_param_.now_pitch_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        tar_param_.target_pitch_angle_ = std::clamp(
            tar_param_.target_pitch_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      } else {
        const float ENCODER_DELTA_MAX_PIT =
            this->limit_.max_pitch_angle_ -
            LibXR::CycleValue(motor_pitch_->GetAngle());
        const float ENCODER_DELTA_MIN_PIT =
            this->limit_.min_pitch_angle_ -
            LibXR::CycleValue(motor_pitch_->GetAngle());
        const float PIT_ERR =
            tar_param_.target_pitch_angle_ - now_param_.now_pitch_angle_;
        const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - PIT_ERR;
        const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - PIT_ERR;
        tar_param_.target_pitch_angle_ = std::clamp(
            tar_param_.target_pitch_angle_, DELTA_MIN_PIT, DELTA_MAX_PIT);
      }
    }
  }

  /**
   *@brief 控制函数
   *
   */
  void OutputToDynamics() {
    if (current_mode_ != RELAX) {
        if constexpr (std::is_same_v<MotorTypeYaw, DMMotor>) {
          if(motor_yaw_->GetState() == 0) {motor_yaw_->Enable();}
          else {motor_yaw_->ClearError(); motor_yaw_->Enable();}
        }
        if constexpr (std::is_same_v<MotorTypePitch, DMMotor>) {
          if(motor_pitch_->GetState() == 0) {motor_pitch_->Enable();}
          else {motor_pitch_->ClearError(); motor_pitch_->Enable();}
        }
    }
    switch (current_mode_) {
      case RELAX: {
        if constexpr (std::is_same_v<MotorTypeYaw, DMMotor>) {
          motor_yaw_->Disable();
        } else if constexpr (std::is_same_v<MotorTypeYaw, RMMotor>) {
          motor_yaw_->Relax();
        }

        if constexpr (std::is_same_v<MotorTypePitch, DMMotor>) {
          motor_pitch_->Disable();
        } else if constexpr (std::is_same_v<MotorTypePitch, RMMotor>) {
          motor_pitch_->Relax();
        }
      } break;
      case INDEPENDENT: {
        /*串级PID位置外环 角速度内环 + 力矩前馈*/
        /*位置环+前馈*/
        tar_param_.target_yaw_omega_ = pid_yaw_angle_.Calculate(
            tar_param_.target_yaw_angle_,
            now_param_.now_yaw_angle_,
            dt_);
        output_yaw_ = FeedforwardControl(
            tar_param_.target_yaw_omega_,
            now_param_.now_yaw_omega_,
            dt_,
            limit_.J_yaw_);

        tar_param_.target_pitch_omega_ =pid_pitch_angle_.Calculate(
            tar_param_.target_pitch_angle_,
            now_param_.now_pitch_angle_,
            dt_);
        output_pitch_ = FeedforwardControl(tar_param_.target_pitch_omega_,
            now_param_.now_pitch_omega_,
            dt_,
            limit_.J_pitch_);
        /*速度环计算*/
        output_yaw_ += (pid_yaw_omega_.Calculate(
            tar_param_.target_yaw_omega_,
            gyro_data_.z(),
            dt_));
        output_pitch_ += (pid_pitch_omega_.Calculate(
            tar_param_.target_pitch_omega_,
            gyro_data_.y(),
            dt_));
        /*力矩输出到电机*/
        if constexpr (std::is_same_v<MotorTypeYaw, RMMotor>) {
              motor_yaw_->TorqueControl(output_yaw_, 1.0f);
        } else if constexpr (std::is_same_v<MotorTypeYaw, DMMotor>) {
              motor_yaw_->MITControl(0, 0, 0, 0, output_yaw_);
        }

        if constexpr (std::is_same_v<MotorTypePitch, RMMotor>) {
              motor_pitch_->TorqueControl(output_pitch_, 1.0f);
        } else if constexpr (std::is_same_v<MotorTypePitch, DMMotor>) {
              motor_pitch_->MITControl(0, 0, 0, 0, output_pitch_);
        }
      } break;
      case AUTOAIM: {
        tar_param_.target_yaw_omega_ = pid_yaw_angle_.Calculate(
            tar_param_.target_yaw_angle_,
            now_param_.now_yaw_angle_,
            dt_);
        tar_param_.target_pitch_omega_ = pid_pitch_angle_.Calculate(
            tar_param_.target_pitch_angle_,
            now_param_.now_pitch_angle_,
            dt_);

        output_yaw_ = (pid_yaw_omega_.Calculate(
            tar_param_.target_yaw_omega_,
            gyro_data_.z(),
            dt_));
        output_pitch_ = (pid_pitch_omega_.Calculate(
            tar_param_.target_pitch_omega_,
            gyro_data_.y(),
            dt_));

        if constexpr (std::is_same_v<MotorTypeYaw, RMMotor>) {
          motor_yaw_->TorqueControl(output_yaw_, 1.0f);
        } else if constexpr (std::is_same_v<MotorTypeYaw, DMMotor>) {
          motor_yaw_->MITControl(0, 0, 0, 0, output_yaw_);
        }

        if constexpr (std::is_same_v<MotorTypePitch, RMMotor>) {
          motor_pitch_->TorqueControl(output_pitch_, 1.0f);
        } else if constexpr (std::is_same_v<MotorTypePitch, DMMotor>) {
          motor_pitch_->MITControl(0, 0, 0, 0, output_pitch_);
        }
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
    if (current_mode_ == RELAX) {
      if (mode == INDEPENDENT) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pitch_angle_ = now_param_.now_pitch_angle_;
      }
    } else if (current_mode_ == INDEPENDENT) {
      if (mode == AUTOAIM || mode == RELAX) {
        tar_param_.target_yaw_angle_ = now_param_.now_yaw_angle_;
        tar_param_.target_pitch_angle_ = now_param_.now_pitch_angle_;
      }
    }
    pid_pitch_angle_.Reset();
    pid_pitch_omega_.Reset();
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

  /*角度映射到[0, 2π]*/
  inline float WrapTo2PI(float angle) {
    angle = static_cast<float>(fmod(angle, M_2PI));
    if (angle < 0) {
      angle += M_2PI;
    }
    return angle;
  }

  /*角度映射到[-π, π]*/
  inline float WrapToPi(float angle) {
    angle = static_cast<float>(fmod(angle, M_2PI));
    if (angle > M_PI) {
      angle -= M_2PI;
    } else if (angle < -M_PI) {
      angle += M_2PI;
    }
    return angle;
  }

 private:
  CMD &cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_pitch_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pitch_omega_;

  MotorTypeYaw *motor_yaw_;
  MotorTypePitch *motor_pitch_;

  const char *gimbal_cmd_name_;
  const char *accl_name_;
  const char *euler_name_;

  Limit limit_;
  NowParam now_param_;
  TarParam tar_param_;

  float last_yaw_angle_ = 0.0f;
  float last_pitch_angle_ = 0.0f;
  float last_yaw_omega_ = 0.0f;
  float last_pitch_omega_ = 0.0f;

  GimbalMode current_mode_ = GimbalMode::RELAX;

  float gyro_buffer_[FILTER_SIZE];
  float filtered_gyro_ = 0.0f;

  float output_yaw_ = 0.0f;
  float output_pitch_ = 0.0f;
  float zero_point_ = 0.0f;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("chassis_yaw");
  LibXR::Topic topic_pitch_angle_ =
      LibXR::Topic::CreateTopic<float>("chassis_pitch");
  LibXR::Topic topic_delta_yaw_ =
      LibXR::Topic::CreateTopic<float>("delta_yaw");

  CMD::GimbalCMD cmd_data_;
  LibXR::Thread thread_;
  LibXR::Event gimbal_event_;

  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  float dt_ = 0.0f;

  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Mutex mutex_;
};
