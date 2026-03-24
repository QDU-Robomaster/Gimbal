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
  - patrol_range: 0.0
  - patrol_omega: 0.0
  - pit_reverse_flag: false
  - thread_priority: LibXR::Thread::Priority::HIGH
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
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"


#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

  static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(M_2PI) * 1.5f;
  enum class GimbalEvent : uint8_t {
    SET_MODE_RELAX,
    SET_MODE_COMMON,
    SET_MODE_AUTOPATROL,
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
     * @param pid_pit_angle Pitch轴角度环PID参数
     * @param pid_pit_omega Pitch轴角速度环PID参数
     * @param motor_pit Pitch轴电机指针
     * @param motor_yaw Yaw轴电机指针
     * @param pit_max_angle Pitch轴最大角度
     * @param pit_min_angle Pitch轴最小角度
     * @param j_pit Pitch轴转动惯量
     * @param j_yaw Yaw轴转动惯量
     * @param pit_zero Pitch轴零点
     * @param yaw_zero Yaw轴零点
     * @param reverse_flag Pitch轴反转标志
     */
    Gimbal(
        LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
        uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
        LibXR::PID<float>::Param pid_yaw_omega,
        LibXR::PID<float>::Param pid_pit_angle,
        LibXR::PID<float>::Param pid_pit_omega,
        Motor* motor_pit,
        Motor* motor_yaw, float pit_max_angle, float pit_min_angle, float j_pit,
        float j_yaw, float pit_zero, float yaw_zero, float patrol_range,
        float patrol_omega, bool reverse_flag,
        LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
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
          patrol_range_(patrol_range),
          patrol_omega_(patrol_omega),
          reverse_flag_(reverse_flag ? 1.0f : -1.0f)
#ifdef DEBUG
          ,
          cmd_file_(LibXR::RamFS::CreateFile(
              "gimbal",
              debug_core::command_thunk<Gimbal, &Gimbal::DebugCommand>, this))
#endif
    {
      UNUSED(app);

#ifdef DEBUG
      hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
#endif

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
      gimbal_event_.Register(
          static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON), callback);
      gimbal_event_.Register(
          static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL), callback);
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
          gimbal->euler_.Pitch() *= -1.0f;
          euler_suber.StartWaiting();
        }
        if (gyro_suber.Available()) {
          gimbal->gyro_data_ = gyro_suber.GetData();
          gimbal->gyro_data_.y() *= -1.0f;
          gyro_suber.StartWaiting();
        }

        gimbal->Update();
        gimbal->ParseCMD();
        gimbal->Control();
        LibXR::Thread::Sleep( 2);
      }
    };

    /**
     * @brief 更新电机反馈及状态
     */
    void Update() {
      motor_yaw_->Update();
      motor_pit_->Update();
      motor_yaw_feedback_ = motor_yaw_->GetFeedback();
      motor_pit_feedback_ = motor_pit_->GetFeedback();

      auto now = LibXR::Timebase::GetMicroseconds();
      this->dt_ = (now - this->last_online_time_).ToSecondf();
      this->last_online_time_ = now;

      abs_angle_pit_ = motor_pit_feedback_.abs_angle - pit_zero_;
      abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;

      topic_yaw_angle_.Publish(abs_angle_yaw_);
      topic_pit_angle_.Publish(abs_angle_pit_);
    }

    /**
     * @brief 解析云台控制命令
     */
    void ParseCMD() {

      if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL) {
        target_yaw_cmd_ -= cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
      } else {
        if (cmd_.GetAIGimbalStatus()) {
          target_yaw_cmd_ = cmd_data_.yaw;
          target_pit_cmd_ = cmd_data_.pit;
        } else {
          if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL){
            target_pit_cmd_ -=
                patrol_range_ * (2 / M_PI) *
                asin(sin(patrol_omega_ * (LibXR::Timebase::GetMilliseconds() -
                                          patrol_start_time))) /
                1000.0f;
            target_yaw_cmd_ += 1 * dt_;
          } else {
            target_yaw_cmd_ -=
                cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
            target_pit_cmd_ +=
                cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
          }
        }
      }
    }

    /**
     * @brief 云台控制计算与输出
     */
    void Control() {
      if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
        motor_yaw_->Relax();
        motor_pit_->Relax();
        return;
      }
      float out_pit = 0.0f;
      float out_yaw = 0.0f;

      PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
                 pit_max_angle_, pit_min_angle_, reverse_flag_);
      Solve(out_pit, out_yaw, target_pit_cmd_, target_yaw_cmd_, dt_);
        auto yaw_motor_cmd = Motor::MotorCmd(
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_yaw});
      auto pit_motor_cmd = Motor::MotorCmd(
          {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_pit});

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

      motor_control(motor_pit_, motor_pit_feedback_, pit_motor_cmd);
      motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
    }

    void OnMonitor() override {}

    LibXR::Event& GetEvent() { return gimbal_event_; }

    /**
     * @brief 调试命令入口
     * @param argc 命令参数个数
     * @param argv 命令参数数组
     * @return int 命令执行结果，0表示成功，负值表示失败
     * @details 支持 state/cmd/pid/motor/full 视图以及 once/monitor 调试模式。
     */
#ifdef DEBUG
    int DebugCommand(int argc, char** argv);
#endif

   private:
     CMD& cmd_;
    LibXR::PID<float> pid_yaw_angle_;
    LibXR::PID<float> pid_yaw_omega_;
    LibXR::PID<float> pid_pit_angle_;
    LibXR::PID<float> pid_pit_omega_;
    Motor* motor_yaw_;
    Motor* motor_pit_;

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
    float patrol_range_ = 0.0f;
    float patrol_omega_ = 0.0f;
    float target_pit_cmd_ = 0.0f;
    LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
    float abs_angle_yaw_ = 0.0f;
    float abs_angle_pit_ = 0.0f;
    float last_pit_omega_ = 0.0f;
    float last_yaw_omega_ = 0.0f;
    float reverse_flag_ = 1.0f;
    LibXR::MillisecondTimestamp patrol_start_time=0.0f;
     float dt_ = 0.0f;
    LibXR::MicrosecondTimestamp last_online_time_;

    LibXR::Thread thread_;

#ifdef DEBUG
    LibXR::RamFS::File cmd_file_;
#endif

    /*----------工具函数--------------------------------*/
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

    /**
     * @brief 解算PID控制输出
     *
     * @param pit_output Pitch轴输出引用
     * @param yaw_output Yaw轴输出引用
     * @param target_pit_angle 目标Pitch角度
     * @param target_yaw_angle 目标Yaw角度
     * @param dt_ 时间间隔
     */
    void Solve(float& pit_output, float& yaw_output, float target_pit_angle,
               const LibXR::CycleValue<float>& target_yaw_angle, float dt_) {
      float pit_error = target_pit_angle - euler_.Pitch();
      float target_pit_omega = pid_pit_angle_.Calculate(pit_error, 0.0f, dt_);
      float ff_pit =
          JFeedforward(target_pit_omega, last_pit_omega_, dt_, j_pit_);
      float fb_pit =
          pid_pit_omega_.Calculate(target_pit_omega, gyro_data_.y(), dt_);
      pit_output = ff_pit + fb_pit;
      last_pit_omega_ = target_pit_omega;
      float yaw_error = target_yaw_angle - euler_.Yaw();
      float target_yaw_omega = pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_);
      float ff_yaw =
          JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_);
      float fb_yaw =
          pid_yaw_omega_.Calculate(target_yaw_omega, gyro_data_.z(), dt_);
      yaw_output = ff_yaw + fb_yaw;
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
        case GimbalEvent::SET_MODE_AUTOPATROL:
          patrol_start_time=LibXR::Timebase::GetMilliseconds();
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

#ifdef DEBUG
#define GIMBAL_DEBUG_IMPL
#include "GimbalDebug.inl"
#undef GIMBAL_DEBUG_IMPL
#endif


#ifdef DEBUG
#define GIMBAL_DEBUG_IMPL
#include "GimbalDebug.inl"
#undef GIMBAL_DEBUG_IMPL
#endif
