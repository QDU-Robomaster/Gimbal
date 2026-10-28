#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 云台控制模块：pitch / yaw 两轴的角度环与角速度环闭环控制 / Gimbal control Module with cascaded angle and angular-velocity loops on the pitch and yaw axes
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "GimbalLog.hpp"
#include "GimbalMbc.hpp"
#include "Motor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "logger.hpp"
#include "pid.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define UI_GIMBAL_LAYER 3

/// 云台最大角速度输入 (rad/s)
/// Maximum angular-velocity input of the gimbal (rad/s)
static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(LibXR::TWO_PI) * 2.0f;

/**
 * @brief 云台模式事件。
 *        Gimbal mode events.
 */
enum class GimbalEvent : uint8_t
{
  SET_MODE_RELAX,           ///< 放松：失能电机，目标清零
                            ///< Relax: motors disabled, targets cleared
  SET_MODE_COMMON,          ///< 常规控制
                            ///< Normal control
  SET_MODE_AUTOPATROL,      ///< 自动巡逻
                            ///< Automatic patrol
  SET_MODE_LOW_SENSITIVITY  ///< 低灵敏度：操作员输入乘以 0.1
                            ///< Low sensitivity: operator input scaled by 0.1
};

/**
 * @brief 云台控制模块：pitch / yaw 两轴的角度环与角速度环闭环控制。
 *        Gimbal control Module with cascaded angle and angular-velocity loops on the
 *        pitch and yaw axes.
 */
class Gimbal
{
 public:
  /**
   * @brief 云台配置参数。
   *        Gimbal configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth;  ///< 线程栈深
    ///< Thread stack depth
    LibXR::PID<float>::Param pid_yaw_angle;  ///< yaw 角度环 PID
    ///< Yaw angle-loop PID
    LibXR::PID<float>::Param pid_yaw_omega;  ///< yaw 角速度环 PID
    ///< Yaw angular-velocity-loop PID
    LibXR::PID<float>::Param pid_pit_angle;  ///< pitch 角度环 PID
    ///< Pitch angle-loop PID
    LibXR::PID<float>::Param pid_pit_omega;  ///< pitch 角速度环 PID
    ///< Pitch angular-velocity-loop PID
    float pit_max_angle;  ///< pitch 电机角度上限 (rad)，上下限均为 0 时不限位
    ///< Pitch motor angle upper limit (rad); limiting is disabled when both limits are 0
    float pit_min_angle;  ///< pitch 电机角度下限 (rad)
    ///< Pitch motor angle lower limit (rad)
    float pit_lc;  ///< pitch 质心距离 (m，水平向上为正) 乘以质心重力 (N)
    ///< Pitch center-of-mass distance (m, positive upward from the horizontal) times its
    ///< weight (N)
    float pit_theta;  ///< pitch 质心与重力轴线夹角 (rad)
    ///< Angle between the pitch center of mass and the gravity axis (rad)
    float yaw_k;  ///< yaw 阻尼系数，乘以 yaw 电机角速度
    ///< Yaw damping coefficient, multiplies the yaw motor angular velocity
    float j_pit;  ///< pitch 转动惯量 (kg·m^2)
    ///< Pitch moment of inertia (kg·m^2)
    float j_yaw;  ///< yaw 转动惯量 (kg·m^2)
    ///< Yaw moment of inertia (kg·m^2)
    float pit_zero;  ///< pitch 电机零点 (rad)
    ///< Pitch motor zero point (rad)
    float yaw_zero;  ///< yaw 电机零点 (rad)
    ///< Yaw motor zero point (rad)
    float patrol_range;  ///< 自动巡逻的 pitch 摆动幅度
    ///< Pitch oscillation amplitude of the automatic patrol
    float patrol_omega;  ///< 自动巡逻的角频率，时间以 ms 计 (rad/ms)
    ///< Angular frequency of the automatic patrol, time in ms (rad/ms)
    bool reverse_flag;  ///< pitch 电机角与欧拉角同向为 true
    ///< True when the pitch motor angle and the Euler angle have the same direction
    LibXR::Thread::Priority thread_priority;  ///< 线程优先级
    ///< Thread priority
    const char* euler_topic_name;  ///< 订阅的云台姿态欧拉角 Topic 名称
    ///< Name of the subscribed gimbal Euler angle Topic
    const char* gyro_topic_name;  ///< 订阅的云台角速度 Topic 名称
    ///< Name of the subscribed gimbal angular velocity Topic
    const char* gimbal_cmd_topic_name;  ///< 订阅的云台控制命令 Topic 名称
    ///< Name of the subscribed gimbal command Topic
  };

  /**
   * @brief 构造 Gimbal，创建控制线程并注册模式事件。
   *        Construct Gimbal, create the control thread and register the mode events.
   *
   * @param cmd 命令模块实例。
   *            Command Module instance.
   * @param motor_pit pitch 轴电机。
   *                  Pitch axis motor.
   * @param motor_yaw yaw 轴电机。
   *                  Yaw axis motor.
   * @param referee Referee 实例指针。
   *                Pointer to a Referee instance.
   * @param param 配置参数。
   *              Configuration parameters.
   * @param ramfs 接收 `gimbal_ident` 辨识命令与 `gimbal_ctrl` 控制器命令的 RamFS，为空时
   *              不注册命令。
   *              RamFS that receives the `gimbal_ident` identification command and the
   *              `gimbal_ctrl` controller command; the commands are not registered when
   *              null.
   * @param mbc 模型补偿控制器参数，见 GimbalMbc.hpp；enable 为 false 或参数无效时用 PID。
   *            Parameters of the model-based controller, see GimbalMbc.hpp; the PID is
   *            used when enable is false or the parameters are invalid.
   */
  Gimbal(CMD& cmd, Motor& motor_pit, Motor& motor_yaw, Referee* referee,
         const Param& param = {.task_stack_depth = 2048,
                               .pid_yaw_angle = {.k = 0.0f,
                                                 .p = 0.0f,
                                                 .i = 0.0f,
                                                 .d = 0.0f,
                                                 .i_limit = 0.0f,
                                                 .out_limit = 0.0f,
                                                 .cycle = true},
                               .pid_yaw_omega = {.k = 0.0f,
                                                 .p = 0.0f,
                                                 .i = 0.0f,
                                                 .d = 0.0f,
                                                 .i_limit = 0.0f,
                                                 .out_limit = 0.0f,
                                                 .cycle = true},
                               .pid_pit_angle = {.k = 0.0f,
                                                 .p = 0.0f,
                                                 .i = 0.0f,
                                                 .d = 0.0f,
                                                 .i_limit = 0.0f,
                                                 .out_limit = 0.0f,
                                                 .cycle = false},
                               .pid_pit_omega = {.k = 0.0f,
                                                 .p = 0.0f,
                                                 .i = 0.0f,
                                                 .d = 0.0f,
                                                 .i_limit = 0.0f,
                                                 .out_limit = 0.0f,
                                                 .cycle = false},
                               .pit_max_angle = 0.0f,
                               .pit_min_angle = 0.0f,
                               .pit_lc = 0.0f,
                               .pit_theta = 0.0f,
                               .yaw_k = 0.0f,
                               .j_pit = 0.0f,
                               .j_yaw = 0.0f,
                               .pit_zero = 0.0f,
                               .yaw_zero = 0.0f,
                               .patrol_range = 0.0f,
                               .patrol_omega = 0.0f,
                               .reverse_flag = false,
                               .thread_priority = LibXR::Thread::Priority::MEDIUM,
                               .euler_topic_name = "ahrs_euler",
                               .gyro_topic_name = "bmi088_gyro",
                               .gimbal_cmd_topic_name = "gimbal_cmd"},
         LibXR::RamFS* ramfs = nullptr,
         const GimbalMbc::Param& mbc = {.enable_pit = false,
                                        .enable_yaw = false,
                                        .j_pit = 0.0f,
                                        .j_yaw0 = 0.0f,
                                        .j_yaw_c2 = 0.0f,
                                        .j_yaw_s2 = 0.0f,
                                        .pit_kp = 0.0f,
                                        .pit_kd = 0.0f,
                                        .pit_wo = 0.0f,
                                        .pit_wmax = 0.0f,
                                        .yaw_kp = 0.0f,
                                        .yaw_kd = 0.0f,
                                        .yaw_wo = 0.0f,
                                        .yaw_wmax = 0.0f,
                                        .pit_b = 0.0f,
                                        .pit_tc = 0.0f,
                                        .yaw_b = 0.0f,
                                        .yaw_tc = 0.0f,
                                        .fric_eps = 0.0f,
                                        .pit_torque_max = 0.0f,
                                        .yaw_torque_max = 0.0f,
                                        .pit_yaw_acc = 0.0f,
                                        .base_tau = 0.0f,
                                        .coupling = false})
      : cmd_(cmd),
        pid_yaw_angle_(param.pid_yaw_angle),
        pid_yaw_omega_(param.pid_yaw_omega),
        pid_pit_angle_(param.pid_pit_angle),
        pid_pit_omega_(param.pid_pit_omega),
        motor_yaw_(&motor_yaw),
        motor_pit_(&motor_pit),
        pit_max_angle_(param.pit_max_angle),
        pit_min_angle_(param.pit_min_angle),
        pit_lc_(param.pit_lc),
        pit_theta_(param.pit_theta),
        yaw_k_(param.yaw_k),
        j_pit_(param.j_pit),
        j_yaw_(param.j_yaw),
        pit_zero_(param.pit_zero),
        yaw_zero_(param.yaw_zero),
        patrol_range_(param.patrol_range),
        patrol_omega_(param.patrol_omega),
        reverse_flag_(param.reverse_flag ? 1.0f : -1.0f),
        euler_topic_name_(param.euler_topic_name),
        gyro_topic_name_(param.gyro_topic_name),
        gimbal_cmd_topic_name_(param.gimbal_cmd_topic_name),
        referee_(referee),
        ident_file_(LibXR::RamFS::CreateFile("gimbal_ident", IdentCommand, this)),
        mbc_(mbc),
        mbc_pit_(mbc.enable_pit && GimbalMbc::Valid(mbc)),
        mbc_yaw_(mbc.enable_yaw && GimbalMbc::Valid(mbc)),
        ctrl_file_(LibXR::RamFS::CreateFile("gimbal_ctrl", CtrlCommand, this))
  {
    UNUSED(referee_);

    if ((mbc.enable_pit || mbc.enable_yaw) && !GimbalMbc::Valid(mbc))
    {
      XR_LOG_WARN("Gimbal: invalid mbc parameters, using the PID");
    }

    if (ramfs != nullptr)
    {
      ramfs->Add(ident_file_);
      ramfs->Add(ctrl_file_);
    }

    thread_.Create(this, ThreadFunc, "GimbalThread", param.task_stack_depth, param.thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          // 与 10.14 版本一致：遥控器上线后云台直接进入可控制（COMMON）状态。
          // As in the 10.14 version: enter the controllable COMMON mode when the RC comes online.
          gimbal->SetMode(GimbalEvent::SET_MODE_COMMON);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id)
        {
          UNUSED(in_isr);
          gimbal->SetMode(static_cast<GimbalEvent>(event_id));
        },
        this);

    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd_.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX), callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON), callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_LOW_SENSITIVITY),
                           callback);
  };

  /**
   * @brief 控制线程函数：订阅命令、姿态与角速度，每 2 ms 执行一轮 Update、ParseCMD 与 Control。
   *        Control thread function: subscribes to the command, attitude and angular
   *        velocity Topics and runs Update, ParseCMD and Control every 2 ms.
   *
   * @param gimbal Gimbal 实例指针。
   *               Pointer to the Gimbal instance.
   */
  static void ThreadFunc(Gimbal* gimbal)
  {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(gimbal->gimbal_cmd_topic_name_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->euler_topic_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        gimbal->gyro_topic_name_);
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true)
    {
      if (cmd_suber.Available())
      {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available())
      {
        gimbal->euler_ = euler_suber.GetData();
        gimbal->euler_.Pitch() *= -1.0f;
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available())
      {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gimbal->gyro_data_.y() *= -1.0f;
        gyro_suber.StartWaiting();
      }

      gimbal->Update();
      gimbal->ParseCMD();
      gimbal->Control();
      LibXR::Thread::Sleep(2);
    }
  };

  /**
   * @brief 刷新电机反馈与采样间隔，并发布 yaw、pitch 电机相对零点的角度。
   *        Refresh the motor feedback and the sample interval, and publish the yaw and
   *        pitch motor angles relative to their zero points.
   */
  void Update()
  {
    motor_yaw_->Update();
    motor_pit_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;
    this->tick_us_ = static_cast<uint32_t>(static_cast<uint64_t>(now));

    abs_angle_pit_ = motor_pit_feedback_.abs_angle - pit_zero_;
    abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;

    topic_yaw_angle_.Publish(abs_angle_yaw_);
    topic_pit_angle_.Publish(abs_angle_pit_);
  }

  /**
   * @brief 按 CMD 控制模式与云台模式解析命令，更新 yaw、pitch 目标及其导数。
   *        Parse the command according to the CMD control mode and the gimbal mode, and
   *        update the yaw and pitch targets and their derivatives.
   */
  void ParseCMD()
  {
    // 辨识模式只接管目标，其余（模式、使能、限位）不变
    // Identification mode takes over the targets only; modes, enabling and limits are
    // unchanged.
    ServiceIdent();
    if (ident_kind_ != IdentKind::NONE)
    {
      RunIdent();
      return;
    }

    if (cmd_.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL)
    {
      if (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY)
      {
        target_yaw_cmd_ += cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 0.1f;
        target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 0.1f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
      }
      else
      {
        target_yaw_cmd_ += cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
      }
    }
    else
    {
      if (cmd_.GetAIGimbalStatus())
      {
        target_yaw_cmd_ = cmd_data_.yaw;
        target_pit_cmd_ = cmd_data_.pit;
        target_pit_dot_ = cmd_data_.pit_dot;
        target_pit_ddot_ = cmd_data_.pit_ddot;
        target_yaw_dot_ = cmd_data_.yaw_dot;
        target_yaw_ddot_ = cmd_data_.yaw_ddot;
      }
      else
      {
        if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL)
        {
          target_pit_cmd_ -=
              patrol_range_ * (2 / M_PI) *
              asin(sin(patrol_omega_ *
                       (LibXR::Timebase::GetMilliseconds() - patrol_start_time))) /
              1000.0f;
          target_yaw_cmd_ += 1 * dt_;
        }
        else
        {
          target_yaw_cmd_ -= cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
          target_pit_cmd_ += cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED * 1.0f;
          target_pit_dot_ = 0.0f;
          target_pit_ddot_ = 0.0f;
          target_yaw_dot_ = 0.0f;
          target_yaw_ddot_ = 0.0f;
        }
      }
    }
  }

  /**
   * @brief 限制 pitch 目标并计算两轴输出；RELAX 模式下电机 Relax，其他模式按电机状态使能、清错或以力矩模式下发。
   *        Clamp the pitch target and compute both axis outputs; in RELAX mode the motors
   *        are relaxed, otherwise they are enabled, cleared of errors or sent torque
   *        commands depending on their state.
   */
  void Control()
  {
    ServiceCtrl();
    // RELAX 时观测器跟随实际角速度，切回控制时从当前状态开始
    // In RELAX the observers follow the measured rates, so control resumes from the
    // current state.
    if (current_mode_ == GimbalEvent::SET_MODE_RELAX)
    {
      mbc_.Reset(euler_.Pitch(), gyro_data_.y(), gyro_data_.z());
    }
    this->torque_ = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    float out_pit = 0.0f;
    float out_yaw = 0.0f;

    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
               pit_max_angle_, pit_min_angle_, reverse_flag_);
    Solve(out_pit, out_yaw, target_pit_cmd_, target_yaw_cmd_, dt_);
    if (log_on_.load(std::memory_order_relaxed))
    {
      PublishLog(out_pit, out_yaw);
    }
    auto yaw_motor_cmd =
        Motor::MotorCmd({.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_yaw});
    auto pit_motor_cmd =
        Motor::MotorCmd({.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_pit});

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX)
    {
      motor_yaw_->Relax();
      motor_pit_->Relax();
      return;
    }

    auto motor_control =
        [&](Motor* motor, const Motor::Feedback& fb, const Motor::MotorCmd& cmd)
    {
      if (fb.state == 0)
      {
        motor->Enable();
      }
      else if (fb.state != 0 and fb.state != 1)
      {
        motor->ClearError();
      }
      else
      {
        motor->Control(cmd);
      }
    };

    motor_control(motor_pit_, motor_pit_feedback_, pit_motor_cmd);
    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  /**
   * @brief 获取云台模式事件对象，激活 GimbalEvent 对应的事件 ID 即切换模式。
   *        Get the gimbal mode event object; activating the event ID of a GimbalEvent
   *        value switches the mode.
   *
   * @return 云台模式事件对象的引用。
   *         Reference to the gimbal mode event object.
   */
  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  CMD& cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_pit_omega_;
  Motor* motor_yaw_;
  Motor* motor_pit_;
  float torque_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ = LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ = LibXR::Topic::CreateTopic<float>("pitchmotor_angle");

  float pit_max_angle_ = 0.0f;
  float pit_min_angle_ = 0.0f;
  float pit_lc_ = 0.0f;
  float pit_theta_ = 0.0f;
  float yaw_k_ = 0.0f;
  float target_yaw_dot_ = 0.0f;
  float target_yaw_ddot_ = 0.0f;
  float target_pit_dot_ = 0.0f;
  float target_pit_ddot_ = 0.0f;
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
  LibXR::MillisecondTimestamp patrol_start_time = 0.0f;
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  const char* euler_topic_name_;
  const char* gyro_topic_name_;
  const char* gimbal_cmd_topic_name_;
  Referee* referee_;
  LibXR::Thread thread_;

  // ---- 逐拍日志与辨识模式 / Per-tick log and identification mode ----

  /// 辨识类型 / Identification kind
  enum class IdentKind : uint8_t
  {
    NONE = 0,       ///< 无辨识，普通控制 / Off, normal control
    PITCH = 1,      ///< pitch 三角波扫描 / Pitch triangle sweep
    YAW = 2,        ///< yaw 梯形速度 / Yaw trapezoid velocity
    PITCH_SINE = 3  ///< pitch 正弦 / Pitch sine
  };

  /// 终端线程交给控制线程的辨识请求
  /// Identification request handed from the terminal thread to the control thread
  struct IdentRequest
  {
    IdentKind kind = IdentKind::NONE;
    bool stop = false;
    float a = 0.0f;  ///< pitch：速度 (rad/s)；yaw：pitch 角 (rad)；pitch 正弦：中心 (rad)
                     ///< / pitch: rate; yaw: pitch angle; pitch sine: center
    float b = 0.0f;  ///< yaw：最大角速度 (rad/s)；pitch 正弦：频率 (Hz) / yaw: peak rate;
                     ///< pitch sine: frequency
    float c = 0.0f;  ///< yaw：角加速度 (rad/s^2)；pitch 正弦：幅值 (rad) / yaw:
                     ///< acceleration; pitch sine: amplitude
    uint32_t cycles = 0;
  };

  /// pitch 扫描距欧拉角限位的余量 (rad) / Margin of the pitch sweep from the Euler limits
  /// (rad)
  static constexpr float IDENT_PITCH_MARGIN = 0.05f;
  /// yaw 梯形的匀速时间 (s) / Cruise time of the yaw trapezoid (s)
  static constexpr float IDENT_YAW_CRUISE_S = 0.5f;
  /// yaw 梯形每段之间的停顿 (s) / Pause between the yaw trapezoid segments (s)
  static constexpr float IDENT_YAW_PAUSE_S = 0.3f;
  /// 辨识结束前的保持时间 (s) / Hold time at the end of a run (s)
  static constexpr float IDENT_HOLD_S = 0.5f;
  /// yaw 辨识前把 pitch 摆到位的速度 (rad/s)
  /// Rate at which pitch is brought into place before a yaw run (rad/s)
  static constexpr float IDENT_PITCH_SETTLE_SPEED = 0.3f;

  LibXR::Topic log_topic_ = LibXR::Topic::CreateTopic<GimbalLog>("gimbal_log");
  std::atomic<bool> log_on_{false};
  uint32_t tick_us_ = 0;
  float ff_pit_ = 0.0f;
  float fb_pit_ = 0.0f;
  float grav_pit_ = 0.0f;
  float ff_yaw_ = 0.0f;
  float fb_yaw_ = 0.0f;
  float eso_pit_ = 0.0f;
  float eso_yaw_ = 0.0f;

  IdentRequest ident_req_;
  std::atomic<bool> ident_req_pending_{false};
  std::atomic<bool> ident_abort_{false};
  IdentKind ident_kind_ =
      IdentKind::NONE;  ///< 控制线程写，终端线程只读 / Written by the control thread,
                        ///< read-only for the terminal thread
  uint8_t ident_phase_ = 0;
  uint32_t ident_cycle_ = 0;
  float ident_t_ = 0.0f;
  float ident_settle_t_ = 0.0f;
  float ident_dir_ = 1.0f;
  float ident_last_dot_ = 0.0f;
  float ident_start_pit_ = 0.0f;
  IdentRequest ident_run_;
  bool ident_log_auto_ = false;
  LibXR::RamFS::File ident_file_;

  // ---- 控制器选择与在线调参 / Controller selection and online tuning ----

  /// 终端线程交给控制线程的控制器请求
  /// Controller request handed from the terminal thread to the control thread
  struct CtrlRequest
  {
    bool mbc_pit = false;
    bool mbc_yaw = false;
    GimbalMbc::Param mbc{};
    float pit_lc = 0.0f;
    float pit_theta = 0.0f;
  };

  GimbalMbc mbc_;
  /// 各轴是否用模型补偿控制器；控制线程写，终端线程只读
  /// Whether each axis uses the model-based controller; written by the control thread,
  /// read-only for the terminal thread
  bool mbc_pit_ = false;
  bool mbc_yaw_ = false;
  LibXR::RamFS::File ctrl_file_;
  CtrlRequest ctrl_req_;
  std::atomic<bool> ctrl_req_pending_{false};

  /**
   * @brief 把 pitch 电机角度上下限换算为欧拉角范围，并限制 pitch 目标；上下限均为 0 时不限位。
   *        Convert the pitch motor angle limits to an Euler angle range and clamp the
   *        pitch target; limiting is disabled when both limits are 0.
   *
   * @param target_pit 目标 pitch 角度 (rad)，被限幅后写回。
   *                   Target pitch angle (rad), clamped in place.
   * @param now_eulr_angle 当前 pitch 欧拉角 (rad)。
   *                       Current pitch Euler angle (rad).
   * @param now_motor_angle 当前 pitch 电机角度 (rad)。
   *                        Current pitch motor angle (rad).
   * @param motor_max 电机角度上限 (rad)。
   *                  Motor angle upper limit (rad).
   * @param motor_min 电机角度下限 (rad)。
   *                  Motor angle lower limit (rad).
   * @param sign 方向符号，电机角与欧拉角同向为 1，反向为 -1。
   *             Direction sign, 1 when the motor angle and the Euler angle have the same
   *             direction, -1 otherwise.
   */
  void PitchLimit(float& target_pit, float now_eulr_angle, float now_motor_angle,
                  float motor_max, float motor_min, float sign)
  {
    if ((motor_max == 0.0f) && (motor_min == 0.0f))
    {
      return;
    };

    float lower_bound = 0.0f;
    float upper_bound = 0.0f;
    PitchBounds(now_eulr_angle, now_motor_angle, motor_max, motor_min, sign, lower_bound,
                upper_bound);
    target_pit = std::clamp(target_pit, lower_bound, upper_bound);
  }

  /**
   * @brief 把 pitch 电机角度上下限换算为欧拉角范围，PitchLimit 与辨识扫描共用。
   *        Convert the pitch motor angle limits to an Euler angle range; shared by
   *        PitchLimit and the identification sweep.
   *
   * @param now_eulr_angle 当前 pitch 欧拉角 (rad)。
   *                       Current pitch Euler angle (rad).
   * @param now_motor_angle 当前 pitch 电机角度 (rad)。
   *                        Current pitch motor angle (rad).
   * @param motor_max 电机角度上限 (rad)。
   *                  Motor angle upper limit (rad).
   * @param motor_min 电机角度下限 (rad)。
   *                  Motor angle lower limit (rad).
   * @param sign 方向符号，电机角与欧拉角同向为 1，反向为 -1。
   *             Direction sign, 1 when the motor angle and the Euler angle have the same
   *             direction, -1 otherwise.
   * @param lower_bound 欧拉角下界 (rad)。
   *                    Euler angle lower bound (rad).
   * @param upper_bound 欧拉角上界 (rad)。
   *                    Euler angle upper bound (rad).
   */
  static void PitchBounds(float now_eulr_angle, float now_motor_angle, float motor_max,
                          float motor_min, float sign, float& lower_bound,
                          float& upper_bound)
  {
    LibXR::CycleValue<float> cycle_motor_min(motor_min);
    LibXR::CycleValue<float> cycle_motor_max(motor_max);

    float diff_min = cycle_motor_min - now_motor_angle;
    float diff_max = cycle_motor_max - now_motor_angle;
    float pitch_bound_0 = now_eulr_angle + diff_min / sign;
    float pitch_bound_1 = now_eulr_angle + diff_max / sign;

    upper_bound = std::max(pitch_bound_0, pitch_bound_1);
    lower_bound = std::min(pitch_bound_0, pitch_bound_1);
  }

  /**
   * @brief 解算两轴的力矩输出：角度环、角速度环、转动惯量前馈与补偿项。
   *        Solve the torque outputs of both axes: angle loop, angular-velocity loop,
   *        moment-of-inertia feedforward and compensation terms.
   *
   * @param pit_output pitch 轴输出。
   *                   Pitch axis output.
   * @param yaw_output yaw 轴输出。
   *                   Yaw axis output.
   * @param target_pit_angle 目标 pitch 角度 (rad)。
   *                         Target pitch angle (rad).
   * @param target_yaw_angle 目标 yaw 角度 (rad)。
   *                         Target yaw angle (rad).
   * @param dt_ 控制周期 (s)。
   *            Control period (s).
   */
  void Solve(float& pit_output, float& yaw_output, float target_pit_angle,
             const LibXR::CycleValue<float>& target_yaw_angle, float dt_)
  {
    SolvePid(pit_output, yaw_output, target_pit_angle, target_yaw_angle, dt_);
    if (!mbc_pit_ && !mbc_yaw_)
    {
      return;
    }
    // 两种控制器都算，按轴选用；观测器按实际下发的力矩继续跟踪
    // Both controllers run and each axis takes one; the observers follow the torque
    // actually sent
    float mbc_pit = 0.0f;
    float mbc_yaw = 0.0f;
    SolveMbc(mbc_pit, mbc_yaw, target_pit_angle, target_yaw_angle, dt_);
    const GimbalMbc::Terms& terms = mbc_.GetTerms();
    if (mbc_pit_)
    {
      pit_output = mbc_pit;
      ff_pit_ = terms.ff_pit;
      fb_pit_ = terms.fb_pit;
      eso_pit_ = terms.eso_pit;
    }
    if (mbc_yaw_)
    {
      yaw_output = mbc_yaw;
      ff_yaw_ = terms.ff_yaw;
      fb_yaw_ = terms.fb_yaw;
      eso_yaw_ = terms.eso_yaw;
    }
    mbc_.Applied(pit_output, yaw_output);
  }

  /**
   * @brief 用串级 PID 解算两轴力矩，并记录各分项。
   *        Solve the torques of both axes with the cascaded PIDs and record the terms.
   *
   * @param pit_output pitch 轴输出。
   *                   Pitch axis output.
   * @param yaw_output yaw 轴输出。
   *                   Yaw axis output.
   * @param target_pit_angle 目标 pitch 角度 (rad)。
   *                         Target pitch angle (rad).
   * @param target_yaw_angle 目标 yaw 角度 (rad)。
   *                         Target yaw angle (rad).
   * @param dt_ 控制周期 (s)。
   *            Control period (s).
   */
  void SolvePid(float& pit_output, float& yaw_output, float target_pit_angle,
                const LibXR::CycleValue<float>& target_yaw_angle, float dt_)
  {
    eso_pit_ = 0.0f;
    eso_yaw_ = 0.0f;
    float pit_error = target_pit_angle - euler_.Pitch();
    float target_pit_omega =
        pid_pit_angle_.Calculate(pit_error, 0.0f, dt_) + target_pit_dot_;
    float ff_pit = JFeedforward(target_pit_omega, last_pit_omega_, dt_, j_pit_) +
                   j_pit_ * target_pit_ddot_;
    float gravity_ff_pit = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    float fb_pit = pid_pit_omega_.Calculate(target_pit_omega, gyro_data_.y(), dt_);
    pit_output = ff_pit + fb_pit + gravity_ff_pit;
    ff_pit_ = ff_pit;
    fb_pit_ = fb_pit;
    grav_pit_ = gravity_ff_pit;
    last_pit_omega_ = target_pit_omega;
    float yaw_error = target_yaw_angle - euler_.Yaw();
    float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_) + target_yaw_dot_;
    float ff_yaw = JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_) +
                   j_yaw_ * target_yaw_ddot_;
    float fb_yaw = pid_yaw_omega_.Calculate(target_yaw_omega, gyro_data_.z(), dt_);
    yaw_output = ff_yaw + fb_yaw + motor_yaw_feedback_.omega * this->yaw_k_;
    ff_yaw_ = ff_yaw;
    fb_yaw_ = fb_yaw;
    last_yaw_omega_ = target_yaw_omega;
  }

  /**
   * @brief 用模型补偿控制器解算两轴力矩，重力补偿与 PID 路径相同；分项由 GetTerms 取得。
   *        Solve the torques of both axes with the model-based controller; the gravity
   *        compensation is the same as on the PID path; the terms come from GetTerms.
   *
   * @param pit_output pitch 轴输出。
   *                   Pitch axis output.
   * @param yaw_output yaw 轴输出。
   *                   Yaw axis output.
   * @param target_pit_angle 目标 pitch 角度 (rad)。
   *                         Target pitch angle (rad).
   * @param target_yaw_angle 目标 yaw 角度 (rad)。
   *                         Target yaw angle (rad).
   * @param dt 控制周期 (s)。
   *           Control period (s).
   */
  void SolveMbc(float& pit_output, float& yaw_output, float target_pit_angle,
                const LibXR::CycleValue<float>& target_yaw_angle, float dt)
  {
    const float gravity = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    const GimbalMbc::Reference ref{.pit_dot = target_pit_dot_,
                                   .pit_ddot = target_pit_ddot_,
                                   .yaw_dot = target_yaw_dot_,
                                   .yaw_ddot = target_yaw_ddot_};
    // pitch 关节与欧拉角的方向由 reverse_flag 给出；yaw 电机与 yaw 同向
    // The pitch joint direction relative to the Euler angle comes from reverse_flag; the
    // yaw motor turns with the yaw
    mbc_.Step(dt, target_pit_angle - euler_.Pitch(), target_yaw_angle - euler_.Yaw(),
              euler_.Pitch(), gyro_data_.y(), gyro_data_.z(),
              reverse_flag_ * motor_pit_feedback_.abs_angle, motor_yaw_feedback_.omega, ref,
              gravity, pit_output, yaw_output);
  }

  /**
   * @brief 当前日志 mode 字段：yaw 用模型补偿控制器加 10000，pitch 用加 20000，RELAX 加
   *        1000，加辨识类型 * 100 与阶段，编码见 GimbalLog.hpp。
   *        Current mode field of the log: 10000 added when the yaw uses the model-based
   *        controller, 20000 when the pitch does, 1000 for RELAX, plus identification
   *        kind * 100 and the phase; see GimbalLog.hpp for the encoding.
   *
   * @return mode 字段。
   *         Mode field.
   */
  uint32_t LogMode() const
  {
    return (mbc_yaw_ ? 10000u : 0u) + (mbc_pit_ ? 20000u : 0u) +
           (current_mode_ == GimbalEvent::SET_MODE_RELAX ? 1000u : 0u) +
           static_cast<uint32_t>(ident_kind_) * 100u + ident_phase_;
  }

  /**
   * @brief 发布本拍的日志：目标、姿态、各分项力矩与电机反馈。
   *        Publish the log of this tick: targets, attitude, torque terms and motor
   *        feedback.
   *
   * @param out_pit Solve 算出的 pitch 力矩。
   *                Pitch torque computed by Solve.
   * @param out_yaw Solve 算出的 yaw 力矩。
   *                Yaw torque computed by Solve.
   */
  void PublishLog(float out_pit, float out_yaw)
  {
    GimbalLog log;
    log.t_us = tick_us_;
    log.dt_us = static_cast<uint32_t>(dt_ * 1.0e6f);
    log.mode = LogMode();
    log.tgt_pit = target_pit_cmd_;
    log.tgt_yaw = target_yaw_cmd_;
    log.tgt_pit_dot = target_pit_dot_;
    log.tgt_yaw_dot = target_yaw_dot_;
    log.pit = euler_.Pitch();
    log.yaw = euler_.Yaw();
    log.gyro_pit = gyro_data_.y();
    log.gyro_yaw = gyro_data_.z();
    log.ff_pit = ff_pit_;
    log.fb_pit = fb_pit_;
    log.grav_pit = grav_pit_;
    log.u_pit = out_pit;
    log.ff_yaw = ff_yaw_;
    log.fb_yaw = fb_yaw_;
    log.u_yaw = out_yaw;
    log.mot_pit_angle = motor_pit_feedback_.abs_angle;
    log.mot_pit_omega = motor_pit_feedback_.omega;
    log.mot_pit_torque = motor_pit_feedback_.torque;
    log.mot_yaw_angle = motor_yaw_feedback_.abs_angle;
    log.mot_yaw_omega = motor_yaw_feedback_.omega;
    log.mot_yaw_torque = motor_yaw_feedback_.torque;
    log.eso_pit = eso_pit_;
    log.eso_yaw = eso_yaw_;
    log_topic_.Publish(log);
  }

  /**
   * @brief 取得 pitch 扫描允许的欧拉角范围：PitchLimit 的换算结果两端各收进 0.05 rad。
   *        Get the Euler angle range allowed for the pitch sweep: the PitchLimit
   *        conversion, narrowed by 0.05 rad at both ends.
   *
   * @param lower 范围下界 (rad)。
   *              Lower end of the range (rad).
   * @param upper 范围上界 (rad)。
   *              Upper end of the range (rad).
   * @return 配置了限位且范围大于 0.1 rad 时为 true。
   *         True when limits are configured and the range is wider than 0.1 rad.
   */
  bool PitchSweepRange(float& lower, float& upper) const
  {
    if ((pit_max_angle_ == 0.0f) && (pit_min_angle_ == 0.0f))
    {
      return false;
    }
    PitchBounds(euler_.Pitch(), motor_pit_feedback_.abs_angle, pit_max_angle_,
                pit_min_angle_, reverse_flag_, lower, upper);
    lower += IDENT_PITCH_MARGIN;
    upper -= IDENT_PITCH_MARGIN;
    return (upper - lower) > 0.1f;
  }

  /**
   * @brief 控制线程每拍处理一次：先处理终止标志，再取走终端线程交来的辨识请求。
   *        Called once per tick by the control thread: handle the abort flag first, then
   *        take the identification request handed over by the terminal thread.
   */
  void ServiceIdent()
  {
    if (ident_abort_.exchange(false, std::memory_order_acq_rel) &&
        ident_kind_ != IdentKind::NONE)
    {
      EndIdent(false);
    }

    if (!ident_req_pending_.load(std::memory_order_acquire))
    {
      return;
    }
    const IdentRequest req = ident_req_;
    ident_req_pending_.store(false, std::memory_order_release);

    if (req.stop)
    {
      if (ident_kind_ != IdentKind::NONE)
      {
        EndIdent(true);
      }
      return;
    }
    StartIdent(req);
  }

  /**
   * @brief 把两轴目标置为当前姿态并复位 PID，与 SetMode 切入控制模式时一致。
   *        Set both targets to the current attitude and reset the PIDs, as SetMode does
   *        when entering a control mode.
   */
  void ResetTargetsToCurrent()
  {
    target_pit_cmd_ = euler_.Pitch();
    target_yaw_cmd_ = euler_.Yaw();
    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    last_pit_omega_ = 0.0f;
    last_yaw_omega_ = 0.0f;
  }

  /**
   * @brief 开始一次辨识；RELAX、已在辨识或 pitch 扫描没有可用范围时忽略。
   *        Start an identification run; ignored in RELAX, while another run is active, or
   *        when the pitch sweep has no usable range.
   *
   * @param req 辨识请求。
   *            Identification request.
   */
  void StartIdent(const IdentRequest& req)
  {
    float lower = 0.0f;
    float upper = 0.0f;
    if (current_mode_ == GimbalEvent::SET_MODE_RELAX || ident_kind_ != IdentKind::NONE ||
        (req.kind == IdentKind::PITCH && !PitchSweepRange(lower, upper)))
    {
      XR_LOG_WARN("Gimbal: identification request ignored");
      return;
    }

    ResetTargetsToCurrent();
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
    target_pit_dot_ = 0.0f;
    target_pit_ddot_ = 0.0f;

    ident_run_ = req;
    ident_kind_ = req.kind;
    ident_phase_ = 1;
    ident_cycle_ = 0;
    ident_t_ = 0.0f;
    ident_settle_t_ = 0.0f;
    ident_dir_ = (req.kind == IdentKind::PITCH) ? -1.0f : 1.0f;
    ident_last_dot_ = 0.0f;
    ident_start_pit_ = target_pit_cmd_;
    if (!log_on_.load(std::memory_order_relaxed))
    {
      log_on_.store(true, std::memory_order_relaxed);
      ident_log_auto_ = true;
    }
  }

  /**
   * @brief 结束辨识；reset_targets 为 true 时回到普通控制：目标取当前姿态并复位 PID。
   *        自动打开的日志同时关闭。
   *        End the identification; with reset_targets true normal control resumes: the
   *        targets take the current attitude and the PIDs are reset. A log that was
   *        switched on automatically is switched off.
   *
   * @param reset_targets 是否把目标置为当前姿态并复位 PID。
   *                      Whether to set the targets to the current attitude and reset the
   *                      PIDs.
   */
  void EndIdent(bool reset_targets)
  {
    ident_kind_ = IdentKind::NONE;
    ident_phase_ = 0;
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
    target_pit_dot_ = 0.0f;
    target_pit_ddot_ = 0.0f;
    if (reset_targets)
    {
      ResetTargetsToCurrent();
    }
    if (ident_log_auto_)
    {
      log_on_.store(false, std::memory_order_relaxed);
      ident_log_auto_ = false;
    }
  }

  /**
   * @brief 辨识模式下替代 ParseCMD 的一拍：按辨识类型更新两轴目标及其导数。
   *        One tick that replaces ParseCMD in identification mode: update both axis
   *        targets and their derivatives according to the identification kind.
   */
  void RunIdent()
  {
    if (current_mode_ == GimbalEvent::SET_MODE_RELAX)
    {
      EndIdent(false);
      return;
    }
    ident_t_ += dt_;
    if (ident_kind_ == IdentKind::PITCH)
    {
      RunIdentPitch();
    }
    else if (ident_kind_ == IdentKind::PITCH_SINE)
    {
      RunIdentPitchSine();
    }
    else
    {
      RunIdentYaw();
    }
  }

  /**
   * @brief pitch 扫描：以恒定速度在 [下限 + 0.05, 上限 - 0.05] 间走三角波，yaw 保持；
   *        先趋近下端，走完指定周期（下端到上端再回下端为一个周期）后回到起点并保持。
   *        Pitch sweep: a triangle wave at constant speed between [lower + 0.05, upper -
   *        0.05] with yaw held; it first approaches the lower end, runs the given number
   *        of cycles (lower end to upper end and back is one cycle), then returns to the
   *        start and holds.
   */
  void RunIdentPitch()
  {
    float lower = 0.0f;
    float upper = 0.0f;
    if (!PitchSweepRange(lower, upper))
    {
      EndIdent(true);
      return;
    }

    const float speed = ident_run_.a;
    // 先切换阶段再输出 / Phase transitions come before the output
    if (ident_phase_ == 1 && target_pit_cmd_ <= lower)
    {
      ident_phase_ = 2;
    }
    if (ident_phase_ == 2 && target_pit_cmd_ >= upper)
    {
      ident_phase_ = 3;
    }
    if (ident_phase_ == 3 && target_pit_cmd_ <= lower)
    {
      ident_phase_ = (++ident_cycle_ >= ident_run_.cycles) ? 4 : 2;
    }

    float dot = 0.0f;
    if (ident_phase_ == 4)
    {
      const float remaining = ident_start_pit_ - target_pit_cmd_;
      if (fabsf(remaining) <= speed * dt_)
      {
        target_pit_cmd_ = ident_start_pit_;
        ident_phase_ = 5;
        ident_t_ = 0.0f;
      }
      else
      {
        dot = (remaining > 0.0f) ? speed : -speed;
      }
    }
    else if (ident_phase_ == 5)
    {
      if (ident_t_ >= IDENT_HOLD_S)
      {
        EndIdent(true);
        return;
      }
    }
    else
    {
      dot = (ident_phase_ == 2) ? speed : -speed;
    }

    target_pit_cmd_ += dot * dt_;
    target_pit_dot_ = dot;
    target_pit_ddot_ = 0.0f;
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
  }

  /**
   * @brief pitch 正弦：先把 pitch 摆到中心并稳定，随后目标 = 中心 + 幅值 * r(t) * sin(2 pi f t)，
   *        target_pit_dot_ 与 target_pit_ddot_ 取解析导数，yaw 保持；跑满指定周期数后保持
   *        0.5 s 退出。包络 r(t) 在第一个与最后一个周期内按升余弦从 0 升到 1、再降回 0，
   *        起止时目标角速度连续，惯量前馈没有尖峰。
   *        Pitch sine: pitch is first brought to the center and settles; the target is then
   *        center + amplitude * r(t) * sin(2 pi f t) with the analytic derivatives in
   *        target_pit_dot_ and target_pit_ddot_ and yaw held; after the given number of
   *        periods it holds for 0.5 s and exits. The envelope r(t) rises from 0 to 1 over
   *        the first period and falls back over the last one (raised cosine), so the target
   *        rate is continuous at both ends and the inertia feed-forward has no spike.
   */
  void RunIdentPitchSine()
  {
    const float center = ident_run_.a;
    const float w = 2.0f * static_cast<float>(M_PI) * ident_run_.b;
    const float amp = ident_run_.c;

    float pit_dot = 0.0f;
    float pit_ddot = 0.0f;

    if (ident_phase_ == 1)
    {
      // pitch 就位，规则同 yaw 辨识 / Pitch settle, same rules as the yaw identification
      const float err = center - target_pit_cmd_;
      if (fabsf(err) <= IDENT_PITCH_SETTLE_SPEED * dt_)
      {
        target_pit_cmd_ = center;
      }
      else
      {
        pit_dot = (err > 0.0f) ? IDENT_PITCH_SETTLE_SPEED : -IDENT_PITCH_SETTLE_SPEED;
        target_pit_cmd_ += pit_dot * dt_;
      }
      if (target_pit_cmd_ == center && fabsf(center - euler_.Pitch()) < 0.02f)
      {
        ident_settle_t_ += dt_;
      }
      else
      {
        ident_settle_t_ = 0.0f;
      }
      if (ident_settle_t_ >= 1.0f)
      {
        ident_phase_ = 2;
        ident_t_ = 0.0f;
      }
      else if (ident_t_ > 10.0f)
      {
        XR_LOG_WARN("Gimbal: pitch did not settle, identification stopped");
        EndIdent(true);
        return;
      }
    }
    else if (ident_phase_ == 2)
    {
      if (ident_t_ * ident_run_.b >= static_cast<float>(ident_run_.cycles))
      {
        ident_phase_ = 3;
        ident_t_ = 0.0f;
        target_pit_cmd_ = center;
      }
      else
      {
        const float sin_wt = sinf(w * ident_t_);
        const float cos_wt = cosf(w * ident_t_);
        // 包络及其导数 / Envelope and its derivatives
        const float total = static_cast<float>(ident_run_.cycles) / ident_run_.b;
        const float ramp = fminf(1.0f / ident_run_.b, 0.5f * total);
        const float edge = fminf(ident_t_, total - ident_t_);
        const float dir = (ident_t_ <= total - ident_t_) ? 1.0f : -1.0f;
        float r = 1.0f;
        float r_dot = 0.0f;
        float r_ddot = 0.0f;
        if (edge < ramp)
        {
          const float k = static_cast<float>(M_PI) / ramp;
          r = 0.5f * (1.0f - cosf(k * edge));
          r_dot = dir * 0.5f * k * sinf(k * edge);
          r_ddot = 0.5f * k * k * cosf(k * edge);
        }
        target_pit_cmd_ = center + amp * r * sin_wt;
        pit_dot = amp * (r_dot * sin_wt + r * w * cos_wt);
        pit_ddot = amp * (r_ddot * sin_wt + 2.0f * r_dot * w * cos_wt - r * w * w * sin_wt);
      }
    }
    else if (ident_t_ >= IDENT_HOLD_S)
    {
      EndIdent(true);
      return;
    }

    target_pit_dot_ = pit_dot;
    target_pit_ddot_ = pit_ddot;
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
  }

  /**
   * @brief yaw 梯形速度：先把 pitch 摆到指定角度并稳定，随后每个周期依次为正向加速、匀速
   *        0.5 s、减速、停顿 0.3 s 与反向的同样四段，最后保持后退出。yaw 目标由速度积分
   *        得到，target_yaw_dot_ 与 target_yaw_ddot_ 同步填写。
   *        Yaw trapezoid velocity: pitch is first brought to the given angle and settles;
   *        each cycle is then accelerate, cruise for 0.5 s, decelerate and pause for
   *        0.3 s in the positive direction followed by the same four segments in the
   *        negative direction, and the run exits after a final hold. The yaw target is
   *        the integral of the rate; target_yaw_dot_ and target_yaw_ddot_ are filled.
   */
  void RunIdentYaw()
  {
    const float goal_pit = ident_run_.a;
    const float omega = ident_run_.b;
    const float accel = ident_run_.c;
    const float ramp_t = omega / accel;

    float pit_dot = 0.0f;
    float dot = 0.0f;
    float ddot = 0.0f;

    if (ident_phase_ == 1)
    {
      // pitch 就位：以固定速度走到目标，再在 0.02 rad 内保持 1 s；10 s 内到不了就放弃
      // Pitch settle: move to the target at a fixed rate, then stay within 0.02 rad for
      // 1 s; give up if that is not reached within 10 s
      const float err = goal_pit - target_pit_cmd_;
      if (fabsf(err) <= IDENT_PITCH_SETTLE_SPEED * dt_)
      {
        target_pit_cmd_ = goal_pit;
      }
      else
      {
        pit_dot = (err > 0.0f) ? IDENT_PITCH_SETTLE_SPEED : -IDENT_PITCH_SETTLE_SPEED;
        target_pit_cmd_ += pit_dot * dt_;
      }
      if (target_pit_cmd_ == goal_pit && fabsf(goal_pit - euler_.Pitch()) < 0.02f)
      {
        ident_settle_t_ += dt_;
      }
      else
      {
        ident_settle_t_ = 0.0f;
      }
      if (ident_settle_t_ >= 1.0f)
      {
        ident_phase_ = 2;
        ident_t_ = 0.0f;
      }
      else if (ident_t_ > 10.0f)
      {
        XR_LOG_WARN("Gimbal: pitch did not settle, identification stopped");
        EndIdent(true);
        return;
      }
    }
    else
    {
      // 阶段 2..5 为正向，6..9 为反向；10 为结束保持
      // Phases 2..5 are positive, 6..9 negative; 10 is the end hold
      if ((ident_phase_ == 2 || ident_phase_ == 6) && ident_t_ >= ramp_t)
      {
        ident_phase_++;
        ident_t_ = 0.0f;
      }
      else if ((ident_phase_ == 3 || ident_phase_ == 7) && ident_t_ >= IDENT_YAW_CRUISE_S)
      {
        ident_phase_++;
        ident_t_ = 0.0f;
      }
      else if ((ident_phase_ == 4 || ident_phase_ == 8) && ident_t_ >= ramp_t)
      {
        ident_phase_++;
        ident_t_ = 0.0f;
      }
      else if ((ident_phase_ == 5 || ident_phase_ == 9) && ident_t_ >= IDENT_YAW_PAUSE_S)
      {
        if (ident_phase_ == 5)
        {
          ident_phase_ = 6;
          ident_dir_ = -1.0f;
        }
        else if (++ident_cycle_ >= ident_run_.cycles)
        {
          ident_phase_ = 10;
        }
        else
        {
          ident_phase_ = 2;
          ident_dir_ = 1.0f;
        }
        ident_t_ = 0.0f;
      }

      switch (ident_phase_)
      {
        case 2:
        case 6:
          dot = ident_dir_ * accel * ident_t_;
          ddot = ident_dir_ * accel;
          break;
        case 3:
        case 7:
          dot = ident_dir_ * omega;
          break;
        case 4:
        case 8:
          dot = ident_dir_ * fmaxf(omega - accel * ident_t_, 0.0f);
          ddot = -ident_dir_ * accel;
          break;
        case 10:
          if (ident_t_ >= IDENT_HOLD_S)
          {
            EndIdent(true);
            return;
          }
          break;
        default:
          break;
      }
    }

    target_yaw_cmd_ += 0.5f * (ident_last_dot_ + dot) * dt_;
    ident_last_dot_ = dot;
    target_yaw_dot_ = dot;
    target_yaw_ddot_ = ddot;
    target_pit_dot_ = pit_dot;
    target_pit_ddot_ = 0.0f;
  }

  /**
   * @brief 解析一个有限浮点数参数。
   *        Parse a finite floating-point argument.
   *
   * @param text 参数文本。
   *             Argument text.
   * @param value 解析结果。
   *              Parsed value.
   * @return 整个文本都是有限浮点数时为 true。
   *         True when the whole text is a finite floating-point number.
   */
  static bool ParseFloat(const char* text, float& value)
  {
    char* end = nullptr;
    value = strtof(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
  }

  /**
   * @brief 解析 1..100 的周期数参数。
   *        Parse a cycle count argument in 1..100.
   *
   * @param text 参数文本。
   *             Argument text.
   * @param value 解析结果。
   *              Parsed value.
   * @return 整个文本都是 1..100 的整数时为 true。
   *         True when the whole text is an integer in 1..100.
   */
  static bool ParseCycles(const char* text, uint32_t& value)
  {
    char* end = nullptr;
    const long parsed = strtol(text, &end, 10);
    value = static_cast<uint32_t>(parsed);
    return end != text && *end == '\0' && parsed >= 1 && parsed <= 100;
  }

  /**
   * @brief 把辨识请求交给控制线程；上一个请求尚未取走时最多等 20 ms。
   *        Hand an identification request to the control thread; waits up to 20 ms when
   *        the previous request has not been taken yet.
   *
   * @param req 辨识请求。
   *            Identification request.
   * @return 成功交出为 true。
   *         True when the request was handed over.
   */
  bool PostIdent(const IdentRequest& req)
  {
    for (int i = 0; i < 10 && ident_req_pending_.load(std::memory_order_acquire); i++)
    {
      LibXR::Thread::Sleep(2);
    }
    if (ident_req_pending_.load(std::memory_order_acquire))
    {
      return false;
    }
    ident_req_ = req;
    ident_req_pending_.store(true, std::memory_order_release);
    return true;
  }

  /**
   * @brief `gimbal_ident` 终端命令：开关日志，启动 pitch 扫描或 yaw 梯形辨识，或停止。
   *        The `gimbal_ident` terminal command: switch the log, start the pitch sweep or
   *        the yaw trapezoid identification, or stop.
   *
   * @param self Gimbal 实例。
   *             Gimbal instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 成功为 0，参数无效或被拒绝为 -1。
   *         0 on success, -1 for invalid arguments or a refused request.
   */
  static int IdentCommand(Gimbal* self, int argc, char** argv)
  {
    auto usage = []()
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  log on|off                        - per-tick gimbal_log\r\n">();
      LibXR::STDIO::Printf<
          "  pitch <rad/s> <cycles>            - pitch triangle sweep\r\n">();
      LibXR::STDIO::Printf<
          "  yaw <pit> <rad/s> <rad/s2> <cyc>  - yaw trapezoid at pitch <pit>\r\n">();
      LibXR::STDIO::Printf<
          "  pitchsine <ctr> <Hz> <amp> <cyc>  - pitch sine around <ctr>\r\n">();
      LibXR::STDIO::Printf<
          "  stop                              - back to normal control\r\n">();
    };

    if (argc < 2)
    {
      usage();
      return 0;
    }

    if (strcmp(argv[1], "log") == 0 && argc == 3)
    {
      if (strcmp(argv[2], "on") == 0)
      {
        self->log_on_.store(true, std::memory_order_relaxed);
        self->ident_log_auto_ = false;
      }
      else if (strcmp(argv[2], "off") == 0)
      {
        self->log_on_.store(false, std::memory_order_relaxed);
      }
      else
      {
        usage();
        return -1;
      }
      LibXR::STDIO::Printf<"log %s\r\n">(argv[2]);
      return 0;
    }

    if (strcmp(argv[1], "stop") == 0 && argc == 2)
    {
      IdentRequest req;
      req.stop = true;
      if (!self->PostIdent(req))
      {
        LibXR::STDIO::Printf<"busy\r\n">();
        return -1;
      }
      LibXR::STDIO::Printf<"stop requested\r\n">();
      return 0;
    }

    IdentRequest req;
    float lower = 0.0f;
    float upper = 0.0f;
    const bool has_range = self->PitchSweepRange(lower, upper);
    if (strcmp(argv[1], "pitch") == 0 && argc == 4)
    {
      req.kind = IdentKind::PITCH;
      if (!ParseFloat(argv[2], req.a) || req.a < 0.005f || req.a > 1.0f ||
          !ParseCycles(argv[3], req.cycles))
      {
        LibXR::STDIO::Printf<"Error: speed 0.005..1.0 rad/s, cycles 1..100\r\n">();
        return -1;
      }
      if (!has_range)
      {
        LibXR::STDIO::Printf<
            "Error: pitch limits not configured or range too small\r\n">();
        return -1;
      }
    }
    else if (strcmp(argv[1], "yaw") == 0 && argc == 6)
    {
      req.kind = IdentKind::YAW;
      if (!ParseFloat(argv[2], req.a) || !ParseFloat(argv[3], req.b) ||
          !ParseFloat(argv[4], req.c) || req.b < 0.1f || req.b > 8.0f || req.c < 0.5f ||
          req.c > 200.0f || !ParseCycles(argv[5], req.cycles))
      {
        LibXR::STDIO::Printf<
            "Error: omega 0.1..8 rad/s, accel 0.5..200 rad/s2, cycles 1..100\r\n">();
        return -1;
      }
      if (has_range && (req.a < lower || req.a > upper))
      {
        LibXR::STDIO::Printf<"Error: pitch must be within [%f, %f] rad\r\n">(lower,
                                                                             upper);
        return -1;
      }
    }
    else if (strcmp(argv[1], "pitchsine") == 0 && argc == 6)
    {
      req.kind = IdentKind::PITCH_SINE;
      if (!ParseFloat(argv[2], req.a) || !ParseFloat(argv[3], req.b) ||
          !ParseFloat(argv[4], req.c) || req.b < 0.1f || req.b > 20.0f || req.c <= 0.0f ||
          req.c > 0.5f || !ParseCycles(argv[5], req.cycles))
      {
        LibXR::STDIO::Printf<
            "Error: freq 0.1..20 Hz, amp 0..0.5 rad, cycles 1..100\r\n">();
        return -1;
      }
      if (!has_range)
      {
        LibXR::STDIO::Printf<
            "Error: pitch limits not configured or range too small\r\n">();
        return -1;
      }
      if (req.a - req.c < lower || req.a + req.c > upper)
      {
        LibXR::STDIO::Printf<"Error: center +- amp must be within [%f, %f] rad\r\n">(
            lower, upper);
        return -1;
      }
    }
    else
    {
      usage();
      return -1;
    }

    if (self->current_mode_ == GimbalEvent::SET_MODE_RELAX)
    {
      LibXR::STDIO::Printf<"Error: gimbal is in RELAX mode\r\n">();
      return -1;
    }
    if (self->ident_kind_ != IdentKind::NONE)
    {
      LibXR::STDIO::Printf<"Error: already running, use stop first\r\n">();
      return -1;
    }
    if (!self->PostIdent(req))
    {
      LibXR::STDIO::Printf<"busy\r\n">();
      return -1;
    }
    LibXR::STDIO::Printf<"started\r\n">();
    return 0;
  }

  /**
   * @brief 控制线程每拍处理一次：取走终端线程交来的控制器请求。某轴切回 PID 时复位该轴
   *        PID；观测器只在有轴用模型补偿控制器时运行，启用时复位。目标不变。
   *        Called once per tick by the control thread: take the controller request handed
   *        over by the terminal thread. An axis switched back to the PID has its PIDs
   *        reset; the observers run only while some axis uses the model-based controller
   *        and are reset when it is enabled. The targets are kept.
   */
  void ServiceCtrl()
  {
    if (!ctrl_req_pending_.load(std::memory_order_acquire))
    {
      return;
    }
    const CtrlRequest req = ctrl_req_;
    ctrl_req_pending_.store(false, std::memory_order_release);

    pit_lc_ = req.pit_lc;
    pit_theta_ = req.pit_theta;
    mbc_.SetParam(req.mbc);
    if (!mbc_pit_ && !mbc_yaw_ && (req.mbc_pit || req.mbc_yaw))
    {
      mbc_.Reset(euler_.Pitch(), gyro_data_.y(), gyro_data_.z());
    }
    // 惯量前馈按目标角速度的差分计算，从实测角速度起算避免切换时的尖峰
    // The inertia feed-forward differences the target rate; starting from the measured
    // rate avoids a spike at the switch.
    if (mbc_pit_ && !req.mbc_pit)
    {
      pid_pit_angle_.Reset();
      pid_pit_omega_.Reset();
      last_pit_omega_ = gyro_data_.y();
    }
    if (mbc_yaw_ && !req.mbc_yaw)
    {
      pid_yaw_angle_.Reset();
      pid_yaw_omega_.Reset();
      last_yaw_omega_ = gyro_data_.z();
    }
    mbc_pit_ = req.mbc_pit;
    mbc_yaw_ = req.mbc_yaw;
  }

  /// `gimbal_ctrl set` 可改的浮点参数 / Float parameters accepted by `gimbal_ctrl set`
  struct CtrlField
  {
    const char* name;
    float GimbalMbc::Param::* field;
  };

  static constexpr CtrlField CTRL_FIELDS[] = {
      {"j_pit", &GimbalMbc::Param::j_pit},
      {"j_yaw0", &GimbalMbc::Param::j_yaw0},
      {"j_yaw_c2", &GimbalMbc::Param::j_yaw_c2},
      {"j_yaw_s2", &GimbalMbc::Param::j_yaw_s2},
      {"pit_kp", &GimbalMbc::Param::pit_kp},
      {"pit_kd", &GimbalMbc::Param::pit_kd},
      {"pit_wo", &GimbalMbc::Param::pit_wo},
      {"pit_wmax", &GimbalMbc::Param::pit_wmax},
      {"yaw_kp", &GimbalMbc::Param::yaw_kp},
      {"yaw_kd", &GimbalMbc::Param::yaw_kd},
      {"yaw_wo", &GimbalMbc::Param::yaw_wo},
      {"yaw_wmax", &GimbalMbc::Param::yaw_wmax},
      {"pit_b", &GimbalMbc::Param::pit_b},
      {"pit_tc", &GimbalMbc::Param::pit_tc},
      {"yaw_b", &GimbalMbc::Param::yaw_b},
      {"yaw_tc", &GimbalMbc::Param::yaw_tc},
      {"fric_eps", &GimbalMbc::Param::fric_eps},
      {"pit_torque_max", &GimbalMbc::Param::pit_torque_max},
      {"yaw_torque_max", &GimbalMbc::Param::yaw_torque_max},
      {"pit_yaw_acc", &GimbalMbc::Param::pit_yaw_acc},
      {"base_tau", &GimbalMbc::Param::base_tau},
  };

  /**
   * @brief 打印当前控制器与全部可调参数。
   *        Print the current controller and every tunable parameter.
   */
  void PrintCtrl() const
  {
    const GimbalMbc::Param& p = mbc_.GetParam();
    LibXR::STDIO::Printf<"pit %s\r\nyaw %s\r\n">(mbc_pit_ ? "mbc" : "pid",
                                                  mbc_yaw_ ? "mbc" : "pid");
    LibXR::STDIO::Printf<"pit_lc %f\r\npit_theta %f\r\n">(pit_lc_, pit_theta_);
    for (const CtrlField& f : CTRL_FIELDS)
    {
      LibXR::STDIO::Printf<"%s %f\r\n">(f.name, p.*(f.field));
    }
    LibXR::STDIO::Printf<"coupling %d\r\n">(p.coupling ? 1 : 0);
  }

  /**
   * @brief `gimbal_ctrl` 终端命令：查看参数，在 PID 与模型补偿控制器之间切换，或在线改参数。
   *        参数在控制线程的下一拍生效，不写入 Flash，重启后恢复 yaml 中的值。
   *        The `gimbal_ctrl` terminal command: show the parameters, switch between the
   *        PID and the model-based controller, or change a parameter online. Changes
   *        take effect on the control thread's next tick and are not written to Flash;
   *        a restart restores the yaml values.
   *
   * @param self Gimbal 实例。
   *             Gimbal instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 成功为 0，参数无效或被拒绝为 -1。
   *         0 on success, -1 for invalid arguments or a refused request.
   */
  static int CtrlCommand(Gimbal* self, int argc, char** argv)
  {
    auto usage = []()
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<"  show                 - controller and parameters\r\n">();
      LibXR::STDIO::Printf<"  pid | mbc            - switch both axes\r\n">();
      LibXR::STDIO::Printf<"  pit|yaw pid|mbc      - switch one axis\r\n">();
      LibXR::STDIO::Printf<"  set <name> <value>   - change one parameter\r\n">();
    };

    if (argc < 2)
    {
      usage();
      return 0;
    }
    if (strcmp(argv[1], "show") == 0 && argc == 2)
    {
      self->PrintCtrl();
      return 0;
    }

    // 上一个请求取走后再读当前值，改动不会互相覆盖
    // Read the current values only after the previous request was taken, so changes do
    // not overwrite each other.
    for (int i = 0; i < 10 && self->ctrl_req_pending_.load(std::memory_order_acquire);
         i++)
    {
      LibXR::Thread::Sleep(2);
    }
    if (self->ctrl_req_pending_.load(std::memory_order_acquire))
    {
      LibXR::STDIO::Printf<"busy\r\n">();
      return -1;
    }
    CtrlRequest req;
    req.mbc_pit = self->mbc_pit_;
    req.mbc_yaw = self->mbc_yaw_;
    req.mbc = self->mbc_.GetParam();
    req.pit_lc = self->pit_lc_;
    req.pit_theta = self->pit_theta_;

    if ((strcmp(argv[1], "pid") == 0 || strcmp(argv[1], "mbc") == 0) && argc == 2)
    {
      req.mbc_pit = req.mbc_yaw = strcmp(argv[1], "mbc") == 0;
    }
    else if ((strcmp(argv[1], "pit") == 0 || strcmp(argv[1], "yaw") == 0) && argc == 3 &&
             (strcmp(argv[2], "pid") == 0 || strcmp(argv[2], "mbc") == 0))
    {
      const bool use = strcmp(argv[2], "mbc") == 0;
      if (strcmp(argv[1], "pit") == 0)
      {
        req.mbc_pit = use;
      }
      else
      {
        req.mbc_yaw = use;
      }
    }
    else if (strcmp(argv[1], "set") == 0 && argc == 4)
    {
      float value = 0.0f;
      if (!ParseFloat(argv[3], value))
      {
        LibXR::STDIO::Printf<"Error: value must be a number\r\n">();
        return -1;
      }
      bool found = true;
      if (strcmp(argv[2], "pit_lc") == 0)
      {
        req.pit_lc = value;
      }
      else if (strcmp(argv[2], "pit_theta") == 0)
      {
        req.pit_theta = value;
      }
      else if (strcmp(argv[2], "coupling") == 0)
      {
        req.mbc.coupling = value != 0.0f;
      }
      else
      {
        found = false;
        for (const CtrlField& f : CTRL_FIELDS)
        {
          if (strcmp(argv[2], f.name) == 0)
          {
            req.mbc.*(f.field) = value;
            found = true;
            break;
          }
        }
      }
      if (!found)
      {
        LibXR::STDIO::Printf<"Error: unknown parameter %s (see show)\r\n">(argv[2]);
        return -1;
      }
    }
    else
    {
      usage();
      return -1;
    }

    // 只在要用模型补偿控制器时检查其参数，PID 下可以分步填写
    // The mbc parameters are checked only when the mbc is to be used, so they can be
    // filled in step by step under the PID.
    if ((req.mbc_pit || req.mbc_yaw) && !GimbalMbc::Valid(req.mbc))
    {
      LibXR::STDIO::Printf<
          "Error: mbc parameters invalid (inertia, kd, wo 0..%f, limits)\r\n">(
          GimbalMbc::MAX_WO);
      return -1;
    }
    self->ctrl_req_ = req;
    self->ctrl_req_pending_.store(true, std::memory_order_release);
    LibXR::STDIO::Printf<"ok, pit %s, yaw %s\r\n">(req.mbc_pit ? "mbc" : "pid",
                                                  req.mbc_yaw ? "mbc" : "pid");
    return 0;
  }

  /**
   * @brief 计算转动惯量前馈 J * (target_omega - last_omega) / dt。
   *        Compute the moment-of-inertia feedforward J * (target_omega - last_omega) / dt.
   *
   * @param target_omega 目标角速度 (rad/s)。
   *                     Target angular velocity (rad/s).
   * @param last_omega 上一次的目标角速度 (rad/s)。
   *                   Previous target angular velocity (rad/s).
   * @param dt_ 控制周期 (s)。
   *            Control period (s).
   * @param J 转动惯量 (kg·m^2)。
   *          Moment of inertia (kg·m^2).
   * @return 前馈力矩。
   *         Feedforward torque.
   */
  static float JFeedforward(float target_omega, float last_omega, float dt_, float J)
  {
    float feedforward = 0.0f;
    float delta_omega = target_omega - last_omega;
    feedforward = (J * delta_omega / dt_);
    return feedforward;
  }

  /**
   * @brief 切换云台模式；RELAX 失能电机并清零目标，其他模式以当前姿态为目标并复位 PID。
   *        Switch the gimbal mode; RELAX disables the motors and clears the targets, the
   *        other modes take the current attitude as target and reset the PIDs.
   *
   * @param gimbal_event 目标模式。
   *                     Target mode.
   */
  void SetMode(GimbalEvent gimbal_event)
  {
    if (gimbal_event == current_mode_)
    {
      return;
    };
    // 任何模式切换都终止辨识（由控制线程在下一拍处理）
    // Any mode switch aborts the identification (handled by the control thread on its
    // next tick)
    ident_abort_.store(true, std::memory_order_release);
    // SET_MODE_COMMON 与 SET_MODE_LOW_SENSITIVITY 之间切换时保持目标与 PID 状态
    if ((current_mode_ == GimbalEvent::SET_MODE_COMMON &&
         gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY) ||
        (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY &&
         gimbal_event == GimbalEvent::SET_MODE_COMMON))
    {
      current_mode_ = gimbal_event;
      return;
    }
    current_mode_ = gimbal_event;

    switch (gimbal_event)
    {
      case GimbalEvent::SET_MODE_RELAX:
        motor_yaw_->Disable();
        motor_pit_->Disable();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_pit_cmd_ = 0.0f;
        target_yaw_cmd_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
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
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        patrol_start_time = LibXR::Timebase::GetMilliseconds();
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        last_pit_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_LOW_SENSITIVITY:
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        last_pit_omega_ = 0.0f;
        last_yaw_omega_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      default:
        break;
    }
  }
};
