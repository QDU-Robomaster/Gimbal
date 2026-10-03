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

#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
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
  SET_MODE_RELAX,  ///< 放松：失能电机，目标清零
                   ///< Relax: motors disabled, targets cleared
  SET_MODE_COMMON,  ///< 常规控制
                    ///< Normal control
  SET_MODE_AUTOPATROL,  ///< 自动巡逻
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
    ///< Pitch center-of-mass distance (m, positive upward from the horizontal) times its weight (N)
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
   */
  Gimbal(
      CMD& cmd,
      Motor& motor_pit,
      Motor& motor_yaw,
      Referee* referee,
      const Param& param = {.task_stack_depth = 2048, .pid_yaw_angle = {.k = 0.0f, .p = 0.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = true}, .pid_yaw_omega = {.k = 0.0f, .p = 0.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = true}, .pid_pit_angle = {.k = 0.0f, .p = 0.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}, .pid_pit_omega = {.k = 0.0f, .p = 0.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}, .pit_max_angle = 0.0f, .pit_min_angle = 0.0f, .pit_lc = 0.0f, .pit_theta = 0.0f, .yaw_k = 0.0f, .j_pit = 0.0f, .j_yaw = 0.0f, .pit_zero = 0.0f, .yaw_zero = 0.0f, .patrol_range = 0.0f, .patrol_omega = 0.0f, .reverse_flag = true, .thread_priority = LibXR::Thread::Priority::MEDIUM, .euler_topic_name = "ahrs_euler", .gyro_topic_name = "bmi088_gyro", .gimbal_cmd_topic_name = "gimbal_cmd"})
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
        referee_(referee)
  {
    UNUSED(referee_);

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
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
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
    this->torque_ = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    float out_pit = 0.0f;
    float out_yaw = 0.0f;

    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
               pit_max_angle_, pit_min_angle_, reverse_flag_);
    Solve(out_pit, out_yaw, target_pit_cmd_, target_yaw_cmd_, dt_);
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
    float pit_error = target_pit_angle - euler_.Pitch();
    float target_pit_omega =
        pid_pit_angle_.Calculate(pit_error, 0.0f, dt_) + target_pit_dot_;
    float ff_pit = JFeedforward(target_pit_omega, last_pit_omega_, dt_, j_pit_) +
                   j_pit_ * target_pit_ddot_;
    float gravity_ff_pit = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    float fb_pit = pid_pit_omega_.Calculate(target_pit_omega, gyro_data_.y(), dt_);
    pit_output = ff_pit + fb_pit + gravity_ff_pit;
    last_pit_omega_ = target_pit_omega;
    float yaw_error = target_yaw_angle - euler_.Yaw();
    float target_yaw_omega =
        pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_) + target_yaw_dot_;
    float ff_yaw = JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_) +
                   j_yaw_ * target_yaw_ddot_;
    float fb_yaw = pid_yaw_omega_.Calculate(target_yaw_omega, gyro_data_.z(), dt_);
    yaw_output = ff_yaw + fb_yaw + motor_yaw_feedback_.omega * this->yaw_k_;
    last_yaw_omega_ = target_yaw_omega;
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
