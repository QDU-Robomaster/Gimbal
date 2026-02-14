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
#include <cstdlib>
#include <cstring>
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "ramfs.hpp"
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
        reverse_flag_(reverse_flag ? 1.0f : -1.0f),
        cmd_file_(LibXR::RamFS::CreateFile("gimbal", CommandFunc, this)) {
    UNUSED(app);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

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

  /**
   * @brief 线程函数
   *
   * @param gimbal Gimbal实例指针
   */
  static void ThreadFunc(Gimbal *gimbal) {
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
      auto last_time = LibXR::Timebase::GetMilliseconds();
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
      LibXR::Thread::SleepUntil(last_time, 2);
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
    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle, pit_max_angle_,
               pit_min_angle_, reverse_flag_);
    Solve(out_pit, out_yaw, target_pit_cmd_, target_yaw_cmd_, dt_);
    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_yaw});
    auto pit_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = out_pit});

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

  LibXR::Event &GetEvent() { return gimbal_event_; }

  /**
   * @brief 调试命令入口
   * @param argc 命令参数个数
   * @param argv 命令参数数组
   * @return int 命令执行结果，0表示成功，负值表示失败
   * @details 支持 state/cmd/pid/motor/full 视图以及 once/monitor 调试模式。
   */
  int DebugCommand(int argc, char **argv) {
    enum class DebugView : uint8_t {
      STATE,
      CMD,
      PID,
      MOTOR,
      FULL,
    };

    struct DebugSnapshot {
      GimbalEvent mode;
      float dt;

      float target_yaw;
      float target_pit;

      float yaw;
      float pit;
      float abs_yaw;
      float abs_pit;

      float cmd_yaw;
      float cmd_pit;

      float gyro_y;
      float gyro_z;

      Motor::Feedback motor_yaw;
      Motor::Feedback motor_pit;

      struct {
        float k, p, i, d, i_limit, out_limit;
        float err, der, out, i_err;
      } yaw_angle_pid, yaw_omega_pid, pit_angle_pid, pit_omega_pid;

      CMD::Mode ctrl_mode;
      bool ai_gimbal;
    };

    auto view_to_string = [](DebugView view) {
      switch (view) {
        case DebugView::STATE:
          return "STATE";
        case DebugView::CMD:
          return "CMD";
        case DebugView::PID:
          return "PID";
        case DebugView::MOTOR:
          return "MOTOR";
        case DebugView::FULL:
          return "FULL";
        default:
          return "UNKNOWN";
      }
    };

    auto mode_to_string = [](GimbalEvent mode) {
      switch (mode) {
        case GimbalEvent::SET_MODE_RELAX:
          return "RELAX";
        case GimbalEvent::SET_MODE_COMMON:
          return "COMMON";
        default:
          return "UNKNOWN";
      }
    };

    auto parse_view = [](const char *s, DebugView *out) -> bool {
      if (s == nullptr || out == nullptr) {
        return false;
      }
      if (strcmp(s, "state") == 0) {
        *out = DebugView::STATE;
        return true;
      }
      if (strcmp(s, "cmd") == 0) {
        *out = DebugView::CMD;
        return true;
      }
      if (strcmp(s, "pid") == 0) {
        *out = DebugView::PID;
        return true;
      }
      if (strcmp(s, "motor") == 0) {
        *out = DebugView::MOTOR;
        return true;
      }
      if (strcmp(s, "full") == 0) {
        *out = DebugView::FULL;
        return true;
      }
      return false;
    };

    auto capture_snapshot = [&]() -> DebugSnapshot {
      DebugSnapshot s{};

      s.mode = current_mode_;
      s.dt = dt_;

      s.target_yaw = static_cast<float>(target_yaw_cmd_);
      s.target_pit = static_cast<float>(target_pit_cmd_);

      s.yaw = euler_.Yaw();
      s.pit = euler_.Pitch();
      s.abs_yaw = abs_angle_yaw_;
      s.abs_pit = abs_angle_pit_;

      s.cmd_yaw = cmd_data_.yaw;
      s.cmd_pit = cmd_data_.pit;

      s.gyro_y = gyro_data_.y();
      s.gyro_z = gyro_data_.z();

      s.motor_yaw = motor_yaw_feedback_;
      s.motor_pit = motor_pit_feedback_;

      auto fill_pid = [](auto &dst, const LibXR::PID<float> &pid) {
        dst.k = pid.K();
        dst.p = pid.P();
        dst.i = pid.I();
        dst.d = pid.D();
        dst.i_limit = pid.ILimit();
        dst.out_limit = pid.OutLimit();
        dst.err = pid.LastError();
        dst.der = pid.LastDerivative();
        dst.out = pid.LastOutput();
        dst.i_err = pid.GetIntegralError();
      };

      fill_pid(s.yaw_angle_pid, pid_yaw_angle_);
      fill_pid(s.yaw_omega_pid, pid_yaw_omega_);
      fill_pid(s.pit_angle_pid, pid_pit_angle_);
      fill_pid(s.pit_omega_pid, pid_pit_omega_);

      s.ctrl_mode = cmd_.GetCtrlMode();
      s.ai_gimbal = cmd_.GetAIGimbalStatus();

      return s;
    };

    auto print_state = [&](const DebugSnapshot &s) {
      LibXR::STDIO::Printf("  mode: %s\r\n", mode_to_string(s.mode));
      LibXR::STDIO::Printf("  dt: %.6f s\r\n", s.dt);
      LibXR::STDIO::Printf("  euler: yaw=%.4f rad, pit=%.4f rad\r\n", s.yaw, s.pit);
      LibXR::STDIO::Printf("  abs_angle: yaw=%.4f rad, pit=%.4f rad\r\n", s.abs_yaw,
                           s.abs_pit);
      LibXR::STDIO::Printf("  target: yaw=%.4f rad, pit=%.4f rad\r\n", s.target_yaw,
                           s.target_pit);
    };

    auto print_cmd = [&](const DebugSnapshot &s) {
      LibXR::STDIO::Printf("  ctrl_mode: %s\r\n",
                           s.ctrl_mode == CMD::Mode::CMD_OP_CTRL ? "OP" : "AI");
      LibXR::STDIO::Printf("  ai_gimbal: %s\r\n", s.ai_gimbal ? "true" : "false");
      LibXR::STDIO::Printf("  cmd: yaw=%.4f, pit=%.4f\r\n", s.cmd_yaw, s.cmd_pit);
      LibXR::STDIO::Printf("  gyro: y=%.4f rad/s, z=%.4f rad/s\r\n", s.gyro_y, s.gyro_z);
    };

    auto print_pid = [&](const DebugSnapshot &s) {
      auto print_one = [](const char *name, const decltype(s.yaw_angle_pid) &p) {
        LibXR::STDIO::Printf("  %s:\r\n", name);
        LibXR::STDIO::Printf(
            "    param: k=%.4f p=%.4f i=%.4f d=%.4f i_lim=%.4f out_lim=%.4f\r\n",
            p.k, p.p, p.i, p.d, p.i_limit, p.out_limit);
        LibXR::STDIO::Printf("    state: err=%.4f i_err=%.4f der=%.4f out=%.4f\r\n",
                             p.err, p.i_err, p.der, p.out);
      };
      print_one("yaw_angle", s.yaw_angle_pid);
      print_one("yaw_omega", s.yaw_omega_pid);
      print_one("pit_angle", s.pit_angle_pid);
      print_one("pit_omega", s.pit_omega_pid);
    };

    auto print_motor = [&](const DebugSnapshot &s) {
      auto print_fb = [](const char *name, const Motor::Feedback &fb) {
        LibXR::STDIO::Printf("  %s:\r\n", name);
        LibXR::STDIO::Printf(
            "    state=%d abs=%.4f rad vel=%.0f rpm omega=%.4f rad/s torque=%.4f "
            "temp=%.0f C\r\n",
            fb.state, static_cast<float>(fb.abs_angle), fb.velocity, fb.omega, fb.torque,
            fb.temp);
      };
      print_fb("motor_yaw", s.motor_yaw);
      print_fb("motor_pit", s.motor_pit);
    };

    auto print_once = [&](DebugView view) {
      const DebugSnapshot s = capture_snapshot();
      LibXR::STDIO::Printf("[%lu ms] gimbal %s\r\n", LibXR::Thread::GetTime(),
                           view_to_string(view));
      switch (view) {
        case DebugView::STATE:
          print_state(s);
          break;
        case DebugView::CMD:
          print_cmd(s);
          break;
        case DebugView::PID:
          print_pid(s);
          break;
        case DebugView::MOTOR:
          print_motor(s);
          break;
        case DebugView::FULL:
          print_state(s);
          print_cmd(s);
          print_pid(s);
          print_motor(s);
          break;
      }
    };

    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  monitor\r\n");
      LibXR::STDIO::Printf(
          "  monitor <time_ms> [interval_ms] [state|cmd|pid|motor|full]\r\n");
      LibXR::STDIO::Printf("  once [state|cmd|pid|motor|full]\r\n");
      LibXR::STDIO::Printf("  state | cmd | pid | motor | full\r\n");
      return 0;
    }

    if (strcmp(argv[1], "monitor") == 0) {
      if (argc == 2) {
        print_once(DebugView::FULL);
        return 0;
      }

      if (argc > 5) {
        LibXR::STDIO::Printf("Error: Too many arguments for monitor.\r\n");
        return -1;
      }

      int time_ms = atoi(argv[2]);
      int interval_ms = 1000;
      DebugView view = DebugView::FULL;
      bool third_is_view = false;
      if (argc >= 4) {
        DebugView parsed_view = DebugView::FULL;
        if (parse_view(argv[3], &parsed_view)) {
          view = parsed_view;
          third_is_view = true;
        } else {
          interval_ms = atoi(argv[3]);
        }
      }
      if (argc == 5) {
        if (third_is_view) {
          LibXR::STDIO::Printf(
              "Error: Invalid monitor args. Use monitor <time_ms> [interval_ms] [view].\r\n");
          return -1;
        }
        if (!parse_view(argv[4], &view)) {
          LibXR::STDIO::Printf("Error: Unknown view '%s'.\r\n", argv[4]);
          return -1;
        }
      }

      if (time_ms <= 0 || interval_ms <= 0) {
        LibXR::STDIO::Printf("Error: time_ms and interval_ms must be > 0.\r\n");
        return -1;
      }

      int elapsed = 0;
      while (elapsed < time_ms) {
        print_once(view);
        LibXR::Thread::Sleep(interval_ms);
        elapsed += interval_ms;
      }
      return 0;
    }

    if (strcmp(argv[1], "once") == 0) {
      if (argc > 3) {
        LibXR::STDIO::Printf("Error: Too many arguments for once.\r\n");
        return -1;
      }
      DebugView view = DebugView::FULL;
      if (argc == 3 && !parse_view(argv[2], &view)) {
        LibXR::STDIO::Printf("Error: Unknown view '%s'.\r\n", argv[2]);
        return -1;
      }
      print_once(view);
      return 0;
    }

    DebugView direct_view = DebugView::FULL;
    if (argc == 2 && parse_view(argv[1], &direct_view)) {
      print_once(direct_view);
      return 0;
    }

    LibXR::STDIO::Printf("Error: Unknown command '%s'.\r\n", argv[1]);
    return -1;
  }

 private:
  /**
   * @brief RamFS 可执行命令入口
   * @param self Gimbal 实例指针
   * @param argc 参数个数
   * @param argv 参数数组
   * @return int 命令执行结果
   */
  static int CommandFunc(Gimbal *self, int argc, char **argv) {
    return self->DebugCommand(argc, argv);
  }

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

  LibXR::RamFS::File cmd_file_;

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

  /**
   * @brief 解算PID控制输出
   *
   * @param pit_output Pitch轴输出引用
   * @param yaw_output Yaw轴输出引用
   * @param target_pit_angle 目标Pitch角度
   * @param target_yaw_angle 目标Yaw角度
   * @param dt_ 时间间隔
   */
  void Solve(float &pit_output, float &yaw_output,
             const LibXR::CycleValue<float> &target_pit_angle,
             const LibXR::CycleValue<float> &target_yaw_angle, float dt_) {
    float pit_error = target_pit_angle - euler_.Pitch();
    float target_pit_omega = pid_pit_angle_.Calculate(pit_error, 0.0f, dt_);
    float ff_pit = JFeedforward(target_pit_omega, last_pit_omega_, dt_, j_pit_);
    float fb_pit =
        pid_pit_omega_.Calculate(target_pit_omega, gyro_data_.y(), dt_);
    pit_output = ff_pit + fb_pit;
    last_pit_omega_ = target_pit_omega;
    float yaw_error = target_yaw_angle - euler_.Yaw();
    float target_yaw_omega = pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_);
    float ff_yaw = JFeedforward(target_yaw_omega, last_yaw_omega_, dt_, j_yaw_);
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
   * @param J 转动惯量
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
      default:
        break;
    }
  }
};
