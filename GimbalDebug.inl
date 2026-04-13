#pragma once

#ifndef GIMBAL_DEBUG_IMPL
#include "Gimbal.hpp"
#endif

inline int Gimbal::DebugCommand(int argc, char** argv) {
  enum class DebugView : uint8_t { STATE, CMD, PID, MOTOR, FULL };

  constexpr uint8_t view_state = static_cast<uint8_t>(DebugView::STATE);
  constexpr uint8_t view_cmd = static_cast<uint8_t>(DebugView::CMD);
  constexpr uint8_t view_pid = static_cast<uint8_t>(DebugView::PID);
  constexpr uint8_t view_motor = static_cast<uint8_t>(DebugView::MOTOR);
  constexpr uint8_t view_full = static_cast<uint8_t>(DebugView::FULL);

  constexpr debug_core::ViewMask mask_state = debug_core::view_bit(view_state);
  constexpr debug_core::ViewMask mask_cmd = debug_core::view_bit(view_cmd);
  constexpr debug_core::ViewMask mask_pid = debug_core::view_bit(view_pid);
  constexpr debug_core::ViewMask mask_motor = debug_core::view_bit(view_motor);

  static constexpr std::array<debug_core::ViewEntry<uint8_t>, 5> view_table{{
      {"state", view_state},
      {"cmd", view_cmd},
      {"pid", view_pid},
      {"motor", view_motor},
      {"full", view_full},
  }};

#define GIMBAL_PID_FIELD(name, member, mask)                              \
  DEBUG_CORE_LIVE_CUSTOM(                                                 \
      Gimbal, (name), (mask),                                             \
      +[](const char* field_name, const Gimbal* self) {                   \
        const auto& pid = (self->member);                                 \
        LibXR::STDIO::Printf(                                             \
            "  %s: k=%.4f p=%.4f i=%.4f d=%.4f i_lim=%.4f out_lim=%.4f "  \
            "err=%.4f i_err=%.4f der=%.4f out=%.4f\r\n",                  \
            field_name, pid.K(), pid.P(), pid.I(), pid.D(), pid.ILimit(), \
            pid.OutLimit(), pid.LastError(), pid.GetIntegralError(),      \
            pid.LastDerivative(), pid.LastOutput());                      \
      })

#define GIMBAL_MOTOR_FIELD(name, member, mask)                           \
  DEBUG_CORE_LIVE_CUSTOM(                                                \
      Gimbal, (name), (mask),                                            \
      +[](const char* field_name, const Gimbal* self) {                  \
        const auto& fb = (self->member);                                 \
        LibXR::STDIO::Printf(                                            \
            "  %s: state=%u abs=%.4f rad vel=%.0f rpm omega=%.4f rad/s " \
            "torque=%.4f temp=%.0f C\r\n",                               \
            field_name, static_cast<unsigned>(fb.state),                 \
            static_cast<float>(fb.abs_angle), fb.velocity, fb.omega,     \
            fb.torque, fb.temp);                                         \
      })

  static const debug_core::LiveFieldDesc<Gimbal> fields[] = {
      DEBUG_CORE_LIVE_CUSTOM(
          Gimbal, "mode", mask_state,
          +[](const char* field_name, const Gimbal* self) {
            const char* text = "UNKNOWN";
            switch (self->current_mode_) {
              case GimbalEvent::SET_MODE_RELAX:
                text = "RELAX";
                break;
              case GimbalEvent::SET_MODE_COMMON:
                text = "COMMON";
                break;
                            case GimbalEvent::SET_MODE_AUTOPATROL:
                text = "AUTOPATROL";
                break;

            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_F32(Gimbal, "dt_s", mask_state, self->dt_),
      DEBUG_CORE_LIVE_F32(Gimbal, "yaw_rad", mask_state, self->euler_.Yaw()),
      DEBUG_CORE_LIVE_F32(Gimbal, "pit_rad", mask_state, self->euler_.Pitch()),
      DEBUG_CORE_LIVE_F32(Gimbal, "abs_yaw_rad", mask_state,
                          self->abs_angle_yaw_),
      DEBUG_CORE_LIVE_F32(Gimbal, "abs_pit_rad", mask_state,
                          self->abs_angle_pit_),
      DEBUG_CORE_LIVE_F32(Gimbal, "target_yaw_rad", mask_state,
                          static_cast<float>(self->target_yaw_cmd_)),
      DEBUG_CORE_LIVE_F32(Gimbal, "target_pit_rad", mask_state,
                          static_cast<float>(self->target_pit_cmd_)),
      DEBUG_CORE_LIVE_CUSTOM(
          Gimbal, "ctrl_mode", mask_cmd,
          +[](const char* field_name, const Gimbal* self) {
            const char* text = "UNKNOWN";
            switch (self->cmd_.GetCtrlMode()) {
              case CMD::Mode::CMD_OP_CTRL:
                text = "CMD_OP_CTRL";
                break;
              case CMD::Mode::CMD_AUTO_CTRL:
                text = "CMD_AUTO_CTRL";
                break;
            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_BOOL(Gimbal, "ai_gimbal", mask_cmd,
                           self->cmd_.GetAIGimbalStatus()),
      DEBUG_CORE_LIVE_F32(Gimbal, "cmd_yaw", mask_cmd, self->cmd_data_.yaw),
      DEBUG_CORE_LIVE_F32(Gimbal, "cmd_pit", mask_cmd, self->cmd_data_.pit),
      DEBUG_CORE_LIVE_F32(Gimbal, "gyro_y_radps", mask_cmd,
                          self->gyro_data_.y()),
      DEBUG_CORE_LIVE_F32(Gimbal, "gyro_z_radps", mask_cmd,
                          self->gyro_data_.z()),
      GIMBAL_PID_FIELD("yaw_angle_pid", pid_yaw_angle_, mask_pid),
      GIMBAL_PID_FIELD("yaw_omega_pid", pid_yaw_omega_, mask_pid),
      GIMBAL_PID_FIELD("pit_angle_pid", pid_pit_angle_, mask_pid),
      GIMBAL_PID_FIELD("pit_omega_pid", pid_pit_omega_, mask_pid),
      GIMBAL_MOTOR_FIELD("motor_yaw", motor_yaw_feedback_, mask_motor),
      GIMBAL_MOTOR_FIELD("motor_pit", motor_pit_feedback_, mask_motor),
  };

#undef GIMBAL_PID_FIELD
#undef GIMBAL_MOTOR_FIELD

  return debug_core::run_live_command(
      this, "gimbal", "state|cmd|pid|motor|full", view_table, fields,
      sizeof(fields) / sizeof(fields[0]), argc, argv, view_full);
}
