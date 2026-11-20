#pragma once

#include <algorithm>
#include <cmath>

// Fixed-base single joint: J*q_ddot + b*q_dot = torque - friction.
// K is designed OFFLINE and supplied through YAML. No online Riccati solver.
class SingleLqr {
 public:
  struct Param {
    float J = 0;
    float b = 0;
    float Fc = 0;
    float k_theta = 0;
    float k_omega = 0;
    float coulomb_smooth = .03f;
    float friction_recovery_gain = 3.0f;
    float torque_limit = 1.0f;
  };
  explicit SingleLqr(const Param& p)
      : param_(p), k_theta_(p.k_theta), k_omega_(p.k_omega), Fc_(p.Fc) {}

  // Runtime tuning does not silently redesign K when J/b are edited.
  Param param_;
  volatile float k_theta_, k_omega_, Fc_;
  volatile bool enable_ = true;
  volatile bool friction_enable_ = true;
  float reference_position_ = 0, reference_velocity_ = 0;
  float reference_acceleration_ = 0;
  float error_position_ = 0, error_velocity_ = 0;
  float tau_lqr_ = 0, tau_inertia_ = 0, tau_viscous_ = 0;
  float tau_friction_ = 0, tau_before_limit_ = 0, tau_output_ = 0;
  bool valid_ = false, saturated_ = false;

  void Reset(float position, float omega) {
    (void)omega;  // Retain the reset interface; no gyro filter state is stored.
    reference_position_ = position;
    reference_velocity_ = reference_acceleration_ = 0;
    error_position_ = error_velocity_ = 0;
    tau_lqr_ = tau_inertia_ = tau_viscous_ = tau_friction_ = 0;
    tau_before_limit_ = tau_output_ = 0;
    saturated_ = false;
    initialized_ = true;
  }

  float Calculate(float target, float position, float omega, float dt) {
    if (!enable_) {
      initialized_ = false;
      valid_ = false;
      tau_output_ = 0;
      return 0;
    }
    const auto& P = param_;
    const float KP = k_theta_, KD = k_omega_, FC = Fc_;
    const float VALUES[] = {target,
                            position,
                            omega,
                            dt,
                            P.J,
                            P.b,
                            FC,
                            KP,
                            KD,
                            P.coulomb_smooth,
                            P.friction_recovery_gain,
                            P.torque_limit};
    valid_ = true;
    for (const float VALUE : VALUES) valid_ = valid_ && std::isfinite(VALUE);
    valid_ = valid_ && P.J > 0 && P.b >= 0 && FC >= 0 && KP > 0 && KD >= 0 &&
             dt > 0 && dt <= .02f && P.coulomb_smooth > 0 &&
             P.friction_recovery_gain >= 0 && P.torque_limit > 0;
    if (!valid_) {
      initialized_ = false;
      tau_output_ = 0;
      return 0;
    }
    if (!initialized_) Reset(position, omega);
    // Direct position setpoint; no velocity/acceleration-limited trajectory.
    // Do not differentiate setpoint steps into velocity or torque impulses.
    reference_position_ = target;
    reference_velocity_ = 0;
    reference_acceleration_ = 0;
    error_position_ = reference_position_ - position;
    error_velocity_ = reference_velocity_ - omega;
    tau_lqr_ = KP * error_position_ + KD * error_velocity_;
    tau_inertia_ = P.J * reference_acceleration_;
    tau_viscous_ = P.b * reference_velocity_;
    // Smooth position-recovery cue to help overcome Coulomb friction.
    float intent =
        reference_velocity_ + P.friction_recovery_gain * error_position_;
    tau_friction_ =
        friction_enable_ ? FC * std::tanh(intent / P.coulomb_smooth) : 0;
    tau_before_limit_ = tau_lqr_ + tau_inertia_ + tau_viscous_ + tau_friction_;
    const float LIMITED =
        std::clamp(tau_before_limit_, -P.torque_limit, P.torque_limit);
    saturated_ = LIMITED != tau_before_limit_;
    tau_output_ = LIMITED;
    return tau_output_;
  }

 private:
  bool initialized_ = false;
};
