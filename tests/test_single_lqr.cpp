#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "single_lqr.hpp"
SingleLqr::Param Config() {
  SingleLqr::Param p;
  p.J = .005505764873850784f;
  p.b = .03659462621840192f;
  p.Fc = .06300068600582383f;
  p.k_theta = 1.95538256f;
  p.k_omega = .12260207f;
  return p;
}
void Sim(float inertia, float damping, float friction, float stiffness,
         bool noise) {
  auto p = Config();
  SingleLqr c(p);
  c.Reset(0, 0);
  float q = 0, w = 0, u = 0, actuator = 0, delay[3] = {}, peak = 0;
  unsigned head = 0;
  for (unsigned n = 0; n < 4000; ++n) {
    const float applied = delay[head];
    delay[head] = u;
    head = (head + 1) % 3;
    for (int k = 0; k < 10; ++k) {
      actuator += .0002f / .0062f * (applied - actuator);
      const float net = actuator - damping * w - stiffness * q;
      float f = std::copysign(friction, w);
      if (std::fabs(w) < .0001f) f = std::clamp(net, -friction, friction);
      float next = w + (net - f) / inertia * .0002f;
      w = w * next < 0 && std::fabs(net) <= friction ? 0 : next;
      q += w * .0002f;
    }
    float target = n < 1000 ? .04f : n < 2000 ? -.04f : n < 3000 ? .02f : 0;
    float gyro = w + (noise ? .03f * std::sin(6.2831853f * 80 * n * .002f) : 0);
    u = c.Calculate(target, q, gyro, .002f);
    assert(c.valid_ && std::isfinite(u));
    assert(std::fabs(u) <= p.torque_limit + .00001f);
    peak = std::max(peak, std::fabs(q));
    if (n % 1000 == 999) {
      std::cout << "J=" << inertia << " b=" << damping << " k=" << stiffness
                << " target=" << target << " q=" << q << " error=" << target - q
                << "\n";
      assert(std::fabs(q - target) < (stiffness ? .025f : .015f));
    }
  }
  assert(peak < .12f);
}
int main() {
  auto p = Config();
  SingleLqr c(p);
  c.Reset(-1.68f, 0);
  assert(c.Calculate(-1.68f, -1.68f, 0, .002f) == 0);
  c.Calculate(-1.679f, -1.68f, 0, .002f);
  assert(c.tau_lqr_ > 0 && c.tau_friction_ > 0);
  c.Calculate(-1.64f, -1.68f, 0, .002f);
  assert(c.reference_position_ == -1.64f);
  assert(c.reference_velocity_ == 0 && c.reference_acceleration_ == 0);
  assert(c.tau_inertia_ == 0 && c.tau_viscous_ == 0);
  for (int n = 0; n < 20; ++n) c.Calculate(-1.64f, -1.68f, 0, .002f);
  assert(c.tau_output_ > 0 && c.tau_friction_ > 0);
  c.enable_ = false;
  assert(c.Calculate(-1.64f, -1.68f, 0, .002f) == 0 && !c.valid_);
  c.enable_ = true;
  c.Reset(-1.68f, 0);
  assert(c.Calculate(-1.72f, -1.68f, 0, .002f) < 0);
  assert(c.Calculate(-1.72f, -1.68f, 0, .03f) == 0 && !c.valid_);
  assert(c.Calculate(std::numeric_limits<float>::quiet_NaN(), 0, 0, .002f) ==
         0);
  // Large steps reach either torque cap in the same control update, including
  // an immediate reversal from +1 to -1 with no previous-output rate limit.
  SingleLqr capped(p);
  capped.Reset(0, 0);
  assert(capped.Calculate(10, 0, 0, .002f) == 1.0f);
  assert(capped.saturated_ && std::fabs(capped.tau_output_ - 1.0f) < .00001f);
  assert(capped.reference_position_ == 10);
  assert(capped.Calculate(-10, 0, 0, .002f) == -1.0f);
  assert(capped.saturated_ && std::fabs(capped.tau_output_ + 1.0f) < .00001f);
  assert(capped.reference_position_ == -10);
  capped.friction_enable_ = false;
  assert(capped.Calculate(.1f, 0, 0, .002f) > 0);
  assert(!capped.saturated_ && capped.tau_output_ == capped.tau_before_limit_);
  SingleLqr direct_gyro(p);
  direct_gyro.friction_enable_ = false;
  direct_gyro.Reset(0, 0);
  direct_gyro.Calculate(0, 0, .5f, .002f);
  assert(direct_gyro.error_velocity_ == -.5f);
  assert(std::fabs(direct_gyro.tau_lqr_ + p.k_omega * .5f) < .000001f);
  direct_gyro.Calculate(0, 0, -.5f, .002f);
  assert(direct_gyro.error_velocity_ == .5f);
  assert(std::fabs(direct_gyro.tau_lqr_ - p.k_omega * .5f) < .000001f);
  assert(capped.Calculate(-.1f, 0, 0, .002f) < 0);
  assert(!capped.saturated_ && capped.tau_output_ == capped.tau_before_limit_);
  Sim(p.J, p.b, p.Fc, 0, false);
  Sim(p.J * .8f, .034f, .060f, 0, true);
  Sim(p.J * 1.2f, .044f, .065f, 0, true);
  Sim(p.J, p.b, p.Fc, .75f, true);
  std::cout << "PASS: sign, direct setpoints, stop/fault reset, limits and "
               "delayed nonlinear plant simulations.\n";
}
