#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "single_lqr.hpp"
constexpr float YAW_FC = .06300068600582383f;
constexpr float INERTIA = .005505764873850784f;
constexpr float DAMPING = .03659462621840192f;
constexpr float TORQUE_LIMIT = 1.0f;
constexpr SingleLqr::Param GAINS{1.95538256f, .12260207f};

void Sim(float inertia, float damping, float friction, float stiffness,
         bool noise) {
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
    const float ANGLE_ERROR = target - q;
    const float FEEDBACK = SingleLqr::Calculate(GAINS, ANGLE_ERROR, -gyro);
    const float FRICTION_FEEDFORWARD =
        YAW_FC * std::tanh(3.0f * ANGLE_ERROR / .03f);
    u = std::clamp(FEEDBACK + FRICTION_FEEDFORWARD, -TORQUE_LIMIT,
                   TORQUE_LIMIT);
    assert(std::isfinite(u));
    assert(std::fabs(u) <= TORQUE_LIMIT);
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
  constexpr SingleLqr::Param TEST_GAINS{2, 4};
  static_assert(SingleLqr::Calculate(TEST_GAINS, 0, 0) == 0);
  static_assert(SingleLqr::Calculate(TEST_GAINS, 1, 0) == 2);
  static_assert(SingleLqr::Calculate(TEST_GAINS, 0, 1) == 4);
  static_assert(SingleLqr::Calculate(TEST_GAINS, 1, -.25f) == 1);
  static_assert(SingleLqr::Calculate(TEST_GAINS, -10, 0) == -20);

  Sim(INERTIA, DAMPING, YAW_FC, 0, false);
  Sim(INERTIA * .8f, .034f, .060f, 0, true);
  Sim(INERTIA * 1.2f, .044f, .065f, 0, true);
  Sim(INERTIA, DAMPING, YAW_FC, .75f, true);
  std::cout
      << "PASS: LQR multiplication and caller-side compensation/limits.\n";
}
