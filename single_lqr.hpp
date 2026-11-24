#pragma once

class SingleLqr {
 public:
  struct Param {
    float k_theta = 0;
    float k_omega = 0;
  };

  // Reference minus measurement. Angle in rad, rate in rad/s, torque in N m.
  static constexpr float Calculate(const Param& K, float angle_error,
                                   float rate_error) {
    return K.k_theta * angle_error + K.k_omega * rate_error;
  }
};
