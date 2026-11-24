#pragma once

class DualYawLqr {
 public:
  struct Param {
    float k11 = 0;
    float k12 = 0;
    float k13 = 0;
    float k14 = 0;
    float k21 = 0;
    float k22 = 0;
    float k23 = 0;
    float k24 = 0;
  };

  struct Torque {
    float big;
    float small;
  };

  // Angles in rad, rates in rad/s. Returns joint feedback torques in N m.
  static constexpr Torque Calculate(const Param& K, float big_angle_error,
                                    float small_angle_error,
                                    float big_rate_error,
                                    float small_rate_error) {
    return {
        K.k11 * big_angle_error + K.k12 * small_angle_error +
            K.k13 * big_rate_error + K.k14 * small_rate_error,
        K.k21 * big_angle_error + K.k22 * small_angle_error +
            K.k23 * big_rate_error + K.k24 * small_rate_error,
    };
  }
};
