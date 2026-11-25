#include <cassert>
#include <cmath>
#include <limits>

#include "DoubleYaw.hpp"

int main() {
  DoubleYaw::Param param;
  param.k11 = 1;
  param.k22 = 1;
  param.big_direction = -1;
  param.big_position_scale = 2;
  param.imu_direction = -1;
  param.big_reference_ramp = false;
  param.big_torque_limit = 1;
  param.small_torque_limit = 0.5f;
  DoubleYaw yaw(param);

  DoubleYaw::Input input{};
  input.aim_error = -0.2f;
  input.min_position = -1;
  input.max_position = 1;
  input.dt = 0.002f;

  yaw.UpdateBigFeedback(0.2f, 1000);
  assert(!yaw.Calculate(input));
  yaw.UpdateBigFeedback(0.2f, 3000);
  assert(yaw.Calculate(input));
  assert(std::fabs(yaw.big_position_ - (-0.4f)) < 1e-6f);
  assert(std::fabs(yaw.big_torque_ - 0.2f) < 1e-6f);
  assert(std::fabs(yaw.small_torque_ - 0.2f) < 1e-6f);

  input.target_rate = -0.3f;
  input.target_accel = -0.4f;
  assert(yaw.Calculate(input, [](const DoubleYaw::SmallYawReference& ref) {
    assert(std::fabs(ref.angle_error - 0.2f) < 1e-6f);
    assert(std::fabs(ref.rate - 0.3f) < 1e-6f);
    assert(std::fabs(ref.aim_accel - 0.4f) < 1e-6f);
    return 0.4f;
  }));
  assert(yaw.small_torque_ == param.small_torque_limit);

  input.small_position = input.max_position;
  assert(yaw.Calculate(input, [](const DoubleYaw::SmallYawReference& ref) {
    assert(ref.rate == 0 && ref.aim_accel == 0);
    return 0.4f;
  }));
  assert(yaw.small_torque_ == 0);

  assert(!yaw.Calculate(input, [](const DoubleYaw::SmallYawReference&) {
    return std::numeric_limits<float>::quiet_NaN();
  }));
  assert(yaw.big_torque_ == 0 && yaw.small_torque_ == 0);

  yaw.UpdateBigFeedback(0.2f, 55000);
  assert(!yaw.Calculate(input));
}
