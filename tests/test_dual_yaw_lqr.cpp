#include "DualYawLqr.hpp"

int main() {
  constexpr DualYawLqr::Param GAINS = {1, 2, 3, 4, 5, 6, 7, 8};
  constexpr auto BIG_ANGLE = DualYawLqr::Calculate(GAINS, 1, 0, 0, 0);
  constexpr auto SMALL_ANGLE = DualYawLqr::Calculate(GAINS, 0, 1, 0, 0);
  constexpr auto BIG_RATE = DualYawLqr::Calculate(GAINS, 0, 0, 1, 0);
  constexpr auto SMALL_RATE = DualYawLqr::Calculate(GAINS, 0, 0, 0, 1);
  static_assert(BIG_ANGLE.big == 1 && BIG_ANGLE.small == 5);
  static_assert(SMALL_ANGLE.big == 2 && SMALL_ANGLE.small == 6);
  static_assert(BIG_RATE.big == 3 && BIG_RATE.small == 7);
  static_assert(SMALL_RATE.big == 4 && SMALL_RATE.small == 8);

  constexpr auto TORQUE = DualYawLqr::Calculate(GAINS, .5f, -1, 2, -.25f);
  static_assert(TORQUE.big == 3.5f && TORQUE.small == 8.5f);

  constexpr auto NO_ERROR = DualYawLqr::Calculate(GAINS, 0, 0, 0, 0);
  static_assert(NO_ERROR.big == 0 && NO_ERROR.small == 0);
}
