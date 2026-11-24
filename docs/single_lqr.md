# SingleLqr: standalone single-axis torque controller

`single_lqr.hpp` is a C++17 header-only function for calculating single-axis
LQR feedback torque.

## Scope and units

The inputs are angle error (rad) and angular rate error (rad/s), each computed as
reference minus measurement in the same coordinate system. The output is
feedback torque in N m. The caller owns enable state, feedback validation,
sample timing, feedforward, limits, and motor transport.

For a nested gimbal, account for base motion when combining angle and rate
measurements from different frames.

## Model and offline gain design

The linear design model is `J*q_ddot + b*q_dot = torque` with states
`[position, angular_velocity]`. Run `tools/design_single_lqr.m` in MATLAB with
Control System Toolbox. Edit J, b, Ts, Q and R. The script uses zero-order-hold
sampling and `dlqr`, checks nominal pole magnitudes, then prints k_theta/k_omega.

The supplied J/b values are experimental candidates from one mechanism, not
universal motor constants. They must be replaced or validated for a different
mechanism. Recompute K whenever the model or sample period changes. Q/R are not
solved online and are not PID gains.

## API

```cpp
#include "single_lqr.hpp"

const SingleLqr::Param GAINS{1.95538256f, 0.12260207f};
const float ANGLE_ERROR = target_position - current_position;
const float RATE_ERROR = target_omega - current_omega;
const float FEEDBACK = SingleLqr::Calculate(GAINS, ANGLE_ERROR, RATE_ERROR);
```

For a fixed position target, `target_omega` is zero. Add feedforward to the
returned feedback torque before applying the caller's final torque limit.

## Standalone checks

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I. tests/test_single_lqr.cpp -o test_single_lqr
./test_single_lqr
```

Checks cover gain ordering, signed torque, and caller-side compensation and
limits in selected delayed and noisy nonlinear model simulations.
