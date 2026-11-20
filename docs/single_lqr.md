# SingleLqr: standalone single-axis torque controller

`single_lqr.hpp` is a C++17 header-only controller with no LibXR, CAN, motor or
remote-control dependencies. This first step does not change `Gimbal.hpp` or add
PID/LQR selection to the existing module.

## Scope and units

The present interface controls a fixed position target using measured position
(rad) and measured angular velocity (rad/s), both in the same coordinate system
and direction. Output is requested torque in N m. The caller owns the sample
schedule, sensor validity, mechanical limits and motor transport.

This is the fixed-base single-joint version. For a nested gimbal, encoder relative
position and IMU absolute angular velocity cannot be combined without accounting
for base motion. Desired velocity/acceleration tracking and auto-aim coordinate
conversion are future integration work.

## Model and offline gain design

The linear design model is `J*q_ddot + b*q_dot = torque` with states
`[position, angular_velocity]`. Run `tools/design_single_lqr.m` in MATLAB with
Control System Toolbox. Edit J, b, Ts, Q and R; the script uses zero-order-hold
sampling and `dlqr`, checks nominal pole magnitudes, then prints k_theta/k_omega.

The supplied J/b values are experimental candidates from one mechanism, not
universal motor constants. They must be replaced or validated for a different
mechanism. Recompute K whenever the model or sample period changes. Q/R are not
solved online and are not PID gains.

## API

```cpp
#include "single_lqr.hpp"

SingleLqr::Param p;
p.J = 0.005505764873850784f; // example identified candidate
p.b = 0.03659462621840192f;  // example identified candidate
p.k_theta = 1.95538256f;    // example: Q=diag([4,0.0025]), R=1, Ts=0.002
p.k_omega = 0.12260207f;
p.Fc = 0.0f;               // enable compensation only after identifying it
p.torque_limit = 1.0f;
SingleLqr controller(p);
controller.Reset(current_position, current_omega);
const float TORQUE = controller.Calculate(target, current_position,
                                         current_omega, dt_seconds);
```

`Calculate` applies the target directly, with no velocity/acceleration trajectory
limits, no software angular-velocity filter, no position deadband and no torque
slew-rate limit. Angular-rate error is `0 - measured_omega`. The caller should
pass a signed, continuous position and target close to each other; the controller
does not unwrap encoder rollover.

The feedback law is `k_theta*(target-position) - k_omega*omega`. Optional friction
compensation is `Fc*tanh(friction_recovery_gain*(target-position)/coulomb_smooth)`.
Fc is in N m, recovery gain in 1/s, and the smoothing scale in rad/s. This is a
position-recovery friction heuristic, not an identified static-friction model.
Set `friction_enable_=false` or `Fc=0` to disable it. Output is clamped to
`[-torque_limit,+torque_limit]` in the same control update.

Set `enable_=false` to request zero output. Invalid/non-finite inputs or invalid
parameters request zero and set `valid_=false`; valid dt must be positive and at
most 0.02 s. Default J and K are zero, so valid parameters must be configured.
`Reset` clears controller state. Current direct-position mode has zero reference
velocity/acceleration and therefore zero inertia/viscous reference feedforward.

Diagnostic fields include `error_position_`, `error_velocity_`, `tau_lqr_`,
`tau_friction_`, `tau_before_limit_`, `tau_output_`, `valid_` and `saturated_`.
Changing `k_theta_`, `k_omega_` or `Fc_` does not recompute the LQR design.

## Standalone checks

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I. tests/test_single_lqr.cpp -o test_single_lqr
./test_single_lqr
```

Checks cover direct position targets, direct angular-rate feedback, small-error
output, disabled/invalid input handling, immediate positive/negative torque
clipping and reversal, and selected delayed/noisy nonlinear model simulations.
The simulations use fixed baseline gains; passing them does not validate every
user-selected Q/R, any particular hardware, or nested-axis auto-aim control.
