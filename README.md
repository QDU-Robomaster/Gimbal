# Gimbal

双 yaw 使用联合 LQR 跟踪 IMU 总指向。大误差时两轴一起追目标，接近目标后小轴平滑回到指定的相对角度，
大轴参考做等量反向修正。Pitch 可接独立的角度环与角速度环 PID。

## 接入

- 用 XRobot 加载本分支的 Gimbal 与 CMD、Motor，并绑定小 yaw、大 yaw 两个电机实例。
  代码订阅 `gimbal_cmd`、`ahrs_euler` 和 `bmi088_gyro`。
- 大小 yaw 电机的 `Motor::Feedback::sample_time_us` 要在每次解码新反馈时更新为当前微秒时间戳。
  本地双 yaw 工程已有这个字段。使用组织仓库的现有 Motor、RMMotor、DMMotor 时，可以从 BSP 根目录
  对 `Modules/QDU-Robomaster` 应用 [时间戳补丁](patches/motor-feedback-sample-time.patch)。
  请使用三份驱动源码一起重新编译。反馈时间戳为零时云台保持零跟踪力矩。

```
git apply --directory=Modules/QDU-Robomaster Modules/QDU-Robomaster/Gimbal/patches/motor-feedback-sample-time.patch
```

以下配置来自双 yaw 台架。K、力矩和行程参数按装置填写，先创建 `cmd`、`motor_yaw`、`motor_yaw2` 实例。

```yaml
modules:
  - module: QDU-Robomaster/Gimbal
    id: gimbal
    args:
      - cmd: cmd
      - task_stack_depth: 1536
      - pid_pit_angle: '{0.0, 13.5, 0.0, 0.0, 0.0, 0.0, false}'
      - pid_pit_omega: '{0.0, 0.9, 0.0, 0.0, 0.0, 0.0, false}'
      - motor_pit: nullptr
      - motor_yaw: '&motor_yaw'
      - motor_yaw2: '&motor_yaw2'
      - pit_max_angle: 0.0
      - pit_min_angle: 0.0
      - pit_lc: 0.0
      - pit_theta: 0.0
      - j_pit: 0.0
      - pit_zero: 0.0
      - yaw_zero: 0.0
      - yaw2_zero: 0.0
      - patrol_range: 0.0
      - patrol_omega: 0.0
      - reverse_flag: true
      - thread_priority: LibXR::Thread::Priority::HIGH
      - yaw_limit:
          min_position: -2.18
          max_position: -1.19
      - dual_yaw:
          k11: 22.653790921
          k12: 1.034482834
          k13: 1.375579493
          k14: 0.135401333
          k21: -1.362651622
          k22: 6.085657979
          k23: 0.054146851
          k24: 0.255937553
          big_direction: -1.0
          big_position_scale: 2.0
          imu_direction: -1.0
          big_torque_limit: 7.0
          small_torque_limit: 1.2
          big_max_travel: 100
          big_max_rate: 3.5
          small_margin: 0.08
          big_reference_ramp: true
          small_center_position: -1.685
          small_recenter_error: 0.05
          small_recenter_rate: 0.15
      - small_yaw_feedforward:
          J: 0.005505764873850784
          B: 0.03659462621840192
          Fc: 0.063000686
```

在 Ozone 把当前 `gimbal.dbg_imu_yaw_` 填到 `gimbal.dbg_imu_yaw_target_`，
再将 `gimbal.current_mode_` 写为 `1`（COMMON）。随后可以写绝对 Euler yaw 目标角（rad）。
写回 `0`（RELAX）停止跟踪。

## Standalone single-axis LQR (single_lqr branch)

[`single_lqr.hpp`](single_lqr.hpp) computes feedback torque from angle and rate
errors. Use [`tools/design_single_lqr.m`](tools/design_single_lqr.m) to calculate
the gains. The calling application supplies the errors and applies feedforward
and motor limits. See [docs/single_lqr.md](docs/single_lqr.md) for the API and units.

## 独立双 yaw 控制器

[DoubleYaw.hpp](DoubleYaw.hpp) 可直接用于 C++17 工程。`Calculate` 算两轴参考、
反馈力矩和限幅，`CalculateFeedback` 算 2×4 增益矩阵与四项误差的乘积。
误差顺序为大轴角度、小轴相对角度、大轴角速度、小轴相对角速度。

1. 用 MATLAB 和 Control System Toolbox 运行 [双轴调参脚本](tools/design_dual_yaw_lqr.m)。
   按装置的 J、B、采样周期和 Q/R 求 K。脚本检查离散闭环极点并打印 `k11` 到 `k24`，
   由调用方填入 YAML 或 `DoubleYaw::Param`。同时配置编码器比例、方向和两轴限幅。
2. 每次取得大轴位置反馈时调用 `UpdateBigFeedback(raw_position, sample_us)`。
   时间戳单位为 µs，重复时间戳会跳过。角速度由连续两帧位置差分后低通估计。
3. 每拍填 `DoubleYaw::Input` 并调用 `Calculate(input)`。
   `aim_error` 为 IMU 坐标下环绕到 ±π 的目标减实测角度，`aim_rate` 为 IMU 总指向的实测角速度。
   `small_position`、`min_position`、`max_position` 使用同一小轴编码器坐标，正方向与校准后的 IMU yaw 一致。
   固定角度目标可将 `target_rate` 和 `target_accel` 填 0。角度用 rad，速度用 rad/s，`dt` 用秒。
4. 成功返回后读取 `big_torque_` 和 `small_torque_`（N·m）。大轴力矩乘以 `big_direction`
   转回电机指令方向。反馈超时等状态由调用方处理，暂停时调用 `Suspend()` 并停止跟踪力矩，
   需要重新记录大轴行程中心时调用 `Reset()`。

带前馈时用 `Calculate(input, small_feedforward)`。回调接收 `DoubleYaw::SmallYawReference`，
并返回小轴前馈力矩。回调输出在做力矩和位置限幅前加入小轴 LQR 输出。
调用示例和限幅验证见 [tests/test_double_yaw.cpp](tests/test_double_yaw.cpp)。
矩阵乘法验证见 [tests/test_dual_yaw_lqr.cpp](tests/test_dual_yaw_lqr.cpp)。
