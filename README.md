# Gimbal

云台控制模块：pitch / yaw 两轴的角度环 + 角速度环闭环控制，带转动惯量前馈、pitch 重力补偿、
yaw 阻尼补偿和 pitch 机械限位。

## 工作方式

- 构造时创建线程 `GimbalThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`），
  每轮循环后休眠 2 ms：刷新电机反馈 → 解析命令 → 计算并下发输出。
- 模式：`GetEvent()` 返回的 `LibXR::Event` 注册了全局枚举 `GimbalEvent` 的四个值
  `SET_MODE_RELAX`、`SET_MODE_COMMON`、`SET_MODE_AUTOPATROL`、`SET_MODE_LOW_SENSITIVITY`，
  激活对应事件 ID 即切换模式（通常由 `EventBinder` 绑定遥控器事件）。CMD 的
  `CMD_EVENT_START_CTRL` 与 `CMD_EVENT_LOST_CTRL` 都切到 `SET_MODE_RELAX`。进入 `RELAX` 时失能电机、
  清零目标；进入其他模式时以当前姿态为目标并复位 PID（`COMMON` 与 `LOW_SENSITIVITY` 之间切换不复位）。
- 目标解析（CMD 的控制模式）：
  - 操作员控制（`CMD_OP_CTRL`）：`gimbal_cmd` 的 yaw / pit 作为角速度输入积分到目标角，
    最大 4π rad/s；`LOW_SENSITIVITY` 模式下乘以 0.1。
  - 自动控制且 `cmd.GetAIGimbalStatus()` 为真：直接使用 `gimbal_cmd` 的目标角及其一阶、二阶导数
    （作为前馈）。
  - 自动控制但没有 AI 目标：`AUTOPATROL` 模式下 yaw 目标以 1 rad/s 匀速增加、pitch 目标按
    `patrol_range` / `patrol_omega` 的三角波函数变化；其他模式按摇杆积分（yaw 方向取反）。
- 控制：角度误差以 IMU 欧拉角为反馈，角速度环以陀螺仪角速度为反馈；
  pitch 输出 = 惯量前馈（`j_pit`）+ 角速度环 + 重力补偿 `-pit_lc · sin(pitch + pit_theta)`；
  yaw 输出 = 惯量前馈（`j_yaw`）+ 角速度环 + `yaw_k · yaw 电机角速度`。两轴都以 `MODE_TORQUE`
  下发；电机 `state == 0` 时先 `Enable()`，`state` 不为 0 / 1 时 `ClearError()`。`RELAX` 模式下两轴
  `Relax()`。
- pitch 限位：`pit_max_angle` 与 `pit_min_angle` 是 pitch 电机 `abs_angle` 的上下限（都为 0 时不限位），
  模块把它们换算为欧拉角范围并限制 pitch 目标；`reverse_flag` 给出电机角与欧拉角的方向关系。

Topic：

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gimbal_cmd` | 订阅 | `CMD::GimbalCMD` | CMD 发布的云台命令 |
| `param.euler_topic_name`（默认 `ahrs_euler`） | 订阅 | `LibXR::EulerAngle<float>` | 云台 IMU 姿态，pitch 取反后使用 |
| `param.gyro_topic_name`（默认 `bmi088_gyro`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 云台 IMU 角速度，y 分量取反后使用 |
| `yawmotor_angle` | 发布 | `float` | yaw 电机 `abs_angle - yaw_zero` (rad)，底盘跟随使用 |
| `pitchmotor_angle` | 发布 | `float` | pitch 电机 `abs_angle - pit_zero` (rad) |

## 依赖

- `QDU-Robomaster/CMD`：云台命令、控制模式与 CMD 事件。
- `QDU-Robomaster/Motor`：pitch / yaw 电机的抽象接口。
- `QDU-Robomaster/Referee`：构造参数类型；当前代码保存该指针但不使用。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
Gimbal(CMD& cmd,
       Motor& motor_pit,
       Motor& motor_yaw,
       Referee* referee,
       const Param& param = {...});
```

依赖：

- `cmd`：`CMD` 实例。
- `motor_pit`、`motor_yaw`：`Motor`，pitch 与 yaw 电机（如 `DMMotor` 实例）。
- `referee`：`Referee*`，当前未使用，可填 `nullptr`。

配置（`Param`；PID 为 `LibXR::PID<float>::Param`，字段 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 2048。
- `pid_yaw_angle`、`pid_yaw_omega`：yaw 角度环 / 角速度环，默认各项为 0、`cycle = true`。
- `pid_pit_angle`、`pid_pit_omega`：pitch 角度环 / 角速度环，默认各项为 0、`cycle = false`。
- `pit_max_angle`、`pit_min_angle`：pitch 电机角度上下限 (rad)，默认 0（不限位）。
- `pit_lc`：pitch 质心距离 (m) × 重力 (N)，用于重力补偿，默认 0。
- `pit_theta`：pitch 质心与重力轴线夹角 (rad)，默认 0。
- `yaw_k`：yaw 阻尼系数，默认 0。
- `j_pit`、`j_yaw`：pitch / yaw 转动惯量 (kg·m²)，用于前馈，默认 0。
- `pit_zero`、`yaw_zero`：pitch / yaw 电机零点 (rad)，默认 0。
- `patrol_range`、`patrol_omega`：自动巡逻的 pitch 幅度与频率，默认 0。
- `reverse_flag`：pitch 电机角与欧拉角同向为 `true`，默认 `true`。
- `thread_priority`：线程优先级，默认 `MEDIUM`。
- `euler_topic_name`：订阅的姿态 Topic，默认 `"ahrs_euler"`。
- `gyro_topic_name`：订阅的角速度 Topic，默认 `"bmi088_gyro"`。

## 使用

```sh
xrobot module add QDU-Robomaster/Gimbal
xrobot setup
xrobot instance add QDU-Robomaster/Gimbal
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
填好依赖后按实车标定 PID、零点与限位：

```yaml
modules:
  - module: QDU-Robomaster/Gimbal
    id: gimbal_0
    args:
      - cmd: cmd
      - motor_pit: motor_pit
      - motor_yaw: motor_yaw
      - referee: nullptr
      - param:
          task_stack_depth: '2048'
          pid_yaw_angle:
            k: 0.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'true'
          pid_yaw_omega:
            k: 0.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'true'
          pid_pit_angle:
            k: 0.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          pid_pit_omega:
            k: 0.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          pit_max_angle: 0.0f
          pit_min_angle: 0.0f
          pit_lc: 0.0f
          pit_theta: 0.0f
          yaw_k: 0.0f
          j_pit: 0.0f
          j_yaw: 0.0f
          pit_zero: 0.0f
          yaw_zero: 0.0f
          patrol_range: 0.0f
          patrol_omega: 0.0f
          reverse_flag: 'true'
          thread_priority: LibXR::Thread::Priority::MEDIUM
          euler_topic_name: '"ahrs_euler"'
          gyro_topic_name: '"bmi088_gyro"'
```

`cmd`、`motor_pit`、`motor_yaw` 是其他模块实例的 id，须在本实例之前列出：`cmd` 由
`QDU-Robomaster/CMD` 提供，`motor_pit` / `motor_yaw` 为 `QDU-Robomaster/DMMotor`（或其他 `Motor`
驱动）实例。`referee` 也可以填 `QDU-Robomaster/Referee` 实例的 id（指针参数会取其地址）。
本例没有需要 BSP 用 `XR_REGISTER` 注册的对象；`euler_topic_name` / `gyro_topic_name` 必须与
BSP 中 IMU 与姿态解算实例发布的 Topic 名一致。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/Gimbal`
（在 BSP 中）打印当前的构造函数。
