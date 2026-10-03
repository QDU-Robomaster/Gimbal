# Gimbal

云台控制模块：pitch / yaw 两轴的角度环与角速度环闭环控制 / Gimbal control Module with cascaded angle and angular-velocity loops on the pitch and yaw axes

## 1. 模块作用 / Purpose

构造时，Gimbal 创建线程 `GimbalThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`）。线程订阅云台命令、姿态欧拉角和角速度三个 Topic，每轮依次刷新电机反馈、解析命令、计算并下发输出，然后休眠 2 ms。

控制以 IMU 欧拉角为角度反馈、IMU 角速度为角速度反馈，两轴各有一组角度环与角速度环。pitch 输出由转动惯量前馈（`j_pit`）、角速度环和重力补偿 `-pit_lc · sin(pitch + pit_theta)` 组成；yaw 输出由转动惯量前馈（`j_yaw`）、角速度环和阻尼项 `yaw_k · yaw 电机角速度` 组成。两轴输出均以力矩模式下发给电机。

每轮刷新反馈后，Gimbal 发布 yaw 与 pitch 电机相对零点的角度（见第 4 节 Topic）。

pitch 的机械限位由 `pit_max_angle` 与 `pit_min_angle` 给出，二者是 pitch 电机 `abs_angle` 的上下限，均为 0 时不限位。Gimbal 把限位换算为欧拉角范围并限制 pitch 目标，`reverse_flag` 给出电机角与欧拉角的方向关系。

Upon construction, Gimbal creates the thread `GimbalThread` (stack depth `param.task_stack_depth`, priority `param.thread_priority`). The thread subscribes to the gimbal command, attitude Euler angle and angular velocity Topics. Each iteration refreshes the motor feedback, parses the command, computes and sends the outputs, and then sleeps for 2 ms.

The control uses the IMU Euler angles as angle feedback and the IMU angular velocity as angular-velocity feedback, with one angle loop and one angular-velocity loop per axis. The pitch output consists of the moment-of-inertia feedforward (`j_pit`), the angular-velocity loop and the gravity compensation `-pit_lc · sin(pitch + pit_theta)`. The yaw output consists of the moment-of-inertia feedforward (`j_yaw`), the angular-velocity loop and the damping term `yaw_k · yaw motor angular velocity`. Both outputs are sent to the motors in torque mode.

After each feedback refresh, Gimbal publishes the yaw and pitch motor angles relative to their zero points (see Topic in section 4).

The pitch mechanical limit is given by `pit_max_angle` and `pit_min_angle`, the upper and lower bounds of the pitch motor `abs_angle`; limiting is disabled when both are 0. Gimbal converts the limits to an Euler angle range and clamps the pitch target with it; `reverse_flag` gives the direction relationship between the motor angle and the Euler angle.

## 2. 模式与目标解析 / Modes and Target Resolution

`GetEvent()` 返回的 `LibXR::Event` 注册了全局枚举 `GimbalEvent` 的四个值：`SET_MODE_RELAX`、`SET_MODE_COMMON`、`SET_MODE_AUTOPATROL`、`SET_MODE_LOW_SENSITIVITY`。激活对应的事件 ID 即切换模式，通常由 `EventBinder` 把遥控器事件绑定到这些 ID。CMD 的 `CMD_EVENT_START_CTRL` 与 `CMD_EVENT_LOST_CTRL` 都切换到 `SET_MODE_RELAX`。

- `SET_MODE_RELAX`：失能电机，目标清零，PID 复位；运行期间两轴电机 `Relax()`。
- 其他模式：切入时以当前姿态为目标并复位 PID；`SET_MODE_COMMON` 与 `SET_MODE_LOW_SENSITIVITY` 之间切换时保持当前目标与 PID 状态。

目标由 CMD 的控制模式决定：

- 操作员控制（`CMD_OP_CTRL`）：`gimbal_cmd` 的 yaw / pit 作为角速度输入，积分为目标角，最大 4π rad/s；`SET_MODE_LOW_SENSITIVITY` 下乘以 0.1。
- 自动控制且 `cmd.GetAIGimbalStatus()` 为真：直接使用 `gimbal_cmd` 的目标角及其一阶、二阶导数，导数作为前馈。
- 自动控制且 `cmd.GetAIGimbalStatus()` 为假：`SET_MODE_AUTOPATROL` 下 yaw 目标以 1 rad/s 匀速增加，pitch 目标按 `patrol_range` 与 `patrol_omega` 周期性摆动；其他模式按摇杆输入积分，yaw 方向取反。

`GetEvent()` returns a `LibXR::Event` on which the four values of the global enum `GimbalEvent` are registered: `SET_MODE_RELAX`, `SET_MODE_COMMON`, `SET_MODE_AUTOPATROL` and `SET_MODE_LOW_SENSITIVITY`. Activating the corresponding event ID switches the mode; an `EventBinder` usually binds remote-controller events to these IDs. The CMD events `CMD_EVENT_START_CTRL` and `CMD_EVENT_LOST_CTRL` both switch to `SET_MODE_RELAX`.

- `SET_MODE_RELAX`: disables the motors, clears the targets and resets the PIDs; while it is active both axis motors are `Relax()`ed.
- Other modes: on entry, the current attitude becomes the target and the PIDs are reset; switching between `SET_MODE_COMMON` and `SET_MODE_LOW_SENSITIVITY` keeps the current target and PID state.

The target depends on the CMD control mode:

- Operator control (`CMD_OP_CTRL`): the yaw / pit of `gimbal_cmd` are angular-velocity inputs integrated into the target angles, at most 4π rad/s; multiplied by 0.1 in `SET_MODE_LOW_SENSITIVITY`.
- Automatic control with `cmd.GetAIGimbalStatus()` true: the target angles of `gimbal_cmd` are used directly, together with their first and second derivatives as feedforward.
- Automatic control with `cmd.GetAIGimbalStatus()` false: in `SET_MODE_AUTOPATROL` the yaw target increases at a constant 1 rad/s and the pitch target oscillates periodically according to `patrol_range` and `patrol_omega`; in the other modes the stick input is integrated, with the yaw direction inverted.

## 3. 构造接口 / Constructor

```cpp
Gimbal(CMD& cmd,
       Motor& motor_pit,
       Motor& motor_yaw,
       Referee* referee,
       const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `cmd`：`CMD` 实例，提供云台命令、控制模式与 CMD 事件。
- `motor_pit`、`motor_yaw`：`Motor`，pitch 与 yaw 电机，例如 `DMMotor` 实例。
- `referee`：`Referee*`，Referee 实例的指针，可为 `nullptr`。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 2048。
- `pid_yaw_angle`、`pid_yaw_omega`：yaw 角度环与角速度环，默认各项为 0（包括 `k`），`cycle = true`。
- `pid_pit_angle`、`pid_pit_omega`：pitch 角度环与角速度环，默认各项为 0（包括 `k`），`cycle = false`。
- `pit_max_angle`、`pit_min_angle`：pitch 电机角度上下限，单位 rad，默认 0（不限位）。
- `pit_lc`：pitch 质心距离（m，水平向上为正）乘以 pitch 质心重力（N），用于重力补偿，默认 0。
- `pit_theta`：pitch 质心与重力轴线的夹角，单位 rad，默认 0。
- `yaw_k`：yaw 阻尼系数，默认 0。
- `j_pit`、`j_yaw`：pitch 与 yaw 的转动惯量，单位 kg·m²，用于前馈，默认 0。
- `pit_zero`、`yaw_zero`：pitch 与 yaw 电机零点，单位 rad，默认 0。
- `patrol_range`、`patrol_omega`：自动巡逻的 pitch 摆动幅度与角频率，默认 0。
- `reverse_flag`：pitch 电机角与欧拉角同向时为 `true`，默认 `false`。
- `thread_priority`：线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。
- `euler_topic_name`：订阅的姿态 Topic 名称，默认 `"ahrs_euler"`。
- `gyro_topic_name`：订阅的角速度 Topic 名称，默认 `"bmi088_gyro"`。
- `gimbal_cmd_topic_name`：订阅的云台命令 Topic 名称，默认 `"gimbal_cmd"`。

Dependencies:

- `cmd`: the `CMD` instance, providing the gimbal command, the control mode and the CMD events.
- `motor_pit`, `motor_yaw`: `Motor` objects for the pitch and yaw motors, for example `DMMotor` instances.
- `referee`: `Referee*`, a pointer to a Referee instance; may be `nullptr`.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 2048.
- `pid_yaw_angle`, `pid_yaw_omega`: yaw angle loop and angular-velocity loop, all terms default to 0 (including `k`), `cycle = true`.
- `pid_pit_angle`, `pid_pit_omega`: pitch angle loop and angular-velocity loop, all terms default to 0 (including `k`), `cycle = false`.
- `pit_max_angle`, `pit_min_angle`: upper and lower bounds of the pitch motor angle in rad, default 0 (limiting disabled).
- `pit_lc`: pitch center-of-mass distance (m, positive upward from the horizontal) multiplied by the pitch center-of-mass weight (N), used for gravity compensation, default 0.
- `pit_theta`: angle between the pitch center of mass and the gravity axis in rad, default 0.
- `yaw_k`: yaw damping coefficient, default 0.
- `j_pit`, `j_yaw`: pitch and yaw moments of inertia in kg·m², used for feedforward, default 0.
- `pit_zero`, `yaw_zero`: pitch and yaw motor zero points in rad, default 0.
- `patrol_range`, `patrol_omega`: pitch oscillation amplitude and angular frequency of the automatic patrol, default 0.
- `reverse_flag`: `true` when the pitch motor angle and the Euler angle have the same direction, default `false`.
- `thread_priority`: thread priority, default `LibXR::Thread::Priority::MEDIUM`.
- `euler_topic_name`: name of the subscribed attitude Topic, default `"ahrs_euler"`.
- `gyro_topic_name`: name of the subscribed angular-velocity Topic, default `"bmi088_gyro"`.
- `gimbal_cmd_topic_name`: name of the subscribed gimbal command Topic, default `"gimbal_cmd"`.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.gimbal_cmd_topic_name`（默认 `gimbal_cmd`） | 订阅 | `CMD::GimbalCMD` | CMD 发布的云台命令 |
| `param.euler_topic_name`（默认 `ahrs_euler`） | 订阅 | `LibXR::EulerAngle<float>` | 云台 IMU 姿态，pitch 取反后使用 |
| `param.gyro_topic_name`（默认 `bmi088_gyro`） | 订阅 | `Eigen::Matrix<float, 3, 1>` | 云台 IMU 角速度，y 分量取反后使用 |
| `yawmotor_angle` | 发布 | `float` | yaw 电机 `abs_angle - yaw_zero`，单位 rad，供底盘跟随使用 |
| `pitchmotor_angle` | 发布 | `float` | pitch 电机 `abs_angle - pit_zero`，单位 rad |

订阅的 Topic 名称与发布方实例使用的名称相同：`gimbal_cmd_topic_name` 与 CMD 的 `gimbal_cmd_topic_name` 相同，`euler_topic_name` 与 `gyro_topic_name` 与 BSP 中 IMU 和姿态解算实例发布的名称相同。

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.gimbal_cmd_topic_name` (default `gimbal_cmd`) | Subscribe | `CMD::GimbalCMD` | Gimbal command published by CMD |
| `param.euler_topic_name` (default `ahrs_euler`) | Subscribe | `LibXR::EulerAngle<float>` | Gimbal IMU attitude, pitch is negated before use |
| `param.gyro_topic_name` (default `bmi088_gyro`) | Subscribe | `Eigen::Matrix<float, 3, 1>` | Gimbal IMU angular velocity, the y component is negated before use |
| `yawmotor_angle` | Publish | `float` | Yaw motor `abs_angle - yaw_zero` in rad, used by the chassis follow |
| `pitchmotor_angle` | Publish | `float` | Pitch motor `abs_angle - pit_zero` in rad |

The subscribed Topic names match the names used by the publishing instances: `gimbal_cmd_topic_name` matches the `gimbal_cmd_topic_name` of CMD, and `euler_topic_name` and `gyro_topic_name` match the names published by the IMU and attitude instances in the BSP.

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/Gimbal` 写入的实例，依赖填写为其他 Module 实例的 id，PID、零点与限位按实机标定：

An instance written by `xrobot instance add QDU-Robomaster/Gimbal`, with the dependencies set to the ids of other Module instances, and the PIDs, zero points and limits calibrated on the machine:

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
          task_stack_depth: 2048
          pid_yaw_angle:
            k: 1.0f
            p: 8.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 20.0f
            cycle: true
          pid_yaw_omega:
            k: 1.0f
            p: 0.5f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 5.0f
            cycle: true
          pid_pit_angle:
            k: 1.0f
            p: 10.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 20.0f
            cycle: false
          pid_pit_omega:
            k: 1.0f
            p: 0.6f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 5.0f
            cycle: false
          pit_max_angle: 0.5f
          pit_min_angle: -0.3f
          pit_lc: 0.2f
          pit_theta: 0.0f
          yaw_k: 0.0f
          j_pit: 0.01f
          j_yaw: 0.02f
          pit_zero: 0.0f
          yaw_zero: 0.0f
          patrol_range: 0.0f
          patrol_omega: 0.0f
          reverse_flag: true
          thread_priority: LibXR::Thread::Priority::MEDIUM
          euler_topic_name: "ahrs_euler"
          gyro_topic_name: "bmi088_gyro"
          gimbal_cmd_topic_name: "gimbal_cmd"
```

`cmd` 取自 `QDU-Robomaster/CMD` 实例，`motor_pit` 与 `motor_yaw` 取自 `QDU-Robomaster/DMMotor`（或其他 `Motor` 实现）实例，它们须在本实例之前列出。`referee` 填 `nullptr`，或填 `QDU-Robomaster/Referee` 实例的 id。

`cmd` is taken from a `QDU-Robomaster/CMD` instance, and `motor_pit` and `motor_yaw` from `QDU-Robomaster/DMMotor` (or other `Motor` implementation) instances; they are listed before this instance. `referee` is `nullptr` or the id of a `QDU-Robomaster/Referee` instance.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CMD`：云台命令、控制模式与 CMD 事件。
- `QDU-Robomaster/Motor`：pitch 与 yaw 电机的接口。
- `QDU-Robomaster/Referee`：`referee` 参数的类型。
- LibXR。

硬件：pitch 与 yaw 两个实现 `Motor` 接口的电机，以及发布欧拉角和角速度 Topic 的云台 IMU 与姿态解算。

Dependencies:

- `QDU-Robomaster/CMD`: gimbal command, control mode and CMD events.
- `QDU-Robomaster/Motor`: interface of the pitch and yaw motors.
- `QDU-Robomaster/Referee`: type of the `referee` parameter.
- LibXR.

Hardware: pitch and yaw motors implementing the `Motor` interface, and a gimbal IMU with attitude estimation that publishes the Euler angle and angular velocity Topics.
