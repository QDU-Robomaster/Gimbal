# Gimbal 模块

## 模块描述
`Gimbal` 模块提供了一个高性能、可配置的云台控制系统。它采用了模板元编程，定义了一个顶层包装类 `Gimbal<MotorType>`，该类将具体的电机驱动实现（如 `RMMotorContainer`）作为模板参数。

该模块的核心功能是实现云台的精确姿态控制。它通过订阅IMU（惯性测量单元）的姿态数据（欧拉角、加速度、角速度），结合串级PID控制器（角度环+角速度环），实现对云台俯仰（Pitch）和偏航（Yaw）轴的稳定控制。

此外，模块内置了**重力补偿**算法，能够根据IMU测量的加速度矢量，实时计算并抵消由重力引起的扰动力矩，确保云台在车体倾斜时依然能保持姿态稳定。

## 依赖
*   **硬件**:
    *   云台电机与电调 (Motor)
    *   CAN 总线
    *   IMU (如 BMI088，提供姿态、角速度和加速度数据)
*   **软件模块**:
    *   `CMD`: 用于接收和解析来自遥控器或其他来源的控制指令。
    *   `Motor`: 用于驱动云台的Pitch和Yaw轴电机。
    *   `app_framework`: LibXR 核心框架。
    *   `pid`: 用于实现角度和角速度的串级闭环控制。
    *   `MadgwickAHRS` 或其他姿态解算模块: 用于提供实时的欧拉角。

## 构造参数
`Gimbal<MotorType>` 类的构造函数参数如下：

*   `hw`: `LibXR::HardwareContainer&`，硬件容器的引用。
*   `app`: `LibXR::ApplicationManager&`，应用管理器的引用。
*   `cmd`: `CMD&`，命令模块的引用，用于获取控制输入。
*   `task_stack_depth`: `uint32_t`，为云台控制任务分配的堆栈大小。
*   `pid_angle_param_yaw`, `pid_angle_param_pitch`: `LibXR::PID<float>::Param`，Yaw轴和Pitch轴的角度环PID参数。
*   `pid_omega_param_yaw`, `pid_omega_param_pitch`: `LibXR::PID<float>::Param`，Yaw轴和Pitch轴的角速度环PID参数。
*   `motor_can1`, `motor_can2`: `Motor<MotorType>&`，分别对应Yaw轴和Pitch轴的电机模块引用。
*   `gimbal_pitch_center_`, `gimbal_yaw_center_`: `float`，Pitch和Yaw轴的机械零点或中心位置。
*   `gimbal_max_`, `gimbal_min_`: `float`，Pitch轴的行程限位。
*   `gimbal_pitch_mass_`: `float`，Pitch轴上负载的质量（kg），用于重力补偿计算。
*   `radius_pitch_to_center_of_mass`: `const LibXR::Position<float>&`，Pitch轴旋转中心到其负载重心的向量（m）。
*   `radius_yaw_to_pitch`: `const LibXR::Position<float>&`，Yaw轴旋转中心到Pitch轴旋转中心的向量（m）。
*   `rotation_pitch_from_imu`: `const LibXR::RotationMatrix<float>&`，从IMU坐标系到Pitch轴坐标系的旋转矩阵，表示IMU的安装姿态。
*   `..._topic_name`: `const char*`，用于订阅IMU数据的Topic名称。

## 主要功能
*   **串级PID控制**: 采用“角度环-角速度环”串级PID控制，外环（角度环）的输出作为内环（角速度环）的输入，内环直接使用陀螺仪数据作为反馈，响应快速，控制稳定。
*   **重力补偿**: 通过IMU的加速度数据，前馈计算并补偿重力对云台姿态的扰动力矩。
*   **姿态解算集成**: 订阅并使用外部姿态解算模块（如 `MadgwickAHRS`）输出的欧拉角，用于闭环控制和补偿。
*   **电机角度处理**: 包含 `MotorNearestTransposition` 逻辑，处理电机角度的周期性问题，防止云台在边界处发生不必要的反转。
*   **可配置性**: 大量的构造参数允许用户精细调整PID参数、物理参数和硬件配置，以适应不同的云台结构。

## 设计原则与工作方式
`Gimbal` 模块在一个独立的线程 (`GimbalThread`) 中运行，其核心工作流程如下：

1.  **数据订阅**: 线程循环地从各个Topic（命令、加速度、欧拉角、陀螺仪）异步订阅最新数据。
2.  **设定点更新**: 调用 `UpdateSetpointFromCMD()`，根据遥控器等外部输入，更新云台的目标角度 `target_angle_yaw_` 和 `target_angle_pitch_`。
3.  **状态更新**: 调用 `SelfResolution()`，从电机编码器获取云台当前的的原始角度和角速度。
4.  **重力补偿计算**: 调用 `GravityCompensation()`，利用最新的加速度数据计算出需要补偿的 `pitch_torque_` 和 `yaw_torque_`。
5.  **动力学输出**: 调用 `OutputToDynamics()`，执行核心控制逻辑：
    *   首先调用 `MotorNearestTransposition()` 对目标角度进行处理。
    *   然后执行角度环PID计算，得到目标角速度。
    *   接着执行角速度环PID计算，得到基础输出力矩/电流。
    *   最后，将基础输出与重力补偿力矩相加，得到最终输出值，并发送给电机。

## 如何使用
```bash
# 添加 Gimbal 模块实例
xrobot_add_mod.exe Gimbal --instance-id gimbal
# 生成主应用程序入口
xrobot_gen_main.exe
```
然后在 `User/xrobot.yaml` 中配置 `Gimbal` 实例的模板参数和构造函数参数，以匹配您的硬件和控制需求。

## 主要方法

`Gimbal` 模块的控制逻辑主要由其内部线程周期性调用的一系列核心方法构成。

### 核心控制方法
这些方法在 `GimbalThread` 中被顺序调用，构成了一个完整的控制循环：

*   `void UpdateSetpointFromCMD()`: 从 `CMD` 模块获取最新的控制指令（如遥控器的输入），并据此更新云台的期望目标角度（`target_angle_pitch_` 和 `target_angle_yaw_`）。

*   `void SelfResolution()`: 从电机驱动层获取电机编码器的原始数据，解算出云台当前实时的角度（`now_angle_...`）和角速度（`now_omega_...`）。

*   `void GravityCompensation(accl_data_)`: 核心补偿算法。它利用IMU提供的加速度数据，通过一系列坐标变换，计算出重力在Pitch和Yaw轴上产生的扰动力矩，并将结果存入 `pitch_torque_` 和 `yaw_torque_`。

*   `void OutputToDynamics()`: 控制循环的最后一步。它执行串级PID计算（角度环和角速度环），将PID输出与 `GravityCompensation` 计算出的补偿力矩相加，得到最终的输出值，并将其作为电流/力矩指令发送给电机。
