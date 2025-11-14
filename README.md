# Gimbal 模块

## 模块描述
`Gimbal` 模块提供了一个高性能、可配置的云台控制系统。它采用了模板元编程，定义了一个顶层包装类 `Gimbal<MotorType>`，该类将具体的电机驱动实现作为模板参数。

该模块的核心功能是实现云台的精确姿态控制。它通过订阅IMU（惯性测量单元）的姿态数据（欧拉角、加速度、角速度），结合串级PID控制器（角度环+角速度环），实现对云台俯仰（Pitch）和偏航（Yaw）轴的稳定控制。

此外，模块设计支持**重力补偿**算法，能够根据IMU测量的加速度矢量，实时计算并抵消由重力引起的扰动力矩，确保云台在车体倾斜时依然能保持姿态稳定。

## 依赖
*   **硬件**:
    *   云台电机与电调 (Motor)
    *   CAN 总线
    *   IMU (如 BMI088，提供姿态、角速度和加速度数据)
*   **软件模块**:
    *   `CMD`用于接收和解析来自遥控器或其他来源的控制指令。
    *   `Motor`: 用于驱动云台的Pitch和Yaw轴电机。
    *   `app_framework`: LibXR 核心框架。
    *   `pid`: 用于实现角度和角速度的串级闭环控制。
    *   `MadgwickAHRS`或其他姿态解算模块: 用于提供实时的欧拉角。

## 构造参数
`Gimbal<MotorType>` 类的构造函数接受以下参数：

*   `hw`: `LibXR::HardwareContainer&`，硬件容器的引用。
*   `app`: `LibXR::ApplicationManager&`，应用管理器的引用。
*   `task_stack_depth`:为云台控制任务分配的堆栈大小。
*   `param_pid_yaw_speed`: `LibXR::PID<float>::Param`，Yaw轴的速度环PID参数。
*   `param_pid_yaw_angle`: `LibXR::PID<float>::Param`，Yaw轴的角度环PID参数。
*   `param_pid_pitch_speed`: `LibXR::PID<float>::Param`，Pitch轴的速度环PID参数。
*   `param_pid_pitch_angle`: `LibXR::PID<float>::Param`，Pitch轴的角度环PID参数。
*   `motor_yaw`: `typename MotorType::RMMotor*`，指向Yaw轴电机实例的指针。
*   `motor_pitch`: `typename MotorType::RMMotor*`，指向Pitch轴电机实例的指针。
*   `gimbal_param`: `GimbalParam`，云台物理参数结构体，包括：
    *   `I_pitch`: Pitch轴转动惯量
    *   `I_yaw`: Yaw轴转动惯量
    *   `M_pitch`: Pitch轴质心质量
    *   `G_pitch`: Pitch轴质心位置
    *   `imu_pitch_min`: IMU视角下Pitch轴下限（弧度）
    *   `imu_pitch_max`: IMU视角下Pitch轴上限（弧度）
*   `gimbal_cmd_name_`: `const char*`，云台控制命令主题名称。
*   `accl_name_`: `const char*`，加速度计数据主题名称。
*   `euler_name_`: `const char*`，欧拉角数据主题名称。
*   `gyro_name_`: `const char*`，陀螺仪数据主题名称。

## 主要功能
*   **双环PID控制**: 采用"角度环-角速度环"串级PID控制，外环（角度环）的输出作为内环（角速度环）的输入，内环直接使用陀螺仪数据作为反馈，响应快速，控制稳定。
*   **重力补偿**: 通过IMU的加速度数据，前馈计算并补偿重力对云台姿态的扰动力矩。
*   **姿态解算集成**: 订阅并使用外部姿态解算模块输出的欧拉角，用于闭环控制和补偿。
*   **多数据源订阅**: 支持从不同主题订阅控制命令、加速度、欧拉角和陀螺仪数据。
*   **可配置性**: 大量的构造参数允许用户精细调整PID参数、物理参数和硬件配置，以适应不同的云台结构。

## 设计原则与工作方式
`Gimbal` 模块在一个独立的线程中运行，其核心工作流程如下：

1.  **数据订阅**: 线程循环地从各个Topic（命令、加速度、欧拉角、陀螺仪）异步订阅最新数据。
2.  **状态更新**: 调用 `UpdateFeedBack()` 方法，从电机编码器获取云台当前的角度和角速度，并计算时间间隔。
3.  **前馈计算**: 调用 `GetGravityFeedForward()` 和 `GetMotionalFeedForward()` 方法，计算重力和运动补偿扭矩。
4.  **控制输出**: 调用 `Control()` 方法，执行核心控制逻辑：
    *   执行角度环和速度环的串级PID计算
    *   将PID输出作为力矩指令发送给电机

## 数据结构

### GimbalParam 结构体
描述云台物理参数：
*   `I_pitch`, `I_yaw`: 各轴转动惯量
*   `M_pitch`: Pitch轴质心质量
*   `G_pitch`: Pitch轴质心位置
*   `pitch_min`, `pitch_max`: Pitch电机机械限位
*   `imu_pitch_min`, `imu_pitch_max`: IMU视角下的Pitch轴限位

### State 结构体
描述云台当前状态：
*   `accl`: 加速度向量
*   `euler`: 欧拉角
*   `gyro`: 角速度向量
*   `yaw_feedback_`, `pitch_feedback_`: 各轴反馈信息（角度、速度、扭矩）
