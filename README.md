# Gimbal

## 1. 模块作用
云台控制模块。实现 roll/yaw 闭环控制和模式切换。

## 2. 主要函数说明
1. ThreadFunc: 云台控制主线程。
2. ParseCMD: 解析 CMD 输入并更新目标。
3. Control: 角度环与角速度环计算控制输出。
4. Update: 刷新电机反馈并发布状态。
5. SetMode / GetEvent: 模式管理与事件接口。
6. DebugCommand: 调试命令入口（Debug 构建）。

## 3. 接入步骤
1. 添加模块并绑定 motor_roll、motor_yaw、cmd。
2. 配置零位、限位、惯量与 PID 参数。
3. 先验证模式切换，再联调控制参数。

云台姿态输入 topic：
- `gimbal_cmd`：CMD 发布的云台控制命令。
- `gimbal_euler`：云台 IMU 融合后的欧拉角。
- `gimbal_gyro`：云台 IMU 原始角速度。


标准命令流程：
    xrobot_add_mod Gimbal --instance-id gimbal
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: Gimbal
entry_header: Modules/Gimbal/Gimbal.hpp
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_yaw_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_yaw_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_roll_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_roll_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_roll: '@&motor_roll'
  - motor_yaw: '@&motor_yaw'
  - roll_max_angle: 0.0
  - roll_min_angle: 0.0
  - roll_lc: 0.0
  - roll_theta: 0.0
  - yaw_k: 0.0
  - j_roll: 0.0
  - j_yaw: 0.0
  - roll_zero: 0.0
  - yaw_zero: 0.0
  - patrol_range: 0.0
  - patrol_omega: 0.0
  - roll_reverse_flag: false
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
  - qdu-future/CMD
  - qdu-future/Motor
  - qdu-future/BMI088

## 6. 代码入口
Modules/Gimbal/Gimbal.hpp
