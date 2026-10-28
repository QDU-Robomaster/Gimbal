#pragma once

#include <cstdint>

/**
 * @brief 云台逐拍日志 Topic `gimbal_log` 的载荷，MCU（Gimbal）与上位机（VisionRecorder）
 *        共用同一份定义，两端必须逐字节一致，改动时同步修改两份。
 *        Payload of the per-tick gimbal log Topic `gimbal_log`. The MCU (Gimbal) and the
 *        host (VisionRecorder) share this definition and the two copies must stay
 *        byte-identical; change both together.
 *
 * 字段顺序即上位机 gimbal_log.csv 的列顺序。pit / gyro_pit 是 Solve 实际使用的值（pitch
 * 已翻转符号）。mode = (RELAX ? 1000 : 0) + 辨识类型 * 100 + 阶段：类型 0 为普通控制，
 * 1 为 pitch 扫描（阶段 1 趋近下限 2 上行 3 下行 4 回到起点 5 结束保持），2 为 yaw 梯形
 * （阶段 1 pitch 就位 2 正向加速 3 正向匀速 4 正向减速 5 正向停顿 6 反向加速 7 反向匀速
 * 8 反向减速 9 反向停顿 10 结束保持），3 为 pitch 正弦（阶段 1 就位 2 正弦 3 结束保持）。
 * The field order is the column order of the host's gimbal_log.csv. pit / gyro_pit are
 * the values Solve actually uses (pitch sign already flipped). mode = (RELAX ? 1000 : 0)
 * + identification kind * 100 + phase: kind 0 is normal control, 1 is the pitch sweep
 * (phase 1 approach the lower end, 2 up, 3 down, 4 return to the start, 5 end hold),
 * 2 is the yaw trapezoid (phase 1 pitch settle, 2 accelerate +, 3 cruise +,
 * 4 decelerate +, 5 pause +, 6 accelerate -, 7 cruise -, 8 decelerate -, 9 pause -,
 * 10 end hold), 3 is the pitch sine (phase 1 settle, 2 sine, 3 end hold).
 *
 * yaw 用模型补偿控制器（GimbalMbc）时 mode 另加 10000，pitch 用时另加 20000。用该控制器的
 * 轴，ff_* 为 J * 目标角加速度加耦合与摩擦，fb_* 为状态反馈，eso_* 为 ESO 补偿；用 PID 的轴
 * eso_* 为 0。
 * mode has 10000 added when the yaw uses the model-based controller (GimbalMbc) and 20000
 * when the pitch does. On an axis run by that controller ff_* is J * target acceleration
 * plus coupling and friction, fb_* the state feedback and eso_* the ESO compensation; on
 * an axis run by the PID eso_* is 0.
 */
struct __attribute__((packed)) GimbalLog
{
  uint32_t t_us;   ///< 本拍时间戳 (µs，32 位回绕) / Tick timestamp (µs, wraps at 32 bits)
  uint32_t dt_us;  ///< 与上一拍的间隔 (µs) / Interval to the previous tick (µs)
  uint32_t mode;   ///< 模式与阶段，见上 / Mode and phase, see above
  float tgt_pit;   ///< pitch 目标角 (rad) / Pitch target angle
  float tgt_yaw;   ///< yaw 目标角 (rad) / Yaw target angle
  float tgt_pit_dot;    ///< pitch 目标角速度 (rad/s) / Pitch target rate
  float tgt_yaw_dot;    ///< yaw 目标角速度 (rad/s) / Yaw target rate
  float pit;            ///< pitch 欧拉角 (rad) / Pitch Euler angle
  float yaw;            ///< yaw 欧拉角 (rad) / Yaw Euler angle
  float gyro_pit;       ///< pitch 角速度 (rad/s) / Pitch angular velocity
  float gyro_yaw;       ///< yaw 角速度 (rad/s) / Yaw angular velocity
  float ff_pit;         ///< pitch 转动惯量前馈 (Nm) / Pitch inertia feedforward
  float fb_pit;         ///< pitch 角速度环输出 (Nm) / Pitch rate-loop output
  float grav_pit;       ///< pitch 重力补偿 (Nm) / Pitch gravity compensation
  float u_pit;          ///< pitch 最终力矩 (Nm) / Pitch final torque
  float ff_yaw;         ///< yaw 转动惯量前馈 (Nm) / Yaw inertia feedforward
  float fb_yaw;         ///< yaw 角速度环输出 (Nm) / Yaw rate-loop output
  float u_yaw;          ///< yaw 最终力矩 (Nm) / Yaw final torque
  float mot_pit_angle;  ///< pitch 电机 abs_angle (rad) / Pitch motor abs_angle
  float mot_pit_omega;  ///< pitch 电机角速度 (rad/s) / Pitch motor angular velocity
  float mot_pit_torque;  ///< pitch 电机反馈力矩 (Nm，电流换算) / Pitch motor torque (from
                         ///< current)
  float mot_yaw_angle;  ///< yaw 电机 abs_angle (rad) / Yaw motor abs_angle
  float mot_yaw_omega;  ///< yaw 电机角速度 (rad/s) / Yaw motor angular velocity
  float mot_yaw_torque;  ///< yaw 电机反馈力矩 (Nm，电流换算) / Yaw motor torque (from
                         ///< current)
  float eso_pit;  ///< pitch ESO 补偿 (Nm) / Pitch ESO compensation
  float eso_yaw;  ///< yaw ESO 补偿 (Nm) / Yaw ESO compensation
};

static_assert(sizeof(GimbalLog) == 12 + 23 * 4, "GimbalLog layout changed");
