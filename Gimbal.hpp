#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_angle_param_yaw:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_angle_param_pitch:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_param_yaw:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_omega_param_pitch:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - gimbal_pitch_center: 0.0
  - gimbal_yaw_center: 0.0
  - gimbal_max: 0.0
  - gimbal_min: 0.0
  - gimbal_pitch_mass: 0.0
  - radius_pitch_to_center_of_mass:
      x: 0.0
      y: 0.0
      z: 0.0
  - radius_yaw_to_pitch:
      x: 0.0
      y: 0.0
      z: 0.0
  - rotation_pitch_from_imu:
      m: [[1.0, 0.0, 0.0],
          [0.0, 1.0, 0.0],
          [0.0, 0.0, 1.0]]
  - gimbal_cmd_topic_name: 'gimbal_cmd'
  - accl_topic_name: 'bmi088_accl'
  - euler_topic_name: 'ahrs_euler'
  - gyro_topic_name: 'bmi088_gyro'
template_args:
  - MotorType: RMMotorContainer
required_hardware:
  - cmd
  - motor_can1
  - motor_can2
  - bmi088
depends:
  - cmd
  - motor_can1
  - motor_can2
  - pid
  - gimbal_cmd_topic_name: 'gimbal_cmd'
  - accl_topic_name: 'bmi088_accl'
  - euler_topic_name: 'ahrs_euler'
  - gyro_topic_name: 'bmi088_gyro'
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "BMI088.hpp"
#include "CMD.hpp"
#include "Eigen/Core"
#include "MadgwickAHRS.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "inertia.hpp"
#include "pid.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define GIMBAL_MAX_SPEED (M_2PI * 1.5f)

/**
 * @brief 云台控制应用类
 *
 * @tparam MotorType 电机驱动类型
 */
template <typename MotorType>
class Gimbal : public LibXR::Application {
 public:
  /**
   * @brief Gimbal 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
   * @param task_stack_depth 任务堆栈深度
   * @param pid_angle_param_yaw Yaw轴角度环PID参数
   * @param pid_angle_param_pitch Pitch轴角度环PID参数
   * @param pid_omega_param_yaw Yaw轴角速度环PID参数
   * @param pid_omega_param_pitch Pitch轴角速度环PID参数
   * @param motor_can1 CAN1上的电机实例 (Yaw)
   * @param motor_can2 CAN2上的电机实例 (Pitch)
   * @param gimbal_pitch_center_ Pitch轴电机中心(零点)位置
   * @param gimbal_yaw_center_ Yaw轴电机中心(零点)位置
   * @param gimbal_max_ Pitch轴最大角度(行程上限)
   * @param gimbal_min_ Pitch轴最小角度(行程下限)
   * @param gimbal_pitch_mass_ Pitch轴固连负载的质量 (kg)
   * @param radius_pitch_to_center_of_mass Pitch轴心到负载重心的向量 (m)
   * @param radius_yaw_to_pitch Yaw轴心到Pitch轴心的向量 (m)
   * @param rotation_pitch_from_imu IMU坐标系到Pitch坐标系的旋转矩阵
   * @param gimbal_cmd_topic_name 云台命令Topic名称
   * @param accl_topic_name 加速度计数据Topic名称
   * @param euler_topic_name 欧拉角数据Topic名称
   * @param gyro_topic_name 陀螺仪数据Topic名称
   */
  Gimbal(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
         uint32_t task_stack_depth,
         LibXR::PID<float>::Param pid_angle_param_yaw,
         LibXR::PID<float>::Param pid_angle_param_pitch,
         LibXR::PID<float>::Param pid_omega_param_yaw,
         LibXR::PID<float>::Param pid_omega_param_pitch,
         Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
         float gimbal_pitch_center_, float gimbal_yaw_center_,
         float gimbal_max_, float gimbal_min_, float gimbal_pitch_mass_,
         const LibXR::Position<float> &radius_pitch_to_center_of_mass,
         const LibXR::Position<float> &radius_yaw_to_pitch,
         const LibXR::RotationMatrix<float> &rotation_pitch_from_imu,
         const char *gimbal_cmd_topic_name, const char *accl_topic_name,
         const char *euler_topic_name, const char *gyro_topic_name)
      : gimbal_cmd_name_(gimbal_cmd_topic_name),
        accl_name_(accl_topic_name),
        euler_name_(euler_topic_name),
        gyro_name_(gyro_topic_name),
        gimbal_pitch_center_(gimbal_pitch_center_),
        gimbal_yaw_center_(gimbal_yaw_center_),
        gimbal_max_(gimbal_max_),
        gimbal_min_(gimbal_min_),
        gimbal_pitch_mass_(gimbal_pitch_mass_),
        radius_pitch_to_center_of_mass_(radius_pitch_to_center_of_mass),
        radius_yaw_to_pitch_(radius_yaw_to_pitch),
        rotation_pitch_from_imu_(rotation_pitch_from_imu),
        pid_angle_yaw_(pid_angle_param_yaw),
        pid_angle_pitch_(pid_angle_param_pitch),
        pid_omega_yaw_(pid_omega_param_yaw),
        pid_omega_pitch_(pid_omega_param_pitch),
        motor_can1_(motor_can1),
        motor_can2_(motor_can2),
        cmd_(cmd) {
    thread_.Create(this, ThreadFunction, "GimbalThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    Eigen::Matrix<float, 3, 1> r_pitch;
    r_pitch << radius_pitch_to_center_of_mass_.x(),
        radius_pitch_to_center_of_mass_.y(),
        radius_pitch_to_center_of_mass_.z();
    const float m = gimbal_pitch_mass_;
    const float r2 = r_pitch.squaredNorm();
    Eigen::Matrix<float, 3, 3> i_com_pitch_ =
        m * (r2 * Eigen::Matrix<float, 3, 3>::Identity() -
             r_pitch * r_pitch.transpose());

    inertia_pitch_ = LibXR::Inertia<float>(m, i_com_pitch_);
  }

  /**
   * @brief 云台控制线程函数，为静态成员函数
   *
   * @param gimbal 指向Gimbal实例的指针
   */
  static void ThreadFunction(Gimbal *gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(
        gimbal->gimbal_cmd_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> accl_suber(
        gimbal->accl_name_);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->euler_name_);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        gimbal->gyro_name_);

    auto now_ = LibXR::Timebase::GetMicroseconds();
    gimbal->dt_ = (now_ - gimbal->last_online_time_);
    gimbal->last_online_time_ = now_;
    cmd_suber.StartWaiting();

    while (true) {
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (accl_suber.Available()) {
        gimbal->accl_data_ = accl_suber.GetData();
        accl_suber.StartWaiting();
      }

      if (euler_suber.Available()) {
        gimbal->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gyro_suber.StartWaiting();
      }
      gimbal->semaphore_.Wait(UINT32_MAX);
      gimbal->Update();
      gimbal->UpdateSetpointFromCMD();
      gimbal->SelfResolution();
      gimbal->GravityCompensation(gimbal->accl_data_);
      gimbal->semaphore_.Post();
      gimbal->OutputToDynamics();
    }
  }

  /**
   * @brief 根据外部命令(如遥控器)更新云台的目标设定点
   *
   */
  void UpdateSetpointFromCMD() {
    float gimbal_yaw_cmd = cmd_data_.yaw * this->dt_ * GIMBAL_MAX_SPEED;
    float gimbal_pitch_cmd = cmd_data_.pit * this->dt_ * GIMBAL_MAX_SPEED;

    const float pitch_err_ = target_angle_pitch_ - euler_.Pitch();
    const float ENCODER_DELTA_MAX_PIT = gimbal_max_ - motor_can2_.GetAngle(5);
    const float ENCODER_DELTA_MIN_PIT = gimbal_min_ - motor_can2_.GetAngle(5);
    const float DELTA_MAX_PIT = ENCODER_DELTA_MAX_PIT - pitch_err_;
    const float DELTA_MIN_PIT = ENCODER_DELTA_MIN_PIT - pitch_err_;
    gimbal_yaw_cmd = std::clamp(gimbal_yaw_cmd, DELTA_MIN_PIT, DELTA_MAX_PIT);

    target_angle_yaw_ += gimbal_yaw_cmd;
    target_angle_pitch_ += gimbal_pitch_cmd;
  }

  /**
   * @brief 更新电机状态，通常在主循环中被调用
   *
   */
  void Update() {
    motor_can1_.Update(6);
    motor_can2_.Update(5);
  }

  /**
   * @brief 自我解算函数，获取云台当前的角度和角速度
   *
   */
  void SelfResolution() {
    prev_omega_pitch_ = now_omega_pitch_;
    prev_omega_yaw_ = now_omega_yaw_;
    now_angle_pitch_ = motor_can2_.GetAngle(5);
    now_angle_yaw_ = motor_can1_.GetAngle(6);
    now_omega_pitch_ = motor_can2_.GetSpeed(5);
    now_omega_yaw_ = motor_can1_.GetSpeed(6);
  }

  /**
   * @brief 根据IMU加速度数据计算重力补偿力矩
   * @details
   * 计算由重力和外部加速度在云台 Pitch 与 Yaw 轴上产生的前馈补偿力矩，
   * 并把惯性项（I·α + ω × (I·ω)）加入补偿，以改善动态响应。
   *
   * 核心思想:
   * 1) 坐标变换：IMU -> Pitch -> Yaw
   * 2) 力矩计算：τ = r × (-m·a)，取轴向分量
   * 3) 惯性项：I·α + ω × (I·ω)
   * 4) 合成：torque = tau_g + tau_i
   *
   * @param accl_data_ 从IMU获取的加速度数据 (m/s^2)，在IMU坐标系下表示。
   */
  void GravityCompensation(Eigen::Matrix<float, 3, 1> &accl_data_) {
    // 构造 Pitch->Yaw 旋转矩阵
    const float cos_p = std::cos(now_angle_pitch_);
    const float sin_p = std::sin(now_angle_pitch_);
    Eigen::Matrix<float, 3, 3> R_yaw_from_pitch;
    R_yaw_from_pitch << cos_p, 0.0f, sin_p, 0.0f, 1.0f, 0.0f, -sin_p, 0.0f,
        cos_p;

    // 坐标变换：IMU -> Pitch -> Yaw
    Eigen::Matrix<float, 3, 1> accel_in_pitch =
        rotation_pitch_from_imu_ * accl_data_;
    Eigen::Matrix<float, 3, 1> accel_in_yaw = R_yaw_from_pitch * accel_in_pitch;

    // 质心向量
    Eigen::Matrix<float, 3, 1> r_pitch;
    r_pitch << radius_pitch_to_center_of_mass_.x(),
        radius_pitch_to_center_of_mass_.y(),
        radius_pitch_to_center_of_mass_.z();

    // 重力力矩：tau_g = r × (-m·a)
    float tau_g_pitch =
        (r_pitch.cross(-gimbal_pitch_mass_ * accel_in_pitch))(1);

    Eigen::Matrix<float, 3, 1> r_yaw_to_pitch;
    r_yaw_to_pitch << radius_yaw_to_pitch_.x(), radius_yaw_to_pitch_.y(),
        radius_yaw_to_pitch_.z();
    Eigen::Matrix<float, 3, 1> r_yaw_to_center =
        r_yaw_to_pitch + R_yaw_from_pitch * r_pitch;
    float tau_g_yaw =
        (r_yaw_to_center.cross(-gimbal_pitch_mass_ * accel_in_yaw))(2);

    // 惯性项：I·α + ω × (I·ω)
    float dt_seconds = static_cast<float>(dt_ * 1e-6f);
    float alpha_pitch =
        (dt_seconds > 0.0f)
            ? (now_omega_pitch_ - prev_omega_pitch_) / dt_seconds
            : 0.0f;
    float alpha_yaw = (dt_seconds > 0.0f)
                          ? (now_omega_yaw_ - prev_omega_yaw_) / dt_seconds
                          : 0.0f;

    // Pitch 惯性项
    auto i_pitch_about_ = inertia_pitch_.Translate(r_pitch);
    Eigen::Matrix<float, 3, 3> i_pitch_mat_ =
        static_cast<Eigen::Matrix<float, 3, 3>>(i_pitch_about_);
    Eigen::Matrix<float, 3, 1> omega_pitch_vec(0.0f, now_omega_pitch_, 0.0f);
    float tau_i_pitch =
        (i_pitch_mat_ * Eigen::Matrix<float, 3, 1>(0.0f, alpha_pitch, 0.0f) +
         omega_pitch_vec.cross(i_pitch_mat_ * omega_pitch_vec))(1);

    // Yaw 惯性项
    auto i_yaw_about_ =
        inertia_pitch_.Rotate(R_yaw_from_pitch).Translate(r_yaw_to_center);
    Eigen::Matrix<float, 3, 3> i_yaw_mat_ =
        static_cast<Eigen::Matrix<float, 3, 3>>(i_yaw_about_);
    Eigen::Matrix<float, 3, 1> omega_yaw_vec(0.0f, 0.0f, now_omega_yaw_);
    float tau_i_yaw =
        (i_yaw_mat_ * Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, alpha_yaw) +
         omega_yaw_vec.cross(i_yaw_mat_ * omega_yaw_vec))(2);

    // 合成输出
    pitch_torque_ = tau_g_pitch + tau_i_pitch;
    yaw_torque_ = tau_g_yaw + tau_i_yaw;
  }

  /**
   * @brief 将一个值限制在给定的最大值和最小值之间
   *
   * @param x 要限制的值的指针
   * @param Min 最小值
   * @param Max 最大值
   * @return float 限制后的值
   */
  float Constrain(float *x, float Min, float Max) {
    if (*x < Min) {
      *x = Min;
    } else if (*x > Max) {
      *x = Max;
    }
    return (*x);
  }

  /**
   * @brief 处理电机角度的周期性，实现最近转置，避免电机在边界处反转
   *
   */
  void MotorNearestTransposition() {
    float tmp_delta_angle_ = 0.0f;
    tmp_delta_angle_ = fmod(target_angle_yaw_ - now_angle_yaw_, 2.0f * M_PI);
    if (tmp_delta_angle_ > M_PI) {
      tmp_delta_angle_ -= 2.0f * M_PI;
    } else if (tmp_delta_angle_ < -M_PI) {
      tmp_delta_angle_ += 2.0f * M_PI;
    }
    target_angle_yaw_ = motor_can1_.GetAngle(6) + tmp_delta_angle_;

    Constrain(&target_angle_pitch_, gimbal_min_, gimbal_max_);
    tmp_delta_angle_ = target_angle_pitch_ - now_angle_pitch_;
    target_angle_pitch_ = -motor_can2_.GetAngle(5) + tmp_delta_angle_;
  }

  /**
   * @brief 计算PID输出并将其应用到电机，实现动力学控制
   *
   */
  void OutputToDynamics() {
    MotorNearestTransposition();

    // 欧拉角 -> Pitch系下的姿态角
    const auto [cy, sy] =
        std::make_pair(std::cos(euler_.Yaw()), std::sin(euler_.Yaw()));
    const auto [cp, sp] =
        std::make_pair(std::cos(euler_.Pitch()), std::sin(euler_.Pitch()));
    const auto [cr, sr] =
        std::make_pair(std::cos(euler_.Roll()), std::sin(euler_.Roll()));

    const Eigen::Matrix3f R_current =
        rotation_pitch_from_imu_.transpose() *
        (Eigen::Matrix3f() << cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr)
            .finished();

    const float current_pitch = std::asin(-R_current(2, 0));
    const float current_yaw = std::atan2(R_current(1, 0), R_current(0, 0));

    LibXR::STDIO::Printf("原始欧拉角(deg) - Yaw:%.2f Pitch:%.2f Roll:%.2f\n",
                         euler_.Yaw(), euler_.Pitch(), euler_.Roll());
    LibXR::STDIO::Printf("Pitch系欧拉角(deg) - Yaw:%.2f Pitch:%.2f\n",
                         current_yaw, current_pitch);

    // PID + 输出
    const float output_yaw = std::clamp(
        pid_omega_yaw_.Calculate(
            pid_angle_yaw_.Calculate(target_angle_yaw_, current_yaw, dt_),
            gyro_data_.z(), dt_) +
            yaw_torque_,
        -max_current_, max_current_);

    const float output_pitch = std::clamp(
        pid_omega_pitch_.Calculate(
            pid_angle_pitch_.Calculate(target_angle_pitch_, current_pitch, dt_),
            gyro_data_.x(), dt_) +
            pitch_torque_,
        -max_current_, max_current_);

    motor_can1_.SetCurrent(6, output_yaw);
    motor_can2_.SetCurrent(5, output_pitch);
  }

  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

 private:
  //! 云台命令Topic名称
  const char *gimbal_cmd_name_;
  //! 加速度计数据Topic名称
  const char *accl_name_;
  //! 欧拉角数据Topic名称
  const char *euler_name_;
  //! 陀螺仪数据Topic名称
  const char *gyro_name_;

  //! 当前Yaw轴角度
  float now_angle_yaw_ = 0.0f;
  //! 当前Pitch轴角度
  float now_angle_pitch_ = 0.0f;
  //! 目标Yaw轴角度
  float target_angle_yaw_ = 0.0f;
  //! 目标Pitch轴角度
  float target_angle_pitch_ = 0.0f;
  //! 当前Yaw轴角速度
  float now_omega_yaw_ = 0.0f;
  //! 当前Pitch轴角速度
  float now_omega_pitch_ = 0.0f;
  //! Pitch轴补偿力矩
  float pitch_torque_ = 0.0f;
  //! Yaw轴补偿力矩
  float yaw_torque_ = 0.0f;
  //! Pitch轴电机中心(零点)位置
  float gimbal_pitch_center_ = 0.0f;
  //! Yaw轴电机中心(零点)位置
  float gimbal_yaw_center_ = 0.0f;
  //! Pitch轴最大角度(行程上限)
  float gimbal_max_ = 0.0f;
  //! Pitch轴最小角度(行程下限)
  float gimbal_min_ = 0.0f;
  //! 上次Pitch轴角速度
  float prev_omega_pitch_ = 0.0f;
  //! 上次Yaw轴角速度
  float prev_omega_yaw_ = 0.0f;

  //! Pitch轴固连负载的质量 (kg)
  const float gimbal_pitch_mass_ = 0.0f;
  //! P系下, Pitch轴心->负载重心 的向量 (m)
  const LibXR::Position<float> radius_pitch_to_center_of_mass_{0.0f, 0.0f,
                                                               0.0f};
  //! Y系下, Yaw轴心->Pitch轴心 的向量 (m)
  const LibXR::Position<float> radius_yaw_to_pitch_{0.0f, 0.0f, 0.0f};
  //! IMU坐标系到Pitch坐标系的旋转矩阵
  const Eigen::Matrix<float, 3, 3> rotation_pitch_from_imu_{
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  //! IMU加速度数据
  const LibXR::Position<float> imu_acceleration_{0.0f, 0.0f, 0.0f};
  //! 质心处惯性对象（Pitch系）
  LibXR::Inertia<float> inertia_pitch_;

  //! 电机最大输出电流
  const float max_current_ = 1.0f;

  //! Yaw轴角度环PID控制器
  LibXR::PID<float> pid_angle_yaw_;
  //! Pitch轴角度环PID控制器
  LibXR::PID<float> pid_angle_pitch_;
  //! Yaw轴角速度环PID控制器
  LibXR::PID<float> pid_omega_yaw_;
  //! Pitch轴角速度环PID控制器
  LibXR::PID<float> pid_omega_pitch_;

  //! CAN1上的电机实例 (Yaw)
  Motor<MotorType> &motor_can1_;
  //! CAN2上的电机实例 (Pitch)
  Motor<MotorType> &motor_can2_;

  //! 命令模块实例
  CMD &cmd_;
  //! 云台命令数据
  CMD::GimbalCMD cmd_data_;

  //! 云台控制线程
  LibXR::Thread thread_;

  //! 时间间隔 (s)
  LibXR::MicrosecondTimestamp::Duration dt_ = 0;
  //! 上次在线时间
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  //! 当前时间
  LibXR::MicrosecondTimestamp now_ = 0;

  //! 陀螺仪和加速度计数据
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  //! 欧拉角数据
  LibXR::EulerAngle<float> euler_;

  LibXR::Semaphore semaphore_;
};
