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
#include "pid.hpp"
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

      gimbal->UpdateSetpointFromCMD();
      gimbal->SelfResolution();
      gimbal->GravityCompensation(gimbal->accl_data_);
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
    now_angle_pitch_ = motor_can2_.GetAngle(5);
    now_angle_yaw_ = motor_can1_.GetAngle(6);
    now_omega_pitch_ = motor_can2_.GetSpeed(5);
    now_omega_yaw_ = motor_can1_.GetSpeed(6);
  }

  /**
   * @brief 根据IMU加速度数据计算重力补偿力矩
   * @details
   * 该函数的目标是计算由重力（以及其他外部加速度）在云台Pitch轴和Yaw轴上产生的扰动力矩。
   * 通过将这些力矩前馈到PID控制器，可以主动抵消重力的影响，使云台在倾斜时也能保持稳定。
   *
   * 核心思想:
   * 1.
   * 将IMU测量到的加速度（在静止或匀速运动时，主要分量为重力加速度）转换到各个云台部件的坐标系下。
   * 2. 根据牛顿第二定律 F = ma，计算出作用在负载（如相机）质心上的惯性力
   * F_disturbance = -m * a_imu。
   * 3. 根据力矩公式 τ = r × F，计算该惯性力相对于Pitch轴和Yaw轴产生的力矩。
   *
   * 坐标系定义:
   * - IMU系 (I): IMU传感器自身的坐标系。
   * - Pitch系 (P): 与Pitch轴固连的坐标系，随Pitch轴运动。
   * - Yaw系 (Y): 与Yaw轴固连的坐标系，随Yaw轴运动。
   *
   * @param accl_data_ 从IMU获取的加速度数据 (m/s^2)，在IMU坐标系下表示。
   */
  void GravityCompensation(Eigen::Matrix<float, 3, 1> &accl_data_) {
    // --- 1. 计算坐标系之间的旋转矩阵 ---
    // 计算从Pitch坐标系(P)到Yaw坐标系(Y)的旋转矩阵。
    // 这取决于当前Pitch轴的电机角度 now_angle_pitch_。
    const float cos_p = std::cos(now_angle_pitch_);
    const float sin_p = std::sin(now_angle_pitch_);
    LibXR::RotationMatrix<float> rotation_yaw_from_pitch(cos_p, 0, sin_p, 0, 1,
                                                         0, -sin_p, 0, cos_p);

    // --- 2. 将加速度矢量变换到目标坐标系 ---
    // 将IMU加速度从IMU坐标系(I)转换到Pitch坐标系(P)。
    // rotation_pitch_from_imu_
    // 是一个配置参数，代表了IMU相对于Pitch组件的安装姿态。
    Eigen::Matrix<float, 3, 1> acceleration_in_pitch =
        rotation_pitch_from_imu_ * accl_data_;

    // 将加速度从Pitch坐标系(P)转换到Yaw坐标系(Y)。
    Eigen::Matrix<float, 3, 1> acceleration_in_yaw =
        rotation_yaw_from_pitch * acceleration_in_pitch;

    // --- 3. 计算扰动力 ---
    // 扰动力 F_disturbance = -m * a_imu。
    // 分别计算在Pitch系和Yaw系下表示的扰动力。
    LibXR::Position<float> force_in_pitch =
        -gimbal_pitch_mass_ * acceleration_in_pitch;
    LibXR::Position<float> force_in_yaw =
        -gimbal_pitch_mass_ * acceleration_in_yaw;

    // --- 4. 计算扰动力矩 ---
    // 在Pitch坐标系(P)下，计算作用在Pitch轴上的扰动力矩。
    // 力臂 r 是从Pitch轴心到负载重心的向量 (radius_pitch_to_center_of_mass_)。
    // 力矩 τ = r × F。
    LibXR::Position<float> torque_vector_pitch =
        radius_pitch_to_center_of_mass_.cross(force_in_pitch);

    // 在Yaw坐标系(Y)下，计算作用在Yaw轴上的扰动力矩。
    // 首先，计算总的力臂 r：从Yaw轴心到负载重心的向量。
    // 这等于 "Yaw轴心到Pitch轴心的向量" +
    // "Pitch轴心到负载重心的向量（变换到Yaw系下）"。
    LibXR::Position<float> radius_pitch_to_center_of_mass_in_yaw =
        rotation_yaw_from_pitch * radius_pitch_to_center_of_mass_;
    LibXR::Position<float> radius_yaw_to_center_of_mass =
        radius_yaw_to_pitch_ + radius_pitch_to_center_of_mass_in_yaw;

    // 计算Yaw轴的扰动力矩 τ = r × F。
    LibXR::Position<float> torque_vector_yaw =
        radius_yaw_to_center_of_mass.cross(force_in_yaw);

    // --- 5. 提取有效力矩分量 ---
    // Pitch电机绕其自身的Y轴旋转，因此我们只关心力矩向量在Y轴上的分量。
    pitch_torque_ = torque_vector_pitch.y();
    // Yaw电机绕其自身的Z轴旋转，因此我们只关心力矩向量在Z轴上的分量。
    yaw_torque_ = torque_vector_yaw.z();
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

    float target_omega_yaw =
        pid_angle_yaw_.Calculate(target_angle_yaw_, euler_.Pitch(), dt_);
    float target_omega_pitch =
        pid_angle_pitch_.Calculate(target_angle_pitch_, euler_.Yaw(), dt_);

    float feedback_yaw =
        pid_omega_yaw_.Calculate(target_omega_yaw, gyro_data_.z(), dt_);
    float feedback_pitch =
        pid_omega_pitch_.Calculate(target_omega_pitch, gyro_data_.x(), dt_);

    float output_yaw = feedback_yaw + yaw_torque_;
    float output_pitch = feedback_pitch + pitch_torque_;

    output_yaw = std::clamp(output_yaw, -max_current_, max_current_);
    output_pitch = std::clamp(output_pitch, -max_current_, max_current_);

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

  //! Pitch轴固连负载的质量 (kg)
  const float gimbal_pitch_mass_ = 0.0f;
  //! P系下, Pitch轴心->负载重心 的向量 (m)
  const LibXR::Position<float> radius_pitch_to_center_of_mass_{0.0f, 0.0f,
                                                               0.0f};
  //! Y系下, Yaw轴心->Pitch轴心 的向量 (m)
  const LibXR::Position<float> radius_yaw_to_pitch_{0.0f, 0.0f, 0.0f};
  //! IMU坐标系到Pitch坐标系的旋转矩阵
  const LibXR::RotationMatrix<float> rotation_pitch_from_imu_{
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  //! IMU加速度数据
  const LibXR::Position<float> imu_acceleration_{0.0f, 0.0f, 0.0f};

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
};
