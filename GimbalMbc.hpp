#pragma once

#include <algorithm>
#include <cmath>

/**
 * @brief 云台模型补偿控制器：刚体模型补偿、状态反馈与扩张状态观测器（ESO）。
 *        Model-based gimbal controller: rigid-body model compensation, state feedback
 *        and an extended state observer (ESO).
 *
 * 角度为世界系 yaw（psi）与 pitch 欧拉角（theta），力矩以使角度增大为正。模型：
 * Angles are the world yaw (psi) and the pitch Euler angle (theta); a positive torque
 * increases the angle. Model:
 *
 *   J_pit theta'' = tau_pit + 1/2 J_yaw'(theta) psi'^2 + k_x psi'' + G(theta) - F_pit
 *   J_yaw(theta) psi'' = tau_yaw - J_yaw'(theta) theta' psi' - F_yaw
 *   J_yaw(theta) = j_yaw0 + j_yaw_c2 cos 2theta + j_yaw_s2 sin 2theta
 *
 * 摩擦作用在关节上。底盘运动在两轴上的角速度
 * b = 世界系角速度 - 关节角速度（经低通），
 * 摩擦补偿按关节角速度参考 w_d - b 计算，
 * 底盘倾斜或转动时关节为保持世界系姿态必须转动的方向也算在内。
 * Friction acts on the joints. The base motion seen on each axis, b = world rate - joint
 * rate (low-passed), makes the friction compensation use the joint rate reference
 * w_d - b, so the joint motion needed to hold the world attitude on a tilted or turning
 * base is included.
 *
 * k_x 是 yaw 角加速度对 pitch 的反作用，来自惯性积，主轴模型里没有，按实测给出；
 * pit_yaw_acc 取 -k_x。
 * k_x is the reaction of the yaw acceleration on the pitch, from a product of inertia
 * that the principal-axis model lacks, set from measurements; pit_yaw_acc is -k_x.
 *
 * 补偿后每轴近似双积分。状态反馈给出加速度指令，ESO 估计剩余的加速度（模型误差与未建模
 * 力矩）并抵消，常值扰动不留稳态误差：
 * After compensation each axis is close to a double integrator. State feedback gives the
 * acceleration command; the ESO estimates the remaining acceleration (model error and
 * unmodelled torque) and cancels it, so constant disturbances leave no steady error:
 *
 *   w_d = clamp(kp / kd * e, +-w_max) + ref_dot
 *   a   = ref_ddot + kd * (w_d - w) - z2
 *   tau = J * a + coupling + gravity + friction
 *   z1 += dt * (a_prev + z2 + 2 wo (w - z1)),  z2 += dt * wo^2 * (w - z1)
 *
 * a_prev 是上一拍限幅后实际施加的加速度。yaw 角速度取 gyro_z / cos(theta)（IMU 在 pitch
 * 架上）。不限幅时 a = ref_ddot + kp e + kd (ref_dot - w) - z2。
 * a_prev is the acceleration actually applied last tick, after the torque clamp. The yaw
 * rate is gyro_z / cos(theta) (IMU on the pitch frame). Without the clamp,
 * a = ref_ddot + kp e + kd (ref_dot - w) - z2.
 */
class GimbalMbc
{
 public:
  /**
   * @brief 控制器参数。
   *        Controller parameters.
   */
  struct Param
  {
    bool enable_pit;  ///< 上电时 pitch 用本控制器，否则用 PID
                      ///< Pitch uses this controller from power-up, otherwise the PID
    bool enable_yaw;  ///< 上电时 yaw 用本控制器，否则用 PID
                      ///< Yaw uses this controller from power-up, otherwise the PID
    float j_pit;     ///< pitch 转动惯量 (kg·m^2) / Pitch moment of inertia (kg·m^2)
    float j_yaw0;    ///< J_yaw 常数项 (kg·m^2) / Constant term of J_yaw (kg·m^2)
    float j_yaw_c2;  ///< J_yaw 的 cos 2theta 系数 / cos 2theta coefficient of J_yaw
    float j_yaw_s2;  ///< J_yaw 的 sin 2theta 系数 / sin 2theta coefficient of J_yaw
    float pit_kp;    ///< pitch 角度增益 (1/s^2) / Pitch angle gain (1/s^2)
    float pit_kd;    ///< pitch 角速度增益 (1/s) / Pitch rate gain (1/s)
    float pit_wo;    ///< pitch ESO 带宽 (rad/s) / Pitch ESO bandwidth (rad/s)
    float pit_wmax;  ///< pitch 由角度误差给出的最大角速度 (rad/s)
                     ///< Largest pitch rate from the angle error (rad/s)
    float yaw_kp;    ///< yaw 角度增益 (1/s^2) / Yaw angle gain (1/s^2)
    float yaw_kd;    ///< yaw 角速度增益 (1/s) / Yaw rate gain (1/s)
    float yaw_wo;    ///< yaw ESO 带宽 (rad/s) / Yaw ESO bandwidth (rad/s)
    float yaw_wmax;  ///< yaw 由角度误差给出的最大角速度 (rad/s)
                     ///< Largest yaw rate from the angle error (rad/s)
    float pit_b;     ///< pitch 粘滞摩擦 (N·m·s/rad) / Pitch viscous friction (N·m·s/rad)
    float pit_tc;    ///< pitch 库仑摩擦 (N·m) / Pitch Coulomb friction (N·m)
    float yaw_b;     ///< yaw 粘滞摩擦 (N·m·s/rad) / Yaw viscous friction (N·m·s/rad)
    float yaw_tc;    ///< yaw 库仑摩擦 (N·m) / Yaw Coulomb friction (N·m)
    float fric_eps;  ///< 库仑摩擦 tanh 平滑的角速度尺度 (rad/s)
                     ///< Rate scale of the tanh smoothing of Coulomb friction (rad/s)
    float pit_torque_max;  ///< pitch 力矩限幅 (N·m) / Pitch torque limit (N·m)
    float yaw_torque_max;  ///< yaw 力矩限幅 (N·m) / Yaw torque limit (N·m)
    float pit_yaw_acc;     ///< pitch 前馈中按 yaw 角加速度加的力矩 (N·m·s^2/rad)
                        ///< Pitch feed-forward torque per yaw acceleration (N·m·s^2/rad)
    float base_tau;  ///< 底盘角速度低通时间常数 (s)，0 时摩擦按世界系角速度
                     ///< Low-pass time constant of the base rate (s); 0 puts the friction
                     ///< on the world rate
    bool coupling;   ///< 补偿两轴的离心与科氏项 / Compensate the centrifugal and Coriolis
                     ///< terms
  };

  /**
   * @brief 目标角速度与角加速度。
   *        Target rates and accelerations.
   */
  struct Reference
  {
    float pit_dot;
    float pit_ddot;
    float yaw_dot;
    float yaw_ddot;
  };

  /**
   * @brief 一拍的力矩分项，供日志使用。
   *        Torque terms of one tick, for the log.
   */
  struct Terms
  {
    float ff_pit;   ///< J * ref_ddot + 耦合 + 摩擦 / J * ref_ddot + coupling + friction
    float fb_pit;   ///< J * (kd * (w_d - w)) 状态反馈 / State feedback
    float eso_pit;  ///< -J * z2，ESO 补偿 / ESO compensation
    float ff_yaw;
    float fb_yaw;
    float eso_yaw;
  };

  /// ESO 带宽上限 (rad/s)：dt 取到 MAX_DT 时 wo * dt 仍不超过 3
  /// Upper bound of the ESO bandwidth (rad/s): wo * dt stays within 3 at dt = MAX_DT
  static constexpr float MAX_WO = 300.0f;

  /**
   * @brief 参数能否使用：惯量在整个 pitch 范围为正，增益、限幅与平滑尺度在有效范围内。
   *        Whether the parameters are usable: the inertias are positive over the whole
   *        pitch range and the gains, limits and smoothing scale are in range.
   */
  static bool Valid(const Param& p)
  {
    const auto finite = [](float v) { return std::isfinite(v); };
    return finite(p.j_pit) && finite(p.j_yaw0) && finite(p.j_yaw_c2) &&
           finite(p.j_yaw_s2) && p.j_pit > 0.0f &&
           p.j_yaw0 > std::hypot(p.j_yaw_c2, p.j_yaw_s2) && p.pit_kp >= 0.0f &&
           p.pit_kd > 0.0f && p.yaw_kp >= 0.0f && p.yaw_kd > 0.0f && p.pit_wo >= 0.0f &&
           p.pit_wo <= MAX_WO && p.yaw_wo >= 0.0f && p.yaw_wo <= MAX_WO &&
           p.pit_wmax > 0.0f && p.yaw_wmax > 0.0f && p.pit_b >= 0.0f &&
           p.pit_tc >= 0.0f && p.yaw_b >= 0.0f && p.yaw_tc >= 0.0f && p.fric_eps > 0.0f &&
           p.pit_torque_max > 0.0f && p.yaw_torque_max > 0.0f && finite(p.pit_kp) &&
           finite(p.pit_kd) && finite(p.yaw_kp) && finite(p.yaw_kd) &&
           finite(p.pit_wmax) && finite(p.yaw_wmax) && finite(p.pit_b) &&
           finite(p.pit_tc) && finite(p.yaw_b) && finite(p.yaw_tc) &&
           finite(p.fric_eps) && finite(p.pit_torque_max) && finite(p.yaw_torque_max) &&
           finite(p.pit_yaw_acc) && finite(p.base_tau) && p.base_tau >= 0.0f;
  }

  explicit GimbalMbc(const Param& param) : param_(param) {}

  const Param& GetParam() const { return param_; }

  /**
   * @brief 更换参数；观测器状态保留。
   *        Replace the parameters; the observer state is kept.
   */
  void SetParam(const Param& param) { param_ = param; }

  /**
   * @brief 观测器从当前角速度重新开始，扰动估计清零。
   *        Restart the observers from the current rates with zero disturbance estimates.
   *
   * @param pit pitch 欧拉角 (rad)。 / Pitch Euler angle (rad).
   * @param gyro_pit pitch 角速度 (rad/s)。 / Pitch rate (rad/s).
   * @param gyro_yaw 机体 z 轴角速度 (rad/s)。 / Body z rate (rad/s).
   */
  void Reset(float pit, float gyro_pit, float gyro_yaw)
  {
    pit_z1_ = gyro_pit;
    pit_z2_ = 0.0f;
    yaw_z1_ = YawRate(pit, gyro_yaw);
    yaw_z2_ = 0.0f;
    pit_a_ = 0.0f;
    yaw_a_ = 0.0f;
    pit_base_ = 0.0f;
    yaw_base_ = 0.0f;
    joint_valid_ = false;
  }

  /**
   * @brief 计算一拍的两轴力矩。
   *        Compute the torques of both axes for one tick.
   *
   * @param dt 距上一拍的时间 (s)。 / Time since the last tick (s).
   * @param pit_err pitch 目标减当前角度 (rad)。 / Pitch target minus angle (rad).
   * @param yaw_err yaw 目标减当前角度，已折算到 [-pi, pi) (rad)。
   *                Yaw target minus angle, wrapped to [-pi, pi) (rad).
   * @param pit pitch 欧拉角 (rad)。 / Pitch Euler angle (rad).
   * @param gyro_pit pitch 角速度 (rad/s)。 / Pitch rate (rad/s).
   * @param gyro_yaw 机体 z 轴角速度 (rad/s)。 / Body z rate (rad/s).
   * @param pit_joint pitch 关节角，与欧拉角同向 (rad)。
   *                  Pitch joint angle, same direction as the Euler angle (rad).
   * @param yaw_joint_rate yaw 关节角速度，与 yaw 同向 (rad/s)。
   *                       Yaw joint rate, same direction as the yaw (rad/s).
   * @param ref 目标角速度与角加速度。 / Target rates and accelerations.
   * @param gravity 重力补偿力矩 (N·m)。 / Gravity compensation torque (N·m).
   * @param out_pit pitch 力矩 (N·m)，已限幅。 / Pitch torque (N·m), clamped.
   * @param out_yaw yaw 力矩 (N·m)，已限幅。 / Yaw torque (N·m), clamped.
   */
  void Step(float dt, float pit_err, float yaw_err, float pit, float gyro_pit,
            float gyro_yaw, float pit_joint, float yaw_joint_rate, const Reference& ref,
            float gravity, float& out_pit, float& out_yaw)
  {
    dt = std::clamp(dt, MIN_DT, MAX_DT);
    const float wp = gyro_pit;
    const float wy = YawRate(pit, gyro_yaw);
    const float s2 = sinf(2.0f * pit);
    const float c2 = cosf(2.0f * pit);
    const float jy = param_.j_yaw0 + param_.j_yaw_c2 * c2 + param_.j_yaw_s2 * s2;
    const float djy = -2.0f * param_.j_yaw_c2 * s2 + 2.0f * param_.j_yaw_s2 * c2;

    Observe(pit_z1_, pit_z2_, pit_a_, wp, param_.pit_wo, dt);
    Observe(yaw_z1_, yaw_z2_, yaw_a_, wy, param_.yaw_wo, dt);
    TrackBase(dt, wp, wy, pit_joint, yaw_joint_rate);

    // yaw
    float wd = std::clamp(param_.yaw_kp / param_.yaw_kd * yaw_err, -param_.yaw_wmax,
                          param_.yaw_wmax) +
               ref.yaw_dot;
    float fb = param_.yaw_kd * (wd - wy);
    float extra = Friction(wd - yaw_base_, param_.yaw_b, param_.yaw_tc);
    if (param_.coupling)
    {
      extra += djy * wp * wy;
    }
    float raw = jy * (ref.yaw_ddot + fb - yaw_z2_) + extra;
    out_yaw = std::clamp(raw, -param_.yaw_torque_max, param_.yaw_torque_max);
    yaw_a_ = (out_yaw - extra) / jy;
    yaw_extra_ = extra;
    jy_ = jy;
    terms_.ff_yaw = jy * ref.yaw_ddot + extra;
    terms_.fb_yaw = jy * fb;
    terms_.eso_yaw = -jy * yaw_z2_;

    // pitch：交叉项用本拍 yaw 实际施加的角加速度
    // pitch: the cross term uses the yaw acceleration applied this tick
    wd = std::clamp(param_.pit_kp / param_.pit_kd * pit_err, -param_.pit_wmax,
                    param_.pit_wmax) +
         ref.pit_dot;
    fb = param_.pit_kd * (wd - wp);
    extra = gravity + Friction(wd - pit_base_, param_.pit_b, param_.pit_tc);
    if (param_.coupling)
    {
      extra -= 0.5f * djy * wy * wy;
    }
    extra += param_.pit_yaw_acc * yaw_a_;
    raw = param_.j_pit * (ref.pit_ddot + fb - pit_z2_) + extra;
    out_pit = std::clamp(raw, -param_.pit_torque_max, param_.pit_torque_max);
    pit_a_ = (out_pit - extra) / param_.j_pit;
    pit_extra_ = extra;
    terms_.ff_pit = param_.j_pit * ref.pit_ddot + extra - gravity;
    terms_.fb_pit = param_.j_pit * fb;
    terms_.eso_pit = -param_.j_pit * pit_z2_;
  }

  /**
   * @brief 告知本拍实际下发的力矩：某轴改用 PID 时，观测器按实际力矩继续跟踪，
   *        之后切回本控制器不需要重新收敛。
   *        Report the torques actually sent this tick: on an axis run by the PID the
   *        observer keeps tracking with the real torque, so switching back needs no
   *        new convergence.
   *
   * @param out_pit 实际 pitch 力矩 (N·m)。 / Pitch torque sent (N·m).
   * @param out_yaw 实际 yaw 力矩 (N·m)。 / Yaw torque sent (N·m).
   */
  void Applied(float out_pit, float out_yaw)
  {
    out_pit = std::clamp(out_pit, -param_.pit_torque_max, param_.pit_torque_max);
    out_yaw = std::clamp(out_yaw, -param_.yaw_torque_max, param_.yaw_torque_max);
    pit_a_ = (out_pit - pit_extra_) / param_.j_pit;
    yaw_a_ = (out_yaw - yaw_extra_) / jy_;
  }

  const Terms& GetTerms() const { return terms_; }

 private:
  /// dt 的取值范围 (s)：调度抖动之外的异常间隔不让观测器发散
  /// Range of dt (s): abnormal intervals beyond scheduling jitter must not make the
  /// observers diverge
  static constexpr float MIN_DT = 0.0005f;
  static constexpr float MAX_DT = 0.01f;
  /// yaw 角速度换算时 cos(theta) 的下限 / Lower bound of cos(theta) in the yaw rate
  static constexpr float MIN_COS = 0.2f;
  static constexpr float TWO_PI = 6.28318530717958647692f;

  static float YawRate(float pit, float gyro_yaw)
  {
    return gyro_yaw / std::max(cosf(pit), MIN_COS);
  }

  float Friction(float w, float b, float tc) const
  {
    return b * w + tc * tanhf(w / param_.fric_eps);
  }

  /// 底盘在两轴上的角速度：世界系角速度减关节角速度，一阶低通；pitch
  /// 关节角速度由编码器差分 Base rate on both axes: world rate minus joint rate,
  /// first-order low-pass; the pitch joint rate is the encoder difference
  void TrackBase(float dt, float wp, float wy, float pit_joint, float yaw_joint_rate)
  {
    if (param_.base_tau <= 0.0f)
    {
      pit_base_ = 0.0f;
      yaw_base_ = 0.0f;
      joint_valid_ = false;
      return;
    }
    float pit_joint_rate = wp;
    if (joint_valid_)
    {
      float d = pit_joint - last_pit_joint_;
      d -= TWO_PI * std::round(d / TWO_PI);
      pit_joint_rate = d / dt;
    }
    last_pit_joint_ = pit_joint;
    joint_valid_ = true;
    const float k = dt / (param_.base_tau + dt);
    pit_base_ += k * (wp - pit_joint_rate - pit_base_);
    yaw_base_ += k * (wy - yaw_joint_rate - yaw_base_);
  }

  static void Observe(float& z1, float& z2, float a_prev, float w, float wo, float dt)
  {
    const float e = w - z1;
    z1 += dt * (a_prev + z2 + 2.0f * wo * e);
    z2 += dt * wo * wo * e;
  }

  Param param_;
  Terms terms_{};
  float pit_z1_ = 0.0f;
  float pit_z2_ = 0.0f;
  float yaw_z1_ = 0.0f;
  float yaw_z2_ = 0.0f;
  float pit_a_ = 0.0f;
  float yaw_a_ = 0.0f;
  float pit_extra_ = 0.0f;
  float yaw_extra_ = 0.0f;
  float jy_ = 1.0f;
  float pit_base_ = 0.0f;
  float yaw_base_ = 0.0f;
  float last_pit_joint_ = 0.0f;
  bool joint_valid_ = false;
};
