#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

// Coupled yaw tracking with a caller-supplied small-axis feedforward.
class DoubleYaw {
 public:
  struct Param {
    float k11 = 0;
    float k12 = 0;
    float k13 = 0;
    float k14 = 0;
    float k21 = 0;
    float k22 = 0;
    float k23 = 0;
    float k24 = 0;
    float big_direction = 1;
    float big_position_scale = 1;
    float imu_direction = -1;
    float big_torque_limit = 0.45f;
    float small_torque_limit = 0.20f;
    float big_max_travel = 0.5f;
    float big_max_rate = 0.65f;
    float small_margin = 0.08f;
    bool big_reference_ramp = true;
    float small_center_position = 0;
    float small_recenter_error = 0;
    float small_recenter_rate = 0.15f;
  };

  struct Torque {
    float big;
    float small;
  };

  // Angles in rad, rates in rad/s. Returns joint feedback torques in N m.
  static constexpr Torque CalculateFeedback(const Param& K,
                                            float big_angle_error,
                                            float small_angle_error,
                                            float big_rate_error,
                                            float small_rate_error) {
    return {
        K.k11 * big_angle_error + K.k12 * small_angle_error +
            K.k13 * big_rate_error + K.k14 * small_rate_error,
        K.k21 * big_angle_error + K.k22 * small_angle_error +
            K.k23 * big_rate_error + K.k24 * small_rate_error,
    };
  }

  struct Input {
    float aim_error;       ///< Target minus measured IMU yaw, wrapped in rad.
    float small_position;  ///< Signed relative position in joint coordinates.
    float aim_rate;        ///< Measured absolute yaw rate in IMU coordinates.
    float min_position;    ///< Small-axis lower bound in joint coordinates.
    float max_position;    ///< Small-axis upper bound in joint coordinates.
    float target_rate;     ///< Absolute yaw reference rate in IMU coordinates.
    float target_accel;    ///< Absolute yaw reference acceleration in IMU
                           ///< coordinates.
    float dt;              ///< Time between control updates in seconds.
  };

  struct SmallYawReference {
    float angle_error;  ///< Relative joint angle error in rad.
    float rate;         ///< Relative joint reference rate in rad/s.
    float aim_accel;    ///< Absolute yaw acceleration reference in rad/s^2.
  };

  explicit DoubleYaw(const Param& param) : param_(param) {}

  // Use the calibrated encoder position for both angle and velocity. The DM
  // velocity field has not been calibrated against joint motion.
  void UpdateBigFeedback(float raw_position, std::uint64_t sample_us) {
    if (sample_us == last_big_sample_us_) return;
    const std::uint64_t PREVIOUS_US = last_big_sample_us_;
    last_big_sample_us_ = sample_us;

    const float SCALE = param_.big_direction * param_.big_position_scale;
    const float POSITION = SCALE * raw_position;
    if (!std::isfinite(POSITION) || !(param_.big_position_scale > 0) ||
        std::fabs(param_.big_direction) != 1) {
      last_big_scale_ = big_rate_ = 0;
      big_feedback_ready_ = false;
      return;
    }

    const float DELTA = POSITION - big_position_;
    big_position_ = POSITION;
    // Reinitialize after the first packet, a stale stream, or an encoder jump.
    const std::uint64_t DT_US = sample_us - PREVIOUS_US;
    big_feedback_ready_ = PREVIOUS_US && sample_us > PREVIOUS_US &&
                          DT_US <= 50000 && SCALE == last_big_scale_ &&
                          std::fabs(DELTA) <= 0.5f;
    last_big_scale_ = SCALE;
    if (big_feedback_ready_) {
      constexpr float FILTER_TIME = 0.025f;
      const float DT = DT_US * 1e-6f;
      big_rate_ += DT / (FILTER_TIME + DT) * (DELTA / DT - big_rate_);
    } else {
      big_rate_ = 0;
    }
  }

  void Reset() {
    initialized_ = false;
    Suspend();
  }

  // An interruption invalidates torque without moving the travel origin.
  void Suspend() {
    target_reachable_ = false;
    small_center_ref_.reset();
    big_ref_ = big_position_;
    big_torque_ = small_torque_ = 0;
    torque_big_lqr_ = torque_small_lqr_ = small_feedforward_torque_ = 0;
    small_ref_rate_ = 0;
    aim_error_ = big_error_ = small_error_ = 0;
    big_saturated_ = small_saturated_ = false;
  }

  bool Calculate(const Input& input) {
    return Calculate(input, [](const SmallYawReference&) { return 0.0f; });
  }

  // A successful update stores both torques in joint coordinates, in N m.
  // The optional callback supplies small-axis feedforward in the same units.
  template <typename Feedforward>
  bool Calculate(const Input& input, Feedforward feedforward) {
    if (!big_feedback_ready_) {
      Suspend();
      return false;
    }

    const auto& P = param_;
    const float BIG_POSITION = big_position_;
    const float BIG_RATE = big_rate_;
    const float AIM_ERROR = P.imu_direction * input.aim_error;
    const float AIM_RATE = P.imu_direction * input.aim_rate;
    const float TARGET_RATE = P.imu_direction * input.target_rate;
    const float TARGET_ACCEL = P.imu_direction * input.target_accel;
    const float VALUES[] = {AIM_ERROR,
                            BIG_POSITION,
                            input.small_position,
                            BIG_RATE,
                            AIM_RATE,
                            input.min_position,
                            input.max_position,
                            TARGET_RATE,
                            TARGET_ACCEL,
                            input.dt,
                            P.k11,
                            P.k12,
                            P.k13,
                            P.k14,
                            P.k21,
                            P.k22,
                            P.k23,
                            P.k24,
                            P.big_torque_limit,
                            P.small_torque_limit,
                            P.big_max_travel,
                            P.small_margin,
                            P.small_center_position,
                            P.small_recenter_error,
                            P.small_recenter_rate};
    for (const float VALUE : VALUES) {
      if (!std::isfinite(VALUE)) {
        Suspend();
        return false;
      }
    }
    const bool INVALID_RAMP =
        P.big_reference_ramp &&
        (!std::isfinite(P.big_max_rate) || P.big_max_rate <= 0);
    if (input.dt <= 0 || input.dt > 0.02f || P.k11 <= 0 || P.k22 <= 0 ||
        std::fabs(P.big_direction) != 1 || std::fabs(P.imu_direction) != 1 ||
        P.big_torque_limit <= 0 || P.small_torque_limit <= 0 ||
        P.big_max_travel <= 0 || INVALID_RAMP || P.small_margin <= 0 ||
        P.small_recenter_error < 0 ||
        (P.small_recenter_error > 0 && P.small_recenter_rate <= 0) ||
        input.max_position - input.min_position <= 2 * P.small_margin) {
      Suspend();
      return false;
    }
    if (!initialized_) {
      big_center_ = big_ref_ = BIG_POSITION;
      initialized_ = true;
    }

    aim_error_ = AIM_ERROR;
    const float AIM_TARGET = BIG_POSITION + input.small_position + AIM_ERROR;
    const float BIG_MIN = big_center_ - P.big_max_travel;
    const float BIG_MAX = big_center_ + P.big_max_travel;
    big_ref_ = std::clamp(big_ref_, BIG_MIN, BIG_MAX);
    const float SMALL_MIN = input.min_position + P.small_margin;
    const float SMALL_MAX = input.max_position - P.small_margin;
    constexpr float ANGLE_TOLERANCE = 1e-5f;
    target_reachable_ = AIM_TARGET >= BIG_MIN + SMALL_MIN - ANGLE_TOLERANCE &&
                        AIM_TARGET <= BIG_MAX + SMALL_MAX + ANGLE_TOLERANCE;
    // Limit the centering target to positions where both joints can hold aim.
    const float CENTER_MIN = std::max(SMALL_MIN, AIM_TARGET - BIG_MAX);
    const float CENTER_MAX = std::min(SMALL_MAX, AIM_TARGET - BIG_MIN);
    float recenter_offset = 0;
    if (std::fabs(AIM_ERROR) < P.small_recenter_error &&
        CENTER_MIN <= CENTER_MAX) {
      const float CENTER =
          std::clamp(P.small_center_position, CENTER_MIN, CENTER_MAX);
      const float PREVIOUS = small_center_ref_.value_or(input.small_position);
      const float STEP = P.small_recenter_rate * input.dt;
      const float CENTER_REF =
          std::clamp(CENTER, PREVIOUS - STEP, PREVIOUS + STEP);
      small_center_ref_ = CENTER_REF;

      const float BLEND = 1.0f - std::fabs(AIM_ERROR) / P.small_recenter_error;
      const float WEIGHT = BLEND * BLEND * (3.0f - 2.0f * BLEND);
      recenter_offset = WEIGHT * (input.small_position - CENTER_REF);
    } else {
      small_center_ref_.reset();
    }

    // Opposite centering corrections preserve the total pointing reference.
    big_goal_ = std::clamp(BIG_POSITION + AIM_ERROR + recenter_offset, BIG_MIN,
                           BIG_MAX);

    if (P.big_reference_ramp) {
      const float MAX_STEP = P.big_max_rate * input.dt;
      big_ref_ =
          std::clamp(big_goal_, big_ref_ - MAX_STEP, big_ref_ + MAX_STEP);
    } else {
      big_ref_ = big_goal_;
    }
    big_error_ = big_ref_ - BIG_POSITION;
    const float REQUESTED_SMALL =
        input.small_position + AIM_ERROR - recenter_offset;
    small_ref_ = std::clamp(REQUESTED_SMALL, SMALL_MIN, SMALL_MAX);
    small_error_ = small_ref_ - input.small_position;
    // The small joint reference follows the pointing target relative to the
    // measured big joint, so its rate error follows the IMU pointing rate.
    small_ref_rate_ = TARGET_RATE - BIG_RATE;
    if (small_ref_ != REQUESTED_SMALL ||
        (small_ref_ <= SMALL_MIN && small_ref_rate_ < 0) ||
        (small_ref_ >= SMALL_MAX && small_ref_rate_ > 0))
      small_ref_rate_ = 0;

    small_rate_ = AIM_RATE - BIG_RATE;
    const float SMALL_RATE_ERROR = small_ref_rate_ - small_rate_;
    // Big yaw uses a zero angular-velocity reference.
    const auto FEEDBACK_TORQUE = CalculateFeedback(P, big_error_, small_error_,
                                                   -BIG_RATE, SMALL_RATE_ERROR);
    torque_big_lqr_ = FEEDBACK_TORQUE.big;
    torque_small_lqr_ = FEEDBACK_TORQUE.small;
    small_feedforward_torque_ = feedforward(
        SmallYawReference{small_error_, small_ref_rate_,
                          small_ref_ == REQUESTED_SMALL ? TARGET_ACCEL : 0.0f});
    const float SMALL_RAW = torque_small_lqr_ + small_feedforward_torque_;
    if (!std::isfinite(torque_big_lqr_) || !std::isfinite(SMALL_RAW)) {
      Suspend();
      return false;
    }
    big_torque_ =
        std::clamp(torque_big_lqr_, -P.big_torque_limit, P.big_torque_limit);
    small_torque_ =
        std::clamp(SMALL_RAW, -P.small_torque_limit, P.small_torque_limit);
    if ((BIG_POSITION <= BIG_MIN && big_torque_ < 0) ||
        (BIG_POSITION >= BIG_MAX && big_torque_ > 0))
      big_torque_ = 0;
    if ((input.small_position <= input.min_position && small_torque_ < 0) ||
        (input.small_position >= input.max_position && small_torque_ > 0))
      small_torque_ = 0;
    big_saturated_ = big_torque_ != torque_big_lqr_;
    small_saturated_ = small_torque_ != SMALL_RAW;
    return true;
  }

  Param param_;
  bool target_reachable_ = false;
  bool big_feedback_ready_ = false;
  bool big_saturated_ = false, small_saturated_ = false;
  float big_position_ = 0, big_rate_ = 0;
  float aim_error_ = 0, big_error_ = 0, small_error_ = 0;
  float big_goal_ = 0, big_ref_ = 0, small_ref_ = 0;
  float small_ref_rate_ = 0, small_rate_ = 0;
  float torque_big_lqr_ = 0, torque_small_lqr_ = 0;
  float small_feedforward_torque_ = 0;
  float big_torque_ = 0, small_torque_ = 0;

 private:
  float big_center_ = 0;
  std::optional<float> small_center_ref_;
  float last_big_scale_ = 0;
  std::uint64_t last_big_sample_us_ = 0;
  bool initialized_ = false;
};
