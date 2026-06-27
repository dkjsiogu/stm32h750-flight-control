#include "flight_control/estimation/state_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace flight_control {

namespace {

/**
 * 计算三维向量叉乘。
 *
 * @param lhs 左向量。
 * @param rhs 右向量。
 * @return 叉乘结果。
 */
Vector3 cross(const Vector3& lhs, const Vector3& rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

/**
 * 对向量执行线性插值。
 *
 * @param lhs 起始向量。
 * @param rhs 目标向量。
 * @param gain 融合增益。
 * @return 插值结果。
 */
Vector3 blend(const Vector3& lhs, const Vector3& rhs, float gain) {
    const float safe_gain = std::clamp(gain, 0.0f, 1.0f);
    return {
        lhs.x + (rhs.x - lhs.x) * safe_gain,
        lhs.y + (rhs.y - lhs.y) * safe_gain,
        lhs.z + (rhs.z - lhs.z) * safe_gain,
    };
}

/**
 * 计算姿态观测右不变误差向量。
 *
 * @param target 外部观测姿态。
 * @param current 当前估计姿态。
 * @return 小角度误差向量，单位 rad。
 */
Vector3 attitude_innovation(const Quaternion& target, const Quaternion& current) {
    Quaternion error = normalize(multiply(conjugate(normalize(current)), normalize(target)));
    if (error.w < 0.0f) {
        error = {-error.w, -error.x, -error.y, -error.z};
    }
    return {2.0f * error.x, 2.0f * error.y, 2.0f * error.z};
}

/**
 * 检查三维向量是否全为有限值。
 *
 * @param value 输入向量。
 * @return true 表示三个分量均为有限值。
 */
bool is_finite(const Vector3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

/**
 * 检查四元数是否全为有限值。
 *
 * @param value 输入四元数。
 * @return true 表示四个分量均为有限值。
 */
bool is_finite(const Quaternion& value) {
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

StateEstimator::StateEstimator(StateEstimatorConfig config)
    : config_(config) {
    reset();
}

void StateEstimator::reset() {
    state_ = {};
    state_.healthy = false;
    gyro_bias_rad_s_ = {};
    accel_bias_body_m_s2_ = {};
    covariance_diag_.fill(0.08f);
    last_timestamp_sec_ = 0.0f;
    initialized_ = false;
}

VehicleState StateEstimator::update(const SensorPacket& packet, const StateEstimatorObservation& observation) {
    if (!initialized_) {
        initialize(packet, observation);
        return state_;
    }

    float dt = packet.timestamp_sec - last_timestamp_sec_;
    if (!(dt > 0.0f) || !std::isfinite(dt)) {
        dt = kDefaultControlPeriodSec;
    }
    dt = std::clamp(dt, 1.0e-4f, config_.max_dt_sec);
    last_timestamp_sec_ = packet.timestamp_sec;
    propagate_covariance(dt);

    Vector3 body_rates = packet.gyro_rad_s - gyro_bias_rad_s_;
    const float accel_norm = norm(packet.accel_m_s2);
    if (accel_norm >= config_.accel_gravity_min_m_s2 && accel_norm <= config_.accel_gravity_max_m_s2) {
        const Vector3 measured_gravity_body = packet.accel_m_s2 / accel_norm;
        const Vector3 expected_gravity_body = rotate(conjugate(state_.attitude), {0.0f, 0.0f, 1.0f});
        const Vector3 gravity_error = cross(expected_gravity_body, measured_gravity_body);
        body_rates += gravity_error * config_.accel_correction_gain;
        if (norm(packet.gyro_rad_s) < config_.gyro_stationary_threshold_rad_s) {
            gyro_bias_rad_s_ += gravity_error * (config_.gyro_bias_learning_gain * dt);
        }
    }

    state_.attitude = integrate_body_rates(state_.attitude, body_rates, dt);
    if (observation.attitude_valid) {
        apply_invariant_attitude_observation(observation.attitude, dt);
    }
    if (observation.yaw_valid && std::isfinite(observation.yaw_rad)) {
        apply_yaw_observation(observation.yaw_rad, dt);
    }

    const Vector3 acceleration_world =
        rotate(state_.attitude, packet.accel_m_s2 - accel_bias_body_m_s2_) - Vector3{0.0f, 0.0f, kGravity};
    state_.velocity_m_s += acceleration_world * dt;
    state_.position_m += state_.velocity_m_s * dt;
    if (observation.velocity_valid) {
        const Vector3 velocity_innovation = state_.velocity_m_s - observation.velocity_m_s;
        const float weight = observation_weight(
            norm_squared(velocity_innovation),
            config_.velocity_observation_variance + covariance_diag_[3] + covariance_diag_[4] + covariance_diag_[5],
            3.0f);
        const Vector3 body_bias_innovation = rotate(conjugate(state_.attitude), velocity_innovation);
        accel_bias_body_m_s2_ += body_bias_innovation * (config_.accel_bias_observation_gain * weight * dt);
        state_.velocity_m_s = blend(
            state_.velocity_m_s,
            observation.velocity_m_s,
            config_.velocity_observation_gain * weight);
        for (std::size_t index = 3U; index < 6U; ++index) {
            covariance_diag_[index] *= (1.0f - 0.35f * weight);
        }
    }
    if (observation.position_valid) {
        const Vector3 position_innovation = state_.position_m - observation.position_m;
        const float weight = observation_weight(
            norm_squared(position_innovation),
            config_.position_observation_variance + covariance_diag_[6] + covariance_diag_[7] + covariance_diag_[8],
            3.0f);
        state_.position_m = blend(state_.position_m, observation.position_m, config_.position_observation_gain * weight);
        for (std::size_t index = 6U; index < 9U; ++index) {
            covariance_diag_[index] *= (1.0f - 0.30f * weight);
        }
    }

    state_.timestamp_sec = packet.timestamp_sec;
    state_.angular_velocity_rad_s = body_rates;
    state_.battery_voltage_v = packet.battery_voltage_v;
    state_.healthy = state_is_finite() && sensor_is_usable(packet);
    return state_;
}

VehicleState StateEstimator::state() const {
    return state_;
}

void StateEstimator::initialize(const SensorPacket& packet, const StateEstimatorObservation& observation) {
    state_ = {};
    state_.timestamp_sec = packet.timestamp_sec;
    state_.attitude = observation.attitude_valid ? normalize(observation.attitude) : Quaternion{};
    state_.velocity_m_s = observation.velocity_valid ? observation.velocity_m_s : packet.velocity_m_s;
    state_.position_m = observation.position_valid ? observation.position_m : packet.position_m;
    state_.angular_velocity_rad_s = packet.gyro_rad_s;
    state_.battery_voltage_v = packet.battery_voltage_v;
    state_.healthy = state_is_finite() && sensor_is_usable(packet);
    covariance_diag_.fill(0.08f);
    last_timestamp_sec_ = packet.timestamp_sec;
    initialized_ = true;
}

void StateEstimator::apply_invariant_attitude_observation(const Quaternion& observed_attitude, float dt) {
    const Vector3 innovation = attitude_innovation(observed_attitude, state_.attitude);
    const float weight = observation_weight(
        norm_squared(innovation),
        config_.attitude_observation_variance + covariance_diag_[0] + covariance_diag_[1] + covariance_diag_[2],
        3.0f);
    const float gain = std::clamp(config_.attitude_observation_gain, 0.0f, 1.0f);
    state_.attitude = integrate_body_rates(state_.attitude, innovation * (gain * weight), 1.0f);
    gyro_bias_rad_s_ -= innovation * (config_.gyro_bias_observation_gain * weight * std::max(dt, 1.0e-4f));
    for (std::size_t index = 0U; index < 3U; ++index) {
        covariance_diag_[index] *= (1.0f - 0.45f * weight);
    }
}

void StateEstimator::apply_yaw_observation(float yaw_rad, float dt) {
    const Vector3 euler = to_euler_zyx(state_.attitude);
    const Quaternion yaw_attitude = from_euler_zyx(euler.x, euler.y, yaw_rad);
    const float innovation = wrap_angle(yaw_rad - euler.z);
    const float weight = observation_weight(
        innovation * innovation,
        config_.yaw_observation_variance + covariance_diag_[2],
        1.0f);
    state_.attitude = integrate_body_rates(
        state_.attitude,
        attitude_innovation(yaw_attitude, state_.attitude) * (config_.attitude_observation_gain * weight),
        1.0f);
    gyro_bias_rad_s_.z -= innovation * (config_.gyro_bias_observation_gain * weight * std::max(dt, 1.0e-4f));
    covariance_diag_[2] *= (1.0f - 0.40f * weight);
}

float StateEstimator::observation_weight(float innovation_sq, float variance, float dimension) const {
    const float safe_variance = std::max(variance, 1.0e-5f);
    const float normalized_innovation = innovation_sq / (safe_variance * std::max(dimension, 1.0f));
    if (normalized_innovation <= config_.nis_gate) {
        return 1.0f;
    }
    const float soft_weight = config_.nis_gate / std::max(normalized_innovation, 1.0e-5f);
    return std::clamp(soft_weight, config_.min_observation_weight, 1.0f);
}

void StateEstimator::propagate_covariance(float dt) {
    for (std::size_t index = 0U; index < 3U; ++index) {
        covariance_diag_[index] = std::clamp(
            covariance_diag_[index] + config_.attitude_process_noise * dt,
            1.0e-5f,
            2.0f);
        covariance_diag_[index + 3U] = std::clamp(
            covariance_diag_[index + 3U] + config_.velocity_process_noise * dt,
            1.0e-5f,
            8.0f);
        covariance_diag_[index + 6U] = std::clamp(
            covariance_diag_[index + 6U] + config_.position_process_noise * dt,
            1.0e-5f,
            20.0f);
    }
}

bool StateEstimator::state_is_finite() const {
    return is_finite(state_.position_m) &&
           is_finite(state_.velocity_m_s) &&
           is_finite(state_.attitude) &&
           is_finite(state_.angular_velocity_rad_s) &&
           std::isfinite(state_.battery_voltage_v);
}

bool StateEstimator::sensor_is_usable(const SensorPacket& packet) const {
    const float accel_norm = norm(packet.accel_m_s2);
    return is_finite(packet.gyro_rad_s) &&
           is_finite(packet.accel_m_s2) &&
           std::isfinite(packet.timestamp_sec) &&
           accel_norm >= config_.accel_gravity_min_m_s2 &&
           accel_norm <= config_.accel_gravity_max_m_s2;
}

}  // namespace flight_control
