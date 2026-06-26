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
 * 对标量执行线性插值。
 *
 * @param lhs 起始值。
 * @param rhs 目标值。
 * @param gain 融合增益。
 * @return 插值结果。
 */
float blend(float lhs, float rhs, float gain) {
    return lhs + (rhs - lhs) * std::clamp(gain, 0.0f, 1.0f);
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
    return {
        blend(lhs.x, rhs.x, gain),
        blend(lhs.y, rhs.y, gain),
        blend(lhs.z, rhs.z, gain),
    };
}

/**
 * 计算四元数点积。
 *
 * @param lhs 左四元数。
 * @param rhs 右四元数。
 * @return 点积结果。
 */
float quaternion_dot(const Quaternion& lhs, const Quaternion& rhs) {
    return lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
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
        blend_attitude(observation.attitude);
    }

    const Vector3 acceleration_world = rotate(state_.attitude, packet.accel_m_s2) - Vector3{0.0f, 0.0f, kGravity};
    state_.velocity_m_s += acceleration_world * dt;
    state_.position_m += state_.velocity_m_s * dt;
    if (observation.velocity_valid) {
        state_.velocity_m_s = blend(state_.velocity_m_s, observation.velocity_m_s, config_.velocity_observation_gain);
    }
    if (observation.position_valid) {
        state_.position_m = blend(state_.position_m, observation.position_m, config_.position_observation_gain);
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
    last_timestamp_sec_ = packet.timestamp_sec;
    initialized_ = true;
}

void StateEstimator::blend_attitude(const Quaternion& observed_attitude) {
    Quaternion target = normalize(observed_attitude);
    if (quaternion_dot(state_.attitude, target) < 0.0f) {
        target = {-target.w, -target.x, -target.y, -target.z};
    }
    const float gain = std::clamp(config_.attitude_observation_gain, 0.0f, 1.0f);
    state_.attitude = normalize(state_.attitude * (1.0f - gain) + target * gain);
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
