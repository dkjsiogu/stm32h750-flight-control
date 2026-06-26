#include "flight_control/control/speed_controller.hpp"

#include <algorithm>
#include <cmath>

#include "flight_control/config.hpp"

namespace flight_control {

SpeedController::SpeedController(SpeedControllerConfig config)
    : config_(config) {}

void SpeedController::reset(const Quaternion& current_attitude) {
    integral_xy_ = {};
    integral_z_ = 0.0f;
    last_acceleration_m_s2_ = {};
    hover_thrust_trim_ = 0.0f;
    yaw_target_ = to_euler_zyx(current_attitude).z;
    yaw_initialized_ = true;
    altitude_hold_target_m_ = 0.0f;
    altitude_hold_initialized_ = false;
}

AttitudeSetpoint SpeedController::update(const VehicleState& state, const GuidanceCommand& command, float dt_sec) {
    if (!command.armed || !state.healthy) {
        reset(state.attitude);
        return {};
    }

    if (!yaw_initialized_) {
        reset(state.attitude);
    }

    const float dt = std::max(dt_sec, 1e-4f);
    const float climb_rate = std::clamp(command.climb_rate_m_s, -config_.max_climb_rate_m_s, config_.max_climb_rate_m_s);
    const float yaw_rate = std::clamp(command.yaw_rate_rad_s, -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s);
    if (!altitude_hold_initialized_) {
        altitude_hold_target_m_ = state.position_m.z;
        altitude_hold_initialized_ = true;
    }
    altitude_hold_target_m_ += climb_rate * dt;
    const float altitude_error = altitude_hold_target_m_ - state.position_m.z;
    const float altitude_correction = std::clamp(
        config_.kp_altitude_hold * altitude_error,
        -config_.max_altitude_correction_m_s,
        config_.max_altitude_correction_m_s);
    float vertical_velocity_target = climb_rate + altitude_correction;

    const Vector3 velocity_error{
        command.target_velocity_m_s.x - state.velocity_m_s.x,
        command.target_velocity_m_s.y - state.velocity_m_s.y,
        vertical_velocity_target - state.velocity_m_s.z,
    };

    integral_xy_.x = std::clamp(integral_xy_.x + velocity_error.x * dt, -config_.integral_limit_xy, config_.integral_limit_xy);
    integral_xy_.y = std::clamp(integral_xy_.y + velocity_error.y * dt, -config_.integral_limit_xy, config_.integral_limit_xy);
    integral_z_ = std::clamp(integral_z_ + velocity_error.z * dt, -config_.integral_limit_z, config_.integral_limit_z);
    if (std::fabs(command.climb_rate_m_s) <= 0.02f) {
        hover_thrust_trim_ = std::clamp(
            hover_thrust_trim_ + config_.hover_thrust_trim_gain * velocity_error.z * dt,
            -config_.hover_thrust_trim_limit,
            config_.hover_thrust_trim_limit);
    }

    Vector3 acceleration_xy{
        config_.kp_xy * velocity_error.x + config_.ki_xy * integral_xy_.x,
        config_.kp_xy * velocity_error.y + config_.ki_xy * integral_xy_.y,
        0.0f,
    };
    acceleration_xy = clamp_magnitude(acceleration_xy, config_.max_accel_xy_m_s2);

    const float acceleration_z = std::clamp(
        config_.kp_z * velocity_error.z + config_.ki_z * integral_z_,
        -config_.max_accel_z_m_s2,
        config_.max_accel_z_m_s2);

    yaw_target_ = wrap_angle(yaw_target_ + yaw_rate * dt);

    Vector3 acceleration{
        acceleration_xy.x,
        acceleration_xy.y,
        acceleration_z,
    };
    const Vector3 accel_delta{
        acceleration.x - last_acceleration_m_s2_.x,
        acceleration.y - last_acceleration_m_s2_.y,
        acceleration.z - last_acceleration_m_s2_.z,
    };
    const Vector3 limited_xy_delta = clamp_magnitude(
        {accel_delta.x, accel_delta.y, 0.0f},
        config_.max_accel_xy_slew_m_s3 * dt);
    const float limited_z_delta = std::clamp(
        accel_delta.z,
        -config_.max_accel_z_slew_m_s3 * dt,
        config_.max_accel_z_slew_m_s3 * dt);
    acceleration = {
        last_acceleration_m_s2_.x + limited_xy_delta.x,
        last_acceleration_m_s2_.y + limited_xy_delta.y,
        last_acceleration_m_s2_.z + limited_z_delta,
    };
    last_acceleration_m_s2_ = acceleration;

    const Quaternion target_attitude = attitude_from_acceleration(acceleration, yaw_target_);

    const Vector3 thrust_vector{acceleration.x, acceleration.y, kGravity + acceleration.z};
    const float collective = std::clamp(
        config_.mass_kg * norm(thrust_vector) / config_.max_total_thrust_n + hover_thrust_trim_,
        0.0f,
        1.0f);

    return {
        target_attitude,
        collective,
        acceleration,
        yaw_target_,
    };
}

Quaternion SpeedController::attitude_from_acceleration(const Vector3& acceleration, float yaw_target_rad) const {
    const float vertical = std::max(0.25f * kGravity, kGravity + acceleration.z);
    const float sin_yaw = std::sin(yaw_target_rad);
    const float cos_yaw = std::cos(yaw_target_rad);

    float roll = std::atan2(acceleration.x * sin_yaw - acceleration.y * cos_yaw, vertical);
    float pitch = std::atan2(acceleration.x * cos_yaw + acceleration.y * sin_yaw, vertical);

    const float tilt = std::sqrt(roll * roll + pitch * pitch);
    if (tilt > config_.max_tilt_rad && tilt > 1e-6f) {
        const float scale = config_.max_tilt_rad / tilt;
        roll *= scale;
        pitch *= scale;
    }

    return from_euler_zyx(roll, pitch, yaw_target_rad);
}

}  // namespace flight_control
