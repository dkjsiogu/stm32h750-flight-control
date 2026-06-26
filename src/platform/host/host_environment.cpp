#include "flight_control/platform/host/host_environment.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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
 * @param lhs 起点值。
 * @param rhs 终点值。
 * @param ratio 插值比例。
 * @return 插值结果。
 */
float lerp(float lhs, float rhs, float ratio) {
    return lhs + (rhs - lhs) * ratio;
}

/**
 * 对向量执行线性插值。
 *
 * @param lhs 起点向量。
 * @param rhs 终点向量。
 * @param ratio 插值比例。
 * @return 插值结果。
 */
Vector3 lerp(const Vector3& lhs, const Vector3& rhs, float ratio) {
    return {
        lerp(lhs.x, rhs.x, ratio),
        lerp(lhs.y, rhs.y, ratio),
        lerp(lhs.z, rhs.z, ratio),
    };
}

}  // namespace

HostFlightEnvironment::HostFlightEnvironment(HostEnvironmentConfig config)
    : config_(config) {
    truth_.position_m = {0.0f, 0.0f, 1.2f};
    truth_.velocity_m_s = {};
    truth_.attitude = {};
    truth_.angular_velocity_rad_s = {};
    truth_.battery_voltage_v = config_.battery_full_voltage_v;
    last_pwm_.pwm_us.fill(config_.pwm_min_us + 0.39f * (config_.pwm_max_us - config_.pwm_min_us));
    motor_state_.fill(0.39f * config_.max_motor_thrust_n);
    truth_history_.fill(truth_);
    history_size_ = truth_history_.size();
}

void HostFlightEnvironment::step(const MotorPwmFrame& pwm, float dt_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    const float dt = std::max(dt_sec, 1e-4f);
    last_pwm_ = pwm;

    const std::array<float, 4> target_thrust = pwm_to_thrust(pwm);
    for (std::size_t index = 0; index < motor_state_.size(); ++index) {
        const float tau = std::max(config_.motor_tau_sec[index], 1e-4f);
        const float alpha = 1.0f - std::exp(-dt / tau);
        motor_state_[index] += alpha * (target_thrust[index] - motor_state_[index]);
    }

    const float f1 = motor_state_[0];
    const float f2 = motor_state_[1];
    const float f3 = motor_state_[2];
    const float f4 = motor_state_[3];
    const float total_thrust = f1 + f2 + f3 + f4;

    const Vector3 wind = wind_at(timestamp_sec_);
    const Vector3 relative_velocity = truth_.velocity_m_s - wind;
    const Vector3 wind_torque{
        config_.wind_torque_coeff * relative_velocity.y,
        -config_.wind_torque_coeff * relative_velocity.x,
        config_.wind_torque_coeff * 0.25f * (relative_velocity.x - relative_velocity.y),
    };
    const Vector3 body_torque{
        config_.arm_length_m * (f1 - f2 - f3 + f4) + wind_torque.x,
        config_.arm_length_m * (f1 + f2 - f3 - f4) + wind_torque.y,
        config_.yaw_torque_coeff * (f1 - f2 + f3 - f4) + wind_torque.z,
    };

    const Vector3 angular_momentum{
        config_.inertia_kg_m2.x * truth_.angular_velocity_rad_s.x,
        config_.inertia_kg_m2.y * truth_.angular_velocity_rad_s.y,
        config_.inertia_kg_m2.z * truth_.angular_velocity_rad_s.z,
    };
    const Vector3 gyro_term = cross(truth_.angular_velocity_rad_s, angular_momentum);
    const Vector3 angular_drag = truth_.angular_velocity_rad_s * config_.angular_drag;
    const Vector3 omega_dot{
        (body_torque.x - gyro_term.x - angular_drag.x) / config_.inertia_kg_m2.x,
        (body_torque.y - gyro_term.y - angular_drag.y) / config_.inertia_kg_m2.y,
        (body_torque.z - gyro_term.z - angular_drag.z) / config_.inertia_kg_m2.z,
    };
    truth_.angular_velocity_rad_s += omega_dot * dt;
    truth_.attitude = integrate_body_rates(truth_.attitude, truth_.angular_velocity_rad_s, dt);

    const Vector3 thrust_world = rotate(truth_.attitude, {0.0f, 0.0f, total_thrust});
    const float relative_speed = norm(relative_velocity);
    const Vector3 aerodynamic_drag = relative_velocity * (-config_.linear_drag - config_.quadratic_drag * relative_speed);
    const float total_mass = std::max(config_.mass_kg + config_.payload_mass_kg, 0.1f);
    const Vector3 acceleration = (thrust_world + aerodynamic_drag) / total_mass + Vector3{0.0f, 0.0f, -kGravity};

    truth_.velocity_m_s += acceleration * dt;
    truth_.position_m += truth_.velocity_m_s * dt;
    if (truth_.position_m.z < 0.03f) {
        truth_.position_m.z = 0.03f;
        truth_.velocity_m_s.z = std::max(0.0f, -truth_.velocity_m_s.z * 0.28f);
        truth_.velocity_m_s.x *= 0.76f;
        truth_.velocity_m_s.y *= 0.76f;
    }

    timestamp_sec_ += dt;
    truth_.timestamp_sec = timestamp_sec_;
    truth_.battery_voltage_v = config_.battery_full_voltage_v - config_.battery_sag_v_per_n * total_thrust;

    truth_history_[history_head_] = truth_;
    history_head_ = (history_head_ + 1) % truth_history_.size();
    history_size_ = std::min(history_size_ + 1, truth_history_.size());
}

void HostFlightEnvironment::set_wind(const Vector3& wind_m_s) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.wind_m_s = wind_m_s;
}

FlightTelemetry HostFlightEnvironment::telemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const VehicleState delayed = delayed_truth(timestamp_sec_ - config_.sensor_latency_sec);
    const VehicleState estimated = estimated_state_from_truth(delayed);
    const Vector3 current_wind = wind_at(timestamp_sec_);
    const Vector3 gyro_noise{
        0.0035f * std::sin(39.0f * timestamp_sec_) + 0.0015f * std::sin(3.1f * timestamp_sec_),
        0.0028f * std::cos(33.0f * timestamp_sec_) + 0.0012f * std::sin(2.7f * timestamp_sec_),
        0.0022f * std::sin(27.0f * timestamp_sec_) + 0.0010f * std::cos(1.9f * timestamp_sec_),
    };
    const Vector3 accel_noise{
        0.08f * std::sin(21.0f * timestamp_sec_),
        0.07f * std::cos(19.0f * timestamp_sec_),
        0.10f * std::sin(17.0f * timestamp_sec_),
    };

    SensorPacket raw{};
    raw.timestamp_sec = timestamp_sec_;
    raw.gyro_rad_s = estimated.angular_velocity_rad_s + gyro_noise;
    raw.accel_m_s2 = rotate(conjugate(estimated.attitude), Vector3{0.0f, 0.0f, kGravity}) + accel_noise;
    raw.attitude = estimated.attitude;
    raw.velocity_m_s = estimated.velocity_m_s;
    raw.position_m = estimated.position_m;
    raw.battery_voltage_v = estimated.battery_voltage_v;

    return {
        raw,
        estimated,
        current_wind.x * 0.88f,
        current_wind.y * 0.88f,
        config_.sensor_latency_sec * 1000.0f,
    };
}

VehicleState HostFlightEnvironment::truth_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return truth_;
}

MotorPwmFrame HostFlightEnvironment::last_pwm() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_pwm_;
}

std::array<float, 4> HostFlightEnvironment::pwm_to_thrust(const MotorPwmFrame& pwm) const {
    std::array<float, 4> thrust{};
    for (std::size_t index = 0; index < thrust.size(); ++index) {
        const float normalized = std::clamp(
            (pwm.pwm_us[index] - config_.pwm_min_us) / (config_.pwm_max_us - config_.pwm_min_us),
            0.0f,
            1.0f);
        const float nonlinear = std::pow(normalized, config_.pwm_thrust_exponent);
        thrust[index] = nonlinear * config_.max_motor_thrust_n * config_.motor_thrust_scale[index];
    }
    return thrust;
}

Vector3 HostFlightEnvironment::wind_at(float time_sec) const {
    return {
        config_.wind_m_s.x + config_.gust_amplitude_m_s.x * std::sin(0.83f * time_sec + 0.4f),
        config_.wind_m_s.y + config_.gust_amplitude_m_s.y * std::sin(1.17f * time_sec + 1.2f),
        config_.wind_m_s.z + config_.gust_amplitude_m_s.z * std::sin(0.53f * time_sec + 0.8f),
    };
}

VehicleState HostFlightEnvironment::delayed_truth(float delayed_time_sec) const {
    if (history_size_ == 0 || delayed_time_sec <= 0.0f) {
        return truth_history_[(history_head_ + truth_history_.size() - 1) % truth_history_.size()];
    }

    VehicleState newest = truth_history_[(history_head_ + truth_history_.size() - 1) % truth_history_.size()];
    VehicleState oldest = newest;
    for (std::size_t offset = 0; offset < history_size_; ++offset) {
        const std::size_t index = (history_head_ + truth_history_.size() - 1 - offset) % truth_history_.size();
        const VehicleState sample = truth_history_[index];
        if (sample.timestamp_sec <= delayed_time_sec) {
            oldest = sample;
            if (offset == 0) {
                return sample;
            }
            const std::size_t newer_index = (index + 1) % truth_history_.size();
            newest = truth_history_[newer_index];
            const float span = std::max(newest.timestamp_sec - sample.timestamp_sec, 1e-4f);
            const float ratio = std::clamp((delayed_time_sec - sample.timestamp_sec) / span, 0.0f, 1.0f);
            VehicleState interpolated = sample;
            interpolated.timestamp_sec = delayed_time_sec;
            interpolated.position_m = lerp(sample.position_m, newest.position_m, ratio);
            interpolated.velocity_m_s = lerp(sample.velocity_m_s, newest.velocity_m_s, ratio);
            interpolated.angular_velocity_rad_s = lerp(sample.angular_velocity_rad_s, newest.angular_velocity_rad_s, ratio);
            interpolated.attitude = normalize(sample.attitude * (1.0f - ratio) + newest.attitude * ratio);
            interpolated.battery_voltage_v = lerp(sample.battery_voltage_v, newest.battery_voltage_v, ratio);
            return interpolated;
        }
        oldest = sample;
    }
    return oldest;
}

VehicleState HostFlightEnvironment::estimated_state_from_truth(const VehicleState& truth) const {
    const Vector3 gyro_drift{
        config_.gyro_drift_rad_s.x * std::sin(0.21f * timestamp_sec_),
        config_.gyro_drift_rad_s.y * std::cos(0.17f * timestamp_sec_),
        config_.gyro_drift_rad_s.z * std::sin(0.13f * timestamp_sec_),
    };
    const Vector3 attitude_bias{
        config_.attitude_drift_rad.x * std::sin(0.11f * timestamp_sec_),
        config_.attitude_drift_rad.y * std::cos(0.09f * timestamp_sec_),
        config_.attitude_drift_rad.z * std::sin(0.07f * timestamp_sec_),
    };
    const Vector3 velocity_bias{
        config_.velocity_bias_m_s.x * std::sin(0.31f * timestamp_sec_ + 0.2f),
        config_.velocity_bias_m_s.y * std::cos(0.27f * timestamp_sec_ + 0.1f),
        config_.velocity_bias_m_s.z * std::sin(0.23f * timestamp_sec_ + 0.5f),
    };
    const Vector3 position_bias{
        config_.position_bias_m.x * std::sin(0.19f * timestamp_sec_),
        config_.position_bias_m.y * std::cos(0.16f * timestamp_sec_),
        config_.position_bias_m.z * std::sin(0.14f * timestamp_sec_),
    };

    VehicleState estimated = truth;
    estimated.attitude = normalize(multiply(truth.attitude, from_euler_zyx(attitude_bias.x, attitude_bias.y, attitude_bias.z)));
    estimated.angular_velocity_rad_s = truth.angular_velocity_rad_s + config_.gyro_bias_rad_s + gyro_drift;
    estimated.velocity_m_s = truth.velocity_m_s + velocity_bias;
    estimated.position_m = truth.position_m + position_bias;
    estimated.battery_voltage_v = truth.battery_voltage_v;
    estimated.healthy = truth.healthy;
    return estimated;
}

HostSensorSource::HostSensorSource(std::shared_ptr<HostFlightEnvironment> environment)
    : environment_(std::move(environment)) {}

FlightTelemetry HostSensorSource::read() {
    return environment_->telemetry();
}

ScriptedCommandSource::ScriptedCommandSource()
    : start_(std::chrono::steady_clock::now()) {}

GuidanceCommand ScriptedCommandSource::read() {
    using namespace std::chrono;
    const float elapsed = duration_cast<duration<float>>(steady_clock::now() - start_).count();

    GuidanceCommand command{};
    command.armed = true;
    if (elapsed < 2.0f) {
        return command;
    }
    if (elapsed < 5.0f) {
        command.target_velocity_m_s = {1.4f, 0.0f, 0.0f};
        return command;
    }
    if (elapsed < 7.5f) {
        command.target_velocity_m_s = {1.1f, 0.35f, 0.0f};
        command.climb_rate_m_s = 0.45f;
        command.yaw_rate_rad_s = 0.16f;
        return command;
    }
    if (elapsed < 10.0f) {
        command.target_velocity_m_s = {1.2f, -0.55f, 0.0f};
        command.climb_rate_m_s = -0.12f;
        command.yaw_rate_rad_s = -0.24f;
        return command;
    }
    return command;
}

HostPwmOutput::HostPwmOutput(std::shared_ptr<HostFlightEnvironment> environment, float dt_sec)
    : environment_(std::move(environment)), dt_sec_(dt_sec) {}

void HostPwmOutput::write(const MotorPwmFrame& frame) {
    environment_->step(frame, dt_sec_);
}

}  // namespace flight_control
