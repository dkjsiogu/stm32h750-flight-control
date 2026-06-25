#include "flight_control/platform/host/simulated_components.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace flight_control {

namespace {

constexpr float kPwmMinUs = 1000.0f;
constexpr float kPwmMaxUs = 2000.0f;

Vector3 cross(const Vector3& lhs, const Vector3& rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

}  // namespace

SimulatedQuadPlant::SimulatedQuadPlant(PlantConfig config)
    : config_(config) {
    state_.position_m = {0.0f, 0.0f, 1.2f};
    state_.velocity_m_s = {};
    state_.attitude = {};
    state_.angular_velocity_rad_s = {};
    state_.battery_voltage_v = 16.4f;
    last_pwm_.pwm_us.fill(kPwmMinUs + 0.38f * (kPwmMaxUs - kPwmMinUs));
    motor_state_.fill(0.38f * config_.max_motor_thrust_n);
    gyro_bias_ = {0.006f, -0.004f, 0.0025f};
}

void SimulatedQuadPlant::step(const MotorPwmFrame& pwm, float dt_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    const float dt = std::max(dt_sec, 1e-4f);
    last_pwm_ = pwm;

    const std::array<float, 4> target_thrust = pwm_to_thrust(pwm);
    const float alpha = 1.0f - std::exp(-dt / std::max(config_.motor_tau_sec, 1e-4f));
    for (std::size_t index = 0; index < motor_state_.size(); ++index) {
        motor_state_[index] += alpha * (target_thrust[index] - motor_state_[index]);
    }

    const float f1 = motor_state_[0];
    const float f2 = motor_state_[1];
    const float f3 = motor_state_[2];
    const float f4 = motor_state_[3];
    const float total_thrust = f1 + f2 + f3 + f4;

    const Vector3 body_torque{
        config_.arm_length_m * (f1 - f2 - f3 + f4),
        config_.arm_length_m * (f1 + f2 - f3 - f4),
        config_.yaw_torque_coeff * (f1 - f2 + f3 - f4),
    };

    const Vector3 inertia{0.024f, 0.024f, 0.045f};
    const Vector3 angular_momentum{
        inertia.x * state_.angular_velocity_rad_s.x,
        inertia.y * state_.angular_velocity_rad_s.y,
        inertia.z * state_.angular_velocity_rad_s.z,
    };
    const Vector3 gyro_term = cross(state_.angular_velocity_rad_s, angular_momentum);
    const Vector3 angular_drag = state_.angular_velocity_rad_s * config_.angular_drag;
    Vector3 omega_dot{
        (body_torque.x - gyro_term.x - angular_drag.x) / inertia.x,
        (body_torque.y - gyro_term.y - angular_drag.y) / inertia.y,
        (body_torque.z - gyro_term.z - angular_drag.z) / inertia.z,
    };
    state_.angular_velocity_rad_s += omega_dot * dt;
    state_.attitude = integrate_body_rates(state_.attitude, state_.angular_velocity_rad_s, dt);

    const Vector3 thrust_world = rotate(state_.attitude, {0.0f, 0.0f, total_thrust});
    const Vector3 rel_velocity = state_.velocity_m_s - config_.wind_m_s;
    const float rel_speed = norm(rel_velocity);
    const Vector3 drag = rel_velocity * (-config_.linear_drag - config_.quadratic_drag * rel_speed);
    const Vector3 acceleration = (thrust_world + drag) / config_.mass_kg + Vector3{0.0f, 0.0f, -kGravity};

    state_.velocity_m_s += acceleration * dt;
    state_.position_m += state_.velocity_m_s * dt;
    if (state_.position_m.z < 0.03f) {
        state_.position_m.z = 0.03f;
        state_.velocity_m_s.z = std::max(0.0f, -state_.velocity_m_s.z * 0.35f);
        state_.velocity_m_s.x *= 0.82f;
        state_.velocity_m_s.y *= 0.82f;
    }

    timestamp_sec_ += dt;
    state_.timestamp_sec = timestamp_sec_;
    state_.battery_voltage_v = 16.4f - 0.018f * total_thrust;
}

void SimulatedQuadPlant::set_wind(const Vector3& wind_m_s) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.wind_m_s = wind_m_s;
}

FlightTelemetry SimulatedQuadPlant::telemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Vector3 gyro_jitter{
        0.003f * std::sin(0.7f * timestamp_sec_),
        0.002f * std::cos(1.1f * timestamp_sec_),
        0.0015f * std::sin(0.35f * timestamp_sec_),
    };

    SensorPacket raw{};
    raw.timestamp_sec = timestamp_sec_;
    raw.gyro_rad_s = state_.angular_velocity_rad_s + gyro_bias_ + gyro_jitter;
    raw.accel_m_s2 = {0.0f, 0.0f, kGravity};
    raw.attitude = state_.attitude;
    raw.velocity_m_s = state_.velocity_m_s;
    raw.position_m = state_.position_m;
    raw.battery_voltage_v = state_.battery_voltage_v;

    VehicleState estimated = state_;
    estimated.angular_velocity_rad_s = raw.gyro_rad_s;

    return {
        raw,
        estimated,
        config_.wind_m_s.x,
        config_.wind_m_s.y,
        4.0f,
    };
}

VehicleState SimulatedQuadPlant::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

MotorPwmFrame SimulatedQuadPlant::last_pwm() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_pwm_;
}

std::array<float, 4> SimulatedQuadPlant::pwm_to_thrust(const MotorPwmFrame& pwm) const {
    const auto channel_to_thrust = [this](float pwm_us) {
        const float normalized = std::clamp((pwm_us - kPwmMinUs) / (kPwmMaxUs - kPwmMinUs), 0.0f, 1.0f);
        return normalized * config_.max_motor_thrust_n;
    };
    return {
        channel_to_thrust(pwm.pwm_us[0]),
        channel_to_thrust(pwm.pwm_us[1]),
        channel_to_thrust(pwm.pwm_us[2]),
        channel_to_thrust(pwm.pwm_us[3]),
    };
}

MockSensorSource::MockSensorSource(std::shared_ptr<SimulatedQuadPlant> plant)
    : plant_(std::move(plant)) {}

FlightTelemetry MockSensorSource::read() {
    return plant_->telemetry();
}

MockCommandSource::MockCommandSource()
    : start_(std::chrono::steady_clock::now()) {}

GuidanceCommand MockCommandSource::read() {
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

MockPwmOutput::MockPwmOutput(std::shared_ptr<SimulatedQuadPlant> plant, float dt_sec)
    : plant_(std::move(plant)), dt_sec_(dt_sec) {}

void MockPwmOutput::write(const MotorPwmFrame& frame) {
    plant_->step(frame, dt_sec_);
}

}  // namespace flight_control
