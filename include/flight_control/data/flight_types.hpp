#pragma once

#include <array>

#include "flight_control/config.hpp"
#include "flight_control/math/vector_quaternion.hpp"

namespace flight_control {

struct SensorPacket {
    float timestamp_sec{0.0f};
    Vector3 gyro_rad_s{};
    Vector3 accel_m_s2{};
    Quaternion attitude{};
    Vector3 velocity_m_s{};
    Vector3 position_m{};
    float battery_voltage_v{0.0f};
};

struct VehicleState {
    float timestamp_sec{0.0f};
    Vector3 position_m{};
    Vector3 velocity_m_s{};
    Quaternion attitude{};
    Vector3 angular_velocity_rad_s{};
    float battery_voltage_v{0.0f};
    bool healthy{true};
};

struct FlightTelemetry {
    SensorPacket raw{};
    VehicleState state{};
    float estimated_wind_x_m_s{0.0f};
    float estimated_wind_y_m_s{0.0f};
    float control_latency_ms{0.0f};
};

struct GuidanceCommand {
    Vector3 target_velocity_m_s{};
    float climb_rate_m_s{0.0f};
    float yaw_rate_rad_s{0.0f};
    bool armed{true};
};

struct AttitudeSetpoint {
    Quaternion target_attitude{};
    float collective{0.0f};
    Vector3 target_acceleration_m_s2{};
    float yaw_target_rad{0.0f};
};

struct TorqueCommand {
    Vector3 body_torque_nm{};
};

struct MotorPwmFrame {
    std::array<float, 4> pwm_us{};
};

struct ModelFrame {
    std::array<float, kFrameDim> values{};
};

struct ControlSolution {
    AttitudeSetpoint attitude_setpoint{};
    TorqueCommand torque_command{};
    MotorPwmFrame motor_pwm{};
};

}  // namespace flight_control

