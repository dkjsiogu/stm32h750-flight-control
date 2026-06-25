#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

#include "flight_control/interfaces/flight_interfaces.hpp"

namespace flight_control {

struct PlantConfig {
    float mass_kg{1.45f};
    float arm_length_m{0.175f};
    float yaw_torque_coeff{0.018f};
    float max_total_thrust_n{30.0f};
    float max_motor_thrust_n{7.5f};
    float motor_tau_sec{0.055f};
    float angular_drag{0.045f};
    float linear_drag{0.22f};
    float quadratic_drag{0.05f};
    Vector3 wind_m_s{0.6f, -0.2f, 0.0f};
};

class SimulatedQuadPlant {
public:
    explicit SimulatedQuadPlant(PlantConfig config = {});

    void step(const MotorPwmFrame& pwm, float dt_sec);
    FlightTelemetry telemetry() const;
    VehicleState state() const;
    MotorPwmFrame last_pwm() const;

private:
    std::array<float, 4> pwm_to_thrust(const MotorPwmFrame& pwm) const;

    mutable std::mutex mutex_;
    PlantConfig config_;
    VehicleState state_{};
    MotorPwmFrame last_pwm_{};
    std::array<float, 4> motor_state_{};
    Vector3 gyro_bias_{0.0f, 0.0f, 0.0f};
    float timestamp_sec_{0.0f};
};

class MockSensorSource final : public ISensorSource {
public:
    explicit MockSensorSource(std::shared_ptr<SimulatedQuadPlant> plant);
    FlightTelemetry read() override;

private:
    std::shared_ptr<SimulatedQuadPlant> plant_;
};

class MockCommandSource final : public ICommandSource {
public:
    MockCommandSource();
    GuidanceCommand read() override;

private:
    std::chrono::steady_clock::time_point start_;
};

class MockPwmOutput final : public IPwmOutput {
public:
    MockPwmOutput(std::shared_ptr<SimulatedQuadPlant> plant, float dt_sec = kDefaultControlPeriodSec);
    void write(const MotorPwmFrame& frame) override;

private:
    std::shared_ptr<SimulatedQuadPlant> plant_;
    float dt_sec_{kDefaultControlPeriodSec};
};

}  // namespace flight_control
