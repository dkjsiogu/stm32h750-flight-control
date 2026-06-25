#pragma once

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

struct SpeedControllerConfig {
    float mass_kg{1.0f};
    float max_total_thrust_n{25.506f};
    float kp_xy{1.2f};
    float ki_xy{0.16f};
    float kp_z{1.8f};
    float ki_z{0.14f};
    float max_accel_xy_m_s2{4.5f};
    float max_accel_z_m_s2{3.5f};
    float max_tilt_rad{0.7f};
    float max_climb_rate_m_s{4.0f};
    float max_yaw_rate_rad_s{2.5f};
    float integral_limit_xy{2.0f};
    float integral_limit_z{2.0f};
};

class SpeedController {
public:
    explicit SpeedController(SpeedControllerConfig config = {});

    void reset(const Quaternion& current_attitude = Quaternion{});
    AttitudeSetpoint update(const VehicleState& state, const GuidanceCommand& command, float dt_sec);

private:
    Quaternion attitude_from_acceleration(const Vector3& acceleration, float yaw_target_rad) const;

    SpeedControllerConfig config_;
    Vector3 integral_xy_{};
    float integral_z_{0.0f};
    float yaw_target_{0.0f};
    bool yaw_initialized_{false};
};

}  // namespace flight_control
