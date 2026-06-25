#pragma once

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

struct TorqueControllerConfig {
    float arm_length_m{0.175f};
    float yaw_torque_coeff{0.018f};
    float max_total_thrust_n{30.0f};
    float max_motor_thrust_n{7.5f};
    float pwm_min_us{1000.0f};
    float pwm_max_us{2000.0f};
    float pwm_slew_rate_us_per_sec{3500.0f};
};

class TorqueController {
public:
    explicit TorqueController(TorqueControllerConfig config = {});

    void reset();
    MotorPwmFrame mix(float collective, const TorqueCommand& torque, float dt_sec);
    MotorPwmFrame mix(float collective, const Vector3& body_torque_nm, float dt_sec);

private:
    float thrust_to_pwm(float thrust_n, float dt_sec, std::size_t motor_index);

    TorqueControllerConfig config_;
    MotorPwmFrame last_pwm_{};
};

}  // namespace flight_control

