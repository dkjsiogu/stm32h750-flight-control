#include "flight_control/control/torque_controller.hpp"

#include <algorithm>

namespace flight_control {

TorqueController::TorqueController(TorqueControllerConfig config)
    : config_(config) {
    reset();
}

void TorqueController::reset() {
    const float hover_pwm = config_.pwm_min_us + 0.38f * (config_.pwm_max_us - config_.pwm_min_us);
    last_pwm_.pwm_us.fill(hover_pwm);
}

MotorPwmFrame TorqueController::mix(float collective, const TorqueCommand& torque, float dt_sec) {
    return mix(collective, torque.body_torque_nm, dt_sec);
}

MotorPwmFrame TorqueController::mix(float collective, const Vector3& body_torque_nm, float dt_sec) {
    const float total_thrust = std::clamp(collective, 0.0f, 1.0f) * config_.max_total_thrust_n;
    const float lx = std::max(config_.arm_length_m, 1e-5f);
    const float yaw = std::max(config_.yaw_torque_coeff, 1e-6f);

    const float tx = body_torque_nm.x;
    const float ty = body_torque_nm.y;
    const float tz = body_torque_nm.z;

    const std::array<float, 4> thrust_n{
        total_thrust / 4.0f + tx / (4.0f * lx) + ty / (4.0f * lx) + tz / (4.0f * yaw),
        total_thrust / 4.0f - tx / (4.0f * lx) + ty / (4.0f * lx) - tz / (4.0f * yaw),
        total_thrust / 4.0f - tx / (4.0f * lx) - ty / (4.0f * lx) + tz / (4.0f * yaw),
        total_thrust / 4.0f + tx / (4.0f * lx) - ty / (4.0f * lx) - tz / (4.0f * yaw),
    };

    MotorPwmFrame frame{};
    for (std::size_t index = 0; index < frame.pwm_us.size(); ++index) {
        const float limited_thrust = std::clamp(thrust_n[index], 0.0f, config_.max_motor_thrust_n);
        frame.pwm_us[index] = thrust_to_pwm(limited_thrust, dt_sec, index);
    }
    return frame;
}

float TorqueController::thrust_to_pwm(float thrust_n, float dt_sec, std::size_t motor_index) {
    const float normalized = std::clamp(thrust_n / std::max(config_.max_motor_thrust_n, 1e-5f), 0.0f, 1.0f);
    const float desired_pwm = config_.pwm_min_us + normalized * (config_.pwm_max_us - config_.pwm_min_us);
    const float max_delta = std::max(dt_sec, 1e-4f) * config_.pwm_slew_rate_us_per_sec;
    const float delta = std::clamp(desired_pwm - last_pwm_.pwm_us[motor_index], -max_delta, max_delta);
    last_pwm_.pwm_us[motor_index] = std::clamp(last_pwm_.pwm_us[motor_index] + delta, config_.pwm_min_us, config_.pwm_max_us);
    return last_pwm_.pwm_us[motor_index];
}

}  // namespace flight_control

