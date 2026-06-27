#include "flight_control/control/torque_controller.hpp"

#include <algorithm>
#include <cmath>

namespace flight_control {

TorqueController::TorqueController(TorqueControllerConfig config)
    : config_(config) {
    reset();
}

void TorqueController::reset() {
    const float hover_thrust_ratio = 0.38f;
    const float curve = std::max(config_.thrust_curve_exponent, 1e-4f);
    const float hover_pwm_ratio = std::pow(hover_thrust_ratio, 1.0f / curve);
    const float hover_pwm = config_.pwm_min_us + hover_pwm_ratio * (config_.pwm_max_us - config_.pwm_min_us);
    last_pwm_.pwm_us.fill(hover_pwm);
}

MotorPwmFrame TorqueController::failsafe() {
    last_pwm_.pwm_us.fill(config_.pwm_min_us);
    return last_pwm_;
}

MotorPwmFrame TorqueController::mix(float collective, const TorqueCommand& torque, float dt_sec) {
    return mix(collective, torque.body_torque_nm, dt_sec, config_.nominal_battery_voltage_v);
}

MotorPwmFrame TorqueController::mix(float collective, const TorqueCommand& torque, float dt_sec, float battery_voltage_v) {
    return mix(collective, torque.body_torque_nm, dt_sec, battery_voltage_v);
}

MotorPwmFrame TorqueController::mix(float collective, const Vector3& body_torque_nm, float dt_sec) {
    return mix(collective, body_torque_nm, dt_sec, config_.nominal_battery_voltage_v);
}

MotorPwmFrame TorqueController::mix(float collective, const Vector3& body_torque_nm, float dt_sec, float battery_voltage_v) {
    const float total_thrust = std::clamp(collective, 0.0f, 1.0f) * config_.max_total_thrust_n;
    const float lx = std::max(config_.arm_length_m, 1e-5f);
    const float yaw = std::max(config_.yaw_torque_coeff, 1e-6f);

    const float tx = body_torque_nm.x;
    const float ty = body_torque_nm.y;
    const float tz = body_torque_nm.z;

    const std::array<float, 4> raw_thrust_n{
        total_thrust / 4.0f + tx / (4.0f * lx) + ty / (4.0f * lx) + tz / (4.0f * yaw),
        total_thrust / 4.0f - tx / (4.0f * lx) + ty / (4.0f * lx) - tz / (4.0f * yaw),
        total_thrust / 4.0f - tx / (4.0f * lx) - ty / (4.0f * lx) + tz / (4.0f * yaw),
        total_thrust / 4.0f + tx / (4.0f * lx) - ty / (4.0f * lx) - tz / (4.0f * yaw),
    };

    const float base_thrust = total_thrust / 4.0f;
    float torque_scale = 1.0f;
    for (const float thrust : raw_thrust_n) {
        const float delta = thrust - base_thrust;
        if (delta > 1e-6f) {
            torque_scale = std::min(torque_scale, (config_.max_motor_thrust_n - base_thrust) / delta);
        } else if (delta < -1e-6f) {
            torque_scale = std::min(torque_scale, (0.0f - base_thrust) / delta);
        }
    }
    torque_scale = std::clamp(torque_scale, 0.0f, 1.0f);

    MotorPwmFrame frame{};
    for (std::size_t index = 0; index < frame.pwm_us.size(); ++index) {
        const float allocated_thrust = base_thrust + torque_scale * (raw_thrust_n[index] - base_thrust);
        const float limited_thrust = std::clamp(allocated_thrust, 0.0f, config_.max_motor_thrust_n);
        frame.pwm_us[index] = thrust_to_pwm(limited_thrust, dt_sec, index, battery_voltage_v);
    }
    return frame;
}

float TorqueController::thrust_to_pwm(float thrust_n, float dt_sec, std::size_t motor_index, float battery_voltage_v) {
    const float voltage_ratio = std::clamp(
        battery_voltage_v / std::max(config_.nominal_battery_voltage_v, 1e-3f),
        0.55f,
        1.15f);
    const float voltage_scale = (1.0f - config_.battery_voltage_compensation_gain) +
                                config_.battery_voltage_compensation_gain * voltage_ratio * voltage_ratio;
    const float effective_max_thrust = std::max(config_.max_motor_thrust_n * voltage_scale, 1e-5f);
    const float normalized = std::clamp(thrust_n / effective_max_thrust, 0.0f, 1.0f);
    const float curve = std::max(config_.thrust_curve_exponent, 1e-4f);
    const float pwm_ratio = std::pow(normalized, 1.0f / curve);
    const float desired_pwm = config_.pwm_min_us + pwm_ratio * (config_.pwm_max_us - config_.pwm_min_us);
    const float raw_delta = desired_pwm - last_pwm_.pwm_us[motor_index];
    const float slew = raw_delta >= 0.0f ?
        std::max(config_.pwm_accel_slew_rate_us_per_sec, config_.pwm_slew_rate_us_per_sec) :
        std::max(config_.pwm_decel_slew_rate_us_per_sec, config_.pwm_slew_rate_us_per_sec);
    const float max_delta = std::max(dt_sec, 1e-4f) * slew;
    const float delta = std::clamp(raw_delta, -max_delta, max_delta);
    last_pwm_.pwm_us[motor_index] = std::clamp(last_pwm_.pwm_us[motor_index] + delta, config_.pwm_min_us, config_.pwm_max_us);
    return last_pwm_.pwm_us[motor_index];
}

}  // namespace flight_control
