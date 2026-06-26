#include "board_io.hpp"

namespace {

/**
 * 弱符号声明宏。
 *
 * 用于让真实板级驱动以同名强符号覆盖默认 failsafe 实现。
 */
#if defined(__GNUC__) || defined(__clang__)
#define H743_BOARD_WEAK __attribute__((weak))
#else
#define H743_BOARD_WEAK
#endif

}  // namespace

extern "C" {

bool H743_BOARD_WEAK board_read_imu_sample(flight_control::SensorPacket* packet) {
    if (packet) {
        *packet = {};
    }
    return false;
}

bool H743_BOARD_WEAK board_read_external_observation(flight_control::StateEstimatorObservation* observation) {
    if (observation) {
        *observation = {};
    }
    return false;
}

bool H743_BOARD_WEAK board_read_guidance_command(flight_control::GuidanceCommand* command) {
    if (command) {
        *command = {};
        command->armed = false;
    }
    return false;
}

void H743_BOARD_WEAK board_write_motor_pwm(const flight_control::MotorPwmFrame* frame) {
    (void)frame;
}

void H743_BOARD_WEAK board_read_estimated_wind(float* wind_x_m_s, float* wind_y_m_s) {
    if (wind_x_m_s) {
        *wind_x_m_s = 0.0f;
    }
    if (wind_y_m_s) {
        *wind_y_m_s = 0.0f;
    }
}

float H743_BOARD_WEAK board_control_latency_ms() {
    return 0.0f;
}

void H743_BOARD_WEAK board_enter_critical() {}

void H743_BOARD_WEAK board_exit_critical() {}

void flight_control_board_read_sensors(
    flight_control::SensorPacket* packet,
    flight_control::StateEstimatorObservation* observation,
    float* estimated_wind_x_m_s,
    float* estimated_wind_y_m_s,
    float* control_latency_ms) {
    if (packet) {
        *packet = {};
        (void)board_read_imu_sample(packet);
    }
    if (observation) {
        *observation = {};
        (void)board_read_external_observation(observation);
    }
    board_read_estimated_wind(estimated_wind_x_m_s, estimated_wind_y_m_s);
    if (control_latency_ms) {
        *control_latency_ms = board_control_latency_ms();
    }
}

void flight_control_board_read_command(flight_control::GuidanceCommand* command) {
    if (!command) {
        return;
    }
    *command = {};
    if (!board_read_guidance_command(command)) {
        command->armed = false;
    }
}

void flight_control_board_write_pwm(const flight_control::MotorPwmFrame* frame) {
    board_write_motor_pwm(frame);
}

void flight_control_board_enter_critical() {
    board_enter_critical();
}

void flight_control_board_exit_critical() {
    board_exit_critical();
}

}  // extern "C"
