#include "flight_control/platform/stm32/stm32_flight_port.hpp"

namespace {

/**
 * 弱符号标记。
 *
 * GCC/Clang 下允许真实 STM32 板级工程用同名强符号覆盖这些默认 hook。
 */
#if defined(__GNUC__) || defined(__clang__)
#define FLIGHT_CONTROL_WEAK __attribute__((weak))
#else
#define FLIGHT_CONTROL_WEAK
#endif

}  // namespace

extern "C" {

/** 默认板级初始化 hook，不接真实板级工程时不执行任何操作。 */
void FLIGHT_CONTROL_WEAK flight_control_board_initialize() {}

void FLIGHT_CONTROL_WEAK flight_control_board_read_sensors(
    flight_control::SensorPacket* packet,
    flight_control::StateEstimatorObservation* observation,
    float* estimated_wind_x_m_s,
    float* estimated_wind_y_m_s,
    float* control_latency_ms) {
    if (packet) {
        *packet = {};
    }
    if (observation) {
        *observation = {};
    }
    if (estimated_wind_x_m_s) {
        *estimated_wind_x_m_s = 0.0f;
    }
    if (estimated_wind_y_m_s) {
        *estimated_wind_y_m_s = 0.0f;
    }
    if (control_latency_ms) {
        *control_latency_ms = 0.0f;
    }
}

void FLIGHT_CONTROL_WEAK flight_control_board_read_command(flight_control::GuidanceCommand* command) {
    if (command) {
        *command = {};
        command->armed = false;
    }
}

void FLIGHT_CONTROL_WEAK flight_control_board_write_pwm(const flight_control::MotorPwmFrame* frame) {
    (void)frame;
}

void FLIGHT_CONTROL_WEAK flight_control_board_enter_critical() {}

void FLIGHT_CONTROL_WEAK flight_control_board_exit_critical() {}

}  // extern "C"
