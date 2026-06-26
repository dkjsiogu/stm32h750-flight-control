#include "flight_control/platform/stm32/stm32_flight_port.hpp"

namespace flight_control {

FlightTelemetry Stm32SensorSource::read() {
    FlightTelemetry telemetry{};
    flight_control_board_read_telemetry(
        &telemetry.raw,
        &telemetry.state,
        &telemetry.estimated_wind_x_m_s,
        &telemetry.estimated_wind_y_m_s,
        &telemetry.control_latency_ms);
    return telemetry;
}

GuidanceCommand Stm32CommandSource::read() {
    GuidanceCommand command{};
    flight_control_board_read_command(&command);
    return command;
}

void Stm32PwmOutput::write(const MotorPwmFrame& frame) {
    flight_control_board_write_pwm(&frame);
}

void Stm32CriticalSection::lock() {
    flight_control_board_enter_critical();
}

void Stm32CriticalSection::unlock() {
    flight_control_board_exit_critical();
}

}  // namespace flight_control
