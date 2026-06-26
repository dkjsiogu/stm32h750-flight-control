#include "board_io.hpp"

namespace {

/** 板级驱动是否已经完成初始化。 */
bool g_board_ready{false};

}  // namespace

extern "C" {

/**
 * 初始化 H743 板级驱动。
 *
 * 启动控制任务前先写 failsafe 输出，再初始化真实驱动；初始化失败时保持 disarm。
 */
void flight_control_board_initialize() {
    board_write_failsafe_outputs();
    g_board_ready = board_drivers_initialize();
    if (!g_board_ready) {
        board_write_failsafe_outputs();
    }
}

/**
 * 读取传感器并交给固件状态估计器。
 *
 * @param packet 输出原始 IMU、电池和低层传感器数据。
 * @param observation 输出可选外部姿态、速度和位置观测。
 * @param estimated_wind_x_m_s 输出估计风速 x 分量，单位 m/s。
 * @param estimated_wind_y_m_s 输出估计风速 y 分量，单位 m/s。
 * @param control_latency_ms 输出控制链路延迟，单位 ms。
 */
void flight_control_board_read_sensors(
    flight_control::SensorPacket* packet,
    flight_control::StateEstimatorObservation* observation,
    float* estimated_wind_x_m_s,
    float* estimated_wind_y_m_s,
    float* control_latency_ms) {
    if (packet) {
        *packet = {};
        if (g_board_ready) {
            (void)board_read_imu_sample(packet);
        }
    }
    if (observation) {
        *observation = {};
        if (g_board_ready) {
            (void)board_read_external_observation(observation);
        }
    }
    board_read_estimated_wind(estimated_wind_x_m_s, estimated_wind_y_m_s);
    if (control_latency_ms) {
        *control_latency_ms = board_control_latency_ms();
    }
}

/**
 * 读取速度/爬升/yaw 指令。
 *
 * @param command 输出上层控制指令；驱动未就绪或指令无效时强制 disarm。
 */
void flight_control_board_read_command(flight_control::GuidanceCommand* command) {
    if (!command) {
        return;
    }
    *command = {};
    if (!g_board_ready || !board_read_guidance_command(command)) {
        command->armed = false;
    }
}

/**
 * 写飞控混控后的 PWM 输出。
 *
 * @param frame 四路电机 PWM；驱动未就绪时改写 failsafe 输出。
 */
void flight_control_board_write_pwm(const flight_control::MotorPwmFrame* frame) {
    if (!g_board_ready || !frame) {
        board_write_failsafe_outputs();
        return;
    }
    board_write_motor_pwm(frame);
}

/** 进入真实板级临界区。 */
void flight_control_board_enter_critical() {
    board_enter_critical();
}

/** 退出真实板级临界区。 */
void flight_control_board_exit_critical() {
    board_exit_critical();
}

}  // extern "C"
