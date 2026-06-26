#include "board_io.hpp"

extern "C" {

/** 链接验证模式的早期系统初始化，不配置真实硬件。 */
void board_system_preinit() {}

/**
 * 链接验证模式的驱动初始化。
 *
 * @return 固定返回 false，保证默认镜像保持 disarm/failsafe。
 */
bool board_drivers_initialize() {
    return false;
}

/** 链接验证模式的安全输出，不驱动任何真实电机。 */
void board_write_failsafe_outputs() {}

/**
 * 链接验证模式的 IMU 读取。
 *
 * @param packet 输出清零后的传感器包。
 * @return 固定返回 false，表示没有真实 IMU。
 */
bool board_read_imu_sample(flight_control::SensorPacket* packet) {
    if (packet) {
        *packet = {};
    }
    return false;
}

/**
 * 链接验证模式的外部观测读取。
 *
 * @param observation 输出清零后的外部观测。
 * @return 固定返回 false，表示没有外部观测。
 */
bool board_read_external_observation(flight_control::StateEstimatorObservation* observation) {
    if (observation) {
        *observation = {};
    }
    return false;
}

/**
 * 链接验证模式的指令读取。
 *
 * @param command 输出 disarm 指令。
 * @return 固定返回 false，表示没有有效控制指令。
 */
bool board_read_guidance_command(flight_control::GuidanceCommand* command) {
    if (command) {
        *command = {};
        command->armed = false;
    }
    return false;
}

/**
 * 链接验证模式的 PWM 写入。
 *
 * @param frame 四路 PWM 输出帧，链接验证模式中忽略。
 */
void board_write_motor_pwm(const flight_control::MotorPwmFrame* frame) {
    (void)frame;
}

/**
 * 链接验证模式的风速读取。
 *
 * @param wind_x_m_s 输出 x 方向风速，固定为 0。
 * @param wind_y_m_s 输出 y 方向风速，固定为 0。
 */
void board_read_estimated_wind(float* wind_x_m_s, float* wind_y_m_s) {
    if (wind_x_m_s) {
        *wind_x_m_s = 0.0f;
    }
    if (wind_y_m_s) {
        *wind_y_m_s = 0.0f;
    }
}

/**
 * 链接验证模式的控制链路延迟读取。
 *
 * @return 固定返回 0ms。
 */
float board_control_latency_ms() {
    return 0.0f;
}

/** 链接验证模式的临界区进入，不执行任何操作。 */
void board_enter_critical() {}

/** 链接验证模式的临界区退出，不执行任何操作。 */
void board_exit_critical() {}

}  // extern "C"
