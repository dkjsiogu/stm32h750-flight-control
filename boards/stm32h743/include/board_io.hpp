#pragma once

#include "flight_control/estimation/state_estimator.hpp"
#include "flight_control/interfaces/flight_interfaces.hpp"

extern "C" {

/**
 * 板级早期系统初始化。
 *
 * 在 `SystemInit` 中调用，真实驱动应在这里配置 HSE/PLL、Flash latency、
 * Cache、MPU 和中断优先级分组。此函数早于 `main`，不要依赖动态分配。
 */
void board_system_preinit();

/**
 * 初始化全部真实板级驱动。
 *
 * 真实实现应完成 GPIO、SPI/I2C/UART、DMA、TIM、ADC、IMU、接收机和电机输出初始化。
 *
 * @return true 表示必需驱动已经就绪，可以启动飞控任务。
 */
bool board_drivers_initialize();

/**
 * 写入安全电机输出。
 *
 * 真实实现应立即输出锁定/最小油门状态，用于启动前和异常时保护电机链路。
 */
void board_write_failsafe_outputs();

/**
 * 读取真实 IMU 和电池数据。
 *
 * @param packet 输出原始传感器数据包。
 * @return true 表示本次 IMU 数据有效。
 */
bool board_read_imu_sample(flight_control::SensorPacket* packet);

/**
 * 读取外部状态观测。
 *
 * @param observation 输出外部姿态、速度、位置观测。
 * @return true 表示至少一个观测字段有效。
 */
bool board_read_external_observation(flight_control::StateEstimatorObservation* observation);

/**
 * 读取上层控制指令。
 *
 * @param command 输出速度、爬升率和 yaw 角速度指令。
 * @return true 表示指令有效且允许继续控制。
 */
bool board_read_guidance_command(flight_control::GuidanceCommand* command);

/**
 * 写四路电机 PWM。
 *
 * @param frame 四路 PWM 输出帧。
 */
void board_write_motor_pwm(const flight_control::MotorPwmFrame* frame);

/**
 * 读取估计风速。
 *
 * @param wind_x_m_s 输出世界系 x 方向风速，单位 m/s。
 * @param wind_y_m_s 输出世界系 y 方向风速，单位 m/s。
 */
void board_read_estimated_wind(float* wind_x_m_s, float* wind_y_m_s);

/**
 * 读取控制链路估计延迟。
 *
 * @return 控制链路延迟，单位 ms。
 */
float board_control_latency_ms();

/** 进入板级临界区。 */
void board_enter_critical();

/** 退出板级临界区。 */
void board_exit_critical();

}  // extern "C"
