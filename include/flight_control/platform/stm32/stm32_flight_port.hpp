#pragma once

#include "flight_control/interfaces/flight_interfaces.hpp"
#include "flight_control/runtime/synchronization.hpp"

namespace flight_control {

/**
 * STM32 传感器数据源。
 *
 * 从板级驱动 hook 读取 IMU、姿态估计、速度、位置和电池电压。
 * 该类只属于真实固件端口，不包含任何仿真环境或 host 真值。
 */
class Stm32SensorSource final : public ISensorSource {
public:
    /**
     * 读取一次飞控遥测。
     *
     * @return 由板级驱动和状态估计器提供的遥测数据。
     */
    FlightTelemetry read() override;
};

/**
 * STM32 指令数据源。
 *
 * 从遥控器、上位机、任务规划器或板级 failsafe 逻辑读取速度指令。
 */
class Stm32CommandSource final : public ICommandSource {
public:
    /**
     * 读取一次速度/爬升/yaw 指令。
     *
     * @return 当前飞行指令。
     */
    GuidanceCommand read() override;
};

/**
 * STM32 PWM 输出端口。
 *
 * 将混控后的四路 PWM 写入定时器比较寄存器或电调驱动。
 */
class Stm32PwmOutput final : public IPwmOutput {
public:
    /**
     * 写入一帧电机 PWM。
     *
     * @param frame 四路电机 PWM 帧。
     */
    void write(const MotorPwmFrame& frame) override;
};

/**
 * STM32 临界区。
 *
 * 通过板级 hook 进入和退出 RTOS/中断临界区，用于保护跨任务共享快照。
 */
class Stm32CriticalSection final : public ICriticalSection {
public:
    /** 进入 STM32 临界区。 */
    void lock() override;
    /** 退出 STM32 临界区。 */
    void unlock() override;
};

}  // namespace flight_control

extern "C" {

/**
 * 板级传感器读取 hook。
 *
 * @param packet 输出原始传感器包。
 * @param state 输出状态估计结果。
 * @param estimated_wind_x_m_s 输出估计风速 x 分量，单位 m/s。
 * @param estimated_wind_y_m_s 输出估计风速 y 分量，单位 m/s。
 * @param control_latency_ms 输出控制链路估计延迟，单位 ms。
 */
void flight_control_board_read_telemetry(
    flight_control::SensorPacket* packet,
    flight_control::VehicleState* state,
    float* estimated_wind_x_m_s,
    float* estimated_wind_y_m_s,
    float* control_latency_ms);

/**
 * 板级指令读取 hook。
 *
 * @param command 输出速度、爬升和 yaw 指令。
 */
void flight_control_board_read_command(flight_control::GuidanceCommand* command);

/**
 * 板级 PWM 写入 hook。
 *
 * @param frame 四路电机 PWM 帧。
 */
void flight_control_board_write_pwm(const flight_control::MotorPwmFrame* frame);

/** 进入板级临界区。 */
void flight_control_board_enter_critical();

/** 退出板级临界区。 */
void flight_control_board_exit_critical();

}  // extern "C"
