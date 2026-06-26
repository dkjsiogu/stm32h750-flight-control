#pragma once

#include <array>

#include "flight_control/config.hpp"
#include "flight_control/math/vector_quaternion.hpp"

namespace flight_control {

/**
 * 传感器原始数据包。
 *
 * 保存一次采样得到的 IMU、姿态估计、速度、位置和电池电压。
 * 这个结构用于数据采集层向状态估计层或控制层传递原始观测。
 */
struct SensorPacket {
    /** 采样时间戳，单位秒。 */
    float timestamp_sec{0.0f};
    /** 陀螺仪角速度，单位 rad/s，机体系 xyz。 */
    Vector3 gyro_rad_s{};
    /** 加速度计读数，单位 m/s^2，机体系 xyz。 */
    Vector3 accel_m_s2{};
    /** 姿态四元数，表示机体系到世界系的旋转。 */
    Quaternion attitude{};
    /** 速度估计，单位 m/s，世界系 xyz。 */
    Vector3 velocity_m_s{};
    /** 位置估计，单位 m，世界系 xyz。 */
    Vector3 position_m{};
    /** 电池电压，单位 V。 */
    float battery_voltage_v{0.0f};
};

/**
 * 控制器使用的飞行状态。
 *
 * 这是经过状态估计整理后的当前状态，
 * 控制器只依赖这个结构而不直接依赖具体传感器。
 */
struct VehicleState {
    /** 状态时间戳，单位秒。 */
    float timestamp_sec{0.0f};
    /** 当前位置，单位 m，世界系 xyz。 */
    Vector3 position_m{};
    /** 当前速度，单位 m/s，世界系 xyz。 */
    Vector3 velocity_m_s{};
    /** 当前姿态四元数。 */
    Quaternion attitude{};
    /** 当前机体系角速度，单位 rad/s。 */
    Vector3 angular_velocity_rad_s{};
    /** 当前电池电压，单位 V。 */
    float battery_voltage_v{0.0f};
    /** 状态是否可用于闭环控制；false 时控制器应复位或失能输出。 */
    bool healthy{true};
};

/**
 * 一帧飞控遥测数据。
 *
 * 只包含真实飞控能获得的原始传感器包、估计状态和链路诊断量。
 * 仿真真值不得进入这个结构，外部评估需要单独从评估环境读取真值。
 */
struct FlightTelemetry {
    /** 原始传感器采样数据。 */
    SensorPacket raw{};
    /** 估计后供控制器使用的状态。 */
    VehicleState state{};
    /** 估计风速 x 分量，单位 m/s。 */
    float estimated_wind_x_m_s{0.0f};
    /** 估计风速 y 分量，单位 m/s。 */
    float estimated_wind_y_m_s{0.0f};
    /** 当前控制链路估计延迟，单位 ms。 */
    float control_latency_ms{0.0f};
};

/**
 * 上层导航或人工输入给速度控制器的指令。
 *
 * 水平方向使用目标速度，竖直方向使用爬升率，
 * yaw 使用角速度指令，速度控制器会转换为目标姿态。
 */
struct GuidanceCommand {
    /** 目标水平速度，单位 m/s，世界系 x/y；z 分量保留不用。 */
    Vector3 target_velocity_m_s{};
    /** 目标爬升率，单位 m/s，正值表示上升。 */
    float climb_rate_m_s{0.0f};
    /** 目标 yaw 角速度，单位 rad/s，正负方向按右手系。 */
    float yaw_rate_rad_s{0.0f};
    /** 是否允许控制输出；false 时控制器复位并输出安全默认值。 */
    bool armed{true};
};

/**
 * 速度外环输出的姿态设定值。
 *
 * 神经网络姿态控制器读取目标姿态和当前姿态误差，
 * collective 由力矩混控器转换为总推力。
 */
struct AttitudeSetpoint {
    /** 目标姿态四元数。 */
    Quaternion target_attitude{};
    /** 归一化总推力指令，范围通常为 0 到 1。 */
    float collective{0.0f};
    /** 速度外环期望加速度，单位 m/s^2，世界系 xyz。 */
    Vector3 target_acceleration_m_s2{};
    /** 目标 yaw 角，单位 rad。 */
    float yaw_target_rad{0.0f};
};

/**
 * 姿态模型输出的机体系力矩指令。
 *
 * 该结构是神经网络输出和 PWM 混控之间的边界。
 */
struct TorqueCommand {
    /** 目标机体系力矩，单位 N*m，xyz 分别对应 roll、pitch、yaw 轴。 */
    Vector3 body_torque_nm{};
};

/**
 * 四路电机 PWM 输出帧。
 *
 * 下标顺序由混控器和硬件接线约定保持一致。
 */
struct MotorPwmFrame {
    /** 四个电机的 PWM 脉宽，单位 us。 */
    std::array<float, 4> pwm_us{};
};

/**
 * 神经网络历史窗口中的单帧输入。
 *
 * 一帧包含姿态误差、角速度和上一动作，维度由 kFrameDim 定义。
 */
struct ModelFrame {
    /** 单帧模型输入值。 */
    std::array<float, kFrameDim> values{};
};

/**
 * 一次控制周期的完整解算结果。
 *
 * 用于日志、外部评估、测试和执行器线程之间传递控制链路输出。
 */
struct ControlSolution {
    /** 速度控制器输出的姿态目标。 */
    AttitudeSetpoint attitude_setpoint{};
    /** NN 姿态模型输出的力矩目标。 */
    TorqueCommand torque_command{};
    /** 混控器输出的四路 PWM。 */
    MotorPwmFrame motor_pwm{};
};

}  // namespace flight_control
