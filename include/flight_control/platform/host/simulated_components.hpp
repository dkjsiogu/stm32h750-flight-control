#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

#include "flight_control/interfaces/flight_interfaces.hpp"

namespace flight_control {

/**
 * 主机侧四旋翼仿真参数。
 *
 * 这些参数定义了简化但可闭环的动力学模型，包括质量、力臂、电机响应、
 * 气动阻尼和风场，用于验证控制链路在延迟和扰动下的效果。
 */
struct PlantConfig {
    /** 飞机总质量，单位 kg。 */
    float mass_kg{1.0f};
    /** 电机到质心的力臂长度，单位 m。 */
    float arm_length_m{0.23f};
    /** 偏航力矩系数。 */
    float yaw_torque_coeff{0.018f};
    /** 最大总推力，单位 N。 */
    float max_total_thrust_n{25.506f};
    /** 单个电机最大推力，单位 N。 */
    float max_motor_thrust_n{6.3765f};
    /** 电机一阶响应时间常数，单位秒。 */
    float motor_tau_sec{0.045f};
    /** 角速度阻尼系数。 */
    float angular_drag{0.018f};
    /** 线性阻力系数。 */
    float linear_drag{0.18f};
    /** 二次阻力系数。 */
    float quadratic_drag{0.035f};
    /** 风速，单位 m/s，世界系 xyz。 */
    Vector3 wind_m_s{0.6f, -0.2f, 0.0f};
};

/**
 * 主机侧四旋翼仿真体。
 *
 * 它接收 PWM 输入，内部更新电机状态、角速度、姿态、速度和位置，
 * 然后通过 telemetry() 输出仿真遥测数据。
 */
class SimulatedQuadPlant {
public:
    /**
     * 构造仿真体。
     *
     * @param config 物理和扰动参数配置。
     */
    explicit SimulatedQuadPlant(PlantConfig config = {});

    /**
     * 推进仿真一步。
     *
     * @param pwm 四路电机 PWM 帧。
     * @param dt_sec 仿真步长，单位秒。
     */
    void step(const MotorPwmFrame& pwm, float dt_sec);
    /**
     * 动态设置风场。
     *
     * @param wind_m_s 新风速，单位 m/s。
     */
    void set_wind(const Vector3& wind_m_s);
    /**
     * 获取当前遥测。
     *
     * @return 仿真体生成的遥测数据。
     */
    FlightTelemetry telemetry() const;
    /**
     * 获取当前状态。
     *
     * @return 仿真体内部状态。
     */
    VehicleState state() const;
    /**
     * 获取上一帧 PWM。
     *
     * @return 最近一次写入的 PWM。
     */
    MotorPwmFrame last_pwm() const;

private:
    /**
     * 将 PWM 脉宽转换为推力。
     *
     * @param pwm 四路电机 PWM 帧。
     * @return 四个电机对应的推力数组。
     */
    std::array<float, 4> pwm_to_thrust(const MotorPwmFrame& pwm) const;

    /** 仿真体状态互斥锁。 */
    mutable std::mutex mutex_;
    /** 仿真配置。 */
    PlantConfig config_;
    /** 当前飞行状态。 */
    VehicleState state_{};
    /** 最近一次 PWM 输出。 */
    MotorPwmFrame last_pwm_{};
    /** 电机一阶滞后内部状态。 */
    std::array<float, 4> motor_state_{};
    /** 陀螺仪零偏。 */
    Vector3 gyro_bias_{0.0f, 0.0f, 0.0f};
    /** 仿真时间戳。 */
    float timestamp_sec_{0.0f};
};

/**
 * 仿真传感器源。
 *
 * 从 SimulatedQuadPlant 读取遥测并向控制器侧暴露为传感器接口。
 */
class MockSensorSource final : public ISensorSource {
public:
    /**
     * 构造仿真传感器源。
     *
     * @param plant 仿真体共享指针。
     */
    explicit MockSensorSource(std::shared_ptr<SimulatedQuadPlant> plant);
    /**
     * 读取一次遥测。
     *
     * @return 仿真遥测数据。
     */
    FlightTelemetry read() override;

private:
    /** 被读取的仿真体。 */
    std::shared_ptr<SimulatedQuadPlant> plant_;
};

/**
 * 仿真指令源。
 *
 * 按时间生成一组固定的速度、爬升和 yaw 指令，用于演示和回归测试。
 */
class MockCommandSource final : public ICommandSource {
public:
    /** 构造仿真指令源。 */
    MockCommandSource();
    /**
     * 读取当前时间对应的指令。
     *
     * @return 当前仿真阶段的指导命令。
     */
    GuidanceCommand read() override;

private:
    /** 记录起始时间。 */
    std::chrono::steady_clock::time_point start_;
};

/**
 * 仿真 PWM 输出。
 *
 * 把控制器输出的 PWM 直接写入 SimulatedQuadPlant，形成闭环。
 */
class MockPwmOutput final : public IPwmOutput {
public:
    /**
     * 构造仿真 PWM 输出。
     *
     * @param plant 仿真体共享指针。
     * @param dt_sec 仿真步长，单位秒。
     */
    MockPwmOutput(std::shared_ptr<SimulatedQuadPlant> plant, float dt_sec = kDefaultControlPeriodSec);
    /**
     * 写入一帧 PWM。
     *
     * @param frame 四路电机 PWM 帧。
     */
    void write(const MotorPwmFrame& frame) override;

private:
    /** 被驱动的仿真体。 */
    std::shared_ptr<SimulatedQuadPlant> plant_;
    /** PWM 写入时使用的仿真步长。 */
    float dt_sec_{kDefaultControlPeriodSec};
};

}  // namespace flight_control
