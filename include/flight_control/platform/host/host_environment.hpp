#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

#include "flight_control/interfaces/flight_interfaces.hpp"

namespace flight_control {

/**
 * 主机侧真实环境替身配置。
 *
 * 这些参数描述飞机、传感器、执行链路和外界扰动。host 端把这些因素当成真实存在，
 * 飞控系统只能通过传感器接口读取估计状态，并通过 PWM 接口驱动电机。
 */
struct HostEnvironmentConfig {
    /** 飞机总质量，单位 kg。 */
    float mass_kg{1.0f};
    /** 额外挂载质量，单位 kg，用于重物或载荷变化场景。 */
    float payload_mass_kg{0.0f};
    /** 电机到质心的力臂长度，单位 m。 */
    float arm_length_m{0.23f};
    /** 偏航力矩系数，用于电机反扭矩计算。 */
    float yaw_torque_coeff{0.018f};
    /** 单个电机标称最大推力，单位 N。 */
    float max_motor_thrust_n{6.3765f};
    /** 机体转动惯量，单位 kg*m^2。 */
    Vector3 inertia_kg_m2{0.024f, 0.024f, 0.045f};
    /** 四个电机一阶响应时间常数，单位秒。 */
    std::array<float, 4> motor_tau_sec{0.044f, 0.049f, 0.047f, 0.052f};
    /** 四个电机推力比例误差，用于模拟电机和桨叶个体差异。 */
    std::array<float, 4> motor_thrust_scale{1.00f, 0.985f, 1.015f, 0.992f};
    /** PWM 最小脉宽，单位 us。 */
    float pwm_min_us{1000.0f};
    /** PWM 最大脉宽，单位 us。 */
    float pwm_max_us{2000.0f};
    /** PWM 到推力映射指数，大于 1 表示低油门段响应更弱。 */
    float pwm_thrust_exponent{1.18f};
    /** 角速度阻尼系数。 */
    float angular_drag{0.021f};
    /** 线性阻力系数。 */
    float linear_drag{0.22f};
    /** 二次阻力系数。 */
    float quadratic_drag{0.046f};
    /** 侧风导致的姿态耦合力矩系数。 */
    float wind_torque_coeff{0.0035f};
    /** 基础风速，单位 m/s，世界系 xyz。 */
    Vector3 wind_m_s{0.6f, -0.2f, 0.0f};
    /** 阵风幅值，单位 m/s，叠加在基础风速上。 */
    Vector3 gust_amplitude_m_s{0.22f, 0.16f, 0.03f};
    /** 标称满电电压，单位 V。 */
    float battery_full_voltage_v{16.4f};
    /** 电池内阻等效系数，单位 V/N，用于推力越大电压越低。 */
    float battery_sag_v_per_n{0.019f};
    /** 陀螺仪初始零偏，单位 rad/s。 */
    Vector3 gyro_bias_rad_s{0.006f, -0.004f, 0.0025f};
    /** 陀螺仪零偏漂移幅值，单位 rad/s。 */
    Vector3 gyro_drift_rad_s{0.0025f, 0.0018f, 0.0012f};
    /** 姿态估计慢漂幅值，单位 rad。 */
    Vector3 attitude_drift_rad{0.003f, -0.0025f, 0.006f};
    /** 速度估计偏差幅值，单位 m/s。 */
    Vector3 velocity_bias_m_s{0.025f, -0.018f, 0.012f};
    /** 位置估计偏差幅值，单位 m。 */
    Vector3 position_bias_m{0.018f, -0.015f, 0.012f};
    /** 传感器到控制器估计链路延迟，单位秒。 */
    float sensor_latency_sec{0.024f};
};

/**
 * 主机侧真实环境替身。
 *
 * 该类内部维护飞机真实状态、电机响应、电池压降、风场、传感器延迟和估计偏差。
 * 飞控系统不直接控制它，只通过 HostSensorSource 和 HostPwmOutput 与环境闭环。
 */
class HostFlightEnvironment {
public:
    /**
     * 构造主机侧真实环境。
     *
     * @param config 主机环境参数配置。
     */
    explicit HostFlightEnvironment(HostEnvironmentConfig config = {});

    /**
     * 推进真实环境一步。
     *
     * @param pwm 四路电机 PWM 帧。
     * @param dt_sec 环境积分步长，单位秒。
     */
    void step(const MotorPwmFrame& pwm, float dt_sec);
    /**
     * 动态设置基础风场。
     *
     * @param wind_m_s 新基础风速，单位 m/s。
     */
    void set_wind(const Vector3& wind_m_s);
    /**
     * 读取当前传感器遥测。
     *
     * @return 包含原始传感器、控制器估计状态和真实状态的遥测。
     */
    FlightTelemetry telemetry() const;
    /**
     * 获取主机环境内部真实状态。
     *
     * @return 未经过传感器延迟和估计误差处理的真实飞行状态。
     */
    VehicleState truth_state() const;
    /**
     * 获取上一帧 PWM。
     *
     * @return 最近一次写入环境的 PWM。
     */
    MotorPwmFrame last_pwm() const;

private:
    /**
     * 将 PWM 脉宽转换为四个电机目标推力。
     *
     * @param pwm 四路电机 PWM 帧。
     * @return 四个电机对应的目标推力数组。
     */
    std::array<float, 4> pwm_to_thrust(const MotorPwmFrame& pwm) const;
    /**
     * 计算当前真实风速。
     *
     * @param time_sec 当前环境时间，单位秒。
     * @return 基础风叠加阵风后的世界系风速。
     */
    Vector3 wind_at(float time_sec) const;
    /**
     * 根据历史真值采样选择延迟后的状态。
     *
     * @param delayed_time_sec 传感器链路延迟后的目标时间，单位秒。
     * @return 延迟后的真实状态样本。
     */
    VehicleState delayed_truth(float delayed_time_sec) const;
    /**
     * 生成带偏差的估计状态。
     *
     * @param truth 延迟后的真实状态。
     * @return 控制器实际可见的估计状态。
     */
    VehicleState estimated_state_from_truth(const VehicleState& truth) const;

    /** 主机环境状态互斥锁。 */
    mutable std::mutex mutex_;
    /** 主机环境配置。 */
    HostEnvironmentConfig config_;
    /** 当前真实飞行状态。 */
    VehicleState truth_{};
    /** 最近一次 PWM 输出。 */
    MotorPwmFrame last_pwm_{};
    /** 电机一阶滞后内部推力状态。 */
    std::array<float, 4> motor_state_{};
    /** 历史真实状态采样，用于传感器延迟。 */
    std::array<VehicleState, 64> truth_history_{};
    /** 历史真实状态循环写入位置。 */
    std::size_t history_head_{0};
    /** 历史真实状态有效样本数量。 */
    std::size_t history_size_{0};
    /** 当前环境时间戳，单位秒。 */
    float timestamp_sec_{0.0f};
};

/**
 * 主机侧传感器源。
 *
 * 从 HostFlightEnvironment 读取带真实延迟、偏差和噪声的遥测，
 * 对飞控应用表现为真实传感器数据源。
 */
class HostSensorSource final : public ISensorSource {
public:
    /**
     * 构造主机侧传感器源。
     *
     * @param environment 主机环境共享指针。
     */
    explicit HostSensorSource(std::shared_ptr<HostFlightEnvironment> environment);
    /**
     * 读取一次遥测。
     *
     * @return 当前传感器遥测。
     */
    FlightTelemetry read() override;

private:
    /** 被读取的主机环境。 */
    std::shared_ptr<HostFlightEnvironment> environment_;
};

/**
 * 脚本化导航指令源。
 *
 * 按主机时间生成一组固定的速度、爬升和 yaw 指令，
 * 用于 host 演示和端到端回归。
 */
class ScriptedCommandSource final : public ICommandSource {
public:
    /** 构造脚本化导航指令源。 */
    ScriptedCommandSource();
    /**
     * 读取当前时间对应的指令。
     *
     * @return 当前阶段的指导命令。
     */
    GuidanceCommand read() override;

private:
    /** 指令脚本起始时间。 */
    std::chrono::steady_clock::time_point start_;
};

/**
 * 主机侧 PWM 输出。
 *
 * 把飞控输出的 PWM 写入 HostFlightEnvironment，
 * 对飞控应用表现为真实电调和电机执行器。
 */
class HostPwmOutput final : public IPwmOutput {
public:
    /**
     * 构造主机侧 PWM 输出。
     *
     * @param environment 主机环境共享指针。
     * @param dt_sec 环境步进周期，单位秒。
     */
    HostPwmOutput(std::shared_ptr<HostFlightEnvironment> environment, float dt_sec = kDefaultControlPeriodSec);
    /**
     * 写入一帧 PWM。
     *
     * @param frame 四路电机 PWM 帧。
     */
    void write(const MotorPwmFrame& frame) override;

private:
    /** 被驱动的主机环境。 */
    std::shared_ptr<HostFlightEnvironment> environment_;
    /** PWM 写入时使用的环境步进周期。 */
    float dt_sec_{kDefaultControlPeriodSec};
};

}  // namespace flight_control
