#pragma once

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

/**
 * 速度控制器配置。
 *
 * 定义速度外环的机体参数、控制增益、限幅和高度保持参数。
 * 速度控制器会根据这些参数把目标速度转换成目标姿态和总推力。
 */
struct SpeedControllerConfig {
    /** 机体质量，单位 kg，用于把期望加速度换算为 collective。 */
    float mass_kg{1.07508683f};
    /** 最大总推力，单位 N，通常等于四个电机最大推力之和。 */
    float max_total_thrust_n{25.506f};
    /** 水平速度比例增益，用于 x/y 速度误差。 */
    float kp_xy{3.48941946f};
    /** 水平速度积分增益，用于抵消稳定风场或拖曳造成的速度偏差。 */
    float ki_xy{0.349083364f};
    /** 竖直速度比例增益，用于爬升率误差。 */
    float kp_z{6.71798515f};
    /** 竖直速度积分增益，用于抵消负载变化或推力偏差。 */
    float ki_z{0.1435f};
    /** 悬停推力自适应增益，用于把持续竖直速度误差转换为 collective trim。 */
    float hover_thrust_trim_gain{0.0f};
    /** 悬停推力自适应修正限幅，单位为归一化 collective。 */
    float hover_thrust_trim_limit{0.0980889276f};
    /** 高度目标比例增益，把内部目标高度误差转为竖直速度修正。 */
    float kp_altitude_hold{3.83119631f};
    /** 内部高度目标产生的竖直速度修正上限，单位 m/s。 */
    float max_altitude_correction_m_s{2.38851595f};
    /** 水平期望加速度上限，单位 m/s^2。 */
    float max_accel_xy_m_s2{7.8083396f};
    /** 竖直期望加速度上限，单位 m/s^2。 */
    float max_accel_z_m_s2{5.86347628f};
    /** 水平期望加速度变化率上限，单位 m/s^3。 */
    float max_accel_xy_slew_m_s3{7.08005333f};
    /** 竖直期望加速度变化率上限，单位 m/s^3。 */
    float max_accel_z_slew_m_s3{26.5897789f};
    /** 最大倾角限制，单位 rad，防止速度外环要求过大姿态。 */
    float max_tilt_rad{0.740161479f};
    /** 最大爬升率指令限幅，单位 m/s。 */
    float max_climb_rate_m_s{4.0f};
    /** 最大 yaw 角速度指令限幅，单位 rad/s。 */
    float max_yaw_rate_rad_s{2.5f};
    /** 水平速度积分项限幅，防止长期风扰下积分饱和。 */
    float integral_limit_xy{2.0186224f};
    /** 竖直速度积分项限幅，防止负载变化下积分饱和。 */
    float integral_limit_z{7.20086813f};
};

/**
 * 速度外环控制器。
 *
 * 输入当前飞行状态和导航指令，输出目标姿态、collective、目标加速度和 yaw 目标。
 * 下游神经网络姿态控制器只需要跟踪这里生成的目标姿态。
 */
class SpeedController {
public:
    /**
     * 构造速度控制器。
     *
     * @param config 速度控制器参数配置。
     */
    explicit SpeedController(SpeedControllerConfig config = {});

    /**
     * 复位控制器内部状态。
     *
     * @param current_attitude 当前姿态，用于初始化 yaw 目标。
     */
    void reset(const Quaternion& current_attitude = Quaternion{});
    /**
     * 执行一次速度外环更新。
     *
     * @param state 当前飞行状态。
     * @param command 上层速度、爬升率和 yaw 角速度指令。
     * @param dt_sec 控制周期，单位秒。
     * @return 目标姿态和 collective 输出。
     */
    AttitudeSetpoint update(const VehicleState& state, const GuidanceCommand& command, float dt_sec);

private:
    /**
     * 根据期望加速度和 yaw 目标生成姿态四元数。
     *
     * @param acceleration 世界系期望加速度，单位 m/s^2。
     * @param yaw_target_rad 目标 yaw 角，单位 rad。
     * @return 目标姿态四元数。
     */
    Quaternion attitude_from_acceleration(const Vector3& acceleration, float yaw_target_rad) const;

    /** 当前速度控制器配置。 */
    SpeedControllerConfig config_;
    /** 水平速度积分项，x/y 分量有效。 */
    Vector3 integral_xy_{};
    /** 竖直速度积分项。 */
    float integral_z_{0.0f};
    /** 内部 yaw 目标，单位 rad。 */
    float yaw_target_{0.0f};
    /** 上一次输出的目标加速度，用于限制目标姿态变化速度。 */
    Vector3 last_acceleration_m_s2_{};
    /** 根据长期竖直误差估计出的悬停推力修正量。 */
    float hover_thrust_trim_{0.0f};
    /** 由爬升率指令积分出的内部目标高度，单位 m。 */
    float altitude_hold_target_m_{0.0f};
    /** yaw 目标是否已用当前姿态初始化。 */
    bool yaw_initialized_{false};
    /** 内部目标高度是否已用当前高度初始化。 */
    bool altitude_hold_initialized_{false};
};

}  // namespace flight_control
