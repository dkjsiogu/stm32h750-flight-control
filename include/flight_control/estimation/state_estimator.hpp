#pragma once

#include <array>

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

/**
 * 外部状态观测。
 *
 * 板级代码可以把视觉、气压计、GPS、光流或外部 AHRS 的观测填入这里。
 * 每个 valid 标志独立控制对应观测是否参与状态估计。
 */
struct StateEstimatorObservation {
    /** 姿态观测是否有效。 */
    bool attitude_valid{false};
    /** 外部姿态观测，表示机体系到世界系的旋转。 */
    Quaternion attitude{};
    /** yaw 观测是否有效，通常来自磁力计、双天线 GPS 或视觉航向。 */
    bool yaw_valid{false};
    /** 外部 yaw 观测，单位 rad，世界系 z 轴航向角。 */
    float yaw_rad{0.0f};
    /** 速度观测是否有效。 */
    bool velocity_valid{false};
    /** 外部速度观测，单位 m/s，世界系 xyz。 */
    Vector3 velocity_m_s{};
    /** 位置观测是否有效。 */
    bool position_valid{false};
    /** 外部位置观测，单位 m，世界系 xyz。 */
    Vector3 position_m{};
};

/**
 * 状态估计器配置。
 *
 * 这些参数控制 IMU 积分、加速度重力方向校正和外部观测融合速度。
 */
struct StateEstimatorConfig {
    /** 允许使用的最大积分步长，单位秒。 */
    float max_dt_sec{0.02f};
    /** 加速度重力方向对 roll/pitch 的修正增益。 */
    float accel_correction_gain{1.6f};
    /** 外部姿态观测融合增益。 */
    float attitude_observation_gain{0.08f};
    /** 姿态观测对陀螺仪零偏的误差状态学习增益。 */
    float gyro_bias_observation_gain{0.018f};
    /** 外部速度观测融合增益。 */
    float velocity_observation_gain{0.18f};
    /** 速度观测对世界系加速度慢偏的误差状态学习增益。 */
    float accel_bias_observation_gain{0.045f};
    /** 外部位置观测融合增益。 */
    float position_observation_gain{0.08f};
    /** 静止附近陀螺仪零偏学习增益。 */
    float gyro_bias_learning_gain{0.018f};
    /** 姿态误差状态过程噪声，数值越大表示更信任外部姿态观测。 */
    float attitude_process_noise{0.0018f};
    /** 速度误差状态过程噪声，数值越大表示速度协方差增长越快。 */
    float velocity_process_noise{0.035f};
    /** 位置误差状态过程噪声，数值越大表示位置协方差增长越快。 */
    float position_process_noise{0.012f};
    /** 姿态观测方差，用于 NIS 创新降权。 */
    float attitude_observation_variance{0.035f};
    /** yaw 观测方差，用于 NIS 创新降权。 */
    float yaw_observation_variance{0.055f};
    /** 速度观测方差，用于 NIS 创新降权。 */
    float velocity_observation_variance{0.20f};
    /** 位置观测方差，用于 NIS 创新降权。 */
    float position_observation_variance{0.45f};
    /** NIS 软门限，超过该值的观测会被降权而不是硬丢弃。 */
    float nis_gate{9.0f};
    /** 观测被判定为异常时保留的最小融合权重。 */
    float min_observation_weight{0.08f};
    /** 判断静止附近的陀螺仪角速度阈值，单位 rad/s。 */
    float gyro_stationary_threshold_rad_s{0.08f};
    /** 允许使用加速度重力方向的最小模长，单位 m/s^2。 */
    float accel_gravity_min_m_s2{0.55f * kGravity};
    /** 允许使用加速度重力方向的最大模长，单位 m/s^2。 */
    float accel_gravity_max_m_s2{1.45f * kGravity};
};

/**
 * 飞控状态估计器。
 *
 * 使用陀螺仪积分姿态，用加速度计修正 roll/pitch，并融合可选外部姿态、
 * 速度和位置观测。该类属于固件核心，不依赖仿真真值。
 */
class StateEstimator {
public:
    /**
     * 构造状态估计器。
     *
     * @param config 状态估计器配置。
     */
    explicit StateEstimator(StateEstimatorConfig config = {});

    /** 重置内部状态和初始化标志。 */
    void reset();
    /**
     * 使用一帧原始传感器和外部观测更新估计状态。
     *
     * @param packet 原始传感器数据。
     * @param observation 外部状态观测。
     * @return 控制器使用的估计飞行状态。
     */
    VehicleState update(const SensorPacket& packet, const StateEstimatorObservation& observation);
    /**
     * 获取最近一次估计状态。
     *
     * @return 当前估计状态。
     */
    VehicleState state() const;

private:
    /**
     * 初始化估计器内部状态。
     *
     * @param packet 原始传感器数据。
     * @param observation 外部状态观测。
     */
    void initialize(const SensorPacket& packet, const StateEstimatorObservation& observation);
    /**
     * 使用右不变误差融合外部姿态观测。
     *
     * @param observed_attitude 外部姿态观测。
     * @param dt 积分步长，单位秒。
     */
    void apply_invariant_attitude_observation(const Quaternion& observed_attitude, float dt);
    /**
     * 使用 yaw 观测修正当前姿态航向。
     *
     * @param yaw_rad 外部 yaw 观测，单位 rad。
     * @param dt 积分步长，单位秒。
     */
    void apply_yaw_observation(float yaw_rad, float dt);
    /**
     * 根据创新和方差计算 NIS 风格软权重。
     *
     * @param innovation_sq 创新平方和。
     * @param variance 观测方差。
     * @param dimension 观测维度。
     * @return 0 到 1 的观测权重。
     */
    float observation_weight(float innovation_sq, float variance, float dimension) const;
    /**
     * 推进误差状态协方差对角线。
     *
     * @param dt 积分步长，单位秒。
     */
    void propagate_covariance(float dt);
    /**
     * 检查当前内部状态是否为有限值。
     *
     * @return true 表示状态可用于闭环控制。
     */
    bool state_is_finite() const;
    /**
     * 检查原始 IMU 是否可用于闭环。
     *
     * @param packet 原始传感器数据。
     * @return true 表示 IMU 数据有限且加速度模长处于合理范围。
     */
    bool sensor_is_usable(const SensorPacket& packet) const;

    /** 状态估计器配置。 */
    StateEstimatorConfig config_{};
    /** 当前估计状态。 */
    VehicleState state_{};
    /** 当前估计陀螺仪零偏，单位 rad/s。 */
    Vector3 gyro_bias_rad_s_{};
    /** 当前估计机体系加速度慢偏，单位 m/s^2。 */
    Vector3 accel_bias_body_m_s2_{};
    /** 右不变误差状态协方差对角线，顺序为姿态、速度、位置。 */
    std::array<float, 9> covariance_diag_{};
    /** 上一次传感器时间戳，单位秒。 */
    float last_timestamp_sec_{0.0f};
    /** 是否已完成初始化。 */
    bool initialized_{false};
};

}  // namespace flight_control
