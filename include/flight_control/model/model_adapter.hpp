#pragma once

#include <memory>

#include "flight_control/data/history_window.hpp"
#include "flight_control/model/attitude_policy.hpp"

namespace flight_control {

/**
 * 神经网络输入归一化参数。
 *
 * 对姿态误差、角速度和上一帧动作分别归一化，参数来自离线随机化训练统计。
 */
struct ModelNormalization {
    /** 姿态误差均值，取四元数误差 xyz 分量。 */
    std::array<float, 3> attitude_error_mean{0.0f, 0.0f, 0.0f};
    /** 姿态误差标准差，取四元数误差 xyz 分量。 */
    std::array<float, 3> attitude_error_std{1.0f, 1.0f, 1.0f};
    /** 角速度均值，单位 rad/s。 */
    std::array<float, 3> omega_mean{0.0f, 0.0f, 0.0f};
    /** 角速度标准差，单位 rad/s。 */
    std::array<float, 3> omega_std{1.0f, 1.0f, 1.0f};
    /** 上一帧归一化动作均值。 */
    std::array<float, 3> previous_action_mean{0.0f, 0.0f, 0.0f};
    /** 上一帧归一化动作标准差。 */
    std::array<float, 3> previous_action_std{1.0f, 1.0f, 1.0f};
    /** 速度外环目标加速度均值，单位 m/s^2。 */
    std::array<float, 3> target_acceleration_mean{0.0f, 0.0f, 0.0f};
    /** 速度外环目标加速度标准差，单位 m/s^2。 */
    std::array<float, 3> target_acceleration_std{4.0f, 4.0f, 4.0f};
    /** collective 均值。 */
    float collective_mean{0.45f};
    /** collective 标准差。 */
    float collective_std{0.20f};
    /** 角加速度均值，单位 rad/s^2。 */
    std::array<float, 3> angular_accel_mean{0.0f, 0.0f, 0.0f};
    /** 角加速度标准差，单位 rad/s^2。 */
    std::array<float, 3> angular_accel_std{12.0f, 12.0f, 9.0f};
};

/**
 * 姿态模型适配器配置。
 *
 * 控制适配器把历史窗口中的状态组织成模型输入，并把模型输出缩放为机体系力矩。
 */
struct ModelAdapterConfig {
    /** 模型输出力矩上限，单位 N*m。 */
    float torque_limit_nm{0.35f};
    /** 目标姿态变化超过该阈值时重置历史窗口。 */
    float target_reset_threshold{0.25f};
    /** 输入归一化参数。 */
    ModelNormalization normalization{};
    /** 安全滤波最大倾角，单位 rad；超过后会抑制水平力矩。 */
    float safety_max_tilt_rad{0.92f};
    /** 安全滤波最大角速度，单位 rad/s；超过后会抑制同向力矩。 */
    float safety_max_angular_rate_rad_s{7.5f};
    /** 安全滤波最小保留力矩比例，避免完全切断闭环控制。 */
    float safety_min_torque_scale{0.28f};
};

/**
 * 姿态神经网络适配器。
 *
 * 负责把当前状态、目标姿态和历史动作组织成模型输入，
 * 再把神经网络输出变换成可直接送给混控器的力矩指令。
 */
class ModelAdapter {
public:
    /**
     * 构造模型适配器。
     *
     * @param policy 姿态策略对象，允许注入神经网络或规则控制器。
     * @param config 适配器配置。
     */
    explicit ModelAdapter(
        std::shared_ptr<IAttitudePolicy> policy = std::make_shared<HeuristicAttitudePolicy>(),
        ModelAdapterConfig config = {});

    /** 重置历史窗口、上一动作和目标记录。 */
    void reset();
    /**
     * 根据当前状态和目标姿态生成力矩指令。
     *
     * @param state 当前飞行状态。
     * @param setpoint 目标姿态设定值。
     * @return 机体系力矩指令。
     */
    TorqueCommand update(const VehicleState& state, const AttitudeSetpoint& setpoint);

    /**
     * 获取最近一次送入模型的输入。
     *
     * @return 展平后的归一化模型输入。
     */
    std::array<float, kModelInputDim> last_input() const;

private:
    /**
     * 生成当前时刻的一帧模型输入。
     *
     * @param state 当前飞行状态。
     * @param setpoint 目标姿态设定值。
     * @return 单帧模型输入。
     */
    ModelFrame build_frame(const VehicleState& state, const AttitudeSetpoint& setpoint) const;
    /**
     * 对神经网络输入做归一化。
     *
     * @param input 原始输入向量。
     * @return 归一化后的输入向量。
     */
    std::array<float, kModelInputDim> normalize_input(const std::array<float, kModelInputDim>& input) const;
    /**
     * 对模型动作执行安全滤波。
     *
     * @param action 模型输出的三轴归一化动作。
     * @param state 当前飞行状态。
     * @return 经过倾角和角速度屏障滤波后的动作。
     */
    std::array<float, 3> apply_safety_filter(std::array<float, 3> action, const VehicleState& state) const;

    /** 注入的姿态策略。 */
    std::shared_ptr<IAttitudePolicy> policy_;
    /** 适配器配置。 */
    ModelAdapterConfig config_;
    /** 历史输入窗口。 */
    HistoryWindow history_;
    /** 最近一次动作，作为输入历史的一部分。 */
    std::array<float, 3> previous_action_{0.0f, 0.0f, 0.0f};
    /** 上一帧角速度，用于估计角加速度。 */
    Vector3 previous_angular_velocity_rad_s_{};
    /** 最近一次目标姿态。 */
    Quaternion last_target_{};
    /** 最近一次目标姿态是否已记录。 */
    bool has_last_target_{false};
    /** 上一帧角速度是否已记录。 */
    bool has_previous_angular_velocity_{false};
};

}  // namespace flight_control
