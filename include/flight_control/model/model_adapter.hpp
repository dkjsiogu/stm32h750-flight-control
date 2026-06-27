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

    /** 注入的姿态策略。 */
    std::shared_ptr<IAttitudePolicy> policy_;
    /** 适配器配置。 */
    ModelAdapterConfig config_;
    /** 历史输入窗口。 */
    HistoryWindow history_;
    /** 最近一次动作，作为输入历史的一部分。 */
    std::array<float, 3> previous_action_{0.0f, 0.0f, 0.0f};
    /** 最近一次目标姿态。 */
    Quaternion last_target_{};
    /** 最近一次目标姿态是否已记录。 */
    bool has_last_target_{false};
};

}  // namespace flight_control
