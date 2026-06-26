#pragma once

#include <array>
#include <memory>

#include "flight_control/model/attitude_policy.hpp"

namespace flight_control {

/** 静态 MLP 单个隐藏层维度；当前导出模型使用 128-128 两层隐藏结构。 */
constexpr std::size_t kStaticMlpHiddenDim = 128;
/** 静态 MLP 输出维度，对应 roll、pitch、yaw 三轴归一化动作。 */
constexpr std::size_t kStaticMlpOutputDim = 3;

/**
 * 静态 MLP 权重集合。
 *
 * 该结构体保存两层隐藏层和一层输出层的全部权重与偏置，
 * 用于在 C++ 中直接执行导出的神经网络推理。
 */
struct StaticMlpPolicyWeights {
    /** 第一层权重，形状为 hidden x input。 */
    std::array<float, kStaticMlpHiddenDim * kModelInputDim> layer1_weight{};
    /** 第一层偏置。 */
    std::array<float, kStaticMlpHiddenDim> layer1_bias{};
    /** 第二层权重，形状为 hidden x hidden。 */
    std::array<float, kStaticMlpHiddenDim * kStaticMlpHiddenDim> layer2_weight{};
    /** 第二层偏置。 */
    std::array<float, kStaticMlpHiddenDim> layer2_bias{};
    /** 输出层权重，形状为 output x hidden。 */
    std::array<float, kStaticMlpOutputDim * kStaticMlpHiddenDim> output_weight{};
    /** 输出层偏置。 */
    std::array<float, kStaticMlpOutputDim> output_bias{};
};

/**
 * 静态多层感知机姿态策略。
 *
 * 使用固定权重在本地执行前向推理，不依赖运行时推理框架。
 * 这是当前项目的主姿态神经网络实现。
 */
class StaticMlpPolicy final : public IAttitudePolicy {
public:
    /**
     * 构造静态 MLP 策略。
     *
     * @param weights 静态权重集合，通常来自导出后的模型文件。
     */
    explicit StaticMlpPolicy(std::shared_ptr<const StaticMlpPolicyWeights> weights);

    /**
     * 执行一次前向推理。
     *
     * @param input 展平后的模型输入。
     * @return 三轴归一化动作。
     */
    std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) override;

private:
    /** 静态权重集合。 */
    std::shared_ptr<const StaticMlpPolicyWeights> weights_;
};

}  // namespace flight_control
