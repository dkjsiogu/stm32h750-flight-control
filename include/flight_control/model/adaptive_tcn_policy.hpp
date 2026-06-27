#pragma once

#include <array>
#include <memory>

#include "flight_control/model/attitude_policy.hpp"

namespace flight_control {

/** 自适应 TCN 姿态策略参数数量；包含时间卷积核、门控项、RMA encoder 和四层 dilated residual TCN 分支。 */
constexpr std::size_t kAdaptiveTcnParameterCount = 363;
/** 自适应 TCN 输出维度，对应 roll、pitch、yaw 三轴归一化动作。 */
constexpr std::size_t kAdaptiveTcnOutputDim = 3;
/** RMA 隐变量维度，用于表达风、载荷、执行器滞后和未建模扰动的历史估计。 */
constexpr std::size_t kAdaptiveTcnLatentDim = 8;

/**
 * 自适应 TCN 策略权重。
 *
 * 该结构保存离线搜索得到的全部可导出参数。参数不包含任何仿真真值，
 * 固件运行时只根据真实历史输入估计 latent 并输出机体系力矩动作。
 */
struct AdaptiveTcnPolicyWeights {
    /** 策略参数数组，顺序由仿真仓库导出器和搜索器共同约定。 */
    std::array<float, kAdaptiveTcnParameterCount> params{};
};

/**
 * RMA 自适应 TCN 姿态策略。
 *
 * 策略先从历史误差、角速度和上一动作中编码 latent，再用宽历史因果核、
 * 分段 ReLU 门控和四层 dilated residual TCN 分支输出归一化姿态力矩。
 * 它是当前固件主神经网络策略。
 */
class AdaptiveTcnPolicy final : public IAttitudePolicy {
public:
    /**
     * 构造自适应 TCN 策略。
     *
     * @param weights 离线导出的 TCN/RMA 参数。
     */
    explicit AdaptiveTcnPolicy(std::shared_ptr<const AdaptiveTcnPolicyWeights> weights);

    /**
     * 执行一次前向推理。
     *
     * @param input 展平后的历史窗口输入。
     * @return 三轴归一化动作，范围通常为 [-1, 1]。
     */
    std::array<float, kAdaptiveTcnOutputDim> predict(const std::array<float, kModelInputDim>& input) override;

private:
    /**
     * 估计 RMA latent。
     *
     * @param input 展平后的历史窗口输入。
     * @return latent 向量，表达历史扰动、执行器滞后和振荡强度。
     */
    std::array<float, kAdaptiveTcnLatentDim> encode_latent(const std::array<float, kModelInputDim>& input) const;

    /** 离线导出的策略权重。 */
    std::shared_ptr<const AdaptiveTcnPolicyWeights> weights_;
};

}  // namespace flight_control
