#pragma once

#include <memory>

#include "flight_control/model/adaptive_tcn_policy.hpp"
#include "flight_control/model/model_adapter.hpp"

namespace flight_control {

/**
 * 获取导出的自适应 TCN 权重。
 *
 * @return 指向只读 TCN/RMA 权重的共享指针，供 AdaptiveTcnPolicy 直接使用。
 */
std::shared_ptr<const AdaptiveTcnPolicyWeights> make_generated_tcn_policy_weights();
/**
 * 创建导出的主姿态策略。
 *
 * @return 自适应 TCN 姿态策略实例。
 */
std::shared_ptr<IAttitudePolicy> make_generated_policy();
/**
 * 获取导出模型对应的适配器配置。
 *
 * @return 包含力矩缩放和输入归一化参数的适配器配置。
 */
ModelAdapterConfig generated_model_config();

}  // namespace flight_control
