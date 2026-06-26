#pragma once

#include <memory>

#include "flight_control/model/model_adapter.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace flight_control {

/**
 * 获取导出的静态 MLP 权重。
 *
 * @return 指向只读静态权重的共享指针，供 StaticMlpPolicy 直接使用。
 */
std::shared_ptr<const StaticMlpPolicyWeights> make_generated_policy_weights();
/**
 * 获取导出模型对应的适配器配置。
 *
 * @return 包含力矩缩放和输入归一化参数的适配器配置。
 */
ModelAdapterConfig generated_model_config();

}  // namespace flight_control
