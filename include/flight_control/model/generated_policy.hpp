#pragma once

#include <memory>

#include "flight_control/model/model_adapter.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace flight_control {

std::shared_ptr<const StaticMlpPolicyWeights> make_generated_policy_weights();
ModelAdapterConfig generated_model_config();

}  // namespace flight_control
