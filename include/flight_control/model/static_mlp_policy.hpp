#pragma once

#include <array>
#include <memory>

#include "flight_control/model/attitude_policy.hpp"

namespace flight_control {

constexpr std::size_t kStaticMlpHiddenDim = 64;
constexpr std::size_t kStaticMlpOutputDim = 3;

struct StaticMlpPolicyWeights {
    std::array<float, kStaticMlpHiddenDim * kModelInputDim> layer1_weight{};
    std::array<float, kStaticMlpHiddenDim> layer1_bias{};
    std::array<float, kStaticMlpHiddenDim * kStaticMlpHiddenDim> layer2_weight{};
    std::array<float, kStaticMlpHiddenDim> layer2_bias{};
    std::array<float, kStaticMlpOutputDim * kStaticMlpHiddenDim> output_weight{};
    std::array<float, kStaticMlpOutputDim> output_bias{};
};

class StaticMlpPolicy final : public IAttitudePolicy {
public:
    explicit StaticMlpPolicy(std::shared_ptr<const StaticMlpPolicyWeights> weights);

    std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) override;

private:
    std::shared_ptr<const StaticMlpPolicyWeights> weights_;
};

}  // namespace flight_control

