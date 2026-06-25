#include "flight_control/model/static_mlp_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace flight_control {

namespace {

float relu(float value) {
    return std::max(0.0f, value);
}

template <std::size_t Rows, std::size_t Cols>
std::array<float, Rows> dense_relu(const std::array<float, Rows * Cols>& weight,
                                   const std::array<float, Rows>& bias,
                                   const float* input) {
    std::array<float, Rows> output{};
    for (std::size_t row = 0; row < Rows; ++row) {
        float value = bias[row];
        for (std::size_t col = 0; col < Cols; ++col) {
            value += weight[row * Cols + col] * input[col];
        }
        output[row] = relu(value);
    }
    return output;
}

}  // namespace

StaticMlpPolicy::StaticMlpPolicy(std::shared_ptr<const StaticMlpPolicyWeights> weights)
    : weights_(std::move(weights)) {}

std::array<float, 3> StaticMlpPolicy::predict(const std::array<float, kModelInputDim>& input) {
    if (!weights_) {
        return {0.0f, 0.0f, 0.0f};
    }

    const auto hidden1 = dense_relu<kStaticMlpHiddenDim, kModelInputDim>(
        weights_->layer1_weight,
        weights_->layer1_bias,
        input.data());
    const auto hidden2 = dense_relu<kStaticMlpHiddenDim, kStaticMlpHiddenDim>(
        weights_->layer2_weight,
        weights_->layer2_bias,
        hidden1.data());

    std::array<float, 3> action{};
    for (std::size_t row = 0; row < action.size(); ++row) {
        float value = weights_->output_bias[row];
        for (std::size_t col = 0; col < kStaticMlpHiddenDim; ++col) {
            value += weights_->output_weight[row * kStaticMlpHiddenDim + col] * hidden2[col];
        }
        action[row] = std::tanh(value);
    }
    return action;
}

}  // namespace flight_control
