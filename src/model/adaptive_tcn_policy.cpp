#include "flight_control/model/adaptive_tcn_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace flight_control {

namespace {

/** 姿态误差历史抽头数量。 */
constexpr std::size_t kErrorTapCount = 12U;
/** 角速度历史抽头数量。 */
constexpr std::size_t kOmegaTapCount = 10U;
/** 上一动作历史抽头数量。 */
constexpr std::size_t kActionTapCount = 6U;
/** 每轴线性 TCN 参数数量。 */
constexpr std::size_t kLinearParamsPerAxis = kErrorTapCount + kOmegaTapCount + kActionTapCount + 2U;
/** 每轴非线性门控参数数量。 */
constexpr std::size_t kNonlinearParamsPerAxis = 20U;
/** 每轴姿态误差卷积核起始下标。 */
constexpr std::size_t kErrorOffset = 0U;
/** 每轴角速度卷积核起始下标。 */
constexpr std::size_t kOmegaOffset = kErrorOffset + kErrorTapCount;
/** 每轴上一动作卷积核起始下标。 */
constexpr std::size_t kActionOffset = kOmegaOffset + kOmegaTapCount;
/** 每轴卷积内偏置下标。 */
constexpr std::size_t kLinearBiasOffset = kActionOffset + kActionTapCount;
/** 每轴输出偏置下标。 */
constexpr std::size_t kOutputBiasOffset = kLinearBiasOffset + 1U;
/** 时间卷积全局缩放参数下标。 */
constexpr std::size_t kLinearScaleIndex = kLinearParamsPerAxis * 3U;
/** 门控参数起始下标。 */
constexpr std::size_t kGateStartIndex = kLinearScaleIndex + 1U;
/** 非线性门控全局缩放参数下标。 */
constexpr std::size_t kNonlinearScaleIndex = kGateStartIndex + kNonlinearParamsPerAxis * 3U;

/**
 * 读取指定历史帧的指定值。
 *
 * @param input 展平后的历史输入。
 * @param frame_index 历史帧下标，从旧到新。
 * @param value_index 单帧内下标。
 * @return 对应输入值。
 */
float frame_value(const std::array<float, kModelInputDim>& input, std::size_t frame_index, std::size_t value_index) {
    return input[frame_index * kFrameDim + value_index];
}

/**
 * 将从最新帧向前数的偏移转换为历史帧下标。
 *
 * @param offset_from_latest 距最新帧的偏移。
 * @return 历史帧下标。
 */
std::size_t back(std::size_t offset_from_latest) {
    return kHistoryFrames - 1U - offset_from_latest;
}

/**
 * 计算带死区绝对值门控。
 *
 * @param value 原始值。
 * @param threshold 死区阈值。
 * @return max(value-threshold,0)+max(-value-threshold,0)。
 */
float dead_abs(float value, float threshold) {
    const float limit = std::max(0.0f, threshold);
    return std::max(0.0f, value - limit) + std::max(0.0f, -value - limit);
}

/**
 * 限制 RMA latent 到可控范围。
 *
 * @param value 原始 latent。
 * @return 限幅后的 latent。
 */
float latent_squash(float value) {
    return std::tanh(std::clamp(value, -4.0f, 4.0f));
}

}  // namespace

AdaptiveTcnPolicy::AdaptiveTcnPolicy(std::shared_ptr<const AdaptiveTcnPolicyWeights> weights)
    : weights_(std::move(weights)) {}

std::array<float, kAdaptiveTcnOutputDim> AdaptiveTcnPolicy::predict(const std::array<float, kModelInputDim>& input) {
    if (!weights_) {
        return {0.0f, 0.0f, 0.0f};
    }

    const auto& params = weights_->params;
    const auto latent = encode_latent(input);

    std::array<float, kAdaptiveTcnOutputDim> action{};
    for (std::size_t axis = 0; axis < action.size(); ++axis) {
        const std::size_t offset = axis * kLinearParamsPerAxis;
        const float error_now = frame_value(input, back(0U), axis);
        const float omega_now = frame_value(input, back(0U), axis + 3U);
        const float previous_action = frame_value(input, back(0U), axis + 6U);
        const float error_delta = error_now - frame_value(input, back(1U), axis);
        const float omega_delta = omega_now - frame_value(input, back(1U), axis + 3U);
        const float action_delta = previous_action - frame_value(input, back(1U), axis + 6U);

        float temporal = params[offset + kLinearBiasOffset];
        for (std::size_t tap = 0; tap < kErrorTapCount; ++tap) {
            temporal += params[offset + kErrorOffset + tap] * frame_value(input, back(tap), axis);
        }
        for (std::size_t tap = 0; tap < kOmegaTapCount; ++tap) {
            temporal += params[offset + kOmegaOffset + tap] * frame_value(input, back(tap), axis + 3U);
        }
        for (std::size_t tap = 0; tap < kActionTapCount; ++tap) {
            temporal += params[offset + kActionOffset + tap] * frame_value(input, back(tap), axis + 6U);
        }

        const std::size_t gate_offset = kGateStartIndex + axis * kNonlinearParamsPerAxis;
        const float nonlinear =
            params[gate_offset + 3U] * dead_abs(error_now, params[gate_offset + 0U]) +
            params[gate_offset + 4U] * dead_abs(omega_now, params[gate_offset + 1U]) +
            params[gate_offset + 5U] * dead_abs(previous_action, params[gate_offset + 2U]) +
            params[gate_offset + 7U] * dead_abs(error_now, params[gate_offset + 6U]) +
            params[gate_offset + 9U] * dead_abs(omega_now, params[gate_offset + 8U]) +
            params[gate_offset + 11U] * dead_abs(previous_action, params[gate_offset + 10U]) +
            params[gate_offset + 12U] +
            params[gate_offset + 14U] * dead_abs(error_delta, params[gate_offset + 13U]) +
            params[gate_offset + 16U] * dead_abs(omega_delta, params[gate_offset + 15U]) +
            params[gate_offset + 18U] * dead_abs(action_delta, params[gate_offset + 17U]);

        const float adaptation =
            params[gate_offset + 19U] *
            (0.58f * latent[axis] + 0.26f * latent[axis + 3U] - 0.16f * latent[6U] + 0.10f * latent[7U]);

        action[axis] = std::tanh(
            params[kLinearScaleIndex] * temporal +
            params[offset + kOutputBiasOffset] +
            params[kNonlinearScaleIndex] * nonlinear +
            adaptation);
    }
    return action;
}

std::array<float, kAdaptiveTcnLatentDim> AdaptiveTcnPolicy::encode_latent(
    const std::array<float, kModelInputDim>& input) const {
    std::array<float, kAdaptiveTcnLatentDim> latent{};
    std::array<float, 3> error_mean{};
    std::array<float, 3> omega_mean{};
    std::array<float, 3> action_mean{};
    std::array<float, 3> error_delta_mean{};
    std::array<float, 3> omega_delta_mean{};
    std::array<float, 3> action_delta_mean{};

    constexpr float inv_window = 1.0f / static_cast<float>(kHistoryFrames - 1U);
    for (std::size_t frame = 0; frame < kHistoryFrames; ++frame) {
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            error_mean[axis] += frame_value(input, frame, axis);
            omega_mean[axis] += frame_value(input, frame, axis + 3U);
            action_mean[axis] += frame_value(input, frame, axis + 6U);
        }
        if (frame > 0U) {
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                error_delta_mean[axis] += frame_value(input, frame, axis) - frame_value(input, frame - 1U, axis);
                omega_delta_mean[axis] += frame_value(input, frame, axis + 3U) - frame_value(input, frame - 1U, axis + 3U);
                action_delta_mean[axis] += frame_value(input, frame, axis + 6U) - frame_value(input, frame - 1U, axis + 6U);
            }
        }
    }

    for (std::size_t axis = 0; axis < 3U; ++axis) {
        error_mean[axis] /= static_cast<float>(kHistoryFrames);
        omega_mean[axis] /= static_cast<float>(kHistoryFrames);
        action_mean[axis] /= static_cast<float>(kHistoryFrames);
        error_delta_mean[axis] *= inv_window;
        omega_delta_mean[axis] *= inv_window;
        action_delta_mean[axis] *= inv_window;
    }

    for (std::size_t axis = 0; axis < 3U; ++axis) {
        latent[axis] = latent_squash(
            2.2f * error_mean[axis] + 0.34f * omega_mean[axis] + 0.55f * error_delta_mean[axis] -
            0.22f * action_mean[axis]);
        latent[axis + 3U] = latent_squash(
            0.42f * omega_delta_mean[axis] - 0.28f * action_delta_mean[axis] + 0.18f * omega_mean[axis]);
    }
    latent[6U] = latent_squash(
        std::fabs(action_mean[0]) + std::fabs(action_mean[1]) + 0.55f * std::fabs(action_mean[2]));
    latent[7U] = latent_squash(
        std::fabs(error_delta_mean[0]) + std::fabs(error_delta_mean[1]) +
        0.65f * std::fabs(omega_delta_mean[2]));
    return latent;
}

}  // namespace flight_control
