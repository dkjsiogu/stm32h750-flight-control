#include "flight_control/model/model_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace flight_control {

ModelAdapter::ModelAdapter(std::shared_ptr<IAttitudePolicy> policy, ModelAdapterConfig config)
    : policy_(std::move(policy)), config_(config), history_(kHistoryFrames) {
    reset();
}

void ModelAdapter::reset() {
    history_.reset(ModelFrame{});
    previous_action_ = {0.0f, 0.0f, 0.0f};
    last_target_ = {};
    has_last_target_ = false;
}

TorqueCommand ModelAdapter::update(const VehicleState& state, const AttitudeSetpoint& setpoint) {
    const ModelFrame frame = build_frame(state, setpoint);
    bool reset_history = !has_last_target_;
    if (has_last_target_) {
        const Quaternion target_delta = attitude_error(setpoint.target_attitude, last_target_);
        reset_history = norm({target_delta.x, target_delta.y, target_delta.z}) > config_.target_reset_threshold;
    }

    if (reset_history) {
        history_.reset(frame);
    } else {
        history_.append(frame);
    }
    has_last_target_ = true;
    last_target_ = setpoint.target_attitude;

    const auto input = normalize_input(history_.flattened());
    if (!policy_) {
        previous_action_ = {0.0f, 0.0f, 0.0f};
        return {};
    }

    auto action = policy_->predict(input);
    for (float& value : action) {
        value = std::clamp(value, -1.0f, 1.0f);
    }
    previous_action_ = action;

    return {
        {
            action[0] * config_.torque_limit_nm,
            action[1] * config_.torque_limit_nm,
            action[2] * config_.torque_limit_nm,
        },
    };
}

std::array<float, kModelInputDim> ModelAdapter::last_input() const {
    return normalize_input(history_.flattened());
}

ModelFrame ModelAdapter::build_frame(const VehicleState& state, const AttitudeSetpoint& setpoint) const {
    const Quaternion q_error = attitude_error(setpoint.target_attitude, state.attitude);
    ModelFrame frame{};
    frame.values = {
        q_error.x,
        q_error.y,
        q_error.z,
        state.angular_velocity_rad_s.x,
        state.angular_velocity_rad_s.y,
        state.angular_velocity_rad_s.z,
        previous_action_[0],
        previous_action_[1],
        previous_action_[2],
    };
    return frame;
}

std::array<float, kModelInputDim> ModelAdapter::normalize_input(const std::array<float, kModelInputDim>& input) const {
    auto normalized = input;
    for (std::size_t frame = 0; frame < kHistoryFrames; ++frame) {
        const std::size_t offset = frame * kFrameDim;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float std_value = std::max(config_.normalization.omega_std[axis], 1e-4f);
            normalized[offset + 3 + axis] =
                (normalized[offset + 3 + axis] - config_.normalization.omega_mean[axis]) / std_value;
        }
    }
    return normalized;
}

}  // namespace flight_control
