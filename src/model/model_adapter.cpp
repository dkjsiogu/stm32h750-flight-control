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
    previous_angular_velocity_rad_s_ = {};
    last_target_ = {};
    has_last_target_ = false;
    has_previous_angular_velocity_ = false;
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
    action = apply_safety_filter(action, state);
    previous_action_ = action;
    previous_angular_velocity_rad_s_ = state.angular_velocity_rad_s;
    has_previous_angular_velocity_ = true;

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
    const float vector_norm = norm({q_error.x, q_error.y, q_error.z});
    const float angle = 2.0f * std::atan2(vector_norm, std::max(q_error.w, 1.0e-6f));
    const float log_scale = vector_norm > 1.0e-6f ? angle / vector_norm : 2.0f;
    const Vector3 attitude_log{
        q_error.x * log_scale,
        q_error.y * log_scale,
        q_error.z * log_scale,
    };
    const Vector3 angular_accel = has_previous_angular_velocity_ ? Vector3{
        (state.angular_velocity_rad_s.x - previous_angular_velocity_rad_s_.x) / kDefaultControlPeriodSec,
        (state.angular_velocity_rad_s.y - previous_angular_velocity_rad_s_.y) / kDefaultControlPeriodSec,
        (state.angular_velocity_rad_s.z - previous_angular_velocity_rad_s_.z) / kDefaultControlPeriodSec,
    } : Vector3{};
    ModelFrame frame{};
    frame.values = {
        attitude_log.x,
        attitude_log.y,
        attitude_log.z,
        state.angular_velocity_rad_s.x,
        state.angular_velocity_rad_s.y,
        state.angular_velocity_rad_s.z,
        previous_action_[0],
        previous_action_[1],
        previous_action_[2],
        setpoint.target_acceleration_m_s2.x,
        setpoint.target_acceleration_m_s2.y,
        setpoint.target_acceleration_m_s2.z,
        setpoint.collective,
        angular_accel.x,
        angular_accel.y,
        angular_accel.z,
    };
    return frame;
}

std::array<float, kModelInputDim> ModelAdapter::normalize_input(const std::array<float, kModelInputDim>& input) const {
    auto normalized = input;
    for (std::size_t frame = 0; frame < kHistoryFrames; ++frame) {
        const std::size_t offset = frame * kFrameDim;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float error_std = std::max(config_.normalization.attitude_error_std[axis], 1e-4f);
            normalized[offset + axis] =
                (normalized[offset + axis] - config_.normalization.attitude_error_mean[axis]) / error_std;

            const float omega_std = std::max(config_.normalization.omega_std[axis], 1e-4f);
            normalized[offset + 3 + axis] =
                (normalized[offset + 3 + axis] - config_.normalization.omega_mean[axis]) / omega_std;

            const float action_std = std::max(config_.normalization.previous_action_std[axis], 1e-4f);
            normalized[offset + 6 + axis] =
                (normalized[offset + 6 + axis] - config_.normalization.previous_action_mean[axis]) / action_std;

            const float target_accel_std = std::max(config_.normalization.target_acceleration_std[axis], 1e-4f);
            normalized[offset + 9 + axis] =
                (normalized[offset + 9 + axis] - config_.normalization.target_acceleration_mean[axis]) / target_accel_std;

            const float angular_accel_std = std::max(config_.normalization.angular_accel_std[axis], 1e-4f);
            normalized[offset + 13 + axis] =
                (normalized[offset + 13 + axis] - config_.normalization.angular_accel_mean[axis]) / angular_accel_std;
        }
        const float collective_std = std::max(config_.normalization.collective_std, 1e-4f);
        normalized[offset + 12] = (normalized[offset + 12] - config_.normalization.collective_mean) / collective_std;
    }
    return normalized;
}

std::array<float, 3> ModelAdapter::apply_safety_filter(std::array<float, 3> action, const VehicleState& state) const {
    const Vector3 euler = to_euler_zyx(state.attitude);
    const float tilt = std::sqrt(euler.x * euler.x + euler.y * euler.y);
    const float tilt_margin = std::max(config_.safety_max_tilt_rad - tilt, 0.0f);
    const float tilt_scale = std::clamp(
        tilt_margin / std::max(0.18f * config_.safety_max_tilt_rad, 1.0e-4f),
        config_.safety_min_torque_scale,
        1.0f);
    if (tilt > config_.safety_max_tilt_rad) {
        if ((euler.x > 0.0f && action[0] > 0.0f) || (euler.x < 0.0f && action[0] < 0.0f)) {
            action[0] *= tilt_scale;
        }
        if ((euler.y > 0.0f && action[1] > 0.0f) || (euler.y < 0.0f && action[1] < 0.0f)) {
            action[1] *= tilt_scale;
        }
    }

    const std::array<float, 3> omega{
        state.angular_velocity_rad_s.x,
        state.angular_velocity_rad_s.y,
        state.angular_velocity_rad_s.z,
    };
    for (std::size_t axis = 0; axis < action.size(); ++axis) {
        const float excess = std::fabs(omega[axis]) - config_.safety_max_angular_rate_rad_s;
        if (excess > 0.0f && ((omega[axis] > 0.0f && action[axis] > 0.0f) || (omega[axis] < 0.0f && action[axis] < 0.0f))) {
            const float rate_scale = std::clamp(
                1.0f - excess / std::max(config_.safety_max_angular_rate_rad_s, 1.0e-4f),
                config_.safety_min_torque_scale,
                1.0f);
            action[axis] *= rate_scale;
        }
    }
    return action;
}

}  // namespace flight_control
