#pragma once

#include <memory>

#include "flight_control/data/history_window.hpp"
#include "flight_control/model/attitude_policy.hpp"

namespace flight_control {

struct ModelNormalization {
    std::array<float, 3> omega_mean{0.0f, 0.0f, 0.0f};
    std::array<float, 3> omega_std{1.0f, 1.0f, 1.0f};
};

struct ModelAdapterConfig {
    float torque_limit_nm{0.35f};
    float target_reset_threshold{0.25f};
    ModelNormalization normalization{};
};

class ModelAdapter {
public:
    explicit ModelAdapter(
        std::shared_ptr<IAttitudePolicy> policy = std::make_shared<HeuristicAttitudePolicy>(),
        ModelAdapterConfig config = {});

    void reset();
    TorqueCommand update(const VehicleState& state, const AttitudeSetpoint& setpoint);

    std::array<float, kModelInputDim> last_input() const;

private:
    ModelFrame build_frame(const VehicleState& state, const AttitudeSetpoint& setpoint) const;
    std::array<float, kModelInputDim> normalize_input(const std::array<float, kModelInputDim>& input) const;

    std::shared_ptr<IAttitudePolicy> policy_;
    ModelAdapterConfig config_;
    HistoryWindow history_;
    std::array<float, 3> previous_action_{0.0f, 0.0f, 0.0f};
    Quaternion last_target_{};
    bool has_last_target_{false};
};

}  // namespace flight_control
