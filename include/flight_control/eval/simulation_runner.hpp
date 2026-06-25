#pragma once

#include <memory>
#include <string>
#include <vector>

#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/model/attitude_policy.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/platform/host/simulated_components.hpp"

namespace flight_control {

struct CommandSegment {
    float start_sec{0.0f};
    GuidanceCommand command{};
};

struct WindStep {
    float start_sec{0.0f};
    Vector3 wind_m_s{};
};

struct MetricLimits {
    float max_velocity_rms_m_s{0.55f};
    float max_altitude_drift_m{1.0f};
    float max_tilt_rad{0.75f};
    float max_pwm_saturation_ratio{0.12f};
    float max_recovery_time_sec{2.0f};
};

struct EvaluationScenario {
    std::string name{};
    float duration_sec{8.0f};
    float metrics_start_sec{1.0f};
    float recovery_start_sec{0.0f};
    PlantConfig plant_config{};
    SpeedControllerConfig speed_config{};
    TorqueControllerConfig torque_config{};
    ModelAdapterConfig model_config{};
    MetricLimits limits{};
    std::vector<CommandSegment> commands{};
    std::vector<WindStep> wind_steps{};
};

struct SimulationMetrics {
    std::string scenario_name{};
    float score{0.0f};
    float velocity_rms_m_s{0.0f};
    float horizontal_velocity_rms_m_s{0.0f};
    float vertical_velocity_rms_m_s{0.0f};
    float altitude_drift_m{0.0f};
    float attitude_error_rms_rad{0.0f};
    float max_tilt_rad{0.0f};
    float pwm_saturation_ratio{0.0f};
    float recovery_time_sec{0.0f};
    Vector3 final_position_m{};
    Vector3 final_velocity_m_s{};
    bool stable{false};
};

struct SimulationResult {
    EvaluationScenario scenario{};
    SimulationMetrics metrics{};
};

class SimulationRunner {
public:
    explicit SimulationRunner(std::shared_ptr<IAttitudePolicy> policy);

    SimulationResult run(const EvaluationScenario& scenario) const;

private:
    std::shared_ptr<IAttitudePolicy> policy_;
};

SpeedControllerConfig optimized_speed_controller_config();
TorqueControllerConfig optimized_torque_controller_config();
std::vector<EvaluationScenario> default_evaluation_scenarios(const ModelAdapterConfig& model_config);
std::string format_metrics_table(const std::vector<SimulationResult>& results);
std::string format_markdown_report(const std::vector<SimulationResult>& results);

}  // namespace flight_control
