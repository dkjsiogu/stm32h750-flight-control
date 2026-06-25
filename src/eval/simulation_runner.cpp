#include "flight_control/eval/simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace flight_control {

namespace {

GuidanceCommand command_at(const std::vector<CommandSegment>& commands, float time_sec) {
    GuidanceCommand selected{};
    selected.armed = true;
    for (const auto& segment : commands) {
        if (time_sec >= segment.start_sec) {
            selected = segment.command;
        }
    }
    return selected;
}

void apply_wind_steps(SimulatedQuadPlant& plant, const std::vector<WindStep>& wind_steps, float last_time_sec, float time_sec) {
    for (const auto& step : wind_steps) {
        if (step.start_sec > last_time_sec && step.start_sec <= time_sec) {
            plant.set_wind(step.wind_m_s);
        }
    }
}

float attitude_error_angle_rad(const Quaternion& target, const Quaternion& current) {
    const Quaternion error = attitude_error(target, current);
    const float vector_norm = norm({error.x, error.y, error.z});
    return 2.0f * std::atan2(vector_norm, std::max(error.w, 1e-6f));
}

float tilt_rad(const Quaternion& attitude) {
    const Vector3 euler = to_euler_zyx(attitude);
    return std::sqrt(euler.x * euler.x + euler.y * euler.y);
}

Vector3 target_velocity_from_command(const GuidanceCommand& command) {
    return {
        command.target_velocity_m_s.x,
        command.target_velocity_m_s.y,
        command.climb_rate_m_s,
    };
}

float score_from_metrics(const SimulationMetrics& metrics, const MetricLimits& limits) {
    const float velocity_cost = 34.0f * metrics.velocity_rms_m_s / std::max(limits.max_velocity_rms_m_s, 1e-4f);
    const float altitude_cost = 18.0f * metrics.altitude_drift_m / std::max(limits.max_altitude_drift_m, 1e-4f);
    const float tilt_cost = 18.0f * metrics.max_tilt_rad / std::max(limits.max_tilt_rad, 1e-4f);
    const float saturation_cost = 15.0f * metrics.pwm_saturation_ratio / std::max(limits.max_pwm_saturation_ratio, 1e-4f);
    const float recovery_cost = 15.0f * metrics.recovery_time_sec / std::max(limits.max_recovery_time_sec, 1e-4f);
    return std::clamp(100.0f - velocity_cost - altitude_cost - tilt_cost - saturation_cost - recovery_cost, 0.0f, 100.0f);
}

bool is_stable(const SimulationMetrics& metrics, const MetricLimits& limits) {
    return std::isfinite(metrics.final_position_m.z) &&
           std::isfinite(metrics.velocity_rms_m_s) &&
           metrics.final_position_m.z > 0.12f &&
           metrics.velocity_rms_m_s <= limits.max_velocity_rms_m_s &&
           metrics.altitude_drift_m <= limits.max_altitude_drift_m &&
           metrics.max_tilt_rad <= limits.max_tilt_rad &&
           metrics.pwm_saturation_ratio <= limits.max_pwm_saturation_ratio &&
           metrics.recovery_time_sec <= limits.max_recovery_time_sec;
}

CommandSegment command_segment(float start_sec, Vector3 velocity_m_s, float climb_rate_m_s, float yaw_rate_rad_s) {
    GuidanceCommand command{};
    command.armed = true;
    command.target_velocity_m_s = velocity_m_s;
    command.climb_rate_m_s = climb_rate_m_s;
    command.yaw_rate_rad_s = yaw_rate_rad_s;
    return {start_sec, command};
}

MetricLimits limits(float velocity_rms, float altitude_drift, float tilt, float saturation, float recovery) {
    return {velocity_rms, altitude_drift, tilt, saturation, recovery};
}

}  // namespace

SimulationRunner::SimulationRunner(std::shared_ptr<IAttitudePolicy> policy)
    : policy_(std::move(policy)) {
    if (!policy_) {
        throw std::invalid_argument("SimulationRunner policy must not be null");
    }
}

SimulationResult SimulationRunner::run(const EvaluationScenario& scenario) const {
    SimulatedQuadPlant plant(scenario.plant_config);
    SpeedController speed_controller(scenario.speed_config);
    TorqueController torque_controller(scenario.torque_config);
    ModelAdapter model_adapter(policy_, scenario.model_config);

    speed_controller.reset(plant.state().attitude);
    torque_controller.reset();
    model_adapter.reset();

    const float dt = kDefaultControlPeriodSec;
    const int steps = std::max(1, static_cast<int>(std::ceil(scenario.duration_sec / dt)));
    const float initial_altitude = plant.state().position_m.z;

    double velocity_error_sum = 0.0;
    double horizontal_velocity_error_sum = 0.0;
    double vertical_velocity_error_sum = 0.0;
    double attitude_error_sum = 0.0;
    int metric_samples = 0;
    int saturated_channels = 0;
    int pwm_channels = 0;
    float max_tilt = 0.0f;
    float max_altitude_drift = 0.0f;
    float recovery_time = std::max(0.0f, scenario.duration_sec - scenario.recovery_start_sec);
    float settled_duration = 0.0f;
    bool recovered = false;
    float last_time = 0.0f;

    ControlSolution latest_solution{};
    GuidanceCommand command = command_at(scenario.commands, 0.0f);
    for (int step = 0; step < steps; ++step) {
        const float time = static_cast<float>(step) * dt;
        apply_wind_steps(plant, scenario.wind_steps, last_time, time);
        last_time = time;

        const FlightTelemetry telemetry = plant.telemetry();
        command = command_at(scenario.commands, time);
        const AttitudeSetpoint attitude = speed_controller.update(telemetry.state, command, dt);
        const TorqueCommand torque = model_adapter.update(telemetry.state, attitude);
        const MotorPwmFrame pwm = torque_controller.mix(attitude.collective, torque, dt);
        latest_solution = {attitude, torque, pwm};
        plant.step(pwm, dt);

        const VehicleState state = plant.state();
        const Vector3 target_velocity = target_velocity_from_command(command);
        const Vector3 velocity_error = state.velocity_m_s - target_velocity;
        const float horizontal_error = std::sqrt(velocity_error.x * velocity_error.x + velocity_error.y * velocity_error.y);
        const float vertical_error = std::fabs(velocity_error.z);
        const float attitude_error = attitude_error_angle_rad(attitude.target_attitude, state.attitude);
        const float current_tilt = tilt_rad(state.attitude);

        if (time >= scenario.metrics_start_sec) {
            velocity_error_sum += norm_squared(velocity_error);
            horizontal_velocity_error_sum += horizontal_error * horizontal_error;
            vertical_velocity_error_sum += vertical_error * vertical_error;
            attitude_error_sum += attitude_error * attitude_error;
            max_tilt = std::max(max_tilt, current_tilt);
            max_altitude_drift = std::max(max_altitude_drift, std::fabs(state.position_m.z - initial_altitude));
            ++metric_samples;

            for (const float pwm_us : latest_solution.motor_pwm.pwm_us) {
                if (pwm_us <= scenario.torque_config.pwm_min_us + 5.0f ||
                    pwm_us >= scenario.torque_config.pwm_max_us - 5.0f) {
                    ++saturated_channels;
                }
                ++pwm_channels;
            }
        }

        if (time >= scenario.recovery_start_sec && !recovered) {
            const bool settled = norm(velocity_error) < 0.28f && current_tilt < 0.30f;
            settled_duration = settled ? settled_duration + dt : 0.0f;
            if (settled_duration >= 0.45f) {
                recovered = true;
                recovery_time = time - scenario.recovery_start_sec - settled_duration;
                recovery_time = std::max(0.0f, recovery_time);
            }
        }
    }

    const VehicleState final_state = plant.state();
    const float sample_count = static_cast<float>(std::max(metric_samples, 1));
    SimulationMetrics metrics{};
    metrics.scenario_name = scenario.name;
    metrics.velocity_rms_m_s = std::sqrt(static_cast<float>(velocity_error_sum / sample_count));
    metrics.horizontal_velocity_rms_m_s = std::sqrt(static_cast<float>(horizontal_velocity_error_sum / sample_count));
    metrics.vertical_velocity_rms_m_s = std::sqrt(static_cast<float>(vertical_velocity_error_sum / sample_count));
    metrics.attitude_error_rms_rad = std::sqrt(static_cast<float>(attitude_error_sum / sample_count));
    metrics.altitude_drift_m = max_altitude_drift;
    metrics.max_tilt_rad = max_tilt;
    metrics.pwm_saturation_ratio = pwm_channels > 0 ? static_cast<float>(saturated_channels) / static_cast<float>(pwm_channels) : 0.0f;
    metrics.recovery_time_sec = recovery_time;
    metrics.final_position_m = final_state.position_m;
    metrics.final_velocity_m_s = final_state.velocity_m_s;
    metrics.score = score_from_metrics(metrics, scenario.limits);
    metrics.stable = is_stable(metrics, scenario.limits);

    return {scenario, metrics};
}

SpeedControllerConfig optimized_speed_controller_config() {
    SpeedControllerConfig config{};
    config.kp_xy = 2.8f;
    config.ki_xy = 0.42f;
    config.kp_z = 2.75f;
    config.ki_z = 0.42f;
    config.kp_altitude_hold = 2.2f;
    config.max_altitude_correction_m_s = 2.0f;
    config.max_accel_xy_m_s2 = 8.0f;
    config.max_accel_z_m_s2 = 5.2f;
    config.max_tilt_rad = 0.74f;
    config.integral_limit_xy = 3.6f;
    config.integral_limit_z = 4.5f;
    return config;
}

TorqueControllerConfig optimized_torque_controller_config() {
    TorqueControllerConfig config{};
    config.pwm_slew_rate_us_per_sec = 4800.0f;
    return config;
}

std::vector<EvaluationScenario> default_evaluation_scenarios(const ModelAdapterConfig& model_config) {
    const SpeedControllerConfig speed_config = optimized_speed_controller_config();
    const TorqueControllerConfig torque_config = optimized_torque_controller_config();

    EvaluationScenario hover{};
    hover.name = "hover_wind_bias";
    hover.duration_sec = 7.0f;
    hover.metrics_start_sec = 1.0f;
    hover.recovery_start_sec = 1.0f;
    hover.speed_config = speed_config;
    hover.torque_config = torque_config;
    hover.model_config = model_config;
    hover.limits = limits(0.20f, 0.18f, 0.32f, 0.05f, 1.2f);
    hover.commands = {command_segment(0.0f, {}, 0.0f, 0.0f)};

    EvaluationScenario cruise{};
    cruise.name = "forward_cruise";
    cruise.duration_sec = 9.0f;
    cruise.metrics_start_sec = 3.5f;
    cruise.recovery_start_sec = 2.0f;
    cruise.speed_config = speed_config;
    cruise.torque_config = torque_config;
    cruise.model_config = model_config;
    cruise.limits = limits(0.24f, 0.22f, 0.36f, 0.06f, 1.8f);
    cruise.commands = {
        command_segment(0.0f, {}, 0.0f, 0.0f),
        command_segment(2.0f, {1.4f, 0.0f, 0.0f}, 0.0f, 0.0f),
    };

    EvaluationScenario gust{};
    gust.name = "gust_recovery";
    gust.duration_sec = 9.0f;
    gust.metrics_start_sec = 1.0f;
    gust.recovery_start_sec = 5.0f;
    gust.speed_config = speed_config;
    gust.torque_config = torque_config;
    gust.model_config = model_config;
    gust.limits = limits(0.34f, 0.24f, 0.42f, 0.08f, 2.1f);
    gust.commands = {command_segment(0.0f, {}, 0.0f, 0.0f)};
    gust.wind_steps = {
        {2.0f, {2.0f, -1.0f, 0.0f}},
        {5.0f, {-1.4f, 0.8f, 0.0f}},
    };

    EvaluationScenario payload{};
    payload.name = "payload_motor_lag";
    payload.duration_sec = 8.0f;
    payload.metrics_start_sec = 1.5f;
    payload.recovery_start_sec = 1.5f;
    payload.plant_config.mass_kg = 1.22f;
    payload.plant_config.motor_tau_sec = 0.09f;
    payload.plant_config.wind_m_s = {0.4f, -0.15f, 0.0f};
    payload.speed_config = speed_config;
    payload.torque_config = torque_config;
    payload.model_config = model_config;
    payload.limits = limits(0.28f, 0.45f, 0.38f, 0.08f, 2.2f);
    payload.commands = {command_segment(0.0f, {}, 0.0f, 0.0f)};

    EvaluationScenario climb{};
    climb.name = "climb_turn_cruise";
    climb.duration_sec = 9.0f;
    climb.metrics_start_sec = 2.5f;
    climb.recovery_start_sec = 5.0f;
    climb.speed_config = speed_config;
    climb.torque_config = torque_config;
    climb.model_config = model_config;
    climb.limits = limits(0.30f, 1.55f, 0.42f, 0.08f, 1.8f);
    climb.commands = {
        command_segment(0.0f, {}, 0.0f, 0.0f),
        command_segment(1.5f, {0.8f, 0.4f, 0.0f}, 0.35f, 0.24f),
        command_segment(5.0f, {0.8f, 0.4f, 0.0f}, 0.0f, 0.0f),
    };

    return {hover, cruise, gust, payload, climb};
}

std::string format_metrics_table(const std::vector<SimulationResult>& results) {
    std::ostringstream output;
    output << "scenario,stable,score,vel_rms,horiz_rms,vert_rms,alt_drift,max_tilt,pwm_sat,recovery,final_z\n";
    output << std::fixed << std::setprecision(3);
    for (const auto& result : results) {
        const auto& metrics = result.metrics;
        output << metrics.scenario_name << ','
               << (metrics.stable ? "yes" : "no") << ','
               << metrics.score << ','
               << metrics.velocity_rms_m_s << ','
               << metrics.horizontal_velocity_rms_m_s << ','
               << metrics.vertical_velocity_rms_m_s << ','
               << metrics.altitude_drift_m << ','
               << metrics.max_tilt_rad << ','
               << metrics.pwm_saturation_ratio << ','
               << metrics.recovery_time_sec << ','
               << metrics.final_position_m.z << '\n';
    }
    return output.str();
}

std::string format_markdown_report(const std::vector<SimulationResult>& results) {
    std::ostringstream output;
    float total_score = 0.0f;
    int stable_count = 0;
    for (const auto& result : results) {
        total_score += result.metrics.score;
        stable_count += result.metrics.stable ? 1 : 0;
    }
    const float average_score = results.empty() ? 0.0f : total_score / static_cast<float>(results.size());

    output << "# Flight Control Closed-Loop Evaluation\n\n";
    output << "- policy: generated static MLP only\n";
    output << "- loop: SpeedController -> ModelAdapter -> TorqueController -> PWM -> motor lag -> rigid-body plant\n";
    output << "- stable scenarios: " << stable_count << "/" << results.size() << "\n";
    output << "- average score: " << std::fixed << std::setprecision(1) << average_score << "/100\n\n";
    output << "| Scenario | Stable | Score | Velocity RMS | Alt Drift | Max Tilt | PWM Sat | Recovery | Final z |\n";
    output << "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    output << std::fixed << std::setprecision(3);
    for (const auto& result : results) {
        const auto& metrics = result.metrics;
        output << "| " << metrics.scenario_name
               << " | " << (metrics.stable ? "yes" : "no")
               << " | " << metrics.score
               << " | " << metrics.velocity_rms_m_s
               << " | " << metrics.altitude_drift_m
               << " | " << metrics.max_tilt_rad
               << " | " << metrics.pwm_saturation_ratio
               << " | " << metrics.recovery_time_sec
               << " | " << metrics.final_position_m.z
               << " |\n";
    }
    return output.str();
}

}  // namespace flight_control
