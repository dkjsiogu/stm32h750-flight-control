#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/data/history_window.hpp"
#include "flight_control/estimation/state_estimator.hpp"
#include "flight_control/model/adaptive_tcn_policy.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/model/static_mlp_policy.hpp"

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_history_window() {
    flight_control::HistoryWindow history;
    flight_control::ModelFrame first{};
    first.values[0] = 1.0f;
    history.reset(first);

    flight_control::ModelFrame second{};
    second.values[0] = 2.0f;
    history.append(second);

    const auto flat = history.flattened();
    check(flat[0] == 1.0f, "history keeps oldest frame after one append");
    check(flat[(flight_control::kHistoryFrames - 1) * flight_control::kFrameDim] == 2.0f,
          "history writes latest frame at tail");
}

void test_speed_controller_forward_pitch() {
    flight_control::SpeedController controller;
    flight_control::VehicleState state{};
    state.attitude = {};
    state.healthy = true;

    flight_control::GuidanceCommand command{};
    command.armed = true;
    command.target_velocity_m_s = {1.2f, 0.0f, 0.0f};

    flight_control::AttitudeSetpoint output{};
    for (int sample = 0; sample < 24; ++sample) {
        output = controller.update(state, command, flight_control::kDefaultControlPeriodSec);
    }
    const auto euler = flight_control::to_euler_zyx(output.target_attitude);
    check(euler.y > 0.04f, "forward velocity requests positive pitch");
    check(output.collective > 0.25f, "forward command produces nonzero collective");
}

void test_speed_controller_keeps_integrated_climb_target() {
    flight_control::SpeedControllerConfig config{};
    config.kp_z = 1.0f;
    config.ki_z = 0.0f;
    config.kp_altitude_hold = 2.0f;
    config.max_altitude_correction_m_s = 2.0f;
    config.max_accel_z_m_s2 = 4.0f;
    config.max_accel_xy_slew_m_s3 = 100.0f;
    config.max_accel_z_slew_m_s3 = 100.0f;
    flight_control::SpeedController controller(config);

    flight_control::VehicleState state{};
    state.attitude = {};
    state.position_m.z = 1.0f;
    state.healthy = true;

    flight_control::GuidanceCommand climb{};
    climb.armed = true;
    climb.climb_rate_m_s = 1.0f;
    for (int sample = 0; sample < 10; ++sample) {
        (void)controller.update(state, climb, 0.1f);
    }

    flight_control::GuidanceCommand hold{};
    hold.armed = true;
    const auto output = controller.update(state, hold, 0.1f);
    check(output.target_acceleration_m_s2.z > 0.5f,
          "speed controller keeps integrated climb target after climb command ends");
}

void test_torque_controller_roll_mix() {
    flight_control::TorqueControllerConfig config{};
    config.pwm_slew_rate_us_per_sec = 1.0e8f;
    flight_control::TorqueController controller(config);

    const flight_control::TorqueCommand torque{{0.08f, 0.0f, 0.0f}};
    const auto pwm = controller.mix(0.4f, torque, flight_control::kDefaultControlPeriodSec);

    check(pwm.pwm_us[0] > pwm.pwm_us[1], "positive roll torque raises front-left over front-right");
    check(pwm.pwm_us[3] > pwm.pwm_us[2], "positive roll torque raises rear-left over rear-right");
}

void test_torque_controller_failsafe_is_immediate() {
    flight_control::TorqueControllerConfig config{};
    config.pwm_slew_rate_us_per_sec = 20.0f;
    flight_control::TorqueController controller(config);

    (void)controller.mix(0.7f, flight_control::TorqueCommand{{0.04f, -0.03f, 0.02f}}, flight_control::kDefaultControlPeriodSec);
    const auto pwm = controller.failsafe();

    for (const float pwm_us : pwm.pwm_us) {
        check(std::fabs(pwm_us - config.pwm_min_us) < 1e-5f, "failsafe immediately writes minimum pwm");
    }
}

class CapturingPolicy final : public flight_control::IAttitudePolicy {
public:
    std::array<float, 3> predict(const std::array<float, flight_control::kModelInputDim>& input) override {
        last_input = input;
        return {0.5f, -0.25f, 0.1f};
    }

    std::array<float, flight_control::kModelInputDim> last_input{};
};

void test_model_adapter() {
    auto policy = std::make_shared<CapturingPolicy>();
    flight_control::ModelAdapterConfig config{};
    config.torque_limit_nm = 0.4f;
    flight_control::ModelAdapter adapter(policy, config);

    flight_control::VehicleState state{};
    state.attitude = {};
    state.angular_velocity_rad_s = {0.1f, -0.2f, 0.3f};
    state.healthy = true;

    flight_control::AttitudeSetpoint setpoint{};
    setpoint.target_attitude = flight_control::from_euler_zyx(0.1f, 0.0f, 0.0f);
    const auto torque = adapter.update(state, setpoint);

    check(std::fabs(torque.body_torque_nm.x - 0.2f) < 1e-5f, "model output scales to torque x");
    check(std::fabs(torque.body_torque_nm.y + 0.1f) < 1e-5f, "model output scales to torque y");
    check(policy->last_input.size() == flight_control::kModelInputDim, "model input dimension is fixed");
}

void test_state_estimator_stationary_observation() {
    flight_control::StateEstimator estimator;
    flight_control::SensorPacket packet{};
    packet.timestamp_sec = 0.0f;
    packet.accel_m_s2 = {0.0f, 0.0f, flight_control::kGravity};
    packet.battery_voltage_v = 16.0f;

    flight_control::StateEstimatorObservation observation{};
    observation.position_valid = true;
    observation.position_m = {0.0f, 0.0f, 1.2f};

    auto state = estimator.update(packet, observation);
    packet.timestamp_sec = flight_control::kDefaultControlPeriodSec;
    state = estimator.update(packet, observation);

    check(state.healthy, "state estimator accepts stationary valid imu");
    check(std::fabs(state.position_m.z - 1.2f) < 1e-4f, "state estimator keeps observed altitude");
    check(std::fabs(state.velocity_m_s.z) < 1e-3f, "state estimator keeps stationary vertical velocity small");
}

void test_state_estimator_rejects_zero_imu() {
    flight_control::StateEstimator estimator;
    flight_control::SensorPacket packet{};
    const auto state = estimator.update(packet, {});
    check(!state.healthy, "state estimator rejects missing imu acceleration");
}

void test_static_mlp_zero_weights() {
    auto weights = std::make_shared<flight_control::StaticMlpPolicyWeights>();
    flight_control::StaticMlpPolicy policy(weights);
    std::array<float, flight_control::kModelInputDim> input{};
    const auto output = policy.predict(input);
    check(output[0] == 0.0f && output[1] == 0.0f && output[2] == 0.0f,
          "zero static mlp weights produce zero output");
}

void test_adaptive_tcn_zero_weights() {
    auto weights = std::make_shared<flight_control::AdaptiveTcnPolicyWeights>();
    flight_control::AdaptiveTcnPolicy policy(weights);
    std::array<float, flight_control::kModelInputDim> input{};
    const auto output = policy.predict(input);
    check(output[0] == 0.0f && output[1] == 0.0f && output[2] == 0.0f,
          "zero adaptive tcn weights produce zero output");
}

void test_generated_policy_is_finite() {
    auto policy = flight_control::make_generated_policy();
    std::array<float, flight_control::kModelInputDim> input{};
    const auto output = policy->predict(input);
    check(std::isfinite(output[0]) && std::isfinite(output[1]) && std::isfinite(output[2]),
          "generated policy output is finite");
    check(flight_control::make_generated_tcn_policy_weights() != nullptr, "generated tcn weights exist");
    const auto config = flight_control::generated_model_config();
    check(config.torque_limit_nm > 0.0f, "generated model config has torque scale");
}

}  // namespace

int main() {
    test_history_window();
    test_speed_controller_forward_pitch();
    test_speed_controller_keeps_integrated_climb_target();
    test_torque_controller_roll_mix();
    test_torque_controller_failsafe_is_immediate();
    test_model_adapter();
    test_state_estimator_stationary_observation();
    test_state_estimator_rejects_zero_imu();
    test_static_mlp_zero_weights();
    test_adaptive_tcn_zero_weights();
    test_generated_policy_is_finite();
    std::cout << "flight_control_tests passed\n";
    return 0;
}
