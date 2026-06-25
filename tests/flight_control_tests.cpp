#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/data/history_window.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/model/static_mlp_policy.hpp"
#include "flight_control/platform/host/simulated_components.hpp"

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

    const auto output = controller.update(state, command, flight_control::kDefaultControlPeriodSec);
    const auto euler = flight_control::to_euler_zyx(output.target_attitude);
    check(euler.y > 0.04f, "forward velocity requests positive pitch");
    check(output.collective > 0.25f, "forward command produces nonzero collective");
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

void test_static_mlp_zero_weights() {
    auto weights = std::make_shared<flight_control::StaticMlpPolicyWeights>();
    flight_control::StaticMlpPolicy policy(weights);
    std::array<float, flight_control::kModelInputDim> input{};
    const auto output = policy.predict(input);
    check(output[0] == 0.0f && output[1] == 0.0f && output[2] == 0.0f,
          "zero static mlp weights produce zero output");
}

void test_application_smoke() {
    auto plant = std::make_shared<flight_control::SimulatedQuadPlant>();
    auto sensors = std::make_shared<flight_control::MockSensorSource>(plant);
    auto commands = std::make_shared<flight_control::MockCommandSource>();
    auto pwm_output = std::make_shared<flight_control::MockPwmOutput>(plant);
    auto policy = std::make_shared<flight_control::HeuristicAttitudePolicy>();
    auto runner = std::make_shared<flight_control::ThreadTaskRunner>();

    flight_control::FlightApplication app({
        runner,
        sensors,
        commands,
        pwm_output,
        policy,
    });
    app.run_demo(std::chrono::milliseconds(120));

    const auto telemetry = app.telemetry();
    const auto solution = app.snapshot();
    check(std::isfinite(telemetry.state.position_m.z), "application telemetry is finite");
    check(solution.motor_pwm.pwm_us[0] >= 1000.0f && solution.motor_pwm.pwm_us[0] <= 2000.0f,
          "application outputs bounded pwm");
}

}  // namespace

int main() {
    test_history_window();
    test_speed_controller_forward_pitch();
    test_torque_controller_roll_mix();
    test_model_adapter();
    test_static_mlp_zero_weights();
    test_application_smoke();
    std::cout << "flight_control_tests passed\n";
    return 0;
}
