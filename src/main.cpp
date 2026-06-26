#include <chrono>
#include <iostream>
#include <memory>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/platform/host/host_environment.hpp"

int main() {
    using namespace flight_control;

    auto environment = std::make_shared<HostFlightEnvironment>();
    auto sensors = std::make_shared<HostSensorSource>(environment);
    auto commands = std::make_shared<ScriptedCommandSource>();
    auto pwm_output = std::make_shared<HostPwmOutput>(environment);
    auto policy = std::make_shared<StaticMlpPolicy>(make_generated_policy_weights());
    auto runner = std::make_shared<ThreadTaskRunner>();

    FlightApplication app({
        runner,
        sensors,
        commands,
        pwm_output,
        policy,
    }, {}, {}, generated_model_config());

    std::cout << "boot: stm32h750 flight-control stack host demo\n";
    app.run_demo(std::chrono::seconds(8));

    const FlightTelemetry telemetry = app.telemetry();
    const ControlSolution solution = app.snapshot();
    std::cout << "final position z=" << telemetry.state.position_m.z
              << " final speed=(" << telemetry.truth.velocity_m_s.x
              << "," << telemetry.truth.velocity_m_s.y
              << "," << telemetry.truth.velocity_m_s.z << ")"
              << " pwm0=" << solution.motor_pwm.pwm_us[0]
              << '\n';
    return 0;
}
