#include <chrono>
#include <iostream>
#include <memory>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/platform/host/host_environment.hpp"
#include "flight_control/platform/host/host_synchronization.hpp"
#include "flight_control/platform/host/thread_task_runner.hpp"

int main() {
    using namespace flight_control;

    auto environment = std::make_shared<HostFlightEnvironment>();
    auto sensors = std::make_shared<HostSensorSource>(environment);
    auto commands = std::make_shared<ScriptedCommandSource>();
    auto pwm_output = std::make_shared<HostPwmOutput>(environment);
    auto policy = std::make_shared<StaticMlpPolicy>(make_generated_policy_weights());
    auto runner = std::make_shared<ThreadTaskRunner>();
    auto critical_section = std::make_shared<HostMutexCriticalSection>();

    FlightApplication app({
        runner,
        sensors,
        commands,
        pwm_output,
        policy,
        critical_section,
    }, {}, {}, generated_model_config());

    std::cout << "boot: host-only flight-control verification demo\n";
    app.run_for_ms(8000U);

    const FlightTelemetry telemetry = app.telemetry();
    const ControlSolution solution = app.snapshot();
    const VehicleState truth = environment->truth_state();
    std::cout << "final estimated z=" << telemetry.state.position_m.z
              << " truth speed=(" << truth.velocity_m_s.x
              << "," << truth.velocity_m_s.y
              << "," << truth.velocity_m_s.z << ")"
              << " pwm0=" << solution.motor_pwm.pwm_us[0]
              << '\n';
    return 0;
}
