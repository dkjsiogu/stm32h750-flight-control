#include <chrono>
#include <iostream>
#include <memory>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/platform/host/simulated_components.hpp"

int main() {
    using namespace flight_control;

    auto plant = std::make_shared<SimulatedQuadPlant>();
    auto sensors = std::make_shared<MockSensorSource>(plant);
    auto commands = std::make_shared<MockCommandSource>();
    auto pwm_output = std::make_shared<MockPwmOutput>(plant);
    auto policy = std::make_shared<HeuristicAttitudePolicy>();
    auto runner = std::make_shared<ThreadTaskRunner>();

    FlightApplication app({
        runner,
        sensors,
        commands,
        pwm_output,
        policy,
    });

    std::cout << "boot: stm32h750 flight-control stack host demo\n";
    app.run_demo(std::chrono::seconds(8));

    const FlightTelemetry telemetry = app.telemetry();
    const ControlSolution solution = app.snapshot();
    std::cout << "final position z=" << telemetry.state.position_m.z
              << " final speed=(" << telemetry.state.velocity_m_s.x
              << "," << telemetry.state.velocity_m_s.y
              << "," << telemetry.state.velocity_m_s.z << ")"
              << " pwm0=" << solution.motor_pwm.pwm_us[0]
              << '\n';
    return 0;
}

