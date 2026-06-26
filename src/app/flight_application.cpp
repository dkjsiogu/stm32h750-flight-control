#include "flight_control/app/flight_application.hpp"

#include <utility>

namespace flight_control {

namespace {

/** 默认控制任务周期，单位 ms。 */
constexpr std::uint32_t kControlPeriodMs = 4;

}  // namespace

FlightApplication::FlightApplication(Dependencies deps,
                                     SpeedControllerConfig speed_config,
                                     TorqueControllerConfig torque_config,
                                     ModelAdapterConfig model_config)
    : deps_(std::move(deps)),
      speed_controller_(speed_config),
      torque_controller_(torque_config),
      model_adapter_(deps_.policy, model_config) {
    if (!deps_.critical_section) {
        deps_.critical_section = std::make_shared<NullCriticalSection>();
    }
}

void FlightApplication::start() {
    deps_.task_runner->spawn("acquisition", [this] { acquisition_loop(); });
    deps_.task_runner->spawn("control", [this] { control_loop(); });
    deps_.task_runner->spawn("actuation", [this] { actuation_loop(); });
}

void FlightApplication::run_for_ms(std::uint32_t duration_ms) {
    start();
    deps_.task_runner->sleep_ms(duration_ms);
    request_stop();
    join();
}

void FlightApplication::request_stop() {
    deps_.task_runner->request_stop();
}

void FlightApplication::join() {
    deps_.task_runner->join();
}

ControlSolution FlightApplication::snapshot() const {
    CriticalSectionGuard lock(*deps_.critical_section);
    return latest_solution_;
}

FlightTelemetry FlightApplication::telemetry() const {
    CriticalSectionGuard lock(*deps_.critical_section);
    return latest_telemetry_;
}

void FlightApplication::acquisition_loop() {
    while (!deps_.task_runner->stop_requested()) {
        const FlightTelemetry telemetry = deps_.sensor_source->read();
        {
            CriticalSectionGuard lock(*deps_.critical_section);
            latest_telemetry_ = telemetry;
            has_telemetry_ = true;
        }
        deps_.task_runner->sleep_ms(kControlPeriodMs);
    }
}

void FlightApplication::control_loop() {
    bool initialized = false;
    while (!deps_.task_runner->stop_requested()) {
        FlightTelemetry telemetry_copy{};
        bool telemetry_ready = false;
        {
            CriticalSectionGuard lock(*deps_.critical_section);
            telemetry_copy = latest_telemetry_;
            telemetry_ready = has_telemetry_;
        }

        if (!telemetry_ready) {
            deps_.task_runner->sleep_ms(kControlPeriodMs);
            continue;
        }

        const GuidanceCommand command = deps_.command_source->read();
        if (!telemetry_copy.state.healthy || !command.armed) {
            const ControlSolution failsafe = make_failsafe_solution(telemetry_copy.state.attitude);
            initialized = false;
            {
                CriticalSectionGuard lock(*deps_.critical_section);
                latest_solution_ = failsafe;
                has_solution_ = true;
            }
            deps_.task_runner->sleep_ms(kControlPeriodMs);
            continue;
        }

        if (!initialized) {
            speed_controller_.reset(telemetry_copy.state.attitude);
            torque_controller_.reset();
            model_adapter_.reset();
            initialized = true;
        }

        const AttitudeSetpoint attitude = speed_controller_.update(
            telemetry_copy.state,
            command,
            kDefaultControlPeriodSec);
        const TorqueCommand torque = model_adapter_.update(telemetry_copy.state, attitude);
        const MotorPwmFrame pwm = torque_controller_.mix(attitude.collective, torque, kDefaultControlPeriodSec);

        {
            CriticalSectionGuard lock(*deps_.critical_section);
            latest_solution_ = {attitude, torque, pwm};
            has_solution_ = true;
        }
        deps_.task_runner->sleep_ms(kControlPeriodMs);
    }
}

ControlSolution FlightApplication::make_failsafe_solution(const Quaternion& current_attitude) {
    speed_controller_.reset(current_attitude);
    torque_controller_.reset();
    model_adapter_.reset();

    ControlSolution solution{};
    solution.motor_pwm = torque_controller_.failsafe();
    return solution;
}

void FlightApplication::actuation_loop() {
    while (!deps_.task_runner->stop_requested()) {
        ControlSolution solution{};
        bool solution_ready = false;
        {
            CriticalSectionGuard lock(*deps_.critical_section);
            solution = latest_solution_;
            solution_ready = has_solution_;
        }

        if (solution_ready) {
            deps_.pwm_output->write(solution.motor_pwm);
        }
        deps_.task_runner->sleep_ms(kControlPeriodMs);
    }
}

}  // namespace flight_control
