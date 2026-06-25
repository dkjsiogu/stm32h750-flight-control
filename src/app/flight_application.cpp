#include "flight_control/app/flight_application.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace flight_control {

namespace {

void sleep_control_period() {
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
}

}  // namespace

FlightApplication::FlightApplication(Dependencies deps,
                                     SpeedControllerConfig speed_config,
                                     TorqueControllerConfig torque_config,
                                     ModelAdapterConfig model_config)
    : deps_(std::move(deps)),
      speed_controller_(speed_config),
      torque_controller_(torque_config),
      model_adapter_(deps_.policy, model_config) {
    if (!deps_.task_runner || !deps_.sensor_source || !deps_.command_source || !deps_.pwm_output) {
        throw std::invalid_argument("FlightApplication dependencies must not be null");
    }
}

void FlightApplication::run_demo(std::chrono::milliseconds duration) {
    deps_.task_runner->spawn("acquisition", [this] { acquisition_loop(); });
    deps_.task_runner->spawn("control", [this] { control_loop(); });
    deps_.task_runner->spawn("actuation", [this] { actuation_loop(); });
    deps_.task_runner->spawn("log", [this] { log_loop(); });

    std::this_thread::sleep_for(duration);
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
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return latest_solution_;
}

FlightTelemetry FlightApplication::telemetry() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return latest_telemetry_;
}

void FlightApplication::acquisition_loop() {
    while (!deps_.task_runner->stop_requested()) {
        const FlightTelemetry telemetry = deps_.sensor_source->read();
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_telemetry_ = telemetry;
            has_telemetry_ = true;
        }
        sleep_control_period();
    }
}

void FlightApplication::control_loop() {
    bool initialized = false;
    while (!deps_.task_runner->stop_requested()) {
        FlightTelemetry telemetry_copy{};
        bool telemetry_ready = false;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            telemetry_copy = latest_telemetry_;
            telemetry_ready = has_telemetry_;
        }

        if (!telemetry_ready) {
            sleep_control_period();
            continue;
        }

        if (!initialized) {
            speed_controller_.reset(telemetry_copy.state.attitude);
            torque_controller_.reset();
            model_adapter_.reset();
            initialized = true;
        }

        const GuidanceCommand command = deps_.command_source->read();
        const AttitudeSetpoint attitude = speed_controller_.update(
            telemetry_copy.state,
            command,
            kDefaultControlPeriodSec);
        const TorqueCommand torque = model_adapter_.update(telemetry_copy.state, attitude);
        const MotorPwmFrame pwm = torque_controller_.mix(attitude.collective, torque, kDefaultControlPeriodSec);

        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_solution_ = {attitude, torque, pwm};
            has_solution_ = true;
        }
        sleep_control_period();
    }
}

void FlightApplication::actuation_loop() {
    while (!deps_.task_runner->stop_requested()) {
        ControlSolution solution{};
        bool solution_ready = false;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            solution = latest_solution_;
            solution_ready = has_solution_;
        }

        if (solution_ready) {
            deps_.pwm_output->write(solution.motor_pwm);
        }
        sleep_control_period();
    }
}

void FlightApplication::log_loop() {
    while (!deps_.task_runner->stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        FlightTelemetry telemetry_copy{};
        ControlSolution solution_copy{};
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            telemetry_copy = latest_telemetry_;
            solution_copy = latest_solution_;
        }

        const auto& state = telemetry_copy.state;
        std::cout << std::fixed << std::setprecision(3)
                  << "t=" << state.timestamp_sec
                  << " pos=(" << state.position_m.x << "," << state.position_m.y << "," << state.position_m.z << ")"
                  << " vel=(" << state.velocity_m_s.x << "," << state.velocity_m_s.y << "," << state.velocity_m_s.z << ")"
                  << " pwm0=" << solution_copy.motor_pwm.pwm_us[0]
                  << '\n';
    }
}

}  // namespace flight_control

