#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/interfaces/flight_interfaces.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/runtime/task_runner.hpp"

namespace flight_control {

class FlightApplication {
public:
    struct Dependencies {
        std::shared_ptr<ITaskRunner> task_runner;
        std::shared_ptr<ISensorSource> sensor_source;
        std::shared_ptr<ICommandSource> command_source;
        std::shared_ptr<IPwmOutput> pwm_output;
        std::shared_ptr<IAttitudePolicy> policy;
    };

    explicit FlightApplication(Dependencies deps,
                               SpeedControllerConfig speed_config = {},
                               TorqueControllerConfig torque_config = {},
                               ModelAdapterConfig model_config = {});

    void run_demo(std::chrono::milliseconds duration);
    void request_stop();
    void join();
    ControlSolution snapshot() const;
    FlightTelemetry telemetry() const;

private:
    void acquisition_loop();
    void control_loop();
    void actuation_loop();
    void log_loop();

    Dependencies deps_;
    SpeedController speed_controller_;
    TorqueController torque_controller_;
    ModelAdapter model_adapter_;

    mutable std::mutex snapshot_mutex_;
    FlightTelemetry latest_telemetry_{};
    ControlSolution latest_solution_{};
    bool has_telemetry_{false};
    bool has_solution_{false};
};

}  // namespace flight_control
