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

/**
 * 飞控应用程序。
 *
 * 负责把任务调度、传感器输入、控制逻辑和 PWM 输出串起来，
 * 在 host 或 FreeRTOS 环境下都能以同一条控制链路运行。
 */
class FlightApplication {
public:
    /**
     * 应用依赖项。
     *
     * 通过接口注入任务运行器、传感器源、指令源、PWM 输出和姿态策略。
     */
    struct Dependencies {
        /** 任务运行器，用于启动 acquisition/control/actuation/log 线程。 */
        std::shared_ptr<ITaskRunner> task_runner;
        /** 传感器数据源。 */
        std::shared_ptr<ISensorSource> sensor_source;
        /** 指令数据源。 */
        std::shared_ptr<ICommandSource> command_source;
        /** PWM 输出接口。 */
        std::shared_ptr<IPwmOutput> pwm_output;
        /** 姿态控制策略。 */
        std::shared_ptr<IAttitudePolicy> policy;
    };

    /**
     * 构造飞控应用。
     *
     * @param deps 外部依赖注入。
     * @param speed_config 速度控制器配置。
     * @param torque_config 力矩控制器配置。
     * @param model_config 姿态模型适配器配置。
     */
    explicit FlightApplication(Dependencies deps,
                               SpeedControllerConfig speed_config = {},
                               TorqueControllerConfig torque_config = {},
                               ModelAdapterConfig model_config = {});

    /**
     * 运行一次演示链路。
     *
     * @param duration 演示运行时长。
     */
    void run_demo(std::chrono::milliseconds duration);
    /** 请求所有任务停止。 */
    void request_stop();
    /** 等待所有任务退出。 */
    void join();
    /**
     * 获取最近一次控制结果快照。
     *
     * @return 最近一次控制解。
     */
    ControlSolution snapshot() const;
    /**
     * 获取最近一次遥测快照。
     *
     * @return 最近一次遥测数据。
     */
    FlightTelemetry telemetry() const;

private:
    /** 采集线程循环。 */
    void acquisition_loop();
    /** 控制线程循环。 */
    void control_loop();
    /** 执行器线程循环。 */
    void actuation_loop();
    /** 日志线程循环。 */
    void log_loop();

    /** 注入依赖。 */
    Dependencies deps_;
    /** 速度控制器实例。 */
    SpeedController speed_controller_;
    /** 力矩控制器实例。 */
    TorqueController torque_controller_;
    /** 姿态模型适配器实例。 */
    ModelAdapter model_adapter_;

    /** 保护快照数据的互斥锁。 */
    mutable std::mutex snapshot_mutex_;
    /** 最近一次遥测。 */
    FlightTelemetry latest_telemetry_{};
    /** 最近一次控制解。 */
    ControlSolution latest_solution_{};
    /** 是否已经收到了至少一帧遥测。 */
    bool has_telemetry_{false};
    /** 是否已经生成至少一帧控制解。 */
    bool has_solution_{false};
};

}  // namespace flight_control
