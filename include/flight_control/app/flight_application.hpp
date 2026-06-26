#pragma once

#include <cstdint>
#include <memory>

#include "flight_control/control/speed_controller.hpp"
#include "flight_control/control/torque_controller.hpp"
#include "flight_control/interfaces/flight_interfaces.hpp"
#include "flight_control/model/model_adapter.hpp"
#include "flight_control/runtime/synchronization.hpp"
#include "flight_control/runtime/task_runner.hpp"

namespace flight_control {

/**
 * 飞控应用程序。
 *
 * 负责把任务调度、传感器输入、控制逻辑和 PWM 输出串起来，
 * 固件入口通过真实传感器、指令、PWM 和 FreeRTOS 端口注入硬件能力。
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
        /** 快照临界区，用于保护跨任务共享的遥测和控制结果。 */
        std::shared_ptr<ICriticalSection> critical_section;
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
     * 启动飞控实时任务。
     *
     * 该函数只创建采集、控制和执行器任务，不包含任何仿真或 host 日志逻辑。
     */
    void start();
    /**
     * 运行指定时长后停止。
     *
     * @param duration_ms 运行时长，单位 ms；host 验证可使用，固件通常直接调用 start。
     */
    void run_for_ms(std::uint32_t duration_ms);
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

    /** 注入依赖。 */
    Dependencies deps_;
    /** 速度控制器实例。 */
    SpeedController speed_controller_;
    /** 力矩控制器实例。 */
    TorqueController torque_controller_;
    /** 姿态模型适配器实例。 */
    ModelAdapter model_adapter_;

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
