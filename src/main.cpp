#include <memory>

#include "flight_control/app/flight_application.hpp"
#include "flight_control/model/generated_policy.hpp"
#include "flight_control/platform/freertos/freertos_task_runner.hpp"
#include "flight_control/platform/stm32/stm32_flight_port.hpp"

/**
 * STM32 固件主入口。
 *
 * 该入口只装配真实板级接口：传感器、速度指令、PWM 输出、FreeRTOS 任务和临界区。
 * host 仿真、闭环评估和文件输出全部在 PC 侧 target 中，不能进入这个入口。
 */
int main() {
    using namespace flight_control;

    auto runner = std::make_shared<FreertosTaskRunner>();
    auto sensors = std::make_shared<Stm32SensorSource>();
    auto commands = std::make_shared<Stm32CommandSource>();
    auto pwm_output = std::make_shared<Stm32PwmOutput>();
    auto policy = std::make_shared<StaticMlpPolicy>(make_generated_policy_weights());
    auto critical_section = std::make_shared<Stm32CriticalSection>();

    FlightApplication app({
        runner,
        sensors,
        commands,
        pwm_output,
        policy,
        critical_section,
    }, {}, {}, generated_model_config());

    app.start();
    runner->join();
    return 0;
}
