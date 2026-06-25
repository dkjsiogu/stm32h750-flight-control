#include "flight_control/platform/freertos/freertos_task_runner.hpp"

#include <stdexcept>

namespace flight_control {

FreertosTaskRunner::FreertosTaskRunner() {
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    enabled_ = true;
#endif
}

FreertosTaskRunner::~FreertosTaskRunner() = default;

void FreertosTaskRunner::spawn(const std::string& name, std::function<void()> task) {
    (void)name;
    (void)task;
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    // STM32 port hook: allocate static task storage and call xTaskCreateStatic here.
#else
    throw std::logic_error("FreertosTaskRunner requires FLIGHT_CONTROL_USE_FREERTOS");
#endif
}

void FreertosTaskRunner::request_stop() {
    enabled_ = false;
}

bool FreertosTaskRunner::stop_requested() const {
    return !enabled_;
}

void FreertosTaskRunner::join() {
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    // STM32 port hook: vTaskStartScheduler normally never returns.
#endif
}

}  // namespace flight_control

