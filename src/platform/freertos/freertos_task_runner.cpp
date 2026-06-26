#include "flight_control/platform/freertos/freertos_task_runner.hpp"

#include <utility>

#ifdef FLIGHT_CONTROL_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

namespace flight_control {

namespace {

#ifdef FLIGHT_CONTROL_USE_FREERTOS
/** FreeRTOS 静态任务控制块数组。 */
std::array<StaticTask_t, FreertosTaskRunner::kMaxTasks> g_task_blocks{};
/** FreeRTOS 静态任务栈数组。 */
std::array<std::array<StackType_t, FreertosTaskRunner::kStackDepthWords>, FreertosTaskRunner::kMaxTasks> g_task_stacks{};

/**
 * FreeRTOS 任务入口转接函数。
 *
 * @param parameter 指向 std::function<void()> 的任务入口对象。
 */
void task_entry(void* parameter) {
    auto* task = static_cast<std::function<void()>*>(parameter);
    (*task)();
    vTaskDelete(nullptr);
}
#endif

}  // namespace

FreertosTaskRunner::FreertosTaskRunner() {
    stop_requested_ = false;
}

FreertosTaskRunner::~FreertosTaskRunner() = default;

void FreertosTaskRunner::spawn(const std::string& name, std::function<void()> task) {
    const std::size_t index = task_count_++;
    if (index >= tasks_.size()) {
        stop_requested_ = true;
        return;
    }
    tasks_[index] = std::move(task);
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    xTaskCreateStatic(
        task_entry,
        name.c_str(),
        kStackDepthWords,
        &tasks_[index],
        kTaskPriority,
        g_task_stacks[index].data(),
        &g_task_blocks[index]);
#else
    (void)name;
#endif
}

void FreertosTaskRunner::request_stop() {
    stop_requested_ = true;
}

bool FreertosTaskRunner::stop_requested() const {
    return stop_requested_;
}

void FreertosTaskRunner::sleep_ms(std::uint32_t duration_ms) {
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
#else
    (void)duration_ms;
#endif
}

void FreertosTaskRunner::join() {
#ifdef FLIGHT_CONTROL_USE_FREERTOS
    vTaskStartScheduler();
#endif
}

}  // namespace flight_control
