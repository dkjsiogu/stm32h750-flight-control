#pragma once

#include <array>
#include <cstddef>
#include <functional>

#include "flight_control/runtime/task_runner.hpp"

namespace flight_control {

/**
 * FreeRTOS 任务运行器。
 *
 * 这是 STM32/FreeRTOS 端口的任务调度适配层，
 * 用来把飞控应用的任务抽象映射到 FreeRTOS task。
 */
class FreertosTaskRunner final : public ITaskRunner {
public:
    /** 最多可由飞控应用创建的实时任务数量。 */
    static constexpr std::size_t kMaxTasks = 4;
    /** 每个任务的栈深度，单位为 FreeRTOS StackType_t 元素。 */
    static constexpr std::size_t kStackDepthWords = 1024;
    /** 每个任务使用的 FreeRTOS 优先级。 */
    static constexpr std::uint32_t kTaskPriority = 5;

    /** 构造 FreeRTOS 任务运行器。 */
    FreertosTaskRunner();
    /** 析构 FreeRTOS 任务运行器并释放相关资源。 */
    ~FreertosTaskRunner() override;

    /**
     * 创建 FreeRTOS 任务。
     *
     * @param name 任务名称。
     * @param task 任务入口函数。
     */
    void spawn(const std::string& name, std::function<void()> task) override;
    /** 请求任务停止。 */
    void request_stop() override;
    /**
     * 查询停止请求状态。
     *
     * @return true 表示任务应停止。
     */
    bool stop_requested() const override;
    /**
     * 让当前 FreeRTOS 任务延时。
     *
     * @param duration_ms 延时时长，单位 ms。
     */
    void sleep_ms(std::uint32_t duration_ms) override;
    /** 等待任务结束；在嵌入式端可按端口实现决定是否阻塞。 */
    void join() override;

private:
    /** 记录 FreeRTOS 任务入口函数，保证任务运行期间函数对象有效。 */
    std::array<std::function<void()>, kMaxTasks> tasks_{};
    /** 已创建任务数量。 */
    std::size_t task_count_{0};
    /** 当前 FreeRTOS 端口是否已启用。 */
    volatile bool stop_requested_{false};
};

}  // namespace flight_control
