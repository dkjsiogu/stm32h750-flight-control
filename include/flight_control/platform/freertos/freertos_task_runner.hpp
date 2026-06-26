#pragma once

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
    /** 等待任务结束；在嵌入式端可按端口实现决定是否阻塞。 */
    void join() override;

private:
    /** 当前 FreeRTOS 端口是否已启用。 */
    bool enabled_{false};
};

}  // namespace flight_control
