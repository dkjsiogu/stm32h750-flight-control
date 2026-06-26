#pragma once

#include <atomic>
#include <thread>
#include <vector>

#include "flight_control/runtime/task_runner.hpp"

namespace flight_control {

/**
 * host 线程任务运行器。
 *
 * 这是 PC 侧验证工具使用的任务运行器，基于 std::thread 实现。
 * 它不属于可烧录固件核心，不能被 STM32 固件入口依赖。
 */
class ThreadTaskRunner final : public ITaskRunner {
public:
    /** 构造 host 线程任务运行器。 */
    ThreadTaskRunner() = default;
    /** 析构时请求停止并等待所有线程退出。 */
    ~ThreadTaskRunner() override;

    /**
     * 启动一个 host 线程任务。
     *
     * @param name 任务名称，仅用于 host 调试输出。
     * @param task 任务入口函数。
     */
    void spawn(const std::string& name, std::function<void()> task) override;
    /** 请求停止所有 host 线程任务。 */
    void request_stop() override;
    /**
     * 查询停止请求状态。
     *
     * @return true 表示线程任务应停止。
     */
    bool stop_requested() const override;
    /**
     * 挂起当前 host 线程一段时间。
     *
     * @param duration_ms 挂起时长，单位 ms。
     */
    void sleep_ms(std::uint32_t duration_ms) override;
    /** 等待所有 host 线程任务退出。 */
    void join() override;

private:
    /** 停止标志。 */
    std::atomic_bool stop_requested_{false};
    /** 已启动线程列表。 */
    std::vector<std::thread> threads_;
};

}  // namespace flight_control
