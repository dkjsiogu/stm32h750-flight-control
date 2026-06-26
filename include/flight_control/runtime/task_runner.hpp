#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace flight_control {

/**
 * 任务运行器接口。
 *
 * 用于屏蔽 pthread/std::thread 与 FreeRTOS 的调度差异，
 * 飞控应用只依赖这个抽象来启动和管理后台任务。
 */
class ITaskRunner {
public:
    /** 虚析构，允许通过基类指针销毁派生对象。 */
    virtual ~ITaskRunner() = default;
    /**
     * 启动一个命名任务。
     *
     * @param name 任务名称，用于日志和调试。
     * @param task 任务入口函数。
     */
    virtual void spawn(const std::string& name, std::function<void()> task) = 0;
    /** 请求所有任务停止。 */
    virtual void request_stop() = 0;
    /**
     * 查询是否已经请求停止。
     *
     * @return true 表示应停止任务。
     */
    virtual bool stop_requested() const = 0;
    /** 等待所有任务退出。 */
    virtual void join() = 0;
};

/**
 * 基于 std::thread 的任务运行器。
 *
 * 这是 host 侧默认实现，用于在 PC 上模拟 FreeRTOS 风格的多线程运行。
 */
class ThreadTaskRunner final : public ITaskRunner {
public:
    /** 构造线程任务运行器。 */
    ThreadTaskRunner() = default;
    /** 析构时等待所有线程退出。 */
    ~ThreadTaskRunner() override;

    /**
     * 启动一个线程任务。
     *
     * @param name 任务名称，仅用于调试和日志。
     * @param task 任务入口函数。
     */
    void spawn(const std::string& name, std::function<void()> task) override;
    /** 请求停止所有任务。 */
    void request_stop() override;
    /** 查询停止请求状态。 */
    bool stop_requested() const override;
    /** 等待所有线程退出。 */
    void join() override;

private:
    /** 停止标志。 */
    std::atomic_bool stop_requested_{false};
    /** 已启动线程列表。 */
    std::vector<std::thread> threads_;
};

}  // namespace flight_control
