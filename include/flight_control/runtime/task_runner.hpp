#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace flight_control {

/**
 * 任务运行器接口。
 *
 * 用于屏蔽 FreeRTOS、host 线程或其他实时调度器差异。
 * 飞控核心只依赖这个抽象，不直接依赖 std::thread 或具体 RTOS API。
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
    /**
     * 挂起当前任务一段时间。
     *
     * @param duration_ms 挂起时长，单位 ms。
     */
    virtual void sleep_ms(std::uint32_t duration_ms) = 0;
    /** 等待所有任务退出。 */
    virtual void join() = 0;
};

}  // namespace flight_control
