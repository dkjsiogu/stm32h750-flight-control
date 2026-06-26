#pragma once

#include <mutex>

#include "flight_control/runtime/synchronization.hpp"

namespace flight_control {

/**
 * host 互斥临界区。
 *
 * PC 侧 demo 和评估工具使用 std::mutex 保护跨线程快照。
 * 该实现位于 host 平台层，不进入 STM32 固件核心。
 */
class HostMutexCriticalSection final : public ICriticalSection {
public:
    /** 进入 host mutex 临界区。 */
    void lock() override;
    /** 退出 host mutex 临界区。 */
    void unlock() override;

private:
    /** host 互斥锁对象。 */
    std::mutex mutex_;
};

}  // namespace flight_control
