#pragma once

namespace flight_control {

/**
 * 临界区接口。
 *
 * 用于让飞控核心在 FreeRTOS、STM32 中断屏蔽或外部验证锁之间切换。
 * 核心应用层不直接包含 std::mutex，也不直接调用平台相关临界区 API。
 */
class ICriticalSection {
public:
    /** 虚析构，允许通过基类指针释放派生对象。 */
    virtual ~ICriticalSection() = default;
    /** 进入临界区。 */
    virtual void lock() = 0;
    /** 退出临界区。 */
    virtual void unlock() = 0;
};

/**
 * 临界区自动守卫。
 *
 * 构造时进入临界区，析构时退出临界区，避免控制循环异常路径遗留锁状态。
 */
class CriticalSectionGuard final {
public:
    /**
     * 构造临界区守卫。
     *
     * @param section 需要保护的临界区对象。
     */
    explicit CriticalSectionGuard(ICriticalSection& section);
    /** 析构时退出临界区。 */
    ~CriticalSectionGuard();

    /** 禁止复制，避免重复退出同一个临界区。 */
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    /** 禁止赋值，避免重复退出同一个临界区。 */
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;

private:
    /** 被守卫的临界区对象。 */
    ICriticalSection& section_;
};

/**
 * 空临界区实现。
 *
 * 用于单线程固件初始化阶段或测试中不需要并发保护的场景。
 */
class NullCriticalSection final : public ICriticalSection {
public:
    /** 进入空临界区，不执行任何操作。 */
    void lock() override;
    /** 退出空临界区，不执行任何操作。 */
    void unlock() override;
};

}  // namespace flight_control
