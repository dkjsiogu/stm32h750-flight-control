#pragma once

#include <memory>

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

/**
 * 传感器源接口。
 *
 * 任何真实硬件、记录回放或仿真器都可以实现这个接口，
 * 供采集线程获取最新遥测数据。
 */
class ISensorSource {
public:
    /** 虚析构，允许通过基类指针销毁派生对象。 */
    virtual ~ISensorSource() = default;
    /**
     * 读取一次遥测数据。
     *
     * @return 最新飞行遥测。
     */
    virtual FlightTelemetry read() = 0;
};

/**
 * 指令源接口。
 *
 * 上层控制站、手柄、自动任务或测试程序都可以通过这个接口提供飞行指令。
 */
class ICommandSource {
public:
    /** 虚析构，允许通过基类指针销毁派生对象。 */
    virtual ~ICommandSource() = default;
    /**
     * 读取一次导航或人工控制指令。
     *
     * @return 最新飞行指令。
     */
    virtual GuidanceCommand read() = 0;
};

/**
 * PWM 输出接口。
 *
 * 执行器层通过这个接口把混控后的 PWM 帧写给硬件或仿真器。
 */
class IPwmOutput {
public:
    /** 虚析构，允许通过基类指针销毁派生对象。 */
    virtual ~IPwmOutput() = default;
    /**
     * 写入一帧 PWM 输出。
     *
     * @param frame 四路电机 PWM 帧。
     */
    virtual void write(const MotorPwmFrame& frame) = 0;
};

}  // namespace flight_control
