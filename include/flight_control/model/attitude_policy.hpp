#pragma once

#include <algorithm>
#include <array>
#include <memory>

#include "flight_control/config.hpp"

namespace flight_control {

/**
 * 姿态控制策略接口。
 *
 * 任何神经网络、规则控制或导出的静态模型都可以实现这个接口，
 * 输入是历史窗口展开后的神经网络特征，输出是归一化力矩动作。
 */
class IAttitudePolicy {
public:
    /** 虚析构，允许通过基类指针释放派生对象。 */
    virtual ~IAttitudePolicy() = default;
    /**
     * 预测归一化姿态动作。
     *
     * @param input 神经网络输入，维度由 kModelInputDim 决定。
     * @return 三轴归一化动作，范围通常为 [-1, 1]。
     */
    virtual std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) = 0;
};

/**
 * 规则型姿态控制策略。
 *
 * 这是一个轻量 fallback，用于在没有静态权重时验证接口和链路。
 * 它根据姿态误差、角速度和上一动作做一个简单的阻尼型输出。
 */
class HeuristicAttitudePolicy final : public IAttitudePolicy {
public:
    /**
     * 构造规则型姿态控制策略。
     *
     * @param kp 姿态误差比例增益。
     * @param kd 角速度阻尼增益。
     * @param prev_gain 上一动作反馈增益。
     */
    HeuristicAttitudePolicy(float kp = 3.2f, float kd = 0.42f, float prev_gain = 0.06f)
        : kp_(kp), kd_(kd), prev_gain_(prev_gain) {}

    /**
     * 计算归一化动作。
     *
     * @param input 展开的历史输入向量。
     * @return 归一化姿态动作。
     */
    std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) override {
        /** 最近一帧历史输入的起始地址。 */
        const float* latest = input.data() + (kHistoryFrames - 1) * kFrameDim;
        /** 最近一帧中的姿态误差 x 分量。 */
        const float qx = latest[0];
        /** 最近一帧中的姿态误差 y 分量。 */
        const float qy = latest[1];
        /** 最近一帧中的姿态误差 z 分量。 */
        const float qz = latest[2];
        /** 最近一帧中的角速度 x 分量。 */
        const float wx = latest[3];
        /** 最近一帧中的角速度 y 分量。 */
        const float wy = latest[4];
        /** 最近一帧中的角速度 z 分量。 */
        const float wz = latest[5];
        /** 最近一帧中的上一动作 x 分量。 */
        const float px = latest[6];
        /** 最近一帧中的上一动作 y 分量。 */
        const float py = latest[7];
        /** 最近一帧中的上一动作 z 分量。 */
        const float pz = latest[8];

        /** 规则策略输出的三轴归一化动作。 */
        std::array<float, 3> action{
            -kp_ * qx - kd_ * wx - prev_gain_ * px,
            -kp_ * qy - kd_ * wy - prev_gain_ * py,
            -kp_ * qz - kd_ * wz - prev_gain_ * pz,
        };

        /** 将每个动作分量限制在 [-1, 1]。 */
        for (float& value : action) {
            value = std::clamp(value, -1.0f, 1.0f);
        }
        return action;
    }

private:
    /** 姿态误差比例增益。 */
    float kp_{3.2f};
    /** 角速度阻尼增益。 */
    float kd_{0.42f};
    /** 上一动作反馈增益。 */
    float prev_gain_{0.06f};
};

}  // namespace flight_control
