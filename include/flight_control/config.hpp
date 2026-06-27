#pragma once

#include <cstddef>

namespace flight_control {

/** 飞控模型历史窗口包含的连续状态帧数量；16 帧在 250Hz 下约覆盖 64ms。 */
constexpr std::size_t kHistoryFrames = 16;

/** 单帧神经网络输入维度：姿态误差、角速度、上一动作、外环前馈、collective 和角加速度。 */
constexpr std::size_t kFrameDim = 16;

/** 神经网络完整输入维度，由历史帧数和单帧维度相乘得到。 */
constexpr std::size_t kModelInputDim = kHistoryFrames * kFrameDim;

/** 标准重力加速度，单位 m/s^2。 */
constexpr float kGravity = 9.81f;

/** 默认控制周期，单位秒；0.004 秒对应 250Hz 控制频率。 */
constexpr float kDefaultControlPeriodSec = 0.004f;

/** 圆周率常量，用于角度归一化和欧拉角计算。 */
constexpr float kPi = 3.14159265358979323846f;

}  // namespace flight_control
