#pragma once

#include <array>
#include <cstddef>

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

/**
 * 固定长度历史输入窗口。
 *
 * 保存最近 kHistoryFrames 帧模型输入，并按从旧到新的顺序展开为神经网络输入。
 * 这个窗口用于让模型感知短时间内的姿态误差、角速度和上一动作变化。
 */
class HistoryWindow {
public:
    /**
     * 构造历史窗口。
     *
     * @param history_frames 期望历史帧数，当前实现固定使用 kHistoryFrames。
     */
    explicit HistoryWindow(std::size_t history_frames = kHistoryFrames);

    /**
     * 用同一帧重置整个历史窗口。
     *
     * @param frame 用于填充全部历史位置的模型帧。
     */
    void reset(const ModelFrame& frame);
    /**
     * 追加一帧新输入，并丢弃最旧的一帧。
     *
     * @param frame 最新模型输入帧。
     */
    void append(const ModelFrame& frame);
    /**
     * 将历史窗口展平为神经网络输入数组。
     *
     * @return 固定维度的模型输入数组。
     */
    std::array<float, kModelInputDim> flattened() const;
    /**
     * 返回当前有效历史帧数。
     *
     * @return 有效帧数量。
     */
    std::size_t size() const noexcept;

private:
    /** 固定长度历史帧缓存。 */
    std::array<ModelFrame, kHistoryFrames> frames_{};
    /** 当前有效历史帧数量。 */
    std::size_t active_frames_{kHistoryFrames};
};

}  // namespace flight_control
