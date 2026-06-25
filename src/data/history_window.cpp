#include "flight_control/data/history_window.hpp"

#include <algorithm>

namespace flight_control {

HistoryWindow::HistoryWindow(std::size_t history_frames)
    : active_frames_(std::clamp(history_frames, std::size_t{1}, kHistoryFrames)) {
    reset(ModelFrame{});
}

void HistoryWindow::reset(const ModelFrame& frame) {
    frames_.fill(frame);
}

void HistoryWindow::append(const ModelFrame& frame) {
    for (std::size_t index = 0; index + 1 < kHistoryFrames; ++index) {
        frames_[index] = frames_[index + 1];
    }
    frames_[kHistoryFrames - 1] = frame;
}

std::array<float, kModelInputDim> HistoryWindow::flattened() const {
    std::array<float, kModelInputDim> output{};
    for (std::size_t frame_index = 0; frame_index < kHistoryFrames; ++frame_index) {
        const std::size_t offset = frame_index * kFrameDim;
        for (std::size_t value_index = 0; value_index < kFrameDim; ++value_index) {
            output[offset + value_index] = frames_[frame_index].values[value_index];
        }
    }
    return output;
}

std::size_t HistoryWindow::size() const noexcept {
    return active_frames_;
}

}  // namespace flight_control

