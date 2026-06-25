#pragma once

#include <array>
#include <cstddef>

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

class HistoryWindow {
public:
    explicit HistoryWindow(std::size_t history_frames = kHistoryFrames);

    void reset(const ModelFrame& frame);
    void append(const ModelFrame& frame);
    std::array<float, kModelInputDim> flattened() const;
    std::size_t size() const noexcept;

private:
    std::array<ModelFrame, kHistoryFrames> frames_{};
    std::size_t active_frames_{kHistoryFrames};
};

}  // namespace flight_control

