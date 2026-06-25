#pragma once

#include <cstddef>

namespace flight_control {

constexpr std::size_t kHistoryFrames = 4;
constexpr std::size_t kFrameDim = 9;
constexpr std::size_t kModelInputDim = kHistoryFrames * kFrameDim;

constexpr float kGravity = 9.81f;
constexpr float kDefaultControlPeriodSec = 0.004f;
constexpr float kPi = 3.14159265358979323846f;

}  // namespace flight_control

