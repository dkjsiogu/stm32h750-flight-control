#pragma once

#include <algorithm>
#include <array>
#include <memory>

#include "flight_control/config.hpp"

namespace flight_control {

class IAttitudePolicy {
public:
    virtual ~IAttitudePolicy() = default;
    virtual std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) = 0;
};

class HeuristicAttitudePolicy final : public IAttitudePolicy {
public:
    HeuristicAttitudePolicy(float kp = 3.2f, float kd = 0.42f, float prev_gain = 0.06f)
        : kp_(kp), kd_(kd), prev_gain_(prev_gain) {}

    std::array<float, 3> predict(const std::array<float, kModelInputDim>& input) override {
        const float* latest = input.data() + (kHistoryFrames - 1) * kFrameDim;
        const float qx = latest[0];
        const float qy = latest[1];
        const float qz = latest[2];
        const float wx = latest[3];
        const float wy = latest[4];
        const float wz = latest[5];
        const float px = latest[6];
        const float py = latest[7];
        const float pz = latest[8];

        std::array<float, 3> action{
            -kp_ * qx - kd_ * wx - prev_gain_ * px,
            -kp_ * qy - kd_ * wy - prev_gain_ * py,
            -kp_ * qz - kd_ * wz - prev_gain_ * pz,
        };

        for (float& value : action) {
            value = std::clamp(value, -1.0f, 1.0f);
        }
        return action;
    }

private:
    float kp_{3.2f};
    float kd_{0.42f};
    float prev_gain_{0.06f};
};

}  // namespace flight_control

