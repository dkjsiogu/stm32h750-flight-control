#pragma once

#include <memory>

#include "flight_control/data/flight_types.hpp"

namespace flight_control {

class ISensorSource {
public:
    virtual ~ISensorSource() = default;
    virtual FlightTelemetry read() = 0;
};

class ICommandSource {
public:
    virtual ~ICommandSource() = default;
    virtual GuidanceCommand read() = 0;
};

class IPwmOutput {
public:
    virtual ~IPwmOutput() = default;
    virtual void write(const MotorPwmFrame& frame) = 0;
};

}  // namespace flight_control

