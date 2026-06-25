#pragma once

#include "flight_control/runtime/task_runner.hpp"

namespace flight_control {

class FreertosTaskRunner final : public ITaskRunner {
public:
    FreertosTaskRunner();
    ~FreertosTaskRunner() override;

    void spawn(const std::string& name, std::function<void()> task) override;
    void request_stop() override;
    bool stop_requested() const override;
    void join() override;

private:
    bool enabled_{false};
};

}  // namespace flight_control

