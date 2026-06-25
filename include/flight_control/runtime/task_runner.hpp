#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace flight_control {

class ITaskRunner {
public:
    virtual ~ITaskRunner() = default;
    virtual void spawn(const std::string& name, std::function<void()> task) = 0;
    virtual void request_stop() = 0;
    virtual bool stop_requested() const = 0;
    virtual void join() = 0;
};

class ThreadTaskRunner final : public ITaskRunner {
public:
    ThreadTaskRunner() = default;
    ~ThreadTaskRunner() override;

    void spawn(const std::string& name, std::function<void()> task) override;
    void request_stop() override;
    bool stop_requested() const override;
    void join() override;

private:
    std::atomic_bool stop_requested_{false};
    std::vector<std::thread> threads_;
};

}  // namespace flight_control

