#include "flight_control/platform/host/thread_task_runner.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <utility>

namespace flight_control {

ThreadTaskRunner::~ThreadTaskRunner() {
    request_stop();
    join();
}

void ThreadTaskRunner::spawn(const std::string& name, std::function<void()> task) {
    threads_.emplace_back([this, name, task = std::move(task)]() mutable {
        try {
            task();
        } catch (const std::exception& error) {
            std::cerr << "task " << name << " stopped: " << error.what() << '\n';
            request_stop();
        } catch (...) {
            std::cerr << "task " << name << " stopped with an unknown error\n";
            request_stop();
        }
    });
}

void ThreadTaskRunner::request_stop() {
    stop_requested_.store(true);
}

bool ThreadTaskRunner::stop_requested() const {
    return stop_requested_.load();
}

void ThreadTaskRunner::sleep_ms(std::uint32_t duration_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
}

void ThreadTaskRunner::join() {
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads_.clear();
}

}  // namespace flight_control
