#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string read_file(const std::string& path) {
    std::ifstream input(path);
    check(input.good(), path.c_str());
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

bool contains(const std::string& text, const std::string& pattern) {
    return text.find(pattern) != std::string::npos;
}

void test_firmware_core_target_is_host_free() {
    const std::string cmake = read_file("CMakeLists.txt");
    const std::string marker = "add_library(flight_control_firmware_core";
    const std::size_t begin = cmake.find(marker);
    check(begin != std::string::npos, "firmware core target must exist");
    const std::size_t end = cmake.find("target_include_directories(flight_control_firmware_core", begin);
    check(end != std::string::npos, "firmware core target block must end before include directories");
    const std::string target_block = cmake.substr(begin, end - begin);

    check(!contains(target_block, "src/platform/host/"), "firmware core must not compile host platform sources");
    check(!contains(target_block, "src/eval/"), "firmware core must not compile evaluation sources");
    check(!contains(target_block, "src/host/"), "firmware core must not compile host demo sources");
}

void test_core_headers_do_not_expose_host_or_truth() {
    const std::vector<std::string> core_headers{
        "include/flight_control/data/flight_types.hpp",
        "include/flight_control/runtime/task_runner.hpp",
        "include/flight_control/app/flight_application.hpp",
    };

    for (const std::string& path : core_headers) {
        const std::string content = read_file(path);
        check(!contains(content, "HostFlightEnvironment"), "core headers must not expose host environment");
        check(!contains(content, "ThreadTaskRunner"), "core headers must not expose host thread runner");
        check(!contains(content, "truth{}"), "core telemetry must not carry simulator truth");
        check(!contains(content, "<thread>"), "core headers must not include std::thread");
        check(!contains(content, "<iostream>"), "core headers must not include iostream");
    }
}

}  // namespace

int main() {
    test_firmware_core_target_is_host_free();
    test_core_headers_do_not_expose_host_or_truth();
    std::cout << "firmware_boundary_tests passed\n";
    return 0;
}
