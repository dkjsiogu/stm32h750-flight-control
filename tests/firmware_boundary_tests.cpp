#include <fstream>
#include <iostream>
#include <filesystem>
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
    check(!contains(cmake, "flight_control_host_support"), "firmware repository must not define host support target");
    check(!contains(cmake, "flight_control_eval"), "firmware repository must not define evaluation executable");
    check(!contains(cmake, "flight_control_policy_search"), "firmware repository must not define policy search executable");
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

void test_simulation_sources_are_not_in_firmware_repository() {
    const std::vector<std::string> forbidden_paths{
        "include/flight_control/platform/host",
        "include/flight_control/eval",
        "src/platform/host",
        "src/eval",
        "src/host",
        "src/evaluate.cpp",
        "tools/policy_search.cpp",
        "tools/export_linear_policy.py",
        "tools/static_policy_params.txt",
    };

    for (const std::string& path : forbidden_paths) {
        check(!std::filesystem::exists(path), "simulation sources must live in the standalone sim repository");
    }
}

void test_h743_real_driver_mode_is_explicit() {
    const std::string cmake = read_file("boards/stm32h743/CMakeLists.txt");
    const std::string board_api = read_file("boards/stm32h743/include/board_io.hpp");

    check(contains(cmake, "H743_DRIVER_MODE"), "H743 board must expose explicit driver mode");
    check(contains(cmake, "drivers/real"), "H743 real mode must load real driver directory");
    check(contains(cmake, "drivers/stub"), "H743 stub mode must be isolated from real drivers");
    check(contains(cmake, "H743_FETCH_FREERTOS"), "H743 real mode must be able to fetch FreeRTOS automatically");
    check(contains(cmake, "H743_FREERTOS_KERNEL_DIR"), "H743 real mode must support FreeRTOS kernel source wiring");
    check(contains(cmake, "真实 H743 驱动模式需要"), "H743 real mode must fail when drivers are missing");
    check(contains(board_api, "board_system_preinit"), "H743 board API must expose system preinit hook");
    check(contains(board_api, "board_drivers_initialize"), "H743 board API must expose driver init hook");
    check(contains(board_api, "board_write_failsafe_outputs"), "H743 board API must expose failsafe output hook");
}

}  // namespace

int main() {
    test_firmware_core_target_is_host_free();
    test_core_headers_do_not_expose_host_or_truth();
    test_simulation_sources_are_not_in_firmware_repository();
    test_h743_real_driver_mode_is_explicit();
    std::cout << "firmware_boundary_tests passed\n";
    return 0;
}
