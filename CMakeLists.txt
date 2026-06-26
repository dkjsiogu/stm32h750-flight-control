cmake_minimum_required(VERSION 3.20)
project(stm32h750_flight_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FLIGHT_CONTROL_BUILD_TESTS "Build host tests" ON)
option(FLIGHT_CONTROL_BUILD_HOST_TOOLS "Build host demo and closed-loop evaluation tools" ON)
option(FLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY "Build STM32 firmware entry target with board hooks" OFF)
option(FLIGHT_CONTROL_USE_FREERTOS "Compile STM32 port against real FreeRTOS headers" OFF)
set(FLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS "" CACHE STRING "FreeRTOS include directories separated by semicolon")

add_library(flight_control_firmware_core
    src/control/speed_controller.cpp
    src/control/torque_controller.cpp
    src/data/history_window.cpp
    src/model/model_adapter.cpp
    src/model/generated_policy.cpp
    src/model/static_mlp_policy.cpp
    src/runtime/synchronization.cpp
    src/app/flight_application.cpp
)

target_include_directories(flight_control_firmware_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(flight_control_stm32_port
    src/platform/freertos/freertos_task_runner.cpp
    src/platform/stm32/stm32_board_hooks_weak.cpp
    src/platform/stm32/stm32_flight_port.cpp
)
target_link_libraries(flight_control_stm32_port PUBLIC flight_control_firmware_core)

if(FLIGHT_CONTROL_USE_FREERTOS)
    target_compile_definitions(flight_control_stm32_port PUBLIC FLIGHT_CONTROL_USE_FREERTOS)
    if(FLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS)
        target_include_directories(flight_control_stm32_port PUBLIC ${FLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS})
    endif()
endif()

if(FLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY)
    add_executable(flight_control_firmware
        src/main.cpp
    )
    target_link_libraries(flight_control_firmware PRIVATE flight_control_firmware_core flight_control_stm32_port)
endif()

if(FLIGHT_CONTROL_BUILD_HOST_TOOLS OR FLIGHT_CONTROL_BUILD_TESTS)
    find_package(Threads REQUIRED)

    add_library(flight_control_host_support
        src/platform/host/host_environment.cpp
        src/platform/host/host_synchronization.cpp
        src/platform/host/thread_task_runner.cpp
        src/eval/closed_loop_evaluator.cpp
    )
    target_link_libraries(flight_control_host_support PUBLIC flight_control_firmware_core Threads::Threads)
endif()

if(FLIGHT_CONTROL_BUILD_HOST_TOOLS)
    add_executable(flight_control_host_demo
        src/host/main.cpp
    )
    target_link_libraries(flight_control_host_demo PRIVATE flight_control_host_support)

    add_executable(flight_control_eval
        src/evaluate.cpp
    )
    target_link_libraries(flight_control_eval PRIVATE flight_control_host_support)

    add_executable(flight_control_policy_search
        tools/policy_search.cpp
    )
    target_link_libraries(flight_control_policy_search PRIVATE flight_control_host_support)
endif()

if(FLIGHT_CONTROL_BUILD_TESTS)
    enable_testing()

    add_executable(flight_control_tests
        tests/flight_control_tests.cpp
    )
    target_link_libraries(flight_control_tests PRIVATE flight_control_host_support)
    add_test(NAME flight_control_tests COMMAND flight_control_tests)

    add_executable(firmware_boundary_tests
        tests/firmware_boundary_tests.cpp
    )
    add_test(NAME firmware_boundary_tests COMMAND firmware_boundary_tests)
    set_tests_properties(firmware_boundary_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    add_executable(flight_control_system_tests
        tests/system_eval_tests.cpp
    )
    target_link_libraries(flight_control_system_tests PRIVATE flight_control_host_support)
    add_test(NAME flight_control_system_tests COMMAND flight_control_system_tests)
endif()
