cmake_minimum_required(VERSION 3.20)
project(stm32h750_flight_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FLIGHT_CONTROL_BUILD_TESTS "Build host tests" ON)

add_library(flight_control_core
    src/control/speed_controller.cpp
    src/control/torque_controller.cpp
    src/data/history_window.cpp
    src/model/model_adapter.cpp
    src/model/generated_policy.cpp
    src/model/static_mlp_policy.cpp
    src/runtime/task_runner.cpp
    src/app/flight_application.cpp
    src/platform/host/simulated_components.cpp
)

target_include_directories(flight_control_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)

add_executable(flight_control_demo
    src/main.cpp
)
target_link_libraries(flight_control_demo PRIVATE flight_control_core Threads::Threads)

if(FLIGHT_CONTROL_BUILD_TESTS)
    enable_testing()
    add_executable(flight_control_tests
        tests/flight_control_tests.cpp
    )
    target_link_libraries(flight_control_tests PRIVATE flight_control_core Threads::Threads)
    add_test(NAME flight_control_tests COMMAND flight_control_tests)
endif()
