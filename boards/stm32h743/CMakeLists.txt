cmake_minimum_required(VERSION 3.20)
project(stm32h743_flight_board LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(H743_FLIGHT_CONTROL_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "飞控核心仓库路径")
option(H743_USE_FREERTOS "启用真实 FreeRTOS 任务调度器" OFF)
set(H743_FREERTOS_INCLUDE_DIRS "" CACHE STRING "FreeRTOS 头文件目录，多个目录用分号分隔")

if(H743_USE_FREERTOS AND NOT H743_FREERTOS_INCLUDE_DIRS)
    message(FATAL_ERROR "H743_USE_FREERTOS=ON 时必须设置 H743_FREERTOS_INCLUDE_DIRS")
endif()

set(FLIGHT_CONTROL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY ON CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_USE_FREERTOS ${H743_USE_FREERTOS} CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS "${H743_FREERTOS_INCLUDE_DIRS}" CACHE STRING "" FORCE)

add_subdirectory("${H743_FLIGHT_CONTROL_DIR}" flight_control_core)

target_sources(flight_control_firmware
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/startup_stm32h743xx.s
        ${CMAKE_CURRENT_LIST_DIR}/src/board_hooks.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/syscalls.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/system_stm32h743.cpp
)

target_include_directories(flight_control_firmware
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_options(flight_control_firmware
    PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -ffunction-sections
        -fdata-sections
)

target_link_options(flight_control_firmware
    PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -T${CMAKE_CURRENT_LIST_DIR}/stm32h743xx_flash.ld
        -Wl,--gc-sections
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/flight_control_firmware.map
)

set_target_properties(flight_control_firmware PROPERTIES OUTPUT_NAME flight_control_h743 SUFFIX ".elf")

set(H743_HEX_FILE "${CMAKE_CURRENT_BINARY_DIR}/flight_control_h743.hex")
set(H743_BIN_FILE "${CMAKE_CURRENT_BINARY_DIR}/flight_control_h743.bin")

add_custom_command(
    OUTPUT
        ${H743_HEX_FILE}
        ${H743_BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:flight_control_firmware> ${H743_HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:flight_control_firmware> ${H743_BIN_FILE}
    DEPENDS flight_control_firmware
    COMMENT "生成 STM32H743 hex/bin 烧录文件"
)

add_custom_target(flight_control_h743_images ALL
    DEPENDS
        ${H743_HEX_FILE}
        ${H743_BIN_FILE}
)
