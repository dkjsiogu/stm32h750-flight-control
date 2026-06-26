cmake_minimum_required(VERSION 3.20)
project(stm32h743_flight_board LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(H743_FLIGHT_CONTROL_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "飞控核心仓库路径")
set(H743_DRIVER_MODE "stub" CACHE STRING "驱动模式：stub 只做链接验证，real 使用真实 drivers/BSP")
set_property(CACHE H743_DRIVER_MODE PROPERTY STRINGS stub real)
option(H743_USE_FREERTOS "启用真实 FreeRTOS 任务调度器" OFF)
set(H743_DRIVER_SOURCES "" CACHE STRING "额外真实驱动源文件，多个文件用分号分隔")
set(H743_DRIVER_INCLUDE_DIRS "" CACHE STRING "额外真实驱动头文件目录，多个目录用分号分隔")
option(H743_FETCH_FREERTOS "未指定本地 FreeRTOS 时自动获取 FreeRTOS-Kernel" ON)
set(H743_FREERTOS_KERNEL_DIR "" CACHE PATH "FreeRTOS-Kernel 根目录；设置后自动加入内核源文件")
set(H743_FREERTOS_SOURCES "" CACHE STRING "额外 FreeRTOS 源文件，多个文件用分号分隔")
set(H743_FREERTOS_INCLUDE_DIRS "" CACHE STRING "FreeRTOS 头文件目录，多个目录用分号分隔")

if(NOT H743_DRIVER_MODE STREQUAL "stub" AND NOT H743_DRIVER_MODE STREQUAL "real")
    message(FATAL_ERROR "H743_DRIVER_MODE 只能是 stub 或 real")
endif()

if(H743_DRIVER_MODE STREQUAL "real")
    set(H743_USE_FREERTOS ON CACHE BOOL "启用真实 FreeRTOS 任务调度器" FORCE)
endif()

if(H743_DRIVER_MODE STREQUAL "real")
    file(GLOB_RECURSE H743_LOCAL_DRIVER_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/drivers/real/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/drivers/real/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/drivers/real/*.s"
        "${CMAKE_CURRENT_LIST_DIR}/drivers/real/*.S"
    )
    set(H743_SELECTED_DRIVER_SOURCES ${H743_LOCAL_DRIVER_SOURCES} ${H743_DRIVER_SOURCES})
    if(NOT H743_SELECTED_DRIVER_SOURCES)
        message(FATAL_ERROR "真实 H743 驱动模式需要在 boards/stm32h743/drivers/real/ 放入驱动源文件，或设置 H743_DRIVER_SOURCES")
    endif()
else()
    set(H743_SELECTED_DRIVER_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/drivers/stub/stub_board_drivers.cpp"
    )
endif()

if(H743_USE_FREERTOS AND NOT H743_FREERTOS_KERNEL_DIR AND NOT H743_FREERTOS_SOURCES)
    if(H743_FETCH_FREERTOS)
        include(FetchContent)
        FetchContent_Declare(
            freertos_kernel
            GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
            GIT_TAG V11.1.0
        )
        FetchContent_MakeAvailable(freertos_kernel)
        set(H743_FREERTOS_KERNEL_DIR "${freertos_kernel_SOURCE_DIR}" CACHE PATH "FreeRTOS-Kernel 根目录；设置后自动加入内核源文件" FORCE)
    else()
        message(FATAL_ERROR "H743_USE_FREERTOS=ON 时必须设置 H743_FREERTOS_KERNEL_DIR/H743_FREERTOS_SOURCES，或打开 H743_FETCH_FREERTOS")
    endif()
endif()

set(H743_SELECTED_FREERTOS_SOURCES ${H743_FREERTOS_SOURCES})
set(H743_SELECTED_FREERTOS_INCLUDE_DIRS ${H743_FREERTOS_INCLUDE_DIRS})

if(H743_USE_FREERTOS AND H743_FREERTOS_KERNEL_DIR)
    if(EXISTS "${H743_FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7/r0p1/port.c")
        set(H743_FREERTOS_PORT_DIR "${H743_FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7/r0p1")
    elseif(EXISTS "${H743_FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7/port.c")
        set(H743_FREERTOS_PORT_DIR "${H743_FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM7")
    else()
        message(FATAL_ERROR "未在 H743_FREERTOS_KERNEL_DIR 中找到 Cortex-M7 GCC portable port.c")
    endif()

    list(APPEND H743_SELECTED_FREERTOS_SOURCES
        "${H743_FREERTOS_KERNEL_DIR}/croutine.c"
        "${H743_FREERTOS_KERNEL_DIR}/event_groups.c"
        "${H743_FREERTOS_KERNEL_DIR}/list.c"
        "${H743_FREERTOS_KERNEL_DIR}/queue.c"
        "${H743_FREERTOS_KERNEL_DIR}/stream_buffer.c"
        "${H743_FREERTOS_KERNEL_DIR}/tasks.c"
        "${H743_FREERTOS_KERNEL_DIR}/timers.c"
        "${H743_FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c"
        "${H743_FREERTOS_PORT_DIR}/port.c"
    )
    list(APPEND H743_SELECTED_FREERTOS_INCLUDE_DIRS
        "${H743_FREERTOS_KERNEL_DIR}/include"
        "${H743_FREERTOS_PORT_DIR}"
    )
endif()

if(H743_USE_FREERTOS)
    list(APPEND H743_SELECTED_FREERTOS_INCLUDE_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/include"
    )
endif()

if(H743_USE_FREERTOS AND NOT H743_SELECTED_FREERTOS_SOURCES)
    message(FATAL_ERROR "H743_USE_FREERTOS=ON 时必须设置 H743_FREERTOS_KERNEL_DIR 或 H743_FREERTOS_SOURCES")
endif()

set(FLIGHT_CONTROL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_BUILD_FIRMWARE_ENTRY ON CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_USE_FREERTOS ${H743_USE_FREERTOS} CACHE BOOL "" FORCE)
set(FLIGHT_CONTROL_FREERTOS_INCLUDE_DIRS "${H743_SELECTED_FREERTOS_INCLUDE_DIRS}" CACHE STRING "" FORCE)

add_subdirectory("${H743_FLIGHT_CONTROL_DIR}" flight_control_core)

target_sources(flight_control_firmware
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/startup_stm32h743xx.s
        ${CMAKE_CURRENT_LIST_DIR}/src/board_hooks.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/syscalls.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/system_stm32h743.cpp
        ${H743_SELECTED_DRIVER_SOURCES}
        ${H743_SELECTED_FREERTOS_SOURCES}
)

target_include_directories(flight_control_firmware
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${H743_DRIVER_INCLUDE_DIRS}
        ${H743_SELECTED_FREERTOS_INCLUDE_DIRS}
)

if(H743_DRIVER_MODE STREQUAL "real")
    target_compile_definitions(flight_control_firmware PRIVATE H743_REAL_DRIVERS=1)
else()
    target_compile_definitions(flight_control_firmware PRIVATE H743_STUB_DRIVERS=1)
endif()

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
