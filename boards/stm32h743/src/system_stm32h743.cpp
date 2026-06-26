#include <cstdint>

#include "board_io.hpp"

extern "C" {

/** 当前系统核心时钟，真实 BSP 可在 `board_system_preinit` 中更新。 */
std::uint32_t SystemCoreClock = 480000000UL;

/**
 * STM32H743 系统初始化入口。
 *
 * 真实板级工程应在这里配置 HSE/PLL、Flash latency、Cache、MPU 和中断优先级分组。
 * 当前默认实现只提供启动符号，避免在未接入 HAL/LL 前误配置硬件。
 */
void SystemInit() {
    board_system_preinit();
}

/**
 * 默认异常处理函数。
 *
 * 未实现的中断会停在这里，便于调试定位缺失的 ISR。
 */
void Default_Handler() {
    while (true) {
    }
}

/**
 * SVC 默认处理函数。
 *
 * 未接入 FreeRTOS port 时停在默认异常处理；接入后由 FreeRTOSConfig 映射为内核入口。
 */
void __attribute__((weak)) SVC_Handler() {
    Default_Handler();
}

/**
 * PendSV 默认处理函数。
 *
 * 未接入 FreeRTOS port 时停在默认异常处理；接入后由 FreeRTOSConfig 映射为内核入口。
 */
void __attribute__((weak)) PendSV_Handler() {
    Default_Handler();
}

/**
 * SysTick 默认处理函数。
 *
 * 未接入 FreeRTOS port 时停在默认异常处理；接入后由 FreeRTOSConfig 映射为内核入口。
 */
void __attribute__((weak)) SysTick_Handler() {
    Default_Handler();
}

/**
 * C++ 纯虚函数兜底处理。
 *
 * 嵌入式构建关闭异常和 RTTI 时仍需要该符号。
 */
void __cxa_pure_virtual() {
    while (true) {
    }
}

/**
 * 退出兜底处理。
 *
 * 固件不应从 main 返回；若返回则停在这里。
 *
 * @param status 退出状态码。
 */
void _exit(int status) {
    (void)status;
    while (true) {
    }
}

}  // extern "C"
