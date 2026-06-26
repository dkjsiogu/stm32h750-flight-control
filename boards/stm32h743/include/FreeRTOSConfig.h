#pragma once

#include <stdint.h>

/* 真实 BSP 在时钟初始化后应更新 SystemCoreClock。 */
extern uint32_t SystemCoreClock;

/* 使用抢占式调度，姿态闭环任务需要固定周期运行。 */
#define configUSE_PREEMPTION 1
/* 不使用低功耗 tickless idle，避免飞控周期被省电策略影响。 */
#define configUSE_TICKLESS_IDLE 0
/* CPU 时钟来自 SystemCoreClock，默认按 480MHz H743 配置。 */
#define configCPU_CLOCK_HZ (SystemCoreClock)
/* RTOS tick 频率，单位 Hz。 */
#define configTICK_RATE_HZ 1000
/* 任务优先级数量。 */
#define configMAX_PRIORITIES 7
/* 空闲任务栈大小，单位 word。 */
#define configMINIMAL_STACK_SIZE 256
/* heap_4 内存池大小，单位 byte。 */
#define configTOTAL_HEAP_SIZE (64U * 1024U)
/* 任务名最大长度。 */
#define configMAX_TASK_NAME_LEN 16
/* 使用 32 位 tick。 */
#define configUSE_16_BIT_TICKS 0
/* 允许 FreeRTOS 维护空闲任务统计。 */
#define configIDLE_SHOULD_YIELD 1
/* 启用静态分配，飞控任务使用 xTaskCreateStatic。 */
#define configSUPPORT_STATIC_ALLOCATION 1
/* 保留动态分配，FreeRTOS 内部对象和第三方驱动可按需使用。 */
#define configSUPPORT_DYNAMIC_ALLOCATION 1
/* 启用互斥锁。 */
#define configUSE_MUTEXES 1
/* 启用递归互斥锁。 */
#define configUSE_RECURSIVE_MUTEXES 1
/* 启用计数信号量。 */
#define configUSE_COUNTING_SEMAPHORES 1
/* 启用软件定时器。 */
#define configUSE_TIMERS 1
/* 软件定时器任务优先级。 */
#define configTIMER_TASK_PRIORITY 2
/* 软件定时器队列长度。 */
#define configTIMER_QUEUE_LENGTH 8
/* 软件定时器任务栈大小，单位 word。 */
#define configTIMER_TASK_STACK_DEPTH 512
/* Cortex-M7 NVIC 优先级位数。 */
#define configPRIO_BITS 4
/* 最低中断优先级。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
/* 允许调用 FreeRTOS API 的最高中断优先级。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
/* 内核中断优先级寄存器值。 */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* 最大 syscall 中断优先级寄存器值。 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* 关闭运行时间统计，真实工程可在驱动层补充高精度计时后打开。 */
#define configGENERATE_RUN_TIME_STATS 0
/* 关闭 trace facility，减少默认固件体积。 */
#define configUSE_TRACE_FACILITY 0
/* 启用任务通知。 */
#define configUSE_TASK_NOTIFICATIONS 1
/* 启用 vTaskDelayUntil，控制周期调度需要该 API。 */
#define INCLUDE_vTaskDelayUntil 1
/* 启用 vTaskDelay。 */
#define INCLUDE_vTaskDelay 1
/* 启用任务删除。 */
#define INCLUDE_vTaskDelete 1
/* 启用任务优先级设置。 */
#define INCLUDE_vTaskPrioritySet 1
/* 启用获取当前任务句柄。 */
#define INCLUDE_xTaskGetCurrentTaskHandle 1
/* 启用调度器状态查询。 */
#define INCLUDE_xTaskGetSchedulerState 1
/* 启用空闲任务让出。 */
#define INCLUDE_taskYIELD 1
/* FreeRTOS port 生成真实 Cortex-M7 SVC 入口。 */
#define vPortSVCHandler SVC_Handler
/* FreeRTOS port 生成真实 Cortex-M7 PendSV 入口。 */
#define xPortPendSVHandler PendSV_Handler
/* FreeRTOS port 生成真实 Cortex-M7 SysTick 入口。 */
#define xPortSysTickHandler SysTick_Handler
/* 断言失败时进入断点并停机，避免带故障状态继续输出电机。 */
#define configASSERT(x)              \
    do {                             \
        if ((x) == 0) {              \
            __asm volatile("bkpt #0"); \
            for (;;) {               \
            }                        \
        }                            \
    } while (0)
