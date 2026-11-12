#pragma once

#include <stdint.h>

extern uint32_t SystemCoreClock;

// ---------------------------------------------------------------------------
// Kernel
// ---------------------------------------------------------------------------
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE ((uint16_t)128)
#define configMAX_TASK_NAME_LEN 8
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1

// ---------------------------------------------------------------------------
// Memory: heap_4 in .bss. The clone board carries a C6 die: 10 KB RAM total.
// ---------------------------------------------------------------------------
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE ((size_t)(4 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP 0

// ---------------------------------------------------------------------------
// Optional features
// ---------------------------------------------------------------------------
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_QUEUE_SETS 0
#define configUSE_TIMERS 0
#define configUSE_EVENT_GROUPS 0
#define configUSE_STREAM_BUFFERS 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configGENERATE_RUN_TIME_STATS 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_NEWLIB_REENTRANT 0

// ---------------------------------------------------------------------------
// Interrupt priorities (Cortex-M3, 4 priority bits)
// ---------------------------------------------------------------------------
#define configPRIO_BITS 4
// Values must be shifted into the implemented bits, otherwise the port's
// configASSERT() in xPortStartScheduler() spins forever.
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (15 << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// ---------------------------------------------------------------------------
// Handlers are provided by the FreeRTOS port
// ---------------------------------------------------------------------------
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define configASSERT(x)      \
    if ((x) == 0) {          \
        taskDISABLE_INTERRUPTS(); \
        for (;;) {           \
        }                    \
    }

// ---------------------------------------------------------------------------
// API inclusion
// ---------------------------------------------------------------------------
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
