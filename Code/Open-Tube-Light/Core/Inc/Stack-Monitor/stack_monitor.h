/**
 * @file stack_monitor.h
 * @brief FreeRTOS stack and heap usage diagnostics
 *
 * Publishes per-task stack high-water marks and FreeRTOS heap usage in global
 * structures that can be inspected with STM32Cube Live Watch, and records the
 * identity of a task that overflows its stack before halting.
 *
 * Live Watch symbols:
 * - `stack_monitor_report`   – refreshed by StackMonitor_Update()
 * - `stack_monitor_overflow` – filled by StackMonitor_OnOverflow()
 */

#ifndef INC_STACK_MONITOR_H_
#define INC_STACK_MONITOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Task name buffer length; must equal configMAX_TASK_NAME_LEN */
#define STACK_MONITOR_NAME_LEN   16U

/**
 * @brief Number of task slots tracked.
 *
 * Current tasks: ControlTask, EffectTask, IDLE, Tmr Svc, tcpip_thread, EthIf,
 * EthLink (7). uxTaskGetSystemState() fails outright when the array is smaller
 * than the task count, so keep headroom here.
 */
#define STACK_MONITOR_MAX_TASKS  12U

/** @brief Stack headroom of one task */
typedef struct {
    char     name[STACK_MONITOR_NAME_LEN]; ///< Task name, empty = unused slot
    void    *handle;                       ///< FreeRTOS task handle
    uint32_t free_min_bytes;               ///< Lowest free stack ever seen (bytes)
} StackMonitor_TaskEntry_t;

/** @brief Snapshot published for Live Watch */
typedef struct {
    StackMonitor_TaskEntry_t tasks[STACK_MONITOR_MAX_TASKS]; ///< Stable order: first seen first
    uint32_t task_count;          ///< Tasks currently known to the kernel
    uint32_t snapshot_failures;   ///< Updates where the task table did not fit
    uint32_t heap_free_bytes;     ///< FreeRTOS heap free now
    uint32_t heap_free_min_bytes; ///< FreeRTOS heap low-water mark since boot
    uint32_t update_count;        ///< Number of completed updates
} StackMonitor_Report_t;

/** @brief Record of the task that overflowed its stack */
typedef struct {
    char     task_name[STACK_MONITOR_NAME_LEN]; ///< Name copied from the TCB
    void    *task_handle;                       ///< Handle passed by the kernel
    uint32_t count;                             ///< Non-zero once an overflow was detected
} StackMonitor_Overflow_t;

/** @brief Live Watch: stack/heap usage, refreshed by StackMonitor_Update() */
extern StackMonitor_Report_t stack_monitor_report;

/** @brief Live Watch: overflow record, written by StackMonitor_OnOverflow() */
extern StackMonitor_Overflow_t stack_monitor_overflow;

/**
 * @brief Refresh stack high-water marks and heap statistics
 *
 * Walks every task's stack, briefly suspending the scheduler (tens of µs).
 * Call from a low-priority task at a slow rate (ControlTask, 1 Hz).
 * Must not be called from an ISR.
 */
void StackMonitor_Update(void);

/**
 * @brief Record a stack overflow and halt
 *
 * Called from vApplicationStackOverflowHook(). Disables interrupts, stores the
 * task identity in `stack_monitor_overflow`, triggers a breakpoint when a
 * debugger is attached, and then loops forever.
 *
 * @param task_handle Handle of the overflowing task
 * @param task_name   Name of the overflowing task (may be NULL)
 */
void StackMonitor_OnOverflow(void *task_handle, const char *task_name) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* INC_STACK_MONITOR_H_ */
