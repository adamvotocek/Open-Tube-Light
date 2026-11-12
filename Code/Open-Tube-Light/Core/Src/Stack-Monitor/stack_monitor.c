/**
 * @file stack_monitor.c
 * @brief FreeRTOS stack and heap usage diagnostics
 *
 * Stack sizes are set in CubeMX (.ioc) and lwipopts.h. This module provides
 * the runtime evidence for sizing them: each task's lowest-ever free stack
 * (FreeRTOS high-water mark) and the FreeRTOS heap low-water mark.
 *
 * Requires configUSE_TRACE_FACILITY = 1 (uxTaskGetSystemState) and
 * configCHECK_FOR_STACK_OVERFLOW = 2 (overflow hook with canary check).
 */

#include "Stack-Monitor/stack_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"
#include <stddef.h>

_Static_assert(STACK_MONITOR_NAME_LEN == configMAX_TASK_NAME_LEN,
               "STACK_MONITOR_NAME_LEN must match configMAX_TASK_NAME_LEN");

#if (configUSE_TRACE_FACILITY != 1)
#error "stack_monitor requires configUSE_TRACE_FACILITY = 1"
#endif

/* ========================== Public Variables ========================== */

StackMonitor_Report_t   stack_monitor_report;
StackMonitor_Overflow_t stack_monitor_overflow;

/* ========================== Private Variables ========================== */

/** @brief Kernel snapshot buffer; static to keep ~0.5 KB off the ControlTask stack */
static TaskStatus_t task_status[STACK_MONITOR_MAX_TASKS];

/* ========================== Private Function Prototypes ========================== */

static StackMonitor_TaskEntry_t *StackMonitor_FindOrAddSlot(void *handle);
static void StackMonitor_CopyName(char *dst, const char *src);

/* ========================== Public Functions ========================== */

void StackMonitor_Update(void)
{
    StackMonitor_Report_t *report = &stack_monitor_report;

    // Returns 0 when the array is smaller than the number of tasks.
    UBaseType_t filled = uxTaskGetSystemState(task_status, STACK_MONITOR_MAX_TASKS, NULL);
    report->task_count = (uint32_t)uxTaskGetNumberOfTasks();

    if (filled == 0U) {
        report->snapshot_failures++;
    }

    for (UBaseType_t i = 0; i < filled; i++) {
        StackMonitor_TaskEntry_t *slot = StackMonitor_FindOrAddSlot(task_status[i].xHandle);
        if (slot == NULL) {
            continue;
        }
        StackMonitor_CopyName(slot->name, task_status[i].pcTaskName);
        // High-water mark is reported in StackType_t words.
        slot->free_min_bytes = (uint32_t)task_status[i].usStackHighWaterMark * sizeof(StackType_t);
    }

    report->heap_free_bytes     = (uint32_t)xPortGetFreeHeapSize();
    report->heap_free_min_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    report->update_count++;
}

void StackMonitor_OnOverflow(void *task_handle, const char *task_name)
{
    // The overflowing task may have corrupted neighbouring heap blocks; stop
    // everything before more damage is done.
    __disable_irq();

    stack_monitor_overflow.task_handle = task_handle;
    StackMonitor_CopyName(stack_monitor_overflow.task_name, task_name);
    stack_monitor_overflow.count++;

    // BKPT without a debugger escalates to HardFault, so only break when attached.
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
        __BKPT(0);
    }

    for (;;) {
        // Halted: inspect stack_monitor_overflow in Live Watch.
    }
}

/* ========================== Private Functions ========================== */

/**
 * @brief Find the slot for a task handle, or claim a free one
 *
 * Slots keep first-seen order so Live Watch rows do not jump around
 * (uxTaskGetSystemState() orders tasks by scheduler state).
 *
 * @return Slot pointer, or NULL if all slots are taken
 */
static StackMonitor_TaskEntry_t *StackMonitor_FindOrAddSlot(void *handle)
{
    StackMonitor_TaskEntry_t *free_slot = NULL;

    for (uint32_t i = 0; i < STACK_MONITOR_MAX_TASKS; i++) {
        StackMonitor_TaskEntry_t *slot = &stack_monitor_report.tasks[i];
        if (slot->handle == handle) {
            return slot;
        }
        if ((slot->handle == NULL) && (free_slot == NULL)) {
            free_slot = slot;
        }
    }

    if (free_slot != NULL) {
        free_slot->handle = handle;
    }
    return free_slot;
}

/**
 * @brief Bounded, always-terminated copy of a task name
 */
static void StackMonitor_CopyName(char *dst, const char *src)
{
    size_t i = 0;

    if (src != NULL) {
        for (; (i < (STACK_MONITOR_NAME_LEN - 1U)) && (src[i] != '\0'); i++) {
            dst[i] = src[i];
        }
    }
    dst[i] = '\0';
}
