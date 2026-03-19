/**
 * @file dmx512_uart.h
 * @brief DMX512 UART Input Driver
 *
 * Receives DMX512 data via UART5 + DMA. Break detection uses the UART
 * Framing Error flag; end-of-packet uses IDLE line detection. The DMA
 * RX buffer lives in D2 SRAM (non-cacheable) while shadow/active DMX
 * buffers live in DTCM for fast CPU access.
 *
 * Single-universe only (one UART wire = one DMX universe).
 *
 * @note Include from dmx512_uart.c and stm32h7xx_it.c only.
 *       Application code should use dmx_input.h.
 */

#ifndef INC_DMX512_UART_H_
#define INC_DMX512_UART_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Constants ========================== */

/** @brief Failsafe timeout per ANSI E1.11 (1.25 seconds without a packet) */
#define DMX512_FAILSAFE_TIMEOUT_MS  1250U

/* ========================== ISR Interface ========================== */

/**
 * @brief Custom UART ISR handler for DMX512 Break/IDLE detection
 *
 * Must be called from UART5_IRQHandler() BEFORE HAL_UART_IRQHandler().
 * Checks Framing Error and IDLE flags, manages DMA restarts, and
 * notifies EffectTask when a valid packet arrives.
 *
 * @param huart UART handle (must be the UART5 instance)
 */
void DMX512_Uart_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief Deferred notification handler for DMA1_Stream1_IRQHandler
 *
 * Since UART5 ISR runs above FreeRTOS BASEPRI (priority 3), it cannot
 * call FreeRTOS APIs. Instead it software-pends DMA1_Stream1_IRQn.
 * Call this from DMA1_Stream1_IRQHandler before HAL_DMA_IRQHandler.
 *
 * @return true if notification was handled (caller should return early)
 */
bool DMX512_Uart_DeferredNotify(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DMX512_UART_H_ */
