/**
 * @file dmx512_uart.c
 * @brief DMX512 UART Input Driver - Implementation
 *
 * Receives DMX512 via UART5 using DMA for byte transfer offload.
 * Break detection via Framing Error, end-of-packet via IDLE line.
 *
 * Buffer strategy:
 *   dma_rx_buf    → D2 SRAM (.uart_buffers, non-cacheable) — DMA writes here
 *   pending_buf[] → DTCM (.DMX_Buffers) — ISR writes to one, Latch reads the other
 *   active_buf    → DTCM (.DMX_Buffers) — EffectTask reads from here after latch
 *
 * ISR/thread synchronization uses a lock-free double-buffer swap.
 *
 * Priority architecture:
 *   UART5 ISR runs at priority 3 (above FreeRTOS BASEPRI threshold of 5)
 *   so it is never masked by critical sections. Because FreeRTOS APIs
 *   cannot be called above the threshold, task notification is deferred
 *   via software-pending DMA1_Stream1_IRQn (priority 5, FreeRTOS-safe).
 *   DMA TC/TE/DME interrupts are disabled after each HAL_UART_Receive_DMA
 *   so HAL_DMA_IRQHandler is a no-op when DMA1_Stream1 fires from our
 *   software pend — it checks __HAL_DMA_GET_IT_SOURCE which returns 0.
 */

#include "DMX512/dmx512_uart.h"
#include "dmx_input.h"
#include "device_config.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* ========================== Private Constants ========================== */

/** @brief Thread notification flag matching EffectTask convention */
#define DMX512_THREAD_FLAG_FRAME_READY  0x01U

/* ========================== Private Types ========================== */

/**
 * @brief ISR state machine states
 *
 * IDLE:            Waiting for Break (Framing Error).
 * BREAK_DETECTED:  Break seen, DMA restarted, waiting for data + IDLE.
 */
typedef enum {
    DMX512_STATE_IDLE,
    DMX512_STATE_BREAK_DETECTED,
} DMX512_State_t;

/* ========================== Private Buffers ========================== */

/**
 * @brief DMA receive buffer in D2 SRAM (non-cacheable via MPU Region 1)
 *
 * 513 bytes: slot 0 (START code) + slots 1-512 (channel data).
 * Must be in D2 SRAM because the DMA controller accesses it via AHB
 * and the D-cache would cause coherency issues otherwise.
 */
static uint8_t dma_rx_buf[DMX512_PACKET_MAX_SIZE]
    __attribute__((section(".uart_buffers"), aligned(32)));

/**
 * @brief Double pending buffers — ISR writes one while Latch reads the other
 *
 * Contains channel data only (slots 1-512, no START code).
 * Placed in DTCM for zero-wait-state CPU access.
 * Lock-free: ISR writes to pending_buf[write_idx], then flips write_idx.
 * Latch reads from pending_buf[write_idx ^ 1] (the last completed buffer).
 */
static uint8_t pending_buf[2][DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/**
 * @brief Active buffer — read by EffectTask during rendering
 *
 * Populated by Latch() from the last-completed pending buffer.
 */
static uint8_t active_buf[DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/* ========================== Private State ========================== */

static osThreadId_t notify_task;
static UART_HandleTypeDef *dmx_huart;
static volatile DMX512_State_t rx_state = DMX512_STATE_IDLE;
static volatile uint8_t write_idx = 0;  /* ISR toggles after each complete write */
static volatile bool initialized = false;

/** @brief Set by UART5 ISR (above BASEPRI), consumed by deferred DMA1_Stream1 ISR */
static volatile bool deferred_notify_flag = false;

// DEBUGGING GLOBAL
static volatile uint32_t recievedData = 0;

/* ========================== Private Helpers ========================== */

/**
 * @brief Start (or restart) DMA reception into dma_rx_buf
 *
 * Aborts any in-flight DMA transfer, then re-arms for a new packet.
 * Re-enables EIE and IDLEIE after restart because HAL_UART_AbortReceive
 * clears EIE, and error paths in HAL can also clear it.
 */
static void DMX512_Uart_StartDmaRx(void)
{
    HAL_UART_AbortReceive(dmx_huart);
    HAL_UART_Receive_DMA(dmx_huart, dma_rx_buf, DMX512_PACKET_MAX_SIZE);

    /* Disable DMA TC/TE/DME interrupts. Without this, a full 513-byte
     * packet triggers HAL's UART_DMAReceiveCplt which clears EIE/IDLEIE
     * and sets RxState=READY — permanently killing break detection.
     * HAL_DMA_IRQHandler checks __HAL_DMA_GET_IT_SOURCE before acting,
     * so with these bits cleared the handler is a safe no-op. */
    CLEAR_BIT(((DMA_Stream_TypeDef *)dmx_huart->hdmarx->Instance)->CR,
              DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT);

    /* HAL_UART_Receive_DMA re-enables EIE but not IDLEIE.
     * Ensure both are set after every restart. */
    SET_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);
    SET_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
}

/**
 * @brief Process a completed DMX packet from dma_rx_buf
 *
 * Validates NULL START code, copies channel data into the pending
 * double-buffer, and notifies EffectTask. Called from both the IDLE
 * handler (normal path) and the FE handler (MBB = 0 path).
 *
 * @param received  Number of bytes DMA transferred (including START code)
 */
static void DMX512_Uart_ProcessPacket(uint16_t received)
{
    recievedData = received; // DEBUGGING
    
    /* Need at least 2 bytes: START code + 1 data slot */
    if (received >= 2 && dma_rx_buf[0] == 0x00) {
        /* NULL START Code — copy channel data (skip slot 0).
         * Clamp to 512 channels max. */
        uint16_t data_len = received - 1;
        if (data_len > DMX_UNIVERSE_MAX_LENGTH) {
            data_len = DMX_UNIVERSE_MAX_LENGTH;
        }

        /* Write to the current pending buffer (lock-free) */
        uint8_t idx = write_idx;
        memcpy(pending_buf[idx], &dma_rx_buf[1], data_len);

        /* Zero remaining channels beyond what this packet carried.
         * Spec: controllers may send fewer than 512 slots. */
        if (data_len < DMX_UNIVERSE_MAX_LENGTH) {
            memset(&pending_buf[idx][data_len], 0,
                   DMX_UNIVERSE_MAX_LENGTH - data_len);
        }

        /* Publish: flip write_idx so Latch reads this buffer */
        write_idx ^= 1;

        /* Defer task notification: UART5 ISR runs above FreeRTOS BASEPRI
         * so we cannot call osThreadFlagsSet here. Instead, set a flag
         * and software-pend DMA1_Stream1_IRQn (priority 5, below BASEPRI)
         * which will safely call osThreadFlagsSet on our behalf. */
        deferred_notify_flag = true;
        NVIC_SetPendingIRQ(DMA1_Stream1_IRQn);
    }
    /* Non-NULL START codes are silently ignored per DMX512-A §8.5.4 */
}

/* ========================== Vtable Functions ========================== */

/**
 * @brief Initialize DMX512 UART reception
 *
 * Zeros buffers, enables FE + IDLE interrupts, starts DMA.
 * UART5 peripheral must already be initialized by CubeMX (MX_UART5_Init).
 */
static int DMX512_Uart_Init(osThreadId_t task)
{
    if (initialized) return 0;

    /* Reference the UART5 handle defined in main.c */
    extern UART_HandleTypeDef huart5;
    dmx_huart   = &huart5;
    notify_task = task;
    rx_state    = DMX512_STATE_IDLE;

    memset(pending_buf, 0, sizeof(pending_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
    write_idx = 0;

    deferred_notify_flag = false;

    /* Start DMA reception — arms the DMA stream for the first packet */
    DMX512_Uart_StartDmaRx();

    /* Enable Error Interrupt Enable (EIE) for Framing Error during DMA.
     * HAL does not enable this automatically; without it, FE during
     * DMA reception doesn't trigger the UART IRQ. */
    SET_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);

    /* Enable IDLE line interrupt for end-of-packet detection */
    SET_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);

    initialized = true;
    return 0;
}

/**
 * @brief Tear down DMX512 UART reception
 *
 * Stops DMA, disables ISR flags, zeros buffers.
 * Safe to call when not initialized.
 */
static void DMX512_Uart_Deinit(void)
{
    if (!initialized) return;

    /* Disable custom interrupt sources before stopping DMA */
    CLEAR_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);

    HAL_UART_AbortReceive(dmx_huart);

    /* Zero buffers so lights go dark on protocol switch (safety) */
    memset(pending_buf, 0, sizeof(pending_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    deferred_notify_flag = false;
    NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);

    notify_task = NULL;
    dmx_huart   = NULL;
    rx_state    = DMX512_STATE_IDLE;
    initialized = false;
}

/**
 * @brief Latch most recent pending buffer into active buffer
 *
 * Reads from the last buffer the ISR completed (write_idx ^ 1).
 * Lock-free: ISR only ever writes to pending_buf[write_idx], which
 * is the OTHER buffer, so no torn read is possible.
 */
static void DMX512_Uart_Latch(void)
{
    uint8_t read_idx = write_idx ^ 1;
    memcpy(active_buf, pending_buf[read_idx], DMX_UNIVERSE_MAX_LENGTH);
}

/**
 * @brief Get pointer to active DMX data
 *
 * DMX512 over UART is single-universe; only index 0 is valid.
 */
static const uint8_t* DMX512_Uart_GetUniverse(uint8_t universe)
{
    if (universe != 0) return NULL;
    return active_buf;
}

/* ========================== ISR Handler ========================== */

/**
 * @brief Custom UART ISR for DMX512 Break/IDLE detection
 *
 * Called from UART5_IRQHandler before HAL_UART_IRQHandler.
 * Reads ISR register directly for lowest latency. Handles:
 *   - ORE/NE/PE: Cleared immediately to prevent HAL_UART_IRQHandler from
 *     seeing them. HAL's error path calls UART_EndRxTransfer() which disables
 *     EIE (kills break detection) and aborts DMA — unrecoverable without this.
 *   - Framing Error (FE): Break detection / mid-packet corruption
 *   - IDLE flag: end-of-packet, calculates byte count from DMA counter
 *
 * Must NOT block — defers task notification via software-pended
 * DMA1_Stream1_IRQn (see DMX512_Uart_DeferredNotify).
 */
void DMX512_Uart_IRQHandler(UART_HandleTypeDef *huart)
{
    if (!initialized || huart->Instance != UART5) return;

    uint32_t isr = READ_REG(huart->Instance->ISR);

    /* ---- Clear ORE / NE / PE before HAL sees them ----
     * Cable disconnect/reconnect causes noise that triggers ORE (overrun)
     * or NE (noise). If HAL_UART_IRQHandler handles these, it calls
     * UART_EndRxTransfer() which clears CR3.EIE and aborts DMA,
     * permanently disabling break detection. */
    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_PE)) {
        SET_BIT(huart->Instance->ICR,
                USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF);

        if (isr & USART_ISR_ORE) {
            /* Overrun: a byte was lost, current packet data is shifted.
             * Discard this packet and wait for next Break to resync. */
            DMX512_Uart_StartDmaRx();
            rx_state = DMX512_STATE_IDLE;
            return;
        }
        /* NE/PE alone: byte may be noisy but UART continues operating.
         * Fall through to FE/IDLE handling — let Break resync naturally. */
    }

    /* ---- Framing Error: Break detection ---- */
    if (isr & USART_ISR_FE) {
        /* Clear FE flag by writing 1 to FECF in ICR */
        SET_BIT(huart->Instance->ICR, USART_ICR_FECF);

        /* Also read RDR to clear RXNE so the bogus break byte
         * doesn't sit in the data register */
        (void)READ_REG(huart->Instance->RDR);

        if (rx_state == DMX512_STATE_IDLE || rx_state == DMX512_STATE_BREAK_DETECTED) {
            /* If we already had a Break and DMA captured data, process the
             * previous packet before restarting. This handles MBB = 0 where
             * the transmitter goes directly from data into the next Break
             * with no IDLE gap (DMX512-A §8.9, Table 6 Designation #10). */
            if (rx_state == DMX512_STATE_BREAK_DETECTED) {
                uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(dmx_huart->hdmarx);
                uint16_t received  = DMX512_PACKET_MAX_SIZE - remaining;
                if (received > 0) {
                    DMX512_Uart_ProcessPacket(received);
                }
            }

            DMX512_Uart_StartDmaRx();
            rx_state = DMX512_STATE_BREAK_DETECTED;
        } else {
            /* Mid-packet FE — corrupted data, discard and wait for next Break */
            HAL_UART_AbortReceive(dmx_huart);
            rx_state = DMX512_STATE_IDLE;
        }
        return;
    }

    /* ---- IDLE line: end-of-packet ---- */
    if (isr & USART_ISR_IDLE) {
        /* Clear IDLE flag by writing 1 to IDLECF in ICR */
        SET_BIT(huart->Instance->ICR, USART_ICR_IDLECF);

        if (rx_state == DMX512_STATE_BREAK_DETECTED) {
            /* Calculate how many bytes DMA actually transferred.
             * DMA counter counts DOWN from DMX512_PACKET_MAX_SIZE. */
            uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
            uint16_t received  = DMX512_PACKET_MAX_SIZE - remaining;

            /* Mark After Break (MAB) is detected as IDLE because the line
             * is HIGH for ~1.24ms, far exceeding the one-character-time
             * threshold (~44µs). If no bytes arrived yet, this is the MAB —
             * stay in BREAK_DETECTED so we catch the real end-of-packet. */
            if (received == 0) {
                return;
            }

            DMX512_Uart_ProcessPacket(received);
            rx_state = DMX512_STATE_IDLE;
        }
    }
}

/* ========================== Deferred Notification ========================== */

/**
 * @brief Deferred ISR notification handler
 *
 * Called from DMA1_Stream1_IRQHandler (priority 5, below FreeRTOS BASEPRI).
 * Consumes the flag set by the UART5 ISR and calls osThreadFlagsSet,
 * which is only safe at priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 *
 * @return true if notification was handled (caller should skip HAL_DMA_IRQHandler)
 */
bool DMX512_Uart_DeferredNotify(void)
{
    if (!deferred_notify_flag) return false;
    deferred_notify_flag = false;

    if (notify_task != NULL) {
        osThreadFlagsSet(notify_task, DMX512_THREAD_FLAG_FRAME_READY);
    }
    return true;
}

/* ========================== Driver Vtable ========================== */

const DMX_Input_Driver_t dmx_input_dmx512 = {
    .init                = DMX512_Uart_Init,
    .deinit              = DMX512_Uart_Deinit,
    .latch               = DMX512_Uart_Latch,
    .get_universe        = DMX512_Uart_GetUniverse,
    .failsafe_timeout_ms = DMX512_FAILSAFE_TIMEOUT_MS,
};
