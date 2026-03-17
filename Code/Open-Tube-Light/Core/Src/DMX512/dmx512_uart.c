/**
 * @file dmx512_uart.c
 * @brief DMX512 UART Input Driver - Implementation
 *
 * Receives DMX512 via UART5 using DMA for byte transfer offload.
 * Break detection via Framing Error, end-of-packet via IDLE line.
 *
 * Buffer strategy:
 *   dma_rx_buf  → D2 SRAM (.uart_buffers, non-cacheable) — DMA writes here
 *   shadow_buf  → DTCM (.DMX_Buffers) — ISR copies valid payload here
 *   active_buf  → DTCM (.DMX_Buffers) — EffectTask reads from here after latch
 *
 * Single-universe driver: DMX512 on one wire carries one universe.
 */

#include "DMX512/dmx512_uart.h"
#include "dmx_input.h"
#include "device_config.h"
#include "cmsis_os2.h"
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
 * RECEIVING:       DMA capturing slot data after valid START code byte.
 */
typedef enum {
    DMX512_STATE_IDLE,
    DMX512_STATE_BREAK_DETECTED,
    DMX512_STATE_RECEIVING,
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
 * @brief Shadow buffer — written by ISR, protected by mutex
 *
 * Contains channel data only (slots 1-512, no START code).
 * Placed in DTCM for zero-wait-state CPU access.
 */
static uint8_t shadow_buf[DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/**
 * @brief Active buffer — read by EffectTask during rendering
 *
 * Populated by Latch() under mutex for a consistent snapshot.
 */
static uint8_t active_buf[DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/* ========================== Private State ========================== */

static osMutexId_t shadow_mutex;
static osThreadId_t notify_task;
static UART_HandleTypeDef *dmx_huart;
static volatile DMX512_State_t rx_state = DMX512_STATE_IDLE;
static volatile bool initialized = false;

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

    /* HAL_UART_Receive_DMA re-enables EIE but not IDLEIE.
     * Ensure both are set after every restart. */
    SET_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);
    SET_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
}

/* ========================== Vtable Functions ========================== */

/**
 * @brief Initialize DMX512 UART reception
 *
 * Sets up mutex, zeros buffers, enables FE + IDLE interrupts, starts DMA.
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

    memset(shadow_buf, 0, sizeof(shadow_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    const osMutexAttr_t mutex_attr = {
        .name      = "dmx512Mtx",
        .attr_bits = osMutexPrioInherit,
    };
    shadow_mutex = osMutexNew(&mutex_attr);
    if (shadow_mutex == NULL) {
        return -1;
    }

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
 * Stops DMA, disables ISR flags, deletes mutex, zeros buffers.
 * Safe to call when not initialized.
 */
static void DMX512_Uart_Deinit(void)
{
    if (!initialized) return;

    /* Disable custom interrupt sources before stopping DMA */
    CLEAR_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);

    HAL_UART_AbortReceive(dmx_huart);

    if (shadow_mutex != NULL) {
        osMutexDelete(shadow_mutex);
        shadow_mutex = NULL;
    }

    /* Zero buffers so lights go dark on protocol switch (safety) */
    memset(shadow_buf, 0, sizeof(shadow_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    notify_task = NULL;
    dmx_huart   = NULL;
    rx_state    = DMX512_STATE_IDLE;
    initialized = false;
}

/**
 * @brief Latch shadow buffer into active buffer
 *
 * Mutex-protected copy so EffectTask sees a consistent snapshot.
 */
static void DMX512_Uart_Latch(void)
{
    osMutexAcquire(shadow_mutex, osWaitForever);
    memcpy(active_buf, shadow_buf, DMX_UNIVERSE_MAX_LENGTH);
    osMutexRelease(shadow_mutex);
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
 * Must NOT block — uses osThreadFlagsSet (ISR-safe) for notification.
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
            /* Valid Break — restart DMA at beginning of buffer */
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

            /* Need at least 2 bytes: START code + 1 data slot */
            if (received >= 2 && dma_rx_buf[0] == 0x00) {
                /* NULL START Code — copy channel data (skip slot 0).
                 * Clamp to 512 channels max. */
                uint16_t data_len = received - 1;
                if (data_len > DMX_UNIVERSE_MAX_LENGTH) {
                    data_len = DMX_UNIVERSE_MAX_LENGTH;
                }

                osMutexAcquire(shadow_mutex, 0);
                memcpy(shadow_buf, &dma_rx_buf[1], data_len);

                /* Zero remaining channels beyond what this packet carried.
                 * Spec: controllers may send fewer than 512 slots. */
                if (data_len < DMX_UNIVERSE_MAX_LENGTH) {
                    memset(&shadow_buf[data_len], 0,
                           DMX_UNIVERSE_MAX_LENGTH - data_len);
                }
                osMutexRelease(shadow_mutex);

                /* Wake EffectTask */
                if (notify_task != NULL) {
                    osThreadFlagsSet(notify_task, DMX512_THREAD_FLAG_FRAME_READY);
                }
            }
            /* Non-NULL START codes are silently ignored per DMX512-A §8.5.4 */

            rx_state = DMX512_STATE_IDLE;
        }
    }
}

/* ========================== Driver Vtable ========================== */

const DMX_Input_Driver_t dmx_input_dmx512 = {
    .init                = DMX512_Uart_Init,
    .deinit              = DMX512_Uart_Deinit,
    .latch               = DMX512_Uart_Latch,
    .get_universe        = DMX512_Uart_GetUniverse,
    .failsafe_timeout_ms = DMX512_FAILSAFE_TIMEOUT_MS,
};
