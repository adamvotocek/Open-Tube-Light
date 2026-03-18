/**
 * @file dmx512_uart.c
 * @brief DMX512 UART Input Driver - Implementation
 *
 * Receives DMX512 via UART5 using DMA for byte transfer offload.
 * Break detection via Framing Error, end-of-packet via IDLE line.
 *
 * Buffer strategy:
 *   dma_rx_buf         → D2 SRAM (.uart_buffers, non-cacheable) — DMA target
 *   pending_buf[]      → DTCM (.DMX_Buffers) — ISR double-buffer
 *   pending_data_len[] → valid channel count per pending buffer
 *   active_buf         → DTCM (.DMX_Buffers) — consumer reads after Latch
 *
 * Partial universe: ISR stores only received channels + length. Latch()
 * merges partial data into active_buf, preserving untouched channels.
 *
 * Priority architecture:
 *   UART5 ISR at priority 3 (above FreeRTOS BASEPRI=5, never masked).
 *   Task notification deferred via software-pended DMA1_Stream1_IRQn (prio 5).
 *   DMA TC/TE/DME interrupts disabled so HAL_DMA_IRQHandler is a no-op.
 *
 * Known limitation — IDLE sensitivity:
 *   IDLE fires after one character time (~44 µs). Transmitters inserting
 *   > 44 µs inter-slot gaps will cause mid-packet truncation. Rare in practice.
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

typedef enum {
    DMX512_STATE_IDLE,       /* Waiting for Break (Framing Error) */
    DMX512_STATE_RECEIVING,  /* Break seen, DMA armed, awaiting data + IDLE */
} DMX512_State_t;

/* ========================== Private Buffers ========================== */

/** DMA target — D2 SRAM (non-cacheable via MPU) for DMA coherency */
static uint8_t dma_rx_buf[DMX512_PACKET_MAX_SIZE]
    __attribute__((section(".uart_buffers"), aligned(32)));

/** ISR double-buffer — channel data only (no START code). DTCM for zero-wait access.
 *  Lock-free: ISR writes [write_idx], Latch reads [write_idx ^ 1]. */
static uint8_t pending_buf[2][DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/** Consumer buffer — populated by Latch(), read by EffectTask */
static uint8_t active_buf[DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/* ========================== Private State ========================== */

static osThreadId_t notify_task;
static UART_HandleTypeDef *dmx_huart;
static volatile DMX512_State_t rx_state = DMX512_STATE_IDLE;
static volatile uint8_t  write_idx = 0;
static volatile bool     initialized = false;
static volatile uint16_t pending_data_len[2] = {0, 0}; /* valid bytes per pending_buf */
static volatile bool     deferred_notify_flag = false;  /* UART5 ISR → DMA1_Stream1 ISR */

// DEBUGGING GLOBAL
static volatile uint16_t recievedData = 0;
static volatile uint16_t smallFrameCnt = 0;
static volatile uint16_t midPcktErrCnt = 0;

/* ========================== Private Helpers ========================== */

/** @brief Bytes DMA has transferred into dma_rx_buf so far */
static uint16_t DMX512_Uart_DmaReceived(void)
{
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(dmx_huart->hdmarx);
    return DMX512_PACKET_MAX_SIZE - remaining;
}

/**
 * @brief Abort in-flight DMA, re-arm for a new packet
 *
 * Re-enables EIE + IDLEIE (HAL_UART_AbortReceive clears EIE).
 * Disables DMA TC/TE/DME/HT interrupts to prevent HAL's DMA-complete
 * callback from tearing down break detection.
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

    SET_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);
    SET_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
}

/**
 * @brief Validate and publish a received DMX packet
 *
 * Checks NULL START code, copies channel data into the pending double-buffer,
 * and defers task notification via software-pended DMA1_Stream1_IRQn.
 *
 * @param received  Bytes DMA transferred (including START code)
 */
static void DMX512_Uart_ProcessPacket(uint16_t received)
{
    recievedData = received; // DEBUGGING
    if (received < 513) {
        smallFrameCnt++;
    }

    if (received < 2 || dma_rx_buf[0] != 0x00) return;
    /* Non-NULL START codes silently ignored per DMX512-A §8.5.4 */

    uint16_t data_len = received - 1;
    if (data_len > DMX_UNIVERSE_MAX_LENGTH) {
        data_len = DMX_UNIVERSE_MAX_LENGTH;
    }

    /* Only received channels written; Latch() preserves the tail */
    uint8_t idx = write_idx;
    memcpy(pending_buf[idx], &dma_rx_buf[1], data_len);
    pending_data_len[idx] = data_len;

    /* Publish: flip write_idx so Latch reads this buffer */
    write_idx ^= 1;

    /* Defer notification — can't call FreeRTOS APIs above BASEPRI */
    deferred_notify_flag = true;
    NVIC_SetPendingIRQ(DMA1_Stream1_IRQn);
}

/* ========================== Vtable Functions ========================== */

/** @brief Initialize UART reception. UART5 must already be configured by CubeMX. */
static int DMX512_Uart_Init(osThreadId_t task)
{
    if (initialized) return 0;

    extern UART_HandleTypeDef huart5;
    dmx_huart   = &huart5;
    notify_task = task;
    rx_state    = DMX512_STATE_IDLE;

    memset(pending_buf, 0, sizeof(pending_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
    pending_data_len[0] = 0;
    pending_data_len[1] = 0;
    write_idx = 0;
    deferred_notify_flag = false;

    DMX512_Uart_StartDmaRx();

    initialized = true;
    return 0;
}

/** @brief Tear down reception. Safe to call when not initialized. */
static void DMX512_Uart_Deinit(void)
{
    if (!initialized) return;

    initialized = false;

    /* Disable custom interrupt sources before stopping DMA */
    CLEAR_BIT(dmx_huart->Instance->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(dmx_huart->Instance->CR3, USART_CR3_EIE);
    HAL_UART_AbortReceive(dmx_huart);

    /* Zero buffers so lights go dark on protocol switch */
    memset(pending_buf, 0, sizeof(pending_buf));
    memset(active_buf, 0, sizeof(active_buf));
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    deferred_notify_flag = false;
    NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);

    notify_task = NULL;
    dmx_huart   = NULL;
    rx_state    = DMX512_STATE_IDLE;
}

/**
 * @brief Merge most recent pending buffer into active buffer
 *
 * Only overwrites channels the packet carried — untouched channels
 * retain their previous values. Lock-free: ISR writes [write_idx],
 * Latch reads [write_idx ^ 1].
 */
static void DMX512_Uart_Latch(void)
{
    uint8_t  read_idx = write_idx ^ 1;
    uint16_t len      = pending_data_len[read_idx];

    if (len > 0) {
        memcpy(active_buf, pending_buf[read_idx], len);
        /* active_buf[len..511] retains values from the previous Latch */
    }
}

/** @brief Single-universe driver; only index 0 is valid. */
static const uint8_t* DMX512_Uart_GetUniverse(uint8_t universe)
{
    if (universe != 0) return NULL;
    return active_buf;
}

/* ========================== ISR Sub-handlers ========================== */

/**
 * @brief Handle ORE / NE / PE — clear before HAL sees them
 *
 * HAL's error path calls UART_EndRxTransfer() which disables EIE and
 * aborts DMA, permanently killing break detection.
 *
 * @return true on ORE (caller must exit ISR), false to fall through
 */
static bool DMX512_Uart_HandleErrors(UART_HandleTypeDef *huart, uint32_t isr)
{
    SET_BIT(huart->Instance->ICR,
            USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF);

    if (isr & USART_ISR_ORE) {
        /* Byte lost → packet data shifted, discard and resync */
        DMX512_Uart_StartDmaRx();
        rx_state = DMX512_STATE_IDLE;
        return true;
    }
    /* NE/PE alone: noisy byte, UART continues — fall through */
    return false;
}

/**
 * @brief Handle Framing Error — Break detection or mid-packet corruption
 *
 * FE + RDR==0x00: valid Break. Process any in-flight packet (MBB=0
 * per DMX512-A §8.9), then restart DMA for the new packet.
 * FE + RDR!=0x00: corruption. Salvage clean prefix (§9.1), then resync.
 */
static void DMX512_Uart_HandleFramingError(UART_HandleTypeDef *huart)
{
    SET_BIT(huart->Instance->ICR, USART_ICR_FECF);
    uint8_t rdr = (uint8_t)READ_REG(huart->Instance->RDR);

    if (rdr == 0x00) {
        /* Valid Break — flush any in-flight packet, then start fresh */
        if (rx_state == DMX512_STATE_RECEIVING) {
            uint16_t received = DMX512_Uart_DmaReceived();
            if (received > 0) {
                DMX512_Uart_ProcessPacket(received);
            }
        }
        DMX512_Uart_StartDmaRx();
        rx_state = DMX512_STATE_RECEIVING;
        return;
    }

    /* Non-zero FE: mid-packet corruption or line noise */
    midPcktErrCnt++; // DEBUGGING

    if (rx_state == DMX512_STATE_RECEIVING) {
        /* Salvage clean bytes before the corrupted slot (§9.1) */
        uint16_t received = DMX512_Uart_DmaReceived();
        if (received >= 2) {
            DMX512_Uart_ProcessPacket(received);
        }
        DMX512_Uart_StartDmaRx();
        rx_state = DMX512_STATE_IDLE;
    }
}

/**
 * @brief Handle IDLE line — end-of-packet detection
 *
 * MAB also triggers IDLE (~1.24 ms HIGH). If no bytes received yet,
 * this is the MAB — stay in RECEIVING for the real end-of-packet.
 */
static void DMX512_Uart_HandleIdle(UART_HandleTypeDef *huart)
{
    SET_BIT(huart->Instance->ICR, USART_ICR_IDLECF);

    if (rx_state != DMX512_STATE_RECEIVING) return;

    uint16_t received = DMX512_Uart_DmaReceived();
    if (received == 0) return;  /* MAB, not end-of-packet */

    DMX512_Uart_ProcessPacket(received);
    rx_state = DMX512_STATE_IDLE;
}

/* ========================== ISR Handler ========================== */

/**
 * @brief Custom UART ISR — called from UART5_IRQHandler before HAL
 *
 * Dispatches to focused sub-handlers for each flag type.
 * Must NOT block — task notification deferred via DMA1_Stream1_IRQn.
 */
void DMX512_Uart_IRQHandler(UART_HandleTypeDef *huart)
{
    if (!initialized || huart->Instance != UART5) return;

    uint32_t isr = READ_REG(huart->Instance->ISR);

    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_PE)) {
        if (DMX512_Uart_HandleErrors(huart, isr)) return;
    }

    if (isr & USART_ISR_FE) {
        DMX512_Uart_HandleFramingError(huart);
        return;
    }

    if (isr & USART_ISR_IDLE) {
        DMX512_Uart_HandleIdle(huart);
    }
}

/* ========================== Deferred Notification ========================== */

/**
 * @brief Deferred notification from UART5 ISR → FreeRTOS-safe context
 * @return true if handled (caller should skip HAL_DMA_IRQHandler)
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
