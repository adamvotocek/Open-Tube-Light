/**
 * @file sk9822.c
 * @brief SK9822 LED Strip Driver - Implementation
 *
 * Ping-pong DMA buffers are declared here in D2 SRAM (non-cacheable via MPU).
 * The handle owns both buffers; callers never touch pointers directly.
 */

#include "SK9822/sk9822.h"
#include "cmsis_os2.h"
#include <string.h>

/* ========================== Internal Buffers ========================== */

/* Both buffers in D2 SRAM — DMA-accessible, non-cacheable via MPU Region 1 */
static uint8_t spi_buf_a[SK9822_BUFFER_SIZE]
    __attribute__((section(".spi_buffers"), aligned(32), used));

static uint8_t spi_buf_b[SK9822_BUFFER_SIZE]
    __attribute__((section(".spi_buffers"), aligned(32), used));

/* ========================== Private Helpers ========================== */

/** Write start frame (4 zero bytes) and end frame into a buffer */
static void SK9822_PrepareFrames(uint8_t *buf)
{
    memset(buf, 0x00, SK9822_START_FRAME_SIZE);

    uint16_t end_offset = (uint16_t)(SK9822_START_FRAME_SIZE
                          + (SK9822_STRIP_LED_COUNT * SK9822_LED_FRAME_SIZE));
    memset(&buf[end_offset], SK9822_END_FRAME_FILL, SK9822_END_FRAME_SIZE);
}

/** Block until DMA is idle (cooperative yield via osDelay) */
static void SK9822_WaitDmaIdle(SK9822_Handle_t *h)
{
    while (h->tx_busy) {
        osDelay(1);
    }
}

/** Kick a DMA transfer of the given buffer */
static HAL_StatusTypeDef SK9822_TransmitDma(SK9822_Handle_t *h, uint8_t *buf)
{
    h->tx_busy = 1U;
    HAL_StatusTypeDef st = HAL_SPI_Transmit_DMA(h->hspi, buf, SK9822_BUFFER_SIZE);
    if (st != HAL_OK) {
        h->tx_busy = 0U;
    }
    return st;
}

/* ========================== Public API ========================== */

void SK9822_Init(SK9822_Handle_t *h, SPI_HandleTypeDef *hspi)
{
    if (!h || !hspi) return;

    h->hspi       = hspi;
    h->active_buf  = spi_buf_a;
    h->prepare_buf = spi_buf_b;
    h->tx_busy     = 0U;

    /* Zero both to prevent stale frame output after soft reset
     * (NOLOAD sections are not cleared by startup code) */
    memset(spi_buf_a, 0, sizeof(spi_buf_a));
    memset(spi_buf_b, 0, sizeof(spi_buf_b));

    SK9822_PrepareFrames(spi_buf_a);
    SK9822_PrepareFrames(spi_buf_b);
}

void SK9822_BeginFrame(SK9822_Handle_t *h)
{
    SK9822_WaitDmaIdle(h);
    SK9822_PrepareFrames(h->prepare_buf);
}

void SK9822_SetLed(SK9822_Handle_t *h, uint16_t index,
                   uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index >= SK9822_STRIP_LED_COUNT) return;

    uint16_t offset = (uint16_t)(SK9822_START_FRAME_SIZE
                      + (index * SK9822_LED_FRAME_SIZE));
    h->prepare_buf[offset + 0] = SK9822_BrightnessByte(brightness);
    h->prepare_buf[offset + 1] = b;
    h->prepare_buf[offset + 2] = g;
    h->prepare_buf[offset + 3] = r;
}

void SK9822_SetAll(SK9822_Handle_t *h,
                   uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    uint8_t gb = SK9822_BrightnessByte(brightness);
    for (uint16_t i = 0; i < SK9822_STRIP_LED_COUNT; ++i) {
        uint16_t offset = (uint16_t)(SK9822_START_FRAME_SIZE
                          + (i * SK9822_LED_FRAME_SIZE));
        h->prepare_buf[offset + 0] = gb;
        h->prepare_buf[offset + 1] = b;
        h->prepare_buf[offset + 2] = g;
        h->prepare_buf[offset + 3] = r;
    }
}

HAL_StatusTypeDef SK9822_EndFrame(SK9822_Handle_t *h)
{
    /* Swap prepare ↔ active, then transmit the new active buffer */
    uint8_t *tmp   = h->active_buf;
    h->active_buf  = h->prepare_buf;
    h->prepare_buf = tmp;

    return SK9822_TransmitDma(h, h->active_buf);
}

HAL_StatusTypeDef SK9822_Refresh(SK9822_Handle_t *h)
{
    SK9822_WaitDmaIdle(h);
    return SK9822_TransmitDma(h, h->active_buf);
}

void SK9822_OnTxComplete(SK9822_Handle_t *h, SPI_HandleTypeDef *hspi)
{
    if (h->hspi == hspi) {
        h->tx_busy = 0U;
    }
}

uint8_t SK9822_BrightnessByte(uint8_t brightness)
{
    uint8_t scaled = (uint8_t)((uint16_t)brightness * 31U / 255U);
    return (uint8_t)(0xE0U | scaled);
}
