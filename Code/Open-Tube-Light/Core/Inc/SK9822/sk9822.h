/**
 * @file sk9822.h
 * @brief SK9822 LED Strip Driver with Ping-Pong DMA
 *
 * Owns both SPI DMA buffers internally. Callers use BeginFrame / SetLed /
 * EndFrame to build and transmit frames without touching buffer pointers.
 */

#ifndef INC_SK9822_H_
#define INC_SK9822_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Configuration ========================== */

#define SK9822_STRIP_LED_COUNT  144

/* ========================== Frame Sizing ========================== */

#define SK9822_START_FRAME_SIZE 4U
#define SK9822_LED_FRAME_SIZE   4U
// Datasheet says 32 '1' bits, but this works more reliably for SK9822
#define SK9822_END_FRAME_SIZE   ((SK9822_STRIP_LED_COUNT + 15U) / 16U)
// End frame fill byte (0x00 or 0xFF) may depend on the specific SK9822 clone
#define SK9822_END_FRAME_FILL   0x00U
#define SK9822_BUFFER_SIZE      (SK9822_START_FRAME_SIZE \
                                + (SK9822_STRIP_LED_COUNT * SK9822_LED_FRAME_SIZE) \
                                + SK9822_END_FRAME_SIZE)

/* ========================== Handle ========================== */

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t           *active_buf;   // currently being transmitted (or last transmitted)
    uint8_t           *prepare_buf;  // being built by caller
    volatile uint8_t   tx_busy;      // 1 while DMA is in-flight
} SK9822_Handle_t;

/* ========================== Public API ========================== */

/**
 * @brief Initialize handle and both internal DMA buffers
 *
 * Zeros buffers, writes valid start/end frames in both, so an idle
 * refresh outputs an all-dark stream immediately.
 *
 * @param h     Handle to initialize
 * @param hspi  SPI peripheral used for DMA transfers
 */
void SK9822_Init(SK9822_Handle_t *h, SPI_HandleTypeDef *hspi);

/**
 * @brief Begin building a new frame
 *
 * Waits for any in-flight DMA to finish, then writes start/end frames
 * into the prepare buffer. After this call, use SK9822_SetLed /
 * SK9822_SetAll to populate pixel data.
 *
 * @param h  Handle
 */
void SK9822_BeginFrame(SK9822_Handle_t *h);

/**
 * @brief Set one LED in the prepare buffer (RGB + global brightness)
 *
 * @param h           Handle
 * @param index       LED index (0-based, clamped to strip length)
 * @param r, g, b     Color channels 0..255
 * @param brightness  Global brightness 0..255 (mapped to 5-bit HW range)
 */
void SK9822_SetLed(SK9822_Handle_t *h, uint16_t index,
                   uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

/**
 * @brief Set all LEDs in the prepare buffer to the same color
 */
void SK9822_SetAll(SK9822_Handle_t *h,
                   uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

/**
 * @brief Swap buffers and start DMA transfer of the new active buffer
 *
 * @return HAL status (HAL_OK on success)
 */
HAL_StatusTypeDef SK9822_EndFrame(SK9822_Handle_t *h);

/**
 * @brief Retransmit the current active buffer (no swap, no rebuild)
 *
 * Waits for DMA idle, then re-sends. Use for periodic refresh when
 * no new pixel data has arrived.
 *
 * @return HAL status
 */
HAL_StatusTypeDef SK9822_Refresh(SK9822_Handle_t *h);

/**
 * @brief SPI TX complete ISR hook - call from HAL_SPI_TxCpltCallback
 */
void SK9822_OnTxComplete(SK9822_Handle_t *h, SPI_HandleTypeDef *hspi);

/**
 * @brief Convert 0..255 brightness to SK9822 global byte (0b111xxxxx)
 */
uint8_t SK9822_BrightnessByte(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif /* INC_SK9822_H_ */
