/*
 * ledStrip.h
 *
 *  Created on: Oct 19, 2025
 *      Author: popvo
 */

#ifndef INC_SK9822_H_
#define INC_SK9822_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Public configuration - adjust as needed
#define SK9822_STRIP_LED_COUNT 144U

// SK9822 frame sizing
#define SK9822_START_FRAME_SIZE 4U
#define SK9822_LED_FRAME_SIZE   4U
#define SK9822_END_FRAME_SIZE   ((SK9822_STRIP_LED_COUNT + 15U) / 16U) //The datasheet says that the end frame should be composed of 32 ‘1’ bits, but this does not work reliably
#define SK9822_END_FRAME_FILL   0x00U // End-frame fill: some strips prefer 0xFF, some work better with 0x00.
#define SK9822_BUFFER_SIZE      (SK9822_START_FRAME_SIZE + (SK9822_STRIP_LED_COUNT * SK9822_LED_FRAME_SIZE) + SK9822_END_FRAME_SIZE)

// Handle structure
    typedef struct {
            SPI_HandleTypeDef *hspi;    // SPI handle to use for transfers
            volatile uint8_t *tx_busy_flag;       // external flag owned by app/ISR
            uint8_t *buffer;       // full SPI frame buffer (start + LEDs + end)
    } SK9822_HandleTypeDef;

// Initialize handle
    void sk9822_init(SK9822_HandleTypeDef *hsk9822, SPI_HandleTypeDef *hspi,
            uint8_t *buffer, volatile uint8_t *tx_busy_flg);

// Prepare start and end frames in the buffer; does not modify LED frames.
    void sk9822_prepare_frames(SK9822_HandleTypeDef *hsk9822);

// Convert 0..255 brightness to SK9822 global brightness byte (0b111xxxxx)
    uint8_t sk9822_brightness_byte(uint8_t brightness);

// Set one LED (BGR order per SK9822) and per-LED brightness (0..255)
    void sk9822_set_led(SK9822_HandleTypeDef *hsk9822, uint16_t index,
            uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

// Convenience: set all LEDs to same color/brightness
    void sk9822_set_all(SK9822_HandleTypeDef *hsk9822, uint8_t r, uint8_t g,
            uint8_t b, uint8_t brightness);

// Kick a DMA transfer of the whole buffer if not busy; returns HAL_BUSY if in-flight
    HAL_StatusTypeDef sk9822_show(SK9822_HandleTypeDef *hsk9822);

// To be called from HAL_SPI_TxCpltCallback to clear busy flag for this handle
    void sk9822_on_tx_cplt_callback(SK9822_HandleTypeDef *hsk9822,
            SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* INC_SK9822_H_ */
