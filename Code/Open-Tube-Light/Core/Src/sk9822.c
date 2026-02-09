/*
 * ledStrip.c
 *
 *  Created on: Oct 19, 2025
 *      Author: popvo
 */

#include <sk9822.h>
#include "main.h"
#include <string.h>

// Prepare handle with references and derived sizes
void sk9822_init(SK9822_HandleTypeDef *hsk9822, SPI_HandleTypeDef *hspi,
        uint8_t *buffer, volatile uint8_t *tx_busy_flg) {
    if(!hsk9822 || !hspi || !buffer || !tx_busy_flg)
        return;
    hsk9822->hspi = hspi;
    hsk9822->buffer = buffer;
    hsk9822->tx_busy_flag = tx_busy_flg;
    sk9822_prepare_frames(hsk9822);
}

// Initialize start and end frames; leaves LED frames untouched
void sk9822_prepare_frames(SK9822_HandleTypeDef *hsk9822) {
    // Start frame: 32 bits of zero
    memset(hsk9822->buffer, 0x00, SK9822_START_FRAME_SIZE);
    // End frame: depends on strip series and length, configurable in header file
    uint16_t end_offset = (uint16_t) (SK9822_START_FRAME_SIZE
            + (SK9822_STRIP_LED_COUNT * SK9822_LED_FRAME_SIZE));
    memset(&hsk9822->buffer[end_offset], SK9822_END_FRAME_FILL,
    SK9822_END_FRAME_SIZE);
}

// Convert 0..255 to 0b111xxxxx where xxxxx is 0..31
uint8_t sk9822_brightness_byte(uint8_t brightness) {
    uint8_t scaled = (uint8_t) ((uint16_t) brightness * 31U / 255U);
    return (uint8_t) (0b11100000 | scaled); // first 3 bits must be '1' as per the sk9822 datasheet
}

// Function for testing
// Set one LED's global brightness and BGR color
void sk9822_set_led(SK9822_HandleTypeDef *hsk9822, uint16_t index, uint8_t r,
        uint8_t g, uint8_t b, uint8_t brightness) {
    if(index >= SK9822_STRIP_LED_COUNT)
        return;
    uint16_t offset = (uint16_t) (SK9822_START_FRAME_SIZE
            + (index * SK9822_LED_FRAME_SIZE));
    hsk9822->buffer[offset + 0] = sk9822_brightness_byte(brightness);
    hsk9822->buffer[offset + 1] = b;
    hsk9822->buffer[offset + 2] = g;
    hsk9822->buffer[offset + 3] = r;
}

// Start DMA transfer of entire buffer if idle
HAL_StatusTypeDef sk9822_show(SK9822_HandleTypeDef *hsk9822) {
    if(*hsk9822->tx_busy_flag)
        return HAL_BUSY;
    *hsk9822->tx_busy_flag = 1U;
    HAL_StatusTypeDef st = HAL_SPI_Transmit_DMA(hsk9822->hspi, hsk9822->buffer,
            SK9822_BUFFER_SIZE);
    if(st != HAL_OK) {
        *hsk9822->tx_busy_flag = 0U; // rollback busy flag on failure
    }
    return st;
}

void sk9822_on_tx_cplt_callback(SK9822_HandleTypeDef *hsk9822,
        SPI_HandleTypeDef *hspi) {
    if(hsk9822->hspi == hspi) {
        *hsk9822->tx_busy_flag = 0U;
    }
}

// Function for testing
void sk9822_set_all(SK9822_HandleTypeDef *hsk9822, uint8_t r, uint8_t g,
        uint8_t b, uint8_t brightness) {
    uint8_t gb = sk9822_brightness_byte(brightness);
    for(uint16_t i = 0; i < SK9822_STRIP_LED_COUNT; ++i) {
        uint16_t offset = (uint16_t) (SK9822_START_FRAME_SIZE
                + (i * SK9822_LED_FRAME_SIZE));
        hsk9822->buffer[offset + 0] = gb;
        hsk9822->buffer[offset + 1] = b;
        hsk9822->buffer[offset + 2] = g;
        hsk9822->buffer[offset + 3] = r;
    }
}
