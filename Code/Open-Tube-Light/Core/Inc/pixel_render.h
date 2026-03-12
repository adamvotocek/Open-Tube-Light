/**
 * @file pixel_render.h
 * @brief Pixel Rendering Module
 *
 * Converts DMX channel data into SK9822 pixel frames. Handles DMX-to-pixel
 * mapping (start address, multi-universe spanning), failsafe behavior,
 * and periodic idle refresh.
 *
 * Decouples main.c task logic from hardware and protocol details.
 */

#ifndef INC_PIXEL_RENDER_H_
#define INC_PIXEL_RENDER_H_

#include "SK9822/sk9822.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the pixel render module
 *
 * @param h  Initialized SK9822 handle (SK9822_Init must have been called)
 */
void Pixel_Render_Init(SK9822_Handle_t *h);

/**
 * @brief Build and transmit a new frame from current DMX data
 *
 * Reads device config (pixel format, start address, universe count),
 * maps DMX channels to SK9822 pixels, and kicks DMA.
 */
void Pixel_Render_Frame(void);

/**
 * @brief Apply failsafe output (blackout, full-on, or hold)
 *
 * Called when the DMX source disconnect timeout expires.
 * Behavior is controlled by DeviceConfig_t.output.failsafe.
 */
void Pixel_Render_Failsafe(void);

/**
 * @brief Retransmit the last frame without modification
 *
 * Used for periodic refresh so the LED strip doesn't lose data
 * (SK9822 has no internal latch memory on some clones).
 */
void Pixel_Render_Refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PIXEL_RENDER_H_ */
