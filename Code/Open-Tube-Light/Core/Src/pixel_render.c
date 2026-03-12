/**
 * @file pixel_render.c
 * @brief Pixel Rendering Module - Implementation
 *
 * Maps DMX channel data to SK9822 pixel frames. Owns the
 * DMX→pixel conversion logic so that the RTOS task in main.c
 * stays a pure state-machine with no hardware or protocol details.
 */

#include "pixel_render.h"
#include "dmx_input.h"
#include "device_config.h"
#include <stddef.h>

/* ========================== Private State ========================== */

static SK9822_Handle_t *strip;

/* ========================== Private Helpers ========================== */

/**
 * @brief Map DMX universes into the prepare buffer using current config
 *
 * Iterates across universes starting at the configured DMX start address
 * and writes one LED per channels-per-pixel group.
 */
static void Pixel_Render_MapDmx(void)
{
    const DeviceConfig_t *cfg = DeviceConfig_Get();
    uint8_t  cpp           = DeviceConfig_GetChannelsPerPixel();
    uint8_t  num_universes = DeviceConfig_GetUniverseCount();
    uint16_t led_idx       = 0;

    for (uint8_t uni = 0; uni < num_universes && led_idx < SK9822_STRIP_LED_COUNT; uni++) {
        const uint8_t *dmx = DMX_Input_GetUniverse(uni);
        if (dmx == NULL) continue;

        /* First universe: skip to dmx_start_address offset (1-based).
         * Subsequent universes: pixel data starts at channel 0. */
        uint16_t ch = (uni == 0) ? (cfg->dmx.dmx_start_address - 1) : 0;

        /* Read pixels until universe boundary or strip full.
         * Ensure a full pixel (cpp channels) fits before reading. */
        for (; (ch + cpp) <= DMX_UNIVERSE_MAX_LENGTH && led_idx < SK9822_STRIP_LED_COUNT;
             ch += cpp) {
            /* TODO: handle RGBW and RGB16 formats (currently RGB only) */
            SK9822_SetLed(strip, led_idx, dmx[ch], dmx[ch + 1], dmx[ch + 2], 255);
            led_idx++;
        }
    }
}

/* ========================== Public API ========================== */

void Pixel_Render_Init(SK9822_Handle_t *h)
{
    strip = h;
}

void Pixel_Render_Frame(void)
{
    SK9822_BeginFrame(strip);
    Pixel_Render_MapDmx();
    SK9822_EndFrame(strip);
}

void Pixel_Render_Failsafe(void)
{
    const DeviceConfig_t *cfg = DeviceConfig_Get();

    switch (cfg->output.failsafe) {
        case FAILSAFE_ZERO:
            SK9822_BeginFrame(strip);
            SK9822_SetAll(strip, 0, 0, 0, 0);
            SK9822_EndFrame(strip);
            break;

        case FAILSAFE_FULL:
            SK9822_BeginFrame(strip);
            SK9822_SetAll(strip, 255, 255, 255, 255);
            SK9822_EndFrame(strip);
            break;

        case FAILSAFE_HOLD:
        default:
            /* Retransmit whatever is in the active buffer (last good frame) */
            SK9822_Refresh(strip);
            break;
    }
}

void Pixel_Render_Refresh(void)
{
    SK9822_Refresh(strip);
}
