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

// DEBUG
uint32_t render_count = 0;
uint32_t failsafe_count = 0;
uint32_t refresh_count = 0;

/* ========================== Private Helpers ========================== */

/**
 * @brief Map DMX universes into the prepare buffer using current config
 *
 * Iterates across universes starting at the configured DMX start address,
 * reads one logical DMX pixel per segment, and fans that color out across
 * the physical LEDs belonging to the segment.
 */
static void Pixel_Render_MapDmx(void)
{
    
    
    
    const DeviceConfig_t *cfg = DeviceConfig_Get();
    uint8_t  channels_per_segment = DeviceConfig_GetChannelsPerSegment();
    uint8_t  num_universes = DeviceConfig_GetUniverseCount();
    uint16_t segment_size;
    uint16_t physical_limit;
    uint16_t segment_idx = 0;
    uint16_t led_idx = 0;

    if (cfg->layout.segment_count == 0U || cfg->layout.pixel_count == 0U) {
        SK9822_SetAll(strip, 0, 0, 0, 0);
        return;
    }

    segment_size = cfg->layout.pixel_count / cfg->layout.segment_count;
    physical_limit = cfg->layout.pixel_count;
    if (physical_limit > SK9822_STRIP_LED_COUNT) {
        physical_limit = SK9822_STRIP_LED_COUNT;
    }

    for (uint8_t uni = 0;
         uni < num_universes && segment_idx < cfg->layout.segment_count && led_idx < physical_limit;
         uni++) {
        const uint8_t *dmx = DMX_Input_GetUniverse(uni);
        if (dmx == NULL) continue;

        /* First universe: skip to dmx_start_address offset (1-based).
         * Subsequent universes: pixel data starts at channel 0. */
        uint16_t ch = (uni == 0) ? (cfg->dmx.dmx_start_address - 1) : 0;

                /* Read one DMX logical segment sample per configured segment, then
                 * copy that color across the physical LEDs belonging to the segment. */
                for (; (ch + channels_per_segment) <= DMX_UNIVERSE_MAX_LENGTH &&
                             segment_idx < cfg->layout.segment_count &&
               led_idx < physical_limit;
                         ch += channels_per_segment, segment_idx++) {
            uint8_t red = dmx[ch];
            uint8_t green = dmx[ch + 1];
            uint8_t blue = dmx[ch + 2];

            /* TODO: handle RGBW and RGB16 formats (currently RGB only) */
            for (uint16_t repeat = 0; repeat < segment_size && led_idx < physical_limit; repeat++) {
                SK9822_SetLed(strip, led_idx, red, green, blue, 255);
                led_idx++;
            }
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
    render_count++;
}

void Pixel_Render_Failsafe(void)
{
    failsafe_count++;

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
    refresh_count++;
}
