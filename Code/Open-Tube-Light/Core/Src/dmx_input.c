/**
 * @file dmx_input.c
 * @brief DMX Input Abstraction Layer - Manager
 * 
 * Manages the active DMX input driver, routing API calls to the
 * currently selected protocol. Handles safe switching between
 * protocols (deinit old, init new).
 */

#include "dmx_input.h"
#include <stddef.h>

/* ========================== Private State ========================== */

static const DMX_Input_Driver_t *active_driver = NULL;

/* ========================== Private Helpers ========================== */

/**
 * @brief Resolve config enum to driver instance
 * @return Driver pointer, or NULL if source not supported
 */
static const DMX_Input_Driver_t* DMX_Input_ResolveDriver(DeviceConfig_DmxInput_t source)
{
    switch (source) {
        case DMX_INPUT_ARTNET: return &dmx_input_artnet;
        // case DMX_INPUT_SACN:   return &dmx_input_sacn;
        // case DMX_INPUT_DMX512: return &dmx_input_dmx512;
        default:               return NULL;
    }
}

/* ========================== Public API ========================== */

int DMX_Input_Start(DeviceConfig_DmxInput_t source, osThreadId_t task)
{
    const DMX_Input_Driver_t *driver = DMX_Input_ResolveDriver(source);
    if (driver == NULL) {
        return -1;
    }
    
    // Tear down previous source before switching
    if (active_driver != NULL) {
        active_driver->deinit();
        active_driver = NULL;
    }
    
    int ret = driver->init(task);
    if (ret == 0) {
        active_driver = driver;
    }
    return ret;
}

void DMX_Input_Stop(void)
{
    if (active_driver != NULL) {
        active_driver->deinit();
        active_driver = NULL;
    }
}

void DMX_Input_Latch(void)
{
    if (active_driver != NULL) {
        active_driver->latch();
    }
}

const uint8_t* DMX_Input_GetUniverse(uint8_t universe)
{
    if (active_driver != NULL) {
        return active_driver->get_universe(universe);
    }
    return NULL;
}

uint32_t DMX_Input_GetFailsafeTimeout(void)
{
    if (active_driver != NULL) {
        return active_driver->failsafe_timeout_ms;
    }
    return 12000;  // Conservative default
}
