/**
 * @file dmx_input.h
 * @brief DMX Input Abstraction Layer
 * 
 * Protocol-agnostic interface for DMX data sources. Each protocol
 * (Art-Net, sACN, DMX512) implements the DMX_Input_Driver_t vtable.
 * The manager selects the active driver based on device configuration,
 * allowing runtime switching between protocols.
 * 
 * Consumer code (e.g. pixel rendering) calls only the DMX_Input_*
 * functions and never references a specific protocol directly.
 */

#ifndef INC_DMX_INPUT_H_
#define INC_DMX_INPUT_H_

#include "device_config.h"
#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Types ========================== */

/**
 * @brief DMX input driver vtable
 * 
 * Each protocol implements this interface. All functions must be non-NULL.
 * init/deinit are called from a thread with LwIP access (for network protocols).
 */
typedef struct {
    /**
     * @brief Initialize the protocol and begin receiving data
     * @param task RTOS task handle to notify on new data (via thread flags)
     * @return 0 on success, -1 on failure
     */
    int           (*init)(osThreadId_t task);
    
    /**
     * @brief Tear down the protocol and release all resources
     * 
     * After deinit, the driver must be safe to re-init or replace.
     */
    void          (*deinit)(void);
    
    /**
     * @brief Latch received data for the consumer
     * 
     * Copies internal shadow buffers to active buffers so the consumer
     * reads a consistent snapshot across all universes.
     */
    void          (*latch)(void);
    
    /**
     * @brief Get pointer to active DMX data for a universe
     * @param universe Universe index (0-based, relative to configured start)
     * @return Pointer to 512-byte buffer, or NULL if index out of range
     */
    const uint8_t*(*get_universe)(uint8_t universe);
} DMX_Input_Driver_t;

/* ========================== Public API ========================== */

/**
 * @brief Start a DMX input source
 * 
 * Tears down any currently active source, then initializes the new one.
 * 
 * @param source Protocol to activate (from device config enum)
 * @param task   RTOS task handle to notify on new data
 * @return 0 on success, -1 on failure (previous source already torn down)
 */
int DMX_Input_Start(DeviceConfig_DmxInput_t source, osThreadId_t task);

/**
 * @brief Stop the current DMX input source
 * 
 * Tears down the active source. Safe to call when nothing is active.
 */
void DMX_Input_Stop(void);

/**
 * @brief Latch received DMX data for rendering
 * 
 * Delegates to the active driver's latch function.
 * No-op if no driver is active.
 */
void DMX_Input_Latch(void);

/**
 * @brief Get active DMX buffer for a universe
 * 
 * Delegates to the active driver's get_universe function.
 * 
 * @param universe Universe index (0-based)
 * @return Pointer to 512-byte buffer, or NULL if no driver active or index invalid
 */
const uint8_t* DMX_Input_GetUniverse(uint8_t universe);

/* ========================== Driver Instances ========================== */

/** @brief Art-Net driver (defined in artnet.c) */
extern const DMX_Input_Driver_t dmx_input_artnet;

#ifdef __cplusplus
}
#endif

#endif /* INC_DMX_INPUT_H_ */
