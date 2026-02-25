/**
 * @file artnet.h
 * @brief Art-Net 4 Protocol Handler for STM32H7 - Public API
 * 
 * This is the main public interface for the Art-Net implementation.
 * It provides functions for initialization, data access, and state monitoring.
 * 
 * The implementation supports Art-Net 4 with OpSync for tear-free LED updates
 * across multiple universes. It uses double-buffering (shadow/active) to
 * ensure atomic updates during LED refresh.
 * 
 * Configuration is provided by the device_config module, allowing runtime
 * changes to Art-Net addressing and behavior.
 * 
 * @see artnet_protocol.h for Art-Net protocol structures and constants
 * @see device_config.h for configuration management
 */

#ifndef INC_ARTNET_H_
#define INC_ARTNET_H_

#include "artnet_protocol.h"
#include "device_config.h"
#include "cmsis_os.h"
#include "lwip/ip_addr.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Public Types ========================== */

/**
 * @brief Per-universe Art-Net state
 * 
 * Each universe independently tracks its source controller, timing, and
 * sync mode. This enables multi-controller operation where different
 * controllers drive different universes on the same node, each with
 * independent sync behavior.
 */
typedef struct {
    ip_addr_t  source_ip;       ///< Source controller IP (0.0.0.0 = unassigned, first-source-wins)
    uint32_t   last_dmx_tick;   ///< Timestamp of last ArtDmx for disconnect detection
    bool       sync_mode;       ///< True when this universe's controller is sending ArtSync
    uint32_t   last_sync_tick;  ///< Timestamp of last ArtSync from this universe's controller
} ArtNet_UniverseState_t;

/**
 * @brief Art-Net operational state
 * 
 * This structure tracks the current operational mode and synchronization
 * state of the Art-Net receiver. Per-universe state enables multi-controller
 * environments where different controllers can independently drive and
 * synchronize different subsets of universes.
 */
typedef struct {
    uint8_t                universes_received;                       ///< Bitmask of universes received since last latch
    uint8_t                universes_expected;                       ///< Bitmask of all configured universes
    ArtNet_UniverseState_t universes[DEVICE_CONFIG_MAX_UNIVERSES];   ///< Per-universe state (source IP, sync, timing)
} ArtNet_State_t;

/* ========================== Public API Functions ========================== */

/**
 * @brief Initialize the Art-Net receiver
 * 
 * Must be called once during system initialization, after LwIP is up.
 * Configuration is read from device_config module.
 * 
 * @param processing_task_handle RTOS task handle to notify on new data
 * @return 0 on success, -1 on failure
 * 
 * @note Must be called from a context where LwIP tcpip_thread is running
 * @note DeviceConfig_Init() must be called before this function
 */
int ArtNet_Init(osThreadId_t processing_task_handle);

/**
 * @brief Get pointer to the active DMX buffer for a specific universe
 * 
 * @param universe Universe index (0 to configured universe count - 1)
 * @return Pointer to 512-byte DMX buffer, or NULL if universe index invalid
 * 
 * @note Buffer is read-only
 */
const uint8_t* ArtNet_GetUniverseData(uint8_t universe);

/**
 * @brief Check if a new frame is ready for processing
 * 
 * @return true if new data is ready, false otherwise
 */
bool ArtNet_IsFrameReady(void);

/**
 * @brief Latch shadow buffer to active buffer
 * 
 * Atomically copies all shadow buffers to active buffers.
 * Call from processing task when ArtNet_IsFrameReady() returns true.
 */
void ArtNet_LatchData(void);

/**
 * @brief Get current Art-Net operational state
 * 
 * @return Pointer to const state structure
 */
const ArtNet_State_t* ArtNet_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_H_ */
