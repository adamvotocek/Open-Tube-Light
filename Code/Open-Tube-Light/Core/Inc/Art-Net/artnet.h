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
 * @brief Art-Net operational state
 * 
 * This structure tracks the current operational mode and synchronization
 * state of the Art-Net receiver. Used for diagnostics and to determine
 * if OpSync mode is active.
 */
typedef struct {
    uint8_t             universes_received;                               ///< Bitmask of universes received this frame
    uint8_t             universes_expected;                               ///< Bitmask of all configured universes
    ip_addr_t           universe_source_ip[DEVICE_CONFIG_MAX_UNIVERSES];  ///< Source IP per universe (first source wins)
    ip_addr_t           last_artdmx_ip;                                   ///< Source IP of last ArtDmx packet
    bool                sync_mode;                                        ///< True when OpSync packets are being received
    uint32_t            last_sync_tick;                                   ///< Timestamp of last OpSync
    uint32_t            last_artdmx_tick;                                 ///< Timestamp of last ArtDmx packet
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