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
 * @see artnet_protocol.h for Art-Net protocol structures and constants
 * @see artnet_config.h for configuration parameters
 */

#ifndef INC_ARTNET_H_
#define INC_ARTNET_H_

#include "artnet_protocol.h"
#include "artnet_config.h"
#include "cmsis_os.h"
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
 * state of the Art-Net receiver. It's primarily used for diagnostics and
 * to determine if OpSync mode is active.
 */
typedef struct {
    bool     sync_mode;             ///< True when OpSync packets are being received
    uint32_t last_sync_tick;        ///< Timestamp of last OpSync (for timeout detection)
    uint8_t  universes_received;    ///< Bitmask of universes received this frame
    uint8_t  universes_expected;    ///< Bitmask of all configured universes
} ArtNet_State_t;

/* ========================== Public API Functions ========================== */

/**
 * @brief Initialize the Art-Net receiver
 * 
 * This function must be called once during system initialization, after
 * the network stack (LwIP) is up and running. It creates a UDP socket,
 * binds to the Art-Net port, and sets up receive callbacks.
 * 
 * The processing_task_handle is used to notify the application when new
 * DMX data is ready via osThreadFlagsSet(task, 0x01).
 * 
 * @param processing_task_handle RTOS task handle to notify on new data
 * @return 0 on success, -1 on failure (socket creation or bind error)
 * 
 * @note Must be called from a context where LwIP tcpip_thread is running
 * @note Buffers are placed in DTCM for fast CPU access (see linker script)
 */
int ArtNet_Init(osThreadId_t processing_task_handle);

/**
 * @brief Get pointer to the active DMX buffer for a specific universe
 * 
 * Returns a read-only pointer to the 512-byte DMX data buffer. This
 * buffer contains the most recently latched data and is safe to read
 * from the processing task at any time (double-buffering ensures
 * atomicity).
 * 
 * @param universe Universe index (0 to ARTNET_NUM_UNIVERSES-1)
 * @return Pointer to 512-byte DMX buffer, or NULL if universe index invalid
 * 
 * @note Buffer is read-only. Do not modify the returned data.
 * @note Always check for NULL before dereferencing
 */
const uint8_t* ArtNet_GetUniverseData(uint8_t universe);

/**
 * @brief Check if a new frame is ready for processing
 * 
 * Returns true when new DMX data has been received and is ready to be
 * latched. In non-sync mode, this is set when all configured universes
 * have been received. In sync mode, it's set when OpSync is received.
 * 
 * @return true if new data is ready, false otherwise
 * 
 * @note Typically called in a processing task's main loop
 * @note Cleared by calling ArtNet_LatchData()
 */
bool ArtNet_IsFrameReady(void);

/**
 * @brief Latch shadow buffer to active buffer
 * 
 * Atomically copies all shadow buffers to active buffers and clears
 * the frame_ready flag. This should be called from the processing task
 * when ArtNet_IsFrameReady() returns true, immediately before rendering.
 * 
 * The double-buffering ensures that DMX data remains stable during LED
 * refresh, preventing tearing artifacts.
 * 
 * @note Must be called from processing task when frame is ready
 * @note Resets universes_received bitmask for next frame
 */
void ArtNet_LatchData(void);

/**
 * @brief Get current Art-Net operational state
 * 
 * Returns a read-only pointer to the internal state structure. This
 * is primarily useful for diagnostics, monitoring, or UI display.
 * 
 * @return Pointer to const state structure
 * 
 * @note State is updated by the Art-Net receive callbacks
 * @note Do not modify the returned structure
 */
const ArtNet_State_t* ArtNet_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_H_ */
