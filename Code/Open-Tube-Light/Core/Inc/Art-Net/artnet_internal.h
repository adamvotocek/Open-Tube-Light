/**
 * @file artnet_internal.h
 * @brief Art-Net Internal Module Interface
 * 
 * Shared types, state, and function declarations used across Art-Net
 * implementation modules (artnet.c, artnet_handlers.c, artnet_pollreply.c).
 * 
 * @note Only include from artnet_*.c files — use artnet.h for application code.
 */

#ifndef INC_ARTNET_INTERNAL_H_
#define INC_ARTNET_INTERNAL_H_

#include "artnet.h"
#include "artnet_protocol.h"
#include "device_config.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "cmsis_os.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Internal Constants ========================== */

/** @brief Thread notification flag for new DMX frame ready */
#define ARTNET_THREAD_FLAG_FRAME_READY  0x01

/** @brief Max queued ArtPollReply destinations (supports multiple controllers) */
#define ARTNET_MAX_PENDING_REPLIES  4

/* ========================== Internal Types ========================== */

/**
 * @brief Single ArtPollReply destination
 */
typedef struct {
    ip_addr_t addr;         ///< Destination IP address for reply
    u16_t     port;         ///< Destination UDP port for reply
} ArtNet_PollReplyDest_t;

/**
 * @brief Queue of pending ArtPollReply destinations
 * 
 * Art-Net spec requires random delay (0-1s) before sending ArtPollReply.
 * Multiple controllers may poll during that window; this queue ensures
 * each receives a unicast reply. Entries are deduplicated by IP.
 */
typedef struct {
    ArtNet_PollReplyDest_t entries[ARTNET_MAX_PENDING_REPLIES];
    uint8_t count;          ///< Number of valid entries (0 = no reply pending)
} ArtNet_PollReplyQueue_t;

/**
 * @brief Art-Net module internal context
 * 
 * Contains all internal state for the Art-Net implementation.
 * This structure consolidates previously scattered global variables.
 */
typedef struct {
    // Per-universe state (source tracking, sync, timing)
    ArtNet_UniverseState_t universes[DEVICE_CONFIG_MAX_UNIVERSES];
    
    // Network layer
    struct udp_pcb *pcb;                    ///< UDP protocol control block
    osThreadId_t processing_task;           ///< Task to notify on new data
    
    // Shadow buffer protection
    osMutexId_t shadow_mutex;               ///< Guards shadow buffer access between tcpip_thread and EffectTask
    
    // ArtPollReply
    osTimerId_t poll_reply_timer;           ///< Timer for delayed reply
    ArtNet_PollReplyQueue_t reply_queue;    ///< Queued reply destinations
    uint16_t poll_reply_counter;            ///< Status report counter
    
    // PRNG state
    uint32_t prng_state;                    ///< XorShift PRNG state
    
    // Initialization flag
    bool initialized;                       ///< Module initialized
} ArtNet_Context_t;

/* ========================== Shared State (defined in artnet.c) ========================== */

/** @brief Global Art-Net context */
extern ArtNet_Context_t g_artnet_ctx;

/** @brief Shadow DMX buffers for incoming data */
extern uint8_t g_artnet_shadow_buffer[][ARTNET_DMX_MAX_LENGTH];

/** @brief Active DMX buffers for rendering */
extern uint8_t g_artnet_active_buffer[][ARTNET_DMX_MAX_LENGTH];

/* ========================== Internal Functions - Packet Handlers ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput) packet
 */
void ArtNet_HandleArtDmx(const ArtNet_ArtDmx_t *pkt, uint16_t len, const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtPoll packet
 */
void ArtNet_HandleArtPoll(const ArtNet_ArtPoll_t *pkt, uint16_t len,
                         const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtSync packet
 */
void ArtNet_HandleArtSync(const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtAddress packet
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len);

/* ========================== Internal Functions - ArtPollReply ========================== */

/**
 * @brief Send ArtPollReply packet
 * @note Must be called from tcpip_thread context
 */
void ArtNet_SendPollReply(const ip_addr_t *src_ip, u16_t src_port);

/** @brief Timer callback for delayed ArtPollReply */
void ArtNet_PollReplyTimerCallback(void *arg);

/** @brief Send ArtPollReply from tcpip_thread context (via tcpip_callback) */
void ArtNet_PollReplySendCallback(void *arg);

/* ========================== Internal Functions - Utilities ========================== */

/** @brief Validate Art-Net packet header ("Art-Net\0" signature) */
bool ArtNet_ValidateHeader(const uint8_t *data, uint16_t len);

/** @brief Get next XorShift pseudo-random number */
uint32_t ArtNet_PrngNext(void);

/**
 * @brief Notify the processing task that new data is available
 * 
 * Sets the ARTNET_THREAD_FLAG_FRAME_READY thread flag on the processing task.
 */
void ArtNet_TriggerFrameOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_INTERNAL_H_ */