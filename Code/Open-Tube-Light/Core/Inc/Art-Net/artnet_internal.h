/**
 * @file artnet_internal.h
 * @brief Art-Net Internal Module Interface
 * 
 * This header provides internal types, state, and function declarations
 * shared between Art-Net implementation modules. Not part of the public API.
 * 
 * @note This file should only be included by artnet_*.c implementation files
 * @note Do not include this in application code - use artnet.h instead
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

/* ========================== Internal Types ========================== */

/**
 * @brief Pending ArtPollReply request
 * 
 * Art-Net spec requires random delay (0-1s) before sending ArtPollReply
 * to prevent network congestion when many nodes respond to a broadcast poll.
 */
typedef struct {
    ip_addr_t addr;         ///< Destination IP address for reply
    u16_t     port;         ///< Destination UDP port for reply
    bool      is_pending;   ///< True if a reply is queued
} ArtNet_PollReplyRequest_t;

/**
 * @brief Art-Net module internal context
 * 
 * Contains all internal state for the Art-Net implementation.
 * This structure consolidates previously scattered global variables.
 */
typedef struct {
    // State management
    ArtNet_State_t state;                   ///< Current operational state
    volatile bool frame_ready;              ///< Frame ready for latching
    
    // Network layer
    struct udp_pcb *pcb;                    ///< UDP protocol control block
    osThreadId_t processing_task;           ///< Task to notify on new data
    
    // ArtPollReply
    osTimerId_t poll_reply_timer;           ///< Timer for delayed reply
    ArtNet_PollReplyRequest_t pending_reply;///< Pending reply destination
    uint16_t poll_reply_counter;            ///< Status report counter
    
    // PRNG state
    uint32_t prng_state;                    ///< XorShift PRNG state
    
    // Initialization flag
    bool initialized;                       ///< Module initialized
} ArtNet_Context_t;

/* ========================== Shared State (defined in artnet.c) ========================== */

/**
 * @brief Global Art-Net context
 * Defined in artnet.c, accessible to all Art-Net modules
 */
extern ArtNet_Context_t g_artnet_ctx;

/**
 * @brief Shadow DMX buffers for incoming data
 * Defined in artnet.c, accessible to packet handlers
 */
extern uint8_t g_artnet_shadow_buffer[][ARTNET_DMX_MAX_LENGTH];

/**
 * @brief Active DMX buffers for rendering
 * Defined in artnet.c, accessible to public API
 */
extern uint8_t g_artnet_active_buffer[][ARTNET_DMX_MAX_LENGTH];

/* ========================== Internal Functions - Packet Handlers ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput) packet
 * 
 * @param pkt Pointer to ArtDmx packet structure
 * @param len Total packet length
 * @param src_ip Source IP address
 */
void ArtNet_HandleArtDmx(const ArtNet_ArtDmx_t *pkt, uint16_t len, const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtPoll packet
 * 
 * @param addr Source IP address (destination for reply)
 * @param port Source UDP port (destination for reply)
 */
void ArtNet_HandleArtPoll(const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtSync packet
 * 
 * @param src_ip Source IP address
 */
void ArtNet_HandleArtSync(const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtAddress packet
 * 
 * @param pkt Pointer to ArtAddress packet structure
 * @param len Total packet length
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len);

/* ========================== Internal Functions - ArtPollReply ========================== */

/**
 * @brief Send ArtPollReply packet
 * 
 * @param addr Destination IP address
 * @param port Destination UDP port
 * 
 * @note Must be called from tcpip_thread context
 */
void ArtNet_SendPollReply(const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Timer callback for delayed ArtPollReply
 * 
 * @param arg Timer callback argument (unused)
 */
void ArtNet_PollReplyTimerCallback(void *arg);

/**
 * @brief Send ArtPollReply from tcpip_thread context
 * 
 * @param arg Callback argument (unused)
 */
void ArtNet_PollReplySendCallback(void *arg);

/* ========================== Internal Functions - Utilities ========================== */

/**
 * @brief Validate Art-Net packet header
 * 
 * @param data Packet data buffer
 * @param len Packet length
 * @return true if valid Art-Net header, false otherwise
 */
bool ArtNet_ValidateHeader(const uint8_t *data, uint16_t len);

/**
 * @brief Get next pseudo-random number
 * 
 * @return Pseudo-random 32-bit value
 */
uint32_t ArtNet_PrngNext(void);

/**
 * @brief Trigger frame output to processing task
 * 
 * Sets frame_ready flag and notifies processing task.
 */
void ArtNet_TriggerFrameOutput(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_INTERNAL_H_ */
