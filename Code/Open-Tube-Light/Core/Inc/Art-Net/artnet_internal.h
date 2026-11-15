/**
 * @file artnet_internal.h
 * @brief Art-Net Internal Module Interface
 * 
 * Shared types, state, and function declarations used across Art-Net
 * implementation modules (artnet.c, artnet_dmx.c, artnet_handlers.c,
 * artnet_command.c, artnet_pollreply.c).
 * 
 * @note Only include from artnet_*.c files — use artnet.h for application code.
 */

#ifndef INC_ARTNET_INTERNAL_H_
#define INC_ARTNET_INTERNAL_H_

#include "artnet.h"
#include "artnet_protocol.h"
#include "Device-Config/device_config.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "cmsis_os.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Internal Constants ========================== */

/**
 * @brief Art-Net failsafe timeout reported to dmx_input (ms)
 *
 * Must exceed the 4 s ArtDmx keep-alive of a controller sending static data
 * (Art-Net 4: resend at least every 4 s), with margin for jitter on that
 * keep-alive; Must not be shorter than ARTNET_DISCONNECT_TIMEOUT_MS (5 s), so
 * the source lock is released by the time failsafe kicks in.
 */
#define ARTNET_FAILSAFE_TIMEOUT_MS  6000U

_Static_assert(ARTNET_DISCONNECT_TIMEOUT_MS <= ARTNET_FAILSAFE_TIMEOUT_MS,
               "Source lock must expire no later than failsafe");

/** @brief Max queued ArtPollReply destinations (supports multiple controllers) */
#define ARTNET_MAX_PENDING_REPLIES  4

/** @brief Retry delay when the delayed ArtPollReply can't be handed to tcpip_thread (ms) */
#define ARTNET_POLL_REPLY_RETRY_MS  10U

/** @brief Maximum length of the free-form detail stored for NodeReport */
#define ARTNET_NODE_REPORT_DETAIL_MAX  24

/** @brief Supported first-pass NodeReport codes */
#define ARTNET_NODE_REPORT_RC_POWER_OK    0x0001U
#define ARTNET_NODE_REPORT_RC_PARSE_FAIL  0x0004U
#define ARTNET_NODE_REPORT_RC_SH_NAME_OK  0x0006U
#define ARTNET_NODE_REPORT_RC_LO_NAME_OK  0x0007U
#define ARTNET_NODE_REPORT_RC_CONFIG_ERR  0x000CU

/* ========================== Internal Types ========================== */

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
    uint8_t    last_sequence;   ///< Last ArtDmx sequence number (0 = no sequenced packet yet)
    bool       sync_mode;       ///< True when this universe's controller is sending ArtSync
    uint32_t   last_sync_tick;  ///< Timestamp of last ArtSync from this universe's controller
} ArtNet_UniverseState_t;

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
 * @brief Current NodeReport status presented in ArtPollReply
 */
typedef struct {
    uint16_t code;                                   ///< Table 3 status code
    char detail[ARTNET_NODE_REPORT_DETAIL_MAX + 1U]; ///< Free-form engineering detail
} ArtNet_NodeReportState_t;

/**
 * @brief Art-Net module internal context
 * 
 * Contains all internal state for the Art-Net implementation. Everything
 * except the shadow buffers (shadow_mutex) is only touched in tcpip_thread,
 * or under the LwIP core lock (ArtNet_NodeInit, ArtNet_EnableOutput/DisableOutput).
 */
typedef struct {
    // Per-universe state (source tracking, sync, timing)
    ArtNet_UniverseState_t universes[DEVICE_CONFIG_MAX_UNIVERSES];
    
    // Network layer
    struct udp_pcb *pcb;                    ///< UDP protocol control block
    osThreadId_t processing_task;           ///< Task to notify on new data (set while output_enabled)
    bool output_enabled;                    ///< ArtDmx/ArtSync processed: Art-Net is the DMX input source
    
    // Shadow buffer protection
    osMutexId_t shadow_mutex;               ///< Guards shadow buffer access between tcpip_thread and EffectTask
    
    // ArtPollReply
    osTimerId_t poll_reply_timer;           ///< Timer for delayed reply
    ArtNet_PollReplyQueue_t reply_queue;    ///< Queued reply destinations
    uint16_t poll_reply_counter;            ///< Status report counter
    ArtNet_NodeReportState_t node_report;   ///< Current report code and text
    
    // PRNG state
    uint32_t prng_state;                    ///< XorShift PRNG state
    
    // Initialization flag
    bool initialized;                       ///< Node running (ArtNet_NodeInit succeeded)
} ArtNet_Context_t;

/* ========================== Shared State (artnet.c, buffers in artnet_dmx.c) ========================== */

/** @brief Global Art-Net context */
extern ArtNet_Context_t g_artnet_ctx;

/** @brief Shadow DMX buffers for incoming data */
extern uint8_t g_artnet_shadow_buffer[DEVICE_CONFIG_MAX_UNIVERSES][ARTNET_DMX_MAX_LENGTH];

/** @brief Active DMX buffers for rendering */
extern uint8_t g_artnet_active_buffer[DEVICE_CONFIG_MAX_UNIVERSES][ARTNET_DMX_MAX_LENGTH];

/* ========================== Internal Functions - Data Part (artnet_dmx.c) ========================== */

/**
 * @brief Switch ArtDmx/ArtSync processing on (dmx_input_artnet.init)
 *
 * Clears the DMX buffers and per-universe state, then enables the data part
 * (output ports: NumPorts > 0, ArtDmx/ArtSync accepted).
 *
 * @param processing_task_handle Task notified with DMX_INPUT_FLAG_FRAME_READY
 * @return 0 on success, -1 if the node is not running
 * @note Call from ControlTask (takes LOCK_TCPIP_CORE)
 */
int ArtNet_EnableOutput(osThreadId_t processing_task_handle);

/**
 * @brief Switch ArtDmx/ArtSync processing off (dmx_input_artnet.deinit)
 *
 * The node keeps answering discovery and configuration. The buffers are left
 * untouched, so the strip holds its last frame until the failsafe applies.
 *
 * @note Call from ControlTask (takes LOCK_TCPIP_CORE). EffectTask reads the
 *       active buffer without the mutex, so a runtime source switch still
 *       needs EffectTask quiesced first (artnet-audit-followups.md item 12).
 */
void ArtNet_DisableOutput(void);

/**
 * @brief Get pointer to the active DMX buffer for a specific universe
 *
 * @param universe Universe index (0 to configured universe count - 1)
 * @return Pointer to 512-byte DMX buffer, or NULL if universe index invalid
 *
 * @note Buffer is read-only and stable until the next ArtNet_LatchData() call
 */
const uint8_t* ArtNet_GetUniverseData(uint8_t universe);

/**
 * @brief Latch shadow buffer to active buffer
 *
 * Copies the configured universes' shadow buffers to the active buffers so
 * the rendering task sees a coherent snapshot. Call from the processing task
 * after being notified via thread flags.
 */
void ArtNet_LatchData(void);

/* ========================== Internal Functions - Packet Handlers ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput) packet (artnet_dmx.c)
 */
void ArtNet_HandleArtDmx(const ArtNet_ArtDmx_t *pkt, uint16_t len, const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtPoll packet
 */
void ArtNet_HandleArtPoll(const ArtNet_ArtPoll_t *pkt, uint16_t len,
                         const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtCommand packet (artnet_command.c)
 */
void ArtNet_HandleArtCommand(const ArtNet_ArtCommand_t *pkt, uint16_t len,
                            const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtSync packet (artnet_dmx.c)
 */
void ArtNet_HandleArtSync(const ip_addr_t *src_ip);

/**
 * @brief Handle incoming ArtAddress packet
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len,
                            const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtInput packet (reply with ArtPollReply)
 * @note Caller guarantees len >= sizeof(ArtNet_ArtInput_t)
 */
void ArtNet_HandleArtInput(const ArtNet_ArtInput_t *pkt,
                           const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Handle incoming ArtDataRequest packet (reply with ArtDataReply)
 * @note Caller guarantees len >= ARTNET_DATA_REQUEST_MIN_LENGTH
 */
void ArtNet_HandleArtDataRequest(const ArtNet_ArtDataRequest_t *pkt,
                                 const ip_addr_t *src_ip, u16_t src_port);

/* ========================== Internal Functions - ArtPollReply ========================== */

/**
 * @brief Send ArtPollReply packet
 * @note Must be called from tcpip_thread context
 */
void ArtNet_SendPollReply(const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Send an ArtPollReply now, replacing a queued one for the same controller
 * @note Must be called from tcpip_thread context
 */
void ArtNet_ReplyImmediate(const ip_addr_t *src_ip, u16_t src_port);

/**
 * @brief Set the NodeReport, then send an immediate ArtPollReply
 * @note Must be called from tcpip_thread context
 */
void ArtNet_ReplyImmediateStatus(const ip_addr_t *src_ip, u16_t src_port,
                                 uint16_t code, const char *detail);

/** @brief Timer callback for delayed ArtPollReply */
void ArtNet_PollReplyTimerCallback(void *arg);

/** @brief Send ArtPollReply from tcpip_thread context (via tcpip_callback) */
void ArtNet_PollReplySendCallback(void *arg);

/** @brief Update current NodeReport code and detail text */
void ArtNet_NodeReportSet(uint16_t code, const char *detail);

/* ========================== Internal Functions - Utilities ========================== */

/** @brief Validate Art-Net packet header ("Art-Net\0" signature) */
bool ArtNet_ValidateHeader(const uint8_t *data);

/**
 * @brief Send a unicast reply packet to a controller
 *
 * Always sends to port 0x1936 (spec). If the request came from a different
 * source port, also sends a copy there so tools that cannot bind 0x1936
 * (e.g. the companion app next to lighting software) receive it.
 *
 * @param p        Packet to send; caller keeps ownership and frees it
 * @param addr     Controller IP
 * @param src_port UDP source port of the request being answered
 * @note Must be called from tcpip_thread context
 */
void ArtNet_SendReply(struct pbuf *p, const ip_addr_t *addr, u16_t src_port);

/**
 * @brief Number of Art-Net output ports to report and match
 *
 * The configured universe count while the data part is enabled, else 0
 * (another DMX input drives the strip).
 * @note tcpip_thread context (reads output_enabled)
 */
uint8_t ArtNet_GetPortCount(void);

/** @brief Get next XorShift pseudo-random number */
uint32_t ArtNet_PrngNext(void);

/**
 * @brief Notify the processing task that new data is available
 * 
 * Sets DMX_INPUT_FLAG_FRAME_READY on the processing task.
 */
void ArtNet_TriggerFrameOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_INTERNAL_H_ */