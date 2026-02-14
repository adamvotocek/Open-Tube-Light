/**
 * @file artnet.c
 * @brief Art-Net 4 Protocol Handler Implementation
 * 
 * This implementation provides Art-Net 4 reception with OpSync support for
 * synchronized multi-universe updates. It uses double-buffering (shadow/active)
 * to ensure atomic data access during rendering operations.
 * 
 * @section architecture Architecture
 * 
 * The implementation is organized into logical sections:
 * - Buffer Management: DMX data storage with double-buffering
 * - Network Layer: LwIP UDP integration with proper callback safety
 * - Packet Handlers: Protocol-specific processing for each OpCode
 * - State Management: Tracking sync mode and universe reception
 * - Utilities: Helper functions for validation and PRNG
 * 
 * @section memory Memory Layout
 * 
 * DMX buffers use the .DMX_Buffers linker section for placement in fast RAM.
 * Configure your linker script to place this section in the appropriate memory
 * region for your target (e.g., tightly-coupled memory, SRAM, or cached/non-cached
 * regions depending on your DMA and cache architecture).
 * 
 * Buffers are 32-byte aligned for optimal memory access patterns and cache line
 * alignment on systems with data caching enabled.
 * 
 * @section threading Thread Safety
 * 
 * - UDP callbacks run in LwIP tcpip_thread context
 * - Timer callbacks run in timer daemon context (must use tcpip_callback)
 * - Processing task runs in its own context (notified via osThreadFlagsSet)
 * - Double-buffering ensures atomic access to active data
 * 
 * @section sync OpSync Behavior
 * 
 * When OpSync packets are received, the node enters sync mode and delays
 * output until the sync packet arrives. This ensures tear-free updates
 * across multiple universes. If no OpSync is received for 4 seconds,
 * the node reverts to immediate mode (outputs after each universe).
 */

#include "artnet.h"
#include "main.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== Private Types ========================== */

/**
 * @brief Pending ArtPollReply request
 * 
 * Art-Net spec requires random delay (0-1s) before sending ArtPollReply
 * to prevent network congestion when many nodes respond to a broadcast poll.
 * This structure stores the destination address until the timer fires.
 */
typedef struct {
    ip_addr_t addr;         ///< Destination IP address for reply
    u16_t     port;         ///< Destination UDP port for reply
    bool      pending;      ///< True if a reply is queued
} PollReplyRequest_t;

/* ========================== Private Variables - Buffer Management ========================== */

/**
 * @brief Shadow buffers for incoming DMX data
 * 
 * Incoming Art-Net packets write to these buffers. Data remains here until
 * ArtNet_LatchData() is called, which atomically copies to active_buffer.
 * 
 * Memory placement: Use linker section .DMX_Buffers to place in fast RAM.
 * Alignment: 32-byte aligned for optimal memory access patterns.
 * 
 * @note Configure linker script to place .DMX_Buffers section in appropriate RAM region
 */
static uint8_t shadow_buffer[ARTNET_NUM_UNIVERSES][ARTNET_DMX_MAX_LENGTH] 
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/**
 * @brief Active buffers for DMX data rendering
 * 
 * Processing task reads from these buffers during rendering. The double-
 * buffering ensures stable data throughout the render cycle, preventing
 * tearing artifacts.
 * 
 * Memory placement: Use linker section .DMX_Buffers to place in fast RAM.
 * Alignment: 32-byte aligned for optimal memory access patterns.
 * 
 * @note Configure linker script to place .DMX_Buffers section in appropriate RAM region
 */
static uint8_t active_buffer[ARTNET_NUM_UNIVERSES][ARTNET_DMX_MAX_LENGTH] 
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/* ========================== Private Variables - State Management ========================== */

/**
 * @brief Current Art-Net operational state
 * 
 * Tracks sync mode status, universe reception bitmask, and timeout tracking.
 */
static ArtNet_State_t artnet_state;

/**
 * @brief Frame ready flag
 * 
 * Set when new DMX data is ready to be latched. In non-sync mode, set when
 * all universes received. In sync mode, set when OpSync received.
 * Cleared by ArtNet_LatchData().
 */
static volatile bool frame_ready = false;

/* ========================== Private Variables - Network Layer ========================== */

/**
 * @brief UDP protocol control block for Art-Net socket
 * 
 * Created by udp_new() and bound to ARTNET_PORT. Manages all incoming
 * Art-Net packets via the receive callback.
 */
static struct udp_pcb *artnet_pcb = NULL;

/**
 * @brief Processing task handle for frame notifications
 * 
 * This task is notified via osThreadFlagsSet(task, 0x01) when new DMX
 * data is ready. Typically the LED rendering task.
 */
static osThreadId_t processing_task = NULL;

/* ========================== Private Variables - ArtPollReply ========================== */

/**
 * @brief Timer for delayed ArtPollReply transmission
 * 
 * Art-Net spec requires 0-1 second random delay to prevent network
 * congestion when many nodes respond to broadcast polls.
 */
static osTimerId_t poll_reply_timer = NULL;

/**
 * @brief Pending ArtPollReply request data
 * 
 * Stores destination address/port while waiting for random delay timer.
 */
static PollReplyRequest_t pending_poll_reply = {0};

/**
 * @brief Poll reply counter for status reporting
 * 
 * Increments with each ArtPollReply sent. Reported in node_report field
 * as a diagnostic counter (rolls at 9999).
 */
static uint16_t poll_reply_counter = 0;

/* ========================== Private Variables - Utilities ========================== */

/**
 * @brief Pseudo-random number generator state
 * 
 * Simple XorShift PRNG for generating random delays. Seeded from
 * system tick count during initialization.
 */
static uint32_t prng_state = 0;

/* ========================== Private Function Prototypes ========================== */

// Network callbacks
static void ArtNet_UdpReceiveCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port);

// Packet handlers
static void ArtDmxPacket_Handle(const ArtNet_ArtDmx_t *pkt, uint16_t len);
static void ArtPollPacket_Handle(const ip_addr_t *addr, u16_t port);
static void ArtSyncPacket_Handle(void);

// ArtPollReply transmission
static void ArtPollReply_Send(const ip_addr_t *addr, u16_t port);
static void ArtPollReply_TimerCallback(void *arg);
static void ArtPollReply_SendFromTcpipThread(void *arg);

// Utilities
static bool ArtNetHeader_Validate(const uint8_t *data, uint16_t len);
static uint32_t PRNG_Next(void);
static void FrameOutput_Trigger(void);

/* ========================== Public API Implementation ========================== */

int ArtNet_Init(osThreadId_t processing_task_handle)
{
    processing_task = processing_task_handle;
    
    // Initialize state structure
    memset(&artnet_state, 0, sizeof(artnet_state));
    artnet_state.universes_expected = (1 << ARTNET_NUM_UNIVERSES) - 1;
    
    // Clear DMX buffers to zero (lights off)
    memset(shadow_buffer, 0, sizeof(shadow_buffer));
    memset(active_buffer, 0, sizeof(active_buffer));
    
    // Initialize poll reply state
    memset(&pending_poll_reply, 0, sizeof(pending_poll_reply));
    poll_reply_counter = 0;
    
    // Seed PRNG with system tick for random delay generation
    // XOR with constant to ensure non-zero seed
    prng_state = osKernelGetTickCount() ^ 0xDEADBEEF;
    
    // Create one-shot timer for ArtPollReply delay
    // Timer runs in daemon context, must use tcpip_callback to access LwIP
    poll_reply_timer = osTimerNew(ArtPollReply_TimerCallback, osTimerOnce, NULL, NULL);
    if (poll_reply_timer == NULL) {
        return -1;
    }
    
    // Create UDP protocol control block
    artnet_pcb = udp_new();
    if (artnet_pcb == NULL) {
        return -1;
    }
    
    // Bind to Art-Net port (6454 / 0x1936)
    err_t err = udp_bind(artnet_pcb, IP_ADDR_ANY, ARTNET_PORT);
    if (err != ERR_OK) {
        udp_remove(artnet_pcb);
        artnet_pcb = NULL;
        return -1;
    }
    
    // Set receive callback (runs in tcpip_thread context)
    udp_recv(artnet_pcb, ArtNet_UdpReceiveCallback, NULL);
    
    return 0;
}

const uint8_t* ArtNet_GetUniverseData(uint8_t universe)
{
    if (universe >= ARTNET_NUM_UNIVERSES) {
        return NULL;
    }
    return active_buffer[universe];
}

bool ArtNet_IsFrameReady(void)
{
    return frame_ready;
}

void ArtNet_LatchData(void)
{
    // Atomic copy from shadow to active buffers
    // Processing task reads from active, network writes to shadow
    memcpy(active_buffer, shadow_buffer, sizeof(active_buffer));
    
    // Clear frame ready flag and universe reception bitmask for next frame
    frame_ready = false;
    artnet_state.universes_received = 0;
}

const ArtNet_State_t* ArtNet_GetState(void)
{
    return &artnet_state;
}

/* ========================== Network Layer Implementation ========================== */

/**
 * @brief UDP receive callback for Art-Net packets
 * 
 * Called by LwIP in tcpip_thread context when Art-Net packets arrive.
 * Validates header and dispatches to appropriate packet handler.
 * 
 * @param arg User argument (unused)
 * @param pcb UDP PCB (unused, we use global artnet_pcb)
 * @param p Packet buffer (pbuf chain)
 * @param addr Source IP address
 * @param port Source UDP port
 * 
 * @note Runs in LwIP tcpip_thread context - safe to call LwIP functions
 * @note Must call pbuf_free() before returning
 */
static void ArtNet_UdpReceiveCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port)
{
    (void)arg;  // Avoid compiler unused parameter warning
    (void)pcb;  // We use global artnet_pcb
    
    // Validate packet exists and has minimum header size
    if (p == NULL || p->tot_len < sizeof(ArtNet_Header_t)) {
        if (p) pbuf_free(p);
        return;
    }
    
    // Get packet data (assumes contiguous payload for small packets)
    // PBUF_POOL_BUFSIZE should be large enough to hold an entire ethernet frame.
    // For large fragmented packets, additional handling would be needed, or they should be rejected entirely (IP_REASSEMBLY in lwipopts.h)
    uint8_t *data = (uint8_t *)p->payload;
    
    // Validate Art-Net header signature
    if (!ArtNetHeader_Validate(data, p->tot_len)) {
        pbuf_free(p);
        return;
    }
    
    // Extract opcode (little-endian in packet)
    uint16_t opcode = data[8] | (data[9] << 8);
    
    // Dispatch to appropriate handler
    switch (opcode) {
        case ARTNET_OP_DMX:
            if (p->tot_len >= 18) { // Minimum ArtDmx size (header + length fields)
                ArtDmxPacket_Handle((const ArtNet_ArtDmx_t *)data, p->tot_len);
            }
            break;
            
        case ARTNET_OP_SYNC:
            ArtSyncPacket_Handle();
            break;

        case ARTNET_OP_POLL:
            if (p->tot_len >= 14) { // Minimum ArtPoll size (header only)
                ArtPollPacket_Handle(addr, port);
            }
            break;
            
        default:
            // Ignore unknown opcodes (Art-Net spec allows this)
            break;
    }
    
    pbuf_free(p);
}

/* ========================== Packet Handler Implementation ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput) packet
 * 
 * Validates protocol version and Port-Address, then copies DMX data to
 * shadow buffer. In non-sync mode, triggers output when all universes
 * received. In sync mode, waits for OpSync.
 * 
 * @param pkt Pointer to ArtDmx packet structure
 * @param len Total packet length (for validation)
 * 
 * @note Runs in tcpip_thread context
 * @note Uses Port-Address decoding per Art-Net 4 spec (15-bit addressing)
 */
static void ArtDmxPacket_Handle(const ArtNet_ArtDmx_t *pkt, uint16_t len)
{
    // Protocol version is not validated, for backwards compatibility. The spec requires validation for controllers, not nodes.
    
    // Decode 15-bit Port-Address from packet
    // Port-Address = Net[14:8] : SubNet[7:4] : Universe[3:0]
    uint8_t pkt_net = ARTNET_GET_NET(pkt->net);
    uint8_t pkt_subnet = ARTNET_GET_SUBNET(pkt->sub_uni);
    uint8_t pkt_universe = ARTNET_GET_UNIVERSE(pkt->sub_uni);
    
    // Check if packet is for our configured Net/SubNet
    if (pkt_net != ARTNET_NET || pkt_subnet != ARTNET_SUBNET) {
        return;
    }
    
    // Calculate universe index relative to our start universe
    int universe_idx = pkt_universe - ARTNET_START_UNIVERSE;
    if (universe_idx < 0 || universe_idx >= ARTNET_NUM_UNIVERSES) {
        return;
    }
    
    // Extract DMX data length (big-endian in packet)
    uint16_t dmx_len = (pkt->length_hi << 8) | pkt->length_lo;
    if (dmx_len > ARTNET_DMX_MAX_LENGTH) {
        dmx_len = ARTNET_DMX_MAX_LENGTH;  // Clamp to spec maximum
    }
    
    // Verify packet contains advertised data
    if (len < (18 + dmx_len)) {
        return;
    }
    
    // Copy DMX data to shadow buffer
    // Partial universe updates are allowed per Art-Net spec
    memcpy(shadow_buffer[universe_idx], pkt->data, dmx_len);
    
    // Mark this universe as received in current frame
    artnet_state.universes_received |= (1 << universe_idx);
    
    // Check OpSync timeout - revert to immediate mode if sync lost
    // Art-Net spec: 4 second timeout for OpSync
    if (artnet_state.sync_mode) {
        uint32_t now = osKernelGetTickCount();
        if ((now - artnet_state.last_sync_tick) > ARTNET_SYNC_TIMEOUT_MS) {
            artnet_state.sync_mode = false;
        }
    }
    
    // In non-sync mode: trigger output immediately when all universes received
    // This provides lowest latency for single-universe or non-synced setups
    if (!artnet_state.sync_mode) {
        if (artnet_state.universes_received == artnet_state.universes_expected) {
            FrameOutput_Trigger();
        }
    }
}

/**
 * @brief Handle incoming ArtPoll packet
 * 
 * Schedules an ArtPollReply with random delay (0-1s) per Art-Net spec.
 * The random delay prevents network congestion when many nodes respond
 * to a broadcast poll simultaneously.
 * 
 * @param addr Source IP address (destination for reply)
 * @param port Source UDP port (destination for reply)
 * 
 * @note Runs in tcpip_thread context
 * @note Ignores poll if a reply is already pending
 */
static void ArtPollPacket_Handle(const ip_addr_t *addr, u16_t port)
{
    // Ignore poll if reply already pending
    // Per Art-Net spec, we have up to 1 second to reply
    if (pending_poll_reply.pending) {
        return;
    }
    
    // Store destination address for delayed reply
    ip_addr_copy(pending_poll_reply.addr, *addr);
    pending_poll_reply.port = port;
    pending_poll_reply.pending = true;
    
    // Generate random delay between 0 and 1000ms
    // Art-Net spec requirement to prevent network storms
    uint32_t delay_ms = PRNG_Next() % (ARTNET_POLL_REPLY_DELAY_MAX_MS - 
                                        ARTNET_POLL_REPLY_DELAY_MIN_MS + 1);
    delay_ms += ARTNET_POLL_REPLY_DELAY_MIN_MS;
    
    // Ensure minimum delay of 1ms for timer API
    if (delay_ms == 0) {
        delay_ms = 1;
    }
    
    // Start one-shot timer
    // Timer callback runs in daemon context, not LwIP context
    osTimerStart(poll_reply_timer, delay_ms);
}

/**
 * @brief Handle incoming ArtSync packet
 * 
 * Enables sync mode and triggers immediate frame output. OpSync signals
 * that all ArtDmx packets for this frame have been received and output
 * should occur now for tear-free synchronized updates.
 * 
 * @note Runs in tcpip_thread context
 * @note Updates sync timeout tracking
 */
static void ArtSyncPacket_Handle(void)
{
    // Enter sync mode (if not already in it)
    artnet_state.sync_mode = true;
    
    // Update timeout tracking
    artnet_state.last_sync_tick = osKernelGetTickCount();
    
    // Trigger frame output immediately
    // In sync mode, we output on sync regardless of universe reception
    FrameOutput_Trigger();
}

/* ========================== ArtPollReply Implementation ========================== */

/**
 * @brief Timer callback for delayed ArtPollReply
 * 
 * Called from timer daemon context after random delay expires.
 * Must use tcpip_callback to safely access LwIP functions.
 * 
 * @param arg User argument (unused)
 * 
 * @note Runs in timer daemon context, NOT tcpip_thread context
 */
static void ArtPollReply_TimerCallback(void *arg)
{
    (void)arg;
    
    if (pending_poll_reply.pending) {
        // Schedule send in LwIP's tcpip_thread context
        tcpip_callback(ArtPollReply_SendFromTcpipThread, NULL);
    }
}

/**
 * @brief Send ArtPollReply from tcpip_thread context
 * 
 * Called via tcpip_callback from timer callback. Now safe to call
 * LwIP functions like udp_sendto().
 * 
 * @param arg User argument (unused)
 * 
 * @note Runs in tcpip_thread context
 */
static void ArtPollReply_SendFromTcpipThread(void *arg)
{
    (void)arg;
    
    if (pending_poll_reply.pending) {
        ArtPollReply_Send(&pending_poll_reply.addr, pending_poll_reply.port);
        pending_poll_reply.pending = false;
    }
}

/**
 * @brief Construct and send ArtPollReply packet
 * 
 * Builds complete ArtPollReply with node information, capabilities,
 * status, and port configuration. Sends unicast reply to requester.
 * 
 * @param addr Destination IP address
 * @param port Destination UDP port
 * 
 * @note Runs in tcpip_thread context (safe to call LwIP functions)
 * @note Uses external gnetif and heth for IP/MAC retrieval
 */
static void ArtPollReply_Send(const ip_addr_t *addr, u16_t port)
{
    // Allocate pbuf for ArtPollReply
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ArtNet_ArtPollReply_t), PBUF_RAM);
    if (p == NULL) return;
    
    ArtNet_ArtPollReply_t *reply = (ArtNet_ArtPollReply_t *)p->payload;
    memset(reply, 0, sizeof(ArtNet_ArtPollReply_t));
    
    // Art-Net header
    memcpy(reply->id, ARTNET_ID, 8);
    reply->opcode = ARTNET_OP_POLL_REPLY;
    
    // Node IP address (get from netif)
    extern struct netif gnetif;
    const ip4_addr_t *our_ip = netif_ip4_addr(&gnetif);
    reply->ip[0] = ip4_addr1(our_ip);
    reply->ip[1] = ip4_addr2(our_ip);
    reply->ip[2] = ip4_addr3(our_ip);
    reply->ip[3] = ip4_addr4(our_ip);
    
    // Port number
    reply->port = ARTNET_PORT;
    
    // Firmware version
    reply->vers_hi = ARTNET_FIRMWARE_VER_HI;
    reply->vers_lo = ARTNET_FIRMWARE_VER_LO;
    
    // Port-Address high bits
    reply->net_switch = ARTNET_NET;
    reply->sub_switch = ARTNET_SUBNET;
    
    // OEM code
    reply->oem_hi = ARTNET_OEM_CODE_HI;
    reply->oem_lo = ARTNET_OEM_CODE_LO;
    
    // UBEA version (0 = not supported)
    reply->ubea_version = 0;
    
    // Status flags
    reply->status1 = ARTNET_STATUS1_DEFAULT;
    
    // ESTA manufacturer code
    reply->esta_man_lo = ARTNET_ESTA_MAN_LO;
    reply->esta_man_hi = ARTNET_ESTA_MAN_HI;
    
    // Device names
    strncpy(reply->short_name, ARTNET_SHORT_NAME, 17);
    strncpy(reply->long_name, ARTNET_LONG_NAME, 63);
    
    // Node report: "#xxxx [yyyy] zzzzz..."
    // xxxx = hex status code (0x0001 = RcPowerOk)
    // yyyy = decimal counter (rolls at 9999)
    poll_reply_counter = (poll_reply_counter + 1) % 10000;
    memcpy(reply->node_report, "#0001 [0000] Power On OK", 25);
    reply->node_report[7]  = '0' + (poll_reply_counter / 1000) % 10;
    reply->node_report[8]  = '0' + (poll_reply_counter / 100) % 10;
    reply->node_report[9]  = '0' + (poll_reply_counter / 10) % 10;
    reply->node_report[10] = '0' + poll_reply_counter % 10;
    
    // Number of ports (max 4 per ArtPollReply)
    uint8_t num_ports = (ARTNET_NUM_UNIVERSES > 4) ? 4 : ARTNET_NUM_UNIVERSES;
    reply->num_ports_hi = 0;
    reply->num_ports_lo = num_ports;
    
    // Configure each output port
    for (int i = 0; i < num_ports; i++) {
        // Port type: output capable, DMX512 protocol
        reply->port_types[i] = ARTNET_PORT_TYPE_OUTPUT;
        
        // Port universe address (low 4 bits of Port-Address)
        reply->sw_out[i] = (ARTNET_START_UNIVERSE + i) & 0x0F;
        
        // Output status
        uint8_t good_output = 0x00;
        if (artnet_state.universes_received & (1 << i)) {
            good_output |= 0x80;  // Bit 7: Data is being transmitted
        }
        reply->good_output[i] = good_output;
        
        // Additional output status (Art-Net 4)
        reply->good_output_b[i] = ARTNET_GOOD_OUTPUT_B;
    }
    
    // Node style
    reply->style = ARTNET_NODE_STYLE;
    
    // MAC address (from ETH peripheral)
    extern ETH_HandleTypeDef heth;
    reply->mac[0] = heth.Init.MACAddr[0];
    reply->mac[1] = heth.Init.MACAddr[1];
    reply->mac[2] = heth.Init.MACAddr[2];
    reply->mac[3] = heth.Init.MACAddr[3];
    reply->mac[4] = heth.Init.MACAddr[4];
    reply->mac[5] = heth.Init.MACAddr[5];
    
    // Bind IP (same as node IP for single-node)
    memcpy(reply->bind_ip, reply->ip, 4);
    reply->bind_index = 1;
    
    // Extended status flags
    reply->status2 = ARTNET_STATUS2_DEFAULT;
    reply->status3 = ARTNET_STATUS3_DEFAULT;
    
    // Send unicast reply
    udp_sendto(artnet_pcb, p, addr, port);
    pbuf_free(p);
}

/* ========================== Utility Functions ========================== */

/**
 * @brief Validate Art-Net packet header
 * 
 * Checks for minimum size and "Art-Net\0" signature.
 * 
 * @param data Packet data buffer
 * @param len Packet length
 * @return true if valid Art-Net packet, false otherwise
 */
static bool ArtNetHeader_Validate(const uint8_t *data, uint16_t len)
{
    if (len < 10) return false;  // Minimum header size
    
    // Check "Art-Net\0" signature (8 bytes)
    if (memcmp(data, ARTNET_ID, 8) != 0) {
        return false;
    }
    
    return true;
}

/**
 * @brief Simple XorShift pseudo-random number generator
 * 
 * Used for generating random delays for ArtPollReply. Not cryptographically
 * secure, but sufficient for network delay randomization.
 * 
 * @return Pseudo-random 32-bit value
 * 
 * @note State is maintained in prng_state global variable
 */
static uint32_t PRNG_Next(void)
{
    uint32_t x = prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    prng_state = x;
    return x;
}

/**
 * @brief Trigger frame output to processing task
 * 
 * Sets frame_ready flag and notifies processing task via thread flags.
 * Called when all universes received (non-sync) or OpSync received (sync).
 * 
 * @note Safe to call from tcpip_thread context
 */
static void FrameOutput_Trigger(void)
{
    frame_ready = true;
    
    // Notify processing task
    if (processing_task != NULL) {
        osThreadFlagsSet(processing_task, 0x01);
    }
}

