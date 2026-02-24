/**
 * @file artnet.c
 * @brief Art-Net 4 Protocol Handler - Core Module
 * 
 * This module provides the core Art-Net functionality:
 * - Initialization and shutdown
 * - Public API (data access, state queries)
 * - UDP receive callback and packet dispatch
 * - Buffer management (double-buffering)
 * - Shared state for other Art-Net modules
 * 
 * Packet-specific handling is delegated to:
 * - artnet_handlers.c: ArtDmx, ArtPoll, ArtSync, ArtAddress handlers
 * - artnet_pollreply.c: ArtPollReply construction and transmission
 * 
 * @see artnet_internal.h for shared internal types and state
 */

#include "Art-Net/artnet_internal.h"
#include "main.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== Shared State (accessible to other Art-Net modules) ========================== */

/**
 * @brief Global Art-Net context
 * Contains all internal state, accessible to all Art-Net modules
 */
ArtNet_Context_t g_artnet_ctx = {0};

/**
 * @brief Shadow buffers for incoming DMX data
 * 
 * Incoming Art-Net packets write to these buffers. Data remains here until
 * ArtNet_LatchData() is called, which atomically copies to active_buffer.
 * 
 * Memory placement: Use linker section .DMX_Buffers to place in fast RAM.
 * Alignment: 32-byte aligned for optimal memory access patterns.
 */
uint8_t g_artnet_shadow_buffer[DEVICE_CONFIG_MAX_UNIVERSES][ARTNET_DMX_MAX_LENGTH] 
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/**
 * @brief Active buffers for DMX data rendering
 * 
 * Processing task reads from these buffers during rendering. The double-
 * buffering ensures stable data throughout the render cycle.
 */
uint8_t g_artnet_active_buffer[DEVICE_CONFIG_MAX_UNIVERSES][ARTNET_DMX_MAX_LENGTH] 
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/* ========================== Private Function Prototypes ========================== */

static void ArtNet_UdpReceiveCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port);

/* ========================== Public API Implementation ========================== */

int ArtNet_Init(osThreadId_t processing_task_handle)
{
    if (g_artnet_ctx.initialized) {
        return 0;  // Already initialized
    }
    
    // Store processing task handle
    g_artnet_ctx.processing_task = processing_task_handle;
    
    // Initialize state structure
    memset(&g_artnet_ctx.state, 0, sizeof(ArtNet_State_t));
    
    // Calculate expected universes from device config
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    g_artnet_ctx.state.universes_expected = (1 << num_universes) - 1;
    
    // Initialize all universe source IPs to "any" (0.0.0.0)
    for (int i = 0; i < DEVICE_CONFIG_MAX_UNIVERSES; i++) {
        ip_addr_set_zero(&g_artnet_ctx.state.universe_source_ip[i]);
    }
    ip_addr_set_zero(&g_artnet_ctx.state.last_artdmx_ip);

    // Clear DMX buffers to zero (lights off)
    memset(g_artnet_shadow_buffer, 0, sizeof(g_artnet_shadow_buffer));
    memset(g_artnet_active_buffer, 0, sizeof(g_artnet_active_buffer));
    
    // Initialize poll reply state
    memset(&g_artnet_ctx.pending_reply, 0, sizeof(ArtNet_PollReplyRequest_t));
    g_artnet_ctx.poll_reply_counter = 0;
    
    // Seed PRNG with system tick for random delay generation
    g_artnet_ctx.prng_state = osKernelGetTickCount() ^ 0xDEADBEEF;
    
    // Create one-shot timer for ArtPollReply delay
    g_artnet_ctx.poll_reply_timer = osTimerNew(ArtNet_PollReplyTimerCallback, osTimerOnce, NULL, NULL);
    if (g_artnet_ctx.poll_reply_timer == NULL) {
        return -1;
    }
    
    // Create UDP protocol control block
    g_artnet_ctx.pcb = udp_new();
    if (g_artnet_ctx.pcb == NULL) {
        return -1;
    }
    
    // Bind to Art-Net port (6454 / 0x1936)
    err_t err = udp_bind(g_artnet_ctx.pcb, IP_ADDR_ANY, ARTNET_PORT);
    if (err != ERR_OK) {
        udp_remove(g_artnet_ctx.pcb);
        g_artnet_ctx.pcb = NULL;
        return -1;
    }
    
    // Set receive callback (runs in tcpip_thread context)
    udp_recv(g_artnet_ctx.pcb, ArtNet_UdpReceiveCallback, NULL);
    
    g_artnet_ctx.initialized = true;
    return 0;
}

const uint8_t* ArtNet_GetUniverseData(uint8_t universe)
{
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    if (universe >= num_universes) {
        return NULL;
    }
    return g_artnet_active_buffer[universe];
}

bool ArtNet_IsFrameReady(void)
{
    return g_artnet_ctx.frame_ready;
}

void ArtNet_LatchData(void)
{
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    
    // Atomic copy from shadow to active buffers
    memcpy(g_artnet_active_buffer, g_artnet_shadow_buffer, 
           num_universes * ARTNET_DMX_MAX_LENGTH);
    
    // Clear frame ready flag and universe reception bitmask
    g_artnet_ctx.frame_ready = false;
    g_artnet_ctx.state.universes_received = 0;
}

const ArtNet_State_t* ArtNet_GetState(void)
{
    return &g_artnet_ctx.state;
}

/* ========================== Network Layer Implementation ========================== */

/**
 * @brief UDP receive callback for Art-Net packets
 * 
 * Called by LwIP in tcpip_thread context when Art-Net packets arrive.
 * Validates header and dispatches to appropriate packet handler.
 */
static void ArtNet_UdpReceiveCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)pcb;
    
    // Validate packet exists and has minimum header size
    if (p == NULL || p->tot_len < sizeof(ArtNet_Header_t)) {
        if (p) pbuf_free(p);
        return;
    }
    
    // Get packet data
    uint8_t *data = (uint8_t *)p->payload;
    
    // Validate Art-Net header signature
    if (!ArtNet_ValidateHeader(data, p->tot_len)) {
        pbuf_free(p);
        return;
    }
    
    // Extract opcode (little-endian)
    uint16_t opcode = data[8] | (data[9] << 8);
    
    // Dispatch to appropriate handler
    switch (opcode) {
        case ARTNET_OP_DMX:
            if (p->tot_len >= 18) {
                ArtNet_HandleArtDmx((const ArtNet_ArtDmx_t *)data, p->tot_len, addr);
            }
            break;
            
        case ARTNET_OP_SYNC:
            ArtNet_HandleArtSync(addr);
            break;

        case ARTNET_OP_POLL:
            if (p->tot_len >= 14) {
                ArtNet_HandleArtPoll(addr, port);
            }
            break;
            
        default:
            // Ignore unknown opcodes (Art-Net spec allows this)
            break;
    }
    
    pbuf_free(p);
}

/* ========================== Utility Functions ========================== */

bool ArtNet_ValidateHeader(const uint8_t *data, uint16_t len)
{
    if (len < 10) return false;
    
    // Check "Art-Net\0" signature (8 bytes)
    if (memcmp(data, ARTNET_ID, 8) != 0) {
        return false;
    }
    
    return true;
}

uint32_t ArtNet_PrngNext(void)
{
    uint32_t x = g_artnet_ctx.prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_artnet_ctx.prng_state = x;
    return x;
}

void ArtNet_TriggerFrameOutput(void)
{
    g_artnet_ctx.frame_ready = true;
    
    // Notify processing task
    if (g_artnet_ctx.processing_task != NULL) {
        osThreadFlagsSet(g_artnet_ctx.processing_task, ARTNET_THREAD_FLAG_FRAME_READY);
    }
}
