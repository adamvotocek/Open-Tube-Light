/**
 * @file artnet.c
 * @brief Art-Net 4 Protocol Handler - Core Module
 * 
 * Core Art-Net functionality: initialization, public API (data access),
 * UDP receive callback, packet dispatch, and buffer management.
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

/* ========================== Shared State ========================== */

ArtNet_Context_t g_artnet_ctx = {0};

/**
 * @brief Shadow buffers for incoming DMX data
 * 
 * Incoming ArtDmx packets write here. Data moves to active_buffer
 * when ArtNet_LatchData() is called by the processing task.
 * Placed in DTCM (.DMX_Buffers) for fast CPU access.
 */
uint8_t g_artnet_shadow_buffer[DEVICE_CONFIG_MAX_UNIVERSES][ARTNET_DMX_MAX_LENGTH] 
    __attribute__((section(".DMX_Buffers"), aligned(32)));

/**
 * @brief Active buffers read by the processing task during rendering
 * 
 * Double-buffering ensures stable data throughout the render cycle.
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
        return 0;
    }
    
    g_artnet_ctx.processing_task = processing_task_handle;
    
    // Zero-init clears all per-universe state (source_ip = 0.0.0.0,
    // sync_mode = false, ticks = 0) and DMX buffers (lights off).
    memset(&g_artnet_ctx.universes, 0, sizeof(g_artnet_ctx.universes));
    memset(g_artnet_shadow_buffer, 0, sizeof(g_artnet_shadow_buffer));
    memset(g_artnet_active_buffer, 0, sizeof(g_artnet_active_buffer));
    
    // ArtPollReply state
    memset(&g_artnet_ctx.reply_queue, 0, sizeof(ArtNet_PollReplyQueue_t));
    g_artnet_ctx.poll_reply_counter = 0;
    
    // Seed PRNG with system tick for random delay generation
    g_artnet_ctx.prng_state = osKernelGetTickCount() ^ 0xDEADBEEF;
    
    // Create mutex for shadow buffer access
    // Priority inheritance prevents inversion if task priorities diverge
    const osMutexAttr_t shadow_mutex_attr = {
        .name = "shadowMtx",
        .attr_bits = osMutexPrioInherit,
    };
    g_artnet_ctx.shadow_mutex = osMutexNew(&shadow_mutex_attr);
    if (g_artnet_ctx.shadow_mutex == NULL) {
        return -1;
    }
    
    // Create one-shot timer for ArtPollReply delay
    g_artnet_ctx.poll_reply_timer = osTimerNew(ArtNet_PollReplyTimerCallback, osTimerOnce, NULL, NULL);
    if (g_artnet_ctx.poll_reply_timer == NULL) {
        return -1;
    }
    
    // Create UDP protocol control block and bind to Art-Net port
    g_artnet_ctx.pcb = udp_new();
    if (g_artnet_ctx.pcb == NULL) {
        return -1;
    }
    
    err_t err = udp_bind(g_artnet_ctx.pcb, IP_ADDR_ANY, ARTNET_PORT);
    if (err != ERR_OK) {
        udp_remove(g_artnet_ctx.pcb);
        g_artnet_ctx.pcb = NULL;
        return -1;
    }
    
    // Receive callback runs in tcpip_thread context
    udp_recv(g_artnet_ctx.pcb, ArtNet_UdpReceiveCallback, NULL);
    
    g_artnet_ctx.initialized = true;
    return 0;
}

const uint8_t* ArtNet_GetUniverseData(uint8_t universe)
{
    if (universe >= DeviceConfig_GetUniverseCount()) {
        return NULL;
    }
    return g_artnet_active_buffer[universe];
}

void ArtNet_LatchData(void)
{
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    
    osMutexAcquire(g_artnet_ctx.shadow_mutex, osWaitForever);
    memcpy(g_artnet_active_buffer, g_artnet_shadow_buffer, 
           num_universes * ARTNET_DMX_MAX_LENGTH);
    osMutexRelease(g_artnet_ctx.shadow_mutex);
}

/* ========================== Network Layer Implementation ========================== */

/**
 * @brief UDP receive callback for Art-Net packets
 * 
 * Called by LwIP in tcpip_thread context. Validates header, extracts
 * opcode, and dispatches to the appropriate packet handler.
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
                ArtNet_HandleArtPoll((const ArtNet_ArtPoll_t *)data, p->tot_len,
                                    addr, port);
            }
            break;
            
        case ARTNET_OP_ADDRESS:
            ArtNet_HandleArtAddress((const ArtNet_ArtAddress_t *)data, p->tot_len);
            break;
            
        default:
            break;
    }
    
    pbuf_free(p);
}

/* ========================== Utility Functions ========================== */

bool ArtNet_ValidateHeader(const uint8_t *data, uint16_t len)
{
    if (len < 10) return false;
    return (memcmp(data, ARTNET_ID, 8) == 0);
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
	// Notify processing task
    if (g_artnet_ctx.processing_task != NULL) {
        osThreadFlagsSet(g_artnet_ctx.processing_task, ARTNET_THREAD_FLAG_FRAME_READY);
    }
}