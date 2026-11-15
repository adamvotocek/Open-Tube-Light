/**
 * @file artnet.c
 * @brief Art-Net 4 Protocol Handler - Core Module
 * 
 * The always-on Art-Net node: started once at boot by ArtNet_NodeInit() and
 * never torn down. UDP socket, receive callback and packet dispatch,
 * NodeReport, reply sending. Discovery and configuration answer in every DMX
 * input mode; ArtDmx/ArtSync are only processed while the data part
 * (artnet_dmx.c) is the active dmx_input driver.
 * 
 * Packet-specific handling is delegated to:
 * - artnet_dmx.c: data part (ArtDmx, ArtSync, DMX buffers, latch,
 *   dmx_input driver vtable)
 * - artnet_handlers.c: ArtPoll, ArtAddress, ArtInput, ArtDataRequest
 * - artnet_command.c: ArtCommand (custom OpenTubeDia verbs)
 * - artnet_pollreply.c: ArtPollReply construction, transmission and
 *   immediate replies
 *
 * Opcodes without a handler are dropped silently (artnet_ignored_count).
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
 * @brief Packets dropped without action (Live Watch)
 *
 * Counts UDP datagrams on port 0x1936 that are too short to be Art-Net,
 * arrive as a chained pbuf, or carry an opcode this node does not handle
 * (other nodes' ArtPollReply, ArtTimeCode, RDM, ...), plus ArtDmx/ArtSync while
 * Art-Net is not the DMX input source. The spec requires these to be ignored,
 * so they do not touch the NodeReport.
 */
uint32_t artnet_ignored_count = 0;

/* ========================== Private Function Prototypes ========================== */

static void ArtNet_UdpReceiveCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port);
static const char *ArtNet_NodeReportDefaultDetail(uint16_t code);
static uint32_t ArtNet_PrngSeed(void);

/* ========================== Node Start ========================== */

int ArtNet_NodeInit(void)
{
    if (g_artnet_ctx.initialized) {
        return 0;
    }

    // g_artnet_ctx is zero-initialized: the data part is off (output_enabled =
    // false), no universe has a source and the reply queue is empty.
    // ArtNet_EnableOutput() switches ArtDmx/ArtSync on when Art-Net is the input.
    ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_POWER_OK, NULL);

    // Per-unit seed, so fixtures powered up together pick different
    // ArtPollReply delays (the delay exists to spread their replies)
    g_artnet_ctx.prng_state = ArtNet_PrngSeed();

    // Shadow buffer lock between tcpip_thread (ArtDmx) and EffectTask (latch).
    // Created once and never deleted, so a latch can't end up waiting on a
    // deleted mutex when the input source changes. Priority inheritance
    // prevents inversion if task priorities diverge.
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
        osMutexDelete(g_artnet_ctx.shadow_mutex);
        g_artnet_ctx.shadow_mutex = NULL;
        return -1;
    }

    // Create the UDP PCB and bind to the Art-Net port. Raw LwIP API from a
    // task: hold the core lock so tcpip_thread is not inside LwIP meanwhile.
    LOCK_TCPIP_CORE();
    g_artnet_ctx.pcb = udp_new();
    err_t err = (g_artnet_ctx.pcb != NULL)
              ? udp_bind(g_artnet_ctx.pcb, IP_ADDR_ANY, ARTNET_PORT)
              : ERR_MEM;
    if (err == ERR_OK) {
        // Receive callback runs in tcpip_thread context
        udp_recv(g_artnet_ctx.pcb, ArtNet_UdpReceiveCallback, NULL);
    } else if (g_artnet_ctx.pcb != NULL) {
        udp_remove(g_artnet_ctx.pcb);
        g_artnet_ctx.pcb = NULL;
    }
    UNLOCK_TCPIP_CORE();

    if (err != ERR_OK) {
        osTimerDelete(g_artnet_ctx.poll_reply_timer);
        g_artnet_ctx.poll_reply_timer = NULL;
        osMutexDelete(g_artnet_ctx.shadow_mutex);
        g_artnet_ctx.shadow_mutex = NULL;
        return -1;
    }

    g_artnet_ctx.initialized = true;
    return 0;
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
    if (p == NULL) {
        return;
    }

    // Too short to be Art-Net at all: not ours, ignore silently (spec: "Any
    // other packets are ignored"). Handlers read p->payload as one contiguous
    // packet, so also drop chained pbufs. Every Art-Net packet fits one
    // ETH_RX_BUFFER_SIZE (1536 B) buffer and IP_REASSEMBLY is 0, so a chain
    // never carries a valid packet today.
    if (p->tot_len < sizeof(ArtNet_Header_t) || p->len != p->tot_len) {
        artnet_ignored_count++;
        pbuf_free(p);
        return;
    }
    
    // Get packet data
    uint8_t *data = (uint8_t *)p->payload;
    
    // Validate Art-Net header signature
    if (!ArtNet_ValidateHeader(data)) {
        artnet_ignored_count++;
        pbuf_free(p);
        return;
    }
    
    // Extract opcode (little-endian)
    uint16_t opcode = data[8] | (data[9] << 8);
    
    // Dispatch to appropriate handler
    switch (opcode) {
        case ARTNET_OP_DMX:
            if (!g_artnet_ctx.output_enabled) {
                // Art-Net is not the DMX input source
                artnet_ignored_count++;
            } else if (p->tot_len >= ARTNET_DMX_HEADER_LENGTH) {
                ArtNet_HandleArtDmx((const ArtNet_ArtDmx_t *)data, p->tot_len, addr);
            } else {
                ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                     "ArtDmx Too Short");
            }
            break;

        case ARTNET_OP_COMMAND:
            ArtNet_HandleArtCommand((const ArtNet_ArtCommand_t *)data, p->tot_len,
                                    addr, port);
            break;
            
        case ARTNET_OP_SYNC:
            if (!g_artnet_ctx.output_enabled) {
                artnet_ignored_count++;  // Same as ArtDmx above
            } else if (p->tot_len >= sizeof(ArtNet_ArtSync_t)) {
                ArtNet_HandleArtSync(addr);
            } else {
                ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                     "ArtSync Too Short");
            }
            break;

        case ARTNET_OP_POLL:
            if (p->tot_len >= ARTNET_POLL_MIN_LENGTH) {
                ArtNet_HandleArtPoll((const ArtNet_ArtPoll_t *)data, p->tot_len,
                                    addr, port);
            } else {
                ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                     "ArtPoll Too Short");
            }
            break;
            
        case ARTNET_OP_ADDRESS:
            ArtNet_HandleArtAddress((const ArtNet_ArtAddress_t *)data, p->tot_len,
                                    addr, port);
            break;
            
        case ARTNET_OP_INPUT:
            if (p->tot_len >= sizeof(ArtNet_ArtInput_t)) {
                ArtNet_HandleArtInput((const ArtNet_ArtInput_t *)data, addr, port);
            } else {
                ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                     "ArtInput Too Short");
            }
            break;

        case ARTNET_OP_DATA_REQUEST:
            if (p->tot_len >= ARTNET_DATA_REQUEST_MIN_LENGTH) {
                ArtNet_HandleArtDataRequest((const ArtNet_ArtDataRequest_t *)data,
                                            addr, port);
            } else {
                ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                     "ArtDataReq Too Short");
            }
            break;

        default:
            // Valid Art-Net we take no action on (OpPollReply from other
            // nodes, OpTimeCode, RDM, ...). Do not overwrite the NodeReport:
            // that would hide real status behind normal network traffic.
            artnet_ignored_count++;
            break;
    }
    
    pbuf_free(p);
}

/* ========================== Utility Functions ========================== */

static const char *ArtNet_NodeReportDefaultDetail(uint16_t code)
{
    switch (code) {
        case ARTNET_NODE_REPORT_RC_POWER_OK:
            return "Power On OK";
        case ARTNET_NODE_REPORT_RC_PARSE_FAIL:
            return "Parse Failure";
        case ARTNET_NODE_REPORT_RC_SH_NAME_OK:
            return "Short Name OK";
        case ARTNET_NODE_REPORT_RC_LO_NAME_OK:
            return "Long Name OK";
        case ARTNET_NODE_REPORT_RC_CONFIG_ERR:
            return "Config Error";
        default:
            return "";
    }
}

void ArtNet_NodeReportSet(uint16_t code, const char *detail)
{
    const char *text = detail;

    if (text == NULL || text[0] == '\0') {
        text = ArtNet_NodeReportDefaultDetail(code);
    }

    g_artnet_ctx.node_report.code = code;
    memset(g_artnet_ctx.node_report.detail, 0, sizeof(g_artnet_ctx.node_report.detail));
    strncpy(g_artnet_ctx.node_report.detail, text,
            sizeof(g_artnet_ctx.node_report.detail) - 1U);
}

bool ArtNet_ValidateHeader(const uint8_t *data)
{
    if (data == NULL) {
        return false;
    }

    return (memcmp(data, ARTNET_ID, 8) == 0);
}

void ArtNet_SendReply(struct pbuf *p, const ip_addr_t *addr, u16_t src_port)
{
    if (p == NULL || addr == NULL || g_artnet_ctx.pcb == NULL) {
        return;
    }

    // Spec: Art-Net uses 0x1936 as source and destination port, and every
    // controller listens there.
    // Tools that share the PC with lighting software (OpenTubeDia) cannot bind
    // 0x1936 and send from an ephemeral port: give them a copy there too.
    // The copy needs its own pbuf, cloned BEFORE the first send: udp_sendto()
    // prepends the UDP/IP/Ethernet headers in place and never strips them, so
    // re-sending `p` would put the stale headers into the second datagram's
    // payload (and the zero-copy ETH driver still holds `p` for TX DMA).
    // If the LwIP heap is short, only the spec reply goes out.
    struct pbuf *copy = NULL;
    if (src_port != ARTNET_PORT) {
        copy = pbuf_clone(PBUF_TRANSPORT, PBUF_RAM, p);
    }

    udp_sendto(g_artnet_ctx.pcb, p, addr, ARTNET_PORT);

    if (copy != NULL) {
        udp_sendto(g_artnet_ctx.pcb, copy, addr, src_port);
        pbuf_free(copy);
    }
}

uint8_t ArtNet_GetPortCount(void)
{
    // No Art-Net ports while another source drives the strip
    return g_artnet_ctx.output_enabled ? DeviceConfig_GetUniverseCount() : 0U;
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

/* ========================== Private Functions ========================== */

/**
 * @brief Build a non-zero XorShift seed that differs between units
 *
 * The boot tick is nearly identical on every fixture, so mix in the
 * UID-derived MAC (bytes 2..5 carry hash bits, byte 0 is fixed).
 */
static uint32_t ArtNet_PrngSeed(void)
{
    uint8_t mac[DEVICE_CONFIG_MAC_LEN];
    uint32_t seed = osKernelGetTickCount() ^ 0xDEADBEEFU;

    if (DeviceConfig_GetMacAddress(mac) == 0) {
        seed ^= ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
    }

    return (seed != 0U) ? seed : 1U;  // XorShift state 0 never leaves 0
}
