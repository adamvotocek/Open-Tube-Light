/**
 * @file artnet_pollreply.c
 * @brief Art-Net ArtPollReply Construction and Transmission
 * 
 * This module handles the construction and transmission of ArtPollReply packets.
 * ArtPollReply is the most complex Art-Net packet, containing extensive node
 * information including IP, ports, status flags, and capabilities.
 * 
 * The reply is sent with a random delay (0-1s) per Art-Net spec to prevent
 * network congestion when many nodes respond to broadcast polls.
 * 
 * @section threading Threading
 * 
 * - Timer callback runs in FreeRTOS timer daemon context
 * - Must use tcpip_callback to safely access LwIP from timer
 * - Actual send runs in tcpip_thread context
 */

#include "Art-Net/artnet_internal.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include <string.h>

/* ========================== External References ========================== */

/** @brief Global network interface (from LwIP) */
extern struct netif gnetif;

/* ========================== Private Types ========================== */

/**
 * @brief Bounded text builder used instead of snprintf
 *
 * newlib's snprintf needs ~1 KB of stack and runs in tcpip_thread here; the
 * NodeReport only needs strings and zero-padded hex/decimal numbers.
 * Output is truncated (never overflowed) and always NUL-terminated.
 */
typedef struct {
    char  *buf;   ///< Destination buffer
    size_t size;  ///< Buffer capacity including the terminator
    size_t len;   ///< Characters written so far
} ArtNet_TextWriter_t;

/* ========================== Private Function Prototypes ========================== */

static void ArtNet_FormatNodeReport(char *out, size_t out_size, const DeviceConfig_t *config);
static void ArtNet_TextAppendChar(ArtNet_TextWriter_t *w, char c);
static void ArtNet_TextAppendStr(ArtNet_TextWriter_t *w, const char *s);
static void ArtNet_TextAppendUnsigned(ArtNet_TextWriter_t *w, uint32_t value,
                                      uint32_t base, uint8_t min_digits);

/* ========================== Timer Callback ========================== */

/**
 * @brief Timer callback for delayed ArtPollReply
 * 
 * Called from timer daemon context after random delay expires.
 * Schedules the actual send in tcpip_thread context.
 */
void ArtNet_PollReplyTimerCallback(void *arg)
{
    (void)arg;
    
    if (g_artnet_ctx.reply_queue.count > 0) {
        // Schedule send in LwIP's tcpip_thread context
        tcpip_callback(ArtNet_PollReplySendCallback, NULL);
    }
}

/**
 * @brief Send queued ArtPollReply packets from tcpip_thread context
 * 
 * Called via tcpip_callback from timer callback. Iterates all queued
 * destinations and sends a unicast reply to each, then clears the queue.
 */
void ArtNet_PollReplySendCallback(void *arg)
{
    (void)arg;
    
    ArtNet_PollReplyQueue_t *q = &g_artnet_ctx.reply_queue;
    
    for (uint8_t i = 0; i < q->count; i++) {
        ArtNet_SendPollReply(&q->entries[i].addr, q->entries[i].port);
    }
    q->count = 0;
}

/* ========================== Status Helpers ========================== */

static const char *ArtNet_GetSegmentFormatName(DeviceConfig_SegmentFormat_t segment_format)
{
    switch (segment_format) {
        case SEGMENT_FORMAT_RGBW:
            return "RGBW";
        case SEGMENT_FORMAT_RGB16:
            return "RGB16";
        case SEGMENT_FORMAT_RGB:
        default:
            return "RGB";
    }
}

/**
 * @brief Build status1, currently static
 *
 *       Bit 7-6: Indicator state (11=normal; locate/mute come with ArtAddress AcLed*)
 *       Bit 5-4: Port-Address programming authority (00=unknown until ArtAddress exists)
 *       Bit 3: Reserved (0)
 *       Bit 2: Boot mode (0=normal)
 *       Bit 1: RDM capable (0=no)
 *       Bit 0: UBEA present (0=no)
 */
static uint8_t ArtNet_BuildStatus1(void)
{
    uint8_t status1 = (uint8_t)(0x3U << 6);  // Indicators in normal mode

    return status1;
}

/**
 * @brief Build status2 byte with dynamic DHCP status
 * 
 * Sets DHCP capability and active bits based on LwIP configuration.
 *       Bit 7: RDM via ArtAddress (0=no)
 *       Bit 6: Output style via ArtAddress (0=no)
 *       Bit 5: Squawking (0=no)
 *       Bit 4: sACN switching (0=no)
 *       Bit 3: 15-bit Port-Address support (1=yes)
 *       Bit 2: DHCP capable (dynamic)
 *       Bit 1: DHCP active (dynamic)
 *       Bit 0: Browser config (0=no)
 */
static uint8_t ArtNet_BuildStatus2(void)
{
    // Base status2: 15-bit Port-Address support
    uint8_t status2 = 0x08;
    
#if LWIP_DHCP
    status2 |= (1 << 2);  // DHCP capable
    
    if (dhcp_supplied_address(&gnetif)) {
        status2 |= (1 << 1);  // DHCP active
    }
#endif
    
    return status2;
}

/**
 * @brief Build status3 byte based on configuration
 * 
 * Sets failsafe state bits based on device config.
 *       Bit 7-6: Failsafe state (dynamic)
 *       Bit 5: Failsafe programming support (0=no) // TODO: Implement failsafe programming via ArtAddress
 *       Bit 4: LLRP support (0=no)
 *       Bit 3: Port direction switching (0=no)
 *       Bit 2: RDMnet support (0=no)
 *       Bit 1: Background queue (0=no)
 *       Bit 0: Background discovery via ArtAddress (0=no)
 */
static uint8_t ArtNet_BuildStatus3(void)
{
    const DeviceConfig_t *config = DeviceConfig_Get();
    uint8_t status3 = 0x00;
    
    // Bits 7-6: Failsafe state
    // 00 = Hold last, 01 = Zero, 10 = Full, 11 = Playback scene
    switch (config->output.failsafe) {
        case FAILSAFE_ZERO:
            status3 |= (0x01 << 6);
            break;
        case FAILSAFE_FULL:
            status3 |= (0x02 << 6);
            break;
        case FAILSAFE_HOLD:
        default:
            // 00 = Hold last (default)
            break;
    }
    
    return status3;
}

/* ========================== ArtPollReply Construction ========================== */

/**
 * @brief Construct and send ArtPollReply packet
 * 
 * Builds complete ArtPollReply with node information from device_config.
 * Sends unicast reply to the requesting controller.
 * 
 * @note Must be called from tcpip_thread context
 */
void ArtNet_SendPollReply(const ip_addr_t *addr, u16_t port)
{
    const DeviceConfig_t *config = DeviceConfig_Get();
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    
    // Allocate pbuf for ArtPollReply
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ArtNet_ArtPollReply_t), PBUF_RAM);
    if (p == NULL) {
        return;
    }
    
    ArtNet_ArtPollReply_t *reply = (ArtNet_ArtPollReply_t *)p->payload;
    // Initialize all fields to zero
    memset(reply, 0, sizeof(ArtNet_ArtPollReply_t));
    
    // ===== HEADER =====
    memcpy(reply->id, ARTNET_ID, 8);
    reply->opcode = ARTNET_OP_POLL_REPLY;
    
    // ===== NODE IP ADDRESS =====
    const ip4_addr_t *our_ip = netif_ip4_addr(&gnetif);
    reply->ip[0] = ip4_addr1(our_ip);
    reply->ip[1] = ip4_addr2(our_ip);
    reply->ip[2] = ip4_addr3(our_ip);
    reply->ip[3] = ip4_addr4(our_ip);
    
    // ===== PORT NUMBER =====
    reply->port = ARTNET_PORT;
    
    // ===== FIRMWARE VERSION =====
    reply->vers_hi = DEVICE_CONFIG_FW_VERSION_MAJOR;
    reply->vers_lo = DEVICE_CONFIG_FW_VERSION_MINOR;
    
    // ===== PORT-ADDRESS (high bits) =====
    reply->net_switch = config->dmx.artnet_net & 0x7F;
    reply->sub_switch = config->dmx.artnet_subnet & 0x0F;
    
    // ===== DEVICE NAMES =====
    strncpy(reply->short_name, config->identity.short_name, 17);
    strncpy(reply->long_name, config->identity.long_name, 63);
    
    // ===== OEM & ESTA CODES =====
    reply->oem_hi = (uint8_t)(DEVICE_CONFIG_ARTNET_OEM_CODE >> 8);
    reply->oem_lo = (uint8_t)(DEVICE_CONFIG_ARTNET_OEM_CODE & 0xFFU);
    reply->esta_man_lo = (uint8_t)(DEVICE_CONFIG_ESTA_MAN_CODE & 0xFFU);
    reply->esta_man_hi = (uint8_t)(DEVICE_CONFIG_ESTA_MAN_CODE >> 8);
    
    // ===== NODE REPORT =====
    // Custom format: "#xxxx [yyyy] @aaa $ppp MODE=[mmm] zzzzz..."
    g_artnet_ctx.poll_reply_counter = (g_artnet_ctx.poll_reply_counter + 1) % 10000;
    ArtNet_FormatNodeReport(reply->node_report, sizeof(reply->node_report), config);
    
    // ===== PORTS =====
    uint32_t now = osKernelGetTickCount();
    uint8_t num_ports = num_universes;
    reply->num_ports_hi = 0;  // Reserved
    reply->num_ports_lo = num_ports;
    
    for (int i = 0; i < num_ports; i++) {
        // Port type: DMX512 output
        reply->port_types[i] = 0x80;  // Bit 7: output capable
        
        // SwOut: Universe address (low 4 bits)
        reply->sw_out[i] = (config->dmx.artnet_start_universe + i) & 0x0F;
        
        // GoodOutputA bit 7: data is being output on this port.
        // source_ip alone is not enough: it is only released when the next
        // ArtDmx arrives, so it stays set forever after a controller stops.
        // Require a recent ArtDmx as well. 
        const ArtNet_UniverseState_t *uni = &g_artnet_ctx.universes[i];
        uint8_t good_output = 0x00;
        if (!ip_addr_isany(&uni->source_ip) &&
            (now - uni->last_dmx_tick) <= ARTNET_DISCONNECT_TIMEOUT_MS) {
            good_output |= 0x80;
        }
        reply->good_output_a[i] = good_output;
        
        // GoodOutputB: RDM disabled, continuous output
        reply->good_output_b[i] = 0xF0;
    }
    
    // ===== NODE STYLE =====
    reply->style = 0x00;  // StNode
    
    // ===== MAC ADDRESS =====
    memcpy(reply->mac, gnetif.hwaddr, 6);
    
    // ===== BIND IP =====
    memcpy(reply->bind_ip, reply->ip, 4);
    reply->bind_index = 1;  // Root device
    
    // ===== STATUS =====
    reply->status1 = ArtNet_BuildStatus1();
    reply->status2 = ArtNet_BuildStatus2();
    reply->status3 = ArtNet_BuildStatus3();
    
    // ===== REFRESH RATE =====
    reply->refresh_rate_hi = 0;
    reply->refresh_rate_lo = config->dmx.artnet_max_refresh_rate;
    
    // Send reply to the requesting controller (0x1936, plus its source port if not 0x1936)
    ArtNet_SendReply(p, addr, port);
    pbuf_free(p);
}

/* ========================== Private Functions ========================== */

/**
 * @brief Build the custom NodeReport string
 *
 * Produces exactly what snprintf("#%04x [%04u] @%03u $%03u MODE=[%s] %s") did,
 * since the companion app parses it (docs/artnet-custom-protocol.md).
 */
static void ArtNet_FormatNodeReport(char *out, size_t out_size, const DeviceConfig_t *config)
{
    if ((out == NULL) || (out_size == 0U)) {
        return;
    }

    ArtNet_TextWriter_t w = { .buf = out, .size = out_size, .len = 0U };
    out[0] = '\0';

    ArtNet_TextAppendChar(&w, '#');
    ArtNet_TextAppendUnsigned(&w, g_artnet_ctx.node_report.code, 16U, 4U);
    ArtNet_TextAppendStr(&w, " [");
    ArtNet_TextAppendUnsigned(&w, g_artnet_ctx.poll_reply_counter, 10U, 4U);
    ArtNet_TextAppendStr(&w, "] @");
    ArtNet_TextAppendUnsigned(&w, config->dmx.dmx_start_address, 10U, 3U);
    ArtNet_TextAppendStr(&w, " $");
    ArtNet_TextAppendUnsigned(&w, config->layout.segment_count, 10U, 3U);
    ArtNet_TextAppendStr(&w, " MODE=[");
    ArtNet_TextAppendStr(&w, ArtNet_GetSegmentFormatName(config->layout.segment_format));
    ArtNet_TextAppendStr(&w, "] ");
    ArtNet_TextAppendStr(&w, g_artnet_ctx.node_report.detail);
}

/**
 * @brief Append one character if room remains, keeping the buffer terminated
 */
static void ArtNet_TextAppendChar(ArtNet_TextWriter_t *w, char c)
{
    if ((w->len + 1U) < w->size) {
        w->buf[w->len] = c;
        w->len++;
        w->buf[w->len] = '\0';
    }
}

/**
 * @brief Append a NUL-terminated string (NULL appends nothing)
 */
static void ArtNet_TextAppendStr(ArtNet_TextWriter_t *w, const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
        ArtNet_TextAppendChar(w, *s);
        s++;
    }
}

/**
 * @brief Append an unsigned number, zero-padded to at least min_digits
 *
 * Like printf's %0Nu / %0Nx: wider values are printed in full, not cut.
 *
 * @param base 10 or 16 (lowercase hex digits)
 */
static void ArtNet_TextAppendUnsigned(ArtNet_TextWriter_t *w, uint32_t value,
                                      uint32_t base, uint8_t min_digits)
{
    static const char digit_chars[] = "0123456789abcdef";
    char digits[10];  // uint32_t needs at most 10 decimal digits
    size_t count = 0U;

    if ((base < 2U) || (base > 16U)) {
        return;
    }

    do {
        digits[count] = digit_chars[value % base];
        count++;
        value /= base;
    } while ((value != 0U) && (count < sizeof(digits)));

    while ((count < min_digits) && (count < sizeof(digits))) {
        digits[count] = '0';
        count++;
    }

    while (count > 0U) {
        count--;
        ArtNet_TextAppendChar(w, digits[count]);
    }
}
