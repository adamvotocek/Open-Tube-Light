/**
 * @file artnet_handlers.c
 * @brief Art-Net Packet Handlers
 * 
 * Handlers for incoming Art-Net control packets:
 * - ArtPoll: Discovery request handling
 * - ArtAddress: stub, replies RcConfigErr (plans/config-commits-and-artaddress.md)
 * - ArtInput: Replies with ArtPollReply (node has no inputs)
 * - ArtDataRequest: Product / user guide / support URLs via ArtDataReply
 *
 * ArtDmx/ArtSync (data part) are in artnet_dmx.c, ArtCommand (custom
 * OpenTubeDia verbs) is in artnet_command.c.
 *
 * All handlers run in LwIP tcpip_thread context.
 */

#include "Art-Net/artnet_internal.h"
#include "cmsis_os.h"
#include <stddef.h>
#include <string.h>

/* ========================== Private Functions ========================== */

static void ArtNet_SendDataReply(uint16_t request, const char *text,
                                 const ip_addr_t *src_ip, u16_t src_port);

/* ========================== ArtPoll Handler ========================== */

/**
 * @brief Handle incoming ArtPoll packet
 * 
 * Parses Flags field for targeted mode, then schedules an ArtPollReply
 * with random delay (0-1s) per Art-Net spec.
 *
 * Flags bit map:
 * - Bit 0: Deprecated
 * - Bit 1: Send ArtPollReply on condition change (TODO: not implemented)
 * - Bit 2: Send diagnostics (TODO: not implemented, requires ArtDiagData)
 * - Bit 3: Diagnostics unicast vs broadcast (TODO: not implemented)
 * - Bit 4: Disable VLC (not applicable to this device)
 * - Bit 5: Targeted mode — implemented below
 */
void ArtNet_HandleArtPoll(const ArtNet_ArtPoll_t *pkt, uint16_t len,
                         const ip_addr_t *addr, u16_t port)
{
    // ===== FLAGS BIT 5: TARGETED MODE =====
    // Only reply if at least one of our Port-Addresses falls within the
    // targeted range [TargetPortAddressBottom, TargetPortAddressTop].
    if (pkt->flags & (1 << 5)) {
        // Target PA fields need the longer packet
        if (len < ARTNET_POLL_TARGETED_MIN_LENGTH) {
            ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                 "ArtPoll Target Short");
            return;
        }

        const DeviceConfig_t *config = DeviceConfig_Get();
        uint8_t num_universes = DeviceConfig_GetUniverseCount();
        
        uint16_t target_top = ((uint16_t)pkt->target_pa_top_hi << 8)
                            | pkt->target_pa_top_lo;
        uint16_t target_bot = ((uint16_t)pkt->target_pa_bot_hi << 8)
                            | pkt->target_pa_bot_lo;
        
        bool in_range = false;
        for (int i = 0; i < num_universes; i++) {
            uint16_t pa = ARTNET_BUILD_PORT_ADDRESS(
                config->dmx.artnet_net,
                config->dmx.artnet_subnet,
                config->dmx.artnet_start_universe + i);
            if (pa >= target_bot && pa <= target_top) {
                in_range = true;
                break;
            }
        }
        
        if (!in_range) {
            return;  // None of our Port-Addresses match targeted range
        }
    }
    
    // ===== QUEUE REPLY =====
    ArtNet_PollReplyQueue_t *q = &g_artnet_ctx.reply_queue;
    
    // Deduplicate per controller. A queued reply always goes to 0x1936 plus
    // the entry's port (ArtNet_SendReply), so de-duplicating by IP alone would
    // swallow polls from a tool on the same PC as the lighting software
    // (OpenTubeDia next to Lightkey): its reply would only reach 0x1936.
    for (uint8_t i = 0; i < q->count; i++) {
        ArtNet_PollReplyDest_t *entry = &q->entries[i];
        if (!ip_addr_cmp(addr, &entry->addr)) {
            continue;
        }
        if (entry->port == port || port == ARTNET_PORT) {
            return;  // Already covered by the queued reply
        }
        if (entry->port == ARTNET_PORT) {
            entry->port = port;  // Widen: one reply now reaches 0x1936 and this port
            return;
        }
        // Same IP, two different non-0x1936 ports: queue a separate reply
    }
    
    // Queue full: drop oldest entry (shift left) to make room
    if (q->count >= ARTNET_MAX_PENDING_REPLIES) {
        for (uint8_t i = 1; i < ARTNET_MAX_PENDING_REPLIES; i++) {
            q->entries[i - 1] = q->entries[i];
        }
        q->count = ARTNET_MAX_PENDING_REPLIES - 1;
    }
    
    // Append new destination
    ip_addr_copy(q->entries[q->count].addr, *addr);
    q->entries[q->count].port = port;
    bool was_empty = (q->count == 0);
    q->count++;
    
    // Start timer only if this is the first entry — subsequent entries
    // piggyback on the already-running timer's expiry
    if (was_empty) {
        uint32_t delay_ms = ArtNet_PrngNext() % (ARTNET_POLL_REPLY_DELAY_MAX_MS - 
                                                  ARTNET_POLL_REPLY_DELAY_MIN_MS + 1);
        delay_ms += ARTNET_POLL_REPLY_DELAY_MIN_MS;
        
        // osTimerStart requires delay >= 1
        if (delay_ms == 0) {
            delay_ms = 1;
        }
        
        // If the timer can't start, nothing would ever send or clear the
        // queue and later polls would only append: drop it instead, the
        // controller polls again within 3 s.
        if (osTimerStart(g_artnet_ctx.poll_reply_timer, delay_ms) != osOK) {
            q->count = 0;
        }
    }
}

/* ========================== ArtAddress Handler ========================== */

/**
 * @brief Handle incoming ArtAddress packet
 * 
 * Stub: answers every full-length ArtAddress with RcConfigErr and the
 * unchanged state. The implementation (names, Net/Sub/SwOut, AcFail*,
 * AcLed*, AcCancelMerge, AcClearOp0) is Phase C/D of
 * docs/plans/config-commits-and-artaddress.md: fields become one
 * DeviceConfig_Patch_t submitted to ControlTask, transient commands run here.
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len,
                            const ip_addr_t *src_ip, u16_t src_port)
{
    (void)pkt;

    if (len < sizeof(ArtNet_ArtAddress_t)) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                    "ArtAddress Too Short");
        return;
    }

    ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                "ArtAddress Unsupported");
}

/* ========================== ArtInput Handler ========================== */

/**
 * @brief Handle incoming ArtInput packet
 *
 * Spec packet strategy: Node receive -> reply with ArtPollReply. This node
 * has no Art-Net inputs, so Input[] disable bits have nothing to act on; the
 * reply just reports the (unchanged) state. NodeReport is left untouched.
 */
void ArtNet_HandleArtInput(const ArtNet_ArtInput_t *pkt,
                           const ip_addr_t *src_ip, u16_t src_port)
{
    // BindIndex 0 or 1 = root device; higher indices address binds we don't have
    if (pkt->bind_index > 1U) {
        return;
    }

    ArtNet_ReplyImmediate(src_ip, src_port);
}

/* ========================== ArtDataRequest Handler ========================== */

/**
 * @brief Handle incoming ArtDataRequest packet
 *
 * Answers DrPoll (empty payload) and the product / user guide / support URLs.
 * Any other code gets no reply, which the spec defines as "not supported".
 */
void ArtNet_HandleArtDataRequest(const ArtNet_ArtDataRequest_t *pkt,
                                 const ip_addr_t *src_ip, u16_t src_port)
{
    uint16_t request = ((uint16_t)pkt->request_hi << 8) | pkt->request_lo;

    switch (request) {
        case ARTNET_DR_POLL:
            ArtNet_SendDataReply(request, NULL, src_ip, src_port);
            break;
        case ARTNET_DR_URL_PRODUCT:
            ArtNet_SendDataReply(request, DEVICE_CONFIG_URL_PRODUCT, src_ip, src_port);
            break;
        case ARTNET_DR_URL_USER_GUIDE:
            ArtNet_SendDataReply(request, DEVICE_CONFIG_URL_USER_GUIDE, src_ip, src_port);
            break;
        case ARTNET_DR_URL_SUPPORT:
            ArtNet_SendDataReply(request, DEVICE_CONFIG_URL_SUPPORT, src_ip, src_port);
            break;
        default:
            // DrUrlPersUdr/Gdtf, manufacturer-specific, unknown: unsupported
            break;
    }
}

/* ========================== Private Functions ========================== */

/**
 * @brief Build and send an ArtDataReply
 *
 * Built directly in the pbuf (no stack buffer): runs in tcpip_thread.
 *
 * @param request Request code to echo
 * @param text    NUL-terminated payload, or NULL for an empty payload (DrPoll)
 */
static void ArtNet_SendDataReply(uint16_t request, const char *text,
                                 const ip_addr_t *src_ip, u16_t src_port)
{
    // Payload is string data including its NUL terminator
    size_t pay_len = (text != NULL) ? (strlen(text) + 1U) : 0U;
    if (pay_len > ARTNET_DATA_REPLY_PAYLOAD_MAX) {
        return;  // Compile-time URLs; cannot happen, but never overflow
    }

    const uint16_t total_len = (uint16_t)(sizeof(ArtNet_ArtDataReplyHeader_t) + pay_len);
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total_len, PBUF_RAM);
    if (p == NULL) {
        return;
    }

    ArtNet_ArtDataReplyHeader_t *hdr = (ArtNet_ArtDataReplyHeader_t *)p->payload;
    memset(hdr, 0, sizeof(*hdr));
    memcpy(hdr->id, ARTNET_ID, 8);
    hdr->opcode      = ARTNET_OP_DATA_REPLY;
    hdr->prot_ver_hi = 0U;
    hdr->prot_ver_lo = ARTNET_PROTOCOL_VERSION;
    hdr->esta_man_hi = (uint8_t)(DEVICE_CONFIG_ESTA_MAN_CODE >> 8);
    hdr->esta_man_lo = (uint8_t)(DEVICE_CONFIG_ESTA_MAN_CODE & 0xFFU);
    hdr->oem_hi      = (uint8_t)(DEVICE_CONFIG_ARTNET_OEM_CODE >> 8);
    hdr->oem_lo      = (uint8_t)(DEVICE_CONFIG_ARTNET_OEM_CODE & 0xFFU);
    hdr->request_hi  = (uint8_t)(request >> 8);
    hdr->request_lo  = (uint8_t)(request & 0xFFU);
    hdr->pay_len_hi  = (uint8_t)(pay_len >> 8);
    hdr->pay_len_lo  = (uint8_t)(pay_len & 0xFFU);

    if (pay_len > 0U) {
        memcpy((uint8_t *)p->payload + sizeof(*hdr), text, pay_len);
    }

    ArtNet_SendReply(p, src_ip, src_port);
    pbuf_free(p);
}
