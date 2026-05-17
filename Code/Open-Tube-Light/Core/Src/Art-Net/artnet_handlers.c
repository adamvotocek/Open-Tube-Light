/**
 * @file artnet_handlers.c
 * @brief Art-Net Packet Handlers
 * 
 * Handlers for incoming Art-Net packets:
 * - ArtDmx: DMX data reception with per-universe source tracking
 * - ArtPoll: Discovery request handling
 * - ArtCommand: Custom runtime configuration commands
 * - ArtSync: Per-universe frame synchronization
 * - ArtAddress: Remote configuration (future)
 * 
 * All handlers run in LwIP tcpip_thread context.
 */

#include "Art-Net/artnet_internal.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== Private Constants ========================== */

#define ARTNET_COMMAND_PACKET_HEADER_SIZE  16U

#define ARTNET_COMMAND_SEEN_DMX_ADDRESS    (1U << 0)
#define ARTNET_COMMAND_SEEN_SEGMENT_COUNT  (1U << 1)
#define ARTNET_COMMAND_SEEN_MODE           (1U << 2)

/* ========================== Private Functions ========================== */

static void ArtNet_ReplyCommandStatus(const ip_addr_t *src_ip, u16_t src_port,
                                      uint16_t code, const char *detail);
static bool ArtNet_IsWhitespace(char ch);
static bool ArtNet_GetNullTerminatedLength(const uint8_t *text, uint16_t max_len,
                                           uint16_t *text_len_out);
static void ArtNet_TrimSpan(const char *text, size_t *start, size_t *end);
static bool ArtNet_SpanEqualsIgnoreCase(const char *text, size_t start, size_t end,
                                        const char *expected);
static bool ArtNet_ParseUint16Span(const char *text, size_t start, size_t end,
                                   uint16_t min_value, uint16_t max_value,
                                   uint16_t *value_out);
static int ArtNet_ParseCommandPayload(const char *text, size_t text_len,
                                      const DeviceConfig_t *current_config,
                                      DeviceConfig_Patch_t *patch,
                                      const char **error_detail_out);

/* ========================== ArtDmx Handler ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput/OpDmx) packet
 * 
 * Validates protocol version and Port-Address, then copies DMX data to
 * shadow buffer. Implements first-source-wins per universe with timeout
 * to allow controller changes.
 * 
 * Per-universe state tracking enables multi-controller operation:
 * each universe independently tracks its source IP, disconnect timeout,
 * and sync mode. In non-sync mode, output is triggered immediately
 * per Art-Net spec. In sync mode, output waits for ArtSync.
 */
void ArtNet_HandleArtDmx(const ArtNet_ArtDmx_t *pkt, uint16_t len, const ip_addr_t *src_ip)
{
    const DeviceConfig_t *config = DeviceConfig_Get();
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    
    // Decode 15-bit Port-Address from packet
    // Port-Address = Net[14:8] : SubNet[7:4] : Universe[3:0]
    uint8_t pkt_net = ARTNET_GET_NET(pkt->net);
    uint8_t pkt_subnet = ARTNET_GET_SUBNET(pkt->sub_uni);
    uint8_t pkt_universe = ARTNET_GET_UNIVERSE(pkt->sub_uni);
    
    // Check if packet is for our configured Net/SubNet
    if (pkt_net != config->dmx.artnet_net || pkt_subnet != config->dmx.artnet_subnet) {
        return;
    }
    
    // Calculate universe index relative to our start universe
    int universe_idx = pkt_universe - config->dmx.artnet_start_universe;
    if (universe_idx < 0 || universe_idx >= num_universes) {
        return;
    }
    
    // Extract DMX data length (big-endian in packet)
    uint16_t dmx_len = (pkt->length_hi << 8) | pkt->length_lo;
    if (dmx_len > ARTNET_DMX_MAX_LENGTH) {
        dmx_len = ARTNET_DMX_MAX_LENGTH;
    }
    
    // Verify packet contains advertised data
    if (len < (18 + dmx_len)) {
        return;
    }
    
    uint32_t now = osKernelGetTickCount();
    ArtNet_UniverseState_t *uni = &g_artnet_ctx.universes[universe_idx];
    
    // ===== CONTROLLER DISCONNECT DETECTION =====
    // Check all universes for stale source IPs
    for (int i = 0; i < num_universes; i++) {
        ArtNet_UniverseState_t *u = &g_artnet_ctx.universes[i];
        if (!ip_addr_isany(&u->source_ip) &&
            (now - u->last_dmx_tick) > ARTNET_DISCONNECT_TIMEOUT_MS) {
            ip_addr_set_zero(&u->source_ip);
            u->sync_mode = false;
            u->last_sequence = 0;
        }
    }
    
    // ===== UNIVERSE SOURCE IP VALIDATION =====
    // Check if this universe already has a source IP assigned
    if (!ip_addr_isany(&uni->source_ip)) {
        if (!ip_addr_cmp(src_ip, &uni->source_ip)) {
        	// Different IP detected - ignore this packet (no merge support)
            return;
        }
    } else {
        // First packet for this universe — record source controller
        ip_addr_copy(uni->source_ip, *src_ip);
    }
    
    // ===== SEQUENCE NUMBER CHECK =====
    // Sequence 0 disables re-ordering protection per Art-Net spec.
    // Non-zero: drop stale/duplicate packets using half-window comparison.
    if (pkt->sequence != 0) {
        if (uni->last_sequence != 0) {
            uint8_t diff = (uint8_t)(pkt->sequence - uni->last_sequence);
            if (diff == 0 || diff > 127) {
                return;  // Stale or duplicate packet
            }
        }
        uni->last_sequence = pkt->sequence;
    }
    
    // Update per-universe timestamp
    uni->last_dmx_tick = now;
    
    // Copy DMX data to shadow buffer (mutex guards against torn latch reads)
    osMutexAcquire(g_artnet_ctx.shadow_mutex, osWaitForever);
    memcpy(g_artnet_shadow_buffer[universe_idx], pkt->data, dmx_len);
    osMutexRelease(g_artnet_ctx.shadow_mutex);
    
    // ===== PER-UNIVERSE SYNC TIMEOUT =====
    // Art-Net spec: revert to non-sync if no ArtSync for 4 seconds
    if (uni->sync_mode) {
        if ((now - uni->last_sync_tick) > ARTNET_SYNC_TIMEOUT_MS) {
            uni->sync_mode = false;
        }
    }
    
    // ===== OUTPUT DECISION =====
    // Non-sync: immediate output per Art-Net spec §ArtSync
    // "ArtDmx packets will be immediately processed and output."
    // Sync: buffer data, wait for ArtSync from this controller
    if (!uni->sync_mode) {
        ArtNet_TriggerFrameOutput();
    }
}

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
        // Target PA fields need at least 18 bytes
        if (len >= 18) {
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
    }
    
    // ===== QUEUE REPLY =====
    ArtNet_PollReplyQueue_t *q = &g_artnet_ctx.reply_queue;
    
    // Deduplicate: if this controller IP is already queued, skip
    for (uint8_t i = 0; i < q->count; i++) {
        if (ip_addr_cmp(addr, &q->entries[i].addr)) {
            return;
        }
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
        
        osTimerStart(g_artnet_ctx.poll_reply_timer, delay_ms);
    }
}

/* ========================== ArtCommand Handler ========================== */

/**
 * @brief Handle incoming ArtCommand packet
 *
 * The proof-of-concept parser accepts only three custom commands and stages
 * all requested changes into one device_config patch so a packet never exposes
 * an intermediate runtime state if several fields change at once.
 */
void ArtNet_HandleArtCommand(const ArtNet_ArtCommand_t *pkt, uint16_t len,
                            const ip_addr_t *src_ip, u16_t src_port)
{
    const DeviceConfig_t *config = DeviceConfig_Get();
    DeviceConfig_Patch_t patch;
    uint32_t change_mask = DEVICE_CONFIG_CHANGE_NONE;
    const char *error_detail = NULL;
    uint16_t text_len = 0U;

    if (config->dmx.input_source != DMX_INPUT_ARTNET) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Art-Net Inactive");
        return;
    }

    if (len < ARTNET_COMMAND_PACKET_HEADER_SIZE) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Parse Error");
        return;
    }

    text_len = ((uint16_t)pkt->length_hi << 8) | pkt->length_lo;
    if (text_len == 0U || text_len > ARTNET_COMMAND_TEXT_MAX_LENGTH) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Parse Error");
        return;
    }

    if (len < (ARTNET_COMMAND_PACKET_HEADER_SIZE + text_len)) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Parse Error");
        return;
    }

    if (!ArtNet_GetNullTerminatedLength(pkt->data, text_len, &text_len)) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Parse Error");
        return;
    }

    DeviceConfig_PatchInit(&patch);
    if (ArtNet_ParseCommandPayload((const char *)pkt->data, text_len, config,
                                   &patch, &error_detail) != 0) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  error_detail);
        return;
    }

    if (DeviceConfig_ApplyPatch(&patch, false, &change_mask) != 0) {
        ArtNet_ReplyCommandStatus(src_ip, src_port,
                                  ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                  "Config Invalid");
        return;
    }

    (void)change_mask;
    ArtNet_ReplyCommandStatus(src_ip, src_port,
                              ARTNET_NODE_REPORT_RC_POWER_OK,
                              "Command OK");
}

/* ========================== ArtSync Handler ========================== */

/**
 * @brief Handle incoming ArtSync packet
 * 
 * Per Art-Net spec §ArtSync "Multiple controllers": compares the ArtSync
 * source IP against each universe's recorded source IP. Universes whose
 * controller matches enter sync mode and output their buffered data.
 * Universes driven by a different controller are unaffected.
 * 
 * This enables multi-controller setups: Controller A can sync its
 * universes independently of Controller B's non-sync universes.
 */
void ArtNet_HandleArtSync(const ip_addr_t *src_ip)
{
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    uint32_t now = osKernelGetTickCount();
    bool any_matched = false;
    
    // Iterate all configured universes and match against ArtSync source IP.
    // Spec: "a node shall compare the source IP of the ArtSync to the source
    //        IP of the most recent ArtDmx packet."
    // We apply this per-universe to support multi-controller environments.
    for (int i = 0; i < num_universes; i++) {
        ArtNet_UniverseState_t *uni = &g_artnet_ctx.universes[i];
        
        // Skip universes with no source assigned
        if (ip_addr_isany(&uni->source_ip)) {
            continue;
        }
        
        // Only sync universes whose controller sent this ArtSync
        if (ip_addr_cmp(src_ip, &uni->source_ip)) {
            uni->sync_mode = true;
            uni->last_sync_tick = now;
            any_matched = true;
        }
    }
    
    // Trigger output for the matched controller's buffered data
    if (any_matched) {
        ArtNet_TriggerFrameOutput();
    }
}

/* ========================== ArtAddress Handler ========================== */

/**
 * @brief Handle incoming ArtAddress packet
 * 
 * TODO: Not yet implemented. When implemented, this handler should:
 * - Parse NetSwitch, SubSwitch, SwOut[] to update Port-Address
 * - Parse ShortName/LongName to update device identity
 * - Parse Command field for merge mode, LED control, failsafe, etc.
 * - Apply changes via DeviceConfig_Set*() functions
 * - Send ArtPollReply to confirm changes
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len)
{
    (void)pkt;
    (void)len;
}

/* ========================== Private Functions ========================== */

static void ArtNet_ReplyCommandStatus(const ip_addr_t *src_ip, u16_t src_port,
                                      uint16_t code, const char *detail)
{
    ArtNet_NodeReportSet(code, detail);
    ArtNet_SendPollReply(src_ip, src_port);
}

static bool ArtNet_IsWhitespace(char ch)
{
    return (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n');
}

static bool ArtNet_GetNullTerminatedLength(const uint8_t *text, uint16_t max_len,
                                           uint16_t *text_len_out)
{
    uint16_t i;

    if (text == NULL || text_len_out == NULL || max_len == 0U) {
        return false;
    }

    for (i = 0U; i < max_len; i++) {
        if (text[i] == '\0') {
            *text_len_out = i;
            return true;
        }
    }

    return false;
}

static void ArtNet_TrimSpan(const char *text, size_t *start, size_t *end)
{
    if (text == NULL || start == NULL || end == NULL) {
        return;
    }

    while (*start < *end && ArtNet_IsWhitespace(text[*start])) {
        (*start)++;
    }

    while (*end > *start && ArtNet_IsWhitespace(text[*end - 1U])) {
        (*end)--;
    }
}

static bool ArtNet_SpanEqualsIgnoreCase(const char *text, size_t start, size_t end,
                                        const char *expected)
{
    size_t i = 0U;

    if (text == NULL || expected == NULL) {
        return false;
    }

    while ((start + i) < end && expected[i] != '\0') {
        char lhs = text[start + i];
        char rhs = expected[i];

        if (lhs >= 'A' && lhs <= 'Z') {
            lhs = (char)(lhs - 'A' + 'a');
        }

        if (rhs >= 'A' && rhs <= 'Z') {
            rhs = (char)(rhs - 'A' + 'a');
        }

        if (lhs != rhs) {
            return false;
        }

        i++;
    }

    return ((start + i) == end && expected[i] == '\0');
}

static bool ArtNet_ParseUint16Span(const char *text, size_t start, size_t end,
                                   uint16_t min_value, uint16_t max_value,
                                   uint16_t *value_out)
{
    uint32_t value = 0U;
    size_t i;

    if (text == NULL || value_out == NULL || start >= end) {
        return false;
    }

    for (i = start; i < end; i++) {
        if (text[i] < '0' || text[i] > '9') {
            return false;
        }

        value = (value * 10U) + (uint32_t)(text[i] - '0');
        if (value > max_value) {
            return false;
        }
    }

    if (value < min_value) {
        return false;
    }

    *value_out = (uint16_t)value;
    return true;
}

static int ArtNet_ParseCommandPayload(const char *text, size_t text_len,
                                      const DeviceConfig_t *current_config,
                                      DeviceConfig_Patch_t *patch,
                                      const char **error_detail_out)
{
    DeviceConfig_Layout_t layout_candidate;
    uint8_t seen_mask = 0U;
    bool parsed_any_command = false;
    bool layout_dirty = false;
    size_t cursor = 0U;

    if (text == NULL || current_config == NULL || patch == NULL ||
        error_detail_out == NULL) {
        return -1;
    }

    *error_detail_out = "Parse Error";
    layout_candidate = current_config->layout;

    while (cursor < text_len) {
        size_t command_start;
        size_t command_end;
        size_t value_start;
        size_t value_end;
        uint16_t parsed_value;

        while (cursor < text_len && ArtNet_IsWhitespace(text[cursor])) {
            cursor++;
        }

        if (cursor >= text_len) {
            break;
        }

        command_start = cursor;
        while (cursor < text_len && text[cursor] != '=' && text[cursor] != '&') {
            cursor++;
        }

        if (cursor >= text_len || text[cursor] != '=') {
            return -1;
        }

        command_end = cursor;
        ArtNet_TrimSpan(text, &command_start, &command_end);
        if (command_start == command_end) {
            return -1;
        }

        cursor++;
        value_start = cursor;
        while (cursor < text_len && text[cursor] != '&') {
            cursor++;
        }

        if (cursor >= text_len || text[cursor] != '&') {
            return -1;
        }

        value_end = cursor;
        ArtNet_TrimSpan(text, &value_start, &value_end);
        if (value_start == value_end) {
            return -1;
        }

        if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                        "DmxAddress")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_DMX_ADDRESS) != 0U) {
                *error_detail_out = "Duplicate Command";
                return -1;
            }

            if (!ArtNet_ParseUint16Span(text, value_start, value_end,
                                        1U, 512U, &parsed_value)) {
                *error_detail_out = "Invalid DmxAddress";
                return -1;
            }

            patch->dmx_start_address_valid = true;
            patch->dmx_start_address = parsed_value;
            seen_mask |= ARTNET_COMMAND_SEEN_DMX_ADDRESS;
        } else if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                               "SegmentCount")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_SEGMENT_COUNT) != 0U) {
                *error_detail_out = "Duplicate Command";
                return -1;
            }

            if (!ArtNet_ParseUint16Span(text, value_start, value_end,
                                        1U, 999U, &parsed_value)) {
                *error_detail_out = "Invalid SegmentCount";
                return -1;
            }

            layout_candidate.segment_count = parsed_value;
            layout_dirty = true;
            seen_mask |= ARTNET_COMMAND_SEEN_SEGMENT_COUNT;
        } else if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                               "Mode")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_MODE) != 0U) {
                *error_detail_out = "Duplicate Command";
                return -1;
            }

            if (!ArtNet_SpanEqualsIgnoreCase(text, value_start, value_end, "RGB")) {
                *error_detail_out = "Unsupported Mode";
                return -1;
            }

            layout_candidate.segment_format = SEGMENT_FORMAT_RGB;
            layout_dirty = true;
            seen_mask |= ARTNET_COMMAND_SEEN_MODE;
        } else {
            *error_detail_out = "Unsupported Command";
            return -1;
        }

        parsed_any_command = true;
        cursor++;
    }

    if (!parsed_any_command) {
        return -1;
    }

    if (layout_dirty) {
        // Fold all segment-related command fields into one layout write so the
        // validator can reject invalid combinations atomically.
        patch->layout_valid = true;
        patch->layout = layout_candidate;
    }

    return 0;
}