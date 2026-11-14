/**
 * @file artnet_command.c
 * @brief ArtCommand handler: custom configuration verbs for OpenTubeDia
 *
 * Parses the transitional text commands ("DmxAddress=…&SegmentCount=…&Mode=RGB&")
 * into one device_config patch and replies with an ArtPollReply whose
 * NodeReport carries the result. Contract: docs/artnet-custom-protocol.md.
 *
 * Kept in its own file because it is temporary: RDM over Art-Net replaces
 * these verbs, and the whole file is then deleted.
 *
 * Runs in LwIP tcpip_thread context.
 */

#include "Art-Net/artnet_internal.h"
#include <stddef.h>
#include <string.h>

/* ========================== Private Constants ========================== */

#define ARTNET_COMMAND_PACKET_HEADER_SIZE  ((uint16_t)offsetof(ArtNet_ArtCommand_t, data))

#define ARTNET_COMMAND_SEEN_DMX_ADDRESS    (1U << 0)
#define ARTNET_COMMAND_SEEN_SEGMENT_COUNT  (1U << 1)
#define ARTNET_COMMAND_SEEN_MODE           (1U << 2)

/* ========================== Private Function Prototypes ========================== */

static const char *ArtNet_GetCommandSuccessDetail(bool dmx_address_changed,
                                                  bool segment_count_changed,
                                                  bool mode_changed);
static const char *ArtNet_GetCommandApplyErrorDetail(bool dmx_address_changed,
                                                     bool segment_count_changed,
                                                     bool mode_changed);
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
                                      uint16_t *error_code_out,
                                      const char **error_detail_out);

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
    uint16_t error_code = ARTNET_NODE_REPORT_RC_PARSE_FAIL;
    const char *error_detail = NULL;
    const char *success_detail = NULL;
    uint16_t text_len = 0U;
    bool dmx_address_changed = false;
    bool segment_count_changed = false;
    bool mode_changed = false;

    if (len < ARTNET_COMMAND_PACKET_HEADER_SIZE) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                    "ArtCommand Too Short");
        return;
    }

    text_len = ((uint16_t)pkt->length_hi << 8) | pkt->length_lo;
    if (text_len == 0U || text_len > ARTNET_COMMAND_TEXT_MAX_LENGTH) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                    "ArtCommand Bad Length");
        return;
    }

    if (len < (ARTNET_COMMAND_PACKET_HEADER_SIZE + text_len)) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                    "ArtCommand Truncated");
        return;
    }

    if (!ArtNet_GetNullTerminatedLength(pkt->data, text_len, &text_len)) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                                    "ArtCommand Missing NUL");
        return;
    }

    DeviceConfig_PatchInit(&patch);

    if (ArtNet_ParseCommandPayload((const char *)pkt->data, text_len, config,
                                   &patch, &error_code, &error_detail) != 0) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port, error_code, error_detail);
        return;
    }

    dmx_address_changed = patch.dmx_start_address_valid &&
                          (patch.dmx_start_address != config->dmx.dmx_start_address);
    segment_count_changed = patch.layout_valid &&
                            (patch.layout.segment_count != config->layout.segment_count);
    mode_changed = patch.layout_valid &&
                   (patch.layout.segment_format != config->layout.segment_format);

    if (DeviceConfig_ApplyPatch(&patch, false, NULL) != 0) {
        ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                    ARTNET_NODE_REPORT_RC_CONFIG_ERR,
                                    ArtNet_GetCommandApplyErrorDetail(dmx_address_changed,
                                                                      segment_count_changed,
                                                                      mode_changed));
        return;
    }

    success_detail = ArtNet_GetCommandSuccessDetail(dmx_address_changed,
                                                    segment_count_changed,
                                                    mode_changed);
    ArtNet_ReplyImmediateStatus(src_ip, src_port,
                                ARTNET_NODE_REPORT_RC_POWER_OK,
                                success_detail);
}

/* ========================== Private Functions ========================== */

static const char *ArtNet_GetCommandSuccessDetail(bool dmx_address_changed,
                                                  bool segment_count_changed,
                                                  bool mode_changed)
{
    uint8_t change_count = 0U;

    if (dmx_address_changed) {
        change_count++;
    }

    if (segment_count_changed) {
        change_count++;
    }

    if (mode_changed) {
        change_count++;
    }

    if (change_count == 0U) {
        return "No Change Applied";
    }

    if (change_count > 1U) {
        return "Multiple Fields Updated";
    }

    if (dmx_address_changed) {
        return "DMX Address Updated";
    }

    if (segment_count_changed) {
        return "Segment Count Updated";
    }

    return "Mode Updated";
}

static const char *ArtNet_GetCommandApplyErrorDetail(bool dmx_address_changed,
                                                     bool segment_count_changed,
                                                     bool mode_changed)
{
    if (dmx_address_changed && (segment_count_changed || mode_changed)) {
        return "Address/Layout Invalid";
    }

    if (dmx_address_changed) {
        return "DMX Address Invalid";
    }

    if (segment_count_changed || mode_changed) {
        return "Layout Invalid";
    }

    return "Command Rejected";
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
                                      uint16_t *error_code_out,
                                      const char **error_detail_out)
{
    DeviceConfig_Layout_t layout_candidate;
    uint8_t seen_mask = 0U;
    bool parsed_any_command = false;
    bool layout_dirty = false;
    size_t cursor = 0U;

    if (text == NULL || current_config == NULL || patch == NULL ||
        error_code_out == NULL || error_detail_out == NULL) {
        return -1;
    }

    *error_code_out = ARTNET_NODE_REPORT_RC_PARSE_FAIL;
    *error_detail_out = "ArtCommand Parse";
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
            *error_detail_out = "Missing =";
            return -1;
        }

        command_end = cursor;
        ArtNet_TrimSpan(text, &command_start, &command_end);
        if (command_start == command_end) {
            *error_detail_out = "Empty Command";
            return -1;
        }

        cursor++;
        value_start = cursor;
        while (cursor < text_len && text[cursor] != '&') {
            cursor++;
        }

        if (cursor >= text_len || text[cursor] != '&') {
            *error_detail_out = "Missing &";
            return -1;
        }

        value_end = cursor;
        ArtNet_TrimSpan(text, &value_start, &value_end);
        if (value_start == value_end) {
            *error_detail_out = "Empty Value";
            return -1;
        }

        if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                        "DmxAddress")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_DMX_ADDRESS) != 0U) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "Duplicate DmxAddress";
                return -1;
            }

            if (!ArtNet_ParseUint16Span(text, value_start, value_end,
                                        1U, 512U, &parsed_value)) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "DmxAddress Invalid";
                return -1;
            }

            patch->dmx_start_address_valid = true;
            patch->dmx_start_address = parsed_value;
            seen_mask |= ARTNET_COMMAND_SEEN_DMX_ADDRESS;
        } else if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                               "SegmentCount")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_SEGMENT_COUNT) != 0U) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "Duplicate SegmentCount";
                return -1;
            }

            if (!ArtNet_ParseUint16Span(text, value_start, value_end,
                                        1U, 999U, &parsed_value)) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "SegmentCount Invalid";
                return -1;
            }

            layout_candidate.segment_count = parsed_value;
            layout_dirty = true;
            seen_mask |= ARTNET_COMMAND_SEEN_SEGMENT_COUNT;
        } else if (ArtNet_SpanEqualsIgnoreCase(text, command_start, command_end,
                                               "Mode")) {
            if ((seen_mask & ARTNET_COMMAND_SEEN_MODE) != 0U) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "Duplicate Mode";
                return -1;
            }

            if (!ArtNet_SpanEqualsIgnoreCase(text, value_start, value_end, "RGB")) {
                *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
                *error_detail_out = "Mode Unsupported";
                return -1;
            }

            layout_candidate.segment_format = SEGMENT_FORMAT_RGB;
            layout_dirty = true;
            seen_mask |= ARTNET_COMMAND_SEEN_MODE;
        } else {
            *error_code_out = ARTNET_NODE_REPORT_RC_CONFIG_ERR;
            *error_detail_out = "Command Unsupported";
            return -1;
        }

        parsed_any_command = true;
        cursor++;
    }

    if (!parsed_any_command) {
        *error_detail_out = "No Commands";
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
