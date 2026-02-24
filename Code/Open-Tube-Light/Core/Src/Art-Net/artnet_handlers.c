/**
 * @file artnet_handlers.c
 * @brief Art-Net Packet Handlers
 * 
 * This module implements handlers for incoming Art-Net packets:
 * - ArtDmx: DMX data reception with sequence tracking
 * - ArtPoll: Discovery request handling
 * - ArtSync: Frame synchronization
 * - ArtAddress: Remote configuration (future)
 * 
 * All handlers run in LwIP tcpip_thread context.
 */

#include "Art-Net/artnet_internal.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== ArtDmx Handler ========================== */

/**
 * @brief Handle incoming ArtDmx (OpOutput/OpDmx) packet
 * 
 * Validates protocol version and Port-Address, then copies DMX data to
 * shadow buffer. Implements first-source-wins per universe with timeout
 * to allow controller changes.
 * 
 * In non-sync mode, triggers output when all universes received.
 * In sync mode, the output is triggered from ArtSync handler.
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
    
    // ===== CONTROLLER DISCONNECT DETECTION =====
    // Reset source IP tracking if no data received recently
    uint32_t now = osKernelGetTickCount();
    if ((now - g_artnet_ctx.state.last_artdmx_tick) > ARTNET_DISCONNECT_TIMEOUT_MS) {
        for (int i = 0; i < DEVICE_CONFIG_MAX_UNIVERSES; i++) {
            ip_addr_set_zero(&g_artnet_ctx.state.universe_source_ip[i]);
        }
        ip_addr_set_zero(&g_artnet_ctx.state.last_artdmx_ip);
        g_artnet_ctx.state.sync_mode = false;
    }
    
    // Update last ArtDmx timestamp
    g_artnet_ctx.state.last_artdmx_tick = now;
    
    // ===== UNIVERSE SOURCE IP VALIDATION =====
    // Check if this universe already has a source IP assigned
    if (!ip_addr_isany(&g_artnet_ctx.state.universe_source_ip[universe_idx])) {
        // Universe has existing source - ignore packets from other sources
        if (!ip_addr_cmp(src_ip, &g_artnet_ctx.state.universe_source_ip[universe_idx])) {
            // Different IP detected - ignore this packet (no merge support)
            return;
        }
    } else {
        // First packet for this universe - record source IP
        ip_addr_copy(g_artnet_ctx.state.universe_source_ip[universe_idx], *src_ip);
    }
    
    // Update last ArtDmx source IP (used for ArtSync validation)
    ip_addr_copy(g_artnet_ctx.state.last_artdmx_ip, *src_ip);
    
    // Copy DMX data to shadow buffer
    memcpy(g_artnet_shadow_buffer[universe_idx], pkt->data, dmx_len);
    
    // Mark this universe as received
    g_artnet_ctx.state.universes_received |= (1 << universe_idx);
    
    // Check OpSync timeout - revert to non-sync mode if sync lost
    if (g_artnet_ctx.state.sync_mode) {
        if ((now - g_artnet_ctx.state.last_sync_tick) > ARTNET_SYNC_TIMEOUT_MS) {
            g_artnet_ctx.state.sync_mode = false;
        }
    }
    
    // In non-sync mode: trigger output when all universes received
    if (!g_artnet_ctx.state.sync_mode) {
        if (g_artnet_ctx.state.universes_received == g_artnet_ctx.state.universes_expected) {
            ArtNet_TriggerFrameOutput();
        }
    }
}

/* ========================== ArtPoll Handler ========================== */

/**
 * @brief Handle incoming ArtPoll packet
 * 
 * Schedules an ArtPollReply with random delay (0-1s) per Art-Net spec.
 */
void ArtNet_HandleArtPoll(const ip_addr_t *addr, u16_t port)
{
    // Ignore poll if reply already pending
    if (g_artnet_ctx.pending_reply.is_pending) {
        return;
    }
    
    // TODO: Implement flags
    // TODO: Implement targeted mode

    // Store destination address for delayed reply
    ip_addr_copy(g_artnet_ctx.pending_reply.addr, *addr);
    g_artnet_ctx.pending_reply.port = port;
    g_artnet_ctx.pending_reply.is_pending = true;
    
    // Generate random delay between 0 and 1000ms
    uint32_t delay_ms = ArtNet_PrngNext() % (ARTNET_POLL_REPLY_DELAY_MAX_MS - 
                                              ARTNET_POLL_REPLY_DELAY_MIN_MS + 1);
    delay_ms += ARTNET_POLL_REPLY_DELAY_MIN_MS;
    
    // Ensure minimum delay of 1ms
    if (delay_ms == 0) {
        delay_ms = 1;
    }
    
    // Start one-shot timer
    osTimerStart(g_artnet_ctx.poll_reply_timer, delay_ms);
}

/* ========================== ArtSync Handler ========================== */

/**
 * @brief Handle incoming ArtSync packet
 * 
 * Validates source IP against last ArtDmx and triggers synchronized output.
 */
void ArtNet_HandleArtSync(const ip_addr_t *src_ip)
{
    uint8_t num_universes = DeviceConfig_GetUniverseCount();
    
    // ArtSync only useful for multi-universe
    if (num_universes <= 1) {
        return;
    }
    
    // Ignore if no ArtDmx received yet
    if (ip_addr_isany(&g_artnet_ctx.state.last_artdmx_ip)) {
        return;
    }
    
    // Ignore if source doesn't match last ArtDmx
    if (!ip_addr_cmp(src_ip, &g_artnet_ctx.state.last_artdmx_ip)) {
        return;
    }
    
    // Enter sync mode and trigger output
    g_artnet_ctx.state.sync_mode = true;
    g_artnet_ctx.state.last_sync_tick = osKernelGetTickCount();
    
    ArtNet_TriggerFrameOutput();
}

/* ========================== ArtAddress Handler ========================== */

/**
 * @brief Handle incoming ArtAddress packet
 * 
 * Applies configuration changes from controller and sends ArtPollReply.
 * 
 * @note Not yet implemented - placeholder for future expansion
 */
void ArtNet_HandleArtAddress(const ArtNet_ArtAddress_t *pkt, uint16_t len)
{
    (void)pkt;
    (void)len;
    
    // TODO: Implement ArtAddress handling
    // - Parse configuration commands
    // - Update device_config via setter functions
    // - Send ArtPollReply to confirm changes
}
