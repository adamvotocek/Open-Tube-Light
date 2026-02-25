/**
 * @file artnet_handlers.c
 * @brief Art-Net Packet Handlers
 * 
 * Handlers for incoming Art-Net packets:
 * - ArtDmx: DMX data reception with per-universe source tracking
 * - ArtPoll: Discovery request handling
 * - ArtSync: Per-universe frame synchronization
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