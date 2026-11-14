/**
 * @file artnet_dmx.c
 * @brief Art-Net data part: ArtDmx/ArtSync reception and the dmx_input driver
 *
 * Owns everything that only matters while Art-Net is the DMX input source:
 * the shadow/active DMX buffers, the per-universe source/sequence/sync
 * tracking driven by ArtDmx and ArtSync, the latch for EffectTask, and the
 * dmx_input_artnet vtable.
 *
 * Threading: the handlers run in tcpip_thread; ArtNet_LatchData() and
 * ArtNet_GetUniverseData() run in EffectTask. The shadow buffers are shared
 * between them under g_artnet_ctx.shadow_mutex.
 */

#include "Art-Net/artnet_internal.h"
#include "DMX-Input/dmx_input.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== Shared State ========================== */

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
    if (len < (ARTNET_DMX_HEADER_LENGTH + dmx_len)) {
        ArtNet_NodeReportSet(ARTNET_NODE_REPORT_RC_PARSE_FAIL,
                             "ArtDmx Truncated");
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

/* ========================== Data Access (EffectTask) ========================== */

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

/* ========================== Frame Notification ========================== */

void ArtNet_TriggerFrameOutput(void)
{
    // Notify processing task
    if (g_artnet_ctx.processing_task != NULL) {
        osThreadFlagsSet(g_artnet_ctx.processing_task, DMX_INPUT_FLAG_FRAME_READY);
    }
}

/* ========================== DMX Input Driver Vtable ========================== */

const DMX_Input_Driver_t dmx_input_artnet = {
    .init                = ArtNet_Init,
    .deinit              = ArtNet_Deinit,
    .latch               = ArtNet_LatchData,
    .get_universe        = ArtNet_GetUniverseData,
    .failsafe_timeout_ms = ARTNET_FAILSAFE_TIMEOUT_MS,
};
