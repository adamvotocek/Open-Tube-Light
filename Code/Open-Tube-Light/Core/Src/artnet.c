/**
 * @file artnet.c
 * @brief Art-Net 4 Protocol Handler Implementation
 */

#include "artnet.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"
#include "cmsis_os.h"
#include <string.h>

/* ========================== Private Defines ========================== */

#define ARTNET_ID "Art-Net" // defined by spec
#define ARTNET_SHORT_NAME "OpenTubeLight" // Max 17 chars + null
#define ARTNET_LONG_NAME "OpenTubeLight V0.1" // Max 63 chars + null

// Random delay range for ArtPollReply (Art-Net spec: 0-1 second)
#define ARTNET_POLL_REPLY_DELAY_MIN_MS  0
#define ARTNET_POLL_REPLY_DELAY_MAX_MS  1000

/* ========================== Private Types ========================== */

// Structure to hold pending ArtPollReply request
typedef struct {
    ip_addr_t addr;
    u16_t port;
    bool pending;
} ArtNet_PollReplyRequest_t;

/* ========================== Private Variables ========================== */

// DMX buffers in DTCM for fast CPU access
static uint8_t shadow_buffer[ARTNET_NUM_UNIVERSES][512] __attribute__((section(".DMX_Buffers"), aligned(32)));
static uint8_t active_buffer[ARTNET_NUM_UNIVERSES][512] __attribute__((section(".DMX_Buffers"), aligned(32)));

// Art-Net state
static ArtNet_State_t artnet_state;

// UDP PCB
static struct udp_pcb *artnet_pcb = NULL;

// Task to notify on new data
static osThreadId_t processing_task = NULL;

// Frame ready flag
static volatile bool frame_ready = false;

// ArtPollReply timer and pending request (for random delay per Art-Net spec)
static osTimerId_t poll_reply_timer = NULL;
static ArtNet_PollReplyRequest_t pending_poll_reply = {0};

// Simple PRNG state for random delay generation
static uint32_t prng_state = 0;

/* ========================== Private Function Prototypes ========================== */

static void artnet_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                  const ip_addr_t *addr, u16_t port);
static void handle_artdmx(const ArtNet_Dmx_t *pkt, uint16_t len);
static void handle_artpoll(const ip_addr_t *addr, u16_t port);
static void handle_artsync(void);
static void send_poll_reply(const ip_addr_t *addr, u16_t port);
static void send_poll_reply_from_timer(void *arg);
static void poll_reply_timer_callback(void *arg);
static uint32_t prng_next(void);
static bool validate_header(const uint8_t *data, uint16_t len);
static void trigger_output(void);

/* ========================== Public Functions ========================== */

int ArtNet_Init(osThreadId_t processing_task_handle)
{
    processing_task = processing_task_handle;
    
    // Initialize state
    memset(&artnet_state, 0, sizeof(artnet_state));
    artnet_state.universes_expected = (1 << ARTNET_NUM_UNIVERSES) - 1;
    
    // Clear buffers
    memset(shadow_buffer, 0, sizeof(shadow_buffer));
    memset(active_buffer, 0, sizeof(active_buffer));
    
    // Initialize pending poll reply state
    memset(&pending_poll_reply, 0, sizeof(pending_poll_reply));
    
    // Seed PRNG with system tick (good enough for random delay)
    prng_state = osKernelGetTickCount() ^ 0xDEADBEEF;
    
    // Create one-shot timer for ArtPollReply delay
    poll_reply_timer = osTimerNew(poll_reply_timer_callback, osTimerOnce, NULL, NULL);
    if (poll_reply_timer == NULL) {
        return -1;
    }
    
    // Create UDP PCB
    artnet_pcb = udp_new();
    if (artnet_pcb == NULL) {
        return -1;
    }
    
    // Bind to Art-Net port
    err_t err = udp_bind(artnet_pcb, IP_ADDR_ANY, ARTNET_PORT);
    if (err != ERR_OK) {
        udp_remove(artnet_pcb);
        artnet_pcb = NULL;
        return -1;
    }
    
    // Set receive callback
    udp_recv(artnet_pcb, artnet_recv_callback, NULL);
    
    return 0;
}

const uint8_t* ArtNet_GetUniverseData(uint8_t universe)
{
    if (universe >= ARTNET_NUM_UNIVERSES) {
        return NULL;
    }
    return active_buffer[universe];
}

bool ArtNet_IsFrameReady(void)
{
    return frame_ready;
}

void ArtNet_LatchData(void)
{
    // Copy shadow to active (atomic-ish with interrupts)
    memcpy(active_buffer, shadow_buffer, sizeof(active_buffer));
    frame_ready = false;
    artnet_state.universes_received = 0;
}

const ArtNet_State_t* ArtNet_GetState(void)
{
    return &artnet_state;
}

/* ========================== Private Functions ========================== */

static void artnet_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                  const ip_addr_t *addr, u16_t port)
{
    (void)arg; // avoid compiler unused parameter warnings
    (void)pcb;
    
    if (p == NULL || p->tot_len < sizeof(ArtNet_Header_t)) {
        if (p) pbuf_free(p);
        return;
    }
    
    // For chained pbufs, we need contiguous access
    uint8_t *data = (uint8_t *)p->payload;
    
    // Validate Art-Net header
    if (!validate_header(data, p->tot_len)) {
        pbuf_free(p);
        return;
    }
    
    // Get opcode (little-endian in packet)
    uint16_t opcode = data[8] | (data[9] << 8);
    
    switch (opcode) {
        case ARTNET_OP_DMX:
            if (p->tot_len >= 18) { // Minimum ArtDmx size
                handle_artdmx((const ArtNet_Dmx_t *)data, p->tot_len);
            }
            break;
            
        case ARTNET_OP_POLL:
            handle_artpoll(addr, port);
            break;
            
        case ARTNET_OP_SYNC:
            handle_artsync();
            break;
            
        default:
            // Ignore unknown opcodes
            break;
    }
    
    pbuf_free(p);
}

static bool validate_header(const uint8_t *data, uint16_t len)
{
    if (len < 10) return false;
    
    // Check "Art-Net\0" signature
    if (memcmp(data, ARTNET_ID, 8) != 0) {
        return false;
    }
    
    return true;
}

static void handle_artdmx(const ArtNet_Dmx_t *pkt, uint16_t len)
{
    // Validate protocol version
    if (pkt->prot_ver_hi != 0 || pkt->prot_ver_lo < ARTNET_PROTOCOL_VERSION) {
        return;
    }
    
    // Calculate 15-bit Port-Address
    // Port-Address = Net[14:8] : SubUni[7:0]
    uint8_t pkt_net = pkt->net & 0x7F;
    uint8_t pkt_subnet = (pkt->sub_uni >> 4) & 0x0F;
    uint8_t pkt_universe = pkt->sub_uni & 0x0F;
    
    // Check if this packet is for our net/subnet
    if (pkt_net != ARTNET_NET || pkt_subnet != ARTNET_SUBNET) {
        return;
    }
    
    // Calculate universe index relative to our start
    int universe_idx = pkt_universe - ARTNET_START_UNIVERSE;
    if (universe_idx < 0 || universe_idx >= ARTNET_NUM_UNIVERSES) {
        return;
    }
    
    // Get data length
    uint16_t dmx_len = (pkt->length_hi << 8) | pkt->length_lo;
    if (dmx_len > 512) dmx_len = 512;
    
    // Verify packet has enough data
    uint16_t expected_len = 18 + dmx_len;
    if (len < expected_len) {
        return;
    }
    
    // Copy DMX data to shadow buffer
    memcpy(shadow_buffer[universe_idx], pkt->data, dmx_len);
    
    // Mark universe as received
    artnet_state.universes_received |= (1 << universe_idx);
    
    // Check sync timeout - revert to instant mode if no sync received
    if (artnet_state.sync_mode) {
        uint32_t now = osKernelGetTickCount();
        if ((now - artnet_state.last_sync_tick) > ARTNET_SYNC_TIMEOUT_MS) {
            artnet_state.sync_mode = false;
        }
    }
    
    // In non-sync mode: trigger output when all universes received
    if (!artnet_state.sync_mode) {
        if (artnet_state.universes_received == artnet_state.universes_expected) {
            trigger_output();
        }
    }
}

static void handle_artpoll(const ip_addr_t *addr, u16_t port)
{
    // Ignore poll if a reply is already pending (per Art-Net spec, we have up to 1s to reply)
    if (pending_poll_reply.pending) {
        return;
    }
    
    // Store the sender's address for the delayed reply
    ip_addr_copy(pending_poll_reply.addr, *addr);
    pending_poll_reply.port = port;
    pending_poll_reply.pending = true;
    
    // Generate random delay between 0 and 1000ms (Art-Net spec requirement)
    uint32_t delay_ms = prng_next() % (ARTNET_POLL_REPLY_DELAY_MAX_MS - ARTNET_POLL_REPLY_DELAY_MIN_MS + 1);
    delay_ms += ARTNET_POLL_REPLY_DELAY_MIN_MS;
    
    // Ensure minimum delay of 1ms (osTimerStart requires non-zero for some implementations)
    if (delay_ms == 0) {
        delay_ms = 1;
    }
    
    // Start the one-shot timer
    osTimerStart(poll_reply_timer, delay_ms);
}

static void handle_artsync(void)
{
    artnet_state.sync_mode = true;
    artnet_state.last_sync_tick = osKernelGetTickCount();
    
    // Trigger immediate output
    trigger_output();
}

static void trigger_output(void)
{
    frame_ready = true;
    
    // Notify processing task
    if (processing_task != NULL) {
        osThreadFlagsSet(processing_task, 0x01);
    }
}

/**
 * @brief Simple XorShift PRNG for generating random delays
 * @return Pseudo-random 32-bit value
 */
static uint32_t prng_next(void)
{
    uint32_t x = prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    prng_state = x;
    return x;
}

/**
 * @brief Timer callback - schedules the actual send via tcpip_callback
 *        Timer callbacks run in timer daemon context, not LwIP context,
 *        so we must use tcpip_callback to safely call LwIP functions.
 */
static void poll_reply_timer_callback(void *arg)
{
    (void)arg;
    
    if (pending_poll_reply.pending) {
        // Schedule send_poll_reply to run in LwIP's tcpip_thread context
        tcpip_callback(send_poll_reply_from_timer, NULL);
    }
}

/**
 * @brief Called from tcpip_thread context to actually send the ArtPollReply
 */
static void send_poll_reply_from_timer(void *arg)
{
    (void)arg;
    
    if (pending_poll_reply.pending) {
        send_poll_reply(&pending_poll_reply.addr, pending_poll_reply.port);
        pending_poll_reply.pending = false;
    }
}

static void send_poll_reply(const ip_addr_t *addr, u16_t port)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ArtNet_PollReply_t), PBUF_RAM);
    if (p == NULL) return;
    
    ArtNet_PollReply_t *reply = (ArtNet_PollReply_t *)p->payload;
    memset(reply, 0, sizeof(ArtNet_PollReply_t)); // all fields default to 0
    
    // ID
    memcpy(reply->id, ARTNET_ID, 8);

    // Opcode
    reply->opcode = ARTNET_OP_POLL_REPLY; // Already little-endian
    
    // IP Address
    // Get our IP from the netif
    extern struct netif gnetif;
    const ip4_addr_t *our_ip = netif_ip4_addr(&gnetif);
    reply->ip[0] = ip4_addr1(our_ip);
    reply->ip[1] = ip4_addr2(our_ip);
    reply->ip[2] = ip4_addr3(our_ip);
    reply->ip[3] = ip4_addr4(our_ip);
    
    // Port
    reply->port = ARTNET_PORT;
    
    // Firmware version
    reply->vers_hi = 0;
    reply->vers_lo = 0;
    
    // NetSwitch, SubSwitch - part of Port-Address
    reply->net_switch = ARTNET_NET;
    reply->sub_switch = ARTNET_SUBNET;
    
    // OEM code (0xFFFF = prototype/development)
    // TODO: apply for OEM code before release
    reply->oem_hi = 0xFF;
    reply->oem_lo = 0xFF;
    
    // Status1
    // Bits 7,6 - Indicator state
    // Bits 5,4 - Port-Address Programming Authority
    // Bit 3 - zero
    // Bit 2 - Boot Mode
    // Bit 1 - RDM Capable
    // Bit 0 - UBEA present
    // 00 - Indicator unknown, 01 - All Port-Address set by front panel controls, 0, 0 - Normal Boot, 0 - RDM not capable, 0 - UBEA not present
    reply->status1 = 0b00010000;
    
    // Names
    strncpy(reply->short_name, ARTNET_SHORT_NAME, 17);
    strncpy(reply->long_name, ARTNET_LONG_NAME, 63);

    // Node report - format: "#xxxx [yyyy] zzzzz..."
    // xxxx = hex status code (0x0001 = RcPowerOk)
    // yyyy = decimal counter that increments each ArtPollReply (rolls at 9999)
    static uint16_t poll_reply_counter = 0;
    poll_reply_counter = (poll_reply_counter + 1) % 10000;
    // TODO: update status code and message based on actual device status
    memcpy(reply->node_report, "#0001 [0000] Power On OK", 25);
    reply->node_report[7]  = '0' + (poll_reply_counter / 1000) % 10;
    reply->node_report[8]  = '0' + (poll_reply_counter / 100) % 10;
    reply->node_report[9]  = '0' + (poll_reply_counter / 10) % 10;
    reply->node_report[10] = '0' + poll_reply_counter % 10;
    
    // Number of ports - Art-Net allows max 4 ports per ArtPollReply
    // For devices with >4 ports, multiple ArtPollReply packets with different BindIndex are used
    uint8_t num_ports = (ARTNET_NUM_UNIVERSES > 4) ? 4 : ARTNET_NUM_UNIVERSES;
    reply->num_ports_hi = 0;
    reply->num_ports_lo = num_ports;
    
    // Configure each output port
    for (int i = 0; i < num_ports; i++) {
        // PortTypes[]: Bit 7 = output capable, Bit 6 = input capable, Bits 5-0 = protocol (0x00 = DMX512)
        // 0x80 = Output only, DMX512 protocol
        reply->port_types[i] = 0x80;
        
        // SwOut[]: Low 4 bits of the 15-bit Port-Address for this output port
        // Combined with NetSwitch and SubSwitch to form full Port-Address
        reply->sw_out[i] = (ARTNET_START_UNIVERSE + i) & 0x0F;
        
        // GoodOutput[]: Output status flags
        // Bit 7: Data is being output (set if we have received ArtDmx)
        // Bit 6: Includes DMX512 test packets
        // Bit 5: Includes DMX512 SIP's
        // Bit 4: Includes DMX512 text packets  
        // Bit 3: Output is merging ArtNet data (not implemented yet)
        // Bit 2: DMX output short detected
        // Bit 1: Merge mode is LTP (0 = HTP)
        // Bit 0: Output converts from sACN (0 = Art-Net)
        uint8_t good_output = 0x00;
        if (artnet_state.universes_received & (1 << i)) {
            good_output |= 0x80;  // Data is being transmitted
        }
        reply->good_output[i] = good_output;
        
        // GoodOutputB[]: Additional output status (Art-Net 4)
        // Bit 7: RDM disabled (0 = enabled)
        // Bit 6: Output style continuous (0 = delta)
        // Bit 5: Discovery not running (0 = running)
        // Bit 4: Background discovery disabled (0 = enabled)
        // We don't support RDM, so set bit 7
        reply->good_output_b[i] = 0x80;
    }
    
    // Style: StNode (0x00), applicable to lights
    reply->style = 0x00;
    
    // MAC address (get from ETH peripheral if needed, or use placeholder)
    extern ETH_HandleTypeDef heth;
    reply->mac[0] = heth.Init.MACAddr[0];
    reply->mac[1] = heth.Init.MACAddr[1];
    reply->mac[2] = heth.Init.MACAddr[2];
    reply->mac[3] = heth.Init.MACAddr[3];
    reply->mac[4] = heth.Init.MACAddr[4];
    reply->mac[5] = heth.Init.MACAddr[5];
    
    // Bind IP (same as node IP for single-node)
    memcpy(reply->bind_ip, reply->ip, 4);
    reply->bind_index = 1;
    
    // Status2
    // 7 - RDM using ArtAddress capable, 6 - output style switch using ArtAddress capable, 5 - squawking, 4 - sACN switching capable, 3 - support of 15-bit Port-Address, 2 - DHCP capable, 1 - DHCP active, 0 - browser config capable
    reply->status2 = 0b00001110;
    
    // Status3
    // Bits 7-6: Failsafe state (00 = hold last, 01 = zero, 10 = full, 11 = scene)
    // Bit 5: Supports failsafe programming (0 = no, 1 = yes)
    // Bit 4: Supports LLRP (0 = no)
    // Bit 3: Supports port direction switching (0 = no)
    // Bit 2: Supports RDMnet (0 = no)
    // Bit 1: BackgroundQueue supported (0 = no)
    // Bit 0: Background discovery controllable via ArtAddress (0 = no)
    // Currently: hold last state, no failsafe programming support
    reply->status3 = 0b00000000;
    
    // Send unicast reply to sender
    // TODO: send with a 0-1s random delay as per the artnet spec
    udp_sendto(artnet_pcb, p, addr, port);

    
    pbuf_free(p);
}
