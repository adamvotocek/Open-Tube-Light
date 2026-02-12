/**
 * @file artnet.h
 * @brief Art-Net 4 Protocol Handler for STM32H7
 * 
 * Implements Art-Net 4 with OpSync support for tear-free LED updates.
 */

#ifndef INC_ARTNET_H_
#define INC_ARTNET_H_

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Configuration ========================== */

#define ARTNET_PORT             0x1936      // UDP port 6454
#define ARTNET_PROTOCOL_VERSION 14

// Universe configuration - adjust based on your LED count
// NOTE: The configuration of Port-Address = 0x0000 (net=0, subnet=0, universe=0) is deprecated and not allowed
//       by the Art-Net 4 specification, but some controllers still use it as the first universe (eg. QLC+).
//       In this device, it is allowed, but the default universe on net/subnet 0/0 is universe 1 (Port-Address 0x0001).
#define ARTNET_NUM_UNIVERSES    1
#define ARTNET_START_UNIVERSE   1           // First universe we subscribe to (0-15)
#define ARTNET_NET              0           // Net (0-127)
#define ARTNET_SUBNET           0           // Sub-Net (0-15)

// Sync timeout (Art-Net spec: 4 seconds)
#define ARTNET_SYNC_TIMEOUT_MS  4000

/* ========================== OpCodes ========================== */

#define ARTNET_OP_POLL          0x2000
#define ARTNET_OP_POLL_REPLY    0x2100
#define ARTNET_OP_DMX           0x5000      // OpOutput / ArtDmx
#define ARTNET_OP_SYNC          0x5200

/* ========================== Packet Structures ========================== */

// Art-Net header (common to all packets)
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         // "Art-Net\0"
    uint16_t opcode;        // Little-endian opcode
} ArtNet_Header_t;

// ArtDmx packet (OpOutput)
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         // "Art-Net\0"
    uint16_t opcode;        // 0x5000 (little-endian)
    uint8_t  prot_ver_hi;   // Protocol version high (0)
    uint8_t  prot_ver_lo;   // Protocol version low (14)
    uint8_t  sequence;      // Sequence number (0 = disabled)
    uint8_t  physical;      // Physical port
    uint8_t  sub_uni;       // Low 8 bits of Port-Address
    uint8_t  net;           // High 7 bits of Port-Address
    uint8_t  length_hi;     // Data length high byte
    uint8_t  length_lo;     // Data length low byte
    uint8_t  data[512];     // DMX data
} ArtNet_Dmx_t;

// ArtPoll packet
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         // "Art-Net\0"
    uint16_t opcode;        // 0x2000
    uint8_t  prot_ver_hi;
    uint8_t  prot_ver_lo;
    uint8_t  flags;
    uint8_t  diag_priority;
} ArtNet_Poll_t;

// ArtPollReply packet
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         // "Art-Net\0"
    uint16_t opcode;        // 0x2100
    uint8_t  ip[4];         // Node IP
    uint16_t port;          // Art-Net port (0x1936)
    uint8_t  vers_hi;       // Firmware version high
    uint8_t  vers_lo;       // Firmware version low
    uint8_t  net_switch;    // Bits 14-8 of Port-Address
    uint8_t  sub_switch;    // Bits 7-4 of Port-Address
    uint8_t  oem_hi;
    uint8_t  oem_lo;
    uint8_t  ubea_version;
    uint8_t  status1;
    uint8_t  esta_man_lo;
    uint8_t  esta_man_hi;
    char     short_name[18];
    char     long_name[64];
    char     node_report[64];
    uint8_t  num_ports_hi;
    uint8_t  num_ports_lo;
    uint8_t  port_types[4];
    uint8_t  good_input[4];
    uint8_t  good_output[4];
    uint8_t  sw_in[4];
    uint8_t  sw_out[4];
    uint8_t  sw_video;
    uint8_t  sw_macro;
    uint8_t  sw_remote;
    uint8_t  spare[3];
    uint8_t  style;
    uint8_t  mac[6];
    uint8_t  bind_ip[4];
    uint8_t  bind_index;
    uint8_t  status2;
    uint8_t  good_output_b[4];
    uint8_t  status3;
    uint8_t  filler[21];
} ArtNet_PollReply_t;

// ArtSync packet
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         // "Art-Net\0"
    uint16_t opcode;        // 0x5200
    uint8_t  prot_ver_hi;
    uint8_t  prot_ver_lo;
    uint8_t  aux1;
    uint8_t  aux2;
} ArtNet_Sync_t;

/* ========================== State Structure ========================== */

typedef struct {
    bool     sync_mode;                 // True if OpSync packets detected
    uint32_t last_sync_tick;            // Last OpSync timestamp
    uint8_t  universes_received;        // Bitmask of received universes
    uint8_t  universes_expected;        // Bitmask of all expected universes
} ArtNet_State_t;

/* ========================== API Functions ========================== */

/**
 * @brief Initialize Art-Net receiver
 * @param processing_task_handle Handle to task that processes DMX data
 * @return 0 on success, -1 on failure
 */
int ArtNet_Init(osThreadId_t processing_task_handle);

/**
 * @brief Get pointer to active DMX buffer for a universe
 * @param universe Universe index (0 to ARTNET_NUM_UNIVERSES-1)
 * @return Pointer to 512-byte DMX buffer, or NULL if invalid
 */
const uint8_t* ArtNet_GetUniverseData(uint8_t universe);

/**
 * @brief Check if new frame is ready (called by processing task)
 * @return true if new data available
 */
bool ArtNet_IsFrameReady(void);

/**
 * @brief Latch shadow buffer to active buffer
 * Call this from processing task before rendering
 */
void ArtNet_LatchData(void);

/**
 * @brief Get current Art-Net state
 * @return Pointer to state structure
 */
const ArtNet_State_t* ArtNet_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_H_ */
