/**
 * @file artnet_protocol.h
 * @brief Art-Net 4 Protocol Definitions
 * 
 * This file contains all Art-Net protocol-specific definitions including
 * opcodes, packet structures, and protocol constants as defined in the
 * Art-Net 4 specification.
 * 
 * These definitions are separate from the implementation to maintain
 * clear separation of concerns and improve portability.
 * 
 * @see docs/reference/Art-Net_4.md (Art-Net 4 spec, local only)
 */

#ifndef INC_ARTNET_PROTOCOL_H_
#define INC_ARTNET_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Protocol Constants ========================== */

/**
 * @brief Art-Net UDP port number (6454 decimal)
 * @note This is the standard Art-Net port as defined in the specification
 */
#define ARTNET_PORT             0x1936

/**
 * @brief Art-Net protocol version (14 for Art-Net 4)
 * @note Devices must report this version to be Art-Net 4 compliant
 */
#define ARTNET_PROTOCOL_VERSION 14

/**
 * @brief Art-Net packet ID string
 * @note All Art-Net packets start with this 8-byte signature including null terminator
 */
#define ARTNET_ID               "Art-Net"

/**
 * @brief Maximum DMX data length per universe (512 bytes)
 * @note This is the DMX512-A specification limit
 */
#define ARTNET_DMX_MAX_LENGTH       512

/**
 * @brief Maximum ArtCommand text length including null terminator
 * @note Art-Net 4 allows up to 512 bytes for the ASCII command payload
 */
#define ARTNET_COMMAND_TEXT_MAX_LENGTH  512

/**
 * @brief OpSync timeout per Art-Net specification (4 seconds)
 * @note If no OpSync received within this time, revert to immediate output mode
 */
#define ARTNET_SYNC_TIMEOUT_MS  4000

/**
 * @brief Controller disconnect timeout (5 seconds)
 * @note If no ArtDmx received within this time, reset source IP tracking.
 *       Must be <= ARTNET_FAILSAFE_TIMEOUT_MS (static assert in
 *       artnet_internal.h) so a new controller can take over by the time
 *       failsafe triggers.
 */
#define ARTNET_DISCONNECT_TIMEOUT_MS  5000

/**
 * @brief Minimum random delay for ArtPollReply (0 ms)
 * @note Art-Net spec requires random delay between 0-1 second to avoid network congestion
 */
#define ARTNET_POLL_REPLY_DELAY_MIN_MS  0

/**
 * @brief Maximum random delay for ArtPollReply (1000 ms)
 * @note Art-Net spec requires random delay between 0-1 second to avoid network congestion
 */
#define ARTNET_POLL_REPLY_DELAY_MAX_MS  1000

/* ========================== OpCodes ========================== */

/**
 * @brief OpPoll - Controller requests node information
 * @note Nodes must respond with ArtPollReply after random delay (0-1s)
 */
#define ARTNET_OP_POLL          0x2000

/**
 * @brief OpPollReply - Node identification and status
 * @note Sent in response to OpPoll or on network configuration change
 */
#define ARTNET_OP_POLL_REPLY    0x2100

/**
 * @brief OpCommand - ASCII property programming commands
 * @note This implementation only supports the custom proof-of-concept commands
 */
#define ARTNET_OP_COMMAND       0x2400

/**
 * @brief OpDataRequest - Controller requests data such as product URLs
 * @note Node replies with ArtDataReply, or not at all if the code is unsupported
 */
#define ARTNET_OP_DATA_REQUEST  0x2700

/**
 * @brief OpDataReply - Node's answer to OpDataRequest
 */
#define ARTNET_OP_DATA_REPLY    0x2800

/**
 * @brief OpDmx (OpOutput) - DMX512 data transmission
 * @note Primary packet type for sending lighting data
 */
#define ARTNET_OP_DMX           0x5000

/**
 * @brief OpSync - Synchronization frame marker
 * @note Signals all nodes to output previously received OpDmx data simultaneously
 *       This enables tear-free updates across multiple universes
 */
#define ARTNET_OP_SYNC          0x5200

/**
 * @brief OpAddress - Remote node configuration
 * @note Controller sends this to reprogram node settings (names, Port-Address, etc.)
 * @note Handler is a stub that replies RcConfigErr until ArtAddress is implemented
 */
#define ARTNET_OP_ADDRESS       0x6000

/**
 * @brief OpInput - Controller enables/disables DMX inputs of a node
 * @note This node has no Art-Net inputs; it only answers with ArtPollReply
 */
#define ARTNET_OP_INPUT         0x7000

/* ========================== ArtDataRequest Codes (Table 4a) ========================== */

#define ARTNET_DR_POLL            0x0000  ///< DrPoll: is ArtDataRequest supported?
#define ARTNET_DR_URL_PRODUCT     0x0001  ///< DrUrlProduct: manufacturer product page
#define ARTNET_DR_URL_USER_GUIDE  0x0002  ///< DrUrlUserGuide: user guide
#define ARTNET_DR_URL_SUPPORT     0x0003  ///< DrUrlSupport: support page

/** @brief Maximum ArtDataReply payload, including the NUL terminator */
#define ARTNET_DATA_REPLY_PAYLOAD_MAX  512

/* ========================== Packet Structures ========================== */

/**
 * @brief Common Art-Net packet header
 * @note All Art-Net packets begin with this structure
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         ///< "Art-Net\0" signature
    uint16_t opcode;        ///< Little-endian opcode
} ArtNet_Header_t;

/**
 * @brief ArtDmx packet (OpOutput/OpDmx) - carries DMX512 data
 * @note This is the main packet type for lighting data transmission
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         ///< "Art-Net\0" signature
    uint16_t opcode;        ///< 0x5000 - OpDmx/OpOutput (little-endian)
    uint8_t  prot_ver_hi;   ///< Protocol version high byte (0 for Art-Net 4)
    uint8_t  prot_ver_lo;   ///< Protocol version low byte (14 for Art-Net 4)
    uint8_t  sequence;      ///< Sequence number (0 = disabled, 1-255 wraps)
    uint8_t  physical;      ///< Physical input port (informational)
    uint8_t  sub_uni;       ///< Lower 8 bits of 15-bit Port-Address
    uint8_t  net;           ///< Upper 7 bits of 15-bit Port-Address
    uint8_t  length_hi;     ///< Data length high byte (big-endian)
    uint8_t  length_lo;     ///< Data length low byte (big-endian)
    uint8_t  data[ARTNET_DMX_MAX_LENGTH];  ///< DMX512 data (up to 512 bytes)
} ArtNet_ArtDmx_t;

/**
 * @brief ArtPoll packet - discovery request from a controller
 * @note Controllers broadcast this to discover nodes on the network
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];            ///< "Art-Net\0" signature
    uint16_t opcode;           ///< 0x2000 (little-endian)
    uint8_t  prot_ver_hi;      ///< Protocol version high byte
    uint8_t  prot_ver_lo;      ///< Protocol version low byte
    uint8_t  flags;            ///< Behavior flags
    uint8_t  diag_priority;    ///< Diagnostic message priority
    uint8_t  target_pa_top_hi; ///< Top of the range of Port-Addresses high byte (optional, only in targeted mode)
    uint8_t  target_pa_top_lo; ///< Top of the range of Port-Addresses low byte (optional, only in targeted mode)
    uint8_t  target_pa_bot_hi; ///< Bottom of the range of Port-Addresses high byte (optional, only in targeted mode)
    uint8_t  target_pa_bot_lo; ///< Bottom of the range of Port-Addresses low byte (optional, only in targeted mode)
    uint8_t  esta_man_hi;      ///< ESTA manufacturer code high byte (optional)
    uint8_t  esta_man_lo;      ///< ESTA manufacturer code low byte (optional)
    uint8_t  oem_hi;           ///< OEM code high byte (optional)
    uint8_t  oem_lo;           ///< OEM code low byte (optional)
} ArtNet_ArtPoll_t;

/**
 * @brief ArtCommand packet - ASCII property commands
 * @note Length includes the null terminator. The payload may contain multiple
 *       Command=Data& assignments in one packet.
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];                                 ///< "Art-Net\0" signature
    uint16_t opcode;                                ///< 0x2400 (little-endian)
    uint8_t  prot_ver_hi;                           ///< Protocol version high byte
    uint8_t  prot_ver_lo;                           ///< Protocol version low byte
    uint8_t  esta_man_hi;                           ///< ESTA manufacturer code high byte
    uint8_t  esta_man_lo;                           ///< ESTA manufacturer code low byte
    uint8_t  length_hi;                             ///< ASCII payload length high byte
    uint8_t  length_lo;                             ///< ASCII payload length low byte
    uint8_t  data[ARTNET_COMMAND_TEXT_MAX_LENGTH];  ///< Null-terminated ASCII payload
} ArtNet_ArtCommand_t;

/**
 * @brief ArtPollReply packet - node identification and capabilities
 * @note Sent in response to ArtPoll or when node configuration changes
 *       Contains extensive node information including IP, ports, status, etc.
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];          ///< "Art-Net\0" signature
    uint16_t opcode;         ///< 0x2100 (little-endian)
    uint8_t  ip[4];          ///< Node IP address
    uint16_t port;           ///< Art-Net port (0x1936)
    uint8_t  vers_hi;        ///< Firmware version high byte
    uint8_t  vers_lo;        ///< Firmware version low byte
    uint8_t  net_switch;     ///< Bits 14-8 of Port-Address
    uint8_t  sub_switch;     ///< Bits 7-4 of Port-Address
    uint8_t  oem_hi;         ///< OEM code high byte
    uint8_t  oem_lo;         ///< OEM code low byte
    uint8_t  ubea_version;   ///< UBEA version (0 if not supported)
    uint8_t  status1;        ///< General status flags
    uint8_t  esta_man_lo;    ///< ESTA manufacturer code low byte
    uint8_t  esta_man_hi;    ///< ESTA manufacturer code high byte
    char     port_name[18]; ///< Short name (max 17 chars + null)
    char     long_name[64];  ///< Long name (max 63 chars + null)
    char     node_report[64];///< Status report string
    uint8_t  num_ports_hi;   ///< Number of ports high byte
    uint8_t  num_ports_lo;   ///< Number of ports low byte (max 4 per reply)
    uint8_t  port_types[4];  ///< Port types (input/output capabilities)
    uint8_t  good_input[4];  ///< Input port status
    uint8_t  good_output_a[4];///< Output port status
    uint8_t  sw_in[4];       ///< Input port universe addresses
    uint8_t  sw_out[4];      ///< Output port universe addresses
    uint8_t  acn_priority;   ///< sACN priority for DMX->sACN conversion (0: no inputs)
    uint8_t  sw_macro;       ///< Macro key switch
    uint8_t  sw_remote;      ///< Remote switch
    uint8_t  spare[3];       ///< Spare bytes
    uint8_t  style;          ///< Node style code
    uint8_t  mac[6];         ///< MAC address
    uint8_t  bind_ip[4];     ///< Binding IP address
    uint8_t  bind_index;     ///< Binding index (for multi-port devices)
    uint8_t  status2;        ///< Extended status flags
    uint8_t  good_output_b[4];///< Additional output status (Art-Net 4)
    uint8_t  status3;        ///< Additional status flags
    uint8_t  def_resp_UID_5; ///< RDMnet & LLRP Default Responder UID MSB
    uint8_t  def_resp_UID_4; ///< RDMnet & LLRP Default Responder UID
    uint8_t  def_resp_UID_3; ///< RDMnet & LLRP Default Responder UID
    uint8_t  def_resp_UID_2; ///< RDMnet & LLRP Default Responder UID
    uint8_t  def_resp_UID_1; ///< RDMnet & LLRP Default Responder UID
    uint8_t  def_resp_UID_0; ///< RDMnet & LLRP Default Responder UID LSB
    uint8_t  user_hi;        ///< Available for user specific data (high byte)
    uint8_t  user_lo;        ///< Available for user specific data (low byte)
    uint8_t  refresh_rate_hi;///< Max universe refresh rate high byte (Hz)
    uint8_t  refresh_rate_lo;///< Max universe refresh rate low byte (Hz)
    uint8_t  bgnd_queue_pol; ///< RDM background queue policy
    uint8_t  filler[10];     ///< Filler bytes for future expansion

} ArtNet_ArtPollReply_t;

/**
 * @brief ArtSync packet - frame synchronization marker
 * @note Minimal packet that triggers simultaneous output on all nodes
 *       Critical for tear-free multi-universe effects
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];         ///< "Art-Net\0" signature
    uint16_t opcode;        ///< 0x5200 (little-endian)
    uint8_t  prot_ver_hi;   ///< Protocol version high byte
    uint8_t  prot_ver_lo;   ///< Protocol version low byte
    uint8_t  aux1;          ///< Auxiliary data 1 (reserved)
    uint8_t  aux2;          ///< Auxiliary data 2 (reserved)
} ArtNet_ArtSync_t;

/**
 * @brief ArtAddress packet - node configuration command
 * @note Sent by controllers to configure node settings. Fields 5 (net_switch)
 * to 13 (command) contain the data that will be programmed into the node.
 * The device should apply the new settings and respond with an ArtPollReply.
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];           ///< "Art-Net\0" signature.
    uint16_t opcode;          ///< 0x6000 (OpAddress).
    uint8_t  prot_ver_hi;     ///< Protocol version high byte
    uint8_t  prot_ver_lo;     ///< Protocol version low byte
    uint8_t  net_switch;      ///< Top 7 bits of the 15-bit Port-Address (Bits 8-14).
    uint8_t  bind_index;      ///< Defines the bound node. 1 = root device.
    uint8_t  port_name[18];  ///< Spec "PortName": max 17 chars + null; empty = no change
    uint8_t  long_name[64];   ///< Max 63 chars + null; empty = no change
    uint8_t  sw_in[4];        ///< Bits 3-0 of the Port-Address of each input port (bit 7 = program)
    uint8_t  sw_out[4];       ///< Bits 3-0 of the Port-Address of each output port (bit 7 = program)
    uint8_t  sub_switch;      ///< Middle 4 bits of the 15-bit Port-Address (Bits 4-7).
    uint8_t  acn_priority;    ///< sACN Priority (0-200). 255 means no change.
    uint8_t  command;         ///< Various commands for node configuration.

} ArtNet_ArtAddress_t;

/**
 * @brief ArtInput packet - enable/disable DMX inputs (20 bytes)
 * @note Node replies with ArtPollReply. Input[] is ignored: no inputs here.
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];           ///< "Art-Net\0" signature
    uint16_t opcode;          ///< 0x7000 (little-endian)
    uint8_t  prot_ver_hi;     ///< Protocol version high byte
    uint8_t  prot_ver_lo;     ///< Protocol version low byte
    uint8_t  filler1;         ///< Pad length to match ArtPoll
    uint8_t  bind_index;      ///< Bound node addressed; 0 or 1 = root device
    uint8_t  num_ports_hi;    ///< Number of ports high byte (0)
    uint8_t  num_ports_lo;    ///< Number of ports low byte (max 4)
    uint8_t  input[4];        ///< Bit 0 set = disable this input
} ArtNet_ArtInput_t;

/**
 * @brief ArtDataRequest packet (40 bytes; only the first 20 are needed)
 * @note EstaMan/Oem identify the sender, they are not filters.
 *       Multi-byte fields are big-endian (Hi byte first).
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];           ///< "Art-Net\0" signature
    uint16_t opcode;          ///< 0x2700 (little-endian)
    uint8_t  prot_ver_hi;     ///< Protocol version high byte
    uint8_t  prot_ver_lo;     ///< Protocol version low byte
    uint8_t  esta_man_hi;     ///< Sender ESTA manufacturer code high byte
    uint8_t  esta_man_lo;     ///< Sender ESTA manufacturer code low byte
    uint8_t  oem_hi;          ///< Sender OEM code high byte
    uint8_t  oem_lo;          ///< Sender OEM code low byte
    uint8_t  request_hi;      ///< Requested data code high byte (Table 4a)
    uint8_t  request_lo;      ///< Requested data code low byte
    uint8_t  spare[22];       ///< Transmit as zero, receivers don't test
} ArtNet_ArtDataRequest_t;

/** @brief Bytes of ArtDataRequest needed to read the request code (spare excluded) */
#define ARTNET_DATA_REQUEST_MIN_LENGTH  20U

/**
 * @brief ArtDataReply packet header (20 bytes); pay_len payload bytes follow
 * @note Unlike ArtPollReply, EstaManHi comes before EstaManLo here.
 *       Multi-byte fields are big-endian (Hi byte first).
 */
typedef struct __attribute__((packed)) {
    uint8_t  id[8];           ///< "Art-Net\0" signature
    uint16_t opcode;          ///< 0x2800 (little-endian)
    uint8_t  prot_ver_hi;     ///< Protocol version high byte (0)
    uint8_t  prot_ver_lo;     ///< Protocol version low byte (14)
    uint8_t  esta_man_hi;     ///< Node ESTA manufacturer code high byte
    uint8_t  esta_man_lo;     ///< Node ESTA manufacturer code low byte
    uint8_t  oem_hi;          ///< Node OEM code high byte
    uint8_t  oem_lo;          ///< Node OEM code low byte
    uint8_t  request_hi;      ///< Echoed request code high byte
    uint8_t  request_lo;      ///< Echoed request code low byte
    uint8_t  pay_len_hi;      ///< Payload length high byte
    uint8_t  pay_len_lo;      ///< Payload length low byte
} ArtNet_ArtDataReplyHeader_t;

/* ========================== Minimum Packet Lengths ========================== */

/** @brief ArtDmx header length; DMX data starts here */
#define ARTNET_DMX_HEADER_LENGTH         18U

/** @brief Shortest ArtPoll a node must accept (through DiagPriority); later fields default to 0 */
#define ARTNET_POLL_MIN_LENGTH           14U

/** @brief ArtPoll length needed to read the targeted-mode Port-Address range */
#define ARTNET_POLL_TARGETED_MIN_LENGTH  18U

/* Wire sizes and offsets per Art-Net 4 spec; catch accidental padding or field edits */
_Static_assert(offsetof(ArtNet_ArtDmx_t, data) == ARTNET_DMX_HEADER_LENGTH, "ArtDmx header must be 18 bytes");
_Static_assert(offsetof(ArtNet_ArtPoll_t, target_pa_top_hi) == ARTNET_POLL_MIN_LENGTH, "ArtPoll minimum is 14 bytes");
_Static_assert(offsetof(ArtNet_ArtPoll_t, esta_man_hi) == ARTNET_POLL_TARGETED_MIN_LENGTH, "ArtPoll target range ends at 18");
_Static_assert(sizeof(ArtNet_ArtPollReply_t) == 239, "ArtPollReply must be 239 bytes");
_Static_assert(sizeof(ArtNet_ArtAddress_t) == 107, "ArtAddress must be 107 bytes");
_Static_assert(sizeof(ArtNet_ArtInput_t) == 20, "ArtInput must be 20 bytes");
_Static_assert(sizeof(ArtNet_ArtDataRequest_t) == 40, "ArtDataRequest must be 40 bytes");
_Static_assert(sizeof(ArtNet_ArtDataReplyHeader_t) == 20, "ArtDataReply header must be 20 bytes");

/* ========================== Protocol Helper Macros ========================== */

/**
 * @brief Extract network number from Port-Address
 * @param net The net byte from ArtDmx packet
 * @return Network number (0-127)
 */
#define ARTNET_GET_NET(net)         ((net) & 0x7F)

/**
 * @brief Extract subnet from Port-Address
 * @param sub_uni The sub_uni byte from ArtDmx packet
 * @return Subnet (0-15)
 */
#define ARTNET_GET_SUBNET(sub_uni)  (((sub_uni) >> 4) & 0x0F)

/**
 * @brief Extract universe from Port-Address
 * @param sub_uni The sub_uni byte from ArtDmx packet
 * @return Universe (0-15)
 */
#define ARTNET_GET_UNIVERSE(sub_uni) ((sub_uni) & 0x0F)

/**
 * @brief Build 15-bit Port-Address from components
 * @param net Network (0-127)
 * @param subnet Subnet (0-15)
 * @param universe Universe (0-15)
 * @return 15-bit Port-Address
 */
#define ARTNET_BUILD_PORT_ADDRESS(net, subnet, universe) \
    ((((uint16_t)(net) & 0x7F) << 8) | (((subnet) & 0x0F) << 4) | ((universe) & 0x0F))

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_PROTOCOL_H_ */
