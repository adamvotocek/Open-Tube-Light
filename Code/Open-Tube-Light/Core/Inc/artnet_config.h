/**
 * @file artnet_config.h
 * @brief Art-Net Configuration Parameters
 * 
 * This file contains all user-configurable Art-Net parameters including
 * universe addressing, device identity, and operational settings.
 * 
 * Modify these constants to configure the Art-Net node behavior for
 * your specific application requirements.
 */

#ifndef INC_ARTNET_CONFIG_H_
#define INC_ARTNET_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== Universe Configuration ========================== */

/**
 * @brief Number of Art-Net universes to receive
 * @note Each universe supports 512 DMX channels
 *       Adjust based on your LED count (e.g., RGB LEDs need 3 channels each)
 *       Maximum 4 universes per ArtPollReply (use multiple replies for >4)
 */
#define ARTNET_NUM_UNIVERSES    1

/**
 * @brief First universe number to subscribe to (0-15)
 * @note Art-Net 4 spec deprecates Port-Address 0x0000 (net=0, subnet=0, universe=0)
 *       but some controllers (e.g., QLC+) still use it as the first universe.
 *       This implementation allows universe 0 for compatibility, but universe 1
 *       is recommended as the default starting point.
 */
#define ARTNET_START_UNIVERSE   1

/**
 * @brief Network number (0-127)
 * @note Part of the 15-bit Port-Address
 *       Use different networks to isolate groups of fixtures
 */
#define ARTNET_NET              0

/**
 * @brief Subnet number (0-15)
 * @note Part of the 15-bit Port-Address
 *       Use different subnets to organize universes within a network
 */
#define ARTNET_SUBNET           0

/* ========================== Device Identity ========================== */

/**
 * @brief Short device name (max 17 characters + null terminator)
 * @note Displayed in controller software for device identification
 *       Keep concise for UI display
 */
#define ARTNET_SHORT_NAME       "OpenTubeLight"

/**
 * @brief Long device name (max 63 characters + null terminator)
 * @note Can include version info and detailed description
 */
#define ARTNET_LONG_NAME        "OpenTubeLight V0.1"

/**
 * @brief Firmware version major (0-255)
 * @note Reported in ArtPollReply vers_hi field
 */
#define ARTNET_FIRMWARE_VER_HI  0

/**
 * @brief Firmware version minor (0-255)
 * @note Reported in ArtPollReply vers_lo field
 */
#define ARTNET_FIRMWARE_VER_LO  1

/**
 * @brief OEM code high byte
 * @note 0xFFFF = prototype/development
 *       Apply for official OEM code at: https://art-net.org.uk/oem-code-zone/
 *       before commercial release
 */
#define ARTNET_OEM_CODE_HI      0xFF

/**
 * @brief OEM code low byte
 * @note 0xFFFF = prototype/development
 */
#define ARTNET_OEM_CODE_LO      0xFF

/**
 * @brief ESTA manufacturer code low byte
 * @note 0x0000 = no ESTA manufacturer code assigned
 *       Apply for code at: https://tsp.esta.org/tsp/working_groups/CP/mfctrIDs.php
 */
#define ARTNET_ESTA_MAN_LO      0x00

/**
 * @brief ESTA manufacturer code high byte
 * @note 0x0000 = no ESTA manufacturer code assigned
 */
#define ARTNET_ESTA_MAN_HI      0x00

/* ========================== Operational Settings ========================== */

/**
 * @brief Node style code
 * @note 0x00 = StNode (stylized as a light fixture)
 *       Other values: 0x01=StController, 0x02=StMedia, 0x03=StRoute, 0x04=StBackup, 0x05=StConfig
 */
#define ARTNET_NODE_STYLE       0x00

/**
 * @brief Status1 default flags
 * @note Bit 7-6: Indicator state (00=unknown)
 *       Bit 5-4: Port-Address programming (01=front panel)
 *       Bit 3: Reserved (0)
 *       Bit 2: Boot mode (0=normal)
 *       Bit 1: RDM capable (0=no)
 *       Bit 0: UBEA present (0=no)
 */
#define ARTNET_STATUS1_DEFAULT  0b00010000

/**
 * @brief Status2 default flags
 * @note Bit 7: RDM via ArtAddress (0=no)
 *       Bit 6: Output style via ArtAddress (0=no)
 *       Bit 5: Squawking (0=no)
 *       Bit 4: sACN switching (0=no)
 *       Bit 3: 15-bit Port-Address support (1=yes)
 *       Bit 2: DHCP capable (1=yes)
 *       Bit 1: DHCP active (1=yes)
 *       Bit 0: Browser config (0=no)
 */
#define ARTNET_STATUS2_DEFAULT  0b00001110

/**
 * @brief Status3 default flags
 * @note Bit 7-6: Failsafe state (00=hold last)
 *       Bit 5: Failsafe programming support (0=no)
 *       Bit 4: LLRP support (0=no)
 *       Bit 3: Port direction switching (0=no)
 *       Bit 2: RDMnet support (0=no)
 *       Bit 1: Background queue (0=no)
 *       Bit 0: Background discovery via ArtAddress (0=no)
 */
#define ARTNET_STATUS3_DEFAULT  0b00000000

/**
 * @brief Port type for output ports
 * @note Bit 7: Output capable (1=yes)
 *       Bit 6: Input capable (0=no)
 *       Bit 5-0: Protocol (0x00=DMX512)
 */
#define ARTNET_PORT_TYPE_OUTPUT 0x80

/**
 * @brief GoodOutputB flags for ports
 * @note Bit 7: RDM disabled (1=disabled)
 *       Bit 6: Output style continuous (0=delta)
 *       Bit 5: Discovery not running (0=running)
 *       Bit 4: Background discovery disabled (0=enabled)
 */
#define ARTNET_GOOD_OUTPUT_B    0x80

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_CONFIG_H_ */
