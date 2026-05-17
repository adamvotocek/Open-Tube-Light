/**
 * @file device_config.h
 * @brief Device Configuration Management
 * 
 * Central configuration module for all user-configurable device settings.
 * Settings are stored in flash and can be modified at runtime via display/buttons
 * or Art-Net commands (ArtAddress, ArtIpProg, etc.).
 * 
 * The configuration is organized into logical groups:
 * - Device identity (names)
 * - Network settings (IP, DHCP)
 * - Pixel/LED configuration
 * - DMX/Art-Net protocol settings
 * - Failsafe behavior
 * 
 * Factory defaults are defined statically and used for initial firmware upload
 * or factory reset operations.
 * 
 * @note Settings changes take effect immediately via callback notification
 * @note Flash storage uses a dedicated flash sector to preserve settings across power cycles
 */

#ifndef INC_DEVICE_CONFIG_H_
#define INC_DEVICE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================== Constants ========================== */

/** @brief Maximum length for device short name (Art-Net compatible: 17 chars + null) */
#define DEVICE_CONFIG_SHORT_NAME_LEN    18

/** @brief Maximum length for device long name (Art-Net compatible: 63 chars + null) */
#define DEVICE_CONFIG_LONG_NAME_LEN     64

/** @brief Maximum number of pixels supported by the device */
#define DEVICE_CONFIG_MAX_PIXELS        288

/** @brief Maximum DMX data length per universe (512 bytes per DMX512-A spec) */
#define DMX_UNIVERSE_MAX_LENGTH         512

/** @brief Maximum number of Art-Net universes */
#define DEVICE_CONFIG_MAX_UNIVERSES     4

/** @brief Configuration version for flash format migration */
#define DEVICE_CONFIG_VERSION           1

/** @brief Magic number to validate flash config integrity */
#define DEVICE_CONFIG_MAGIC             0xA1B2C3D4

/* ========================== Enumerations ========================== */

/**
 * @brief IP address configuration mode
 */
typedef enum {
    IP_MODE_DHCP   = 0,     ///< Obtain IP via DHCP (default)
    IP_MODE_STATIC = 1,     ///< Use static IP configuration
} DeviceConfig_IpMode_t;

/**
 * @brief Pixel color format/channel configuration
 */
typedef enum {
    PIXEL_FORMAT_RGB      = 0,  ///< 3 channels: Red, Green, Blue
    PIXEL_FORMAT_RGBW     = 1,  ///< 4 channels: Red, Green, Blue, White
    PIXEL_FORMAT_RGB16    = 2,  ///< 6 channels: 16-bit RGB (high/low bytes)
} DeviceConfig_PixelFormat_t;

/**
 * @brief DMX input source selection
 */
typedef enum {
    DMX_INPUT_ARTNET = 0,   ///< Art-Net over Ethernet (default)
    DMX_INPUT_SACN   = 1,   ///< sACN (E1.31) over Ethernet
    DMX_INPUT_DMX512 = 2,   ///< DMX512 over UART
} DeviceConfig_DmxInput_t;

/**
 * @brief Failsafe behavior when DMX signal is lost
 */
typedef enum {
    FAILSAFE_HOLD = 0,      ///< Hold last received values (default)
    FAILSAFE_FULL = 1,      ///< All channels to 255 (full on)
    FAILSAFE_ZERO = 2,      ///< All channels to 0 (blackout)
} DeviceConfig_Failsafe_t;



/* ========================== Configuration Structures ========================== */

/**
 * @brief Device identity configuration
 */
typedef struct {
    char short_name[DEVICE_CONFIG_SHORT_NAME_LEN];  ///< Short name (17 chars max)
    char long_name[DEVICE_CONFIG_LONG_NAME_LEN];    ///< Long name (63 chars max)
} DeviceConfig_Identity_t;

/**
 * @brief Network configuration
 */
typedef struct {
    DeviceConfig_IpMode_t ip_mode;      ///< DHCP or static IP mode
    uint8_t static_ip[4];               ///< Static IP address (if ip_mode == STATIC)
    uint8_t static_mask[4];             ///< Static subnet mask (if ip_mode == STATIC)
    uint8_t static_gateway[4];          ///< Static gateway (if ip_mode == STATIC)
} DeviceConfig_Network_t;

/**
 * @brief Pixel/LED strip configuration
 */
typedef struct {
    uint16_t pixel_count;                           ///< Number of pixels in the strip
    DeviceConfig_PixelFormat_t pixel_format;        ///< Pixel color format
} DeviceConfig_Pixel_t;

/**
 * @brief DMX/Art-Net protocol configuration
 */
typedef struct {
    DeviceConfig_DmxInput_t input_source;          ///< DMX input source (Art-Net or DMX512)
    uint16_t dmx_start_address;                    ///< DMX start address (1-512)
    uint8_t artnet_net;                            ///< Art-Net network (0-127)
    uint8_t artnet_subnet;                         ///< Art-Net subnet (0-15)
    uint8_t artnet_start_universe;                 ///< Art-Net first universe (0-15)
    uint8_t artnet_max_refresh_rate;               ///< Max refresh rate in Hz (44 - 255)
} DeviceConfig_Dmx_t;

/**
 * @brief Output behavior configuration
 */
typedef struct {
    DeviceConfig_Failsafe_t failsafe;       ///< Behavior when DMX signal lost
} DeviceConfig_Output_t;

/**
 * @brief Change mask reported after a runtime config commit
 *
 * Lets callers react only to the parts of the configuration that changed.
 */
typedef enum {
    DEVICE_CONFIG_CHANGE_NONE              = 0,
    DEVICE_CONFIG_CHANGE_IDENTITY          = (1U << 0),
    DEVICE_CONFIG_CHANGE_NETWORK           = (1U << 1),
    DEVICE_CONFIG_CHANGE_PIXEL             = (1U << 2),
    DEVICE_CONFIG_CHANGE_DMX_SOURCE        = (1U << 3),
    DEVICE_CONFIG_CHANGE_DMX_START_ADDRESS = (1U << 4),
    DEVICE_CONFIG_CHANGE_ARTNET_ADDRESS    = (1U << 5),
    DEVICE_CONFIG_CHANGE_DMX_REFRESH_RATE  = (1U << 6),
    DEVICE_CONFIG_CHANGE_OUTPUT_FAILSAFE   = (1U << 7),
    DEVICE_CONFIG_CHANGE_DMX               = DEVICE_CONFIG_CHANGE_DMX_SOURCE |
                                             DEVICE_CONFIG_CHANGE_DMX_START_ADDRESS |
                                             DEVICE_CONFIG_CHANGE_ARTNET_ADDRESS |
                                             DEVICE_CONFIG_CHANGE_DMX_REFRESH_RATE,
    DEVICE_CONFIG_CHANGE_OUTPUT            = DEVICE_CONFIG_CHANGE_OUTPUT_FAILSAFE,
} DeviceConfig_ChangeFlags_t;

/**
 * @brief Partial runtime update for one or more config fields
 *
 * Each `*_valid` flag controls whether the corresponding field should be
 * applied to a candidate configuration before validation and commit.
 */
typedef struct {
    bool short_name_valid;
    char short_name[DEVICE_CONFIG_SHORT_NAME_LEN];

    bool long_name_valid;
    char long_name[DEVICE_CONFIG_LONG_NAME_LEN];

    bool network_valid;
    DeviceConfig_Network_t network;

    bool pixel_valid;
    DeviceConfig_Pixel_t pixel;

    bool input_source_valid;
    DeviceConfig_DmxInput_t input_source;

    bool dmx_start_address_valid;
    uint16_t dmx_start_address;

    bool artnet_net_valid;
    uint8_t artnet_net;

    bool artnet_subnet_valid;
    uint8_t artnet_subnet;

    bool artnet_start_universe_valid;
    uint8_t artnet_start_universe;

    bool artnet_max_refresh_rate_valid;
    uint8_t artnet_max_refresh_rate;

    bool failsafe_valid;
    DeviceConfig_Failsafe_t failsafe;
} DeviceConfig_Patch_t;

/**
 * @brief Complete device configuration
 * 
 * This structure contains all user-configurable settings. It is stored in
 * flash and loaded at startup. The magic and version fields are used to
 * detect if the flash contains valid configuration data.
 */
typedef struct {
    uint32_t magic;                         ///< Magic number for validation
    uint16_t version;                       ///< Config format version
    uint16_t reserved;                      ///< Reserved for alignment/future use
    
    DeviceConfig_Identity_t identity;       ///< Device names
    DeviceConfig_Network_t network;         ///< Network settings
    DeviceConfig_Pixel_t pixel;             ///< Pixel configuration
    DeviceConfig_Dmx_t dmx;                 ///< DMX/Art-Net settings
    DeviceConfig_Output_t output;           ///< Output behavior
    
    uint32_t checksum;                      ///< CRC32 for data integrity
} DeviceConfig_t;

/**
 * @brief Configuration change notification callback type
 * 
 * Called when configuration is modified. Modules register callbacks to
 * receive notifications and update their behavior accordingly.
 * 
 * @param config Pointer to the updated configuration
 */
typedef void (*DeviceConfig_ChangeCallback_t)(const DeviceConfig_t *config);

/* ========================== Public API ========================== */

/**
 * @brief Initialize the configuration module
 * 
 * Loads configuration from flash or initializes factory defaults if flash
 * is empty or corrupted. Must be called before any other config functions.
 * 
 * @return 0 on success, -1 on failure
 * 
 * @note Call this early in system initialization, before modules that depend on config
 */
int DeviceConfig_Init(void);

/**
 * @brief Get read-only pointer to current configuration
 * 
 * Returns a pointer to the active configuration. This pointer remains valid
 * for the lifetime of the application. The returned structure should not be
 * modified directly; use the setter functions instead.
 * 
 * @return Pointer to const configuration structure
 */
const DeviceConfig_t* DeviceConfig_Get(void);

/**
 * @brief Copy current configuration into caller-owned storage
 *
 * Use this when building a runtime patch so later validation is based on a
 * stable snapshot rather than a live internal pointer.
 *
 * @param config_out Destination for a full config copy
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_GetSnapshot(DeviceConfig_t *config_out);

/**
 * @brief Get calculated number of DMX channels needed
 * 
 * Calculates total DMX channels based on pixel count and format:
 * - RGB: 3 channels per pixel
 * - RGBW: 4 channels per pixel
 * - RGB16: 6 channels per pixel
 * 
 * @return Number of DMX channels
 */
uint16_t DeviceConfig_GetChannelCount(void);

/**
 * @brief Get number of DMX channels consumed per pixel
 * 
 * Determined by pixel_format: RGB=3, RGBW=4, RGB16=6.
 * 
 * @return Channels per pixel (3, 4, or 6)
 */
uint8_t DeviceConfig_GetChannelsPerPixel(void);

/**
 * @brief Get calculated number of Art-Net universes needed
 * 
 * Calculates universes needed based on channel count and start address.
 * Accounts for partial universe at start address offset.
 * 
 * @return Number of universes (1 to DEVICE_CONFIG_MAX_UNIVERSES)
 */
uint8_t DeviceConfig_GetUniverseCount(void);

/**
 * @brief Build the active Art-Net output Port-Address list for a config
 *
 * The current runtime model only supports a contiguous output range starting
 * at `dmx.artnet_start_universe`. This helper validates that model and builds
 * the resulting 15-bit Port-Address values.
 *
 * @param config Configuration to evaluate
 * @param port_addresses Output array sized for DEVICE_CONFIG_MAX_UNIVERSES
 * @param port_count Number of valid entries written to port_addresses
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_BuildArtNetOutputAddressList(const DeviceConfig_t *config,
                                             uint16_t *port_addresses,
                                             uint8_t *port_count);

/**
 * @brief Update device identity settings
 * 
 * Updates short and/or long name. Pass NULL to keep existing value.
 * Changes are saved to flash and callbacks are notified.
 * 
 * @param short_name New short name (max 17 chars) or NULL to keep existing
 * @param long_name New long name (max 63 chars) or NULL to keep existing
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_SetIdentity(const char *short_name, const char *long_name);

/**
 * @brief Update network configuration
 * 
 * @param network New network configuration
 * @return 0 on success, -1 on failure
 * 
 * @note Network changes may require restart to take effect
 */
int DeviceConfig_SetNetwork(const DeviceConfig_Network_t *network);

/**
 * @brief Update pixel configuration
 * 
 * @param pixel New pixel configuration
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_SetPixel(const DeviceConfig_Pixel_t *pixel);

/**
 * @brief Update DMX/Art-Net configuration
 * 
 * @param dmx New DMX configuration
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_SetDmx(const DeviceConfig_Dmx_t *dmx);

/**
 * @brief Update output behavior configuration
 * 
 * @param output New output configuration
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_SetOutput(const DeviceConfig_Output_t *output);

/**
 * @brief Initialize a patch structure to "no change"
 *
 * Zero-initializes all flags and payload fields so callers can set only the
 * fields they intend to modify.
 *
 * @param patch Patch structure to initialize
 */
void DeviceConfig_PatchInit(DeviceConfig_Patch_t *patch);

/**
 * @brief Apply a partial runtime update atomically
 *
 * Applies the patch to a candidate configuration, validates the full result,
 * then commits once so multi-field updates do not expose intermediate states.
 *
 * @param patch Partial update to apply
 * @param persist true to request flash persistence, false for runtime-only
 * @param change_mask Optional output mask of fields changed by the commit
 * @return 0 on success, -1 on failure
 */
int DeviceConfig_ApplyPatch(const DeviceConfig_Patch_t *patch, bool persist,
                            uint32_t *change_mask);

/**
 * @brief Reset configuration to factory defaults
 * 
 * Clears all user settings and restores factory defaults.
 * Changes are saved to flash and callbacks are notified.
 * 
 * @return 0 on success, -1 on failure
 * 
 * @note This is a destructive operation - all user settings will be lost
 */
int DeviceConfig_FactoryReset(void);

/**
 * @brief Save current configuration to flash
 * 
 * Normally called automatically by setter functions. Can be called manually
 * after batch updates to minimize flash writes.
 * 
 * @return 0 on success, -1 on failure (flash write error)
 */
int DeviceConfig_SaveToFlash(void);

/**
 * @brief Register a configuration change callback
 * 
 * Registered callbacks are invoked when any configuration setting changes.
 * Use this to update module behavior dynamically without polling.
 * 
 * @param callback Function to call on config changes
 * @return 0 on success, -1 if callback table is full
 * 
 * @note Maximum 4 callbacks can be registered
 */
int DeviceConfig_RegisterCallback(DeviceConfig_ChangeCallback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* INC_DEVICE_CONFIG_H_ */
