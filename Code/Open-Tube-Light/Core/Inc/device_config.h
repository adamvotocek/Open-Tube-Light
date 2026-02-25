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
#define DEVICE_CONFIG_MAX_PIXELS        144

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
    DMX_INPUT_DMX512 = 1,   ///< DMX512 over UART
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
