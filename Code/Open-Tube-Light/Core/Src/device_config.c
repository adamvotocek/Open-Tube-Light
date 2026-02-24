/**
 * @file device_config.c
 * @brief Device Configuration Management Implementation
 * 
 * Manages device configuration storage, factory defaults, and runtime updates.
 * Configuration is stored in flash for persistence across power cycles.
 * 
 * @section flash Flash Storage
 * 
 * Configuration is stored in a dedicated flash sector. The STM32H7 flash
 * requires sector-based erasing, so we use a simple strategy:
 * - Read config from flash at startup
 * - Validate magic number and checksum
 * - Keep working copy in RAM
 * - Write entire config on save (erase + program)
 * 
 * TODO: Implement actual flash read/write using HAL flash functions.
 * For now, configuration is initialized to factory defaults on each boot.
 * 
 * @section callbacks Callbacks
 * 
 * Modules can register callbacks to be notified when configuration changes.
 * This allows dynamic reconfiguration without polling or restart.
 */

#include "device_config.h"
#include "main.h"
#include <string.h>

/* ========================== Private Constants ========================== */

/** @brief Maximum number of change callbacks */
#define MAX_CALLBACKS   4

/* ========================== Factory Defaults ========================== */

/**
 * @brief Factory default configuration
 * 
 * These values are used on first boot or after factory reset.
 * Modify these defaults to match your application requirements.
 */
static const DeviceConfig_t factory_defaults = {
    .magic = DEVICE_CONFIG_MAGIC,
    .version = DEVICE_CONFIG_VERSION,
    .reserved = 0,
    
    .identity = {
        .short_name = "OpenTubeLight",
        .long_name = "OpenTubeLight V0.1",
    },
    
    .network = {
        .ip_mode = IP_MODE_DHCP,
        .static_ip = {192, 168, 1, 100},
        .static_mask = {255, 255, 255, 0},
        .static_gateway = {192, 168, 1, 1},
    },
    
    .pixel = {
        .pixel_count = 288,
        .pixel_format = PIXEL_FORMAT_RGB,
    },
    
    .dmx = {
        .input_source = DMX_INPUT_ARTNET,
        .dmx_start_address = 1,
        .artnet_net = 0,
        .artnet_subnet = 0,
        .artnet_start_universe = 1,
        .artnet_max_refresh_rate = 44,
    },
    
    .output = {
        .failsafe = FAILSAFE_HOLD,
    },
    
    .checksum = 0,  // Will be calculated on save
};

/* ========================== Private Variables ========================== */

/** @brief Active configuration in RAM */
static DeviceConfig_t active_config;

/** @brief Initialization state */
static bool initialized = false;

/** @brief Registered change callbacks */
static DeviceConfig_ChangeCallback_t callbacks[MAX_CALLBACKS] = {0};

/** @brief Number of registered callbacks */
static uint8_t callback_count = 0;

/* ========================== Private Function Prototypes ========================== */

static uint32_t Config_CalculateChecksum(const DeviceConfig_t *config);
static bool Config_ValidateFlash(const DeviceConfig_t *config);
static void Config_NotifyCallbacks(void);
static int Config_LoadFromFlash(void);

/* ========================== Public API Implementation ========================== */

int DeviceConfig_Init(void)
{
    if (initialized) {
        return 0;  // Already initialized
    }
    
    // Try to load config from flash
    if (Config_LoadFromFlash() != 0) {
        // Flash config invalid or empty - use factory defaults
        memcpy(&active_config, &factory_defaults, sizeof(DeviceConfig_t));
        active_config.checksum = Config_CalculateChecksum(&active_config);
        
        // TODO: Save defaults to flash on first boot
        // DeviceConfig_SaveToFlash();
    }
    
    initialized = true;
    return 0;
}

const DeviceConfig_t* DeviceConfig_Get(void)
{
    return &active_config;
}

uint16_t DeviceConfig_GetChannelCount(void)
{
    uint16_t channels_per_pixel;
    
    switch (active_config.pixel.pixel_format) {
        case PIXEL_FORMAT_RGBW:
            channels_per_pixel = 4;
            break;
        case PIXEL_FORMAT_RGB16:
            channels_per_pixel = 6;
            break;
        case PIXEL_FORMAT_RGB:
        default:
            channels_per_pixel = 3;
            break;
    }
    
    return active_config.pixel.pixel_count * channels_per_pixel;
}

uint8_t DeviceConfig_GetUniverseCount(void)
{
    uint16_t total_channels = DeviceConfig_GetChannelCount();
    uint16_t start_addr = active_config.dmx.dmx_start_address;
    
    // Calculate how many channels fit in first universe (512 - start_address + 1)
    uint16_t first_universe_capacity = 513 - start_addr;
    
    if (total_channels <= first_universe_capacity) {
        return 1;
    }
    
    // Remaining channels after first universe
    uint16_t remaining = total_channels - first_universe_capacity;
    
    // Additional universes needed (512 channels each)
    uint8_t additional = (remaining + 511) / 512;
    
    uint8_t total = 1 + additional;
    
    // Clamp to maximum
    if (total > DEVICE_CONFIG_MAX_UNIVERSES) {
        total = DEVICE_CONFIG_MAX_UNIVERSES;
    }
    
    return total;
}

int DeviceConfig_SetIdentity(const char *short_name, const char *long_name)
{
    if (!initialized) {
        return -1;
    }
    
    if (short_name != NULL) {
        strncpy(active_config.identity.short_name, short_name, 
                DEVICE_CONFIG_SHORT_NAME_LEN - 1);
        active_config.identity.short_name[DEVICE_CONFIG_SHORT_NAME_LEN - 1] = '\0';
    }
    
    if (long_name != NULL) {
        strncpy(active_config.identity.long_name, long_name, 
                DEVICE_CONFIG_LONG_NAME_LEN - 1);
        active_config.identity.long_name[DEVICE_CONFIG_LONG_NAME_LEN - 1] = '\0';
    }
    
    // Update checksum and save
    active_config.checksum = Config_CalculateChecksum(&active_config);
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_SetNetwork(const DeviceConfig_Network_t *network)
{
    if (!initialized || network == NULL) {
        return -1;
    }
    
    memcpy(&active_config.network, network, sizeof(DeviceConfig_Network_t));
    
    active_config.checksum = Config_CalculateChecksum(&active_config);
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_SetPixel(const DeviceConfig_Pixel_t *pixel)
{
    if (!initialized || pixel == NULL) {
        return -1;
    }
    
    // Validate pixel count
    if (pixel->pixel_count > DEVICE_CONFIG_MAX_PIXELS) {
        return -1;
    }
    
    memcpy(&active_config.pixel, pixel, sizeof(DeviceConfig_Pixel_t));
    
    active_config.checksum = Config_CalculateChecksum(&active_config);
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_SetDmx(const DeviceConfig_Dmx_t *dmx)
{
    if (!initialized || dmx == NULL) {
        return -1;
    }
    
    // Validate DMX start address (1-512)
    if (dmx->dmx_start_address < 1 || dmx->dmx_start_address > 512) {
        return -1;
    }
    
    // Validate Art-Net addressing
    if (dmx->artnet_net > 127 || dmx->artnet_subnet > 15 || dmx->artnet_start_universe > 15) {
        return -1;
    }
    
    memcpy(&active_config.dmx, dmx, sizeof(DeviceConfig_Dmx_t));
    
    active_config.checksum = Config_CalculateChecksum(&active_config);
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_SetOutput(const DeviceConfig_Output_t *output)
{
    if (!initialized || output == NULL) {
        return -1;
    }
    
    memcpy(&active_config.output, output, sizeof(DeviceConfig_Output_t));
    
    active_config.checksum = Config_CalculateChecksum(&active_config);
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_FactoryReset(void)
{
    if (!initialized) {
        return -1;
    }
    
    // Restore factory defaults
    memcpy(&active_config, &factory_defaults, sizeof(DeviceConfig_t));
    active_config.checksum = Config_CalculateChecksum(&active_config);
    
    Config_NotifyCallbacks();
    
    return DeviceConfig_SaveToFlash();
}

int DeviceConfig_SaveToFlash(void)
{
    // TODO: Implement flash storage
    // For STM32H7:
    // 1. Unlock flash: HAL_FLASH_Unlock()
    // 2. Erase sector: HAL_FLASHEx_Erase()
    // 3. Program: HAL_FLASH_Program() (32-byte flash words on H7)
    // 4. Lock flash: HAL_FLASH_Lock()
    // 5. Verify written data
    
    // For now, return success (config is volatile - lost on power cycle)
    return 0;
}

int DeviceConfig_RegisterCallback(DeviceConfig_ChangeCallback_t callback)
{
    if (callback == NULL || callback_count >= MAX_CALLBACKS) {
        return -1;
    }
    
    callbacks[callback_count++] = callback;
    return 0;
}

/* ========================== Private Function Implementation ========================== */

/**
 * @brief Calculate CRC32 checksum for configuration data
 * 
 * Simple checksum calculation for config integrity verification.
 * Excludes the checksum field itself from calculation.
 * 
 * @param config Configuration to checksum
 * @return Calculated checksum value
 */
static uint32_t Config_CalculateChecksum(const DeviceConfig_t *config)
{
    // Simple checksum: sum all bytes except the checksum field
    const uint8_t *data = (const uint8_t *)config;
    size_t len = offsetof(DeviceConfig_t, checksum);
    
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
        sum = (sum << 1) | (sum >> 31);  // Rotate left
    }
    
    return sum ^ 0x5A5A5A5A;  // XOR with constant for better distribution
}

/**
 * @brief Validate configuration loaded from flash
 * 
 * Checks magic number, version, and checksum.
 * 
 * @param config Configuration to validate
 * @return true if valid, false otherwise
 */
static bool Config_ValidateFlash(const DeviceConfig_t *config)
{
    if (config->magic != DEVICE_CONFIG_MAGIC) {
        return false;
    }
    
    if (config->version != DEVICE_CONFIG_VERSION) {
        // Future: implement migration for older versions
        return false;
    }
    
    uint32_t calculated = Config_CalculateChecksum(config);
    if (calculated != config->checksum) {
        return false;
    }
    
    return true;
}

/**
 * @brief Notify all registered callbacks of configuration change
 */
static void Config_NotifyCallbacks(void)
{
    for (uint8_t i = 0; i < callback_count; i++) {
        if (callbacks[i] != NULL) {
            callbacks[i](&active_config);
        }
    }
}

/**
 * @brief Load configuration from flash storage
 * 
 * @return 0 if valid config loaded, -1 if flash empty or corrupted
 */
static int Config_LoadFromFlash(void)
{
    // TODO: Implement flash read
    // For STM32H7:
    // 1. Define flash address for config sector
    // 2. Read config structure from flash
    // 3. Validate with Config_ValidateFlash()
    // 4. Copy to active_config if valid
    
    // For now, always return failure to use factory defaults
    return -1;
}
