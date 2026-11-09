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
#include "Art-Net/artnet_protocol.h"
#include "main.h"
#include <stddef.h>
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
    
    .layout = {
        .pixel_count = 144,
        .segment_count = 144,
        .segment_format = SEGMENT_FORMAT_RGB,
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
static size_t Config_StringLengthBounded(const char *text, size_t max_len);
static void Config_CopyString(char *dst, size_t dst_len, const char *src);
static bool Config_IsSupportedSegmentCount(uint16_t pixel_count,
                                           uint16_t segment_count);
static uint8_t Config_GetChannelsPerSegmentFor(const DeviceConfig_t *config);
static uint16_t Config_GetChannelCountFor(const DeviceConfig_t *config);
static uint8_t Config_GetUniverseCountFor(const DeviceConfig_t *config, bool *overflow);
static int Config_ValidateCandidate(const DeviceConfig_t *config);
static uint32_t Config_CalculateChangeMask(const DeviceConfig_t *before,
                                           const DeviceConfig_t *after);
static void Config_ApplyPatchToCandidate(DeviceConfig_t *candidate,
                                         const DeviceConfig_Patch_t *patch);
static int Config_CommitCandidate(const DeviceConfig_t *candidate,
                                  uint32_t change_mask, bool persist);
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

int DeviceConfig_GetSnapshot(DeviceConfig_t *config_out)
{
    if (!initialized || config_out == NULL) {
        return -1;
    }

    memcpy(config_out, &active_config, sizeof(DeviceConfig_t));
    return 0;
}

uint8_t DeviceConfig_GetChannelsPerSegment(void)
{
    return Config_GetChannelsPerSegmentFor(&active_config);
}

uint16_t DeviceConfig_GetChannelCount(void)
{
    return Config_GetChannelCountFor(&active_config);
}

uint8_t DeviceConfig_GetUniverseCount(void)
{
    return Config_GetUniverseCountFor(&active_config, NULL);
}

int DeviceConfig_BuildArtNetOutputAddressList(const DeviceConfig_t *config,
                                             uint16_t *port_addresses,
                                             uint8_t *port_count)
{
    if (config == NULL || port_addresses == NULL || port_count == NULL) {
        return -1;
    }

    if (Config_ValidateCandidate(config) != 0) {
        return -1;
    }

    // The current node model exposes one logical output stream that spans a
    // contiguous Art-Net universe range. ArtAddress support builds on this
    // helper instead of trying to represent four unrelated SwOut values.
    uint8_t count = Config_GetUniverseCountFor(config, NULL);
    for (uint8_t i = 0; i < count; i++) {
        port_addresses[i] = ARTNET_BUILD_PORT_ADDRESS(config->dmx.artnet_net,
                                                      config->dmx.artnet_subnet,
                                                      config->dmx.artnet_start_universe + i);
    }

    *port_count = count;
    return 0;
}

void DeviceConfig_PatchInit(DeviceConfig_Patch_t *patch)
{
    if (patch == NULL) {
        return;
    }

    memset(patch, 0, sizeof(DeviceConfig_Patch_t));
}

int DeviceConfig_ApplyPatch(const DeviceConfig_Patch_t *patch, bool persist,
                            uint32_t *change_mask)
{
    if (!initialized || patch == NULL) {
        return -1;
    }

    // Stage multi-field updates in a local candidate first so one ArtAddress
    // packet can change several related fields without exposing intermediate
    // runtime states between individual setter-style operations.
    DeviceConfig_t candidate = active_config;
    Config_ApplyPatchToCandidate(&candidate, patch);

    if (Config_ValidateCandidate(&candidate) != 0) {
        return -1;
    }

    uint32_t local_change_mask = Config_CalculateChangeMask(&active_config, &candidate);
    if (change_mask != NULL) {
        *change_mask = local_change_mask;
    }

    if (local_change_mask == DEVICE_CONFIG_CHANGE_NONE) {
        return 0;
    }

    return Config_CommitCandidate(&candidate, local_change_mask, persist);
}

int DeviceConfig_SetIdentity(const char *short_name, const char *long_name)
{
    DeviceConfig_Patch_t patch;

    DeviceConfig_PatchInit(&patch);

    if (short_name != NULL) {
        patch.short_name_valid = true;
        Config_CopyString(patch.short_name, sizeof(patch.short_name), short_name);
    }

    if (long_name != NULL) {
        patch.long_name_valid = true;
        Config_CopyString(patch.long_name, sizeof(patch.long_name), long_name);
    }

    return DeviceConfig_ApplyPatch(&patch, true, NULL);
}

int DeviceConfig_SetNetwork(const DeviceConfig_Network_t *network)
{
    DeviceConfig_Patch_t patch;

    if (network == NULL) {
        return -1;
    }

    DeviceConfig_PatchInit(&patch);
    patch.network_valid = true;
    memcpy(&patch.network, network, sizeof(DeviceConfig_Network_t));

    return DeviceConfig_ApplyPatch(&patch, true, NULL);
}

int DeviceConfig_SetLayout(const DeviceConfig_Layout_t *layout)
{
    DeviceConfig_Patch_t patch;

    if (layout == NULL) {
        return -1;
    }

    DeviceConfig_PatchInit(&patch);
    patch.layout_valid = true;
    memcpy(&patch.layout, layout, sizeof(DeviceConfig_Layout_t));

    return DeviceConfig_ApplyPatch(&patch, true, NULL);
}

int DeviceConfig_SetDmx(const DeviceConfig_Dmx_t *dmx)
{
    DeviceConfig_Patch_t patch;

    if (dmx == NULL) {
        return -1;
    }

    DeviceConfig_PatchInit(&patch);
    patch.input_source_valid = true;
    patch.input_source = dmx->input_source;
    patch.dmx_start_address_valid = true;
    patch.dmx_start_address = dmx->dmx_start_address;
    patch.artnet_net_valid = true;
    patch.artnet_net = dmx->artnet_net;
    patch.artnet_subnet_valid = true;
    patch.artnet_subnet = dmx->artnet_subnet;
    patch.artnet_start_universe_valid = true;
    patch.artnet_start_universe = dmx->artnet_start_universe;
    patch.artnet_max_refresh_rate_valid = true;
    patch.artnet_max_refresh_rate = dmx->artnet_max_refresh_rate;

    return DeviceConfig_ApplyPatch(&patch, true, NULL);
}

int DeviceConfig_SetOutput(const DeviceConfig_Output_t *output)
{
    DeviceConfig_Patch_t patch;

    if (output == NULL) {
        return -1;
    }

    DeviceConfig_PatchInit(&patch);
    patch.failsafe_valid = true;
    patch.failsafe = output->failsafe;

    return DeviceConfig_ApplyPatch(&patch, true, NULL);
}

int DeviceConfig_FactoryReset(void)
{
    if (!initialized) {
        return -1;
    }

    if (Config_ValidateCandidate(&factory_defaults) != 0) {
        return -1;
    }

    return Config_CommitCandidate(&factory_defaults,
                                  Config_CalculateChangeMask(&active_config,
                                                             &factory_defaults),
                                  true);
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

static size_t Config_StringLengthBounded(const char *text, size_t max_len)
{
    size_t len = 0;

    if (text == NULL) {
        return 0;
    }

    while (len < max_len && text[len] != '\0') {
        len++;
    }

    return len;
}

static void Config_CopyString(char *dst, size_t dst_len, const char *src)
{
    size_t copy_len = 0;

    if (dst == NULL || dst_len == 0U) {
        return;
    }

    memset(dst, 0, dst_len);
    if (src == NULL) {
        return;
    }

    copy_len = Config_StringLengthBounded(src, dst_len - 1U);
    memcpy(dst, src, copy_len);
}

static bool Config_IsSupportedSegmentCount(uint16_t pixel_count,
                                           uint16_t segment_count)
{
    if (pixel_count == 0U || segment_count == 0U) {
        return false;
    }

    if (segment_count > pixel_count || ((pixel_count % segment_count) != 0U)) {
        return false;
    }

    // The current proof-of-concept fixture is a fixed 144-pixel product.
    // Keep the supported personalities centralized here so future 288-pixel
    // hardware can extend this policy without rewriting the render path.
    switch (pixel_count) {
        case 144U:
            switch (segment_count) {
                case 144U:
                case 72U:
                case 48U:
                case 36U:
                case 24U:
                case 18U:
                case 16U:
                case 12U:
                case 9U:
                case 8U:
                case 6U:
                case 4U:
                case 3U:
                case 2U:
                case 1U:
                    return true;
                default:
                    return false;
            }

        default:
            return false;
    }
}

static uint8_t Config_GetChannelsPerSegmentFor(const DeviceConfig_t *config)
{
    switch (config->layout.segment_format) {
        case SEGMENT_FORMAT_RGBW:  return 4;
        case SEGMENT_FORMAT_RGB16: return 6;
        case SEGMENT_FORMAT_RGB:
        default:                 return 3;
    }
}

static uint16_t Config_GetChannelCountFor(const DeviceConfig_t *config)
{
    return config->layout.segment_count * Config_GetChannelsPerSegmentFor(config);
}

static uint8_t Config_GetUniverseCountFor(const DeviceConfig_t *config, bool *overflow)
{
    uint16_t total_channels = Config_GetChannelCountFor(config);
    uint16_t start_addr = config->dmx.dmx_start_address;
    uint16_t first_universe_capacity = 513U - start_addr;
    uint16_t total_universes = 1U;

    if (total_channels > first_universe_capacity) {
        uint16_t remaining = total_channels - first_universe_capacity;
        total_universes += (remaining + 511U) / 512U;
    }

    // Validation needs to know whether a candidate exceeds the supported
    // runtime universe window, while the public getter still returns a bounded
    // value for existing read-side callers.
    if (overflow != NULL) {
        *overflow = (total_universes > DEVICE_CONFIG_MAX_UNIVERSES);
    }

    if (total_universes > DEVICE_CONFIG_MAX_UNIVERSES) {
        total_universes = DEVICE_CONFIG_MAX_UNIVERSES;
    }

    return (uint8_t)total_universes;
}

static int Config_ValidateCandidate(const DeviceConfig_t *config)
{
    bool universe_overflow = false;
    uint8_t universe_count;

    if (config == NULL) {
        return -1;
    }

    if (Config_StringLengthBounded(config->identity.short_name,
                                   DEVICE_CONFIG_SHORT_NAME_LEN) >= DEVICE_CONFIG_SHORT_NAME_LEN) {
        return -1;
    }

    if (Config_StringLengthBounded(config->identity.long_name,
                                   DEVICE_CONFIG_LONG_NAME_LEN) >= DEVICE_CONFIG_LONG_NAME_LEN) {
        return -1;
    }

    if (config->network.ip_mode != IP_MODE_DHCP &&
        config->network.ip_mode != IP_MODE_STATIC) {
        return -1;
    }

    if (config->layout.segment_format != SEGMENT_FORMAT_RGB &&
        config->layout.segment_format != SEGMENT_FORMAT_RGBW &&
        config->layout.segment_format != SEGMENT_FORMAT_RGB16) {
        return -1;
    }

    if (config->layout.pixel_count > DEVICE_CONFIG_MAX_PIXELS) {
        return -1;
    }

    if (!Config_IsSupportedSegmentCount(config->layout.pixel_count,
                                        config->layout.segment_count)) {
        return -1;
    }

    if (config->dmx.input_source != DMX_INPUT_ARTNET &&
        config->dmx.input_source != DMX_INPUT_SACN &&
        config->dmx.input_source != DMX_INPUT_DMX512) {
        return -1;
    }

    if (config->dmx.dmx_start_address < 1U || config->dmx.dmx_start_address > 512U) {
        return -1;
    }

    if (config->dmx.artnet_net > 127U ||
        config->dmx.artnet_subnet > 15U ||
        config->dmx.artnet_start_universe > 15U) {
        return -1;
    }

    if (config->dmx.artnet_max_refresh_rate < 44U) {
        return -1;
    }

    if (config->output.failsafe != FAILSAFE_HOLD &&
        config->output.failsafe != FAILSAFE_FULL &&
        config->output.failsafe != FAILSAFE_ZERO) {
        return -1;
    }

    universe_count = Config_GetUniverseCountFor(config, &universe_overflow);
    if (universe_overflow) {
        return -1;
    }

    // The current Art-Net implementation only supports a contiguous output
    // range within one subnet low-nibble window. Reject rollover across
    // universe 15 until the config model grows into a true per-port layout.
    if (((uint16_t)config->dmx.artnet_start_universe + universe_count) > 16U) {
        return -1;
    }

    return 0;
}

static uint32_t Config_CalculateChangeMask(const DeviceConfig_t *before,
                                           const DeviceConfig_t *after)
{
    uint32_t change_mask = DEVICE_CONFIG_CHANGE_NONE;

    if (memcmp(&before->identity, &after->identity, sizeof(DeviceConfig_Identity_t)) != 0) {
        change_mask |= DEVICE_CONFIG_CHANGE_IDENTITY;
    }

    if (memcmp(&before->network, &after->network, sizeof(DeviceConfig_Network_t)) != 0) {
        change_mask |= DEVICE_CONFIG_CHANGE_NETWORK;
    }

    if (memcmp(&before->layout, &after->layout, sizeof(DeviceConfig_Layout_t)) != 0) {
        change_mask |= DEVICE_CONFIG_CHANGE_LAYOUT;
    }

    if (before->dmx.input_source != after->dmx.input_source) {
        change_mask |= DEVICE_CONFIG_CHANGE_DMX_SOURCE;
    }

    if (before->dmx.dmx_start_address != after->dmx.dmx_start_address) {
        change_mask |= DEVICE_CONFIG_CHANGE_DMX_START_ADDRESS;
    }

    if (before->dmx.artnet_net != after->dmx.artnet_net ||
        before->dmx.artnet_subnet != after->dmx.artnet_subnet ||
        before->dmx.artnet_start_universe != after->dmx.artnet_start_universe) {
        change_mask |= DEVICE_CONFIG_CHANGE_ARTNET_ADDRESS;
    }

    if (before->dmx.artnet_max_refresh_rate != after->dmx.artnet_max_refresh_rate) {
        change_mask |= DEVICE_CONFIG_CHANGE_DMX_REFRESH_RATE;
    }

    if (before->output.failsafe != after->output.failsafe) {
        change_mask |= DEVICE_CONFIG_CHANGE_OUTPUT_FAILSAFE;
    }

    return change_mask;
}

static void Config_ApplyPatchToCandidate(DeviceConfig_t *candidate,
                                         const DeviceConfig_Patch_t *patch)
{
    // Each *_valid flag distinguishes "no change" from a real update whose
    // value may legitimately be zero, empty, or the current enum default.
    if (patch->short_name_valid) {
        Config_CopyString(candidate->identity.short_name,
                          sizeof(candidate->identity.short_name),
                          patch->short_name);
    }

    if (patch->long_name_valid) {
        Config_CopyString(candidate->identity.long_name,
                          sizeof(candidate->identity.long_name),
                          patch->long_name);
    }

    if (patch->network_valid) {
        memcpy(&candidate->network, &patch->network, sizeof(DeviceConfig_Network_t));
    }

    if (patch->layout_valid) {
        memcpy(&candidate->layout, &patch->layout, sizeof(DeviceConfig_Layout_t));
    }

    if (patch->input_source_valid) {
        candidate->dmx.input_source = patch->input_source;
    }

    if (patch->dmx_start_address_valid) {
        candidate->dmx.dmx_start_address = patch->dmx_start_address;
    }

    if (patch->artnet_net_valid) {
        candidate->dmx.artnet_net = patch->artnet_net;
    }

    if (patch->artnet_subnet_valid) {
        candidate->dmx.artnet_subnet = patch->artnet_subnet;
    }

    if (patch->artnet_start_universe_valid) {
        candidate->dmx.artnet_start_universe = patch->artnet_start_universe;
    }

    if (patch->artnet_max_refresh_rate_valid) {
        candidate->dmx.artnet_max_refresh_rate = patch->artnet_max_refresh_rate;
    }

    if (patch->failsafe_valid) {
        candidate->output.failsafe = patch->failsafe;
    }
}

static int Config_CommitCandidate(const DeviceConfig_t *candidate,
                                  uint32_t change_mask, bool persist)
{
    memcpy(&active_config, candidate, sizeof(DeviceConfig_t));
    active_config.magic = DEVICE_CONFIG_MAGIC;
    active_config.version = DEVICE_CONFIG_VERSION;
    active_config.checksum = Config_CalculateChecksum(&active_config);

    // Keep runtime commit and flash persistence as separate steps. ArtAddress
    // can use this path in volatile mode now, and later enable persistence
    // without having to redesign the update flow.
    (void)change_mask;
    Config_NotifyCallbacks();

    if (persist) {
        return DeviceConfig_SaveToFlash();
    }

    return 0;
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
