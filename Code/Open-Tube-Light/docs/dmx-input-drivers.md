# DMX Input Drivers

How the protocol-agnostic input layer works and how to add a new source (e.g. sACN) under the `DMX_Input_Driver_t` abstraction.

## Architecture

```
┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐
│ EffectTask       │────▶│  dmx_input.c     │────▶│  artnet.c        │  DMX_INPUT_ARTNET
│  (main.c) Latch  │     │  (manager,       │     │  dmx512_uart.c   │  DMX_INPUT_DMX512
│ pixel_render.c   │     │   vtable)        │     │  sacn.c          │  DMX_INPUT_SACN (not implemented)
│  GetUniverse     │     │                  │     │                  │
└──────────────────┘     └──────────────────┘     └──────────────────┘
```

- **Consumers:** `EffectTask` calls `DMX_Input_Latch()` once per frame, then `pixel_render.c` reads data via `DMX_Input_GetUniverse()`.
- **Manager** (`dmx_input.c`) resolves the driver from `config->dmx.input_source` and dispatches through the vtable. With no active driver, `DMX_Input_GetFailsafeTimeout()` returns a conservative 12000 ms.
- **Each driver** owns its buffers, its synchronisation and its transport, and wakes `EffectTask` with thread flag `0x01` when a new frame is ready.

| Driver | Sync between producer and `Latch()` | Producer context | Failsafe timeout |
|---|---|---|---|
| Art-Net | Priority-inheritance mutex around shadow buffer | `tcpip_thread` (UDP callback) | 5000 ms |
| DMX512 | Lock-free double buffer (`pending_buf[2]` + `write_idx`) | UART5 ISR, NVIC 3 — **above** BASEPRI | 1250 ms (`DMX512_FAILSAFE_TIMEOUT_MS`) |
| sACN | — | — | 2500 ms per E1.31 |

## Adding a driver

### 1. Create the files

`Core/Src/YourProto/yourproto.c`, `Core/Inc/YourProto/yourproto.h`.

### 2. Implement the vtable

```c
// yourproto.c
#include "dmx_input.h"
#include "device_config.h"
#include "cmsis_os.h"
#include <string.h>

// Double-buffered DMX data in DTCM (.DMX_Buffers → .dtcm_dmx). CPU-only: no DMA may touch these.
static uint8_t shadow_buf[DEVICE_CONFIG_MAX_UNIVERSES][DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));
static uint8_t active_buf[DEVICE_CONFIG_MAX_UNIVERSES][DMX_UNIVERSE_MAX_LENGTH]
    __attribute__((section(".DMX_Buffers"), aligned(32)));

static osMutexId_t shadow_mutex;
static osThreadId_t notify_task;
static bool initialized;

static int YourProto_Init(osThreadId_t task) {
    if (initialized) return 0;
    notify_task = task;
    // Create mutex (osMutexPrioInherit), bind transport, etc.
    // On partial failure: release everything created before the failure point.
    initialized = true;
    return 0;
}

static void YourProto_Deinit(void) {
    if (!initialized) return;
    // Release transport, delete mutex, zero buffers
    memset(shadow_buf, 0, sizeof(shadow_buf));
    memset(active_buf, 0, sizeof(active_buf));
    notify_task = NULL;
    initialized = false;
}

static void YourProto_Latch(void) {
    uint8_t n = DeviceConfig_GetUniverseCount();
    osMutexAcquire(shadow_mutex, osWaitForever);
    memcpy(active_buf, shadow_buf, n * DMX_UNIVERSE_MAX_LENGTH);
    osMutexRelease(shadow_mutex);
}

static const uint8_t* YourProto_GetUniverse(uint8_t universe) {
    if (universe >= DeviceConfig_GetUniverseCount()) return NULL;
    return active_buf[universe];
}

const DMX_Input_Driver_t dmx_input_yourproto = {
    .init                = YourProto_Init,
    .deinit              = YourProto_Deinit,
    .latch               = YourProto_Latch,
    .get_universe        = YourProto_GetUniverse,
    .failsafe_timeout_ms = 2500,  // Per protocol spec, see table above
};
```

### 3. Deliver new data

```c
osMutexAcquire(shadow_mutex, osWaitForever);
memcpy(shadow_buf[universe_idx], incoming_data, len);
osMutexRelease(shadow_mutex);

osThreadFlagsSet(notify_task, 0x01);  // frame ready
```

- **Network (LwIP):** the UDP receive callback already runs in `tcpip_thread`; writing the shadow buffer there is fine.
- **ISR at or below BASEPRI (NVIC ≥ 5):** no mutex — use a lock-free double buffer; `osThreadFlagsSet` is allowed.
- **ISR above BASEPRI (NVIC < 5, e.g. UART5 at 3):** no FreeRTOS calls at all. Copy the DMX512 pattern: lock-free double buffer plus a software-pended lower-priority IRQ that performs the notify (`DMX512_Uart_DeferredNotify()`).

### 4. Register the driver

- `Core/Inc/dmx_input.h` — `extern const DMX_Input_Driver_t dmx_input_yourproto;`
- `Core/Src/dmx_input.c` — add `case DMX_INPUT_YOURPROTO: return &dmx_input_yourproto;` in `DMX_Input_ResolveDriver()`
- `Core/Inc/device_config.h` — `DMX_INPUT_SACN` already exists; for anything else append to `DeviceConfig_DmxInput_t` and extend validation in `device_config.c`.

### 5. Add to the build

Add the `.c` file to `target_sources` in the root `CMakeLists.txt`.

## Constraints

| Rule | Why |
|---|---|
| Buffers in `.DMX_Buffers`, 32-byte aligned | DTCM: zero-wait-state CPU access |
| Mutex with `osMutexPrioInherit` (task producers only) | Avoids priority inversion between `tcpip_thread` and `EffectTask` |
| Clean up on partial init failure | No runtime `malloc`; leaked RTOS objects can't be recovered |
| `deinit` zeros buffers | Lights go dark on protocol switch |
| Correct `failsafe_timeout_ms` | `EffectTask` uses it to detect a lost controller |
| LwIP calls from non-tcpip threads via `tcpip_callback()` | LwIP core is not thread-safe |
| Universe count from `DeviceConfig_GetUniverseCount()` | Derived from segment count, format and start address |

## Config values relevant to drivers (`DeviceConfig_Get()`)

- `dmx.input_source` — selects the driver
- `dmx.dmx_start_address` — 1-based start channel (all protocols)
- `dmx.artnet_net`, `dmx.artnet_subnet`, `dmx.artnet_start_universe` — Art-Net only
- `identity.short_name`, `identity.long_name` — discovery replies
- `output.failsafe` — `FAILSAFE_HOLD` / `FAILSAFE_ZERO` / `FAILSAFE_FULL` (applied by `pixel_render`, not the driver)
