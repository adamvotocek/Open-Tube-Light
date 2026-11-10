# Open-Tube-Light Firmware

STM32H723VET6 (Cortex-M7 @ 500 MHz / 250 MHz HCLK) LED tube fixture firmware using STM32CubeMX HAL, FreeRTOS (CMSIS-RTOS v2), and LwIP. Receives Art-Net (Ethernet; sACN planned, no driver yet) or DMX512 (UART5/RS-485), maps segments onto a 144-LED SK9822 strip (SPI1 + DMA), and displays status on an SSD1306 OLED (I2C1 + DMA) with 4 UI buttons.

> **Repo Root:** The git root is two levels up (`../../`), which also contains `PCB/`, `Enclosure/`, and QLC+ workspaces. Git paths are relative to that root.

## Build & Verification

Run from project root (requires STM32Cube VS Code extension `cube-cmake` on PATH):

```sh
cube-cmake --preset Debug          # Configure (run once or after CMakeLists.txt changes)
cube-cmake --build --preset Debug  # Build -> build/Debug/Open-Tube-Light.elf
```

* **Adding files:** Add new `.c` files to `target_sources` in the root `CMakeLists.txt`. Never edit `cmake/stm32cubemx/CMakeLists.txt`.
* **Verification Criteria:** There are no unit tests. A valid change must:
  1. Build cleanly with zero *new* compiler warnings (baseline: one existing `-Wunused-function` for `Config_ValidateFlash`).
  2. Fit within the linker memory-region table printed at build end (`RAM_D1_LWIP` is ~98% full by design).
* **Hardware Testing:** Flashing/debugging is done by the user via ST-Link. When a change requires bench verification, explicitly state what to test and which Live Watch counters to monitor (`render_count`/`failsafe_count`/`refresh_count` in `pixel_render.c`; `receivedData`/`smallFrameCnt`/`midPcktErrCnt` in `dmx512_uart.c`).

## CubeMX & Linker Boundaries

* **`Open-Tube-Light.ioc` is the source of truth** for clocks, peripherals, DMA, NVIC priorities, MPU regions, FreeRTOS tasks/stacks, and LwIP config.
* **Generated files** (`main.c`, `stm32h7xx_it.c`, `stm32h7xx_hal_msp.c`, `freertos.c`, `LWIP/**`, `Core/Inc/FreeRTOSConfig.h`): **ONLY** edit inside `/* USER CODE BEGIN */ ... /* USER CODE END */` blocks.
* **Need a config change?** Do not hand-edit generated config values. Read `.ioc` to inspect current settings, then stop and instruct the user on what to update in CubeMX and regenerate.
* **`STM32H723XX_FLASH.ld`** is hand-maintained and mostly safe to edit. Keep `docs/memory-layout.md` in sync with it and with the MPU/ETH/LwIP settings.

## Architecture & Module Map

```text
Art-Net (UDP, tcpip_thread) ─┐
                             ├─> dmx_input (vtable) ─> EffectTask ─> pixel_render ─> SK9822 SPI DMA
DMX512 (UART5 ISR + DMA) ────┘
```

* `Core/Src/main.c`: Startup, `ControlTaskRun`, `EffectTaskRun`, OLED status, HAL callbacks.
* `Core/Src/device_config.c/.h`: Config schema, defaults, validation, atomic patch/commit (`DeviceConfig_ApplyPatch`).
* `Core/Src/dmx_input.c/.h`: Protocol-agnostic input manager (`DMX_Input_Driver_t` vtable).
* `Core/Src/Art-Net/`: `artnet.c` (UDP/init), `artnet_handlers.c`, `artnet_pollreply.c`, `artnet_internal.h` (`g_artnet_ctx`).
* `Core/Src/DMX512/dmx512_uart.c`: Break via framing error, EoP via IDLE, lock-free double buffer.
* `Core/Src/pixel_render.c`: DMX -> segment -> LED mapping, failsafe, idle refresh.
* `Core/Src/SK9822/sk9822.c`: Ping-pong SPI DMA driver (`BeginFrame` / `SetLed` / `EndFrame` / `Refresh`).
* `Core/Src/OLED/ssd1306.c`: RTOS-aware async I2C DMA driver.

## Execution Contexts & Threading Rules

Always verify execution context before calling RTOS or LwIP APIs:

| Context | Priority | Responsibilities & Strict Constraints |
| --- | --- | --- |
| **UART5 ISR** | NVIC 3 (Above `configMAX_SYSCALL_INTERRUPT_PRIORITY` = 5) | Runs `DMX512_Uart_IRQHandler`. **NEVER call FreeRTOS APIs here.** Defer notifications by software-pending `DMA1_Stream1_IRQn` (`DMX512_Uart_DeferredNotify()`). Keep execution ultra-short. |
| **DMA/SPI/I2C ISRs** | NVIC 5 | HAL callbacks, `SK9822_OnTxComplete`. Non-blocking `osThreadFlagsSet` is allowed. |
| **`tcpip_thread`** | LwIP (2048B stack) | Runs Art-Net UDP callbacks & handlers. **Only context allowed to call raw LwIP APIs.** Other threads must use `tcpip_callback()`. Watch stack usage (no large stack buffers or heavy `snprintf`). |
| **`ControlTask`** | BelowNormal | Control plane: `MX_LWIP_Init`, input driver init, 1 Hz OLED refresh, config commits, UI. |
| **`EffectTask`** | AboveNormal | Real-time render path. Waits on flag `0x01` (150 ms timeout) after `ControlTask` sets start flag `0x02`. **Never mutate config here.** |

## Memory Placement & DMA Coherency

D-Cache is enabled. DMA cannot access DTCM (`0x20000000`). All DMA buffers **must** reside in MPU non-cacheable regions. Never use `malloc` at runtime; use static allocation. Explicitly zero `NOLOAD` sections during init.

* **`.DMX_Buffers` (DTCM `0x20000000`):** Fast CPU-only RAM. Used for DMX shadow/active/pending buffers. **No DMA allowed.**
* **`.d2_dma_nocache` (RAM_D2 `0x30000000`, 32 KB, MPU non-cacheable):** ETH DMA descriptors, `.spi_buffers`, `.uart_buffers`, `.i2c_buffers`.
* **`RAM_D1_LWIP` (`0x24000000`, 64 KB, MPU non-cacheable):** LwIP heap (`0x24000000`, 32 KB) and `.Rx_PoolSection` (`0x24008000`).
* **`RAM_D1` (`0x24010000`, 256 KB):** Cached AXI SRAM. Free for large CPU-only structures.

## Code Style & Conventions

* **Naming (Broad-to-Narrow):**
  * Functions: `Module_PascalCase` (e.g., `ArtNet_HandleArtDmx`, `UART_Read` not `Read_UART`).
  * Types: `Module_PascalCase_t` (e.g., `DeviceConfig_Patch_t`).
  * Variables: `snake_case` (e.g., `hdma_spi1_tx`).
  * Constants/Macros: `UPPER_SNAKE_CASE`.
* **File Layout:** Public headers in `Core/Inc/<Module>/x.h`, source in `Core/Src/<Module>/x.c`, private shared headers in `*_internal.h` (single-file modules like `device_config`, `dmx_input`, `pixel_render` sit directly in `Core/Src` / `Core/Inc`). Private functions are `static`: prototypes at the top of the file, definitions at the bottom, each under a `Private Functions` banner comment.
* **Safety & Scope:** Validate pointers and packet lengths at public/network boundaries. Return `0`/`-1` or `HAL_StatusTypeDef` consistently. Keep functions under ~100 lines with a single responsibility; don't invent premature abstractions.
* **Comments:** Explain what and *why* (timing constraints, MPU/DMA rules, spec citations). Use Doxygen `@brief` on public APIs.

## Docs, Specs & Project State

All documentation lives in `docs/` — never put `.md` files in `Core/` or other source folders.

* **`docs/reference/`** (gitignored, copyrighted — **never force-stage or commit**):
  * `Art-Net_4.md` (~330 KB): **Grep for specific opcodes/sections; NEVER read the whole file into context.**
  * `dmx512-A.md`: ANSI E1.11 §8–9 timings and packet layout.
  * `protocol-failsafes.md`, ST manuals (`UM2217` HAL/LL, `UM1713` LwIP) as PDFs.
* **`docs/plans/`** (gitignored): design drafts. `device-config-refactor.md` (config ownership, persistence, ArtAddress), `memory-tuning.md` (stacks, `snprintf` in `tcpip_thread`).
* **Tracked docs:**
  * `docs/artnet-custom-protocol.md`: contract with the companion app (NodeReport format, `ArtCommand` verbs, ArtTrigger locate). Change it together with the firmware.
  * `docs/dmx-input-drivers.md`: input driver vtable and how to add a protocol.
  * `docs/memory-layout.md`: RAM regions, linker sections, MPU/ETH/LwIP CubeMX settings.
  * `docs/roadmap.md`: stubs and planned features. `docs/issues.md`: open/solved bench-observed bugs.
* **Before multi-file feature work or debugging**, read `docs/roadmap.md` and `docs/issues.md`. Key gaps: config is never saved to flash, `ArtNet_HandleArtAddress` is a stub, ArtTrigger/sACN/RGBW/RGB16 are not implemented, and `device_config` network fields are not applied (LwIP uses the CubeMX static IP `192.168.1.179/24`).
* **Keep docs current:** when a change completes a roadmap item, fixes an issue, or alters something a doc describes, update that doc in the same change.