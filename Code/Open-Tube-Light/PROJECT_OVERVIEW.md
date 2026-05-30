# Open-Tube-Light

## Overview

Open-Tube-Light is an embedded lighting-control firmware project for the STM32H723VET6 microcontroller. The device receives external lighting data, converts it into pixel output, and drives an SK9822 LED strip in real time. The firmware combines STM32 HAL-based peripheral control, FreeRTOS task scheduling, and LwIP networking to build a compact lighting node suitable for Ethernet-based and DMX-based stage or installation workflows.

At a high level, the project acts as a protocol-aware bridge between lighting control data and a segmented LED fixture. The software is structured so that input handling, configuration, rendering, and physical output remain clearly separated.

## Main Capabilities

- Receives Art-Net lighting data over Ethernet
- Receives DMX512 lighting data over UART5 / RS-485-style signaling
- Drives an SK9822 LED strip through SPI1 with DMA
- Maps logical DMX segments onto physical LEDs
- Supports configurable DMX start address, Art-Net addressing, and segment layout
- Applies configurable failsafe behavior when control data is lost
- Shows live device status on an SSD1306 OLED connected over I2C

## Hardware and Software Stack

| Layer | Implementation |
| --- | --- |
| Target MCU | STM32H723VET6 |
| Peripheral framework | STM32CubeMX-generated HAL |
| RTOS | FreeRTOS via CMSIS-RTOS v2 |
| Network stack | LwIP |
| Network lighting protocol | Art-Net 4 |
| Wired lighting protocol | DMX512 |
| LED output | SK9822 over SPI + DMA |
| Local status interface | SSD1306 OLED over I2C |
| Local inputs | 4 hardware buttons defined in the board mapping |

## Runtime Architecture

The firmware is organized around two main application tasks:

1. `ControlTask` initializes the LwIP stack, starts the selected DMX input source, and owns the OLED status display.
2. `EffectTask` waits for incoming frame notifications, latches the latest control data, renders a pixel frame, and transmits it to the LED strip.

This split keeps control-plane responsibilities separate from the time-sensitive rendering loop.

### Data Flow

```text
Art-Net (Ethernet) or DMX512 (UART)
                ->
           dmx_input manager
                ->
          active protocol driver
                ->
            EffectTask latch
                ->
           pixel_render module
                ->
            SK9822 SPI driver
                ->
               LED strip
```

### Execution Sequence

1. `main()` configures the MPU, CPU caches, clocks, GPIO, DMA, SPI, UART, and I2C peripherals.
2. The device configuration module loads the active configuration and exposes derived settings such as channel count and universe count.
3. `ControlTask` starts the network stack and activates the configured DMX input source.
4. The active input driver notifies `EffectTask` whenever a new frame arrives.
5. `EffectTask` renders the frame into the SK9822 transfer buffer and sends it through SPI DMA.
6. If new data stops arriving, the firmware either refreshes the last output or applies the configured failsafe mode.

## Major Firmware Modules

| Module | Role |
| --- | --- |
| `Core/Src/main.c` | System startup, peripheral initialization, task creation, OLED status handling |
| `Core/Src/device_config.c` | Central configuration model for identity, network settings, layout, DMX addressing, and output policy |
| `Core/Src/dmx_input.c` | Protocol-independent input manager that selects the active driver |
| `Core/Src/Art-Net/artnet.c` | Core Art-Net receiver, UDP binding, buffering, and public API |
| `Core/Src/Art-Net/artnet_handlers.c` | Packet handling for ArtDmx, ArtPoll, ArtSync, and runtime command processing |
| `Core/Src/Art-Net/artnet_pollreply.c` | Construction and transmission of ArtPollReply discovery responses |
| `Core/Src/DMX512/dmx512_uart.c` | UART + DMA-based DMX512 receiver with break and idle detection |
| `Core/Src/pixel_render.c` | DMX-to-pixel mapping and output-frame generation |
| `Core/Src/SK9822/sk9822.c` | Ping-pong SPI DMA driver for the LED strip |
| `Core/Src/OLED/ssd1306.c` | OLED drawing and asynchronous screen transfer support |

## Input and Output Design

### Input Side

The project uses a driver abstraction for DMX sources. This allows the same rendering path to work regardless of whether frame data comes from Art-Net or DMX512. Each driver owns its own buffers and signals the effect task when a new frame is ready.

The Art-Net implementation supports multi-universe reception, discovery replies, and synchronized frame handling. The DMX512 implementation uses UART5 with DMA to reduce CPU load and detect packet boundaries using UART framing and IDLE events.

### Output Side

The pixel renderer converts incoming DMX channel data into logical segment colors and expands those logical segments across the physical LED count. The SK9822 driver then transmits the prepared frame using SPI DMA with ping-pong buffers, which keeps the output path efficient and stable.

## Memory Strategy

This project is explicitly organized around DMA-safe and protocol-safe memory placement:

- DMX frame buffers are placed in DTCM for fast CPU access
- SPI and UART DMA buffers are placed in D2 SRAM in non-cacheable regions
- LwIP heap and Ethernet RX pools are placed in a dedicated AXI RAM region
- MPU regions are configured so DMA-visible memory avoids cache coherency problems

This memory layout is important for an STM32H7-class device because Ethernet, SPI DMA, and UART DMA all interact with different RAM regions and cache behavior.

## Default Device Configuration

The firmware defines a factory-default operating profile in the configuration module:

| Setting | Default value |
| --- | --- |
| Device short name | `OpenTubeLight` |
| Device long name | `OpenTubeLight V0.1` |
| IP mode | DHCP |
| Pixel count | 144 |
| Segment count | 8 |
| Segment format | RGB |
| Input source | Art-Net |
| DMX start address | 1 |
| Art-Net Net / SubNet / Universe | 0 / 0 / 1 |
| Art-Net max refresh rate | 44 Hz |
| Failsafe mode | Hold last frame |

## Repository Structure

```text
Core/
  Inc/                 Public headers for application modules
  Src/                 Application source files and protocol modules
LWIP/                  Network integration and Ethernet interface code
Drivers/               STM32 HAL and CMSIS device support
Middlewares/           Third-party middleware such as FreeRTOS and LwIP
cmake/                 Toolchain and CubeMX-generated CMake integration
Open-Tube-Light.ioc    STM32CubeMX hardware configuration
```

## Build and Debug Workflow

The repository includes a CMake-based build flow and preset definitions for standard builds.

### Configure and Build

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Available presets:

- `Debug`
- `Release`

The CMake project builds a single firmware target named `Open-Tube-Light` and pulls in the STM32CubeMX-generated sources through `cmake/stm32cubemx`.

### Debugging

The repository also includes an ST-LINK launch configuration in `Open-Tube-Light Debug.launch`. This provides a ready-made debug entry point for loading and running the generated ELF on the target hardware.

## Summary

Open-Tube-Light is a resource-aware embedded firmware project for a networked LED fixture. Its design combines professional lighting protocols, RTOS-based task separation, DMA-backed LED output, and careful memory placement on STM32H7 hardware. The codebase is structured around a clear pipeline: receive control data, validate and latch it, render pixel values, and transmit them efficiently to the LED hardware.