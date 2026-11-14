# Memory Management Follow-Up

## Current Status

- `TCPIP_THREAD_STACKSIZE` was increased from `1024` to `2048` (CubeMX → `LWIP/Target/lwipopts.h`), and the reported tcpip-thread overflow stopped. The value is passed to `osThreadNew` as **bytes**, so tcpip_thread has 2048 B. Treat it as a stabilization step, not a final budget.
- Done since: `stack_monitor` watermarks (1 Hz, Live Watch) and `snprintf` removed from the ArtPollReply path. Items 2 and 4 below are kept for context.
- Still to do: move ArtCommand/ArtAddress config work out of `tcpip_thread`. That is now planned as the `config_service` in `config-commits-and-artaddress.md` Phase B: the commit runs in `ControlTask`, not in a dedicated worker task. Then re-measure and shrink.
- The tuning pass is measurement-driven: `.su` files from `-fstack-usage` (largest Art-Net frame: `ArtNet_HandleArtCommand`, 184 B) plus the runtime watermarks.

## Goal

Reduce RAM pressure and improve confidence in task memory sizing without breaking the existing DMA, MPU, and LwIP memory placement strategy.

## Improvement Areas

### 1. Keep `tcpip_thread` Focused on Network Work

**Where**

- `Core/Src/Art-Net/artnet.c`
- `Core/Src/Art-Net/artnet_handlers.c`

**Why**

The UDP receive callback currently does more than packet transport work. The ArtCommand path validates the packet, parses ASCII commands, builds a config patch, applies the patch, updates report state, and sends a reply from LwIP context. That increases stack demand and also keeps the network thread busy longer than necessary.

**Future approach**

- Keep the UDP callback responsible for header checks, source tracking, and capturing the supported payload.
- Copy the relevant programming payload into a fixed-size queue or scratch buffer.
- Hand parsing and config application to a dedicated worker task.
- Send any reply that touches LwIP back through `tcpip_callback()`.

**Expected benefit**

- Lower peak stack usage in `tcpip_thread`
- Better networking latency under heavy protocol traffic
- Cleaner separation between transport code and application logic

### 2. Remove `snprintf()` from the PollReply Hot Path (done)

**Where**

- `Core/Src/Art-Net/artnet_pollreply.c`

**Why**

`ArtNet_SendPollReply()` currently formats `node_report` with `snprintf()`. The reply format is fixed and small, but `snprintf()` still pulls in a comparatively heavy formatting path for an embedded target.

**Future approach**

- Replace `snprintf()` with a small bounded formatter dedicated to the current `NodeReport` format.
- Better still, prebuild the report text when the report state changes and reuse that cached string when sending PollReply.

**Expected benefit**

- Lower stack use in the reply path
- Less libc formatting overhead on the network thread
- More predictable timing during ArtPoll, ArtAddress, and ArtCommand responses

### 3. Shrink `device_config` Patch/Application Stack Usage

**Where**

- `Core/Src/Device-Config/device_config.c`
- `Core/Inc/Device-Config/device_config.h`

**Why**

`DeviceConfig_ApplyPatch()` stages a full `DeviceConfig_t candidate` on the caller stack before validating and committing. That is safe and clear, but it becomes expensive when called from `tcpip_thread`. The Art-Net programming path also carries a fairly broad `DeviceConfig_Patch_t` even when only a few fields change.

**Future approach**

- Introduce a smaller Art-Net programming update structure for the subset of fields the protocol actually changes.
- Validate only the fields touched by the incoming command when that can be done safely.
- If full candidate staging remains the preferred model, move that work to a dedicated config task or a protected scratch context instead of using the network thread stack.

**Expected benefit**

- Lower peak stack during ArtCommand and future ArtAddress programming
- Better-defined ownership for runtime configuration writes
- Easier extension when more programmable fields are added later

### 4. Add Real Stack Visibility (done: `stack_monitor`)

**Where**

- `Core/Src/freertos.c`
- `Core/Src/Art-Net/artnet_handlers.c`
- Any future debug diagnostics module

**Why**

The current `vApplicationStackOverflowHook()` loops forever and discards the task identity. That is enough to halt on failure, but it does not help size stacks correctly.

**Future approach**

- Preserve the task name and handle in `vApplicationStackOverflowHook()` so the failing task is obvious during bring-up.
- Add temporary `uxTaskGetStackHighWaterMark()` checks around the ArtCommand, ArtAddress, and PollReply paths.
- Keep using the existing `.su` files from `-fstack-usage` after significant protocol changes.
- Optionally add a debug-only periodic task stack report for the main application threads.

**Expected benefit**

- Evidence-based stack sizing
- Faster root-cause analysis the next time a feature grows unexpectedly
- Less need for conservative stack over-allocation

### 5. Review Stack Sizes as a System, Not One Task at a Time

**Where**

- `LWIP/Target/lwipopts.h`
- `Core/Src/main.c`
- CubeMX `.ioc` task configuration

**Why**

The tcpip-thread overflow was the visible failure, but the project should budget task memory as a system. Otherwise each new feature will get a one-off stack increase and RAM use will drift upward without a clear reason.

**Future approach**

- Record the configured sizes for `tcpip_thread`, `ControlTask`, `EffectTask`, and any later worker/UI tasks.
- Measure runtime headroom under worst-case network traffic, rendering load, and protocol programming activity.
- Keep CubeMX-owned stack settings synchronized through the `.ioc` instead of relying on manual drift in generated files.

**Expected benefit**

- A balanced RAM budget across the whole firmware
- Clear justification for each task's stack allocation
- Fewer surprises when new features are added

### 6. Preserve the Existing SRAM Placement Strategy

**Where**

- `docs/memory-layout.md`
- Linker script and MPU configuration
- Any future queue or scratch-buffer additions

**Why**

The current project already uses a deliberate memory layout: Ethernet/LwIP buffers in AXI RAM, DMA-related data in D2 SRAM, and DMX-specific buffers in DTCM. Stack reduction work should not accidentally move temporary data into the wrong memory region.

**Future approach**

- Keep Ethernet RX/TX and LwIP pool memory in the existing non-cacheable network memory area.
- Keep DMX and SPI DMA buffers in their dedicated sections.
- Place any future Art-Net command queue or worker scratch buffers in ordinary CPU-owned RAM unless DMA visibility is explicitly required.

**Expected benefit**

- Lower stack usage without breaking cache coherency or DMA behavior
- Fewer hidden memory-placement regressions during optimization work

### 7. Avoid Heap-Based Fixes for Protocol Work

**Where**

- Any future deferred Art-Net programming path

**Why**

Moving protocol work off the stack should not turn into opportunistic `malloc()` use. That would trade a deterministic stack problem for fragmentation and less predictable failure modes.

**Future approach**

- Use fixed-size queues, ring buffers, or static pools for deferred protocol messages.
- Bound queue depth and payload size to the Art-Net commands that are actually supported.

**Expected benefit**

- Predictable worst-case RAM usage
- Easier code review and field debugging
- Fewer latent memory failures during long runtimes

## Suggested Order For The Future Pass

1. ~~Improve stack visibility and overflow reporting.~~ Done (2026-11-11): `Stack-Monitor` module, `stack_monitor_report` / `stack_monitor_overflow`. Per-path checks were not added — the tcpip_thread high-water mark already captures the worst path.
2. ~~Remove `snprintf()` from `ArtNet_SendPollReply()`.~~ Done (2026-11-11). Note: `TCPIP_THREAD_STACKSIZE` is passed to `osThreadNew` as **bytes** (CMSIS-RTOS2 `stack_size`), so tcpip_thread has 2048 B, not 2048 words.
3. Move heavy Art-Net programming work out of `tcpip_thread` (`config-commits-and-artaddress.md` Phase B).
4. Re-measure stack headroom and reduce `TCPIP_THREAD_STACKSIZE` only after the above changes are in place.
5. Do a wider task-by-task RAM budget review.

## Constraints

- If the final solution changes CubeMX-owned task sizes, LwIP settings, MPU regions, or linker-managed memory layout, make those changes through the `.ioc` and regenerate rather than patching generated code outside `USER CODE` blocks.
- Do not trade stack savings for unsafe shared state.
- Do not move buffers into DMA-visible or non-cacheable regions unless the hardware path actually requires it.

## Short-Term Position

The current `2048` quick fix is acceptable for now because it restored stability quickly. It is on the larger side for an MCU task, but not unreasonable for a network thread that currently performs packet parsing, config work, and formatted reply generation. The future goal should be to make that stack smaller by reducing the work and temporary data that live on the tcpip-thread path, not by guessing a smaller number and hoping it holds.