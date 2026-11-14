# Device Config Refactor Plan

## Status (2026-11-14)

This is the long-range plan. Where it disagrees with a newer plan, the newer one wins:

| Part | Status |
|---|---|
| Phase 1 contract cleanup | Partly done in the audit cleanup: the misleading flash, callback and "CRC32" comments are rewritten, and the unused `DeviceConfig_BuildArtNetOutputAddressList()` is gone. Still open: the checksum is not a real CRC32, and config validation still accepts RGBW/RGB16 (only ArtCommand rejects them). |
| Phases 2–3 (commit path, side effects) | **Superseded** by `config-commits-and-artaddress.md` Phases A–B. That plan keeps `DeviceConfig_Get()` as a live pointer and publishes commits with a scheduler-locked `memcpy`. It rejects the immutable-snapshot / pointer-swap read model described below, and puts the queue in a new `config_service` module instead of `main.c`. |
| Phase 4 (persistence) | Open. One sector (compliance 11.2) or two (below) is undecided, and so is the erase-stall policy on the single-bank H723 (`artnet-audit-followups.md` item 13). Settle both in a persistence plan before writing flash code. |
| Phase 5 (network) | Open. Now scheduled **before** RDM (`artnet-compliance.md` §12). |
| ArtAddress section at the end | Superseded by `config-commits-and-artaddress.md` Phase C. |

## Purpose

This document describes the current `device_config` architecture as it exists now,
why it is the main control-plane weakness in the project, and the recommended refactor
path. The goal is not to add abstraction for its own sake. The goal is to make
configuration ownership explicit, runtime-safe, persistent, and ready for both Art-Net
and future OLED/button editing without fighting the current FreeRTOS, HAL, and LwIP design.

## Current Architecture

### Startup and ownership today

- `main()` now calls `DeviceConfig_Init()` before `osKernelStart()`.
- `ControlTaskRun()` later calls `MX_LWIP_Init()`, reads the active config through
  `DeviceConfig_Get()`, starts the configured DMX input source, and then releases `EffectTask`.
- `EffectTaskRun()` owns pixel rendering and output refresh, but it still reads config-derived
  behavior from the shared config module.
- Art-Net handlers run in lwIP `tcpip_thread` context.
- `ArtNet_HandleArtCommand()` currently parses a patch and calls
  `DeviceConfig_ApplyPatch(&patch, false, NULL)` directly from `tcpip_thread`.
- Several modules read the live singleton directly today, including `main.c`,
  `pixel_render.c`, `artnet_pollreply.c`, and `artnet_handlers.c`.

### What `device_config` does today

- Stores one global `active_config` instance.
- Applies changes by copying `active_config` to a candidate, validating the candidate,
  and then copying the whole candidate back into `active_config` with `memcpy`.
- Exposes both:
  - `DeviceConfig_Get()` returning a live pointer to the current singleton
  - `DeviceConfig_GetSnapshot()` returning a copy
- Calculates change masks and supports callback registration.
- Does not actually use callbacks anywhere in the current codebase.
- Pretends to support persistence, but:
  - `DeviceConfig_SaveToFlash()` is still a stub
  - `Config_LoadFromFlash()` still fails, so boot always falls back to defaults
- Describes the checksum as CRC32 in comments, but the implementation is not CRC32.

### What is already good

- The schema shape is good: identity, network, layout, DMX, and output are grouped sensibly.
- Defaults and limits are now mostly internally consistent for the current product intent.
  The current code agrees on a 144-pixel fixture.
- Startup ordering is better than before.
  Config initialization now happens before task startup, so the old “load-after-start” problem
  is no longer the main issue.
- The system-level split between control-plane work and render/output work is already correct enough
  to build on.

### What is bad and why it should be fixed

- The module looks centralized, but write ownership is not centralized.
  Today the network stack thread can commit runtime config directly.
- Readers observe a live shared singleton while another context is allowed to replace it.
  That is a weak contract for a system that will soon have more than one writer.
- There is no single serialized apply path for side effects.
  A config change can affect DMX input selection, Art-Net identity, poll replies, render behavior,
  and later flash save timing. Those effects should not depend on which thread received the request.
- Persistence is fake right now.
  A change can appear accepted and still disappear on reboot.
- The callback API invites wrong-thread reconfiguration.
  It is easy to imagine future code registering a callback that touches lwIP, DMA, or drivers from
  the wrong context.
- The public API promises more than the implementation currently delivers.
  Network fields exist in the schema, but boot-time and runtime application policy is not yet defined.

## Recommended Architecture

### Core decision

Keep `device_config` as the module that owns config data structures, defaults, validation,
diff/change-mask logic, and persistence helpers. Move runtime commit ownership into `ControlTask`.

Do not add a separate config task yet.
At the moment configuration, UI, persistence, and network-control work are all control-plane work.
`ControlTask` is the right place to serialize them.

### Ownership model

- `ControlTask` is the only context allowed to commit a new active config.
- All other writers submit requests:
  - Art-Net handlers submit requests from `tcpip_thread`
  - future OLED/button UI submits requests through the same control-plane path
  - any later UART/CLI/config service does the same
- `EffectTask` never mutates config.
  It consumes published config state and reacts to change notifications.
- `device_config` stops using “live mutable singleton” as its primary public contract.

### Read model

Use one published immutable snapshot for readers.

Recommended shape:

- `DeviceConfig_GetSnapshot(DeviceConfig_t *out)` becomes the normal public read API.
- If a fast pointer-based path is still needed internally, publish a read-only snapshot that is only
  swapped by `ControlTask` under a short critical section and never mutated in place after publication.
- Render code should cache a local snapshot when relevant config changes occur, instead of reaching into
  shared global state on every control path.

This keeps the read side simple and avoids sprinkling locks across the whole codebase.

### Write model

Introduce one request structure and one serialized commit path.

Recommended flow:

1. A writer creates `DeviceConfig_Request_t`.
2. The request is queued or signaled to `ControlTask`.
3. `ControlTask` copies the current config to a candidate.
4. `device_config` validation and patch helpers apply the request to the candidate.
5. If valid, `ControlTask` computes change flags and publishes the new snapshot.
6. `ControlTask` schedules side effects in the correct owning contexts.
7. `ControlTask` marks config dirty and debounces the flash save.

The important architectural change is that `ArtNet_HandleArtCommand()` must stop calling
`DeviceConfig_ApplyPatch()` directly.
It should translate protocol input into a config request and hand it off.

### Apply model for side effects

Do not use generic synchronous callbacks as the main behavior mechanism.
Use explicit apply steps owned by the control plane.

Recommended change classes:

- Render-related changes:
  - layout
  - segment format
  - failsafe behavior
  - publish snapshot and notify `EffectTask`
- DMX transport changes:
  - stop active input driver
  - start requested driver
  - preserve sane output state during switch
- Art-Net identity and addressing changes:
  - update poll reply state
  - update universe bookkeeping if address ownership changes
  - marshal lwIP-affine work with `tcpip_callback`
- Network changes:
  - classify as live-apply or controlled restart/rebind
  - never execute lwIP-affine work directly from arbitrary tasks

This makes side effects explicit, testable, and thread-correct.

### Persistence model

Split flash persistence from runtime ownership.

Recommended file split:

- `device_config.c`
  - schema defaults
  - validation
  - patch application to candidates
  - change-mask calculation
  - derived-value helpers
- `device_config_flash.c`
  - flash record scan/load/save/compact
  - CRC32 and record verification
- control-plane code in `main.c` or a dedicated control module
  - request queue
  - commit serialization
  - deferred save timing
  - side-effect orchestration

Recommended flash layout:

- two reserved flash sectors
- journaled records with:
  - magic
  - version
  - payload size
  - monotonic sequence number
  - config payload
  - real CRC32
- ping-pong sector usage:
  - boot scans both sectors and selects the newest valid record
  - save appends or compacts
  - no in-place overwrite dependency

This fits STM32H7 flash behavior and power-loss tolerance much better than pretending a single save call is enough.

### Network authority policy

Startup ordering is now compatible with authoritative network config, but network authority is still incomplete.

The target behavior should be:

- load config before network start
- explicitly map `network.ip_mode` to DHCP or static startup behavior
- define which network fields require a controlled restart
- keep restart and rebind orchestration in `ControlTask`
- marshal lwIP-affine work correctly with `tcpip_callback`

Do not treat network config as done until both boot-time application and runtime policy are implemented.

## Recommended Module Boundaries

### `device_config.h`

Keep public and narrow:

- config types and enums
- snapshot read API
- request types and result codes
- derived-value helpers that other modules genuinely need

Remove or de-emphasize:

- raw live-pointer access as the normal public contract
- generic callback registration as the primary reconfiguration mechanism

### `device_config.c`

Own:

- factory defaults
- candidate validation
- patch application to candidate configs
- change-mask calculation
- derived-value helpers
- snapshot publication helpers if they stay local to the module

Do not let it own:

- queue and task policy
- lwIP calls
- DMX driver switching
- flash sector orchestration if that is split into `device_config_flash.c`

### `ControlTask`

Own:

- request queue
- commit serialization
- apply sequencing
- dirty/deferred-save timing
- requester response handling if needed
- coordination between network, DMX input, render, and persistence

### Art-Net handlers

Own:

- protocol parsing
- translation from protocol payload into config requests
- response/result formatting if needed

They should stop owning direct config mutation.

## Phased Plan

### Phase 1: Contract cleanup

Goal: make the API reflect the actual device and current implementation.

- keep the current 144-pixel product contract unless hardware requirements change
- replace the fake CRC32 comment or implement real CRC32
- decide whether unsupported `segment_format` values should be rejected until the renderer supports them
- document which fields are live-apply, boot-only, or restart-required

Expected output:

- no misleading API comments
- no config fields that claim behavior without a policy

### Phase 2: Serialized runtime commit path

Goal: establish one authoritative write path before adding more config features.

- add `DeviceConfig_Request_t` and a control-plane request path
- stop direct config commits from `tcpip_thread`
- keep one published read-only snapshot for readers
- emit explicit change flags for apply logic

Expected output:

- one authoritative commit path
- no direct in-place mutation from protocol handlers

### Phase 3: Explicit subsystem apply logic

Goal: make side effects controlled and predictable.

- notify `EffectTask` only for render-relevant changes
- add DMX input reconfiguration hooks in control-plane context
- add Art-Net identity/address refresh hooks through the correct lwIP context
- remove dependence on the unused generic callback model

Expected output:

- each supported config field has a defined runtime effect and owning context

### Phase 4: Real flash persistence

Goal: survive reboot and interrupted writes.

- reserve flash space intentionally
- implement scan/load/save/compact with real CRC32 and sequence numbers
- save from control-plane context after debounce, not directly from packet handling

Expected output:

- accepted config changes survive power loss
- partially written records do not break boot

### Phase 5: Network configuration authority

Goal: make stored network fields actually control startup and change behavior.

- apply DHCP or static settings on bring-up
- define live update versus controlled restart for network fields
- keep CubeMX boundaries intact and coordinate `.ioc` changes explicitly if needed

Expected output:

- network settings become authoritative in a real, documented way

### Phase 6: Remote and local writers

Goal: add more editing surfaces without bypassing the commit path.

- route Art-Net config changes through the request path
- build OLED/button UI as another client of the same path
- persist once per confirmed edit batch, not once per button press

Expected output:

- Art-Net and local UI share validation, apply, and persistence behavior

### Phase 7: Validation and recovery coverage

Goal: prove the system fails safely.

- boot with empty flash
- boot with corrupted records
- boot after interrupted save
- switch DMX source while frames are active
- edit config while Art-Net traffic is active
- exercise restart-required network changes
- reject or fully implement unsupported layout/output formats

Expected output:

- bounded and predictable recovery behavior

## First Concrete Work Items

1. Replace the public live-pointer contract with a request-and-snapshot model.
2. Add a `ControlTask`-owned config request queue.
3. Change `ArtNet_HandleArtCommand()` so it submits a request instead of calling `DeviceConfig_ApplyPatch()` directly.
4. Add explicit apply handlers for DMX input selection and render-related changes.
5. Implement real flash persistence with CRC32 and reserved sectors.
6. After the commit path is stable, wire network settings into boot behavior and restart policy.

## Definition of Done

The refactor is done when all of the following are true:

- only `ControlTask` commits runtime config
- readers never observe a half-written config (locked publish; see `config-commits-and-artaddress.md` Phase A)
- Art-Net and future UI use the same request path
- side effects run in the correct owning context
- config survives reboot and interrupted save
- network settings are either applied or explicitly rejected
- every advertised config field has a defined behavior policy

## ArtAddress (runtime-only first slice)

Superseded by `config-commits-and-artaddress.md` Phase C, which carries the field rules, the reset hook and the test matrix. One difference from the earlier draft here: BindIndex 0 and 1 both mean the root device (the earlier draft required exactly 1).
