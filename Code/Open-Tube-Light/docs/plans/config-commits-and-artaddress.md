# Plan: serialized config commits → ArtAddress → locate

Step 1 of the post-quick-wins order: `artnet-compliance.md` §12 item 2, infrastructure 11.1 and 11.8, and optionally 11.5/11.6. It implements phases 2–3 and the "ArtAddress first slice" of `device-config-refactor.md` with concrete code placement. Persistence (phase 4) is the **next** step and is out of scope here, so every change made through this path is still runtime-only.

**Out of scope:** flash persistence, network/DHCP, runtime input-source switching, Art-Net control plane in DMX512 mode, merge engine, sACN, RDM.

**Before starting, read `artnet-audit-followups.md` items 1–3.** They change the shape of Phase B and still need a decision:
- the control-plane split (11.4) first or alongside
- Art-Net-owned reply slots with an opaque cookie instead of `ipv4/port/kind/hint`
- moving the ArtCommand parser to `artnet_command.c` as a pure move commit first

OpenTubeDia's custom protocol is transitional. RDM over Art-Net replaces its config verbs after persistence and network configuration exist (roadmap), and OpenTubeDia is then deprecated. Lightkey has no RDM, so the custom verbs stay until then, but nothing in this plan extends them.

---

## Decisions (agreed 2026-11-13)

| # | Topic | Decision |
|---|---|---|
| 1 | Locate | Standard ArtAddress `AcLedLocate` / `AcLedNormal`. The custom ArtTrigger KeyWink is dropped from the contract: with Oem 0xFFFF the spec marks Key 4 "Undefined". RDM `IDENTIFY_DEVICE` will drive the same locate state later. |
| 2 | What "indicators" means here | The spec talks about the node's front-panel indicators. This fixture has no visible indicator LED (the board LED is debug-only and hidden on this hardware revision), so the OLED counts as the front panel. Locate is meant to find the unit, so it **also** flashes the tube. |
| 3 | Locate pattern | Step 1: tube full white at 2 Hz (same load as `FAILSAFE_FULL`) until `AcLedNormal`; the spec has no timeout. The OLED part comes with the **menu system** (roadmap, Local UI): a flashing dot in one corner, so the menu stays usable while locating. |
| 4 | Mute | Deferred to the menu system, which will own the display timeout. `AcLedMute` will then force the display off before the timeout. Until then `AcLedMute` is ignored and Status1 never reports mute (don't claim what isn't done). |
| 5 | After a Port-Address / DMX address change | Keep the last frame. If nothing arrives on the new universe, the normal failsafe takes over after 5 s. |

---

## Target design

```text
tcpip_thread (Art-Net)                 ControlTask (only config writer)            tcpip_thread
──────────────────────                 ────────────────────────────────            ────────────
parse ArtCommand / ArtAddress
build DeviceConfig_Patch_t
ConfigService_Submit(req) ──queue──▶  ConfigService_Process():
  (queue full → immediate                DeviceConfig_ApplyPatch()  (atomic publish)
   RcConfigErr "Busy")                   side effects by change mask:
                                           • EffectTask flag 0x04 (re-render)
                                           • DMX_Input_OnConfigChanged(mask)
                                           • OLED refresh now
                                         req.done(result) ──tcpip_callback──▶  Art-Net reset (if address changed)
                                                                                set NodeReport, send ArtPollReply
```

Rules this establishes:
- Only `ControlTask` commits config. `DeviceConfig_ApplyPatch()` checks this and fails loudly from any other thread.
- Readers keep using `DeviceConfig_Get()`. The commit publishes under a scheduler lock, so a reader never sees a half-written config. A reader must not hold the pointer across a blocking call. Every current reader already follows this rule; it gets written into the header.
- Transient Art-Net commands (locate, cancel merge, clear output, reset flags) are **not** config. They run where their state lives and never go through the queue.

---

## Phase A — `device_config` contract (no visible behaviour change)

**Placement:** `device_config.c/.h`. This module owns the data and the commit. Rejected alternative: a double-buffered snapshot with pointer swap. With a single lowest-priority writer, a locked `memcpy` is equally safe and simpler.

1. **Atomic publish.** In `Config_CommitCandidate()`, wrap the `memcpy` and checksum in `osKernelLock()` / `osKernelRestoreLock()`. About 200 B are copied, which takes well under a microsecond. No ISR reads config; this was checked, and none of the DMX512, SPI or I2C ISRs call `DeviceConfig_*`.
2. **Commit owner.** Add `DeviceConfig_SetCommitOwner(osThreadId_t)`, called once at the top of `ControlTaskRun`. If an owner is set, `DeviceConfig_ApplyPatch()` returns -1 when `osThreadGetId() != owner`.
3. **Generation counter.** Add `DeviceConfig_GetGeneration()`, which is incremented on each commit. It's cheap change detection for later readers (UI, persistence debounce).
4. **API cleanup.** Remove the unused `DeviceConfig_SetIdentity/SetNetwork/SetLayout/SetDmx/SetOutput`, `RegisterCallback`, `Config_NotifyCallbacks` and `GetSnapshot`, which bypass or duplicate the patch path (confirmed: there are no callers). `BuildArtNetOutputAddressList` was already removed in the audit cleanup. Keep `FactoryReset` for the UI, but restrict it to the owner as well.
5. **Header docs.** Document the reader rule and the writer rule on `DeviceConfig_Get()` / `ApplyPatch()`.

**Verify:** the build shows only the baseline warning. On the bench: ArtCommand still works, but **only after Phase B**, because between A and B the owner check rejects commits from `tcpip_thread`. So land A and B as one commit, or temporarily skip setting the owner.

## Phase B — `config_service` + ControlTask loop + ArtCommand

**Placement:** a new module `Core/Src/Config-Service/config_service.c` with `Core/Inc/Config-Service/config_service.h`. It's added to `target_sources`. It owns the request queue and the apply sequencing, which is control-plane policy. Rejected alternatives: putting the queue in `device_config` would mix RTOS policy into the data module, which the refactor plan says not to do. Putting it in `main.c` USER CODE would grow a generated file.

**API sketch**

```c
typedef struct {
    uint32_t ipv4;      // requester, network byte order (no LwIP types in this header)
    uint16_t port;
    uint8_t  kind;      // requester-defined (e.g. ArtCommand vs ArtAddress)
    uint8_t  hint;      // requester-defined (e.g. which ArtCommand verbs were present)
} ConfigService_ReplyTo_t;

typedef struct {
    int      status;        // 0 = committed (or no change), -1 = rejected
    uint32_t change_mask;   // DEVICE_CONFIG_CHANGE_*
} ConfigService_Result_t;

typedef void (*ConfigService_DoneFn_t)(const ConfigService_ReplyTo_t *reply_to,
                                       const DeviceConfig_Patch_t *patch,
                                       const ConfigService_Result_t *result);

typedef struct {
    DeviceConfig_Patch_t   patch;
    ConfigService_ReplyTo_t reply_to;
    ConfigService_DoneFn_t done;     // runs in ControlTask; must not call LwIP directly
} ConfigService_Request_t;

int  ConfigService_Init(osThreadId_t effect_task);            // ControlTask, before the loop
int  ConfigService_Submit(const ConfigService_Request_t *req); // any task, non-blocking, -1 = full
void ConfigService_Process(uint32_t timeout_ms);              // ControlTask only
```

1. **Queue.** `osMessageQueueNew(4, sizeof(ConfigService_Request_t), &attr)` with **static** `cb_mem`/`mq_mem` (`configSUPPORT_STATIC_ALLOCATION=1`), so there is no FreeRTOS heap use. Each entry is about 150 B.
2. **`ConfigService_Process()`.** Wait on the queue up to `timeout_ms`, then:
   1. `DeviceConfig_ApplyPatch(&req.patch, false, &mask)`
   2. If `mask != 0`, run the side effects:
      - `LAYOUT | DMX_START_ADDRESS | ARTNET_ADDRESS | OUTPUT_FAILSAFE` → `osThreadFlagsSet(effect_task, 0x04)`
      - any `DMX` or `LAYOUT` bit → `DMX_Input_OnConfigChanged(mask)`
      - any bit → the OLED redraws on this loop pass
   3. `req.done(...)`
   4. Bump the counters `config_commit_count` / `config_reject_count` / `config_queue_full_count` (Live Watch).
3. **Reject runtime input-source changes** (`DMX_SOURCE`) in `Process` for now. Nothing can request one yet. `ArtNet_Init/Deinit` already hold `LOCK_TCPIP_CORE()` around the PCB calls, but switching also needs `EffectTask` quiesced (`deinit` deletes the mutex `latch` takes, `artnet-audit-followups.md` item 12). With the control-plane split the socket is not touched by a switch at all. This is noted for the UI step.
4. **`dmx_input` hook.** Add an optional vtable entry `void (*on_config_changed)(uint32_t mask)` (NULL = ignore) and `DMX_Input_OnConfigChanged()`. Update `docs/dmx-input-drivers.md`.
   - Art-Net implements it as `tcpip_callback(ArtNet_ResetUniverseState, NULL)`: clear `source_ip`, `sync_mode`, `last_sequence` for all universes, and keep the buffers (decision 3).
   - DMX512 leaves it NULL.
5. **ControlTask loop.** Replace `osDelay(1000)` with `ConfigService_Process(time until next 1 Hz tick)`. Redraw the OLED when the 1 Hz tick is due, or right after a commit. `OledStatus_RunStep()` stays as it is: it starts the I2C DMA flush and then sleeps in `ssd1306_WaitForUpdate()` until it completes (about 25 ms: 1 KB at 400 kHz). A request arriving during that window waits for it. Making the flush non-blocking is deliberately left to the menu system (roadmap, Local UI), which rewrites this code and needs it anyway for button responsiveness.
6. **EffectTask.** Also wait for flag `0x04`. On `0x04`:
   - if `failsafe_applied`, call `Pixel_Render_Failsafe()` again (picks up a new failsafe mode)
   - otherwise, if `first_frame_received`, call `Pixel_Render_Frame()`, which re-renders the active buffer with the new layout or address without waiting for new DMX

   It doesn't latch. Name the flag `EFFECT_TASK_FLAG_CONFIG_CHANGED`.
7. **ArtCommand migration** (`artnet_handlers.c`):
   - The parse and validation of the text stays in `tcpip_thread`; it's bounded and cheap.
   - The handler fills a request with `reply_to` = source IP/port and `hint` = which verbs were present, submits it, and returns. Parse errors still reply immediately with `RcParseFail` / `RcConfigErr`, as today.
   - If the queue is full, reply immediately with `RcConfigErr "Config Busy"`.
   - `ArtNet_OnConfigDone()` runs in ControlTask. It maps the result to a NodeReport code and detail: the success detail comes from `change_mask` (more accurate than today's pre-commit comparison) and the failure detail from `hint`.
   - It stores `{ip, port, code, detail}` in one of 4 static reply slots in `g_artnet_ctx`. Detail strings are literals, so nothing is copied. *(Open: `artnet-audit-followups.md` item 2 recommends allocating the slot in `tcpip_thread` at submit time and passing an opaque cookie, so `ControlTask` only writes the result.)*
   - It then calls `tcpip_callback(ArtNet_SendConfigReply, slot)`. Use blocking `tcpip_callback`, not `tcpip_try_callback`, because the mbox (6) can be momentarily full of ArtDmx and ControlTask can afford to wait.
   - `ArtNet_SendConfigReply` runs in `tcpip_thread`: `ArtNet_ReplyImmediateStatus(...)`, then it frees the slot.

**Verify (bench)**

- `tools/artnet_bench_test.py`, full run: still 57/57. ArtCommand replies arrive in under 100 ms. The script's restore uses a 0.8 s window.
- `SegmentCount=12&` with **no** Lightkey running: the strip re-renders immediately; before, it waited for the next frame.
- Changing the failsafe mode while in failsafe can't be triggered from the network yet; it gets tested in Phase C with AcFail*.
- Live Watch:
  - `config_commit_count` increments once per accepted ArtCommand
  - `config_reject_count` increments on an invalid value
  - `stack_monitor_report`: `ControlTask` min free stack (the candidate copy plus the request now live there) and `tcpip_thread` (lighter than before)

**Docs:**
- CLAUDE.md: the threading table (ControlTask is the only config writer) and the module map (`config_service`).
- `dmx-input-drivers.md`: the new hook.
- Roadmap: "Serialized config commits" and "Move ArtCommand … out of tcpip_thread" → Done.
- Compliance 11.1 → Done.

## Phase C — ArtAddress (config fields + transient commands)

**Placement:** `ArtNet_HandleArtAddress()` in `artnet_handlers.c` (the protocol mapping), with a new `ArtNet_ResetUniverseState()` in `artnet.c` next to the universe state it clears. Rejected alternative: a separate `artnet_address.c`. About 150 lines doesn't justify a module. `artnet_handlers.c` is already about 900 lines, so move the ArtCommand parser out first (`artnet-audit-followups.md` item 3). Use one Port-Address helper for SwOut contiguity, ArtDmx and ArtPoll (item 4).

**Accept rules**

| Check | Action |
|---|---|
| `len < 107` | `RcParseFail "ArtAddress Too Short"` (unchanged) |
| `BindIndex` not 0/1 | Silent ignore, no reply, like ArtInput |

**Fields → one `DeviceConfig_Patch_t`**

| Field | Rule (spec: bit 7 = program, 0x00 = reset to "physical switch" = factory default, anything else = no change) |
|---|---|
| ShortName [18] / LongName [64] | Non-empty → patch. Copy bounded and NUL-terminate. |
| NetSwitch | `0x80 \| n` → `artnet_net = n`. `0x00` → default. |
| SubSwitch | `0x80 \| n` → `artnet_subnet = n`. `0x00` → default. |
| SwOut[0] | `0x80 \| u` → `artnet_start_universe = u`. `0x00` → default. |
| SwOut[1..num_ports-1] | If programmed, it must equal `start + i` (contiguous). Otherwise reject the whole packet with `RcConfigErr "SwOut Not Contiguous"`. Ports beyond `num_ports` are ignored. |
| SwIn, AcnPriority | Ignored (no inputs, no sACN generation). |

The existing validation rejects Port-Address 0 and rollover past universe 15. Those surface as `RcConfigErr` with an unchanged state.

**Command byte**

| Command | Where | Effect |
|---|---|---|
| `AcNone` 0x00 | — | No-op. |
| `AcCancelMerge` 0x01 | tcpip_thread, immediately | `ArtNet_ResetUniverseState()`. The next ArtDmx sender owns each universe (backup-console takeover). |
| `AcLedNormal/Locate` 0x02/0x04 | Phase D | Until then: ignored. |
| `AcLedMute` 0x03 | Menu system (roadmap) | Ignored until the menu owns the display. |
| `AcResetRxFlags` 0x05 | tcpip_thread | NodeReport → `RcPowerOk`. |
| `AcFailHold/Zero/Full` 0x08–0x0A | Config patch | `output.failsafe`. Runtime-only until persistence, so **Status3 bit 5 stays 0** until step 2. |
| `AcClearOp0` 0x90 | tcpip_thread | Zero the shadow buffers (under `shadow_mutex`), then `ArtNet_TriggerFrameOutput()` so EffectTask latches and renders black. |
| Everything else (scene, analysis, merge, direction, ArtNetSel/AcnSel, style, RDM, deprecated port 1–3 codes) | — | Ignored. The reply shows the unchanged state, which is how a controller sees it wasn't applied. |

**Reply**

- **The patch is empty** (command only): reply immediately, as today's `ArtNet_ReplyImmediateStatus`.
- **Otherwise:** submit a config request with `kind = ARTNET_REQ_ADDRESS`. `ArtNet_OnConfigDone` picks the NodeReport:
  - rejected → `RcConfigErr` + detail
  - short name changed → `RcShNameOk`
  - else long name changed → `RcLoNameOk`
  - else → `RcPowerOk "Address Programmed"`
- The reply is always a full ArtPollReply with the **resulting** state.

**Status1 bits 5-4** (Port-Address authority): `g_artnet_ctx.address_authority`. It is 01 at boot (defaults / front panel) and 10 after any network-originated commit that changed `ARTNET_ADDRESS`. It's runtime-only; revisit with persistence.

**Verify (bench).** Run this test matrix through a new `address` test in `tools/artnet_bench_test.py`:

- rename the short name only, then the long name only
- Net + Sub change
- contiguous SwOut
- non-contiguous SwOut rejected
- rollover rejected
- Port-Address 0 rejected (finally testable on the bench)
- Fail Hold/Zero/Full: stop Lightkey output and watch the strip after 5 s
- CancelMerge
- ClearOp
- ResetRxFlags
- BindIndex 2 ignored

Then interop: change the universe from DMX-Workshop (Windows, Artistic Licence) if available, and check that Lightkey output on the **new** universe drives the strip immediately.

Live Watch: `failsafe_count`, `render_count`, `config_commit_count`, and `stack_monitor_report` for `tcpip_thread` (the ArtAddress handler has a ~150 B patch on its stack).

**Docs:**
- Compliance §6 rows → Done/Ignored as implemented. Status1 5-4 → Done.
- Roadmap: ArtAddress → Done.
- `artnet-custom-protocol.md`: a note that ArtAddress is standard, plus the NodeReport codes it produces.
- Remove the stale "ArtAddress Unsupported" mentions.

## Phase D — locate on the tube (indicator state + render override)

The OLED side of locate (corner dot) and mute (forced display off) belong to the future menu system, which will own the display and its timeout. Phase D only does the tube and the reporting, but the indicator state is built so the menu can read it later without changes.

**Placement**
- **Indicator state:** a new module, `Core/Src/Indicator/indicator.c` with `Core/Inc/Indicator/indicator.h`. It's about 30 lines and protocol-agnostic. Writers: Art-Net `AcLed*` now; RDM `IDENTIFY_DEVICE` and the local UI later. Readers: EffectTask (tube) and Art-Net (Status1) now; the menu system (corner dot, mute) later.
  - Rejected alternative: keeping it in `g_artnet_ctx`. RDM on the DMX512 line and the menu would then depend on Art-Net internals.
- **Locate pattern:** rendered by `pixel_render.c` (it already owns failsafe output) and applied in `EffectTask`, so SPI DMA ownership stays in one task.
  - Rejected alternative: flashing the strip from the UDP callback, which would give a second writer to the SK9822 buffers.

1. **`indicator.h`**
   - `typedef enum { INDICATOR_NORMAL, INDICATOR_LOCATE } Indicator_Mode_t;` `INDICATOR_MUTE` is added with the menu system.
   - `void Indicator_Set(Indicator_Mode_t)` and `Indicator_Mode_t Indicator_Get(void)`.
   - Atomic 32-bit store and load, callable from any task. `Indicator_Set` also wakes EffectTask with a thread flag, so the change shows at once.
2. **Tube (`pixel_render` + EffectTask)**
   - Add `void Pixel_Render_Locate(uint32_t now)`: full white while `(now / 250) % 2 == 0`, black otherwise.
   - EffectTask checks `Indicator_Get()` first on every wake. While it's `LOCATE`, it uses a 50 ms wait timeout instead of 150 ms and renders the locate pattern instead of DMX/failsafe/refresh. It keeps latching new frames, so the data stays current.
   - Leaving `LOCATE`: re-render as for flag `0x04` (failsafe if failsafe was applied, otherwise the active buffer).
3. **Art-Net**
   - In `ArtNet_HandleArtAddress`: `AcLedLocate` → `Indicator_Set(LOCATE)`, `AcLedNormal` → `NORMAL`. These run immediately in `tcpip_thread` and aren't config.
   - `AcLedMute` is ignored until the menu system exists.
   - `ArtNet_BuildStatus1()` maps `Indicator_Get()` to bits 7-6: 11 normal / 01 locate.
4. **Docs**
   - `artnet-custom-protocol.md`: the Locate section becomes "implemented" for the tube.
   - Compliance: §6 AcLedLocate/AcLedNormal → Done (AcLedMute stays Missing until the menu), Status1 7-6 → Partial (no mute yet), 11.8 → Done.
   - CLAUDE.md module map: add `indicator`.

**Verify (bench)** with a new `locate` test in `tools/artnet_bench_test.py`:
- Locate: the tube flashes about 2 Hz and Status1 = `0x4x`.
- Normal: the last frame returns, or the failsafe output if no data is arriving.
- Mute: nothing changes and Status1 stays `0xCx`.
- Locate **while Lightkey streams**: still flashes, and `render_count` keeps rising (frames are latched, not lost).
- Locate **during failsafe**: Normal brings the failsafe output back.
- Live Watch: `stack_monitor_report` for EffectTask.

## Phase E (optional, can split off) — ArtPoll Flags bit 1, reply on change

**Placement:** the controller registry in `g_artnet_ctx` (11.5). Change events come from the Art-Net `on_config_changed` hook (config commits from any origin) and from AcLed* (11.6). Without the control-plane split, the hook and the registry only exist while Art-Net is the input source. With it, they belong to the always-on node part.

1. Registry: up to 4 entries of `{ip, port, flags, last_poll_tick}`, refreshed on every ArtPoll and expired after 10 s without a poll.
2. On a change event: queue an ArtPollReply to every entry with bit 1 set, through the existing reply queue with no random delay.
3. Compliance §3 Flags bit 1 → Done.

**Verify:** the script polls with flags 0x02, then a config change from a second socket → an unsolicited ArtPollReply arrives on the first.

---

## Order and commits

| Commit | Content | Bench gate |
|---|---|---|
| 1 | Phase A + B (A alone breaks ArtCommand) | Script 57/57, immediate re-render on SegmentCount |
| 2 | Phase C | `address` test matrix |
| 3 | Phase D | `locate` test + visual |
| 4 | Phase E (optional) | Unsolicited reply test |

Each commit: build with only the baseline warning and an unchanged memory table (the queue is static, about 650 B of `.bss`). Docs are updated in the same commit.

## Risks

- **ControlTask stack (2048 B).** `ApplyPatch` puts a ~200 B candidate on the stack, and `Process` adds a ~150 B request. Watch `stack_monitor_report` after Phase B. The fix is the `.ioc` stack size: regenerate, don't hand-edit.
- **Reply latency** for config commands: about 1 ms usually, up to about 30 ms when a request lands during the 1 Hz OLED flush (about 2.5 % of the time), because `ControlTask` sleeps until the I2C DMA transfer completes. The CPU is free meanwhile, the strip updates at commit, and controllers allow 3 s, so this is accepted until the menu system makes the flush non-blocking.
- **`tcpip_callback` memp pool** (`MEMP_NUM_TCPIP_MSG_API` = 8) is shared with the poll-reply timer. A burst of config requests is limited by the 4-entry queue, so at most 4 + 1 messages are outstanding.
- **AcFail\* without persistence** is reset by a reboot. That's acceptable for now; Status3 bit 5 isn't advertised until step 2.
- **RDM and the menu system later.** Keep the indicator module and the commit path protocol-agnostic: RDM `IDENTIFY_DEVICE`, `DMX_START_ADDRESS` and `DMX_PERSONALITY` will reuse `Indicator_Set()` and `ConfigService_Submit()` unchanged, and the menu reads `Indicator_Get()` for the corner dot and mute.
