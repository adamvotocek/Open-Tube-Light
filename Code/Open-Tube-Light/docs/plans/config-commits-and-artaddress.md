# Plan: serialized config commits → ArtAddress → locate

The config-service step: `artnet-compliance.md` §12 step 3, infrastructure 11.1 and 11.8, and optionally 11.5/11.6. It comes after the Art-Net groundwork (`completed/artnet-groundwork.md`, done 2026-11-15). It implements phases 2–3 and the "ArtAddress first slice" of `device-config-refactor.md` with concrete code placement.

**Out of scope:** flash persistence, network/DHCP, runtime input-source switching, merge engine, sACN, RDM.

OpenTubeDia's custom protocol is transitional. RDM over Art-Net replaces its config verbs after persistence and network configuration exist (roadmap), and OpenTubeDia is then deprecated. Lightkey has no RDM, so the custom verbs stay until then, but nothing in this plan extends them.

## Changes since the first version (2026-11-15, after the groundwork)

The plan was written on 2026-11-13, before the groundwork. This revision brings it up to date. Nothing here is implemented yet.

1. **ArtCommand lives in `artnet_command.c`.** Phase B's ArtCommand migration targets that file. Phase C's ArtAddress handler stays in `artnet_handlers.c`.
2. **Reply slots owned by Art-Net** (`artnet-audit-followups.md` item 2, folded in). `config_service` no longer carries `ipv4/port/kind/hint`. A request carries an opaque `cookie`; Art-Net allocates the slot in `tcpip_thread` at submit time.
3. **The config reply is sent under `LOCK_TCPIP_CORE()` from ControlTask**, not through `tcpip_callback`. That is the pattern `ArtNet_NodeInit` and `ArtNet_EnableOutput/DisableOutput` already use. It removes a `tcpip_callback` message per reply (the `MEMP_NUM_TCPIP_MSG_API` risk) and the hand-off of the result between threads.
4. **Art-Net gets config changes directly** (`ArtNet_OnConfigChanged(mask)`, called by `config_service`), not through a new `dmx_input` vtable hook. Since the split, the node runs in every input mode, but the `dmx_input` vtable only reaches the active driver. Phase E (reply on change) and the Status1 address authority need the event in DMX512 mode too. DMX512 needs no hook. sACN can add one when it exists.
5. **DMX512 mode** (new decision 6, **proposed**): which ArtCommand / ArtAddress fields and commands apply while Art-Net is not the input.
6. **Smaller updates:** the failsafe timeout is 6 s (was 5 s in the Phase C test), the bench baseline is 58/58 (was 57/57), each phase is also run with `--mode dmx512`, and the ControlTask loop uses `OLED_STATUS_UPDATE_PERIOD_MS`. Phase B item 4 cited "decision 3" for keeping the buffers; it is decision 5.

---

## Decisions (agreed 2026-11-13, decision 6 proposed 2026-11-15)

| # | Topic | Decision |
|---|---|---|
| 1 | Locate | Standard ArtAddress `AcLedLocate` / `AcLedNormal`. The custom ArtTrigger KeyWink is dropped from the contract: with Oem 0xFFFF the spec marks Key 4 "Undefined". RDM `IDENTIFY_DEVICE` will drive the same locate state later. |
| 2 | What "indicators" means here | The spec talks about the node's front-panel indicators. This fixture has no visible indicator LED (the board LED is debug-only and hidden on this hardware revision), so the OLED counts as the front panel. Locate is meant to find the unit, so it **also** flashes the tube. |
| 3 | Locate pattern | Step 1: tube full white at 2 Hz (same load as `FAILSAFE_FULL`) until `AcLedNormal`; the spec has no timeout. The OLED part comes with the **menu system** (roadmap, Local UI): a flashing dot in one corner, so the menu stays usable while locating. |
| 4 | Mute | Deferred to the menu system, which will own the display timeout. `AcLedMute` will then force the display off before the timeout. Until then `AcLedMute` is ignored and Status1 never reports mute (don't claim what isn't done). |
| 5 | After a Port-Address / DMX address change | Keep the last frame. If nothing arrives on the new universe, the normal failsafe takes over after the failsafe timeout (6 s). Same rule as an input switch (`dmx-input-drivers.md`, "Input switch = input lost"). |
| 6 | **Proposed:** configuration while DMX512 is the input | **Fixture-wide settings apply in every mode; Art-Net output settings apply only where the reply can show them.** In every mode: ArtCommand (all verbs; `DmxAddress` sets the DMX512 start address), ArtAddress ShortName/LongName, NetSwitch/SubSwitch (reported in every ArtPollReply), `AcFail*` (the failsafe is fixture-wide), `AcLed*`, `AcResetRxFlags`. While Art-Net output is disabled: SwOut is ignored (NumPorts = 0, so "ports beyond NumPorts are ignored" covers it), and `AcCancelMerge` / `AcClearOp0` are ignored (there is no Art-Net output to cancel or clear). The reply always shows the resulting state, so a controller sees what was applied. **Rejected alternative:** accept SwOut[0] in DMX512 mode against the configured universe count, so a universe can be preset before switching to Art-Net. The controller couldn't verify it (no SwOut in the reply), and nothing can switch the input at runtime yet. Revisit with runtime input switching. |

---

## Target design

```text
tcpip_thread (Art-Net)                 ControlTask (only config writer)
──────────────────────                 ────────────────────────────────
parse ArtCommand / ArtAddress
build DeviceConfig_Patch_t
allocate a reply slot (ip, port,
  kind, hint); none free → reply
  RcConfigErr "Config Busy" now
ConfigService_Submit(req, cookie) ─queue─▶ ConfigService_Process():
  (queue full → free slot, reply            DeviceConfig_ApplyPatch()  (atomic publish)
   RcConfigErr "Config Busy" now)           side effects by change mask:
                                              • EffectTask flag 0x04 (re-render)
                                              • ArtNet_OnConfigChanged(mask)
                                              • OLED refresh now
                                            req.done(cookie, result):
                                              LOCK_TCPIP_CORE()
                                              NodeReport + ArtPollReply to the slot's controller
                                              free the slot
                                              UNLOCK_TCPIP_CORE()
```

Rules this establishes:
- Only `ControlTask` commits config. `DeviceConfig_ApplyPatch()` checks this and fails loudly from any other thread.
- Readers keep using `DeviceConfig_Get()`. The commit publishes under a scheduler lock, so a reader never sees a half-written config. A reader must not hold the pointer across a blocking call. Every current reader already follows this rule; it gets written into the header.
- `config_service` knows nothing about Art-Net requests: it only passes the cookie back. Art-Net state (reply slots, universe state) is touched only in `tcpip_thread` or under the core lock.
- Transient Art-Net commands (locate, cancel merge, clear output, reset flags) are **not** config. They run where their state lives and never go through the queue.

---

## Phase A — `device_config` contract (no visible behaviour change)

**Placement:** `device_config.c/.h`. This module owns the data and the commit. Rejected alternative: a double-buffered snapshot with pointer swap. With a single lowest-priority writer, a locked `memcpy` is equally safe and simpler.

1. **Atomic publish.** In `Config_CommitCandidate()`, wrap the `memcpy` and checksum in `osKernelLock()` / `osKernelRestoreLock()`. About 200 B are copied, which takes well under a microsecond. No ISR reads config; this was checked, and none of the DMX512, SPI or I2C ISRs call `DeviceConfig_*`.
2. **Commit owner.** Add `DeviceConfig_SetCommitOwner(osThreadId_t)`, called once at the top of `ControlTaskRun`. If an owner is set, `DeviceConfig_ApplyPatch()` returns -1 when `osThreadGetId() != owner`.
3. **Generation counter.** Add `DeviceConfig_GetGeneration()`, which is incremented on each commit. It's cheap change detection for later readers (UI, persistence debounce).
4. **API cleanup.** Remove the unused `DeviceConfig_SetIdentity/SetNetwork/SetLayout/SetDmx/SetOutput`, `RegisterCallback`, `Config_NotifyCallbacks` and `GetSnapshot`, which bypass or duplicate the patch path (re-checked 2026-11-15: there are no callers). Keep `FactoryReset` for the UI, but restrict it to the owner as well.
5. **Header docs.** Document the reader rule and the writer rule on `DeviceConfig_Get()` / `ApplyPatch()`.

**Verify:** the build shows only the baseline warning. On the bench: ArtCommand still works, but **only after Phase B**, because between A and B the owner check rejects commits from `tcpip_thread`. So A and B land as one commit.

## Phase B — `config_service` + ControlTask loop + ArtCommand

**Placement:** a new module `Core/Src/Config-Service/config_service.c` with `Core/Inc/Config-Service/config_service.h`. It's added to `target_sources`. It owns the request queue and the apply sequencing, which is control-plane policy. Rejected alternatives: putting the queue in `device_config` would mix RTOS policy into the data module, which the refactor plan says not to do. Putting it in `main.c` USER CODE would grow a generated file.

**API sketch**

```c
typedef struct {
    int      status;        // 0 = committed (or no change), -1 = rejected
    uint32_t change_mask;   // DEVICE_CONFIG_CHANGE_*
} ConfigService_Result_t;

/** Runs in ControlTask after the commit. cookie is the requester's own handle. */
typedef void (*ConfigService_DoneFn_t)(void *cookie, const ConfigService_Result_t *result);

typedef struct {
    DeviceConfig_Patch_t   patch;
    ConfigService_DoneFn_t done;     // NULL = no reply wanted (e.g. the UI later)
    void                  *cookie;   // opaque to config_service
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
      - any bit → `ArtNet_OnConfigChanged(mask)`
      - any bit → the OLED redraws on this loop pass
   3. `req.done(req.cookie, &result)` if `done` is set
   4. Bump the counters `config_commit_count` / `config_reject_count` / `config_queue_full_count` (Live Watch).
3. **Reject runtime input-source changes** (`DMX_SOURCE`) in `Process` for now. Nothing can request one yet. Since the control-plane split a switch no longer touches the Art-Net socket (`ArtNet_EnableOutput/DisableOutput` only toggle the output part), but it still needs `EffectTask` quiesced and the stale frame flag cleared (`artnet-audit-followups.md` item 12). This is noted for the UI step.
4. **Art-Net change hook.** Add `void ArtNet_OnConfigChanged(uint32_t mask)` to `artnet.h` (ControlTask only). It takes `LOCK_TCPIP_CORE()` and:
   - if `mask & (ARTNET_ADDRESS | LAYOUT | DMX_START_ADDRESS)` and the output is enabled → `ArtNet_ResetUniverseState()` (new, `artnet_dmx.c`): clear `source_ip`, `sync_mode`, `last_sequence` for all universes, and keep the buffers (decision 5). The port set or its Port-Addresses may have changed, so the old source locks no longer apply.
   - Phase C adds the Status1 address authority here, Phase E the reply-on-change events.

   `ArtNet_EnableOutput` already clears the same state with a `memset`; it calls the new helper instead.
   **Rejected alternative:** an optional `on_config_changed` entry in the `dmx_input` vtable (the first version of this plan). After the split it would only reach the active driver, so the always-on node would miss changes in DMX512 mode, and DMX512 itself needs no hook.
5. **ControlTask loop.** Replace `osDelay(OLED_STATUS_UPDATE_PERIOD_MS)` with `ConfigService_Process(time until next 1 Hz tick)`. Redraw the OLED when the 1 Hz tick is due, or right after a commit. `OledStatus_RunStep()` stays as it is: it starts the I2C DMA flush and then sleeps in `ssd1306_WaitForUpdate()` until it completes (about 25 ms: 1 KB at 400 kHz). A request arriving during that window waits for it. Making the flush non-blocking is deliberately left to the menu system (roadmap, Local UI), which rewrites this code and needs it anyway for button responsiveness.
6. **EffectTask.** Also wait for flag `0x04` (`0x02` is the start flag, `0x01` the frame flag). On `0x04`:
   - if `failsafe_applied`, call `Pixel_Render_Failsafe()` again (picks up a new failsafe mode)
   - otherwise, if `first_frame_received`, call `Pixel_Render_Frame()`, which re-renders the active buffer with the new layout or address without waiting for new DMX

   It doesn't latch. Name the flag `EFFECT_TASK_FLAG_CONFIG_CHANGED`.
7. **Art-Net reply slots** (`artnet_internal.h`, `artnet_command.c`):
   - `ArtNet_ConfigReply_t { bool in_use; ip_addr_t ip; u16_t port; uint8_t kind; uint8_t hint; }`, an array of `ARTNET_CONFIG_REPLY_SLOTS` (4, the queue depth) in `g_artnet_ctx`. `kind` is ArtCommand or ArtAddress; `hint` records which ArtCommand verbs were present, for the failure detail.
   - The slots are tcpip-owned: allocated in `tcpip_thread` (the handler) and freed under the core lock (`ArtNet_OnConfigDone`). No other code touches them.
   - Shared helpers `ArtNet_ConfigReplyAlloc()` and `ArtNet_SubmitConfig(slot, patch)` live in `artnet_command.c` for Phase B. Phase C's ArtAddress uses them too; if they grow, they move to their own file then, not before.
8. **ArtCommand migration** (`artnet_command.c`):
   - The parse and validation of the text stays in `tcpip_thread`; it's bounded and cheap.
   - Parse errors still reply immediately with `RcParseFail` / `RcConfigErr`, as today.
   - Otherwise: allocate a slot (none free → reply `RcConfigErr "Config Busy"`), then `ConfigService_Submit(patch, ArtNet_OnConfigDone, slot)`. If the queue is full, free the slot and reply `"Config Busy"`. Return without replying.
   - `ArtNet_OnConfigDone(cookie, result)` runs in ControlTask. Under `LOCK_TCPIP_CORE()` it maps the result to a NodeReport code and detail (success detail from `change_mask`, which is more accurate than today's pre-commit comparison; failure detail from the slot's `hint`), calls `ArtNet_ReplyImmediateStatus(&slot->ip, slot->port, …)`, and frees the slot. Detail strings are literals, so nothing is copied.
   - **Rejected alternative:** `done` stores the result in the slot and posts `tcpip_callback(ArtNet_SendConfigReply, slot)` (first version of this plan). It needs a `tcpip_callback` message per reply from the shared `MEMP_NUM_TCPIP_MSG_API` pool (8) and a result hand-off between threads. The core lock does the same job with no allocation, and ControlTask already uses it.

**Verify (bench)**

- `tools/artnet_bench_test.py all` in Art-Net mode: still 58/58. ArtCommand replies arrive in under 100 ms. The script's restore uses a 0.8 s window.
- `--mode dmx512 all` with the temporary DMX512 default (as in groundwork commit 3, not committed): 58/58. ArtCommand goes through the queue in that mode too.
- `SegmentCount=12&` with **no** Lightkey running: the strip re-renders immediately; before, it waited for the next frame.
- Changing the failsafe mode while in failsafe can't be triggered from the network yet; it gets tested in Phase C with AcFail*.
- Live Watch:
  - `config_commit_count` increments once per accepted ArtCommand
  - `config_reject_count` increments on an invalid value
  - `stack_monitor_report`: `ControlTask` min free stack (the candidate copy, the request and now the ArtPollReply build live there; the reply build uses the bounded text builder, not `snprintf`) and `tcpip_thread` (lighter than before)

**Docs:**
- CLAUDE.md: the threading table (ControlTask is the only config writer and sends config replies under the core lock) and the module map (`config_service`).
- `dmx-input-drivers.md`: config changes reach Art-Net through `ArtNet_OnConfigChanged`, not the vtable; a future driver that needs them adds a hook then.
- Roadmap: "Serialized config commits" and "Move ArtCommand … out of tcpip_thread" → Done.
- Compliance 11.1 → Done. Follow-ups item 2 → done.

## Phase C — ArtAddress (config fields + transient commands)

**Placement:** `ArtNet_HandleArtAddress()` in `artnet_handlers.c` (the protocol mapping). `ArtNet_ResetUniverseState()` already exists in `artnet_dmx.c` from Phase B. Rejected alternative: a separate `artnet_address.c`. About 150 lines doesn't justify a module, and `artnet_handlers.c` is about 265 lines since the groundwork. Use one Port-Address helper for SwOut contiguity, ArtDmx and ArtPoll (item 4).

**Accept rules**

| Check | Action |
|---|---|
| `len < 107` | `RcParseFail "ArtAddress Too Short"` (unchanged) |
| `BindIndex` not 0/1 | Silent ignore, no reply, like ArtInput |

**Fields → one `DeviceConfig_Patch_t`**

| Field | Rule (spec: bit 7 = program, 0x00 = reset to "physical switch" = factory default, anything else = no change) |
|---|---|
| ShortName [18] / LongName [64] | Non-empty → patch. Copy bounded and NUL-terminate. Every input mode. |
| NetSwitch | `0x80 \| n` → `artnet_net = n`. `0x00` → default. Every input mode (decision 6). |
| SubSwitch | `0x80 \| n` → `artnet_subnet = n`. `0x00` → default. Every input mode. |
| SwOut[0] | `0x80 \| u` → `artnet_start_universe = u`. `0x00` → default. Only while `ArtNet_GetPortCount() > 0`. |
| SwOut[1..num_ports-1] | If programmed, it must equal `start + i` (contiguous). Otherwise reject the whole packet with `RcConfigErr "SwOut Not Contiguous"`. Ports beyond `num_ports = ArtNet_GetPortCount()` are ignored, so in DMX512 mode all of SwOut is ignored (decision 6). |
| SwIn, AcnPriority | Ignored (no inputs, no sACN generation). |

The existing validation rejects Port-Address 0 and rollover past universe 15. Those surface as `RcConfigErr` with an unchanged state.

**Command byte**

| Command | Where | Effect |
|---|---|---|
| `AcNone` 0x00 | — | No-op. |
| `AcCancelMerge` 0x01 | tcpip_thread, immediately | `ArtNet_ResetUniverseState()`. The next ArtDmx sender owns each universe (backup-console takeover). Ignored while the output is disabled. |
| `AcLedNormal/Locate` 0x02/0x04 | Phase D | Until then: ignored. |
| `AcLedMute` 0x03 | Menu system (roadmap) | Ignored until the menu owns the display. |
| `AcResetRxFlags` 0x05 | tcpip_thread | NodeReport → `RcPowerOk`. |
| `AcFailHold/Zero/Full` 0x08–0x0A | Config patch, every input mode | `output.failsafe`. Runtime-only until persistence, so **Status3 bit 5 stays 0** until persistence (§12 step 4, `flash-persistence.md`). |
| `AcClearOp0` 0x90 | tcpip_thread | Zero the shadow buffers (under `shadow_mutex`), then `ArtNet_TriggerFrameOutput()` so EffectTask latches and renders black. Ignored while the output is disabled. |
| Everything else (scene, analysis, merge, direction, ArtNetSel/AcnSel, style, RDM, deprecated port 1–3 codes) | — | Ignored. The reply shows the unchanged state, which is how a controller sees it wasn't applied. |

**Reply**

- **The patch is empty** (command only): reply immediately, as today's `ArtNet_ReplyImmediateStatus`.
- **Otherwise:** allocate a slot with `kind = ARTNET_REQ_ADDRESS` and submit, as ArtCommand does. `ArtNet_OnConfigDone` picks the NodeReport:
  - rejected → `RcConfigErr` + detail
  - short name changed → `RcShNameOk`
  - else long name changed → `RcLoNameOk`
  - else → `RcPowerOk "Address Programmed"`
- The reply is always a full ArtPollReply with the **resulting** state.

**Status1 bits 5-4** (Port-Address authority): `g_artnet_ctx.address_authority`. It is 01 at boot (defaults / front panel) and 10 after any network-originated commit that changed `ARTNET_ADDRESS`. Set in `ArtNet_OnConfigDone` (it knows the origin), under the core lock. It's runtime-only; revisit with persistence.

**Verify (bench).** Run this test matrix through a new `address` test in `tools/artnet_bench_test.py`:

- rename the short name only, then the long name only
- Net + Sub change
- contiguous SwOut
- non-contiguous SwOut rejected
- rollover rejected
- Port-Address 0 rejected (finally testable on the bench)
- Fail Hold/Zero/Full: stop Lightkey output and watch the strip after 6 s
- CancelMerge
- ClearOp
- ResetRxFlags
- BindIndex 2 ignored

In `--mode dmx512`, the same test checks decision 6: names, Net/Sub and AcFail* apply; SwOut, CancelMerge and ClearOp change nothing; the reply still shows NumPorts = 0.

Then interop: change the universe from DMX-Workshop (Windows, Artistic Licence) if available, and check that Lightkey output on the **new** universe drives the strip immediately.

Live Watch: `failsafe_count`, `render_count`, `config_commit_count`, and `stack_monitor_report` for `tcpip_thread` (the ArtAddress handler has a ~150 B patch on its stack).

**Docs:**
- Compliance §6 rows → Done/Ignored as implemented. Status1 5-4 → Done.
- Roadmap: ArtAddress → Done.
- `artnet-custom-protocol.md`: a note that ArtAddress is standard, the NodeReport codes it produces, and what applies in DMX512 mode.
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
   - In `ArtNet_HandleArtAddress`: `AcLedLocate` → `Indicator_Set(LOCATE)`, `AcLedNormal` → `NORMAL`. These run immediately in `tcpip_thread`, aren't config, and work in every input mode (decision 6).
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
- Locate in `--mode dmx512`: flashes, and Normal brings back the DMX512 output.
- Live Watch: `stack_monitor_report` for EffectTask.

## Phase E (optional, can split off) — ArtPoll Flags bit 1, reply on change

**Placement:** the controller registry in `g_artnet_ctx` (11.5), part of the always-on node. Change events come from `ArtNet_OnConfigChanged` (config commits from any origin, in every input mode) and from AcLed* (11.6).

1. Registry: up to 4 entries of `{ip, port, flags, last_poll_tick}`, refreshed on every ArtPoll and expired after 10 s without a poll.
2. On a change event: queue an ArtPollReply to every entry with bit 1 set, through the existing reply queue with no random delay.
3. Compliance §3 Flags bit 1 → Done.

**Verify:** the script polls with flags 0x02, then a config change from a second socket → an unsolicited ArtPollReply arrives on the first.

---

## Order and commits

| Commit | Content | Bench gate |
|---|---|---|
| 1 | Phase A + B (A alone breaks ArtCommand) | Script 58/58 in both modes, immediate re-render on SegmentCount |
| 2 | Phase C | `address` test matrix, both modes |
| 3 | Phase D | `locate` test + visual |
| 4 | Phase E (optional) | Unsolicited reply test |

Each commit: build with only the baseline warning and an unchanged memory table apart from the new static state (the queue is about 650 B of `.bss`, the reply slots under 100 B). Docs are updated in the same commit.

## Risks

- **ControlTask stack (2048 B).** `ApplyPatch` puts a ~200 B candidate on the stack, `Process` adds a ~150 B request, and `ArtNet_OnConfigDone` now builds the ArtPollReply there. These don't nest (the candidate is gone before `done` runs), so the peak is the largest of them, not the sum. Watch `stack_monitor_report` after Phase B. The fix is the `.ioc` stack size: regenerate, don't hand-edit.
- **Core lock held by ControlTask.** While `ArtNet_OnConfigDone` holds `LOCK_TCPIP_CORE()`, `tcpip_thread` (Normal priority) waits; ArtDmx queues in the mbox (6). FreeRTOS mutexes inherit priority, so ControlTask (BelowNormal) runs at Normal meanwhile. The hold is one ArtPollReply build and two `udp_sendto` calls, the same work `tcpip_thread` does for an immediate reply today.
- **Reply latency** for config commands: about 1 ms usually, up to about 30 ms when a request lands during the 1 Hz OLED flush (about 2.5 % of the time), because `ControlTask` sleeps until the I2C DMA transfer completes. The CPU is free meanwhile, the strip updates at commit, and controllers allow 3 s, so this is accepted until the menu system makes the flush non-blocking.
- **"Config Busy" under bursts.** Four slots and four queue entries: a fifth config packet before ControlTask catches up gets `RcConfigErr "Config Busy"`. Controllers send config one packet at a time, so this only shows under test floods.
- **AcFail\* without persistence** is reset by a reboot. That's acceptable for now; Status3 bit 5 isn't advertised until persistence (§12 step 4).
- **RDM and the menu system later.** Keep the indicator module and the commit path protocol-agnostic: RDM `IDENTIFY_DEVICE`, `DMX_START_ADDRESS` and `DMX_PERSONALITY` will reuse `Indicator_Set()` and `ConfigService_Submit()` unchanged (with their own cookie), and the menu reads `Indicator_Get()` for the corner dot and mute.
