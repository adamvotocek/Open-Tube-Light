# Plan: Art-Net groundwork (always-on node, file split, 6 s failsafe)

**Status (2026-11-15):** commits 1–3 done (1–2 bench 57/57 with Lightkey; 3 bench 58/58 in Art-Net mode and 58/58 with `--mode dmx512`), commit 4 open. This is `artnet-compliance.md` §12 step 2. It comes before `config-commits-and-artaddress.md` and implements `artnet-audit-followups.md` items 1, 3, 5 and 11.

## In short

Today the Art-Net "node" (the UDP socket, discovery, ArtCommand) only exists while Art-Net is the selected input. In DMX512 mode the tube is invisible on the network and OpenTubeDia can't configure it. After this plan:

- The **node** starts once at boot and never stops. It answers ArtPoll, ArtCommand, ArtInput and ArtDataRequest in **every** input mode.
- The **data part** (ArtDmx, ArtSync, the DMX buffers) is the only thing the `dmx_input` driver switches on and off.
- `artnet_handlers.c` (896 lines) is split by job, so the config-service work that follows touches small, focused files.
- The Art-Net failsafe timeout becomes 6 s.

Nothing changes for a tube in Art-Net mode: the bench script must stay at 57/57.

## Decision

**What the ArtPollReply says while the tube is in DMX512 mode.** Decided 2026-11-14: **NumPorts = 0** (no Art-Net ports, so the port arrays are empty). The node still answers polls with its name, IP, MAC and NodeReport, so OpenTubeDia sees it. Controllers don't list it as an Art-Net output and don't send ArtDmx to it.

- Rejected alternative: keep reporting the ports, just never with "data" (GoodOutputA bit 7 = 0). Controllers would then patch universes to a tube that ignores them, which is confusing.
- OpenTubeDia must cope with an empty port list in DMX512 mode. Note it in `artnet-custom-protocol.md`.

## Target file layout

| File | Owns | Today |
|---|---|---|
| `artnet.c` | **Node:** `ArtNet_NodeInit()`, UDP socket, dispatcher, NodeReport, reply sending, PRNG | Also holds init/deinit of everything, buffers, latch, the driver vtable |
| `artnet_dmx.c` (new) | **Data part:** shadow/active buffers, `ArtNet_EnableOutput/DisableOutput`, latch, `get_universe`, the `dmx_input_artnet` vtable, ArtDmx and ArtSync handlers | Spread over `artnet.c` and `artnet_handlers.c` |
| `artnet_command.c` (new) | The transitional ArtCommand text parser (OpenTubeDia verbs). Deleted as one file when RDM replaces it. | ~385 lines in `artnet_handlers.c` |
| `artnet_handlers.c` | Control packets: ArtPoll, ArtAddress, ArtInput, ArtDataRequest (~270 lines) | Also ArtDmx, ArtSync, ArtCommand |
| `artnet_pollreply.c` | ArtPollReply build/send **and** the reply queue, now including `ArtNet_ReplyImmediate(Status)` (shared) and `ArtNet_RemoveQueuedPollReply` (private) | Those three are `static` in `artnet_handlers.c` |

Placement reasoning: each file owns one kind of state. The data part owns the buffers and per-universe tracking that only matter while Art-Net is the input. The reply queue helpers move next to the queue they edit, because ArtCommand, ArtAddress and ArtInput all use them. Rejected alternative: keep the data part in `artnet.c` behind an "enabled" flag. It is fewer files, but the node file would again mix always-on and switchable state, which is what this plan untangles.

## Commits

Four commits, each built and bench-tested before the next. The first two are **pure moves**: code is cut and pasted, with only `static` ↔ shared declarations changed. That keeps their diffs easy to review and makes commit 3 contain only the real behaviour change.

| # | Commit | Behaviour change | Bench gate |
|---|---|---|---|
| 1 | Move ArtCommand parser → `artnet_command.c` | None | `all` 57/57 |
| 2 | Move data part → `artnet_dmx.c` | None | `all` 57/57 |
| 3 | Always-on node, switchable data part, narrow `artnet.h` | Node runs in DMX512 mode | Art-Net mode 57/57 + DMX512-mode checks |
| 4 | Failsafe 6 s | Failsafe 1 s later | Stopwatch test |

### Commit 1: ArtCommand parser → `artnet_command.c` (pure move)

1. New `Core/Src/Art-Net/artnet_command.c` with the file-header convention. Move these from `artnet_handlers.c`:
   - `ArtNet_HandleArtCommand`
   - the `ARTNET_COMMAND_*` constants
   - all parser helpers: `IsWhitespace`, `GetNullTerminatedLength`, `TrimSpan`, `SpanEqualsIgnoreCase`, `ParseUint16Span`, `ParseCommandPayload`, `GetCommandSuccessDetail`, `GetCommandApplyErrorDetail`

   They stay `static`, with prototypes at the top and definitions under the Private Functions banner.
2. Move `ArtNet_ReplyImmediate`, `ArtNet_ReplyImmediateStatus` and `ArtNet_RemoveQueuedPollReply` to `artnet_pollreply.c`. The first two become non-static and are declared in `artnet_internal.h` (ArtPollReply section), because the handlers and the command file both call them. `ArtNet_RemoveQueuedPollReply` is only used by `ArtNet_ReplyImmediate`, so it stays `static`.
3. Add `artnet_command.c` to `target_sources` in the root `CMakeLists.txt`.
4. Update the file headers of `artnet_handlers.c` and `artnet.c` (which file handles what).

### Commit 2: data part → `artnet_dmx.c` (pure move)

1. New `Core/Src/Art-Net/artnet_dmx.c`. Move these into it:
   - from `artnet.c`: `g_artnet_shadow_buffer`, `g_artnet_active_buffer`, `ArtNet_GetUniverseData`, `ArtNet_LatchData`, `ArtNet_TriggerFrameOutput`, the `dmx_input_artnet` vtable
   - from `artnet_handlers.c`: `ArtNet_HandleArtDmx`, `ArtNet_HandleArtSync`
2. `ArtNet_Init/Deinit` stay in `artnet.c` for now. The vtable keeps pointing at them through their existing declarations. They clear the buffers with `sizeof`, so the `extern` declarations in `artnet_internal.h` now carry the full array size (the compiler then also checks them against the definition).
3. Add it to `target_sources`.

### Commit 3: always-on node, switchable data part

**`artnet.c` (node)**
- `int ArtNet_NodeInit(void)` replaces `ArtNet_Init`. It runs once and creates:
  - the PCB (under `LOCK_TCPIP_CORE`, as today)
  - the poll-reply timer
  - the shadow mutex
  - the PRNG seed
  - NodeReport "Power On OK"

  A second call returns 0.
- `ArtNet_Deinit` is deleted. The node lives as long as the firmware.
- **The shadow mutex is never deleted any more.** That removes the "deinit deletes the mutex `latch` is waiting on" half of follow-up item 12. The other half (non-atomic `active_driver` swap) stays with the UI/input-switching step.
- Dispatcher: when `!g_artnet_ctx.output_enabled`, ArtDmx and ArtSync are dropped silently and counted in `artnet_ignored_count`, before any length check. Dropping them must not touch the NodeReport.

**`artnet_dmx.c` (data part)**
- `int ArtNet_EnableOutput(osThreadId_t task)` becomes the vtable `init`. It returns -1 if the node isn't initialized. Under `LOCK_TCPIP_CORE` it:
  - zeroes `universes[]`
  - sets `processing_task = task`
  - sets `output_enabled = true`

  Before that it zeroes both buffers under `shadow_mutex`, so a new session never shows stale universes.
- `void ArtNet_DisableOutput(void)` becomes the vtable `deinit`. Under the core lock it sets `output_enabled = false`, zeroes `universes[]` and sets `processing_task = NULL`. It leaves the buffers alone: a switch counts as input lost, the strip holds its last frame and the failsafe policy applies (zeroing here could tear a frame `EffectTask` is rendering). The DMX512 driver follows the same rule. Names: `EnableOutput/DisableOutput` because, in Art-Net terms, the switched thing is the node's output ports (NumPorts, SwOut, ArtDmx = OpOutput); "Input" would clash with ArtInput, which works in the opposite direction.
- `output_enabled` is a new `bool` in `ArtNet_Context_t`. It is tcpip-owned: written only under the core lock and read in `tcpip_thread`. The lock orders the accesses, so no atomics are needed.

**Port count in one place**
- Add `uint8_t ArtNet_GetPortCount(void)` in `artnet.c`, declared internal. It returns `output_enabled ? DeviceConfig_GetUniverseCount() : 0`.
- Use it in `ArtNet_SendPollReply` (NumPorts and the port loop) and in `ArtNet_HandleArtPoll` (targeted mode: with 0 ports no range matches, so there is no reply, which is correct).
- This is the first step towards follow-up item 4's Port-Address helpers. Don't do item 4 itself here; it belongs to Phase C.

**Boot order (`main.c`, `ControlTaskRun` USER CODE)**
- `MX_LWIP_Init()`, then `ArtNet_NodeInit()`, then `DMX_Input_Start(...)`. Include `Art-Net/artnet.h`.
- If `ArtNet_NodeInit()` fails (LwIP out of memory), carry on. DMX512 still works, and `DMX_Input_Start(ARTNET)` then fails as it would today.

**Narrow `artnet.h` (follow-up item 5)**
- Public: only `ArtNet_NodeInit()`, plus the Doxygen for it. The `dmx_input_artnet` driver symbol stays declared in `dmx_input.h`.
- Move `ArtNet_UniverseState_t`, `ArtNet_GetUniverseData`, `ArtNet_LatchData`, `ArtNet_EnableOutput/DisableOutput` to `artnet_internal.h`. `artnet.h` then no longer pulls in LwIP headers.

**`dmx_input.h`:** the vtable comment "init/deinit are called from a thread with LwIP access" becomes "init/deinit are called from `ControlTask`". The Art-Net driver no longer creates network resources.

### Commit 4: failsafe 6 s

- `artnet_internal.h`: `ARTNET_FAILSAFE_TIMEOUT_MS 6000U`. The comment says why: a 4 s keep-alive plus jitter, faster than the spec's 10 s merge timeout. `ARTNET_DISCONNECT_TIMEOUT_MS` stays at 5 s, so universe takeover and GoodOutputA are unchanged; the static assert still holds.
- `dmx_input.h`: the timeout comment "Art-Net=5000" becomes 6000.

## Verification

**Every commit**
- Build: zero new warnings (baseline: `Config_ValidateFlash`). Memory table unchanged apart from one `bool`.
- Bench in Art-Net mode with Lightkey streaming: `python tools/artnet_bench_test.py --node 2.106.12.177 --bind 2.0.0.50 all` → 57/57. The strip follows Lightkey. Live Watch: `render_count` rises, and `stack_monitor_report` shows no drop in `tcpip_thread` minimum free stack.

**Commit 3, DMX512 mode.** There is no runtime source switch or persistence yet. For this test only, build with `.input_source = DMX_INPUT_DMX512` in the defaults in `device_config.c`, and don't commit that change.
- The script gets a `--mode dmx512` option (committed with commit 3). `poll` then expects NumPorts = 0 and skips the GoodOutputA hint. Everything else is unchanged.
- `python tools/artnet_bench_test.py ... --mode dmx512 all` passes: poll, ignored, input, datareq, replyport, parsefail. ArtCommand works in DMX512 mode; that is the point of the change.
- ArtCommand `DmxAddress=…` changes the DMX512 start address (NodeReport `@aaa`, OLED).
- Lightkey keeps streaming Art-Net: `artnet_ignored_count` rises with every ArtDmx, and the strip does **not** follow Lightkey.
- If a DMX512 source is available: the strip follows it, and `receivedData` rises. Without one, the strip stays in failsafe, and `failsafe_count` rose once after boot.
- Lightkey's patch window doesn't show the tube as an Art-Net output (expected).

**Commit 4**
- `monitor` test, stop Lightkey output: GoodOutputA clears after about 5 s (unchanged).
- Stopwatch: the strip changes to the failsafe output about 6 s after Lightkey stops. `failsafe_count` +1.

## Docs (in the same commits)

- Commit 1: `.claude/CLAUDE.md` module map (`artnet_command.c`). Follow-ups item 3 → done.
- Commit 2: CLAUDE.md module map (`artnet_dmx.c`).
- Commit 3:
  - CLAUDE.md: the architecture diagram shows the always-on node beside the `dmx_input` path, plus the threading notes.
  - `dmx-input-drivers.md`: the Art-Net driver is now data-only.
  - `artnet-compliance.md`: §3 "Respond with ArtPollReply while the input source is DMX512" → Done; 11.4 → Done; §12 step 2 → done.
  - `artnet-custom-protocol.md`: DMX512 mode (NumPorts 0, ArtCommand still works).
  - Follow-ups items 1 and 5 → done; item 12 updated (mutex half solved).
  - Roadmap: "Art-Net control plane always on" → Done.
  - Repo `README.md` module table.
- Commit 4: compliance §2 row (6 s), `dmx-input-drivers.md` table, follow-ups item 11 → done, roadmap robustness line.

## Risks

- **Pure moves that aren't pure.** If a moved function turns out to need a helper from its old file, make that helper internal; don't duplicate it. Review the diff with `git diff --color-moved` to confirm commits 1–2 only move code.
- **Hidden reliance on `ArtNet_Deinit`.** Nothing calls `DMX_Input_Stop()` or switches sources at runtime today, so deleting the node teardown is safe. Re-check with a grep before deleting.
- **Lightkey or other controllers** may cache a node they saw with ports. After a reboot into DMX512 mode, the next ArtPollReply shows 0 ports, and they update on their next poll.
- **OpenTubeDia** may assume at least one port. Check it against an empty port list before relying on DMX512 mode with the app.
