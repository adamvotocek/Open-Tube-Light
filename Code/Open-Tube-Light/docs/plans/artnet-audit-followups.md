# Art-Net audit: follow-ups

Findings from the Art-Net code and docs audit (2026-11-14) that the cleanup commit did not fix, with a recommendation and a slot for each. The small fixes are listed first so this file also records what changed.

## Done in the cleanup commit

- Plans are tracked (`docs/plans/` is no longer gitignored). The bench script moved to `tools/artnet_bench_test.py`. Lightkey/QLC+ scenes stay untracked in the repo-root `Temp_Scenes/`.
- Order: network configuration (11.3) now comes before RDM (11.12) in `artnet-compliance.md` §12 and the roadmap.
- `ArtNet_Init` / `ArtNet_Deinit` hold `LOCK_TCPIP_CORE()` around `udp_new/bind/recv/remove` and the tcpip-owned state they reset. They used to call raw LwIP from `ControlTask` without the lock.
- The ArtPollReply PRNG seed mixes in the UID-derived MAC, so fixtures powered up together no longer pick the same reply delays.
- The poll-reply chain can no longer wedge. If `osTimerStart` fails, the queue is dropped (the next poll re-arms it); if `tcpip_callback` fails, the timer retries after 10 ms.
- The UDP callback drops chained pbufs (`p->len != p->tot_len`), because handlers read `p->payload` as one buffer.
- Named minimum lengths (`ARTNET_DMX_HEADER_LENGTH`, `ARTNET_POLL_MIN_LENGTH`, `ARTNET_POLL_TARGETED_MIN_LENGTH`) with `offsetof` static asserts replace the bare 18/14/18.
- `ARTNET_FAILSAFE_TIMEOUT_MS` replaces the literal 5000 in the vtable, with a static assert that the disconnect timeout is not longer.
- `DMX_INPUT_FLAG_FRAME_READY` is defined once in `dmx_input.h`. Art-Net, DMX512 and `EffectTask` each had their own `0x01`.
- `DeviceConfig_BuildArtNetOutputAddressList()` is removed. It had no callers and was the only reason `device_config.c` included an Art-Net header.
- Stale comments rewritten: `device_config.h/.c` (flash, callbacks, "CRC32"), the ArtAddress stub, the `artnet.c` file header, and typos and wrong `@param`s in `artnet_protocol.h`. Static helpers in `artnet_pollreply.c` now follow the prototype-at-top convention. The `pixel_render.c` loop indentation is fixed.

**Bench-verified 2026-11-14:** `tools/artnet_bench_test.py` 57/57 over a dedicated 2.x NIC while Lightkey streamed to the tube.

---

## Architecture

### 1. Pull the control/data-plane split (compliance 11.4) forward

**Finding.** `dmx_input_artnet.init/deinit` creates and destroys the UDP socket, poll timer, mutex and node state. The Art-Net node only exists while Art-Net is the input source. Three planned features depend on a node that is always there:
- Phase E (reply on change) hooks a `dmx_input` callback that only fires in Art-Net mode.
- RDM and remote configuration are most useful in DMX512 mode.
- Runtime input switching would tear the socket down at runtime.

**Recommendation.** Do 11.4 before Phase B, or as part of it.
- **Boot:** `ArtNet_NodeInit()` once from `ControlTask`. It owns the PCB, ArtPoll/PollReply, ArtCommand, ArtAddress and ArtDataRequest.
- **Data part:** the `dmx_input` driver only enables ArtDmx/ArtSync processing and owns the buffers.
- **DMX512 mode:** NumPorts = 0, or GoodOutputA.7 = 0.

The socket is then never removed at runtime, and `ArtNet_ResetUniverseState` and the Phase E events have one fixed home. **Slot:** before or with Phase B.

### 2. Config service: let Art-Net own its reply slots

**Finding.** In `config-commits-and-artaddress.md` Phase B, `ConfigService_ReplyTo_t` carries `ipv4/port/kind/hint`, and `ArtNet_OnConfigDone()` picks a reply slot in `g_artnet_ctx` from `ControlTask`. That is a cross-thread write into tcpip-owned state, and Art-Net details leak into a generic header.

**Recommendation.** Allocate the slot in `tcpip_thread` at submit time (IP, port, kind, verb hint), and pass an opaque `void *cookie` (or a slot index) through the request. `done()` stores only the result into the slot and calls `tcpip_callback(ArtNet_SendConfigReply, slot)`. If no slot is free, reply `RcConfigErr "Config Busy"` without submitting. **Slot:** Phase B; amend the plan before implementing.

### 3. Split the ArtCommand parser out of `artnet_handlers.c`

**Finding.** `artnet_handlers.c` is about 900 lines, past the 800-line split threshold in the plan, and about 400 of them are the transitional ArtCommand text parser.

**Recommendation.** Move `ArtNet_HandleArtCommand` and its parser helpers into `artnet_command.c` as a pure move commit before Phase B. Phase B rewrites this handler, so the move keeps that diff readable, and the later RDM deprecation becomes a one-file delete. **Slot:** a separate commit just before Phase B.

### 4. One Port-Address mapping helper

**Finding.** Port-Address ↔ port index is computed by hand in three places: `ArtNet_HandleArtDmx` (decompose and subtract), `ArtNet_HandleArtPoll` (targeted mode) and `ArtNet_SendPollReply` (SwOut). ArtAddress (SwOut contiguity) adds a fourth.

**Recommendation.** Add `ArtNet_PortAddress(cfg, i)` and `ArtNet_PortIndexOf(cfg, pa)` in `artnet_internal.h` and use them everywhere. The 288-pixel / multi-bind work (11.9) then changes one place. **Slot:** Phase C.

### 5. Narrow `artnet.h`

**Finding.** Only `artnet_internal.h` includes `artnet.h`. The module's real public surface is the `dmx_input_artnet` symbol, yet `artnet.h` exposes `ArtNet_UniverseState_t` and LwIP types.

**Recommendation.** Move `ArtNet_UniverseState_t` into `artnet_internal.h`. Leave `artnet.h` with what `main.c` / `ControlTask` will call after the split (`ArtNet_NodeInit`, config-done entry points). **Slot:** with item 1.

### 6. Remaining unused config API

`DeviceConfig_Set*`, `DeviceConfig_RegisterCallback` / `Config_NotifyCallbacks` and `DeviceConfig_GetSnapshot` have no callers. Phase A item 4 already removes the first two groups; add `GetSnapshot` to that list. **Slot:** Phase A.

## Robustness

### 7. Poll-reply timer: use `sys_timeout()`

The cleanup made the osTimer path recoverable, but it still hops through the lowest-priority timer task and then the tcpip mbox. LwIP `sys_timeout()` runs the callback in `tcpip_thread` directly, so there is no osTimer, no `tcpip_callback`, and no daemon-priority latency. It needs one more `MEMP_NUM_SYS_TIMEOUT` entry: set it in CubeMX (LwIP → Key Options) and regenerate. **Slot:** any time; it also simplifies Phase E.

### 8. Sequence-number resync

A controller that restarts its sequence counter (console reboot) is rejected as "stale" until the 5 s disconnect timeout clears `last_sequence`. **Recommendation:** accept the packet and resync after N consecutive rejects (e.g. 3), or when the last accepted packet is older than about 1 s. Count the rejects (item 9). **Slot:** small standalone commit with a bench test (send seq 200, then restart at 1).

### 9. Art-Net stats for Live Watch

Drops and send failures are silent:
- `udp_sendto` errors and `pbuf_alloc` failures
- ArtDmx dropped by the source lock or the sequence check
- parse failures

This matters on 2.x.y.z/8 with no gateway, where replies to a controller on another subnet fail to route. **Recommendation:** one `ArtNet_Stats_t artnet_stats` struct next to `artnet_ignored_count`, with the counters above. List them in CLAUDE.md's Live Watch line. **Slot:** standalone, before Phase C (ArtAddress testing benefits).

### 10. Sticky NodeReport

One malformed packet from any host leaves `#0004 RcParseFail` in every later ArtPollReply until something else sets the report. `AcResetRxFlags` (Phase C) clears it explicitly. **Recommendation:** also revert to `RcPowerOk` after a timeout (e.g. 10 s) unless the code is a config result. **Slot:** Phase C.

### 11. Failsafe timeout 5 s → about 10 s?

A controller sending static data may resend only every 4 s (spec keep-alive), so 5 s tolerates no lost packet. The spec's own source-loss timeout for merging is 10 s. **Recommendation:** raise `ARTNET_FAILSAFE_TIMEOUT_MS` to about 9–10 s. Keep the source-lock release (`ARTNET_DISCONNECT_TIMEOUT_MS`) at 5 s so takeover stays quick; the static assert allows that. Update README's merge-policy text if the takeover time changes. **Slot:** decision only; one-line change.

### 12. Runtime input switching races

`DMX_Input_Start/Stop` call `deinit()` while `EffectTask` may be inside `latch()` (Art-Net deletes its mutex), and `active_driver` is not published atomically. Nothing switches at runtime today. **Recommendation:** when the UI adds switching, quiesce `EffectTask` first (a flag plus a handshake, or do the switch from `EffectTask` itself), then swap the driver. With item 1 done, the Art-Net socket is not touched by a switch at all. **Slot:** UI / input switching step.

### 13. Flash persistence: decide the erase-stall policy before step 3

The H723 is single-bank: a 128 KB sector erase stalls all code fetch, interrupts included, for on the order of a second (check `tERASE` in the datasheet). The strip, DMX512 RX and Ethernet all stop during that time. The two plans also disagree: `device-config-refactor.md` wants a two-sector journal, while compliance 11.2 reserves one sector.

**Recommendation:** decide in the persistence plan between:
- (a) an append-only journal whose runtime saves only program flash words, erasing only at boot or on factory reset, or
- (b) accepting a stall but only erasing while output is idle or in failsafe.

(a) is the robust choice. One 128 KB sector holds about 500 records of 256 B, so one sector with erase at boot is enough. **Slot:** persistence plan, before any flash code.

## Process

### 14. Host-side unit tests for pure logic

The ArtCommand parser, the NodeReport formatter and `Config_ValidateCandidate` have no RTOS or LwIP dependency, and the ArtAddress field decoding can be written the same way. **Recommendation:** a native CMake target under `tests/` (CTest, plain `assert` or Unity) that compiles those files with tiny stubs. Run it before every Art-Net commit. It catches parser and validation regressions that the bench script only reaches with hardware. **Slot:** before Phase C, which adds the most decoding logic.

### 15. Link compliance rows to their evidence

Several "Done" rows had caveats the matrix did not show (reply delay seed, failsafe margin). **Recommendation:** when a row turns Done, name the test that proves it (`artnet_bench_test.py <test>`, a host test, or "bench: Lightkey"). Extend `tools/artnet_bench_test.py` with every phase (`address`, `locate`, `onchange`). **Slot:** ongoing.

### 16. Spec naming

In Art-Net 4 rev DP the 18-byte name field is **PortName** (per port), not ShortName. It matters once multi-bind (11.9) arrives, where each bind reports its own PortName but the same LongName. The struct comment now says so. **Slot:** 11.9.
