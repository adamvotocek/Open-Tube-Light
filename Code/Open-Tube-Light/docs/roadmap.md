# Roadmap

What is still to be built. Detailed designs for larger items live in `docs/plans/` (local only).

## Configuration & persistence

- [ ] **Flash persistence** — `Config_LoadFromFlash()` / `DeviceConfig_SaveToFlash()` are stubs, the checksum is not a real CRC32, factory defaults are not written on first boot. Needs reserved flash sectors, CRC32, and safe handling of an interrupted save. → `plans/device-config-refactor.md` phase 4
- [ ] **Factory reset** from the local UI.
- [ ] **Serialized config commits** — only `ControlTask` should commit; Art-Net and UI submit requests. → `plans/config-commits-and-artaddress.md` phases A–B
- [ ] **Apply network settings from config** — `network.ip_mode` / static IP in `device_config` are not applied; LwIP always uses the Art-Net default `2.x.y.z/8` derived from the MAC (`DeviceConfig_GetArtNetDefaultIp()`, applied in `lwip.c` USER CODE).

## Local UI (OLED + 4 buttons)

Settings must be editable at runtime and take effect without a reboot:

- Device name — short name (Art-Net, 18 B) and long name (Art-Net, 64 B)
- IP — DHCP / static; address and mask when static
- Strip layout — pixel count, segment count, segment format
- DMX — input source (Art-Net / DMX512 / sACN), start address, start universe. Channel count follows from the layout; universe count from channel count + start address.
- Failsafe — hold / zero / full
- Display — timeout after which the OLED turns off (a button press wakes it)

Menu system:
- A full menu on the OLED replaces the 1 Hz status page and owns the display: on/off and the timeout. The OLED code moves out of `main.c` into its own module.
- The display flush must not block `ControlTask`. Today `OledStatus_RunStep()` starts the I2C DMA transfer, then sleeps in `ssd1306_WaitForUpdate()` for about 25 ms per redraw. The menu redraws far more often than 1 Hz, and `ControlTask` also handles buttons and config requests, so start the transfer and check completion on the next loop pass instead.
- Edits go through the same config request path as Art-Net (`ConfigService_Submit()`, `plans/config-commits-and-artaddress.md`), so validation and persistence are shared.
- Art-Net indicator integration via `Indicator_Get()`:
  - Locate shows a flashing dot in one corner while the menu stays usable.
  - `AcLedMute` forces the display off before the timeout, and Status1 reports mute (10). `AcLedNormal` returns to the normal timeout behaviour.

Switching input source should commit through `device_config` and restart the active driver via the `dmx_input` abstraction.

## Art-Net

Full spec-vs-firmware feature matrix, infrastructure prerequisites and suggested order: `artnet-compliance.md`.

- [ ] **ArtAddress** — handler is a stub that replies `RcConfigErr`. First slice: short/long name, Net/SubSwitch, contiguous SwOut, AcFailHold/Zero/Full, AcCancelMerge, AcClearOp0; runtime-only. → `plans/config-commits-and-artaddress.md` phase C
- [ ] **Locate via ArtAddress `AcLedLocate`** — tube flashes white at 2 Hz until `AcLedNormal`. Replaces the dropped custom ArtTrigger KeyWink. The OLED corner dot and `AcLedMute` come with the menu system (Local UI). Contract in `artnet-custom-protocol.md`. → `plans/config-commits-and-artaddress.md` phase D
- [ ] **RDM responder over Art-Net** (after flash persistence) — `DMX_START_ADDRESS`, `DMX_PERSONALITY` (segment count × mode), `IDENTIFY_DEVICE`, `DEVICE_LABEL`. Replaces OpenTubeDia's custom ArtCommand verbs and NodeReport fields; OpenTubeDia is then deprecated (it stays until then because Lightkey has no RDM). Later the same responder on the DMX512 line (UART5 has RS-485 DE). Design notes: `artnet-compliance.md` 11.12.
- [ ] **2.x.y.z fallback when DHCP times out** — the 2.x.y.z address itself is the boot default already; needs DHCP first.
- [ ] **BindIndex** for layouts needing more than 4 universes (currently always `bind_index = 1`).
- [ ] **ArtPoll flags** — send ArtPollReply on change, diagnostics (ArtDiagData).
- [ ] **ArtPollReply** — Status1 indicator / Port-Address-authority bits.
- [ ] **Art-Net control plane while in DMX512 mode** — today Art-Net (discovery, ArtCommand) only runs when it is the input source.

## Input & rendering

- [ ] **sACN (E1.31) input driver** — `DMX_INPUT_SACN` exists in config, no driver. See `dmx-input-drivers.md`.
- [ ] **RGBW and RGB16 rendering** — accepted by config, rendered as RGB only (`pixel_render.c`).
- [ ] **288-pixel hardware** variant. Plan: move `DEVICE_CONFIG_HW_REVISION` / `DEVICE_CONFIG_MAX_PIXELS` into CMake build presets (one per hardware variant).

## Robustness

- [ ] Re-budget task stacks and FreeRTOS heap from measured `stack_monitor_report` values (bench, under Art-Net load + ArtPoll/ArtCommand traffic); `tcpip_thread` (2048 B) is the first candidate to shrink. → `plans/memory-tuning.md`
- [ ] Move ArtCommand parsing / config apply out of `tcpip_thread` (comes with serialized config commits).

## Done

- [x] Art-Net quick wins (`artnet-compliance.md` §12 step 1): unhandled opcodes dropped silently (`artnet_ignored_count`), replies to 0x1936 + requester's source port, GoodOutputA only while data is fresh, Status1 indicators = normal, Port-Address 0 rejected, ArtInput and ArtDataRequest (URLs) answered, credit line + merge policy in the repo-root README — bench-verified with `Temporary_Tests/python_artnet_scripts/artnet_quickwins_test.py` (57/57) and Lightkey
- [x] ArtPoll reply queue de-duplicated per IP + port, so a tool on the same PC as the lighting software no longer loses poll replies (seen on the bench: every few polls swallowed while Lightkey ran) — bench-verified: 0 swallowed polls with Lightkey on the same PC

- [x] Stack overflow hook records the task (`stack_monitor_overflow`) and halts at a breakpoint; per-task stack + heap high-water marks in `stack_monitor_report` (1 Hz) — watermarks bench-verified (all tasks have safe headroom); overflow halt not yet triggered on purpose
- [x] `snprintf` removed from the ArtPollReply path in `tcpip_thread` (bounded NodeReport formatter, output byte-identical)
- [x] Product identity constants (firmware version, HW revision, manufacturer, OEM/ESTA codes) in `device_config.h`; ArtPollReply uses them
- [x] Per-unit MAC address derived from the MCU unique ID (was `00:80:E1:00:00:00` on every unit) — *not yet bench-verified*

- [x] Idle refresh of the LED strip (every 150 ms) to wipe interference artifacts
- [x] ArtPollReply NodeReport status reporting (custom format)
- [x] Custom ArtCommand: `DmxAddress`, `SegmentCount`, `Mode=RGB`
- [x] Segment count decoupled from pixel count
- [x] Atomic config patch/commit path (`DeviceConfig_ApplyPatch`)
- [x] DMX512 UART receiver (lock-free, deferred notify)
