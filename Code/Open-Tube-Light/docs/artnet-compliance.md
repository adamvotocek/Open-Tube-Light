# Art-Net 4 Compliance Matrix 

Every feature of the Art-Net 4 specification (`docs/reference/Art-Net_4.md`, rev. DP), checked against the firmware as of commit `12eacf9`. OEM and ESTA codes are out of scope for now. Features that depend on them are marked **needs OEM/ESTA**.

Coverage was rechecked by grepping every *shall / must / should / not allowed* statement in the spec. Requirements that apply only to controllers are listed in section 13 so it is clear they were considered. The opcode table in the markdown conversion is incomplete: `OpIpProg` = 0xF800 and `OpRdmSub` = 0x8400 come from the official spec.

**Legend**

| Status | Meaning |
|---|---|
| **Done** | Implemented and matches the spec |
| **Partial** | Implemented, with gaps or deviations (listed) |
| **Missing** | Not implemented |
| **Ignored** | Not implemented on purpose; the packet is dropped |

| Implement? | Meaning |
|---|---|
| **Must** | Required by the spec ("shall"/"must") for an Art-Net output node, or core behaviour every controller expects |
| **Should** | Spec recommends it ("should"/"preferred"), or controllers commonly use it |
| **Nice** | Optional. Useful for this product |
| **No** | Not applicable to an output-only pixel fixture, deprecated, or not worth the cost |

---

## 1. Short answers

**Do all features have to be implemented to be compliant?** No. Art-Net 4 has no conformance levels and no certification program. Each packet has a *packet strategy* table that says what a Node does on receipt, and many of those actions are "Application Specific" or conditional ("*if* the Node supports this feature it will respond…"). A node that does not support ArtIpProg, ArtDataRequest, RDM or firmware upload stays compliant by ignoring those packets and reporting the matching capability bits as 0 in ArtPollReply. The real requirements for this product are:

1. Correct discovery: ArtPoll → unicast ArtPollReply with truthful fields and status bits.
2. Correct ArtDmx / ArtSync reception for the Port-Addresses the node subscribes to.
3. A reply to ArtAddress, and real remote programming of names, Port-Address and failsafe.
4. A documented merge policy (merge, or treat a second source as an error), plus the user-guide credit line.
5. Retaining failsafe/merge settings across power cycles ("should").

Section 10 lists the minimum set. In practice, conformance is checked with Artistic Licence's DMX-Workshop and with real consoles (grandMA, Hog, QLC+, Resolume, …).

**What infrastructure has to change?** See section 11. In short: serialized config commits, flash persistence, runtime network configuration (DHCP + 2.x.x.x), an always-on Art-Net control plane that runs in any input mode, a controller registry, an output override layer in the render path, a merge engine, and later sACN.

---

## 2. General protocol requirements

| Feature | Status | Implement? | Suggestion |
|---|---|---|---|
| Validate `ID[8]` = `"Art-Net\0"` | Done | Must | — |
| Accept packets longer than the minimum, ignore extra bytes | Done | Must | — |
| UDP port 0x1936 as source **and destination** | Partial | Must | Replies go to the sender's source port (`artnet_pollreply.c:295`). Send to 0x1936, or to both if the companion app sends from an ephemeral port. |
| Silently ignore packets the node does not handle | Partial | Must | Every unknown opcode overwrites the NodeReport with `RcParseFail` (`artnet.c:254`), so other nodes' ArtPollReply, ArtTimeCode, ArtTodRequest and so on clobber real status. Drop known-but-unused opcodes silently. Flag only truly unknown ones. |
| Protocol version check (`ProtVerLo` ≥ 14) | Missing | Nice | Spec only requires the ID check. Optional: drop `ProtVer < 14`. |
| Spare/unused bits and fields: transmit as 0, do not test on receipt | Done | Must | Keep this rule for every new packet (ArtDataReply, ArtIpProgReply, ArtDiagData). |
| Never send to the limited broadcast 255.255.255.255 | Done | Must | Only unicast today. Future ArtDiagData broadcast must use the directed broadcast of the netif subnet. |
| Port-Address 0 is deprecated (valid range 1–32767) | Partial | Should | The default is universe 1, which is fine. Validation still allows `0:0:0` (`device_config.c:581`). Reject it in `Config_ValidateCandidate`. |
| One Net, one Sub-Net and one Universe "switch" per port (hard or soft) | Partial | Must | They exist as config fields, but nothing can change them at runtime: no UI, ArtAddress is a stub, and ArtCommand doesn't touch them. Fixed by ArtAddress plus the local UI. |
| Failsafe timeout longer than the 4 s ArtDmx keep-alive | Done | Must | 5 s (`ARTNET_DISCONNECT_TIMEOUT_MS`). A controller with static data may resend only every 4 s. The comment in `dmx_input.h:72` still says 4000. |
| Do not ship DMX-Workshop / Art-Net SDK code | Done | Must | The SDK is copyright Singularity (UK) Ltd. Nothing from it is used. Copying OEM code *numbers* is fine. |
| User-guide credit "Art-Net™ Designed by and Copyright Artistic Licence" | Missing | Must | Add it to the README / user guide. |
| Status display (Power / Com / DMX indicators, 6 s timeout) | Missing | Nice | OLED: "Com" icon when any Art-Net packet arrived in the last 6 s; per-universe "receiving" marks. |

## 3. Discovery — ArtPoll (0x2000)

| Feature | Status | Implement? | Suggestion |
|---|---|---|---|
| Reply with ArtPollReply, random 0–1 s delay | Done | Must | — |
| Unicast reply (broadcast not allowed in Art-Net 4) | Done | Must | — |
| Accept ArtPoll ≥ 14 bytes, missing fields = 0 | Done | Must | — |
| Multiple controllers polling in the same window | Done | Must | Queue of 4, deduplicated by IP. |
| Flags bit 5 — Targeted Mode | Done | Must | — |
| Flags bit 1 — send ArtPollReply whenever node conditions change | Missing | Should | Controller registry (IP + flags, expires after ~10 s without a poll). On a config change, merge start/stop, locate on/off or IP change → unicast ArtPollReply to every registered controller with bit 1 set. |
| Flags bit 2/3 + DiagPriority — diagnostics requested | Missing | Should | Needs ArtDiagData (section 9). Rules: any controller asks → send. Several ask → broadcast. Use the lowest DiagPriority. |
| Flags bit 4 — disable VLC | Ignored | No | No VLC transmitter. |
| ArtPoll EstaMan / Oem fields | Ignored | No | They only identify the sender. They are not filters. |
| Respond with ArtPollReply while the input source is DMX512 / sACN | Missing | Should | Art-Net only runs when it is the input source, so the node is invisible in DMX512 mode. Needs the control/data-plane split (section 11.4). |

## 4. ArtPollReply (0x2100) fields

| Field / bit | Status | Implement? | Suggestion |
|---|---|---|---|
| Full-length reply (239 B; consumers accept ≥ 207), unused fields 0 | Done | Must | — |
| IP, Port, VersInfo, MAC | Done | Must | — |
| NetSwitch / SubSwitch / SwOut[4] | Done | Must | Contiguous range inside one sub-net, max 4 ports. |
| NumPorts, PortTypes (0x80 = DMX512 output) | Done | Must | — |
| ShortName / LongName | Done | Must | Programmable once ArtAddress lands. |
| NodeReport `#xxxx [yyyy] text` | Done | Must | Custom extension inside the free-text part is allowed (`artnet-custom-protocol.md`). |
| NodeReport codes (Table 3) | Partial | Should | Uses 0x0001/0x0004/0x0006/0x0007/0x000C. Add `RcDmxError` (DMX512 receive errors, once Art-Net runs in DMX512 mode), `RcFactoryRes` (after a factory reset), `RcUserFail` (only if network programming ever locks the UI). `RcDebug` is optional for Debug builds. |
| Oem / EstaMan | Partial | Should | Placeholders 0xFFFF / 0x0000. **Needs OEM/ESTA.** |
| GoodOutputA bit 7 — data being output | Partial | Must | Set while `source_ip` is assigned, but that is cleared only when the *next* ArtDmx arrives, so the bit stays on forever after a controller stops (`artnet_pollreply.c:265`). Compute it from `now - last_dmx_tick < timeout` at reply time. |
| GoodOutputA bit 3 — merging, bit 1 — LTP | Missing | Should | Comes with the merge engine. |
| GoodOutputA bit 0 — output converts from sACN | Missing | Nice | Set when the input source is sACN (after AcAcnSel0). |
| GoodOutputA bits 6/5/4/2 — test / SIP / text packets, short | Done (0) | No | Not applicable. |
| GoodOutputB — RDM off, continuous, no discovery | Done | Must | 0xF0 is correct for this device. |
| GoodInput / SwIn / PortTypes bit 6 | Done (0) | No | No Art-Net inputs. |
| Status1 bits 7-6 — indicator state | Missing | Should | 11 = normal, 01 = locate, 10 = mute. Driven by AcLed* (section 6). |
| Status1 bits 5-4 — Port-Address authority | Missing | Should | 01 = set from the front panel / defaults, 10 = programmed over the network (ArtAddress/ArtCommand). Track the source of the last addressing change. |
| Status1 bit 2 boot mode, bit 1 RDM, bit 0 UBEA | Done (0) | — | Correct values for current capabilities. |
| Status2 bit 3 — 15-bit Port-Address | Done | Must | — |
| Status2 bits 2/1 — DHCP capable / active | Partial | Should | Code is correct but `LWIP_DHCP=0`, so both bits are always 0. Fixed by the network infrastructure. |
| Status2 bit 4 — can switch Art-Net/sACN | Missing | Nice | Set once the sACN driver and AcArtNetSel0/AcAcnSel0 exist. |
| Status2 bits 7/6/5/0 — RDM via ArtAddress, output style, squawking, web config | Done (0) | No | — |
| Status3 bits 7-6 — failsafe state | Done | Must | — |
| Status3 bit 5 — programmable failsafe | Missing | Should | Set once ArtAddress AcFail* is implemented. |
| Status3 bits 4-0 — LLRP, direction switch, RDMnet, background queue | Done (0) | No | — |
| BindIp / BindIndex | Done | Must | Root device, index 1. Multi-bind only needed for more than 4 ports (section 11.9). |
| RefreshRate | Done | Should | From config (≥ 44). SPI output is not limited to DMX512 timing, so a measured higher rate may be reported. |
| AcnPriority (field 25) | Partial | No | Always 0, which is fine. The struct still calls it `sw_video` (`artnet_protocol.h:220`). Rename it to `acn_priority`. |
| SwMacro / SwRemote | Done (0) | No | The UI buttons are not remote triggers. |
| DefaultRespUID, BackgroundQueuePolicy, User | Done (0) | No | Only with RDMnet/LLRP. |

## 5. DMX data — ArtDmx (0x5000), merging, ArtSync (0x5200)

| Feature | Status | Implement? | Suggestion |
|---|---|---|---|
| Receive ArtDmx for subscribed Port-Addresses | Done | Must | — |
| Variable length (2–512), keep untouched channels | Done | Must | — |
| Sequence re-ordering (0 = disabled) | Done | Should | — |
| Continuous output without new ArtDmx | Done | Must | 150 ms idle refresh. |
| Failsafe on loss of data (hold/zero/full) | Done | Should | — |
| `Physical` field identifies a separate source | Missing | Should | Key sources by (IP, Physical), not IP alone. |
| Data merging, two sources, HTP (default) / LTP | Missing | Should ("preferred") | Today a second source is ignored for 5 s ("first source wins"). That is allowed only if it is **documented in the user guide**. Better: a merge engine with 2 source slots per universe (section 11.7). |
| Merge exit: 10 s timeout of a failed source; hold result if both fail | Missing | Should | Part of the merge engine. |
| ArtSync: power-on in non-sync mode, enter sync on ArtSync, 4 s timeout | Done | Must | — |
| ArtSync: only from the IP of the last ArtDmx | Done | Must | Per universe. |
| ArtSync ignored while merging | Missing | Must (with merge) | One check once merging exists. |
| Sync snapshot isolation | Partial | Nice | `ArtNet_LatchData()` copies *all* shadow buffers. A non-sync universe from controller B can latch controller A's half-written sync frame. Fix: copy the sync universes into a staging buffer when ArtSync arrives. |
| Input gateway: DMX512 input → transmit ArtDmx (first frame, on change, 4 s / 800–1000 ms keep-alive) | Ignored | No | The XLR input feeds the strip. It is not a gateway onto the network. |
| ArtNzs (0x5100) non-zero start codes | Ignored | No | A pixel fixture has no use for alternate start codes. Drop silently. |
| ArtVlc (ArtNzs, start code 0x91) | Ignored | No | No VLC hardware. |

## 6. Remote configuration — ArtAddress (0x6000)

The handler is a stub that replies `RcConfigErr` (roadmap: first slice in `plans/device-config-refactor.md`). Replying with an ArtPollReply is already correct. The node must always reply with its *resulting* state.

The per-port commands for ports 1–3 (`0x11–0x13`, `0x21–0x23`, …) are deprecated in Art-Net 4. All universes of this fixture form one logical stream, so apply the port-0 variant to every port and ignore the deprecated codes.

| Field / command | Status | Implement? | Suggestion |
|---|---|---|---|
| Reply with unicast ArtPollReply | Done | Must | Reply after the commit, with the new state. |
| BindIndex match | Missing | Must | Accept 0 or 1 (root). Ignore others until multi-bind exists. |
| ShortName / LongName (ignore if empty) | Missing | Must | Patch `identity`. NodeReport `RcShNameOk` / `RcLoNameOk`. |
| NetSwitch / SubSwitch (bit 7 = program, 0x7F = no change, 0x00 = reset) | Missing | Must | Patch `artnet_net` / `artnet_subnet`. "Reset to physical switch" = back to the UI/default value. |
| SwOut[0..3] | Missing | Must | Accept a contiguous range starting at SwOut[0]. Reject anything else and reply with the unchanged state (`RcConfigErr` + detail). |
| SwIn[4] | Ignored | No | No inputs. |
| AcnPriority | Ignored | No | The node never generates sACN. |
| `AcNone` 0x00 | Missing | Must | No-op. Fields are still applied. |
| `AcCancelMerge` 0x01 | Missing | Should | With merge: the next ArtDmx becomes the only source. Without merge: release `source_ip` locks. Useful even now for backup-console takeover. |
| `AcLedNormal` / `AcLedMute` / `AcLedLocate` 0x02–0x04 | Missing | Should | The standard locate. Flash the strip from the render override layer. Mute = OLED off. Report the state in Status1 bits 7-6. Can replace the planned custom ArtTrigger KeyWink (section 9). |
| `AcResetRxFlags` 0x05 | Missing | Nice | Reset NodeReport to `RcPowerOk` and clear the error counters. |
| `AcAnalysisOn/Off` 0x06/0x07 | Missing | Nice | Toggle verbose ArtDiagData, or ignore. |
| `AcFailHold/Zero/Full` 0x08–0x0A | Missing | Must (if Status3.5 is set) | Patch `output.failsafe`. **Must be persisted.** |
| `AcFailScene` / `AcFailRecord` 0x0B/0x0C | Missing | Nice | Record the active buffers as a scene (≤ 4 × 512 B) in persisted config. Status3 = 11 when selected. |
| `AcMergeLtp0` / `AcMergeHtp0` 0x10 / 0x50 | Missing | Should (with merge) | Per-port merge mode. Persist. Ports 1–3 commands are deprecated in Art-Net 4. |
| `AcDirectionTx0` / `AcDirectionRx0` 0x20 / 0x30 | Ignored | No | Output-only. Tx0 is a harmless no-op. |
| `AcArtNetSel0` / `AcAcnSel0` 0x60 / 0x70 | Missing | Nice | Switch the input source Art-Net ↔ sACN through `dmx_input`. Needs the sACN driver. |
| `AcClearOp0` 0x90 | Missing | Nice | Zero the shadow and active buffers and render once. |
| `AcStyleDelta0` / `AcStyleConst0` 0xA0 / 0xB0 | Ignored | No | Output is always continuous (Status2 bit 6 = 0). |
| `AcRdmEnable0/Disable0`, `AcBqp0–15` | Ignored | No | No RDM. |

## 7. IP configuration and ArtIpProg (0xF800) / ArtIpProgReply (0xF900)

| Feature | Status | Implement? | Suggestion |
|---|---|---|---|
| Factory default IP 2.x.y.z / 255.0.0.0 from MAC + OEM (10.x.y.z alternative) | Missing | Should | New `IP_MODE_ARTNET_2X`. B = MAC[3] + OemHi + OemLo, C/D = MAC[4]/MAC[5]. Also use it as the DHCP-timeout fallback. A meaningful B byte **needs OEM**, but it works with 0xFFFF. The user guide must warn that 2.x/10.x networks must not be routed to the Internet. |
| DHCP | Missing | Should | `LWIP_DHCP=1` in CubeMX. Apply `network.ip_mode` at boot. |
| Static IP from config | Missing | Should | Apply via `netif_set_addr()` in `tcpip_thread` instead of the CubeMX constant `192.168.1.179`. |
| ArtIpProg: enquiry (Command = 0) → ArtIpProgReply | Missing | Should | Reply with the current IP, mask, gateway and DHCP bit. "Not supported" = no reply (compliant). |
| ArtIpProg: program IP / mask / gateway, enable DHCP, reset to default | Missing | Should | Patch `network`, commit, apply live, persist. Send the reply **before** switching the address. |
| ArtIpProg: program port | Ignored | No | Deprecated. |

## 8. Text commands — ArtCommand (0x2400)

| Feature | Status | Implement? | Suggestion |
|---|---|---|---|
| Parse `Command=Data&` text, case-insensitive | Done | Should | Custom verbs `DmxAddress`, `SegmentCount`, `Mode`. |
| `SwoutText` / `SwinText` (EstaMan = 0xFFFF) | Missing | No | Only labels a controller UI. Low value. |
| Gate custom verbs on EstaMan | Missing | Should | Custom verbs should only be accepted with the product's own EstaMan, and standard verbs with 0xFFFF. **Needs ESTA.** Until then, accept any value. |
| Run outside `tcpip_thread` | Missing | Should | Covered by serialized config commits (section 11.1). |

## 9. Other packets

| Packet | Status | Implement? | Suggestion |
|---|---|---|---|
| **ArtDataRequest / ArtDataReply** (0x2700/0x2800) | Missing | Nice | Easy win. Answer `DrPoll`, `DrUrlProduct`, `DrUrlUserGuide`, `DrUrlSupport` (GitHub URLs as `#define`s next to the product identity). No reply = unsupported, which is compliant. |
| **ArtDiagData** (0x2300) | Missing | Should | Send short text (parse errors, config errors, merge start, DMX errors) when the controller registry says diagnostics are wanted. Rate-limit. Unicast or directed broadcast per the ArtPoll rules. |
| **ArtTrigger** (0x9900) | Missing | Nice | Application specific. The planned KeyWink locate uses Key 4 with Oem 0xFFFF, which the spec reserves ("Undefined"). Prefer ArtAddress `AcLedLocate` for locate. Keep ArtTrigger for OEM-specific keys gated on `Oem == own OEM`. **Needs OEM.** Update `artnet-custom-protocol.md` if this changes. |
| **ArtTimeCode** (0x9700) | Ignored | No | No time-based effects. Drop silently. |
| **ArtInput** (0x7000) | Ignored | Nice | No inputs, but the strategy says "reply with ArtPollReply". That is a trivial addition. |
| **ArtFirmwareMaster / ArtFirmwareReply** (0xF200/0xF300), UBEA | Ignored | No | The app is about 158 KB, the flash is 512 KB in four 128 KB sectors, and one sector is needed for config. There is no room for A/B images, so this needs a bootloader. The `.alf` header lists valid OEM codes (**needs OEM**). Keep ST-Link / DFU. Not supported = no reply. UbeaVersion = 0 and Status1 bit 0 = 0 are correct. |
| **ArtTodRequest** (0x8000) | Ignored | Nice (later) | Reply with ArtTodData for the matching Net + Address[]. It must not force a full discovery. |
| **ArtTodData** (0x8100) | Ignored | Nice (later) | Unicast only. BindIndex must match ArtPollReply. `TodNak` when not available. |
| **ArtTodControl** (0x8200) | Ignored | Nice (later) | Must be answered with ArtTodData. `AtcFlush` restarts discovery. |
| **ArtRdm** (0x8300) | Ignored | Nice (later) | Unicast only. `ArProcess`. The RDM packet excludes the start code. |
| **ArtRdmSub** (0x8400) | Ignored | No | Only allowed *in addition* to ArtRdm. No sub-devices here. |
| RDM as a whole | — | Nice (later) | No downstream DMX/RDM line. The option is to make the fixture itself an RDM responder over ArtRdm: own UID in the ToD, PIDs such as DEVICE_INFO, DMX_START_ADDRESS, PERSONALITY, IDENTIFY. Set Status1 bit 1. **Needs ESTA** (the UID contains the manufacturer ID). |
| ArtMedia*, ArtVideo*, ArtTimeSync, ArtDirectory, ArtFile* | Ignored | No | Media-server or undefined in this spec. |
| OpMacMaster / OpMacSlave | Ignored | No | Deprecated. |

---

## 10. Minimum set for a credible Art-Net 4 node

The essential items for this device, in order:

1. Fix the deviations: silent drop of unused opcodes, GoodOutputA staleness, reply port, Port-Address 0, `sw_video` rename.
2. Documentation: the credit line and the merge policy ("first source wins for 5 s; second source ignored") in the user guide.
3. ArtAddress: names, Net/Sub/SwOut, AcFail Hold/Zero/Full, AcLed*, AcCancelMerge. Then Status1 indicator/authority bits and Status3 bit 5.
4. Persistence of everything ArtAddress programs (the spec explicitly asks for failsafe/merge settings to survive power cycles).
5. ArtPoll Flags bit 1 (reply on change).
6. Art-Net discovery and configuration in every input mode.

Everything else (merge, DHCP/2.x, ArtIpProg, diagnostics, DataRequest, sACN select, RDM) is recommended or optional. It improves interoperability but is not needed to be "compliant".

## 11. Infrastructure changes required

| # | Change | Unlocks | Notes |
|---|---|---|---|
| 11.1 | **Serialized config commits** — `tcpip_thread` posts a request, `ControlTask` validates/commits/persists, then `tcpip_callback()` sends the ArtPollReply | ArtAddress, ArtIpProg, ArtCommand off `tcpip_thread` | Roadmap item. Keep the reply destination in the request. |
| 11.2 | **Flash persistence** — reserve the last 128 KB sector, shrink `FLASH` in `STM32H723XX_FLASH.ld`, real CRC32 | Retained names, Port-Address, failsafe, merge mode, failsafe scene, IP | H723 is single-bank: erase/program stalls code fetch from the same bank. Save from `ControlTask`, debounce (about 2 s after the last change), accept a short render stall. Update `memory-layout.md`. |
| 11.3 | **Runtime network config** — `LWIP_DHCP=1` (CubeMX), apply `ip_mode`/static/2.x after `MX_LWIP_Init()` in a USER CODE block, DHCP-timeout fallback to 2.x, `LWIP_NETIF_STATUS_CALLBACK=1` | DHCP, 2.x default, ArtIpProg, correct Status2 | All `netif` calls in `tcpip_thread`. Extend the `DeviceConfig_IpMode_t` enum and its validation. |
| 11.4 | **Art-Net control plane always on** — split `artnet.c` into a control part (PCB, ArtPoll/PollReply, ArtAddress, ArtIpProg, ArtCommand, ArtDataRequest; started once at boot) and a data part (ArtDmx/ArtSync/buffers; the `dmx_input` driver) | Discovery and config in DMX512/sACN mode, AcArtNetSel/AcAcnSel | In DMX512 mode, report NumPorts = 0 (or GoodOutputA.7 = 0) so controllers don't unicast unused ArtDmx. Roadmap item. |
| 11.5 | **Controller registry** — IP, ArtPoll flags, DiagPriority, last-poll tick | Flags bit 1 (reply on change), ArtDiagData routing | Small static table in `g_artnet_ctx`. Expire after about 10 s. |
| 11.6 | **Node-change events** — hook the `DeviceConfig` change mask plus Art-Net events (merge, locate, source change, IP change) to "send ArtPollReply to subscribers" | Flags bit 1, SwMacro-style updates | Reuse `DEVICE_CONFIG_CHANGE_*` flags. |
| 11.7 | **Merge engine** — 2 source slots per universe keyed by (IP, Physical), per-source buffers (2 × 4 × 512 B ≈ 4 KB DTCM), HTP/LTP per port, 10 s source timeout | Merge, AcMerge*, AcCancelMerge, GoodOutputA bits 3/1, ArtSync-ignore-while-merging | Merge into the shadow buffer in `tcpip_thread` (cheap for ≤ 4 universes). The merge mode is persisted config. |
| 11.8 | **Render override layer** in `pixel_render` — locate flash, mute, clear output, failsafe-scene playback | AcLed*, AcClearOp0, AcFailScene, ArtTrigger locate | Transient state (atomic flags/ticks), not config. Applied in `EffectTask` so DMA ownership stays in one place. |
| 11.9 | **Multi-bind (BindIndex > 1)** — one ArtPollReply per bind, ArtAddress routed by BindIndex | More than 4 universes or non-contiguous SwOut | Needed by the 288-pixel variant: RGB16 = 1728 channels plus a start offset can reach 5 universes. Also raise `DEVICE_CONFIG_MAX_UNIVERSES`. Every bind must report the same LongName. |
| 11.10 | **sACN driver + IGMP** — `LWIP_IGMP=1` and ETH multicast filtering (CubeMX) | AcAcnSel0, Status2 bit 4, GoodOutputA bit 0 | Roadmap item. See `dmx-input-drivers.md`. |
| 11.11 | **OEM code + ESTA manufacturer ID** (deferred) | ArtTrigger OEM keys, 2.x B byte, RDM UID, EstaMan gating of ArtCommand, firmware files | The OEM code is free from Artistic Licence. The ESTA ID comes from ESTA/TSP. |

## 12. Suggested order

1. **Quick fixes, no infrastructure:** section 10 item 1; Status1 indicator = 11 (normal); ArtInput → ArtPollReply; ArtDataRequest; user-guide credit and merge-policy text. Detailed plan: `plans/artnet-quick-wins.md` (local only).
2. **11.1 → ArtAddress** first slice (runtime-only), with 11.8 for AcLed*/AcClearOp0, plus 11.5/11.6 for Flags bit 1.
3. **11.2 persistence**, then AcFail* is fully compliant and Status3 bit 5 can be set.
4. **11.3 network**, then ArtIpProg.
5. **11.4 control-plane split.**
6. **11.7 merge engine** and ArtDiagData.
7. **11.10 sACN** + AcArtNetSel0/AcAcnSel0. Later, with OEM/ESTA: RDM responder, ArtTrigger OEM keys.

## 13. Controller-only requirements (not applicable)

These requirements bind controllers, not nodes. They are listed so the matrix is complete:

- Poll every 2.5–3 s, poll both 2.255.255.255 and 10.255.255.255, and reply to one's own ArtPoll.
- Treat a node as gone after 3 s without an ArtPollReply. Ignore nodes with protocol < 14.
- Unicast ArtDmx only to subscribers (nodes listing the universe in SwIn/SwOut). Never broadcast ArtDmx. Don't send to universes without subscribers.
- Broadcast ArtSync (directed broadcast only). ArtIpProg, ArtAddress, ArtInput, ArtDataRequest and ArtFirmwareMaster are unicast only.
- Check the `.alf` OEM list before a firmware upload.

What the node side has to get right for these: accurate SwOut subscriptions, a reply within 1 s, and accepting ArtSync on its broadcast address.
