# Custom Art-Net Behavior

Project-specific extensions on top of Art-Net 4, used by the companion programming app. This is a contract: change the firmware and this document together.

All of it is only reachable while Art-Net is the active input source (`dmx.input_source == DMX_INPUT_ARTNET`). Changes made through it are runtime-only until config persistence exists.

**Status: transitional.** OpenTubeDia is a proof of concept. Its custom parts (the NodeReport fields and the ArtCommand verbs) will be replaced by standard RDM over Art-Net (`DMX_START_ADDRESS`, `DMX_PERSONALITY`, `IDENTIFY_DEVICE`, read via `DEVICE_INFO`), after which they are deprecated together with OpenTubeDia. Until then they stay as they are, because Lightkey has no RDM support. Don't add new custom verbs; use standard Art-Net (ArtAddress) or plan them as RDM parameters. See `roadmap.md`.

## Reply port — implemented

OpenTubeDia must be able to run next to lighting software on the same PC. Only one program can bind UDP 0x1936 (6454) on a NIC, so OpenTubeDia sends from an ephemeral port. The node therefore answers to **both** ports:

- Every reply (`ArtPollReply`, `ArtDataReply`) goes to the requester's IP on port **0x1936**, as the Art-Net spec requires.
- If the request's source port is **not** 0x1936, an identical copy also goes to that source port.

For OpenTubeDia this means:

- Send requests (`ArtPoll`, `ArtCommand`, …) from any local port and listen on that same port. Replies arrive there.
- Expect the copy on 0x1936 to reach whatever program owns that port (e.g. QLC+). That is harmless: it is a normal unicast `ArtPollReply` / `ArtDataReply`.
- The delayed reply to a broadcast `ArtPoll` follows the same rule, because the source port is stored with each queued reply.
- Lighting software on the same PC does not swallow OpenTubeDia's polls. Queued replies are de-duplicated per IP **and** port; if Lightkey's poll (from 0x1936) is already queued, OpenTubeDia's port is added to that entry and the one reply reaches both.

Reply timing:

- `ArtPoll` → `ArtPollReply` after a random 0–1 s delay (Art-Net spec, avoids reply bunching on broadcast polls).
- `ArtCommand`, `ArtInput`, `ArtAddress` → immediate `ArtPollReply` (acknowledgement). It replaces a queued poll reply to the same controller and port.
- `ArtDataRequest` → immediate `ArtDataReply`, or no reply for unsupported codes.

Implementation: `ArtNet_SendReply()` in `Core/Src/Art-Net/artnet.c`; reply queue in `ArtNet_HandleArtPoll()` / `ArtNet_RemoveQueuedPollReply()` in `artnet_handlers.c`.

## NodeReport format — implemented

The `NodeReport` field of every `ArtPollReply` is formatted as:

```
#xxxx [yyyy] @aaa $ppp MODE=[mmm...mmm] zzzzz...
```

| Field | Meaning |
|---|---|
| `xxxx` | Hex status code (Art-Net spec, Table 3 — NodeReport codes) |
| `yyyy` | Decimal counter, incremented on every `ArtPollReply`, rolls over after 9999. Lets the controller detect node events. |
| `aaa` | DMX start address on the first universe, fixed width (001–512) |
| `ppp` | Segment count, fixed width (001–999) |
| `mmm...mmm` | Current mode name: `RGB` (implemented), `RGBW`, `RGB16` (reserved) |
| `zzzzz...` | Free-form detail consistent with `xxxx`, e.g. `ArtCommand Bad Length` |

Status codes currently produced: `0x0001` RcPowerOk, `0x0004` RcParseFail, `0x000C` RcConfigErr. `0x0006` RcShNameOk and `0x0007` RcLoNameOk are defined for the future ArtAddress handler.

`RcParseFail` is only set for malformed packets of opcodes the node handles (e.g. `ArtDmx Truncated`, `ArtCommand Missing NUL`). Non-Art-Net datagrams, and Art-Net opcodes the node takes no action on (other nodes' `ArtPollReply`, `ArtTimeCode`, RDM, …), are dropped silently and do not change the NodeReport. They are counted in `artnet_ignored_count` (Live Watch).

Implementation: `ArtNet_SendPollReply()` in `Core/Src/Art-Net/artnet_pollreply.c`, state in `g_artnet_ctx.node_report`.

## Custom ArtCommands — implemented

Commands are sent as text in the `ArtCommand` packet. One packet may contain several commands:

```
Command=Data&Command=Data&
```

- `&` separates commands. Names and values are case-insensitive. Surrounding whitespace is tolerated.
- A packet is applied **atomically**: every command is parsed and validated first, then all changes are committed in one `DeviceConfig_ApplyPatch()` call. Any error rejects the whole packet.
- Duplicate commands in one packet reject the packet.
- The node answers every `ArtCommand` with an immediate `ArtPollReply`:
  - success → `RcPowerOk` with a detail naming what changed
  - malformed packet (too short, bad length, missing NUL) → `RcParseFail`
  - unknown command, invalid value, duplicate, unsupported mode → `RcConfigErr` (`0x000C`)

Implementation: `ArtNet_HandleArtCommand()` / `ArtNet_ParseCommandPayload()` in `Core/Src/Art-Net/artnet_handlers.c`.

### Change DMX address

```
DmxAddress=aaa&
```

`aaa` = 1–512. The resulting config must still pass `DeviceConfig_ApplyPatch()` validation, otherwise `RcConfigErr`.

### Change segment count

```
SegmentCount=ppp&
```

`ppp` = 1–999. Must be ≤ pixel count and divide the pixel count evenly (for 144 pixels: 1, 2, 3, 4, 6, 8, 9, 12, 16, 18, 24, 36, 48, 72, 144).

### Change mode

```
Mode=mmm...mmm&
```

Only `RGB` is accepted today. `RGBW` and `RGB16` exist in the config enum but are not rendered and are rejected with `Mode Unsupported`.

## Locate — standard ArtAddress, planned

Locate uses the standard ArtAddress `AcLedLocate` / `AcLedNormal` commands, not a custom packet. The earlier custom ArtTrigger KeyWink is dropped: with Oem 0xFFFF the spec marks Key 4 "Undefined", and ArtTrigger is not handled by this device.

To start locate, unicast an ArtAddress (0x6000) to the fixture with:

| Field | Value | Why |
|---|---|---|
| BindIndex | 1 | Root device |
| ShortName, LongName | all zero | Empty string = no change |
| NetSwitch, SubSwitch, SwIn[0..3], SwOut[0..3] | `0x7F` | Bit 7 clear = no change. **Not `0x00`**, which means "reset to default". |
| AcnPriority | 255 | No change |
| Command | `0x04` AcLedLocate | `0x02` AcLedNormal ends it |

Effect:
- The tube flashes full white at 2 Hz until `AcLedNormal`. There is no timeout.
- The node replies with an ArtPollReply. Status1 bits 7-6 report the state: `01` locate, `11` normal.
- When locate ends, the tube shows current DMX data again, or the failsafe output if no data is arriving.

Planned with the OLED menu system (roadmap, Local UI):
- Locate also shows a flashing dot in one corner of the OLED, so the menu stays usable while locating.
- `AcLedMute` (`0x03`) forces the display off before its normal timeout; the tube is not affected. Status1 then reports `10`. Until the menu exists, `AcLedMute` is ignored and Status1 never reports mute.

The board LED is debug-only and not visible on this hardware revision, so it is not used. RDM `IDENTIFY_DEVICE` will drive the same locate state once RDM exists.

Design and status: `plans/config-commits-and-artaddress.md` Phase D. Implementation will be `Indicator_Set()` (state) and `Pixel_Render_Locate()` (tube); the menu system will read `Indicator_Get()`.
