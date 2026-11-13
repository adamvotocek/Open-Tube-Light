# Custom Art-Net Behavior

Project-specific extensions on top of Art-Net 4, used by the companion programming app. This is a contract: change the firmware and this document together.

All of it is only reachable while Art-Net is the active input source (`dmx.input_source == DMX_INPUT_ARTNET`). Changes made through it are runtime-only until config persistence exists.

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

## ArtTrigger fixture locate — planned, not implemented

`OpTrigger` is not dispatched yet. Intended behavior:

- `Key` = `0x04` (KeyWink)
- `SubKey` = duration in seconds, 0–255
- The fixture flashes white at full brightness at 2 Hz for that duration, then resumes normal rendering.
- All other keys, and the standard Art-Net trigger key meanings, are ignored.

Design notes: this is a transient output override, not a config change. Store a `locate_until_tick` and apply the effect from the render path (`EffectTask` / `pixel_render`), not from the UDP callback, so LED timing and DMA ownership stay in one place.
