# Custom Art-Net Behavior

Project-specific extensions on top of Art-Net 4, used by the companion programming app. This is a contract: change the firmware and this document together.

All of it is only reachable while Art-Net is the active input source (`dmx.input_source == DMX_INPUT_ARTNET`). Changes made through it are runtime-only until config persistence exists.

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
