# Plan: Art-Net quick wins

> **Completed** in commit `4a2da15`, bench-verified 57/57 and with Lightkey. Kept for the reasoning. Later changes: option A was chosen; `ArtNet_SendReply()` clones the pbuf for the second copy (re-sending one pbuf would carry stale headers, so "a pbuf can be sent twice" in step 2 is wrong); the test script is now `tools/artnet_bench_test.py`.

The easiest items from `docs/artnet-compliance.md` §12 step 1. None of them needs new infrastructure: no config-commit queue, no persistence, no CubeMX changes. Five steps, each one commit, ordered so that later steps reuse earlier ones. Total size is roughly 150–200 lines of firmware plus docs.

**Out of scope here:** ArtAddress, reply-on-change, persistence, DHCP/2.x, merge. These need §11 infrastructure.

## Decision needed before step 2 — reply port

The spec says Art-Net uses UDP 0x1936 as both source and destination. Today every ArtPollReply goes to the **sender's source port** (`artnet_pollreply.c:295`). That is non-compliant, but it is what makes a tool that sends from an ephemeral port work. A companion app that can't bind 6454 while QLC+ is running on the same PC is one example.

| Option | Behaviour | Trade-off |
|---|---|---|
| **A (recommended)** | Always send to 0x1936. If the request came from another port, send a second copy to that port. | Compliant, and nothing breaks. One extra datagram only for non-standard senders. |
| B | Always send to 0x1936 only. | Strictly by the book. Breaks any tool that doesn't listen on 6454. |
| C | Keep replying to the source port. | No change. Stays non-compliant. |

Whatever you pick, record it in `artnet-custom-protocol.md`, because it is part of the companion-app contract.

---

## Step 1 — Packet-handling correctness

**Placement:** each fix stays in the module that already owns the behaviour. No new files.

| # | Change | File | Detail |
|---|---|---|---|
| 1a | Drop unhandled opcodes silently | `artnet.c` (dispatcher `default:`) | Remove `ArtNet_NodeReportSet(RcParseFail, "Unsupported OpCode")`. Increment a new global `uint32_t artnet_ignored_count` instead (Live Watch, like `render_count`). Keep `RcParseFail` for *malformed* packets of opcodes the node handles (short ArtDmx, ArtPoll, ArtSync…). |
| 1b | Drop packets shorter than the Art-Net header silently | `artnet.c` (`p->tot_len < sizeof(ArtNet_Header_t)`) | These can't be Art-Net (spec: "any other packets are ignored"). Today any stray UDP on 6454 overwrites the NodeReport. Count them in `artnet_ignored_count`. |
| 1c | GoodOutputA bit 7 only while data is fresh | `artnet_pollreply.c` (port loop) | `fresh = !ip_addr_isany(&u->source_ip) && (now - u->last_dmx_tick) <= ARTNET_DISCONNECT_TIMEOUT_MS`. Read-only, with no side effects on `universes[]`. Both readers and writers run in `tcpip_thread`, so no lock is needed. |
| 1d | Status1 indicator state = 11 (normal) | `artnet_pollreply.c` `ArtNet_BuildStatus1()` | `status1 = (0x3U << 6)`. Leave Port-Address authority at 00 until ArtAddress exists. Update the bit comment. |
| 1e | Reject Port-Address 0 | `device_config.c` `Config_ValidateCandidate()` | After the range check: `if (net == 0 && subnet == 0 && start_universe == 0) return -1;`. Defaults (universe 1) are unaffected. Nothing at runtime can set it yet. |
| 1f | Rename `sw_video` → `acn_priority` | `artnet_protocol.h:220` | Comment: "sACN priority used for DMX→sACN conversion; 0 (no inputs)". The only use is the `memset`. |
| 1g | Fix stale comments | `dmx_input.h:72`, `:118` | Art-Net timeout is 5000 (> 4 s ArtDmx keep-alive). The no-driver default is 12000, not 4000. |

**Bench test**
- QLC+ outputs to the fixture while a second Art-Net source runs on the LAN, e.g. DMX-Workshop or another node answering polls. Poll repeatedly: the NodeReport stays `#0001 … Power On OK`, and `artnet_ignored_count` rises.
- Start QLC+ output, poll: port 1 shows "data". Stop the output, wait more than 5 s, poll again: port 1 no longer shows "data".
- Strip behaviour is unchanged. `render_count` advances, and `failsafe_count` only increments on real loss.

**Docs:** compliance §2 rows "silently ignore" and "Port-Address 0", plus §4 rows "GoodOutputA bit 7", "AcnPriority" and "Status1 7-6" → Done.

## Step 2 — Reply-port policy in one helper

**Placement:** `ArtNet_SendReply(struct pbuf *p, const ip_addr_t *addr, u16_t src_port)` in `artnet.c` next to the PCB it uses, declared in `artnet_internal.h`. Rejected alternative: keep `udp_sendto` inline in each reply builder, which would duplicate the policy into ArtDataReply (step 4).

- Implements the option chosen above. For A: `udp_sendto(pcb, p, addr, ARTNET_PORT)`, then, if `src_port != ARTNET_PORT`, a second `udp_sendto(…, src_port)`. A pbuf can be sent twice. The caller frees it once afterwards.
- `ArtNet_SendPollReply()` calls the helper instead of `udp_sendto`.

**Bench test:** Wireshark `udp.port == 6454`. ArtPollReply destination port is 6454 for QLC+ / DMX-Workshop polls. The companion app still sees replies, and with option A it receives the extra copy on its own port.

**Docs:** `artnet-custom-protocol.md` gets the reply-port rule. Compliance §2 "UDP port 0x1936" → Done.

## Step 3 — ArtInput (0x7000) → ArtPollReply

**Placement:** the handler goes in `artnet_handlers.c` beside ArtAddress. ArtInput is the same kind of "configure a port, reply with state" packet.

- `artnet_protocol.h`: `ARTNET_OP_INPUT 0x7000`, plus a packed `ArtNet_ArtInput_t` (20 B: header, ProtVer, Filler1, BindIndex, NumPortsHi/Lo, Input[4]).
- Dispatcher: `len >= sizeof(ArtNet_ArtInput_t)`, else `RcParseFail "ArtInput Too Short"`.
- Handler: ignore it unless BindIndex is 0 or 1. The Input[] disable bits are meaningless for a node without inputs.
- Reply: an immediate ArtPollReply **without changing the NodeReport**. Split `ArtNet_ReplyImmediateStatus()` into `ArtNet_ReplyImmediate(src, port)` (remove the queued reply + send) and a status wrapper that calls it.

**Bench test:** the script below sends ArtInput. An ArtPollReply comes back at once, and the NodeReport text is unchanged.

## Step 4 — ArtDataRequest (0x2700) / ArtDataReply (0x2800)

**Placement:** URLs are product identity, so they go in `device_config.h` next to `DEVICE_CONFIG_MANUFACTURER` (CLAUDE.md: identity is `#define`d, never stored). Handler and reply builder go in `artnet_handlers.c`. Rejected alternative: a new `artnet_datareply.c`, since about 50 lines don't justify a module.

- `device_config.h`:
  - `DEVICE_CONFIG_URL_PRODUCT` = `"https://github.com/adamvotocek/Open-Tube-Light"`
  - `DEVICE_CONFIG_URL_USER_GUIDE` = `…/blob/main/README.md`
  - `DEVICE_CONFIG_URL_SUPPORT` = `…/issues`
- `artnet_protocol.h`: opcodes, and the `ArtNet_ArtDataRequest_t` / `ArtNet_ArtDataReply_t` headers. In the reply, field order is **EstaManHi before EstaManLo** (unlike ArtPollReply), all multi-byte fields are big-endian, and the payload is ≤ 512 B including the NUL.
- Dispatcher: `len >= 20` (through RequestLo). The 22 spare bytes are not required.
- Handler: map the Request code:
  - `DrPoll` (0x0000) → reply with PayLen 0
  - `DrUrlProduct` / `DrUrlUserGuide` / `DrUrlSupport` (1/2/3) → URL + NUL, PayLen = strlen + 1
  - Everything else (UDR, GDTF, 0x8000+) → **no reply**, which is the spec's "not supported"
- Build the reply directly in a `pbuf_alloc(PBUF_TRANSPORT, 20 + paylen, PBUF_RAM)`. No stack buffer and no `snprintf`, per the `tcpip_thread` stack rules. Send it through `ArtNet_SendReply()`. Oem/EstaMan come from the existing placeholder constants.

**Bench test:** the script below. DrPoll gets an empty reply, codes 1–3 return the URLs, and code 5 gets no reply. Check `stack_monitor_report`: there should be no change in the `tcpip_thread` minimum free stack.

## Step 5 — User-facing text

**Placement:** the repo-root `README.md` is the only user-facing document today. Add a short "Art-Net" section there. A future user guide can absorb it.

- Credit, verbatim: "Art-Net™ Designed by and Copyright Artistic Licence".
- Merge policy: one controller per universe. The first sender owns the universe. ArtDmx for that universe from other IPs is ignored until the owner has been silent for 5 s. No HTP/LTP merge. ArtSync is supported per controller.
- The firmware `README.md` Capabilities list gets ArtDataRequest (product URLs).

**Docs:** compliance §2 "credit" → Done, and §5 merge row: policy documented. Roadmap "Done" list.

---

## Test script (bench, steps 3–4)

With option A, the script can run next to QLC+ because it doesn't need port 6454. Set `NODE` to the fixture IP.

```python
import socket, struct
NODE = "192.168.1.179"
def art(op, body): return b"Art-Net\0" + struct.pack("<H", op) + b"\x00\x0e" + body
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1.5)
tests = {
    "ArtInput":      art(0x7000, b"\x00\x01\x00\x01" + bytes(4)),
    "DrPoll":        art(0x2700, b"\x00\x00\xff\xff" + struct.pack(">H", 0) + bytes(22)),
    "DrUrlProduct":  art(0x2700, b"\x00\x00\xff\xff" + struct.pack(">H", 1) + bytes(22)),
    "DrUrlPersGdtf": art(0x2700, b"\x00\x00\xff\xff" + struct.pack(">H", 5) + bytes(22)),  # expect timeout
}
for name, pkt in tests.items():
    s.sendto(pkt, (NODE, 0x1936))
    try:
        data, _ = s.recvfrom(1024); op = struct.unpack_from("<H", data, 8)[0]
        print(name, hex(op), data[20:].split(b"\0")[0] if op == 0x2800 else data[108:172].split(b"\0")[0])
    except socket.timeout:
        print(name, "no reply")
```

`data[108:172]` is the ArtPollReply NodeReport, and `data[20:]` is the ArtDataReply payload.

## Verification for every step

- `cube-cmake --build --preset Debug`: zero new warnings (the baseline is the one `Config_ValidateFlash` warning), and the memory table fits. None of these steps adds static RAM beyond one counter.
- Flash, then run the step's bench test. Live Watch: `artnet_ignored_count`, `render_count`, `failsafe_count`, `stack_monitor_report`.
