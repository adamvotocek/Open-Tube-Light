"""
Art-Net bench regression test for the fixture (started with the quick wins,
docs/artnet-compliance.md section 12, step 1). Extend it with each Art-Net
feature (ArtAddress, locate, ...) and run the full suite before every Art-Net commit.

Runs next to Lightkey / QLC+: it sends from an ephemeral UDP port, so it does
not need 6454. The node answers to 6454 AND to the source port (option A),
so the replies come back to this script.

Standard library only (Python 3.8+).

Usage
-----
    python tools/artnet_bench_test.py --node 2.x.y.z      # all automatic tests (IP from the OLED)
    python tools/artnet_bench_test.py --node 2.x.y.z poll
    python tools/artnet_bench_test.py ignored --repeat 50
    python tools/artnet_bench_test.py monitor             # GoodOutputA live view; Enter = marker, Ctrl+C = stop
    python tools/artnet_bench_test.py -i                  # pause before each test (Live Watch / Wireshark)

Tests
-----
    poll       ArtPoll -> ArtPollReply fields: Status1, AcnPriority, BindIndex, Port-Address != 0, GoodOutputA
    ignored    junk / unhandled opcodes: no reply, NodeReport unchanged, artnet_ignored_count rises
    input      ArtInput -> immediate ArtPollReply, NodeReport unchanged, queued poll reply cancelled
    datareq    ArtDataRequest -> ArtDataReply (DrPoll, URLs), unsupported codes -> no reply
    replyport  reply copies on 6454 and on the source port (needs 6454 free on this PC)
    parsefail  short ArtInput / ArtDataRequest -> RcParseFail, then restored via ArtCommand
    monitor    (not in "all") poll every second, print GoodOutputA per port and its transitions

Not testable over the network: Port-Address 0 rejection (ArtAddress is still a stub and
ArtCommand has no universe verb).
"""

import argparse
import re
import selectors
import socket
import struct
import sys
import time

ARTNET_PORT = 0x1936
ARTNET_ID = b"Art-Net\x00"
PROT_VER = b"\x00\x0e"  # 14, big-endian

OP_POLL = 0x2000
OP_POLL_REPLY = 0x2100
OP_COMMAND = 0x2400
OP_DATA_REQUEST = 0x2700
OP_DATA_REPLY = 0x2800
OP_INPUT = 0x7000

POLL_WINDOW_S = 1.5    # ArtPollReply is delayed 0..1 s at random
REPLY_WINDOW_S = 0.8   # immediate replies (ArtInput, ArtDataRequest, ArtCommand)
SILENCE_WINDOW_S = 1.2  # "expect no reply"

NODE_REPORT_RE = re.compile(
    r"#([0-9A-Fa-f]{4})\s+\[(\d+)\]\s+@(\d+)\s+\$(\d+)\s+MODE=\[([^\]]*)\]\s*(.*)")


# --------------------------------------------------------------------------- packets

def art(op, body=b""):
    """Art-Net header + little-endian opcode + body."""
    return ARTNET_ID + struct.pack("<H", op) + body


def pkt_poll(flags=0x00):
    # ProtVer, Flags, DiagPriority, TargetTop(2), TargetBottom(2), EstaMan(2), Oem(2) = 22 B
    return art(OP_POLL, PROT_VER + bytes([flags, 0x00]) + bytes(8))


def pkt_input(bind_index=1, num_ports=1):
    # ProtVer, Filler1, BindIndex, NumPortsHi, NumPortsLo, Input[4] = 20 B
    return art(OP_INPUT, PROT_VER + bytes([0x00, bind_index, 0x00, num_ports]) + bytes(4))


def pkt_data_request(code):
    # ProtVer, EstaMan(2), Oem(2), Request(2, big-endian), Spare[22] = 40 B
    return art(OP_DATA_REQUEST, PROT_VER + b"\xff\xff\xff\xff" + struct.pack(">H", code) + bytes(22))


def pkt_command(text):
    data = text.encode("ascii") + b"\x00"
    # ProtVer, EstaMan(2) 0xFFFF = all, Length(2, big-endian, incl. NUL), Data
    return art(OP_COMMAND, PROT_VER + b"\xff\xff" + struct.pack(">H", len(data)) + data)


# Packets the node must drop silently (each one increments artnet_ignored_count).
IGNORED_PACKETS = [
    ("non-Art-Net datagram (5 B)",        b"hello"),
    ("shorter than Art-Net header (9 B)", ARTNET_ID + b"\x00"),
    ("bad ID, otherwise a valid ArtPoll", b"Art-Nex\x00" + pkt_poll()[8:]),
    ("OpPollReply from 'another node'",   art(OP_POLL_REPLY, bytes(229))),
    ("OpTimeCode 0x9700",                 art(0x9700, PROT_VER + bytes(7))),
    ("OpRdm 0x8300",                      art(0x8300, PROT_VER + bytes(12))),
    ("OpIpProg 0xF800 (command 0)",       art(0xF800, PROT_VER + bytes(22))),
    ("OpTrigger 0x9900 (not dispatched)", art(0x9900, PROT_VER + b"\x00\x00\xff\xff" + bytes(514))),
    ("unknown opcode 0x1234",             art(0x1234, PROT_VER + bytes(8))),
]


# --------------------------------------------------------------------------- decoding

def cstr(raw):
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def parse_node_report(text):
    m = NODE_REPORT_RE.match(text)
    if not m:
        return None
    return {"code": m.group(1).upper(), "counter": int(m.group(2)), "address": int(m.group(3)),
            "segments": int(m.group(4)), "mode": m.group(5), "detail": m.group(6).strip()}


def parse_poll_reply(d):
    if len(d) < 207:
        return None
    num_ports = d[173]
    r = {
        "length": len(d),
        "ip": ".".join(str(b) for b in d[10:14]),
        "port_field": struct.unpack_from("<H", d, 14)[0],
        "version": (d[16] << 8) | d[17],
        "net": d[18], "sub": d[19],
        "oem": (d[20] << 8) | d[21],
        "status1": d[23],
        "esta": (d[25] << 8) | d[24],
        "short_name": cstr(d[26:44]),
        "long_name": cstr(d[44:108]),
        "node_report_raw": cstr(d[108:172]),
        "num_ports": num_ports,
        "port_types": list(d[174:178]),
        "good_output_a": list(d[182:186]),
        "sw_out": list(d[190:194]),
        "acn_priority": d[194],
        "style": d[200],
        "mac": ":".join(f"{b:02X}" for b in d[201:207]),
    }
    if len(d) >= 218:
        r.update({"bind_index": d[211], "status2": d[212],
                  "good_output_b": list(d[213:217]), "status3": d[217]})
    if len(d) >= 228:
        r["refresh_rate"] = (d[226] << 8) | d[227]
    r["node_report"] = parse_node_report(r["node_report_raw"])
    r["port_addresses"] = [((r["net"] & 0x7F) << 8) | ((r["sub"] & 0x0F) << 4) | (r["sw_out"][i] & 0x0F)
                           for i in range(min(num_ports, 4))]
    return r


def parse_data_reply(d):
    if len(d) < 20:
        return None
    pay_len = (d[18] << 8) | d[19]
    return {"length": len(d), "prot_ver": (d[10] << 8) | d[11], "esta": (d[12] << 8) | d[13],
            "oem": (d[14] << 8) | d[15], "request": (d[16] << 8) | d[17],
            "pay_len": pay_len, "payload": d[20:20 + pay_len]}


def report_key(pr):
    """(code, detail) of a parsed ArtPollReply NodeReport: what must stay unchanged."""
    nr = pr["node_report"] if pr else None
    return (nr["code"], nr["detail"]) if nr else (None, pr["node_report_raw"] if pr else None)


# --------------------------------------------------------------------------- harness

class Rx:
    def __init__(self, t, src, dst_port, data):
        self.t, self.src, self.dst_port, self.data = t, src, dst_port, data
        self.op = struct.unpack_from("<H", data, 8)[0] if len(data) >= 10 else None


class Tester:
    def __init__(self, node, bind_ip, interactive):
        self.node = node
        self.bind_ip = bind_ip
        self.interactive = interactive
        self.passed = self.failed = 0
        self.malformed = []  # (local port, datagram) from the node that is not Art-Net
        self.sock = self._open_socket(0)
        self.local_port = self.sock.getsockname()[1]

    # ---- sockets
    def _open_socket(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sys.platform == "win32":
            # Stop Windows from raising ConnectionResetError on ICMP port-unreachable.
            try:
                s.ioctl(socket.SIO_UDP_CONNRESET, False)
            except (AttributeError, OSError):
                pass
        s.bind((self.bind_ip, port))
        return s

    def send(self, pkt, sock=None):
        (sock or self.sock).sendto(pkt, (self.node, ARTNET_PORT))
        return time.monotonic()

    def drain(self, *socks):
        for s in socks or (self.sock,):
            s.setblocking(False)
            try:
                while True:
                    s.recvfrom(4096)
            except (BlockingIOError, ConnectionResetError, OSError):
                pass
            s.setblocking(True)

    def collect(self, seconds, socks=None, until_op=None):
        """Receive Art-Net packets from the node on the given sockets for `seconds`
        (or until the first packet with opcode `until_op`)."""
        socks = socks or [self.sock]
        out, end = [], time.monotonic() + seconds
        sel = selectors.DefaultSelector()
        for s in socks:
            sel.register(s, selectors.EVENT_READ)
        try:
            while True:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                for key, _ in sel.select(remaining):
                    try:
                        data, src = key.fileobj.recvfrom(4096)
                    except (ConnectionResetError, OSError):
                        continue
                    if src[0] != self.node:
                        continue
                    if not data.startswith(ARTNET_ID):
                        # From the node but not Art-Net: e.g. a reply with stale
                        # protocol headers in front of the payload.
                        self.malformed.append((key.fileobj.getsockname()[1], data))
                        continue
                    out.append(Rx(time.monotonic(), src, key.fileobj.getsockname()[1], data))
                    if until_op is not None and out[-1].op == until_op:
                        return out
        finally:
            sel.close()
        return out

    def poll(self, quiet=False, first_only=False):
        """Unicast ArtPoll, return the parsed ArtPollReply. Retries up to 3 times;
        self.last_poll_attempts tells how many were needed (> 1 = a poll was swallowed)."""
        for attempt in range(3):
            self.last_poll_attempts = attempt + 1
            self.drain()
            self.send(pkt_poll())
            rx = self.collect(POLL_WINDOW_S, until_op=OP_POLL_REPLY if first_only else None)
            replies = [r for r in rx if r.op == OP_POLL_REPLY]
            if replies:
                return parse_poll_reply(replies[0].data), replies
            if not quiet:
                self.info(f"no ArtPollReply to ArtPoll (attempt {attempt + 1}/3)")
        return None, []

    # ---- reporting
    def check(self, ok, text):
        if ok:
            self.passed += 1
            print(f"  [PASS] {text}")
        else:
            self.failed += 1
            print(f"  [FAIL] {text}")
        return ok

    @staticmethod
    def info(text):
        print(f"  [INFO] {text}")

    @staticmethod
    def watch(text):
        print(f"  [WATCH] {text}")

    def header(self, title, watch_lines=()):
        print()
        print("=" * 78)
        print(f" {title}")
        print("=" * 78)
        for w in watch_lines:
            self.watch(w)
        if self.interactive:
            input("  -- press Enter to run --")


# --------------------------------------------------------------------------- tests

def print_poll_reply(t, pr):
    nr = pr["node_report"]
    t.info(f"'{pr['short_name']}' / '{pr['long_name']}'  IP {pr['ip']}  MAC {pr['mac']}  "
           f"ver {pr['version']:#06x}  len {pr['length']} B")
    t.info(f"NodeReport: {pr['node_report_raw']!r}")
    if nr is None:
        t.info("NodeReport does not match the custom '#xxxx [yyyy] @aaa $ppp MODE=[..]' format")
    t.info(f"Status1 {pr['status1']:#04x}  Status2 {pr.get('status2', 0):#04x}  "
           f"Status3 {pr.get('status3', 0):#04x}  BindIndex {pr.get('bind_index')}  "
           f"refresh {pr.get('refresh_rate')} Hz")
    for i in range(min(pr["num_ports"], 4)):
        go = pr["good_output_a"][i]
        t.info(f"port {i + 1}: Port-Address {pr['port_addresses'][i]:5d} "
               f"(Net {pr['net']} Sub {pr['sub']} Uni {pr['sw_out'][i]})  "
               f"GoodOutputA {go:#04x}  {'DATA' if go & 0x80 else 'no data'}")


def test_poll(t):
    t.header("POLL - ArtPollReply fields",
             ["Wireshark: one ArtPollReply to <this PC>:6454 AND one to <this PC>:%d" % t.local_port])
    pr, replies = t.poll()
    if not t.check(pr is not None, "ArtPollReply received on the source port"):
        return None
    print_poll_reply(t, pr)
    t.check(len(replies) == 1, f"exactly one copy on the source port (got {len(replies)})")
    t.check(pr["length"] == 239, f"length 239 B (got {pr['length']})")
    t.check(pr["ip"] == t.node, f"IP field = node IP ({pr['ip']})")
    t.check((pr["status1"] >> 6) == 0b11, f"Status1 bits 7-6 = 11 'indicators normal' ({pr['status1']:#04x})")
    t.check(pr["acn_priority"] == 0, f"AcnPriority = 0, no inputs (got {pr['acn_priority']})")
    t.check(pr.get("bind_index") == 1, f"BindIndex = 1 (got {pr.get('bind_index')})")
    t.check(all(pa != 0 for pa in pr["port_addresses"]),
            f"no port uses deprecated Port-Address 0 ({pr['port_addresses']})")
    t.check(pr["node_report"] is not None, "NodeReport parses as the custom format")
    if any(go & 0x80 for go in pr["good_output_a"][:pr["num_ports"]]):
        t.info("GoodOutputA shows DATA -> Lightkey is sending to at least one port")
    else:
        t.info("GoodOutputA shows no data -> no ArtDmx in the last 5 s (start Lightkey output to see DATA)")
    return pr


def test_ignored(t, repeat):
    n = len(IGNORED_PACKETS) * repeat
    t.header("IGNORED - unhandled opcodes / junk must be dropped silently",
             [f"Live Watch: note artnet_ignored_count now; it must rise by >= {n}",
              "  (Lightkey's own ArtPoll/ArtPollReply broadcasts also count, so it may rise more)",
              "Wireshark: no reply from the node to any of these packets"])
    before, _ = t.poll()
    if not t.check(before is not None, "baseline ArtPollReply"):
        return
    if before["node_report"] and before["node_report"]["code"] == "0004":
        t.info(f"NodeReport already shows RcParseFail before the test: '{before['node_report']['detail']}'")
    t.drain()
    for _ in range(repeat):
        for _, pkt in IGNORED_PACKETS:
            t.send(pkt)
            time.sleep(0.002)  # stay well below the node's RX pool
    for name, _ in IGNORED_PACKETS:
        t.info(f"sent {repeat} x {name}")
    stray = t.collect(SILENCE_WINDOW_S)
    t.check(not stray, f"node did not answer any ignored packet (got {len(stray)} replies)")
    after, _ = t.poll()
    if not t.check(after is not None, "ArtPollReply after the burst"):
        return
    t.info(f"NodeReport before: {before['node_report_raw']!r}")
    t.info(f"NodeReport after:  {after['node_report_raw']!r}")
    t.check(report_key(after) == report_key(before), "NodeReport code + detail unchanged")
    bad = ("Unsupported OpCode", "Header Too Short")
    t.check(not any(b in after["node_report_raw"] for b in bad), "no 'Unsupported OpCode' / 'Header Too Short'")
    t.watch(f"artnet_ignored_count should now be >= previous + {n}")


def test_input(t):
    t.header("INPUT - ArtInput -> immediate ArtPollReply, NodeReport unchanged",
             ["Wireshark: ArtInput, then an ArtPollReply within a few ms (no 0..1 s poll delay)"])
    before, _ = t.poll()
    if not t.check(before is not None, "baseline ArtPollReply"):
        return

    for bind in (1, 0):
        t.drain()
        t0 = t.send(pkt_input(bind_index=bind))
        rx = [r for r in t.collect(REPLY_WINDOW_S) if r.op == OP_POLL_REPLY]
        if t.check(bool(rx), f"ArtInput BindIndex {bind} -> ArtPollReply"):
            ms = (rx[0].t - t0) * 1000
            t.check(ms < 200, f"reply is immediate ({ms:.0f} ms)")
            t.check(len(rx) == 1, f"exactly one copy on the source port (got {len(rx)})")
            pr = parse_poll_reply(rx[0].data)
            t.check(report_key(pr) == report_key(before),
                    f"NodeReport unchanged ({pr['node_report_raw']!r})")

    t.drain()
    t.send(pkt_input(bind_index=2))
    rx = [r for r in t.collect(SILENCE_WINDOW_S) if r.op == OP_POLL_REPLY]
    t.check(not rx, "ArtInput BindIndex 2 (not ours) -> no reply")

    # ArtPoll queues a delayed reply; ArtInput right after must replace it, not add a second one.
    t.drain()
    t.send(pkt_poll())
    t.send(pkt_input())
    rx = [r for r in t.collect(POLL_WINDOW_S) if r.op == OP_POLL_REPLY]
    t.check(len(rx) == 1, f"ArtPoll + ArtInput back-to-back -> one reply, queued one cancelled (got {len(rx)})")


DATA_REQ_CODES = {0x0001: "DrUrlProduct", 0x0002: "DrUrlUserGuide", 0x0003: "DrUrlSupport"}
DATA_REQ_UNSUPPORTED = {0x0004: "DrUrlPersUdr", 0x0005: "DrUrlPersGdtf", 0x8000: "DrManSpec 0x8000"}


def data_request(t, code, window):
    t.drain()
    t0 = t.send(pkt_data_request(code))
    rx = [r for r in t.collect(window) if r.op == OP_DATA_REPLY]
    return rx, t0


def test_datareq(t):
    t.header("DATAREQ - ArtDataRequest -> ArtDataReply",
             ["Wireshark: ArtDataReply (OpCode 0x2800) to 6454 and to the source port",
              "Live Watch: stack_monitor_report tcpip_thread min free stack should not drop"])
    before, _ = t.poll(quiet=True)

    rx, t0 = data_request(t, 0x0000, REPLY_WINDOW_S)
    if t.check(bool(rx), "DrPoll -> ArtDataReply"):
        dr = parse_data_reply(rx[0].data)
        t.check(len(rx) == 1, f"exactly one copy on the source port (got {len(rx)})")
        t.check(dr["request"] == 0 and dr["pay_len"] == 0 and dr["length"] == 20,
                f"DrPoll reply: request 0, PayLen 0, 20 B (got {dr['request']}, {dr['pay_len']}, {dr['length']} B)")
        t.check(dr["prot_ver"] == 14, f"ProtVer 14 (got {dr['prot_ver']})")
        t.info(f"EstaMan {dr['esta']:#06x}  Oem {dr['oem']:#06x}  ({(rx[0].t - t0) * 1000:.0f} ms)")

    for code, name in DATA_REQ_CODES.items():
        rx, _ = data_request(t, code, REPLY_WINDOW_S)
        if not t.check(bool(rx), f"{name} ({code:#06x}) -> ArtDataReply"):
            continue
        dr = parse_data_reply(rx[0].data)
        payload = dr["payload"]
        url = cstr(payload)
        t.info(f"{name}: {url}")
        t.check(dr["request"] == code, f"request code echoed ({dr['request']:#06x})")
        t.check(payload.endswith(b"\x00") and dr["pay_len"] == len(url) + 1 and dr["length"] == 20 + dr["pay_len"],
                f"PayLen = strlen + NUL = {dr['pay_len']}, packet {dr['length']} B")
        t.check(url.startswith("https://"), "payload is a URL")

    for code, name in DATA_REQ_UNSUPPORTED.items():
        rx, _ = data_request(t, code, SILENCE_WINDOW_S)
        t.check(not rx, f"{name} ({code:#06x}) -> no reply (= not supported)")

    after, _ = t.poll(quiet=True)
    if before and after:
        t.check(report_key(after) == report_key(before), "NodeReport unchanged by ArtDataRequest")


def test_replyport(t):
    t.header("REPLYPORT - copies to 6454 and to the source port",
             ["Wireshark filter: udp.port == 6454 && artnet.header.opcode == 0x2100"])
    try:
        s6454 = t._open_socket(ARTNET_PORT)
    except OSError as e:
        t.info(f"cannot bind UDP 6454 on this PC ({e.strerror or e}); Lightkey/QLC+ probably owns it.")
        t.watch(f"Check in Wireshark instead: each request from this script (src port {t.local_port}) "
                f"gets TWO replies: dst port 6454 and dst port {t.local_port}.")
        t.watch("A request from Lightkey (src port 6454) gets exactly ONE reply, to 6454.")
        return
    try:
        # 1) Request from the ephemeral port: expect one copy on each socket.
        t.drain(t.sock, s6454)
        t.send(pkt_input())
        rx = [r for r in t.collect(REPLY_WINDOW_S, [t.sock, s6454]) if r.op == OP_POLL_REPLY]
        on_eph = sum(1 for r in rx if r.dst_port == t.local_port)
        on_6454 = sum(1 for r in rx if r.dst_port == ARTNET_PORT)
        t.check(on_eph == 1, f"request from :{t.local_port} -> 1 reply on :{t.local_port} (got {on_eph})")
        if not t.check(on_6454 == 1, f"request from :{t.local_port} -> 1 reply on :6454 (got {on_6454})"):
            t.info("if Wireshark shows the 6454 copy, Windows Firewall is dropping it. Check for an inbound Block rule for "
                   "THIS python.exe (rules are per interpreter) and the NIC's network profile (an unidentified 2.x network is Public)")

        # 2) Request from 6454 itself: exactly one reply, no duplicate.
        t.drain(t.sock, s6454)
        t.send(pkt_input(), sock=s6454)
        rx = [r for r in t.collect(REPLY_WINDOW_S, [t.sock, s6454]) if r.op == OP_POLL_REPLY]
        on_eph = sum(1 for r in rx if r.dst_port == t.local_port)
        on_6454 = sum(1 for r in rx if r.dst_port == ARTNET_PORT)
        t.check(on_6454 == 1 and on_eph == 0,
                f"request from :6454 -> exactly 1 reply on :6454, none elsewhere (got {on_6454} / {on_eph})")

        # 3) Delayed (queued) poll reply follows the same rule.
        t.drain(t.sock, s6454)
        t.send(pkt_poll())
        rx = [r for r in t.collect(POLL_WINDOW_S, [t.sock, s6454]) if r.op == OP_POLL_REPLY]
        on_eph = sum(1 for r in rx if r.dst_port == t.local_port)
        on_6454 = sum(1 for r in rx if r.dst_port == ARTNET_PORT)
        t.check(on_eph == 1 and on_6454 == 1,
                f"queued ArtPoll reply -> 1 on :{t.local_port} and 1 on :6454 (got {on_eph} / {on_6454})")
    finally:
        s6454.close()


def test_parsefail(t, restore):
    t.header("PARSEFAIL - malformed packets of handled opcodes still set RcParseFail",
             ["This test changes the NodeReport; it is restored with a no-op ArtCommand "
              "(DmxAddress=<current>) unless --no-restore",
              "Live Watch: artnet_ignored_count must NOT rise for these two packets"])
    cases = [("ArtInput 15 B", pkt_input()[:15], "ArtInput Too Short"),
             ("ArtDataRequest 16 B", pkt_data_request(1)[:16], "ArtDataReq Too Short")]
    current = None
    for name, pkt, detail in cases:
        t.drain()
        t.send(pkt)
        rx = t.collect(SILENCE_WINDOW_S)
        t.check(not rx, f"{name} -> no direct reply")
        pr, _ = t.poll()
        if not t.check(pr is not None, "ArtPollReply"):
            return
        current = pr
        t.info(f"NodeReport: {pr['node_report_raw']!r}")
        t.check(report_key(pr) == ("0004", detail), f"{name} -> #0004 '{detail}'")

    if not restore or current is None or current["node_report"] is None:
        return
    addr = current["node_report"]["address"]
    t.drain()
    t.send(pkt_command(f"DmxAddress={addr}&"))
    rx = [r for r in t.collect(REPLY_WINDOW_S) if r.op == OP_POLL_REPLY]
    if t.check(bool(rx), f"restore: ArtCommand 'DmxAddress={addr}&' -> ArtPollReply"):
        pr = parse_poll_reply(rx[0].data)
        t.info(f"NodeReport: {pr['node_report_raw']!r}")
        t.check(pr["node_report"] is not None and pr["node_report"]["code"] == "0001",
                "NodeReport back to #0001")


def enter_pressed():
    """Non-blocking: True if Enter was pressed since the last call."""
    if sys.platform == "win32":
        import msvcrt
        hit = False
        while msvcrt.kbhit():
            hit = msvcrt.getwch() in ("\r", "\n") or hit
        return hit
    import select
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def monitor(t, interval):
    t.header("MONITOR - GoodOutputA per port (Ctrl+C to stop)",
             ["Start Lightkey output: ports driven by Lightkey show DATA.",
              "Then STOP the output (disable the Art-Net interface or quit Lightkey; "
              "a blackout still sends ArtDmx) and press Enter at the same moment.",
              "DATA must clear about 5-7 s after Enter (5 s timeout + poll period + 0..1 s reply delay).",
              "'swallowed' = the first poll got no reply; with Lightkey on this PC it must stay 0.",
              "Live Watch: failsafe_count rises once when the signal is lost, render_count stops."])
    last_state, marker, start = None, None, time.monotonic()
    polls = swallowed = 0
    try:
        while True:
            t_loop = time.monotonic()
            if enter_pressed():
                marker = t_loop
                print(f"  {marker - start:7.1f}s  ---- marker (output stopped?) ----")
            pr, _ = t.poll(quiet=True, first_only=True)
            now = time.monotonic()
            polls += 1
            note = ""
            if t.last_poll_attempts > 1:
                swallowed += 1
                note = f"   (swallowed: reply on attempt {t.last_poll_attempts})"
            if pr is None:
                print(f"  {now - start:7.1f}s  no reply{note}")
            else:
                n = min(pr["num_ports"], 4)
                state = tuple(bool(pr["good_output_a"][i] & 0x80) for i in range(n))
                ports = "  ".join(f"P{i + 1}(PA {pr['port_addresses'][i]}):{'DATA' if s else '----'}"
                                  for i, s in enumerate(state))
                nr = pr["node_report"]
                rep = f"#{nr['code']} [{nr['counter']:04d}] {nr['detail']}" if nr else pr["node_report_raw"]
                line = f"  {now - start:7.1f}s  {ports}   {rep}{note}"
                if last_state is not None and state != last_state:
                    line += "   <-- changed"
                    if not any(state) and marker is not None:
                        line += f" ({now - marker:.1f} s after marker)"
                print(line)
                last_state = state
            time.sleep(max(0.0, interval - (time.monotonic() - t_loop)))
    except KeyboardInterrupt:
        print(f"\n  stopped: {polls} polls, {swallowed} swallowed")


# --------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", nargs="?", default="all",
                    choices=["all", "poll", "ignored", "input", "datareq", "replyport", "parsefail", "monitor"])
    ap.add_argument("--node", required=True, help="fixture IP, 2.x.y.z as shown on its OLED")
    ap.add_argument("--bind", default="", help="local IP to send from (pick the NIC on the fixture's network)")
    ap.add_argument("--repeat", type=int, default=20, help="'ignored': copies of each junk packet (default 20)")
    ap.add_argument("--interval", type=float, default=1.0, help="'monitor': poll period in s (default 1.0)")
    ap.add_argument("--no-restore", action="store_true", help="'parsefail': leave the RcParseFail NodeReport")
    ap.add_argument("-i", "--interactive", action="store_true", help="pause before each test")
    args = ap.parse_args()

    t = Tester(args.node, args.bind, args.interactive)
    print(f"Node {args.node}:{ARTNET_PORT}   local {t.sock.getsockname()[0] or '0.0.0.0'}:{t.local_port}")
    print("Wireshark: udp.port == 6454 || udp.port == %d" % t.local_port)

    if args.test == "monitor":
        monitor(t, args.interval)
        return 0

    tests = {
        "poll": lambda: test_poll(t),
        "ignored": lambda: test_ignored(t, args.repeat),
        "input": lambda: test_input(t),
        "datareq": lambda: test_datareq(t),
        "replyport": lambda: test_replyport(t),
        "parsefail": lambda: test_parsefail(t, not args.no_restore),
    }
    for name in (tests if args.test == "all" else [args.test]):
        tests[name]()

    if t.malformed:
        print()
        print(f"  [WARN] {len(t.malformed)} non-Art-Net datagram(s) from the node were dropped by this script:")
        for port, data in t.malformed[:5]:
            at = data.find(ARTNET_ID)
            where = f"'Art-Net' found at offset {at}" if at >= 0 else "no 'Art-Net' ID inside"
            print(f"         on :{port}, {len(data)} B, {where}, first bytes {data[:16].hex(' ')}")
    print()
    print("=" * 78)
    print(f" {t.passed} passed, {t.failed} failed")
    print("=" * 78)
    if args.test == "all":
        print("  Next: run 'monitor' and stop Lightkey output to check GoodOutputA clears after ~5 s.")
    return 1 if t.failed else 0


if __name__ == "__main__":
    sys.exit(main())
