# Issues

Bug log for hardware-observed problems. Newest open issues first; move entries to *Solved* with the fix once verified on the bench.

## Open

### DMX512: one-frame glitches with a cheap USB-DMX adapter

**Symptom.** With a cheap USB→DMX adapter the strip occasionally blacks out or shows shifted pixel data for a single frame, then recovers. A high-end sACN→DMX node on the same setup is glitch-free. Glitch rate grows with refresh rate: ~1 per 10 s at 30–40 Hz, ~1 per second at 44 Hz.

**Measured transmitter timings** (DMX512-A requirements: `docs/reference/dmx512-A.md`, Tables 6/7):

| # | Parameter | High-end node | Cheap adapter |
|---|---|---|---|
| — | Bit rate | 250 kbit/s ✓ | 250 kbit/s ✓ |
| — | Bit time, logic 0 | 4 µs ✓ | 4.5 µs ✗ (+0.5 µs) |
| — | Bit time, logic 1 | 4 µs ✓ | 3.5 µs ✗ (−0.5 µs) |
| 1 | BREAK | 234 µs ✓ | 1.5 ms ✓ |
| 2 | MAB | 38 µs ✓ | 1.3 ms ✓ |
| 7+8+9 | Stop bits + inter-slot MARK | 14 µs ✓ | 7.5 µs ✗ (≈ 1 stop bit) |
| 10 | MBB | 0 ✓ | 780 µs ✓ |

**Observations.** Received byte count at packet processing (then `recievedData`, now `receivedData` in `dmx512_uart.c`) is a steady 513 with the good node. With the cheap adapter it is mostly 480–513 and occasionally drops to 0–30 — those drops coincide with the visible glitches. Test layout was 144 × 1-pixel RGB segments (432 channels), so short-but-not-tiny packets were invisible on the strip.

**Tried.** UART set to 1 stop bit instead of 2 — no change.

**Since the report** the driver gained partial-universe support and mid-packet error / short-frame handling (commits `97895f9`, `b01b40a`, `f57beac`). Re-test with the adapter and watch `receivedData`, `smallFrameCnt`, `midPcktErrCnt` in Live Watch.

### DMX512: tearing when strobing fast

**Setup.** Lightkey → sACN (40 Hz) → Showtec NET 2/3 Pocket → DMX512 (40 Hz) → device. Fast strobe (10 Hz, 50 % duty) across a full universe (42 × 4-beam RGB fixtures).

**Symptom.** Tearing and glitched pixels on the strip.

**Next step.** Logic analyzer on the DMX line to decide whether the tearing is already in the transmitted signal (Lightkey/node rendering) or introduced by the receiver.

## Accepted / by design

### DMX512 ISR does memcpy/memset above BASEPRI

`DMX512_Uart_ProcessPacket()` runs from the UART5 ISR at priority 3, above `configMAX_SYSCALL_INTERRUPT_PRIORITY` (5), so it blocks every FreeRTOS-aware interrupt while it runs. Worst case is a 512-byte `memcpy` plus a 512-byte `memset` in DTCM — a few µs at 500 MHz. Acceptable today; re-evaluate if more above-BASEPRI interrupts are added.

## Solved

### Network: not reachable on a direct laptop connection after reset

**Symptom (with DHCP enabled).** Works on the home network. If moved to a direct cable to the laptop while running, it keeps working; after a reset on the direct link, it is not discoverable. Laptop: static 192.168.1.2/24, router 192.168.1.3.

**Fix.** Firmware uses a static IP (`192.168.1.179/24`, `LWIP_DHCP=0`); a direct connection after a cold reset works on the bench. The remaining unreachability was caused by the laptop's network setup, not the firmware. DHCP with fallback to Art-Net 2.x.x.x / link-local stays on the roadmap.

### Random flashes at the start of the strip

**Symptom.** With Art-Net all-zero data, the first LED flashed blue roughly once per second for ~300 ms. With a random-colour effect, the first few LEDs or whole leading segments flashed random colours. SPI DMA buffers (`spi_buf_a`/`spi_buf_b`) always contained correct data when halted mid-glitch; Wireshark showed correct Art-Net. Live Watch: `render_count`, `failsafe_count`, `refresh_count` all behaved as expected.

**Fix.** Signal integrity, not firmware: 100 Ω series resistors on the SPI data lines.
