# Plan: flash persistence (settings survive power-off)

**Status (2026-11-14):** design decided, not started. This is `artnet-compliance.md` §12 step 4 (infrastructure 11.2) and `device-config-refactor.md` phase 4. Where those older texts disagree with this plan, this plan wins.

## In short

- Settings live in RAM while the tube runs. A save appends a whole settings record to a **journal** spread over **two reserved 128 KB flash sectors**.
- A save only *writes* flash. That takes microseconds and nobody notices it.
- A sector is *erased* only **at boot**, never while the tube is running. On this chip an erase freezes everything for about 1–2 s.
- Saves are **lazy and de-duplicated**: 3 s after the last change, at most once every 30 s, and only if something changed.
- The boot scan must survive a **half-written flash word**. On the H7 that raises an ECC fault when it is read (see "Power cuts").

## Why it is designed this way

### The chip

- The STM32H723VE has 512 KB of flash in 4 sectors of 128 KB, all in **one bank**. The program runs from the same flash that would hold the settings.
- **Write (program):** 32 bytes at a time (one "flash word" of 256 bits plus ECC bits). Each flash word can be written only once per erase. That takes microseconds.
- **Erase:** the whole 128 KB sector at once. ST forum figures for the H7x3 family give a typical 1.1 s and a maximum of 2.2 s at x32 parallelism, or 1 s / 2 s at x64.
- While an erase or write runs, **nothing can be read from that bank**, so the CPU stops fetching instructions. Tasks, interrupts, Ethernet, DMX512 and the LED refresh all stop. For a write this lasts microseconds. For an erase it is a full 1–2 s freeze: the LEDs hold their last frame, and Art-Net and DMX512 frames are lost.
- **Endurance:** 10,000 erases per sector (the H7 family datasheet figure, from the H743; confirm it in the H723 datasheet's "Flash memory endurance and data retention" table). Reads cause no wear. Data retention also drops as a sector nears its erase limit.

### Consequences

| Design | Freeze | Saves until the rated erase limit | Controller changing a setting every second |
|---|---|---|---|
| Erase + write on every save | 1–2 s per save | 10,000 | worn out within hours |
| Journal in 2 sectors, lazy saves (this plan) | none at runtime | ~10 million (2 × ~500 × 10,000) | capped at 120 saves/h → ~9.5 years nonstop |

At 20 saves on a busy day the journal lasts over 1,000 years. The journal is its own wear levelling: every slot is used once before an erase, and the two sectors take turns.

## Flash layout

| Sector | Address | Use |
|---|---|---|
| 0–1 | `0x08000000`–`0x0803FFFF` | Firmware, 256 KB. Today the Release build (`-Os`) uses 92 KB and the Debug build (`-O0`) 162 KB. |
| 2 | `0x08040000`–`0x0805FFFF` | Settings journal, sector A |
| 3 | `0x08060000`–`0x0807FFFF` | Settings journal, sector B |

- Shrink `FLASH` in `STM32H723XX_FLASH.ld` to 256 KB and export the journal bounds as linker symbols, so outgrowing the firmware space is a **link error**, not silently overwritten settings. Update `memory-layout.md`.
- If Debug builds get tight, switch `CMAKE_C_FLAGS_DEBUG` from `-O0` to `-Og`. It is much smaller and still debuggable.
- An ST-Link reflash normally erases only the sectors the image occupies, so settings survive firmware updates. A full chip erase clears them. Verify this once on the bench.

## Records

Each record is aligned to a flash word (32 B) and padded to a whole number of flash words:

| Field | Size | Notes |
|---|---|---|
| magic | 4 B | Marks a record start. The scanner resynchronises on it. |
| type | 2 B | `SETTINGS`, later `SCENE`, `COUNTERS` (see "What gets stored") |
| format version | 2 B | Newer firmware reads old versions and fills new fields with defaults. **A firmware update never resets settings.** |
| payload length | 2 B | |
| reserved | 2 B | |
| sequence | 4 B | Increases by one per record, across both sectors. The highest valid sequence of each type wins. |
| payload | n B | `DeviceConfig_t` for `SETTINGS` (124 B today) |
| CRC32 | 4 B | Over header + payload. A real CRC32; the H7 CRC peripheral can compute it. |

A settings record is 160 B today (5 flash words), so a sector holds about 800 of them. All numbers in this plan assume about 500 per sector, to leave room for the record to grow.

**Rejected:** one record per variable (the style of ST's X-CUBE-EEPROM). The H7 writes at least 32 B at a time, so a single changed variable still costs a whole flash word. That saves very little space and needs per-variable compaction. A whole record is always self-consistent and simple.

## Algorithms

### Boot (in `ControlTask`, before any driver or LwIP init)

1. Scan both sectors flash word by flash word using the **guarded read** (see "Power cuts"). Collect the newest valid record of each type (magic, length and CRC all OK). Skip anything damaged and resynchronise on the next magic.
2. Note the **active** sector: the one holding the newest record. Note where its free space starts: the first flash word after the last record that is blank (all `0xFF`).
3. **Stale sector:** if the other sector is not blank and holds nothing newer, erase it now. That is a leftover from a swap, or an interrupted erase.
4. **Proactive compaction:** if the active sector is more than 75 % full and the other one is blank, write the current newest records into the other sector, read them back to verify, then erase the old sector. Each running session then starts with plenty of room.
5. No valid `SETTINGS` record means first boot or full corruption. Use the defaults and write them as the first record.

Boot gets 1–2 s longer only when step 3 or 4 erases, which happens about once per ~400 saves. The strip is dark and the network is not up yet, so nobody sees it. If an independent watchdog is ever added, its timeout must exceed the maximum erase time (it can't be refreshed during the stall).

### Save (in `ControlTask`, from the config service's dirty flag)

1. Compare the new record's payload with the last saved one. If they are equal, do nothing.
2. If the active sector has room: check that the target flash words are blank (guarded read), program them, then read back and compare. If a word is not blank (torn by an earlier power cut), skip past it. Never program a flash word twice.
3. If the active sector is full and the other is blank: write all current newest records into the other sector, which becomes active. The full sector is erased **at the next boot**.
4. If the active sector is full and the other one has not been erased yet (no reboot since the last swap): keep the change in RAM, set a "save pending" state (Live Watch counter now, an OLED/NodeReport hint later), and write it at the next boot. With the 30 s limit this needs 4+ hours of nonstop setting changes without a reboot.

Steps 3–4 never erase at runtime, so a save never freezes the tube.

### Save timing (the lazy part)

- A change takes effect in RAM immediately. Art-Net and RDM replies come from RAM, so controllers see the new values at once.
- Flash is written when **3 s have passed without further changes**, and **at most once every 30 s**. A slider drag or a scrolled menu value becomes one save. A controller hammering changes nonstop gets at most 120 saves an hour. A power cut loses at most the last ~30 s of changes.
- The menu commits on confirm, not on every button press (`device-config-refactor.md` phase 6).
- **Factory reset** = save the defaults as a normal record. Never erase both sectors at runtime.

## What gets stored, and how often it changes

Every new persisted field must be given one of these categories when it is added. Add the category to the field's comment in `device_config.h`.

| Category | Examples (now and planned) | Rule |
|---|---|---|
| **Setting**: set once, rarely touched | Names (Art-Net short/long, RDM device label); Net/Sub-Net/Universe, sACN universe/priority, DMX start address, input source, per-port Art-Net/sACN choice, merge mode; pixel/segment count, format, RDM personality, colour order; DHCP/static IP, mask, gateway; failsafe mode, power-on behaviour, dimming curve, brightness limit; OLED timeout/brightness/flip, panel lock | In the `SETTINGS` record. Lazy save. |
| **Large item**: rarely written | Failsafe/startup scene from ArtAddress `AcFailRecord` or RDM: 432 B for 144 RGB pixels, 864 B for 288, more for RGBW/16-bit, up to 4 × 512 B | Its own record type (`SCENE`), so a name change doesn't rewrite it. Rate-limited like settings. |
| **Counter**: changes by design | RDM `DEVICE_HOURS`, `DEVICE_POWER_CYCLES` (optional PIDs) | Counted in RAM. Saved with the next normal save or every few hours, never on every increment. Losing up to the last hour at power-off is accepted practice. |
| **Runtime only**: never stored | Locate/identify, mute, current colours, which controller owns a universe, sequence numbers, ArtSync state | Never in a record. |
| **Factory data**: written once | Per-unit colour calibration, if ever added | Survives a factory reset. Decide where it lives when it exists. |

**Sources of rapid changes** to keep in mind:
- a person scrubbing a value (held menu button, OpenTubeDia slider, RDM tool sending a SET per drag step);
- controllers re-sending the same ArtAddress or RDM SETs (handled by the equality check);
- a console macro sending `AcFailRecord` on every cue;
- counters.

Reads (ArtPoll, RDM GET, Live Watch) never write anything.

## Power cuts

- **Interrupted record write:** the CRC fails, so the previous record of that type stays the newest. At most the newest change is lost.
- **Interrupted erase:** the sector is half-erased garbage holding nothing newer. Boot step 3 erases it again. The newest records are always in the *other* sector, because the design never erases the sector that holds them. That is why there are two sectors.
- **ECC trap (STM32H7 specific).** Every flash word carries ECC bits. A word whose programming was cut off can have inconsistent ECC, and *reading* it raises a double-error (DBECCERR) bus fault. On the ST forum this has caused boards that fault on every boot, and in one case a sector that STM32CubeProgrammer could not read until a full chip erase. So:
  - all journal reads go through one **guarded read** helper: run at FAULTMASK with `SCB->CCR.BFHFNMIGN` set so the precise data bus fault is ignored, then check and clear the FLASH ECC error flags (`FLASH_SR1` DBECCERR/SNECCERR, failing address in `FLASH_ECC_FA1R`). Invalidate the D-cache for the journal range first, so the read really goes to flash. **Verify on the bench that this catches the fault**; fall back to recognising the address in the HardFault handler if it does not;
  - a damaged word is treated as "not blank, not valid": skipped, never programmed;
  - single-bit errors are corrected by hardware and only counted.

## Rejected and deferred alternatives

- **One sector** (journal with erase when full). There is always a 1–2 s window with **no** valid copy in flash: from the start of the erase until the newest record is written back. A power cut in that window boots the tube on factory defaults, with a different address and name and nothing telling the operator why. Erases also tend to happen exactly when crews switch rigs off (show over → idle → erase → power off). **May be needed in the distant future** if firmware outgrows 256 KB or a bootloader needs the space. Keep the sector count a single constant in the code so the switch stays small.
- **Idle-time erase** (erase the full sector while there is no DMX data). Not needed: with two sectors, boot-only erasing runs out only after ~500+ saves without a reboot. Add it later only if counters are stored on a 24/7 installation that never reboots.
- **Running the whole program from RAM.** Possible: the image is 92 KB (Release) or 162 KB (Debug), and the 256 KB `RAM_D1` is unused. A startup copy takes about 1 ms; then no flash access stalls the CPU and erases can happen at any time. Costs:
  - code in RAM can be overwritten by a stray pointer, so it needs a read-only MPU region (CubeMX);
  - a one-time linker/startup change that must survive CubeMX regeneration;
  - *everything* that runs during an erase must be in RAM: code, constants and the vector table.

  It removes only the freeze, not the need for the journal (wear, power cuts). Deferred: keep it in mind if a future feature has to erase while a show runs.
- **External memory** (I2C/SPI EEPROM or FRAM). FRAM has ~10^14 write cycles, byte writes and no erase, so it removes every trade-off above and frees all internal flash for firmware. It needs a PCB change. An I2C part could share I2C1 with the OLED, with the OLED DMA traffic and storage access serialised in `ControlTask`. Revisit at a PCB respin.
- **STM32H723VG** (same chip and package, 1 MB / 8 sectors). Also a respin option. It would leave room for a bootloader and network firmware updates (`artnet-compliance.md` §9, ArtFirmwareMaster), which two settings sectors rule out on the 512 KB part.
- **Save on power failure** (PVD interrupt). It needs guaranteed hold-up energy on the board. Not reliable without it.

## Module placement

- `Core/Src/Device-Config/device_config_flash.c` + `device_config_flash_internal.h` hold the journal: scan, save, swap, compaction, guarded read and CRC. It is the only code that touches the FLASH peripheral. It owns flash storage of the config record, so it sits inside the `Device-Config` module. Rejected: a generic `Flash-Storage` module, a premature abstraction with a single user.
- `ControlTask` calls load at boot and save from the config-service loop (`config-commits-and-artaddress.md` Phase B adds the generation counter that drives the dirty check).

## Verification

- Build: only the baseline warning. The memory table shows `FLASH` at 256 KB.
- Bench, normal: change settings via ArtCommand/ArtAddress, power-cycle, and confirm they survive. Change a value 50 times quickly and confirm one save (Live Watch save counter).
- Bench, swap: temporarily shrink the journal capacity (a test build constant) so a swap and the boot erase happen within a few saves.
- Bench, power cuts: a test mode that saves continuously while power is pulled repeatedly (20+ times, including during the boot-time erase). Every boot must come up with the last good settings, and never with a fault.
- Bench, reflash: flash a new build with ST-Link and confirm settings survive.
- Extend `tools/artnet_bench_test.py` with a persistence check: set via ArtAddress, ask the user to power-cycle, then poll and compare.

## Open before implementing

- Confirm `tERASE` (sector erase) and `NEND` (endurance) in the STM32H723 datasheet.
- Confirm that the guarded read (FAULTMASK + BFHFNMIGN) catches a flash DBECCERR on this core, or use the HardFault fallback.