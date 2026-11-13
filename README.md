# Open-Tube-Light

An open-hardware LED tube fixture for stage and installation lighting. It takes Art-Net over Ethernet or DMX512 over XLR and drives a 144-pixel SK9822 strip inside an acrylic tube, with an OLED status display and buttons on the fixture.

## Repository layout

| Path | Contents |
|---|---|
| [`Code/Open-Tube-Light/`](Code/Open-Tube-Light/) | STM32H723 firmware (CubeMX + FreeRTOS + LwIP) — see its [README](Code/Open-Tube-Light/README.md) |
| [`PCB/ledTubeFixture/`](PCB/ledTubeFixture/) | Main board, KiCad (MCU, Ethernet, DMX, power, display & buttons) |
| [`PCB/xlrConBoards/`](PCB/xlrConBoards/) | XLR connector boards, KiCad |
| [`PCB/releases/`](PCB/releases/) | Fabrication packages for released board revisions |
| [`PCB/TODOs.txt`](PCB/TODOs.txt) | Known issues for the next board revision |
| [`Enclosure/`](Enclosure/) | Mechanical drawings (acrylic tube) |


## Art-Net

The fixture is an Art-Net 4 node (output only). Discovery (ArtPoll), ArtSync, ArtCommand and ArtDataRequest (product, user guide and support URLs) are supported. Spec coverage is tracked in [`Code/Open-Tube-Light/docs/artnet-compliance.md`](Code/Open-Tube-Light/docs/artnet-compliance.md).

**Multiple controllers / merging.** The fixture does not merge (no HTP/LTP). Each universe is owned by the first controller that sends ArtDmx to it. ArtDmx for that universe from any other IP address is ignored until the owner has sent nothing for 5 seconds, after which the next sender takes over. ArtSync is honoured per controller: a universe enters synchronous mode only on ArtSync from its own controller.

Art-Net™ Designed by and Copyright Artistic Licence
