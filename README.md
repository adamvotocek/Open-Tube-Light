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
| `*.qxw` | QLC+ workspaces used for testing (single universe, 4 universes, 8 segments) |
