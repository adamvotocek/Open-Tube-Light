# OpenTubeLight 

## BUGS:

### The device is not discoverable, or broadcastable with a direct ethernet connection to a laptop
- this may be an issue with the networking settings on the laptop, or with the networking settings of the device, or both.

#### When the direct connection doesn't work:
- connecting straight to the laptop, after reset, or before reset

Laptop's network settings:
- Manual IP address: 192.168.1.2
- Manual subnet mask: 255.255.255.0
- Manual router: 192.168.1.3

Device's network settings:
- idk, dhcp enabled

#### When the direct connection works:
- When we start the device and connect it to my larger home network, it works as it should - responds to data on the network. If I then unplug the device's ethernet from my home network and connect it directly to the laptop, it is discoverable and works great with the direct connection. However, if I restart the device, it stops working on the direct connection.

- The laptop's network settings are the same as above

### DMX input periodic glitches
- needs more info


## Solved bugs:

### Random flashes on the beginning of the strip - SOLVED

I am testing the device's Art-Net input. It is connected to my home network and gets its address via DHCP. The device is set to 8 segments while testing, but problems also happen with 144 segments.

I am debugging it and monitoring 3 global variables: ```render_count```, ```failsafe_count```, and ```refresh_count```.

1. When I upload the code, start debugging and wait for the device to connect to the network without sending any data to it:
    - The led strip stays off, as expected
    - ```render_count``` and ```failsafe_count``` stay at 0
    - ```refresh_count``` increments, which indicates that the LED  strip is being refreshed to stay dark
    - This is all expected and good

2. When I launch QLC+, it starts sending Art-Net data to the device, which is just currently just a universe full of zeroes. This is where the problems begin.
    - Monitoring the Art-Net data in Wireshark, I can see that all the channels are at zero, as expected
    - Even though the whole LED strip should be off, the first LED of the strip flashes with a blue color. The flashes are at seemingly random intervals, but they are not fast. One flash happens approximately once per second and it is not a single frame, the LED stays on for a random amount of time, which I approximate at around 300 milliseconds.
    - ```render_count``` increments when QLC+ refreshes the Art-Net data, approximately once per 2 seconds, which is expected.
    - ```failsafe_count``` is at 0, which is expected
    - ```refresh_count``` increments faster than ```render_count```. That indicates that the LED strip is being refreshed between the packets, which is expected
    - If I pause the debuggging while the LED is on with a blue color and monitor the output spi buffers ```spi_buf_a``` and ```spi_buf_b```, I can see that the data in the buffers is correct. Both of the buffers have the correct start frame, led frames with a 255 brightness byte and zeroes for the color bytes, and the correct end frame.
       - That might indicate that the problem is not with the data in the buffers. Or it could be that the DMA somehow sends a different buffer, or at least different data than what I see? I don't know.

3. If I start a random color effect on all the pixels in QLC+, the flashes get different. 
    - The flashes are not just blue, but a random color combination. The flashes also seem to be more frequent. Sometimes only the first LED flashes, sometimes the first few LEDs flash, and sometimes the flashes happen for a whole segments, or multiple segments.
    - it seems like either only a few first LEDs flash, or a few whole segments (at the beginning of the strip). The flashing of the segments indicates a compute error, however, despite multiple attempts, I have not been able to find the flashes in the spi buffers. The buffers seem to contain correct data, even when I stop the debugging when the glitch is displaying on the strip. 
    - ```render_count``` increments rapidly now, as QLC+ sends approximately 30 frames per second
    - ```failsafe_count``` is still at 0
    - ```refresh_count``` doesn't change
    - Wireshark data is correct 
    - if I stop the effect in QLC+, the behavior goes back to the one described in point 2

4. If I disconnect the ethernet cable, the failsafe behavior happens, which is that the device holds the last state. 
    - No more flashes happen.
    - ```render_count``` doesn't increment
    - ```failsafe_count``` increases by 1
    - ```refresh_count``` increments
    - all expected behavior 

#### This issue was solved by adding 100 Ω series resistors to the SPI data lines


## Very unorganized TODO LIST:
- spi led strips should be continuously updated, even if no new data is received. this is because interference can cause artifacts in the led strip over time. (maybe every 300 ms or so)
- Class A ip addressing after conneciton per artnet spec
- Implement ArtPollReply node status reporting
- For modes that require more than 4 universes, implement bindindex

Configuration via the display and buttons will be added to this device for changing the settings. These settings must update the functionality of the device while it is running, so the code must be structured in a way that allows for this. 
The settings must saved to flash so that they are kept if the device loses power. There needs to be a factory default device configuration that the device will start with after uploading the firmware. Also, the user will have the ability to reset the device to factory defaults. 
These are some of the settings that should be configurable:

Device name
- long name (must be compatible with Art-Net long name)
- short name (must be compatible with Art-Net short name)

IP configuration
- DHCP/static
- if static: IP address, mask

Strip layout configuration
- number of physical pixels
- segment count
- segment format (so far we only have RGB, but later there will be options for RGBD, or 16-bit RGB, etc.)

DMX protocol
- input via either DMX or Art-Net
- start address (and also start universe for Art-Net)
- note: the channel count of the device is calculated by the strip layout configuration 
- note: the number of universes that Art-Net must recieve is determined by the channel count and the start address

Failsafe behavior
- hold last state/full/zero



Other settings may appear as development continues.


## Ethernet and LwIP Setup

### Memory structure

Here is how I structured the memory for the Ethernet and LwIP components. All are 32-byte aligned.

| Item           | Name                     | File         | Memory Location | Size [B]             |
|----------------|--------------------------|--------------|-----------------|----------------------|
| TX descriptors | DMATxDscrTab             | ethernetif.c | 0x300001e0      | 96                   |
| RX descriptors | DMARxDscrTab             | ethernetif.c | 0x30000000      | 480                  |
| RX buffers     | memp_memory_RX_POOL_base | ethernetif.c | 0x24008000      | 20*(1536+32) = 31360 |
| LwIP heap      | LWIP_RAM_HEAP_POINTER    | lwipopts.h   | 0x24000000      | 32*1024              |

### Ethernet MX settings

Mode: RMII

#### Parameter settings
Tx Descriptor Length: 4

First Tx Descriptor Address: 0x300001e0

Rx Descriptor Length: 20

First Rx Descriptor Address: 0x30000000

Rx Buffers Address: 0x24008000

Rx Buffers Length: 1536

#### NVIC settings
Ethernet global interrupt: Enabled

#### GPIO settings
All set to "Very High" Maximum output speed.

### LwIP MX settings

#### General settings
MEMP_NUM_UDP_PCB: 10

#### Key Options
MEM_SIZE (Heap Memory Size): 32*1024

LWIP_RAM_HEAP_POINTER: 0x24000000

ETH_RX_BUFFER_CNT: 20

MEMP_NUM_PBUF: 16

PBUF_POOL_SIZE: 24

PBUF_POOL_BUFSIZE: 1536

Advanced:
IP_REASSEMBLY: Disabled (caused issues with ICMP packets of large size such as ping with 1500 bytes)

#### Platform settings
Selected LAN8742A driver.

#### Checksum
CHECKSUM_BY_HARDWARE: Enabled 


### MPU MX settings
#### Base - Region 0
This disables speculative access for unused memory regions.

![MPU Region 0](DocumentationPics/20260207MPUBaseRegion0.png)

#### D2 - Region 1
This disables cache for the D2 ram in which DMA descriptors are placed.

![MPU Region 1](DocumentationPics/20260207MPUD2Region1.png)


#### D1 - Region 2
This disables cache for the beginning of D1 ram in which the LwIP heap and RX pools are placed.

![MPU Region 2](DocumentationPics/20260207MPUD1Region2.png)

### Linker script
The modifications of STM32H723VETX_FLASH.ld are as follows:

The definition of the memory areas is modified. The first 64K of D1 RAM are reserved for LwIP Heap and RX pools. The original RAM_D1 area begins at 0x24010000 instead of 0x24000000 and is shortened by 64K. 
```ld
/* Specify the memory areas */
MEMORY
{
  ITCMRAM (xrw)    : ORIGIN = 0x00000000,   LENGTH = 64K
  DTCMRAM (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH    (rx)    : ORIGIN = 0x08000000,   LENGTH = 512K 
  RAM_D1_LWIP  (xrw)    : ORIGIN = 0x24000000,   LENGTH = 64K /* USER ADDED FOR A NICE MPU ALIGNMENT */
  RAM_D1  (xrw)    : ORIGIN = 0x24010000,   LENGTH = 256K /* ORIGINAL ADDRESS WAS 0x24000000 */
  RAM_D2  (xrw)    : ORIGIN = 0x30000000,   LENGTH = 32K
  RAM_D3  (xrw)    : ORIGIN = 0x38000000,   LENGTH = 16K
}
```


Added a section for the RX pools, which are placed in the second half of the 64KB RAM_D1_LWIP area. ".Rx_PoolSection" attribute is defined in ethernetif.c.
```ld
  .rxpool_sec 0x24008000 (NOLOAD) : {
      . = ALIGN(32);
      *(.Rx_PoolSection) 
      . = ALIGN(32);
  } >RAM_D1_LWIP
```


Added a section for D2 DMA stuff.
```ld
  .d2_dma_nocache_sec (NOLOAD) : {
        . = ABSOLUTE(0x30000000);
        . = ALIGN(32);
        *(.RxDecripSection)   /* Eth Rx Descriptors */
        *(.TxDecripSection)   /* Eth Tx Descriptors */
        
        . = ALIGN(32);
        *(.spi_buffers)       /* LED Strip SPI buffers*/
        *(.uart_buffers)      /* DMX UART buffers*/
    } >RAM_D2
```