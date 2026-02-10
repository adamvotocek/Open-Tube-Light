# 1. Art-Net recieving
Via LwIP Netconn. Fixed number of universes. 

# 2. LED strip control from Art-Net data
SK9822 via SPI. Very basic 8 bit color depth, no calculations for global brightness.


# X. Lay down the foundations for the existance of different modes of operation.
Input protocols: Art-Net, sACN, DMX. And their parameters.
Input modes: adjustable number of pixels; DMX color modes for each pixel: 8bit control, 16bit control, 8 bit + dimmer.

# X. Implement I2C display and 4 buttons for configuration.
Selection of input protocol and its settings (address, nets etc.), number of pixels, DMX color mode for each pixel.

# X. Fully implement DMX recieving via UART. 

# X. Fully implement sACN recieving

# X. Web interface for configuration

