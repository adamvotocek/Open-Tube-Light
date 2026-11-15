/**
 * @file artnet.h
 * @brief Art-Net 4 node - public API
 *
 * The Art-Net node (UDP 0x1936: ArtPoll, ArtCommand, ArtAddress, ArtInput,
 * ArtDataRequest) runs in every DMX input mode. The DMX data part (ArtDmx,
 * ArtSync) is the dmx_input driver `dmx_input_artnet`, declared in
 * dmx_input.h; it is only active while Art-Net is the input source.
 *
 * @see artnet_protocol.h for Art-Net protocol structures and constants
 */

#ifndef INC_ARTNET_H_
#define INC_ARTNET_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the Art-Net node
 *
 * Creates the UDP socket on port 0x1936, the ArtPollReply timer and the DMX
 * shadow-buffer mutex. Runs once: the node is never torn down, and a second
 * call returns 0.
 *
 * @return 0 on success, -1 if the socket, timer or mutex could not be created
 *
 * @note Call from ControlTask after MX_LWIP_Init() and before
 *       DMX_Input_Start(), never from tcpip_thread: the socket is set up under
 *       the LwIP core lock (LOCK_TCPIP_CORE).
 * @note DeviceConfig_Init() must have run (the MAC seeds the reply delay).
 */
int ArtNet_NodeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARTNET_H_ */
