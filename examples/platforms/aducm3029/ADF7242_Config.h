#ifndef _ADF7242_CONFIG_H_
#define _ADF7242_CONFIG_H_

#define FIRMWARE_ENABLE_d 

#define ADF7242_DEBUG_d

#define ADF7242_CONF_PHY_MODE           IEEE_802_15_4_packet_mode
#define ADF7242_CONF_CHANNEL_FREQ       2435000000
#ifdef FIRMWARE_ENABLE_d
#define ADF7242_CONF_PACKET_OPR_MODE    (IEEE_802_15_4_AUTO_CSMACA | IEEE_802_15_4_RX_AUTO_ACK | IEEE_802_15_4_RX_ON_WHEN_IDLE)
#else
#define ADF7242_CONF_PACKET_OPR_MODE    IEEE_802_15_4_RX_ON_WHEN_IDLE
#endif
#define ADF7242_CONF_DEFAULT_TX_POWER	3 //TODO

#define ADF7242_CONF_PAN_ID             0xABCD
#define ADF7242_CONF_MAC_SHORT_ADDR     0x5678
#define ADF7242_CONF_MAC_EXTENDED_ADDR  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}
/* Maximum number of CCA retires[6:4] and frame retries[3:0] */
#define ADF7242_CONF_MAX_CCA_RETRY      4
#define ADF7242_CONF_MAX_FRAME_RETRY    3
/* Minimum BE[7:4] and maximum BE[3:0] */
#define ADF7242_CONF_MIN_BE             3
#define ADF7242_CONF_MAX_BE             5

/* Minimum 260 bytes should be supported.*/
#define ADF7242_CONF_RX_BUFF_SIZE       260
#define ADF7242_CONF_TX_BUFF_SIZE       260


#endif /* _ADF7242_CONFIG_H_ */
