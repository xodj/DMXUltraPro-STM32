#ifndef __USBFTDI_H
#define __USBFTDI_H

#include <stdint.h>

//defines
#define DMXUSBPRO_MESSAGE_START (0x7E)
#define DMXUSBPRO_MESSAGE_END 	(0xE7)

//type defines
typedef unsigned char uchar;

typedef enum {
	PRE_SOM = 0,
 	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_DATA_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
} rx_state_t;
enum {
	LABEL_PARAMS 		= 3,
	LABEL_RECEIVED_DMX 	= 5,
	LABEL_DMXDATA 		= 6,
	LABEL_SERIAL 		= 10,
	LABEL_VENDOR 		= 77,
	LABEL_NAME 			= 78,
	LABEL_RDM 			= 82,
	LABEL_UNIVERSE_0	= 100,
	LABEL_UNIVERSE_1	= 101
};

//variables
static uint8_t DEVICE_PARAMS[] = {0, 1, 9, 1, 40};
static uint8_t DEVICE_SERIAL[] = {0x89, 0x95, 0x44, 0x40};
static char	DEVICE_PROVIDER[] 	= {0x6B, 0x6A, 'D', 'M', 'X', 'L', 'I', 'F', 'E'};
static char	DEVICE_NAME[] 		= {0x02, 0x00, 'D', 'M', 'X', ' ','U', 'S', 'B', ' ', 'P', 'R', 'O', ' ', 'M', 'K', '2'};
static rx_state_t	rx_state = PRE_SOM;
static uint16_t 	rx_data_offset = 0;

void usb_rx_handler(uint8_t buf[], uint32_t *size);

#endif /* __USBFTDI_H */
