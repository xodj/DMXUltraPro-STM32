#include <enttecdmx.h>
#include <usbftdi.h>
#include <string.h>
#include <main.h>
#include <dmx512.h>

extern uchar dmxDataU0[DMX_CHANNELS_COUNT];
extern uchar dmxDataU1[DMX_CHANNELS_COUNT];
extern uchar dmxDataU2[DMX_CHANNELS_COUNT];

extern void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_);

static void usb_send(uint8_t label, uint8_t *data, uint16_t size)
{
	uint8_t usb_txbuf[size + 7];
	usb_txbuf[0] = 0x01;
	usb_txbuf[1] = 0x60;
	usb_txbuf[2] = DMXUSBPRO_MESSAGE_START;
	usb_txbuf[3] = label;
	usb_txbuf[4] = (size & 0xFF);
	usb_txbuf[5] = (size >> 8);
	memcpy (&usb_txbuf[6], data, size);
	usb_txbuf[6 + size] = DMXUSBPRO_MESSAGE_END;
	ftdi_tx_buf(usb_txbuf, size + 7);
}

static void dmessage_handler(uint8_t label)
{
	if(label == LABEL_SERIAL){//10
		usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		return;
	}
	if(label == LABEL_DMXDATA ||
			label == LABEL_UNIVERSE_0 ||
			label == LABEL_UNIVERSE_1){//6
	    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		return;
	}
	switch (label)
	{
		case LABEL_PARAMS://3
			usb_send(LABEL_PARAMS, 	DEVICE_PARAMS, sizeof(DEVICE_PARAMS));
		break;
		case LABEL_SERIAL:
			usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		break;

		case LABEL_VENDOR://77
			usb_send(LABEL_VENDOR, (uint8_t*)DEVICE_PROVIDER, sizeof(DEVICE_PROVIDER));
		break;

		case LABEL_NAME://78
			usb_send(LABEL_NAME, (uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
		break;

		case LABEL_DMXDATA://6
		{
		    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		}
		break;
		case LABEL_UNIVERSE_0://100
		{
		    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		}
		break;
		case LABEL_UNIVERSE_1://101
		{
		    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		}
		break;

		case 13://0x0d ENTTEC_PRO_ENABLE_API2 ignore
			usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		break;
		case 203://0xcb ENTTEC_PRO_PORT_ASS_REQ
			usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		break;

		default:
		break;
	}
}

void usb_rx_handler(uint8_t buf[], uint32_t *size)
{
	uint32_t cnt = 0;
	uint8_t data = 0;
	static uint8_t 		label = 0;
	static uint16_t 	expected_size = 0;

	for (cnt=0; cnt<*size; cnt++)
	{
		data = buf[cnt];
		switch (rx_state)
		{
			case PRE_SOM:
				if (data == DMXUSBPRO_MESSAGE_START)
					{rx_state = GOT_SOM;}
				break;
			case GOT_SOM:
				label = data;
				rx_state = GOT_LABEL;
				break;
			case GOT_LABEL:
				rx_data_offset = 0;
				expected_size = data;
				rx_state = GOT_DATA_LSB;
				break;
			case GOT_DATA_LSB:
				expected_size += (data << 8);
				if (expected_size == 0)	{rx_state = WAITING_FOR_EOM;}
				else {
					rx_state = IN_DATA;
					cnt++;
					if(expected_size > 512)
						expected_size = 512;
				}
				break;
			case IN_DATA:
				(label == LABEL_UNIVERSE_0 || label == LABEL_DMXDATA
						? dmxDataU0 : dmxDataU1) [rx_data_offset] = data;
				/*if(label == LABEL_UNIVERSE_0)
					dmxDataU0[rx_data_offset] = data;
				else
					dmxDataU1[rx_data_offset] = data;*/
				rx_data_offset++;
				if (rx_data_offset == expected_size)
					{rx_state = WAITING_FOR_EOM;}
				break;
			case WAITING_FOR_EOM:
				if (data == DMXUSBPRO_MESSAGE_END)
					{
					dmessage_handler(label);
					rx_state = PRE_SOM;
					}
				break;
			}
		}
}
