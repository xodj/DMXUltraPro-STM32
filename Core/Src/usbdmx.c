#include "usbdmx.h"
#include "stm32f1xx_hal.h"

#define BMAXPACKETSIZE0     		0x8
#define WMAXPACKETSIZE     			0x40
#define USB_DATA_SZ     			0x200
#define BINTERVAL 					0x0
#define BULK_OUT_ENDPOINT_TOKEN 	0x02
#define BULK_IN_ENDPOINT_TOKEN 		0x81

typedef unsigned char uchar;
extern uchar dmxDataU0[DMX_CHANNELS_COUNT];
extern uchar dmxDataU1[DMX_CHANNELS_COUNT];
extern uchar dmxDataU2[DMX_CHANNELS_COUNT];

struct usb_config {
    struct usb_config_descriptor        config;
    struct usb_interface_descriptor     interface;
    struct usb_endpoint_descriptor		bulk_in;
    struct usb_endpoint_descriptor		bulk_out;
} __attribute__((packed));

/* Device descriptor */
static const struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = VERSION_BCD(2,0,0),
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTO_NONE,
    .bMaxPacketSize0    = BMAXPACKETSIZE0,
    .idVendor           = 0x0403,
    .idProduct          = 0x6001,
    .bcdDevice          = VERSION_BCD(6,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

/* Device configuration descriptor */
static const struct usb_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct usb_config),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED,
        .bMaxPower              = USB_CFG_POWER_MA(100),
    },
    .interface = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_VENDOR,
        .bInterfaceSubClass     = USB_CLASS_VENDOR,
        .bInterfaceProtocol     = USB_CLASS_VENDOR,
        .iInterface             = 2,
    },
    .bulk_in = {
            .bLength                = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType        = USB_DTYPE_ENDPOINT,
            .bEndpointAddress       = BULK_IN_ENDPOINT_TOKEN,
            .bmAttributes      		= USB_EPTYPE_BULK,
            .wMaxPacketSize         = WMAXPACKETSIZE,
            .bInterval        		= BINTERVAL,
    },
    .bulk_out = {
            .bLength                = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType        = USB_DTYPE_ENDPOINT,
            .bEndpointAddress       = BULK_OUT_ENDPOINT_TOKEN,
            .bmAttributes      		= USB_EPTYPE_BULK,
            .wMaxPacketSize         = WMAXPACKETSIZE,
            .bInterval        		= BINTERVAL,
    },
};

static const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("DMXKing");
static const struct usb_string_descriptor prod_desc_en  = USB_STRING_DESC("UltraDMX PRO");
static const struct usb_string_descriptor prod_sn_en  	= USB_STRING_DESC("SS128034");

static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
	&prod_sn_en,
};

usbd_device udev;
uint32_t	usb_buffer[USB_DATA_SZ];

// Message Label Codes
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

typedef enum {
	PRE_SOM = 0,
 	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_DATA_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
} rx_state_t;

#define DMXUSBPRO_MESSAGE_START (0x7E)
#define DMXUSBPRO_MESSAGE_END 	(0xE7)

#define DMX_TX_PORT_A (0)
#define DMX_TX_PORT_B (1)

static uint8_t usb_rxbuf[512];
static uint8_t usb_txbuf[32];
static uint8_t usb_txbuf_bool = 0;
static uint16_t usb_txbuf_size = 0;

uint8_t DEVICE_PARAMS[] = {0, 1, 9, 1, 40};
uint8_t DEVICE_SERIAL[] = {0x01, 0x00, 0x00, 0x00};
char	DEVICE_PROVIDER[] 	= {0x6B, 0x6A, 'D', 'M', 'X', 'K', 'i', 'n', 'g'};
char	DEVICE_NAME[] 		= {0x02, 0x00, 'U', 'l', 't', 'r', 'a', 'D', 'M', 'X', ' ', 'P', 'r', 'o'};
uint8_t DEVICE_ID[] = {1, 0};


static rx_state_t	rx_state = PRE_SOM;
static uint16_t 	rx_data_offset = 0;

static void usb_send (uint8_t label, uint8_t *data, uint16_t size)
{
	usb_txbuf[0] = 0x01;
	usb_txbuf[1] = 0x60;
	usb_txbuf[2] = DMXUSBPRO_MESSAGE_START;
	usb_txbuf[3] = label;
	usb_txbuf[4] = (size & 0xFF);
	usb_txbuf[5] = (size >> 8);
	memcpy (&usb_txbuf[6], data, size);
	usb_txbuf[6 + size] = DMXUSBPRO_MESSAGE_END;
	usb_txbuf_bool = 1;
	usb_txbuf_size = size + 7;
}

static void dmessage_handler (uint8_t label, uint8_t *buf, uint16_t size)
{
	if(label == LABEL_SERIAL){
		usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		return;
	}
	switch (label)
	{
		case LABEL_PARAMS:
			usb_send(LABEL_PARAMS, 	DEVICE_PARAMS, sizeof(DEVICE_PARAMS));
		break;
		case LABEL_SERIAL:
			usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		break;

		case LABEL_VENDOR:
			usb_send(LABEL_VENDOR, 	(uint8_t*)DEVICE_PROVIDER, sizeof(DEVICE_PROVIDER));
		break;

		case LABEL_NAME:
			usb_send(LABEL_NAME, 	(uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
		break;

		case LABEL_DMXDATA:
		case LABEL_UNIVERSE_0:
		{
		    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		}
		break;
		case LABEL_UNIVERSE_1:
		{
		    HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		}
		break;

		case 13://0x0d
		break;
		case 203://0xcb
		break;

		default:
		break;
	}
}

static void usb_rx_handler (uint8_t *buf, uint32_t *size)
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
				(label == LABEL_UNIVERSE_0 ? dmxDataU0 : dmxDataU1) [rx_data_offset] = data;
				/*if(label == LABEL_UNIVERSE_0)
					dmxDataU0[rx_data_offset] = data;
				else
					dmxDataU1[rx_data_offset] = data;*/
				//usb_rxbuf[rx_data_offset] = data;
				rx_data_offset++;
				if (rx_data_offset == expected_size)
					{rx_state = WAITING_FOR_EOM;}
				break;
			case WAITING_FOR_EOM:
				if (data == DMXUSBPRO_MESSAGE_END)
					{
					dmessage_handler(label, usb_rxbuf, rx_data_offset);
					rx_state = PRE_SOM;
					}
				break;
			}
		}
}

static usbd_respond usb_getdesc (usbd_ctlreq *req, void **address, uint16_t *length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc;
    uint16_t len = 0;
    switch (dtype) {
    case USB_DTYPE_DEVICE:
        desc = &device_desc;
        break;
    case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
    case USB_DTYPE_STRING:
        if (dnumber < 4)
            desc = dtable[dnumber];
        else
            return usbd_fail;
        break;
    case USB_DTYPE_QUALIFIER:
    	return usbd_ack;
    default:
        return usbd_fail;
    }
    if (len == 0) {
        len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
}

static uint16_t ftdiStatus = 0x0000;

static usbd_respond usb_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
	switch (req->bRequest){
    case USB_STD_GET_DESCRIPTOR://0x06
        return usbd_fail;
    case USB_STD_SET_ADDRESS://0x05 or FT GetModemStat (5)
    	if(req->bmRequestType == 0xc0){
    		ftdiStatus = 0x6031;
	        dev->status.data_ptr = &ftdiStatus;
	        dev->status.data_count = sizeof(ftdiStatus);
            return usbd_ack;
    	} else
        	return usbd_fail;
    case USB_STD_GET_STATUS://0x00 or FT Reset (0)
    	if(req->bmRequestType == 0x40)
            return usbd_ack;
    	else
        	return usbd_ack;
    case 0x01: //FT ModemCtrl (1)
    	if(req->bmRequestType == 0x40)
    		return usbd_ack;
    	else if(req->bmRequestType == 0x09)
    		return usbd_ack;
    	else if(req->bmRequestType == 0x02)
    		return usbd_ack;
    	else
    		return usbd_ack;
    case 0x02://FT SetFlowCtrl (2)
    	return usbd_ack;
    case 0x03://FT SetBaudRate (3)
    	return usbd_ack;
    case 0x04://FT SetData (4)
    	return usbd_ack;
    case 0x09://FT SetLatTimer (9) or USB_STD_SET_CONFIG
    	if(req->bmRequestType == 0x00)
    	    return usbd_fail; //USB_STD_SET_CONFIG
    	if(req->bmRequestType == 0x40)
    	    return usbd_ack;
    	break;
    case 0x90: //144 ftdi
    	if(req->bmRequestType == 0xc0){ //FT DRIVER CMDS
    		/*switch (req->wIndex){
    	    case 0x00: ftdiStatus = 0x4001; break;
    	    case 0x01: ftdiStatus = 0x0403; break;
    	    case 0x02: ftdiStatus = 0x6001; break;
    	    case 0x03: ftdiStatus = 0x0600; break;
    	    case 0x04: ftdiStatus = 0x3280; break;
    	    case 0x05: ftdiStatus = 0x0008; break;
    	    case 0x06: ftdiStatus = 0x0000; break;
    	    case 0x07: ftdiStatus = 0x2498; break;
    	    case 0x08: ftdiStatus = 0x18bc; break;
    	    case 0x09: ftdiStatus = 0x12d4; break;
    	    case 0x0a: ftdiStatus = 0x0000; break;
    	    case 0x0b: ftdiStatus = 0x0000; break;
    	    case 0x0c: ftdiStatus = 0x0324; break;
    	    case 0x0d: ftdiStatus = 0x0053; break;
    	    case 0x0e: ftdiStatus = 0x0069; break;
    	    case 0x0f: ftdiStatus = 0x0062; break;
    	    case 0x10: ftdiStatus = 0x0065; break;
    	    case 0x11: ftdiStatus = 0x0072; break;
    	    case 0x12: ftdiStatus = 0x0069; break;
    	    case 0x13: ftdiStatus = 0x0061; break;
    	    case 0x14: ftdiStatus = 0x006e; break;
    	    case 0x15: ftdiStatus = 0x0020; break;
    	    case 0x16: ftdiStatus = 0x004c; break;
    	    case 0x17: ftdiStatus = 0x0069; break;
    	    case 0x18: ftdiStatus = 0x0067; break;
    	    case 0x19: ftdiStatus = 0x0068; break;
    	    case 0x1a: ftdiStatus = 0x0074; break;
    	    case 0x1b: ftdiStatus = 0x0069; break;
    	    case 0x1c: ftdiStatus = 0x006e; break;
    	    case 0x1d: ftdiStatus = 0x0067; break;
    	    case 0x1e: ftdiStatus = 0x0318; break;
    	    case 0x1f: ftdiStatus = 0x0044; break;
    	    case 0x20: ftdiStatus = 0x004d; break;
    	    case 0x21: ftdiStatus = 0x0058; break;
    	    case 0x22: ftdiStatus = 0x0020; break;
    	    case 0x23: ftdiStatus = 0x0055; break;
    	    case 0x24: ftdiStatus = 0x0053; break;
    	    case 0x25: ftdiStatus = 0x0042; break;
    	    case 0x26: ftdiStatus = 0x0020; break;
    	    case 0x27: ftdiStatus = 0x0050; break;
    	    case 0x28: ftdiStatus = 0x0052; break;
    	    case 0x29: ftdiStatus = 0x004f; break;
    	    case 0x2a: ftdiStatus = 0x0312; break;
    	    case 0x2b: ftdiStatus = 0x0053; break;
    	    case 0x2c: ftdiStatus = 0x004c; break;
    	    case 0x2d: ftdiStatus = 0x0034; break;
    	    case 0x2e: ftdiStatus = 0x0034; break;
    	    case 0x2f: ftdiStatus = 0x0039; break;
    	    case 0x30: ftdiStatus = 0x0035; break;
    	    case 0x31: ftdiStatus = 0x0038; break;
    	    case 0x32: ftdiStatus = 0x0039; break;
    	    case 0x33: ftdiStatus = 0x0000; break;
    	    case 0x34: ftdiStatus = 0x0000; break;
    	    case 0x35: ftdiStatus = 0x0000; break;
    	    case 0x36: ftdiStatus = 0x0000; break;
    	    case 0x37: ftdiStatus = 0x0000; break;
    	    case 0x38: ftdiStatus = 0x0000; break;
    	    case 0x39: ftdiStatus = 0x0000; break;
    	    case 0x3a: ftdiStatus = 0x0000; break;
    	    case 0x3b: ftdiStatus = 0x0000; break;
    	    case 0x3c: ftdiStatus = 0x0000; break;
    	    case 0x3d: ftdiStatus = 0x0000; break;
    	    case 0x3e: ftdiStatus = 0x0000; break;
    	    case 0x3f: ftdiStatus = 0x091f; break;
    	    default:   ftdiStatus = 0x0000; break;
    		}*/
    		ftdiStatus = 0x0000;
	        dev->status.data_ptr = &ftdiStatus;
	        dev->status.data_count = sizeof(ftdiStatus);
	        return usbd_ack;
    	} else
    		break;
    default: break;
    }
    return usbd_ack;
}

static uint8_t INTERFACE_A_RX[] = {0x01, 0x60};

static void usb_bulk_in(usbd_device *dev, uint8_t event, uint8_t ep) {
	if(usb_txbuf_bool){
		usbd_ep_write(dev, BULK_IN_ENDPOINT_TOKEN, &usb_txbuf, usb_txbuf_size);
		usb_txbuf_size = 0;
		usb_txbuf_bool = 0;
	} else {
		usbd_ep_write(dev, BULK_IN_ENDPOINT_TOKEN, &INTERFACE_A_RX, 2);
	}
}

static uint8_t bulk_out_buf[WMAXPACKETSIZE];

static void usb_bulk_out(usbd_device *dev, uint8_t event, uint8_t ep) {
	usbd_ep_stall(dev, BULK_IN_ENDPOINT_TOKEN);
	uint32_t bsize = usbd_ep_read(dev, BULK_OUT_ENDPOINT_TOKEN, bulk_out_buf, WMAXPACKETSIZE);
    if(bsize > 0) {
    	usb_rx_handler(bulk_out_buf, &bsize);
    }
    usbd_ep_unstall(dev, BULK_IN_ENDPOINT_TOKEN);
}

static usbd_respond usb_setconf (usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        // deconfiguring device
        usbd_ep_deconfig(dev, BULK_OUT_ENDPOINT_TOKEN);
        usbd_ep_deconfig(dev, BULK_IN_ENDPOINT_TOKEN);
        usbd_reg_endpoint(dev, BULK_OUT_ENDPOINT_TOKEN, 0);
        usbd_reg_endpoint(dev, BULK_IN_ENDPOINT_TOKEN, 0);
        return usbd_ack;
    case 1:
        // configuring device
        usbd_ep_config(dev, BULK_OUT_ENDPOINT_TOKEN, USB_EPTYPE_BULK, WMAXPACKETSIZE);
        usbd_ep_config(dev, BULK_IN_ENDPOINT_TOKEN, USB_EPTYPE_BULK, WMAXPACKETSIZE);
        usbd_reg_endpoint(dev, BULK_OUT_ENDPOINT_TOKEN, usb_bulk_out);
        usbd_reg_endpoint(dev, BULK_IN_ENDPOINT_TOKEN, usb_bulk_in);
        usbd_ep_write(dev, BULK_IN_ENDPOINT_TOKEN, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
	return usbd_ack;
}

#define USB_HANDLER     USB_LP_CAN1_RX0_IRQHandler
#define USB_NVIC_IRQ    USB_LP_CAN1_RX0_IRQn

void USB_HANDLER(void) {
    usbd_poll(&udev);
}

void InitUSB(){
	//init
    usbd_init(&udev, &usbd_hw, BMAXPACKETSIZE0, usb_buffer, USB_DATA_SZ);
    usbd_reg_config(&udev, usb_setconf);
    usbd_reg_control(&udev, usb_control);
    usbd_reg_descr(&udev, usb_getdesc);

	//connect
    NVIC_EnableIRQ(USB_NVIC_IRQ);
    usbd_enable(&udev, true);
    usbd_connect(&udev, true);
}

void USBPoll(){
    __WFI();
}
