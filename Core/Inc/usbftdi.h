#ifndef __USBFTDI_H
#define __USBFTDI_H

#include <stdint.h>
#include "usb.h"
#include "usb_cdc.h"

//defines
#define BMAXPACKETSIZE0     			0x8
#define WMAXPACKETSIZE     				0x40
#define USB_DATA_SZ     				0x200
#define BINTERVAL 						0x0
#define BULK_OUT_ENDPOINT_TOKEN 		0x02
#define BULK_IN_ENDPOINT_TOKEN 			0x81
#define MANUFACTURER_STRING 			"DMXLIFE"
#define PRODUCT_STRING					"DMX USB PRO MK2"
#define PRODUCT_SERIAL_NUMBER_STRING	"SS128034"

//type defines
typedef unsigned char uchar;
typedef struct {
    struct usb_config_descriptor        config;
    struct usb_interface_descriptor     interface;
    struct usb_endpoint_descriptor		bulk_in;
    struct usb_endpoint_descriptor		bulk_out;
} usb_config;

// Device descriptor
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

// Device configuration descriptor
static const usb_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(usb_config),
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

static const struct usb_string_descriptor lang_desc
	= USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en
	= USB_STRING_DESC(MANUFACTURER_STRING);
static const struct usb_string_descriptor prod_desc_en
	= USB_STRING_DESC(PRODUCT_STRING);
static const struct usb_string_descriptor prod_sn_en
	= USB_STRING_DESC(PRODUCT_SERIAL_NUMBER_STRING);

static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
	&prod_sn_en,
};

void InitUSB();
void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_);

#endif /* __USBFTDI_H */
