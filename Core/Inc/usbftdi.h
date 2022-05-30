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

static uint16_t FTDI_DESCRIPTOR[] = {
		/*0000*/ 0x4000, 0x0403, 0x6001, 0x0600, 0x3280, 0x0008, 0x0200, 0x1098,
		/*0008*/ 0x20A8, 0x12C8, 0x0000, 0x0000, 0x0310, 0x0044, 0x004D, 0x0058,
		/*0010*/ 0x004C, 0x0049, 0x0046, 0x0045, 0x0320, 0x0044, 0x004D, 0x0058,
		/*0018*/ 0x0020, 0x0055, 0x0053, 0x0042, 0x0020, 0x0050, 0x0052, 0x004F,
		/*0020*/ 0x0020, 0x004D, 0x004B, 0x0032, 0x0312, 0x0053, 0x0053, 0x0031,
		/*0028*/ 0x0032, 0x0038, 0x0030, 0x0033, 0x0034, 0x0000, 0x0000, 0x0000,
		/*0030*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		/*0038*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8FF8,
		/*0040*/ 0x0204, 0x0109, 0x0728, 0x6a01, 0x406b, 0x9544, 0x0589, 0x0402,
		/*0048*/ 0x0000, 0x0000, 0x0256, 0xffff, 0x0032, 0x00b0, 0x000c, 0x005a,
		/*0050*/ 0x0200, 0x413c, 0xe0a4, 0x0000, 0x0d20, 0x2000, 0x0d88, 0x2000,
		/*0058*/ 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		/*0060*/ 0x9a9b, 0x0800, 0x9b13, 0x0800, 0x9b99, 0x0800, 0x9bcd, 0x0800,
		/*0068*/ 0x0000, 0x0100, 0x0002, 0x0000, 0x0000, 0x0000, 0x9c0b, 0x0800,
		/*0070*/ 0x9c25, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		/*0078*/ 0x0000, 0x0100, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};
/*
 * 0000: 4000 0403 6001 0600 3280 0008 0200 1098   @...`...2.......
 * 0008: 20A8 12C8 0000 0000 0310 0044 004D 0058    ..........D.M.X
 * 0010: 004C 0049 0046 0045 0320 0044 004D 0058   .L.I.F.E. .D.M.X
 * 0018: 0020 0055 0053 0042 0020 0050 0052 004F   . .U.S.B. .P.R.O
 * 0020: 0020 004D 004B 0032 0312 0053 0053 0031   . .M.K.2...S.S.1
 * 0028: 0032 0038 0030 0033 0034 0000 0000 0000   .2.8.0.3.4......
 * 0030: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0038: 0000 0000 0000 0000 0000 0000 0000 8FF8   ................
 * 0040: 0204 0109 0728 6A01 406B 9544 0589 0402   .....(j.@k.D....
 * 0048: 0000 0000 0256 FFFF 0032 00B0 000C 005A   .....V...2.....Z
 * 0050: 0200 413C E0A4 0000 0D20 2000 0D88 2000   ..A<.....  ... .
 * 0058: 0000 0000 0001 0000 0000 0000 0000 0000   ................
 * 0060: 9A9B 0800 9B13 0800 9B99 0800 9BCD 0800   ................
 * 0068: 0000 0100 0002 0000 0000 0000 9C0B 0800   ................
 * 0070: 9C25 0800 0000 0000 0000 0000 0000 0000   .%..............
 * 0078: 0000 0100 0000 0000 0000 0000 0000 0000   ................
 */

void InitUSB();
void USBPoll();
void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_);

#endif /* __USBFTDI_H */
