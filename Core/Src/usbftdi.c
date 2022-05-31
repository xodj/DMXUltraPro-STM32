#include <usbftdi.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include <enttecdmx.h>

static usbd_device udev;
static uint32_t	usb_buffer[USB_DATA_SZ];
static uint8_t usb_rxbuf[WMAXPACKETSIZE];

static uint8_t usb_txbuf[WMAXPACKETSIZE];
static bool usb_txbuf_bool = false;
static uint16_t usb_txbuf_size = 0;

extern void usb_rx_handler(uint8_t buf[], uint32_t *size);

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

static uint16_t ftdiStatus = 0x0000;
static uint8_t ftdiStatusSize = 0x02;

static usbd_respond usb_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
	switch (req->bRequest){
    case USB_STD_GET_DESCRIPTOR://0x06
        return usbd_fail;
    case USB_STD_SET_ADDRESS://0x05 or FT GetModemStat (5)
    	if(req->bmRequestType == 0xc0){
    		ftdiStatus = 0x6031;
	        dev->status.data_ptr = &ftdiStatus;
	        dev->status.data_count = ftdiStatusSize;
            return usbd_ack;
    	} else
        	return usbd_fail;
    case USB_STD_GET_STATUS://0x00 or FT Reset (0)
    	if(req->bmRequestType == 0x40){
            return usbd_ack;
    	} else
        	return usbd_ack;
    case 0x01: //FT ModemCtrl (1)
    	if(req->bmRequestType == 0x40
    			&&req->wValue == 512){
    		return usbd_ack;
    	} else if(req->bmRequestType == 0x09)
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
    case 0x0a://FT GetLatTimer (10)
    	if(req->bmRequestType == 0xc0){
    		ftdiStatus = 0x1010;
	        dev->status.data_ptr = &ftdiStatus;
	        dev->status.data_count = 0x0001;
    	    return usbd_ack;
    	} else
    	    return usbd_ack;
    	break;
    case 0x90: //144 ftdi
    	if(req->bmRequestType == 0xc0){ //FT DRIVER CMDS
    		int wIndex = (int)req->wIndex;
    		if(wIndex >= 0 && wIndex < 128){
    			ftdiStatus = FTDI_DESCRIPTOR[wIndex];
    	        dev->status.data_ptr = &ftdiStatus;
    	        dev->status.data_count = ftdiStatusSize;
    		} else {
        		ftdiStatus = 0x0000;
    	        dev->status.data_ptr = &ftdiStatus;
    	        dev->status.data_count = ftdiStatusSize;
    		}
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

static void usb_bulk_out(usbd_device *dev, uint8_t event, uint8_t ep) {
	uint32_t bsize = usbd_ep_read(dev, BULK_OUT_ENDPOINT_TOKEN, usb_rxbuf, WMAXPACKETSIZE);
    if(bsize > 0) {
    	usb_rx_handler(usb_rxbuf, &bsize);
    }
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

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  usbd_poll(&udev);
}

void InitUSB(){
	//init
    usbd_init(&udev, &usbd_hw, BMAXPACKETSIZE0, usb_buffer, USB_DATA_SZ);
    usbd_reg_config(&udev, usb_setconf);
    usbd_reg_control(&udev, usb_control);
    usbd_reg_descr(&udev, usb_getdesc);

	//connect
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    usbd_enable(&udev, true);
    usbd_connect(&udev, true);
}

void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_){
	if(usb_txbuf_size_ > WMAXPACKETSIZE)
		usb_txbuf_size = WMAXPACKETSIZE;
	else
		usb_txbuf_size = usb_txbuf_size_;
	memcpy(usb_txbuf, usb_txbuf_, usb_txbuf_size);
	usb_txbuf_bool = 1;
}
