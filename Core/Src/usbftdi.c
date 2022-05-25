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
    	if(req->bmRequestType == 0x40
    			&&req->wValue == 512){
    		//usbd_ep_unstall(dev, BULK_IN_ENDPOINT_TOKEN);
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
	        dev->status.data_count = 1;
    	    return usbd_ack;
    	} else
    	    return usbd_ack;
    	break;
    case 0x90: //144 ftdi
    	if(req->bmRequestType == 0xc0){ //FT DRIVER CMDS
    		//usbd_ep_stall(dev, BULK_IN_ENDPOINT_TOKEN);
    		ftdiStatus = 0x0000;
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
    	    default: break;
    		}*/
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
	//usbd_ep_stall(dev, BULK_OUT_ENDPOINT_TOKEN);
	if(usb_txbuf_bool){
		usbd_ep_write(dev, BULK_IN_ENDPOINT_TOKEN, &usb_txbuf, usb_txbuf_size);
		usb_txbuf_size = 0;
		usb_txbuf_bool = 0;
	} else {
		usbd_ep_write(dev, BULK_IN_ENDPOINT_TOKEN, &INTERFACE_A_RX, 2);
	}
    //usbd_ep_unstall(dev, BULK_OUT_ENDPOINT_TOKEN);
}

static void usb_bulk_out(usbd_device *dev, uint8_t event, uint8_t ep) {
	usbd_ep_stall(dev, BULK_IN_ENDPOINT_TOKEN);
	uint32_t bsize = usbd_ep_read(dev, BULK_OUT_ENDPOINT_TOKEN, usb_rxbuf, WMAXPACKETSIZE);
    if(bsize > 0) {
    	usb_rx_handler(usb_rxbuf, &bsize);
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

/*#define USB_HANDLER     USB_LP_CAN1_RX0_IRQHandler
#define USB_NVIC_IRQ    USB_LP_CAN1_RX0_IRQn

void USB_HANDLER(void) {
    usbd_poll(&udev);
}*/

void InitUSB(){
	//init
    usbd_init(&udev, &usbd_hw, BMAXPACKETSIZE0, usb_buffer, USB_DATA_SZ);
    usbd_reg_config(&udev, usb_setconf);
    usbd_reg_control(&udev, usb_control);
    usbd_reg_descr(&udev, usb_getdesc);

	//connect
    //NVIC_EnableIRQ(USB_NVIC_IRQ);
    usbd_enable(&udev, true);
    usbd_connect(&udev, true);
}

void USBPoll(){
    //__WFI();
    usbd_poll(&udev);
}

void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_){
	if(usb_txbuf_size_ > WMAXPACKETSIZE)
		usb_txbuf_size = WMAXPACKETSIZE;
	else
		usb_txbuf_size = usb_txbuf_size_;
	memcpy(usb_txbuf, usb_txbuf_, usb_txbuf_size);
	usb_txbuf_bool = 1;
}
