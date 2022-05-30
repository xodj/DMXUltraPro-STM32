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
	//usbd_ep_stall(dev, BULK_IN_ENDPOINT_TOKEN);
	uint32_t bsize = usbd_ep_read(dev, BULK_OUT_ENDPOINT_TOKEN, usb_rxbuf, WMAXPACKETSIZE);
    if(bsize > 0) {
    	usb_rx_handler(usb_rxbuf, &bsize);
    }
    //usbd_ep_unstall(dev, BULK_IN_ENDPOINT_TOKEN);
}

static uint16_t ftdiStatus = 0x0000;
static uint8_t ftdiStatusSize = 0x02;
uint16_t bwIndex[128];
uint8_t iwIndex = 0x00;

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
	        dev->status.data_count = 0x0001;
    	    return usbd_ack;
    	} else
    	    return usbd_ack;
    	break;
    case 0x90: //144 ftdi
    	if(req->bmRequestType == 0xc0){ //FT DRIVER CMDS
    		//usbd_ep_stall(dev, BULK_IN_ENDPOINT_TOKEN);
    		int wIndex = (int)req->wIndex;
    		bwIndex[iwIndex] = req->wIndex;
    		iwIndex++;
    		if(FTDI_DESCRIPTOR[wIndex]){
    			ftdiStatus = FTDI_DESCRIPTOR[wIndex];
    	        dev->status.data_ptr = &ftdiStatus;
    	        dev->status.data_count = ftdiStatusSize;
    		} else {
        		ftdiStatus = 0x0000;
    	        dev->status.data_ptr = &ftdiStatus;
    	        dev->status.data_count = ftdiStatusSize;
    		}
    		if(iwIndex == 0x40)
    	        return usbd_ack;
	        return usbd_ack;
    	} else
    		break;
    default: break;
    }
    return usbd_ack;
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
