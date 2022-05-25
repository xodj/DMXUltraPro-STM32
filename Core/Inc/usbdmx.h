#ifndef __USBDMX_H
#define __USBDMX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usb.h"
#include "usb_cdc.h"
#include "DMX512.h"

void InitUSB();
void USBPoll();

#endif /* __USBDMX_H */
