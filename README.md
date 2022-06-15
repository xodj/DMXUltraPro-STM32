# DMXKing-TinyUSB
 USB<->DMX controller with TinyUSB (STM32) <br>
 Using [tinyusb](https://github.com/hathach/tinyusb) library <br>
 Clone into ~/tinyusb/examples/device/ <br>



### How to (you can replace `~` with your own directory):
- Change directory `cd ~`
- Clone tinyusb `git clone https://github.com/hathach/tinyusb.git`
- Change directory `cd ~/tinyusb/examples/device/`
- Clone DMXKing-TinyUSB `git clone https://github.com/xodj/DMXKing-TinyUSB.git`
- Patch `~/tinyusb/src/device/usbd.c` add to function `static bool process_control_request(uint8_t rhport, tusb_control_request_t const * p_request)` into start:<br>
`#ifdef CONTROLL_CB_OVERRIDE`  <br>
  `TU_VERIFY(tud_vendor_control_xfer_cb);` <br>
  `if (tud_vendor_control_xfer_cb(rhport, CONTROL_STAGE_SETUP, p_request))` <br>
    `return true;` <br>
`#endif` <br>
- Change directory `cd ~/tinyusb/examples/device/DMXKing-TinyUSB`
- Build and flash library `make BOARD=stm32f103_bluepill flash-stlink` (for example bluepill board, see [supported](https://github.com/hathach/tinyusb/blob/master/docs/reference/supported.rst))
