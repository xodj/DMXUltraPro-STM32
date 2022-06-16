#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "tusb.h"
#include "enttecdmx.h"
//#include "stm32f1xx_hal_iwdg.h"

#define FTDI_TX_INTERVAL_MS 4294967295
#define FTDI_DRIVER_NUMBER 0

//IWDG_HandleTypeDef hiwdg;
bool iwdg_initiated = false;

static uint8_t usb_ftdi_tx_buf[CFG_TUD_VENDOR_TX_BUFSIZE];
static uint16_t usb_ftdi_tx_buf_size = 0;

static bool usb_ftdi_tx_polling = false;
static const uint8_t FTDI_TX_START_MESSAGE[] = {0x01, 0x60};
static uint32_t tx_interval_ms = FTDI_TX_INTERVAL_MS;

static uint8_t usb_ftdi_rx_buf[CFG_TUD_VENDOR_RX_BUFSIZE];

void led_blinking_task(void);
void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_);

extern uint32_t board_millis(void);

void USBWakeUp_IRQHandler(void){
	tud_int_handler(0);
}

void usb_ftdi_init(void)
{
	/* Need to put your interrupts here */
#if CFG_TUSB_MCU  == OPT_MCU_STM32F1
	__HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(USBWakeUp_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USBWakeUp_IRQn);
#endif
	tusb_init();
  /*hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 512;*/
}

void usb_ftdi_poll(void)
{
  tud_task();
  if (usb_ftdi_tx_polling)
  {
    static uint32_t tx_interval_start_ms = 0;
    uint32_t current_time = ((((uint64_t)xTaskGetTickCount()) * 1000) / configTICK_RATE_HZ);
    if (current_time - tx_interval_start_ms > tx_interval_ms)
    {
      if (usb_ftdi_tx_buf_size > 0)
      {
        tud_vendor_n_write(FTDI_DRIVER_NUMBER, &usb_ftdi_tx_buf, usb_ftdi_tx_buf_size);
        usb_ftdi_tx_buf_size = 0;
      }
      else
        tud_vendor_n_write(FTDI_DRIVER_NUMBER, &FTDI_TX_START_MESSAGE, 2);
      tud_vendor_n_flush(FTDI_DRIVER_NUMBER);
      tx_interval_start_ms = current_time;
    }
  }
}

void ftdi_tx_buf(uint8_t usb_txbuf_[], uint16_t usb_txbuf_size_)
{
  if (usb_txbuf_size_ > CFG_TUD_VENDOR_TX_BUFSIZE)
    usb_ftdi_tx_buf_size = CFG_TUD_VENDOR_TX_BUFSIZE;
  else
    usb_ftdi_tx_buf_size = usb_txbuf_size_;
  memcpy(usb_ftdi_tx_buf, usb_txbuf_, usb_ftdi_tx_buf_size);
}

/*
 * Device callbacks
 */

// Invoked when ftdi interface received data from host
void tud_vendor_rx_cb(uint8_t itf)
{
  /*if(iwdg_initiated){
    HAL_IWDG_Refresh(&hiwdg);
  } else {
    HAL_IWDG_Init(&hiwdg);
    iwdg_initiated = true;
  }*/
  uint32_t num_read = tud_vendor_n_read(itf, &usb_ftdi_rx_buf, CFG_TUD_VENDOR_RX_BUFSIZE);
  if (num_read > 0)
  {
    usb_rx_handler(usb_ftdi_rx_buf, &num_read);
  }
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
  usb_ftdi_tx_polling = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  usb_ftdi_tx_polling = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  usb_ftdi_tx_polling = false;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  usb_ftdi_tx_polling = true;
}

static uint16_t FTDI_DESCRIPTOR[128] = {
    /* 0000 */ 0x4000, 0x0403, 0x6001, 0x0600, 0x3280, 0x0008, 0x0200, 0x1098,
    /* 0008 */ 0x1CA8, 0x12C4, 0x1023, 0x0005, 0x0310, 0x0044, 0x004D, 0x0058,
    /* 0010 */ 0x004C, 0x0049, 0x0046, 0x0045, 0x031C, 0x0044, 0x004D, 0x0058,
    /* 0018 */ 0x0020, 0x0055, 0x006C, 0x0074, 0x0072, 0x0061, 0x0020, 0x0050,
    /* 0020 */ 0x0072, 0x006F, 0x0312, 0x0044, 0x004C, 0x0037, 0x0032, 0x0058,
    /* 0028 */ 0x0037, 0x0050, 0x0032, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0030 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0038 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7FFC,
    /* 0040 */ 0x041A, 0xFBE5, 0x0000, 0xD23B, 0x50B0, 0x0042, 0x0000, 0x0000,
    /* 0048 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x4135, 0x5754, 0x4334, 0x3655,
    /* 0050 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0058 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0060 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0068 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0070 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* 0078 */ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*
 *
 * 0000: 4000 0403 6001 0600 3280 0008 0200 1098   @...`...2.......
 * 0008: 1CA8 12C4 1023 0005 0310 0044 004D 0058   .....#.....D.M.X
 * 0010: 004C 0049 0046 0045 031C 0044 004D 0058   .L.I.F.E...D.M.X
 * 0018: 0020 0055 006C 0074 0072 0061 0020 0050   . .U.l.t.r.a. .P
 * 0020: 0072 006F 0312 0044 004C 0037 0032 0058   .r.o...D.L.7.2.X
 * 0028: 0037 0050 0032 0000 0000 0000 0000 0000   .7.P.2..........
 * 0030: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0038: 0000 0000 0000 0000 0000 0000 0000 7FFC   ................
 * 0040: 041A FBE5 0000 D23B 50B0 0042 0000 0000   .......;P..B....
 * 0048: 0000 0000 0000 0000 4135 5754 4334 3655   ........A5WTC46U
 * 0050: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0058: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0060: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0068: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0070: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 * 0078: 0000 0000 0000 0000 0000 0000 0000 0000   ................
 *
 */

// Control message callback
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  (void)stage;
  switch (request->bRequest)
  {
  case 0x00: // FT Reset (0)
    if (request->bmRequestType == 0x40)
    {
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x01: // FT ModemCtrl (1)
    if (request->bmRequestType == 0x40)
    {
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x02: // FT SetFlowCtrl (2)
    if (request->bmRequestType == 0x40)
    {
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x03: // FT SetBaudRate (3)
    if (request->bmRequestType == 0x40)
    {
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x04: // FT SetData (4)
    if (request->bmRequestType == 0x40)
    {
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x05: // FT GetModemStat (5)
    if (request->bmRequestType == 0xc0)
    {
      uint8_t ftdiStatus[] = {0x01, 0x60};
      tud_control_xfer(rhport, request, (void *)&ftdiStatus, 2);
      return true;
    }
    break;
  case 0x09: // FT SetLatTimer (9)
    if (request->bmRequestType == 0x40)
    {
      tx_interval_ms = (uint32_t)(request->wValue);
      tud_control_status(rhport, request);
      return true;
    }
    break;
  case 0x0a: // FT GetLatTimer (10)
    if (request->bmRequestType == 0xc0)
    {
      int8_t ftdiStatus = 0x10;
      tud_control_xfer(rhport, request, (void *)&ftdiStatus, 1);
      return true;
    }
    break;
  case 0x90: // FT Request (144)
    if (request->bmRequestType == 0xc0)
    {
      int wIndex = (int)request->wIndex;
      uint16_t ftdiStatus = 0x0000;
      if (wIndex >= 0 && wIndex < 128)
        ftdiStatus = FTDI_DESCRIPTOR[wIndex];
      tud_control_xfer(rhport, request, (void *)&ftdiStatus, 2);
      return true;
    }
    break;
  default:
    break;
  }
  return false;
}
