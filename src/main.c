#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "bsp/board.h"
#include "usbftdi.h"
#include "dmx512.h"
#if TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)
// ESP-IDF need "freertos/" prefix in include path.
// CFG_TUSB_OS_INC_PATH should be defined accordingly.
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define USBD_STACK_SIZE 4096
#else
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

// Increase stack size when debug log is enabled
#define USBD_STACK_SIZE (3 * configMINIMAL_STACK_SIZE / 2) * (CFG_TUSB_DEBUG ? 2 : 1)
#endif

// static task
StackType_t usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

// static task for cdc
#define CDC_STACK_SZIE configMINIMAL_STACK_SIZE
StackType_t cdc_stack[CDC_STACK_SZIE];
StaticTask_t cdc_taskdef;

void usb_device_task(void *param);
void cdc_task(void *params);

int main(void)
{
  board_init();
  board_led_write(false);

  // Create a task for tinyusb device stack
  (void)xTaskCreateStatic(usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, usb_device_stack, &usb_device_taskdef);

  // Create CDC task
  (void)xTaskCreateStatic(cdc_task, "cdc", CDC_STACK_SZIE, NULL, configMAX_PRIORITIES - 2, cdc_stack, &cdc_taskdef);

  // skip starting scheduler (and return) for ESP32-S2 or ESP32-S3
#if !TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)
  vTaskStartScheduler();
#endif

  while (1)
  {
  }

  return 0;
}

#if TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)
void app_main(void)
{
  main();
}
#endif

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void *param)
{
  (void)param;
  usb_ftdi_init();
  // RTOS forever loop
  while (1)
  {
    usb_ftdi_poll();
  }
}
void cdc_task(void *params)
{
  (void)params;
  DMX_Init();
  // RTOS forever loop
  while (1)
  {
    DMX_Demo();
    DMX_Send_Packet(512);
  }
}

void led_blinking_task(void)
{
  static bool led_state = false;
  board_led_write(led_state);
  led_state = 1 - led_state;
}
