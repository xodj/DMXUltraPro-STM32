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

// LED
#define LED_PORT              GPIOC
#define LED_PIN               GPIO_PIN_13
#define LED_STATE_ON          0

int main(void)
{
#if (CFG_TUSB_MCU==OPT_MCU_STM32F1)
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitTypeDef oscinitstruct = {0};
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.HSIState = RCC_HSI_ON;
  HAL_RCC_OscConfig(&oscinitstruct);

  /* USB clock selection */
  RCC_PeriphCLKInitTypeDef rccperiphclkinit = {0};
  rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  rccperiphclkinit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitTypeDef clkinitstruct = {0};
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2);
#else
  board_init();
#endif

  // Create a task for tinyusb device stack
  (void)xTaskCreateStatic(usb_device_task, "usbd", USBD_STACK_SIZE,
    NULL, configMAX_PRIORITIES - 1, usb_device_stack, &usb_device_taskdef);
  // Create CDC task
  (void)xTaskCreateStatic(cdc_task, "cdc", CDC_STACK_SZIE,
    NULL, configMAX_PRIORITIES - 2, cdc_stack, &cdc_taskdef);

  // skip starting scheduler (and return) for ESP32-S2 or ESP32-S3
#if !TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3)
  vTaskStartScheduler();
#endif

  while(1);
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
#if (CFG_TUSB_MCU==OPT_MCU_STM32F1)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  // LED
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = LED_STATE_ON ? GPIO_PULLDOWN : GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // USB Pins
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  osal_task_delay(1000);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
  // Configure USB DM and DP pins.
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_SetPriority(USBWakeUp_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

  // USB Clock enable
  __HAL_RCC_USB_CLK_ENABLE();

  board_led_write(false);
  #endif

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
    DMX_Send_Packet(512);
  }
}

void led_blinking_task(void)
{
  static bool led_state = false;
  board_led_write(led_state);
  led_state = 1 - led_state;
}
