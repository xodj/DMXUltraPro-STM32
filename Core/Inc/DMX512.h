#ifndef __DMX_H
#define __DMX_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define DMX_CHANNELS_COUNT 512

#define DMX_UART_U0 USART1
#define DMX_UART_U1 USART3
#define DMX_TX_PIN_U0 GPIO_PIN_9
#define DMX_TX_PIN_U1 GPIO_PIN_10
#define DMX_TX_GPIO_PORT_U0 GPIOA
#define DMX_TX_GPIO_PORT_U1 GPIOB
#define DMX_TX_HIGH_U0  HAL_GPIO_WritePin(DMX_TX_GPIO_PORT_U0,DMX_TX_PIN_U0,GPIO_PIN_SET)
#define DMX_TX_HIGH_U1  HAL_GPIO_WritePin(DMX_TX_GPIO_PORT_U1,DMX_TX_PIN_U1,GPIO_PIN_SET)
#define DMX_TX_LOW_U0  HAL_GPIO_WritePin(DMX_TX_GPIO_PORT_U0,DMX_TX_PIN_U0,GPIO_PIN_RESET)
#define DMX_TX_LOW_U1  HAL_GPIO_WritePin(DMX_TX_GPIO_PORT_U1,DMX_TX_PIN_U1,GPIO_PIN_RESET)

void DMX_Reset();
void clrDmxData(void);
void GPIO_Tx_Config_OUT(void);
void GPIO_Tx_Config_AF(void);
void DMX_Break();
void DMX_Send_9Data(uint16_t i);
void DMX_Send_Packet(uint16_t tempnum);
void DMX_Init();

void DMX_Demo();

#endif
