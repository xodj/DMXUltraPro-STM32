#include <dmx512.h>
#include "stm32f1xx_hal.h"
/**
 * @brief Blocks for usec nanoseconds by using TIM4.
 *
 * The wait is achieved by setting up the timer TIM4, followed by enabling
 * the timer, and waiting until the timer period has lapsed. After the timer
 * period has lapsed, the timer is disabled again.
 *
 * This method is implemented as a register call because it is highly
 * performance critical.
 *
 * This method is implemented as a define, to ensure that the code is inlined by
 * the compiler. This is done for performance reasons.
 *
 * GCC treats the inline keyword as an optimization hint. The compiler may still
 * ignore the keyword and not inline the function. This is avoided by using a
 * define.
 *
 * @param X The amount of microseconds to block.
 * @return None.
 */
#define APB2TimFreq 48000000
#define TIM4Freq APB2TimFreq / 1000000 - 1
#define TIM4WaitUsec(usec)         \
    TIM4->PSC = TIM4Freq;          \
    TIM4->ARR = usec;              \
    TIM4->CNT = 0;                 \
    TIM4->CR1 |= TIM_CR1_CEN;      \
    while (TIM4->CNT != TIM4->ARR) \
    {                              \
    }                              \
    TIM4->CR1 &= ~TIM_CR1_CEN

typedef unsigned char uchar;

uchar dmxDataU0[DMX_CHANNELS_COUNT];
uchar dmxDataU1[DMX_CHANNELS_COUNT];

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);

/* Clear DMX Buffer */
void clrdmxData(void)
{
    for (int i = 0; i < DMX_CHANNELS_COUNT; i++)
    {
        dmxDataU0[i] = 0;
        dmxDataU1[i] = 0;
    }
}
/* Set Tx_GPIO_Mode */
void GPIO_Tx_Config_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct_U0;
    GPIO_InitStruct_U0.Pin = DMX_TX_PIN_U0;
    GPIO_InitStruct_U0.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_U0.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitTypeDef GPIO_InitStruct_U1;
    GPIO_InitStruct_U1.Pin = DMX_TX_PIN_U1;
    GPIO_InitStruct_U1.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_U1.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DMX_TX_GPIO_PORT_U0, &GPIO_InitStruct_U0);
    HAL_GPIO_Init(DMX_TX_GPIO_PORT_U1, &GPIO_InitStruct_U1);
}

void GPIO_Tx_Config_AF(void)
{
    /*Configure GPIO pin : PtPin */
    GPIO_InitTypeDef GPIO_InitStruct_U0;
    GPIO_InitStruct_U0.Pin = DMX_TX_PIN_U0;
    GPIO_InitStruct_U0.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_U0.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitTypeDef GPIO_InitStruct_U1;
    GPIO_InitStruct_U1.Pin = DMX_TX_PIN_U1;
    GPIO_InitStruct_U1.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_U1.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DMX_TX_GPIO_PORT_U0, &GPIO_InitStruct_U0);
    HAL_GPIO_Init(DMX_TX_GPIO_PORT_U1, &GPIO_InitStruct_U1);
}
/* Send Break sign and 00 Code */
void DMX_Break(void)
{
    GPIO_Tx_Config_OUT(); // Set UART TX pin mode to OUTPUT
    DMX_TX_LOW_U0;
    DMX_TX_LOW_U1;
    TIM4WaitUsec(88); // DMX512 1990's BREAK >88us
    DMX_TX_HIGH_U0;
    DMX_TX_HIGH_U1;
    TIM4WaitUsec(8); // DMX512 1990's Mark after break MAB >8us
    GPIO_Tx_Config_AF();
    /* Send Start Code 00 */
    DMX_Send_9Data(0x00);
}
/* Send 9bit data and 9bit always set */
void DMX_Send_9Data(uint16_t i)
{
    if (DMX_UART_U0->SR & (1 << 6))
    {
        DMX_UART_U0->DR = 0x0100 | dmxDataU0[i];
    }
    if (DMX_UART_U1->SR & (1 << 6))
    {
        DMX_UART_U1->DR = 0x0100 | dmxDataU1[i];
    }
    // waiting for Send data over
    while ((DMX_UART_U0->SR & 0X40) == 0)
        ;
    while ((DMX_UART_U1->SR & 0X40) == 0)
        ;
}

void DMX_Send_Packet(void)
{
    uint16_t i = 0;
    DMX_Break();        // Break and Start Code
    while (i < 512) // 1-512
    {
        DMX_Send_9Data(i);
        i++;
    }
}
/* Init DMX parameter */
void DMX_Init(void)
{
    GPIO_Tx_Config_AF();
    // enable TIM4
    __HAL_RCC_TIM4_CLK_ENABLE();
    clrdmxData();
    DMX_Reset();
}
/* Send Reset sign and 00Code */
void DMX_Reset(void)
{
    GPIO_Tx_Config_OUT();
    DMX_TX_LOW_U0;
    DMX_TX_LOW_U1;
    TIM4WaitUsec(2000);
    DMX_TX_HIGH_U0;
    DMX_TX_HIGH_U1;
    TIM4WaitUsec(88);
    GPIO_Tx_Config_AF();
    DMX_Send_9Data(0x00);
}
