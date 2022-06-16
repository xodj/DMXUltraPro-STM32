#include <dmx512.h>
#include "stm32f1xx_hal.h"
#define HAL_UART_MODULE_ENABLED
#include "stm32f1xx_hal_uart.h"
/**
 * @brief Blocks for usec nanoseconds by using TIM3.
 *
 * The wait is achieved by setting up the timer TIM3, followed by enabling
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
#define APBTimFreq 72000000
#define Tim3Freq APBTimFreq / 1000000 - 1
#define TIM3WaitUsec(usec)         \
    TIM3->PSC = Tim3Freq;          \
    TIM3->ARR = usec;              \
    TIM3->CNT = 0;                 \
    TIM3->CR1 |= TIM_CR1_CEN;      \
    while (TIM3->CNT != TIM3->ARR) \
    {                              \
    }                              \
    TIM3->CR1 &= ~TIM_CR1_CEN

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
    TIM3WaitUsec(88); // DMX512 1990's BREAK >88us
    DMX_TX_HIGH_U0;
    DMX_TX_HIGH_U1;
    TIM3WaitUsec(8); // DMX512 1990's Mark after break MAB >8us
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
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    UART_HandleTypeDef huart1;
    UART_HandleTypeDef huart3;
    huart1.Instance = DMX_UART_U0;
    huart1.Init.BaudRate = 250000;
    huart1.Init.WordLength = UART_WORDLENGTH_9B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Instance = DMX_UART_U1;
    huart3.Init.BaudRate = 250000;
    huart3.Init.WordLength = UART_WORDLENGTH_9B;
    huart3.Init.StopBits = UART_STOPBITS_2;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
        HAL_NVIC_SystemReset();
    if (HAL_UART_Init(&huart3) != HAL_OK)
        HAL_NVIC_SystemReset();

    // enable TIM3
    __HAL_RCC_TIM3_CLK_ENABLE();
    clrdmxData();
    DMX_Reset();
}
/* Send Reset sign and 00Code */
void DMX_Reset(void)
{
    GPIO_Tx_Config_OUT();
    DMX_TX_LOW_U0;
    DMX_TX_LOW_U1;
    TIM3WaitUsec(2000);
    DMX_TX_HIGH_U0;
    DMX_TX_HIGH_U1;
    TIM3WaitUsec(88);
    GPIO_Tx_Config_AF();
    DMX_Send_9Data(0x00);
}
