#include "usart.h"
#include "wit_c_sdk.h"

/**********************************************************
*** X_V2 stepper closed-loop example
**********************************************************/

__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

__IO bool rx2FrameFlag = false;
__IO uint8_t rx2Cmd[FIFO_SIZE] = {0};
__IO uint8_t rx2Count = 0;
static __IO uint8_t rx2WriteIdx = 0;
static __IO bool rx2InFrame = false;


/**
 * @brief UART4 IRQ (JY61P dedicated line)
 * @note  UART4 is added as an independent IMU channel, does not replace USART1/2/3.
 *        Every received byte is fed into WIT SDK parser.
 */
void UART4_IRQHandler(void)
{
    uint8_t data = 0;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        data = (uint8_t)(UART4->DR & 0xFF);

        /* Feed 1 byte at a time to WIT parser */
        WitSerialDataIn(data);

        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
    else if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE interrupt by reading SR then DR */
        UART4->SR; UART4->DR;
    }
}

/**
 * @brief USART1 send bytes
 */
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0;
    for(i = 0; i < len; i++) { usart_SendByte3(cmd[i]); }
}

/**
 * @brief USART1 send one byte
 */
void usart_SendByte(uint16_t data)
{
    __IO uint16_t t0 = 0;

    USART3->DR = (data & (uint16_t)0x01FF);

    while(!(USART3->SR & USART_FLAG_TC))
    {
        ++t0;
        if(t0 > 8000) { return; }
    }
}

/**
 * @brief USART3 send bytes
 */
void usart_SendCmd3(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0;
    for(i = 0; i < len; i++) { usart_SendByte3(cmd[i]); }
}

/**
 * @brief USART3 send one byte
 */
void usart_SendByte3(uint16_t data)
{
    __IO uint16_t t0 = 0;

    USART3->DR = (data & (uint16_t)0x01FF);

    while(!(USART3->SR & USART_FLAG_TXE))
    {
        ++t0;
        if(t0 > 8000) { return; }
    }
}

/**
 * @brief UART4 send bytes (for JY61P command/configuration)
 */
void usart_SendCmd4(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0;
    for(i = 0; i < len; i++) { usart_SendByte4(cmd[i]); }
}

/**
 * @brief UART4 send one byte
 */
void usart_SendByte4(uint16_t data)
{
    __IO uint16_t t0 = 0;

    UART4->DR = (data & (uint16_t)0x01FF);

    while(!(UART4->SR & USART_FLAG_TXE))
    {
        ++t0;
        if(t0 > 8000) { return; }
    }
}

void usart_send_string(char *str)
{
    while(*str)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
    }
}
