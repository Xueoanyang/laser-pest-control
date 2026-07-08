#ifndef __USART_H
#define __USART_H

#include "headfile.h"
#include "fifo.h"

/**********************************************************
***	X_V2步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;
extern __IO bool rx2FrameFlag;
extern __IO uint8_t rx2Cmd[FIFO_SIZE];
extern __IO uint8_t rx2Count;

void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);
void usart_SendCmd3(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte3(uint16_t data);
void usart_SendCmd4(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte4(uint16_t data);
void usart_send_string(char *str);

#endif
