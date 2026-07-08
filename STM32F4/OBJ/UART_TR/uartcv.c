#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "uartcv.h"
#include <string.h>   // 用于 strchr
#include <stdlib.h>   // 用于 atoi

#define USART2_BAUDRATE 115200

// 全局变量
uint8_t rx_buffer[6]; // 接收缓冲区（两个包共用）
volatile uint8_t coord_packet_ready = 0; // 坐标包就绪标志
volatile uint8_t area_packet_ready = 0;  // 面积包就绪标志
CoordPacket current_coord_packet = {-1, -1, -1, -1}; // 坐标包存储
AreaPacket current_area_packet; // 面积包存储
volatile uint8_t coord_ready = 0;
uint8_t coord_buf[32];

void USART2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 2. 配置GPIO引脚复用
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  // PA2(TX), PA3(RX)
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 映射引脚到USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    
    // 4. 配置USART参数
    USART_InitStruct.USART_BaudRate = USART2_BAUDRATE;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStruct);
    
    // 5. 配置接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    // 6. 配置NVIC中断
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 7. 使能USART2
    USART_Cmd(USART2, ENABLE);
}

// 中断服务函数
void USART2_IRQHandler(void) {
	  
    static uint8_t rx_index = 0;
    static uint8_t receiving = 0;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        if(data == '$') {
            rx_index = 0;
            receiving = 1;
        } else if(receiving && data == '#') {
            coord_buf[rx_index] = '\0';
            coord_ready = 1;
            receiving = 0;
        } else if(receiving && rx_index < 31) {
            coord_buf[rx_index++] = data;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// ===== 主循环中调用 =====
void ParseCoordFrame(void) {
    char *c1 = strchr((char*)coord_buf, ',');
    if(c1 != NULL) {
        *c1 = '\0';
        char *c2 = strchr(c1 + 1, ',');
        if(c2 != NULL) {
            *c2 = '\0';
            char *c3 = strchr(c2 + 1, ',');
            if(c3 != NULL) {
                *c3 = '\0';
                current_coord_packet.obj_x   = atoi((char*)coord_buf);
                current_coord_packet.obj_y   = atoi(c1 + 1);
                current_coord_packet.laser_x = atoi(c2 + 1);
                current_coord_packet.laser_y = atoi(c3 + 1);
            }
        }
    }
}

// 坐标包处理函数
void ProcessCoordPacket(void) {
    uint16_t x_raw = (rx_buffer[2] << 8) | rx_buffer[1];
    uint16_t y_raw = (rx_buffer[4] << 8) | rx_buffer[3];
    current_coord_packet.obj_x = (int16_t)x_raw;
    current_coord_packet.obj_y = (int16_t)y_raw;
}

// 面积包处理函数
void ProcessAreaPacket(void) {
    // 小端模式转换：rx_buffer[1]是最低字节
    current_area_packet.area_value = ((uint32_t)rx_buffer[4] << 24) |
                                     ((uint32_t)rx_buffer[3] << 16) |
                                     ((uint32_t)rx_buffer[2] << 8)  |
                                     (uint32_t)rx_buffer[1];
}

