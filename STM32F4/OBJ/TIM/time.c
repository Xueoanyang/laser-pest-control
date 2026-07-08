#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_it.h"
#include "time.h"
#include "pid.h"
#include "uartcv.h"
#include "fun.h"
#include "exti.h"
#include "uart_motor.h"
#include "fun.h"
#include "key.h"

int oled_clear_flag = 0;
volatile uint64_t uwTick;

// TIM6初始化（10ms中断）
void TIM6_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    // 1. 开启TIM6时钟（APB1总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    
    // 2. 配置时基参数
    TIM_InitStruct.TIM_Prescaler = 8399;      // 预分频值：84MHz / 8400 = 10kHz（0.1ms/计数）
    TIM_InitStruct.TIM_Period = 99;           // 自动重载值：100个计数 = 10ms (99+1=100)
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6, &TIM_InitStruct);
    
    // 3. 使能更新中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    
    // 4. 配置NVIC
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 5. 启动定时器
    TIM_Cmd(TIM6, ENABLE);
}

// TIM7初始化（10ms中断）
void TIM7_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    TIM_InitStruct.TIM_Prescaler = 8399;      // 与TIM6相同（10kHz）
    TIM_InitStruct.TIM_Period = 99;           // 100个计数 = 10ms
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_InitStruct);
    
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2; // 优先级低于TIM6
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    TIM_Cmd(TIM7, ENABLE);
}

// TIM6中断服务函数
void TIM6_DAC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
			  oled_clear_flag++;
        light_set();
			  key_proc();
    }
}

// TIM7中断服务函数
void TIM7_IRQHandler(void) {
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
				uwTick ++;
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        if(current_coord_packet.obj_x==0&&current_coord_packet.obj_y==0&&flag_move==1)count++;
        // 此处添加用户任务
			  bu_dt += 0.01f;
			  if(flag_turn>=1)
				{
					if(++flag_turn>=100){
					flag_turn=0;
					Set_PositionSpeedControl(&usart3_ctrl, 0.9f , 1.0f , CW_DIRECTION, 0x20);
					}
				}
    }
}




