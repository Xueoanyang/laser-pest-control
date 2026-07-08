#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "fun.h"

int flag_count = 0;
int flag_turn = 0;

// 外部中断初始化函数
void EXTI_PD13_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 使能GPIOD时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // 2. 配置PD13为输入模式
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;       // 输入模式
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉电阻（防干扰）
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速响应
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 3. 使能SYSCFG时钟（关键！）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // 4. 映射PD13到EXTI线13
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);

    // 5. 配置EXTI线13
    EXTI_InitStruct.EXTI_Line = EXTI_Line13;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;   // 中断模式
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; // 上升沿触发
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;             // 使能中断线
    EXTI_Init(&EXTI_InitStruct);

    // 6. 配置NVIC（中断控制器）
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn; // EXTI线10-15共享此通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F; // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;        // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;               // 使能中断
    NVIC_Init(&NVIC_InitStruct);
}

// 中断服务函数（EXTI线10-15共用）
void EXTI15_10_IRQHandler(void) {
    // 检查EXTI线13是否触发
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
        // 清除中断标志（必须！否则会重复进入中断）
        EXTI_ClearITPendingBit(EXTI_Line13);

        // 用户逻辑（示例：翻转LED或设置标志位）
        flag_count++;
			  bu_dt = 0.0f;
			  flag_turn = 1;
    }
}


