#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "gpio.h"

void GPIO_Jiguangbi(void) {
    // 1. 使能GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // 3. 配置PC8为推挽输出（激光笔）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    GPIO_SetBits(GPIOC, GPIO_Pin_8);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    
    // 按键PB1/PB2的GPIOB时钟（保留，按键还用）
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//    GPIO_SetBits(GPIOC, GPIO_Pin_8);
}

void GPIO_Input_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // 1. 使能GPIOA时钟（AHB1总线）
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // 2. 配置PA4和PA5为下拉输入模式
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;  // 选择PA4和PA5
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;            // 输入模式
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;          // 启用内部下拉电阻
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;       // 速度配置（输入模式下可忽略，但需合法值）

    // 3. 应用配置到GPIOA
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}



