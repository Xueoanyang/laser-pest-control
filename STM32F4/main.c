/**
 * @file    main.c
 * @brief   主程序文件 - 步进电机控制与激光目标跟踪系统
 * @note    编码: GB2312
 */

#include "headfile.h"

float i = 0.0;                  /* 循环计数变量 */
char string[128];               /* OLED显示字符串缓冲区 */
int ready_flag = 0;             /* 系统就绪标志: 0-未就绪, 1-已就绪 */



int main(void)
{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetPriority(USART3_IRQn, 0);
    NVIC_SetPriority(USART2_IRQn, 15);
    /* 系统时钟初始化: 168MHz */
    delay_init(168);
    DWT_Init();                 /* DWT调试单元初始化 */

    /* 串口初始化 */
    USART1_Config();            /* 串口1: 电机1通信 */
    USART3_Config();            /* 串口3: 电机2通信 */
    USART2_Init();              /* 串口2: 与上位机/视觉模块通信 */
		USART6_Config();

    /* 外部中断初始化: PD13引脚, 用于按键触发 */
    EXTI_PD13_Init();

    /* OLED显示屏初始化 */
    OLED_Init();
    OLED_Clear();

    /* GPIO初始化: 激光笔控制 + 输入信号检测 */
    GPIO_Jiguangbi();           /* 激光笔控制引脚 */
    GPIO_Input_Config();        /* 输入检测引脚配置 */

    /* 定时器初始化 */
    TIM6_Init();                /* 定时器6: 用于周期性任务调度 */
    TIM7_Init();                /* 定时器7: 用于周期性任务调度 */

    /* PID控制器初始化: 电机位置/速度闭环控制 */
    
		Emm_V5_En_Control(MOTOR2_ADDR, true, false);
		delay_ms(100);

		// ↓↓↓ 用这条代替 Reset_CurPos_To_Zero ↓↓↓
		Emm_V5_Read_Sys_Params(MOTOR2_ADDR, S_CPOS);    // 发一条查询命令，电机有回复
		delay_ms(50);                                    // 等电机回复，激活 USART3
		PID_Init();

    delay_ms(2000);             /* 上电延时2秒, 等待各模块稳定 */
		
    ready_flag = 1;             /* 系统就绪 */

    /* 主循环 */
    while (1) {
    // USART2 坐标帧解析（从 ISR 移到主循环）
        if(coord_ready) {
            ParseCoordFrame();
            coord_ready = 0;
        }
        base_control();         /* 基础控制: 执行模式判断与状态机调度 */

        /* 在OLED上显示当前自动状态 */
        switch (g_auto_state)
        {
            case STATE_NO_TARGET:       /* 无目标状态 */
                sprintf((char*)string, "S:NO_OBJ  ");
                OLED_ShowString(0, 0, (unsigned char*)string, 12);
                break;
            case STATE_TRACKING:        /* 目标跟踪中 */
                sprintf((char*)string, "S:TRACK   ");
                OLED_ShowString(0, 1, (unsigned char*)string, 12);
                break;
            case STATE_LASER_LOST:      /* 激光丢失 */
                sprintf((char*)string, "S:NO_LASER");
                OLED_ShowString(0, 2, (unsigned char*)string, 12);
                break;
            case STATE_STRIKE_LOCK:     /* 打击锁定状态 */
                sprintf((char*)string, "S:LOCK %02d", strike_lock_cnt);
                OLED_ShowString(0, 3, (unsigned char*)string, 12);
                break;
            case STATE_SCAN:
                /* 扫描中 */
                sprintf((char*)string, "S:SCAN    ");
                OLED_ShowString(0, 0, (unsigned char*)string, 12);
                break;
            default:                    /* 未知状态 */
                OLED_Clear();
                sprintf((char*)string, "S:UNKNOWN ");
                OLED_ShowString(0, 2, (unsigned char*)string, 12);
                break;
        }


        /* OLED定时刷新: 计数到200时清屏, 防止残影 */
        if (oled_clear_flag >= 200)
        {
            oled_clear_flag = 0;
            OLED_Clear();
        }

        /* 主循环延时20ms, 控制循环频率约50Hz */
        delay_ms(20);
    }
}