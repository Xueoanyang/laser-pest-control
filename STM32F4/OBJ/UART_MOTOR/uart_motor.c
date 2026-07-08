#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <string.h>
#include "uart_motor.h"
#include "oled.h"


// 全局控制器实例
USART_Controller_t usart1_ctrl = {
    .USARTx = USART6,
    .addr = MOTOR1_ADDR,
    .rx_index = 0,
    .frame_ready = 0
};

USART_Controller_t usart3_ctrl = {
    .USARTx = USART3,
    .addr = MOTOR2_ADDR,
    .rx_index = 0,
    .frame_ready = 0
};

uint64_t irq_count = 0;


// 计算BCC校验（异或校验）
uint8_t Calculate_BCC(uint8_t *data, uint8_t len) {
    uint8_t bcc = 0;
    for (uint8_t i = 0; i < len; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

// 初始化定时器用于超时检测
void TIM_Config(void) {
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 定时器配置（1ms中断）
    TIM_InitStruct.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1MHz
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = 1000 - 1; // 1ms
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    // 3. 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 4. NVIC配置
    NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 5. 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

// USART1初始化（PA9为TX，PA10为RX）
void USART1_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 2. 配置GPIO为复用功能
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    
    // 4. 串口参数设置
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);
    
    // 5. 使能接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 6. NVIC配置
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 7. 启动串口
    USART_Cmd(USART1, ENABLE);
}

// USART3初始化（PC10为TX，PC11为RX）
void USART3_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 2. 配置GPIO为复用功能
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
    
    // 4. 串口参数设置
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);
    
    // 5. 使能接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    
    // 6. NVIC配置
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 7. 启动串口
    USART_Cmd(USART3, ENABLE);
}

void USART6_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART_InitStruct);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART6, ENABLE);
}

// 发送控制帧函数（BCC校验范围修正）
void Send_ControlFrame(USART_Controller_t *ctrl, Motor_CtrlFrame_t *frame) {
    // 填充帧头
    frame->header = 0x7B;
    frame->address = ctrl->addr;
    frame->footer = 0x7D;
    
    // 计算BCC校验（帧头到速度低字节共9字节）
    // 重要：协议要求校验范围是从帧头(0x7B)到速度低字节(speed_l)
    uint8_t bcc_data[9] = {
        frame->header,
        frame->address,
        frame->control_mode,
        frame->direction,
        frame->subdivide,
        frame->pos_h,
        frame->pos_l,
        frame->speed_h,
        frame->speed_l
    };
    
    frame->bcc = Calculate_BCC(bcc_data, 9);
    
    // 发送整个帧
    uint8_t *p = (uint8_t*)frame;
    for (int i = 0; i < sizeof(Motor_CtrlFrame_t); i++) {
        USART_SendData(ctrl->USARTx, p[i]);
        while (USART_GetFlagStatus(ctrl->USARTx, USART_FLAG_TXE) == RESET);
    }
}

// 设置速度控制模式
void Set_SpeedControl(USART_Controller_t *ctrl, float speed, uint8_t direction) {
    Motor_CtrlFrame_t frame = {0};
    
    // 配置控制参数
    frame.control_mode = SPEED_CTRL_MODE;
    frame.direction = direction;
    frame.subdivide = 0x20; // 32细分（常用值）
    
    // 速度参数处理（放大10倍）
    uint16_t speed_10x = (uint16_t)(speed * 10.0f);
    frame.speed_h = (speed_10x >> 8) & 0xFF;
    frame.speed_l = speed_10x & 0xFF;
    
    // 位置参数清零（速度模式）
    frame.pos_h = 0x00;
    frame.pos_l = 0x00;
    
    Send_ControlFrame(ctrl, &frame);
}

// 设置位置控制模式
void Set_PositionControl(USART_Controller_t *ctrl, float angle, uint8_t direction, uint8_t subdivide) {
    Motor_CtrlFrame_t frame = {0};
    
    // 配置控制参数
    frame.control_mode = POSITION_CTRL_MODE;
    frame.direction = direction;
    frame.subdivide = subdivide;
    
    // 角度参数处理（放大10倍）
    uint16_t angle_10x = (uint16_t)(angle * 10.0f);
    frame.pos_h = (angle_10x >> 8) & 0xFF;
    frame.pos_l = angle_10x & 0xFF;
    
    // 速度参数（使用默认速度，可根据需要调整）
    frame.speed_h = 0x00;
    frame.speed_l = 0x64; // 10 rad/s
    
    Send_ControlFrame(ctrl, &frame);
}

// 新增绝对角度控制函数实现（添加在位置控制函数后）
void Set_AbsoluteAngleControl(USART_Controller_t *ctrl, float angle, float speed_rps, uint8_t subdivide) {
	   // 角度边界检查
//    angle = angle > 360.0f ? 360.0f : angle;
//    angle = angle < -360.0f ? -360.0f : angle;
    
    // 速度边界检查（根据电机性能设定）
    speed_rps = speed_rps > 20.0f ? 20.0f : speed_rps;
    speed_rps = speed_rps < 0.1f ? 0.1f : speed_rps;
	
    Motor_CtrlFrame_t frame = {0};
    
    // 配置控制参数
    frame.control_mode = ABS_ANGLE_CTRL_MODE;
    frame.subdivide = subdivide;
    
    // 绝对角度参数处理（放大10倍发送）
    uint16_t angle_10x = (int16_t)(angle * 10.0f);
    frame.pos_h = (angle_10x >> 8) & 0xFF;
    frame.pos_l = angle_10x & 0xFF;
    
    // 转速参数处理（放大10倍发送，单位：弧度/秒）
    uint16_t speed_10x = (uint16_t)(speed_rps * 10.0f);
    frame.speed_h = (speed_10x >> 8) & 0xFF;
    frame.speed_l = speed_10x & 0xFF;
    
    // 方向参数在绝对角度模式中固定设置（因方向由目标角度决定）
    frame.direction = 0x00;  // 协议中指定此模式方向参数无效
    
    Send_ControlFrame(ctrl, &frame);
}

// 设置位置控制模式（增加速度参数）
void Set_PositionSpeedControl(USART_Controller_t *ctrl, float angle, float speed, uint8_t direction, uint8_t subdivide) {
    Motor_CtrlFrame_t frame = {0};
    
    // 配置控制参数
    frame.control_mode = POSITION_CTRL_MODE;
    frame.direction = direction;
    frame.subdivide = subdivide;
    
    // 1. 角度参数处理（放大10倍）
    uint16_t angle_10x = (uint16_t)(angle * 10.0f);
    frame.pos_h = (angle_10x >> 8) & 0xFF;
    frame.pos_l = angle_10x & 0xFF;
    
    // 2. 速度参数处理（新增形参 speed）
    // 限制速度范围：0~43 rad/s
    if (speed < 0.0f) speed = 0.0f;
    else if (speed > 43.0f) speed = 43.0f; // 最高43 rad/s
    
    // 速度放大10倍并转换为整数
    uint16_t speed_10x = (uint16_t)(speed * 10.0f);
    // 拆分高/低字节
    frame.speed_h = (speed_10x >> 8) & 0xFF; // 高8位
    frame.speed_l = speed_10x & 0xFF;        // 低8位
    
    Send_ControlFrame(ctrl, &frame);
}

extern char string[128];
#include "stdio.h"

// 完全重写反馈处理函数
void Process_Feedback(USART_Controller_t *ctrl) {
    if (!ctrl->frame_ready) return;
    
    // 直接解析缓冲区
    Motor_FeedbackFrame_t frame;
    memcpy(&frame, ctrl->rx_buf, sizeof(Motor_FeedbackFrame_t));
    
    // BCC校验 - 范围：地址到角度低8位（共8字节）
    uint8_t bcc_data[8] = {
        frame.address,
        frame.arrived,
        frame.speed_h,
        frame.speed_l,
        frame.pos_hh,    // 变量名改为pos_xx
        frame.pos_hl,
        frame.pos_lh,
        frame.pos_ll
    };
    
    uint8_t calc_bcc = Calculate_BCC(bcc_data, 8);
    
    // 调试输出原始数据
    char raw_str[50];
    sprintf(raw_str, "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
            frame.address, frame.arrived, frame.speed_h, frame.speed_l,
            frame.pos_hh, frame.pos_hl, frame.pos_lh, frame.pos_ll, frame.bcc);
    OLED_ShowString(0, 12, (uint8_t*)raw_str, 8);
    
    if (frame.bcc == calc_bcc) {
        // 解析速度（10倍放大）
        uint16_t speed_10x = (frame.speed_h << 8) | frame.speed_l;
        ctrl->state.current_speed = speed_10x / 10.0f;
        
        // 解析角度（32位有符号整数）
        // 重要：使用联合体直接处理32位整数
        union {
            int32_t full_value;
            struct {
                uint8_t b0;
                uint8_t b1;
                uint8_t b2;
                uint8_t b3;
            } bytes;
        } angle_data;
        
        angle_data.bytes.b3 = frame.pos_hh;
        angle_data.bytes.b2 = frame.pos_hl;
        angle_data.bytes.b1 = frame.pos_lh;
        angle_data.bytes.b0 = frame.pos_ll;
        
        // 处理负角度（图片说明为补码）
        int32_t angle_10x = angle_data.full_value;
        ctrl->state.current_angle = (float)angle_10x / 10.0f;
        
        ctrl->state.arrived_status = frame.arrived;
        
        // 显示处理结果
        char angle_str[30];
        sprintf(angle_str, "Angle: %.1f", ctrl->state.current_angle);
        OLED_ShowString(0, 8, (uint8_t*)angle_str, 8);
        
        char speed_str[30];
        sprintf(speed_str, "Speed: %.1f", ctrl->state.current_speed);
        OLED_ShowString(0, 10, (uint8_t*)speed_str, 8);
    } else {
        // 显示BCC错误
        char err_str[30];
        sprintf(err_str, "BCC:RCV%02X CALC%02X", frame.bcc, calc_bcc);
        OLED_ShowString(0, 14, (uint8_t*)err_str, 8);
    }
    
    // 重置接收状态
    ctrl->frame_ready = 0;
    ctrl->rx_index = 0;
}

// 直接在中断中处理并显示反馈数据
void ProcessAndDisplay_Feedback(uint8_t *data,USART_Controller_t *ctrl) {
    // 解析各字段
    uint8_t address = data[0];
    uint8_t arrived = data[1];
    uint16_t speed = (data[2] << 8) | data[3];
    int32_t angle = ((int32_t)data[4] << 24) | 
                  ((int32_t)data[5] << 16) | 
                  ((int32_t)data[6] << 8) | 
                  data[7];
    uint8_t bcc = data[8];
    
    // BCC校验（前8字节）
    uint8_t calc_bcc = Calculate_BCC(data, 8);
    
//    // 创建显示字符串
//    char display_str[128];
    
    if (bcc == calc_bcc) {
        // 处理负角度（反码转换）
        if (angle & 0x80000000) {
            angle = -(int32_t)(((uint32_t)~angle + 1) & 0x7FFFFFFF);
        }
        
        ctrl->state.current_angle = (float)angle / 10.0f;
        ctrl->state.current_speed = (float)speed / 10.0f;
				
				
        
        // 根据图片中的示例格式创建显示内容
//        if (arrived == 0x00) {
//					 if(address == 0x01){
//            sprintf((char*)display_str, "%.1f", actual_angle);
//					  OLED_ShowString(0, 8, (unsigned char*)display_str, 16);
					 }
//					 else if(address == 0x02){
//						sprintf((char*)display_str, "%.1f", actual_angle);
//					  OLED_ShowString(1, 10, (unsigned char*)display_str, 16);
//					 }
//        } else {
//					  if(address == 0x01){
//            sprintf((char*)display_str, "%.1f", actual_angle);
//						OLED_ShowString(0, 8, (unsigned char*)display_str, 12);
//						}
//						else if(address == 0x02){
//						sprintf((char*)display_str, "%.1f", actual_angle);
//						OLED_ShowString(60, 10, (unsigned char*)display_str, 12);
//						}
//        }
//    } 
//		else {
        // 校验失败显示
//        sprintf(display_str, "M%d:BCC  %02X/%02X", address, bcc, calc_bcc);
//			    OLED_ShowString(0, 12, (uint8_t*)display_str, 16);
//    }
    
//    // 直接在OLED上显示
//    OLED_ShowString(0, 10, (uint8_t*)display_str, 16);
}

char display_str[128];

// 在中断服务函数中直接处理并显示数据
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        static uint8_t rx_buffer[9];  // 9字节反馈帧
        static uint8_t byte_count = 0;
        static uint8_t frame_started = 0;
        
        uint8_t data = USART_ReceiveData(USART1);
        
        // 状态机处理
        if (!frame_started) {
            // 等待地址字节（0x01）
            if (data == MOTOR1_ADDR) {
                rx_buffer[0] = data;
                byte_count = 1;
                frame_started = 1;
            }
        } else {
            // 接收剩余字节
            rx_buffer[byte_count++] = data;
            
            // 完整帧接收完成
            if (byte_count >= 9) {
                frame_started = 0;
                byte_count = 0;
                
                // 直接在中断中处理并显示数据
                ProcessAndDisplay_Feedback(rx_buffer,&usart1_ctrl);
            }
        }

//        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void USART6_IRQHandler(void) {
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
        static uint8_t rx_buffer[9];
        static uint8_t byte_count = 0;
        static uint8_t frame_started = 0;

        uint8_t data = USART_ReceiveData(USART6);

        if (!frame_started) {
            if (data == MOTOR1_ADDR) {
                rx_buffer[0] = data;
                byte_count = 1;
                frame_started = 1;
            }
        } else {
            rx_buffer[byte_count++] = data;

            if (byte_count >= 9) {
                frame_started = 0;
                byte_count = 0;
                ProcessAndDisplay_Feedback(rx_buffer, &usart1_ctrl);
            }
        }
    }
}

/// 同样的处理适用于USART3
void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
				
        static uint8_t rx_buffer[9];
        static uint8_t byte_count = 0;
        static uint8_t frame_started = 0;
        static uint8_t frame_type;  // 0=未知, 1=MS42DC(9B), 2=EmmV5_S_CPOS(7B)
        
        uint8_t data = USART_ReceiveData(USART3);
        
        if (!frame_started) {
            if (data == MOTOR2_ADDR) {
                rx_buffer[0] = data;
                byte_count = 1;
                frame_started = 1;
                frame_type = 0;
            }
        } else {
            rx_buffer[byte_count] = data;
            byte_count++;
            
            // 第2字节判断帧类型: 0x36 = Emm_V5 S_CPOS回复
            if(byte_count == 2 && frame_type == 0) {
                frame_type = (data == 0x36) ? 2 : 1;
            }
            
           // Emm_V5 S_CPOS: 8字节 [addr, 0x36, sign, pos(4B), 0x6B]
            if(frame_type == 2 && byte_count >= 8) {
								irq_count++;   // ← 加这行，看中断有没有触发
                frame_started = 0;
                byte_count = 0;
                if(rx_buffer[7] == 0x6B) {
										
                    uint8_t sign = rx_buffer[2];
                    uint32_t raw = ((uint32_t)rx_buffer[3] << 24) |
                                   ((uint32_t)rx_buffer[4] << 16) |
                                   ((uint32_t)rx_buffer[5] << 8)  |
                                    (uint32_t)rx_buffer[6];
                    int32_t pos = (sign == 0x01) ? -(int32_t)raw : (int32_t)raw;
                    extern float y_motor_angle;
                    y_motor_angle = (float)pos / 65536.0f * 360.0f;
                }
            }
            // MS42DC反馈: 9字节
            else if(frame_type == 1 && byte_count >= 9) {
                frame_started = 0;
                byte_count = 0;
                ProcessAndDisplay_Feedback(rx_buffer, &usart3_ctrl);
            }
        }
    }
}


// 使用定时器中断处理超时
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
//        // 超时处理（每1ms一次）
//        static uint16_t usart1_timeout = 0;
//        static uint8_t frame_started = 0;
//        
//        usart1_timeout++;
//        
//        // 超时复位（20ms未接收完成）
//        if (usart1_timeout > 20) {
//            usart1_ctrl.rx_index = 0;
//            frame_started = 0;
//            usart1_timeout = 0;
//        }
    }
}



