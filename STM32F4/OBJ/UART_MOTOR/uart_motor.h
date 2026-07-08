#ifndef __MOTOR_H
#define __MOTOR_H	
#include "stm32f4xx.h"

// 电机地址定义
#define MOTOR1_ADDR 0x01
#define MOTOR2_ADDR 0x02

// 控制模式定义
#define SPEED_CTRL_MODE      0x01
#define POSITION_CTRL_MODE   0x02
#define TORQUE_CTRL_MODE     0x03
#define ABS_ANGLE_CTRL_MODE  0x04 

// 转动方向定义
#define CCW_DIRECTION 0     // 逆时针
#define CW_DIRECTION  1     // 顺时针

// 帧结构定义
#pragma pack(push, 1)

// 控制帧结构（表3-1） - 保持不变
typedef struct {
    uint8_t header;         // 帧头 0x7B
    uint8_t address;        // 设备地址
    uint8_t control_mode;   // 控制模式
    uint8_t direction;      // 转向
    uint8_t subdivide;      // 细分值
    uint8_t pos_h;          // 位置高字节
    uint8_t pos_l;          // 位置低字节
    uint8_t speed_h;        // 速度高字节
    uint8_t speed_l;        // 速度低字节
    uint8_t bcc;            // BCC校验
    uint8_t footer;         // 帧尾 0x7D
} Motor_CtrlFrame_t;

// 反馈帧结构（表3-13） - 根据图片描述修改
typedef struct {
    uint8_t address;        // 设备地址
    uint8_t arrived;        // 位置到达状态
    uint8_t speed_h;        // 速度高字节
    uint8_t speed_l;        // 速度低字节
    uint8_t pos_hh;         // 角度高16位高8位（注意变量名改为pos_xx）
    uint8_t pos_hl;         // 角度高16位低8位
    uint8_t pos_lh;         // 角度低16位高8位
    uint8_t pos_ll;         // 角度低16位低8位
    uint8_t bcc;            // BCC校验
} Motor_FeedbackFrame_t; // 总共9字节
#pragma pack(pop)

// 电机数据结构 - 保持不变
typedef struct {
    float current_speed;    // 当前速度 (rad/s)
    float current_angle;    // 当前角度 (度)
    uint8_t arrived_status; // 是否到达目标位置
} Motor_State_t;

// 串口控制器结构
typedef struct {
    USART_TypeDef* USARTx;
    uint8_t addr;           // 电机地址
    Motor_State_t state;    // 当前状态
    uint8_t rx_buf[sizeof(Motor_FeedbackFrame_t)]; // 接收缓冲区
    uint8_t rx_index;       // 接收索引
    uint8_t frame_ready;    // 帧就绪标志
} USART_Controller_t;

extern uint64_t irq_count;

// 全局控制器实例
extern USART_Controller_t usart1_ctrl;
extern USART_Controller_t usart3_ctrl;

// 自定义函数声明
void USART1_Config(void);
void USART3_Config(void);
void USART6_Config(void);
void Send_ControlFrame(USART_Controller_t *ctrl, Motor_CtrlFrame_t *frame);
void Process_Feedback(USART_Controller_t *ctrl);
uint8_t Calculate_BCC(uint8_t *data, uint8_t len);
void Set_SpeedControl(USART_Controller_t *ctrl, float speed, uint8_t direction);
void Set_PositionControl(USART_Controller_t *ctrl, float angle, uint8_t direction, uint8_t subdivide);
void Set_AbsoluteAngleControl(USART_Controller_t *ctrl, float angle, float speed_rps, uint8_t subdivide);
void Set_PositionSpeedControl(USART_Controller_t *ctrl, float angle, float speed, uint8_t direction, uint8_t subdivide);

#endif


