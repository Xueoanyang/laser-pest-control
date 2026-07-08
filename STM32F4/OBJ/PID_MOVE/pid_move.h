#ifndef __MOVE_H
#define __MOVE_H
#include "stm32f4xx.h"              
#include "math.h"

// PID结构体（含抗饱和与模糊自适应）
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float max_output;   // 输出限幅
    float integral_threshold; // 积分分离阈值
} PID_Controller_move;

extern PID_Controller_move pid_pos_x_move,pid_pos_y_move,pid_vel_x_move,pid_vel_y_move;

void ControlLoop(float target_x, float target_y, float current_angle_x, float current_angle_y);

#endif

