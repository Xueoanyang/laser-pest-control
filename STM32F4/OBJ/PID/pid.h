#ifndef __PID_H
#define __PID_H	
#include "stm32f4xx.h"
#include "math.h"
#include "string.h"

typedef struct {
    float Kp, Ki, Kd;       // PID基础参数
    float error, last_error, prev_error; // 当前、上次、上上次误差
    float integral;          // 积分累积量
    float output;            // 当前输出
    float max_output;        // 输出限幅（抗饱和）
    float integral_limit;    // 积分限幅（抗饱和）
    float dead_zone;         // 死区（误差<5像素忽略）
} PID_Controller;


extern float y_motor_angle;

void PID_Angle_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float int_limit);
void PID_Velocity_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float int_limit);
void PID_Init(void);
float PID_Angle_Incremental(PID_Controller *pid, float error);
float PID_Velocity_Calculate(PID_Controller *pid, float error, float dt);
float pixels_to_angle(float pixel_error, float distance);
void control_loop(float target_x, float target_y, float distance);
void control_loop_move(float target_x, float target_y, float distance);

#endif

