#include "pid_move.h"
#include "uart_motor.h"

// 位置环PID（水平轴）
PID_Controller_move pid_pos_x_move = {
    .Kp = 0.25, .Ki = 0.02, .Kd = 0.05,
    .max_output = 10.0f,     // 最大30°/s
    .integral_threshold = 20 // |error|>20像素时禁用积分
};
PID_Controller_move pid_pos_y_move = {
    .Kp = 0.25, .Ki = 0.02, .Kd = 0.05,
    .max_output = 10.0f,     // 最大30°/s
    .integral_threshold = 20 // |error|>20像素时禁用积分
};

// 速度环PID（垂直轴）
PID_Controller_move pid_vel_x_move = {
    .Kp = 1.2, .Ki = 0.1, .Kd = 0.03,
    .max_output = 1.0f,      // 最大2rad/s
    .integral_threshold = 5  // |error|>5°时禁用积分
};
PID_Controller_move pid_vel_y_move = {
    .Kp = 1.2, .Ki = 0.1, .Kd = 0.03,
    .max_output = 1.0f,      // 最大2rad/s
    .integral_threshold = 5  // |error|>5°时禁用积分
};

// 模糊自适应PID计算函数
float PID_Compute(PID_Controller_move *pid, float error, float dt) {
    // 动态调整Kp
    float dynamic_Kp = pid->Kp * (1 + 0.5 * fabs(error));
    
    // 积分分离
    float integral = 0;
    if (fabs(error) < pid->integral_threshold) {
        pid->integral += error * dt*4;
        integral = pid->Ki * pid->integral;
    }
    
    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 计算输出
    float output = dynamic_Kp * error + integral + derivative;
    
    // 抗积分饱和
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral = 0;  // 清零积分
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral = 0;
    }
    return output;
}

// 主控制循环（30Hz调用）
void ControlLoop(float target_x, float target_y, float current_angle_x, float current_angle_y) {
    // 1. 位置环计算目标角度
    float angle_set_x = PID_Compute(&pid_pos_x_move, target_x, 0.02f);
    float angle_set_y = PID_Compute(&pid_pos_y_move, target_y, 0.02f);
    
    // 2. 速度环计算角速度
    float speed_set_x = PID_Compute(&pid_vel_x_move, angle_set_x - current_angle_x, 0.02f)*0.1;
    float speed_set_y = PID_Compute(&pid_vel_y_move, angle_set_y - current_angle_y, 0.02f)*0.1;
    
    // 3. 发送给电机（单位转换：rad/s → 步进电机脉冲）
	if(speed_set_x>=0) Set_SpeedControl(&usart3_ctrl,speed_set_x,CCW_DIRECTION);
	else if(speed_set_x<0)
	{
		speed_set_x = -speed_set_x;
		Set_SpeedControl(&usart3_ctrl,speed_set_x,CW_DIRECTION);
	}
	if(speed_set_y>=0) Set_SpeedControl(&usart1_ctrl,speed_set_y,CCW_DIRECTION);
	else if(speed_set_y<0)
	{
		speed_set_y = -speed_set_y;
		Set_SpeedControl(&usart1_ctrl,speed_set_y,CW_DIRECTION);
	}
}

