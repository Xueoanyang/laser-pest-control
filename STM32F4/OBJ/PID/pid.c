#include "stm32f4xx.h"               
#include "pid.h" 
#include "uartcv.h"
#include "uart_motor.h"
#include "Emm_V5.h"

#define CONSTRAIN(val, min, max) ((val) < (min) ? (min) : (val) > (max) ? (max) : (val))
#define IMAGE_WIDTH 640.0f

PID_Controller pid_angle_x,pid_angle_y,pid_vel_x,pid_vel_y;
#define Y_ANGLE_MIN   -100.0f   // Y电机关节角度下限
#define Y_ANGLE_MAX     80.0f     // Y电机关节角度上限
#define Y_LIMIT_MARGIN 1.0f      // 软限位容差，先用1度
float y_motor_angle = 0.0f;     // Y电机当前角度(来自Emm_V5反馈)

static float y_min_angle = 0.0f;     // 上电时的Y轴下限角
static int y_limit_inited = 0;       // 是否已经记录过下限角
#define Y_LIMIT_MARGIN 0.0f          // 软限位容差，先用1度
#define Y_DOWN_DIR CCW_DIRECTION     // 先假设“往视觉下方”对应CCW    // 转动方向定义#define CCW_DIRECTION 0     // 逆时针

/**
 * @brief  初始化角度环PID（增量式）
 * @param  pid        PID控制器指针
 * @param  kp         比例系数
 * @param  ki         积分系数
 * @param  kd         微分系数
 * @param  max_out    输出限幅（最大角度增量，单位°）
 * @param  int_limit  积分限幅（防止积分饱和）
 * @note   增量式PID输出的是角度增量 delta_angle，不是绝对值
 *         dead_zone=10像素：偏差小于10像素时不响应，避免抖动
 */
void PID_Angle_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float int_limit) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd; 
    pid->max_output = max_out;
    pid->integral_limit = int_limit;
    pid->dead_zone = 0.0f; // 死区：偏差<1.5像素时忽略
    pid->error = 0; pid->last_error = 0; pid->prev_error = 0; pid->integral = 0; pid->output = 0;
    memset(&pid->error, 0, sizeof(pid->error));  // 清零历史误差
}

/**
 * @brief  初始化速度环PID（位置式）
 * @param  pid        PID控制器指针
 * @param  kp         比例系数
 * @param  ki         积分系数
 * @param  kd         微分系数
 * @param  max_out    输出限幅（最大角速度 rad/s）
 * @param  int_limit  积分限幅
 * @note   位置式PID输出的是角速度绝对值 (rad/s)
 *         输入为角度环的增量输出 delta_angle
 */
void PID_Velocity_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float int_limit) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->max_output = max_out;
    pid->integral_limit = int_limit;
    pid->dead_zone = 0.0f; // 死区：偏差<10像素时忽略注意：PID_Velocity_Init 里那个 dead_zone 实际没被用到，改不改都行，顺手改掉也干净。
    pid->error = 0; pid->last_error = 0; pid->prev_error = 0; pid->integral = 0; pid->output = 0;
    memset(&pid->error, 0, sizeof(pid->error));  // 清零历史误差
}
/**
 * @brief  初始化四个PID控制器（X/Y轴各一个角度环+一个速度环）
 * @note   级联PID结构：
 *         - 外环：角度环（增量式），输入=像素偏差→角度误差，输出=角度增量
 *         - 内环：速度环（位置式），输入=角度增量，输出=角速度(rad/s)
 *         参数顺序：Kp, Ki, Kd, 输出限幅, 积分限幅
 *         X轴→电机1(USART1)，Y轴→电机2(USART3)
 *         640×480分辨率，FOV=52°，距离=0.7m，1度≈8.6像素
 */
void PID_Init(void)
{
        // 让Emm_V5 Y电机每100ms自动上报当前位置
    Emm_V5_Auto_Return_Sys_Params_Timed(MOTOR2_ADDR, S_CPOS, 20);
	//max_out：很重要，直接生效
//int_limit：在角度环里你现在这版基本没真用上；在速度环里才真正生效
    // X轴角度环：Kp较大(0.70)响应快，适合水平方向大范围追踪
    PID_Angle_Init(&pid_angle_x, 0.1, 0.1, 0.05, 15.0, 1000);
            //                   Kp      Ki     Kd    max  int_limit
            //                   比例    积分   微分   15°  1000
            //                   灵敏度  消余差  阻尼  单次最大转过15°≈129像素

    // Y轴角度环：Kp较小(0.25)响应慢，竖直方向通常移动幅度小，防过冲
    PID_Angle_Init(&pid_angle_y, 0.08, 0.10, 0.00, 15.0, 1000);
            //                   Kp      Ki     Kd    max  int_limit
            //                   比例    积分   微分   15°  1000
            //                   Kd更大(0.20)增加阻尼，Y轴更稳

    // X轴速度环：将角度增量转换为角速度，max=8.0rad/s≈1.27rps
    PID_Velocity_Init(&pid_vel_x, 0.65, 0.0, 0.15, 15.0, 10000);
            //                      Kp    Ki    Kd   max   int_limit
            //                      比例  积分  微分  8.0   10000
            //                      rad/s               rad/s

    // Y轴速度环：参数同X轴，保证两轴速度响应一致
    PID_Velocity_Init(&pid_vel_y, 0.15, 0.0, 0.0, 8.5, 10000);
            //                      Kp    Ki    Kd   max   int_limit
            //                      比例  积分  微分  3.0   10000
            //                      rad/s               rad/s
}
	
float PID_Angle_Incremental(PID_Controller *pid, float error) {
    // 死区处理
    if (fabs(error) < pid->dead_zone) error = 0;

    // 动态Kp调整
    float kp_effective = pid->Kp * (1.0 +2.5f * fabs(error)); 
    
    // 增量计算
    float delta_out = 
        kp_effective * (error - pid->last_error) 
        + pid->Ki * error 
        + pid->Kd * (error - 2 * pid->last_error + pid->prev_error);

    // 积分分离：大误差时禁用积分
    if (fabs(error) > 30.0f) { // 30像素为阈值
        delta_out -= pid->Ki * error; // 移除积分项
    }
		else if(fabs(error) < 10.0f) delta_out += pid->Ki * error*5;

    // 更新误差历史
    pid->prev_error = pid->last_error;
    pid->last_error = error;
    
    // 输出限幅
    if (delta_out > pid->max_output) delta_out = pid->max_output;
    if (delta_out < -pid->max_output) delta_out = -pid->max_output;
    
    return delta_out*1.1f; // 单位：度
}

float PID_Velocity_Calculate(PID_Controller *pid, float error, float dt) {
    // 积分抗饱和：仅当输出未饱和时累积积分
    if (fabs(pid->output) < pid->max_output) {
        pid->integral += pid->Ki * error * dt;
        // 积分限幅
        if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

		// 在 PID_Velocity_Calculate 里，pid->output 那行之前加：
		float kp_vel_effective = pid->Kp * (1.0 + 0.5f * fabs(error));

		pid->output = 
				kp_vel_effective * error     // 原来是 pid->Kp * error
				+ pid->integral 
				+ pid->Kd * (error - pid->last_error) / dt;
    
    // 输出限幅
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    
    pid->last_error = error;
		if(pid->output<0)pid->output=-pid->output;
    return pid->output*0.9f; // 单位：rad/s
}

// 根据距离（0.5~1.5m）和摄像头FOV计算角度误差
float pixels_to_angle(float pixel_error, float distance) {
    float fov_degrees = 52.0f; // 摄像头FOV=52°
    float pixels_per_degree = (IMAGE_WIDTH / fov_degrees); // 每度像素数
    return (pixel_error / pixels_per_degree) * (1.0f / distance); 
}

// 主控制循环示例
/**
 * @brief  串级PID控制循环：像素误差 → 角度环 → 速度环 → 电机指令
 * @param  target_x   X轴像素偏差（目标坐标 - 当前坐标）
 * @param  target_y   Y轴像素偏差
 * @param  distance   目标距离（米），用于像素→角度换算
 * @note   双环串级结构：
 *         外环(角度环)：像素误差 → 角度增量（增量式PID）
 *         内环(速度环)：角度增量 → 角速度（位置式PID）
 *         Y轴 → 电机1(usart1)，X轴 → 电机2(usart3)
 */
void control_loop(float target_x, float target_y, float distance)
{
    // 1. 像素误差（直接使用传入的坐标偏差）
    float error_x = target_x;
    float error_y = target_y;
    //第一次进入跟踪时，把当前 Y 轴角度记成“下限角”。
    // if(!y_limit_inited)
    // {
    //     y_min_angle = usart3_ctrl.state.current_angle;
    //     y_limit_inited = 1;
    // }

    // 2. 像素误差 → 角度误差（根据距离和相机FOV换算）
    float angle_error_x = pixels_to_angle(error_x, distance);
    float angle_error_y = pixels_to_angle(error_y, distance);

    // 3. 角度环PID（增量式）：角度误差 → 角度增量
    float delta_angle_x = PID_Angle_Incremental(&pid_angle_x, angle_error_x);
    float delta_angle_y = PID_Angle_Incremental(&pid_angle_y, angle_error_y);

    // 4. 速度环PID（位置式）：角度增量 → 角速度 (rad/s)，dt=10ms
    float angular_vel_x = PID_Velocity_Calculate(&pid_vel_x, delta_angle_x, 0.01f);
    float angular_vel_y = PID_Velocity_Calculate(&pid_vel_y, delta_angle_y, 0.01f);

    if(fabs(delta_angle_x) < 0.1f && fabs(delta_angle_y) < 0.1f)
//		if((fabs(target_x) < 0.1f && fabs(target_y) < 0.1f ) || (fabs(delta_angle_x) < 0.1f && fabs(target_y) < 0.1f))
    {
        pid_angle_x.error = 0; pid_angle_x.last_error = 0; pid_angle_x.prev_error = 0;
        pid_angle_y.error = 0; pid_angle_y.last_error = 0; pid_angle_y.prev_error = 0;
        pid_vel_x.error = 0;   pid_vel_x.last_error = 0;   pid_vel_x.integral = 0; pid_vel_x.output = 0;
        pid_vel_y.error = 0;   pid_vel_y.last_error = 0;   pid_vel_y.integral = 0; pid_vel_y.output = 0;
        return;
    }

//先判断，再发
    float y_cmd_angle;
    uint8_t y_cmd_dir;

    if(delta_angle_y >= 0)
    {
        y_cmd_angle = delta_angle_y;
        y_cmd_dir = CCW_DIRECTION;
    }
    else
    {
        y_cmd_angle = -delta_angle_y;
        y_cmd_dir = CW_DIRECTION;
    }

			uint16_t speed_rpm;
			uint8_t acc;
			uint8_t emm_dir;

			if(delta_angle_y >= 0)
					emm_dir = 0;   // 先试，不对就反过来
			else
					emm_dir = 1;

			speed_rpm = (uint16_t)(angular_vel_y * 9.55f);

			acc = 8;
			
			Emm_V5_Read_Sys_Params(MOTOR2_ADDR, S_CPOS);
			delay_ms(5);
			
			// ===== Layer 2: 反馈兜底限位（越界直接停） =====
			if(y_motor_angle < (Y_ANGLE_MIN - Y_LIMIT_MARGIN - 30.0f) || y_motor_angle > (Y_ANGLE_MAX + Y_LIMIT_MARGIN + 30.0f))
			{
					Emm_V5_Stop_Now(MOTOR2_ADDR, false);
					pid_angle_y.error = 0;
					pid_angle_y.last_error = 0;
					pid_angle_y.prev_error = 0;
					pid_vel_y.error = 0;
					pid_vel_y.last_error = 0;
					pid_vel_y.integral = 0;
					pid_vel_y.output = 0;
					return;
			}
			
			// ===== Layer 1: 软件命令限位（方向拦截） =====
			// emm_dir=0 往0度方向, emm_dir=1 往150度方向
			// 如果实测反了，把0和1对调
			uint8_t y_limit_hit = 0;

			if(y_motor_angle <= (Y_ANGLE_MIN + Y_LIMIT_MARGIN))
					y_limit_hit = 1;
			if(y_motor_angle >= (Y_ANGLE_MAX - Y_LIMIT_MARGIN))
					y_limit_hit = 1;
			// 已经在极限位置，但目标是往回走 → 允许
			if(y_motor_angle <= Y_ANGLE_MIN && emm_dir == 0) {  // 想往上走
					y_limit_hit = 0;  // 放行
			}
			if(y_motor_angle >= Y_ANGLE_MAX && emm_dir == 1) {  // 想往下走
					y_limit_hit = 0;  // 放行
			}
			if(fabs(target_y) < 2)
			{
					Emm_V5_Stop_Now(MOTOR2_ADDR, false);
			}
			else
			{
					if(y_limit_hit)
					{
							Emm_V5_Stop_Now(MOTOR2_ADDR, false);
							pid_vel_y.integral = 0;
							pid_vel_y.output = 0;
					}
					else
							Emm_V5_Vel_Control(MOTOR2_ADDR, emm_dir, speed_rpm, acc, false);
			}

    // X轴 → 电机1
    if(delta_angle_x >= 0)
        Set_PositionSpeedControl(&usart1_ctrl, delta_angle_x, angular_vel_x, CW_DIRECTION, 0x20);
    else {
        delta_angle_x = -delta_angle_x;     // 取绝对值
        Set_PositionSpeedControl(&usart1_ctrl, delta_angle_x, angular_vel_x, CCW_DIRECTION, 0x20);
    }
}

void control_loop_move(float target_x, float target_y, float distance) {
    // 1. 计算像素误差
    float error_x = target_x; 
    float error_y = target_y;
    
    // 2. 转换为角度误差
    float angle_error_x = pixels_to_angle(error_x, distance);
    float angle_error_y = pixels_to_angle(error_y, distance);
    
    // 3. 角度环PID计算（增量输出）
    float delta_angle_x = PID_Angle_Incremental(&pid_angle_x, angle_error_x);
    float delta_angle_y = PID_Angle_Incremental(&pid_angle_y, angle_error_y);
    
    // 4. 速度环PID计算（角速度输出）
    float angular_vel_x = PID_Velocity_Calculate(&pid_vel_x, delta_angle_x, 0.01f); // dt=10ms
    float angular_vel_y = PID_Velocity_Calculate(&pid_vel_y, delta_angle_y, 0.01f);
    
    // 5. dead zone: skip if both deltas < 0.1 deg
//    avoid flooding motor with "move 0" cmd that locks driver state
        // 5. 到位判断：偏差<0.1°时不发指令，同时清零PID历史避免状态残留
    if(fabs(delta_angle_x) < 0.1f && fabs(delta_angle_y) < 0.1f)
    {
        pid_angle_x.error = 0; pid_angle_x.last_error = 0; pid_angle_x.prev_error = 0;
        pid_angle_y.error = 0; pid_angle_y.last_error = 0; pid_angle_y.prev_error = 0;
        pid_vel_x.error = 0;   pid_vel_x.last_error = 0;   pid_vel_x.integral = 0; pid_vel_x.output = 0;
        pid_vel_y.error = 0;   pid_vel_y.last_error = 0;   pid_vel_y.integral = 0; pid_vel_y.output = 0;
        return;
    }

    // 5. 输出到舵机（需适配步进电机接口）
	if(delta_angle_y>=0) Set_PositionSpeedControl(&usart1_ctrl, delta_angle_y*0.1f , angular_vel_y , CCW_DIRECTION, 0x20);
	else if(delta_angle_y<0){
		delta_angle_y = -delta_angle_y;
		Set_PositionSpeedControl(&usart1_ctrl, delta_angle_y*0.1f , angular_vel_y , CW_DIRECTION, 0x20);
	}
	if(delta_angle_x>=0) Set_PositionSpeedControl(&usart3_ctrl, delta_angle_x*1.6f , angular_vel_x , CCW_DIRECTION, 0x20);
	else if(delta_angle_x<0){
		delta_angle_x = -delta_angle_x;
		Set_PositionSpeedControl(&usart3_ctrl, delta_angle_x*1.6f , angular_vel_x , CW_DIRECTION, 0x20);
	}
}




