#include "stm32f4xx.h" 
#include "math.h"
#include "stdio.h"
#include "headfile.h"

#define SPEED_MOVE 0.3

int hit_latched = 0;   // 0=还没触发过命中闪烁，1=已经触发过
int light_time=0;
int light_flag=0;
int light_flag_old=1;
int j=0;
float bu_dt =0.0f;
float x = 0.0f;
float number1 = 0.0f;
float number2 = 0.0f;
float number3 = 0.0f;
float bu_number = 0.0f;
int flag_jiguang = 0;
int flag_move = 0;
int flag_at = 0;

int ctrl_state = 0;          // 0启动等待 1跟踪 2巡检 3短暂停留
int startup_cnt = 0;         // 上电等待计数
int lost_cnt = 0;            // 连续丢目标计数
int found_cnt = 0;           // 连续识别到目标计数
int scan_tick = 0;           // 巡检步进节拍
int scan_dir = 0;            // 0逆时针 1顺时针
float scan_step = 2.0f;      // 每次巡检步长，先用2度
float scan_range_cnt = 0;    // 当前这一边已经走了多少步




int g_auto_state = STATE_NO_TARGET;

#define SCAN_ENTER_CNT      (10UL * 1UL * 50UL)  // 10分钟，主循环约50Hz
#define SCAN_REFRESH_CNT    50                    // 约1秒刷新一次速度命令
#define SCAN_X_SPEED        1.30f                 // X轴巡检低速，先用0.30
#define SCAN_X_DIR          CCW_DIRECTION          // 方向不对就改成 CW_DIRECTION

static uint8_t  scan_mode = 1;        // 0=非巡检，1=正在巡检
static uint32_t no_obj_cnt = SCAN_ENTER_CNT;       // 连续无目标计数
static uint16_t scan_refresh_cnt = 0; // 巡检速度命令刷新计数

int strike_lock_cnt = 0;

void Set_Jiguangbi(int mode)
{
	if(mode) GPIO_SetBits(GPIOC, GPIO_Pin_8);   // 原来是 GPIOD, GPIO_Pin_14
	else GPIO_ResetBits(GPIOC, GPIO_Pin_8);      // 原来是 GPIOD, GPIO_Pin_14

}

void light_set(void)
{
    if(ready_flag == 0)
    {
        Set_Jiguangbi(1);
        return;
    }

    if(light_time == 0)
    {
        Set_Jiguangbi(1);   // 平时常亮
        return;
    }

    light_time++;

    if(light_time <= 30)
        Set_Jiguangbi(0);   // 灭300ms
    else if(light_time <= 60)
        Set_Jiguangbi(1);   // 亮100ms
    else if(light_time <= 90)
        Set_Jiguangbi(0);   // 再灭100ms，更明显
		else if(light_time <= 120)
        Set_Jiguangbi(1);   // 亮100ms
    else if(light_time <= 150)
        Set_Jiguangbi(0);   // 再灭100ms，更明显
    else
    {
        light_time = 0;
        Set_Jiguangbi(1);   // 闪完恢复常亮
    }
}

int count=0;
static int16_t last_x = 0, last_y = 0;   // ← 新增：记录上次坐标

// ============================================================
//  base_control — 固定式二维云台主控制逻辑（单帧处理）
// ============================================================

/**
 * @brief  基础控制函数 - 根据目标和激光位置控制电机跟踪目标
 * @note   状态机逻辑: 打击锁定 -> 无目标 -> 激光有效判断 -> 偏差计算 -> 打击判断 -> 跟踪控制
 */
void base_control(void)
{
    int16_t dev_x = 0;             /* X方向偏差: 目标X - 激光X */
    int16_t dev_y = 0;             /* Y方向偏差: 目标Y - 激光Y */
    int laser_valid = 0;           /* 激光有效标志: 1-激光检测到, 0-激光丢失 */

//    // 1. 如果当前正在打击锁定，电机直接锁住
    if(strike_lock_cnt > 0)
    {
        g_auto_state = STATE_STRIKE_LOCK;         /* 设置状态为打击锁定 */
        strike_lock_cnt--;       /* 锁定计数递减, 倒计时结束后解除锁定 */
    }

    // 2. 没目标：停机，同时允许下次重新打击
    if(current_coord_packet.obj_x == -1 && current_coord_packet.obj_y == -1)
    {
        g_auto_state = STATE_NO_TARGET;

        // Y轴始终停，防止俯仰轴乱动
        Emm_V5_Stop_Now(MOTOR2_ADDR, false);

        // 无目标时清除打击状态
        hit_latched = 0;
        strike_lock_cnt = 0;

        // 系统没ready之前，不巡检，只停机
        if(ready_flag == 0)
        {
            Set_SpeedControl(&usart1_ctrl, 0.0f, CCW_DIRECTION);
            return;
        }

        // 先保留 -1,-1 立刻停逻辑：10分钟以内，X也停
        if(no_obj_cnt < SCAN_ENTER_CNT)
        {
            no_obj_cnt++;
            Set_SpeedControl(&usart1_ctrl, 0.0f, CCW_DIRECTION);
            return;
        }

        // 连续无目标超过10分钟，进入巡检
        scan_mode = 1;
        g_auto_state = STATE_SCAN;

        // 巡检：Y停，X低速一直转
        if(scan_refresh_cnt == 0)
        {
            Set_SpeedControl(&usart1_ctrl, SCAN_X_SPEED, SCAN_X_DIR);
        }

        scan_refresh_cnt++;
        if(scan_refresh_cnt >= SCAN_REFRESH_CNT)
        {
            scan_refresh_cnt = 0;
        }

        return;
    }

    // 重新看到目标：如果之前在巡检，先停X，重置PID，下一轮再正常跟踪
    if(scan_mode)
    {
        Set_SpeedControl(&usart1_ctrl, 0.0f, CCW_DIRECTION);
        Emm_V5_Stop_Now(MOTOR2_ADDR, false);

        PID_Init();

        scan_mode = 0;
        no_obj_cnt = 0;
        scan_refresh_cnt = 0;

        return;   // 先停一下，下一次主循环再进入正常跟踪
    }

    // 只要看到目标，就清零无目标计数
    no_obj_cnt = 0;

    // 3. 判断激光点是否有效
    laser_valid = (current_coord_packet.laser_x != -1 && current_coord_packet.laser_y != -1);

    // 4. 算偏差
    if(laser_valid)
    {
        g_auto_state = STATE_TRACKING;            /* 设置状态为跟踪中 */
        dev_x = current_coord_packet.obj_x - current_coord_packet.laser_x;  /* 计算X方向偏差 */
        dev_y = current_coord_packet.obj_y - current_coord_packet.laser_y;  /* 计算Y方向偏差 */
    }
    else
    {
        g_auto_state = STATE_LASER_LOST;          /* 设置状态为激光丢失 */
        dev_x = current_coord_packet.obj_x - 320;  /* 以图像中心为基准计算偏差 */
        dev_y = current_coord_packet.obj_y - 240;  /* 以图像中心为基准计算偏差 */
    }
		


    // 5. 只有"激光点有效"时，才允许进入打击

//    if(laser_valid && abs(dev_x) < HIT_ENTER_TH && abs(dev_y) < HIT_ENTER_TH && ready_flag == 1)
		static uint64_t hit_uwtick = 0;        // ← 加 static
		static uint8_t  hit_uwtick_flag = 0;   // ← 加 static，类型改 uint8_t

		if(abs(dev_x) < HIT_ENTER_TH && abs(dev_y) < HIT_ENTER_TH && ready_flag == 1)
		{
				Set_SpeedControl(&usart1_ctrl, 0.0f, CCW_DIRECTION);  /* 电机1锁停 */
				Emm_V5_Stop_Now(MOTOR2_ADDR, false);
				if(hit_uwtick_flag == 0)           // ← 只在首次进入时记录
				{
						hit_uwtick = uwTick;
						hit_uwtick_flag = 1;
				}
		}
		else
		{
				hit_uwtick_flag = 0;               // ← 离开范围时清零，下次重新计时
		}

		if(hit_uwtick_flag == 1 && uwTick - hit_uwtick >= 100)
		{
				if(abs(dev_x) < HIT_ENTER_TH && abs(dev_y) < HIT_ENTER_TH && ready_flag == 1)
				{
					// ... 打击逻辑 ...
					if(hit_latched == 0 && light_time == 0)
					{
	            light_time = 1;                 /* 触发激光开启 */
							hit_latched = 1;                /* 锁存打击标志, 避免连续触发 */
							strike_lock_cnt = STRIKE_LOCK_TICK; /* 设置锁定时长, 保持打击位置 */
					}

					g_auto_state = STATE_STRIKE_LOCK;         /* 设置状态为打击锁定 */
					Set_SpeedControl(&usart1_ctrl, 0.0f, CCW_DIRECTION);  /* 电机1锁停 */
					Emm_V5_Stop_Now(MOTOR2_ADDR, false);
					uwTick = 0;
					hit_uwtick_flag = 0;               // ← 打击后清零，允许下次再触发
					return;
				}
		}

    // 6. 重新解锁条件：目标离开命中区，允许下次再打
    if(abs(dev_x) > HIT_EXIT_TH || abs(dev_y) > HIT_EXIT_TH)
    {
        hit_latched = 0;          /* 清除打击锁存, 允许下次打击 */
    }

    // 7. 正常跟踪
		if(hit_uwtick_flag == 0)
		{
			control_loop(dev_x, dev_y, 0.7f);   /* PID控制循环, 根据偏差调整电机速度 */
		}
}


void buchangMove(void)
{
	switch(flag_count%4)
	{
		case 0:
				x=bu_dt*SPEED_MOVE;
				number1 = x-0.5f;
				number2 = 2.0f*number1*number1+0.5f;
				number3 = SPEED_MOVE/number2;
			if(bu_number<=0.1f) bu_number += number3*0.0075f;
			else 
			{
				bu_number += number3*0.01f;
				Set_PositionSpeedControl(&usart3_ctrl, bu_number , 1.0f , CCW_DIRECTION, 0x20);
				bu_number = 0;
			}
			break;
		case 1:
				x=bu_dt*SPEED_MOVE;
			number1 = x-1.5f;
			number2 = 1.0f+number1*number1;
			number3 = SPEED_MOVE/number2;
			if(bu_number<=0.1f) bu_number += number3*0.0075f;
				else 
				{
					bu_number += number3*0.01f;
					Set_PositionSpeedControl(&usart3_ctrl, bu_number , 1.0f , CW_DIRECTION, 0x20);
					bu_number = 0;
				}
				break;
		case 2:
			number1 = x-0.5f;
			number2 = 2.25f+number1*number1;
			number3 = 1.5f*SPEED_MOVE/number2;
			if(bu_number<=0.1f) bu_number += number3*0.0075f;
					else 
					{
						bu_number += number3*0.01f;
						Set_PositionSpeedControl(&usart3_ctrl, bu_number , 1.0f , CW_DIRECTION, 0x20);
						bu_number = 0;
					}
		case 3:
			number1 = x-1.5f;
			number2 = 1.0f+number1*number1;
			number3 = SPEED_MOVE/number2;
			if(bu_number<=0.1f) bu_number += number3*0.0075f;
					else 
					{
						bu_number += number3*0.01f;
						Set_PositionSpeedControl(&usart3_ctrl, bu_number , 1.0f , CCW_DIRECTION, 0x20);
						bu_number = 0;
					}
	}
}










