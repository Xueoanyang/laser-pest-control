#ifndef __FUN_H
#define __FUN_H
#include "stm32f4xx.h"         

extern int light_time;
extern int light_flag;
extern int light_flag_old;
extern int count;
extern float bu_dt;
extern int flag_jiguang;
extern int flag_move;

void Set_Jiguangbi(int mode);
void light_set(void);
void base_control(void);
void read_state(void);
void buchangMove(void);

#define STATE_NO_TARGET      0
#define STATE_TRACKING       1
#define STATE_LASER_LOST     2
#define STATE_STRIKE_LOCK    3
#define STATE_SCAN           4

#define STRIKE_LOCK_TICK     150   // base_control每20ms跑一次，150*20ms约3000ms
#define HIT_ENTER_TH         3   // 先按你当前PID参数来,激光进入命中区
#define HIT_EXIT_TH          5   // 离开命中区后重新允许下一次打击

extern int g_auto_state;
extern int strike_lock_cnt;



#endif


