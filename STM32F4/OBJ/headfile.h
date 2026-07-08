#ifndef __HEADFILE_H
#define __HEADFILE_H	

#include "stm32f4xx.h"  
#include "stdio.h"
#include "delay.h"
#include "exti.h"
#include "uart_motor.h"
#include "oled.h"
#include "pid.h"
#include "pid_move.h"
#include "fun.h"
#include "gpio.h"
#include "time.h"
#include "uartcv.h"
#include "key.h"
#include "wit_c_sdk.h"
#include "Emm_V5.h"
#include "usart.h"

extern int ready_flag;
extern char string[128];
extern volatile uint64_t uwTick;




#endif


