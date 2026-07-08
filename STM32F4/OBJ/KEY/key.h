#ifndef __KEY_H
#define __KEY_H
#include <stm32f4xx.h>

extern int key1;
extern int key2;
extern int key_flag_1;
extern int key_flag_2;
extern int key_flag_1_old;
extern int key_flag_2_old;

void key_proc(void);
	
#endif

