#include <stm32f4xx.h>
#include "key.h"

int key_flag_1;
int key_flag_2;
int key_flag_1_old;
int key_flag_2_old;
int key1 = 0;
int key2 = 0;

void key_proc(void)
{
	key_flag_1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
	key_flag_2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2);
	
	if(key_flag_1==0&&key_flag_1_old==1) key1++;
	if(key_flag_2==0&&key_flag_2_old==1) key2++;
	
	key_flag_1_old = key_flag_1;
	key_flag_2_old = key_flag_2;
}


