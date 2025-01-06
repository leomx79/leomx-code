#ifndef  __PWM_H
#define  __PWM_H
#include "stm32f10x.h"
 
void PWM_Init(void);
void PWM_SetCompare(uint16_t Compare1,uint16_t Compare2,uint16_t Compare3,uint16_t Compare4);
	
 
#endif
