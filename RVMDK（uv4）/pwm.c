#include "pwm.h"


 
void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    TIM_InternalClockConfig(TIM3);//连接内部时钟  打通第一个通道
	
	TIM_TimeBaseInitTypeDef Timebasestructer;
	Timebasestructer.TIM_ClockDivision=TIM_CKD_DIV1 ;    	// 时钟为72MHz   选择1  为部分频  即72 000 000 Hz
	Timebasestructer.TIM_CounterMode=TIM_CounterMode_Up; //  计数模式   为向上计数  即从0开始
	Timebasestructer.TIM_Period=10000 -1;                		//   ARR    自动重装值 当计数到达时  就触发中断
	Timebasestructer.TIM_Prescaler=7200 -1;             	 //   PSC     预分频     将72MHz 频率设置为10k Hz
//	Timebasestructer.TIM_RepetitionCounter=0;            
	//该时基单元    一秒    发生一次中断
	TIM_TimeBaseInit(TIM3, &Timebasestructer);
 
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//配置默认项
	
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);//开启四个PWM通道
	
    TIM_Cmd(TIM3, ENABLE);                  //打开TIIM3
 
}
 
void PWM_SetCompare(uint16_t Compare1,uint16_t Compare2,uint16_t Compare3,uint16_t Compare4)
{
	TIM_SetCompare1(TIM3, Compare1);
	TIM_SetCompare2(TIM3, Compare2);
	TIM_SetCompare3(TIM3, Compare3);
	TIM_SetCompare4(TIM3, Compare4);
}


