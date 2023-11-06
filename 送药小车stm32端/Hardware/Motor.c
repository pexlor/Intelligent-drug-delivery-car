#include "stm32f10x.h"                  // Device header
#include "Motor.h"
/*
TB6612
STBY 
PWMA PWMB 

BIN1 BIN2
AIN1 AIN2
1    0		正转		
0    1	  反转
0    0		停止
*/
#define PWMA_Pin GPIO_Pin_0
#define PWMB_Pin GPIO_Pin_1
#define PWM_Port GPIOA

#define AIN1 GPIO_Pin_11
#define AIN2 GPIO_Pin_12
#define BIN1 GPIO_Pin_12
#define BIN2 GPIO_Pin_13
#define BIN_Port GPIOB
#define AIN_Port GPIOA

#define DeedLine 0
#define MaxSpeed 10000
int MotorA=0;
int MotorB=0;

void Motor_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=PWMA_Pin|PWMB_Pin;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(PWM_Port,&GPIO_InitStruct);//TIME2 CH12
	
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=10000-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=1-1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse=0;
	TIM_OC1Init(TIM2,&TIM_OCInitStruct);
	TIM_OC2Init(TIM2,&TIM_OCInitStruct);

	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=AIN1|AIN2|BIN2|BIN1;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(BIN_Port,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin=AIN1|AIN2;
	GPIO_Init(AIN_Port,&GPIO_InitStruct);
	
	GPIO_WriteBit(BIN_Port, AIN1, (BitAction) 0);
	GPIO_WriteBit(BIN_Port, AIN2, (BitAction) 0);
	GPIO_WriteBit(BIN_Port, BIN1, (BitAction) 0);
	GPIO_WriteBit(BIN_Port, BIN2, (BitAction) 0);//开始时电机不转
}

/*设置PWM的比较值
	参数：PWMA，PWMB
				Mode:Forward_Turn 1//正转模式
						 Reverse 2//反转模式
						 Stop//停止模式
*/
void SetMotor(int16_t PWMA,int16_t PWMB){
	if(PWMA>0){
		GPIO_WriteBit(AIN_Port, AIN1, (BitAction) 0);
		GPIO_WriteBit(AIN_Port, AIN2, (BitAction) 1);
	}else if(PWMA<0){
		GPIO_WriteBit(AIN_Port, AIN1, (BitAction) 1);
		GPIO_WriteBit(AIN_Port, AIN2, (BitAction) 0);
	}else{
		GPIO_WriteBit(AIN_Port, AIN1, (BitAction) 0);
		GPIO_WriteBit(AIN_Port, AIN2, (BitAction) 0);
	}
	
	if(PWMB>0){
		GPIO_WriteBit(BIN_Port, BIN1, (BitAction) 0);
		GPIO_WriteBit(BIN_Port, BIN2, (BitAction) 1);
	}else if(PWMB<0){
		GPIO_WriteBit(BIN_Port, BIN1, (BitAction) 1);
		GPIO_WriteBit(BIN_Port, BIN2, (BitAction) 0);
	}else{
		GPIO_WriteBit(BIN_Port, BIN1, (BitAction) 0);
		GPIO_WriteBit(BIN_Port, BIN2, (BitAction) 0);
	}
	TIM_SetCompare1(TIM2,myabs(PWMA)+DeedLine);
	TIM_SetCompare2(TIM2,myabs(PWMB)+DeedLine);
}

uint16_t myabs(int16_t num){
	uint16_t temp=num;
	if(num<0){
		temp=-num;
	}
	if(temp>MaxSpeed){
		return MaxSpeed;
	}
	return temp;
}





