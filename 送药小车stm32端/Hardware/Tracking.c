#include "stm32f10x.h"                  // Device header

#define S1 GPIO_Pin_2
#define S2 GPIO_Pin_11
#define S3 GPIO_Pin_4
#define S4 GPIO_Pin_12
#define S5 GPIO_Pin_6
void Tracking_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;//下拉输入
	GPIO_InitStruct.GPIO_Pin=S1|S2|S3;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin=S4|S5;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	
}
void GetState(uint8_t state[]){
	state[0]=GPIO_ReadInputDataBit(GPIOA,S1);
	state[1]=GPIO_ReadInputDataBit(GPIOA,S2);
	state[2]=GPIO_ReadInputDataBit(GPIOA,S3);
	state[3]=GPIO_ReadInputDataBit(GPIOA,S4);
	state[4]=GPIO_ReadInputDataBit(GPIOA,S5);
}
