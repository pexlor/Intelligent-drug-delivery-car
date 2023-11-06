#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#define  Trig	GPIO_Pin_10
#define  Echo GPIO_Pin_11
void HC_SR_Init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPD;//下拉输入
	GPIO_InitStruct.GPIO_Pin=Echo;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStruct.GPIO_Pin=Trig;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
}
void Trig_Start(void){
	GPIO_WriteBit(GPIOB,Trig, (BitAction)0);
	GPIO_WriteBit(GPIOB,Trig, (BitAction)1);
	Delay_us(20);
	GPIO_WriteBit(GPIOB,Trig, (BitAction)0);
}
float getDistance(void){
	uint16_t time=0;
	int distance=0;
	int j=0;
	for(int i=0;i<5;i++){
		TIM_SetCounter(TIM1,0x0000);
		Trig_Start();
		while(GPIO_ReadInputDataBit(GPIOB,Echo)==Bit_RESET){
			j++;
			if(j>1000000  ){return 0;}
		}
		TIM_Cmd(TIM1, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOB,Echo)==Bit_SET);
		TIM_Cmd(TIM1, DISABLE);
		time=TIM_GetCounter(TIM1);
		distance+=(int)((float)time*1.7);
		TIM_SetCounter(TIM1,0x0000);
		TIM_Cmd(TIM1, ENABLE);
		Delay_ms(5);
	}
	return distance/5*0.0857-29.571;
}
