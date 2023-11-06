#include "stm32f10x.h"                  // Device header

uint16_t currentadc[5];
void AD_Init(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_InitTypeDef DMA_InitStruct;
	
	DMA_InitStruct.DMA_BufferSize=5;
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;
	
	DMA_InitStruct.DMA_MemoryBaseAddr=(uint32_t)currentadc;
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;
	
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority=DMA_Priority_Medium;
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);
	
	//DMA_Cmd(DMA1_Channel1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,2,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,3,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,4,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,5,ADC_SampleTime_55Cycles5);
	
	
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel=5;
	ADC_InitStruct.ADC_ScanConvMode=ENABLE;
	ADC_Init(ADC1,&ADC_InitStruct);
	
	ADC_Cmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1,ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1)==SET);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1)==SET);
	
	//ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	DMA_Cmd(DMA1_Channel1,ENABLE);
}


void GetAdc(void){
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
}
