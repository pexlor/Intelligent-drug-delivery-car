#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"
extern uint16_t currentadc[5];
void AD_Init(void);
void GetAdc(void);
#endif
