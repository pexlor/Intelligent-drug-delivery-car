#include "stm32f10x.h"                  // Device header
struct PID_arg{
	float Speed_p;
	float Speed_i;
	float Speed_d;
};
struct PID_arg Arg={
	.Speed_p=-100,
	.Speed_i=-0.1,
	.Speed_d=-1.5,
};

uint16_t GetPositionPID(uint16_t Encoder_Speed,uint16_t Target_Speed){
	uint16_t error;
	uint16_t PID_Value;
	static uint16_t last_error;
	error=Encoder_Speed-Target_Speed;
	last_error=last_error+error;
	
}