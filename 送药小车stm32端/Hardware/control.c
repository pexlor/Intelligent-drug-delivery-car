#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "control.h"

float error;
PID_Struct PID;
PID_Struct PIDB;
void PID_Param_Init(void){
	PID.POSITION_KP=0.5;//位置环
	PID.POSITION_KI=0;
	PID.POSITION_KD=0;
	PID.POSITION_Error=0;
	PID.POSITION_ErrorLast=0;
	PID.POSITION_Integral=0;
	PID.POSITION_Target_Val=0;
	PID.POSITION_Actul_Val=0;
	PID.POSITION_Out=0;
	
	PID.LOCITION_KP=500;//方向环
	PID.LOCITION_KI=0;
	PID.LOCITION_KD=600;
	PID.LOCITION_Error=0;
	PID.LOCITION_ErrorLast=0;
	PID.LOCITION_Integral=0;
	PID.LOCITION_Target_Val=0;
	PID.LOCITION_Actul_Val=0;
	PID.LOCITION_Out=0;
	
	PID.Velocity_KP=50;//速度环
	PID.Velocity_KI=0;
	PID.Velocity_KD=5;
	PID.Velocity_Error=0;
	PID.Velocity_ErrorLast=0;
	PID.Velocity_Integral=0;
	PID.Velocity_Target_Val=0;
	PID.Velocity_Actul_Val=0;
	PID.Velocity_Out=0;
	
	PIDB.POSITION_KP=45;//位置环
	PIDB.POSITION_KI=0;
	PIDB.POSITION_KD=6;
	PIDB.POSITION_Error=0;
	PIDB.POSITION_ErrorLast=0;
	PIDB.POSITION_Integral=0;
	PIDB.POSITION_Target_Val=0;
	PIDB.POSITION_Actul_Val=0;
	PIDB.POSITION_Out=0;
	
	PIDB.LOCITION_KP=600;//方向环
	PIDB.LOCITION_KI=6;
	PIDB.LOCITION_KD=60;
	PIDB.LOCITION_Error=0;
	PIDB.LOCITION_ErrorLast=0;
	PIDB.LOCITION_Integral=0;
	PIDB.LOCITION_Target_Val=0;
	PIDB.LOCITION_Actul_Val=0;
	PIDB.LOCITION_Out=0;
	
	PIDB.Velocity_KP=45;//速度环
	PIDB.Velocity_KI=0;
	PIDB.Velocity_KD=6;
	PIDB.Velocity_Error=0;
	PIDB.Velocity_ErrorLast=0;
	PIDB.Velocity_Integral=0;
	PIDB.Velocity_Target_Val=0;
	PIDB.Velocity_Actul_Val=0;
	PIDB.Velocity_Out=0;
	
}


int	PositionRing_PID_Realize(int actual_value,int Target_val){//位置环
	PID.POSITION_Target_Val=Target_val;
	PID.POSITION_Error=PID.POSITION_Target_Val-actual_value;//计算偏差
	
	//PID.POSITION_Integral+=PID.POSITION_Error;//累计偏差
	
	if(PID.POSITION_Error<50&&PID.POSITION_Error>-50){
		PID.POSITION_Error=0;
	}
	
	PID.POSITION_Out=PID.POSITION_KP*PID.POSITION_Error+PID.POSITION_KD*(PID.POSITION_Error-PID.POSITION_ErrorLast);
	
	PID.POSITION_ErrorLast=PID.POSITION_Error;
	if(PID.POSITION_Out>2500){
		PID.POSITION_Out=2500;
	}else if(PID.POSITION_Out<0){
		PID.POSITION_Out=0;
	}
	
	return PID.POSITION_Out;
}

int	LocationRing_PID_Realize(float actual_value){//方向环
	PID.LOCITION_Error=PID.LOCITION_Target_Val-actual_value;//计算偏差
	
	PID.LOCITION_Integral+=PID.LOCITION_Error;//累计偏差
	if(PID.LOCITION_Error<0.15&&PID.LOCITION_Error>-0.15){
		PID.LOCITION_Error=0;
	}
	
	if(PID.LOCITION_Integral>100){
		PID.LOCITION_Integral=100;
	}else if(PID.LOCITION_Integral<-100){
		PID.LOCITION_Integral=-100;
	}
	
	PID.LOCITION_Out=PID.LOCITION_KP*PID.LOCITION_Error+PID.LOCITION_Integral*PID.LOCITION_KI+PID.LOCITION_KD*(PID.LOCITION_Error-PID.LOCITION_ErrorLast);
	
	PID.LOCITION_ErrorLast=PID.LOCITION_Error;
	
	return PID.LOCITION_Out;
}

int VelocityRing_PID_Realize(float actual_value,float Target_Val,PID_Struct* PIDS){//速度环
	PIDS->Velocity_Target_Val=Target_Val;
	PIDS->Velocity_Error=PIDS->Velocity_Target_Val-actual_value;//计算偏差
	
	PIDS->Velocity_Integral+=PIDS->Velocity_Error;//累计偏差
	
	if(PIDS->Velocity_Integral>1000){
		PIDS->Velocity_Integral=1000;
	}else if(PIDS->Velocity_Integral<-1000){
		PIDS->Velocity_Integral=-1000;
	}
	
	PIDS->Velocity_Out=PIDS->Velocity_KP*PIDS->Velocity_Error+PIDS->Velocity_Integral*PIDS->Velocity_KI+PIDS->Velocity_KD*(PIDS->Velocity_Error-PIDS->Velocity_ErrorLast);
	
	PIDS->Velocity_ErrorLast=PIDS->Velocity_Error;
	return PIDS->Velocity_Out;
}

