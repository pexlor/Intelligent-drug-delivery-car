#ifndef __CONTROL_H
#define __CONTROL_H
#define Wheel_R  0.05
typedef struct PIDST{
	float POSITION_KP;//位置环
	float POSITION_KI;
	float POSITION_KD;
	float POSITION_Error;
	float POSITION_ErrorLast;
	float POSITION_Integral;
	float POSITION_Target_Val;
	float POSITION_Actul_Val;
	int POSITION_Out;
	
	float LOCITION_KP;//方向环
	float LOCITION_KI;
	float LOCITION_KD;
	float LOCITION_Error;
	float LOCITION_ErrorLast;
	float LOCITION_Integral;
	float LOCITION_Target_Val;
	float LOCITION_Actul_Val;
	int LOCITION_Out;
	
	float Velocity_KP;//速度环
	float Velocity_KI;
	float Velocity_KD;
	float Velocity_Error;
	float Velocity_ErrorLast;
	float Velocity_Integral;
	float Velocity_Target_Val;
	float Velocity_Actul_Val;
	int Velocity_Out;
	
}PID_Struct;
extern PID_Struct PID;
extern PID_Struct PIDB;
int LocationRing_PID_Realize(float actual_value);

int VelocityRing_PID_Realize(float actual_value,float Target_Val,PID_Struct* PIDS);
int	PositionRing_PID_Realize(int actual_value,int Target_val);
void PID_Param_Init(void);
#endif
