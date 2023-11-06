#include "stm32f10x.h"// Device header
#include "Motor.h"
#include "OLED.h"
#include "Encoder.h"
#include "HC-SR04.h"
#include "Tracking.h"
#include "Serial.h"
#include "Delay.h"
#include "key.h"
#include "control.h"
#include "ADC.h"
#include <stdio.h>

#define BaseSpeed 3500
typedef struct Return_PD{
	uint8_t ReturnStack[5];
	uint8_t topIndex;
}ReturnPD;

ReturnPD PD;
int MinAdc[]={2000,1500,1750,1700,2100};
int MaxAdc[]={3300,3400,3500,3900,2700};

uint64_t distanceA;//运行总距离
uint64_t distanceB;

int64_t EncoderA;//周期运行总距离
int64_t EncoderB;

int64_t temp_target_EncoderA;//用于跳过十字路口
int64_t temp_target_EncoderB;

int64_t stop_target_EncoderA;//用于辅助停车
int64_t stop_target_EncoderB;
float truck;

int PWMA;
int PWMB;
 
uint8_t corassNum=0;

uint8_t state[5],RxData,keyNum;

uint8_t identify_Num,temp_Num;

uint8_t stop_flag=0;//停止标志位，置1,停止转动
uint8_t turn_flag=0;//开始转向标志位，置1，表示要进行转向
uint8_t turn_where_flag=0;

uint8_t identify_flag=0;//识别成功标志位，置1表示数字识别成功
uint8_t count_flag=0;//开始计时标志位，置1，表示开始计算距离
uint8_t temp_identifu_flag=0;//中途数字识别标志

uint8_t run_flag=0;//开始运动标志
uint8_t stop_run_flag=0;//开始运动标志

uint8_t correct_location_flag=0;//0速度，寻迹，摆正

uint8_t turn_right_flag=0;
uint8_t turn_left_flag=0;
uint8_t go_flag=0;



int16_t disA=0,disB=0;
int Tracking_In(void);
void turn_left(void);
void turn_right(void);
void turn_around(void);
void find_1(void);
void find_2(void);
void go_order(void);
void go_mideld_right(void);
void go_mideld_left(void);
void head_right(void);
void head_left(void);

int main(){
	Motor_Init();
	OLED_Init();
	Encoder_Init();
	AD_Init();
	Serial_Init();
	Key_Init();
	OLED_ShowString(3, 1, "START!");
	TIM_Cmd(TIM2, ENABLE);
	PID_Param_Init();
	PD.topIndex=0;
	OLED_ShowNum(4,1,PD.topIndex,1);
	//while(identify_flag==0);
	
	OLED_ShowString(1, 1, "Ready!");
	OLED_ShowString(2, 1, "NUM:");
	OLED_ShowChar(2,5,identify_Num);
	
	temp_target_EncoderA=13000;
	stop_flag=1;
	turn_flag=1;
	run_flag=0;
	SetMotor(9900,9900);
	//TIM_Cmd(TIM1, ENABLE);
	/*
	while(keyNum==0){
				keyNum=Key_GetNum();
				OLED_ShowNum(4,1,keyNum,1);
			
	}
	
	stop_flag=0;
	turn_flag=0;
	run_flag=1;
	for(;;){
	//	go_order();
		//head_right();
			switch (identify_Num)
			{
				case '1':
					find_1();
					break;
				case '2':
					find_2();
					break;
				default:
					go_order();
					break;
					
			}
			
		}
	 */
}
void USART1_IRQHandler (void){
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET){
		RxData=USART_ReceiveData(USART1);
		if(identify_flag==0){
			identify_Num=RxData;
			identify_flag=1;
		}else if(identify_flag==1&&run_flag==1&&temp_identifu_flag==1){
			temp_Num=RxData;
			stop_flag=1;
		}
		//OLED_ShowChar(2,1,RxData);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

void TIM1_UP_IRQHandler (void){
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)==SET){
		//Serial_Printf("%d,%d,%d,%d,%d\n",currentadc[0],currentadc[1],currentadc[2],currentadc[3],currentadc[4]);
		
		if(count_flag==1){//开启计数功能 执行
			Encoder_A=Encoder_Get_A();
			Encoder_B=Encoder_Get_B();
			EncoderA+=Encoder_A;
			EncoderB+=Encoder_B;
			//BaseSpeed=(PositionRing_PID_Realize(EncoderA,temp_target_EncoderA)+PositionRing_PID_Realize(EncoderB,temp_target_EncoderB))/2;
			if(EncoderA>=temp_target_EncoderA-30||EncoderB>=temp_target_EncoderB-30){
				EncoderA=0;
				EncoderB=0;
				count_flag=0;
				stop_flag=1;
			}
		}
		
		if(correct_location_flag==1){
			if(count_flag==0){
				Encoder_A=Encoder_Get_A();
				Encoder_B=Encoder_Get_B();
			}
			GetAdc();
			Tracking_In();
			SetMotor((PWMA+VelocityRing_PID_Realize(Encoder_A,PWMA*0.0103,&PID)),(PWMB+VelocityRing_PID_Realize(Encoder_B,PWMB*0.0103,&PIDB)));
		}
		
		if((stop_flag==0&&correct_location_flag==0&&turn_flag==0)){//寻迹状态时
			if(count_flag==0){
				Encoder_A=Encoder_Get_A();
				Encoder_B=Encoder_Get_B();
			}
			GetAdc();
			Tracking_In();
			SetMotor(PWMA+VelocityRing_PID_Realize(Encoder_A,PWMA*0.0103,&PID),PWMB+VelocityRing_PID_Realize(Encoder_B,PWMB*0.0103,&PIDB));
		}
		else if(turn_flag==0&&stop_flag==1&&correct_location_flag==0){//准备转弯前的停止
			if(count_flag==0){
				Encoder_A=Encoder_Get_A();
				Encoder_B=Encoder_Get_B();
			}
			SetMotor(VelocityRing_PID_Realize(Encoder_A,0,&PID),VelocityRing_PID_Realize(Encoder_B,0,&PIDB));
		}
		
		Serial_Printf("%d,%d\n",Encoder_A,Encoder_B);
		
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}

/*
	寻迹函数
	功能：对获得到的灰度传感器值进行归一化处理，并根据小车目前的状况进行利用PID进行PWM值的计算
*/
int Tracking_In(void){
	static float NormalData[5];
		for( int i=0;i<5;i++){
			NormalData[i]=(float)(currentadc[i]-MinAdc[i])/(float)(MaxAdc[i]-MinAdc[i]);//线性归一化处理
			if(NormalData[i]<0){
				NormalData[i]=0;
			}
			if(NormalData[i]>1){
				NormalData[i]=1;
			}
		}
		if(currentadc[0]<MinAdc[0]||currentadc[4]<MinAdc[4]){//是否遇到十字路口
				if(EncoderA<temp_target_EncoderA&&EncoderB<temp_target_EncoderB&&identify_Num>'2'&&count_flag==1){//在除1,2的其他情况下，通过第一个十字路口
					stop_flag=0;
					PWMA=BaseSpeed;
					PWMB=BaseSpeed;
					return 1;
				}else{
					stop_flag=1;
					return 0;
				}
		}
		else{
			if(correct_location_flag==1){
				truck = (NormalData[0]*-4+NormalData[1]*-2 + NormalData[3]*2+NormalData[4]*4);
			}else{
				truck = (NormalData[1]*-2 + NormalData[3]*2);
			}
			
			int PWM = LocationRing_PID_Realize(truck);	
			if(correct_location_flag==1){
				PWMA=PWM;
				PWMB=-PWM;
			}else{
				PWMA = BaseSpeed+PWM;
				PWMB = BaseSpeed-PWM;
			}
		//	Serial_Printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",currentadc[0],currentadc[1],currentadc[2],currentadc[3],currentadc[4],PWMA,PWMB,Encoder_A,Encoder_B);
			stop_flag=0;
			return 1;
		}
}
/*等待校准函数*/
void Calibration_Start(void){
		correct_location_flag=1;
		Delay_ms(150);
		correct_location_flag=0;
}
/*
	左转90度

*/
void turn_left(void){
	
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	SetMotor(0,2500);
	while(!(disB>2400)){
		disB=Encoder_Get_B_R();
	}
	disA=0;disB=0;
	SetMotor(0,0);
}
/*
	右转90度

*/
void turn_right(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	SetMotor(2500,0);
	while(!(disA>2400)){

		disA=Encoder_Get_A_R();
	}
	disA=0;disB=0;
	SetMotor(0,0);
}

/*
	调头
*/
void turn_around(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	SetMotor(-2300,2300);
	while(!(disA<-1900&&disB>1900)){
		disA=Encoder_Get_A_R();
		disB=Encoder_Get_B_R();
		Serial_Printf("%d,%d\n",disA,disB);
		Delay_ms(5);
	}
	disA=0;disB=0;
	SetMotor(0,0);
}

/*右偏头*/
void head_right(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	temp_identifu_flag=1;
	SetMotor(2000,0);
	while(!(disA>400)){
		disA=Encoder_Get_A_R();
	}
	SetMotor(0,0);
	disA=0;disB=0;
	Delay_ms(500);
	temp_identifu_flag=0;
	
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	SetMotor(-2000,0);
	disA=0;disB=0;
	while(!(disA<-400)){
		disA=Encoder_Get_A_R();
	}
	disA=0;disB=0;
	SetMotor(0,0);
}

/*左偏头*/
void head_left(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	temp_identifu_flag=1;
	SetMotor(0,2000);
	while(!(disB>400)){
		disB=Encoder_Get_B_R();
	}
	disA=0;disB=0;
	SetMotor(0,0);
	Delay_ms(500);//延时等待数字检测
	temp_identifu_flag=0;
	
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	disA=0;disB=0;
	SetMotor(0,-2000);
	while(!(disB<-400)){
		disB=Encoder_Get_B_R();
	}
	disA=0;disB=0;
	SetMotor(0,0);
}

void head_left_half(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	temp_identifu_flag=1;
	SetMotor(0,1500);
	while(!(disB>250)){
		disB=Encoder_Get_B_R();
		Serial_Printf("%d,%d\n",disA,disB);
		Delay_ms(5);
	}
	disA=0;disB=0;
	SetMotor(0,0);
	Delay_ms(2000);//延时等待数字检测
	temp_identifu_flag=0;
}

void head_left_GO(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	disA=0;disB=0;
	SetMotor(0,-1500);
	while(!(disB<-250)){
		disB=Encoder_Get_B_R();
		Serial_Printf("%d,%d\n",disA,disB);
		Delay_ms(5);
	}
	disA=0;disB=0;
	SetMotor(0,0);
}
void head_right_half(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	temp_identifu_flag=1;
	SetMotor(1500,0);
	while(!(disA>250)){
		disA=Encoder_Get_A_R();
		Serial_Printf("%d,%d\n",disA,disB);
		Delay_ms(5);
	}
	SetMotor(0,0);
	disA=0;disB=0;
	Delay_ms(2000);
	temp_identifu_flag=0;
}

void head_right_go(void){
	TIM3 -> CNT=0;TIM4 -> CNT=0;
	SetMotor(-1500,0);
	disA=0;disB=0;
	while(!(disA<-250)){
		disA=Encoder_Get_A_R();
		Serial_Printf("%d,%d\n",disA,disB);
		Delay_ms(5);
	}
	disA=0;disB=0;
	SetMotor(0,0);
}

/*返回函数*/
void Return(void){
	turn_flag=1;
	//Serial_Printf("%d",PD.ReturnStack[PD.topIndex-1]);
	
	turn_around();
	Calibration_Start();
	
	Delay_ms(50);
	
	if(PD.topIndex==1){
		temp_target_EncoderA=17500;
		temp_target_EncoderB=17500;
	}else{
		temp_target_EncoderA=19500;
		temp_target_EncoderB=19500;
	}
	
	while((PD.topIndex)>=1){
		PD.topIndex-=1;
		stop_flag=0;
		turn_flag=0;
		while(stop_flag==0);
		Delay_ms(50);
		turn_flag=1;
		if(PD.ReturnStack[PD.topIndex]==1){
			 turn_right();
				Calibration_Start();
		}else if(PD.ReturnStack[PD.topIndex]==2){
				turn_left();
				Calibration_Start();
		}else{
				stop_flag=0;
				turn_flag=0;
		}
	}
	OLED_ShowString(4,1,"RETURN");
	stop_flag=0;
	turn_flag=0;
	count_flag=1;
	while(stop_flag==0);
	Delay_ms(50);
	count_flag=0;
	turn_flag=1;
	turn_around();
	Calibration_Start();
}


/*
	任务1，到近端左病房
*/
void find_1(void){//近端病房
	while(stop_flag==0);
	
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_left();
				turn_flag=0;
		
				Calibration_Start();
		
				stop_flag=0;

	}

	while(stop_flag==0);
	
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_around();
		
				turn_flag=0;
		
				Calibration_Start();
		
		
				stop_flag=0;
	}
	while(stop_flag==0);
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_right();
				turn_flag=0;
				Calibration_Start();
				stop_flag=0;
	
	}
	while(stop_flag==0);
	
	turn_flag=0;
	Delay_ms(50);
	turn_flag=1;
	
	turn_around();
	
}

/*任务2：到近端右病房*/
void find_2(void){//近端病房
	while(stop_flag==0);
	
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_right();
				turn_flag=0;
		
				Calibration_Start();
				stop_flag=0;

	}

	while(stop_flag==0);
	
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_around();
				turn_flag=0;
		
				Calibration_Start();
		

				stop_flag=0;
	}
	while(stop_flag==0);
	if(stop_flag==1){
				Delay_ms(100);
				turn_flag=1;
				turn_left();
				turn_flag=0;
		
				Calibration_Start();
		
				stop_flag=0;
		
	}
	
	while(stop_flag==0);
	
	turn_flag=0;
	Delay_ms(50);
	turn_flag=1;
	turn_around();
	
}

void go_order(void){
	temp_target_EncoderA=15200;
	temp_target_EncoderB=15200;
	
	count_flag=1;
	while(stop_flag==0);//走到第二个十字路口前停下
	count_flag=0;
	
	if(stop_flag==1){//第一个十字路口的识别
				Delay_ms(100);//等待完全停止
				turn_flag=1;
		
				Calibration_Start();
		
				head_left();//左摆并等待识别
		
				if(temp_Num==identify_Num){
					turn_left_flag=1;
					PD.ReturnStack[PD.topIndex]=1;
					PD.topIndex++;
					
				}else{
					head_right();
					
					if(temp_Num==identify_Num){
						PD.ReturnStack[PD.topIndex]=2;
						PD.topIndex++;
						turn_right_flag=1;
					}else{
						go_flag=1;
						PD.ReturnStack[PD.topIndex]=0;
						PD.topIndex++;
					}
				}
	}
	
	stop_flag=0;
	turn_flag=0;
	
	while(stop_flag==0);//走到第一个十字路口
	
	
	if(stop_flag==1){
		Delay_ms(100);
		turn_flag=1;
		
		
		if(turn_right_flag==1){//开始判断
			turn_right();
			Calibration_Start();
			turn_right_flag=0;
			
		}else if(turn_left_flag==1){
			turn_left();
			Calibration_Start();
			turn_left_flag=0;
			
		}else if(go_flag==1){ //直走
			temp_target_EncoderA=7000;
			temp_target_EncoderB=7000;
			
			count_flag=1;
			
			stop_flag=0;
			turn_flag=0;
			go_flag=0;
			
			while(stop_flag==0);//到第三个十字路口前停下
	
			if(stop_flag==1){//开始识别
				Delay_ms(100);
				turn_flag=1;
				
				head_left_half();
				
				if(temp_Num==identify_Num){
					head_left_GO();
					turn_left_flag=1;
					PD.ReturnStack[PD.topIndex]=1;
					PD.topIndex++;
				}else{
					head_left_half();
					if(temp_Num==identify_Num){
						
						head_left_GO();
						head_left_GO();
						
						turn_left_flag=1;
						PD.ReturnStack[PD.topIndex]=1;
						PD.topIndex++;
					}else{
						head_left_GO();
						head_left_GO();
						
						head_right_half();
						if(temp_Num==identify_Num){
							head_right_go();
							turn_right_flag=1;
							PD.ReturnStack[PD.topIndex]=2;
							PD.topIndex++;
						}else{
							head_right_half();
							if(temp_Num==identify_Num){
								head_right_go();
								head_right_go();
								turn_right_flag=1;
								PD.ReturnStack[PD.topIndex]=2;
								PD.topIndex++;
							}else{
								OLED_ShowString(1,1,"ERROR!!!");
							}
						}
					}
				}
			}//识别完毕
			
				stop_flag=0;
				turn_flag=0;
				
				while(stop_flag==0);//到第三个十字路口
				
				if(stop_flag==1){
					Delay_ms(100);
					turn_flag=1;

					if(turn_right_flag==1){//开始判断
						turn_right();
						Calibration_Start();
						turn_right_flag=0;
					}else if(turn_left_flag==1){
						turn_left();
						Calibration_Start();
						turn_left_flag=0;
					}
				}
				
				turn_left_flag=0;
				turn_right_flag=0;
				
				temp_target_EncoderA=6000;
				temp_target_EncoderB=6000;
				stop_flag=0;
				turn_flag=0;
				count_flag=1;
				
				while(stop_flag==0);//直走到最后一个十字路口
				
				
				if(stop_flag==1){//最后一个路口 开始识别
					Delay_ms(100);
					turn_flag=1;
					head_left();
					if(temp_Num==identify_Num){
						turn_left_flag=1;
						PD.ReturnStack[PD.topIndex]=1;
						PD.topIndex++;
					}else{
						head_right();
						if(temp_Num==identify_Num){
							PD.ReturnStack[PD.topIndex]=2;
							PD.topIndex++;
							turn_right_flag=1;
						}
					}
				}
				
				stop_flag=0;
				turn_flag=0;
				while(stop_flag==0);
				
				if(stop_flag==1){//到最后一个路口，转弯
					
					Delay_ms(100);
					turn_flag=1;
					 
					if(turn_right_flag==1){//开始判断
						turn_right();
						Calibration_Start();
						turn_right_flag=0;
					}else if(turn_left_flag==1){
						turn_left();
						Calibration_Start();
						turn_left_flag=0;
					}
				}
				stop_flag=0;
				turn_flag=0;
				while(stop_flag==0);
		}
	}
	stop_flag=0;
	turn_flag=0;
	Delay_ms(2000);
	Return();
}

