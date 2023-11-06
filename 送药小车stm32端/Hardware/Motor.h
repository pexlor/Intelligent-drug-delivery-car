#ifndef __MOTOR_H_
#define __MOTOR_H_
extern int MotorA;
extern int MotorB;
void Motor_Init(void);
void SetMotor(int16_t PWMA,int16_t PWMB);
uint16_t myabs(int16_t num);
#endif
