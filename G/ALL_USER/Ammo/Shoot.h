#ifndef SHOOT_H_
#define SHOOT_H_

#include "main.h"

#define M3508_Speed_Pid_Left_KP					8.0f//12.0
#define M3508_Speed_Pid_Left_KI					0.08f//0.02
#define M3508_Speed_Pid_Left_KD					0.0f//2.0
#define M3508_Speed_Pid_Left_Max_IOUT		160.0f//2500
#define M3508_Speed_Pid_Left_Max_OUT		16000.0f//15000

#define M3508_Speed_Pid_Right_KP				8.0f//60.0
#define M3508_Speed_Pid_Right_KI				0.08f
#define M3508_Speed_Pid_Right_KD				0.0f
#define M3508_Speed_Pid_Right_Max_IOUT	160.0f
#define M3508_Speed_Pid_Right_Max_OUT		16000.0f//15000

extern uint8_t Shoot_Flag;

void Shoot_Init(void);
void Shoot_Speed_Out(float Left_except,float Right_except);
void Shoot_Sleep(void);
void Shoot_Control(void);

#endif


