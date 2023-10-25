#ifndef FIRE_H_
#define FIRE_H_

#include "main.h"

#define M2006_Angle_Pid_KP 				0.4f
#define M2006_Angle_Pid_KI 				0.0f
#define M2006_Angle_Pid_KD 				0.0f
#define M2006_Angle_Pid_Max_IOUT 	0.0f
#define M2006_Angle_Pid_Max_OUT 	2000.0f

#define M2006_Speed_Pid_KP 				4.0f
#define M2006_Speed_Pid_KI 				0.0f
#define M2006_Speed_Pid_KD 				0.0f
#define M2006_Speed_Pid_Max_IOUT 	0.0f
#define M2006_Speed_Pid_Max_OUT 	10000.0f

void Fire_Init(void);
void Fire_Angle_Out(float except);
void Fire_Speed_Out(float except);
void Fire_Sleep(void);
void Fire_Control(void);

#endif


