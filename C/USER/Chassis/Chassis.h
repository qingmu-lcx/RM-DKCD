#ifndef _Chassis_H_
#define _Chassis_H_

#include "main.h"

//底盘电机速度环PID
#define M3508_SPEED_PID_KP 				3.0f 			//3.0f
#define M3508_SPEED_PID_KI 				0.002f  	//0.002f
#define M3508_SPEED_PID_KD 				0.0f			//0.0f
#define M3508_SPEED_PID_MAX_IOUT 	4000.0f 	//2000.0f
#define M3508_SPEED_PID_MAX_OUT 	16000.0f	//16000.0f

void Chassis_init(void);
void Chassis_out(float I1, float I2, float I3, float I4);
void Chassis_sleep(void);
void MecanumCalculate(float vx,float vy,float wz);
void MecanumCalculate360(float VX_x,float VX_y,float VY_x,float VY_y,float wz);
void Chassis_move(void);

#endif


