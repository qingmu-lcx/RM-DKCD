#ifndef _DBUS_CONTROL_H_
#define _DBUS_CONTROL_H_

#include "main.h"

enum chassis_mode
{
	chassis_follow_normol = 0,//0
  chassis_360,							//1
  chassis_wait,   					//2
};

//获取机器人模式 //底盘跟随0 	//小陀螺1
uint8_t Read_Robot_mode(void);

//获取底盘控制指令 获取控制的方向
void Read_chassis_opt(float *vx,float *vy);

//获取云台控制指令
void Read_gimbal_opt(float *pitch, float *yaw);


#endif
