#include "Shoot.h"
#include "User_Can.h"
#include "User_Dbus_Control.h"
#include "User_Kalman.h"
#include "User_Pid.h"
#include "User_Gpio.h"
#include "User_Dbus.h"
//#include "Judge.h"
#include "User_Time.h"
#include "Gimbal.h"
#include "User_Debug.h"
#include "chassis.h"

pid_type_def M3508_Speed_Pid_Left;
pid_type_def M3508_Speed_Pid_Right;

void Shoot_Init(void)
{
	static const float M3508_Speed_Pid_Left_3[3] = {M3508_Speed_Pid_Left_KP, M3508_Speed_Pid_Left_KI, M3508_Speed_Pid_Left_KD};
	static const float M3508_Speed_Pid_Right_3[3] = {M3508_Speed_Pid_Right_KP, M3508_Speed_Pid_Right_KI, M3508_Speed_Pid_Right_KD};
	
	PID_init(&M3508_Speed_Pid_Left,PID_POSITION, M3508_Speed_Pid_Left_3, M3508_Speed_Pid_Left_Max_IOUT, M3508_Speed_Pid_Left_Max_OUT);
	PID_init(&M3508_Speed_Pid_Right,PID_POSITION, M3508_Speed_Pid_Right_3, M3508_Speed_Pid_Right_Max_IOUT, M3508_Speed_Pid_Right_Max_OUT);
}

void Shoot_Speed_Out(float Left_except,float Right_except)//CAN1 id 1 左边  ; id 2 右边
{
	PID_calc(&M3508_Speed_Pid_Left, CAN1_Motor[0].Speed, Left_except);
	PID_calc(&M3508_Speed_Pid_Right, CAN1_Motor[1].Speed, Right_except);
	
	CAN1_OPerate_Motor(1,(short)M3508_Speed_Pid_Left.out);
	CAN1_OPerate_Motor(2,(short)M3508_Speed_Pid_Right.out);
	
	CAN1_to_Motor(0x200);
}

void Shoot_Sleep(void)
{
	//电流值清空
	CAN1_OPerate_Motor(1,0);
	CAN1_OPerate_Motor(2,0);
	//PID输出清空
	PID_clear(&M3508_Speed_Pid_Left);
	PID_clear(&M3508_Speed_Pid_Right);
}

uint8_t Shoot_Flag = 0;
uint8_t Magazine_Flag = 0;
void Shoot_Control(void)
{
	Robot_Grade.Shoot_Speed=4000;
	if(Read_Robot_mode() == chassis_wait)//待机模式
  {
		Laser_Off();//关闭激光
		Shoot_Speed_Out(0,0);
    Shoot_Sleep();
		Shoot_Flag = 0;
		
		if(RC.r_s == 3)//待机时遥控器控制弹舱盖
			Magazine(1150);
		else if(RC.r_s == 2)
			Magazine(2200);
		
    return;
  }
	else if(Read_Robot_mode() != chassis_wait)
	{
		if(RC.r && RC.last_r == 0)//按r开关弹舱盖
			Magazine_Flag =	~Magazine_Flag;
		
		if(Magazine_Flag)
			Magazine(2200);
		else
			Magazine(1150);
	}
	
	if((RC.r_s != 3 || RC.press_r == 1) &&  Shoot_Flag == 0)//开启摩擦轮
		Shoot_Flag = 1;
	
	else if(Read_Robot_mode() != chassis_wait && Shoot_Flag == 1)
	{
		Laser_On();//打开激光
		Shoot_Speed_Out(Robot_Grade.Shoot_Speed,-Robot_Grade.Shoot_Speed);//根据射速改摩擦轮转速
	}
}

