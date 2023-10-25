#include "Fire.h"
#include "User_Can.h"
#include "User_Dbus_Control.h"
#include "User_Kalman.h"
#include "User_Pid.h"
#include "User_Dbus.h"
#include "cmsis_os.h"
#include "Shoot.h"
//#include "Judge.h"
#include "PC_Link.h"
#include "User_Debug.h"
#include "Gimbal.h"
#include "chassis.h"

#define Fire_Speed  			-6000

#define Stepping_Angle		-36864			//=8192*36/8 * 3

pid_type_def M2006_Angle_Pid;
pid_type_def M2006_Speed_Pid;

void Fire_Init(void)
{
	static const float M2006_Angle_Pid_3[3] = {M2006_Angle_Pid_KP, M2006_Angle_Pid_KI, M2006_Angle_Pid_KD};
	static const float M2006_Speed_Pid_3[3] = {M2006_Speed_Pid_KP, M2006_Speed_Pid_KI, M2006_Speed_Pid_KD};
	
	PID_init(&M2006_Angle_Pid,PID_POSITION, M2006_Angle_Pid_3, M2006_Angle_Pid_Max_IOUT, M2006_Angle_Pid_Max_OUT);
	PID_init(&M2006_Speed_Pid,PID_POSITION, M2006_Speed_Pid_3, M2006_Speed_Pid_Max_IOUT, M2006_Speed_Pid_Max_OUT);
}

void Fire_Angle_Out(float except)
{
	PID_calc(&M2006_Angle_Pid, CAN1_Motor[7].Continuous_Angle, except);
	PID_calc(&M2006_Speed_Pid, CAN1_Motor[7].Speed, M2006_Angle_Pid.out);
	
	CAN1_OPerate_Motor(8,(short)M2006_Speed_Pid.out);
//	CAN1_to_Motor(0x1FF);
}
void Fire_Speed_Out(float except)
{
	PID_calc(&M2006_Speed_Pid, CAN1_Motor[7].Speed, except);
	
	CAN1_OPerate_Motor(8,(short)M2006_Speed_Pid.out);
	CAN1_to_Motor(0x1FF);
}

void Fire_Sleep(void)
{
	//电流值清空
	CAN1_OPerate_Motor(8,0);
	//PID输出清空
	PID_clear(&M2006_Angle_Pid);
	PID_clear(&M2006_Speed_Pid);
}

uint8_t Fire_Flag = 0;
uint8_t Fire_RC = 3;
uint16_t Fire_Count = 0;
int Fire_Expect_Angle = 0;
uint8_t Fire_Press_Flag = 0;
void Fire_Control(void)
{
	if((Read_Robot_mode() == chassis_wait) && (Shoot_Flag == 0) && (CAN1_Motor[0].Speed < (Robot_Grade.Shoot_Speed - 400)) && (CAN1_Motor[1].Speed > -(Robot_Grade.Shoot_Speed - 400)))//待机模式
  {
		Fire_Speed_Out(0);
    Fire_Sleep();
		Fire_RC = 2;
		Fire_Flag = 0;
		Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle;
    return;
  }
	
	if((Shoot_Flag == 1) && (Fire_Flag == 0) && (CAN1_Motor[0].Speed > (Robot_Grade.Shoot_Speed - 400)) && (CAN1_Motor[1].Speed < -(Robot_Grade.Shoot_Speed - 400)))
	{//判断摩擦轮开启并且转速达到设定转速后才能拨弹
		osDelay(500);
		Fire_Flag = 1;
	}
//	usart_printf("22\r\n");
	if((Read_Robot_mode() != chassis_wait) && (Fire_Flag == 1) && (CAN1_Motor[0].Speed > (Robot_Grade.Shoot_Speed - 400)) && (CAN1_Motor[1].Speed < -(Robot_Grade.Shoot_Speed - 400)))
	{
//		usart_printf("22\r\n");
//		if(Judge_User.Shooter_ID1_17mm_Cooling_Limit - Judge_User.Shooter_ID1_17mm_Cooling_Heat > 24)//限热量
//		{
//			usart_printf("11\r\n");
			if(RC.press_r)//RC.r_s == 2  RC.press_r 视觉瞄准后自动打单
			{
				if( obj_data.yaw < 0.8f && obj_data.yaw > -0.8f && obj_data.pitch < 2.4f && obj_data.pitch > -2.4f && HAL_GetTick() - obj_data.time < 100)
				{
					Fire_Speed_Out(Fire_Speed/0.6f);
					Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle;
				}
				else
					Fire_Angle_Out(Fire_Expect_Angle);
			}
			else
			{
				Fire_Angle_Out(Fire_Expect_Angle);
				
				if(Fire_Press_Flag == 0)//鼠标点射
				{
					if(RC.press_l && (RC.last_press_l == 0))
					{
						Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle + Stepping_Angle;
						Fire_Angle_Out(Fire_Expect_Angle);
					}
				}
				
				if(RC.press_l)//鼠标连射
				{ 
					if(Fire_Count < 60)
						Fire_Count++;
				}
				else if(RC.press_l == 0)
				{
					Fire_Count = 0;
					Fire_Press_Flag = 0;
					Fire_Angle_Out(Fire_Expect_Angle);
				}
				
				if(Fire_Count == 60)
				{
					Fire_Press_Flag = 1;
					Fire_Speed_Out(Fire_Speed);
					Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle;
				}
				else
					Fire_Angle_Out(Fire_Expect_Angle);
				
				
				
				if(RC.r_s == 2)//遥控器点射
				{
					if(Fire_RC == 2)
					{
						Fire_RC = 1;
						Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle + Stepping_Angle;
						Fire_Angle_Out(Fire_Expect_Angle);
					}
					else
						Fire_Angle_Out(Fire_Expect_Angle);
				}
				else if(RC.r_s == 1)//遥控器连射
				{
					Fire_Speed_Out(Fire_Speed);
					Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle;
				}
				else if(RC.r_s == 3 && Fire_Press_Flag == 0)
				{
					Fire_RC = 2;
					Fire_Angle_Out(Fire_Expect_Angle);
				}	
			}
//		}
//		else
//		{
//			Fire_Angle_Out(Fire_Expect_Angle);
//			Fire_Expect_Angle = CAN1_Motor[7].Continuous_Angle;
//		}
	}
}


