#include "Chassis.h"
#include "math.h"
#include "User_Pid.h"
#include "User_Can.h"
#include "User_Dbus.h"
#include "User_Dbus_Control.h"
#include "User_Debug.h"
//#include "Judge.h"
#include "AHRS_Task.h"
//#include "Judge.h"
#include "Gimbal.h"


//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿；云台靠前补偿为正；最大±1
#define CHASSIS_WZ_SET_SCALE 0.0f
//轮子中心到云台中心的距离  单位m
#define MOTOR_DISTANCE_TO_CENTER 0.28f

uint16_t RNG_Number;

pid_type_def M3508_Speed_Pid[4];

Gimbal_PID_t Chassis_Follow_Angle_Pid;
pid_type_def Chassis_Follow_Speed_Pid;

void Chassis_Init(void)
{
	const static float M3508_Speed_Pid_3[3] = {M3508_Speed_Pid_KP, M3508_Speed_Pid_KI, M3508_Speed_Pid_KD};
	
	const static float Chassis_Follow_Pid_3[3] = {Chassis_Follow_Speed_Pid_KP, Chassis_Follow_Speed_Pid_KI, Chassis_Follow_Speed_Pid_KD};
	
	for(uint8_t i = 0; i < 4; i++)
	{
		PID_init(&M3508_Speed_Pid[i],PID_POSITION,M3508_Speed_Pid_3,M3508_Speed_Pid_Max_IOUT,M3508_Speed_Pid_Max_OUT);
	}
	
	GIMBAL_PID_Init(&Chassis_Follow_Angle_Pid, Chassis_Follow_Angle_Pid_KP, Chassis_Follow_Angle_Pid_KI, Chassis_Follow_Angle_Pid_KD, Chassis_Follow_Angle_Pid_Max_IOUT, Chassis_Follow_Angle_Pid_Max_OUT);
	PID_init(&Chassis_Follow_Speed_Pid,PID_POSITION,Chassis_Follow_Pid_3,Chassis_Follow_Angle_Pid_Max_IOUT,Chassis_Follow_Angle_Pid_Max_OUT);
}

float CAP = 2.0f;
void Chassis_Out(float I1, float I2, float I3, float I4)
{	
	/*
	if(RC.ctrl == 1 && Judge_User.Cap_Voltage > 18)//控制超级电容放电
	{
		I1 *= CAP;
		I2 *= CAP;
		I3 *= CAP;
		I4 *= CAP;
	}
	else
	{
		I1 *= 1.0f;
		I2 *= 1.0f;
		I3 *= 1.0f;
		I4 *= 1.0f;
	}
	*/
	PID_calc(&M3508_Speed_Pid[0], M3508_Speed_Get[0], I1);
	PID_calc(&M3508_Speed_Pid[1], M3508_Speed_Get[1], I2);
	PID_calc(&M3508_Speed_Pid[2], M3508_Speed_Get[2], I3);
	PID_calc(&M3508_Speed_Pid[3], M3508_Speed_Get[3], I4);
//	PID_calc(&M3508_Speed_Pid[0], CAN2_Motor[0].Speed, I1);
//	PID_calc(&M3508_Speed_Pid[1], CAN2_Motor[1].Speed, I2);
//	PID_calc(&M3508_Speed_Pid[2], CAN2_Motor[2].Speed, I3);
//	PID_calc(&M3508_Speed_Pid[3], CAN2_Motor[3].Speed, I4);
	for(uint8_t i = 0; i < 4; i++)
	{
		if(CAN2_Motor[i].Temperate > 80)
			Chassis_Sleep();
	}
	
	CAN2_OPerate_Motor(1, (short)M3508_Speed_Pid[0].out);
	CAN2_OPerate_Motor(2, (short)M3508_Speed_Pid[1].out);
	CAN2_OPerate_Motor(3, (short)M3508_Speed_Pid[2].out);
	CAN2_OPerate_Motor(4, (short)M3508_Speed_Pid[3].out);	
	CAN2_to_Motor(0x200);
}
/****底盘待机****/
void Chassis_Sleep(void)
{
	//电流值清空
//	CAN2_OPerate_Motor(1,0);
//	CAN2_OPerate_Motor(2,0);
//	CAN2_OPerate_Motor(3,0);
//	CAN2_OPerate_Motor(4,0);
	//PID输出清空
	PID_clear(&M3508_Speed_Pid[0]);
	PID_clear(&M3508_Speed_Pid[1]);
	PID_clear(&M3508_Speed_Pid[2]);
	PID_clear(&M3508_Speed_Pid[3]);
}

//官方底盘解算
void Chassis_Vector_To_Mecanum_Wheel_Speed(const float vx_set, const float vy_set, const float wz_set)
{
  float wheel_speed[4];
	wheel_speed[0] =  vx_set + vy_set - (	CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] =  vx_set - vy_set - (	CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = -vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set + vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	Chassis_Out(wheel_speed[0],wheel_speed[1],wheel_speed[2],wheel_speed[3]);
}

//底盘旋转的W解算
float Chassis_Follow(float except)
{
	float W_Output;
	if(CAN1_Motor[4].diversity_Angle > 48 || CAN1_Motor[4].diversity_Angle < -48)
	{
		GIMBAL_PID_Calc(&Chassis_Follow_Angle_Pid, CAN1_Motor[4].diversity_Angle, except, IMU.gyro_z,0);
		PID_calc(&Chassis_Follow_Speed_Pid,IMU.gyro_z,Chassis_Follow_Angle_Pid.OUT);
		W_Output = -Chassis_Follow_Speed_Pid.out;
	}
	else
		W_Output = 0;
	
	return W_Output;
}


/**** 底盘控制（放到线程中执行）****/
float Vx_Input,Vy_Input,W_Iuput;
void Chassis_Control(void)
{
//	usart_printf("2\r\n");
	float Angle;//小陀螺时 的云台坐标系和底盘坐标系的差异角
	float Vx,Vy;
	if(Read_Robot_mode() == chassis_wait)	//待机模式
	{
		if(RC.x == 1)
			HAL_NVIC_SystemReset();// 复位系统
		
		Chassis_Sleep();
		return;
	}
	else//键盘鼠标、遥控器
  {
		Read_chassis_opt(&Vx_Input,&Vy_Input);
	       
		Angle = (float)CAN1_Motor[4].Angle_Radian*Motor_Ecd_to_Rad;//小陀螺时 的云台坐标系和底盘坐标系的差异角 //Radian*360/8192*Π/180

		if(Read_Robot_mode() == chassis_follow_normol)
		{
//			Read_gimbal_opt(0,&W_Iuput);
//			W_Iuput = Chassis_Follow(900);
			
			W_Iuput = Chassis_Follow(0);
			
			if(RC.b && RC.q == 0 && RC.e == 0)//按b地盘与云台分离
				W_Iuput = 0;
			
			if(RC.q && RC.b == 0 && RC.e == 0)//按q锁左边中间
				W_Iuput = Chassis_Follow(-1000); 
			
			if(RC.e && RC.b == 0 && RC.q == 0)//按e锁右边中间
				W_Iuput = Chassis_Follow(1000); 
				
			
			if(W_Iuput != 0)//转弯时减速
			{
				Vx_Input *= 0.6f;
				Vy_Input *= 0.6f;
			}
			if(RC.c)//补单按c减速
			{
				Vx_Input *= 0.5f;
				Vy_Input *= 0.5f;
			}
//			else
//			{
//				Vx_Input *= 0.6f;
//				Vy_Input *= 0.6f;
//			}
			
			Chassis_Vector_To_Mecanum_Wheel_Speed(Vx_Input, Vy_Input, W_Iuput);
		}
		else if(Read_Robot_mode() == chassis_360)
		{
			if(RC_Vx == 0 && RC_Vy == 0 && RC.w == 0 && RC.s == 0 && RC.a == 0 && RC.d == 0)//移动时减缓小陀螺转速
			{
				W_Iuput = -8000;//-Robot_Grade.Chassis_W;//小陀螺转速根据底盘功率来确定
			}
			else
				W_Iuput = -8000;
//			usart_printf("n=%d\r\n",Angle);
			Vx = (float)(  Vx_Input * cos(Angle) + Vy_Input * sin(Angle));
			Vy = (float)( -Vx_Input * sin(Angle) + Vy_Input * cos(Angle));
			Chassis_Vector_To_Mecanum_Wheel_Speed(Vx, Vy, W_Iuput);
		}
	}
	
}
