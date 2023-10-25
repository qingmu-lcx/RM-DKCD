#include "Gimbal.h"
#include "User_Can.h"
#include "User_Dbus_Control.h"
#include "User_Kalman.h"
#include "User_Pid.h"
#include "PC_Link.h"
#include "AHRS_Task.h"
#include "User_Debug.h"
//#include "Judge.h"
#include "User_Dbus.h"
#include "Chassis.h"
#include "can.h"


Gimbal_PID_t Yaw_Angle_Pid;
pid_type_def Yaw_Speed_Pid;
Gimbal_PID_t Pitch_Angle_Pid;
pid_type_def Pitch_Speed_Pid;

Gimbal_PID_t Yaw_Vision_Angle_Pid;
pid_type_def Yaw_Vision_Speed_Pid;
Gimbal_PID_t Pitch_Vision_Angle_Pid;
pid_type_def Pitch_Vision_Speed_Pid;

extKalman_t Vision_Yaw;
extKalman_t Vision_Pitch;

static float Yaw_SetPosition;//yaw设定值
static float Pitch_SetPosition;//PItch设定值


void Gimbal_Init(void)//云台初始化
{
	Yaw_SetPosition = 0;
	Pitch_SetPosition = 0;
	
	static const float Yaw_Speed_Pid_3[3] = {Yaw_Speed_Pid_KP, Yaw_Speed_Pid_KI, Yaw_Speed_Pid_KD};
	static const float Pitch_Speed_Pid_3[3] = {Pitch_Speed_Pid_KP, Pitch_Speed_Pid_KI, Pitch_Speed_Pid_KD};
	
	static const float Yaw_Vision_Speed_Pid_3[3] = {Yaw_Vision_Speed_Pid_KP, Yaw_Vision_Speed_Pid_KI, Yaw_Vision_Speed_Pid_KD};
	static const float Pitch_Vision_Speed_Pid_3[3] = {Pitch_Vision_Speed_Pid_KP, Pitch_Vision_Speed_Pid_KI, Pitch_Vision_Speed_Pid_KD};
	
	GIMBAL_PID_Init(&Yaw_Angle_Pid, Yaw_Angle_Pid_KP, Yaw_Angle_Pid_KI, Yaw_Angle_Pid_KD, Yaw_Angle_Pid_Max_IOUT, Yaw_Angle_Pid_Max_OUT);
	PID_init(&Yaw_Speed_Pid, PID_POSITION, Yaw_Speed_Pid_3, Yaw_Speed_Pid_Max_IOUT, Yaw_Speed_Pid_Max_OUT);
	
	GIMBAL_PID_Init(&Pitch_Angle_Pid, Pitch_Angle_Pid_KP, Pitch_Angle_Pid_KI, Pitch_Angle_Pid_KD, Pitch_Angle_Pid_Max_IOUT, Pitch_Angle_Pid_Max_OUT);
	PID_init(&Pitch_Speed_Pid, PID_POSITION, Pitch_Speed_Pid_3, Pitch_Speed_Pid_Max_IOUT, Pitch_Speed_Pid_Max_OUT);
	
	GIMBAL_PID_Init(&Yaw_Vision_Angle_Pid, Yaw_Vision_Angle_Pid_KP, Yaw_Vision_Angle_Pid_KI, Yaw_Vision_Angle_Pid_KD, Yaw_Vision_Angle_Pid_Max_IOUT, Yaw_Vision_Angle_Pid_Max_OUT);
	PID_init(&Yaw_Vision_Speed_Pid, PID_POSITION, Yaw_Vision_Speed_Pid_3, Yaw_Vision_Speed_Pid_Max_IOUT, Yaw_Vision_Speed_Pid_Max_OUT);
	
	GIMBAL_PID_Init(&Pitch_Vision_Angle_Pid, Pitch_Vision_Angle_Pid_KP, Pitch_Vision_Angle_Pid_KI, Pitch_Vision_Angle_Pid_KD, Pitch_Vision_Angle_Pid_Max_IOUT, Pitch_Vision_Angle_Pid_Max_OUT);
	PID_init(&Pitch_Vision_Speed_Pid, PID_POSITION, Pitch_Vision_Speed_Pid_3, Pitch_Vision_Speed_Pid_Max_IOUT, Pitch_Vision_Speed_Pid_Max_OUT);
	
	KalmanCreate(&Vision_Yaw, 1, 2);
	KalmanCreate(&Vision_Pitch, 1, 2);
}
void Gimbal_Out(float Yaw, float Pitch)//正常云台pid
{
	GIMBAL_PID_Calc(&Yaw_Angle_Pid, IMU.Yaw* Motor_Ang_to_Ecd, Yaw, IMU.gyro_z,0);
	PID_calc(&Yaw_Speed_Pid, IMU.gyro_z, Yaw_Angle_Pid.OUT);
	
	GIMBAL_PID_Calc(&Pitch_Angle_Pid, IMU.Pitch* Motor_Ang_to_Ecd, Pitch, IMU.gyro_x,0);
	PID_calc(&Pitch_Speed_Pid, IMU.gyro_x, Pitch_Angle_Pid.OUT);
	
	if(CAN1_Motor[4].Temperate > 80 || CAN1_Motor[5].Temperate > 80)
		Gimbal_Sleep();
	CAN1_OPerate_Motor(5,(short)Yaw_Speed_Pid.out);
	CAN1_OPerate_Motor(6, -(short)Pitch_Speed_Pid.out);
//	usart_printf("set=%.2f get=%.2f\r\n",Pitch,IMU.Pitch* Motor_Ang_to_Ecd);
//	usart_printf("set=%.2f get=%.2f\r\n",Yaw,IMU.Yaw* Motor_Ang_to_Ecd);
//	usart_printf("Yaw=%d\r\n",(short)Yaw_Speed_Pid.out);
	CAN1_to_Motor(0x1FF);
}
void Gimbal_Vision_Out(float Vision_Yaw, float Vision_Pitch)//视觉pid
{
	GIMBAL_PID_Calc(&Yaw_Vision_Angle_Pid, IMU.Yaw* Motor_Ang_to_Ecd, Vision_Yaw,-IMU.gyro_z,0);
	PID_calc(&Yaw_Vision_Speed_Pid, IMU.gyro_z, Yaw_Vision_Angle_Pid.OUT);
	
	GIMBAL_PID_Calc(&Pitch_Vision_Angle_Pid, IMU.Pitch* Motor_Ang_to_Ecd, Vision_Pitch,IMU.gyro_x,0);
	PID_calc(&Pitch_Vision_Speed_Pid, IMU.gyro_x, Pitch_Vision_Angle_Pid.OUT);
	
	if(CAN1_Motor[4].Temperate > 80 || CAN1_Motor[5].Temperate > 80)
		Gimbal_Sleep();
	
	CAN1_OPerate_Motor(5,(short)Yaw_Vision_Speed_Pid.out);
	CAN1_OPerate_Motor(6, -(short)Pitch_Vision_Speed_Pid.out);
	CAN1_to_Motor(0x1FF);
}
void Gimbal_Sleep(void)
{
	//电流值清空
	CAN1_OPerate_Motor(5,0);
	CAN1_OPerate_Motor(6,0);
	CAN1_to_Motor(0x1FF);
	//PID输出清空
	Gimbal_PID_clear(&Yaw_Angle_Pid);
	PID_clear(&Yaw_Speed_Pid);
	Gimbal_PID_clear(&Pitch_Angle_Pid);
	PID_clear(&Pitch_Speed_Pid);
	
	Gimbal_PID_clear(&Yaw_Vision_Angle_Pid);
	PID_clear(&Yaw_Vision_Speed_Pid);
	Gimbal_PID_clear(&Pitch_Vision_Angle_Pid);
	PID_clear(&Pitch_Vision_Speed_Pid);
}

void Gimbal_Control(void)
{
	float Inc_Yaw,Inc_Pitch;
	if(Read_Robot_mode() == chassis_wait )//待机模式
  {
    Gimbal_Sleep();
    return;
  }
	else
	{
		Read_gimbal_opt(&Inc_Pitch, &Inc_Yaw);
		if(RC.press_r)//RC.r_s == 2  RC.press_r 按鼠标右键启动视觉
		{
			Gimbal_Vision_Aim();//视觉跟随
			
			if(Pitch_SetPosition > 30 *Motor_Ang_to_Ecd)//pitch轴限制角度
				Pitch_SetPosition = 30 *Motor_Ang_to_Ecd;
			if(Pitch_SetPosition < -12 *Motor_Ang_to_Ecd)
			 Pitch_SetPosition = -12 *Motor_Ang_to_Ecd;
			
			Gimbal_Vision_Out(Yaw_SetPosition,Pitch_SetPosition);
		}
		else
		{
			Pitch_SetPosition += Inc_Pitch ; //pitch赋值	
			if(Read_Robot_mode() == chassis_follow_normol)
			{
//				usart_printf("n=%d\r\n",1);
				if(((CAN1_Motor[4].diversity_Angle >  1200) || (CAN1_Motor[4].diversity_Angle < -1200)) && (RC.b == 0))
					Yaw_SetPosition += 0;
				else
				{
					Yaw_SetPosition -= Inc_Yaw;
				}
			}
			else if(Read_Robot_mode() == chassis_360)
 				Yaw_SetPosition -= Inc_Yaw;
			
			if(Pitch_SetPosition > 30 *Motor_Ang_to_Ecd)
				Pitch_SetPosition = 30 *Motor_Ang_to_Ecd;
			if(Pitch_SetPosition < -12 *Motor_Ang_to_Ecd)
			 Pitch_SetPosition = -12 *Motor_Ang_to_Ecd;
//			usart_printf("n=%d\r\n",Yaw_SetPosition);
			Gimbal_Out(Yaw_SetPosition,Pitch_SetPosition);
		}	
	}
}

float Yaw_K,Pitch_K;
void Gimbal_Vision_Aim(void)
{
	if(obj_data.new_data == 0)//没有视觉数据
	{
    Yaw_SetPosition +=0;
		Pitch_SetPosition +=0;
	}
	else
	{
		obj_data.new_data = 0;//清除视觉标志位，等待下次视觉数据到达  
		Yaw_K = KalmanFilter(&Vision_Yaw,obj_data.yaw);
		Pitch_K = KalmanFilter(&Vision_Pitch,obj_data.pitch);
		
		Yaw_SetPosition = (IMU.Yaw - Yaw_K) * Motor_Ang_to_Ecd;
		Pitch_SetPosition = (IMU.Pitch + Pitch_K) * Motor_Ang_to_Ecd;
	}
}

uint8_t Chassis_Reset_Flag = 0;

void Tx_Gimbal_To_Chassis(uint16_t id)
{
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef     tx_message;
	
	uint8_t M3508_Speed_Set[8];
	uint8_t Gimbal_To_Chassis_Data1[8];
	
	switch(id)
	{
		case 0x220:
		{
			if(RC.z == 1 && RC.l_s == 1)
				Chassis_Reset_Flag = 1;
			else
				Chassis_Reset_Flag = 0;
			
//			Gimbal_To_Chassis_Data1[0] = RC.shift << 7 | Shoot_Flag << 6 | Chassis_Reset_Flag << 5 | RC.ctrl << 4;
			Gimbal_To_Chassis_Data1[1] = (uint8_t)((short)(IMU.Pitch*100) >> 8);
			Gimbal_To_Chassis_Data1[2] = (uint8_t)((short)(IMU.Pitch*100) & 0x00ff);
			Gimbal_To_Chassis_Data1[3] = (uint8_t)((short)(IMU.Send_Yaw*100) >> 8);
			Gimbal_To_Chassis_Data1[4] = (uint8_t)((short)(IMU.Send_Yaw*100) & 0x00ff);
			
			tx_message.StdId = 0x220;
			tx_message.IDE   = CAN_ID_STD;
			tx_message.RTR   = CAN_RTR_DATA;
			tx_message.DLC   = 0x08;

			HAL_CAN_AddTxMessage(&hcan2,&tx_message,Gimbal_To_Chassis_Data1,&TxMailbox);
			
			break;
		}
		case 0x221:
		{
			M3508_Speed_Set[0] = (uint8_t)((short)M3508_Speed_Pid[0].out>>8);
			M3508_Speed_Set[1] = (uint8_t)((short)M3508_Speed_Pid[0].out&0x00ff);
			M3508_Speed_Set[2] = (uint8_t)((short)M3508_Speed_Pid[1].out>>8);
			M3508_Speed_Set[3] = (uint8_t)((short)M3508_Speed_Pid[1].out&0x00ff);
			M3508_Speed_Set[4] = (uint8_t)((short)M3508_Speed_Pid[2].out>>8);
			M3508_Speed_Set[5] = (uint8_t)((short)M3508_Speed_Pid[2].out&0x00ff);
			M3508_Speed_Set[6] = (uint8_t)((short)M3508_Speed_Pid[3].out>>8);
			M3508_Speed_Set[7] = (uint8_t)((short)M3508_Speed_Pid[3].out&0x00ff);
//			usart_printf("123\r\n");
			tx_message.StdId = 0x221;
			tx_message.IDE   = CAN_ID_STD;
			tx_message.RTR   = CAN_RTR_DATA;
			tx_message.DLC   = 0x08;

			HAL_CAN_AddTxMessage(&hcan2,&tx_message,M3508_Speed_Set,&TxMailbox);
			break;
		}
		case 0x222:
		{
			break;
		}
		default: break;
	}
}

short M3508_Speed_Get[4];
void Rx_Chassis_To_Gimbal(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]) //接收底盘数据
{
	if(pHeader->StdId == 0x230)
	{
		Judge_User.Robot_ID = aData[0];
		Judge_User.Shooter_ID1_17mm_Cooling_Limit = (short)(aData[1]<<8 | aData[2]);
		Judge_User.Shooter_ID1_17mm_Speed_Limit = aData[3];
		Judge_User.Bullet_Speed = ((float)((short)(aData[4]<<8 | aData[5])))/100.0f;
		Judge_User.Shooter_ID1_17mm_Cooling_Heat = (short)(aData[6]<<8 | aData[7]);
		return ;
	}
	if(pHeader->StdId == 0x231)
	{
//		usart_printf("%d\r\n",aData[0]);
		M3508_Speed_Get[0] = (short)(aData[0]<<8 | aData[1]);
    M3508_Speed_Get[1] = (short)(aData[2]<<8 | aData[3]);
    M3508_Speed_Get[2] = (short)(aData[4]<<8 | aData[5]);
    M3508_Speed_Get[3] = (short)(aData[6]<<8 | aData[7]);
		return ;
	}
	else if(pHeader->StdId == 0x232)
	{
//		Judge_User.Chassis_Power_Max = (short)(aData[0]<<8 | aData[1]);
//		Judge_User.Cap_Voltage = (short)(aData[2]);
	}
	else
		return ;
}

Robot_Grade_T	 Robot_Grade;
Judge_User_T	Judge_User;

