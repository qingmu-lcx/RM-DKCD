#include "User_Debug.h"
//#include "usb_device.h"
//#include "usbd_cdc_if.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"
#include "cmsis_os.h"

#include "User_Dbus.h"
#include "User_Can.h"
#include "AHRS_Task.h"
//#include "Chassis.h"
//#include "Judge.h"
#include "Gimbal.h"
#include "PC_Link.h"

void Debug_Usart(void)
{
//	usart_printf("123\r\n");
//	usart_printf("n=%d\r\n",1);
	/*CAN电机读取*/
//	usart_printf("YAW = %d pitch = %d Roll = %d\r\n",CAN1_Motor[4].Angle,CAN1_Motor[5].Angle,CAN1_Motor[6].Angle);
//	usart_printf("YAW = %d pitch = %d Roll = %d\r\n",CAN1_Motor[4].Speed,CAN1_Motor[5].Speed,CAN1_Motor[6].Speed);
	usart_printf("Angle6=%d\r\n",CAN1_Motor[4].Angle);//5->pich    4->yaw  
//	usart_printf("S1=%d	S2=%d\r\n",CAN1_Motor[0].Speed,CAN1_Motor[1].Speed);
//	usart_printf("Angle=%d   OUT=%d\r\n",CAN1_Motor[4].Angle,(short)Pitch_Speed_Pid.OUT);
//	usart_printf("speed=%d   out=%d\r\n",CAN1_Motor[0].Speed,(short)M3508_Speed_Pid_Left.OUT);
//	usart_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN1_Motor[0].Angle,CAN1_Motor[1].Angle,CAN1_Motor[2].Angle,CAN1_Motor[3].Angle);
//	usart_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN1_Motor[4].Angle,CAN1_Motor[5].Angle,CAN1_Motor[6].Angle,CAN1_Motor[7].Angle);
//	usart_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN2_Motor[0].Angle,CAN2_Motor[1].Angle,CAN2_Motor[2].Angle,CAN2_Motor[3].Angle);
//	usart_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN2_Motor[4].Angle,CAN2_Motor[5].Angle,CAN2_Motor[6].Angle,CAN2_Motor[7].Angle);
//	usart_printf("S1=%d  S2=%d  S3=%d  S4=%d\r\n",CAN1_Motor[0].Speed,CAN1_Motor[1].Speed,CAN1_Motor[2].Speed,CAN1_Motor[3].Speed);
//	usart_printf("S5=%d  S6=%d  S7=%d  S8=%d\r\n",CAN1_Motor[4].Speed,CAN1_Motor[5].Speed,CAN1_Motor[6].Speed,CAN1_Motor[7].Speed);
//	usart_printf("S1=%d  S2=%d  S3=%d  S4=%d\r\n",CAN2_Motor[0].Speed,CAN2_Motor[1].Speed,CAN2_Motor[2].Speed,CAN2_Motor[3].Speed);
//	usart_printf("S5=%d  S6=%d  S7=%d  S8=%d\r\n",CAN2_Motor[4].Speed,CAN2_Motor[5].Speed,CAN2_Motor[6].Speed,CAN2_Motor[7].Speed);
//	usart_printf("S1=%d  S2=%d  S3=%d  S4=%d\r\n",M3508_Speed_Get[0],M3508_Speed_Get[1],M3508_Speed_Get[2],M3508_Speed_Get[3]);
//	usart_printf("Temperate=%d\r\n",CAN1_Motor[0].Temperate);
//	usart_printf("Continuous_Angle=%d\r\n",CAN1_Motor[7].Continuous_Angle);
//	usart_printf("diversity_Angle=%d\r\n",CAN1_Motor[4].diversity_Angle);
//	usart_printf("Angle_Radian=%d\r\n",CAN1_Motor[4].Angle_Radian);
//	usart_printf("6=%d\r\n",CAN1_Motor[5].Temperate);
//	Current = (fabs(CAN2_Motor[0].Given_Current)+fabs(CAN2_Motor[1].Given_Current)+fabs(CAN2_Motor[2].Given_Current)+fabs(CAN2_Motor[3].Given_Current)) / 1000.0f;
//	Power = 24.0f * Current;
//	usart_printf("Current=%.2fA  Power=%.2fW\r\n",Current,Power);
		
							/*DBus遥控器数据*/ 
//	usart_printf("RC.l_x=%d  RC.r_x=%d\r\n", RC.l_x, RC.r_x);
//	usart_printf("RC.l_y=%d  RC.r_y=%d\r\n", RC.l_y, RC.r_y);
//	usart_printf("RC.l_s=%d  RC.r_s=%d\r\n", RC.l_s, RC.r_s);
//	usart_printf("RC.m_x=%d  RC.m_y\r\n", RC.m_x, RC.m_y);
//	usart_printf("w=%d  s=%d  a=%d  d=%d\r\n", RC.w, RC.s, RC.a, RC.d);
		
							/*ATK_IMU*/
//	usart_printf("ah = %.2f\r\n",IMU.Read_Yaw);
//	usart_printf("Yaw= %.2f  Pitch= %.2f Roll= %.2f Temp = %.2f\r\n",IMU.Yaw,IMU.Pitch,IMU.Roll,IMU.Temp);
//	usart_printf("q0= %f  q1= %f  q2= %f  q3= %f\r\n",IMU.q0,IMU.q1,IMU.q2,IMU.q3);
//	usart_printf("gyro_x= %.2f  gyro_y= %.2f  gyro_z= %.2f\r\n",IMU.gyro_x,IMU.gyro_y,IMU.gyro_z);
//	usart_printf("fgyroD_x= %.2f  fgyroD_y= %.2f  fgyroD_z= %.2f\r\n",IMU.fgyroD_x,IMU.fgyroD_y,IMU.fgyroD_z);
//	usart_printf("acc_x= %d  acc_y= %d  acc_z= %d\r\n",IMU.acc_x,IMU.acc_y,IMU.acc_z);
//	usart_printf("Yaw = %.2f  Read_Yaw = %.2f  Circle = %f\r\n",IMU.Yaw,IMU.Read_Yaw,IMU.Send_Yaw);
//	usart_printf(" a  %f  %f\r\n",IMU.Yaw,IMU.Yaw * Motor_Ang_to_Ecd);

//	usart_printf("yaw %f %f  pitch %f %f\r\n",obj_data.yaw,Yaw_K,obj_data.pitch,Pitch_K);
	
//	usart_printf("yaw %f   pitch %f \r\n",Yaw_Vision_Speed_Pid.out,Pitch_Vision_Speed_Pid.out);
//  usart_printf("ojb= %.2f\r\n",obj_data.yaw);
//	usart_printf("Vx_Input = %f  Vy_Input = %f	W_Iuput = %f\r\n",Vx_Input,Vy_Input,W_Iuput);
//	usart_printf(" a  %f %f\r\n",Yaw_Speed_Pid.out,IMU.Yaw);


									/*裁判系统数据*/ 
//	usart_printf("Red_1_Robot_HP = %d\r\n",Judge_User.Red_1_Robot_HP);
//	usart_printf("Red_2_Robot_HP = %d\r\n",Judge_User.Red_2_Robot_HP);
//	usart_printf("Red_3_Robot_HP = %d\r\n",Judge_User.Red_3_Robot_HP);
//	usart_printf("Red_4_Robot_HP = %d\r\n",Judge_User.Red_4_Robot_HP);
//	usart_printf("Red_5_Robot_HP = %d\r\n",Judge_User.Red_5_Robot_HP);
//	usart_printf("Red_7_Robot_HP = %d\r\n",Judge_User.Red_7_Robot_HP);
//	usart_printf("Red_Outpost_HP = %d\r\n",Judge_User.Red_Outpost_HP);
//	usart_printf("Red_Base_HP =%d\r\n",Judge_User.Red_Base_HP);
//	usart_printf("Blue_1_Robot_HP = %d\r\n",Judge_User.Blue_1_Robot_HP);
//	usart_printf("Blue_2_Robot_HP = %d\r\n",Judge_User.Blue_2_Robot_HP);
//	usart_printf("Blue_3_Robot_HP = %d\r\n",Judge_User.Blue_3_Robot_HP);
//	usart_printf("Blue_4_Robot_HP = %d\r\n",Judge_User.Blue_4_Robot_HP);
//	usart_printf("Blue_5_Robot_HP = %d\r\n",Judge_User.Blue_5_Robot_HP);
//	usart_printf("Blue_7_Robot_HP = %d\r\n",Judge_User.Blue_7_Robot_HP);
//	usart_printf("Blue_Outpost_HP = %d\r\n",Judge_User.Blue_Outpost_HP); 
//	usart_printf("Blue_Base_HP = %d\r\n",Judge_User.Blue_Base_HP);
//	
//	usart_printf("Robot_ID = %d\r\n",Judge_User.Robot_ID);
//	usart_printf("Robot_Level = %d\r\n",Judge_User.Robot_Level);
//	usart_printf("Remain_HP = %d\r\n",Judge_User.Remain_HP);
//	usart_printf("Shooter_ID1_17mm_Cooling_Rate = %d\r\n",Judge_User.Shooter_ID1_17mm_Cooling_Rate);
//	usart_printf("Shooter_ID1_17mm_Cooling_Limit = %d\r\n",Judge_User.Shooter_ID1_17mm_Cooling_Limit);
//	usart_printf("Shooter_ID1_17mm_Speed_Limit = %d\r\n",Judge_User.Shooter_ID1_17mm_Speed_Limit);
//	usart_printf("Shooter_ID2_17mm_Cooling_Rate = %d\r\n",Judge_User.Shooter_ID2_17mm_Cooling_Rate);
//	usart_printf("Shooter_ID2_17mm_Cooling_Limit = %d\r\n",Judge_User.Shooter_ID2_17mm_Cooling_Limit);
//	usart_printf("Shooter_ID2_17mm_Speed_Limit = %d\r\n",Judge_User.Shooter_ID2_17mm_Speed_Limit);
//	usart_printf("Shooter_ID1_42mm_Cooling_Rate = %d\r\n",Judge_User.Shooter_ID1_42mm_Cooling_Rate);
//	usart_printf("Shooter_ID1_42mm_Cooling_Limit = %d\r\n",Judge_User.Shooter_ID1_42mm_Cooling_Limit);
//	usart_printf("Shooter_ID1_42mm_Speed_Limit = %d\r\n",Judge_User.Shooter_ID1_42mm_Speed_Limit);
//	usart_printf("Chassis_Power_Max = %d\r\n",Judge_User.Chassis_Power_Max);
//	usart_printf("Mains_Power_Gimbal_Output = %d\r\n",Judge_User.Mains_Power_Gimbal_Output);
//	usart_printf("Mains_Power_Chassis_Output = %d\r\n",Judge_User.Mains_Power_Chassis_Output);
//	usart_printf("Mains_Power_Shooter_Output = %d\r\n",Judge_User.Mains_Power_Shooter_Output);
	
//	usart_printf("Chassis_Power = %.2f\r\n",Judge_User.Chassis_Power);
//	usart_printf("Chassis_Power_Buffer = %d\r\n",Judge_User.Chassis_Power_Buffer);
//	usart_printf("Shooter_ID1_17mm_Cooling_Heat = %d\r\n",Judge_User.Shooter_ID1_17mm_Cooling_Heat);
//	usart_printf("Shooter_ID2_17mm_Cooling_Heat = %d\r\n",Judge_User.Shooter_ID2_17mm_Cooling_Heat);
//	usart_printf("Shooter_ID1_42mm_Cooling_Heat = %d\r\n",Judge_User.Shooter_ID1_42mm_Cooling_Heat);
//	
//	usart_printf("Bullet_Freq = %d\r\n",Judge_User.Bullet_Freq);
//	usart_printf("Bullet_Speed = %.2f\r\n",Judge_User.Bullet_Speed);

//	usart_printf("Cap_Voltage = %d\r\n",Judge_User.Cap_Voltage);

//	usart_printf("Over_Speed = %d  Robot_Grade.Shoot_Speed = %d\r\n",Over_Speed,Robot_Grade.Shoot_Speed);
//	usart_printf("Robot_Grade.Shoot_Speed = %d\r\n",Robot_Grade.Shoot_Speed);



}

void Debug_Usb(void)
{
	/*CAN电机读取*/
//	usb_printf("YAW = %d pitch = %d Roll = %d\r\n",CAN1_Motor[4].Angle,CAN1_Motor[5].Angle,CAN1_Motor[6].Angle);
//	usb_printf("Angle=%d\r\n",CAN1_Motor[7].Angle);
//	usb_printf("Speed=%d\r\n",CAN2_Motor[3].Speed);
//	usb_printf("Angle=%d   OUT=%d\r\n",CAN1_Motor[4].Angle,(short)Pitch_Speed_Pid.OUT);
//	usb_printf("speed=%d   out=%d\r\n",CAN1_Motor[0].Speed,(short)M3508_Speed_Pid_Left.OUT);
//	usb_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN1_Motor[0].Angle,CAN1_Motor[1].Angle,CAN1_Motor[2].Angle,CAN1_Motor[3].Angle);
//	usb_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN1_Motor[4].Angle,CAN1_Motor[5].Angle,CAN1_Motor[6].Angle,CAN1_Motor[7].Angle);
//	usb_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN2_Motor[0].Angle,CAN2_Motor[1].Angle,CAN2_Motor[2].Angle,CAN2_Motor[3].Angle);
//	usb_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN2_Motor[4].Angle,CAN2_Motor[5].Angle,CAN2_Motor[6].Angle,CAN2_Motor[7].Angle);
//	usb_printf("Temperate=%d\r\n",CAN1_Motor[0].Temperate);
//	usb_printf("Continuous_Angle=%d\r\n",CAN1_Motor[7].Continuous_Angle);
//	usb_printf("diversity_Angle=%d\r\n",CAN1_Motor[4].diversity_Angle);
	
//	Current = (fabs(CAN2_Motor[0].Given_Current)+fabs(CAN2_Motor[1].Given_Current)+fabs(CAN2_Motor[2].Given_Current)+fabs(CAN2_Motor[3].Given_Current)) / 1000.0f;
//	Power = 24.0f * Current;
//	usb_printf("Current=%.2fA  Power=%.2fW\r\n",Current,Power);
	
	            /*DBus遥控器数据*/ 
//	usb_printf("RC.l_x=%d  RC.r_x=%d\r\n", RC.l_x, RC.r_x);
//	usb_printf("RC.l_y=%d  RC.r_y=%d\r\n", RC.l_y, RC.r_y);
//	usb_printf("RC.l_s=%d  RC.r_s=%d\r\n", RC.l_s, RC.r_s);
//	usb_printf("RC.m_x=%d  RC.m_y\r\n", RC.m_x, RC.m_y);
//	usb_printf("w=%d  s=%d  a=%d  d=%d\r\n", RC.w, RC.s, RC.a, RC.d);
		
							 /*ATK_IMU*/
//	usb_printf("Yaw= %.2f  Pitch= %.2f Roll= %.2f Temp = %.2f\r\n",IMU.Yaw,IMU.Pitch,IMU.Roll,IMU.Temp);
//	usb_printf("q0= %f  q1= %f  q2= %f  q3= %f\r\n",IMU.q0,IMU.q1,IMU.q2,IMU.q3);
//	usb_printf("gyro_x= %.2f  gyro_y= %.2f  gyro_z= %.2f\r\n",IMU.gyro_x,IMU.gyro_y,IMU.gyro_z);
//	usb_printf("fgyroD_x= %.2f  fgyroD_y= %.2f  fgyroD_z= %.2f\r\n",IMU.fgyroD_x,IMU.fgyroD_y,IMU.fgyroD_z);
//	usb_printf("acc_x= %d  acc_y= %d  acc_z= %d\r\n",IMU.acc_x,IMU.acc_y,IMU.acc_z);
//	usb_printf("faccG_x= %f  faccG_y= %f  faccG_z= %f\r\n",IMU.faccG_x,IMU.faccG_y,IMU.faccG_z);
//	usb_printf("Yaw = %f  Last_Yaw = %f  Read_Yaw = %f Circle = %d\r\n",IMU.Yaw,IMU.Last_Yaw,IMU.Read_Yaw,Circle);
							/*Surge_Wheel*/
//	usb_printf("SW_L_Speed=%d  SW_R_Speed=%d\r\n",SW_L_Speed,SW_R_Speed);


//	usb_printf("%d %d %d\r\n",ANO.PID1_P,ANO.PID1_I,ANO.PID1_D);
	
//	usb_printf(" a  %f  %f\r\n",IMU.Yaw,IMU.Yaw * Motor_Ang_to_Ecd);

//	usb_printf("yaw %f %f  pitch %f %f\r\n",obj_data.yaw,Yaw_K,obj_data.pitch,Pitch_K);
}

void usb_printf(const char *fmt,...)
{
	static uint8_t usb_buf[256];
  static va_list ap;
  uint16_t len = 0;
	
  va_start(ap, fmt);
  len = vsprintf((char *)usb_buf, fmt, ap);
  va_end(ap);
//  CDC_Transmit_FS(usb_buf, len);
}

void usart_printf(const char *fmt,...)//C板4pin的串口
{
	static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
		
  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart1,tx_buf,len,100);
}

