#include "Chassis.h"
#include "math.h"
#include "PID.h"
#include "USER_CAN.h"
#include "DBUS_Control.h"
#include "Debug.h"
#include "Fire.h"

#define L1   0.42//
#define L2   0.37//
#define R    0.0762//
#define pi   3.1415926 

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ��������̨��ǰ����Ϊ��������1
#define CHASSIS_WZ_SET_SCALE 0.0f
//�������ĵ���̨���ĵľ���  ��λm
#define MOTOR_DISTANCE_TO_CENTER 0.28f

PidTypeDef M3508_SPEED_PID[4];

void Chassis_init(void)
{
	const static float M3508_SPEED_PID_3[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
	
	for(uint8_t i = 0; i < 4; i++)
	{
		PID_Init(&M3508_SPEED_PID[i],PID_POSITION,M3508_SPEED_PID_3,M3508_SPEED_PID_MAX_IOUT,M3508_SPEED_PID_MAX_OUT);
	}
}

void Chassis_out(float I1, float I2, float I3, float I4)
{
//  float error[4];
//	float error_avg;
//	
//	error[0] = I1 - (float)CAN1_Motor[0].Speed;
//	error[1] = I2 - (float)CAN1_Motor[1].Speed;
//	error[2] = I3 - (float)CAN1_Motor[2].Speed;
//	error[3] = I4 - (float)CAN1_Motor[3].Speed;
//	for(uint8_t i = 0; i < 4; i++)
//	{
//		error[i] = error[i]>0? error[i]:-error[i]; 
//	}
//	error_avg = ((error[0] + error[1] + error[2] + error[3])/4);
	
	
	float M3508_SPEED_PID_Test_3[3] = {Fire.PID1_P,Fire.PID1_I,Fire.PID1_D};//Fire_PID
	for(uint8_t i = 0; i < 4; i++)
	{
		PID_Init(&M3508_SPEED_PID[i],PID_POSITION,M3508_SPEED_PID_Test_3,M3508_SPEED_PID_MAX_IOUT,M3508_SPEED_PID_MAX_OUT);//Fire_PID
	}
	
	
	
	PID_Calc(&M3508_SPEED_PID[0], CAN1_Motor[0].Speed, I1);
	PID_Calc(&M3508_SPEED_PID[1], CAN1_Motor[1].Speed, I2);
	PID_Calc(&M3508_SPEED_PID[2], CAN1_Motor[2].Speed, I3);
	PID_Calc(&M3508_SPEED_PID[3], CAN1_Motor[3].Speed, I4);
	
	for(uint8_t i = 0; i < 4; i++)
	{
		if(CAN1_Motor[i].Temperate > 80)
			Chassis_sleep();
	}
//	uart_printf("Speed = %d\r\n",(short)M3508_SPEED_PID[0].OUT);
	CAN1_OPerate_Motor(1, (short)M3508_SPEED_PID[0].OUT);
	CAN1_OPerate_Motor(2, (short)M3508_SPEED_PID[1].OUT);
	CAN1_OPerate_Motor(3, (short)M3508_SPEED_PID[2].OUT);
	CAN1_OPerate_Motor(4, (short)M3508_SPEED_PID[3].OUT);	
}
/****���̴���****/
void Chassis_sleep(void)
{
	//����ֵ���
	CAN1_OPerate_Motor(1,0);
	CAN1_OPerate_Motor(2,0);
	CAN1_OPerate_Motor(3,0);
	CAN1_OPerate_Motor(4,0);
	//PID������
	PID_clear(&M3508_SPEED_PID[0]);
	PID_clear(&M3508_SPEED_PID[1]);
	PID_clear(&M3508_SPEED_PID[2]);
	PID_clear(&M3508_SPEED_PID[3]);
}

/*
���ֽ���
���Ӱ�װλ�ã�
              "\"w1 ID1          "/"w2ID2
              "/"w4  ID4         "\"w3ID3
*/
void MecanumCalculate(float vx,float vy,float wz)//���ֽ���//152.5mm
{    //1140
	float Buffer[4];
	Buffer[0]=1140.0*((1/R)*( (vy+vx) +wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy-wz*(L1+L2));
	Buffer[1]=1140.0*((1/R)*( (-vy+vx)+wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy-wz*(L1+L2));
	Buffer[2]=1140.0*((1/R)*( (-vy-vx)+wz*(L1+L2)))/(2*pi);//(1/R)*( vx+vy+wz*(L1+L2));
	Buffer[3]=1140.0*((1/R)*( (vy-vx) +wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy+wz*(L1+L2));
	Chassis_out(Buffer[0],Buffer[1],Buffer[2],Buffer[3]);
}
/*
360���ֽ���
���Ӱ�װλ�ã�
              "\"w1 ID1          "/"w2ID2
              "/"w4  ID4         "\"w3ID3
*/
void MecanumCalculate360(float VX_x,float VX_y,float VY_x,float VY_y,float wz)
{
  //�����ǣ�1140����ı���
	float Buffer[4];
	Buffer[0]=1140.0*((1/R)*((VY_y+VY_x)  + (-VX_x+VX_y) +wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy-wz*(L1+L2));
	Buffer[1]=1140.0*((1/R)*((-VY_y+VY_x) + (VX_x+VX_y)  +wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy-wz*(L1+L2));
	Buffer[2]=1140.0*((1/R)*((-VY_y-VY_x) + (VX_x-VX_y)  +wz*(L1+L2)))/(2*pi);//(1/R)*( vx+vy+wz*(L1+L2));
	Buffer[3]=1140.0*((1/R)*((VY_y+VY_x)  + (-VX_x-VX_y) +wz*(L1+L2)))/(2*pi);//(1/R)*(-vx+vy+wz*(L1+L2));//y
	Chassis_out(Buffer[0],Buffer[1],Buffer[2],Buffer[3]);
}
//�ٷ����̽���
void Chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set)
{
  float wheel_speed[4];
	wheel_speed[0] =  vx_set + vy_set - (	CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] =  vx_set - vy_set - (	CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = -vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set + vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	Chassis_out(wheel_speed[0],wheel_speed[1],wheel_speed[2],wheel_speed[3]);
}



/**** ���̿��ƣ��ŵ��߳���ִ�У�****/
//void Chassis_move(void)
//{
//	float angle;//С����ʱ ����̨����ϵ�͵�������ϵ�Ĳ����
//	float vx_input,vy_input,w_iuput;
////  float VX_x,VX_y,VY_x,VY_y; 
//	float Vx,Vy;
//	if(Read_Robot_mode() == chassis_wait)	//����ģʽ
//	{
//		Chassis_sleep();
//		return;
//	}
//	else//������ꡢң����
//  {
//		Read_chassis_opt(&vx_input,&vy_input);
//        
//		angle = (float)CAN1_Motor[4].Angle_Radian*0.000767f;//С����ʱ ����̨����ϵ�͵�������ϵ�Ĳ���� //Radian*360/8192*��/180
////		
////		VX_x = (float)(sin(angle) *  vx_input);
////		VX_y = (float)(cos(angle) *  vx_input);
////		VY_x = (float)(sin(angle) *  vy_input);
////		VY_y = (float)(cos(angle) *  vy_input); 
//		
//		if(Read_Robot_mode() == chassis_follow_normol)
//		{
//			Read_gimbal_opt(0,&w_iuput);
////			w_iuput = chassis_follow(0);         //���̸�����ٶ�
////			MecanumCalculate360(VX_x,VX_y,VY_x,VY_y,w_iuput);  //���ֽ���  
////			MecanumCalculate(vx_input, vy_input, w_iuput);
//			Chassis_vector_to_mecanum_wheel_speed(vx_input, vy_input, w_iuput);
//		}
//		else if(Read_Robot_mode() == chassis_360)//360
//		{		
//			w_iuput=2;   //�������ٶ�
////			MecanumCalculate360(VX_x,VX_y,VY_x,VY_y,w_iuput);  //���ֽ���
//			Vx = (float)(vx_input * cos(angle) - vy_input * sin(angle));
//			Vy= (float)(vx_input * sin(angle) - vy_input * cos(angle));
//			Chassis_vector_to_mecanum_wheel_speed(Vx, Vy, w_iuput);
////				Chassis_sleep();
//		}
//	}
//	
//}
