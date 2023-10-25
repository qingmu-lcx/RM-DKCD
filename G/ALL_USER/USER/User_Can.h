#ifndef USER_CAN_H_
#define USER_CAN_H_

#include "main.h"

//#define Yaw_Position_Init 		6400                 //2050
#define Yaw_Position_Init 		5594
#define Pitch_Position_Init 	3300  


//电机编码值转化成弧度值
#define Motor_Ang_to_Ecd			22.755555555555f				//=8192/360
//电机编码值转化成弧度值
#define Motor_Ecd_to_Rad 			0.000766990394f 			//=360/8192*Π/180

#define CAN1_Motor_NUM  8
#define CAN2_Motor_NUM  8

typedef struct
{
	short Angle;					//当前角度
	short Last_Angle;				//上一次的角度
	short Speed;					//当前速度
	short Last_Speed;				//上一次的速度
	short Given_Current;			//电机当前转矩电流
	uint8_t Temperate;				//电机温度
	short Circle;				    //转动的圈数
	short Angle_Difference;		    //角\1度差
	short Angle_Radian;			    //角速度
	int   Continuous_Angle;		    //连续角
  int   diversity_Angle;           //最小回正角
	int   Last_Continuous_Angle;    //上一次的连续角
	uint8_t    Flag;			    //标志位
}MotorBack;

extern MotorBack CAN1_Motor[CAN1_Motor_NUM];	//CAN1上的所有电机回馈信息

extern MotorBack CAN2_Motor[CAN2_Motor_NUM];	//CAN2上的所有电机回馈信息

extern uint32_t n;

void CAN1_to_Motor(uint16_t id);
void CAN2_to_Motor(uint16_t id);
void CAN1_OPerate_Motor(uint8_t id,short i);
void CAN2_OPerate_Motor(uint8_t id,short i);
void CAN_Filter_Init(void);

#endif

