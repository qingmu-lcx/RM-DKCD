#ifndef Chassis_H_
#define Chassis_H_

#include "main.h"
#include "User_Pid.h"

//底盘电机速度环PID
#define M3508_Speed_Pid_KP 				2.0f 			//3.0f
#define M3508_Speed_Pid_KI 				0.002f  	//0.002f
#define M3508_Speed_Pid_KD 				0.0f			//0.0f
#define M3508_Speed_Pid_Max_IOUT 	4000.0f 	//2000.0f
#define M3508_Speed_Pid_Max_OUT 	33000.0f	//16000.0f

//底盘跟随PID
#define Chassis_Follow_Angle_Pid_KP 				18.8f 			//8.8f
#define Chassis_Follow_Angle_Pid_KI 				0.2f  		//0.0f
#define Chassis_Follow_Angle_Pid_KD 				6.0f			//6.0f
#define Chassis_Follow_Angle_Pid_Max_IOUT 	0.0f 			//2000.0f
#define Chassis_Follow_Angle_Pid_Max_OUT 		20000.0f	//16000.0f
//底盘跟随PID
#define Chassis_Follow_Speed_Pid_KP 				2.4f 			//2.0f
#define Chassis_Follow_Speed_Pid_KI 				0.48f  		//0.4f
#define Chassis_Follow_Speed_Pid_KD 				0.0f			//0.0f
#define Chassis_Follow_Speed_Pid_Max_IOUT 	6000.0f 	//6000.0f
#define Chassis_Follow_Speed_Pid_Max_OUT 		300000.0f	//24000.0f


extern pid_type_def M3508_Speed_Pid[4];
extern float Vx_Input,Vy_Input,W_Iuput;
extern uint16_t RNG_Number;


void Chassis_Init(void);
void Chassis_Out(float I1, float I2, float I3, float I4);
void Chassis_Sleep(void);
void Chassis_Vector_To_Mecanum_Wheel_Speed(const float vx_set, const float vy_set, const float wz_set);
float Chassis_Follow(float except);
void Chassis_Control(void);

typedef struct
{
	float Chassis_Key_Vx;
	float Chassis_Key_Vy;
	float Chassis_W;
	float M3508_Speed_Max;
	uint16_t Shoot_Speed;
} Robot_Grade_T;

typedef struct
{
	uint16_t Red_1_Robot_HP;										//红 1 英雄机器人血量，未上场以及罚下血量为 0
	uint16_t Red_2_Robot_HP;										//红 2 工程机器人血量
	uint16_t Red_3_Robot_HP;        						//红 3 步兵机器人血量
	uint16_t Red_4_Robot_HP;       					 		//红 4 步兵机器人血量
	uint16_t Red_5_Robot_HP;      					  	//红 5 步兵机器人血量
	uint16_t Red_7_Robot_HP;       					 		//红 7 哨兵机器人血量
	uint16_t Red_Outpost_HP;										//红方前哨战血量
	uint16_t Red_Base_HP;         					  	//红方基地血量
	uint16_t Blue_1_Robot_HP;    					   		//蓝 1 英雄机器人血量
	uint16_t Blue_2_Robot_HP;    					   		//蓝 2 工程机器人血量
	uint16_t Blue_3_Robot_HP;     					  	//蓝 3 步兵机器人血量
	uint16_t Blue_4_Robot_HP;     					 	 	//蓝 4 步兵机器人血量
	uint16_t Blue_5_Robot_HP;      					 		//蓝 5 步兵机器人血量
	uint16_t Blue_7_Robot_HP;       						//蓝 7 哨兵机器人血量
	uint16_t Blue_Outpost_HP;										//蓝方前哨站血量
  uint16_t Blue_Base_HP;         							//蓝方基地血量
	
	uint8_t Robot_ID;           								//本机器人 ID
	uint8_t Robot_Level;												//机器人等级
	uint16_t Remain_HP;													//机器人剩余血量
	uint16_t Max_HP;														//机器人上限血量
	uint16_t Shooter_ID1_17mm_Cooling_Rate; 		//机器人 1 号 17mm 枪口每秒冷却值
	uint16_t Shooter_ID1_17mm_Cooling_Limit;		//机器人 1 号 17mm 枪口热量上限
	uint16_t Shooter_ID1_17mm_Speed_Limit;			//机器人 1 号 17mm 枪口上限速度 单位 m/s
	uint16_t Shooter_ID2_17mm_Cooling_Rate;			//机器人 2 号 17mm 枪口每秒冷却值
	uint16_t Shooter_ID2_17mm_Cooling_Limit;    //机器人 2 号 17mm 枪口热量上限
	uint16_t Shooter_ID2_17mm_Speed_Limit;      //机器人 2 号 17mm 枪口上限速度 单位 m/s
	uint16_t Shooter_ID1_42mm_Cooling_Rate;     //机器人 42mm 枪口每秒冷却值
	uint16_t Shooter_ID1_42mm_Cooling_Limit;    //机器人 42mm 枪口热量上限
	uint16_t Shooter_ID1_42mm_Speed_Limit;      //机器人 42mm 枪口上限速度 单位 m/s
	uint16_t Chassis_Power_Max;									//机器人底盘功率限制上限
	uint8_t Mains_Power_Gimbal_Output : 1;			//主控电源输出情况
	uint8_t Mains_Power_Chassis_Output : 1;			//主控电源输出情况
	uint8_t Mains_Power_Shooter_Output : 1;			//主控电源输出情况
	
	float Chassis_Power;                        //底盘输出功率 单位 W 瓦
	uint16_t Chassis_Power_Buffer;              //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
	uint16_t Shooter_ID1_17mm_Cooling_Heat;     //1 号 17mm 枪口热量
	uint16_t Shooter_ID2_17mm_Cooling_Heat;     //2 号 17mm 枪口热量
	uint16_t Shooter_ID1_42mm_Cooling_Heat;     //42mm 枪口热量
	
	uint8_t Bullet_Freq;												//子弹射频 单位 Hz
	float Bullet_Speed;													//子弹射速 单位 m/s
	
	uint8_t	Cap_Voltage;
} Judge_User_T;

extern Robot_Grade_T	Robot_Grade;
extern Judge_User_T	Judge_User;

#endif


