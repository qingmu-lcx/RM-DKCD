#ifndef Gimbal_H_
#define Gimbal_H_

#include "main.h"
#include "User_Pid.h"

//yaw 角度环 PID参数以及 PID最大输出，积分输出
#define Yaw_Angle_Pid_KP 					0.4f				//0.8f
#define Yaw_Angle_Pid_KI 					0.0f        //0.0f
#define Yaw_Angle_Pid_KD 					0.2f        //0.2f
#define Yaw_Angle_Pid_Max_IOUT 		0.0f        //0.0f
#define Yaw_Angle_Pid_Max_OUT 		400.0f      //400.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define Yaw_Speed_Pid_KP 					90.0f				//240.0f
#define Yaw_Speed_Pid_KI 					0.05f       //10.0f
#define Yaw_Speed_Pid_KD 					0.0f        //0.0f
#define Yaw_Speed_Pid_Max_IOUT 		5000.0f     //5000.0f
#define Yaw_Speed_Pid_Max_OUT 		30000.0f    //30000.0f

//pitch 角度环 PID参数以及 PID最大输出，积分输出
#define Pitch_Angle_Pid_KP 				2.0f    //1.2
#define Pitch_Angle_Pid_KI 				0.0f    //0.2
#define Pitch_Angle_Pid_KD 				0.2f    //0.5
#define Pitch_Angle_Pid_Max_IOUT 	0.0f
#define Pitch_Angle_Pid_Max_OUT 	400.0f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define Pitch_Speed_Pid_KP 				60.0f
#define Pitch_Speed_Pid_KI 				0.0f
#define Pitch_Speed_Pid_KD 				0.0f
#define Pitch_Speed_Pid_Max_IOUT 	5000.0f
#define Pitch_Speed_Pid_Max_OUT 	30000.0f


//yaw 角度环 PID参数以及 PID最大输出，积分输出
#define Yaw_Vision_Angle_Pid_KP 					0.8f				//0.4f
#define Yaw_Vision_Angle_Pid_KI 					0.0f        //0.0f
#define Yaw_Vision_Angle_Pid_KD 					2.0f       	//0.16f
#define Yaw_Vision_Angle_Pid_Max_IOUT 		0.0f        //0.0f
#define Yaw_Vision_Angle_Pid_Max_OUT 			400.0f      //400.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define Yaw_Vision_Speed_Pid_KP 					240.0f			//120.0f 
#define Yaw_Vision_Speed_Pid_KI 					0.0f       //10.0f
#define Yaw_Vision_Speed_Pid_KD 					0.0f        //0.0f
#define Yaw_Vision_Speed_Pid_Max_IOUT 		5000.0f     //5000.0f
#define Yaw_Vision_Speed_Pid_Max_OUT 			30000.0f    //30000.0f

//pitch 角度环 PID参数以及 PID最大输出，积分输出
#define Pitch_Vision_Angle_Pid_KP 				0.3f			//0.3f
#define Pitch_Vision_Angle_Pid_KI 				0.12f			//0.0f
#define Pitch_Vision_Angle_Pid_KD 				0.3f			//0.12f
#define Pitch_Vision_Angle_Pid_Max_IOUT 	0.0f			//0.0f
#define Pitch_Vision_Angle_Pid_Max_OUT 		400.0f		//400.0f

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define Pitch_Vision_Speed_Pid_KP 				120.0f		//120.0f
#define Pitch_Vision_Speed_Pid_KI 				15.0f			//10.0f
#define Pitch_Vision_Speed_Pid_KD 				0.0f			//0.0f
#define Pitch_Vision_Speed_Pid_Max_IOUT 	5000.0f		//5000.0f
#define Pitch_Vision_Speed_Pid_Max_OUT 		30000.0f	//30000.0f


extern float Yaw_K,Pitch_K;
extern pid_type_def Pitch_Speed_Pid;;
extern pid_type_def Yaw_Speed_Pid;

extern Gimbal_PID_t Yaw_Vision_Angle_Pid;
extern pid_type_def Yaw_Vision_Speed_Pid;
extern Gimbal_PID_t Pitch_Vision_Angle_Pid;
extern pid_type_def Pitch_Vision_Speed_Pid;

extern short M3508_Speed_Get[4];

void Gimbal_Init(void);
void Gimbal_Out(float Yaw, float Pitch);
void Gimbal_Vision_Out(float Vision_Yaw, float Vision_Pitch);
void Gimbal_Sleep(void);
void Gimbal_Control(void);
void Gimbal_Vision_Aim(void);
void Gimbal_Control_Init(void);

void Tx_Gimbal_To_Chassis(uint16_t id);
void Rx_Chassis_To_Gimbal(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);




#endif
