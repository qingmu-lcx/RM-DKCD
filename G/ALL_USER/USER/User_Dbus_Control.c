#include "User_Dbus_Control.h"
#include "User_Dbus.h"
#include "User_Debug.h"
#include "User_Lib.h"
//#include "Judge.h"

first_order_filter_type_t Chassis_Set_Vx;
first_order_filter_type_t Chassis_Set_Vy;

//调节灵敏度
float Chassis_RC_Vx = 9.0f;
float Chassis_RC_Vy = 9.0f;
float Chassis_Key_Vx = 10.0f;
float Chassis_Key_Vy = 10.0f;

float Chassis_Shift = 0.4f;

float Gimbal_RC_Pitch = 0.008f;
float Gimbal_RC_Yaw = 0.015f;
float Gimbal_M_Pitch = 0.015f;
float Gimbal_M_Yaw = 0.03f;


const static float chassis_x_order_filter[1] = { 0.1f };
const static float chassis_y_order_filter[1] = { 0.1f };
void Dbus_Control_Init(void)
{
	first_order_filter_init(&Chassis_Set_Vx, 0.002f, chassis_x_order_filter);
  first_order_filter_init(&Chassis_Set_Vy, 0.002f, chassis_y_order_filter);
}


//获取底盘模式
uint8_t Read_Robot_mode(void)
{
	//待机0    
	//底盘跟随1
	//小陀螺2    
    uint8_t mode;
		if(RC.l_s == 1)//正常模式
    {
        mode = chassis_wait;
    } 
		else if(RC.shift)
    {
        mode  = chassis_360;    
    }		
    else if(RC.l_s == 2)// 3
    {
        mode  = chassis_360;
    }    
    else if(RC.l_s == 3)//360
    {
         mode = chassis_follow_normol;   
    }
	return mode;    
}


//获取底盘指令
float RC_Vx;
float RC_Vy;
void Read_chassis_opt(float *vx,float *vy)
{	
	float Key_Vx;
	float Key_Vy;
	
	RC_Vx = RC.r_x - 1024;
	RC_Vy = RC.r_y - 1024;
	
	/*遥控器防止静态飘逸*/
	if(RC_Vx<4 && RC_Vx>-4)
	{
		RC_Vx=0;
	}
	if(RC_Vy<4 && RC_Vy>-4)
	{
		RC_Vy=0;
	}
	
	RC_Vx *= 5;//Robot_Grade.Chassis_Key_Vx;//Chassis_RC_Vx
	RC_Vy *= 5;//Robot_Grade.Chassis_Key_Vy;//Chassis_RC_Vy
	
//	Key_Vx = (RC.d - RC.a) * 660 * Robot_Grade.Chassis_Key_Vx;//Chassis_Key_Vx
//	Key_Vy = (RC.w - RC.s) * 660 * Robot_Grade.Chassis_Key_Vy;//Chassis_Key_Vy
	
	first_order_filter_cali(&Chassis_Set_Vx,Key_Vx);
	first_order_filter_cali(&Chassis_Set_Vy,Key_Vy);
	
	*vx = RC_Vx + 0;//Chassis_Set_Vx.out;
	*vy = RC_Vy + 0;//Chassis_Set_Vy.out;
}

//获取云台指令
void Read_gimbal_opt(float *pitch, float *yaw)
{
	if( RC.l_y - 1024 != 0 | RC.l_x - 1024 != 0 )//判断是否有键盘按下 0没有 1有
	{
		*pitch = RC.l_y - 1024;
		*yaw   = RC.l_x - 1024;
		
		/*遥控器防止静态飘逸*/
		if(*pitch<4 && *pitch>-4)
		{
			*pitch=0;
		}
		if(*yaw<4 && *yaw>-4)
		{
			*yaw=0;
		}
		
    *pitch *= Gimbal_RC_Pitch;
    *yaw   *= Gimbal_RC_Yaw; 
	}
	else 
	{ 
		*pitch =   -RC.m_y ;
		*yaw   =   RC.m_x ;
    *pitch *=  Gimbal_M_Pitch;
    *yaw   *=  Gimbal_M_Yaw;    
	}
}


