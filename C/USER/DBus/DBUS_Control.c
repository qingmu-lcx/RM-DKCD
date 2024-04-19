#include "DBUS_Control.h"
#include "Dbus.h"

//��ȡ����ģʽ
uint8_t Read_Robot_mode(void)
{
	//����0    
	//���̸���1
	//С����2    
    uint8_t mode;
		if(RC.l_s == 1)//����ģʽ
    {
        mode = chassis_wait;
    } 
		else if(RC.press_r)
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

//��ȡ����ָ��
void Read_chassis_opt(float *vx,float *vy)
{
	if(RC.key == 0)
	{
		*vx = RC.r_x - 1024;
		*vy = RC.r_y - 1024;
		
		/*ң������ֹ��̬Ʈ��*/
		if(*vx<4 && *vx>-4)
		{
			*vx=0;
		}
		if(*vy<4 && *vy>-4)
		{
			*vy=0;
		}

		*vx *= 10.0f;//0.00281;2/660=0.03
	  *vy *= 10.0f;//0.00281
	}
	else
	{
		*vx = (RC.d - RC.a) * 660 * ( 1 + RC.shift * 0.4f );//1684-1024=660;1024
		*vy = (RC.w - RC.s) * 660 * ( 1 + RC.shift * 0.4f );
		*vx *= 0.00381f;//0.00381
    *vy *= 0.0038f;//0.00381
	}
}

//��ȡ��ָ̨��
void Read_gimbal_opt(float *pitch, float *yaw)
{
	if( RC.l_y - 1024 != 0 | RC.l_x - 1024 != 0 )//�ж��Ƿ��м��̰��� 0û�� 1��
	{
		*pitch = RC.l_y - 1024;
		*yaw   = RC.l_x - 1024;
		
		/*ң������ֹ��̬Ʈ��*/
		if(*pitch<4 && *pitch>-4)
		{
			*pitch=0;
		}
		if(*yaw<4 && *yaw>-4)
		{
			*yaw=0;
		}
		
    *pitch *= 1.0f;//����������0.015
    *yaw   *= 10.0f; //����������
	}
	else 
	{ 
		*pitch =   RC.m_y ;
		*yaw   =   RC.m_x ;
    *pitch *=  0.2f;//����������0.015
    *yaw   *=  0.6f;//����������            
	}
}


