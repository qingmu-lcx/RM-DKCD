#include "Debug.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"

#include "USER_CAN.h"
#include "DBus.h"
#include "ATK_IMU.h"
#include "Fire.h"
#include "Chassis.h"


/**
	* @brief          通过串口1的DMA发送字符串
	* @param[in]      fmt:	需要发送的字符串
	* @retval         none
	*/
void uart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
		HAL_UART_Transmit(&Debug_uart,tx_buf,len,10);
}


void Debug(void)
{
							/*CAN电机读取*/
//	uart_printf("Speed=%d\r\n",CAN1_Motor[0].Speed);
//	uart_printf("Angle=%d\r\n",CAN1_Motor[4].Angle);
//	uart_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN1_Motor[0].Angle,CAN1_Motor[1].Angle,CAN1_Motor[2].Angle,CAN1_Motor[3].Angle);
//	uart_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN1_Motor[4].Angle,CAN1_Motor[5].Angle,CAN1_Motor[6].Angle,CAN1_Motor[7].Angle);
//	uart_printf("A1=%d  A2=%d  A3=%d  A4=%d\r\n",CAN2_Motor[0].Angle,CAN2_Motor[1].Angle,CAN2_Motor[2].Angle,CAN2_Motor[3].Angle);
//	uart_printf("A5=%d  A6=%d  A7=%d  A8=%d\r\n",CAN2_Motor[4].Angle,CAN2_Motor[5].Angle,CAN2_Motor[6].Angle,CAN2_Motor[7].Angle);
//	uart_printf("Temperate=%d\r\n",CAN1_Motor[0].Temperate);
	
	            /*DBus遥控器数据*/ 
//	uart_printf("RC.l_x=%d  RC.r_x=%d\r\n", RC.l_x, RC.r_x);
//	uart_printf("RC.l_y=%d  RC.r_y=%d\r\n", RC.l_y, RC.r_y);
//	uart_printf("RC.l_s=%d  RC.r_s=%d\r\n", RC.l_s, RC.r_s);
//	uart_printf("RC.m_x=%d  RC.m_y\r\n", RC.m_x, RC.m_y);
//	uart_printf("w=%d  s=%d  a=%d  d=%d\r\n", RC.w, RC.s, RC.a, RC.d);
		
	
//	uart_printf("%d %d %d\r\n",ANO.PID1_P,ANO.PID1_I,ANO.PID1_D);
	
	
}

void Fire_PID_init(void)
{
	Fire.PID1_P = 3;
	Fire.PID1_I = 0.002;
	Fire.PID1_D = 0;
	
	Fire.PID2_P = 3;
	Fire.PID2_I = 0.002;
	Fire.PID2_D = 0;
}

void Fire_Debug_Chassis(void)
{
	if(Fire.Flag == 1)
	{
		float pid_temp[3] = {Fire.PID1_P,Fire.PID1_I,Fire.PID1_D};
		int32_t M1_Speed = (int32_t)CAN1_Motor[0].Speed;
		set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, &pid_temp, 3);
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &M1_Speed, 1);
	}
	else
	{
		Chassis_out(0,0,0,0);
	}
}






