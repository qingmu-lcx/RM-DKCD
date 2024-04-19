#include "Friction_wheel.h"
#include "cmsis_os.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "Debug.h"

#define FW_uart huart3

int16_t FW_flag = 0;

void Friction_wheel_speed(int16_t ID1_Speed,int16_t ID2_Speed)
{
	if(ID1_Speed != FW_flag)
	{
		uart_send_speed(ID1_Speed, 1);
		osDelay(2);
		uart_send_speed(ID2_Speed, 2);
		osDelay(2);
		FW_flag = ID1_Speed;
	}
}

void uart_send_speed(int16_t speed,uint8_t ID)//speed为设置的转速，单位为rpm;ID为电调ID
{
	static uint8_t tx_buf[8];
	tx_buf[0]=0xAA;
	tx_buf[1]=0xAF;
	tx_buf[2]=0xE0;
	tx_buf[3]=0x03;
	tx_buf[4]=ID;
	tx_buf[5]=(uint8_t)(((uint16_t)speed>>8)&0x00ff);
	tx_buf[6]=(uint8_t)((uint16_t)speed&0x00ff);
	tx_buf[7]=0;
	for(uint8_t i=0;i<=6;i++)
		tx_buf[7]+=tx_buf[i];
	
	HAL_UART_Transmit_DMA(&FW_uart, tx_buf, 8);//串口发送函数
}

