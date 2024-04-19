#include "ATK_IMU.h"
#include "cmsis_os.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"


uint8_t IMU_data[256];
TTK_IMU IMU;

void ATM_IMU_init(void)
{
	__HAL_UART_ENABLE_IT(&ATK_uart, UART_IT_RXNE);  //�����ж�
  __HAL_UART_ENABLE_IT(&ATK_uart, UART_IT_IDLE);  //�����ж�
	
	HAL_UART_Receive_DMA(&ATK_uart,IMU_data,256);//��ʼ���գ�ʹ��DMA��
}

void UART4_IRQHandler(void)     //Ҫ�� stm32f4xx_it.c�����  void USART2_IRQHandler(void) ע�͵�
{
	uint8_t receive;
	if(__HAL_UART_GET_FLAG(&ATK_uart, UART_FLAG_IDLE) != RESET)//��⵽������·
	{
		__HAL_UART_CLEAR_IDLEFLAG(&ATK_uart);
		
		receive = ATK_uart.Instance->SR;
		receive = ATK_uart.Instance->DR;
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		
		HAL_UART_DMAStop(&ATK_uart);			//�ȹر�DMA����
		IMU_Read_Data(IMU_data);		//��ȡ����ϵͳ����
		memset(IMU_data, 0, 256);
		HAL_UART_Receive_DMA(&ATK_uart,IMU_data,256);//���¿�ʼ���գ�ʹ��DMA��
	}
}

void IMU_Read_Data(uint8_t *Read_Data)
{
	if(Read_Data == NULL)
	{
		return;
	}
	if(Read_Data[0] == 0x55)
	{
		if(Read_Data[1] == 0x55)
		{
			switch(Read_Data[2])
			{
				case 0x01:
				{
					IMU.Roll 	= (float)((int16_t)(Read_Data[5]<<8)|Read_Data[4])/32768*180;
					IMU.Pitch = (float)((int16_t)(Read_Data[7]<<8)|Read_Data[6])/32768*180;
					IMU.Yaw 	= (float)((int16_t)(Read_Data[9]<<8)|Read_Data[8])/32768*180;
					
				}
				case 0x02:
				{
					IMU.q0 	= (float)((int16_t)(Read_Data[5]<<8)|Read_Data[4])/32768;
					IMU.q1 	= (float)((int16_t)(Read_Data[7]<<8)|Read_Data[6])/32768;
					IMU.q2 	= (float)((int16_t)(Read_Data[9]<<8)|Read_Data[8])/32768;
					IMU.q3 	= (float)((int16_t)(Read_Data[11]<<8)|Read_Data[10])/32768;
					
				}
				case 0x03:
				{
					IMU.gyro_x 	= (int16_t)(Read_Data[5]<<8)|Read_Data[4];
					IMU.gyro_y 	= (int16_t)(Read_Data[7]<<8)|Read_Data[6];
					IMU.gyro_z 	= (int16_t)(Read_Data[9]<<8)|Read_Data[8];
					
					IMU.fgyroD_x 	= (float)IMU.gyro_x/32768*4;
					IMU.fgyroD_y 	= (float)IMU.gyro_y/32768*4;
					IMU.fgyroD_z 	= (float)IMU.gyro_z/32768*4;
					
					
					IMU.acc_x 	= (int16_t)(Read_Data[11]<<8)|Read_Data[10];
					IMU.acc_y 	= (int16_t)(Read_Data[13]<<8)|Read_Data[12];
					IMU.acc_z 	= (int16_t)(Read_Data[16]<<8)|Read_Data[14];
					
					IMU.faccG_x 	= (float)IMU.acc_x/32768*2000;
					IMU.faccG_y 	= (float)IMU.acc_y/32768*2000;
					IMU.faccG_z 	= (float)IMU.acc_z/32768*2000;
		
				}
			}
		}	
	}
}

