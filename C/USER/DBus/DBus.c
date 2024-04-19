#include "DBus.h"
#include "usart.h"

#define DBus_uart huart2

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
REMOTE_CONTROL RC;
static uint8_t rx_buffer_num = 0;

//����ң��������
void Remote_Control_Init(void)
{
	__HAL_UART_ENABLE_IT(&DBus_uart, UART_IT_RXNE);  //�����ж�
  __HAL_UART_ENABLE_IT(&DBus_uart, UART_IT_IDLE);  //�����ж�
	Set_Remote_Default();	//��ʼ��ң��������
	HAL_UART_Receive_DMA(&DBus_uart,sbus_rx_buf[rx_buffer_num],SBUS_RX_BUF_NUM);//��ʼ���գ�ʹ��DMA��
}

//ң�����ݻָ�Ĭ��
void Set_Remote_Default(void)
{
	RC.l_s = 1;
	RC.l_x = 1024;
	RC.l_y = 1024;
	RC.r_s = 3;
	RC.r_x = 1024;
	RC.r_y = 1024;
	RC.m_x     = 0;
	RC.m_y     = 0;
	RC.m_z     = 0; 
	RC.press_l = 0;
	RC.press_r = 0;
	RC.key     = 0;
	RC.w       = 0;
	RC.s       = 0;
	RC.a       = 0;
	RC.d       = 0;
	RC.shift   = 0;
	RC.ctrl    = 0;
	RC.q       = 0;
	RC.e       = 0;
	RC.r			 = 0;
	RC.f			 = 0;
	RC.g			 = 0;
	RC.z			 = 0;
	RC.x			 = 0;
	RC.c			 = 0;
	RC.v			 = 0;
	RC.b			 = 0;
	RC.ch      = 0;
	RC.last_time = HAL_GetTick();
}

//ң�������Ź�
void Wdog_Remote(void)
{
	uint32_t time = HAL_GetTick();
	if((time - RC.last_time) > 20)
	{
		Set_Remote_Default();
	}
}

//ң�������
void Check_Remote(void)
{
	//������ݳ���ͳһ����ң�����������ݻָ�Ĭ��
	if (RC.l_x > 1684 || RC.l_x < 364	||	//��ҡ��x
			RC.l_y > 1684 || RC.l_y < 364	||	//��ҡ��y
			RC.r_x > 1684 || RC.r_x < 364 ||	//��ҡ��x
			RC.r_y > 1684 || RC.r_y < 364 ||	//��ҡ��y
			RC.l_s == 0   || RC.r_s == 0)			//���Ҳ���
	{
		Set_Remote_Default();
	}
}

//ң�������ݽ���
void Dbus_To_RC(uint8_t* dbus_buffer)
{
	if (dbus_buffer == NULL)
		return;
	RC.last_time = HAL_GetTick();	//��¼ң����ʱ�����ʱ�䣬���ڳ�ʱ����
	RC.r_x     = (((uint16_t)dbus_buffer[1]<<8)|(uint16_t)dbus_buffer[0])&0x07FF;//0000011111111111
	RC.r_y     = (((uint16_t)dbus_buffer[1]>>3)|((uint16_t)dbus_buffer[2]<<5))&0x07FF;
	RC.l_x     = (((uint16_t)dbus_buffer[2]>>6)|((uint16_t)dbus_buffer[3]<<2)|((uint16_t)dbus_buffer[4]<<10))&0x07FF;
	RC.l_y     = (((uint16_t)dbus_buffer[4]>>1)|((uint16_t)dbus_buffer[5]<<7))&0x07FF;
	RC.r_s     = (dbus_buffer[5]>>4)&0x03;
	RC.l_s     = (dbus_buffer[5]>>6)&0x03;
	RC.m_x     = ((int16_t)dbus_buffer[6]) | ((int16_t)dbus_buffer[7] << 8);
	RC.m_y     = ((int16_t)dbus_buffer[8]) | ((int16_t)dbus_buffer[9] << 8);
	RC.m_z     = ((int16_t)dbus_buffer[10]) | ((int16_t)dbus_buffer[11] << 8);   
	RC.press_l = dbus_buffer[12];
	RC.press_r = dbus_buffer[13];
	RC.key     = ((uint16_t)dbus_buffer[14] | ((uint16_t)dbus_buffer[15] << 8));
	RC.w       =  RC.key&0x0001;
	RC.s       = (RC.key>>1)&0x0001;
	RC.a       = (RC.key>>2)&0x0001;
	RC.d       = (RC.key>>3)&0x0001;
	RC.shift   = (RC.key>>4)&0x0001;
	RC.ctrl    = (RC.key>>5)&0x0001;
	RC.q       = (RC.key>>6)&0x0001;
	RC.e       = (RC.key>>7)&0x0001;
	RC.r			 = (RC.key>>8)&0x0001;
	RC.f			 = (RC.key>>9)&0x0001;
	RC.g			 = (RC.key>>10)&0x0001;
	RC.z			 = (RC.key>>11)&0x0001;
	RC.x			 = (RC.key>>12)&0x0001;
	RC.c			 = (RC.key>>13)&0x0001;
	RC.v			 = (RC.key>>14)&0x0001;
	RC.b			 = (RC.key>>15)&0x0001;
	RC.ch			 = ((uint16_t)dbus_buffer[16] | ((uint16_t)dbus_buffer[17] << 8));
	Check_Remote();  
}

//����2�жϷ�����
//��Ҫ��stm32f4xx_it������ε�
void USART2_IRQHandler(void)
{
	volatile uint8_t receive;
	uint8_t i = rx_buffer_num;	//��¼��һ��ʹ�õĽ��ջ���
	if(DBus_uart.Instance->SR & UART_FLAG_RXNE)//���յ�����
	{
			__HAL_UART_CLEAR_PEFLAG(&DBus_uart);
	}
	//�����ж�
	if(DBus_uart.Instance->SR & UART_FLAG_IDLE)
	{
		receive = DBus_uart.Instance->DR;//�ȶ�SR���ٶ�DR������������жϱ�־λ
		HAL_UART_DMAStop(&DBus_uart);	//�ȹر�DMA����
		rx_buffer_num ++;
		if(rx_buffer_num == 2)
			rx_buffer_num = 0;
		HAL_UART_Receive_DMA(&DBus_uart,sbus_rx_buf[rx_buffer_num],SBUS_RX_BUF_NUM);//���¿�ʼ���գ�ʹ��DMA��
		Dbus_To_RC(sbus_rx_buf[i]);	//������һ�ν��յ�������
	}
}


