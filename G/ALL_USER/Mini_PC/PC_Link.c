#include "PC_Link.h"
#include "usart.h"
#include "My_Crc.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "User_Debug.h"

//接收PC数据，一次最多传输255个字节，给了512个字节长度，防止DMA传输越界
static uint8_t pc_rx_buf[512];
static uint8_t pc_tx_buf[255];

VISION_DATA obj_data;	//接收数据

//开启PC通信
void PC_Link_Init(void)
{
	__HAL_UART_ENABLE_IT(&PC_Uart, UART_IT_RXNE);  //接收中断
  __HAL_UART_ENABLE_IT(&PC_Uart, UART_IT_IDLE);  //空闲中断
	Reset_PC_Data();	//防止出错，先复位PC数据
	HAL_UART_Receive_DMA(&PC_Uart,pc_rx_buf,512);//开始接收（使用DMA）
}

//给PC发送数据
void PC_Link_Send(uint16_t cmd_id,uint8_t *tx_buff,uint16_t size)
{
	static uint8_t SEQ = 0;

	pc_tx_buf[0] = 0xA5;	//起始标志
	pc_tx_buf[1] = (uint8_t)(size>>8);		//LEN的高8位
	pc_tx_buf[2] = (uint8_t)(size&0xff);	//LEN的低8位
	pc_tx_buf[3] = SEQ;
	if(SEQ < 255)
		SEQ++;
	else
		SEQ = 0;
	Append_CRC8_Check_Sum(pc_tx_buf,4);			//帧头校验
	pc_tx_buf[5] = (uint8_t)(cmd_id>>8);		//cmd_id的高8位
	pc_tx_buf[6] = (uint8_t)(cmd_id&0xff);	//cmd_id的低8位
	memcpy(pc_tx_buf+7, tx_buff, size);			//复制数组
	Append_CRC16_Check_Sum(pc_tx_buf,9+size);			//整包校验
	HAL_UART_Transmit(&PC_Uart,pc_tx_buf,size+9,0xffff);	//使用DMA进行数据发送有问题，待解决
}

void Send_Robot_Id(uint16_t id)
{
	uint16_t tx_buff[4];
	tx_buff[0] = 0x03ee;	//消息ID（用裁判系统时，这个值应该在0x0200到0x02ff之间）
	tx_buff[1] = 0x1001;	//发送者ID（用在裁判系统时，这里应该是机器人ID）
	tx_buff[2] = 0x1000;	//接收者ID（用在裁判系统时，这里应该是机器人ID）
	tx_buff[3] = id;			//这里时机器人实际ID，用来给视觉程序判断阵营的
	PC_Link_Send(0x0301,(uint8_t*)tx_buff,8);
}

void Send_Robot_Data(float shoot_speed, float yaw, float pitch, float roll, float move_speed_x, float move_speed_y, float move_speed_z)
{
	FormatTrans format;
	uint16_t tx_buff[17];
	tx_buff[0] = 0x0301;	//消息ID（用裁判系统时，这个值应该在0x0200到0x02ff之间）
	tx_buff[1] = 0x1001;	//发送者ID（用在裁判系统时，这里应该是机器人ID）
	tx_buff[2] = 0x1000;	//接收者ID（用在裁判系统时，这里应该是机器人ID）
	format.F = shoot_speed;
	tx_buff[3] = format.U16[0];		//装载子弹射速数据
	tx_buff[4] = format.U16[1];		//装载子弹射速数据
	format.F = yaw;
	tx_buff[5] = format.U16[0];			//装载yaw轴数据
	tx_buff[6] = format.U16[1];			//装载yaw轴数据
	format.F = pitch;
	tx_buff[7] = format.U16[0];			//装载pitch轴数据
	tx_buff[8] = format.U16[1];			//装载pitch轴数据
	format.F = roll;
	tx_buff[9] = format.U16[0];			//装载roll轴数据
	tx_buff[10] = format.U16[1];			//装载roll轴数据
	format.F = move_speed_x;
	tx_buff[11] = format.U16[0];			//装载move_speed_x数据
	tx_buff[12] = format.U16[1];		//装载move_speed_x数据
	format.F = move_speed_y;
	tx_buff[13] = format.U16[0];		//装载move_speed_y数据
	tx_buff[14] = format.U16[1];		//装载move_speed_y数据
	format.F = move_speed_z;
	tx_buff[15] = format.U16[0];		//装载move_speed_z数据
	tx_buff[16] = format.U16[1];		//装载move_speed_z数据
	
	PC_Link_Send(0x0301,(uint8_t*)tx_buff,34);//发送到PC
}

//重置PC数据
void Reset_PC_Data()
{
	obj_data.new_data = 0;
//	uart_printf("pc data reset\r\n");
}

//视觉数据解析
void Vision_Data()
{
	FormatTrans format;
	//获取2维坐标
	obj_data.x2d = ((uint16_t)pc_rx_buf[13])<<8 | pc_rx_buf[14];	
	obj_data.y2d = ((uint16_t)pc_rx_buf[15])<<8 | pc_rx_buf[16];
	//获取3维坐标
	format.U8[0] = pc_rx_buf[20];
	format.U8[1] = pc_rx_buf[19];
	format.U8[2] = pc_rx_buf[18];
	format.U8[3] = pc_rx_buf[17];
	obj_data.x3d = format.F;
	
	format.U8[0] = pc_rx_buf[24];
	format.U8[1] = pc_rx_buf[23];
	format.U8[2] = pc_rx_buf[22];
	format.U8[3] = pc_rx_buf[21];
	obj_data.y3d = format.F;

	format.U8[0] = pc_rx_buf[28];
	format.U8[1] = pc_rx_buf[27];
	format.U8[2] = pc_rx_buf[26];
	format.U8[3] = pc_rx_buf[25];
	obj_data.z3d = format.F;
	
	format.U8[0] = pc_rx_buf[32];
	format.U8[1] = pc_rx_buf[31];
	format.U8[2] = pc_rx_buf[30];
	format.U8[3] = pc_rx_buf[29];
	obj_data.d3d = format.F;
	//获取云台控制信息
	format.U8[0] = pc_rx_buf[36];
	format.U8[1] = pc_rx_buf[35];
	format.U8[2] = pc_rx_buf[34];
	format.U8[3] = pc_rx_buf[33];
	obj_data.yaw = format.F;
	
	format.U8[0] = pc_rx_buf[40];
	format.U8[1] = pc_rx_buf[39];
	format.U8[2] = pc_rx_buf[38];
	format.U8[3] = pc_rx_buf[37];
	obj_data.pitch = format.F;

	//表明获取新指令
	obj_data.new_data =  1;
	obj_data.time = HAL_GetTick();

}

//机器人之间的交互消息
void Robot_Msg()
{
	uint16_t msg_id = ((uint16_t)pc_rx_buf[7])<<8 | pc_rx_buf[8];
	uint16_t sender_id = ((uint16_t)pc_rx_buf[9])<<8 | pc_rx_buf[10];
	uint16_t receiver_id = ((uint16_t)pc_rx_buf[11])<<8 | pc_rx_buf[12];
	switch(msg_id)
	{
		case 0x0300://可能是视觉数据
			if(sender_id == 0x1000 && receiver_id == 0x1001)
				//确定是视觉数据
				Vision_Data();	
			break;
		case 0x03ff://可能是PC连接服务
			if(sender_id == 0x1000 && receiver_id == 0x1001)
				//受到连接请求，需要回应PC
				Send_Robot_Id(0x101);//在这里输入的ID是从裁判系统读到的ID
			break;
		default:break;
	}
}

//数据解析
void Analysis_RX_Buffer()
{
	uint8_t SOF = pc_rx_buf[0];
//	uint16_t LEN = ((uint16_t)pc_rx_buf[1])<<8 | pc_rx_buf[2];
	uint16_t cmd_id = ((uint16_t)pc_rx_buf[5])<<8 | pc_rx_buf[6];
	if(SOF == 0xA5)
	{
			switch(cmd_id)
			{
				case 0x0301:	//机器人交互消息
					Robot_Msg();
					break;
			default:break;
			}
	}

}

//串口6中断服务函数     
//C板3pin的串口
//需要把stm32f4xx_it里的屏蔽掉
void USART6_IRQHandler(void)
{
	volatile uint8_t receive;
	if(PC_Uart.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
			__HAL_UART_CLEAR_PEFLAG(&PC_Uart);
	}
	//空闲中断
	if(PC_Uart.Instance->SR & UART_FLAG_IDLE)
	{
		receive = PC_Uart.Instance->DR;	//先读SR，再读DR可以清除空闲中断标志位
		HAL_UART_DMAStop(&PC_Uart);			//先关闭DMA传输
		HAL_UART_Receive_DMA(&PC_Uart,pc_rx_buf,512);//重新开始接收（使用DMA）
		Analysis_RX_Buffer();
	}
}
