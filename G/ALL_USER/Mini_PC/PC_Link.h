#ifndef PC_LINK_H_
#define PC_LINK_H_

#include "main.h"

#define PC_Uart  huart6

typedef struct
{
	uint8_t new_data;	//新指令标志
	uint16_t x2d;			//2维的X坐标，坐标原点在图像左上角，单位是像素
	uint16_t y2d;			//2维的Y坐标
	float x3d;				//3维的X坐标，坐标原点在枪管中心，单位是cm
	float y3d;				//3维的Y坐标
	float z3d;				//3维的Z坐标
	float d3d;				//目标的距离
	float yaw;				//云台还需要转动的yaw轴角度
	float pitch;			//云台还需要转动的pitch轴角度
	uint32_t time;		//获取数据的时间
}VISION_DATA;

typedef union{
	uint8_t U8[4];
	uint16_t U16[2];
	uint32_t U32;
	float F;
	int I;
}FormatTrans;

extern VISION_DATA obj_data;

void Send_Robot_Id(uint16_t id);
void Send_Robot_Data(float shoot_speed, float yaw, float pitch, float roll, float move_speed_x, float move_speed_y, float move_speed_z);
void PC_Link_Init(void);
void PC_Link_Send(uint16_t cmd_id,uint8_t *tx_buff,uint16_t size);
void Reset_PC_Data(void);

#endif

