#ifndef _ATK_IMU_H_
#define _ATK_IMU_H_

#include "main.h"

#define ATK_uart huart4

typedef struct
{
  float 	Roll;
  float 	Pitch;
  float 	Yaw;
	float 	q0, q1, q2, q3;
	int16_t gyro_x; 	/*!< 陀螺仪原始数据 */
	int16_t gyro_y; 	/*!< 陀螺仪原始数据 */
	int16_t gyro_z; 	/*!< 陀螺仪原始数据 */
  int16_t acc_x;		/*!< 加速度原始数据 */
	int16_t acc_y;		/*!< 加速度原始数据 */
	int16_t acc_z;		/*!< 加速度原始数据 */
  float 	fgyroD_x;	/*!< 陀螺仪转速°/S */
	float 	fgyroD_y;	/*!< 陀螺仪转速°/S */
	float 	fgyroD_z;	/*!< 陀螺仪转速°/S */
  float 	faccG_x;	/*!< 加速度重力 G */
	float 	faccG_y;	/*!< 加速度重力 G */
	float 	faccG_z;	/*!< 加速度重力 G */
	int16_t mag_x;		/*!< 磁场数据 */
	int16_t mag_y;		/*!< 磁场数据 */
	int16_t mag_z;		/*!< 磁场数据 */
  float  	temp_m;		/*!< 磁力计温度 */
	int32_t pressure;	/*!< 气压值Pa */
  int32_t altitude;	/*!< 海拔高度cm */
  float  	temp_b;		/*!< 气压计温度 */
}	TTK_IMU;

extern TTK_IMU IMU;

void ATM_IMU_init(void);
void USART2_IRQHandler(void);
void IMU_Read_Data(uint8_t *Read_Data);

#endif

