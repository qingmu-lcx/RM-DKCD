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
	int16_t gyro_x; 	/*!< ������ԭʼ���� */
	int16_t gyro_y; 	/*!< ������ԭʼ���� */
	int16_t gyro_z; 	/*!< ������ԭʼ���� */
  int16_t acc_x;		/*!< ���ٶ�ԭʼ���� */
	int16_t acc_y;		/*!< ���ٶ�ԭʼ���� */
	int16_t acc_z;		/*!< ���ٶ�ԭʼ���� */
  float 	fgyroD_x;	/*!< ������ת�١�/S */
	float 	fgyroD_y;	/*!< ������ת�١�/S */
	float 	fgyroD_z;	/*!< ������ת�١�/S */
  float 	faccG_x;	/*!< ���ٶ����� G */
	float 	faccG_y;	/*!< ���ٶ����� G */
	float 	faccG_z;	/*!< ���ٶ����� G */
	int16_t mag_x;		/*!< �ų����� */
	int16_t mag_y;		/*!< �ų����� */
	int16_t mag_z;		/*!< �ų����� */
  float  	temp_m;		/*!< �������¶� */
	int32_t pressure;	/*!< ��ѹֵPa */
  int32_t altitude;	/*!< ���θ߶�cm */
  float  	temp_b;		/*!< ��ѹ���¶� */
}	TTK_IMU;

extern TTK_IMU IMU;

void ATM_IMU_init(void);
void USART2_IRQHandler(void);
void IMU_Read_Data(uint8_t *Read_Data);

#endif

