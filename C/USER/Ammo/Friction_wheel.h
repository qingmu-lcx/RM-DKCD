#ifndef _Friction_wheel_H_
#define _Friction_wheel_H_

#include "main.h"

extern int16_t FW_ID1_Speed;
extern int16_t FW_ID2_Speed;

void uart_send_speed(int16_t speed,uint8_t ID);
void Friction_wheel_speed(int16_t ID1_Sspeed,int16_t ID2_Sspeed);

void Friction_wheel_init(void);
void USART5_IRQHandler(void);
void FW_Read_Data(uint8_t *Read_Data);

#endif


