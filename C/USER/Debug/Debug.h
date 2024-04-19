#ifndef __DEBUG_H
#define __DEBUG_H

#include "main.h"

#define Debug_uart huart1

typedef struct
{
    int32_t pid_p;
    int32_t pid_i;
    int32_t pid_d;
} user_t;

void uart_printf(const char *fmt,...);
void Debug(void);
void Fire_PID_init(void);
void Fire_Debug_Chassis(void);

#endif

