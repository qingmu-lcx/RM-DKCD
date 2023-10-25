#ifndef USER_DEBUG_H
#define USER_DEBUG_H

#include "main.h"

extern void usb_printf(const char *fmt,...);
extern void usart_printf(const char *fmt,...);
extern void Debug_Usb(void);
extern void Debug_Usart(void);
#endif
