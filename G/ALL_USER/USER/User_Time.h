#ifndef USER_TIME_H
#define USER_TIME_H

#include "main.h"

extern void All_Time_Init(void);
extern void imu_pwm_set(uint16_t pwm);
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern void Magazine(uint16_t pwm);

#endif
