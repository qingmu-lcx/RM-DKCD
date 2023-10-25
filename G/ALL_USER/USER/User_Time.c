#include "User_Time.h"
#include "tim.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;

void All_Time_Init(void)
{
	HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	
	
	HAL_TIM_Base_Start(&htim10);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}


void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}


void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
void Magazine(uint16_t pwm)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
}

