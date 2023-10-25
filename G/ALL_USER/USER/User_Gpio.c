#include "User_Gpio.h"
#include "gpio.h"


void Laser_On(void)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
}
void Laser_Off(void)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET);
}


