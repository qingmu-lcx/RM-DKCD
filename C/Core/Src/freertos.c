/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "Debug.h"
#include "Friction_wheel.h"
#include "Chassis.h"
#include "USER_CAN.h"
//#include "AS5048A.h"
#include "Fire.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Chassis_TaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Ammo_TaskHandle;
osThreadId Mini_PC_TaskHandle;
osThreadId Judge_TaskHandle;
osThreadId Debug_TaskHandle;
osThreadId IWDG_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Chassis_Task(void const * argument);
void Start_Gimbal_Task(void const * argument);
void Start_Ammo_Task(void const * argument);
void Start_Mini_PC_Task(void const * argument);
void Start_Judge_Task(void const * argument);
void Start_Debug_Task(void const * argument);
void Start_IWDG_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Start_Chassis_Task, osPriorityNormal, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Gimbal_Task */
  osThreadDef(Gimbal_Task, Start_Gimbal_Task, osPriorityIdle, 0, 128);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

  /* definition and creation of Ammo_Task */
  osThreadDef(Ammo_Task, Start_Ammo_Task, osPriorityIdle, 0, 128);
  Ammo_TaskHandle = osThreadCreate(osThread(Ammo_Task), NULL);

  /* definition and creation of Mini_PC_Task */
  osThreadDef(Mini_PC_Task, Start_Mini_PC_Task, osPriorityIdle, 0, 128);
  Mini_PC_TaskHandle = osThreadCreate(osThread(Mini_PC_Task), NULL);

  /* definition and creation of Judge_Task */
  osThreadDef(Judge_Task, Start_Judge_Task, osPriorityIdle, 0, 128);
  Judge_TaskHandle = osThreadCreate(osThread(Judge_Task), NULL);

  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, Start_Debug_Task, osPriorityIdle, 0, 128);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of IWDG_Task */
  osThreadDef(IWDG_Task, Start_IWDG_Task, osPriorityIdle, 0, 128);
  IWDG_TaskHandle = osThreadCreate(osThread(IWDG_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Chassis_Task */
/**
  * @brief  Function implementing the Chassis_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Chassis_Task */
void Start_Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Chassis_Task */
  /* Infinite loop */
	Chassis_init();
  for(;;)
  {
//		Chassis_move();
//		Chassis_out(1000,1000,1000,1000);
//		Chassis_out(Fire.Target_value,0,0,0);
		CAN1_to_Motor(0x200);
//		CAN2_to_Motor(0x200);
    osDelay(1);
  }
  /* USER CODE END Start_Chassis_Task */
}

/* USER CODE BEGIN Header_Start_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Gimbal_Task */
void Start_Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
		CAN1_to_Motor(0x1FF);
//		CAN2_to_Motor(0x1FF);
    osDelay(10);
  }
  /* USER CODE END Start_Gimbal_Task */
}

/* USER CODE BEGIN Header_Start_Ammo_Task */
/**
* @brief Function implementing the Ammo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Ammo_Task */
void Start_Ammo_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Ammo_Task */
  /* Infinite loop */
  for(;;)
  {
//		if(HAL_GPIO_ReadPin(HW_GPIO_Port,HW_Pin) == 1)
//			uart_printf("1\r\n");
//		else if(HAL_GPIO_ReadPin(HW_GPIO_Port,HW_Pin) == 0)
//			uart_printf("0\r\n");
		Friction_wheel_speed(6000, -6000);
    osDelay(20);
  }
  /* USER CODE END Start_Ammo_Task */
}

/* USER CODE BEGIN Header_Start_Mini_PC_Task */
/**
* @brief Function implementing the Mini_PC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Mini_PC_Task */
void Start_Mini_PC_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Mini_PC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Mini_PC_Task */
}

/* USER CODE BEGIN Header_Start_Judge_Task */
/**
* @brief Function implementing the Judge_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Judge_Task */
void Start_Judge_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Judge_Task */
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(1);
  }
  /* USER CODE END Start_Judge_Task */
}

/* USER CODE BEGIN Header_Start_Debug_Task */
/**
* @brief Function implementing the Debug_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Debug_Task */
void Start_Debug_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Debug_Task */
  /* Infinite loop */
	Fire_PID_init();
  for(;;)
  {
		Fire_Receiving_process();//Fire
		Fire_Debug_Chassis();	
//		Debug();
    osDelay(12);
  }
  /* USER CODE END Start_Debug_Task */
}

/* USER CODE BEGIN Header_Start_IWDG_Task */
/**
* @brief Function implementing the IWDG_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IWDG_Task */
void Start_IWDG_Task(void const * argument)
{
  /* USER CODE BEGIN Start_IWDG_Task */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		HAL_IWDG_Refresh(&hiwdg);
    osDelay(400);
  }
  /* USER CODE END Start_IWDG_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
