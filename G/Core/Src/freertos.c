/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "rng.h"

#include "AHRS_Task.h"
#include "Calibrate_Task.h"

#include "User_Debug.h"
#include "User_Dbus.h"
#include "User_led.h"
#include "Gimbal.h"
#include "Chassis.h"

#include "PC_Link.h"

#include "Shoot.h"
#include "Fire.h"
//#include "Chassis.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
osThreadId Task_ChassisHandle;
osThreadId Task_GimbalHandle;
osThreadId Task_DebugHandle;
osThreadId Task_AHRSHandle;
osThreadId Task_IWDGHandle;
osThreadId myTask06Handle;
osThreadId Task_Ammo_ShootHandle;
osThreadId Task_Ammo_FireHandle;
osThreadId Task_Mini_PCHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Task_Chassis(void const * argument);
void Start_Task_Gimbal(void const * argument);
void Start_Task_Debug(void const * argument);
void Start_Task_AHRS(void const * argument);
void Start_Task_IWDG(void const * argument);
void StartTask06(void const * argument);
void Start_Task_Ammo_Shoot(void const * argument);
void Start_Task_Ammo_Fire(void const * argument);
void Start_Task__Mini_PC(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of Task_Chassis */
  osThreadDef(Task_Chassis, Start_Task_Chassis, osPriorityNormal, 0, 1024);
  Task_ChassisHandle = osThreadCreate(osThread(Task_Chassis), NULL);

  /* definition and creation of Task_Gimbal */
  osThreadDef(Task_Gimbal, Start_Task_Gimbal, osPriorityIdle, 0, 1024);
  Task_GimbalHandle = osThreadCreate(osThread(Task_Gimbal), NULL);

  /* definition and creation of Task_Debug */
  osThreadDef(Task_Debug, Start_Task_Debug, osPriorityIdle, 0, 512);
  Task_DebugHandle = osThreadCreate(osThread(Task_Debug), NULL);

  /* definition and creation of Task_AHRS */
  osThreadDef(Task_AHRS, Start_Task_AHRS, osPriorityIdle, 0, 1024);
  Task_AHRSHandle = osThreadCreate(osThread(Task_AHRS), NULL);

  /* definition and creation of Task_IWDG */
  osThreadDef(Task_IWDG, Start_Task_IWDG, osPriorityIdle, 0, 256);
  Task_IWDGHandle = osThreadCreate(osThread(Task_IWDG), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, StartTask06, osPriorityIdle, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of Task_Ammo_Shoot */
  osThreadDef(Task_Ammo_Shoot, Start_Task_Ammo_Shoot, osPriorityIdle, 0, 512);
  Task_Ammo_ShootHandle = osThreadCreate(osThread(Task_Ammo_Shoot), NULL);

  /* definition and creation of Task_Ammo_Fire */
  osThreadDef(Task_Ammo_Fire, Start_Task_Ammo_Fire, osPriorityIdle, 0, 512);
  Task_Ammo_FireHandle = osThreadCreate(osThread(Task_Ammo_Fire), NULL);

  /* definition and creation of Task_Mini_PC */
  osThreadDef(Task_Mini_PC, Start_Task__Mini_PC, osPriorityIdle, 0, 256);
  Task_Mini_PCHandle = osThreadCreate(osThread(Task_Mini_PC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Task_Chassis */
/**
  * @brief  Function implementing the Task_Chassis thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Task_Chassis */
void Start_Task_Chassis(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Chassis */
  /* Infinite loop */
	Chassis_Init();
  for(;;)
  {
    Chassis_Control();
		Tx_Gimbal_To_Chassis(0x221);//发送底盘电机数据给底盘主控
    osDelay(2);
  }
  /* USER CODE END Start_Task_Chassis */
}

/* USER CODE BEGIN Header_Start_Task_Gimbal */
/**
* @brief Function implementing the Task_Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Gimbal */
void Start_Task_Gimbal(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Gimbal */
  /* Infinite loop */
	Gimbal_Init();
  for(;;)
  {
    Gimbal_Control();
    osDelay(2);
  }
  /* USER CODE END Start_Task_Gimbal */
}

/* USER CODE BEGIN Header_Start_Task_Debug */
/**
* @brief Function implementing the Task_Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Debug */
void Start_Task_Debug(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Debug */
  /* Infinite loop */
  for(;;)
  {
		Debug_Usart();
    osDelay(20);
		//usart_printf("3\r\n");
  }
  /* USER CODE END Start_Task_Debug */
}

/* USER CODE BEGIN Header_Start_Task_AHRS */
/**
* @brief Function implementing the Task_AHRS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_AHRS */
void Start_Task_AHRS(void const * argument)
{
  /* USER CODE BEGIN Start_Task_AHRS */
  /* Infinite loop */
//	usart_printf("3\r\n");
	AHRS_Init();
	Cali_Param_Init();
  for(;;)
  {
		
		AHRS_Read();//陀螺仪
		Calibrate_AHRS();//陀螺仪校准
    osDelay(1);
  }
  /* USER CODE END Start_Task_AHRS */
}

/* USER CODE BEGIN Header_Start_Task_IWDG */
/**
* @brief Function implementing the Task_IWDG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_IWDG */
void Start_Task_IWDG(void const * argument)
{
  /* USER CODE BEGIN Start_Task_IWDG */
  /* Infinite loop */
  for(;;)
  {
//		usart_printf("3\r\n");
		Wdog_Remote();
		Led_RGB_Flow();
    osDelay(1);
  }
  /* USER CODE END Start_Task_IWDG */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	usart_printf("2\r\n");
  for(;;)
  {
		

    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_Start_Task_Ammo_Shoot */
/**
* @brief Function implementing the Task_Ammo_Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Ammo_Shoot */
void Start_Task_Ammo_Shoot(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Ammo_Shoot */
  /* Infinite loop */
	Shoot_Init();
  for(;;)
  {
    Shoot_Control();
    osDelay(2);
  }
  /* USER CODE END Start_Task_Ammo_Shoot */
}

/* USER CODE BEGIN Header_Start_Task_Ammo_Fire */
/**
* @brief Function implementing the Task_Ammo_Fire thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Ammo_Fire */
void Start_Task_Ammo_Fire(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Ammo_Fire */
  /* Infinite loop */
	Fire_Init();
  for(;;)
  {
		Fire_Control();
    osDelay(1);
  }
  /* USER CODE END Start_Task_Ammo_Fire */
}

/* USER CODE BEGIN Header_Start_Task__Mini_PC */
/**
* @brief Function implementing the Task_Mini_PC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task__Mini_PC */
void Start_Task__Mini_PC(void const * argument)
{
  /* USER CODE BEGIN Start_Task__Mini_PC */
  /* Infinite loop */
  for(;;)
  {
		Send_Robot_Id(001);// R  0x003 B //给pc发机器人id     001、002...蓝方, 101、102...红方
//    Send_Robot_Id(Judge_User.Robot_ID);//0x103 R  0x003 B //给pc发机器人id
		Send_Robot_Data(Judge_User.Shooter_ID1_17mm_Speed_Limit,0,0,0,0,0,0);//给pc发射速上限
		RNG_Number = HAL_RNG_GetRandomNumber(&hrng) & 0x02;//max 2
    osDelay(500);
  }
  /* USER CODE END Start_Task__Mini_PC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
