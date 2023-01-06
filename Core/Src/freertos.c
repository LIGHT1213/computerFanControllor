/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
ring_buffer RB;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId initTaskHandle;
osThreadId fanControlTaskHandle;
osThreadId rgbTaskHandle;
osThreadId shellTaskHandle;
osSemaphoreId uartSendBinarySemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void osInitTask(void const * argument);
void osFanControlTask(void const * argument);
void osRgbTask(void const * argument);
void osShellTask(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of uartSendBinarySem */
  osSemaphoreDef(uartSendBinarySem);
  uartSendBinarySemHandle = osSemaphoreCreate(osSemaphore(uartSendBinarySem), 1);

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
  /* definition and creation of initTask */
  osThreadDef(initTask, osInitTask, osPriorityNormal, 0, 128);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* definition and creation of fanControlTask */
  osThreadDef(fanControlTask, osFanControlTask, osPriorityAboveNormal, 0, 256);
  fanControlTaskHandle = osThreadCreate(osThread(fanControlTask), NULL);

  /* definition and creation of rgbTask */
  osThreadDef(rgbTask, osRgbTask, osPriorityAboveNormal, 0, 256);
  rgbTaskHandle = osThreadCreate(osThread(rgbTask), NULL);

  /* definition and creation of shellTask */
  osThreadDef(shellTask, osShellTask, osPriorityIdle, 0, 128);
  shellTaskHandle = osThreadCreate(osThread(shellTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_osInitTask */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_osInitTask */
void osInitTask(void const * argument)
{
  /* USER CODE BEGIN osInitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END osInitTask */
}

/* USER CODE BEGIN Header_osFanControlTask */
/**
* @brief Function implementing the fanControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_osFanControlTask */
void osFanControlTask(void const * argument)
{
  /* USER CODE BEGIN osFanControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END osFanControlTask */
}

/* USER CODE BEGIN Header_osRgbTask */
/**
* @brief Function implementing the rgbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_osRgbTask */
void osRgbTask(void const * argument)
{
  /* USER CODE BEGIN osRgbTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END osRgbTask */
}

/* USER CODE BEGIN Header_osShellTask */
/**
* @brief Function implementing the shellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_osShellTask */
void osShellTask(void const * argument)
{
  /* USER CODE BEGIN osShellTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END osShellTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

