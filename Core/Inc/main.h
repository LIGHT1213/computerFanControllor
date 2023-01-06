/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern ring_buffer RB;
extern osSemaphoreId uartSendBinarySemHandle;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIVE_LED_Pin GPIO_PIN_13
#define LIVE_LED_GPIO_Port GPIOC
#define fan7In_Pin GPIO_PIN_0
#define fan7In_GPIO_Port GPIOA
#define fan8In_Pin GPIO_PIN_1
#define fan8In_GPIO_Port GPIOA
#define fan7Out_Pin GPIO_PIN_2
#define fan7Out_GPIO_Port GPIOA
#define fan8Out_Pin GPIO_PIN_3
#define fan8Out_GPIO_Port GPIOA
#define fan5In_Pin GPIO_PIN_6
#define fan5In_GPIO_Port GPIOA
#define fan6In_Pin GPIO_PIN_7
#define fan6In_GPIO_Port GPIOA
#define fan5Out_Pin GPIO_PIN_0
#define fan5Out_GPIO_Port GPIOB
#define fan6Out_Pin GPIO_PIN_1
#define fan6Out_GPIO_Port GPIOB
#define fan3Out_Pin GPIO_PIN_10
#define fan3Out_GPIO_Port GPIOB
#define fan4Out_Pin GPIO_PIN_11
#define fan4Out_GPIO_Port GPIOB
#define RGB1_Pin GPIO_PIN_6
#define RGB1_GPIO_Port GPIOC
#define RGB2_Pin GPIO_PIN_7
#define RGB2_GPIO_Port GPIOC
#define RGB3_Pin GPIO_PIN_8
#define RGB3_GPIO_Port GPIOC
#define RGB4_Pin GPIO_PIN_9
#define RGB4_GPIO_Port GPIOC
#define fan1In_Pin GPIO_PIN_8
#define fan1In_GPIO_Port GPIOA
#define fan2In_Pin GPIO_PIN_9
#define fan2In_GPIO_Port GPIOA
#define Fan1Out_Pin GPIO_PIN_10
#define Fan1Out_GPIO_Port GPIOA
#define Fan2Out_Pin GPIO_PIN_11
#define Fan2Out_GPIO_Port GPIOA
#define fan3In_Pin GPIO_PIN_15
#define fan3In_GPIO_Port GPIOA
#define fan4In_Pin GPIO_PIN_3
#define fan4In_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
