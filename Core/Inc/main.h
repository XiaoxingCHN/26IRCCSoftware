/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_24V_2_Pin GPIO_PIN_13
#define POWER_24V_2_GPIO_Port GPIOC
#define POWER_24V_1_Pin GPIO_PIN_14
#define POWER_24V_1_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define CS2_ACCEL_Pin GPIO_PIN_0
#define CS2_ACCEL_GPIO_Port GPIOC
#define CS2_GYRO_Pin GPIO_PIN_3
#define CS2_GYRO_GPIO_Port GPIOC
#define ID6_Pin GPIO_PIN_5
#define ID6_GPIO_Port GPIOA
#define DCMI_PWDN_Pin GPIO_PIN_5
#define DCMI_PWDN_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define ID1_Pin GPIO_PIN_15
#define ID1_GPIO_Port GPIOE
#define DCMI_REST_Pin GPIO_PIN_12
#define DCMI_REST_GPIO_Port GPIOB
#define ID5_Pin GPIO_PIN_10
#define ID5_GPIO_Port GPIOD
#define ID4_Pin GPIO_PIN_7
#define ID4_GPIO_Port GPIOD
#define ID2_Pin GPIO_PIN_3
#define ID2_GPIO_Port GPIOB
#define ID3_Pin GPIO_PIN_4
#define ID3_GPIO_Port GPIOB
#define ID7_Pin GPIO_PIN_8
#define ID7_GPIO_Port GPIOB
#define ID8_Pin GPIO_PIN_9
#define ID8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
