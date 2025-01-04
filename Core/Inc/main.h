/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define G_SN_Pin GPIO_PIN_0
#define G_SN_GPIO_Port GPIOA
#define Y_SN_Pin GPIO_PIN_1
#define Y_SN_GPIO_Port GPIOA
#define R_SN_Pin GPIO_PIN_2
#define R_SN_GPIO_Port GPIOA
#define PS_NS_Pin GPIO_PIN_3
#define PS_NS_GPIO_Port GPIOA
#define G_WE_Pin GPIO_PIN_0
#define G_WE_GPIO_Port GPIOB
#define Y_WE_Pin GPIO_PIN_1
#define Y_WE_GPIO_Port GPIOB
#define R_WE_Pin GPIO_PIN_2
#define R_WE_GPIO_Port GPIOB
#define PS_WE_Pin GPIO_PIN_3
#define PS_WE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
