/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Sign_Pin GPIO_PIN_1
#define Sign_GPIO_Port GPIOC
#define Eight_hour_Pin GPIO_PIN_3
#define Eight_hour_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_0
#define RST_GPIO_Port GPIOA
#define RST_EXTI_IRQn EXTI0_IRQn
#define Four_hour_Pin GPIO_PIN_1
#define Four_hour_GPIO_Port GPIOA
#define Two_hour_Pin GPIO_PIN_3
#define Two_hour_GPIO_Port GPIOA
#define One_hour_Pin GPIO_PIN_5
#define One_hour_GPIO_Port GPIOA
#define Half_hour_Pin GPIO_PIN_7
#define Half_hour_GPIO_Port GPIOA
#define NTP_On_Pin GPIO_PIN_5
#define NTP_On_GPIO_Port GPIOC
#define PPS_Pin GPIO_PIN_4
#define PPS_GPIO_Port GPIOD
#define PPS_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
