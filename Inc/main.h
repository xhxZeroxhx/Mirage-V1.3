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
#include "stm32f1xx_hal.h"

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
#define Board_Led_Pin GPIO_PIN_13
#define Board_Led_GPIO_Port GPIOC
#define TLC5947_BLANK4_Pin GPIO_PIN_0
#define TLC5947_BLANK4_GPIO_Port GPIOB
#define TLC5947_BLANK3_Pin GPIO_PIN_1
#define TLC5947_BLANK3_GPIO_Port GPIOB
#define Hall_sensor_Pin GPIO_PIN_10
#define Hall_sensor_GPIO_Port GPIOA
#define Hall_sensor_EXTI_IRQn EXTI15_10_IRQn
#define TLC5947_BLANK2_Pin GPIO_PIN_3
#define TLC5947_BLANK2_GPIO_Port GPIOB
#define TLC5947_BLANK1_Pin GPIO_PIN_4
#define TLC5947_BLANK1_GPIO_Port GPIOB
#define TLC5947_XLAT_Pin GPIO_PIN_6
#define TLC5947_XLAT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern uint16_t g_degreeCount;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
