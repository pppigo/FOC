/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZCV_Pin GPIO_PIN_1
#define ZCV_GPIO_Port GPIOF
#define RED_Pin GPIO_PIN_0
#define RED_GPIO_Port GPIOA
#define GREEN_Pin GPIO_PIN_1
#define GREEN_GPIO_Port GPIOA
#define BLUE_Pin GPIO_PIN_2
#define BLUE_GPIO_Port GPIOA
#define ZCW_Pin GPIO_PIN_1
#define ZCW_GPIO_Port GPIOB
#define HF_TIM1_CH1N_Pin GPIO_PIN_13
#define HF_TIM1_CH1N_GPIO_Port GPIOB
#define HF_TIM1_CH2N_Pin GPIO_PIN_14
#define HF_TIM1_CH2N_GPIO_Port GPIOB
#define HF_TIM1_CH3N_Pin GPIO_PIN_15
#define HF_TIM1_CH3N_GPIO_Port GPIOB
#define OC_SEL_GPIOA_11_Pin GPIO_PIN_11
#define OC_SEL_GPIOA_11_GPIO_Port GPIOA
#define OC_GPIOF_7_Pin GPIO_PIN_7
#define OC_GPIOF_7_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
