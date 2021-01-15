/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_6B_Pin GPIO_PIN_13
#define ENC_6B_GPIO_Port GPIOC
#define ENC_5A_Pin GPIO_PIN_14
#define ENC_5A_GPIO_Port GPIOC
#define ENC_5B_Pin GPIO_PIN_15
#define ENC_5B_GPIO_Port GPIOC
#define ENC_1A_Pin GPIO_PIN_0
#define ENC_1A_GPIO_Port GPIOC
#define ENC_1B_Pin GPIO_PIN_1
#define ENC_1B_GPIO_Port GPIOC
#define ENC_2A_Pin GPIO_PIN_2
#define ENC_2A_GPIO_Port GPIOC
#define ENC_2B_Pin GPIO_PIN_3
#define ENC_2B_GPIO_Port GPIOC
#define USER_ENC_A_Pin GPIO_PIN_0
#define USER_ENC_A_GPIO_Port GPIOA
#define USER_ENC_B_Pin GPIO_PIN_1
#define USER_ENC_B_GPIO_Port GPIOA
#define SHIFT_LA_Pin GPIO_PIN_4
#define SHIFT_LA_GPIO_Port GPIOA
#define SHIFT_SCK_Pin GPIO_PIN_5
#define SHIFT_SCK_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_6
#define PWM_3_GPIO_Port GPIOA
#define PWM_4_Pin GPIO_PIN_7
#define PWM_4_GPIO_Port GPIOA
#define ENC_3A_Pin GPIO_PIN_4
#define ENC_3A_GPIO_Port GPIOC
#define ENC_3B_Pin GPIO_PIN_5
#define ENC_3B_GPIO_Port GPIOC
#define PWM_2_Pin GPIO_PIN_0
#define PWM_2_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_1
#define PWM_1_GPIO_Port GPIOB
#define USER_ENC_BTN_Pin GPIO_PIN_2
#define USER_ENC_BTN_GPIO_Port GPIOB
#define ENC_4A_Pin GPIO_PIN_6
#define ENC_4A_GPIO_Port GPIOC
#define ENC_4B_Pin GPIO_PIN_7
#define ENC_4B_GPIO_Port GPIOC
#define ENC_8A_Pin GPIO_PIN_8
#define ENC_8A_GPIO_Port GPIOC
#define ENC_8B_Pin GPIO_PIN_9
#define ENC_8B_GPIO_Port GPIOC
#define PWM_5_Pin GPIO_PIN_8
#define PWM_5_GPIO_Port GPIOA
#define PWM_6_Pin GPIO_PIN_9
#define PWM_6_GPIO_Port GPIOA
#define PWM_7_Pin GPIO_PIN_10
#define PWM_7_GPIO_Port GPIOA
#define PWM_8_Pin GPIO_PIN_11
#define PWM_8_GPIO_Port GPIOA
#define ENC_7B_Pin GPIO_PIN_10
#define ENC_7B_GPIO_Port GPIOC
#define ENC_7A_Pin GPIO_PIN_11
#define ENC_7A_GPIO_Port GPIOC
#define ENC_6A_Pin GPIO_PIN_12
#define ENC_6A_GPIO_Port GPIOC
#define SHIFT_MOSI_Pin GPIO_PIN_5
#define SHIFT_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
