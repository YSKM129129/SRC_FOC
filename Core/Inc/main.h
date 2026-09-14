/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define DRV_ENBLE_Pin GPIO_PIN_14
#define DRV_ENBLE_GPIO_Port GPIOC
#define DRV_cotr_Pin GPIO_PIN_15
#define DRV_cotr_GPIO_Port GPIOC
#define ADC_IU_Pin GPIO_PIN_0
#define ADC_IU_GPIO_Port GPIOA
#define ADC_IV_Pin GPIO_PIN_1
#define ADC_IV_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOA
#define ADC_V_Pin GPIO_PIN_4
#define ADC_V_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define AS5047P_CS_Pin GPIO_PIN_2
#define AS5047P_CS_GPIO_Port GPIOB
#define ADC_T_Pin GPIO_PIN_11
#define ADC_T_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define PWM_U_Pin GPIO_PIN_8
#define PWM_U_GPIO_Port GPIOA
#define PWM_V_Pin GPIO_PIN_9
#define PWM_V_GPIO_Port GPIOA
#define PWM_W_Pin GPIO_PIN_10
#define PWM_W_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
