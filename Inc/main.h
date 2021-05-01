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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIB_SYNC_Pin GPIO_PIN_15
#define DIB_SYNC_GPIO_Port GPIOC
#define PCB_ID0_Pin GPIO_PIN_0
#define PCB_ID0_GPIO_Port GPIOF
#define PCB_ID1_Pin GPIO_PIN_1
#define PCB_ID1_GPIO_Port GPIOF
#define P1_0_Pin GPIO_PIN_0
#define P1_0_GPIO_Port GPIOA
#define P1_1_Pin GPIO_PIN_1
#define P1_1_GPIO_Port GPIOA
#define P1_2_Pin GPIO_PIN_2
#define P1_2_GPIO_Port GPIOA
#define P1_3_Pin GPIO_PIN_3
#define P1_3_GPIO_Port GPIOA
#define P1_4_Pin GPIO_PIN_4
#define P1_4_GPIO_Port GPIOA
#define P1_5_Pin GPIO_PIN_5
#define P1_5_GPIO_Port GPIOA
#define P1_6_Pin GPIO_PIN_6
#define P1_6_GPIO_Port GPIOA
#define CJ_AIN_Pin GPIO_PIN_7
#define CJ_AIN_GPIO_Port GPIOA
#define P2_0_Pin GPIO_PIN_0
#define P2_0_GPIO_Port GPIOB
#define P2_1_Pin GPIO_PIN_1
#define P2_1_GPIO_Port GPIOB
#define P2_2_Pin GPIO_PIN_2
#define P2_2_GPIO_Port GPIOB
#define P1_Pin GPIO_PIN_10
#define P1_GPIO_Port GPIOB
#define DIB_CSA_Pin GPIO_PIN_12
#define DIB_CSA_GPIO_Port GPIOB
#define DIB_SCLK_Pin GPIO_PIN_13
#define DIB_SCLK_GPIO_Port GPIOB
#define DIB_MISO_Pin GPIO_PIN_14
#define DIB_MISO_GPIO_Port GPIOB
#define DIB_MOSI_Pin GPIO_PIN_15
#define DIB_MOSI_GPIO_Port GPIOB
#define DIB_IRQ_Pin GPIO_PIN_12
#define DIB_IRQ_GPIO_Port GPIOA
#define P2_EXT_Pin GPIO_PIN_15
#define P2_EXT_GPIO_Port GPIOA
#define P2_3_Pin GPIO_PIN_3
#define P2_3_GPIO_Port GPIOB
#define P2_4_Pin GPIO_PIN_4
#define P2_4_GPIO_Port GPIOB
#define P2_5_Pin GPIO_PIN_5
#define P2_5_GPIO_Port GPIOB
#define P2_6_Pin GPIO_PIN_6
#define P2_6_GPIO_Port GPIOB
#define P2_Pin GPIO_PIN_7
#define P2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
