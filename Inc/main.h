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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_lib.h"
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
extern FOC_t foc;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define CURR_W_Pin GPIO_PIN_0
#define CURR_W_GPIO_Port GPIOC
#define CURR_V_Pin GPIO_PIN_1
#define CURR_V_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_2
#define TEMP_GPIO_Port GPIOC
#define BEMF_U_Pin GPIO_PIN_3
#define BEMF_U_GPIO_Port GPIOC
#define CURR_U_Pin GPIO_PIN_0
#define CURR_U_GPIO_Port GPIOA
#define V_BUS_Pin GPIO_PIN_1
#define V_BUS_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define POTEN_Pin GPIO_PIN_4
#define POTEN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BEMF_V_Pin GPIO_PIN_4
#define BEMF_V_GPIO_Port GPIOC
#define BEMF_W_Pin GPIO_PIN_5
#define BEMF_W_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define GPIO_BEMF_Pin GPIO_PIN_9
#define GPIO_BEMF_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
