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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"

#include "stm32f4xx_ll_exti.h"

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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_0
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_2
#define GPIO3_GPIO_Port GPIOC
#define GPIO4_Pin GPIO_PIN_3
#define GPIO4_GPIO_Port GPIOC
#define G01_CS_Pin GPIO_PIN_4
#define G01_CS_GPIO_Port GPIOA
#define G01_SCK_Pin GPIO_PIN_5
#define G01_SCK_GPIO_Port GPIOA
#define G01_MISO_Pin GPIO_PIN_6
#define G01_MISO_GPIO_Port GPIOA
#define G01_MOSI_Pin GPIO_PIN_7
#define G01_MOSI_GPIO_Port GPIOA
#define G01_IRQ_Pin GPIO_PIN_4
#define G01_IRQ_GPIO_Port GPIOC
#define G01_CE_Pin GPIO_PIN_5
#define G01_CE_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_0
#define ADC1_GPIO_Port GPIOB
#define ADC2_Pin GPIO_PIN_1
#define ADC2_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_2
#define BUTTON2_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_12
#define BUTTON1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
