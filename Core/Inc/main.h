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
#define TFT_RES_Pin GPIO_PIN_3
#define TFT_RES_GPIO_Port GPIOE
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOE
#define TFT_BL_Pin GPIO_PIN_5
#define TFT_BL_GPIO_Port GPIOE
#define M1_ENC_B_Pin GPIO_PIN_0
#define M1_ENC_B_GPIO_Port GPIOA
#define M1_ENC_A_Pin GPIO_PIN_1
#define M1_ENC_A_GPIO_Port GPIOA
#define MPU6050_SCL_Pin GPIO_PIN_10
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOB
#define PWM_L1_Pin GPIO_PIN_13
#define PWM_L1_GPIO_Port GPIOB
#define PWM_R1_Pin GPIO_PIN_14
#define PWM_R1_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOD
#define M2_ENC_B_Pin GPIO_PIN_12
#define M2_ENC_B_GPIO_Port GPIOD
#define M2_ENC_A_Pin GPIO_PIN_13
#define M2_ENC_A_GPIO_Port GPIOD
#define TFT_DC_Pin GPIO_PIN_6
#define TFT_DC_GPIO_Port GPIOG
#define CS100A_ECHO_Pin GPIO_PIN_6
#define CS100A_ECHO_GPIO_Port GPIOC
#define CS100A_TRIG_Pin GPIO_PIN_7
#define CS100A_TRIG_GPIO_Port GPIOC
#define PWM_L2_Pin GPIO_PIN_8
#define PWM_L2_GPIO_Port GPIOA
#define PWM_R2_Pin GPIO_PIN_9
#define PWM_R2_GPIO_Port GPIOA
#define ESP8266_RX_Pin GPIO_PIN_11
#define ESP8266_RX_GPIO_Port GPIOC
#define ESP8266_RST_Pin GPIO_PIN_12
#define ESP8266_RST_GPIO_Port GPIOC
#define OPENMV_RX_Pin GPIO_PIN_9
#define OPENMV_RX_GPIO_Port GPIOG
#define OPENMV_TX_Pin GPIO_PIN_14
#define OPENMV_TX_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
