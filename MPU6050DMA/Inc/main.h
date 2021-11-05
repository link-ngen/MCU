/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "MPU6050.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_gfx.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
MPU6050_t mpu6050;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU6050_I2C2_SCL_Pin GPIO_PIN_10
#define MPU6050_I2C2_SCL_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_9
#define GREEN_LED_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_10
#define RED_LED_GPIO_Port GPIOA
#define MPU6050_I2C2_SDA_Pin GPIO_PIN_3
#define MPU6050_I2C2_SDA_GPIO_Port GPIOB
#define MPU6050_EXT_INT_Pin GPIO_PIN_4
#define MPU6050_EXT_INT_GPIO_Port GPIOB
#define MPU6050_EXT_INT_EXTI_IRQn EXTI4_IRQn
#define SSD1306_I2C1_SCL_Pin GPIO_PIN_8
#define SSD1306_I2C1_SCL_GPIO_Port GPIOB
#define SSD1306_I2C1_SDA_Pin GPIO_PIN_9
#define SSD1306_I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
