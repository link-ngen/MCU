/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : stm32f4xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_i2c2_rx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	__HAL_RCC_SYSCFG_CLK_ENABLE()
	;
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	/* System interrupt init*/

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hi2c->Instance == I2C1)
	{
		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE()
		;
		/**I2C1 GPIO Configuration
		 PB8     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = SSD1306_I2C1_SCL_Pin | SSD1306_I2C1_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE()
		;
		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	} else if (hi2c->Instance == I2C2)
	{
		/* USER CODE BEGIN I2C2_MspInit 0 */

		/* USER CODE END I2C2_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE()
		;
		/**I2C2 GPIO Configuration
		 PB10     ------> I2C2_SCL
		 PB3     ------> I2C2_SDA
		 */
		GPIO_InitStruct.Pin = MPU6050_I2C2_SCL_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
		HAL_GPIO_Init(MPU6050_I2C2_SCL_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = MPU6050_I2C2_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
		HAL_GPIO_Init(MPU6050_I2C2_SDA_GPIO_Port, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE()
		;

		/* I2C2 DMA Init */
		/* I2C2_RX Init */
		hdma_i2c2_rx.Instance = DMA1_Stream3;
		hdma_i2c2_rx.Init.Channel = DMA_CHANNEL_7;
		hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
		hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(hi2c, hdmarx, hdma_i2c2_rx);

		/* USER CODE BEGIN I2C2_MspInit 1 */

		/* USER CODE END I2C2_MspInit 1 */
	}

}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		 PB8     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, SSD1306_I2C1_SCL_Pin | SSD1306_I2C1_SDA_Pin);

		/* USER CODE BEGIN I2C1_MspDeInit 1 */

		/* USER CODE END I2C1_MspDeInit 1 */
	} else if (hi2c->Instance == I2C2)
	{
		/* USER CODE BEGIN I2C2_MspDeInit 0 */

		/* USER CODE END I2C2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		 PB10     ------> I2C2_SCL
		 PB3     ------> I2C2_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, MPU6050_I2C2_SCL_Pin | MPU6050_I2C2_SDA_Pin);

		/* I2C2 DMA DeInit */
		HAL_DMA_DeInit(hi2c->hdmarx);
		/* USER CODE BEGIN I2C2_MspDeInit 1 */

		/* USER CODE END I2C2_MspDeInit 1 */
	}

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
