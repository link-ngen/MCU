#include "main.h"

#define PI_180 0.0174532

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;


//char string_mpu6050_temp[10];
//char string_intStatus[10];
char string_fps[3];
uint8_t data[14];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);

volatile uint8_t mpu6050_exti_flag = 0;
//Store cube vertices
int8_t cube_vertex[8][3] = {{ -20, -20, 20 }, { 20, -20, 20 },
		 	 	 	 	 	{20, 20, 20 }, { -20, 20, 20 },
							{ -20, -20, -20 }, { 20, -20, -20 },
							{ 20, 20, -20 }, { -20, 20, -20 }};
uint16_t wireframe[12][2];
const uint8_t originx = 64;
const uint8_t originy = 32; 

void drawVectors(void);
void rotateCube(int16_t pitch, int16_t roll, int16_t yaw);

uint32_t stime, fps = 0, frames = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C2_Init();
	MX_I2C1_Init();

	ssd1306_Init(); // initialize the display

	stime = HAL_GetTick();

	while (1)
	{
		if (MPU6050_Init(&mpu6050, MPU6050_AFS_SEL_2G, MPU6050_FS_SEL_250)
				== MPU6050_OK)
		{
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
			break;
		} else
		{
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
					GPIO_PIN_RESET);
		}
		HAL_Delay(10);
	}
//	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	MPU6050_enableInterrupts();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		ssd1306_Fill(Black);
		if (mpu6050_exti_flag == 1)
		{
			mpu6050_exti_flag = 0;
			rotateCube(mpu6050.Pitch, mpu6050.Roll, 0);
		}
		drawVectors();

		fps += 1000 / (HAL_GetTick() - stime);
		stime = HAL_GetTick();
		frames++;

		ssd1306_SetCursor(1, 55);
		sprintf(string_fps, "%d", fps / frames);
		ssd1306_WriteString(string_fps, Font_6x8, White);
		ssd1306_WriteString(" fps", Font_6x8, White);
		ssd1306_UpdateScreen();
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void drawVectors(void)
{
	 ssd1306_Line(wireframe[0][0], wireframe[0][1], wireframe[1][0], wireframe[1][1], White);
	 ssd1306_Line(wireframe[1][0], wireframe[1][1], wireframe[2][0], wireframe[2][1], White);
	 ssd1306_Line(wireframe[2][0], wireframe[2][1], wireframe[3][0], wireframe[3][1], White);
	 ssd1306_Line(wireframe[3][0], wireframe[3][1], wireframe[0][0], wireframe[0][1], White);

	 //cross face above
	 ssd1306_Line(wireframe[1][0], wireframe[1][1], wireframe[3][0], wireframe[3][1], White);
	 ssd1306_Line(wireframe[0][0], wireframe[0][1], wireframe[2][0], wireframe[2][1], White);

	 ssd1306_Line(wireframe[4][0], wireframe[4][1], wireframe[5][0], wireframe[5][1], White);
	 ssd1306_Line(wireframe[5][0], wireframe[5][1], wireframe[6][0], wireframe[6][1], White);
	 ssd1306_Line(wireframe[6][0], wireframe[6][1], wireframe[7][0], wireframe[7][1], White);
	 ssd1306_Line(wireframe[7][0], wireframe[7][1], wireframe[4][0], wireframe[4][1], White);

	 ssd1306_Line(wireframe[0][0], wireframe[0][1], wireframe[4][0], wireframe[4][1], White);
	 ssd1306_Line(wireframe[1][0], wireframe[1][1], wireframe[5][0], wireframe[5][1], White);
	 ssd1306_Line(wireframe[2][0], wireframe[2][1], wireframe[6][0], wireframe[6][1], White);
	 ssd1306_Line(wireframe[3][0], wireframe[3][1], wireframe[7][0], wireframe[7][1], White);
}

void rotateCube(int16_t pitch, int16_t roll, int16_t yaw)
{
	float rot, rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
	float pitchRad, rollRad, yawRad;
	pitchRad = pitch * PI_180;
	rollRad = roll * PI_180;
	yawRad = yaw * PI_180;

	for (int i = 0; i < 8; ++i)
	{
		//rotateY
		rotz = cube_vertex[i][2] * cos(pitchRad) - cube_vertex[i][0] * sin(pitchRad);
		rotx = cube_vertex[i][2] * sin(pitchRad) + cube_vertex[i][0] * cos(pitchRad);
		roty = cube_vertex[i][1];

		//rotateX
		rotyy = roty * cos(rollRad) - rotz * sin(rollRad);
		rotzz = roty * sin(rollRad) + rotz * cos(rollRad);
		rotxx = rotx;

		//rotateZ
		rotxxx = rotxx * cos(yawRad) - rotyy * sin(yawRad);
		rotyyy = rotxx * sin(yawRad) + rotyy * cos(yawRad);
		rotzzz = rotzz;

		//orthographic projection
		rotxxx = rotxxx + originx;
		rotyyy = rotyyy + originy;

		//store new vertices values for wireframe drawing
		wireframe[i][0] = rotxxx;
		wireframe[i][1] = rotyyy;
		wireframe[i][2] = rotzzz;
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin | RED_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
	GPIO_InitStruct.Pin = GREEN_LED_Pin | RED_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MPU6050_EXT_INT_Pin */
	GPIO_InitStruct.Pin = MPU6050_EXT_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MPU6050_EXT_INT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == MPU6050_EXT_INT_Pin)
	{
		//MPU6050_read_scaled_data(&mpu6050);
		MPU6050_readAll_DMA(&mpu6050, data);
		mpu6050_exti_flag = 1;
	}
	//__NOP();
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
