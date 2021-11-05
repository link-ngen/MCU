/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define PI_180 0.0174532

//#define CUBE
#define PYRAMID
//#define OCTAHEDRON

I2C_HandleTypeDef hi2c1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void drawCubeVectors(void);
void rotateCube(int16_t pitch, int16_t roll, int16_t yaw);
void drawPyramid(void);
void rotatePyramid(int16_t pitch, int16_t roll, int16_t yaw);
void drawOctahedron(void);
void rotateOctahedron(int16_t pitch, int16_t roll, int16_t yaw);

// Demo programm
void main_rotate(void);

//Store cube vertices
const int8_t cube_vertex[8][3] = { { -20, -20, 20 }, { 20, -20, 20 }, { 20, 20,
		20 }, { -20, 20, 20 }, { -20, -20, -20 }, { 20, -20, -20 }, { 20, 20,
		-20 }, { -20, 20, -20 } };

const int8_t pyramid_vertex[5][3] = { { 0, -20, 0 },		// 0
		{ 20, 20, 20 },		// 1
		{ 20, 20, -20 },	// 2
		{ -20, 20, -20 },	// 3
		{ -20, 20, 20 } };	// 4

const int8_t octahedron_vertex[6][3] = { { 0, 30, 0 }, { -20, 0, 0 },
		{ 0, 0, 20 }, { 20, 0, 0 }, { 0, 0, -20 }, { 0, -30, 0 } };

uint16_t wireoctahedron[6][2];
uint16_t wirepyramid[5][2];
uint16_t wirecube[8][2];

const uint8_t originx = 64;
const uint8_t originy = 32;

uint32_t stime, fps = 0, frames = 0;
char string_fps[3];
uint16_t angle = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();

	ssd1306_Init(); // initialize the display
	stime = HAL_GetTick();

	while (1)
	{
		main_rotate();
	}
}

void drawPyramid(void)
{
	ssd1306_Line(wirepyramid[0][0], wirepyramid[0][1], wirepyramid[1][0],
			wirepyramid[1][1], White);
	ssd1306_Line(wirepyramid[0][0], wirepyramid[0][1], wirepyramid[2][0],
			wirepyramid[2][1], White);
	ssd1306_Line(wirepyramid[0][0], wirepyramid[0][1], wirepyramid[3][0],
			wirepyramid[3][1], White);
	ssd1306_Line(wirepyramid[0][0], wirepyramid[0][1], wirepyramid[4][0],
			wirepyramid[4][1], White);

	//cross face above
	ssd1306_Line(wirepyramid[4][0], wirepyramid[4][1], wirepyramid[2][0],
			wirepyramid[2][1], White);
	ssd1306_Line(wirepyramid[3][0], wirepyramid[3][1], wirepyramid[1][0],
			wirepyramid[1][1], White);

	ssd1306_Line(wirepyramid[4][0], wirepyramid[4][1], wirepyramid[3][0],
			wirepyramid[3][1], White);
	ssd1306_Line(wirepyramid[3][0], wirepyramid[3][1], wirepyramid[2][0],
			wirepyramid[2][1], White);
	ssd1306_Line(wirepyramid[2][0], wirepyramid[2][1], wirepyramid[1][0],
			wirepyramid[1][1], White);
	ssd1306_Line(wirepyramid[1][0], wirepyramid[1][1], wirepyramid[4][0],
			wirepyramid[4][1], White);
}

void drawCubeVectors(void)
{
	ssd1306_Line(wirecube[0][0], wirecube[0][1], wirecube[1][0], wirecube[1][1],
			White);
	ssd1306_Line(wirecube[1][0], wirecube[1][1], wirecube[2][0], wirecube[2][1],
			White);
	ssd1306_Line(wirecube[2][0], wirecube[2][1], wirecube[3][0], wirecube[3][1],
			White);
	ssd1306_Line(wirecube[3][0], wirecube[3][1], wirecube[0][0], wirecube[0][1],
			White);

	//cross face above
	ssd1306_Line(wirecube[1][0], wirecube[1][1], wirecube[3][0], wirecube[3][1],
			White);
	ssd1306_Line(wirecube[0][0], wirecube[0][1], wirecube[2][0], wirecube[2][1],
			White);

	ssd1306_Line(wirecube[4][0], wirecube[4][1], wirecube[5][0], wirecube[5][1],
			White);
	ssd1306_Line(wirecube[5][0], wirecube[5][1], wirecube[6][0], wirecube[6][1],
			White);
	ssd1306_Line(wirecube[6][0], wirecube[6][1], wirecube[7][0], wirecube[7][1],
			White);
	ssd1306_Line(wirecube[7][0], wirecube[7][1], wirecube[4][0], wirecube[4][1],
			White);

	ssd1306_Line(wirecube[0][0], wirecube[0][1], wirecube[4][0], wirecube[4][1],
			White);
	ssd1306_Line(wirecube[1][0], wirecube[1][1], wirecube[5][0], wirecube[5][1],
			White);
	ssd1306_Line(wirecube[2][0], wirecube[2][1], wirecube[6][0], wirecube[6][1],
			White);
	ssd1306_Line(wirecube[3][0], wirecube[3][1], wirecube[7][0], wirecube[7][1],
			White);
}

void drawOctahedron(void)
{
	ssd1306_Line(wireoctahedron[0][0], wireoctahedron[0][1],
			wireoctahedron[1][0], wireoctahedron[1][1], White);
	ssd1306_Line(wireoctahedron[0][0], wireoctahedron[0][1],
			wireoctahedron[2][0], wireoctahedron[2][1], White);
	ssd1306_Line(wireoctahedron[0][0], wireoctahedron[0][1],
			wireoctahedron[3][0], wireoctahedron[3][1], White);
	ssd1306_Line(wireoctahedron[0][0], wireoctahedron[0][1],
			wireoctahedron[4][0], wireoctahedron[4][1], White);

	ssd1306_Line(wireoctahedron[1][0], wireoctahedron[1][1],
			wireoctahedron[2][0], wireoctahedron[2][1], White);
	ssd1306_Line(wireoctahedron[2][0], wireoctahedron[2][1],
			wireoctahedron[3][0], wireoctahedron[3][1], White);
	ssd1306_Line(wireoctahedron[3][0], wireoctahedron[3][1],
			wireoctahedron[4][0], wireoctahedron[4][1], White);
	ssd1306_Line(wireoctahedron[4][0], wireoctahedron[4][1],
			wireoctahedron[1][0], wireoctahedron[1][1], White);

	ssd1306_Line(wireoctahedron[5][0], wireoctahedron[5][1],
			wireoctahedron[1][0], wireoctahedron[1][1], White);
	ssd1306_Line(wireoctahedron[5][0], wireoctahedron[5][1],
			wireoctahedron[2][0], wireoctahedron[2][1], White);
	ssd1306_Line(wireoctahedron[5][0], wireoctahedron[5][1],
			wireoctahedron[3][0], wireoctahedron[3][1], White);
	ssd1306_Line(wireoctahedron[5][0], wireoctahedron[5][1],
			wireoctahedron[4][0], wireoctahedron[4][1], White);
}

void rotateCube(int16_t pitch, int16_t roll, int16_t yaw)
{
	float rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
	float pitchRad, rollRad, yawRad;
	pitchRad = pitch * PI_180;
	rollRad = roll * PI_180;
	yawRad = yaw * PI_180;

	for (int i = 0; i < 8; ++i)
	{
		//rotateY
		rotz = cube_vertex[i][2] * cos(pitchRad)
				- cube_vertex[i][0] * sin(pitchRad);
		rotx = cube_vertex[i][2] * sin(pitchRad)
				+ cube_vertex[i][0] * cos(pitchRad);
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
		wirecube[i][0] = rotxxx;
		wirecube[i][1] = rotyyy;
		//wirecube[i][2] = rotzzz;
	}
}

void rotatePyramid(int16_t pitch, int16_t roll, int16_t yaw)
{
	float rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
	float pitchRad, rollRad, yawRad;
	pitchRad = pitch * PI_180;
	rollRad = roll * PI_180;
	yawRad = yaw * PI_180;

	for (int i = 0; i < 5; ++i)
	{
		//rotateY
		rotz = pyramid_vertex[i][2] * cos(pitchRad)
				- pyramid_vertex[i][0] * sin(pitchRad);
		rotx = pyramid_vertex[i][2] * sin(pitchRad)
				+ pyramid_vertex[i][0] * cos(pitchRad);
		roty = pyramid_vertex[i][1];

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
		wirepyramid[i][0] = rotxxx;
		wirepyramid[i][1] = rotyyy;
	}
}

void rotateOctahedron(int16_t pitch, int16_t roll, int16_t yaw)
{
	float rotx, roty, rotz, rotxx, rotyy, rotzz, rotxxx, rotyyy, rotzzz;
	float pitchRad, rollRad, yawRad;
	pitchRad = pitch * PI_180;
	rollRad = roll * PI_180;
	yawRad = yaw * PI_180;

	for (int i = 0; i < 6; ++i)
	{
		//rotateY
		rotz = octahedron_vertex[i][2] * cos(pitchRad)
				- octahedron_vertex[i][0] * sin(pitchRad);
		rotx = octahedron_vertex[i][2] * sin(pitchRad)
				+ octahedron_vertex[i][0] * cos(pitchRad);
		roty = octahedron_vertex[i][1];

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
		wireoctahedron[i][0] = rotxxx;
		wireoctahedron[i][1] = rotyyy;
	}
}

void main_rotate(void)
{
	ssd1306_Fill(Black);
	if (angle > 360)
	{
		angle = 0;
	}
#if defined(CUBE)
	rotateCube(angle, angle, angle);
	drawCubeVectors();
#elif defined(PYRAMID)
	rotatePyramid(angle, angle, angle);
	drawPyramid();
#elif defined(OCTAHEDRON)
	drawOctahedron();
	rotateOctahedron(angle, 0, 0);
#endif

	angle += 3;
	fps += 1000 / (HAL_GetTick() - stime);
	stime = HAL_GetTick();
	frames++;

	ssd1306_SetCursor(1, 55);
	sprintf(string_fps, "%d", fps / frames);
	ssd1306_WriteString(string_fps, Font_6x8, White);
	ssd1306_WriteString(" fps", Font_6x8, White);
	ssd1306_UpdateScreen();
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
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
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
	/** Initializes the CPU, AHB and APB buses clocks
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
