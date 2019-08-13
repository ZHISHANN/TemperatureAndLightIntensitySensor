/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "LCD.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUs(volatile uint32_t us)
{
	while(us != 0)
		us--;
}

void lcdWriteNibble(uint8_t data)
{
	// write mode = 0
	//GPIOA->BRR |= READ_WRITE_Pin;
	GPIOA->BSRR = (1 << 18) | (1 << 1);

	//GPIOA->BSRR |= ENABLE_PIN_Pin;
	DelayUs(7200);			// 100us

	GPIOB->BRR = LCD_D7_Pin | LCD_D6_Pin | LCD_D5_Pin | LCD_D4_Pin;
	GPIOB->BSRR = (((~data) & 0x0f) << 20) | (data << 4);

	DelayUs(7200);
	GPIOA->BRR |= ENABLE_PIN_Pin;

}

void lcdWrite4BitData(uint8_t data)
{
	lcdWriteNibble(data >> 4);
	lcdWriteNibble(data);
}

void lcdWrite8BitData(uint8_t data)
{
	GPIOA->BSRR = (1 << 18) | (1 << 1);
	DelayUs(7200);		// 100us

	GPIOB->BRR = LCD_D7_Pin | LCD_D6_Pin | LCD_D5_Pin | LCD_D4_Pin | LCD_D3_Pin | LCD_D2_Pin | LCD_D1_Pin | LCD_D0_Pin;
	GPIOB->BSRR = ((~data) << 16) | data;

	DelayUs(7200);
	GPIOA->BRR |= ENABLE_PIN_Pin;
}

void lcdWriteMsg(uint8_t msg)
{
	GPIOA->BSRR |= REG_SEL_Pin;
	lcdWrite8BitData(msg);
	//lcdWrite4BitData(msg);
}

// send command to LCD
void lcdWriteCmd(uint8_t msg)
{
	GPIOA->BRR |= REG_SEL_Pin;
	lcdWrite8BitData(msg);
	//lcdWrite4BitData(msg);
}

float measureLightIntensity(float Voltage, float stepDownVolatge)
{
	HAL_ADC_Start(&hadc1);
	uint16_t ADC_Light = HAL_ADC_GetValue(&hadc1);
	float intensity, voltage, actual_volatge;
	float IRRADIANCE_CONST = 0.19;

	ADC_Light = HAL_ADC_GetValue(&hadc1);
 	voltage = ((float )ADC_Light / 4096) * stepDownVolatge;
 	actual_volatge = (float)voltage * (Voltage/stepDownVolatge);
 	intensity = (float)actual_volatge * IRRADIANCE_CONST;

 	return intensity;
}

float measureTemperature(float Beta)
{
	HAL_ADC_Start(&hadc1);
	uint16_t ADC_Value;
	const float adcMax = 4095.00;
	const float invBeta = 1.00 / Beta;
	const float invT0 = 1.00 / 298.15;
	volatile float T, C;

	lcdSetCustomLoc(0,LINE_1_ADDR,1);
	ADC_Value = HAL_ADC_GetValue(&hadc1);
	T = 1.00 / (invT0 + invBeta*(log((adcMax / ADC_Value) - 1.00)));
	C = T - 273.15;

	return C;
}

void lcdPrintTemp()
{
	float C = measureTemperature(3721.8);

	printf(" ");
	printf("%.2f", C);
	lcdSetCustomLoc(7,LINE_1_ADDR,0);
	printf("C");
}

void lcdPrintIntensity()
{
	float intensity = measureLightIntensity(20, 3.214);

	lcdGotoNextLine();
 	lcdSetCustomLoc(0,LINE_2_ADDR,3);
 	printf(" ");
 	printf("%.2f", intensity);
 	printf("W/m");
 	lcdSetCustomLoc(9,LINE_2_ADDR,2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  initialise_monitor_handles();

  HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start(&hadc2);

  lcdInit();

  uint8_t char1[] = {0x0E, 0x0A, 0x0E, 0x0, 0x0, 0x0, 0x0, 0x0};
  lcdCreateCustom(0,char1);

  uint8_t char2[] = {0x0E, 0x0A, 0x0A, 0x0A, 0x0A, 0x11, 0x11, 0x0E};
  lcdCreateCustom(1,char2);

  uint8_t char3[] = {0x0C, 0x12, 0x04, 0x08, 0x1E, 0x0, 0x0, 0x0};
  lcdCreateCustom(2,char3);

  uint8_t char4[] = {0x04, 0x15, 0x0E, 0x1B, 0x0E, 0x15, 0x04, 0x0};
  lcdCreateCustom(3,char4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 measureTemperature(3721.8);
	 lcdPrintTemp();
	 measureLightIntensity(20, 3.214);
	 lcdPrintIntensity();
	 HAL_Delay(1000);
	 lcdReturnToHome();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_Pin|READ_WRITE_Pin|REG_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_PIN_Pin READ_WRITE_Pin REG_SEL_Pin */
  GPIO_InitStruct.Pin = ENABLE_PIN_Pin|READ_WRITE_Pin|REG_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin 
                           LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	lcdWriteMsg(ch);
	return ch;
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
