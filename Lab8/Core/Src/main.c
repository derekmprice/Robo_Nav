/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char buffer[100];
#define MPU_I2C 0b1101000 << (1 + 0) // left shifted 7 bit address
#define TEMP_MSB 0x41

#define ACCEL_CONFIG 0x1C
#define X_ACCEL_MSB 0x3B
#define Y_ACCEL_MSB 0x3D
#define Z_ACCEL_MSB 0x3F
float xAccel;
float yAccel;
float zAccel;

int16_t rawGyro;
float gyroX, gyroY, gyroZ;

#define GYRO_CONFIG 0x1B // from datasheet
#define X_GYRO_MSB 0x43
#define Y_GYRO_MSB 0x45
#define Z_GYRO_MSB 0x47

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void tip (float x, float y, float z){
	float meas  = 0.6;
	if (x > meas || y > meas || z > meas || x < -meas || y < -meas || z < -meas){
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}else{
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	uint8_t data[1]; //buffer to hold addr
	uint8_t data_temp[2]; //buffer to hold temp data

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //turn of LED
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, 0b1101000 << (1 + 0), 1,
              100); //staus update

  	if (ret == HAL_OK){
  		snprintf(buffer, sizeof(buffer), "the device is ready \r\n");
  		    		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
  	} else{
  		snprintf(buffer, sizeof(buffer), "not ready \r\n");
  		    		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
  	}

  //i2c turn on MPU - TASK 2
    	data[0] = 0x00;
    	ret = HAL_I2C_Mem_Write(&hi2c1, 0b1101000 << (1 + 0), 0x6B, 1, data, 1, HAL_MAX_DELAY);
    	if (ret != HAL_OK){
    		snprintf(buffer, sizeof(buffer), "Error in configuring the alert pin \r\n");
    		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    	} else{
    		snprintf(buffer, sizeof(buffer), "No error in configuring the alert pin \r\n");
    		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    	}
//
    	//acceleration +-4g - TASK 3
    	  	data[0] = 0x08;
    	  	ret = HAL_I2C_Mem_Write(&hi2c1, 0b1101000 << (1 + 0), ACCEL_CONFIG, 1, data, 1, HAL_MAX_DELAY);
    		if (ret != HAL_OK){
    			snprintf(buffer, sizeof(buffer), "Error in configuring the acceleration pin \r\n");
    			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    		} else{
    			snprintf(buffer, sizeof(buffer), "No error in configuring the acceleration pin \r\n");
    			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    		}
//
//    		//gyroscope +-4gs - TASK 4
    		data[0] = 0x10;
    		ret = HAL_I2C_Mem_Write(&hi2c1, MPU_I2C, GYRO_CONFIG, 1, data, 1, HAL_MAX_DELAY);
    		if (ret != HAL_OK){
    			snprintf(buffer, sizeof(buffer), "Error in configuring the gyroscope pin \r\n");
    			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    		} else{
    			snprintf(buffer, sizeof(buffer), "No error in configuring the gyroscope pin \r\n");
    			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
    		}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //temp!!- TASK 2
	  // --- Temperature Read ---
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, TEMP_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret != HAL_OK) {
	      snprintf(buffer, sizeof(buffer), "ERROR: Temp read failed.\r\n");
	      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
	  } else {
	      int16_t rawTemp = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      float degC = (rawTemp / 340.0) + 36.53;
	      snprintf(buffer, sizeof(buffer), "TEMP,%.2f\r\n", degC);  // New cleaner format
	      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
	  }
	  HAL_Delay(500);

	  // --- Acceleration Reads ---
	  int16_t rawAccel;
	  float accelX, accelY, accelZ;

	  // X-axis acceleration
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, X_ACCEL_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawAccel = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      accelX = rawAccel / 8192.0; // ±4g mode sensitivity
	  }

	  // Y-axis acceleration
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, Y_ACCEL_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawAccel = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      accelY = rawAccel / 8192.0;
	  }

	  // Z-axis acceleration
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, Z_ACCEL_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawAccel = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      accelZ = rawAccel / 8192.0;
	  }

	  // Print accelerometer data in a nice CSV format
	  snprintf(buffer, sizeof(buffer), "ACCEL,%.3f,%.3f,%.3f\r\n", accelX, accelY, accelZ);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);

	  HAL_Delay(500);

	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, X_GYRO_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawGyro = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      gyroX = rawGyro / 32.8;  // Sensitivity for ±1000 deg/s range
	  }

	  // Y-axis gyroscope
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, Y_GYRO_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawGyro = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      gyroY = rawGyro / 32.8;
	  }

	  // Z-axis gyroscope
	  ret = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C, Z_GYRO_MSB, 1, data_temp, 2, HAL_MAX_DELAY);
	  if (ret == HAL_OK) {
	      rawGyro = (int16_t)((data_temp[0] << 8) | data_temp[1]);
	      gyroZ = rawGyro / 32.8;
	  }

	  // Print gyroscope data
	  snprintf(buffer, sizeof(buffer), "GYRO,%.3f,%.3f,%.3f\r\n", gyroX, gyroY, gyroZ);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);

	  HAL_Delay(500);

  }

  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
