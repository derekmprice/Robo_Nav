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
#include <stdio.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// === UART DMA Constants ===
#define UART_RX_BUFFER_SIZE 256
#define LINE_BUFFER_SIZE 128

// === Buffers and State ===
uint8_t uart_dma_rx_buffer[UART_RX_BUFFER_SIZE];
char line_buffer[LINE_BUFFER_SIZE];
uint16_t line_pos = 0;
volatile uint16_t old_pos = 0;

// === Temporary Pin Definitions (to silence compiler) ===
#define AIN1 GPIO_PIN_1
#define BIN1 GPIO_PIN_4
#define STBY_Port GPIOA
#define STBY GPIO_PIN_6

// === External Peripherals (defined in auto-gen files) ===
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

// === Optional Dummy Variable (for debug output) ===

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
float Kp = 1.5, Ki = 0.0, Kd = 0.2;
float prev_error = 0, integral = 0;
float target_distance = 100.0;   // Target distance in cm
float current_distance = 0.0;    // Measured UWB distance
float distance_tolerance = 5.0;  // Acceptable error
uint8_t uart_rx_buffer[100];     // UART receive buffer

int drove = 0;
int start = 1;
float start_pwm = 50;
float current_pwm = 0;
uint8_t txt_Buffer[100];
uint8_t rx_buffer [100];
uint8_t rx_index;
uint8_t rx_byte;
uint8_t transfer_clpt;
uint16_t new_pos;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void print_debug(const char *msg);
void process_uart_data(void);
void parse_uart_block(uint8_t *data, uint16_t len);
void mainBegin(void);
void mainEnd(void);
void infiniteWhileLoop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void print_debug(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

void mainBegin(void) {
	HAL_GPIO_WritePin(GPIOC, AIN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, BIN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STBY_Port, STBY, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

void mainEnd(void) {
	// placeholder
}

void infiniteWhileLoop(void) {
	process_uart_data();
}

void handle_button_press(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13 && start == 0) {
		start = 1;
		drove = 0;
		current_pwm = start_pwm;
	} else if (GPIO_Pin == GPIO_PIN_13 && start == 1) {
		start = 0;
		current_pwm = 0;
	}
}

int parse_uart_block(void) {
//    for (uint16_t i = 0; i < len; i++)
//    {
//        char c = data[i];
//
//        if (c == '\n') // End of message
//        {
//            line_buffer[line_pos] = '\0';
//
//            // Extract range value
//            char *start = strstr(line_buffer, "range:(");
//            if (start)
//            {
//                start += strlen("range:(");
//                int range_val = atoi(start);
//
//                // Print value to serial (adjust huart1 if needed)
//                char msg[64];
//                snprintf(msg, sizeof(msg), "Parsed Range: %d\r\n", range_val);
//                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//            }
//
//            line_pos = 0;  // Reset for next line
//        }
//        else if (line_pos < LINE_BUFFER_SIZE - 1)
//        {
//            line_buffer[line_pos++] = c;
//        }
//        else
//        {
//            // Line buffer overflow — reset
//            line_pos = 0;
//        }
//    }

	int result = 0;
	for (uint16_t i = 0; i < rx_index; i++) {
	if (rx_buffer[i] >= '0' && rx_buffer[i] <= '9') {
		result = result * 10 + (rx_buffer[i] - 0);
	} else if (rx_buffer[i] == '\n') {
		break;
	}
}
return result;
}

void process_uart_data(void) {
new_pos = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

if (new_pos != old_pos) {
	print_debug("Data received via DMA\r\n");  // <== Add this
//        sprintf(rx_data, sizeof(rx_data), "new pos: %u", new_pos);
//        HAL_UART_Transmit(&huart2, (uint8_t*)rx_data, strlen(rx_data), HAL_MAX_DELAY);

	if (new_pos > old_pos) {
		parse_uart_block(&uart_dma_rx_buffer[old_pos], new_pos - old_pos);
	} else {
		parse_uart_block(&uart_dma_rx_buffer[old_pos],
				UART_RX_BUFFER_SIZE - old_pos);
		if (new_pos > 0)
			parse_uart_block(&uart_dma_rx_buffer[0], new_pos);
	}

	old_pos = new_pos;
	if (old_pos >= UART_RX_BUFFER_SIZE)
		old_pos = 0;
}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

/* USER CODE BEGIN 1 */
print_debug("DEBUG: System booted and UART is working.\r\n");

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
MX_DMA_Init();
MX_USART2_UART_Init();
MX_TIM2_Init();
/* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart2, rx_data, 1);
mainBegin();  // Initialize motor pins and PWM
print_debug("DEBUG: System booted and UART is working.\r\n");
HAL_UART_Receive_DMA(&huart2, uart_dma_rx_buffer, UART_RX_BUFFER_SIZE);

mainEnd();
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(500);
	infiniteWhileLoop();  // Move PID loop logic into this function
	process_uart_data();  // ✅ NO arguments
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

/** Configure the main internal regulator output voltage
 */
if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
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
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
	Error_Handler();
}

/** Initializes the CPU, AHB and APB buses clocks
 */
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
	Error_Handler();
}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

/* USER CODE BEGIN TIM2_Init 0 */

/* USER CODE END TIM2_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
TIM_MasterConfigTypeDef sMasterConfig = { 0 };

/* USER CODE BEGIN TIM2_Init 1 */

/* USER CODE END TIM2_Init 1 */
htim2.Instance = TIM2;
htim2.Init.Prescaler = 79;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 4294967295;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
	Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
	Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
	Error_Handler();
}
/* USER CODE BEGIN TIM2_Init 2 */

/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
if (HAL_UART_Init(&huart2) != HAL_OK) {
	Error_Handler();
}
/* USER CODE BEGIN USART2_Init 2 */

/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

/* DMA controller clock enable */
__HAL_RCC_DMA1_CLK_ENABLE();

/* DMA interrupt init */
/* DMA1_Channel6_IRQn interrupt configuration */
HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
GPIO_InitTypeDef GPIO_InitStruct = { 0 };
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_4, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_6, GPIO_PIN_RESET);

/*Configure GPIO pin : B1_Pin */
GPIO_InitStruct.Pin = B1_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : PC1 PC4 */
GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : LD2_Pin PA6 */
GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
if (GPIO_Pin == GPIO_PIN_13) {
	print_debug("Button pressed!\r\n");
	handle_button_press(GPIO_Pin);
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
if (huart->Instance == USART2) {
	if (rx_index < 100) {
		rx_buffer[rx_index++] = rx_data[0];
	}
	HAL_UART_Receive_IT(&huart2, rx_data, 1);
}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
__disable_irq();
while (1) {
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
