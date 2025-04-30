/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : driving.c
  * @brief          : Implementation of driving, turning and braking functionality
  ******************************************************************************
  */
/* USER CODE END Header */

#include "driving.h"
#include "main.h"
#include <stdio.h>  // For sprintf
#include <math.h>   // For fabs

/* External timer and UART handles */
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

/* External tag position from trilateration in main.c */
extern struct {
    float x;
    float y; 
    uint8_t status;
} tag;

/* Global variables - defined here since they're declared as extern in driving.h */
float start_pwm = 50;
float current_pwm = 0;
int drive_time = 1100;
int turned = 0;
int drove = 0;
int start = 0;
char buffer[150];

/* Navigation variables */
float target_x = 0.0f;
float target_y = 0.0f;
int navigation_stage = 0; // 0: not started, 1: moving in Y, 2: turning, 3: moving in X, 4: done
int navigation_active = 0;

/**
  * @brief  The mainBegin function has been moved to main.c
  * @note   This is just a stub to maintain compatibility if it's called elsewhere
  * @retval None
  */
void mainBegin(void) {
    // This function is now implemented directly in main.c
    // We keep this stub for compatibility
}

/**
  * @brief  GPIO EXTI Callback
  * @param  GPIO_Pin: Specifies the pins connected to EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13 && start == 0){
        start = 1;
        drove = 0;
        current_pwm = start_pwm;
    }
    else if (GPIO_Pin == GPIO_PIN_13 && start == 1){
        start = 0;
        current_pwm = 0;
    }
}

/**
  * @brief  Placeholder for main end function
  * @retval None
  */
void mainEnd(void) {
    /* Nothing to do here for now */
}

/**
  * @brief  Function to be called in the main while loop
  * @retval None
  */
void infiniteWhileLoop(void) {
    if (drove == 0 && start == 1) {
        // Debug message to verify navigation start
        int len = sprintf(buffer, "Starting navigation from (%.2f, %.2f) to (%.2f, %.2f)\r\n", 
                          tag.x, tag.y, target_x, target_y);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);

        // Only proceed if we have valid position data
        if (tag.status == 1) {
            // Calculate the differences in X and Y directions
            float diff_y = target_y - tag.y;
            float diff_x = target_x - tag.x;

            // Convert differences from UWB units to meters
            float meters_y = diff_y / UWB_UNITS_PER_METER;
            float meters_x = diff_x / UWB_UNITS_PER_METER;

            // Calculate drive times in milliseconds
            int y_drive_time = (int)(fabs(meters_y) * DRIVE_TIME_PER_METER);
            int x_drive_time = (int)(fabs(meters_x) * DRIVE_TIME_PER_METER);

            // Show navigation plan
            len = sprintf(buffer, "Navigation plan: Y=%.2fm (%dms), X=%.2fm (%dms)\r\n", 
                         meters_y, y_drive_time, meters_x, x_drive_time);
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);

            // Determine direction for Y movement
            if (fabs(meters_y) > 0.05) { // Only move if difference is significant
                // First move in Y direction
                HAL_Delay(1000);
                
                // Drive in appropriate direction
                if (meters_y > 0) {
                    // Target is ahead - drive forward
                    drive(y_drive_time);
                } else {
                    // Target is behind - drive backward (you'll need to implement this)
                    HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_SET); // Reverse direction
                    HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_SET); // Reverse direction
                    drive(y_drive_time);
                    HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET); // Reset direction
                    HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_RESET); // Reset direction
                }
            }
            
            // Turn right (90 degrees)
            if (fabs(meters_x) > 0.05) { // Only turn if X difference is significant
                turn(0); 
                
                // Determine direction for X movement
                if (meters_x > 0) {
                    // Target is to the right - drive forward
                    drive(x_drive_time);
                } else {
                    // Target is to the left - drive backward
                    HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_SET); // Reverse direction
                    HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_SET); // Reverse direction
                    drive(x_drive_time);
                    HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET); // Reset direction
                    HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_RESET); // Reset direction
                }
            }
            
            // Navigation complete
            len = sprintf(buffer, "Navigation complete\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
        } else {
            // No valid position data
            int len = sprintf(buffer, "Cannot navigate: No valid position data\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
        }
        
        drove = 1;
        start = 0;
    }
}

/**
  * @brief  Placeholder for function after while loop
  * @retval None
  */
void afterinfiniteWhileLoop(void) {
    /* Nothing to do here for now */
}

/**
  * @brief  Drive the robot forward for a specific time
  * @param  time: Duration to drive in milliseconds
  * @retval None
  */
void drive(uint32_t time){
    // Turn ON LED to indicate driving
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ((current_pwm * 0.95)* 255) / 100);  // Convert duty cycle to PWM value
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (((current_pwm) * 0.95) * 255)/ 100);  // Convert duty cycle to PWM value
    HAL_Delay(time);

    // Turn OFF LED before braking
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    
    brake(1200);
}

/**
  * @brief  Brake the robot gradually over time
  * @param  brake_time: Time over which to apply braking in milliseconds
  * @retval None
  */
void brake(uint32_t brake_time){
    // Ensure LED is off during braking
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    
    float brake_interval = current_pwm/20;
    while (current_pwm > 0){
        current_pwm -= brake_interval;
        int brake_int_time = brake_time/20;
        HAL_Delay(brake_int_time);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ((current_pwm * 0.95)* 255) / 100);  // Convert duty cycle to PWM value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (((current_pwm) * 1) * 255)/ 100);  // Convert duty cycle to PWM value
        int len = sprintf(buffer, "The PWM is currently: %0.2f   The brake interval is : %0.2f   The brake interval time is : %d \r\n", current_pwm, brake_interval, brake_int_time);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
        if (start == 0){
            break;
        }
    }
    
    if (start == 1){
        current_pwm = start_pwm;
    }
}

/**
  * @brief  Turn the robot in a specified direction
  * @param  dir: Direction to turn (0 for right, 1 for left)
  * @retval None
  */
void turn(uint8_t dir){
    // Calculate turning time based on the number of turns already performed
    int turn_time = 900 - 150*turned;
    int blink_interval = 100; // LED will blink every 100ms
    
    switch(dir){
    case 0:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ((current_pwm * 0)* 255) / 100);  // Convert duty cycle to PWM value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (((current_pwm) * 0.95) * 255)/ 100);  // Convert duty cycle to PWM value
        
        // Blink LED during turn to indicate turning
        for(int i = 0; i < turn_time; i += (2 * blink_interval)) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // LED ON
            HAL_Delay(blink_interval);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
            HAL_Delay(blink_interval);
        }
        
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);  // Convert duty cycle to PWM value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);  // Convert duty cycle to PWM value
        break;
        
    case 1:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ((current_pwm * 0.95)* 255) / 100);  // Convert duty cycle to PWM value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (((-current_pwm) * 0) * 255)/ 100);  // Convert duty cycle to PWM value
        
        // Blink LED during turn to indicate turning
        for(int i = 0; i < turn_time; i += (2 * blink_interval)) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // LED ON
            HAL_Delay(blink_interval);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
            HAL_Delay(blink_interval);
        }
        
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);  // Convert duty cycle to PWM value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);  // Convert duty cycle to PWM value
        break;
    }
    
    turned += 1;
}

/**
  * @brief  Set the target position for navigation
  * @param  x: Target X coordinate in UWB units
  * @param  y: Target Y coordinate in UWB units
  * @retval None
  */
void setTargetPosition(float x, float y) {
    target_x = x;
    target_y = y;
    navigation_active = 1;
    navigation_stage = 0;
    turned = 0;
    drove = 0;
    
    int len = sprintf(buffer, "New target position set: (%.2f, %.2f)\r\n", target_x, target_y);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
}

/**
  * @brief  Navigate to the target position using current position from trilateration
  * @param  target_x: Target X coordinate in UWB units
  * @param  target_y: Target Y coordinate in UWB units
  * @retval None
  */
void navigateToPosition(float target_x, float target_y) {
    // Only proceed if we have a valid position from trilateration
    if (tag.status == 0) {
        int len = sprintf(buffer, "Cannot navigate: No valid position data\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
        return;
    }

    // Calculate the differences in X and Y directions
    float diff_y = target_y - tag.y;
    float diff_x = target_x - tag.x;

    // Convert differences from UWB units to meters
    float meters_y = diff_y / UWB_UNITS_PER_METER;
    float meters_x = diff_x / UWB_UNITS_PER_METER;

    // Calculate drive time in milliseconds
    int y_drive_time = (int)(fabs(meters_y) * DRIVE_TIME_PER_METER);
    int x_drive_time = (int)(fabs(meters_x) * DRIVE_TIME_PER_METER);

    int len = sprintf(buffer, "Navigation plan: Y dist=%.2fm (%dms), X dist=%.2fm (%dms)\r\n", 
                     meters_y, y_drive_time, meters_x, x_drive_time);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);

    // Start navigation sequence
    navigation_stage = 1;
    navigation_active = 1;
    
    // Begin by moving in Y direction
    if (y_drive_time > 0) {
        HAL_Delay(1000); // Short delay before starting
        drive(y_drive_time);
    }
    
    // Turn right (90 degrees)
    navigation_stage = 2;
    turn(0); // 0 = right turn
    
    // Move in X direction
    navigation_stage = 3;
    if (x_drive_time > 0) {
        HAL_Delay(500);
        drive(x_drive_time);
    }
    
    navigation_stage = 4; // Navigation complete
    navigation_active = 0;
    
    len = sprintf(buffer, "Navigation complete, reached target position\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
}