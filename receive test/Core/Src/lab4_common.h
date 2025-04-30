/*
 * lab4_common.h
 *
 *  Created on: Feb 10, 2025
 *      Author: derek
 */

#ifndef LAB4_COMMON_H_
#define LAB4_COMMON_H_

// Include Statements
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"

// Function Prototypes/Declarations
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Pin Assignments
#define led_builtin_port GPIOA
#define led_builtin_pin GPIO_PIN_5
#define BTN GPIO_PIN_13
#define PWMA GPIO_PIN_0
#define PWMB GPIO_PIN_10
#define AIN1 GPIO_PIN_1
#define STBY GPIO_PIN_6
#define BIN1 GPIO_PIN_4
#define TRIG GPIO_PIN_7
#define ECHO_Pin GPIO_PIN_8
#define PWMA_Port GPIOA
#define PWMB_Port GPIOA
#define STBY_Port GPIOA
#define TRIG_Port GPIOA
#define IN_Port GPIOA
#define BTN_Port GPIOC
#define ECHO_Port GPIOA
#define led_g_port GPIOA
#define led_g_pin GPIO_PIN_5
#define BLUE_Step_Pin GPIO_PIN_7
#define BLUE_Step_Port GPIOC
#define PINK_Step_Pin GPIO_PIN_6
#define PINK_Step_Port GPIOB
#define YELLOW_Step_Pin GPIO_PIN_9
#define YELLOW_Step_Port GPIOB
#define ORANGE_Step_Pin GPIO_PIN_8
#define ORANGE_Step_Port GPIOB


//GPIO_TypeDef* led_b_port = GPIOB; uint16_t led_b_pin = GPIO_PIN_4; // D5
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
#define button_b1_port GPIOC
#define button_b1_pin GPIO_PIN_13
const float pwm_period = 255;
//GPIO_TypeDef* button_r_port = GPIOA; uint16_t button_r_pin = GPIO_PIN_9;		// D8
//GPIO_TypeDef* button_y_port = GPIOC; uint16_t button_y_pin = GPIO_PIN_7;		// D9
//GPIO_TypeDef* button_g_port = GPIOB; uint16_t button_g_pin = GPIO_PIN_6;		// D10
//GPIO_TypeDef* button_b_port = GPIOA; uint16_t button_b_pin = GPIO_PIN_7;		// D11

// Global Variables

volatile bool button_r_pressed = false;
volatile int deadzone = 10;
volatile int mode = 1;
float pulse = 0;
int reverse = 1;


//volatile bool button_y_pressed = false;
//volatile bool button_g_pressed = false;
//volatile bool button_b_pressed = false;
//volatile bool tilt_switch_triggered = false;

// I can also write my interrupt ISRs/callbacks here (or they can be in the task-specific header files)


#endif /* LAB4_COMMON_H_ */
