#ifndef SRC_DRIVING_H_
#define SRC_DRIVING_H_

// Include Statements
#include "main.h"
#include "robot_config.h" // Include the configuration file

// Add any necessary pin definitions
#define AIN1 GPIO_PIN_1
#define BIN1 GPIO_PIN_4
#define STBY GPIO_PIN_6
#define STBY_Port GPIOA
#define IN_Port GPIOA

// Function Prototypes/Declarations
void drive(uint32_t time);
void brake(uint32_t time);
void turn(uint8_t dir);
void navigateToPosition(float target_x, float target_y);
void setTargetPosition(float x, float y);
void updateNavigation(void);
void mainBegin(void);
void mainEnd(void);
void infiniteWhileLoop(void);
void afterinfiniteWhileLoop(void);

// External variable declarations
extern float start_pwm;
extern float current_pwm;
extern int drive_time;
extern int turned;
extern int drove;
extern int start;
extern char buffer[150];

// Navigation variables
extern float target_x;
extern float target_y;
extern int navigation_stage;
extern int navigation_active;

#endif /* SRC_DRIVING_H_ */