/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with UWB trilateration
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
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "driving.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Define structure for anchor points
typedef struct {
    char name[10];
    float x;
    float y;
} Anchor_t;

// Define structure for tag position
typedef struct {
    float x;
    float y;
    uint8_t status; // 1 if position valid, 0 if not
} Tag_t;

// Define structure for a point
typedef struct {
    float x;
    float y;
} Point_t;

// Define structure for circle intersection results
typedef struct {
    Point_t points[2];
    uint8_t count;
} Intersection_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256      // Increased buffer size for longer messages
#define MAX_ANCHORS 3           // Number of anchors in the system
#define RANGE_BUFFER_SIZE 10    // Number of samples to average (smaller than Python for memory)
#define MAX_INTERSECTIONS 6     // Maximum number of intersections (3 anchors = max 6 intersections)

// Navigation variables
#define TARGET_X_POSITION    200.0f
#define TARGET_Y_POSITION    200.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_byte;
uint8_t rx_data_ready = 0;

// Motor control pin definitions
#define AIN1 GPIO_PIN_1
#define BIN1 GPIO_PIN_4
#define STBY GPIO_PIN_6
#define IN_Port GPIOA
#define STBY_Port GPIOA

// Navigation variables - these are defined in driving.c, just reference them here
extern int navigation_active;
extern int navigation_stage;
#define UWB_UNITS_PER_METER  175.0f
#define DRIVE_TIME_PER_METER 1805.0f

// Anchor configuration (can be changed as needed)
Anchor_t anchors[MAX_ANCHORS] = {
    {"ANC 0", 0.0f, 0.0f},      // Anchor 0 at origin
    {"ANC 1", 270.0f, 0.0f},    // Anchor 1 at (270, 0)
    {"ANC 2", 270.0f, 400.0f}   // Anchor 2 at (270, 400)
};

// Range measurements
int32_t tag_ranges[MAX_ANCHORS];
int32_t tag_ranges_buffer[MAX_ANCHORS][RANGE_BUFFER_SIZE];
uint8_t buffer_index[MAX_ANCHORS] = {0};
uint8_t buffer_full[MAX_ANCHORS] = {0};

// Tag position
Tag_t tag = {0.0f, 0.0f, 0};

// Temporary storage for intersection calculations
Intersection_t intersections[MAX_INTERSECTIONS];
uint8_t intersection_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_debug(const char *msg);
void parse_range_data(char *data);
void update_range_buffers(int32_t new_ranges[]);
void calculate_tag_position(void);
Intersection_t circle_intersections(float x1, float y1, float r1, float x2, float y2, float r2);
float calculate_distance(Point_t p1, Point_t p2);
void find_best_intersection_set(Point_t *result);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send debug message to PC via UART2 (ST-Link Virtual COM port)
void print_debug(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Calculate Euclidean distance between two points
float calculate_distance(Point_t p1, Point_t p2) {
    return sqrtf((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Calculate intersection between two circles
Intersection_t circle_intersections(float x1, float y1, float r1, float x2, float y2, float r2) {
    Intersection_t result = {{0}, 0};
    char debug_msg[100];

    // Calculate distance between centers
    float d = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    sprintf(debug_msg, "DEBUG: Circle intersection: centers distance=%.2f, r1=%.2f, r2=%.2f\r\n", d, r1, r2);
    print_debug(debug_msg);

    // Check if circles are too far apart or one inside the other
    if (d > r1 + r2 || d < fabsf(r1 - r2)) {
        // No intersection, find closest point on each circle and average
        float closest_x1 = x1 + (r1 * (x2 - x1) / d);
        float closest_y1 = y1 + (r1 * (y2 - y1) / d);
        float closest_x2 = x2 - (r2 * (x2 - x1) / d);
        float closest_y2 = y2 - (r2 * (y2 - y1) / d);

        result.points[0].x = (closest_x1 + closest_x2) / 2.0f;
        result.points[0].y = (closest_y1 + closest_y2) / 2.0f;
        result.count = 1;

        sprintf(debug_msg, "DEBUG: No intersection, using closest point: (%.2f, %.2f)\r\n",
                result.points[0].x, result.points[0].y);
        print_debug(debug_msg);
        return result;
    }

    // Calculate intersection points
    float a = (r1*r1 - r2*r2 + d*d) / (2.0f * d);
    float h = sqrtf(r1*r1 - a*a);
    float x3 = x1 + a * (x2 - x1) / d;
    float y3 = y1 + a * (y2 - y1) / d;

    // First intersection
    result.points[0].x = x3 + h * (y2 - y1) / d;
    result.points[0].y = y3 - h * (x2 - x1) / d;

    // Second intersection
    result.points[1].x = x3 - h * (y2 - y1) / d;
    result.points[1].y = y3 + h * (x2 - x1) / d;

    result.count = 2;

    sprintf(debug_msg, "DEBUG: Found 2 intersections: (%.2f, %.2f) and (%.2f, %.2f)\r\n",
            result.points[0].x, result.points[0].y, result.points[1].x, result.points[1].y);
    print_debug(debug_msg);
    return result;
}

// Find the best set of intersection points
void find_best_intersection_set(Point_t *result) {
    // Debug: print intersection count
    char debug_msg[100];
    sprintf(debug_msg, "DEBUG: Processing %d intersections\r\n", intersection_count);
    print_debug(debug_msg);

    // If too few intersections, use a simpler approach
    if (intersection_count < 3) {
        // If we have at least one intersection, use it
        if (intersection_count > 0) {
            result->x = intersections[0].points[0].x;
            result->y = intersections[0].points[0].y;
            tag.status = 1;

            // If we have exactly two intersections, average them
            if (intersection_count == 2) {
                result->x = (intersections[0].points[0].x + intersections[1].points[0].x) / 2.0f;
                result->y = (intersections[0].points[0].y + intersections[1].points[0].y) / 2.0f;
            }

            sprintf(debug_msg, "DEBUG: Using simple averaging with %d points\r\n", intersection_count);
            print_debug(debug_msg);
            return;
        }

        tag.status = 0;
        return;
    }

    // Find the set of 3 points with minimum total distance between them
    float min_dist = 1000000.0f; // Use a very large number instead of FLT_MAX
    Point_t best_points[3];
    int found_set = 0;

    // Try all combinations of 3 points
    for (int i = 0; i < intersection_count - 2; i++) {
        for (int j = i + 1; j < intersection_count - 1; j++) {
            for (int k = j + 1; k < intersection_count; k++) {
                Point_t p1 = intersections[i].points[0];
                Point_t p2 = intersections[j].points[0];
                Point_t p3 = intersections[k].points[0];

                // Calculate total distance between points
                float dist = calculate_distance(p1, p2) +
                             calculate_distance(p1, p3) +
                             calculate_distance(p2, p3);

                if (dist < min_dist) {
                    min_dist = dist;
                    best_points[0] = p1;
                    best_points[1] = p2;
                    best_points[2] = p3;
                    found_set = 1;
                }
            }
        }
    }

    if (found_set) {
        // Calculate centroid of best three points
        result->x = (best_points[0].x + best_points[1].x + best_points[2].x) / 3.0f;
        result->y = (best_points[0].y + best_points[1].y + best_points[2].y) / 3.0f;
        tag.status = 1;

        // Debug output
        sprintf(debug_msg, "DEBUG: Best set found, min_dist=%.2f\r\n", min_dist);
        print_debug(debug_msg);
        sprintf(debug_msg, "DEBUG: Points used: (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f)\r\n",
                best_points[0].x, best_points[0].y,
                best_points[1].x, best_points[1].y,
                best_points[2].x, best_points[2].y);
        print_debug(debug_msg);
    } else {
        // If we somehow didn't find a valid set, use simple averaging of all points
        result->x = 0;
        result->y = 0;
        for (int i = 0; i < intersection_count; i++) {
            result->x += intersections[i].points[0].x;
            result->y += intersections[i].points[0].y;
        }
        result->x /= intersection_count;
        result->y /= intersection_count;
        tag.status = 1;

        sprintf(debug_msg, "DEBUG: Using average of all %d points\r\n", intersection_count);
        print_debug(debug_msg);
    }

    // Calculate centroid of best three points
    result->x = (best_points[0].x + best_points[1].x + best_points[2].x) / 3.0f;
    result->y = (best_points[0].y + best_points[1].y + best_points[2].y) / 3.0f;
    tag.status = 1;
}

// Calculate the tag position using trilateration
void calculate_tag_position(void) {
    intersection_count = 0;

    // Check if we have valid ranges
    char debug_msg[100];
    sprintf(debug_msg, "DEBUG: Starting trilateration with ranges: %ld, %ld, %ld\r\n",
            (long)tag_ranges[0], (long)tag_ranges[1], (long)tag_ranges[2]);
    print_debug(debug_msg);

    // Skip calculation if any range is zero (invalid)
    if (tag_ranges[0] <= 0 || tag_ranges[1] <= 0 || tag_ranges[2] <= 0) {
        print_debug("DEBUG: Invalid ranges, skipping trilateration\r\n");
        tag.status = 0;
        return;
    }

    // Calculate all pairwise intersections
    for (int i = 0; i < MAX_ANCHORS; i++) {
        for (int j = i + 1; j < MAX_ANCHORS; j++) {
            // Skip if we've reached maximum number of intersections
            if (intersection_count >= MAX_INTERSECTIONS) break;

            // Skip if either range is zero
            if (tag_ranges[i] <= 0 || tag_ranges[j] <= 0) continue;

            // Calculate intersection
            Intersection_t result = circle_intersections(
                anchors[i].x, anchors[i].y, (float)tag_ranges[i],
                anchors[j].x, anchors[j].y, (float)tag_ranges[j]
            );

            // Debug output for each intersection calculation
            sprintf(debug_msg, "DEBUG: Intersection %d-%d: found %d points\r\n",
                    i, j, result.count);
            print_debug(debug_msg);

            // Store all intersection points
            for (int k = 0; k < result.count; k++) {
                if (intersection_count < MAX_INTERSECTIONS) {
                    intersections[intersection_count].points[0] = result.points[k];
                    intersections[intersection_count].count = 1;

                    // Debug output for point coordinates
                    sprintf(debug_msg, "  Point %d: (%.2f, %.2f)\r\n",
                            intersection_count, result.points[k].x, result.points[k].y);
                    print_debug(debug_msg);

                    intersection_count++;
                }
            }
        }
    }

    // Find best intersection set and calculate position
    Point_t position = {0};
    find_best_intersection_set(&position);

    // Update tag position if calculation was successful
    if (tag.status) {
        tag.x = position.x;
        tag.y = position.y;

        sprintf(debug_msg, "DEBUG: Final position calculated: (%.2f, %.2f)\r\n",
                tag.x, tag.y);
        print_debug(debug_msg);
    } else {
        print_debug("DEBUG: Position calculation failed\r\n");
    }
}

// Update range buffers with new readings
void update_range_buffers(int32_t new_ranges[]) {
    char debug_msg[100];

    // Add new readings to buffer and calculate averages
    for (int i = 0; i < MAX_ANCHORS; i++) {
        // Skip invalid readings (negative values)
        if (new_ranges[i] < 0) continue;

        // Add to buffer
        tag_ranges_buffer[i][buffer_index[i]] = new_ranges[i];
        buffer_index[i] = (buffer_index[i] + 1) % RANGE_BUFFER_SIZE;

        // Mark buffer as full after we've collected enough samples
        if (buffer_index[i] == 0) {
            buffer_full[i] = 1;
        }

        // Calculate average
        int32_t sum = 0;
        int count = buffer_full[i] ? RANGE_BUFFER_SIZE : buffer_index[i];

        if (count > 0) {
            for (int j = 0; j < count; j++) {
                sum += tag_ranges_buffer[i][j];
            }
            tag_ranges[i] = sum / count;
        }

        // Debug output
        sprintf(debug_msg, "ANC%d Range: %ld cm  (Raw: %ld, Samples: %d)\r\n",
                i, (long)tag_ranges[i], (long)new_ranges[i], count);
        print_debug(debug_msg);
    }
}

// Parse the range data from the received string
void parse_range_data(char *data) {
    char *range_start = strstr(data, "range:(");

    if (range_start) {
        // Move pointer to start of range values
        range_start += 7; // Skip "range:("

        // Create a copy to tokenize
        char range_str[100];
        strncpy(range_str, range_start, 99);
        range_str[99] = '\0';

        // Find end of range values
        char *range_end = strchr(range_str, ')');
        if (range_end) *range_end = '\0';

        // Parse comma-separated values
        int32_t new_ranges[MAX_ANCHORS] = {0};
        char *token = strtok(range_str, ",");
        int i = 0;

        while (token != NULL && i < MAX_ANCHORS) {
            new_ranges[i++] = atoi(token);
            token = strtok(NULL, ",");
        }

        // Update range buffers with new readings
        update_range_buffers(new_ranges);

        // Calculate new tag position
        calculate_tag_position();

        // Output tag position
        char pos_msg[100];
        if (tag.status) {
            sprintf(pos_msg, "TAG Position: (%.2f, %.2f) cm\r\n", tag.x, tag.y);
        } else {
            sprintf(pos_msg, "TAG Position: Unknown\r\n");
        }
        // Debug intersection count
        sprintf(pos_msg + strlen(pos_msg), "Debug: Found %d intersections\r\n", intersection_count);
        print_debug(pos_msg);
    }
}

// Process received UART data
void process_uart_data(void) {
    if (rx_index > 0) {
        // Add null terminator for string functions
        rx_buffer[rx_index] = '\0';

        // Forward the received data to UART2 (PC)
        print_debug("UART3 Data: ");
        print_debug((char*)rx_buffer);

        // If there's no newline at the end, add one for better readability in terminal
        if (rx_buffer[rx_index-1] != '\n' && rx_buffer[rx_index-1] != '\r') {
            print_debug("\r\n");
        }

        // Check if this is a range data message
        if (strstr((char*)rx_buffer, "AT+RANGE")) {
            parse_range_data((char*)rx_buffer);
        }

        // Reset buffer for next message
        rx_index = 0;
    }
}

// UART receive interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {  // If the received data is on USART3 (PA10)
        // Store the received byte
        if (rx_index < RX_BUFFER_SIZE - 1) {  // Leave room for null terminator
            rx_buffer[rx_index++] = rx_byte;
        }

        // If newline character received or buffer is getting full, mark data as ready
        if (rx_byte == '\n' || rx_byte == '\r' || rx_index >= RX_BUFFER_SIZE - 1) {
            rx_data_ready = 1;
        }

        // Re-enable UART receive interrupt for next byte
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

/* USER CODE BEGIN 4 */
// Button press handler for navigation
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) { // Blue button on PC13
        // Only start navigation if we have a valid position
        if (tag.status == 1) {
            char debug_msg[100];
            sprintf(debug_msg, "Starting navigation from (%.2f, %.2f) to (%.2f, %.2f)\r\n", 
                    tag.x, tag.y, TARGET_X_POSITION, TARGET_Y_POSITION);
            print_debug(debug_msg);
            
            navigation_active = 1;
            navigation_stage = 1; // Start with Y-direction movement
        } else {
            print_debug("Cannot navigate: No valid position data\r\n");
        }
    }
}
/* USER CODE END 4 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Clear buffer and flags
  memset(rx_buffer, 0, RX_BUFFER_SIZE);
  rx_index = 0;
  rx_data_ready = 0;

  // Initialize the range buffers
  for (int i = 0; i < MAX_ANCHORS; i++) {
    memset(tag_ranges_buffer[i], 0, sizeof(tag_ranges_buffer[i]));
    tag_ranges[i] = 0;
    buffer_index[i] = 0;
    buffer_full[i] = 0;
  }

  // Start the UART3 receive interrupt (PA10 is UART1 RX)
  if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1) != HAL_OK) {
    print_debug("ERROR: Failed to start UART3 reception\r\n");
  }

  // Send initial message and anchor information to PC
  print_debug("UWB Trilateration System Initialized\r\n");
  print_debug("----------------------------------\r\n");

  char anchor_info[100];
  for (int i = 0; i < MAX_ANCHORS; i++) {
    sprintf(anchor_info, "Anchor %d: (%.2f, %.2f) cm\r\n", i, anchors[i].x, anchors[i].y);
    print_debug(anchor_info);
  }
  print_debug("----------------------------------\r\n");
  
  // Set the target position for navigation
  char target_info[100];
  sprintf(target_info, "Target position set to: (%.2f, %.2f) cm\r\n", 
          TARGET_X_POSITION, TARGET_Y_POSITION);
  print_debug(target_info);
  print_debug("Press blue button (PC13) to start navigation\r\n");
  print_debug("----------------------------------\r\n");
  print_debug("Listening for range data...\r\n\r\n");

  // Initialize motor control pins
  HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_Port, STBY, GPIO_PIN_SET);
  
  // Start PWM for motor control
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // If we have received data, process it
    if (rx_data_ready) {
        process_uart_data();
        rx_data_ready = 0;
    }

    // Process navigation if active and we have valid position
    if (navigation_active && tag.status) {
        char debug_msg[100];
        
        // Calculate differences in X and Y
        float diff_y = TARGET_Y_POSITION - tag.y;
        float diff_x = TARGET_X_POSITION - tag.x;
        
        // Convert to physical units (assuming 175 units = 1 meter)
        float meters_y = diff_y / UWB_UNITS_PER_METER;
        float meters_x = diff_x / UWB_UNITS_PER_METER;
        
        // Calculate drive times (based on the robot's speed)
        int y_drive_time = (int)(fabs(meters_y) * DRIVE_TIME_PER_METER);
        int x_drive_time = (int)(fabs(meters_x) * DRIVE_TIME_PER_METER);
        
        // Handle navigation stages
        switch(navigation_stage) {
            case 1: // Y-direction movement
                sprintf(debug_msg, "Moving %.2f meters in Y direction\r\n", meters_y);
                print_debug(debug_msg);
                
                // Determine direction for Y movement
                if (fabs(meters_y) > 0.05f) { // Only move if difference is significant
                    if (meters_y > 0) {
                        // Set motor direction forward
                        HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_RESET);
                    } else {
                        // Set motor direction backward
                        HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_SET);
                    }
                    
                    // Enable motors
                    HAL_GPIO_WritePin(STBY_Port, STBY, GPIO_PIN_SET);
                    
                    // Set PWM for both motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 128);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 128);
                    
                    // Run for calculated time
                    HAL_Delay(y_drive_time);
                    
                    // Stop motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                    
                    print_debug("Y movement complete\r\n");
                }
                
                // Proceed to turning stage
                navigation_stage = 2;
                break;
                
            case 2: // Turning 90 degrees right
                if (fabs(meters_x) > 0.05f) { // Only turn if X difference is significant
                    print_debug("Turning 90 degrees right\r\n");
                    
                    // Set one motor forward, one backward for turning
                    HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_SET);
                    
                    // Enable motors
                    HAL_GPIO_WritePin(STBY_Port, STBY, GPIO_PIN_SET);
                    
                    // Set PWM for both motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 128);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 128);
                    
                    // Turn for fixed time (adjust as needed for 90 degrees)
                    HAL_Delay(900);
                    
                    // Stop motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                    
                    print_debug("Turn complete\r\n");
                }
                
                // Proceed to X-direction movement
                navigation_stage = 3;
                break;
                
            case 3: // X-direction movement
                sprintf(debug_msg, "Moving %.2f meters in X direction\r\n", meters_x);
                print_debug(debug_msg);
                
                // Determine direction for X movement
                if (fabs(meters_x) > 0.05f) { // Only move if difference is significant
                    if (meters_x > 0) {
                        // Set motor direction forward
                        HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_RESET);
                    } else {
                        // Set motor direction backward
                        HAL_GPIO_WritePin(IN_Port, AIN1, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(IN_Port, BIN1, GPIO_PIN_SET);
                    }
                    
                    // Enable motors
                    HAL_GPIO_WritePin(STBY_Port, STBY, GPIO_PIN_SET);
                    
                    // Set PWM for both motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 128);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 128);
                    
                    // Run for calculated time
                    HAL_Delay(x_drive_time);
                    
                    // Stop motors
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                    
                    print_debug("X movement complete\r\n");
                }
                
                // Navigation complete
                navigation_stage = 4;
                print_debug("Navigation complete!\r\n");
                navigation_active = 0;
                break;
                
            case 4: // Done
                // Already completed navigation
                navigation_active = 0;
                break;
        }
    }

    // Toggle LED to indicate system is running
    static uint32_t last_led_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_led_time > 500) {
        last_led_time = current_time;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    
    // Small delay to prevent CPU hogging
    HAL_Delay(100);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Enable EXTI interrupt for the blue button
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
