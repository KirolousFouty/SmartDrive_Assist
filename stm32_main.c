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
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FLASH_USER_START_ADDR   0x0801F800  // Last page of 128KB flash (page 127)
#define FLASH_MAGIC_NUMBER      0xCAFEBEEF  // Magic number to verify valid calibration
typedef struct {
    uint32_t magic;           // Magic number for validation
    uint16_t joy_center_x;
    uint16_t joy_center_y;
    uint16_t joy_max_x;
    uint16_t joy_min_x;
    uint16_t joy_max_y;
    uint16_t joy_min_y;
    uint16_t joy_deadzone;
    uint32_t checksum;        // Simple checksum for data integrity
} CalibrationData;
typedef struct {
    int8_t m1_value;        // Motor 1: negative=backward, 0=stop, positive=forward
    int8_t m2_value;        // Motor 2: negative=backward, 0=stop, positive=forward
    uint8_t mode;           // Current mode code
    int16_t angle;          // Angle in degrees (-90 to +90, negative=left, positive=right)
    char direction[32];     // Full direction description
    char mode_desc[24];     // Mode description in words
} MotorCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============== Pre-defined Trip Definitions ==============
#define TRIP_MAX_STEPS      20
#define TRIP_STEP_FORWARD   0x01
#define TRIP_STEP_BACKWARD  0x02
#define TRIP_STEP_LEFT      0x03
#define TRIP_STEP_RIGHT     0x04
#define TRIP_STEP_SPIN_L    0x05
#define TRIP_STEP_SPIN_R    0x06
#define TRIP_STEP_PAUSE     0x07
#define TRIP_STEP_END       0xFF

typedef struct {
    uint8_t action;         // Action type
    uint8_t speed_percent;  // Speed 0-100%
    uint16_t duration_ms;   // Duration in milliseconds
} TripStep;



// ============== Protocol Definitions (STM32 -> ESP32) ==============
#define PROTO_START     0xAA    // Frame start byte

// Message types
#define MSG_SPEED       0x01    // Speed value update
#define MSG_MODE        0x02    // Mode update
#define MSG_DISTANCE    0x03    // Ultrasonic distance
#define MSG_MOTOR       0x04    // Motor values (M1, M2)
#define MSG_STATUS      0x05    // System status flags
#define MSG_MOTOR_CMD   0x06    // Motor command for ESP32 to relay

// Mode codes
#define MODE_STOP       0x00
#define MODE_FORWARD    0x01
#define MODE_BACKWARD   0x02
#define MODE_LEFT       0x03
#define MODE_RIGHT      0x04
#define MODE_SPIN_LEFT  0x05
#define MODE_SPIN_RIGHT 0x06
#define MODE_PRECISION  0x07

// ============== ADC Definitions ==============
#define ADC_MIN         0
#define ADC_MAX         4095

// ============== Dagu Motor Driver Commands ==============
#define CMD_M1_FORWARD  0xCA
#define CMD_M1_BACKWARD 0xC9
#define CMD_M1_STOP     0xC8
#define CMD_M2_FORWARD  0xC2
#define CMD_M2_BACKWARD 0xC1
#define CMD_M2_STOP     0xC0

// ============== Safety Thresholds ==============
#define OBSTACLE_WARN_DIST   20  // cm - reduce speed
#define OBSTACLE_STOP_DIST   10  // cm - emergency stop

// ============== Timing Definitions ==============
#define ULTRASONIC_INTERVAL    100   // ms between ultrasonic triggers
#define STATUS_PRINT_INTERVAL  500   // ms between status prints (0.5 sec)
#define CONTROL_LOOP_DELAY     20    // ms main loop delay

// ============== Direction Constants ==============
#define DIR_THRESHOLD_STRAIGHT  10   // Degrees within which is "straight"
#define DIR_THRESHOLD_SLIGHT    25   // Degrees for "slight" turn
#define DIR_THRESHOLD_MODERATE  50   // Degrees for "moderate" turn

// ============== Joystick Calibration ==============
#define CALIB_SAMPLES      20       // Number of samples per direction
#define CALIB_TIME_MS      2000     // Time to hold each direction (2 sec)
#define DEADZONE_PERCENT   15       // Deadzone as percentage of range
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ============== Spin State ==============
volatile bool spin_active = false;
volatile uint32_t spin_start_time = 0;
#define SPIN_DURATION_MS  500  // Spin for 500ms per key press

// ============== Pre-defined Trip Variables ==============
volatile bool trip_active = false;
volatile bool trip_paused = false;
volatile uint8_t trip_current_step = 0;
volatile uint32_t trip_step_start_time = 0;

// Pre-defined trip: Demo pattern (customize as needed)
// Pattern: Forward -> Pause -> Right turn -> Pause -> Forward -> Spin Left -> Back to start
const TripStep predefined_trip[TRIP_MAX_STEPS] = {
    {TRIP_STEP_FORWARD,  50, 2000},  // Forward at 50% for 2 sec
    {TRIP_STEP_PAUSE,     0,  500},  // Pause 0.5 sec
    {TRIP_STEP_RIGHT,    40, 1000},  // Turn right at 40% for 1 sec
    {TRIP_STEP_PAUSE,     0,  500},  // Pause 0.5 sec
    {TRIP_STEP_FORWARD,  50, 2000},  // Forward at 50% for 2 sec
    {TRIP_STEP_PAUSE,     0,  500},  // Pause 0.5 sec
    {TRIP_STEP_SPIN_L,   35, 1500},  // Spin left at 35% for 1.5 sec
    {TRIP_STEP_PAUSE,     0,  500},  // Pause 0.5 sec
    {TRIP_STEP_BACKWARD, 50, 2000},  // Backward at 50% for 2 sec
    {TRIP_STEP_END,       0,    0},  // End marker
};


// ============== Ultrasonic Sensor Variables ==============
volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;
volatile uint8_t Distance = 0;
volatile bool distance_ready = false;

// ============== Joystick Calibration Values ==============
uint16_t joy_center_x = 2048;
uint16_t joy_center_y = 2048;
uint16_t joy_max_x = 4095;      // Right
uint16_t joy_min_x = 0;         // Left
uint16_t joy_max_y = 4095;      // Up/Forward
uint16_t joy_min_y = 0;         // Down/Backward
uint16_t joy_deadzone = 200;    // Calculated deadzone

// ============== Motor Control Variables ==============
uint8_t current_speed = 0;
uint8_t current_mode = MODE_STOP;
volatile int16_t last_m1_value = 0;
volatile int16_t last_m2_value = 0;
bool precision_mode = false;
bool obstacle_detected = false;
bool emergency_stop = false;

// ============== Current Joystick State ==============
int16_t current_joy_angle = 0;      // 0-359 degrees (compass style)
uint8_t current_joy_magnitude = 0;  // 0-100 percent
bool joystick_active = false;

// ============== Timing Variables ==============
uint32_t last_ultrasonic_time = 0;
uint32_t last_status_print_time = 0;

// ============== Communication Buffers ==============
char debug_msg[150];
uint8_t esp32_tx_buffer[16];

// ============== Keypad Configuration ==============
char keymap[4][4] = {
    {'4', '7', '*', '1'},
    {'5', '8', '0', '2'},
    {'B', 'C', 'D', 'A'},
    {'6', '9', '#', '3'}
};

GPIO_TypeDef* row_ports[4] = {GPIOA, GPIOA, GPIOB, GPIOB};
uint16_t row_pins[4] = {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_4, GPIO_PIN_5};
GPIO_TypeDef* col_ports[4] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t col_pins[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_1};

// ============== Last Action String ==============
char last_action[32] = "";
uint32_t last_action_time = 0;
#define ACTION_DISPLAY_TIME 2000  // Show action for 2 seconds
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Process_Spin(void);

// Trip functions
void Start_Trip(void);
void Stop_Trip(void);
void Process_Trip(void);
void Execute_Trip_Step(const TripStep *step);

// Utility functions
void delay_us(uint16_t us);
void UART_Debug(const char *str);

// Protocol functions
uint8_t Calculate_Checksum(uint8_t *data, uint8_t len);
void Send_To_ESP32(uint8_t msg_type, uint8_t *data, uint8_t len);
void Send_Motor_To_ESP32(uint8_t motor_cmd, uint8_t speed_val);

// Input functions
char Keypad_Read(void);
uint16_t Read_ADC_X(void);
uint16_t Read_ADC_Y(void);

// Calibration functions
void Calibrate_Joystick(void);
uint16_t Read_ADC_Average(bool is_x_axis, uint16_t samples);

// Motor control functions
MotorCommand Get_Motor_Command(uint16_t x, uint16_t y, uint8_t max_speed);
void Execute_Motor_Command(MotorCommand cmd);
void Force_Stop(void);

// Direction helper functions
void Calculate_Joystick_Vector(uint16_t x, uint16_t y, int16_t *angle, uint8_t *magnitude);
void Get_Direction_String(int16_t angle, uint8_t magnitude, bool is_reverse, char *dir_str);
void Get_Compass_Direction(int16_t angle, char *compass_str);

// Processing functions
void Trigger_Ultrasonic(void);
void Process_Keypad(void);
void Process_Joystick(void);
void Process_Safety(void);
void Print_Status(void);
void Set_Action(const char *action);

bool Load_Calibration_From_Flash(void);
bool Save_Calibration_To_Flash(void);
void Erase_Calibration_From_Flash(void);
uint32_t Calculate_Calib_Checksum(CalibrationData *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== TRIP FUNCTIONS ====================
void Start_Trip(void)
{
    if (emergency_stop)
    {
        UART_Debug("Cannot start trip - emergency stop active!\r\n");
        Set_Action("Trip blocked");
        return;
    }
    
    if (obstacle_detected)
    {
        UART_Debug("Cannot start trip - obstacle detected!\r\n");
        Set_Action("Trip blocked");
        return;
    }
    
    trip_active = true;
    trip_paused = false;
    trip_current_step = 0;
    trip_step_start_time = HAL_GetTick();
    
    UART_Debug("\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    UART_Debug("|       PRE-DEFINED TRIP STARTED           |\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    UART_Debug("Press [C] or [0] to abort trip.\r\n\r\n");
    
    Set_Action("Trip Started");
    
    // Execute first step immediately
    Execute_Trip_Step(&predefined_trip[0]);
}

void Stop_Trip(void)
{
    if (trip_active)
    {
        trip_active = false;
        trip_paused = false;
        trip_current_step = 0;
        
        // Stop motors
        Send_Motor_To_ESP32(CMD_M1_STOP, 0);
        Send_Motor_To_ESP32(CMD_M2_STOP, 0);
        
        last_m1_value = 0;
        last_m2_value = 0;
        current_mode = MODE_STOP;
        
        UART_Debug("\r\n>>> Trip stopped/completed.\r\n\r\n");
        Set_Action("Trip Ended");
    }
}

void Execute_Trip_Step(const TripStep *step)
{
    if (step->action == TRIP_STEP_END)
    {
        Stop_Trip();
        return;
    }
    
    // Convert speed percent to motor value (0-127)
    uint8_t motor_speed = (uint8_t)((step->speed_percent * 127) / 100);
    
    int8_t m1 = 0, m2 = 0;
    const char *action_str = "";
    
    switch (step->action)
    {
        case TRIP_STEP_FORWARD:
            m1 = motor_speed;
            m2 = motor_speed;
            action_str = "Trip: FWD";
            current_mode = MODE_FORWARD;
            break;
            
        case TRIP_STEP_BACKWARD:
            m1 = -motor_speed;
            m2 = -motor_speed;
            action_str = "Trip: REV";
            current_mode = MODE_BACKWARD;
            break;
            
        case TRIP_STEP_LEFT:
            m1 = motor_speed / 3;      // Slow inner wheel
            m2 = motor_speed;          // Fast outer wheel
            action_str = "Trip: LEFT";
            current_mode = MODE_LEFT;
            break;
            
        case TRIP_STEP_RIGHT:
            m1 = motor_speed;          // Fast outer wheel
            m2 = motor_speed / 3;      // Slow inner wheel
            action_str = "Trip: RIGHT";
            current_mode = MODE_RIGHT;
            break;
            
        case TRIP_STEP_SPIN_L:
            m1 = -motor_speed;
            m2 = motor_speed;
            action_str = "Trip: SPIN L";
            current_mode = MODE_SPIN_LEFT;
            break;
            
        case TRIP_STEP_SPIN_R:
            m1 = motor_speed;
            m2 = -motor_speed;
            action_str = "Trip: SPIN R";
            current_mode = MODE_SPIN_RIGHT;
            break;
            
        case TRIP_STEP_PAUSE:
            m1 = 0;
            m2 = 0;
            action_str = "Trip: PAUSE";
            current_mode = MODE_STOP;
            break;
            
        default:
            m1 = 0;
            m2 = 0;
            action_str = "Trip: ???";
            current_mode = MODE_STOP;
            break;
    }
    
    // Send motor commands
    if (m1 > 0)
        Send_Motor_To_ESP32(CMD_M1_FORWARD, (uint8_t)m1);
    else if (m1 < 0)
        Send_Motor_To_ESP32(CMD_M1_BACKWARD, (uint8_t)(-m1));
    else
        Send_Motor_To_ESP32(CMD_M1_STOP, 0);
    
    if (m2 > 0)
        Send_Motor_To_ESP32(CMD_M2_FORWARD, (uint8_t)m2);
    else if (m2 < 0)
        Send_Motor_To_ESP32(CMD_M2_BACKWARD, (uint8_t)(-m2));
    else
        Send_Motor_To_ESP32(CMD_M2_STOP, 0);
    
    last_m1_value = m1;
    last_m2_value = m2;
    
    Set_Action(action_str);
    
    sprintf(debug_msg, "Trip Step %d: %s, Speed=%d%%, Duration=%dms\r\n",
            trip_current_step, action_str, step->speed_percent, step->duration_ms);
    UART_Debug(debug_msg);
}


// ==================== TIMER CALLBACK ====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (Is_First_Captured == 0)
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (Is_First_Captured == 1)
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            if (IC_Val2 > IC_Val1)
                Difference = IC_Val2 - IC_Val1;
            else
                Difference = (0xFFFF - IC_Val1) + IC_Val2;

            Distance = (uint8_t)(Difference * 0.0343f / 2.0f);
            distance_ready = true;
            Is_First_Captured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
    }
}

// ==================== UTILITY FUNCTIONS ====================
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void UART_Debug(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}

void Set_Action(const char *action)
{
    strncpy(last_action, action, sizeof(last_action) - 1);
    last_action[sizeof(last_action) - 1] = '\0';
    last_action_time = HAL_GetTick();
}

// ==================== CALIBRATION FUNCTIONS ====================
uint16_t Read_ADC_Average(bool is_x_axis, uint16_t samples)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++)
    {
        if (is_x_axis)
            sum += Read_ADC_X();
        else
            sum += Read_ADC_Y();
        HAL_Delay(10);
    }
    return (uint16_t)(sum / samples);
}


// ==================== FLASH STORAGE FUNCTIONS ====================
uint32_t Calculate_Calib_Checksum(CalibrationData *data)
{
    uint32_t sum = 0;
    sum += data->magic;
    sum += data->joy_center_x;
    sum += data->joy_center_y;
    sum += data->joy_max_x;
    sum += data->joy_min_x;
    sum += data->joy_max_y;
    sum += data->joy_min_y;
    sum += data->joy_deadzone;
    return sum;
}

bool Load_Calibration_From_Flash(void)
{
    CalibrationData *stored_data = (CalibrationData *)FLASH_USER_START_ADDR;
    
    // Check magic number
    if (stored_data->magic != FLASH_MAGIC_NUMBER)
    {
        UART_Debug("No valid calibration found in flash.\r\n");
        return false;
    }
    
    // Verify checksum
    uint32_t calculated_checksum = Calculate_Calib_Checksum(stored_data);
    if (stored_data->checksum != calculated_checksum)
    {
        UART_Debug("Calibration data corrupted!\r\n");
        return false;
    }
    
    // Load calibration values
    joy_center_x = stored_data->joy_center_x;
    joy_center_y = stored_data->joy_center_y;
    joy_max_x = stored_data->joy_max_x;
    joy_min_x = stored_data->joy_min_x;
    joy_max_y = stored_data->joy_max_y;
    joy_min_y = stored_data->joy_min_y;
    joy_deadzone = stored_data->joy_deadzone;
    
    UART_Debug("\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    UART_Debug("|   CALIBRATION LOADED FROM FLASH          |\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    sprintf(debug_msg, "Center: X=%d, Y=%d\r\n", joy_center_x, joy_center_y);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Range X: %d to %d\r\n", joy_min_x, joy_max_x);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Range Y: %d to %d\r\n", joy_min_y, joy_max_y);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Deadzone: %d\r\n\r\n", joy_deadzone);
    UART_Debug(debug_msg);
    
    return true;
}

bool Save_Calibration_To_Flash(void)
{
    HAL_StatusTypeDef status;
    CalibrationData calib_data;
    
    // Prepare calibration data
    calib_data.magic = FLASH_MAGIC_NUMBER;
    calib_data.joy_center_x = joy_center_x;
    calib_data.joy_center_y = joy_center_y;
    calib_data.joy_max_x = joy_max_x;
    calib_data.joy_min_x = joy_min_x;
    calib_data.joy_max_y = joy_max_y;
    calib_data.joy_min_y = joy_min_y;
    calib_data.joy_deadzone = joy_deadzone;
    calib_data.checksum = Calculate_Calib_Checksum(&calib_data);
    
    // Unlock flash
    HAL_FLASH_Unlock();
    
    // Erase the page first
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = 127;  // Last page
    EraseInitStruct.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        UART_Debug("Flash erase failed!\r\n");
        return false;
    }
    
    // Write calibration data (64-bit aligned writes)
    uint64_t *data_ptr = (uint64_t *)&calib_data;
    uint32_t addr = FLASH_USER_START_ADDR;
    size_t num_dwords = (sizeof(CalibrationData) + 7) / 8;  // Round up to 64-bit
    
    for (size_t i = 0; i < num_dwords; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data_ptr[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            UART_Debug("Flash write failed!\r\n");
            return false;
        }
        addr += 8;
    }
    
    // Lock flash
    HAL_FLASH_Lock();
    
    UART_Debug("\r\n>>> Calibration saved to flash successfully!\r\n\r\n");
    return true;
}

void Erase_Calibration_From_Flash(void)
{
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = 127;
    EraseInitStruct.NbPages = 1;
    
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Lock();
    
    UART_Debug("Calibration erased from flash.\r\n");
}

// ==================== MODIFIED CALIBRATION FUNCTION ====================
void Calibrate_Joystick(void)
{
    UART_Debug("\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    UART_Debug("|       JOYSTICK CALIBRATION               |\r\n");
    UART_Debug("+------------------------------------------+\r\n\r\n");
    
    // Step 1: Center position
    UART_Debug(">> Release joystick to CENTER position...\r\n");
    UART_Debug("   Calibrating in 5 seconds...\r\n");
    HAL_Delay(5000);
    
    joy_center_x = Read_ADC_Average(true, CALIB_SAMPLES);
    joy_center_y = Read_ADC_Average(false, CALIB_SAMPLES);
    sprintf(debug_msg, "   Center: X=%d, Y=%d\r\n\r\n", joy_center_x, joy_center_y);
    UART_Debug(debug_msg);
    
    // Step 2: Forward (Up)
    UART_Debug(">> Push joystick FORWARD (UP) and hold...\r\n");
    for (int i = 10; i > 0; i--)
    {
        sprintf(debug_msg, "   Reading in %d...\r\n", i);
        UART_Debug(debug_msg);
        HAL_Delay(1000);
    }
    joy_max_y = Read_ADC_Average(false, CALIB_SAMPLES);
    sprintf(debug_msg, "   Forward: Y=%d\r\n\r\n", joy_max_y);
    UART_Debug(debug_msg);
    
    // Step 3: Right
    UART_Debug(">> Push joystick RIGHT and hold...\r\n");
    for (int i = 10; i > 0; i--)
    {
        sprintf(debug_msg, "   Reading in %d...\r\n", i);
        UART_Debug(debug_msg);
        HAL_Delay(1000);
    }
    joy_max_x = Read_ADC_Average(true, CALIB_SAMPLES);
    sprintf(debug_msg, "   Right: X=%d\r\n\r\n", joy_max_x);
    UART_Debug(debug_msg);
    
    // Step 4: Backward (Down)
    UART_Debug(">> Push joystick BACKWARD (DOWN) and hold...\r\n");
    for (int i = 10; i > 0; i--)
    {
        sprintf(debug_msg, "   Reading in %d...\r\n", i);
        UART_Debug(debug_msg);
        HAL_Delay(1000);
    }
    joy_min_y = Read_ADC_Average(false, CALIB_SAMPLES);
    sprintf(debug_msg, "   Backward: Y=%d\r\n\r\n", joy_min_y);
    UART_Debug(debug_msg);
    
    // Step 5: Left
    UART_Debug(">> Push joystick LEFT and hold...\r\n");
    for (int i = 10; i > 0; i--)
    {
        sprintf(debug_msg, "   Reading in %d...\r\n", i);
        UART_Debug(debug_msg);
        HAL_Delay(1000);
    }
    joy_min_x = Read_ADC_Average(true, CALIB_SAMPLES);
    sprintf(debug_msg, "   Left: X=%d\r\n\r\n", joy_min_x);
    UART_Debug(debug_msg);
    
    // Calculate deadzone
    uint16_t range_x = (joy_max_x > joy_min_x) ? (joy_max_x - joy_min_x) : (joy_min_x - joy_max_x);
    uint16_t range_y = (joy_max_y > joy_min_y) ? (joy_max_y - joy_min_y) : (joy_min_y - joy_max_y);
    uint16_t min_range = (range_x < range_y) ? range_x : range_y;
    joy_deadzone = (min_range * DEADZONE_PERCENT) / 100;
    
    // Summary
    UART_Debug("+------------------------------------------+\r\n");
    UART_Debug("|       CALIBRATION COMPLETE               |\r\n");
    UART_Debug("+------------------------------------------+\r\n");
    sprintf(debug_msg, "Center: X=%d, Y=%d\r\n", joy_center_x, joy_center_y);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Range X: %d to %d\r\n", joy_min_x, joy_max_x);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Range Y: %d to %d\r\n", joy_min_y, joy_max_y);
    UART_Debug(debug_msg);
    sprintf(debug_msg, "Deadzone: %d\r\n\r\n", joy_deadzone);
    UART_Debug(debug_msg);
    
    // **SAVE TO FLASH**
    Save_Calibration_To_Flash();
}

// ==================== DIRECTION/BEARING FUNCTIONS ====================
void Calculate_Joystick_Vector(uint16_t x, uint16_t y, int16_t *angle, uint8_t *magnitude)
{
    // Normalize to -100 to +100 range
    float x_norm = 0.0f;
    float y_norm = 0.0f;
    
    // X axis normalization
    if (x > joy_center_x + joy_deadzone)
    {
        // Right
        uint16_t range = (joy_max_x > joy_center_x) ? (joy_max_x - joy_center_x) : 1;
        x_norm = (float)(x - joy_center_x - joy_deadzone) / (float)(range - joy_deadzone);
        if (x_norm > 1.0f) x_norm = 1.0f;
    }
    else if (x < joy_center_x - joy_deadzone)
    {
        // Left
        uint16_t range = (joy_center_x > joy_min_x) ? (joy_center_x - joy_min_x) : 1;
        x_norm = (float)((int16_t)x - (int16_t)joy_center_x + (int16_t)joy_deadzone) / (float)(range - joy_deadzone);
        if (x_norm < -1.0f) x_norm = -1.0f;
    }
    
    // Y axis normalization
    if (y > joy_center_y + joy_deadzone)
    {
        // Forward
        uint16_t range = (joy_max_y > joy_center_y) ? (joy_max_y - joy_center_y) : 1;
        y_norm = (float)(y - joy_center_y - joy_deadzone) / (float)(range - joy_deadzone);
        if (y_norm > 1.0f) y_norm = 1.0f;
    }
    else if (y < joy_center_y - joy_deadzone)
    {
        // Backward
        uint16_t range = (joy_center_y > joy_min_y) ? (joy_center_y - joy_min_y) : 1;
        y_norm = (float)((int16_t)y - (int16_t)joy_center_y + (int16_t)joy_deadzone) / (float)(range - joy_deadzone);
        if (y_norm < -1.0f) y_norm = -1.0f;
    }
    
    // Calculate magnitude (0-100)
    float mag = sqrtf(x_norm * x_norm + y_norm * y_norm);
    if (mag > 1.0f) mag = 1.0f;
    *magnitude = (uint8_t)(mag * 100);
    
    // Calculate angle (0-359, compass style: 0=forward, 90=right, 180=back, 270=left)
    if (*magnitude < 5)
    {
        *angle = 0;
        *magnitude = 0;
    }
    else
    {
        // atan2 gives angle from positive X axis, counterclockwise
        // We want: 0=up, 90=right, 180=down, 270=left
        float rad = atan2f(x_norm, y_norm);  // Note: swapped for compass orientation
        int16_t deg = (int16_t)(rad * 180.0f / 3.14159f);
        
        // Convert to 0-359
        if (deg < 0) deg += 360;
        *angle = deg;
    }
}

void Get_Compass_Direction(int16_t angle, char *compass_str)
{
    if (angle >= 337 || angle < 23)
        sprintf(compass_str, "N");
    else if (angle >= 23 && angle < 67)
        sprintf(compass_str, "NE");
    else if (angle >= 67 && angle < 113)
        sprintf(compass_str, "E");
    else if (angle >= 113 && angle < 157)
        sprintf(compass_str, "SE");
    else if (angle >= 157 && angle < 203)
        sprintf(compass_str, "S");
    else if (angle >= 203 && angle < 247)
        sprintf(compass_str, "SW");
    else if (angle >= 247 && angle < 293)
        sprintf(compass_str, "W");
    else
        sprintf(compass_str, "NW");
}

void Get_Direction_String(int16_t angle, uint8_t magnitude, bool is_reverse, char *dir_str)
{
    if (magnitude == 0)
    {
        sprintf(dir_str, "Centered");
        return;
    }
    
    // Determine turn angle from straight (0 or 180)
    int16_t turn_angle;
    const char *base_dir;
    const char *turn_dir;
    
    if (angle >= 270 || angle < 90)
    {
        // Forward hemisphere
        base_dir = is_reverse ? "Reverse" : "Forward";
        if (angle >= 270)
            turn_angle = 360 - angle;
        else
            turn_angle = angle;
        turn_dir = (angle > 0 && angle < 180) ? "right" : "left";
    }
    else
    {
        // Backward hemisphere
        base_dir = is_reverse ? "Forward" : "Reverse";
        turn_angle = (angle > 180) ? (angle - 180) : (180 - angle);
        turn_dir = (angle > 180) ? "left" : "right";
    }
    
    // Handle pure sideways as spin
    if (angle >= 80 && angle <= 100)
    {
        sprintf(dir_str, "Spin right, %d%%", magnitude);
        return;
    }
    else if (angle >= 260 && angle <= 280)
    {
        sprintf(dir_str, "Spin left, %d%%", magnitude);
        return;
    }
    
    // Format direction string
    if (turn_angle <= DIR_THRESHOLD_STRAIGHT)
    {
        sprintf(dir_str, "%s, straight", base_dir);
    }
    else
    {
        sprintf(dir_str, "%s, %d deg %s", base_dir, turn_angle, turn_dir);
    }
}

// ==================== PROTOCOL FUNCTIONS ====================
uint8_t Calculate_Checksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++)
        checksum ^= data[i];
    return checksum;
}

void Send_To_ESP32(uint8_t msg_type, uint8_t *data, uint8_t len)
{
    uint8_t frame[16];
    uint8_t idx = 0;
    
    frame[idx++] = PROTO_START;
    frame[idx++] = msg_type;
    frame[idx++] = len;
    
    for (uint8_t i = 0; i < len && i < 10; i++)
        frame[idx++] = data[i];
    
    frame[idx] = Calculate_Checksum(&frame[1], len + 2);
    idx++;
    
    HAL_UART_Transmit(&huart1, frame, idx, 10);
}

void Send_Motor_To_ESP32(uint8_t motor_cmd, uint8_t speed_val)
{
    uint8_t data[2] = {motor_cmd, speed_val};
    Send_To_ESP32(MSG_MOTOR_CMD, data, 2);
}

// ==================== INPUT FUNCTIONS ====================
char Keypad_Read(void)
{
    char key = 0;
    
    for (int r = 0; r < 4; r++)
    {
        // Set all rows HIGH first
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        
        // Small delay for signal to settle
        for (volatile int d = 0; d < 100; d++);
        
        // Set current row LOW
        HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_RESET);
        
        // Longer delay for signal to settle (critical for reliable reading)
        for (volatile int d = 0; d < 200; d++);
        
        // Check columns
        for (int c = 0; c < 4; c++)
        {
            if (HAL_GPIO_ReadPin(col_ports[c], col_pins[c]) == GPIO_PIN_RESET)
            {
                key = keymap[r][c];
                
                // Set row back to HIGH before returning
                HAL_GPIO_WritePin(row_ports[r], row_pins[r], GPIO_PIN_SET);
                
                return key;
            }
        }
    }
    
    return 0;  // No key pressed
}

uint16_t Read_ADC_X(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return joy_center_x;
}

uint16_t Read_ADC_Y(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_16;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint16_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return joy_center_y;
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
MotorCommand Get_Motor_Command(uint16_t x, uint16_t y, uint8_t max_speed)
{
    MotorCommand cmd = {0, 0, MODE_STOP, 0, "Centered", "Idle"};
    
    // Calculate joystick vector
    Calculate_Joystick_Vector(x, y, &current_joy_angle, &current_joy_magnitude);
    
    if (current_joy_magnitude < 5)
    {
        joystick_active = false;
        return cmd;
    }
    
    joystick_active = true;
    
    // Convert angle and magnitude to normalized X/Y
    float rad = (float)current_joy_angle * 3.14159f / 180.0f;
    float x_norm = sinf(rad) * (current_joy_magnitude / 100.0f);
    float y_norm = cosf(rad) * (current_joy_magnitude / 100.0f);
    
		// Tank drive mixing:
		float m1_float = y_norm - x_norm;
		float m2_float = y_norm + x_norm;
    
    // Clamp to valid range
    if (m1_float > 1.0f) m1_float = 1.0f;
    if (m1_float < -1.0f) m1_float = -1.0f;
    if (m2_float > 1.0f) m2_float = 1.0f;
    if (m2_float < -1.0f) m2_float = -1.0f;
    
    // Apply precision mode (50% speed reduction)
    uint8_t effective_speed = precision_mode ? (max_speed / 2) : max_speed;
    
    // Scale to motor values
    cmd.m1_value = (int8_t)(m1_float * effective_speed);
    cmd.m2_value = (int8_t)(m2_float * effective_speed);
    cmd.angle = current_joy_angle;
    
    // Determine mode code
    if (cmd.m1_value == 0 && cmd.m2_value == 0)
        cmd.mode = MODE_STOP;
    else if (cmd.m1_value > 0 && cmd.m2_value > 0)
        cmd.mode = MODE_FORWARD;
    else if (cmd.m1_value < 0 && cmd.m2_value < 0)
        cmd.mode = MODE_BACKWARD;
    else if (cmd.m1_value > 0 && cmd.m2_value < 0)
        cmd.mode = MODE_SPIN_RIGHT;
    else if (cmd.m1_value < 0 && cmd.m2_value > 0)
        cmd.mode = MODE_SPIN_LEFT;
    else
        cmd.mode = MODE_FORWARD;
    
    // Get direction string
    bool is_reverse = (cmd.m1_value < 0 && cmd.m2_value < 0);
    Get_Direction_String(current_joy_angle, current_joy_magnitude, is_reverse, cmd.direction);
    
    // Mode description
    switch (cmd.mode)
    {
        case MODE_FORWARD:
            sprintf(cmd.mode_desc, "Driving");
            break;
        case MODE_BACKWARD:
            sprintf(cmd.mode_desc, "Reversing");
            break;
        case MODE_SPIN_LEFT:
            sprintf(cmd.mode_desc, "Spinning Left");
            break;
        case MODE_SPIN_RIGHT:
            sprintf(cmd.mode_desc, "Spinning Right");
            break;
        default:
            sprintf(cmd.mode_desc, "Idle");
    }
    
    return cmd;
}

void Force_Stop(void)
{
    // Stop any active trip first
    if (trip_active)
    {
        trip_active = false;
        trip_paused = false;
        trip_current_step = 0;
    }
    
		 spin_active = false;
		
    // Send stop commands to motors
    Send_Motor_To_ESP32(CMD_M1_STOP, 0);
    HAL_Delay(5);  // Small delay to ensure command is sent
    Send_Motor_To_ESP32(CMD_M2_STOP, 0);
    
    // Reset all motor-related state
    current_mode = MODE_STOP;
    last_m1_value = 0;
    last_m2_value = 0;
    current_speed = 0;  // Reset speed to 0
    emergency_stop = true;
    joystick_active = false;
    current_joy_magnitude = 0;
    current_joy_angle = 0;
    
    // Notify ESP32 of mode change
    uint8_t mode_data = MODE_STOP;
    Send_To_ESP32(MSG_MODE, &mode_data, 1);
    
    // Notify ESP32 of speed change
    Send_To_ESP32(MSG_SPEED, &current_speed, 1);
    
    // Visual indication
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    
    UART_Debug("\r\n!!! EMERGENCY STOP ACTIVATED !!!\r\n");
}

// ==================== PROCESSING FUNCTIONS ====================
void Trigger_Ultrasonic(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

void Process_Keypad(void)
{
    static uint32_t last_key_time = 0;
    static char last_key = 0;
    
    char key = Keypad_Read();
    uint32_t now = HAL_GetTick();
    
    // Debounce: ignore if same key within 200ms
    if (key != 0 && (key != last_key || (now - last_key_time) > 200))
    {
        last_key = key;
        last_key_time = now;
        
        // Process the key
        switch (key)
        {
            case '0':
            {
                // Emergency Stop
                current_speed = 0;
                Force_Stop();
                Set_Action("E-STOP [0]");
                break;
            }
            
            case 'C':
            {
                // Emergency Stop - also stops any active trip
                current_speed = 0;  // Ensure speed is reset
                Force_Stop();
                Set_Action("E-STOP [C]");
                break;
            }
                
            case '1':
            {
                current_speed = 14;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=11%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '2':
            {
                current_speed = 28;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=22%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '3':
            {
                current_speed = 42;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=33%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '4':
            {
                current_speed = 56;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=44%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '5':
            {
                current_speed = 70;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=55%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '6':
            {
                current_speed = 84;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=66%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '7':
            {
                current_speed = 98;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=77%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '8':
            {
                current_speed = 112;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=88%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
            
            case '9':
            {
                current_speed = 127;
                emergency_stop = false;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                Set_Action("Speed=100%");
                Send_To_ESP32(MSG_SPEED, &current_speed, 1);
                break;
            }
                
            case 'A':
            {

                if (!emergency_stop && !obstacle_detected && !trip_active)
                {
                    int8_t spin_speed;
										if (current_speed == 0)
												spin_speed = 30;  // Default spin speed when no speed is set
										else
										{
												spin_speed = (int8_t)(current_speed / 2);
												if (spin_speed < 20) spin_speed = 20;  // Minimum spin speed
										}
                    
                    Send_Motor_To_ESP32(CMD_M1_BACKWARD, (uint8_t)spin_speed);
                    Send_Motor_To_ESP32(CMD_M2_FORWARD, (uint8_t)spin_speed);
                    last_m1_value = -spin_speed;
                    last_m2_value = spin_speed;
                    current_mode = MODE_SPIN_LEFT;
                    
                    spin_active = true;
                    spin_start_time = HAL_GetTick();
                    Set_Action("Spin Left");
                }
                else
                {
                    Set_Action("Spin blocked");
                }
                break;
            }
                
            case 'B':
            {
                // Pre-defined trip toggle
                if (trip_active)
                {
                    // Stop the trip if already running
                    Stop_Trip();
                    Set_Action("Trip Stopped");
                }
                else
                {
                    // Start the pre-defined trip
                    Start_Trip();
                }
                break;
            }
                
            case 'D':
            {
                UART_Debug("\r\n>>> MANUAL RECALIBRATION INITIATED\r\n");
								Force_Stop();  // Stop motors first
								Calibrate_Joystick();  // Run full calibration
								Set_Action("Recalibrated");
                break;
            }
                
            case '*':
            {
															if (!emergency_stop && !obstacle_detected && !trip_active)
								{
										int8_t spin_speed;
										if (current_speed == 0)
												spin_speed = 30;  // Default spin speed when no speed is set
										else
										{
												spin_speed = (int8_t)(current_speed / 2);
												if (spin_speed < 20) spin_speed = 20;  // Minimum spin speed
										}
										
										Send_Motor_To_ESP32(CMD_M1_FORWARD, (uint8_t)spin_speed);
										Send_Motor_To_ESP32(CMD_M2_BACKWARD, (uint8_t)spin_speed);
										last_m1_value = spin_speed;
										last_m2_value = -spin_speed;
										current_mode = MODE_SPIN_RIGHT;
										
										spin_active = true;
										spin_start_time = HAL_GetTick();
										Set_Action("Spin Right");
								}
								else
								{
										Set_Action("Spin blocked");
								}
								break;
							
            }
                
						case '#':
						{
							  precision_mode = !precision_mode;
                uint8_t status_flag = precision_mode ? 0x01 : 0x00;
                if (precision_mode)
                    Set_Action("Precision ON");
                else
                    Set_Action("Precision OFF");
                Send_To_ESP32(MSG_STATUS, &status_flag, 1);
                break;			
						}
            
            default:
                break;
        }
    }
    else if (key == 0)
    {
        // Key released - clear last_key after debounce period
        if ((now - last_key_time) > 300)
        {
            last_key = 0;
        }
    }
    
    // No blocking wait loop - return immediately
}

void Process_Trip(void)
{
    if (!trip_active)
        return;
    
    // Check for emergency conditions during trip
    if (emergency_stop)
    {
        UART_Debug("Trip aborted - Emergency stop!\r\n");
        Stop_Trip();
        return;
    }
    
    // Handle paused state - check if we can resume
    if (trip_paused)
    {
        if (!obstacle_detected)
        {
            // Obstacle cleared, resume trip
            trip_paused = false;
            trip_step_start_time = HAL_GetTick();  // Reset timer for current step
            Execute_Trip_Step(&predefined_trip[trip_current_step]);
            UART_Debug("Trip resumed - Path clear.\r\n");
        }
        return;  // Don't process steps while paused
    }
    
    // Check for obstacle during forward movement
    if (obstacle_detected)
    {
        const TripStep *current = &predefined_trip[trip_current_step];
        
        // If moving forward, pause the trip
        if (current->action == TRIP_STEP_FORWARD || 
            current->action == TRIP_STEP_LEFT || 
            current->action == TRIP_STEP_RIGHT)
        {
            // Stop motors but don't end trip
            Send_Motor_To_ESP32(CMD_M1_STOP, 0);
            Send_Motor_To_ESP32(CMD_M2_STOP, 0);
            last_m1_value = 0;
            last_m2_value = 0;
            
            trip_paused = true;
            Set_Action("Trip PAUSED");
            UART_Debug("Trip paused - Obstacle detected!\r\n");
            return;
        }
    }
    
    // Check if current step duration has elapsed
    uint32_t elapsed = HAL_GetTick() - trip_step_start_time;
    const TripStep *current = &predefined_trip[trip_current_step];
    
    if (elapsed >= current->duration_ms)
    {
        // Move to next step
        trip_current_step++;
        trip_step_start_time = HAL_GetTick();
        
        // Check bounds
        if (trip_current_step >= TRIP_MAX_STEPS)
        {
            Stop_Trip();
            return;
        }
        
        // Execute next step
        Execute_Trip_Step(&predefined_trip[trip_current_step]);
    }
}

void Process_Spin(void)
{
    if (spin_active)
    {
        uint32_t elapsed = HAL_GetTick() - spin_start_time;
        if (elapsed >= SPIN_DURATION_MS)
        {
            // Stop spinning
            Send_Motor_To_ESP32(CMD_M1_STOP, 0);
            Send_Motor_To_ESP32(CMD_M2_STOP, 0);
            last_m1_value = 0;
            last_m2_value = 0;
            current_mode = MODE_STOP;
            spin_active = false;
        }
    }
}

void Process_Safety(void)
{
    uint32_t now = HAL_GetTick();
    
    // Trigger ultrasonic periodically
    if (now - last_ultrasonic_time >= ULTRASONIC_INTERVAL)
    {
        Trigger_Ultrasonic();
        last_ultrasonic_time = now;
    }
    
    // Process distance reading when ready
    if (distance_ready)
    {
        distance_ready = false;
        
        // Send distance to ESP32
        Send_To_ESP32(MSG_DISTANCE, (uint8_t*)&Distance, 1);
        
        // Safety logic - check for valid reading (Distance > 0)
        if (Distance > 0 && Distance <= OBSTACLE_STOP_DIST)
        {
            // CRITICAL: 10cm or less - Emergency stop!
            bool moving_forward = (last_m1_value > 0 || last_m2_value > 0);
            
            if (moving_forward && !emergency_stop)
            {
                // Full emergency stop
                Force_Stop();
                sprintf(debug_msg, "EMERGENCY @%dcm!", Distance);
                Set_Action(debug_msg);
                UART_Debug("\r\n!!! CRITICAL DISTANCE - EMERGENCY STOP !!!\r\n");
            }
            
            obstacle_detected = true;
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        }
        else if (Distance > OBSTACLE_STOP_DIST && Distance <= OBSTACLE_WARN_DIST)
        {
            // WARNING: 11-20cm - Stop forward motion but allow recovery
            bool moving_forward = (last_m1_value > 0 || last_m2_value > 0);
            
            if (moving_forward && !emergency_stop)
            {
                // Stop motors but don't trigger full emergency stop
                // This allows user to back up or turn
                Send_Motor_To_ESP32(CMD_M1_STOP, 0);
                Send_Motor_To_ESP32(CMD_M2_STOP, 0);
                last_m1_value = 0;
                last_m2_value = 0;
                current_mode = MODE_STOP;
                
                // Also stop any active spin
                spin_active = false;
                
                sprintf(debug_msg, "WARNING @%dcm", Distance);
                Set_Action(debug_msg);
            }
            
            obstacle_detected = true;
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        }
        else if (Distance > OBSTACLE_WARN_DIST || Distance == 0)
        {
            // Clear path (Distance == 0 means no reading/out of range = assume clear)
            obstacle_detected = false;
            if (!emergency_stop)
            {
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            }
        }
    }
}

void Process_Joystick(void)
{
    // Don't process joystick during active trip
    if (trip_active)
        return;
    
    if (emergency_stop)
        return;
     
		if (spin_active)
        return;
		 
    uint16_t x = Read_ADC_X();
    uint16_t y = Read_ADC_Y();
    
    MotorCommand cmd = Get_Motor_Command(x, y, current_speed);
    
    // Track if joystick state changed
    static bool was_active = false;
    bool state_changed = (joystick_active != was_active);
    was_active = joystick_active;
    
    // **FIX: Move static variable OUTSIDE the if block**
    static uint32_t last_send_time = 0;
    uint32_t now = HAL_GetTick();
    
    // Calculate difference from last command
    int16_t m1_diff = cmd.m1_value - last_m1_value;
    int16_t m2_diff = cmd.m2_value - last_m2_value;
    
    if (m1_diff < 0) m1_diff = -m1_diff;
    if (m2_diff < 0) m2_diff = -m2_diff;
    
    // Decide if we should send commands
    bool should_send = false;
    
    if (joystick_active)
    {
        // Joystick is being used - always send commands when active!
        if (m1_diff > 3 || m2_diff > 3)
        {
            // Significant change in motor values
            should_send = true;
            last_send_time = now;
        }
        else if (state_changed)
        {
            // Just started using joystick
            should_send = true;
            last_send_time = now;
        }
        else if (now - last_send_time > 100)
        {
            // Periodic refresh every 100ms to keep motors running
            // Changed from 500ms to 100ms for more responsive control
            should_send = true;
            last_send_time = now;
        }
    }
    else
    {
        // Joystick is centered - stop motors if they're running
        if (last_m1_value != 0 || last_m2_value != 0)
        {
            should_send = true;
            last_send_time = now;
        }
    }
    
    if (should_send)
    {
        // Check safety: block forward movement if obstacle detected
        if (obstacle_detected)
        {
            // Only block if trying to move forward (positive motor values)
            if (cmd.m1_value > 0 || cmd.m2_value > 0)
            {
                // Allow backward and turning, block forward
                bool pure_forward = (cmd.m1_value > 0 && cmd.m2_value > 0);
                if (pure_forward)
                {
                    cmd.m1_value = 0;
                    cmd.m2_value = 0;
                    cmd.mode = MODE_STOP;
                    sprintf(cmd.direction, "BLOCKED");
                    sprintf(cmd.mode_desc, "Obstacle!");
                    Set_Action("FWD BLOCKED");
                }
            }
        }
        
        Execute_Motor_Command(cmd);
    }
}

// Also uncomment M1 in Execute_Motor_Command:
void Execute_Motor_Command(MotorCommand cmd)
{
    // UNCOMMENT THIS BLOCK FOR M1:
    if (cmd.m1_value > 0)
        Send_Motor_To_ESP32(CMD_M1_FORWARD, (uint8_t)cmd.m1_value);
    else if (cmd.m1_value < 0)
        Send_Motor_To_ESP32(CMD_M1_BACKWARD, (uint8_t)(-cmd.m1_value));
    else
        Send_Motor_To_ESP32(CMD_M1_STOP, 0);
    
    // M2 commands
    if (cmd.m2_value > 0)
        Send_Motor_To_ESP32(CMD_M2_FORWARD, (uint8_t)cmd.m2_value);
    else if (cmd.m2_value < 0)
        Send_Motor_To_ESP32(CMD_M2_BACKWARD, (uint8_t)(-cmd.m2_value));
    else
        Send_Motor_To_ESP32(CMD_M2_STOP, 0);
    
    current_mode = cmd.mode;
    last_m1_value = cmd.m1_value;
    last_m2_value = cmd.m2_value;
}


void Print_Status(void)
{
    uint32_t now = HAL_GetTick();
    
    if (now - last_status_print_time >= STATUS_PRINT_INTERVAL)
    {
        last_status_print_time = now;
        
        // Send data to ESP32
        Send_To_ESP32(MSG_MODE, &current_mode, 1);
        uint8_t motor_data[2];
        motor_data[0] = (uint8_t)((last_m1_value >= 0) ? last_m1_value : -last_m1_value);
        motor_data[1] = (uint8_t)((last_m2_value >= 0) ? last_m2_value : -last_m2_value);
        if (last_m1_value < 0) motor_data[0] |= 0x80;
        if (last_m2_value < 0) motor_data[1] |= 0x80;
        Send_To_ESP32(MSG_MOTOR, motor_data, 2);
        
        // Build status components
        char status_line[200];
        char dir_info[40] = "";
        char action_info[35] = "";
        char trip_info[20] = "";
        char spin_info[15] = "";
        char prec_str[6] = "";
        char dist_str[25];
        
        // Distance string with zone indicator
        if (Distance == 0)
        {
            sprintf(dist_str, "D=---");  // No valid reading
        }
        else if (Distance <= OBSTACLE_STOP_DIST)
        {
            sprintf(dist_str, "D=%dcm [DANGER!]", Distance);
        }
        else if (Distance <= OBSTACLE_WARN_DIST)
        {
            sprintf(dist_str, "D=%dcm [WARN]", Distance);
        }
        else
        {
            sprintf(dist_str, "D=%dcm", Distance);
        }
        
        // Precision mode indicator
        if (precision_mode)
            sprintf(prec_str, "[P] ");
        
        // Trip status
        if (trip_active)
        {
            if (trip_paused)
                sprintf(trip_info, "[TRIP PAUSED] ");
            else
                sprintf(trip_info, "[TRIP %d] ", trip_current_step + 1);
        }
        
        // Spin status
        if (spin_active)
            sprintf(spin_info, "[SPIN] ");
        
        // Direction info
        if (emergency_stop)
        {
            sprintf(dir_info, "STOPPED");
        }
        else if (spin_active)
        {
            if (last_m1_value < 0 && last_m2_value > 0)
                sprintf(dir_info, "Spinning LEFT");
            else if (last_m1_value > 0 && last_m2_value < 0)
                sprintf(dir_info, "Spinning RIGHT");
            else
                sprintf(dir_info, "Spin M1:%d M2:%d", (int)last_m1_value, (int)last_m2_value);
        }
        else if (trip_active && !trip_paused)
        {
            sprintf(dir_info, "AUTO M1:%d M2:%d", (int)last_m1_value, (int)last_m2_value);
        }
        else if (joystick_active && current_joy_magnitude > 0)
        {
            char compass[4];
            Get_Compass_Direction(current_joy_angle, compass);
            sprintf(dir_info, "%d deg (%s) %d%%", current_joy_angle, compass, current_joy_magnitude);
        }
        else if (last_m1_value != 0 || last_m2_value != 0)
        {
            sprintf(dir_info, "M1:%d M2:%d", (int)last_m1_value, (int)last_m2_value);
        }
        else
        {
            sprintf(dir_info, "Idle");
        }
        
        // Recent action
        if (strlen(last_action) > 0 && (now - last_action_time) < ACTION_DISPLAY_TIME)
        {
            sprintf(action_info, " | %s", last_action);
        }
        
        // Build final status line
        int speed_pct = (current_speed * 100) / 127;
        
        if (emergency_stop)
        {
            sprintf(status_line, "Status: %s | %s%s%sEMERGENCY STOP | %s%s\r\n",
                    dist_str, prec_str, trip_info, spin_info, dir_info, action_info);
        }
        else if (obstacle_detected)
        {
            sprintf(status_line, "Status: %s | %s%s%sSpd=%d%% | %s%s\r\n",
                    dist_str, prec_str, trip_info, spin_info, speed_pct, dir_info, action_info);
        }
        else
        {
            sprintf(status_line, "Status: %s | %s%s%sSpd=%d%% | %s%s\r\n",
                    dist_str, prec_str, trip_info, spin_info, speed_pct, dir_info, action_info);
        }
        
        UART_Debug(status_line);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  
  last_ultrasonic_time = HAL_GetTick();
  last_status_print_time = HAL_GetTick();
  
  UART_Debug("\r\n");
  UART_Debug("================================================\r\n");
  UART_Debug("    SmartDrive Assist - Motion Control System\r\n");
  UART_Debug("    STM32L432KC + ESP32 Dual-MCU Platform\r\n");
  UART_Debug("================================================\r\n\r\n");
  
  // Perform joystick calibration
  if (!Load_Calibration_From_Flash())
  {
      // No valid calibration found - run calibration procedure
      UART_Debug(">>> Starting NEW calibration sequence...\r\n");
      Calibrate_Joystick();
  }
  else
  {
      UART_Debug(">>> Using saved calibration values.\r\n");
      UART_Debug(">>> Press 'D' to recalibrate if needed.\r\n\r\n");
  }
	
  UART_Debug("------------------------------------------------\r\n");
  UART_Debug("Controls:\r\n");
  UART_Debug("  [1-9] Set speed (1=11%%, 9=100%%)\r\n");
  UART_Debug("  [0/C] Emergency Stop\r\n");
  UART_Debug("  [A]   Spin Right\r\n");
  UART_Debug("  [B]   Start/Stop Pre-defined Trip\r\n");
  UART_Debug("  [D]   Recalibrate Joystick\r\n");
  UART_Debug("  [*]   Spin Left    [#] Toggle Precision Mode (50%% speed)\r\n");
  UART_Debug("  Joystick: 360-degree directional control\r\n");
  UART_Debug("------------------------------------------------\r\n\r\n");
	
  HAL_Delay(100);
  Send_To_ESP32(MSG_SPEED, &current_speed, 1);
  Send_To_ESP32(MSG_MODE, &current_mode, 1);
  
  // Ready indication
  for (int i = 0; i < 3; i++)
  {
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
  }
  
  UART_Debug("System Ready! Status updates every 0.5 seconds.\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  
    Process_Safety();
		Process_Keypad();
    Process_Trip();
		Process_Spin();
    Process_Joystick();
    Print_Status();
    
    HAL_Delay(CONTROL_LOOP_DELAY);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|ROW1_Pin|ROW2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|ROW3_Pin|ROW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL4_Pin COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL4_Pin|COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin ROW1_Pin ROW2_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|ROW1_Pin|ROW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin ROW3_Pin ROW4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|ROW3_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
		      // Blink LED rapidly to indicate error
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      for (volatile int i = 0; i < 100000; i++);
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
