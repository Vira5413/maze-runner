/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  _MOTOR_R = 0,
  _MOTOR_L,
  _TOTAL_MOTOR,
} motor_e;

typedef enum {
  _DIR_FORWARD = 0,
  _DIR_REVERSE,
  _DIR_COAST,
  _DIR_BRAKE,
} motor_dir_e;

typedef enum { _TURN_LEFT = 0, _TURN_RIGHT = 1 } turn_dir_e;

typedef enum { _US_RIGHT = 0, _US_LEFT, _US_FRONT, _TOTAL_US } ultrasonic_e;

typedef enum { _ENC_RIGHT = 0, _ENC_LEFT, _TOTAL_ENC } encoder_e;

// The events returned by the driving scanner
typedef enum {
  _MAZE_EVENT_RIGHT_OPEN = 0,
  _MAZE_EVENT_LEFT_OPEN,
  _MAZE_EVENT_FRONT_BLOCKED,
  _MAZE_EVENT_SOLVED,
  _MAZE_EVENT_ERROR
} maze_event_e;

// Available routing algorithms
typedef enum {
  _ALGO_RIGHT_WALL = 0,
  _ALGO_LEFT_WALL,
  _ALGO_FLOOD_FILL // Placeholder for future advanced algorithm
} maze_algo_e;

// Hardware mapping struct for a DC motor
typedef struct {
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  TIM_HandleTypeDef *htim;
  uint32_t channel;
} motor_hw_t;

// Hardware mapping struct for an HC-SR04 ultrasonic sensor
typedef struct {
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
} ultrasonic_hw_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CRASH_SAFETY_CM 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Map hardware pins exactly as wired in your original code
static const motor_hw_t motors[_TOTAL_MOTOR] = {
    [_MOTOR_R] = {GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_0, &htim3, TIM_CHANNEL_2},
    [_MOTOR_L] = {GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_10, &htim3,
                  TIM_CHANNEL_1}};

static const ultrasonic_hw_t ultrasonics[_TOTAL_US] = {
    [_US_RIGHT] = {GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4},
    [_US_LEFT] = {GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6},
    [_US_FRONT] = {GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8}};

// Centralized encoder arrays
volatile uint32_t encoder_counts[_TOTAL_ENC] = {0, 0};
volatile uint32_t encoder_totals[_TOTAL_ENC] = {0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // Left encoder is PA4, Right is PA5 based on original EXTI layout
  if (GPIO_Pin == GPIO_PIN_4) {
    encoder_counts[_ENC_LEFT]++;
    encoder_totals[_ENC_LEFT]++;
  } else if (GPIO_Pin == GPIO_PIN_5) {
    encoder_counts[_ENC_RIGHT]++;
    encoder_totals[_ENC_RIGHT]++;
  }
}

static void delay_us(uint32_t us);
static void init_DWT(void);

void init_motor(void);
void motor_set_dir(motor_e motor, motor_dir_e dir);
void motor_set_speed(motor_e motor, int32_t speed);

uint32_t encoder_read_and_reset(encoder_e enc);
uint32_t encoder_get_total(encoder_e enc);

uint32_t ultrasonic_read(ultrasonic_e sensor);

void drive_dist(uint32_t target_pulses, motor_dir_e dir);
void turn_in_place(float target_angle_deg, turn_dir_e dir);

void drive_until_wall_reengaged(turn_dir_e pref_dir);
maze_event_e drive_to_intersection(void);
void algo_right_wall_step(void);
void algo_left_wall_step(void);
void maze_solve_step(maze_algo_e active_algo);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Turn on status LED to indicate power/init phase
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // Initialize cycle counter for microsecond delays
  init_DWT();

  // Initialize MPU6050 if required (from original code)
  MPU6050_Init();

  // Initialize abstracted motor layer
  init_motor();

  // 2-second safety delay before any autonomous movement begins
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    // Heartbeat LED to indicate the main loop is running and blocking functions
    // are complete
    // Execute one step of the selected algorithm
    maze_solve_step(_ALGO_RIGHT_WALL);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 |
                        GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB3 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 |
                        GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void delay_us(uint32_t us) {
  uint32_t startCycles = DWT->CYCCNT;
  uint32_t delayCycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - startCycles) < delayCycles)
    ;
}

static void init_DWT(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// MOTOR DRIVERS
void init_motor(void) {
  // Enable motor driver standby pin (STBY = HIGH)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  // Start PWM channels
  for (int i = 0; i < _TOTAL_MOTOR; i++) {
    HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
    __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0);
  }
}
void motor_set_dir(motor_e motor, motor_dir_e dir) {
  if (motor >= _TOTAL_MOTOR)
    return;

  const motor_hw_t *hw = &motors[motor];

  switch (dir) {
  case _DIR_FORWARD:
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
    break;
  case _DIR_REVERSE:
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
    break;
  case _DIR_BRAKE: // Both HIGH to short motor terminals
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
    break;
  case _DIR_COAST: // Both LOW to freewheel
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
    break;
  }
}
/**
 * @brief Sets the motor PWM speed, safely clamping out-of-bounds PID values.
 * @param motor The motor to control.
 * @param speed Signed speed value (safely clamped to 0-999).
 */
void motor_set_speed(motor_e motor, int32_t speed) {
  if (motor >= _TOTAL_MOTOR)
    return;

  // Centralized clamping
  if (speed < 0)
    speed = 0;
  if (speed > 999)
    speed = 999;

  __HAL_TIM_SET_COMPARE(motors[motor].htim, motors[motor].channel,
                        (uint32_t)speed);
}

// ENCODER DRIVERS

// Safely read and clear encoder delta (interrupt safe)
uint32_t encoder_read_and_reset(encoder_e enc) {
  if (enc >= _TOTAL_ENC)
    return 0;

  __disable_irq();
  uint32_t count = encoder_counts[enc];
  encoder_counts[enc] = 0;
  __enable_irq();

  return count;
}

uint32_t encoder_get_total(encoder_e enc) {
  if (enc >= _TOTAL_ENC)
    return 0;

  __disable_irq();
  uint32_t total = encoder_totals[enc];
  __enable_irq();

  return total;
}

// ULTRASONIC DRIVERS

uint32_t ultrasonic_read(ultrasonic_e sensor) {
  if (sensor >= _TOTAL_US)
    return 999;

  const ultrasonic_hw_t *hw = &ultrasonics[sensor];

  // Trigger pulse
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

  uint32_t waitStart = DWT->CYCCNT;

  // Wait for echo to go HIGH
  while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_RESET) {
    if ((DWT->CYCCNT - waitStart) > (SystemCoreClock / 1000) * 30)
      return 999;
  }

  uint32_t startCycles = DWT->CYCCNT;
  uint8_t timedOut = 0;

  // Wait for echo to go LOW
  while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_SET) {
    if ((DWT->CYCCNT - startCycles) > (SystemCoreClock / 1000) * 30) {
      timedOut = 1;
      break;
    }
  }

  if (timedOut)
    return 999;

  uint32_t endCycles = DWT->CYCCNT;
  uint32_t pulseMicros =
      (endCycles - startCycles) / (SystemCoreClock / 1000000);

  return pulseMicros / 58;
}

// GENRIC

/**
 * @brief Drives the car forward for a specified number of encoder pulses.
 *        Stops immediately if an obstacle is detected within CRASH_SAFETY_CM.
 * @param target_pulses The distance to travel, measured in encoder ticks.
 * @param dir Direction of travel (_DIR_FORWARD or _DIR_REVERSE).
 */
void drive_dist(uint32_t target_pulses, motor_dir_e dir) {
  // Record starting encoder totals
  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  // Set both motors to desired direction
  motor_set_dir(_MOTOR_L, dir);
  motor_set_dir(_MOTOR_R, dir);

  const uint32_t BASE_SPEED = 400;
  const float Kp = 30.0f; // Proportional gain for straight-line correction

  // Software Watchdog Initialization
  uint32_t watchdog_start = HAL_GetTick();
  // Dynamic timeout: Give it 5ms per pulse, plus a 1.5-second base allowance
  uint32_t watchdog_timeout_ms = (target_pulses * 5) + 1500;

  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0; // 0: Front, 1: Left, 2: Right

  while (1) {
    uint32_t now = HAL_GetTick();

    // Software Watchdog Check
    if (now - watchdog_start > watchdog_timeout_ms) {
      // Trap the system if it takes too long to reach the  target
      Error_Handler();
    }

    //  Crash Safety Polling (Round-robin to prevent PID blocking)
    // Only engage safety checks if driving forward
    if (dir == _DIR_FORWARD && (now - last_us_check >= 10)) {
      last_us_check = now;
      uint32_t dist = 999;

      if (us_poll_target == 0) {
        dist = ultrasonic_read(_US_FRONT);
      } else if (us_poll_target == 1) {
        dist = ultrasonic_read(_US_LEFT);
      } else {
        dist = ultrasonic_read(_US_RIGHT);
      }

      // Emergency stop condition
      if (dist <= CRASH_SAFETY_CM) {
        break;
      }

      us_poll_target = (us_poll_target + 1) % 3;
    }

    //  Encoder PID Control
    uint32_t current_left = encoder_get_total(_ENC_LEFT) - start_left;
    uint32_t current_right = encoder_get_total(_ENC_RIGHT) - start_right;

    // Target reached condition
    if (current_left >= target_pulses && current_right >= target_pulses) {
      break;
    }

    // Calculate cross-track error to keep the car straight
    int32_t error = (int32_t)current_left - (int32_t)current_right;
    int32_t correction = (int32_t)(Kp * (float)error);

    int32_t left_speed = BASE_SPEED - correction;
    int32_t right_speed = BASE_SPEED + correction;

    // If one wheel finishes early, stop it while the other catches up
    if (current_left >= target_pulses)
      left_speed = 0;
    if (current_right >= target_pulses)
      right_speed = 0;

    motor_set_speed(_MOTOR_L, left_speed);
    motor_set_speed(_MOTOR_R, right_speed);

    // 1ms control loop delay
    HAL_Delay(1);
  }

  // Active braking to stop momentum instantly
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);

  // Hold brake briefly, then release to coast so motors aren't held under load
  HAL_Delay(100);
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);
}

/**
 * @brief Rotates the car in place by a specified angle using gyro integration.
 * @param target_angle_deg The angle to turn in degrees.
 * @param dir The direction to turn (TURN_LEFT or TURN_RIGHT).
 */
void turn_in_place(float target_angle_deg, turn_dir_e dir) {

  // Chassis Clearance Nudge
  // Advance half the body length (8cm) to prevent bumper clipping during
  // rotation. 8 cm / 1.021 cm/pulse = ~8 pulses.
  drive_dist(8, _DIR_FORWARD);

  // Pause briefly to let the chassis settle and prevent gyro motion artifacts
  HAL_Delay(500);

  float turn_sum = 0.0f;
  uint32_t last_gyro_check = HAL_GetTick();
  uint32_t last_control_time = HAL_GetTick();

  // Reset local encoder tracking to synchronize wheel speeds during the turn
  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  // Dynamic speed parameters
  const uint32_t MIN_SPEED = 350; // Minimum PWM to overcome motor stall torque
  const uint32_t MAX_SPEED = 500;
  const float Kp_angle = 5.0f; // Proportional gain for deceleration
  const float Kp_enc = 30.0f;  // Proportional gain for wheel speed matching

  // Set opposite wheel directions for a zero-radius turn
  if (dir == _TURN_RIGHT) {
    motor_set_dir(_MOTOR_L, _DIR_FORWARD);
    motor_set_dir(_MOTOR_R, _DIR_REVERSE);
  } else {
    motor_set_dir(_MOTOR_L, _DIR_REVERSE);
    motor_set_dir(_MOTOR_R, _DIR_FORWARD);
  }

  // A small offset (e.g., 5.0 degrees) can be subtracted from target_angle_deg
  // here if your physical chassis has high inertia and overshoots despite
  // braking.
  const float c_offset = 17.0f;
  float target_ang = target_angle_deg - c_offset;
  while (fabsf(turn_sum) < target_ang) {
    uint32_t now = HAL_GetTick();

    //  Gyro Integration (Read every ~2ms)
    if (now - last_gyro_check >= 2) {
      float dt = (now - last_gyro_check) * 0.001f; // Convert ms to seconds
      last_gyro_check = now;

      // Integrate Z-axis degrees-per-second into total degrees
      turn_sum += MPU6050_ReadGyroZ_DPS() * dt;
    }

    // Motor Control Loop (Update every 1ms)
    if (now - last_control_time >= 1) {
      last_control_time = now;

      // Calculate base speed proportionally based on remaining angle
      float angle_remaining = target_angle_deg - fabsf(turn_sum);
      uint32_t base_speed = (uint32_t)(angle_remaining * Kp_angle);

      if (base_speed > MAX_SPEED)
        base_speed = MAX_SPEED;
      if (base_speed < MIN_SPEED)
        base_speed = MIN_SPEED;

      // Encoder synchronization to prevent the pivot point from drifting
      uint32_t current_left = encoder_get_total(_ENC_LEFT) - start_left;
      uint32_t current_right = encoder_get_total(_ENC_RIGHT) - start_right;

      int32_t enc_error = (int32_t)current_left - (int32_t)current_right;
      int32_t correction = (int32_t)(Kp_enc * (float)enc_error);

      int32_t left_speed = (int32_t)base_speed - correction;
      int32_t right_speed = (int32_t)base_speed + correction;

      motor_set_speed(_MOTOR_L, left_speed);
      motor_set_speed(_MOTOR_R, right_speed);
    }
  }

  // Active Braking: short the motor terminals to kill momentum instantly
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);

  // Hold brake briefly to bleed momentum
  HAL_Delay(100);

  // Release to coast so motors aren't continually driven against physical
  // resistance
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);
}

void turn_in_place_2(float target_angle_deg, turn_dir_e dir) {

  // Chassis Clearance Nudge
  // Advance half the body length (8cm) to prevent bumper clipping during
  // rotation. 8 cm / 1.021 cm/pulse = ~8 pulses.
  drive_dist(16, _DIR_FORWARD);

  // Let the chassis settle before reading the gyro
  HAL_Delay(250);

  float turn_sum = 0.0f;
  uint32_t last_gyro_check = HAL_GetTick();
  uint32_t last_control_time = HAL_GetTick();

  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  const uint32_t MIN_SPEED = 350;
  const uint32_t MAX_SPEED = 500;
  const float Kp_angle = 5.0f;
  const float Kp_enc = 30.0f;

  if (dir == _TURN_RIGHT) {
    motor_set_dir(_MOTOR_L, _DIR_FORWARD);
    motor_set_dir(_MOTOR_R, _DIR_REVERSE);
  } else {
    motor_set_dir(_MOTOR_L, _DIR_REVERSE);
    motor_set_dir(_MOTOR_R, _DIR_FORWARD);
  }

  // Reduced offset: Rely on active braking rather than pre-emptive cutoff
  const float c_offset = 17.0f;
  float target_ang = target_angle_deg - c_offset;

  while (fabsf(turn_sum) < target_ang) {
    uint32_t now = HAL_GetTick();

    if (now - last_gyro_check >= 2) {
      float dt = (now - last_gyro_check) * 0.001f;
      last_gyro_check = now;
      turn_sum += MPU6050_ReadGyroZ_DPS() * dt;
    }

    if (now - last_control_time >= 1) {
      last_control_time = now;

      float angle_remaining = target_angle_deg - fabsf(turn_sum);
      uint32_t base_speed = (uint32_t)(angle_remaining * Kp_angle);

      if (base_speed > MAX_SPEED)
        base_speed = MAX_SPEED;
      if (base_speed < MIN_SPEED)
        base_speed = MIN_SPEED;

      uint32_t current_left = encoder_get_total(_ENC_LEFT) - start_left;
      uint32_t current_right = encoder_get_total(_ENC_RIGHT) - start_right;

      int32_t enc_error = (int32_t)current_left - (int32_t)current_right;
      int32_t correction = (int32_t)(Kp_enc * (float)enc_error);

      motor_set_speed(_MOTOR_L, (int32_t)base_speed - correction);
      motor_set_speed(_MOTOR_R, (int32_t)base_speed + correction);
    }
  }

  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);

  HAL_Delay(100);
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);
}



/**
 * @brief Drives straight until the preferred wall is detected again or front is
 * blocked.
 * @param pref_dir The wall we are trying to re-engage (_TURN_RIGHT or
 * _TURN_LEFT).
 */
void drive_until_wall_reengaged(turn_dir_e pref_dir) {
  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  motor_set_dir(_MOTOR_L, _DIR_FORWARD);
  motor_set_dir(_MOTOR_R, _DIR_FORWARD);

  const uint32_t BASE_SPEED = 400;
  const float Kp_enc = 30.0f;
  const uint32_t REENGAGE_THRESHOLD_CM =
      10; // Closer than this means we found the wall again
  const uint32_t STOP_DISTANCE_CM = 6;

  ultrasonic_e pref_sensor = (pref_dir == _TURN_RIGHT) ? _US_RIGHT : _US_LEFT;

  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0; // 0 = Pref Sensor, 1 = Front Sensor

  while (1) {
    uint32_t now = HAL_GetTick();

    // 1. Environmental Polling (Toggle between front and preferred side only)
    if (now - last_us_check >= 10) {
      last_us_check = now;

      if (us_poll_target == 0) {
        if (ultrasonic_read(pref_sensor) < REENGAGE_THRESHOLD_CM) {
          break; // Wall successfully found!
        }
      } else {
        if (ultrasonic_read(_US_FRONT) <= STOP_DISTANCE_CM) {
          break; // Hit a T-junction or dead end before wall returned
        }
      }
      us_poll_target = (us_poll_target + 1) % 2;
    }

    // 2. Encoder Straight-Line PID
    uint32_t current_left = encoder_get_total(_ENC_LEFT);
    uint32_t current_right = encoder_get_total(_ENC_RIGHT);

    int32_t error = (int32_t)(current_left - start_left) -
                    (int32_t)(current_right - start_right);
    int32_t correction = (int32_t)(Kp_enc * (float)error);

    motor_set_speed(_MOTOR_L, BASE_SPEED - correction);
    motor_set_speed(_MOTOR_R, BASE_SPEED + correction);

    HAL_Delay(1);
  }

  // Active Braking
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);
  HAL_Delay(100);
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);
}

/**
 * @brief Drives forward until an opening or a wall is detected.
 *        Traps system if motors stall for > 500ms.
 * @return The detected maze event.
 */
maze_event_e drive_to_intersection(void) {
  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  // Watchdog variables
  uint32_t last_left_count = start_left;
  uint32_t last_right_count = start_right;
  uint32_t last_movement_time = HAL_GetTick();
  const uint32_t STALL_TIMEOUT_MS = 500;

  motor_set_dir(_MOTOR_L, _DIR_FORWARD);
  motor_set_dir(_MOTOR_R, _DIR_FORWARD);

  const uint32_t BASE_SPEED = 400;
  const float Kp = 30.0f;
  const uint32_t WALL_DETECT_CM = 25;
  const uint32_t STOP_DISTANCE_CM = 5;

  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0;
  maze_event_e detected_event = _MAZE_EVENT_ERROR;

  while (1) {
    uint32_t now = HAL_GetTick();

    uint32_t current_left = encoder_get_total(_ENC_LEFT);
    uint32_t current_right = encoder_get_total(_ENC_RIGHT);

    //  Stall-Detecting Software Watchdog
    if (current_left != last_left_count || current_right != last_right_count) {
      last_left_count = current_left;
      last_right_count = current_right;
      last_movement_time = now; // Reset timer as long as wheels are moving
    }

    if (now - last_movement_time > STALL_TIMEOUT_MS) {
      Error_Handler(); // Trap if wheels stop turning for 500ms
    }

    //  Environmental Polling (Round-robin)
    if (now - last_us_check >= 5) {
      last_us_check = now;

      if (us_poll_target == 0) {
        if (ultrasonic_read(_US_FRONT) <= STOP_DISTANCE_CM) {
          detected_event = _MAZE_EVENT_FRONT_BLOCKED;
          break;
        }
      } else if (us_poll_target == 1) {
        if (ultrasonic_read(_US_RIGHT) > WALL_DETECT_CM) {
          detected_event = _MAZE_EVENT_RIGHT_OPEN;
          break;
        }
      } else {
        if (ultrasonic_read(_US_LEFT) > WALL_DETECT_CM) {
          detected_event = _MAZE_EVENT_LEFT_OPEN;
          break;
        }
      }

      us_poll_target = (us_poll_target + 1) % 3;
    }

    //  PID Straight-Line Control
    int32_t error = (int32_t)(current_left - start_left) -
                    (int32_t)(current_right - start_right);
    int32_t correction = (int32_t)(Kp * (float)error);

    motor_set_speed(_MOTOR_L, BASE_SPEED - correction);
    motor_set_speed(_MOTOR_R, BASE_SPEED + correction);

    HAL_Delay(1);
  }

  // Halt motors instantly upon event detection
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);
  HAL_Delay(100);
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);

  // Final solved verification (delay to let acoustics clear)
  HAL_Delay(50);
  if (ultrasonic_read(_US_FRONT) >= 40 && ultrasonic_read(_US_RIGHT) >= 40 &&
      ultrasonic_read(_US_LEFT) >= 40) {
    return _MAZE_EVENT_SOLVED;
  }

  return detected_event;
}

/**
 * @brief Drives forward using dynamic ultrasonic PID centering.
 *        Adapts to varying corridor widths automatically.
 */
maze_event_e drive_to_intersection_2(void) {
  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  uint32_t last_left_count = start_left;
  uint32_t last_right_count = start_right;
  uint32_t last_movement_time = HAL_GetTick();
  const uint32_t STALL_TIMEOUT_MS = 500;

  motor_set_dir(_MOTOR_L, _DIR_FORWARD);
  motor_set_dir(_MOTOR_R, _DIR_FORWARD);

  const uint32_t BASE_SPEED = 400;
  const float Kp_enc = 30.0f;
  const float Kp_center = 20.0f;

  const uint32_t OPENING_THRESHOLD_CM = 35;

  // RESTORED: 5cm puts the 8cm axle perfectly at the 13cm center mark
  const uint32_t STOP_DISTANCE_CM = 5;

  static int32_t dynamic_ideal_dist = 10;

  uint32_t latest_front_us = 999;
  uint32_t latest_right_us = 999;
  uint32_t latest_left_us = 999;

  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0;
  maze_event_e detected_event = _MAZE_EVENT_ERROR;

  while (1) {
    uint32_t now = HAL_GetTick();

    uint32_t current_left = encoder_get_total(_ENC_LEFT);
    uint32_t current_right = encoder_get_total(_ENC_RIGHT);

    if (current_left != last_left_count || current_right != last_right_count) {
      last_left_count = current_left;
      last_right_count = current_right;
      last_movement_time = now;
    }
    if (now - last_movement_time > STALL_TIMEOUT_MS)
      Error_Handler();

    if (now - last_us_check >= 10) {
      last_us_check = now;

      if (us_poll_target == 0) {
        latest_front_us = ultrasonic_read(_US_FRONT);
        if (latest_front_us <= STOP_DISTANCE_CM) {
          detected_event = _MAZE_EVENT_FRONT_BLOCKED;
          break;
        }
      } else if (us_poll_target == 1) {
        latest_right_us = ultrasonic_read(_US_RIGHT);
        if (latest_right_us > OPENING_THRESHOLD_CM &&
            (current_left - start_left) > 5) {
          detected_event = _MAZE_EVENT_RIGHT_OPEN;
          break;
        }
      } else {
        latest_left_us = ultrasonic_read(_US_LEFT);
        if (latest_left_us > OPENING_THRESHOLD_CM &&
            (current_left - start_left) > 5) {
          detected_event = _MAZE_EVENT_LEFT_OPEN;
          break;
        }
      }
      us_poll_target = (us_poll_target + 1) % 3;
    }

    int32_t correction = 0;

    if (latest_left_us < OPENING_THRESHOLD_CM &&
        latest_right_us < OPENING_THRESHOLD_CM) {
      dynamic_ideal_dist = (latest_left_us + latest_right_us) / 2;
      int32_t center_error = (int32_t)latest_left_us - (int32_t)latest_right_us;
      correction = (int32_t)(Kp_center * (float)center_error);
    } else if (latest_left_us < OPENING_THRESHOLD_CM) {
      int32_t center_error = (int32_t)latest_left_us - dynamic_ideal_dist;
      correction = (int32_t)(Kp_center * (float)center_error);
    } else if (latest_right_us < OPENING_THRESHOLD_CM) {
      int32_t center_error = dynamic_ideal_dist - (int32_t)latest_right_us;
      correction = (int32_t)(Kp_center * (float)center_error);
    } else {
      int32_t enc_error = (int32_t)(current_left - start_left) -
                          (int32_t)(current_right - start_right);
      correction = (int32_t)(Kp_enc * (float)enc_error);
    }

    motor_set_speed(_MOTOR_L, BASE_SPEED - correction);
    motor_set_speed(_MOTOR_R, BASE_SPEED + correction);
    HAL_Delay(1);
  }

  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);
  HAL_Delay(100);
  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);

  HAL_Delay(50);
  if (ultrasonic_read(_US_FRONT) >= 50 && ultrasonic_read(_US_RIGHT) >= 50 &&
      ultrasonic_read(_US_LEFT) >= 50) {
    return _MAZE_EVENT_SOLVED;
  }

  return detected_event;
}

/**
 * @brief Universal wall follower step.
 * @param pref_dir The wall to track (_TURN_RIGHT or _TURN_LEFT).
 */
void algo_wall_follower_step(turn_dir_e pref_dir) {
  // CORRECTED: Drives axle 20cm forward to reach the dead center of the
  // intersection
  const uint32_t PIVOT_OFFSET_PULSES = 20;

  // Increased to 20cm to ensure the car doesn't falsely detect the corner
  // it is currently rotating around as a "wall" blocking its path.
  const uint32_t WALL_DETECT_CM = 20;

  turn_dir_e opp_dir = (pref_dir == _TURN_RIGHT) ? _TURN_LEFT : _TURN_RIGHT;
  ultrasonic_e pref_sensor = (pref_dir == _TURN_RIGHT) ? _US_RIGHT : _US_LEFT;
  ultrasonic_e opp_sensor = (pref_dir == _TURN_RIGHT) ? _US_LEFT : _US_RIGHT;

  maze_event_e event = drive_to_intersection();

  if (event == _MAZE_EVENT_SOLVED) {
    while (1) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(100);
    }
  }

  if ((pref_sensor == _US_RIGHT && event == _MAZE_EVENT_RIGHT_OPEN) ||
      (pref_sensor == _US_LEFT && event == _MAZE_EVENT_LEFT_OPEN)) {

    drive_dist(PIVOT_OFFSET_PULSES, _DIR_FORWARD);
    turn_in_place(90.0f, pref_dir);
    drive_until_wall_reengaged(pref_dir);
  } else if (event == _MAZE_EVENT_FRONT_BLOCKED) {

    if (ultrasonic_read(pref_sensor) > WALL_DETECT_CM) {
      turn_in_place(90.0f, pref_dir);
      drive_until_wall_reengaged(pref_dir);
    } else if (ultrasonic_read(opp_sensor) > WALL_DETECT_CM) {
      turn_in_place(90.0f, opp_dir);
      drive_until_wall_reengaged(opp_dir);
    } else {
      turn_in_place(180.0f, pref_dir);
    }
  }
}

/**
 * @brief Top-level router for the active maze-solving strategy.
 * @param active_algo The selected routing algorithm.
 */
void maze_solve_step(maze_algo_e active_algo) {
  switch (active_algo) {
  case _ALGO_RIGHT_WALL:
    algo_wall_follower_step(_TURN_RIGHT);
    break;
  case _ALGO_LEFT_WALL:
    algo_wall_follower_step(_TURN_LEFT);
    break;
  case _ALGO_FLOOD_FILL:
    // future_flood_fill_step();
    break;
  default:
    Error_Handler();
    break;
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
  //  Immediately drop motors into active brake
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);
  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);
  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // Crude blocking delay (~100ms depending on your specific clock frequency)
    // Adjust the loop ceiling if it blinks too fast or too slow on your
    // specific board
    for (volatile uint32_t i = 0; i < 1500000; i++) {
      __NOP();
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
