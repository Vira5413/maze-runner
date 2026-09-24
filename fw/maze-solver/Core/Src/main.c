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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_gcc.h"
#include "mpu6050.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>

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

#define ULT_SNC_READ_MS 2 // Tultrasonicpoll​

#define CONTROL_PERIOD_MS 4
// @dev: control loop does not need to run faster than new sensor information is
// available. in gneral Tultrasonicpoll​ < Tcontrol ≪ Tvehicle dynamics our
// case:   ​ Tcontrol >= 2*Tultrasonicpoll Threshold indicating an open path
#define OPENING_THRESHOLD_CM 15

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

// Ideal corridor half-width (cm), updated dynamically whenever both side
// walls are in range. Used as fallback target when only one wall is seen.
static int32_t corridor_target_cm = 12;

static uint32_t s_base_speed = 400;

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

static int32_t compute_centering_correction(uint32_t left_us, uint32_t right_us,
                                            uint32_t enc_left_delta,
                                            uint32_t enc_right_delta,
                                            uint32_t now_ms);
void drive_guided(uint32_t target_pulses, motor_dir_e dir);
void drive_reacquire_corridor(turn_dir_e pref_dir);

void drive_dist(uint32_t target_pulses, motor_dir_e dir);
void turn_in_place(float target_angle_deg, turn_dir_e dir);

void drive_until_wall_reengaged(turn_dir_e pref_dir);
maze_event_e drive_to_intersection(void);
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

    /* USER CODE BEGIN 3 */
    maze_solve_step(_ALGO_RIGHT_WALL);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
  htim3.Init.Prescaler = 83;
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

// Application

/**
 * @brief Drives a fixed distance with only encoder cross-track PID, no
 *        ultrasonic centering. DEPRECATED for forward corridor travel — use
 *        drive_guided() instead, which is what caused the post-turn wall
 *        collisions. Kept only because it's the one primitive that supports
 *        _DIR_REVERSE.
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

  s_base_speed = 400;
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
    if (dir == _DIR_FORWARD && (now - last_us_check >= ULT_SNC_READ_MS)) {
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

    int32_t left_speed = s_base_speed - correction;
    int32_t right_speed = s_base_speed + correction;

    // If one wheel finishes early, stop it while the other catches up
    if (current_left >= target_pulses) {
      left_speed = 0;
    }

    if (current_right >= target_pulses) {
      right_speed = 0;
    }

    motor_set_speed(_MOTOR_L, left_speed);
    motor_set_speed(_MOTOR_R, right_speed);

    HAL_Delay(CONTROL_PERIOD_MS);
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

  // @Patch: Chassis Clearance Nudge
  // Advance half the body length (8cm) to prevent bumper clipping during
  // rotation. 8 cm / 1.021 cm/pulse = ~8 pulses. Guided (not blind) so the
  // car stays centered right up to the pivot point.
  drive_guided(7, _DIR_FORWARD);
  // Pause briefly to let the chassis settle and prevent gyro motion artifacts
  HAL_Delay(100);

  float turn_sum = 0.0f;

  uint32_t last_gyro_check = HAL_GetTick();
  uint32_t last_control_time = HAL_GetTick();

  // Reset local encoder tracking to synchronize wheel speeds during the turn

  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  // Dynamic speed parameters
  const uint32_t MIN_SPEED = 400; // Minimum PWM to overcome motor stall torque
  const uint32_t MAX_SPEED = 450;

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

      if (base_speed > MAX_SPEED) {
        base_speed = MAX_SPEED;
      }

      if (base_speed < MIN_SPEED) {
        base_speed = MIN_SPEED;
      }

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

/**
 * @brief Shared centering law: cross-track error from side ultrasonics when a
 *        wall is present, falling back to encoder cross-track error in open
 *        space. Dynamically updates corridor_target_cm whenever both walls
 *        are seen, so single-wall tracking uses a fresh half-width.
 */

static int32_t compute_centering_correction(uint32_t left_us, uint32_t right_us,
                                            uint32_t enc_left_delta,
                                            uint32_t enc_right_delta,
                                            uint32_t now_ms) {
  const int32_t DEADBAND_CM = 2;

  const float Kp_center = 10.0f;
  const float Kd_center = 2.0f;
  const float Kp_enc = 30.0f;

  static float previous_error = 0.0f;
  static uint32_t previous_time_ms = 0;

  int32_t center_error = 0;
  uint8_t use_encoder_pid = 0;

  float dt = 0.0f;

  if (previous_time_ms != 0U) {
    uint32_t dt_ms = now_ms - previous_time_ms;
    dt = (float)dt_ms / 1000.0f;
  }

  previous_time_ms = now_ms;

  // Determine centering error.
  if (left_us < OPENING_THRESHOLD_CM && right_us < OPENING_THRESHOLD_CM) {
    corridor_target_cm = ((int32_t)left_us + (int32_t)right_us) / 2;

    center_error = (int32_t)left_us - (int32_t)right_us;
  } else if (left_us < OPENING_THRESHOLD_CM) {
    center_error = (int32_t)left_us - corridor_target_cm;
  } else if (right_us < OPENING_THRESHOLD_CM) {
    center_error = corridor_target_cm - (int32_t)right_us;
  } else {
    use_encoder_pid = 1;
  }

  // Open-space encoder fallback.
  if (use_encoder_pid) {
    int32_t enc_error = (int32_t)enc_left_delta - (int32_t)enc_right_delta;

    /*
     * Keep the previous ultrasonic error synchronized
     * with the current controller state.
     */
    previous_error = 0.0f;

    return (int32_t)(Kp_enc * (float)enc_error);
  }

  if (center_error >= -DEADBAND_CM && center_error <= DEADBAND_CM) {
    center_error = 0;
  }

  float error = (float)center_error;
  float derivative = 0.0f;

  if (dt > 0.0f) {
    derivative = (error - previous_error) / dt;
  }

  previous_error = error;

  // PD controller.
  float correction = Kp_center * error + Kd_center * derivative;

  return (int32_t)correction;
}

/**

 * @brief Universal guided translation. Drives forward exactly target_pulses
 *        while continuously running the ultrasonic centering PID (falls back
 *        to encoder cross-track PID in open space). Replaces every blind
 *        encoder-only forward move, including the post-turn advance, so the
 *        car never drifts diagonally into a wall before centering kicks in.
 * @param target_pulses Distance to travel, in encoder ticks. 0 = no-op.
 */

void drive_guided(uint32_t target_pulses, motor_dir_e dir) {

  if (target_pulses == 0)

    return;

  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  uint32_t last_left_count = start_left;
  uint32_t last_right_count = start_right;

  uint32_t last_movement_time = HAL_GetTick();

  const uint32_t STALL_TIMEOUT_MS = 500;
  s_base_speed = 450;

  motor_set_dir(_MOTOR_L, dir);
  motor_set_dir(_MOTOR_R, dir);

  // Seed with "unknown/far" so the very first control tick still runs a
  // centering pass instead of a blind one, on stale-but-safe defaults.
  uint32_t latest_front_us = 999;
  uint32_t latest_left_us = 999;
  uint32_t latest_right_us = 999;
  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0; // 0: front (safety), 1: left, 2: right

  while (1) {

    uint32_t now = HAL_GetTick();

    uint32_t current_left = encoder_get_total(_ENC_LEFT);
    uint32_t current_right = encoder_get_total(_ENC_RIGHT);

    uint32_t delta_left = current_left - start_left;
    uint32_t delta_right = current_right - start_right;

    // Stall watchdog
    if (current_left != last_left_count || current_right != last_right_count) {
      last_left_count = current_left;
      last_right_count = current_right;
      last_movement_time = now;
    }

    if (now - last_movement_time > STALL_TIMEOUT_MS) {
      Error_Handler();
    }
    if (delta_left >= target_pulses && delta_right >= target_pulses) {
      break;
    }

    // Round-robin polling: front (crash safety) + both side walls.
    if (now - last_us_check >= ULT_SNC_READ_MS) {
      last_us_check = now;
      if (us_poll_target == 0) {
        latest_front_us = ultrasonic_read(_US_FRONT);
        if (latest_front_us <= CRASH_SAFETY_CM) {
          break; // Emergency stop, short of target.
        }
      } else if (us_poll_target == 1) {
        latest_left_us = ultrasonic_read(_US_LEFT);
      } else {
        latest_right_us = ultrasonic_read(_US_RIGHT);
      }
      us_poll_target = (us_poll_target + 1) % 3;
    }

    int32_t correction = compute_centering_correction(
        latest_left_us, latest_right_us, delta_left, delta_right, now);

    int32_t left_speed = (int32_t)s_base_speed - correction;
    int32_t right_speed = (int32_t)s_base_speed + correction;

    // Let the near-finished wheel coast so it doesn't overshoot the target.
    if (delta_left >= target_pulses) {
      left_speed = 0;
    }
    if (delta_right >= target_pulses) {
      right_speed = 0;
    }

    motor_set_speed(_MOTOR_L, left_speed);
    motor_set_speed(_MOTOR_R, right_speed);

    HAL_Delay(CONTROL_PERIOD_MS);
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
 * @brief Advances into the new corridor right after a turn, actively
 *        centering/squaring from tick 0 (no blind phase) until the preferred
 *        wall is reacquired or a front wall is hit. This is what fixes the
 *        post-turn wall collision: the old version only ran encoder
 *        cross-track PID here, which does not correct for the small heading
 *        error left over from the turn.
 * @param pref_dir The wall we are trying to re-engage (_TURN_RIGHT or
 * _TURN_LEFT).
 */

void drive_reacquire_corridor(turn_dir_e pref_dir) {

  // @Patch: needs to be solved: as maze is know; moving car forward to reduce
  // reacuire corridor moves car forward to its halg lenght distance
  drive_guided(8, _DIR_FORWARD);
  // Pause briefly to let the chassis settle and prevent gyro motion artifacts
  HAL_Delay(250);

  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  uint32_t last_left_count = start_left;
  uint32_t last_right_count = start_right;
  uint32_t last_movement_time = HAL_GetTick();

  const uint32_t STALL_TIMEOUT_MS = 500;
  s_base_speed = 450;
  // Matches OPENING_THRESHOLD_CM
  const uint32_t REENGAGE_THRESHOLD_CM = OPENING_THRESHOLD_CM;
  const uint32_t STOP_DISTANCE_CM = 5;

  ultrasonic_e pref_sensor = (pref_dir == _TURN_RIGHT) ? _US_RIGHT : _US_LEFT;

  motor_set_dir(_MOTOR_L, _DIR_FORWARD);
  motor_set_dir(_MOTOR_R, _DIR_FORWARD);

  uint32_t latest_front_us = 999;
  uint32_t latest_left_us = 999;
  uint32_t latest_right_us = 999;
  uint32_t last_us_check = HAL_GetTick();
  uint8_t us_poll_target = 0; // 0: front, 1: left, 2: right

  while (1) {

    uint32_t now = HAL_GetTick();

    uint32_t current_left = encoder_get_total(_ENC_LEFT);
    uint32_t current_right = encoder_get_total(_ENC_RIGHT);
    uint32_t delta_left = current_left - start_left;
    uint32_t delta_right = current_right - start_right;

    if (current_left != last_left_count || current_right != last_right_count) {

      last_left_count = current_left;
      last_right_count = current_right;
      last_movement_time = now;
    }

    if (now - last_movement_time > STALL_TIMEOUT_MS) {
      Error_Handler();
    }

    if (now - last_us_check >= ULT_SNC_READ_MS) {
      last_us_check = now;

      if (us_poll_target == 0) {
        latest_front_us = ultrasonic_read(_US_FRONT);
        if (latest_front_us <= STOP_DISTANCE_CM) {
          break; // T-junction / dead end reached before wall reappeared.
        }

      } else if (us_poll_target == 1) {
        latest_left_us = ultrasonic_read(_US_LEFT);

      } else {
        latest_right_us = ultrasonic_read(_US_RIGHT);
      }
      us_poll_target = (us_poll_target + 1) % 3;
    }

    uint32_t pref_us =
        (pref_sensor == _US_LEFT) ? latest_left_us : latest_right_us;

    if (pref_us < REENGAGE_THRESHOLD_CM) {
      break; // Preferred wall reacquired; squared and centered.
    }

    int32_t correction = compute_centering_correction(
        latest_left_us, latest_right_us, delta_left, delta_right, now);

    motor_set_speed(_MOTOR_L, s_base_speed - correction);
    motor_set_speed(_MOTOR_R, s_base_speed + correction);

    HAL_Delay(CONTROL_PERIOD_MS);
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
 * @brief Drives forward using dynamic ultrasonic PID centering with a deadband.
 */

maze_event_e drive_to_intersection(void) {

  uint32_t start_left = encoder_get_total(_ENC_LEFT);
  uint32_t start_right = encoder_get_total(_ENC_RIGHT);

  uint32_t last_left_count = start_left;
  uint32_t last_right_count = start_right;

  uint32_t last_movement_time = HAL_GetTick();

  const uint32_t STALL_TIMEOUT_MS = 500;

  motor_set_dir(_MOTOR_L, _DIR_FORWARD);
  motor_set_dir(_MOTOR_R, _DIR_FORWARD);

  s_base_speed = 450;

  const uint32_t GOAL_OPENING_THRESHOLD_CM = 20; // Threshold for open goal zone
  const uint32_t STOP_DISTANCE_CM = 5;           // Front wall stopping distance

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

    //  Stall-Detecting Software Watchdog
    if (current_left != last_left_count || current_right != last_right_count) {

      last_left_count = current_left;
      last_right_count = current_right;
      last_movement_time = now;
    }

    if (now - last_movement_time > STALL_TIMEOUT_MS) {
      Error_Handler();
    }

    // Environmental Polling (Round-robin)
    if (now - last_us_check >= ULT_SNC_READ_MS) {
      last_us_check = now;

      if (us_poll_target == 0) {
        latest_right_us = ultrasonic_read(_US_RIGHT);

        // @dev: (current_left - start_left) > 8) = Don't consider a side
        // opening until the car has moved
        // more than 8 encoder counts from the starting position.
        if (latest_right_us > OPENING_THRESHOLD_CM &&
            (current_left - start_left) > 8) {
          detected_event = _MAZE_EVENT_RIGHT_OPEN;
          break;
        }

      } else if (us_poll_target == 1) {
        latest_left_us = ultrasonic_read(_US_LEFT);
        if (latest_left_us > OPENING_THRESHOLD_CM &&
            (current_left - start_left) > 8) {
          detected_event = _MAZE_EVENT_LEFT_OPEN;
          break;
        }
      } else {
        latest_front_us = ultrasonic_read(_US_FRONT);

        if (latest_front_us <= STOP_DISTANCE_CM) {
          detected_event = _MAZE_EVENT_FRONT_BLOCKED;
          break;
        }
      }

      us_poll_target = (us_poll_target + 1) % 3;
    }

    // --- GOAL CHECK ---
    // Evaluated continuously once sensors update to catch open target zones
    if ((current_left - start_left) > 8 &&
        latest_front_us > GOAL_OPENING_THRESHOLD_CM &&
        latest_left_us > GOAL_OPENING_THRESHOLD_CM &&
        latest_right_us > GOAL_OPENING_THRESHOLD_CM) {
      detected_event = _MAZE_EVENT_SOLVED;
      break;
    }

    //  Centering PID with dynamic corridor tracking + deadband (shared law)
    int32_t correction = compute_centering_correction(
        latest_left_us, latest_right_us, current_left - start_left,
        current_right - start_right, now);

    motor_set_speed(_MOTOR_L, s_base_speed - correction);
    motor_set_speed(_MOTOR_R, s_base_speed + correction);

    HAL_Delay(CONTROL_PERIOD_MS);
  }

  // Active Braking
  motor_set_dir(_MOTOR_L, _DIR_BRAKE);
  motor_set_dir(_MOTOR_R, _DIR_BRAKE);

  motor_set_speed(_MOTOR_L, 0);
  motor_set_speed(_MOTOR_R, 0);

  HAL_Delay(100);

  motor_set_dir(_MOTOR_L, _DIR_COAST);
  motor_set_dir(_MOTOR_R, _DIR_COAST);

  return detected_event;
}

/**
 * @brief Universal wall follower step.
 * @param pref_dir The wall to track (_TURN_RIGHT or _TURN_LEFT).
 */

void algo_wall_follower_step(turn_dir_e pref_dir) {

  const uint32_t WALL_DETECT_CM = 20; // Secondary check threshold
  turn_dir_e opp_dir = (pref_dir == _TURN_RIGHT) ? _TURN_LEFT : _TURN_RIGHT;
  ultrasonic_e pref_sensor = (pref_dir == _TURN_RIGHT) ? _US_RIGHT : _US_LEFT;
  ultrasonic_e opp_sensor = (pref_dir == _TURN_RIGHT) ? _US_LEFT : _US_RIGHT;

  maze_event_e event = drive_to_intersection();

  if (event == _MAZE_EVENT_SOLVED) {

    while (1) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(500);
    }
  }

  //  Passing an Open Intersection
  if ((pref_sensor == _US_RIGHT && event == _MAZE_EVENT_RIGHT_OPEN) ||
      (pref_sensor == _US_LEFT && event == _MAZE_EVENT_LEFT_OPEN)) {
    turn_in_place(90.0f, pref_dir);
    drive_reacquire_corridor(pref_dir);
  }

  //  Approaching a Corner / Dead End
  else if (event == _MAZE_EVENT_FRONT_BLOCKED) {
    if (ultrasonic_read(pref_sensor) > WALL_DETECT_CM) {
      turn_in_place(90.0f, pref_dir);
      drive_reacquire_corridor(pref_dir);
    } else if (ultrasonic_read(opp_sensor) > WALL_DETECT_CM) {
      turn_in_place(90.0f, opp_dir);
      drive_reacquire_corridor(opp_dir);
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
