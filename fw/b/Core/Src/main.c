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
#include <stdio.h>
# include "mpu6050.h"
# include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t right;
  uint32_t middle;
  uint32_t left;
} UltrasonicReading;

typedef enum {
  DRIVE_RESULT_RIGHT_OPEN,
  DRIVE_RESULT_FRONT_WALL
} DriveResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WALL_DETECT_CM 10   // ultrasonic reading above this = opening (no wall); at/below = wall present - tune by testing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t leftPulseCount = 0;
volatile uint32_t rightPulseCount = 0;
volatile uint32_t leftTotalCount = 0;
volatile uint32_t rightTotalCount = 0;
volatile uint8_t irLeftTriggered = 0;
volatile uint8_t irCenterTriggered = 0;
volatile uint8_t irRightTriggered = 0;

int32_t total_error = 0;
int32_t error = 0;
int32_t correction = 0;
static uint8_t solved = 0;

volatile float manual_max_dps = 0.0f;
volatile float manual_min_dps = 0.0f;
volatile uint8_t dps_is_saturated = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void DriveUntil(int8_t leftDir, int8_t rightDir, uint32_t targetPulses);
void MoveForward(uint32_t targetPulses);
uint32_t PulsesForAngle(float angleDegrees);
void MazeSolveStep(void);
void TurnRight(float angleDegrees);
void TurnLeft(float angleDegrees);
static void DWT_Init(void);
static void DelayMicroseconds(uint32_t us);
uint32_t ReadUltrasonicCM(GPIO_TypeDef* trigPort, uint16_t trigPin, GPIO_TypeDef* echoPort, uint16_t echoPin);
UltrasonicReading ReadAllUltrasonic(void);
DriveResult DriveForwardAdaptive(void);
void take_turn(int8_t dir, float angle);
void take_turn_2(int8_t dir, float angle);
void check_gyro_saturation_manual(void);
void TestFreewheel(uint32_t speed, uint8_t useBrake);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  DWT_Init();
  MPU6050_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // PWMB - PA6
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // PWMA - PA7

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // STBY = HIGH (enable driver)

  // Motor A forward: AIN1=1, AIN2=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  // Motor B forward: BIN1=1, BIN2=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);


  HAL_Delay(2000);   // 5 second delay before motors start

  //TestFreewheel(400,1); // to test free wheell
  //HAL_Delay(5000);

  //MoveForward(200);
  //HAL_Delay(100);

//	take_turn(1, 90.0);
//	HAL_Delay(300);
//	take_turn(1, 90.0);
//	HAL_Delay(300);
//	take_turn(1, 90.0);
//	HAL_Delay(300);
//	take_turn(1, 90.0);
//	HAL_Delay(300);


//  take_turn_2(1,90.0);
//  HAL_Delay(300);
//  take_turn_2(1,90.0);
//  HAL_Delay(3000);
//
//  take_turn_2(1,180.0);
//  HAL_Delay(300);
//  take_turn_2(1,180.0);
//  HAL_Delay(300);


//  check_gyro_saturation_manual();

//  uint8_t gyroConfig = MPU6050_ReadRegister(0x1B);
//  char msg[32];
//  int len = snprintf(msg, sizeof(msg), "GYRO_CONFIG: 0x%02X\r\n", gyroConfig);
//  HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);


  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 480); // Motor B speed (~50%)
  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500); // Motor A speed (~50%)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastLedToggle = 0;
  uint32_t lastUltrasonicRead = 0;
  uint32_t last_gyro_check = 0;
  float read_gyro_dps = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (solved)
	      {
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // LED solid on = done
	        continue;
	      }
	  else
	  {
		  MazeSolveStep();
	  }


    uint32_t now = HAL_GetTick();

		if (now - lastLedToggle >= 200)
		{
			lastLedToggle = now;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB3 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10; // AIN1, AIN2, STBY
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4) { leftPulseCount++; leftTotalCount++; }
  else if (GPIO_Pin == GPIO_PIN_5) { rightPulseCount++; rightTotalCount++; }
  else if (GPIO_Pin == GPIO_PIN_0) { irRightTriggered = 1; }
  else if (GPIO_Pin == GPIO_PIN_1) { irCenterTriggered = 1; }
  else if (GPIO_Pin == GPIO_PIN_2) { irLeftTriggered = 1; }
}


// leftDir/rightDir: +1 = forward, -1 = backward, 0 = stopped (for future use)
void DriveUntil(int8_t leftDir, int8_t rightDir, uint32_t targetPulses)
{
  // Set left wheel (Motor B) direction
  if (leftDir > 0)      { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); }
  else if (leftDir < 0)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); }

  // Set right wheel (Motor A) direction
  if (rightDir > 0)      { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); }
  else if (rightDir < 0)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); }

  uint32_t startLeft  = leftTotalCount;
  uint32_t startRight = rightTotalCount;

  uint32_t BASE_SPEED = 400;   // <-- change this one value to adjust speed everywhere below

  uint32_t lastControlTime = HAL_GetTick();
  float integral = 0.0f;
  float lastError = 0.0f;
  const float Kp = 30.0f;
  const float Ki = 30.0f;
  const float Kd = 0.0f;
  const float INTEGRAL_LIMIT = 300.0f;
  uint32_t baseDutyLeft  = BASE_SPEED;
  uint32_t baseDutyRight = BASE_SPEED;

  // centering state - only active during straight moves, not turns
	uint32_t lastCenteringCheck = HAL_GetTick();
	float center_integral = 0.0f;
	float center_lastError = 0.0f;
	const float centerKp = 10.0f;
	const float centerKi = 5.0f;
	const float centerKd = 5.0f;
	const float CENTRAL_INTEGRAL_LIMIT = 200.0f;
	const uint32_t CENTERING_MAX_RANGE = 35;  // cm - ignore readings beyond this (no wall in range)
	uint8_t centeringEnabled = 0; // disabling centering

  int leftDone = 0, rightDone = 0;

  while (!leftDone || !rightDone)
  {
    uint32_t now = HAL_GetTick();

    if (centeringEnabled && (now - lastCenteringCheck >= 100))
    {
      lastCenteringCheck = now;

      uint32_t leftDist  = ReadUltrasonicCM(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6);
      uint32_t rightDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4);

      if (leftDist <= CENTERING_MAX_RANGE && rightDist <= CENTERING_MAX_RANGE)
      {
        int32_t centerError = (int32_t)leftDist - (int32_t)rightDist;

        center_integral += (float)centerError;

        if (center_integral > CENTRAL_INTEGRAL_LIMIT)  center_integral = CENTRAL_INTEGRAL_LIMIT;
        if (center_integral < -CENTRAL_INTEGRAL_LIMIT) center_integral = -CENTRAL_INTEGRAL_LIMIT;

        float centre_derivative = (float)centerError - center_lastError;
        center_lastError = centerError;
        int32_t centerBias  = (int32_t)(centerKp * (float)centerError + centerKi * center_integral + centerKd * centre_derivative);

        baseDutyLeft  = BASE_SPEED - centerBias;
        baseDutyRight = BASE_SPEED + centerBias;
      }
    }

    if (now - lastControlTime >= 1)
    {
      lastControlTime = now;

      __disable_irq();
      uint32_t left  = leftPulseCount;
      uint32_t right = rightPulseCount;
      leftPulseCount = 0;
      rightPulseCount = 0;
      __enable_irq();

      error = (int32_t)left - (int32_t)right;
      integral += (float)error;
      if (integral > INTEGRAL_LIMIT)  integral = INTEGRAL_LIMIT;
      if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

      float derivative = (float)error - lastError;
      lastError = (float)error;

      correction = (int32_t)(Kp * (float)error + Ki * integral + Kd * derivative);

      int32_t leftDuty, rightDuty;

      if (!leftDone && !rightDone)
      {
        // normal case: both still moving, use PI correction as before
        leftDuty  = (int32_t)baseDutyLeft  - correction;
        rightDuty = (int32_t)baseDutyRight + correction;
      }
      else
      {
        // one side already finished - stop cross-correcting, just drive the other plainly
        leftDuty  = leftDone  ? 0 : (int32_t)baseDutyLeft;
        rightDuty = rightDone ? 0 : (int32_t)baseDutyRight;
      }

      if (leftDuty  > 999) leftDuty  = 999;
      if (leftDuty  < 0)   leftDuty  = 0;
      if (rightDuty > 999) rightDuty = 999;
      if (rightDuty < 0)   rightDuty = 0;

      if ((leftTotalCount - startLeft) >= targetPulses)  { leftDuty = 0;  leftDone = 1; }
      if ((rightTotalCount - startRight) >= targetPulses) { rightDuty = 0; rightDone = 1; }

      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, leftDuty);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightDuty);
    }
  }

//  char msg[64];
//  int len = snprintf(msg, sizeof(msg), "DONE left:%lu right:%lu target:%lu\r\n",
//                      (unsigned long)(leftTotalCount - startLeft),
//                      (unsigned long)(rightTotalCount - startRight),
//                      (unsigned long)targetPulses);
//  HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
}



// Drives forward continuously, using only ultrasonic sensors (right/front/left) - no IR.
// Stops either when the right side opens up (past a small nudge to clear the sensor's
// forward offset) or when the front wall gets within STOP_DISTANCE_CM.

void take_turn_2(int8_t dir, float angle)
{
	float turn_sum = 0;
	uint32_t lastControlTime = HAL_GetTick();
	uint32_t last_gyro_check = HAL_GetTick();
	float integral = 0.0f;
	float lastError = 0.0f;

	// Motor PID constants
	const float Kp = 30.0f;
	const float Ki = 30.0f;
	const float Kd = 0.0f;
	const float INTEGRAL_LIMIT = 300.0f;
	const uint32_t gyro_check = 2;

	// Variable for time - moved to top so it stays in scope after the turn loop
	uint32_t now;

	// Variables for error and correction
	int32_t error = 0;
	int32_t correction = 0;

	// Dynamic Speed Parameters for Proportional Deceleration
	const uint32_t MIN_SPEED = 340;
	const uint32_t MAX_SPEED = 500;
	const float Kp_angle = 5.0f;

	if (dir == 0)
	{
		// Left wheel backward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		// Right wheel forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	} else
	{
		// Left wheel Forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		// Right wheel Backward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}

	while (fabsf(turn_sum) <= angle - 17.0)
	{
		// --- Dynamic Speed Profiling ---
		// Calculate how many degrees are left to turn
		float angle_remaining = angle - fabsf(turn_sum);

		// Calculate base speed proportionally based on remaining angle
		uint32_t current_base_speed = (uint32_t)(angle_remaining * Kp_angle);

		// Clamp the speed so it doesn't exceed maximum or drop below stall limit
		if (current_base_speed > MAX_SPEED) current_base_speed = MAX_SPEED;
		if (current_base_speed < MIN_SPEED) current_base_speed = MIN_SPEED;

		uint32_t baseDutyLeft  = current_base_speed;
		uint32_t baseDutyRight = current_base_speed;

		now = HAL_GetTick();

		// --- Encoder PID Control ---
		if (now - lastControlTime >= 1)
		{
			lastControlTime = now;

			__disable_irq();
			uint32_t left  = leftPulseCount;
			uint32_t right = rightPulseCount;
			leftPulseCount = 0;
			rightPulseCount = 0;
			__enable_irq();

			error = (int32_t)left - (int32_t)right;
			integral += (float)error;

			// Integral windup guard
			if (integral > INTEGRAL_LIMIT)  integral = INTEGRAL_LIMIT;
			if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

			float derivative = (float)error - lastError;
			lastError = (float)error;

			correction = (int32_t)(Kp * (float)error + Ki * integral + Kd * derivative);

			int32_t leftDuty  = (int32_t)baseDutyLeft  - correction;
			int32_t rightDuty = (int32_t)baseDutyRight + correction;

			if (leftDuty  > 999) leftDuty  = 999;
			if (leftDuty  < 0)   leftDuty  = 0;
			if (rightDuty > 999) rightDuty = 999;
			if (rightDuty < 0)   rightDuty = 0;

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, leftDuty);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightDuty);
		}

		// --- Gyroscope Angle Integration ---
		if (now - last_gyro_check >= gyro_check)
		{
			float dt = (now - last_gyro_check) * 0.001f;   // actual elapsed ms -> seconds
			last_gyro_check = now;
			turn_sum += MPU6050_ReadGyroZ_DPS() * dt;
		}
	}

	// --- Active Braking ---
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

	// Both direction pins HIGH shorts the motor terminals,
	// using back-EMF to kill momentum fast instead of coasting
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // BIN1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // BIN2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // AIN1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // AIN2

	// instead of a blind delay, keep recording turn_sum during the brake hold
	uint32_t record_more_start = HAL_GetTick();
	now = HAL_GetTick();
	float turn_sum_extra = turn_sum;
	while (now - record_more_start <= 100)
	{
		now = HAL_GetTick();
		if (now - last_gyro_check >= gyro_check)
		{
			float dt = (now - last_gyro_check) * 0.001f; // actual elapsed ms -> seconds
			last_gyro_check = now;
			turn_sum_extra += MPU6050_ReadGyroZ_DPS() * dt;
		}
	}

	// Release to coast/off afterward so the motors aren't held under load
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	char msg[64];
	int len = snprintf(msg, sizeof(msg), "turn_sum: %.2f | turn_sum_extra: %.2f\r\n", turn_sum, turn_sum_extra);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
}

// dir-> 0:left , 1 right

void check_gyro_saturation_manual(void)
{
    // 1. Reset the debug variables
    manual_max_dps = 0.0f;
    manual_min_dps = 0.0f;
    dps_is_saturated = 0;

    // 2. Ensure motors are completely off
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // 3. Infinite loop to monitor while you spin it by hand
    while (1)
    {
        // Read the actual Degrees Per Second
        float current_dps = MPU6050_ReadGyroZ_DPS();

        // Track the highest and lowest DPS recorded
        if (current_dps > manual_max_dps) manual_max_dps = current_dps;
        if (current_dps < manual_min_dps) manual_min_dps = current_dps;

        // The absolute ceiling is ~1998 dps.
        // We check against 1990 to allow a tiny margin for calibration offsets.
        if (current_dps >= 1990.0f || current_dps <= -1990.0f)
        {
            dps_is_saturated = 1;
        }

        // 2ms delay to simulate your standard polling rate
        HAL_Delay(2);
    }
}

void TestFreewheel(uint32_t speed, uint8_t useBrake)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(5000);
  // spin both wheels forward
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // left fwd
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // right fwd

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);

  HAL_Delay(2000);   // spin for 2 seconds

  // cut PWM either way
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  if (useBrake)
  {
    // active brake: both direction pins HIGH shorts the motor terminals
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // LED turns on the instant the brake is applied - watch the wheel starting now
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_Delay(500);   // hold the brake briefly

    // release afterward so motors aren't held under load
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  else
  {
    // coast: both direction pins LOW
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // LED turns on the instant power is cut - watch the wheel starting now
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
}

// right = 1 , left = 0
void take_turn(int8_t dir, float angle)
{
	float turn_sum = 0;
	uint32_t lastControlTime = HAL_GetTick();
	uint32_t last_gyro_check = HAL_GetTick();
	float integral = 0.0f;
	float lastError = 0.0f;
	const float Kp = 30.0f;
	const float Ki = 30.0f;
	const float Kd = 0.0f;
	const float INTEGRAL_LIMIT = 300.0f;
	const uint32_t gyro_check = 2;

	// Variable for time
	uint32_t now;



	uint32_t BASE_SPEED = 450;   // <-- change this one value to adjust speed everywhere below
	uint32_t baseDutyLeft  = BASE_SPEED;
	uint32_t baseDutyRight = BASE_SPEED;

	if (dir == 0)
	{
		//Left wheel backward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		// Right wheel forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	} else
	{
		// Left wheel Forward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		//Right wheel Backward
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}

	while (fabsf(turn_sum) <= angle)
	{
		now = HAL_GetTick();
		if (now - lastControlTime >= 1)
		    {
		      lastControlTime = now;

		      __disable_irq();
		      uint32_t left  = leftPulseCount;
		      uint32_t right = rightPulseCount;
		      leftPulseCount = 0;
		      rightPulseCount = 0;
		      __enable_irq();

		      error = (int32_t)left - (int32_t)right;
		      integral += (float)error;
		      //integral windup
		      if (integral > INTEGRAL_LIMIT)  integral = INTEGRAL_LIMIT;
		      if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

		      float derivative = (float)error - lastError;
		      lastError = (float)error;

		      correction = (int32_t)(Kp * (float)error + Ki * integral + Kd * derivative);

		      int32_t leftDuty  = (int32_t)baseDutyLeft  - correction;
		      int32_t rightDuty = (int32_t)baseDutyRight + correction;

		      if (leftDuty  > 999) leftDuty  = 999;
		      if (leftDuty  < 0)   leftDuty  = 0;
		      if (rightDuty > 999) rightDuty = 999;
		      if (rightDuty < 0)   rightDuty = 0;

			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, leftDuty);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightDuty);
	}
		if (now - last_gyro_check >= gyro_check)
		{
			float dt = (now - last_gyro_check) * 0.001f;   // actual elapsed ms -> seconds
			last_gyro_check = now;
			//turn_sum += MPU6050_ReadGyroZ_DPS() * (gyro_check * 0.001f);
			turn_sum += MPU6050_ReadGyroZ_DPS() * dt;
		}

	}
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	  // active brake: both direction pins HIGH shorts the motor terminals,
	  // using back-EMF to kill momentum fast instead of coasting
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // BIN1
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // BIN2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // AIN1
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // AIN2

	  //HAL_Delay(100);   // hold the brake briefly so momentum actually bleeds off
	  uint32_t record_more_start = HAL_GetTick();
	  now = HAL_GetTick();
	  float turn_sum_extra = turn_sum;
	  while (now - record_more_start <= 100)
	  {
		now = HAL_GetTick();
		if (now - last_gyro_check >= gyro_check)
		{
			float dt = (now - last_gyro_check) * 0.001f; // actual elapsed ms -> seconds
			last_gyro_check = now;
			//turn_sum += MPU6050_ReadGyroZ_DPS() * (gyro_check * 0.001f);
			turn_sum_extra += MPU6050_ReadGyroZ_DPS() * dt;
		}

	  }

	  // release to coast/off afterward so the motors aren't held under load
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	  char msg[64];
	  int len = snprintf(msg, sizeof(msg), "turn_sum: %.2f | turn_sum_extra: %.2f\r\n", turn_sum,turn_sum_extra);
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
}


DriveResult DriveForwardAdaptive(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // left fwd
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // right fwd

  uint32_t BASE_SPEED = 430;   // <-- change this one value to adjust forward speed everywhere below
  uint32_t BASE_SPEEDRIGHT = BASE_SPEED + 15;

  uint32_t lastControlTime = HAL_GetTick();
  float integral = 0.0f, lastError = 0.0f;
  const float Kp = 30.0f, Ki = 30.0f, Kd = 0.0f, INTEGRAL_LIMIT = 300.0f;

  float center_integral = 0.0f, center_lastError = 0.0f;
  const float centerKp = 12.0f, centerKi = 2.0f, centerKd = 2.0f, CENTRAL_INTEGRAL_LIMIT = 200.0f;
  const uint32_t CENTERING_MAX_RANGE = 20;

  uint32_t lastSensorCheck = HAL_GetTick();
  const uint32_t APPROACH_DISTANCE_CM = 25;
  const uint32_t STOP_DISTANCE_CM = 4;
  uint8_t approaching = 0;

  uint8_t rightOpenDetected = 0;
  uint32_t nudgeStartPulses = 0;
  const uint32_t RIGHT_OPEN_NUDGE_PULSES = 12;

  uint32_t baseDutyLeft = BASE_SPEED, baseDutyRight = BASE_SPEED;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (rightOpenDetected && (leftTotalCount - nudgeStartPulses) >= RIGHT_OPEN_NUDGE_PULSES)
    {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      char msg[64];
      int len = snprintf(msg, sizeof(msg), "Right Detected\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
      return DRIVE_RESULT_RIGHT_OPEN;
    }

    if (now - lastSensorCheck >= 10)
    {
      lastSensorCheck = now;
      uint32_t rightDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4);

      if (!rightOpenDetected && rightDist > WALL_DETECT_CM)
      {
        rightOpenDetected = 1;
        nudgeStartPulses = leftTotalCount;

      }

      uint32_t frontDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8);

      if (frontDist <= STOP_DISTANCE_CM)
      {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "Front Detected\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
        if (!rightOpenDetected)
        {
        	return DRIVE_RESULT_FRONT_WALL;
        }
      }
      approaching = (frontDist <= APPROACH_DISTANCE_CM);

      if (!approaching && !rightOpenDetected)
      {
        uint32_t leftDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6);

        if (leftDist <= CENTERING_MAX_RANGE && rightDist <= CENTERING_MAX_RANGE)
        {
          int32_t centerError = (int32_t)leftDist - (int32_t)rightDist;
          center_integral += (float)centerError;
          if (center_integral > CENTRAL_INTEGRAL_LIMIT)  center_integral = CENTRAL_INTEGRAL_LIMIT;
          if (center_integral < -CENTRAL_INTEGRAL_LIMIT) center_integral = -CENTRAL_INTEGRAL_LIMIT;
          float centre_derivative = (float)centerError - center_lastError;
          center_lastError = (float)centerError;
          int32_t centerBias = (int32_t)(centerKp*(float)centerError + centerKi*center_integral + centerKd*centre_derivative);
          baseDutyLeft  = BASE_SPEED - centerBias;
          baseDutyRight = BASE_SPEEDRIGHT + centerBias; //// changessss made revert
        }
      }
      else
      {
        baseDutyLeft = BASE_SPEED;
        baseDutyRight = BASE_SPEED;
      }
    }

    if (now - lastControlTime >= 1)
    {
      lastControlTime = now;
      __disable_irq();
      uint32_t left = leftPulseCount, right = rightPulseCount;
      leftPulseCount = 0; rightPulseCount = 0;
      __enable_irq();

      error = (int32_t)left - (int32_t)right;
      integral += (float)error;
      if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
      if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
      float derivative = (float)error - lastError;
      lastError = (float)error;
      correction = (int32_t)(Kp*(float)error + Ki*integral + Kd*derivative);

      int32_t leftDuty  = (int32_t)baseDutyLeft  - correction;
      int32_t rightDuty = (int32_t)baseDutyRight + correction;
      if (leftDuty  > 999) leftDuty  = 999;
      if (leftDuty  < 0)   leftDuty  = 0;
      if (rightDuty > 999) rightDuty = 999;
      if (rightDuty < 0)   rightDuty = 0;

      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, leftDuty);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightDuty);
    }
  }
}


void MoveForward(uint32_t targetPulses)
{
  DriveUntil(+1, +1, targetPulses);   // both wheels forward
}

// pulses = angle * wheelbase * slots / (360 * wheel_diameter), for your 131mm wheelbase, 20-slot disc, 66mm wheel
uint32_t PulsesForAngle(float angleDegrees)
{
  float pulses = angleDegrees * 110.0f * 20.0f / (360.0f * 66.0f);
  return (uint32_t)(pulses + 0.5f);  // round to nearest whole pulse
}

void TurnRight(float angleDegrees)
{
  uint32_t targetPulses = PulsesForAngle(angleDegrees);
  DriveUntil(+1, -1, targetPulses);   // left wheel forward, right wheel backward -> pivots clockwise (right)
}

void TurnLeft(float angleDegrees)
{
  uint32_t targetPulses = PulsesForAngle(angleDegrees);
  DriveUntil(-1, +1, targetPulses);   // left wheel backward, right wheel forward -> pivots counter-clockwise (left)
}

void MazeSolveStep(void)
{
  uint32_t rightDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4);
  HAL_Delay(10);
  uint32_t frontDist = ReadUltrasonicCM(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8);
  HAL_Delay(10);
  uint32_t leftDist  = ReadUltrasonicCM(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6);

  char msg[64];
  int len = snprintf(msg, sizeof(msg), "STEP R:%lu F:%lu L:%lu\r\n", rightDist, frontDist, leftDist);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

  if (rightDist >= 25 && frontDist >= 25 && leftDist >= 25)
  {
    solved = 1;
    char msg[] = "MAZE SOLVED\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 100);
    return;
  }

  DriveResult result = DriveForwardAdaptive();

  if (result == DRIVE_RESULT_RIGHT_OPEN)
  {
    HAL_Delay(100);
    take_turn_2(1,90.0);
    HAL_Delay(100);
    MoveForward(10);
    HAL_Delay(100);
    return;
  }

  uint32_t leftCheck = ReadUltrasonicCM(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6);

  if (leftCheck > WALL_DETECT_CM)
  {
    HAL_Delay(100);
    take_turn_2(0,90.0);
    HAL_Delay(100);
    MoveForward(10);
    HAL_Delay(100);
  }
  else
  {
    HAL_Delay(100);
    take_turn_2(1,180.0);
    HAL_Delay(100);
    MoveForward(13);
    HAL_Delay(100);
  }
}


static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DelayMicroseconds(uint32_t us)
{
  uint32_t startCycles = DWT->CYCCNT;
  uint32_t delayCycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - startCycles) < delayCycles);
}

uint32_t ReadUltrasonicCM(GPIO_TypeDef* trigPort, uint16_t trigPin, GPIO_TypeDef* echoPort, uint16_t echoPin)
{
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);
  DelayMicroseconds(2);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_SET);
  DelayMicroseconds(10);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);

  uint32_t waitStart = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_RESET)
  {
    if ((DWT->CYCCNT - waitStart) > (SystemCoreClock / 1000) * 30) return 999; // no echo - nothing in range
  }

  uint32_t startCycles = DWT->CYCCNT;
  uint8_t timedOut = 0;
  while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_SET)
  {
    if ((DWT->CYCCNT - startCycles) > (SystemCoreClock / 1000) * 30) { timedOut = 1; break; }
  }
  if (timedOut) return 999;

  uint32_t endCycles = DWT->CYCCNT;
  uint32_t pulseMicros = (endCycles - startCycles) / (SystemCoreClock / 1000000);
  return pulseMicros / 58;
}


UltrasonicReading ReadAllUltrasonic(void)
{
  UltrasonicReading u;
  u.right  = ReadUltrasonicCM(GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4);
  HAL_Delay(10);   // brief pause so sensors don't cross-talk with each other's echoes
  u.left = ReadUltrasonicCM(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_6);
  HAL_Delay(10);
  u.middle   = ReadUltrasonicCM(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8);
  return u;
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
