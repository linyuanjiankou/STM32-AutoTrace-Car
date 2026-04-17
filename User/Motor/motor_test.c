/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_test.c
  * @brief   Motor driver test program
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
#include "sys.h"
#include "motor.h"

/* USER CODE BEGIN 0 */

/* Private function prototypes -----------------------------------------------*/
static void Motor_Test_Speed_Control(void);
static void Motor_Test_Direction_Control(void);
static void Motor_Test_Encoder_Read(void);
static void Motor_Test_Run_Stop(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Test speed control function
  */
static void Motor_Test_Speed_Control(void)
{
    printf("\r\n=== Motor Speed Control Test ===\r\n");

    /* Test Motor A at different speeds */
    printf("Motor A at 30%% speed forward...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_FWD, MOTOR_SPEED_LOW);
    HAL_Delay(2000);

    printf("Motor A at 50%% speed forward...\r\n");
    Motor_SetSpeed(MOTOR_ID_A, MOTOR_SPEED_MEDIUM);
    HAL_Delay(2000);

    printf("Motor A at 70%% speed forward...\r\n");
    Motor_SetSpeed(MOTOR_ID_A, MOTOR_SPEED_HIGH);
    HAL_Delay(2000);

    /* Test Motor B at different speeds */
    printf("Motor B at 40%% speed backward...\r\n");
    Motor_Run(MOTOR_ID_B, MOTOR_BWD, 40);
    HAL_Delay(2000);

    printf("Motor B at 60%% speed backward...\r\n");
    Motor_SetSpeed(MOTOR_ID_B, 60);
    HAL_Delay(2000);

    /* Stop motors */
    Motor_Stop(MOTOR_ID_A);
    Motor_Stop(MOTOR_ID_B);
    printf("Motors stopped.\r\n");
}

/**
  * @brief  Test direction control function
  */
static void Motor_Test_Direction_Control(void)
{
    printf("\r\n=== Motor Direction Control Test ===\r\n");

    /* Test Motor A direction changes */
    printf("Motor A forward...\r\n");
    Motor_SetDirection(MOTOR_ID_A, MOTOR_FWD);
    Motor_SetSpeed(MOTOR_ID_A, 50);
    HAL_Delay(1000);

    printf("Motor A backward...\r\n");
    Motor_SetDirection(MOTOR_ID_A, MOTOR_BWD);
    HAL_Delay(1000);

    printf("Motor A stop...\r\n");
    Motor_SetDirection(MOTOR_ID_A, MOTOR_STOP);
    HAL_Delay(1000);

    /* Test Motor B direction changes */
    printf("Motor B forward...\r\n");
    Motor_SetDirection(MOTOR_ID_B, MOTOR_FWD);
    Motor_SetSpeed(MOTOR_ID_B, 50);
    HAL_Delay(1000);

    printf("Motor B backward...\r\n");
    Motor_SetDirection(MOTOR_ID_B, MOTOR_BWD);
    HAL_Delay(1000);

    printf("Motor B stop...\r\n");
    Motor_SetDirection(MOTOR_ID_B, MOTOR_STOP);
}

/**
  * @brief  Test encoder read function
  */
static void Motor_Test_Encoder_Read(void)
{
  uint32_t count_a, count_b;
  uint32_t rpm_a, rpm_b;
  uint32_t total_count_a, total_count_b;

  printf("\r\n=== Encoder Read Test ===\r\n");

  /* Reset encoder counts */
  Motor_ResetEncoderCount(MOTOR_ID_A);
  Motor_ResetEncoderCount(MOTOR_ID_B);
  printf("Encoder counts reset.\r\n");

  /* Start motors */
  Motor_Run(MOTOR_ID_A, MOTOR_FWD, 50);
  Motor_Run(MOTOR_ID_B, MOTOR_FWD, 50);
  printf("Motors running forward at 50%% speed.\r\n");

  /* Read encoder values periodically */
  for (uint8_t i = 0; i < 10; i++)
  {
      HAL_Delay(500);

      /* Update encoder counts */
      Motor_UpdateEncoderCount();

      /* Get encoder counts */
      count_a = Motor_GetEncoderCount(MOTOR_ID_A);
      count_b = Motor_GetEncoderCount(MOTOR_ID_B);
      if (!count_b) printf("Encoder B count is zero.\r\n");
      /* Update speed in RPM */
      Motor_UpdateSpeedRPM();
      rpm_a = Motor_GetSpeedRPM(MOTOR_ID_A);
      rpm_b = Motor_GetSpeedRPM(MOTOR_ID_B);

      /* Print values */
      printf("Count A: %lu, Count B: %lu\r\n", count_a, count_b);
      printf("RPM A: %lu (actual: %lu.%03lu), RPM B: %lu (actual: %lu.%03lu)\r\n",
              rpm_a, rpm_a / 1000, rpm_a % 1000,
              rpm_b, rpm_b / 1000, rpm_b % 1000);
      printf("--------------------------------------------------\r\n");
  }

  /* Stop motors */
  Motor_Stop(MOTOR_ID_A);
  Motor_Stop(MOTOR_ID_B);
  printf("Motors stopped.\r\n");
}

/**
  * @brief  Test run and stop function
  */
static void Motor_Test_Run_Stop(void)
{
    printf("\r\n=== Motor Run/Stop Test ===\r\n");

    printf("Running both motors forward at 60%% speed...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_FWD, 60);
    Motor_Run(MOTOR_ID_B, MOTOR_FWD, 60);

    HAL_Delay(2000);

    printf("Running both motors backward at 40%% speed...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_BWD, 40);
    Motor_Run(MOTOR_ID_B, MOTOR_BWD, 40);

    HAL_Delay(2000);

    printf("Stopping Motor A...\r\n");
    Motor_Stop(MOTOR_ID_A);

    HAL_Delay(1000);

    printf("Stopping Motor B...\r\n");
    Motor_Stop(MOTOR_ID_B);

    HAL_Delay(1000);

    /* Test individual motor stop */
    printf("Running Motor A at 70%% speed...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_FWD, 70);

    HAL_Delay(1500);

    printf("Individual Motor_Stop test...\r\n");
    Motor_Stop(MOTOR_ID_A);
}

/* USER CODE END 0 */

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  Run all motor tests
  * @note   Call this function in main() after Motor_Init()
  */
void Motor_Run_Tests(void)
{
    printf("\r\n========================================\r\n");
    printf("       Motor Driver Test Suite         \r\n");
    printf("========================================\r\n");

    /* Test 1: Speed Control */
    Motor_Test_Speed_Control();

    /* Test 2: Direction Control */
    Motor_Test_Direction_Control();

    /* Test 3: Encoder Read */
    Motor_Test_Encoder_Read();

    /* Test 4: Run/Stop */
    Motor_Test_Run_Stop();

    printf("\r\n========================================\r\n");
    printf("       All Tests Completed!            \r\n");
    printf("========================================\r\n");
}

/**
  * @brief  Run basic motor test (single function call)
  * @note   Runs a simple forward-backward cycle
  */
void Motor_Basic_Test(void)
{
    printf("\r\n=== Basic Motor Test ===\r\n");

    /* Reset encoders */
    Motor_ResetEncoderCount(MOTOR_ID_A);
    Motor_ResetEncoderCount(MOTOR_ID_B);

    /* Run forward */
    printf("Both motors forward at 50%% speed (2s)...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_FWD, 50);
    Motor_Run(MOTOR_ID_B, MOTOR_FWD, 50);
    HAL_Delay(2000);

    /* Run backward */
    printf("Both motors backward at 40%% speed (2s)...\r\n");
    Motor_Run(MOTOR_ID_A, MOTOR_BWD, 40);
    Motor_Run(MOTOR_ID_B, MOTOR_BWD, 40);
    HAL_Delay(2000);

    /* Stop */
    printf("Stopping motors...\r\n");
    Motor_Stop(MOTOR_ID_A);
    Motor_Stop(MOTOR_ID_B);

    printf("Motor test completed.\r\n");
}

/* USER CODE END 1 */
