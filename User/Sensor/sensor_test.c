/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_test.c
  * @brief          : Sensor test program for verifying sensor driver functionality
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
#include "sensor_test.h"
#include "sys.h"

/* USER CODE BEGIN 0 */

/* Private variables ---------------------------------------------------------*/
static uint32_t s_test_tick = 0;

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  Sensor test initialization
  * @note   Called in main() after SENSOR_Init()
  * @retval None
  */
void Sensor_Test_Init(void)
{
    s_test_tick = HAL_GetTick();

    /* Print test information */
    printf("=== Sensor Test Init ===\r\n");
    printf("Sensors: [L2,L1,C,R1,R2]\r\n");
    printf("1 = black line, 0 = white surface\r\n\r\n");
}

/**
  * @brief  Sensor test task - call this in main loop
  * @note   Tests sensor functionality and outputs data via USART
  * @retval None
  */
void Sensor_Test_Task(void)
{
    uint32_t current_tick = HAL_GetTick();

    /* Read and output sensor data every 100ms */
    if (current_tick - s_test_tick >= 100)
    {
        s_test_tick = current_tick;

        /* Check if new sensor data is available */
        if (SENSOR_CheckNewData())
        {
            SENSOR_Status_t sensors;

            /* Read sensor data */
            SENSOR_ReadRaw(&sensors);

            /* Output sensor data via USART */
            printf("[%d%d%d%d%d] ",
                   sensors.LEFT2, sensors.LEFT1, sensors.CENTER,
                   sensors.RIGHT1, sensors.RIGHT2);
            printf("L2:%d L1:%d C:%d R1:%d R2:%d\r\n",
                   sensors.LEFT2, sensors.LEFT1, sensors.CENTER,
                   sensors.RIGHT1, sensors.RIGHT2);
        }
    }
}

/**
  * @brief  Sensor interrupt test - verify INT pins work
  * @note   This function prints test information via USART
  * @retval None
  */
void Sensor_Test_Interrupt(void)
{
    printf("=== Sensor Interrupt Test ===\r\n");
    printf("All sensors should be HIGH (1) when over black line\r\n");
    printf("Move sensor over white surface to test LOW (0) detection\r\n\r\n");
}

/**
  * @brief  Sensor line position test - basic line detection
  * @note   Tests if sensor can detect line position
  * @retval None
  */
void Sensor_Test_LineDetect(void)
{
    SENSOR_Status_t sensors;
    uint8_t line_position = 0;
    uint8_t line_detected = 0;

    SENSOR_ReadRaw(&sensors);

    /* Check if any sensor detects line (1 = black line) */
    if (sensors.LEFT2 == 1 || sensors.LEFT1 == 1 ||
        sensors.CENTER == 1 || sensors.RIGHT1 == 1 || sensors.RIGHT2 == 1)
    {
        line_detected = 1;

        /* Calculate rough line position */
        if (sensors.CENTER == 1)
        {
            line_position = 2;  /* Center */
        }
        else if (sensors.LEFT1 == 1 || sensors.LEFT2 == 1)
        {
            line_position = 1;  /* Left side */
        }
        else if (sensors.RIGHT1 == 1 || sensors.RIGHT2 == 1)
        {
            line_position = 3;  /* Right side */
        }
    }

    printf("Line Detected: %s, Position: %d\r\n",
           line_detected ? "YES" : "NO", line_position);
}

/* USER CODE END 1 */
