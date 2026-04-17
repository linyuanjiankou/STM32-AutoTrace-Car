/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor.c
  * @brief          : 5-channel infrared line tracking sensor driver implementation
  *                   Using interrupt mode with dual-edge triggering
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
#include "sensor.h"
#include "stm32f1xx_hal_gpio.h"

/* USER CODE BEGIN 0 */

/* Private variables ---------------------------------------------------------*/
/* Sensor status cache - updated by interrupt service routines */
static SENSOR_Status_t SENSOR_Data = {0};

/* Flag to indicate new sensor data is available */
static volatile uint8_t SENSOR_NewDataFlag = 0;

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize sensor driver
  * @note   This function initializes the sensor state and clears the new data flag
  *         GPIO pins are already configured in MX_GPIO_Init() with interrupt mode
  * @retval None
  */
void SENSOR_Init(void)
{
    /* Clear sensor data */
    SENSOR_Data.LEFT2   = 1;  /* Default to high (white surface) due to pull-up */
    SENSOR_Data.LEFT1   = 1;
    SENSOR_Data.CENTER  = 1;
    SENSOR_Data.RIGHT1  = 1;
    SENSOR_Data.RIGHT2  = 1;

    /* Clear new data flag */
    SENSOR_NewDataFlag = 0;
}

/**
  * @brief  Get latest sensor data
  * @note   This function reads all sensor pins directly to ensure consistent data
  *         for all 5 channels at the time of read.
  * @param  data: Pointer to SENSOR_Status_t structure to store readings
  * @retval None
  */
void SENSOR_ReadRaw(SENSOR_Status_t *data)
{
    if (data != NULL)
    {
        data->LEFT2  = HAL_GPIO_ReadPin(SENSOR_LEFT2_PORT, SENSOR_LEFT2_PIN);
        data->LEFT1  = HAL_GPIO_ReadPin(SENSOR_LEFT1_PORT, SENSOR_LEFT1_PIN);
        data->CENTER = HAL_GPIO_ReadPin(SENSOR_CENTER_PORT, SENSOR_CENTER_PIN);
        data->RIGHT1 = HAL_GPIO_ReadPin(SENSOR_RIGHT1_PORT, SENSOR_RIGHT1_PIN);
        data->RIGHT2 = HAL_GPIO_ReadPin(SENSOR_RIGHT2_PORT, SENSOR_RIGHT2_PIN);
    }
}

/**
  * @brief  Check if new sensor data is available
  * @note   This function reads and clears the new data flag.
  *         Should be called before reading sensor data in main loop.
  * @retval 1: New data available, 0: No new data
  */
uint8_t SENSOR_CheckNewData(void)
{
    uint8_t flag = SENSOR_NewDataFlag;
    SENSOR_NewDataFlag = 0;  /* Clear flag after reading */
    return flag;
}

/* USER CODE BEGIN 1 */

/* HAL_GPIO_EXTI_Callback() - Called by HAL_GPIO_EXTI_IRQHandler() */
/**
  * @brief  GPIO EXTI callback function
  * @note   This function is called by HAL_GPIO_EXTI_IRQHandler() when an EXTI
  *         interrupt occurs. We use this to update sensor state.
  * @param  GPIO_Pin: The port pin number that triggered the interrupt
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case SENSOR_CENTER_PIN:
            SENSOR_Data.CENTER = HAL_GPIO_ReadPin(SENSOR_CENTER_PORT, SENSOR_CENTER_PIN);
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_RIGHT2_PIN:
            SENSOR_Data.RIGHT2 = HAL_GPIO_ReadPin(SENSOR_RIGHT2_PORT, SENSOR_RIGHT2_PIN);
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_RIGHT1_PIN:
            SENSOR_Data.RIGHT1 = HAL_GPIO_ReadPin(SENSOR_RIGHT1_PORT, SENSOR_RIGHT1_PIN);
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_LEFT1_PIN:
            SENSOR_Data.LEFT1 = HAL_GPIO_ReadPin(SENSOR_LEFT1_PORT, SENSOR_LEFT1_PIN);
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_LEFT2_PIN:
            SENSOR_Data.LEFT2 = HAL_GPIO_ReadPin(SENSOR_LEFT2_PORT, SENSOR_LEFT2_PIN);
            SENSOR_NewDataFlag = 1;
            break;

        default:
            break;
    }
}

/* USER CODE END 1 */
