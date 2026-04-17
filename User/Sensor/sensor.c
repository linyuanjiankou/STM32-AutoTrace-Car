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
  * @note   This function initializes the sensor state by reading pins directly
  *         GPIO pins are already configured in MX_GPIO_Init() with interrupt mode
  *         Sensor hardware reads: 0=black line, 1=white surface
  *         We invert here to present: 1=black line, 0=white surface to upper layers
  * @retval None
  */
void SENSOR_Init(void)
{
    /* Initial read to set initial state (invert to get 1=black line) */
    SENSOR_Data.LEFT2  = (HAL_GPIO_ReadPin(SENSOR_LEFT2_PORT, SENSOR_LEFT2_PIN) == 0) ? 1 : 0;
    SENSOR_Data.LEFT1  = (HAL_GPIO_ReadPin(SENSOR_LEFT1_PORT, SENSOR_LEFT1_PIN) == 0) ? 1 : 0;
    SENSOR_Data.CENTER = (HAL_GPIO_ReadPin(SENSOR_CENTER_PORT, SENSOR_CENTER_PIN) == 0) ? 1 : 0;
    SENSOR_Data.RIGHT1 = (HAL_GPIO_ReadPin(SENSOR_RIGHT1_PORT, SENSOR_RIGHT1_PIN) == 0) ? 1 : 0;
    SENSOR_Data.RIGHT2 = (HAL_GPIO_ReadPin(SENSOR_RIGHT2_PORT, SENSOR_RIGHT2_PIN) == 0) ? 1 : 0;

    /* Clear new data flag */
    SENSOR_NewDataFlag = 0;
}

/**
  * @brief  Get latest sensor data from cache
  * @note   This function returns the sensor status that was last updated
  *         by the interrupt service routine. The data represents the sensor
  *         state at the time of the last edge detection.
  * @param  data: Pointer to SENSOR_Status_t structure to store readings
  * @retval None
  */
void SENSOR_ReadRaw(SENSOR_Status_t *data)
{
    if (data != NULL)
    {
        data->LEFT2  = SENSOR_Data.LEFT2;
        data->LEFT1  = SENSOR_Data.LEFT1;
        data->CENTER = SENSOR_Data.CENTER;
        data->RIGHT1 = SENSOR_Data.RIGHT1;
        data->RIGHT2 = SENSOR_Data.RIGHT2;
    }
}

/**
  * @brief  Check if new sensor data is available
  * @note   This function reads and clears the new data flag.
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
  *         interrupt occurs. We use this to update sensor state in cache.
  *         Sensor hardware: 0=black line, 1=white surface
  *         We invert here to present: 1=black line, 0=white surface to upper layers
  * @param  GPIO_Pin: The port pin number that triggered the interrupt
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case SENSOR_CENTER_PIN:
            SENSOR_Data.CENTER = (HAL_GPIO_ReadPin(SENSOR_CENTER_PORT, SENSOR_CENTER_PIN) == 0) ? 1 : 0;
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_RIGHT2_PIN:
            SENSOR_Data.RIGHT2 = (HAL_GPIO_ReadPin(SENSOR_RIGHT2_PORT, SENSOR_RIGHT2_PIN) == 0) ? 1 : 0;
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_RIGHT1_PIN:
            SENSOR_Data.RIGHT1 = (HAL_GPIO_ReadPin(SENSOR_RIGHT1_PORT, SENSOR_RIGHT1_PIN) == 0) ? 1 : 0;
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_LEFT1_PIN:
            SENSOR_Data.LEFT1 = (HAL_GPIO_ReadPin(SENSOR_LEFT1_PORT, SENSOR_LEFT1_PIN) == 0) ? 1 : 0;
            SENSOR_NewDataFlag = 1;
            break;

        case SENSOR_LEFT2_PIN:
            SENSOR_Data.LEFT2 = (HAL_GPIO_ReadPin(SENSOR_LEFT2_PORT, SENSOR_LEFT2_PIN) == 0) ? 1 : 0;
            SENSOR_NewDataFlag = 1;
            break;

        default:
            break;
    }
}

/* USER CODE END 1 */
