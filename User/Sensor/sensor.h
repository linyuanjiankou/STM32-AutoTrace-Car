/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor.h
  * @brief          : 5-channel infrared line tracking sensor driver header
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

#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Sensor status structure - stores all 5 channel readings */
/* Sensor value: 1 = black line, 0 = white surface */
typedef struct
{
    uint8_t LEFT2;    /* PC14 - 1: black line, 0: white surface */
    uint8_t LEFT1;    /* PC13 - 1: black line, 0: white surface */
    uint8_t CENTER;   /* PB1 - 1: black line, 0: white surface */
    uint8_t RIGHT1;   /* PA3 - 1: black line, 0: white surface */
    uint8_t RIGHT2;   /* PA2 - 1: black line, 0: white surface */
} SENSOR_Status_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Sensor pin definitions - ordered from left to right */
/* Left outer sensor */
#define SENSOR_LEFT2_PIN          GPIO_PIN_14
#define SENSOR_LEFT2_PORT         GPIOC

/* Left inner sensor */
#define SENSOR_LEFT1_PIN          GPIO_PIN_13
#define SENSOR_LEFT1_PORT         GPIOC

/* Center sensor */
#define SENSOR_CENTER_PIN         GPIO_PIN_1
#define SENSOR_CENTER_PORT        GPIOB

/* Right inner sensor */
#define SENSOR_RIGHT1_PIN         GPIO_PIN_3
#define SENSOR_RIGHT1_PORT        GPIOA

/* Right outer sensor */
#define SENSOR_RIGHT2_PIN         GPIO_PIN_2
#define SENSOR_RIGHT2_PORT        GPIOA

/* Sensor read value definitions */
#define SENSOR_LINE_DETECTED      0x01    /* Black line detected (HIGH) */
#define SENSOR_NO_LINE            0x00    /* White surface detected (LOW) */

/* USER CODE END EC */

/* Exported function prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* Sensor initialization */
void SENSOR_Init(void);

/* Sensor reading functions */
void SENSOR_ReadRaw(SENSOR_Status_t *data);              /* Read all 5 channels */

/* Helper functions */
uint8_t SENSOR_CheckNewData(void);                       /* Check if new data available */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H */
