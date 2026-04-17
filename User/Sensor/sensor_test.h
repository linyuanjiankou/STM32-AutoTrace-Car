/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_test.h
  * @brief          : Sensor test header file
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

#ifndef __SENSOR_TEST_H
#define __SENSOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/* Exported functions prototypes ---------------------------------------------*/
void Sensor_Test_Init(void);
void Sensor_Test_Task(void);
void Sensor_Test_Interrupt(void);
void Sensor_Test_LineDetect(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TEST_H */
