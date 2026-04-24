/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : circle.h
  * @brief          : Circle mode state machine header file
  *                   Implements circular track:
  *                   - Uses LEFT1, CENTER, RIGHT1 for line tracking
  *                   - Counts LEFT2 and RIGHT2 black line detections
  *                   - Stops after 3 detections
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

#ifndef __CIRCLE_H
#define __CIRCLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "motor.h"
#include "mode.h"

/* Exported types ------------------------------------------------------------*/
/* State machine enumeration */
typedef enum {
    CIRCLE_STATE_START,     /* Initialization state */
    CIRCLE_STATE_TRACK,     /* Line following using LEFT1, CENTER, RIGHT1 */
    CIRCLE_STATE_LOST,      /* Line lost, searching */
    CIRCLE_STATE_STOP       /* 3 detections complete, stop */
} CircleState_t;

/* Exported constants --------------------------------------------------------*/
/* Circle mode parameters - adjust these for circle mode tuning */
#define CIRCLE_BASE_SPEED       30      /* Base speed for both wheels */
#define CIRCLE_COUNT_TARGET     3       /* Number of LEFT2/RIGHT2 detections before stopping */

/* Exported variables --------------------------------------------------------*/
/* Current circle state */
extern CircleState_t g_circle_current_state;

/* Number of LEFT2/RIGHT2 black line detections */
extern int g_circle_detection_count;

/* Exported function prototypes ---------------------------------------------*/
/* Initialize circle mode */
void Circle_Init(void);

/* Update circle mode state machine - call in main loop */
void Circle_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCLE_H */
