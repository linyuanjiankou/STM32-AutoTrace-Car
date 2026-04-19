/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : triangle.h
  * @brief          : Triangle mode state machine header file
  *                   Implements 3-turn triangular path using sensor-based turning
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

#ifndef __TRIANGLE_H
#define __TRIANGLE_H

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
    STATE_START,   /* Initialization state */
    STATE_TRACK,   /* Normal line following (black line = 0) */
    STATE_TURN,    /* Turning when >= 3 sensors detect black line */
    STATE_LOST,    /* Line lost (all sensors = 1, white surface) */
    STATE_STOP     /* Triangle complete, stop */
} CarState_t;

/* Exported constants --------------------------------------------------------*/
/* Turn direction definitions */
#define TURN_LEFT_DIR   0   /* Left wheel BWD, Right wheel FWD */
#define TURN_RIGHT_DIR  1   /* Left wheel FWD, Right wheel BWD */

/* Turn speed */
#define TURN_SPEED      15

/* Lost line search speed */
#define LOST_SEARCH_SPEED 12

/* Turn completion timeout in ms (safety fallback) */
#define TURN_COMPLETE_TIMEOUT 5000

/* Exported variables --------------------------------------------------------*/
/* Current triangle state */
extern CarState_t g_triangle_current_state;

/* Number of turns completed (0-3) */
extern int g_triangle_turn_count;

/* Exported function prototypes ---------------------------------------------*/
/* Initialize triangle mode */
void Triangle_Init(void);

/* Update triangle mode state machine - call in main loop */
void Triangle_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRIANGLE_H */
