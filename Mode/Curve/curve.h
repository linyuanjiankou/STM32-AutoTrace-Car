/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : curve.h
  * @brief          : Curve mode header file
  *                   Implements three-state PID switching for curve handling:
  *                   - Straight: aggressive Kp for fast correction
  *                   - Gentle: moderate Kp for mild curves
  *                   - Sharp: conservative Kp for sharp turns
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

#ifndef __CURVE_H
#define __CURVE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "motor.h"
#include "linefollow.h"
#include "mode.h"

/* Exported types ------------------------------------------------------------*/
/* State machine enumeration */
typedef enum {
    CURVE_STATE_STRAIGHT,   /* Line centered, use PID set 1 (aggressive Kp) */
    CURVE_STATE_GENTLE,     /* Mild deviation, use PID set 2 (moderate Kp) */
    CURVE_STATE_SHARP,      /* Large deviation, use PID set 3 (safe Kp for sharp turns) */
    CURVE_STATE_COUNT       /* Total number of states (not a real state) */
} CurveState_t;

/* Exported constants --------------------------------------------------------*/
/* Curve detection thresholds based on weighted sum range (-5 to +5) */
/* Straight: -1 <= weighted_sum <= 1 */
#define CURVE_STRAIGHT_THRESHOLD_LOW   -1
#define CURVE_STRAIGHT_THRESHOLD_HIGH   1

/* Gentle curve: -3 <= weighted_sum <= -2 OR 2 <= weighted_sum <= 3 */
#define CURVE_GENTLE_THRESHOLD_LOW     -3
#define CURVE_GENTLE_THRESHOLD_HIGH     3

/* Sharp curve: weighted_sum <= -4 OR weighted_sum >= 4 */
#define CURVE_SHARP_THRESHOLD_LOW      -4
#define CURVE_SHARP_THRESHOLD_HIGH     4

/* PID parameters for Straight line tracking (high speed for acceleration) */
#define CURVE_STRAIGHT_KP         4.0f
#define CURVE_STRAIGHT_KI         0.0f
#define CURVE_STRAIGHT_KD         0.1f
#define CURVE_STRAIGHT_SPEED      15
#define CURVE_STRAIGHT_OUTPUT_LIMIT  40.0f

/* PID parameters for Gentle curve (moderate speed) */
#define CURVE_GENTLE_KP           3.0f
#define CURVE_GENTLE_KI           0.0f
#define CURVE_GENTLE_KD           0.05f
#define CURVE_GENTLE_SPEED        10
#define CURVE_GENTLE_OUTPUT_LIMIT    35.0f

/* PID parameters for Sharp curve (low speed for deceleration) */
#define CURVE_SHARP_KP            2.0f
#define CURVE_SHARP_KI            0.0f
#define CURVE_SHARP_KD            0.0f
#define CURVE_SHARP_SPEED         5
#define CURVE_SHARP_OUTPUT_LIMIT     30.0f

/* Hysteresis thresholds to prevent frequent state switching */
/* Transition from STRAIGHT to GENTLE requires crossing this boundary */
#define CURVE_STRAIGHT_TO_GENTLE_THRESHOLD  2
/* Transition from GENTLE to SHARP requires crossing this boundary */
#define CURVE_GENTLE_TO_SHARP_THRESHOLD     4
/* Transition from SHARP to GENTLE requires crossing this boundary */
#define CURVE_SHARP_TO_GENTLE_THRESHOLD     3
/* Transition from GENTLE to STRAIGHT requires crossing this boundary */
#define CURVE_GENTLE_TO_STRAIGHT_THRESHOLD  1

/* Default state to enter on startup */
#define CURVE_DEFAULT_STATE   CURVE_STATE_STRAIGHT

/* Control period in milliseconds */
#define CURVE_CONTROL_PERIOD_MS   20

/* Exported variables --------------------------------------------------------*/
/* Current curve state */
extern CurveState_t g_curve_current_state;

/* Exported function prototypes ---------------------------------------------*/
/* Initialize curve mode */
void Curve_Init(void);

/* Update curve mode state machine - call in main loop */
void Curve_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __CURVE_H */
