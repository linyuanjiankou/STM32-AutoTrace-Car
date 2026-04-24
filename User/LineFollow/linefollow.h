/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : linefollow.h
  * @brief          : PID-based line follow control header file
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

#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

/* Exported types ------------------------------------------------------------*/
/* PID controller structure */
typedef struct
{
    float setpoint;      /* Desired value (target deviation = 0) */
    float feedback;      /* Current value (current weighted sum) */
    float error;         /* Current error */
    float error_sum;     /* Sum of errors (integral) */
    float error_prev;    /* Previous error (for derivative) */
    float output;        /* PID output */
} PID_t;

/* Sensor value definition: 0=black line, 1=white surface */
/* For PID calculation: LEFT1, CENTER, RIGHT1 are used */
/* LEFT2, RIGHT2 are only for state switching and extreme case protection */

/* Exported constants --------------------------------------------------------*/
/* Sensor weight definitions - ordered from left to right */
/* Note: Only LEFT1, CENTER, RIGHT1 are used for PID calculation */
/* LEFT2 and RIGHT2 are only used for state switching and extreme case protection */
#define SENSOR_WEIGHT_LEFT1   -1.0f   /* Left inner sensor */
#define SENSOR_WEIGHT_CENTER   0.0f   /* Center sensor */
#define SENSOR_WEIGHT_RIGHT1   1.0f   /* Right inner sensor */

/* Exported parameters - modify these to tune performance */
/* Base speed for both wheels (0-100) */
extern uint16_t g_linefollow_base_speed;

/* Max speed adjustment (0-100) */
extern uint16_t g_linefollow_max_adjust;

/* PID parameters for deviation control */
/* Kp: Proportional gain - higher = more aggressive correction */
extern float g_linefollow_pid_kp;

/* Ki: Integral gain - eliminates steady-state error */
extern float g_linefollow_pid_ki;

/* Kd: Derivative gain - dampens oscillations */
extern float g_linefollow_pid_kd;

/* PID output limit (max speed difference) */
extern float g_linefollow_pid_output_limit;

/* Integral windup limit */
extern float g_linefollow_pid_integral_limit;

/* Control loop period in milliseconds */
extern uint16_t g_linefollow_control_period;

/* Exported function prototypes ---------------------------------------------*/
/* Initialization */
void LineFollow_Init(void);

/* Main control function - call in main loop */
void LineFollow_Update(void);

/* Set base speed (0-100) */
void LineFollow_SetBaseSpeed(uint16_t speed);

/* Set PID parameters */
void LineFollow_SetPID(float kp, float ki, float kd);
void StraightLine_SetPID(void);
void Circle_SetPID(void);
void Curve_SetPID(void);

/* Set PID output limit */
void LineFollow_SetOutputLimit(float limit);

/* Set integral limit (for anti-windup) */
void LineFollow_SetIntegralLimit(float limit);

/* Set control period in ms */
void LineFollow_SetControlPeriod(uint16_t period_ms);

/* Get current weighted sum (for debugging) */
int32_t LineFollow_GetWeightedSum(void);

/* Get current position/deviation (for debugging) */
float LineFollow_GetPosition(void);

/* Get current left/right wheel speeds */
uint16_t LineFollow_GetLeftSpeed(void);
uint16_t LineFollow_GetRightSpeed(void);

/* Get derivative (error_dot) for feedforward calculation */
float LineFollow_GetDerivative(void);

/* Reset integral term (clear accumulated error) */
void LineFollow_ResetIntegral(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINEFOLLOW_H */
