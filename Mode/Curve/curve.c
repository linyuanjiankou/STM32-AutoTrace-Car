/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : curve.c
  * @brief          : Curve mode implementation with PID parameter switching
  *                   Uses weighted sum from LineFollow for curve detection:
  *                   weighted_sum = LEFT2*(-2) + LEFT1*(-1) + CENTER*0 + RIGHT1*1 + RIGHT2*2
  *                   Range: -5 (far left) to +5 (far right)
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
#include "curve.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* PID configuration structure for each curve state */
typedef struct {
    float kp;                /* Proportional gain */
    float ki;                /* Integral gain */
    float kd;                /* Derivative gain */
    uint16_t base_speed;     /* Base speed for both wheels */
    float output_limit;      /* PID output limit */
} CurvePIDConfig_t;

/* Private variables ---------------------------------------------------------*/
/* Current state */
CurveState_t g_curve_current_state = CURVE_STATE_STRAIGHT;

/* PID configuration for each curve state - configured via macros in curve.h */
static const CurvePIDConfig_t s_curve_pid_configs[CURVE_STATE_COUNT] = {
    /* Straight line tracking - aggressive correction for high speed */
    [CURVE_STATE_STRAIGHT] = {
        .kp = CURVE_STRAIGHT_KP,
        .ki = CURVE_STRAIGHT_KI,
        .kd = CURVE_STRAIGHT_KD,
        .base_speed = CURVE_STRAIGHT_SPEED,
        .output_limit = CURVE_STRAIGHT_OUTPUT_LIMIT
    },
    /* Gentle curve - moderate correction */
    [CURVE_STATE_GENTLE] = {
        .kp = CURVE_GENTLE_KP,
        .ki = CURVE_GENTLE_KI,
        .kd = CURVE_GENTLE_KD,
        .base_speed = CURVE_GENTLE_SPEED,
        .output_limit = CURVE_GENTLE_OUTPUT_LIMIT
    },
    /* Sharp curve - conservative correction for low speed */
    [CURVE_STATE_SHARP] = {
        .kp = CURVE_SHARP_KP,
        .ki = CURVE_SHARP_KI,
        .kd = CURVE_SHARP_KD,
        .base_speed = CURVE_SHARP_SPEED,
        .output_limit = CURVE_SHARP_OUTPUT_LIMIT
    }
};

/* Last update tick for state machine */
static uint32_t s_last_update_tick = 0;

/* Private function prototypes -----------------------------------------------*/
static CurveState_t Curve_DetectState(int32_t weighted_sum);
static void Curve_UpdateCurveMode(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Detect curve state based on weighted sum with hysteresis
  * @param  weighted_sum: Weighted sum from LineFollow (-5 to +5)
  * @retval CurveState_t: Detected curve state
  *
  * State transitions with hysteresis to prevent frequent switching:
  *
  * Sharp Curve (|weighted_sum| >= 4)
  *    |
  *    v (when |weighted_sum| < 3)
  * Gentle Curve (2 <= |weighted_sum| < 4)
  *    |
  *    v (when |weighted_sum| < 1)
  * Straight (|weighted_sum| < 1)
  */
static CurveState_t Curve_DetectState(int32_t weighted_sum)
{
    CurveState_t new_state = g_curve_current_state;
    int32_t abs_weighted_sum = (weighted_sum < 0) ? -weighted_sum : weighted_sum;

    /* State machine with hysteresis using macros to prevent frequent switching */
    switch (g_curve_current_state) {
        case CURVE_STATE_STRAIGHT:
            /* Transition to gentle curve if deviation is moderate (|weighted_sum| >= 2) */
            if (abs_weighted_sum >= CURVE_STRAIGHT_TO_GENTLE_THRESHOLD &&
                abs_weighted_sum < CURVE_GENTLE_TO_SHARP_THRESHOLD) {
                new_state = CURVE_STATE_GENTLE;
            }
            /* Transition to sharp curve if deviation is large (|weighted_sum| >= 4) */
            else if (abs_weighted_sum >= CURVE_GENTLE_TO_SHARP_THRESHOLD) {
                new_state = CURVE_STATE_SHARP;
            }
            break;

        case CURVE_STATE_GENTLE:
            /* Transition to straight if deviation is small (|weighted_sum| < 1) */
            if (abs_weighted_sum < CURVE_GENTLE_TO_STRAIGHT_THRESHOLD) {
                new_state = CURVE_STATE_STRAIGHT;
            }
            /* Transition to sharp curve if deviation is large (|weighted_sum| >= 4) */
            else if (abs_weighted_sum >= CURVE_GENTLE_TO_SHARP_THRESHOLD) {
                new_state = CURVE_STATE_SHARP;
            }
            break;

        case CURVE_STATE_SHARP:
            /* Transition to gentle curve if deviation is moderate (2 <= |weighted_sum| < 4) */
            if (abs_weighted_sum >= CURVE_STRAIGHT_TO_GENTLE_THRESHOLD &&
                abs_weighted_sum < CURVE_GENTLE_TO_SHARP_THRESHOLD) {
                new_state = CURVE_STATE_GENTLE;
            }
            /* Transition to straight if centered (|weighted_sum| < 1) */
            else if (abs_weighted_sum < CURVE_GENTLE_TO_STRAIGHT_THRESHOLD) {
                new_state = CURVE_STATE_STRAIGHT;
            }
            break;

        default:
            /* Unknown state - reset to straight */
            new_state = CURVE_STATE_STRAIGHT;
            break;
    }

    return new_state;
}

/**
  * @brief  Update curve mode with PID parameter switching
  * @retval None
  */
static void Curve_UpdateCurveMode(void)
{
    SENSOR_Status_t sensors;
    int32_t weighted_sum;
    CurveState_t detected_state;

    /* Read sensor data (0=black line, 1=white surface) */
    SENSOR_ReadRaw(&sensors);

    /* Calculate weighted sum using LineFollow's calculation */
    weighted_sum = LineFollow_GetWeightedSum();

    /* Detect current curve state based on weighted sum */
    detected_state = Curve_DetectState(weighted_sum);

    /* Update state if changed */
    if (g_curve_current_state != detected_state) {
        LineFollow_ResetIntegral();
        g_curve_current_state = detected_state;

        /* Apply new PID parameters for the detected state */
        LineFollow_SetPID(
            s_curve_pid_configs[detected_state].kp,
            s_curve_pid_configs[detected_state].ki,
            s_curve_pid_configs[detected_state].kd
        );
        LineFollow_SetBaseSpeed(s_curve_pid_configs[detected_state].base_speed);
        LineFollow_SetOutputLimit(s_curve_pid_configs[detected_state].output_limit);
    }

    /* Call LineFollow_Update to perform actual line following with current PID */
    LineFollow_Update();
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize curve mode
  * @retval None
  */
void Curve_Init(void)
{
    g_curve_current_state = CURVE_STATE_STRAIGHT;
    s_last_update_tick = HAL_GetTick();

    /* Initialize with straight line PID parameters */
    LineFollow_SetPID(
        s_curve_pid_configs[CURVE_STATE_STRAIGHT].kp,
        s_curve_pid_configs[CURVE_STATE_STRAIGHT].ki,
        s_curve_pid_configs[CURVE_STATE_STRAIGHT].kd
    );
    LineFollow_SetBaseSpeed(s_curve_pid_configs[CURVE_STATE_STRAIGHT].base_speed);
    LineFollow_SetOutputLimit(s_curve_pid_configs[CURVE_STATE_STRAIGHT].output_limit);

    /* Initialize LineFollow subsystem */
    LineFollow_Init();
}

/**
  * @brief  Update curve mode state machine
  * @note   Call this function in main loop
  * @retval None
  */
void Curve_Update(void)
{
    Curve_UpdateCurveMode();
}
