/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : linefollowsequence.c
  * @brief          : Line Follow Sequence mode implementation
  *                   Sensor-driven state machine with comprehensive line tracking
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
#include "linefollowsequence.h"
#include "linefollow.h"
#include "motor.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
LFS_State_t g_lfs_current_state = LFS_STATE_STRAIGHT;

/* Detection count for dashed line states */
int g_lfs_detection_count = 0;

/* State transition tracking variables */
uint32_t g_lfs_stable_start_tick = 0;
uint8_t g_lfs_intersection_count = 0;

/* Circle state variables - encoder-based transition */
uint32_t g_lfs_circle_start_count = 0;
uint32_t g_lfs_circle_target_count = LFS_CIRCLE_TARGET_COUNT;

/*Straight to Circle transition tracking */
uint32_t g_lfs_straight_to_circle_count = LFS_STRAIGHT_TO_CIRCLE_COUNT;

/* Private function prototypes -----------------------------------------------*/
static void LFS_EnterState(LFS_State_t new_state);
static uint8_t LFS_IsAllWhite(SENSOR_Status_t *sensors);

/* Exported functions --------------------------------------------------------*/
void LFS_Init(void)
{
    g_lfs_current_state = LFS_STATE_STRAIGHT;
    g_lfs_stable_start_tick = HAL_GetTick();
    g_lfs_detection_count = 0;
    g_lfs_circle_start_count = 0;
    g_lfs_circle_target_count = LFS_CIRCLE_TARGET_COUNT;
    g_lfs_straight_to_circle_count = LFS_STRAIGHT_TO_CIRCLE_COUNT;

    /* Initialize with straight tracking */
    // StraightLine_SetPID();
    // LineFollow_SetBaseSpeed(LFS_STRAIGHT_SPEED);
    // LineFollow_SetOutputLimit(LFS_STRAIGHT_OUTPUT);
    Motor_Run(MOTOR_ID_A, MOTOR_FWD, 20);
    Motor_Run(MOTOR_ID_B, MOTOR_FWD, 20);
}

void LFS_Update(void)
{
    SENSOR_Status_t sensors;

    SENSOR_ReadRaw(&sensors);

    switch (g_lfs_current_state) {
        case LFS_STATE_START:
            /* Wait for all balck to transition to straight*/
            if (LFS_IsAllBlack(&sensors)) {
                LFS_EnterState(LFS_STATE_STRAIGHT);
            }
            break;

        case LFS_STATE_STRAIGHT:
            StraightLine_SetPID();
            LineFollow_SetBaseSpeed(LFS_STRAIGHT_SPEED);
            LineFollow_SetOutputLimit(LFS_STRAIGHT_OUTPUT);
            LineFollow_Update();

            /* Encoder-based transition to Circle state */
            Motor_UpdateEncoderCount();
            if (Motor_GetEncoderCount(MOTOR_ID_A) >= g_lfs_straight_to_circle_count) {
                g_lfs_circle_start_count = Motor_GetEncoderCount(MOTOR_ID_A);
                g_lfs_circle_target_count = LFS_CIRCLE_TO_STRAIGHT_COUNT;
                LFS_EnterState(LFS_STATE_CIRCLE);
            }

            /* Transition: detect 4+ black lines to enter rotation */
            if (LFS_GetAbsDeviation() > LFS_LARGE_DEV_THRESHOLD ||
                LFS_IsIntersection(&sensors)) {
                LFS_EnterState(LFS_STATE_ROTATION);
            }

            /* Transition: detect white to enter dash_straight*/
            if (LFS_IsAllWhite(&sensors)) {
                LFS_EnterState(LFS_STATE_DASH_STRAIGHT_1);
            }
            break;

        case LFS_STATE_CIRCLE:
            Circle_SetPID();
            LineFollow_SetBaseSpeed(LFS_CIRCLE_SPEED);
            LineFollow_SetOutputLimit(LFS_CIRCLE_OUTPUT);
            LineFollow_Update();

            /* Encoder-based transition back to Straight state */
            Motor_UpdateEncoderCount();
            uint32_t current_count = Motor_GetEncoderCount(MOTOR_ID_A);
            uint32_t elapsed_count = current_count - g_lfs_circle_start_count;
            if (elapsed_count >= g_lfs_circle_target_count) {
                LFS_EnterState(LFS_STATE_STRAIGHT);
            }
            break;

        case LFS_STATE_DASH_STRAIGHT_1:{
            StraightLine_SetPID();
            LineFollow_SetBaseSpeed(LFS_DASH_SPEED);
            LineFollow_SetOutputLimit(LFS_DASH_OUTPUT);

            uint16_t left_sp = LineFollow_GetLeftSpeed();
            uint16_t right_sp = LineFollow_GetRightSpeed();

            if (LFS_IsAllWhite(&sensors)) {
                /* All white: retain last motor speed */
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, left_sp);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, right_sp);
            } else {
                LineFollow_Update();
                left_sp = LineFollow_GetLeftSpeed();
                right_sp = LineFollow_GetRightSpeed();
            }

            /* Transition: DASH_STRAIGHT_1 -> DASH_TURN (intersection) */
            if (LFS_GetAbsDeviation() > LFS_LARGE_DEV_THRESHOLD ||
                LFS_IsIntersection(&sensors)) {
                g_lfs_intersection_count++;
                LFS_EnterState(LFS_STATE_DASH_TURN);
            }
            break;
        }

        case LFS_STATE_DASH_TURN:{
            Curve_SetPID();
            LineFollow_SetBaseSpeed(LFS_CIRCLE_SPEED);
            LineFollow_SetOutputLimit(LFS_DASH_OUTPUT);

            if (LFS_IsAllWhite(&sensors)) {
                /* Dashed turn: no LineFollow_Update to avoid integral accumulation */
                /* Calculate feedforward: feedforward = K_ff * error_dot */
                float error_dot = LineFollow_GetDerivative();
                int16_t feedforward = (int16_t)(LFS_K_FF * error_dot);

                /* Get base speeds from LineFollow (last calculated values) */
                int16_t base_left = (int16_t)LineFollow_GetLeftSpeed();
                int16_t base_right = (int16_t)LineFollow_GetRightSpeed();

                /* Left turn: left wheel subtract feedforward, right wheel add feedforward */
                int16_t left_sp = base_left - feedforward;
                int16_t right_sp = base_right + feedforward;

                /* Clamp speeds */
                if (left_sp < 0) left_sp = 0;
                if (right_sp > 100) right_sp = 100;
                if (left_sp > 100) left_sp = 100;
                if (right_sp < 0) right_sp = 0;

                Motor_Run(MOTOR_ID_A, MOTOR_FWD, (uint16_t)left_sp);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, (uint16_t)right_sp);
            } else {
                Curve_SetPID();
                LineFollow_Update();
            }

            /* Transition: DASH_TURN -> DASH_STRAIGHT_2 (intersection) */
            if (LFS_IsIntersection(&sensors)) {
                g_lfs_intersection_count++;
                LFS_EnterState(LFS_STATE_DASH_STRAIGHT_2);
            }
            break;
        }

        case LFS_STATE_DASH_STRAIGHT_2:
            StraightLine_SetPID();
            LineFollow_SetBaseSpeed(LFS_STRAIGHT_SPEED);
            LineFollow_SetOutputLimit(LFS_STRAIGHT_OUTPUT);

            uint16_t left_sp = LineFollow_GetLeftSpeed();
            uint16_t right_sp = LineFollow_GetRightSpeed();

            if (LFS_IsAllWhite(&sensors)) {
                /* All white: no LineFollow_Update to avoid integral accumulation */
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, left_sp);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, right_sp);
            } else if (!LFS_IsIntersection(&sensors)) {
                /* Not all white and not intersection: normal line tracking */
                LineFollow_Update();
                left_sp = LineFollow_GetLeftSpeed();
                right_sp = LineFollow_GetRightSpeed();
            }
            /* If 4+ black lines detected, keep previous speed (ignore intersection) */

            /* Transition: DASH_STRAIGHT_2 -> STRAIGHT (normal black line, not intersection) */
            if (LFS_IsAllBlack(&sensors)) {
                LFS_EnterState(LFS_STATE_STRAIGHT);
            }
            break;

        case LFS_STATE_ROTATION:
            /* Rotation: Fixed speed for reorientation */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, LFS_LOST_ROTATION_SPEED);
            Motor_Run(MOTOR_ID_B, MOTOR_BWD, LFS_LOST_ROTATION_SPEED);

            /* Transition: ROTATION -> STRAIGHT (any black line detected) */
            if (!sensors.LEFT2 || !sensors.LEFT1 || !sensors.CENTER ||
                !sensors.RIGHT1 || !sensors.RIGHT2) {
                StraightLine_SetPID();
                LineFollow_SetBaseSpeed(LFS_STRAIGHT_SPEED);
                LineFollow_SetOutputLimit(LFS_STRAIGHT_OUTPUT);
                LFS_EnterState(LFS_STATE_STRAIGHT);

                if (LFS_IsAllWhite(&sensors)){
                    LFS_EnterState(LFS_STATE_STOP);
                }
            }
            break;

        case LFS_STATE_STOP:
            Motor_Stop(MOTOR_ID_A);
            Motor_Stop(MOTOR_ID_B);
            break;

        default:
            LFS_EnterState(LFS_STATE_STRAIGHT);
            break;
    }
}

float LFS_GetDeviation(void)
{
    return LineFollow_GetPosition();
}

static void LFS_EnterState(LFS_State_t new_state)
{
    g_lfs_current_state = new_state;
    g_lfs_stable_start_tick = HAL_GetTick();
    g_lfs_detection_count = 0;

    /* Save encoder count when entering Circle state */
    if (new_state == LFS_STATE_CIRCLE) {
        g_lfs_circle_start_count = Motor_GetEncoderCount(MOTOR_ID_A);
    }
}

static uint8_t LFS_IsAllWhite(SENSOR_Status_t *sensors)
{
    return (sensors->LEFT2 && sensors->LEFT1 && sensors->CENTER &&
            sensors->RIGHT1 && sensors->RIGHT2);
}

uint8_t LFS_IsAllBlack(SENSOR_Status_t *sensors)
{
    return ((!sensors->LEFT2) && (!sensors->LEFT1) && (!sensors->CENTER) &&
            (!sensors->RIGHT1) && (!sensors->RIGHT2));
}

uint8_t LFS_IsIntersection(SENSOR_Status_t *sensors)
{
    uint8_t black_count = 0;
    if (!sensors->LEFT2) black_count++;
    if (!sensors->LEFT1) black_count++;
    if (!sensors->CENTER) black_count++;
    if (!sensors->RIGHT1) black_count++;
    if (!sensors->RIGHT2) black_count++;
    return (black_count >= 4);
}

float LFS_GetAbsDeviation(void)
{
    float deviation = LineFollow_GetPosition();
    return (deviation >= 0) ? deviation : -deviation;
}
