/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : circle.c
  * @brief          : Circle mode state machine implementation
  *                   Implements circular track:
  *                   - Uses LEFT1, CENTER, RIGHT1 for line tracking (calls LineFollow)
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

/* Includes ------------------------------------------------------------------*/
#include "circle.h"
#include "linefollow.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
/* Current state */
CircleState_t g_circle_current_state = CIRCLE_STATE_START;

/* Number of LEFT2/RIGHT2 black line detections */
int g_circle_detection_count = 0;

/* Private function prototypes -----------------------------------------------*/
static void Circle_EnterState(CircleState_t new_state);

/* Private functions ---------------------------------------------------------*/
static void Circle_EnterState(CircleState_t new_state)
{
    g_circle_current_state = new_state;
}

/* Exported functions --------------------------------------------------------*/
void Circle_Init(void)
{
    g_circle_current_state = CIRCLE_STATE_START;
    g_circle_detection_count = 0;

    /* Set PID parameters for circle mode using LineFollow's API */
    Circle_SetPID();    /* Kp, Ki, Kd */
    LineFollow_SetBaseSpeed(CIRCLE_BASE_SPEED);
    LineFollow_SetOutputLimit(60.0f);
}

void Circle_Update(void)
{
    SENSOR_Status_t sensors;

    uint16_t left_sp = LineFollow_GetLeftSpeed();
    uint16_t right_sp = LineFollow_GetRightSpeed();

    /* Read sensor data (0=black line, 1=white surface) */
    SENSOR_ReadRaw(&sensors);

    switch (g_circle_current_state) {
        case CIRCLE_STATE_START:
            /* Initialize and transition to tracking */
            g_circle_detection_count = 0;
            Circle_EnterState(CIRCLE_STATE_TRACK);
            break;

        case CIRCLE_STATE_TRACK:
            LineFollow_ResetIntegral();
            /* Use LineFollow for line tracking with LEFT1, CENTER, RIGHT1 */
            Circle_SetPID();
            LineFollow_Update();

            left_sp = LineFollow_GetLeftSpeed();
            right_sp = LineFollow_GetRightSpeed();

            /* Check for RIGHT2 black line detection (value = 0) */
            SENSOR_ReadRaw(&sensors);
            if (!(sensors.RIGHT2)) {
                /* Wait for sensor to leave the line to avoid multiple counts */
                while (!(sensors.RIGHT2)) {
                    SENSOR_ReadRaw(&sensors);
                    Motor_Run(MOTOR_ID_A, MOTOR_FWD, left_sp);
                    Motor_Run(MOTOR_ID_B, MOTOR_FWD, right_sp);
                    if (g_circle_detection_count >= CIRCLE_COUNT_TARGET) {
                        break;
                    }
                }
                g_circle_detection_count++;
            }

            /* Check if we've completed the required count */
            if (g_circle_detection_count >= CIRCLE_COUNT_TARGET) {
                Circle_EnterState(CIRCLE_STATE_STOP);
            }

            /* Check for lost line (all sensors are white) */
            if (sensors.LEFT2 && sensors.LEFT1 && sensors.CENTER &&
                sensors.RIGHT1 && sensors.RIGHT2) {
                Circle_EnterState(CIRCLE_STATE_LOST);
            }
            break;

        case CIRCLE_STATE_LOST:
            /* Spin in place searching for line */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, CIRCLE_BASE_SPEED);
            Motor_Run(MOTOR_ID_B, MOTOR_BWD, CIRCLE_BASE_SPEED);

            /* Check if line is found */
            if (!(sensors.LEFT2) || !(sensors.LEFT1) || !(sensors.CENTER) ||
                !(sensors.RIGHT1) || !(sensors.RIGHT2)) {
                /* Line found - return to tracking */
                Circle_EnterState(CIRCLE_STATE_TRACK);
            }
            break;

        case CIRCLE_STATE_STOP:
            /* Stop both motors */
            Motor_Stop(MOTOR_ID_A);
            Motor_Stop(MOTOR_ID_B);
            break;

        default:
            /* Unknown state - reset to start */
            Circle_EnterState(CIRCLE_STATE_START);
            break;
    }
}
