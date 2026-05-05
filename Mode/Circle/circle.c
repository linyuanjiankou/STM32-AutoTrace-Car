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
  * Copyright (c) 2026 LiminalStill.
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

/* Non-blocking edge detection + lost tolerance */
static uint8_t s_right2_was_on_line = 0;   /* RIGHT2 上一帧是否在黑线上 */
static uint8_t s_ensure_lost = 0;          /* 丢线容忍标记 */
static uint32_t s_start_lost = 0;          /* 丢线计时起点 */

/* Private function prototypes -----------------------------------------------*/
static void Circle_EnterState(CircleState_t new_state);

/* Private functions ---------------------------------------------------------*/
static void Circle_EnterState(CircleState_t new_state)
{
    g_circle_current_state = new_state;
    if (new_state == CIRCLE_STATE_TRACK) {
        Motor_SetDirection(MOTOR_ID_A, MOTOR_FWD);
        Motor_SetDirection(MOTOR_ID_B, MOTOR_FWD);
        LineFollow_SetPID(2.85f, 0.0f, 0.0f);
        LineFollow_SetBaseSpeed(CIRCLE_BASE_SPEED);
        LineFollow_ResetIntegral();
        Motor_ResetPIDOutput();
    }
}

/* Exported functions --------------------------------------------------------*/
void Circle_Init(void)
{
    g_circle_current_state = CIRCLE_STATE_START;
    g_circle_detection_count = 0;

    /* Set PID parameters for circle mode using LineFollow's API */
    Circle_SetPID();    /* Kp, Ki, Kd */
    LineFollow_SetBaseSpeed(CIRCLE_BASE_SPEED);
    LineFollow_SetOutputLimit(19660.5f);
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
            LineFollow_Update();

            /* RIGHT2 黑线检测：非阻塞边沿检测（避免阻塞主循环影响 5ms 内环 PI） */
            SENSOR_ReadRaw(&sensors);
            if (!sensors.RIGHT2) {
                if (!s_right2_was_on_line) {
                    g_circle_detection_count++;
                    s_right2_was_on_line = 1;
                }
            } else {
                s_right2_was_on_line = 0;
            }

            /* 丢线容忍：等 200ms 让 PID 自行修正 */
            {
                uint8_t black_line_detected = (!(sensors.LEFT1) || !(sensors.CENTER) || !(sensors.RIGHT1));
                if (!black_line_detected) {
                    if (s_ensure_lost == 0) {
                        s_start_lost = HAL_GetTick();
                        s_ensure_lost = 1;
                    } else if (HAL_GetTick() - s_start_lost > 200) {
                        s_ensure_lost = 0;
                        Circle_EnterState(CIRCLE_STATE_LOST);
                        break;
                    }
                } else {
                    s_ensure_lost = 0;
                }
            }

            /* 完成计数 → 停止 */
            if (g_circle_detection_count >= CIRCLE_COUNT_TARGET) {
                Circle_EnterState(CIRCLE_STATE_STOP);
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
