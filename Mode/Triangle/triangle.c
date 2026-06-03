/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : triangle.c
  * @brief          : Triangle mode state machine implementation
  *                   Logic: Black line = 0, White surface = 1
  *                   Turns when LEFT2 or RIGHT2 detects black line (start of curve)
  *                   No stop after turns - runs continuously
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
#include "triangle.h"
#include "motor.h"
#include "linefollow.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
TriangleState_t g_triangle_current_state = TRIANGLE_STATE_START;

static uint8_t s_direction_memory = TURN_RIGHT_DIR;
static char count = 0; //记录转弯几次
static uint8_t s_reentry_count = 8;
static char s_ensure_lost = 0;
static uint32_t s_start_lost = 0;
// static uint32_t s_wait_entry_tick = 0;
// static char s_turn_saw_right2_clear = 0;
static char pass_turn = 0;

/* Private function prototypes -----------------------------------------------*/
static void Triangle_EnterState(TriangleState_t new_state);

/* Private functions ---------------------------------------------------------*/
static void Triangle_EnterState(TriangleState_t new_state)
{
    g_triangle_current_state = new_state;
    if (new_state == TRIANGLE_STATE_TRACK){
        Motor_SetDirection(MOTOR_ID_A, MOTOR_FWD);
        Motor_SetDirection(MOTOR_ID_B, MOTOR_FWD);
        LineFollow_SetPID(2.4f, 0.0f, 0.11f);
        LineFollow_SetBaseSpeed(STRAIGHT_SPEED);
        LineFollow_ResetIntegral();
        Motor_ResetPIDOutput();
        s_start_lost = HAL_GetTick();
        s_ensure_lost = 0;
    }else if (new_state == TRIANGLE_STATE_TURN){
        pass_turn = 0;
    }
}

/* Exported functions --------------------------------------------------------*/
void Triangle_Init(void)
{
    g_triangle_current_state = TRIANGLE_STATE_START;
    s_direction_memory = TURN_RIGHT_DIR;

    StraightLine_SetPID();
    LineFollow_SetBaseSpeed(STRAIGHT_SPEED);
    LineFollow_SetOutputLimit(19660.5f);
}

void Triangle_Update(void)
{
    SENSOR_Status_t sensors;
    uint8_t black_line_detected;

    SENSOR_ReadRaw(&sensors);

    /* 黑线=0，白线=1 */
    black_line_detected = (!(sensors.LEFT1) || !(sensors.CENTER) || !(sensors.RIGHT1));

    switch (g_triangle_current_state) {
        case TRIANGLE_STATE_START:{
            count = 0;
            // s_wait_entry_tick = 0;
            // s_turn_saw_right2_clear = 0;
            pass_turn = 0;
            s_direction_memory = TURN_RIGHT_DIR;
            Triangle_EnterState(TRIANGLE_STATE_TRACK);
            break;
        }
        case TRIANGLE_STATE_TRACK:{
            LineFollow_Update();

            /* RIGHT2检测到黑线（值为0）时设置转弯标志 */
            if (!sensors.RIGHT2) {
                // s_wait_entry_tick = HAL_GetTick();
                Triangle_EnterState(TRIANGLE_STATE_TURN);
            }

            /* 丢线容忍：等 200ms 让 PID 自行修正 */
            if (!black_line_detected) {
                if (s_ensure_lost == 0) {
                    /* 第一次丢线：记录时间，不干预电机 */
                    s_start_lost = HAL_GetTick();
                    s_ensure_lost = 1;
                } else if (HAL_GetTick() - s_start_lost > 200) {
                    /* 超时未恢复 → 真正丢线 */
                    s_ensure_lost = 0;
                    Triangle_EnterState(TRIANGLE_STATE_LOST);
                    break;
                }
            } else {
                /* 线还在，清零容忍标记 */
                s_ensure_lost = 0;
                s_start_lost = 0;
            }

            /* 转弯三次，结束该模式*/
            // if (count > 3){
            //     Triangle_EnterState(TRIANGLE_STATE_STOP);
            // }
            break;
            
        }

        case TRIANGLE_STATE_TURN:{
            SENSOR_ReadRaw(&sensors);
            black_line_detected = (!(sensors.LEFT1) || !(sensors.CENTER) || !(sensors.RIGHT1));

            if (!pass_turn) {
                /* Phase 1：左转，等 CENTER 变白（离开原线） */
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, OUTER_TURN_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_BWD, INNER_TURN_SPEED);
                s_direction_memory = TURN_LEFT_DIR;
                SENSOR_ReadRaw(&sensors);

                if (sensors.CENTER) {
                    pass_turn = 1;
                }
            } else {
                /* Phase 2：继续旋转，直到检测到新黑线 */
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, OUTER_TURN_SPEED / 2);
                Motor_Run(MOTOR_ID_B, MOTOR_BWD, INNER_TURN_SPEED);
                SENSOR_ReadRaw(&sensors);

                if (black_line_detected) {
                    Motor_Stop(MOTOR_ID_A);
                    Motor_Stop(MOTOR_ID_B);
                    pass_turn = 0;
                    count++;
                    Triangle_EnterState(TRIANGLE_STATE_TRACK);
                }
            }
            break;
        }

        case TRIANGLE_STATE_WAIT:{
            SENSOR_ReadRaw(&sensors);

            /* CENTER 对准黑线 → 成功切入 TRACK */
            if (!(sensors.CENTER)) {
                Motor_Stop(MOTOR_ID_A);
                Motor_Stop(MOTOR_ID_B);
                Triangle_EnterState(TRIANGLE_STATE_TRACK);
                break;
            }

            /* 线在左侧 → 向右低速修 */
            if (!(sensors.LEFT1) || !(sensors.LEFT2)) {
                Motor_Run(MOTOR_ID_A, MOTOR_BWD, LOST_SEARCH_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, LOST_SEARCH_SPEED);
            }
            /* 线在右侧 → 向左低速修 */
            else if (!(sensors.RIGHT1) || !(sensors.RIGHT2)) {
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, LOST_SEARCH_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_BWD, LOST_SEARCH_SPEED);
            }
            /* 都不见 → 丢线，向右回转 */
            else {
                Motor_Run(MOTOR_ID_A, MOTOR_BWD, LOST_SEARCH_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, LOST_SEARCH_SPEED);
            }
            break;
        }

        case TRIANGLE_STATE_LOST:
            /* 丢失黑线时继续向反方向旋转搜索 */
            if (s_direction_memory == TURN_LEFT_DIR) {
                // Motor_Stop(MOTOR_ID_A);
                Motor_Run(MOTOR_ID_A, MOTOR_BWD, INNER_TURN_SPEED);
                // Motor_Stop(MOTOR_ID_B);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, INNER_TURN_SPEED);
            } else {
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, INNER_TURN_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_BWD, INNER_TURN_SPEED);
                // Motor_Stop(MOTOR_ID_A);
            }

            /* 检测到黑线后返回跟踪状态 */
            if (!(sensors.CENTER)) {
                Triangle_EnterState(TRIANGLE_STATE_TRACK);
            }
            break;

        case TRIANGLE_STATE_STOP:
            Motor_Stop(MOTOR_ID_A);
            Motor_Stop(MOTOR_ID_B);
            break;

        default:
            Triangle_EnterState(TRIANGLE_STATE_START);
            break;
    }
}

