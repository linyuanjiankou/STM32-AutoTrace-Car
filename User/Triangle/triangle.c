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
#include "triangle.h"
#include "linefollow.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
CarState_t g_triangle_current_state = STATE_START;
int g_triangle_turn_count = 0;

static uint8_t s_direction_memory = TURN_RIGHT_DIR;
static uint32_t s_state_entry_tick = 0;
static uint8_t s_in_turn_debounce = 0;
static uint8_t s_turn_complete_flag = 0;
static uint8_t s_post_turn_debounce = 0;  // 退出转弯后的防抖

/* Private function prototypes -----------------------------------------------*/
static void Triangle_EnterState(CarState_t new_state);

/* Private functions ---------------------------------------------------------*/
static void Triangle_EnterState(CarState_t new_state)
{
    g_triangle_current_state = new_state;
    s_state_entry_tick = HAL_GetTick();
}

/* Exported functions --------------------------------------------------------*/
void Triangle_Init(void)
{
    g_triangle_current_state = STATE_START;
    g_triangle_turn_count = 0;
    s_direction_memory = TURN_RIGHT_DIR;
    s_state_entry_tick = HAL_GetTick();
    s_in_turn_debounce = 0;
    s_turn_complete_flag = 0;
    s_post_turn_debounce = 0;

    LineFollow_SetPID(4.5f, 0.0f, 0.0f);
    LineFollow_SetBaseSpeed(12);
    LineFollow_SetOutputLimit(50.0f);
}

void Triangle_Update(void)
{
    SENSOR_Status_t sensors;
    uint8_t black_line_detected;

    SENSOR_ReadRaw(&sensors);

    /* 黑线=0，白线=1 */
    black_line_detected = (!(sensors.LEFT2) || !(sensors.LEFT1) || !(sensors.CENTER) ||
                           !(sensors.RIGHT1) || !(sensors.RIGHT2));

    switch (g_triangle_current_state) {
        case STATE_START:
            g_triangle_turn_count = 0;
            s_direction_memory = TURN_RIGHT_DIR;
            s_in_turn_debounce = 0;
            s_turn_complete_flag = 0;
            Triangle_EnterState(STATE_TRACK);
            break;

        case STATE_TRACK:
            LineFollow_Update();

            /* 检测LEFT2或RIGHT2检测到黑线（值为0）时设置转弯标志 */
            if (!(sensors.LEFT2) || !(sensors.RIGHT2)) {
                s_in_turn_debounce = 1;
            }

            /* 进入转弯：防抖标志已设置 且 当前在TRACK状态 */
            if (s_in_turn_debounce && g_triangle_current_state == STATE_TRACK) {
                s_in_turn_debounce = 0;
                g_triangle_turn_count++;
                s_turn_complete_flag = 0;
                Triangle_EnterState(STATE_TURN);
            }

            /* 丢失黑线 - 所有传感器都是白线 */
            if (!black_line_detected) {
                s_in_turn_debounce = 0;
                Triangle_EnterState(STATE_LOST);
            }

            /* 退出转弯后的防抖：避免立即再次进入转弯 */
            if (s_post_turn_debounce) {
                /* 保持在TRACK状态，等待传感器离开黑线 */
                if (!(sensors.LEFT2) && !(sensors.RIGHT2)) {
                    s_post_turn_debounce = 0;
                }
            }
            break;

        case STATE_TURN:
            /* 默认右转：左轮前进，右轮后退 */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, TURN_SPEED);
            Motor_Run(MOTOR_ID_B, MOTOR_BWD, TURN_SPEED);

            /* 检测退出转弯条件：只有一个传感器检测到黑线，其他都是白线 */
            uint8_t black_count = 0;
            if (!(sensors.LEFT2))  black_count++;
            if (!(sensors.LEFT1))  black_count++;
            if (!(sensors.CENTER)) black_count++;
            if (!(sensors.RIGHT1)) black_count++;
            if (!(sensors.RIGHT2)) black_count++;

            /* 只有一个传感器检测到黑线时退出转弯 */
            if (black_count == 1) {
                s_turn_complete_flag = 1;
                s_post_turn_debounce = 1;  // 设置后防抖
                Triangle_EnterState(STATE_TRACK);
            }
            break;

        case STATE_LOST:
            /* 丢失黑线时继续按原方向旋转搜索 */
            if (s_direction_memory == TURN_LEFT_DIR) {
                Motor_Run(MOTOR_ID_A, MOTOR_BWD, LOST_SEARCH_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_FWD, LOST_SEARCH_SPEED);
            } else {
                Motor_Run(MOTOR_ID_A, MOTOR_FWD, LOST_SEARCH_SPEED);
                Motor_Run(MOTOR_ID_B, MOTOR_BWD, LOST_SEARCH_SPEED);
            }

            /* 检测到黑线后返回跟踪状态 */
            if (black_line_detected) {
                s_in_turn_debounce = 0;
                Triangle_EnterState(STATE_TRACK);
            }
            break;

        case STATE_STOP:
            Motor_Stop(MOTOR_ID_A);
            Motor_Stop(MOTOR_ID_B);
            break;

        default:
            Triangle_EnterState(STATE_START);
            break;
    }
}
