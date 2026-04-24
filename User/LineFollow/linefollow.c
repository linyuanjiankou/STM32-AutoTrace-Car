/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : linefollow.c
  * @brief          : Line follow control with weighted sum logic
  * @note          :
  *         Sensor value: 0 = black line, 1 = white surface
  *         PID calculation uses only: LEFT1, CENTER, RIGHT1
  *         Weighted sum: LEFT1*(-1) + CENTER*0 + RIGHT1*1
  *         LEFT2 and RIGHT2 are only used for:
  *           - State switching
  *           - Extreme case protection (when inner sensors lose the line)
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
#include "linefollow.h"
#include "motor.h"
#include "sys.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
static PID_t s_pid = {0};
static uint16_t s_left_speed = 0;
static uint16_t s_right_speed = 0;
static uint32_t s_last_update_tick = 0;

/* Exported variables - modify these to tune performance */
/* Base speed for both wheels (0-100) */
uint16_t g_linefollow_base_speed = 10;

/* PID parameters for deviation control */
float g_linefollow_pid_kp = 3.5f;
float g_linefollow_pid_ki = 0.0f;
float g_linefollow_pid_kd = 0.1f;

/* PID output limit */
float g_linefollow_pid_output_limit = 40.0f;

/* Integral windup limit */
float g_linefollow_pid_integral_limit = 20.0f;

/* Control loop period in milliseconds */
uint16_t g_linefollow_control_period = 20;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate weighted sensor deviation for PID
  * @note   Uses only LEFT1, CENTER, RIGHT1 sensors
  *         Weighted sum: LEFT1*(-1) + CENTER*0 + RIGHT1*1
  *         Sensor readings: 0=black line, 1=white surface
  *         Conversion: 0(黑线) -> -1, 1(白线) -> 1
  *         LEFT2 and RIGHT2 are NOT used for PID calculation
  *         They are only used for line detection/state switching
  * @retval int32_t: Weighted sum value (-2 to +2)
  */
static int32_t LineFollow_CalcWeightedSum(SENSOR_Status_t *sensors)
{
    int32_t weighted_sum = 0;

    /* 只使用 LEFT1, CENTER, RIGHT1 三个传感器进行PID计算 */
    /* 将 0/1 转换为 -1/1: 0(黑线) -> -1, 1(白线) -> 1 */
    int32_t s_left1  = (sensors->LEFT1  == 0) ? -1 : 1;
    int32_t s_center = (sensors->CENTER == 0) ? -1 : 1;
    int32_t s_right1 = (sensors->RIGHT1 == 0) ? -1 : 1;

    weighted_sum += s_left1  * (-1);
    weighted_sum += s_center * 0;
    weighted_sum += s_right1 * 1;

    return weighted_sum;
}

/**
  * @brief  PID update function
  * @retval float: PID output
  */
static float PID_Update(float setpoint, float feedback, float dt)
{
    static float error_prev = 0.0f;

    float error = setpoint - feedback;

    float kp_term = g_linefollow_pid_kp * error;

    s_pid.error_sum += error * dt;
    if (s_pid.error_sum > g_linefollow_pid_integral_limit) {
        s_pid.error_sum = g_linefollow_pid_integral_limit;
    } else if (s_pid.error_sum < -g_linefollow_pid_integral_limit) {
        s_pid.error_sum = -g_linefollow_pid_integral_limit;
    }
    float ki_term = g_linefollow_pid_ki * s_pid.error_sum;

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - error_prev) / dt;
    }
    error_prev = error;
    float kd_term = g_linefollow_pid_kd * derivative;

    float output = kp_term + ki_term + kd_term;

    if (output > g_linefollow_pid_output_limit) {
        output = g_linefollow_pid_output_limit;
    } else if (output < -g_linefollow_pid_output_limit) {
        output = -g_linefollow_pid_output_limit;
    }

    return output;
}

/* Exported functions --------------------------------------------------------*/
void LineFollow_Init(void)
{
    s_pid.setpoint = 0.0f;
    s_pid.feedback = 0.0f;
    s_pid.error = 0.0f;
    s_pid.error_sum = 0.0f;
    s_pid.error_prev = 0.0f;
    s_pid.output = 0.0f;

    s_last_update_tick = HAL_GetTick();
}

void LineFollow_Update(void)
{
    SENSOR_Status_t sensors;
    int32_t weighted_sum;
    uint8_t line_detected;
    uint8_t left_edge_detected;
    uint8_t right_edge_detected;

    /* Read sensor data (0=black line, 1=white surface) */
    SENSOR_ReadRaw(&sensors);

    /* Calculate weighted sum (only uses LEFT1, CENTER, RIGHT1) */
    weighted_sum = LineFollow_CalcWeightedSum(&sensors);

    /* 检测外侧传感器是否检测到黑线 (LEFT2, RIGHT2) */
    left_edge_detected  = (sensors.LEFT2 == 0);
    right_edge_detected = (sensors.RIGHT2 == 0);

    /* 检测是否检测到黑线 (使用所有5个传感器) */
    line_detected = (sensors.LEFT2 == 0 || sensors.LEFT1 == 0 ||
                     sensors.CENTER == 0 || sensors.RIGHT1 == 0 ||
                     sensors.RIGHT2 == 0);

    /* 极端情况保护：如果内侧三个传感器都未检测到黑线，但外侧有检测到 */
    /* 说明线已严重偏移，需要大转向 */
    if (line_detected && (sensors.LEFT1 == 1 && sensors.CENTER == 1 && sensors.RIGHT1 == 1))
    {
        if (left_edge_detected) {
            /* 左侧外侧检测到黑线，大左转 */
            s_left_speed  = g_linefollow_base_speed - 30;
            s_right_speed = g_linefollow_base_speed + 30;
        } else if (right_edge_detected) {
            /* 右侧外侧检测到黑线，大右转 */
            s_left_speed  = g_linefollow_base_speed + 30;
            s_right_speed = g_linefollow_base_speed - 30;
        }
    }
    else if (!line_detected) {
        /* 完全丢线：原地旋转 */
        s_left_speed  = g_linefollow_base_speed;
        s_right_speed = g_linefollow_base_speed;
    } else if (weighted_sum != 0) {
        /* 正常循迹：使用PID correction */
        float dt = (float)g_linefollow_control_period / 1000.0f;
        float speed_diff = PID_Update(0.0f, (float)weighted_sum, dt);
        int32_t speed_abs = my_abs((int32_t)speed_diff);

        if (speed_diff >= 0) {
            /* Turn left: left wheel faster, right wheel slower */
            s_left_speed  = g_linefollow_base_speed + speed_abs;
            s_right_speed = g_linefollow_base_speed - speed_abs;
        } else {
            /* Turn right: left wheel slower, right wheel faster */
            s_left_speed  = g_linefollow_base_speed - speed_abs;
            s_right_speed = g_linefollow_base_speed + speed_abs;
        }

        /* Clamp speeds to 0-100 */
        if (s_left_speed > 100) s_left_speed = 100;
        if (s_right_speed > 100) s_right_speed = 100;

        /* Ensure minimum speed */
        if (s_left_speed < 10) s_left_speed = 10;
        if (s_right_speed < 10) s_right_speed = 10;
    } else {
        /* Line detected but weighted_sum == 0: go straight */
        s_left_speed = g_linefollow_base_speed;
        s_right_speed = g_linefollow_base_speed;
    }

    /* Clamp speeds again after extreme case handling */
    if (s_left_speed > 100) s_left_speed = 100;
    if (s_right_speed > 100) s_right_speed = 100;
    if (s_left_speed < 0) s_left_speed = 0;
    if (s_right_speed < 0) s_right_speed = 0;

    /* Run motors */
    if (!line_detected) {
        /* No line detected: spin in place */
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
        Motor_Run(MOTOR_ID_B, MOTOR_BWD, s_right_speed);
    } else if (weighted_sum != 0) {
        /* Line detected with deviation */
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
        Motor_Run(MOTOR_ID_B, MOTOR_FWD, s_right_speed);
    } else {
        /* Line detected but centered */
        Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
        Motor_Run(MOTOR_ID_B, MOTOR_FWD, s_right_speed);
    }
}

void LineFollow_SetBaseSpeed(uint16_t speed)
{
    g_linefollow_base_speed = (speed > 100) ? 100 : speed;
}

void LineFollow_SetPID(float kp, float ki, float kd)
{
    g_linefollow_pid_kp = kp;
    g_linefollow_pid_ki = ki;
    g_linefollow_pid_kd = kd;
}

void LineFollow_SetOutputLimit(float limit)
{
    g_linefollow_pid_output_limit = limit;
}

void LineFollow_SetIntegralLimit(float limit)
{
    g_linefollow_pid_integral_limit = limit;
}

void LineFollow_SetControlPeriod(uint16_t period_ms)
{
    g_linefollow_control_period = (period_ms < 10) ? 10 : period_ms;
}

int32_t LineFollow_GetWeightedSum(void)
{
    SENSOR_Status_t sensors;
    SENSOR_ReadRaw(&sensors);
    return LineFollow_CalcWeightedSum(&sensors);
}

float LineFollow_GetPosition(void)
{
    return (float)LineFollow_GetWeightedSum();
}

uint16_t LineFollow_GetLeftSpeed(void)
{
    return s_left_speed;
}

uint16_t LineFollow_GetRightSpeed(void)
{
    return s_right_speed;
}

void StraightLine_SetPID(void){
    LineFollow_SetPID(3.5f, 0.0f, 0.1f);
}

void Circle_SetPID(void){
    LineFollow_SetPID(4.0f, 0.0f, 0.1f);
}

void Curve_SetPID(void){
    LineFollow_SetPID(4.0f, 0.0f, 0.0f);
}

/**
  * @brief  Reset integral term (clear accumulated error)
  * @retval None
  */
void LineFollow_ResetIntegral(void)
{
    s_pid.error_sum = 0.0f;
}

/**
  * @brief  Get derivative (error_dot) for feedforward calculation
  * @note   Calculates the rate of change of error: (error - error_prev) / dt
  * @retval float: Derivative value (error_dot)
  */
float LineFollow_GetDerivative(void)
{
    float dt = (float)g_linefollow_control_period / 1000.0f;
    if (dt <= 0.0f) return 0.0f;
    return (s_pid.error - s_pid.error_prev) / dt;
}
