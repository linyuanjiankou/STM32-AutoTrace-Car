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
float g_linefollow_pid_kp = 2.0f;
float g_linefollow_pid_ki = 0.0f;
float g_linefollow_pid_kd = 0.05f;

/* PID output limit */
float g_linefollow_pid_output_limit = 80.0f;

/* Integral windup limit */
float g_linefollow_pid_integral_limit = 20.0f;

/* Control loop period in milliseconds */
uint16_t g_linefollow_control_period = 20;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate weighted sensor deviation for PID
  * @note   Uses only LEFT1, CENTER, RIGHT1 sensors
  *         Weighted sum: LEFT1*(-1) + CENTER*0 + RIGHT1*1
  *         Sensor readings: 1=black line, 0=white surface
  *         Conversion: 1(黑线) -> -1, 0(白面) -> +1
  *         LEFT2 and RIGHT2 are NOT used for PID calculation
  *         They are only used for line detection/state switching
  * @retval int32_t: Weighted sum value (-2 to +2)
  */
static int32_t LineFollow_CalcWeightedSum(SENSOR_Status_t *sensors)
{
    int32_t weighted_sum = 0;

    /* 传感器：1=黑线, 0=白面 */
    /* 转换为：1(黑线)->-1, 0(白面)->+1 */
    int32_t s_left1  = (sensors->LEFT1)  ? -1 : 1;
    int32_t s_center = (sensors->CENTER) ? -1 : 1;
    int32_t s_right1 = (sensors->RIGHT1) ? -1 : 1;

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
        derivative = (error - s_pid.error) / dt;
    }
    float kd_term = g_linefollow_pid_kd * derivative;

    float output = kp_term + ki_term + kd_term;

    if (output > g_linefollow_pid_output_limit) {
        output = g_linefollow_pid_output_limit;
    } else if (output < -g_linefollow_pid_output_limit) {
        output = -g_linefollow_pid_output_limit;
    }

    /* Sync back to s_pid structure */
    s_pid.error_prev = s_pid.error;  /* Save previous error before updating */
    s_pid.error = error;
    s_pid.feedback = feedback;
    s_pid.output = output;

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

    /* Calculate real time interval (no early return — dt may vary) */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_update_tick) / 1000.0f;

    /* Read sensor`	data (0=black line, 1=white surface) */
    SENSOR_ReadRaw(&sensors);

    /* Calculate weighted sum (only uses LEFT1, CENTER, RIGHT1) */
    weighted_sum = LineFollow_CalcWeightedSum(&sensors);

    if (weighted_sum != 0) {
        /* 正常循迹：使用PID correction */
        float speed_diff = PID_Update(0.0f, (float)weighted_sum, dt);
        int32_t speed_abs = my_abs((int32_t)speed_diff);

        int32_t left_raw  = (int32_t)g_linefollow_base_speed + (speed_diff <= 0 ?  speed_abs : -speed_abs);
        int32_t right_raw = (int32_t)g_linefollow_base_speed + (speed_diff <= 0 ? -speed_abs :  speed_abs);

        // Clamp
        if (left_raw  > 100) left_raw  = 100;
        if (left_raw  < 10)  left_raw  = 10;
        if (right_raw > 100) right_raw = 100;
        if (right_raw < 10)  right_raw = 10;

        s_left_speed  = (uint16_t)left_raw;
        s_right_speed = (uint16_t)right_raw;
        
    } else {
        /* Line detected but weighted_sum == 0: go straight */
        s_left_speed = g_linefollow_base_speed;
        s_right_speed = g_linefollow_base_speed;
    }

    /* Write speed targets to motor speed setpoint for inner PI loop */
    Motor_SetSpeed(MOTOR_ID_A, s_left_speed);
    Motor_SetSpeed(MOTOR_ID_B, s_right_speed);
    s_last_update_tick = now;
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

void LineFollow_SetBaseSpeed(uint16_t speed)
{
    g_linefollow_base_speed = speed;
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
    LineFollow_SetPID(1.5f, 0.0f, 0.0f);
}

void Circle_SetPID(void){
    LineFollow_SetPID(2.85f, 0.0f, 0.0f);
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
