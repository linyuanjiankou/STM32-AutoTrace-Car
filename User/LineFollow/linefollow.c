/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : linefollow.c
  * @brief          : Line follow control with weighted sum logic
  * @note          :
  *         Sensor value: 1 = black line, 0 = white surface
  *         Weighted sum: LEFT2*(-2) + LEFT1*(-1) + CENTER*0 + RIGHT1*1 + RIGHT2*2
  *         1. If NO line detected (all sensors = 0): spin in place
  *         2. If line detected AND weighted_sum != 0: use PID to correct direction
  *         3. If line detected AND weighted_sum == 0: go straight (both wheels at base speed)
  *            This covers: centered on line, full black, symmetric sensor patterns
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
/* Kp: Proportional gain - higher = more aggressive correction */
float g_linefollow_pid_kp = 3.0f;

/* Ki: Integral gain - eliminates steady-state error */
float g_linefollow_pid_ki = 0.0f;

/* Kd: Derivative gain - dampens oscillations */
float g_linefollow_pid_kd = 0.0f;

/* PID output limit (max speed difference) */
float g_linefollow_pid_output_limit = 40.0f;

/* Integral windup limit */
float g_linefollow_pid_integral_limit = 20.0f;

/* Control loop period in milliseconds */
uint16_t g_linefollow_control_period = 20;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate weighted sensor deviation
  * @note   Sensor readings: 1=black line, 0=white surface
  *         Weighted sum: LEFT2*(-2) + LEFT1*(-1) + CENTER*0 + RIGHT1*1 + RIGHT2*2
  * @retval int32_t: Weighted sum value (-5 to +5)
  */
static int32_t LineFollow_CalcWeightedSum(SENSOR_Status_t *sensors)
{
    int32_t weighted_sum = 0;

    /* Weighted sum calculation */
    weighted_sum += (int32_t)sensors->LEFT2  * (-2);
    weighted_sum += (int32_t)sensors->LEFT1  * (-1);
    weighted_sum += (int32_t)sensors->CENTER * 0;
    weighted_sum += (int32_t)sensors->RIGHT1 * 1;
    weighted_sum += (int32_t)sensors->RIGHT2 * 2;

    return weighted_sum;
}

/**
  * @brief  PID update function
  * @note   Computes PID output based on error (deviation from center)
  * @param  setpoint: Target value (0 = centered on line)
  * @param  feedback: Current measured value (current weighted sum)
  * @param  dt: Time delta in seconds
  * @retval float: PID output (speed difference between left and right wheels)
  */
static float PID_Update(float setpoint, float feedback, float dt)
{
    static float error_prev = 0.0f;
    static float error_sum = 0.0f;

    /* Calculate error */
    float error = setpoint - feedback;

    /* Proportional term */
    float kp_term = g_linefollow_pid_kp * error;

    /* Integral term */
    error_sum += error * dt;
    if (error_sum > g_linefollow_pid_integral_limit) {
        error_sum = g_linefollow_pid_integral_limit;
    } else if (error_sum < -g_linefollow_pid_integral_limit) {
        error_sum = -g_linefollow_pid_integral_limit;
    }
    float ki_term = g_linefollow_pid_ki * error_sum;

    /* Derivative term */
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - error_prev) / dt;
    }
    error_prev = error;
    float kd_term = g_linefollow_pid_kd * derivative;

    /* Calculate PID output */
    float output = kp_term + ki_term + kd_term;

    /* Clamp output to limit */
    if (output > g_linefollow_pid_output_limit) {
        output = g_linefollow_pid_output_limit;
    } else if (output < -g_linefollow_pid_output_limit) {
        output = -g_linefollow_pid_output_limit;
    }

    return output;
}

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize line follow control
  * @retval None
  */
void LineFollow_Init(void)
{
    /* Initialize PID structure */
    s_pid.setpoint = 0.0f;
    s_pid.feedback = 0.0f;
    s_pid.error = 0.0f;
    s_pid.error_sum = 0.0f;
    s_pid.error_prev = 0.0f;
    s_pid.output = 0.0f;

    /* Reset timestamps */
    s_last_update_tick = HAL_GetTick();

    /* Initialize motor */
    Motor_Init();

    /* Print configuration */
    printf("\r\n=== Line Follow Init ===\r\n");
    printf("Base speed: %d\r\n", g_linefollow_base_speed);
    printf("PID: Kp=%.1f Ki=%.1f Kd=%.1f\r\n",
           g_linefollow_pid_kp, g_linefollow_pid_ki, g_linefollow_pid_kd);
    printf("Output limit: %.1f\r\n", g_linefollow_pid_output_limit);
    printf("Control period: %d ms\r\n\r\n", g_linefollow_control_period);
}

/**
  * @brief  Update line follow control - call in main loop
  * @note   Implementation of simplified logic:
  *         1. Calculate weighted sum from sensor readings
  *         2. Check if any sensor detects the line (value = 1 = black line)
  *         3. If no line detected (all sensors = 0): spin in place
  *         4. If line detected and weighted_sum != 0: use PID to correct direction
  *         5. If line detected and weighted_sum == 0: go straight at base speed
  *            (covers centered, full black, symmetric sensor patterns)
  * @retval None
  */
void LineFollow_Update(void)
{
    uint32_t current_tick = HAL_GetTick();
    SENSOR_Status_t sensors;
    int32_t weighted_sum;
    uint8_t line_detected;

    /* Check if it's time to update (control period) */
    if (current_tick - s_last_update_tick >= g_linefollow_control_period)
    {
        s_last_update_tick = current_tick;

        /* Read sensor data (value: 1=black line, 0=white surface) */
        SENSOR_ReadRaw(&sensors);

        /* Calculate weighted sum */
        weighted_sum = LineFollow_CalcWeightedSum(&sensors);

        /* Check if line is detected (any sensor detecting black line = 1) */
        line_detected = (sensors.LEFT2 == 1 || sensors.LEFT1 == 1 ||
                         sensors.CENTER == 1 || sensors.RIGHT1 == 1 ||
                         sensors.RIGHT2 == 1);

        if (!line_detected) {
            /* No line detected: spin in place */
            /* Left wheel forward, right wheel backward */
            s_left_speed  = g_linefollow_base_speed;
            s_right_speed = g_linefollow_base_speed;
        } else if (weighted_sum != 0) {
            /* Line detected with deviation: use PID to correct */
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
        } else {
            /* Line detected but weighted_sum == 0: go straight at base speed */
            /* This covers: centered on line, full black, symmetric patterns */
            s_left_speed = g_linefollow_base_speed;
            s_right_speed = g_linefollow_base_speed;
        }

        /* Run motors */
        if (!line_detected) {
            /* No line detected: spin in place */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
            Motor_Run(MOTOR_ID_B, MOTOR_BWD, s_right_speed);
        } else if (weighted_sum != 0) {
            /* Line detected with deviation: normal line following */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
            Motor_Run(MOTOR_ID_B, MOTOR_FWD, s_right_speed);
        } else {
            /* Line detected but centered: continue forward */
            Motor_Run(MOTOR_ID_A, MOTOR_FWD, s_left_speed);
            Motor_Run(MOTOR_ID_B, MOTOR_FWD, s_right_speed);
        }
    }
}

/**
  * @brief  Set base speed for both wheels
  * @param  speed: Base speed (0-100)
  * @retval None
  */
void LineFollow_SetBaseSpeed(uint16_t speed)
{
    g_linefollow_base_speed = (speed > 100) ? 100 : speed;
}

/**
  * @brief  Set PID parameters dynamically
  * @param  kp: Proportional gain
  * @param  ki: Integral gain
  * @param  kd: Derivative gain
  * @retval None
  */
void LineFollow_SetPID(float kp, float ki, float kd)
{
    g_linefollow_pid_kp = kp;
    g_linefollow_pid_ki = ki;
    g_linefollow_pid_kd = kd;
}

/**
  * @brief  Set PID output limit
  * @param  limit: Maximum speed difference output
  * @retval None
  */
void LineFollow_SetOutputLimit(float limit)
{
    g_linefollow_pid_output_limit = limit;
}

/**
  * @brief  Set integral limit (for anti-windup)
  * @param  limit: Integral windup limit
  * @retval None
  */
void LineFollow_SetIntegralLimit(float limit)
{
    g_linefollow_pid_integral_limit = limit;
}

/**
  * @brief  Set control period in ms
  * @param  period_ms: Control loop period in milliseconds
  * @retval None
  */
void LineFollow_SetControlPeriod(uint16_t period_ms)
{
    g_linefollow_control_period = (period_ms < 10) ? 10 : period_ms;
}

/**
  * @brief  Get current weighted sum (for debugging)
  * @note   Returns the weighted sum calculated in the last update cycle
  * @retval int32_t: Current weighted sum (-5 to +5)
  */
int32_t LineFollow_GetWeightedSum(void)
{
    SENSOR_Status_t sensors;
    return LineFollow_CalcWeightedSum(&sensors);
}

/**
  * @brief  Get current left wheel speed
  * @retval uint16_t: Left wheel speed (0-100)
  */
uint16_t LineFollow_GetLeftSpeed(void)
{
    return s_left_speed;
}

/**
  * @brief  Get current right wheel speed
  * @retval uint16_t: Right wheel speed (0-100)
  */
uint16_t LineFollow_GetRightSpeed(void)
{
    return s_right_speed;
}
