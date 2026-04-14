/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor driver source file for TB6621FNG
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
#include "motor.h"

/* USER CODE BEGIN 0 */

/* Private variables ---------------------------------------------------------*/
/* Motor structure instances */
static Motor_t Motor_A =
{
    .id = MOTOR_ID_A,
    .direction = MOTOR_STOP,
    .speed = 0,
    .encoder_count = 0,
    .last_encoder_count = 0,
    .total_encoder_count = 0
};

static Motor_t Motor_B =
{
    .id = MOTOR_ID_B,
    .direction = MOTOR_STOP,
    .speed = 0,
    .encoder_count = 0,
    .last_encoder_count = 0,
    .total_encoder_count = 0
};

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Motor Initialization                                                       */
/*----------------------------------------------------------------------------*/

/**
  * @brief  Initialize all motors
  * @note   This function initializes both Motor A and Motor B
  */
void Motor_Init(void)
{
    /* Initialize Motor A */
    Motor_A_Init();

    /* Initialize Motor B */
    Motor_B_Init();
}

/**
  * @brief  Initialize Motor A (Left Motor)
  * @note   PB12, PB13 for direction control; PA11 for PWM (TIM1_CH4)
  *         Encoder: PB6, PB7 (TIM4_CH1, TIM4_CH2) with X4 quadrature decoding
  */
void Motor_A_Init(void)
{
    /* Set initial direction to stop */
    MOTOR_A_STOP_DIRECTION();

    /* Set initial speed to 0 */
    Motor_A.speed = 0;
    LL_TIM_OC_SetCompareCH4(TIM1, 0);

    /* Reset encoder count using LL library */
    LL_TIM_SetCounter(TIM4, 0);
    Motor_A.encoder_count = 0;
    Motor_A.last_encoder_count = 0;
    Motor_A.total_encoder_count = 0;

    /* Enable encoder interface with X4 quadrature decoding */
    LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_EnableCounter(TIM4);
}

/**
  * @brief  Initialize Motor B (Right Motor)
  * @note   PB14, PB15 for direction control; PA8 for PWM (TIM1_CH1)
  *         Encoder: PA0, PA1 (TIM2_CH1, TIM2_CH2) with X4 quadrature decoding
  */
void Motor_B_Init(void)
{
    /* Set initial direction to stop */
    MOTOR_B_STOP_DIRECTION();

    /* Set initial speed to 0 */
    Motor_B.speed = 0;
    LL_TIM_OC_SetCompareCH1(TIM1, 0);

    /* Reset encoder count using LL library */
    LL_TIM_SetCounter(TIM2, 0);
    Motor_B.encoder_count = 0;
    Motor_B.last_encoder_count = 0;
    Motor_B.total_encoder_count = 0;

    /* Enable encoder interface with X4 quadrature decoding */
    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_EnableCounter(TIM2);
}

/*----------------------------------------------------------------------------*/
/* Motor Control Functions                                                    */
/*----------------------------------------------------------------------------*/

/**
  * @brief  Set motor speed
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @param  speed: Speed value (0-100%)
  */
void Motor_SetSpeed(uint8_t motor_id, uint16_t speed)
{
    uint16_t pwm_value;

    /* Limit speed to valid range */
    if (speed > 100)
    {
        speed = 100;
    }

    /* Calculate PWM value based on speed percentage (integer math) */
    pwm_value = (speed * MOTOR_PWM_PERIOD) / 100;

    if (motor_id == MOTOR_ID_A)
    {
        Motor_A.speed = speed;
        LL_TIM_OC_SetCompareCH4(TIM1, pwm_value);
    }
    else if (motor_id == MOTOR_ID_B)
    {
        Motor_B.speed = speed;
        LL_TIM_OC_SetCompareCH1(TIM1, pwm_value);
    }
}

/**
  * @brief  Set motor direction
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @param  direction: Direction (MOTOR_FWD, MOTOR_BWD, or MOTOR_STOP)
  */
void Motor_SetDirection(uint8_t motor_id, MotorDirection_t direction)
{
    if (motor_id == MOTOR_ID_A)
    {
        switch (direction)
        {
            case MOTOR_FWD:
                MOTOR_A_FORWARD();
                Motor_A.direction = MOTOR_FWD;
                break;
            case MOTOR_BWD:
                MOTOR_A_BACKWARD();
                Motor_A.direction = MOTOR_BWD;
                break;
            default: /* MOTOR_STOP */
                MOTOR_A_STOP_DIRECTION();
                Motor_A.direction = MOTOR_STOP;
                break;
        }
    }
    else if (motor_id == MOTOR_ID_B)
    {
        switch (direction)
        {
            case MOTOR_FWD:
                MOTOR_B_FORWARD();
                Motor_B.direction = MOTOR_FWD;
                break;
            case MOTOR_BWD:
                MOTOR_B_BACKWARD();
                Motor_B.direction = MOTOR_BWD;
                break;
            default: /* MOTOR_STOP */
                MOTOR_B_STOP_DIRECTION();
                Motor_B.direction = MOTOR_STOP;
                break;
        }
    }
}

/**
  * @brief  Run motor with specified direction and speed
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @param  direction: Direction (MOTOR_FWD, MOTOR_BWD)
  * @param  speed: Speed value (0-100%)
  */
void Motor_Run(uint8_t motor_id, MotorDirection_t direction, uint16_t speed)
{
    Motor_SetDirection(motor_id, direction);
    Motor_SetSpeed(motor_id, speed);
}

/**
  * @brief  Stop motor
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  */
void Motor_Stop(uint8_t motor_id)
{
    Motor_SetDirection(motor_id, MOTOR_STOP);
    Motor_SetSpeed(motor_id, 0);
}

/*----------------------------------------------------------------------------*/
/* Encoder Read Functions (using LL library and register operations)          */
/*----------------------------------------------------------------------------*/

/**
  * @brief  Get encoder count for specified motor
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Encoder count
  */
uint32_t Motor_GetEncoderCount(uint8_t motor_id)
{
    if (motor_id == MOTOR_ID_A)
    {
        Motor_A.encoder_count = LL_TIM_GetCounter(TIM4);
        return Motor_A.encoder_count;
    }
    else if (motor_id == MOTOR_ID_B)
    {
        Motor_B.encoder_count = LL_TIM_GetCounter(TIM2);
        return Motor_B.encoder_count;
    }
    else return 0;
}

/**
  * @brief  Reset encoder count for specified motor
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  */
void Motor_ResetEncoderCount(uint8_t motor_id)
{
    if (motor_id == MOTOR_ID_A)
    {
        LL_TIM_SetCounter(TIM4, 0);
        Motor_A.encoder_count = 0;
        Motor_A.last_encoder_count = 0;
        Motor_A.total_encoder_count = 0;
    }
    else if (motor_id == MOTOR_ID_B)
    {
        LL_TIM_SetCounter(TIM2, 0);
        Motor_B.encoder_count = 0;
        Motor_B.last_encoder_count = 0;
        Motor_B.total_encoder_count = 0;
    }
}

/**
  * @brief  Get total encoder count (with direction) for specified motor
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Total encoder count (positive for forward, negative for backward)
  */
int32_t Motor_GetTotalEncoderCount(uint8_t motor_id)
{
    int32_t current_count;

    if (motor_id == MOTOR_ID_A)
    {
        current_count = LL_TIM_GetCounter(TIM4);
        Motor_A.total_encoder_count = current_count;
        return current_count;
    }
    else if (motor_id == MOTOR_ID_B)
    {
        current_count = LL_TIM_GetCounter(TIM2);
        Motor_B.total_encoder_count = current_count;
        return current_count;
    }
    else return 0;
}

/**
  * @brief  Calculate motor speed in RPM
  * @note   Using integer math with fixed-point arithmetic (scale factor = 1000)
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Speed in RPM (multiplied by 1000 for integer precision)
  */
uint32_t Motor_GetSpeedRPM(uint8_t motor_id)
{
    uint32_t speed_rpm = 0;
    int32_t delta_count = 0;
    static uint32_t last_update_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t current_count = 0;

    /* Calculate time difference (in milliseconds, scaled by 1000 for integer math) */
    uint32_t dt_ms = current_time - last_update_time;

    if (dt_ms >= 100)  /* Update every 100ms */
    {
        if (motor_id == MOTOR_ID_A)
        {
            current_count = LL_TIM_GetCounter(TIM4);
            delta_count = (int32_t)(current_count - Motor_A.last_encoder_count);
            Motor_A.last_encoder_count = current_count;

            /* Calculate RPM using integer math:
             * speed_rpm = (delta_count * 60 * 1000) / (counts_per_rev * dt_seconds)
             * dt_ms is in milliseconds, so dt_seconds = dt_ms / 1000
             * Simplified: speed_rpm = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
             * Using scale factor of 1000: speed_rpm_scaled = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
             */
            if (dt_ms > 0)
            {
                speed_rpm = (uint32_t)((uint64_t)delta_count * 60000000UL / ((uint64_t)ENCODER_COUNT_PER_REV * dt_ms));
            }
        }
        else if (motor_id == MOTOR_ID_B)
        {
            current_count = LL_TIM_GetCounter(TIM2);
            delta_count = (int32_t)(current_count - Motor_B.last_encoder_count);
            Motor_B.last_encoder_count = current_count;

            if (dt_ms > 0)
            {
                speed_rpm = (uint32_t)((uint64_t)delta_count * 60000000UL / ((uint64_t)ENCODER_COUNT_PER_REV * dt_ms));
            }
        }

        last_update_time = current_time;
    }

    return speed_rpm;
}

/**
  * @brief  Motor update function (call in main loop or interrupt)
  * @note   This function should be called periodically to update motor status
  */
void Motor_Update(void)
{
    Motor_GetEncoderCount(MOTOR_ID_A);
    Motor_GetEncoderCount(MOTOR_ID_B);

    Motor_GetSpeedRPM(MOTOR_ID_A);
    Motor_GetSpeedRPM(MOTOR_ID_B);
}

/* USER CODE END 1 */
