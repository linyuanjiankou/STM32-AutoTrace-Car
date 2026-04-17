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
#include "sys.h"

/* USER CODE BEGIN 0 */

/* Private variables ---------------------------------------------------------*/
/* Motor structure instances */
static Motor_t Motor_A =
{
    .id = MOTOR_ID_A,
    .direction = MOTOR_STOP,
    .speed = 0,
    .speed_rpm = 0,
    .encoder_count = 0,
    .last_encoder_count = 0,
};

static Motor_t Motor_B =
{
    .id = MOTOR_ID_B,
    .direction = MOTOR_STOP,
    .speed = 0,
    .speed_rpm = 0,
    .encoder_count = 0,
    .last_encoder_count = 0,
};

/* USER CODE END 0 */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Set motor direction using GPIO pins
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @param  direction: Direction to set (MOTOR_FWD, MOTOR_BWD, MOTOR_STOP)
  */
static void Motor_SetDirectionGPIO(uint8_t motor_id, MotorDirection_t direction)
{
    if (motor_id == MOTOR_ID_A)
    {
        switch (direction)
        {
            case MOTOR_FWD:
                HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_RESET);
                break;
            case MOTOR_BWD:
                HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_SET);
                break;
            default: /* MOTOR_STOP */
                HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_RESET);
                break;
        }
    }
    else if (motor_id == MOTOR_ID_B)
    {
        switch (direction)
        {
            case MOTOR_BWD:
                HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_RESET);
                break;
            case MOTOR_FWD:
                HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_SET);
                break;
            default: /* MOTOR_STOP */
                HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_RESET);
                break;
        }
    }
}

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
    Motor_SetDirectionGPIO(MOTOR_ID_A, MOTOR_STOP);

    /* Set initial speed to 0 */
    Motor_A.speed = 0;
    LL_TIM_OC_SetCompareCH4(TIM1, 0);

    /* Reset encoder count using LL library */
    LL_TIM_SetCounter(TIM4, 0);
    Motor_A.encoder_count = 0;
    Motor_A.last_encoder_count = 0;

    /* Start PWM output for Motor A (TIM1_CH4) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


}

/**
  * @brief  Initialize Motor B (Right Motor)
  * @note   PB14, PB15 for direction control; PA8 for PWM (TIM1_CH1)
  *         Encoder: PA6, PA7 (TIM3_CH1, TIM3_CH2) with X4 quadrature decoding
  */
void Motor_B_Init(void)
{
    /* Set initial direction to stop */
    Motor_SetDirectionGPIO(MOTOR_ID_B, MOTOR_STOP);

    /* Set initial speed to 0 */
    Motor_B.speed = 0;
    LL_TIM_OC_SetCompareCH1(TIM1, 0);

    /* Reset encoder count using LL library */
    LL_TIM_SetCounter(TIM3, 0);
    Motor_B.encoder_count = 0;
    Motor_B.last_encoder_count = 0;

    /* Start PWM output for Motor B (TIM1_CH1) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
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
    /* Set direction via GPIO */
    Motor_SetDirectionGPIO(motor_id, direction);

    /* Update direction in motor structure */
    if (motor_id == MOTOR_ID_A)
    {
        Motor_A.direction = direction;
    }
    else if (motor_id == MOTOR_ID_B)
    {
        Motor_B.direction = direction;
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
    }
    else if (motor_id == MOTOR_ID_B)
    {
        LL_TIM_SetCounter(TIM3, 0);
        Motor_B.encoder_count = 0;
        Motor_B.last_encoder_count = 0;
    }
}

/**
  * @brief  Get total encoder count (with direction) for specified motor
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Total encoder count (positive for forward, negative for backward)
  */
void Motor_UpdateEncoderCount(void)
{
    int32_t current_count_a = 0;
    int32_t current_count_b = 0;

    current_count_a = LL_TIM_GetCounter(TIM4);
    Motor_A.encoder_count = current_count_a;

    current_count_b = LL_TIM_GetCounter(TIM3);
    printf("Encoder Count B: %d\r\n", current_count_b);
    Motor_B.encoder_count = current_count_b;

}

uint32_t Motor_GetEncoderCount(uint8_t motor_id)
{
    if (motor_id == MOTOR_ID_A)
    {
        return Motor_A.encoder_count;
    }
    else if (motor_id == MOTOR_ID_B)
    {
        return Motor_B.encoder_count;
    }
    else return 0;
}
/**
  * @brief  Calculate motor speed in RPM
  * @note   Using integer math with fixed-point arithmetic (scale factor = 1000)
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Speed in RPM (multiplied by 1000 for integer precision)
  */
void MotorA_UpdateSpeedRPM(void)
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
            Motor_A.speed_rpm = speed_rpm;
        }

        last_update_time = current_time;
    }
}

void MotorB_UpdateSpeedRPM(void)
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
        current_count = LL_TIM_GetCounter(TIM3);
        delta_count = (int32_t)(current_count - Motor_B.last_encoder_count);
        Motor_B.last_encoder_count = current_count;

        /* Calculate RPM using integer math:
            * speed_rpm = (delta_count * 60 * 1000) / (counts_per_rev * dt_seconds)
            * dt_ms is in milliseconds, so dt_seconds = dt_ms / 1000
            * Simplified: speed_rpm = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
            * Using scale factor of 1000: speed_rpm_scaled = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
            */
        if (dt_ms > 0)
        {
            speed_rpm = (uint32_t)((uint64_t)delta_count * 60000000UL / ((uint64_t)ENCODER_COUNT_PER_REV * dt_ms));
            Motor_B.speed_rpm = speed_rpm;
        }

        last_update_time = current_time;
    }
}

void Motor_UpdateSpeedRPM(void){
    MotorA_UpdateSpeedRPM();
    MotorB_UpdateSpeedRPM();
}

uint32_t Motor_GetSpeedRPM(uint8_t motor_id){
    if (motor_id == MOTOR_ID_A){
        return Motor_A.speed_rpm;
    }
    else if (motor_id == MOTOR_ID_B){
        return Motor_B.speed_rpm;
    }
    else return 0;
}

/* USER CODE END 1 */
