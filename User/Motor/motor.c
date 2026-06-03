/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor driver source file for TB6621FNG
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
#include "motor.h"
#include "sys.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"

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
/**
  * @note  Direction pin mapping is intentionally reversed for Motor B (FWD↔BWD)
  *        because Motor B is physically mounted backwards on the chassis.
  *        MOTOR_FWD for Motor B still means "car moves forward".
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
/* Base PWM storage for speed PID overlay */
static uint16_t s_motor_a_base_pwm = 0;
static uint16_t s_motor_b_base_pwm = 0;

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
        s_motor_a_base_pwm = pwm_value;
        /* When speed PID is disabled, write PWM directly (backward compatible) */
        if (g_motor_speed_pid_enable == 0)
        {
            LL_TIM_OC_SetCompareCH4(TIM1, pwm_value);
        }
    }
    else if (motor_id == MOTOR_ID_B)
    {
        Motor_B.speed = speed;
        s_motor_b_base_pwm = pwm_value;
        /* When speed PID is disabled, write PWM directly (backward compatible) */
        if (g_motor_speed_pid_enable == 0)
        {
            LL_TIM_OC_SetCompareCH1(TIM1, pwm_value);
        }
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
static void Motor_UpdateSpeedRPM_Single(uint8_t motor_id)
{
    uint32_t speed_rpm = 0;
    int32_t delta_count = 0;
    static uint32_t last_update_ms[2] = {0, 0};
    uint32_t current_count = 0;
    int idx = (motor_id == MOTOR_ID_A) ? 0 : 1;
    TIM_TypeDef *encoder_tim = (motor_id == MOTOR_ID_A) ? TIM4 : TIM3;
    Motor_t *motor = (motor_id == MOTOR_ID_A) ? &Motor_A : &Motor_B;

    /* Calculate time difference (in milliseconds, scaled by 1000 for integer math) */
    uint32_t dt_ms = ms_counter - last_update_ms[idx];

    if (dt_ms >= 100)  /* Update every 100ms */
    {
        current_count = LL_TIM_GetCounter(encoder_tim);
        delta_count = (int32_t)(current_count - motor->last_encoder_count);
        motor->last_encoder_count = current_count;

        /* Calculate RPM using integer math:
            * speed_rpm = (delta_count * 60 * 1000) / (counts_per_rev * dt_seconds)
            * dt_ms is in milliseconds, so dt_seconds = dt_ms / 1000
            * Simplified: speed_rpm = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
            * Using scale factor of 1000: speed_rpm_scaled = (delta_count * 60 * 1000 * 1000) / (counts_per_rev * dt_ms)
            */
        if (dt_ms > 0)
        {
            speed_rpm = (uint32_t)((uint64_t)delta_count * 60000000UL / ((uint64_t)ENCODER_COUNT_PER_REV * dt_ms));
            motor->speed_rpm = speed_rpm;
        }

        last_update_ms[idx] = ms_counter;
    }
}

void MotorA_UpdateSpeedRPM(void)
{
    Motor_UpdateSpeedRPM_Single(MOTOR_ID_A);
}

void MotorB_UpdateSpeedRPM(void)
{
    Motor_UpdateSpeedRPM_Single(MOTOR_ID_B);
}

void Motor_UpdateSpeedRPM(void){
    Motor_UpdateSpeedRPM_Single(MOTOR_ID_A);
    Motor_UpdateSpeedRPM_Single(MOTOR_ID_B);
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

/*----------------------------------------------------------------------------*/
/* Speed PID Control (Incremental PI)                                         */
/*----------------------------------------------------------------------------*/

/* Motor channel abstraction: hardware config + PID state */
typedef struct
{
    TIM_TypeDef    *encoder_tim;      /* TIM4(A) / TIM3(B) */
    uint32_t        pwm_channel;      /* TIM_CHANNEL_4(A) / TIM_CHANNEL_1(B) */
    uint16_t       *base_pwm_ptr;     /* Points to s_motor_a_base_pwm or s_motor_b_base_pwm */
    MotorPIDState_t pid;
    int32_t         last_enc_count;
    float           actual_rpm;        /* Latest computed RPM (by PI controller) */
} MotorChannel_t;

static MotorChannel_t g_motor_ch[2];  /* 0=A, 1=B */

/* Global tunable parameters (default: disabled, gains need calibration) */
uint8_t g_motor_speed_pid_enable = 0;
float g_motor_speed_pid_max_rpm = 800.0f;
uint16_t g_motor_speed_pid_period = 5;     /* Speed PID update period, default 5ms */

float g_motor_speed_pid_kp = 0.5f;
float g_motor_speed_pid_ki = 0.01f;

float g_motor_speed_pid_output_limit = 19661.0f;   /* Max absolute PID output correction */

/**
  * @brief  Incremental PI PID calculation for one motor channel
  * @param  ch: Pointer to the motor channel
  * @param  target_rpm: Target RPM (always positive)
  * @param  dt: Time delta in seconds
  */
static void MotorChannel_PID_Calc(MotorChannel_t *ch, float target_rpm, float dt)
{
    int32_t current_enc = (int32_t)LL_TIM_GetCounter(ch->encoder_tim);
    int32_t delta = current_enc - ch->last_enc_count;
    ch->last_enc_count = current_enc;

    /* Calculate actual RPM (output shaft), use absolute value */
    float actual_rpm = (dt > 0.0f) ? (float)delta * 60.0f / ((float)ENCODER_COUNT_PER_REV * dt) : 0.0f;
    if (actual_rpm < 0.0f) actual_rpm = -actual_rpm;

    /* Low-pass filter to suppress encoder quantization noise */
    ch->pid.filtered_rpm += 0.5f * (actual_rpm - ch->pid.filtered_rpm);

    /* Error: positive means we need more speed (use filtered RPM) */
    float error = target_rpm - ch->pid.filtered_rpm;

    /* Incremental PI: delta_output = Kp*(error - prev_error) + Ki*error */
    float delta_output = g_motor_speed_pid_kp * (error - ch->pid.prev_error)
                       + g_motor_speed_pid_ki * error;

    /* Accumulate total_output */
    ch->pid.total_output += delta_output;

    /* Clamp total_output to output_limit to prevent integral windup */
    if (ch->pid.total_output > ch->pid.output_limit)
        ch->pid.total_output = ch->pid.output_limit;
    else if (ch->pid.total_output < -ch->pid.output_limit)
        ch->pid.total_output = -ch->pid.output_limit;

    /* Apply correction to base PWM, clamp to [0, MOTOR_PWM_PERIOD] */
    int32_t final_pwm = (int32_t)(*(ch->base_pwm_ptr)) + (int32_t)ch->pid.total_output;

    /* Clamping anti-windup: back-calculate total_output when PWM saturates,
     * so the integral doesn't keep winding up in the saturated direction */
    if (final_pwm > (int32_t)MOTOR_PWM_PERIOD) {
        final_pwm = (int32_t)MOTOR_PWM_PERIOD;
        ch->pid.total_output = (float)((int32_t)MOTOR_PWM_PERIOD - (int32_t)(*(ch->base_pwm_ptr)));
    } else if (final_pwm < 0) {
        final_pwm = 0;
        ch->pid.total_output = (float)(-(int32_t)(*(ch->base_pwm_ptr)));
    }

    /* Write to timer compare register */
    if (ch->pwm_channel == TIM_CHANNEL_4)
        LL_TIM_OC_SetCompareCH4(TIM1, (uint16_t)final_pwm);
    else
        LL_TIM_OC_SetCompareCH1(TIM1, (uint16_t)final_pwm);

    /* Store computed RPM and shift error history */
    ch->actual_rpm = ch->pid.filtered_rpm;
    ch->pid.prev_prev_error = ch->pid.prev_error;
    ch->pid.prev_error = error;
}

/**
  * @brief  Initialize speed PID controller
  */
void Motor_SpeedPID_Init(void)
{
    /* Initialize Motor A channel */
    g_motor_ch[0].encoder_tim  = TIM4;
    g_motor_ch[0].pwm_channel  = TIM_CHANNEL_4;
    g_motor_ch[0].base_pwm_ptr = &s_motor_a_base_pwm;
    g_motor_ch[0].pid.total_output    = 0.0f;
    g_motor_ch[0].pid.prev_error      = 0.0f;
    g_motor_ch[0].pid.prev_prev_error = 0.0f;
    g_motor_ch[0].pid.output_limit    = g_motor_speed_pid_output_limit;
    g_motor_ch[0].pid.filtered_rpm    = 0.0f;
    g_motor_ch[0].last_enc_count      = (int32_t)LL_TIM_GetCounter(TIM4);
    g_motor_ch[0].actual_rpm           = 0.0f;

    /* Initialize Motor B channel */
    g_motor_ch[1].encoder_tim  = TIM3;
    g_motor_ch[1].pwm_channel  = TIM_CHANNEL_1;
    g_motor_ch[1].base_pwm_ptr = &s_motor_b_base_pwm;
    g_motor_ch[1].pid.total_output    = 0.0f;
    g_motor_ch[1].pid.prev_error      = 0.0f;
    g_motor_ch[1].pid.prev_prev_error = 0.0f;
    g_motor_ch[1].pid.output_limit    = g_motor_speed_pid_output_limit;
    g_motor_ch[1].pid.filtered_rpm    = 0.0f;
    g_motor_ch[1].last_enc_count      = (int32_t)LL_TIM_GetCounter(TIM3);
    g_motor_ch[1].actual_rpm           = 0.0f;

}

/**
  * @brief  Update all motor speed PID controllers (call periodically)
  */
void Motor_SpeedPID_UpdateAll(void)
{
    if (g_motor_speed_pid_enable == 0)
        return;

    float dt = (float)g_motor_speed_pid_period / 1000.0f;

    /* Motor A */
    float target_rpm_a = ((float)Motor_A.speed / 100.0f) * g_motor_speed_pid_max_rpm;
    MotorChannel_PID_Calc(&g_motor_ch[0], target_rpm_a, dt);

    /* Motor B */
    float target_rpm_b = ((float)Motor_B.speed / 100.0f) * g_motor_speed_pid_max_rpm;
    MotorChannel_PID_Calc(&g_motor_ch[1], target_rpm_b, dt);
}

/**
  * @brief  Set PID gains and reset PID state to avoid PWM jump
  * @param  kp: Proportional gain
  * @param  ki: Integral gain
  * @param  kd: Derivative gain (reserved, set to 0.0f)
  */
void Motor_SetPID(float kp, float ki, float output_limit)
{
    g_motor_speed_pid_kp = kp;
    g_motor_speed_pid_ki = ki;
    g_motor_speed_pid_output_limit = output_limit;

    /* Reset PID states on gain change to prevent PWM jump */
    for (int i = 0; i < 2; i++)
    {
        g_motor_ch[i].pid.total_output    = 0.0f;
        g_motor_ch[i].pid.prev_error      = 0.0f;
        g_motor_ch[i].pid.prev_prev_error = 0.0f;
        g_motor_ch[i].pid.filtered_rpm    = g_motor_ch[i].actual_rpm;
        g_motor_ch[i].pid.output_limit    = output_limit;
    }
}

/**
  * @brief  Get the latest actual RPM computed by the PI controller
  * @param  motor_id: Motor ID (MOTOR_ID_A or MOTOR_ID_B)
  * @retval Actual RPM as float (output shaft RPM)
  */
float Motor_GetActualRPM(uint8_t motor_id)
{
    int idx = (motor_id == MOTOR_ID_A) ? 0 : 1;
    return g_motor_ch[idx].actual_rpm;
}
/* USER CODE END 1 */

void Motor_ResetPIDOutput(void){
    g_motor_ch[0].pid.total_output = 0.0f;
    g_motor_ch[1].pid.total_output = 0.0f;
}
