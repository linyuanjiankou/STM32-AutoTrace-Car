/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Motor driver header file for TB6621FNG
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

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_bus.h"

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Motor direction enumeration */
typedef enum
{
    MOTOR_STOP = 0,    /* Stop */
    MOTOR_FWD,         /* Forward */
    MOTOR_BWD          /* Backward */
} MotorDirection_t;

/* Motor structure */
typedef struct
{
    uint8_t id;                    /* Motor ID: 1 for Motor A, 2 for Motor B */
    MotorDirection_t direction;    /* Current direction */
    uint16_t speed;                /* Speed value (0-100%) - setpoint for PID */
    uint32_t encoder_count;        /* Encoder count */
    uint32_t last_encoder_count;   /* Last encoder count for speed calculation */
    int32_t total_encoder_count;   /* Total encoder count (for position) */
} Motor_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Motor ID definitions */
#define MOTOR_ID_A                    1
#define MOTOR_ID_B                    2

/* Motor pin definitions - Motor A (Left Motor) */
#define MOTOR_A_DIR1_PIN              GPIO_PIN_12
#define MOTOR_A_DIR1_PORT             GPIOB
#define MOTOR_A_DIR2_PIN              GPIO_PIN_13
#define MOTOR_A_DIR2_PORT             GPIOB
#define MOTOR_A_PWM_PIN               GPIO_PIN_11
#define MOTOR_A_PWM_PORT              GPIOA
#define MOTOR_A_ENCODER_TIM           htim4
#define MOTOR_A_ENCODER_CH1_PIN       GPIO_PIN_6
#define MOTOR_A_ENCODER_CH1_PORT      GPIOB
#define MOTOR_A_ENCODER_CH2_PIN       GPIO_PIN_7
#define MOTOR_A_ENCODER_CH2_PORT      GPIOB

/* Motor pin definitions - Motor B (Right Motor) */
#define MOTOR_B_DIR1_PIN              GPIO_PIN_14
#define MOTOR_B_DIR1_PORT             GPIOB
#define MOTOR_B_DIR2_PIN              GPIO_PIN_15
#define MOTOR_B_DIR2_PORT             GPIOB
#define MOTOR_B_PWM_PIN               GPIO_PIN_8
#define MOTOR_B_PWM_PORT              GPIOA
#define MOTOR_B_ENCODER_TIM           htim2
#define MOTOR_B_ENCODER_CH1_PIN       GPIO_PIN_0
#define MOTOR_B_ENCODER_CH1_PORT      GPIOA
#define MOTOR_B_ENCODER_CH2_PIN       GPIO_PIN_1
#define MOTOR_B_ENCODER_CH2_PORT      GPIOA

/* Speed control - Macro definitions for speed levels */
#define MOTOR_SPEED_MIN               0
#define MOTOR_SPEED_LOW               30          /* Low speed: 30% duty cycle */
#define MOTOR_SPEED_MEDIUM            50          /* Medium speed: 50% duty cycle */
#define MOTOR_SPEED_HIGH              70          /* High speed: 70% duty cycle */
#define MOTOR_SPEED_MAX               100         /* Maximum speed: 100% duty cycle */

/* PWM period for speed control (based on TIM1 clock) */
#define MOTOR_PWM_PERIOD              1000        /* PWM period value */

/* Encoder related definitions */
/*
 * Encoder pulses per revolution (counts per revolution)
 * PPR = 13 (Pulses Per Revolution from motor datasheet)
 * Using X4 quadrature encoding (TI12_EDGE mode with both edges)
 * Encoder is mounted on MOTOR SHAFT (before gear reduction)
 * Gear ratio = 20:1 (motor shaft to output shaft)
 *
 * ENCODER_COUNT_PER_REV = 13 * 4 * 20 = 1040
 *
 * Note: When encoder is on motor shaft, the counts represent motor shaft rotation.
 * To get output shaft RPM: output_rpm = motor_rpm / 20
 * For RPM calculation, the formula automatically accounts for this ratio.
 */
#define ENCODER_COUNT_PER_REV        1040


/* Speed RPM calculation notes:
 *
 * Motor_GetSpeedRPM() returns RPM multiplied by 1000 (integer fixed-point format).
 * To get actual RPM: actual_rpm = returned_value / 1000.0f
 *
 * Encoder specifications:
 * - PPR = 13 (from motor datasheet)
 * - X4 quadrature mode: multiplies counts by 4
 * - Gear ratio = 20:1 (encoder on motor shaft, before reduction)
 * - ENCODER_COUNT_PER_REV = 13 * 4 * 20 = 1040
 *
 * The RPM returned by Motor_GetSpeedRPM() represents the MOTOR SHAFT RPM.
 * To get OUTPUT SHAFT RPM (after gear reduction): output_rpm = motor_rpm / 20
 *
 * Parameters that need to be adjusted based on your motor:
 * 1. ENCODER_COUNT_PER_REV - Encoder pulses per revolution (see above)
 *    - Encoder on motor shaft (before gear reduction): PPR * 4 * gear_ratio
 *    - Encoder on output shaft (after gear reduction): PPR * 4
 *    - For X4 mode: multiply PPR by 4
 *
 * 2. Update interval (currently 100ms) - in Motor_GetSpeedRPM():
 *    - Shorter interval: better time resolution but more noise
 *    - Longer interval: smoother but slower response
 *    - Formula: dt_ms = current_time - last_update_time (using HAL_GetTick())
 *
 * 3. Calculation formula: RPM = (delta_count * 60 * 1000) / (counts_per_rev * dt_ms)
 *    - delta_count: encoder count difference
 *    - counts_per_rev: ENCODER_COUNT_PER_REV (1040 for this motor)
 *    - dt_ms: time interval in milliseconds
 *    - Result: RPM * 1000 (fixed-point format) - represents MOTOR SHAFT RPM
 *
 * Example calculations (with ENCODER_COUNT_PER_REV = 1040):
 * - If delta_count = 1040 in 100ms, motor_rpm = (1040 * 60000) / (1040 * 100) = 600 RPM
 * - Output shaft RPM = 600 / 20 = 30 RPM
 * - If delta_count = 52 in 100ms, motor_rpm = (52 * 60000) / (1040 * 100) = 30 RPM
 * - Output shaft RPM = 30 / 20 = 1.5 RPM
 *
 * For accurate speed control, measure your motor's actual RPM
 * and adjust ENCODER_COUNT_PER_REV accordingly.
 */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Direction control macros - Motor A (using HAL GPIO) */
#define MOTOR_A_FORWARD()             do { \
    HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_A_BACKWARD()            do { \
    HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_SET); \
} while(0)

#define MOTOR_A_STOP_DIRECTION()      do { \
    HAL_GPIO_WritePin(MOTOR_A_DIR1_PORT, MOTOR_A_DIR1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOTOR_A_DIR2_PORT, MOTOR_A_DIR2_PIN, GPIO_PIN_RESET); \
} while(0)

/* Direction control macros - Motor B (using HAL GPIO) */
#define MOTOR_B_FORWARD()             do { \
    HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_B_BACKWARD()            do { \
    HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_SET); \
} while(0)

#define MOTOR_B_STOP_DIRECTION()      do { \
    HAL_GPIO_WritePin(MOTOR_B_DIR1_PORT, MOTOR_B_DIR1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(MOTOR_B_DIR2_PORT, MOTOR_B_DIR2_PIN, GPIO_PIN_RESET); \
} while(0)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* Motor initialization functions */
void Motor_Init(void);
void Motor_A_Init(void);
void Motor_B_Init(void);

/* Motor control functions */
void Motor_SetSpeed(uint8_t motor_id, uint16_t speed);
void Motor_SetDirection(uint8_t motor_id, MotorDirection_t direction);
void Motor_Run(uint8_t motor_id, MotorDirection_t direction, uint16_t speed);
void Motor_Stop(uint8_t motor_id);

/* Encoder read functions (using LL library and direct register access) */
/* These functions return values directly - no need to store in struct */
uint32_t Motor_GetEncoderCount(uint8_t motor_id);
void Motor_ResetEncoderCount(uint8_t motor_id);
int32_t Motor_GetTotalEncoderCount(uint8_t motor_id);
uint32_t Motor_GetSpeedRPM(uint8_t motor_id);

/* Motor update function (call in main loop or interrupt) */
/* This function updates encoder counts, but speed_rpm is returned directly */
void Motor_Update(void);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
