/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : linefollowsequence.h
  * @brief          : Line Follow Sequence mode -综合型巡线模式
  *                   Implements 7-state line tracking:
  *                   - Unified straight tracking (STRAIGHT)
  *                   - Dashed line tracking (DASH_STRAIGHT_1, DASH_TURN, DASH_STRAIGHT_2)
  *                   - Rotation state for reorientation
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

#ifndef __LINEFOLLOWSEQUENCE_H
#define __LINEFOLLOWSEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "motor.h"
#include "linefollow.h"
#include "mode.h"

/* Exported types ------------------------------------------------------------*/
/* State machine enumeration - 7 states */
typedef enum {
    LFS_STATE_START,                /* 0: 初始化状态，检测全白进入DASH_STRAIGHT_1 */
    LFS_STATE_STRAIGHT,             /* 1: 统一直行状态 - 使用StraightLine_SetPID() */
    LFS_STATE_CIRCLE,               /* 2: 圆形巡线状态 - 使用Circle_SetPID() */
    LFS_STATE_DASH_STRAIGHT_1,      /* 2: 虚线直线巡线1 */
    LFS_STATE_DASH_TURN,            /* 3: 虚线弯道巡线 - 使用error_dot动态前馈 */
    LFS_STATE_DASH_STRAIGHT_2,      /* 4: 虚线直线巡线2 - 忽略4+黑线检测 */
    LFS_STATE_ROTATION,             /* 5: 旋转 - 检测黑线后切换 */
    LFS_STATE_STOP                  /* 6: STOP - 停止 */
} LFS_State_t;

/* Exported constants --------------------------------------------------------*/
/* LFS Mode Parameters - adjust these for tuning */
#define LFS_STRAIGHT_SPEED      20      /* 直线巡线基础速度 */
#define LFS_CIRCLE_SPEED        10      /* 圆形巡线基础速度 */
#define LFS_DASH_SPEED          10      /* 虚线巡线基础速度 */

#define LFS_STRAIGHT_OUTPUT     50.0f   /* 直线输出限制 */
#define LFS_CIRCLE_OUTPUT       40.0f   /* 圆形输出限制 */
#define LFS_DASH_OUTPUT         30.0f   /* 虚线转弯输出限制 */

/* Feedforward gain for error_dot calculation */
#define LFS_K_FF                  0.5f      /* Feedforward gain K_ff */

#define LFS_LARGE_DEV_THRESHOLD   4     /* 大偏差阈值 */
#define LFS_LOST_ROTATION_SPEED   15    /* Lost line rotation speed */

/* Circle state - encoder-based transition */
/* Encoder counts per revolution = 1040 (PPR=13 * 4 * gear_ratio=20) */
#define LFS_CIRCLE_REVOLUTIONS    3     /* Number of wheel revolutions in Circle state */
#define LFS_CIRCLE_COUNT_PER_REV  1040  /* Encoder counts per revolution */
#define LFS_CIRCLE_TARGET_COUNT   (LFS_CIRCLE_REVOLUTIONS * LFS_CIRCLE_COUNT_PER_REV)

/* Straight to Circle transition - first threshold */
#define LFS_STRAIGHT_TO_CIRCLE_REVOLUTIONS    5     /* 圈数阈值 */
#define LFS_STRAIGHT_TO_CIRCLE_COUNT   (LFS_STRAIGHT_TO_CIRCLE_REVOLUTIONS * LFS_CIRCLE_COUNT_PER_REV)

/* Circle to Straight transition - second threshold */
#define LFS_CIRCLE_TO_STRAIGHT_REVOLUTIONS    3     /* 圈数阈值 */
#define LFS_CIRCLE_TO_STRAIGHT_COUNT   (LFS_CIRCLE_TO_STRAIGHT_REVOLUTIONS * LFS_CIRCLE_COUNT_PER_REV)

/* Counters */
extern int g_lfs_detection_count; /* Detection count for dashed line states */

/* State transition tracking variables */
extern uint32_t g_lfs_stable_start_tick;      /* Track stable state duration */
extern uint8_t g_lfs_intersection_count;      /* Count intersection detections */

/* Circle state variables - encoder-based transition */
extern uint32_t g_lfs_circle_start_count;     /* Encoder count when entering Circle state */
extern uint32_t g_lfs_circle_target_count;    /* Target encoder count for Circle state */

/*Straight to Circle transition tracking */
extern uint32_t g_lfs_straight_to_circle_count;

/* Exported function prototypes ---------------------------------------------*/
/* Get deviation (weighted sum position) */
float LFS_GetDeviation(void);

/* Get absolute deviation */
float LFS_GetAbsDeviation(void);

/* Check if all sensors detect white */
uint8_t LFS_IsAllWhite(SENSOR_Status_t *sensors);

/* Check if all sensors detect black */
uint8_t LFS_IsAllBlack(SENSOR_Status_t *sensors);

/* Check if intersection detected (4+ black lines) */
uint8_t LFS_IsIntersection(SENSOR_Status_t *sensors);

#ifdef __cplusplus
}
#endif

#endif /* __LINEFOLLOWSEQUENCE_H */
