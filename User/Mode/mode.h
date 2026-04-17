/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mode.h
  * @brief          : Mode control system header file
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

#ifndef __MODE_H
#define __MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Exported types ------------------------------------------------------------*/
/* Mode enumeration */
typedef enum {
    MODE_TRIANGLE,  /* Triangle pattern mode */
    MODE_CIRCLE,    /* Circle pattern mode */
    MODE_FULL,      /* Full功能 mode */
    MODE_COUNT      /* Total number of modes (not a real mode) */
} Mode_t;

/* Exported constants --------------------------------------------------------*/
/* Mode switch interval in milliseconds */
#define MODE_SWITCH_INTERVAL_MS    100

/* Exported variables --------------------------------------------------------*/
/* Current mode variable */
extern Mode_t g_current_mode;

/* Exported function prototypes ---------------------------------------------*/
/* Initialize mode system */
void Mode_Init(void);

/* Update mode - call every 100ms to check for mode switch */
void Mode_Update(void);

/* Get current mode */
Mode_t Mode_GetCurrent(void);

/* Set mode directly */
void Mode_Set(Mode_t mode);

/* Switch to next mode */
void Mode_SwitchToNext(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODE_H */
