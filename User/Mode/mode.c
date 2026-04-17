/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mode.c
  * @brief          : Mode control system source file
  * @note          : This module handles mode switching based on button input.
  *                  The button check is performed every 100ms (MODE_SWITCH_INTERVAL_MS).
  *                  When a button press is detected, the mode switches to the next one.
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
#include "mode.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
/* Current mode */
Mode_t g_current_mode = MODE_TRIANGLE;

/* Button pin definition - modify according to your hardware */
#define MODE_BUTTON_PIN       GPIO_PIN_13
#define MODE_BUTTON_PORT      GPIOC

/* Last update tick for mode checking */
static uint32_t s_last_mode_check_tick = 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t Button_GetState(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize mode system
  * @retval None
  */
void Mode_Init(void)
{
    g_current_mode = MODE_TRIANGLE;
    s_last_mode_check_tick = HAL_GetTick();
    printf("\r\n=== Mode System Init ===\r\n");
    printf("Initial mode: MODE_TRIANGLE\r\n\r\n");
}

/**
  * @brief  Get current mode
  * @retval Mode_t: Current mode
  */
Mode_t Mode_GetCurrent(void)
{
    return g_current_mode;
}

/**
  * @brief  Set mode directly
  * @param  mode: Mode to set
  * @retval None
  */
void Mode_Set(Mode_t mode)
{
    if (mode < MODE_COUNT) {
        g_current_mode = mode;
        printf("Mode switched to: %d\r\n", (int)mode);
    }
}

/**
  * @brief  Switch to next mode (circular)
  * @retval None
  */
void Mode_SwitchToNext(void)
{
    g_current_mode = (Mode_t)((g_current_mode + 1) % MODE_COUNT);
    printf("Mode switched to: %d\r\n", (int)g_current_mode);
}

/**
  * @brief  Update mode - call every 100ms to check for mode switch
  * @note   This function checks the button state every 100ms.
  *         If button is pressed (active low), it waits for release and then switches mode.
  * @retval None
  */
void Mode_Update(void)
{
    uint32_t current_tick = HAL_GetTick();

    /* Check if it's time to check for mode switch */
    if (current_tick - s_last_mode_check_tick >= MODE_SWITCH_INTERVAL_MS) {
        s_last_mode_check_tick = current_tick;

        /* Check button state (active low, returns 0 when pressed) */
        if (Button_GetState() == 0) {
            /* Wait for debounce */
            HAL_Delay(10);

            /* Confirm button is still pressed */
            if (Button_GetState() == 0) {
                /* Wait for button release */
                while (Button_GetState() == 0) {
                    HAL_Delay(10);
                    /* Timeout to prevent infinite loop */
                    if (HAL_GetTick() - current_tick > 500) {
                        break;
                    }
                }

                /* Button released, switch to next mode */
                Mode_SwitchToNext();
            }
        }
    }
}

/**
  * @brief  Get button state
  * @note   Returns 0 when pressed (active low), 1 when released
  * @retval uint8_t: Button state
  */
static uint8_t Button_GetState(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(MODE_BUTTON_PORT, MODE_BUTTON_PIN);
}
