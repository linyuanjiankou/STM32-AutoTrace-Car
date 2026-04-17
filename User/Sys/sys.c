#include <stdio.h>
#include "sys.h"
#include "stm32f1xx_hal.h"

/**
  * @brief  Retarget printf to UART
  * @param  ch: Character to send
  * @param  f: File pointer (unused)
  * @retval Character sent
  */
int fputc(int ch, FILE *f)
{
    if (ch == '\n')
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"\r", 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

uint32_t my_abs(int32_t a){
    if (a < 0) return -a;
    else return a;
}
