/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor.h"
#include "sys.h"
#include "motor.h"
#include "linefollow.h"
#include "triangle.h"
#include "circle.h"
#include "linefollowsequence.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SENSOR_Status_t sensors;
volatile uint32_t ms_counter = 0;
volatile char flag_5ms = 0;
volatile char flag_20ms = 0;
volatile char flag_100ms = 0;
volatile char flag_200ms = 0;
// uint16_t base_speed = 18;
// #define UART_RX_BUF_SIZE    32
// static char     g_uart_rx_buf[UART_RX_BUF_SIZE];
// static uint8_t  g_uart_rx_idx = 0;
// static uint8_t  g_uart_rx_byte = 0;
// volatile char   g_uart_rx_line_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  Motor_Init();
  Motor_SpeedPID_Init();
  SENSOR_Init();
  LineFollow_Init();
  Triangle_Init();
  // Circle_Init();
  // Mode_Init();

  // HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 5ms: inner motor speed PI */
    if (flag_5ms) {
      flag_5ms = 0;
      SENSOR_ReadRaw(&sensors);
      g_motor_speed_pid_enable = 1;
      Motor_SpeedPID_UpdateAll();
    }

    /* 100ms: outer linefollow + Firewater telemetry */
    if (flag_20ms) {
      flag_20ms = 0;
      LFS_Update();
      // Circle_Update();
      // LineFollow_Update();
      // Triangle_Update();

      /* Firewater binary frame: 4 channels (target_L, actual_L, target_R, actual_R) */
      // float ch[4];
      // ch[0] = (float)LineFollow_GetLeftSpeed()  / 100.0f * g_motor_speed_pid_max_rpm;
      // ch[1] = Motor_GetActualRPM(MOTOR_ID_A);
      // ch[2] = (float)LineFollow_GetRightSpeed() / 100.0f * g_motor_speed_pid_max_rpm;
      // ch[3] = Motor_GetActualRPM(MOTOR_ID_B);

      // printf("%.3f,%.3f,%.3f,%.3f\n", ch[0], ch[1], ch[2], ch[3]);
    }

    /* UART command: host sends kp,ki,kd\n -> sscanf -> SetPID */
    // if (g_uart_rx_line_ready) {
    //   g_uart_rx_line_ready = 0;
    //   float kp, ki, kd;
    //   if (sscanf(g_uart_rx_buf, "%f,%f,%f", &kp, &ki, &kd) == 3) {
    //     LineFollow_SetPID(kp, ki, kd);
    //     printf("PID: kp=%.3f ki=%.4f kd=%.4f\r\n", kp, ki, kd);
    //   }
    // }
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM2) {
      ms_counter++;
      if (ms_counter % 5 == 0)    flag_5ms = 1;
      if (ms_counter % 20 == 0)   flag_20ms = 1;
      if (ms_counter % 100 == 0)  flag_100ms = 1;
      if (ms_counter % 200 == 0)  flag_200ms = 1;
  }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance != USART1) return;

//     if (g_uart_rx_byte == '\r') {
//         /* skip CR from terminal \r\n */
//     } else if (g_uart_rx_byte == '\n') {
//         g_uart_rx_buf[g_uart_rx_idx] = '\0';
//         g_uart_rx_line_ready = 1;
//         g_uart_rx_idx = 0;
//     } else {
//         if (g_uart_rx_idx < (UART_RX_BUF_SIZE - 1))
//             g_uart_rx_buf[g_uart_rx_idx++] = (char)g_uart_rx_byte;
//     }
//     HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1);
// }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
