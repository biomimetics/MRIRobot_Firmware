/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void aError_Handler(void)
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

extern UART_HandleTypeDef huart2;

// converts a float value to a milli-[unit] for printing. only works out to two decimal places
int _conv_float(float val){
  return ((int) val * 1000.0f);
}

int _write(int file, char *ptr, int len){
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY);

  // HAL_UART_Transmit_IT(&huart2, (uint8_t *)ptr, len);
  return len;
}

double timer_get_elapsed_time(uint32_t *timer_start){
  uint32_t timer_stop = LOOP_TIMER->CNT;
  uint32_t timer_elapsed = 0;

  if (timer_stop < *timer_start){
    //printf("Timer overflow!\r\n");
    timer_elapsed = ((uint32_t)65535 - *timer_start) + timer_stop;
  }
  else{
    timer_elapsed = timer_stop - *timer_start;
  }
  //timer_elapsed = timer_stop - *timer_start;
  *timer_start = LOOP_TIMER->CNT;

  // Incorporate prescaler
  //uint32_t prescaler = TIM6->PSC; // Timer prescaler register
  double timer_elapsed_time = (double)(timer_elapsed) * (double)(TIM6->PSC + 1) / (double)SystemCoreClock; // time in seconds

  
  //printf("SystemCoreClock: %d\r\n", SystemCoreClock);
  //printf("TIM6->PSC (prescaler): %lu\r\n", TIM6->PSC); // prescaler);
  
  printf("timer_elapsed: %d, timer_elapsed_time: %d us\r\n", timer_elapsed, ((int)(timer_elapsed_time * 1000000)));
  return timer_elapsed_time;
  //return (double)(timer_elapsed) * (double)(LOOP_TIMER->PSC + 1) / (double)SystemCoreClock; //timer_elapsed_time;
}

// this function works great! maybe update the whole thing with its own struct later.
void timer_updated_elapsed_time(uint32_t *timer_start, double *timer_elapsed_sec){
  uint32_t timer_stop = LOOP_TIMER->CNT;
  uint32_t timer_elapsed = 0;

  if (timer_stop < *timer_start){
    //printf("Timer overflow!\r\n");
    timer_elapsed = ((uint32_t)65535 - *timer_start) + timer_stop;
  }
  else{
    timer_elapsed = timer_stop - *timer_start;
  }
  //timer_elapsed = timer_stop - *timer_start;
  *timer_start = LOOP_TIMER->CNT;

  // Incorporate prescaler
  //uint32_t prescaler = TIM6->PSC; // Timer prescaler register
  *timer_elapsed_sec = (double)(timer_elapsed) * (double)(TIM6->PSC + 1) / (double)SystemCoreClock; // time in seconds

  
  //printf("SystemCoreClock: %d\r\n", SystemCoreClock);
  //printf("TIM6->PSC (prescaler): %lu\r\n", TIM6->PSC); // prescaler);
  
  //printf("timer_elapsed: %d, timer_elapsed_time: %d us\r\n", timer_elapsed, ((int)(*timer_elapsed_sec * 1000000)));
  return;
}