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
#include "canopen_cmd.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
# include "lidar_reader.h"
# include "lipkg_copy.h"
# include "stm32g4xx_hal_def.h"
# include <stdio.h>
# include <memory.h>

# include "CO_app_STM32.h"

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
# define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

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
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /** CANopen stack initialization */
  CANopenNodeSTM32 canopenNodeSTM32;
  canopenNodeSTM32.CANHandle = &hfdcan1;
  canopenNodeSTM32.HWInitFunction = MX_FDCAN1_Init;
  canopenNodeSTM32.timerHandle = &htim6;
  canopenNodeSTM32.desiredNodeID = 3;
  canopenNodeSTM32.baudrate = 500;
  canopen_app_init(&canopenNodeSTM32);

  /** Mode POLLING simple - PAS d'interruptions UART2 */
  // __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);  // DÉSACTIVÉ: polling pur

  printf("[LIDAR] Using POLLING mode (simple 1-buffer) for UART2 reception\r\n");
  printf("[LIDAR] LIDAR configured at %lu bauds\r\n", huart2.Init.BaudRate);
  printf("[LIDAR] Buffer size: %d bytes (single buffer)\r\n", LIDAR_RX_BUFFER_SIZE);
  printf("[LIDAR] Ready to receive LIDAR data\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Machine à états simple: 0 = FILLING, 1 = PROCESSING
  static int buffer_state = 0;
  static uint16_t buffer_index = 0;

  // Variables de diagnostic
  static uint32_t loop_count = 0;
  static uint32_t last_print_tick = 0;
  static uint32_t bytes_received = 0;
  static uint32_t errors_ore = 0;
  static uint32_t errors_fe = 0;
  static uint32_t errors_ne = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    loop_count++;

	canopen_cmd_fetch_robot_pos();

    // === DIAGNOSTIC: Affiche les flags UART2 toutes les secondes ===
    if (HAL_GetTick() - last_print_tick >= 1000)
    {
      printf("[DEBUG] RXNE=%d ORE=%d FE=%d NE=%d PE=%d TXE=%d TC=%d | Loops=%lu Bytes=%u State=%d Idx=%u\r\n",
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) ? 1 : 0,
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) ? 1 : 0,
        loop_count, bytes_received, buffer_state, buffer_index);

      printf("[ERRORS] ORE=%lu FE=%lu NE=%lu\r\n", errors_ore, errors_fe, errors_ne);

      last_print_tick = HAL_GetTick();
      loop_count = 0;  // Reset compteur de loops
    }

    // === CLEAR des flags d'erreur UART2 ===
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
    {
      __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
      errors_ore++;
    }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
    {
      __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
      errors_fe++;
    }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE))
    {
      __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);
      errors_ne++;
    }

    // === ÉTAT FILLING: Remplir le buffer via polling UART2 ===
    if (buffer_state == 0)
    {
      // Polling: Y a-t-il un byte disponible sur UART2 ?
      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
      {
        // Lire le byte (efface automatiquement le flag RXNE)
        uint8_t byte = (uint8_t)(huart2.Instance->RDR & 0xFFU);

        // Stocker dans le buffer unique
        lidar_rx_buffer_1[buffer_index] = byte;
        buffer_index++;
        bytes_received++;

        // Buffer plein (4000 bytes) ? → Passer en mode PROCESSING
        if (buffer_index >= LIDAR_RX_BUFFER_SIZE)
        {
          buffer_state = 1;  // PROCESSING
          buffer_index = 0;
        }
      }
    }
    // === ÉTAT PROCESSING: Traiter le buffer complet ===
    else
    {
      // Traiter les 4000 bytes avec AnalysisOne() et détection 360°
      lidar_process_buffer(lidar_rx_buffer_1);

      // Traitement terminé → Retour en mode FILLING
      buffer_state = 0;
      buffer_index = 0;
    }

    canopen_app_process();

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    printf("Error_Handler\r\n");
    HAL_Delay(1000);
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
