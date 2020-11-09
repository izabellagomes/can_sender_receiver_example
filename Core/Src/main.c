/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
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
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t txMailbox;
CAN_FilterTypeDef sFilterConfig;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  pHeader.DLC = 1;
  pHeader.IDE = CAN_ID_EXT;
//  pHeader.IDE = CAN_ID_STD;
  pHeader.RTR = CAN_RTR_DATA;
//  pHeader.StdId = 0x244;
  pHeader.ExtId = 0x1FFFFF00;

  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;

  // Mask extended
  uint32_t filter_id = 0x000;
  uint32_t filter_mask = 0x000;

//  sFilterConfig.FilterIdHigh = 0;//((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
//  sFilterConfig.FilterIdLow = 0;//(filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
//  sFilterConfig.FilterMaskIdHigh = 0;//((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
//  sFilterConfig.FilterMaskIdLow = 0;//(filter_mask >> (11 - 3)) & 0xFFF8;

  // Receive all messages.
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdHigh = filter_id; //<< 5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = filter_mask; // << 5;
  sFilterConfig.FilterMaskIdLow = 0;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

//  // Filter 0x100.0x1FF Standard
//  sFilterConfig.FilterBank = 0;
//  sFilterConfig.FilterIdHigh = 0x100 << 5;
//  sFilterConfig.FilterIdLow = 0;
//  sFilterConfig.FilterMaskIdHigh = 0x77 << 5;
//  sFilterConfig.FilterMaskIdLow = 0;
//  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

//	// Filter list for Standard Ids: 0x245, 0x246, 0x247.
//  sFilterConfig.FilterBank = 0;
//  sFilterConfig.FilterIdHigh = 0x245<<5;
//  sFilterConfig.FilterIdLow = 0;
//  sFilterConfig.FilterMaskIdHigh = 0;
//  sFilterConfig.FilterMaskIdLow = 0;
//  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
//
//  sFilterConfig.FilterBank = 1;
//  sFilterConfig.FilterIdHigh = 0x246<<5;
//  sFilterConfig.FilterIdLow = 0;
//  sFilterConfig.FilterMaskIdHigh = 0;
//  sFilterConfig.FilterMaskIdLow = 0;
//  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
//
//  sFilterConfig.FilterBank = 2;
//  sFilterConfig.FilterIdHigh = 0x247<<5;
//  sFilterConfig.FilterIdLow = 0;
//  sFilterConfig.FilterMaskIdHigh = 0;
//  sFilterConfig.FilterMaskIdLow = 0;
//  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
