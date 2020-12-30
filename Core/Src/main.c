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
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t dmxData[DMX_LENGTH_ARRAY];
static uint8_t curByte = 0;
static uint8_t curBit = 0;

enum State {
    Idle,
    SendingBreak,
    SendingMark,
    SendingStartBit,
    SendingData,
    SendingStopBits
} dmxState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetTim10ToUs(uint32_t us) {
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = us - 1;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 119;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

//    htim10.Init.Prescaler = us - 1;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
        Error_Handler();
    }
}

void SendDataBit() {
    uint8_t shifted = dmxData[curByte] << curBit;
    uint8_t isHigh = shifted & 128U;
    if (isHigh)
        HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_RESET);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void) {
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_Base_Stop_IT(&htim10);
    switch (dmxState) {
        case SendingBreak:
            /* send mark after BREAK */
            dmxState = SendingMark;
            curByte = 0;
            HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_SET);
            SetTim10ToUs(DMX_LENGTH_MAB);
            break;
        case SendingMark:
            /* send start bit (0) */
            dmxState = SendingStartBit;
            HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_RESET);
            SetTim10ToUs(DMX_LENGTH_BIT);
            break;
        case SendingStartBit:
            /* send first bit */
            dmxState = SendingData;
            curBit = 0;
            SendDataBit();
            SetTim10ToUs(DMX_LENGTH_BIT);
            break;
        case SendingData:
            if (curBit < 8) {
                /* send next bit */
                curBit++;
                SendDataBit();
                SetTim10ToUs(DMX_LENGTH_BIT);
            } else {
                /* send two stop bits and mark between slots (4 + 4 + 20) */
                dmxState = SendingStopBits;
                HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_SET);
                SetTim10ToUs(DMX_LENGTH_BIT * 2 + DMX_LENGTH_MBS);
            }
            break;
        case SendingStopBits:
            curByte++;
            if (curByte < DMX_LENGTH_ARRAY) {
                /* send start bit (0) */
                dmxState = SendingStartBit;
                HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_RESET);
                SetTim10ToUs(DMX_LENGTH_BIT);
            } else {
                /* send mark before break */
                dmxState = Idle;
                SetTim10ToUs(DMX_LENGTH_MBB);
            }
            break;
    }
    HAL_TIM_Base_Start_IT(&htim10);

    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_IRQHandler(&htim10);
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

void DMX_Send_Packet() {
    /* send BREAK */
    dmxState = SendingBreak;
    HAL_GPIO_WritePin(DMX_BREAK_OUT_GPIO_Port, DMX_BREAK_OUT_Pin, GPIO_PIN_RESET);
    SetTim10ToUs(DMX_LENGTH_BREAK);
    HAL_TIM_Base_Start_IT(&htim10);
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
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
//    clrDmxData();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    uint8_t brig = 0;
    dmxData[0] = 255;
    dmxData[1] = 255;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (dmxState == Idle) {
            HAL_Delay(600);
//            if (!(brig > 245)) {
//                brig += 10;
//            }
//            dmxData[0] = brig;
            DMX_Send_Packet();
//    HAL_Delay(20);      //Mark time after Slot
        }
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//        HAL_UART_Transmit(&huart1, data2, sizeof(data2), HAL_MAX_DELAY);
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 119;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DMX_BREAK_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DMX_BREAK_OUT_Pin */
  GPIO_InitStruct.Pin = DMX_BREAK_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DMX_BREAK_OUT_GPIO_Port, &GPIO_InitStruct);

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
    while (1) {
    }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
