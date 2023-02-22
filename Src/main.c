/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "TLC5947.h"
#include "hall_sensor.h"
#include "stdlib.h" // for abs function

//#include "spi1.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

uint8_t g_uartByte, g_UARTVal,g_TLCFlag = 0;//val received, val stored, Timer flag
uint8_t g_imain=0,g_MotorSync = FALSE;//g_MotorSync is used to indicate that the motor is working in nominal speed
uint16_t g_degreeCount =0,g_prevTime = 0 ,g_curTime = 0, g_tDelay = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
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
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart3, &g_uartByte, 1);//this triggers only once,uses interrupt and it has to be re-enabled
//  FillArray(BLUE,FABITEST);//it will be red
  HAL_TIM_Base_Start_IT(&htim4);//Starts the TIM Base generation in interrupt mode. The oder mode just allows the count
//  HAL_TIM_Base_Start(&htim2);//Start the TIM just as a counter with no interrupts

  HAL_GPIO_WritePin(TLC5947_BLANK1_GPIO_Port, TLC5947_BLANK1_Pin, GPIO_PIN_SET);//Turn off leds
  HAL_GPIO_WritePin(TLC5947_BLANK2_GPIO_Port, TLC5947_BLANK2_Pin, GPIO_PIN_SET);//Turn off leds
  HAL_GPIO_WritePin(TLC5947_BLANK3_GPIO_Port, TLC5947_BLANK3_Pin, GPIO_PIN_SET);//Turn off leds
  HAL_GPIO_WritePin(TLC5947_BLANK4_GPIO_Port, TLC5947_BLANK4_Pin, GPIO_PIN_SET);//Turn off leds

  HAL_GPIO_WritePin(Board_Led_GPIO_Port, Board_Led_Pin, GPIO_PIN_RESET);//Turn off board led


//  HAL_GPIO_WritePin(Board_Led_GPIO_Port,Board_Led_Pin,GPIO_PIN_RESET);//Turn off led


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




	  if(g_TLCFlag){//enter when TIM4 interrupts
		  TLC_Update();//renew PWM

	  if(g_imain > 2 ) //only enables RGB
		  g_imain = 0;

//	  g_imain ++;

	 FillArray(g_imain,FABITEST);
//	  FillArray(BLUE,LINE);
//		  FillArray(BLUE,LETTERo);
//	  FillArray(BLUE,LETTERS);
//	  FillArray(BLUE,MATITEST);
//	  FillArray(BLUE,BMWLOGO);
		  g_TLCFlag = 0;//disable TLC Update
	  }

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 35999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_Led_GPIO_Port, Board_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TLC5947_BLANK4_Pin|TLC5947_BLANK3_Pin|TLC5947_BLANK2_Pin|TLC5947_BLANK1_Pin 
                          |TLC5947_XLAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Board_Led_Pin */
  GPIO_InitStruct.Pin = Board_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TLC5947_BLANK4_Pin TLC5947_BLANK3_Pin TLC5947_BLANK2_Pin TLC5947_BLANK1_Pin 
                           TLC5947_XLAT_Pin */
  GPIO_InitStruct.Pin = TLC5947_BLANK4_Pin|TLC5947_BLANK3_Pin|TLC5947_BLANK2_Pin|TLC5947_BLANK1_Pin 
                          |TLC5947_XLAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Hall_sensor_Pin */
  GPIO_InitStruct.Pin = Hall_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Hall_sensor_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART3) {
//	    // USART3
//	  HAL_UART_Transmit(&huart3, &g_uartByte, 1, 100);//echo the received msg to verify that all is good
//	  HAL_UART_Receive_IT(&huart3, &g_uartByte, 1);//re-enable the rx int
//	  HAL_UART_Transmit(&huart3, (uint8_t *)"BP\r\n", 4U, 100);//so as to know it comes from the BP
//	  HAL_UART_Receive_IT(&huart3, &g_uartByte, 1);//re-enable the rx int
//
//	  if(g_uartByte > 47 && g_uartByte <55)
//		  g_UARTVal=g_uartByte;//the values that are relevant are stored
//	}
//
//
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance== TIM4)
	{
		if(g_MotorSync<= 5 )//wait 5" before turning leds on
			g_MotorSync++;
		else
		{
			g_TLCFlag = 1;//enable TLC Update
			g_degreeCount++;

			if (g_degreeCount>=360)//reset val, used only for testing
				g_degreeCount = 0;
		}
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

 static uint16_t newPeriod = FALSE; //stores timertick, LoopCounter is used to count 'x' amount of loops to allow the motor to stabilize


  if(GPIO_Pin == Hall_sensor_Pin) // hall sensor's pin
  {
	  g_prevTime = g_curTime;
	  HAL_GPIO_TogglePin(Board_Led_GPIO_Port, Board_Led_Pin);//Switch on/off board pin

	  /*
	   * 1 way of acquiring period of each turn
	   */
	  g_curTime = HAL_GetTick(); //Provides a tick value in millisecond
	  g_tDelay = abs(g_curTime - g_prevTime);//abs guarantees that there is no issues related to the sign

	  /*
	   * time it takes to do 1° = 360/g_tDelay;
	   * The hall sensor is at 270° and for testing the leds
	   * i want a straight line at 90°, thus i have to wait (270-90)° = 180°
	   */
	  //for this to work i have to change the prescaler of timer 4 so that tim4 interrupts every 1ms
	  g_degreeCount = 0;//indicates that a full spin has been made and thus degree count is reset to 0°



	  	  if(g_MotorSync >= 5 && newPeriod == FALSE) //this will update TIM4's period only once
	  	  {
	  		  __HAL_TIM_SET_AUTORELOAD(&htim4,__HAL_TIM_GET_AUTORELOAD(&htim4)*180*360/g_tDelay);//1"*180°*time1°. For the time being this will only be done once
	  		newPeriod = TRUE;
	  	  }

	  if(g_imain > 2 ) //only enables RGB
		  g_imain = 0;

	  g_imain ++;

  }

}

void TLC_Write(uint8_t data[])
//void TLC_Write(uint8_t *data)
{
	HAL_SPI_Transmit(&hspi1,data, SPI_BYTE_AMOUNT,1000); // envio via el sp1 de 1 todos los bytes que tenga que mandar
//    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY); // espero a que termine la transferencia
}
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
