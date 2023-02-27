/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../ECUAL/STEPPER/STEPPER.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPPER_MOTOR1 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint16_t tim_val_ms; 			// tick is 0.1ms
//  uint16_t tim_begin_ms;
//  uint16_t tim_end_ms;
//	uint16_t tim_v_on;
//	uint16_t tim_v_off;

  uint16_t sensor_data[100000];

  uint32_t i = 0;
  uint32_t j;

  char uart_buf[50]; 				// 50 char buffer to store our message
  int uart_buf_len;
  uint16_t raw; 					//store our 16bit ADC reading

  uint16_t edge;
  uint16_t pulses;
  uint32_t htime;
  uint16_t t_rl;
  uint32_t begin_vibr;

  bool bad;

  void measure(void);
  void analyse(void);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_ADC1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* __SAMPLE ROUTIN BEGIN__ */

//  HAL_TIM_Base_Start(&htim16); 								// start tim16
////  tim_begin_ms = __HAL_TIM_GET_COUNTER(&htim16); 			// get first number of tim16
//  while (tim_val_ms < 10000-1) 								// with presc of 9600 it is exactly 1 sec
//  {
//	HAL_GPIO_WritePin(GPIOE , GPIO_PIN_1, GPIO_PIN_SET);
//	tim_val_ms = __HAL_TIM_GET_COUNTER(&htim16);
//
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	raw = HAL_ADC_GetValue(&hadc1);
//	sensor_data[i] = raw;
//	i++;
//  }
//  HAL_GPIO_WritePin(GPIOE , GPIO_PIN_1, GPIO_PIN_RESET);
////  tim_end_ms = __HAL_TIM_GET_COUNTER(&htim16);
////  uart_buf_len = sprintf(uart_buf, "%u ms Ende %u ms i =  %d \r\n",  tim_begin_ms, tim_end_ms, i );
//  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);
//  /* Transmit an array with data via uart */
//  for (j=0; j < i; j++){
//	  uart_buf_len = sprintf(uart_buf, "%d, %u \r\n", j, sensor_data[j]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);
//  }

  /* __SAMPLE ROUTIN END__ */


  /* ACTOREN/SENSOREN CHECK BEGIN */

  /* __STEPPER CONTROL BLOCK BEGIN__ */
  /* see also STEPPER_cfg.h */
//  STEPPERS_Init_TMR(&htim17);								// nötig für init look also ISR CODE BEGIN 4
//  STEPPER_SetSpeed(STEPPER_MOTOR1, 12); 					// set RPM
//  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 1048, DIR_CCW); 	// drive motor, steps, direction(immer verkehrt k.a. warum)
  /* __STEPPER CONTROL BLOCK END__ */

  /* __VIBRATIONSMOTOR PA4__ PG12 */
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
//  HAL_Delay(2000);
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /* __PUMPE PA1__ PA5 */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//  HAL_Delay(2000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//  HAL_Delay(2000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*__TEST_LED___*/
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
  /* ACTOREN/SENSOREN CHECK END */

  /* test area */
//  don't forget to init timer16 for counting 1 sec and timer17 with ISR for Stepper

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 		// Pumpe
//  HAL_Delay(1500);

//start
  STEPPERS_Init_TMR(&htim17);								// nötig für init look also ISR CODE BEGIN 4
  STEPPER_SetSpeed(STEPPER_MOTOR1, 12); 					// set RPM

  measure();
  /* Transmit an array with data via uart */
//  for (j=0; j < i; j++){
//	  uart_buf_len = sprintf(uart_buf, "%lu, %u \r\n", j, sensor_data[j]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);
//  }

  analyse();

//  uart_buf_len = sprintf(uart_buf, "Digital \n");
//  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);

//  for (j=0; j < i; j++){
//	  uart_buf_len = sprintf(uart_buf, "%lu, %u \r\n", j, sensor_data[j]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);
//  }

  uart_buf_len = sprintf(uart_buf, "htime = %lu pulses = %u bad state = %d vibr = %lu  \r\n", htime, pulses, bad, begin_vibr);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);

  if (bad == false && pulses >= 5){
	  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 1048, DIR_CW); 	// rotate 180
	  HAL_Delay(2000);
	  measure();
	  analyse();

	  uart_buf_len = sprintf(uart_buf, "htime2 = %lu pulses2 = %u bad state = %d vibr = %lu  \r\n", htime, pulses, bad, begin_vibr);
	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len , 100);

	  if (bad == false && pulses >= 5){
		  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 262, DIR_CCW);		//rotate -45 (262CCW) && pumpe aus
		  HAL_Delay(2000);
		  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 786, DIR_CCW);		//rotate to 0 -> 1048 -262
	  }
	  else{
		  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 262, DIR_CW);		//rotate +45 && pumpe aus
		  HAL_Delay(2000);
		  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 786, DIR_CW);		//rotate to 360 -> 180 + 45 = 225 need +135
	  }
  }
  else{
	  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 1048, DIR_CW); 	// rotate 180
	  HAL_Delay(2000);
	  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 262, DIR_CW);		//rotate +45 && pumpe aus
	  HAL_Delay(2000);
	  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 786, DIR_CW); 	// rotate 180
  }



//  HAL_Delay(1500);
//  STEPPERS_Init_TMR(&htim17);								// nötig für init
//  STEPPER_SetSpeed(STEPPER_MOTOR1, 11); 					// set RPM
//  STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 1024, DIR_CW); 	// drive motor, steps, direction(immer verkehrt k.a. warum)

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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 9600-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 20000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin PB6 */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE13 PE14 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG12 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//calling an ISR for STEPPER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
 {
 	STEPPER_TMR_OVF_ISR(htim);
 }

void measure(void){
  HAL_TIM_Base_Start(&htim16); 								// start tim16
//  HAL_TIM_RE
  i = 0;
  while (tim_val_ms < 10000-1) 								// with presc of 9600 it is exactly 1 sec
  {
	HAL_GPIO_WritePin(GPIOG , GPIO_PIN_9, GPIO_PIN_SET); 	// LED yellow
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET); // vibrator
	tim_val_ms = __HAL_TIM_GET_COUNTER(&htim16);

	if (tim_val_ms == 2000-1){
		begin_vibr = i;										// saving time of begin vibration
	}
	if (tim_val_ms >= 2000-1 && tim_val_ms <= 3000-1){
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET); // vibrator on
//		TODO check if it is working without next 2 lines -> plot oszi and mcu compare
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		raw = HAL_ADC_GetValue(&hadc1);
		sensor_data[i] = raw;
		i++;
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);
	sensor_data[i] = raw;
	i++;
  }
	HAL_GPIO_WritePin(GPIOG , GPIO_PIN_9, GPIO_PIN_RESET); 	// LED yellow
}
//
void analyse(void){
	// digitalize signal v:[0,1]
	for (j = 0; j <= i; j++) {
		if (sensor_data[j] < 39718){  		//aprox. 2V
			sensor_data[j] = 0;
		}
		else{
			sensor_data[j] = 1;
		}
	}
	// count pulses
	for (j = 0; j <= i; j++){
		edge = sensor_data[j+1] - sensor_data[j];
		if (edge == 1){ 					// count only negative pulses
			pulses++;
		}
		if (sensor_data[j] == 1){
			htime++;
		}
	}
	// check if sensor been open before vibration
	bad = false;
	for (j = 0; j <= begin_vibr; j++){
		if (sensor_data[j] == 0){
			bad = true;
			break;
		}
//		else{
//			bad = false;
//		}
	}
	// check if sensor been open after vibration + Einschwingungszeit
	for (j = begin_vibr + 36500; j<=i; j++){
		if (sensor_data[j] == 0 ){
			bad = true;
			break;
		}
//		else{
//			bad = false;
//		}
	}
	//find on ratio
//	t_rl = htime * 1/i;
//	on_ratio = t_rl/1000;
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
  __disable_irq();
  while (1)
  {
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
