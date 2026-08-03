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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bq79600.h"
#include "bq79600_def.h"
#include "bq79616_def.h"
#include "SEGGER_RTT.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_hal_gpio.h"
#include <string.h>
#include <stdio.h>
//#include "Structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MSGQUEUE_OBJECTS 16
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
extern USBD_HandleTypeDef hUsbDeviceFS;
FDCAN_HandleTypeDef hfdcan1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
int Safety_Error;
int Battery_status; // 0 = standby, 1= charging , 2=discharging , 3 =  Error
int ballancing;
int data_in;
int current_raw;
int prev_button_state;
BQ_Data BqMeasurements[2];

typedef struct {
  char Buf[64];
  uint32_t Timestamp;
} Message;
Message msg;

//usb receive variables
uint8_t usbRxBuf[128];
uint16_t usbRxBufLen;
uint8_t usbRxFlag = 0 ;

// digital lowpass filter variable
float alpha = 0.05f;    // adjust as needed (0.05–0.2 is common)

int current_raw = 31000;   // your ADC reading each loop
float current_filtered = 31000;  // filtered output
float current = 0 ;
////

char message[128];
char buffer_usb1[] = "BMS TEST /n";
char buffer_usb2[] = "PORT COMM TEST /n";
char buffer_usb3[] = "USB FS TEST /n";
char bq_message[32] = {0};
char message_buffer[32];

volatile uint8_t CDC_TransmitReady = 1;


// VARIABLES FOR SAFETY TASK

int relay_status = 0 ;
int overvoltage = 0 ;
int undervoltage = 0 ;
int overtemperature = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void MX_UART4_Init(int boudrate);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */
float convert_adc_to_current(float data);
void BQ_COMM(bq79600_t *bms_instance);
void Usb_COMM();
//#define bms_fault(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (state) ? GPIO_PIN_RESET : GPIO_PIN_SET)
//#define bms_run() HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  static bq79600_t *instance = NULL;
  instance = open_bq79600_instance(0);
  if (instance == NULL) instance = open_bq79600_instance(0);
  instance->rx_len = size;
  bq79600_rx_callback(instance);
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, instance->rx_buf, sizeof(instance->rx_buf));
} */
void USB_RXCallback(uint8_t* Buf, uint32_t *Len)
{
	memcpy(usbRxBuf, Buf, *Len);
	usbRxBufLen = *Len;
	usbRxFlag = 1;
}
/*
int voltage_to_temperature(float voltage) // formula based of fitted logarytmic function fitted in curve fitting
                                          // toolbox in matlab
{
	if(voltage > 3500)
		return 0;
	else
		return -50.2*log(voltage)+ 416;
}
int voltage_to_temperature2(float voltage) // formula based of fitted logarytmic function fitted in curve fitting
                                          // toolbox in matlab this one if for 0603 ntc thermistor that has a different B value
{
	if(voltage > 3500)
		return 0;
	else
		return 154/(1 + exp(0.0012*(voltage-910)));
} */

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart4_tx;
DMA_HandleTypeDef hdma_usart4_rx;


#define bms_run() HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5) ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //SEGGER_RTT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init(1000000);
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start(&hadc1);
  Safety_Error = 0 ;
   bq79600_t *bms_instance = open_bq79600_instance(0);
   BQ_INNIT(bms_instance);
   prev_button_state = HAL_GPIO_ReadPin (GPIOE, GPIO_PIN_14);

   while(1)
   {
 	  BQ_COMM(bms_instance);
 	 if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
 	 {
 		        current_raw = HAL_ADC_GetValue(&hadc1);
 	 			HAL_ADC_Start(&hadc1);
 	 			current_filtered = current_filtered +
 	 			                       alpha * (current_raw - current_filtered);
 	 }
 	 current = convert_adc_to_current(current_filtered);
 	  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
 	  Usb_COMM();
 	  Safety();
 	  Led();
   }
    /* USER CODE END WHILE */




    /* USER CODE BEGIN 3 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
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
  hadc1.Init.Oversampling.Ratio = 1;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART4_Init(int boudrate)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = boudrate;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void BQ_INNIT(bq79600_t *bms_instance)
{
	//bq79600_t *bms_instance = open_bq79600_instance(0);

	bms_instance->mode = BQ_UART;
	bms_instance->state = BQ_SHUTDOWN;
	bms_instance->rx_port = GPIOA;
	bms_instance->tx_port = GPIOA;
	bms_instance->rx_pin = 1;
	bms_instance->tx_pin = 0;

	// wake up ping using slowed uart communication
	HAL_UART_DeInit(&huart4);
	MX_UART4_Init(3250);
	HAL_Delay(10);
	uint8_t zero=0x00;
	HAL_UART_Transmit(&huart4,&zero,1,100);
	HAL_Delay(13);
	HAL_UART_Transmit(&huart4,&zero,1,100);
	HAL_Delay(50);
	HAL_UART_DeInit(&huart4);
	MX_UART4_Init(1000000);
	 if( HAL_UARTEx_ReceiveToIdle_IT(&huart4, bms_instance->rx_buf, sizeof(bms_instance->rx_buf)) == HAL_ERROR)
	 {
		 while (1);
	 }
	  HAL_Delay(10);

	   if( HAL_UART_Transmit_IT(&huart4, bms_instance->tx_buf, bms_instance->tx_len) == HAL_BUSY)
	   {
		   while (1);
	   }
	initalize_communication(bms_instance,&huart4,N_DEVICES,N_CELLS_PER_DEVICE);
	uint8_t buf = 0xFF;
	bq79600_construct_command(bms_instance, STACK_WRITE, 0, FAULT_MSK1, 1 , NULL);

	//uint8_t buf = 0x3;
	// uint32_t start_cbcells = CB_CELL1_CTRL - N_CELLS_PER_DEVICE  + 1;


	HAL_Delay(15);
}
void BQ_COMM(bq79600_t *bms_instance)
{
	        uint8_t buf;



 	         bq79600_construct_command(bms_instance, STACK_READ, 0, DIETEMP1_HI, 2, NULL);
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         {
 	           modules[i].dietemp = raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.025;
 	         }


 	         uint32_t start_vcells = VCELL1_HI - N_CELLS_PER_DEVICE * 2 + 2;
 	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_vcells, N_CELLS_PER_DEVICE * 2, NULL);
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i <N_DEVICES - 1; i++)
 	         {
 	           for (int j = 0; j < N_CELLS_PER_DEVICE; j++)
 	           {
 	             modules[i].vcells[j] =
 	                 raw_to_float(&bms_instance->rx_buf[4 + i * (N_CELLS_PER_DEVICE * 2 + 6) + 2 * j]) * 0.19073;
 	           }
 	         } // bq2 temp 2 works fine



 	         uint32_t start_temp = GPIO1_HI  ;
 	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_temp, N_TEMPS_PER_DEVICE * 2, NULL);
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);

 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         {
 	           for (int j = 0; j < N_TEMPS_PER_DEVICE; j++)
 	           {
 	             modules[i].temperature[j] =voltage_to_temperature(
 	                 raw_to_float(&bms_instance->rx_buf[4 + i * (N_TEMPS_PER_DEVICE * 2 + 6) + 2 * j ] )  * 0.15259);
 	           }
 	         }




 	         uint32_t start_temp_ref = TSREF_HI ;
 	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_temp_ref, 2, NULL);
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	             modules[i].t_ref =
 	                 raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.16954;






 	         for (int i = 0; i < N_DEVICES - 1; i++) modules[i].timestamp = HAL_GetTick() ;


 	         bq79600_construct_command(bms_instance, STACK_READ, 0, BAL_STAT, 1, NULL); // BAL_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].BAL_STAT_RAW = bms_instance->rx_buf[4 + i * 7];

 	         bq79600_construct_command(bms_instance, STACK_READ, 0, CB_COMPLETE1, 1, NULL); // BAL_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].CB_COMPLETE1_RAW = bms_instance->rx_buf[4 + i * 7];

 	         bq79600_construct_command(bms_instance, STACK_READ, 0, CB_COMPLETE2, 1, NULL); // BAL_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].CB_COMPLETE2_RAW = bms_instance->rx_buf[4 + i * 7];



 	         bq79600_construct_command(bms_instance, STACK_READ, 0, DEV_STAT, 1, NULL); // DEV_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].DEV_STAT_RAW = bms_instance->rx_buf[4 + i * 7];



 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_UV1, 1, NULL); // DEV_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].UV_RAW_1 = bms_instance->rx_buf[4 + i * 7];



 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_UV2, 1, NULL); // DEV_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].UV_RAW_2 = bms_instance->rx_buf[4 + i * 7];


 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_OV1, 1, NULL); // DEV_STAT READ.
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].OV_RAW_1 = bms_instance->rx_buf[4 + i * 7];



 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_OV2, 1, NULL); //
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].OV_RAW_2 = bms_instance->rx_buf[4 + i * 7];

 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_UT, 1, NULL); //
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].UT_RAW = bms_instance->rx_buf[4 + i * 7];

 	         bq79600_construct_command(bms_instance, STACK_READ, 0,  FAULT_OT, 1, NULL); //
 	         bq79600_tx(bms_instance);
 	         bq79600_bsp_ready(bms_instance);
 	         for (int i = 0; i < N_DEVICES - 1; i++)
 	         modules[i].OT_RAW = bms_instance->rx_buf[4 + i * 7];

 	        	switch(data_in)
 	        	{
 	        	case 1:
	 	        	buf = 0x3; // fault detection off, autoballancing on, ballgo = 1
	 	            bq79600_construct_command(bms_instance, STACK_WRITE, 0, BAL_CTRL2, 1, &buf);
	 	        	bq79600_tx(bms_instance);
	 	        	data_in =0;
	 	        	break;
 	        	case 2:
	 	        	buf = 0x40;
	 	            bq79600_construct_command(bms_instance, STACK_WRITE, 0, BAL_CTRL2, 1, &buf);
	 	        	bq79600_tx(bms_instance);
	 	        	data_in =0;
	 	        	break;
 	        	case 3:
	 	        	buf = 0x3;
	 	            bq79600_construct_command(bms_instance, STACK_WRITE, 0, CONTROL1, 1, &buf); // send soft restart up the stack
	 	        	bq79600_tx(bms_instance);
	 	        	//BQ_INNIT(bms_instance);
	 	        	data_in =0;
 	        		break;
 	        	}




 	         // end of reading data from BQ79600
 	         for (int i = 0; i < N_DEVICES - 1; i++) // send data from bq to different tasks.
 	         {
             if(modules[i].dietemp > 150 || modules[i].dietemp < 5) // redundancy, if the comm with bq failes, initalize it again!
             {
            	 initalize_communication(bms_instance,&huart4,N_DEVICES,N_CELLS_PER_DEVICE);
            	 break;
             }
             BqMeasurements[i].BQ_Number = i ;
 		     for (int j = 0; j < N_CELLS_PER_DEVICE; j++)
 		     {
 		     BqMeasurements[i].Bq_Voltages[j] = modules[i].vcells[j];

 		    BqMeasurements[i].Bq_Temperatures[j] = modules[i].temperature[j];
 		     }
 		    BqMeasurements[i].dietemp =  modules[i].dietemp;
 		    BqMeasurements[i].Bq_Timestamp = modules[i].timestamp;
 		    BqMeasurements[i].T_ref = modules[i].t_ref;

 		   //  uint8_t dev_stat =   bms_instance->rx_buf[4 + i * 7];
 		     BqMeasurements[i].Device_Stat.MAIN_ADC_RUN = (modules[i].DEV_STAT_RAW >> 0) & 0x01;
 		     BqMeasurements[i].Device_Stat.AUX_ADC_RUN = (modules[i].DEV_STAT_RAW >> 1) & 0x01;
 		     BqMeasurements[i].Device_Stat.CS_RUN = (modules[i].DEV_STAT_RAW >> 2) & 0x01;
 		     BqMeasurements[i].Device_Stat.OVUV_RUN = (modules[i].DEV_STAT_RAW >> 3) & 0x01;
 		     BqMeasurements[i].Device_Stat.OTUT_RUN = (modules[i].DEV_STAT_RAW >> 4) & 0x01;

 		     BqMeasurements[i].CB_DONE = (modules[i].BAL_STAT_RAW >> 0) & 0x01;
 		     BqMeasurements[i].MB_DONE = (modules[i].BAL_STAT_RAW >> 1) & 0x01;
 		     BqMeasurements[i].ABORTFLT = (modules[i].BAL_STAT_RAW >> 2) & 0x01;
 		     BqMeasurements[i].CB_RUN = (modules[i].BAL_STAT_RAW >> 3) & 0x01;
 		     BqMeasurements[i].MB_RUN = (modules[i].BAL_STAT_RAW >> 4) & 0x01;
 		     BqMeasurements[i].CB_INPAUSE = (modules[i].BAL_STAT_RAW >> 5) & 0x01;
 		     BqMeasurements[i].OT_PAUSE_DET = (modules[i].BAL_STAT_RAW >> 6) & 0x01;
 		     BqMeasurements[i].INVALID_CBCONF = (modules[i].BAL_STAT_RAW >> 7) & 0x01;

 		    for(int x = 0 ; x < 8  ; x++)
 		    {
 		    	BqMeasurements[i].OT_ERROR[x] = (modules[i].OT_RAW >> x  ) & 0x01;
 		    	BqMeasurements[i].UT_ERROR[x] = (modules[i].UT_RAW >> x  ) & 0x01;
 		    }
 		     for(int x = 0 ; x < N_CELLS_PER_DEVICE - 8  ; x++)
 		     {
 		    	BqMeasurements[i].UV_ERROR[x+8] = (modules[i].UV_RAW_2 >> x  ) & 0x01;
 		    	BqMeasurements[i].OV_ERROR[x+8] = (modules[i].OV_RAW_2 >> x  ) & 0x01;
 		    	BqMeasurements[i].CB_Done[x+8] = (modules[i].CB_COMPLETE1_RAW >> x  ) & 0x01;

 		     }
 		     for(int x = 0 ; x < 8 ; x++)
 		     {
 		    	BqMeasurements[i].UV_ERROR[x] = (modules[i].UV_RAW_1 >> (x) ) & 0x01;
 		    	BqMeasurements[i].OV_ERROR[x] = (modules[i].OV_RAW_1 >> (x) ) & 0x01;
 		    	BqMeasurements[i].CB_Done[x] = (modules[i].CB_COMPLETE2_RAW >> (x) ) & 0x01;
 		     }
 	         }


     }


void Usb_COMM()
{
	  char message[64]={0};


	  for(int x =0; x < N_DEVICES - 1 ; x++)
	  {   //   (Messages_QueueHandle
		  if(Battery_status == 3) // error
		  sprintf(message  , "Battery error, relay open. \n"  );
		  else if(Battery_status == 2 )
		  sprintf(message  , "Battery discharge mode, relay closed! \n"  );
		  else if(Battery_status == 1 )
		  sprintf(message  , "Battery charge mode, relay closed! \n"  );
		  else if (Battery_status == 0 )
		  sprintf(message  , "Battery on standby, relay open. \n"  );
		  Send_USB_Message(message, 5); // Send with a 5ms timeout


		    	   	   	   for(int i = 0 ; i< N_CELLS_PER_DEVICE; i++ )
		    	                 {

		    	                 sprintf(message  , "BQ Number:%d bq voltage value:%d [mV]  " ,BqMeasurements[x].BQ_Number+1 , (int)BqMeasurements[x].Bq_Voltages[i] );
		    	                // CDC_Transmit_FS((uint8_t*)message, strlen(message));
		    	                 Send_USB_Message(message, 5); // Send with a 5ms timeout
		    	                 if(BqMeasurements[x].UV_ERROR[i] && BqMeasurements[x].OV_ERROR[i])
		    	                 {
		    	                	 sprintf(message  , " - OV/UV SETPOINT ERROR! \n");
		    	                 }
		    	                 if(BqMeasurements[x].OV_ERROR[i] )
		    	                 {
		    	                	 sprintf(message  , " - OVERVOLTAGE ON THIS CELL! \n");
		    	                 }
		    	                 else if(BqMeasurements[x].UV_ERROR[i])
		    	                 {
		    	                	 sprintf(message  , " - UNDERVOLTAGE ON THIS CELL! \n");
		    	                 }
		    	                 else
		    	                 {
	    	                	 sprintf(message  , "\n");
		    	                 }
		    	                 Send_USB_Message(message, 2); // Send with a 5ms timeout


		    	                 }




		    	   	   	for(int i = 0 ; i< N_TEMPS_PER_DEVICE; i++ ) {
		    	   	   	               	   	   	   	  sprintf(message  , "BQ Number:%d bq temperature value: %d [deg C] " ,BqMeasurements[x].BQ_Number+1 , (int)BqMeasurements[x].Bq_Temperatures[i]);
		    	   	   	               	   	          Send_USB_Message(message, 5); // Send with a 5ms timeout

		    	   	   		    	    	                 if(BqMeasurements[x].UT_ERROR[i] && BqMeasurements[x].OV_ERROR[i])
		    	   	   		    	    	                 {
		    	   	   		    	    	                	 sprintf(message  , " - OT/UT SETPOINT ERROR! \n");
		    	   	   		    	    	                 }
		    	   	   		    	    	                 if(BqMeasurements[x].OT_ERROR[i])
		    	   	   		    	    	                 {
		    	   	   		    	    	                	 sprintf(message  , " - OVERTEMPERATURE! \n");
		    	   	   		    	    	                 }
		    	   	   		    	    	                 else if(BqMeasurements[x].UT_ERROR[i])
		    	   	   		    	    	                 {
		    	   	   		    	    	                	 sprintf(message  , " - UNDERTEMPERATURE! \n");
		    	   	   		    	    	                 }
		    	   	   		    	    	                 else
		    	   	   		    	    	                 {
		    	   	   		        	                	 sprintf(message  , "\n");
		    	   	   		    	    	                 }
		    	   	   		    	    	           Send_USB_Message(message, 5); // Send with a 5ms timeout

		    	   	   		    	   	   	   } // END OF TEMPERATURE PRINT

	               	   	   	   	  sprintf(message  , "BQ Number:%d bq REFERENCE temperature value: %d [mV] \n" ,BqMeasurements[x].BQ_Number+1 , (int)BqMeasurements[x].T_ref);
	               	   	        	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	   	   	   	   	if(BqMeasurements[x].BQ_Overvoltage_Error)
		    	   	   	   	   	{
		    	   	   	   	   		sprintf(message  , "BQ OVERVOLTAGE ERROR! \n" );
		    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	   	   	   	   	}
			    	   	   	   	   	if(BqMeasurements[x].BQ_Undervoltage_Error)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "BQ UNDERVOLTAGE ERROR! \n" );
			    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	if(BqMeasurements[x].BQ_Communication_Error)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "BQ COMM ERROR! \n" );
			    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }

		    	   	   	   	   		sprintf(message  , "DEVICE STATUS READOUT: \n" );
		    	   	   	        	Send_USB_Message(message, 2); // Send with a 5ms timeout

			    	   	   	   	   	if(BqMeasurements[x].Device_Stat.MAIN_ADC_RUN)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "MAIN ADC IS RUNNING. \n" );
			    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	else
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "MAIN ADC IS TURNED OFF. \n" );
			    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }

			    	   	   	   	   	if(BqMeasurements[x].Device_Stat.AUX_ADC_RUN)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "AUXILIARY ADC IS RUNNING. \n" );
			    	   	   	            Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	else
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "AUXILIARY ADC IS TURNED OFF. \n" );
			    	   	   	   	        Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }



			    	   	   	   	   	if(BqMeasurements[x].Device_Stat.CS_RUN)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "CS IS RUNNING. \n" );
			    	   	   	   	        Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	else
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "CS IS TURNED OFF. \n" );
			    	   	   	   	        Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }



			    	   	   	   	   	if(BqMeasurements[x].Device_Stat.OVUV_RUN)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "OVERVOLTAGE/UNDERVOLTAGE PROTECTION CURRENTLY ACTIVE \n" );
			    	   	   	   	        Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	else
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "OVERVOLTAGE/UNDERVOLTAGE PROTECTION CURRENTLY DISABLED \n" );
			    	   	   	   	        Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }


			    	   	   	   	   	if(BqMeasurements[x].Device_Stat.OTUT_RUN)
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "OVERTEMPERATUE/UNDERTEMPERATURE PROTECTION CURRENTLY ACTIVE \n" );
			    	   	   	         	Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }
			    	   	   	   	   	else
			    	   	   	   	   	{
			    	   	   	   	   		sprintf(message  , "OVERTEMPERATUE/UNDERTEMPERATURE PROTECTION CURRENTLY DISABLED \n" );
			    	   	   	   	       Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	        	    	   }

			    	   	   	          sprintf(message  , "BALLANCING STATS: \n" );
			    	   	             Send_USB_Message(message, 2); // Send with a 5ms timeout

			    	   	   			if(BqMeasurements[x].CB_DONE)
			    	   	   			  	{
			    	   	   			    sprintf(message  , "CELL BALLANCING DONE \n" );
			    	   	   			  Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}

			    	   	   			if(BqMeasurements[x].MB_DONE)
			    	   	   			  	{
			    	   	   			    sprintf(message  , "MODULE BALLANCING DONE \n" );
			    	   	   			    Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}

			    	   	   			if(BqMeasurements[x].ABORTFLT)
			    	   	   			  	{
			    	   	   			    sprintf(message  , "BALLANCING ABORTED, ABORTFLT=1 \n" );
			    	   	   			    Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}
			    	   	   			if(BqMeasurements[x].CB_RUN)
			    	   	   			  	{
			    	   	   			    sprintf(message  , "BALLANCING IS RUNNING \n" );
			    	   	   			    Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}
			    	   	   			if(BqMeasurements[x].MB_RUN)
			    	   	   			  	{
			    	   	   			    sprintf(message  , "MODULE BALLANCING IS RUNNING \n" );
			    	   	   			  Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}
			    	   	   			if(BqMeasurements[x].CB_INPAUSE)
			    	   	   			  	{
			    	   	   			    sprintf(message  , " BALLANCING IS PAUSED \n" );
			    	   	   			   Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}
			    	   	   			if(BqMeasurements[x].OT_PAUSE_DET)
			    	   	   			  	{
			    	   	   			    sprintf(message  , " BALLANCING IS PAUSED DUE TO OVERTEMPERATURE \n" );
			    	   	   			  Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   		    	}
			    	   	   			if(BqMeasurements[x].INVALID_CBCONF)
			    	   	   			  	{
			    	   	   			    sprintf(message  , " INVALID BALLANCE CONFIG  \n" );
			    	   	   			 Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   			  	}
			    	   	   		for(int i = 0 ; i< N_CELLS_PER_DEVICE; i++ )
			    	   	   			    {


			    	   	   			   if(BqMeasurements[x].CB_Done[i])
			    	   	   			     {
			    	   	   			    sprintf(message  , "Ballancing done on cell: %d \n",i+1);
			    	   	   			Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   			    }

			    	   	   			    }

			    	   	   	   	   	   sprintf(message  , "Temperature of BQ: %d  [deg C]\n" , (int)BqMeasurements[x].dietemp );
			    	   	   	   	  Send_USB_Message(message, 2); // Send with a 5ms timeout
			    	   	   	   	   	   sprintf(message  , "Timestamp: %u \n" , BqMeasurements[x].Bq_Timestamp );
			    	   	   	   	  Send_USB_Message(message, 2); // Send with a 5ms timeout

			    	   	   	   	  	  printf(message  , "Current:" , (int)current );
			    	   	   	   	  Send_USB_Message(message, 2); // Send with a 5ms timeout
		    	                 //CDC_Transmit_FS((uint8_t*)message2, strlen(message2));
			    	   	   	     HAL_Delay(5);
		    	                 char message[6] = " \n";
		    	                 for(int i = 0 ; i< 3; i++ )
		    	                 Send_USB_Message(message, 2); // Send with a 5ms timeout



	  }



		  if(usbRxFlag && usbRxBufLen)
		  {

			  //int data_out =0;
			  if (strcmp((char*)usbRxBuf, "Ballance start\n") == 0)
			  {
				  data_in =1;
				 // osMessageQueuePut(BQ79614Incoming_QueueHandle, &data_out, 5, 5) ;
			      // handle start
			  }
			  else if (strcmp((char*)usbRxBuf, "Ballance end\n") == 0)
			  {
				  data_in =2;
				 // osMessageQueuePut(BQ79614Incoming_QueueHandle, &data_out, 5, 5) ;
			      // handle stop
			  }
			  else if (strcmp((char*)usbRxBuf, "Restart\n") == 0)
			  {
				  data_in =3;
			   Safety_Error = 0;
				Battery_status = 0;
				  //NVIC_SystemReset(); // testing
				//  HAL_Delay(100);
				//  memset(usbRxBuf,0,sizeof(usbRxBuf));
				//  HAL_Delay(100);
				 // HAL_NVIC_SystemReset();
			  }
			  else if (strcmp((char*)usbRxBuf, "Start charging\n") == 0)
				  data_in =4;
			  else if (strcmp((char*)usbRxBuf, "Go to standby\n") == 0)
				  data_in =5;
			  else if (strcmp((char*)usbRxBuf, "Start discharging\n") == 0)
				  data_in = 6;
			  memset(usbRxBuf,0, sizeof(usbRxBuf));
			  usbRxFlag = 0 ;
			  usbRxBufLen  = 0 ;
		  }






}
float convert_adc_to_current(float data)
{
	return data*0.0213 -694.3999 ;
}
  void Safety()
  {
	  // Battery status = 0 - standby , 1- charging, 2 - discharging , 3 - error



     for (int bq_num = 0 ; bq_num < N_DEVICES - 2 ; bq_num++ )
     {
     for(int iterator = 0 ; iterator < N_CELLS_PER_DEVICE ; iterator ++)
     {
       if(BqMeasurements[bq_num].OV_ERROR[iterator] == 1 || BqMeasurements[bq_num].OV_ERROR[iterator] == 1 )
       {
    	   overvoltage = 1;
       }
       if(BqMeasurements[bq_num].UV_ERROR[iterator] == 1 || BqMeasurements[bq_num].UV_ERROR[iterator] == 1 )
       {
    	   undervoltage = 1;
       }
       if(BqMeasurements[bq_num].OT_ERROR[iterator] == 1)
       {
    	   overtemperature = 1;
       }

     }

     ////////////////////////////////////////

     }
	 if( overvoltage != 0    || overtemperature != 0  ) // if the status is charging undervoltage is not applicable
	 {
		 Safety_Error = 1;
		 Battery_status = 3; // enter error staus
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // open safety relay

	 }

		if(Battery_status != 3 && Battery_status != 4 && data_in == 4 ) // if not in error and command sent to charge
		{
			Battery_status = 1 ;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // close safety relay
			data_in = 999; // block the variable so it doesnt repat
			HAL_Delay(400);

		}


		if(data_in == 5 && Battery_status!=3 ) // turn charging off, relay to open
		{
			Battery_status = 0 ;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			data_in = 999; // block the variable so it doesnt repat
			HAL_Delay(200);
		}


		if(( data_in == 6  || (!HAL_GPIO_ReadPin (GPIOE, GPIO_PIN_14)) && prev_button_state == 1)&& (Battery_status != 3  && undervoltage == 0 )  ) // enable discharge
		{
			Battery_status = 2 ;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // close safety relay
			data_in = 999; // block the variable so it doesnt repat
			HAL_Delay(400);
			relay_status = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10);
		}
		if( Battery_status == 2 && HAL_GPIO_ReadPin (GPIOE, GPIO_PIN_14) )
		{
			Battery_status = 0 ;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_Delay(500);
		}



        //xd = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10); // read state of auxilary relay contacts
		relay_status = HAL_GPIO_ReadPin (GPIOE, GPIO_PIN_14);
		relay_status = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_10);
        if ((Battery_status == 1 || Battery_status == 2) && relay_status  == 0 ) // this means that relay auxilary contacts are not present
        {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // if not open it and go to error
			Safety_Error = 1;
			Battery_status = 3; // enter error staus
        }
        prev_button_state = HAL_GPIO_ReadPin (GPIOE, GPIO_PIN_14);

  }
  void Led()
  {
  	  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_2);
  	  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_3);
	  if( (Battery_status == 3 || Safety_Error == 1)  )
	  {
		  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_5);
		  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_4);
		  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_3);
		  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_2);
		  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
	  }
	  else if(Battery_status == 1 || Battery_status == 2 )
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  	  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_5);
	  	  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_4);

	  }

	  else
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 , GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_TogglePin (GPIOE, GPIO_PIN_2);
	  }


  }




  void Send_USB_Message(const char* message, uint32_t timeout_ms)
  {
      extern USBD_HandleTypeDef hUsbDeviceFS; // Assumes this is your handle

      // 1. Check if USB is configured
      if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
      {
          return; // Not connected, do nothing
      }

      // 2. Try to send with a timeout
      uint32_t start_time = HAL_GetTick();
      while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY)
      {
          if (HAL_GetTick() - start_time > timeout_ms)
          {
              return; // Timeout, give up
          }
          HAL_Delay(1); // Wait for USB stack
      }
  }



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
