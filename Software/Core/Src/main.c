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
#include "cmsis_os.h"
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
#include "queue.h"
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

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* Definitions for Default_task */
osThreadId_t Default_taskHandle;
const osThreadAttr_t Default_task_attributes = {
  .name = "Default_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BQ_comm_task */
osThreadId_t BQ_comm_taskHandle;
const osThreadAttr_t BQ_comm_task_attributes = {
  .name = "BQ_comm_task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Safety_task */
osThreadId_t Safety_taskHandle;
const osThreadAttr_t Safety_task_attributes = {
  .name = "Safety_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Led_task */
osThreadId_t Led_taskHandle;
const osThreadAttr_t Led_task_attributes = {
  .name = "Led_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Can_task */
osThreadId_t Can_taskHandle;
const osThreadAttr_t Can_task_attributes = {
  .name = "Can_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Usb_task */
osThreadId_t Usb_taskHandle;
const osThreadAttr_t Usb_task_attributes = {
  .name = "Usb_task",
  .stack_size = 3086 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BQ79614_Queue */
osMessageQueueId_t BQ79614_QueueHandle;
const osMessageQueueAttr_t BQ79614_Queue_attributes = {
  .name = "BQ79614_Queue"
};
/* Definitions for BQ79600_Queue */
osMessageQueueId_t BQ79600_QueueHandle;
const osMessageQueueAttr_t BQ79600_Queue_attributes = {
  .name = "BQ79600_Queue"
};
/* Definitions for Messages_Queue */
osMessageQueueId_t Messages_QueueHandle;
const osMessageQueueAttr_t Messages_Queue_attributes = {
  .name = "Messages_Queue"
};
/* Definitions for BQ_Safety_Status_Queue */
osMessageQueueId_t BQ_Safety_Status_QueueHandle;
const osMessageQueueAttr_t BQ_Safety_Status_Queue_attributes = {
  .name = "BQ_Safety_Status_Queue"
};
/* Definitions for Other_Safety_Status_Queue */
osMessageQueueId_t Other_Safety_Status_QueueHandle;
const osMessageQueueAttr_t Other_Safety_Status_Queue_attributes = {
  .name = "Other_Safety_Status_Queue"
};
/* USER CODE BEGIN PV */
#define n_devices 3
#define n_cells_per_device 13
#define n_temp_pre_device 8

typedef struct {
  float temperature[n_temp_pre_device];  // degC
  float vcells[n_cells_per_device];   // mV
  float t_ref;
  float dietemp;                         // degC
  int timestamp;
} module_t;
module_t modules[n_devices - 1] = {0};
typedef struct {
int BQ_Number ;
int BQ_Overvoltage_Error  ;
int BQ_Undervoltage_Error  ;
int BQ_Autoadressing_Error ;
int BQ_Communication_Error ;
int Bq_Timestamp ;
float Bq_Voltages[n_cells_per_device] ;   // mV
float Bq_Temperatures[n_temp_pre_device]; // degC
float dietemp;                            // degC
} BQ_Data;
BQ_Data Data_Receaved = {0} ;

int ballancing;


typedef struct {                                // object data type
  char Buf[64];
  uint32_t Timestamp;
} Message;
Message msg;

//usb receive variables
uint8_t usbRxBuf[128];
uint16_t usbRxBufLen;
uint8_t usbRxFlag = 0 ;


char message[128];
char buffer_usb1[] = "BMS TEST /n";
char buffer_usb2[] = "PORT COMM TEST /n";
char buffer_usb3[] = "USB FS TEST /n";
char bq_message[32] = {0};
char message_buffer[32];

volatile uint8_t CDC_TransmitReady = 1;

typedef struct {
  float R_top ;  // R100 (ohms)  pull-up from TSREF to node
  float R_bias ;  // R101 (ohms)  bias to GND (parallel with NTC)
  float R0 ;     // 10k at 25°C
  float B ;       // e.g., 3380
} ntc_cfg_t;

float gpio_ratio_to_celsius(float gpio_meas, float tsref_meas, ntc_cfg_t cfg)
{
    // 1) ratiometric ratio g = Vgpio / Vtsref
    float g = gpio_meas / tsref_meas;
    if (g <= 0.0f) return -273.15f;        // out-of-range guard
    if (g >= 0.9999f) g = 0.9999f;         // avoid div-by-zero

    // 2) Equivalent bottom resistance from ratio
    float Req = (g * cfg.R_top) / (1.0f - g);

    // 3) Solve NTC resistance from Req = (Rntc || R_bias)
    float denom = cfg.R_bias - Req;
    if (denom <= 1e-6f) denom = 1e-6f;     // guard
    float Rntc = (Req * cfg.R_bias) / denom;

    // 4) Beta model -> temperature
    const float T0 = 298.15f;              // 25°C in Kelvin
    float invT = (1.0f / T0) + (1.0f / cfg.B) * logf(Rntc / cfg.R0);
    float T_K = 1.0f / invT;
    return T_K - 273.15f;
}




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
void Default(void *argument);
void Bq_comm(void *argument);
void Safety(void *argument);
void Led(void *argument);
void StartTask05(void *argument);
void Usb(void *argument);

/* USER CODE BEGIN PFP */



//#define bms_fault(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (state) ? GPIO_PIN_RESET : GPIO_PIN_SET)
//#define bms_run() HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float raw_to_float(void *raw) {
  return (float)(int16_t)(((*(uint16_t *)raw & 0xFF) << 8) | ((*(uint16_t *)raw & 0xFF00) >> 8));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  static bq79600_t *instance = NULL;
  instance = open_bq79600_instance(0);
  if (instance == NULL) instance = open_bq79600_instance(0);
  instance->rx_len = size;
  bq79600_rx_callback(instance);
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, instance->rx_buf, sizeof(instance->rx_buf));
}
void USB_RXCallback(uint8_t* Buf, uint32_t *Len)
{
	memcpy(usbRxBuf, Buf, *Len);
	usbRxBufLen = *Len;
	usbRxFlag = 1;
}

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim1);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of BQ79614_Queue */
  BQ79614_QueueHandle = osMessageQueueNew (10, sizeof(BQ_Data), &BQ79614_Queue_attributes);

  /* creation of BQ79600_Queue */
  BQ79600_QueueHandle = osMessageQueueNew (10, sizeof(Message), &BQ79600_Queue_attributes);

  /* creation of Messages_Queue */
  Messages_QueueHandle = osMessageQueueNew (16, sizeof(char), &Messages_Queue_attributes);

  /* creation of BQ_Safety_Status_Queue */
  BQ_Safety_Status_QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &BQ_Safety_Status_Queue_attributes);

  /* creation of Other_Safety_Status_Queue */
  Other_Safety_Status_QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &Other_Safety_Status_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Default_task */
  Default_taskHandle = osThreadNew(Default, NULL, &Default_task_attributes);

  /* creation of BQ_comm_task */
  BQ_comm_taskHandle = osThreadNew(Bq_comm, NULL, &BQ_comm_task_attributes);

  /* creation of Safety_task */
  Safety_taskHandle = osThreadNew(Safety, NULL, &Safety_task_attributes);

  /* creation of Led_task */
  Led_taskHandle = osThreadNew(Led, NULL, &Led_task_attributes);

  /* creation of Can_task */
  Can_taskHandle = osThreadNew(StartTask05, NULL, &Can_task_attributes);

  /* creation of Usb_task */
  Usb_taskHandle = osThreadNew(Usb, NULL, &Usb_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
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
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
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
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Default */
/**
  * @brief  Function implementing the Default_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Default */
void Default(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Bq_comm */
/**
* @brief Function implementing the BQ_comm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Bq_comm */
void Bq_comm(void *argument)
{
  /* USER CODE BEGIN Bq_comm */

	bq79600_t *bms_instance = open_bq79600_instance(0);

	    bms_instance->mode = BQ_UART;
	    bms_instance->state = BQ_SHUTDOWN;
	    bms_instance->rx_port = GPIOA;
	    bms_instance->tx_port = GPIOA;
	    bms_instance->rx_pin = 1;
	    bms_instance->tx_pin = 0;

	    // wake up ping using slowed uart communication
	    HAL_UART_DeInit(&huart4);
	    MX_UART4_Init(3250);
	    osDelay(10);
	    uint8_t zero=0x00;
	    HAL_UART_Transmit(&huart4,&zero,1,100);
	    osDelay(13);
	   HAL_UART_Transmit(&huart4,&zero,1,100);
	   osDelay(50);
	   HAL_UART_DeInit(&huart4);
	    MX_UART4_Init(1000000);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);


	     if( HAL_UARTEx_ReceiveToIdle_IT(&huart4, bms_instance->rx_buf, sizeof(bms_instance->rx_buf)) == HAL_ERROR)
	     {
	    	 while (1);
	     }
	      HAL_Delay(10);

	       if( HAL_UART_Transmit_IT(&huart4, bms_instance->tx_buf, bms_instance->tx_len) == HAL_BUSY)
	       {
	    	   while (1);
	       }


	      uint8_t buf = 0x20;
	      bq79600_write_reg(bms_instance, 0x00, CONTROL1, &buf, 1);
	      osDelay(12 * n_devices);

	      bq79600_error_t err = bq79600_auto_addressing(bms_instance, n_devices);
	      if (err) {
	    	  Message autoadress = {0};
	    	  strcpy(autoadress.Buf, "Autoadressing failed!\n0");
	    	  autoadress.Timestamp = HAL_GetTick();
	    	  osMessageQueuePut(Messages_QueueHandle, &autoadress, 0, 50);
	    	  }
	      else
	      {
	    	  Message autoadress = {0};
	    	  strcpy(autoadress.Buf, "Autoadressing succesful!\n0");
	    	  autoadress.Timestamp = HAL_GetTick();
	    	  osMessageQueuePut(Messages_QueueHandle, &autoadress, 0, 50);
	      }

	      /* Set long communication timeout */
	      buf = 0x0A;  // CTL_ACT=1 | CTL_TIME=010 (2s)
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, COMM_TIMEOUT_CONF, 5, &buf);
	      bq79600_tx(bms_instance);
	      osDelay(1);

	      /* Config stack device ADCs */
	      buf = n_cells_per_device - 6;
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, ACTIVE_CELL, 1, &buf);
	      bq79600_tx(bms_instance);

	      buf = 0x06;
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, ADC_CTRL1, 1, &buf);
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);

	  //    buf = 0x06;
	   //   bq79600_construct_command(bms_instance, STACK_WRITE, 0, ADC_CTRL3, 1, &buf);
	  //    bq79600_tx(bms_instance);
	  //    osDelay(1 * n_devices);

	      // temp readings  CONTROL2
	     // GPIO_CONF1
   /*

	      buf = 0x01;
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, CONTROL2, 1, &buf); // enable T_REF adc reading
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);


	      buf = 0x09; // 001001  see page 35, 162

	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, GPIO_CONF1, 1, &buf); // enable gpio as OTUT input
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);
   */

	      /*
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, GPIO_CONF2, 1, &buf); // enable gpio as OTUT input
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);


	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, GPIO_CONF3, 1, &buf); // enable gpio as OTUT input
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);


	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, GPIO_CONF4, 1, &buf); // enable gpio as OTUT input
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);








	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, ADC_CTRL3, 1, &buf);
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices);

	      /*  Setup OV, UV for balancing  */
	      uint8_t ov_threshold = 0x22; // 4175 mV threshold value
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, OV_THRESH, 1, &ov_threshold);

	      uint8_t uv_threshold = 0x26; // 3100 mV threshold value
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, UV_THRESH, 1, &uv_threshold);
	      bq79600_tx(bms_instance);
	      uint8_t OV_UV_MODE = 0x01; // Set mode to run OV and UV round robin on all cells
	      uint8_t OV_UV_GO = 0x01; // Start OV UV comparators
	      uint8_t OV_UV_CONTROL_DATA[] = {OV_UV_MODE,OV_UV_GO};
	      buf = 0x3;
	      bq79600_construct_command(bms_instance, STACK_WRITE, 0, OVUV_CTRL, 1, &buf); // 0x26 = 3100mV
	      bq79600_tx(bms_instance);
	      osDelay(1 * n_devices); // wait for stack write

  /* Infinite loop */
	  while (1) {


	         bq79600_construct_command(bms_instance, STACK_READ, 0, DIETEMP1_HI, 2, NULL);
	         bq79600_tx(bms_instance);
	         bq79600_bsp_ready(bms_instance);

	         for (int i = 0; i < n_devices - 1; i++)
	         {
	           modules[i].dietemp = raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.025;
	         }
	         uint32_t start_vcells = VCELL1_HI - n_cells_per_device * 2 + 2;
	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_vcells, n_cells_per_device * 2, NULL);
	         bq79600_tx(bms_instance);
	         bq79600_bsp_ready(bms_instance);

	         for (int i = 0; i < n_devices - 1; i++)
	         {
	           for (int j = 0; j < n_cells_per_device; j++)
	           {
	             modules[i].vcells[j] =
	                 raw_to_float(&bms_instance->rx_buf[4 + i * (n_cells_per_device * 2 + 6) + 2 * j]) * 0.19073;
	           }
	         }
	         memset(&bms_instance->rx_buf, 0 , sizeof(&bms_instance->rx_buf));
	         uint32_t start_temp = GPIO1_HI; // GPIO8_HI ;
	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_temp, n_temp_pre_device  * 2, NULL);
	         bq79600_tx(bms_instance);
	         bq79600_bsp_ready(bms_instance);

	         for (int i = 0; i < n_devices - 1; i++)
	         {
	           for (int j = 0; j < n_temp_pre_device; j++)
	           {
	             modules[i].temperature[j] =
	                 raw_to_float(&bms_instance->rx_buf[4 + i * (n_temp_pre_device * 2 + 6) + 2 * j]) * 0.15259;
	           }
	         }
	         uint32_t start_temp_ref = TSREF_HI ;
	         bq79600_construct_command(bms_instance, STACK_READ, 0, start_temp_ref, 2, NULL);
	         bq79600_tx(bms_instance);
	         bq79600_bsp_ready(bms_instance);

	         for (int i = 0; i < n_devices - 1; i++)
	             modules[i].t_ref =
	                 raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.16954;

	         ntc_cfg_t cfg = {
	           .R_top  = 10000.0f,   // R100
	           .R_bias = 100000.0f,  // R101
	           .R0     = 10000.0f,
	           .B      = 3380.0f     // or 3000 if that’s your actual NTC
	         };
	         float t_c   = gpio_ratio_to_celsius( (modules[1].temperature[0] * -1), modules[1].t_ref ,cfg );



	         for (int i = 0; i < n_devices - 1; i++) modules[i].timestamp = HAL_GetTick();
	         bq79600_construct_command(bms_instance, STACK_READ, 0, DEV_STAT, 1, NULL); // DEV_STAT READ.
	         bq79600_tx(bms_instance);
	         bq79600_bsp_ready(bms_instance);

	         // end of reading data from BQ79600

	         for (int i = 0; i < n_devices - 1; i++) // send data from bq to different tasks.
	         {
		     BQ_Data Data_to_send = {0}; // struct to send to queue
		     Data_to_send.BQ_Number = i ;
		     for (int j = 0; j < n_cells_per_device; j++)
		     Data_to_send.Bq_Voltages[j] = modules[i].vcells[j];
		   //  Data_to_send.Bq_Temperatures[j] = modules[i].temperature;
		     Data_to_send.dietemp =  modules[i].dietemp;
		     Data_to_send.Bq_Timestamp = modules[i].timestamp;

		     uint8_t dev_stat =  bms_instance->rx_buf[4 + i];
		     if (dev_stat & (1 << 6))
		    	 Data_to_send.BQ_Undervoltage_Error = 1;
		     if (dev_stat & (1 << 5))
		    	 Data_to_send.BQ_Overvoltage_Error = 1;
		     if (dev_stat & (1 << 4))
		    	 Data_to_send.BQ_Communication_Error = 1;
            if( osMessageQueuePut(BQ79614_QueueHandle, &Data_to_send, 5, 5) == osOK)
            	 {
           	 Message bq_mes = {0};
             strcpy(bq_mes.Buf, "BQ_Message succesfully sent!\n0");
           	 bq_mes.Timestamp = HAL_GetTick();
           	 osMessageQueuePut(Messages_QueueHandle, &bq_mes, 0, 5);
           	 }
	         }
	         HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2) ;
	         memset(&bms_instance->rx_buf, 0 , sizeof(&bms_instance->rx_buf));
	         vTaskDelay(100);
	         //osDelay(50);
	  }

  /* USER CODE END Bq_comm */
}

/* USER CODE BEGIN Header_Safety */
/**
* @brief Function implementing the Safety_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Safety */
void Safety(void *argument)
{
  /* USER CODE BEGIN Safety */
	// this task is responsible for enabling the relay responsible for supplying power to the inverter.
	int overvoltage = 0;
    int undervoltage = 0;
    int comm_err = 0 ;
    int autoadressing_error = 0 ;
  /* Infinite loop */
  for(;;)
  {

	 // BQ_Data Data_received = {0};
	 /*  if (osMessageQueueGet(BQ79614_QueueHandle, &Data_received, NULL, 10) == osOK)
	  	 {
	  		undervoltage = Data_received.BQ_Undervoltage_Error ;
	  	    overvoltage = Data_received.BQ_Overvoltage_Error ;
	  	    comm_err = Data_received.BQ_Communication_Error ;
	  	    autoadressing_error = Data_received.BQ_Autoadressing_Error;
	  	 } */

	  if(!autoadressing_error && !comm_err && !overvoltage && !undervoltage)
	  	{
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  1);
	  	}
	  	else
	  	{
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  0);
	  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  0);
	  	}
	      osDelay(10);
    osDelay(1);
  }
  /* USER CODE END Safety */
}

/* USER CODE BEGIN Header_Led */
/**
* @brief Function implementing the Led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led */
void Led(void *argument)
{
  /* USER CODE BEGIN Led */

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	int overvoltage = 0;
	int undervoltage = 0;
	int comm_err = 0 ;
  /* Infinite loop */
  for(;;)
  {

	  BQ_Data Data_received = {0};
	  if (osMessageQueueGet(BQ79614_QueueHandle, &Data_received, NULL, 10) == osOK)
	  {
		  undervoltage = Data_received.BQ_Undervoltage_Error ;
	      overvoltage = Data_received.BQ_Overvoltage_Error ;
	      comm_err = Data_received.BQ_Communication_Error ;
	  }


	 if( undervoltage || overvoltage )
	  {
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2) ;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,1); // led pins serve as a pulldown thats why the led-off is to set pin high.

	  }
	  if( comm_err )
	  {
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3) ;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 ,1);
	  }

	  osDelay(250);


  }
  /* USER CODE END Led */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Can_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
	  if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) // for debugging the button resets all the errors; pressed state is zero
		{
		  Data_Receaved.BQ_Autoadressing_Error = Data_Receaved.BQ_Overvoltage_Error = Data_Receaved.BQ_Communication_Error = 0;
		  osDelay(2000);
		  HAL_NVIC_SystemReset();

		}
	  osDelay(100);

  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_Usb */
/**
* @brief Function implementing the Usb_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usb */
void Usb(void *argument)
{
  /* USER CODE BEGIN Usb */
  /* Infinite loop */
	uint8_t priority = 5 ;
	BQ_Data Data_received = {0};
	Message Message_received = {0};
	char message[32]={0};
  for(;;)
  {   //   (Messages_QueueHandle

	       if (osMessageQueueGet(Messages_QueueHandle, &Message_received, &priority, 1) == osOK)
	           {

		         while (CDC_Transmit_FS((uint8_t*)Message_received.Buf, strlen(Message_received.Buf)) == USBD_BUSY){
		        	 vTaskDelay(10);
		         }

	           }

	       if (osMessageQueueGet(BQ79614_QueueHandle, &Data_received, &priority, 1) == osOK)
	  		{
	    	   for(int i = 0 ; i< n_cells_per_device; i++ )
	    	                 {
	    	                 char message[64] = {0};
	    	                 sprintf(message  , "BQ Number:%d bq voltage value:%d [mV] \n" ,Data_received.BQ_Number+1 , (int)Data_received.Bq_Voltages[i] );
	    	                // CDC_Transmit_FS((uint8_t*)message, strlen(message));
	    	                 while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
	    	                	 	 	 vTaskDelay(1); // Delay to allow USB stack to process
	    	                 	    	   }
	    	                 }

	    	   	   	   	   	if(Data_received.BQ_Overvoltage_Error)
	    	   	   	   	   	{
	    	   	   	   	   		sprintf(message  , "BQ OVERVOLTAGE ERROR! \n" );
	    	   	   	   		   while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
	    	   	   	   		    	   vTaskDelay(1); // Delay to allow USB stack to process
	    	   	   	   	          	}
	    	   	   	   	   	}
		    	   	   	   	   	if(Data_received.BQ_Undervoltage_Error)
		    	   	   	   	   	{
		    	   	   	   	   		sprintf(message  , "BQ UNDERVOLTAGE ERROR! \n" );
		    	   	   	   		   while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
		    	   	   	   		    	   vTaskDelay(1); // Delay to allow USB stack to process
		    	   	   	   	   	         }
	    	        	    	   }
		    	   	   	   	   	if(Data_received.BQ_Communication_Error)
		    	   	   	   	   	{
		    	   	   	   	   		sprintf(message  , "BQ COMM ERROR! \n" );
		    	   	   	   		   while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
		    	   	   	   		    	   vTaskDelay(1); // Delay to allow USB stack to process
		    	   	   	   	   	         }
	    	        	    	   }
		    	   	   	   	   	   sprintf(message  , "Temperature of BQ: %d  [deg C]\n" , (int)Data_received.dietemp );
		    	   	   	   	   	   while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
		    	   	   	   	   	   		   	   vTaskDelay(1); // Delay to allow USB stack to process
		    	   	   	   	   }
		    	   	   	   	   	   sprintf(message  , "Timestamp: %d \n" , Data_received.Bq_Timestamp );
		    	   	   	   	   	   while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
		    	   	   	   	   		   	   vTaskDelay(1); // Delay to allow USB stack to process
		    	   	   	   	   	   }
	    	                 //CDC_Transmit_FS((uint8_t*)message2, strlen(message2));
	    	                 osDelay(5);
	    	                 char message[6] = " \n";
	    	                 for(int i = 0 ; i< 3; i++ )
	    	                while (CDC_Transmit_FS((uint8_t*)message, strlen(message)) == USBD_BUSY) {
	    	                			vTaskDelay(10); // Delay to allow USB stack to process
	    	                }




	  		}
	       HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3) ;

	  if(usbRxFlag && usbRxBufLen)
	  {

		 /* switch(usbd_ch)
		  {
		  case '1' :
			   CDC_Transmit_FS((uint8_t*)buffer_usb1, strlen(buffer_usb1));
			   break;
		  case '2' :
			   CDC_Transmit_FS((uint8_t*)buffer_usb2, strlen(buffer_usb2));
			   break;
		  case '3' :
			   CDC_Transmit_FS((uint8_t*)buffer_usb3, strlen(buffer_usb3));
			   break;
		  case '4' :
			  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
			   break;
		  }
		  flag_usb = 0 ; */
		  CDC_Transmit_FS((uint8_t*)buffer_usb1, strlen(buffer_usb1));
		  usbRxFlag = 0 ;
		  usbRxBufLen = 0 ;
		  vTaskDelay(100);

	  }


	  vTaskDelay(1);
    //osDelay(1);
  }

  /* USER CODE END Usb */
}

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
  if (htim->Instance == TIM5) {
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
