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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "canhandler.h"
#include "commu.h""
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 2049 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* USER CODE BEGIN PV */
#define MQTT_PAYLOAD_LIMIT      550//was 450
#define MQTT_CMD_LIMIT          600//was 700
static char mqttPayload[MQTT_PAYLOAD_LIMIT];
static char mqttCmd[MQTT_CMD_LIMIT];

#define UART_RX_BUF_SIZE 256
uint8_t uartRxByte;
uint8_t uartRxBuffer[UART_RX_BUF_SIZE];
volatile uint8_t uartRxIndex = 0;
volatile uint8_t uartPacketReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART8_Init(void);
static void MX_TIM4_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART7_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEVICE_ID "STM_XY"
uint8_t rxData[8]={0}, txData[5]={0};
//static uint32_t modemFaultTimer = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART8_Init();
  MX_TIM4_Init();
  MX_LPUART1_UART_Init();
  MX_UART7_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart7, &uartRxByte, 1);
  HAL_UART_Receive_IT(&huart6,rxBuffer,1);
    memset(UART5_Data,'\0',sizeof(UART5_Data));

    receivedCount=0;
  //  HAL_UART_Receive_IT(&huart6, &gps_rx_byte, 1); // GPS disabled — USART6 used for 4G

    step = 0;
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_6;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 32;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 32;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
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
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 32;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 32;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 32;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00707CBB;
  hi2c4.Init.OwnAddress1 = 110;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  HAL_UART_Receive_IT(&hlpuart1, &rxData[0], 1);

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  HAL_UART_Receive_IT(&huart5, &rxData[0], 1);

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */
  HAL_UART_Receive_IT(&huart8, &rxData[0], 1);

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  HAL_UART_Receive_IT(&huart2, &rxData[0], 1);

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
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

  HAL_UART_Receive_IT(&huart3, &rxData[0], 1);

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  HAL_UART_Receive_IT(&huart6, &rxData[0], 1);

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6399;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim4.Init.Prescaler = 63999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(
UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART6)
    {
        uart5TxBusy=0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	/* ================= wifi on USART7 ================= */
	if (huart->Instance == UART7)
	    {
	        /* Check for line endings ('\n' or '\r') or buffer limit */
	        if (uartRxByte == '\n' || uartRxByte == '\r' || uartRxIndex >= (UART_RX_BUF_SIZE - 1))
	        {
	            if (uartRxIndex > 0) // Only process if we actually received characters
	            {
	                uartRxBuffer[uartRxIndex] = '\0'; // Null-terminate string

	                /* Copy full command packet to currentRx for default task */
	                strncpy(currentRx, (char*)uartRxBuffer, sizeof(currentRx) - 1);
	                currentRx[sizeof(currentRx) - 1] = '\0';

	                uartPacketReady = 1;
	                uartRxIndex = 0; // Reset index for next command
	            }
	        }
	        else
	        {
	            /* Store character in temporary RX line buffer */
	            uartRxBuffer[uartRxIndex++] = uartRxByte;
	        }

	        /* Re-enable UART RX interrupt for the next byte */
	        HAL_UART_Receive_IT(&huart7, &uartRxByte, 1);
	    }

    /* ================= 4G on USART5 ================= */

    if (huart->Instance == USART6)
    {
    receiveUart5();

    }
}


void receiveUart5(void)
{
    uint8_t ch=rxBuffer[0];

    static uint16_t idx=0;

    if(ch=='\r')
    {
        HAL_UART_Receive_IT(&huart6,rxBuffer, 1);

        return;
    }

    if(ch=='\n')
    {
        UART5_Data[idx]='\0';

        if(idx>0)
        {
            UART5_Push((char*)UART5_Data);
        }

        idx=0;

        memset(UART5_Data,0,sizeof(UART5_Data));
    }
    else
    {
        if(idx<
        sizeof(UART5_Data)-1)
        {
            UART5_Data[idx++]=ch;
        }
    }

    HAL_UART_Receive_IT(&huart6,rxBuffer,1);
}


void Uart5Transmit(char* txBuffer)
{
    while(uart5TxBusy)
    {
        osDelay(1);
    }

    uart5TxBusy=1;

    memset(lastTx,0,sizeof(lastTx));

    strcpy(lastTx,txBuffer);

//    HAL_UART_Transmit_IT(&huart6,(uint8_t*)txBuffer,strlen(txBuffer));

    if(HAL_UART_Transmit_IT(&huart6,(uint8_t*)lastTx,strlen(lastTx))!=HAL_OK)
    {
    	Uart5transmit_errorflag=1;
    }


}
void UART5_Push(char *line)
{
    uint8_t next=
    (uart5QWrite+1)%UART5_QUEUE_SIZE;

    if(next!=uart5QRead)
    {
        strcpy(uart5Queue[uart5QWrite],line);

        uart5QWrite=next;
    }
}


uint8_t UART5_Pop(char *out)
{
    if(uart5QRead==uart5QWrite)
        return 0;

    strcpy(out,uart5Queue[uart5QRead]);

    uart5QRead=(uart5QRead+1)%UART5_QUEUE_SIZE;

    return 1;
}


void Parse4GCommand(char *rx)
{
    char stmId[10] = {0};
    char type[20]  = {0};
    float value    = 0.0f;

    if (sscanf(rx, "%9[^,],%19[^,],%f", stmId, type, &value) != 3)
        return;

    if (strcmp(stmId, DEVICE_ID) != 0)
        return;

    strncpy((char *)g_cmdType, type, sizeof(g_cmdType) - 1);
    g_cmdType[sizeof(g_cmdType) - 1] = '\0';

    g_cmdValue = value;

//    update_tmcm_from_uart();
//    TMCM_CheckAndSendCAN();

    /* Prepare ACK (DO NOT SEND HERE) */
    snprintf(ackBuffer, sizeof(ackBuffer),
             "ACK,%s,%s,%.2f", stmId, type, value);

    ackPendingHW = 1;
    lastAckHWTick = HAL_GetTick();
}

char* ExtractDeviceCommand(char *rx, char *out, uint16_t outSize)
{
	char searchStr[20];
	snprintf(searchStr,
	sizeof(searchStr),
	"%s,",
	DEVICE_ID);
	char*start=strstr(rx, searchStr);

    if (!start)
        return NULL;

    char *end = strstr(start, "\r");
    uint16_t len = end ? (end - start) : strlen(start);

    if (len >= outSize)
        len = outSize - 1;

    memcpy(out, start, len);
    out[len] = '\0';

    return out;
}
void ProcessMQTTSubscription(void)
{
	char searchStr[20];
	snprintf(searchStr,
	sizeof(searchStr),
	"%s,",
	DEVICE_ID);
	if(responseReady&&
	strstr(lastRx, searchStr))
    {
        if(ExtractDeviceCommand(
                lastRx,
                mqttCmdBuffer,
                sizeof(mqttCmdBuffer)))
        {
            mqttCmdReady=1;
        }

        responseReady=0;
        memset(lastRx,0,sizeof(lastRx));
    }
}

///* ============================================================
// * APPLY MODEM TIME TO STM32 RTC (with TZ correction)
// * ============================================================ */
//void ApplyCCLKToRTC(char *cclk_str)
//{
//    char *start = strchr(cclk_str, '"');
//    if (start == NULL)
//        return;
//    start++;
//
//    int year = 0, month = 0, day = 0;
//    int hour = 0, minute = 0, second = 0;
//    int tz_quarters = 0;
//
//    /*
//     * Expected format:
//     * "26/06/05,11:45:48+22"
//     * tz_quarters is signed, in units of 15 minutes from GMT
//     */
//    int n = sscanf(start,
//                    "%d/%d/%d,%d:%d:%d%d",
//                    &year, &month, &day,
//                    &hour, &minute, &second,
//                    &tz_quarters);
//
//    if (n < 6)
//    {
//        printf("CCLK parse failed\r\n");
//        return;
//    }
//    if (n < 7)
//    {
//        printf("TZ field missing, assuming 0\r\n");
//        tz_quarters = 0;
//    }
//
//    printf("Modem reported: 20%02d-%02d-%02d %02d:%02d:%02d, tz=%d (%d min)\r\n",
//           year, month, day, hour, minute, second,
//           tz_quarters, tz_quarters * 15);
//
//    /* Build a struct tm from the modem's raw fields */
//    struct tm t = {0};
//    t.tm_year = year + 2000 - 1900;   /* tm_year is years since 1900 */
//    t.tm_mon  = month - 1;            /* tm_mon is 0-11 */
//    t.tm_mday = day;
//    t.tm_hour = hour;
//    t.tm_min  = minute;
//    t.tm_sec  = second;
//
//    /* mktime() on bare-metal newlib has no tz database, so it
//     * treats the struct as a plain calendar->epoch conversion
//     * (effectively UTC-agnostic arithmetic) - safe to use here. */
//    time_t epoch = mktime(&t);
//
//    /* Undo the modem's (possibly wrong) local adjustment to get true UTC */
//    epoch -= (time_t)tz_quarters * 15 * 60;
//
//    /* Apply OUR known-correct IST offset */
//    epoch += (time_t)IST_OFFSET_MINUTES * 60;
//
//    struct tm *local = gmtime(&epoch);
//    if (local == NULL)
//    {
//        printf("Time conversion failed\r\n");
//        return;
//    }
//
//    printf("Corrected IST Time: %04d-%02d-%02d %02d:%02d:%02d\r\n",
//           local->tm_year + 1900, local->tm_mon + 1, local->tm_mday,
//           local->tm_hour, local->tm_min, local->tm_sec);
//
//    RTC_TimeTypeDef sTime = {0};
//    RTC_DateTypeDef sDate = {0};
//
//    sTime.Hours          = (uint8_t)local->tm_hour;
//    sTime.Minutes        = (uint8_t)local->tm_min;
//    sTime.Seconds        = (uint8_t)local->tm_sec;
//    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//
//    sDate.Year    = (uint8_t)(local->tm_year + 1900 - 2000);
//    sDate.Month   = (uint8_t)(local->tm_mon + 1);
//    sDate.Date    = (uint8_t)local->tm_mday;
//    sDate.WeekDay = (local->tm_wday == 0) ? RTC_WEEKDAY_SUNDAY : (local->tm_wday); /* fix below */
//
//    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
//    {
//        printf("HAL_RTC_SetTime failed\r\n");
//        return;
//    }
//
//    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//    {
//        printf("HAL_RTC_SetDate failed\r\n");
//        return;
//    }
//
//    printf("RTC successfully written\r\n");
//}
//
//
///* ── Helper: read RTC and format timestamp string ───────────────────── */
//void GetRTCTimestamp(char *buffer, size_t max_len)
//{
//    RTC_TimeTypeDef sTime = {0};
//    RTC_DateTypeDef sDate = {0};
//
//    // Read Time FIRST, then Date
//    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//
//    uint32_t msec = 0;
//    if (sTime.SecondFraction > 0)
//    {
//        msec = ((sTime.SecondFraction - sTime.SubSeconds) * 1000) / (sTime.SecondFraction + 1);
//    }
//
//    // FIXED: Changed "202%d" to "20%02d" to fix the 20226 year error
//    // Format out a clean, standard space-separated log string: "2026-05-25 15:17:01.430"
//    snprintf(buffer, max_len, "20%02d-%02d-%02d %02d:%02d:%02d.%03lu",
//             sDate.Year, sDate.Month, sDate.Date,
//             sTime.Hours, sTime.Minutes, sTime.Seconds, msec);
//}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	CAN_RAM_Init();
	canInit();
	HAL_TIM_Base_Start_IT(&htim4);
	//CAN_RAM_Log_t rxFrame;
  /* Infinite loop */
  for(;;)
  {
	  CheckServerCommand(currentRx);
	  if (g_sendCanRamDataFlag == 1)
	        {
	            CAN_RAM_FlushToESP();
//
//	            /* Optional: Clear currentRx command string after processing so it doesn't re-trigger */
//	            memset(currentRx, 0, sizeof(currentRx));
	        }
//	  while (CAN_RAM_ReadFrame(&rxFrame))
//	          {
//
//	          }
	  subModuleCanHandler();
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	static uint8_t ntp_triggered = 0;
	static uint8_t ntp_completed = 0;
	static uint32_t ntpTimer     = 0;
	volatile uint8_t rtc_sync_performed = 0;
  /* Infinite loop */
  for(;;)
  {
	  uint8_t rxAvailable = 0;

	         if(UART5_Pop(currentRx))
	         {
	             rxAvailable = 1;
	             int signal;
	             int ber;

	             /* Look for any xx,yy pattern that comes from +CSQ */
	             if(sscanf(currentRx, "%*[^0-9]%d,%d", &signal, &ber) == 2)
	             {
	                 if(signal >= 0 && signal <= 31)
	                 {
	                     gsm_csq = (uint8_t)signal;

	                     gsm_rssi = -113 + (2 * signal);

	                     printf("CSQ=%d BER=%d RSSI=%d\r\n",
	                            signal,
	                            ber,
	                            gsm_rssi);
	                 }
	             }

	             if(strstr(currentRx, "+MQTTSUB"))
	             {
	                 char cmdBuf[4096]; //was 100
	                 if(ExtractDeviceCommand(currentRx, cmdBuf, sizeof(cmdBuf)))
	                 {
	                 	strcpy(mqttPacketBuffer, cmdBuf);
	                     mqttPacketReady = 1;
	                     Parse4GCommand(cmdBuf);
	                 }
	                 continue;
	             }

	             if(strstr(currentRx, "Link Closed") ||
	                strstr(currentRx, "ERROR")        ||
	                strstr(currentRx, "FAIL"))
	             {
	                 tcpRetryCount = 0;
	                 modemFaultActive = 1;
	                                // modemFaultTimer = HAL_GetTick();
	                 step = STEP_CGATT;
	             }
	         }
	         /* ================= INIT ================= */
	         if(step == STEP_INIT)
	         {
	             Uart5Transmit("ATE0\r");
	             stepTimer    = HAL_GetTick();
	             responseReady = 0;
	             step = STEP_AT;
	         }

	         /* ================= AT ================= */
	         else if(step == STEP_AT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 Uart5Transmit("AT+CPIN?\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_CPIN;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_INIT;
	             }
	         }

	         /* ================= CPIN ================= */
	         else if(step == STEP_CPIN)
	         {
	             if(rxAvailable && strstr(currentRx, "READY"))
	             {
	                 Uart5Transmit("AT+CSQ\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_CSQ;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_AT;
	             }
	         }

	         /* ================= CSQ ================= */
	         else if(step == STEP_CSQ)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 Uart5Transmit("AT+CGATT=1\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_CGATT;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CPIN;
	             }
	         }

	         /* ================= CGATT ================= */
	         else if(step == STEP_CGATT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 Uart5Transmit("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_CGDCONT;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CSQ;
	             }
	         }

	         /* ================= CGDCONT ================= */
	         else if(step == STEP_CGDCONT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 step = STEP_XIIC;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CGATT;
	             }
	         }

	         /* ================= XIIC ENABLE ================= */
	         else if(step == STEP_XIIC)
	         {
	             Uart5Transmit("AT+XIIC=1\r");
	             stepTimer = HAL_GetTick();
	             step = STEP_XIIC + 1;
	         }

	         else if(step == STEP_XIIC + 1)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 Uart5Transmit("AT+XIIC?\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_XIIC + 2;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CGATT;
	             }
	         }

	         else if(step == STEP_XIIC + 2)
	         {
	             if(rxAvailable && strstr(currentRx, "XIIC"))
	             {
	                 step = STEP_MQTT_CONFIG;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CGATT;
	             }
	         }

	         /* ---------- MQTT PARAM ---------- */
	         else if(step == STEP_MQTT_CONFIG)
	         {
	             Uart5Transmit("AT+MQTTMUX=0\r");
	             stepTimer = HAL_GetTick();
	             step = STEP_MQTTMUX_WAIT;
	         }

	         else if(step == STEP_MQTTMUX_WAIT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 char cmd[100];
	                 sprintf(cmd, "AT+MQTTCONNPARAM=\"%s\",\"\",\"\"\r", DEVICE_ID);
	                 Uart5Transmit(cmd);
	                 stepTimer = HAL_GetTick();
	                 step = STEP_MQTT_CONNECT;
	             }
	         }

	         /* ---------- MQTT CONNECT ---------- */
	         else if(step == STEP_MQTT_CONNECT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 char cmd[100];
	                 sprintf(cmd, "AT+MQTTCONN=\"%s\",0,60\r", MQTT_BROKER);
	                 Uart5Transmit(cmd);
	                 stepTimer = HAL_GetTick();
	                 step = STEP_MQTT_CONNECT_WAIT;
	             }
	         }

	         else if(step == STEP_MQTT_CONNECT_WAIT)
	         {
	             if(rxAvailable && strstr(currentRx, "OK"))
	             {
	                 step = STEP_MQTT_SUB;
	             }
	             else if(HAL_GetTick() - stepTimer > 8000)
	             {
	                 step = STEP_CGATT;
	             }
	         }

	         /* ---------- SUBSCRIBE ---------- */
	         else if(step == STEP_MQTT_SUB)
	         {
	             char cmd[100];
	             sprintf(cmd, "AT+MQTTSUB=\"%s\",0\r", MQTT_TOPIC_CMD);
	             Uart5Transmit(cmd);
	             osDelay(100);
	             step = STEP_CONNECTED;
	         }

	         /* ---------- CONNECTED ---------- */
	         else if(step == STEP_CONNECTED)
	         {
	         	modemFaultActive = 0;
	         	        	//motorStopSent = 0;
	         	            static uint32_t dataTimer = 0;
	         	            static uint32_t canTimer  = 0;
	         	            static uint32_t csqTimer = 0;
	         	            static CAN4GPacket_t pkt;

	             // Shared array to fetch the timestamp when needed
	             char ts[30];

	             if (!rtc_sync_performed)
	             {
	                 Uart5Transmit("AT+UPDATETIME=1,ntp.ubuntu.com,30,\"E5\",0\r");
	                 stepTimer = HAL_GetTick();
	                 step = STEP_NTP_WAIT;
	                 continue;
	             }

	             /* ── TMCM ACK timeout — publish with PCM_ACK suffix if HW ACK never arrives ── */
	             if(ackPendingHW && (HAL_GetTick() - lastAckHWTick > 500))
	             {
	                 strncat(ackBuffer, ",PCM_ACK", sizeof(ackBuffer) - strlen(ackBuffer) - 1);
	                 ackPending = 1;
	                 ackPendingHW = 0;
	             }

	             /* ── ACK publish ── */
	             if(ackPending && !ackSending)
	             {
	                 char cmd[180];
	                 sprintf(cmd, "AT+MQTTPUB=0,0,\"%s\",\"%s\"\r", MQTT_TOPIC_ACK, ackBuffer);
	                 Uart5Transmit(cmd);
	                 ackSending  = 1;
	                 ackPending  = 0;
	                 step        = STEP_SEND_ACK_WAIT;
	                 stepTimer   = HAL_GetTick();
	             }

//	             /* ── CAN publish (Fires strictly every 1000ms) ── */
//	                         else if(HAL_GetTick() - canUploadTimer >= 1000)
//	                         {
//	                             // Snapshot the current count in a local variable to prevent race conditions
//	                             uint16_t packetsToProcess = canLogCount;
//
//	                             if(packetsToProcess > 0)
//	                             {
//	                                 // Fetch fresh timestamp for this 1-second transmission burst
//	                               //  GetRTCTimestamp(ts, sizeof(ts));
//
//	                                 // Format payload prefix string
//	                                 snprintf(mqttPayload, sizeof(mqttPayload), "%s,TS,%s,", DEVICE_ID, ts);
//
//	                                 CANLogPacket_t tempPkt;
//
//	                                 // Only pull out the exact number of logs that were waiting when the timer ticked
//	                                 while(packetsToProcess > 0)
//	                                 {
//	                                 	if((strlen(mqttPayload) + 50) >= 540)//was 380
//	                                 	    {
//	                                 	       dropcount++;
//
//	                                 	        break;
//	                                 	     }
//	                                     if(CANLog_Pop(&tempPkt))
//	                                     {
//	                                     	mqttPayloadcount++;
//	                                     	if(mqttPayloadcount==1){
//	                                     		initial_flag=1;
//	                                     	}
//	                                         char tempStr[64];
//	                                         snprintf(tempStr, sizeof(tempStr), "CAN%d,%02X%02X%02X%02X%02X%02X%02X%02X,%d,%d,%s;",
//	                                                  tempPkt.frameType,
//	                                                  tempPkt.data[0], tempPkt.data[1],
//	                                                  tempPkt.data[2], tempPkt.data[3],
//	                                                  tempPkt.data[4], tempPkt.data[5],
//	                                                  tempPkt.data[6], tempPkt.data[7],mqttPayloadcount,tempPkt.seq,tempPkt.time_var);
//
//	                                         /* Prevent string payload sizing overflow (Safeguard) */
//
//
//	                                         strcat(mqttPayload, tempStr);
//	                                     }
//	                                     packetsToProcess--;
//	                                 }
//
//	                                 // Transmit compiled batch array over AT command
//	                                 snprintf(mqttCmd, sizeof(mqttCmd), "AT+MQTTPUB=0,1,\"%s\",\"%s\"\r",
//	                                          MQTT_TOPIC_CAN,
//	                                          mqttPayload);
//
//	                                 Uart5Transmit(mqttCmd);
//
//	                                 step = STEP_SEND_DATA_WAIT;
//	                                 stepTimer = HAL_GetTick();
//	                             }
//
//
//	                             canUploadTimer += 1000;//canUploadTimer = HAL_GetTick(); // Always reset window execution point
//	                         }
//
//	             	            /* ── Single CAN packet publish (Fallback queue) ── */
//	             	            else if(can4GCount && (HAL_GetTick() - canTimer > 1000))
//	             	            {
//	             	                if(CAN4G_Pop(&pkt))
//	             	                {
//	             	                    char payload[180]; // Increased buffer size to fit timestamp
//	             	                    char cmd[260];
//
//	             	                    // Fetch current timestamp
//	             	                   // GetRTCTimestamp(ts, sizeof(ts));
//
//	             	                    // Formatted matching the telemetry patterns
//	             	                    snprintf(payload, sizeof(payload), "%s,TS,%s,CAN%d,%02X%02X%02X%02X%02X%02X%02X%02X",
//	             	                             DEVICE_ID, ts,
//	             	                             pkt.frameType,
//	             	                             pkt.data[0], pkt.data[1], pkt.data[2], pkt.data[3],
//	             	                             pkt.data[4], pkt.data[5], pkt.data[6], pkt.data[7]);
//
//	             	                    sprintf(cmd, "AT+MQTTPUB=0,0,\"%s\",\"%s\"\r", MQTT_TOPIC_CAN, payload);
//	             	                    Uart5Transmit(cmd);
//	             	                    canDataReady = 0;
//	             	                    step         = STEP_SEND_DATA_WAIT;
//	             	                    stepTimer    = HAL_GetTick();
//	             	                }
//	             	                canTimer = HAL_GetTick();
//	             	            }
//	             	            else if(HAL_GetTick() - csqTimer >= 5000)   // every 30 sec
//	             	                        	            {
//	             	                        	                Uart5Transmit("AT+CSQ\r");
//
//	             	                        	                step = STEP_CSQ_UPDATE;
//	             	                        	                stepTimer = HAL_GetTick();
//	             	                        	                csqTimer = HAL_GetTick();
//	             	                        	            }
	             /* ── TMCM ACK immediate publish (no throttle) ── */
	             if(canTmcmAckReady)
	             {
	                 canTmcmAckReady = 0;

	                 char ackPayload[180];
	                 char cmd[260];
	                // GetRTCTimestamp(ts, sizeof(ts));

	                 snprintf(ackPayload, sizeof(ackPayload),
	                          "%s,TS,%s,TYPE,TMCM_ACK,RESP,%02X",
	                          DEVICE_ID, ts,
	                          canTmcmAckFrame[0]);

	                 sprintf(cmd, "AT+MQTTPUB=0,0,\"%s\",\"%s\"\r", MQTT_TOPIC_ACK, ackPayload);
	                 Uart5Transmit(cmd);

	                 step      = STEP_SEND_DATA_WAIT;
	                 stepTimer = HAL_GetTick();
	             }





	             /* ── Rotating telemetry (1 Hz) ── */
	             //else if(HAL_GetTick() - dataTimer > 1000)
	             else if ((int32_t)(HAL_GetTick() - dataTimer) >= 1000)
	             {
	                 char ts[30];
	                 char cmd[2200];

	                 //GetRTCTimestamp(ts, sizeof(ts));
	                 memset(dataBuffer, 0, sizeof(dataBuffer));
	 //
	 //                if(telemetryIndex == 0)
	 //                {
	 ////                    snprintf(dataBuffer, sizeof(dataBuffer), "DATA,%s,TS,%s,TYPE,TEMP,T1,%.2f,T2,%.2f",
	 ////                             DEVICE_ID, ts, temp1_celsius, temp2_celsius);
	 //                }
	 //                 if(telemetryIndex == 0)
	 //                {
	 //                     snprintf(dataBuffer, sizeof(dataBuffer), "DATA,%s,TS,%s,TYPE,COUNT,VALUE,%lu",
	 //                              DEVICE_ID, ts, transition_count);
	 //                     snprintf(dataBuffer, sizeof(dataBuffer),
	 //                	                  "DATA,%s,TS,%s,TYPE,TOF_STOP,DIST,%.1f,POS_CNT,%lu,SUD_CHG,%lu",
	 //                	                  DEVICE_ID,
	 //                	                  ts,
	 //                	                  distance_cm,
	 //                	                  (unsigned long)position_count,
	 //                	                  (unsigned long)sudden_change_counter);
	 //                }
	                 if(telemetryIndex == 0)
	                                 {
//	                                 /* Compact header string: reduced key lengths and dropped brackets */
//	                                 int offset = snprintf(dataBuffer, sizeof(dataBuffer),
//	                                 "DATA,%s,TS,%s,T,TB,S:",
//	                                 DEVICE_ID, ts);
//
//	                                 TofSample_t rxSample;
//	                                 uint8_t sample_count = 0;
//
//	                                 /* Drain all accumulated samples from the queue */
//	                                 while(osMessageQueueGet(tofQueueHandle, &rxSample, NULL, 0) == osOK)
//	                                 {
//	                                 /* Compact sample format: DIST,POS,SUD separated by semicolon */
//	                                 int written = snprintf(dataBuffer + offset, sizeof(dataBuffer) - offset,
//	                                 "%s%.1f,%lu,%lu",
//	                                 (sample_count > 0) ? ";" : "",
//	                                 rxSample.distance_cm,
//	                                 (unsigned long)rxSample.position_count,
//	                                 (unsigned long)rxSample.sudden_change_counter);
//
//	                                 if (written > 0 && (offset + written) < (sizeof(dataBuffer) - 5))
//	                                 {
//	                                 offset += written;
//	                                 sample_count++;
	                                 //}
//	                                 else
//	                                 {
//	                                // break; /* Prevent buffer overflow */
//	                                 }
//	                                 }
	                                 }
	 //                if(telemetryIndex == 0)
	 //                {
	 //                    /* Initialize the buffer with the new header format and save the initial offset */
	 //                    int offset = snprintf(dataBuffer, sizeof(dataBuffer),
	 //                                          "DATA,%s,TS,%s,TYPE,COUNT,VALUE,%lu",
	 //                                          DEVICE_ID, ts, transition_count);
	 //
	 //                    TofSample_t rxSample;
	 //                    uint8_t sample_count = 0;
	 //
	 //                    /* Check if the initial snprintf succeeded */
	 //                    if (offset > 0 && (size_t)offset < sizeof(dataBuffer))
	 //                    {
	 //                        /* Drain all accumulated samples from the queue */
	 //                        while(osMessageQueueGet(tofQueueHandle, &rxSample, NULL, 0) == osOK)
	 //                        {
	 //                            /* Compact sample format: DIST,POS,SUD separated by semicolon */
	 //                            int written = snprintf(dataBuffer + offset, sizeof(dataBuffer) - offset,
	 //                                                   "%s%.1f,%lu,%lu",
	 //                                                   (sample_count > 0) ? ";" : "",
	 //                                                   rxSample.distance_cm,
	 //                                                   (unsigned long)rxSample.position_count,
	 //                                                   (unsigned long)rxSample.sudden_change_counter);
	 //
	 //                            if (written > 0 && (offset + written) < (int)(sizeof(dataBuffer) - 5))
	 //                            {
	 //                                offset += written;
	 //                                sample_count++;
	 //                            }
	 //                            else
	 //                            {
	 //                                break; /* Prevent buffer overflow */
	 //                            }
	 //                        }
	 //                    }
	 //                }
	                 // else if(telemetryIndex == 1)
	                 // {
	                 // 	if(gps_fix_valid)
	                 //     {
	                 // 	     snprintf(dataBuffer, sizeof(dataBuffer), "DATA,%s,TS,%s,TYPE,GPS,LAT,%.7f,LON,%.7f",
	                 // 	              DEVICE_ID, ts, gps_latitude_deg, gps_longitude_deg);
	                 // 	 }
	                 // 	 else
	                 // 	 {
	                 // 	     snprintf(dataBuffer, sizeof(dataBuffer), "DATA,%s,TS,%s,TYPE,GPS,NOFIX", DEVICE_ID, ts);
	                 // 	  }
	                 //  }
	                 // else if(telemetryIndex == 2)
	                 //                 {
	                 //                     snprintf(dataBuffer,
	                 //                              sizeof(dataBuffer),
	                 //                              "DATA,%s,TS,%s,TYPE,SIGNAL,CSQ,%u,RSSI,%d",
	                 //                              DEVICE_ID,
	                 //                              ts,
	                 //                              gsm_csq,
	                 //                              gsm_rssi);
	                 //                 }
	                 /*else if(telemetryIndex == 3) // uncomment to publish rpm values
	                                 {
	                                     if(mqttRpmSentFlag)
	                                     {
	                                         snprintf(dataBuffer, sizeof(dataBuffer),
	                                                  "DATA,%s,TS,%s,TYPE,RPM,VALUE,%d",
	                                                  DEVICE_ID, ts, lastMqttRpmSent);
	                                         mqttRpmSentFlag = 0;
	                                     }
	                                     else
	                                     {
	                                         snprintf(dataBuffer, sizeof(dataBuffer),
	                                                  "DATA,%s,TS,%s,TYPE,RPM,IDLE",
	                                                  DEVICE_ID, ts);
	                                     }
	                                 }*/


	                 sprintf(cmd, "AT+MQTTPUB=0,0,\"%s\",\"%s\"\r", MQTT_TOPIC_DATA, dataBuffer);
	                 Uart5Transmit(cmd);

	                 telemetryIndex++;
	                 if(telemetryIndex >= 1) telemetryIndex = 0; //was 4

	                 step      = STEP_SEND_DATA_WAIT;
	                 stepTimer = HAL_GetTick();
	                 dataTimer += 1000;
	                 //dataTimer = HAL_GetTick(); // Reset execution window
	                 /* Prevent burst/catch-up if the modem delayed us badly */
	                 if ((int32_t)(HAL_GetTick() - dataTimer) >= 1000)
	                 {
	                     dataTimer = HAL_GetTick();
	                 }
	             }
	         }

	         else if(step == STEP_CSQ_UPDATE)
	                 {
	                     if(rxAvailable)
	                     {
	                         if(strstr(currentRx, "OK"))
	                         {
	                             step = STEP_CONNECTED;
	                         }
	                     }

	                     if(HAL_GetTick() - stepTimer > 3000)
	                     {
	                         step = STEP_CONNECTED;
	                     }
	                 }

	         /* ================= NTP WAIT ================= */
	         else if(step == STEP_NTP_WAIT)
	         {
	             if(rxAvailable)
	             {
	                 printf("NTP RX: %s\r\n", currentRx);

	                 if(strstr(currentRx, "OK") || strstr(currentRx, "UPDATE") || strstr(currentRx, "Update") || strstr(currentRx, "+TIME"))
	                 {
	                     printf("Requesting modem clock...\r\n");
	                     Uart5Transmit("AT+CCLK?\r");
	                     stepTimer = HAL_GetTick();
	                     step = STEP_CCLK_WAIT;
	                 }
	                 else if(strstr(currentRx, "ERROR") || strstr(currentRx, "FAIL") || strstr(currentRx, "Time Out") || strstr(currentRx, "tOut"))
	                 {
	                     printf("NTP sync failed\r\n");
	                     rtc_sync_performed = 1;
	                     step = STEP_CONNECTED;
	                 }
	             }

	             if(HAL_GetTick() - stepTimer > 15000)
	             {
	                 printf("NTP wait timeout\r\n");
	                 rtc_sync_performed = 1;
	                 step = STEP_CONNECTED;
	             }
	         }

	         /* ================= CCLK WAIT ================= */
	         else if(step == STEP_CCLK_WAIT)
	         {
	             if(rxAvailable)
	             {
	                 printf("CCLK RX: %s\r\n", currentRx);
	                 if(strstr(currentRx, "+CCLK:"))
	                 {
	                     printf("Applying RTC time...\r\n");
	                     //ApplyCCLKToRTC(currentRx);
	                     rtc_sync_performed = 1;
	                     printf("RTC updated successfully\r\n");
	                     dataTimer = HAL_GetTick();
	                     step = STEP_CONNECTED;
	                 }
	             }

	             if(HAL_GetTick() - stepTimer > 5000)
	             {
	                 printf("CCLK timeout\r\n");
	                 rtc_sync_performed = 1;
	                 step = STEP_CONNECTED;
	             }
	         }

	         /* ---------- WAIT ACK ---------- */
	         else if(step == STEP_SEND_ACK_WAIT)
	         {
	             if(rxAvailable && (strstr(currentRx, "OK") || strstr(currentRx, "ERROR")))
	             {
	                 ackSending = 0;
	                 step = STEP_CONNECTED;
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 ackSending = 0;
	                 step = STEP_CONNECTED;
	             }
	         }

	         /* ---------- WAIT DATA ---------- */
	         else if(step == STEP_SEND_DATA_WAIT)
	         {
	             if(rxAvailable)
	             {
	                 // Clear state if we get an implicit confirmation or generic error response
	                 if(strstr(currentRx, "OK") || strstr(currentRx, "ERROR") || strstr(currentRx, "FAIL"))
	                 {
	                	 uint32_t responseTime =
	                	                     HAL_GetTick() - stepTimer;
	                     step = STEP_CONNECTED;
	                 }
	             }
	             else if(HAL_GetTick() - stepTimer > STEP_TIMEOUT_MS)
	             {
	                 step = STEP_CONNECTED;
	             }
	         }

	  osDelay(5);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */

/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
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
