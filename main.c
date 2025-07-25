/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"//

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stlogo.h"
#include "app_timers.h"

//#include "stm32_lcd.h"

#include "uart.h"
#include "retarget.h"
#include "dma.h"

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000080
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000080))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

//ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
//ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
#endif

//ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

//ETH_HandleTypeDef heth;

//FDCAN_HandleTypeDef hfdcan1;
//FDCAN_HandleTypeDef hfdcan2;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

//RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;

TIM_HandleTypeDef TIM3Handle;

MMC_HandleTypeDef hmmc1;

//SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

//PCD_HandleTypeDef hpcd_USB_OTG_FS;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
__IO uint32_t ButtonState = 0;

uint16_t timer_val_start, timer_val_end;
uint16_t elapsed_1st, elapsed_2nd, elapsed_3rd;

uint8_t time_str1[60];
uint8_t time_str2[40];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ETH_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_RTC_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_MMC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RNG_Init(void);
static void CPU_CACHE_Enable(void);
void StartDefaultTask(void *argument);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TS_Init_t *hTSs;

char rx_buff[10];

uint8_t podatki;
uint32_t pozicija_za_char = 0;
uint32_t pozicija_za_konec = 0;
char vnos[100];
uint32_t y_size_global;

uint32_t HIGHSCORE = 0;
uint32_t timing_uart = 0;
uint32_t limit_uart = 5; //mej osveževanja
uint32_t timing_display = 0;
uint32_t limit_display = 2;
uint32_t timing_fall = 0;
const uint32_t slow_limit_fall = 500;
const uint32_t fast_limit_fall = 100;
uint32_t limit_fall = slow_limit_fall;

uint8_t rotation_change = 0;
uint8_t new_block = 1;

uint8_t cbc[7];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	uint32_t RNG_PTR[2];
	for(uint8_t i=0;i<10;i++)rx_buff[i]='\0';

	/* USER CODE BEGIN 1 */
	 CPU_CACHE_Enable();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	//PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	MX_USART3_UART_Init(); //inicializiramo UART
	MX_RNG_Init(); 		  //inicializiramo RNG
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */

	//inicializiramo UART interrupt, rx_buff je dolg 10 znakov
	HAL_UART_Receive_IT(&huart3, rx_buff, 10);

	/* Configure LED1 */
	BSP_LED_Init(LED1);

	/*##-1- LCD Initialization #################################################*/
	/* Initialize the LCD */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//step pin:
	//direction pin:
	//mf pin:
	while (1) {
		/* USER CODE END WHILE */

		static uint8_t smer=0;
		static uint8_t smer_change=0;
		static uint8_t korak=0;
		static uint32_t stevilo_korakov=0; // od zacetka

		if (stevilo_korakov<5000 && korak==0)
		{
			korak=1;
			smer=1;
			HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_2, KeyState);
			HAL_delay(1); //med zamenjavo smeri in pulzom mora biti vec kot 125us
		}
		else if (stevilo_korakov==0 && korak==1)
		{
			korak=0;
			smer=0;
			HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_2, KeyState);
			HAL_delay(1); //med zamenjavo smeri in pulzom mora biti vec kot 125us
		}


		//spremeni za toggle step pina
	    HAL_GPIO_TogglePin (GPIOJ, GPIO_PIN_2);
	    //HAL_Delay (500);

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}


static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(200);
    //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    BSP_LED_On(LED_GREEN);
    osDelay(200);
    //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    BSP_LED_Off(LED_GREEN);
  }
  /* USER CODE END 5 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
	//		| RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI
	//		| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48; //
	//RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	//RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; //
	//RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	//RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;//
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;//
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	//RCC_OscInitStruct.PLL.PLLM = 22;
	RCC_OscInitStruct.PLL.PLLM = 5;
	//RCC_OscInitStruct.PLL.PLLN = 169;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	//RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
/*	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
*/

	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
	                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
	//if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	  /*activate CSI clock mondatory for I/O Compensation Cell*/
	  __HAL_RCC_CSI_ENABLE() ;

	  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
	  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

	  /* Enables the I/O Compensation Cell */
	  HAL_EnableCompensationCell();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 5;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_16B;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
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
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_16B;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */

	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc3.Init.OversamplingMode = DISABLE;
	hadc3.Init.Oversampling.Ratio = 1;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	hadc3.Init.Resolution = ADC_RESOLUTION_16B;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}






/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */

static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };
	LTDC_LayerCfgTypeDef pLayerCfg1 = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 7;
	hltdc.Init.VerticalSync = 3;
	hltdc.Init.AccumulatedHBP = 14;
	hltdc.Init.AccumulatedVBP = 5;
	hltdc.Init.AccumulatedActiveW = 654;
	hltdc.Init.AccumulatedActiveH = 485;
	hltdc.Init.TotalWidth = 660;
	hltdc.Init.TotalHeigh = 487;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 0;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 0;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg.Alpha = 0;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg.FBStartAdress = 0;
	pLayerCfg.ImageWidth = 0;
	pLayerCfg.ImageHeight = 0;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg1.WindowX0 = 0;
	pLayerCfg1.WindowX1 = 0;
	pLayerCfg1.WindowY0 = 0;
	pLayerCfg1.WindowY1 = 0;
	pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg1.Alpha = 0;
	pLayerCfg1.Alpha0 = 0;
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg1.FBStartAdress = 0;
	pLayerCfg1.ImageWidth = 0;
	pLayerCfg1.ImageHeight = 0;
	pLayerCfg1.Backcolor.Blue = 0;
	pLayerCfg1.Backcolor.Green = 0;
	pLayerCfg1.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void) {

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	hqspi.Init.FlashID = QSPI_FLASH_ID_1;
	hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void) {

	/* USER CODE BEGIN RNG_Init 0 */

	/* USER CODE END RNG_Init 0 */

	/* USER CODE BEGIN RNG_Init 1 */

	/* USER CODE END RNG_Init 1 */
	hrng.Instance = RNG;
	hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
	if (HAL_RNG_Init(&hrng) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RNG_Init 2 */

	/* USER CODE END RNG_Init 2 */

}



/**
 * @brief SAI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SAI2_Init(void) {

	/* USER CODE BEGIN SAI2_Init 0 */

	/* USER CODE END SAI2_Init 0 */

	/* USER CODE BEGIN SAI2_Init 1 */

	/* USER CODE END SAI2_Init 1 */
	hsai_BlockA2.Instance = SAI2_Block_A;
	hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
	hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
	hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockA2.Init.PdmInit.Activation = DISABLE;
	hsai_BlockA2.Init.PdmInit.MicPairsNbr = 1;
	hsai_BlockA2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockA2.FrameInit.FrameLength = 8;
	hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
	hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockA2.SlotInit.FirstBitOffset = 0;
	hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockA2.SlotInit.SlotNumber = 1;
	hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
	if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK) {
		Error_Handler();
	}
	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
	hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
	hsai_BlockB2.Init.PdmInit.MicPairsNbr = 1;
	hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockB2.FrameInit.FrameLength = 8;
	hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
	hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockB2.SlotInit.FirstBitOffset = 0;
	hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockB2.SlotInit.SlotNumber = 1;
	hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
	if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SAI2_Init 2 */

	/* USER CODE END SAI2_Init 2 */

}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_MMC_Init(void) {

	/* USER CODE BEGIN SDMMC1_Init 0 */

	/* USER CODE END SDMMC1_Init 0 */

	/* USER CODE BEGIN SDMMC1_Init 1 */

	/* USER CODE END SDMMC1_Init 1 */
	hmmc1.Instance = SDMMC1;
	hmmc1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hmmc1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hmmc1.Init.BusWide = SDMMC_BUS_WIDE_8B;
	hmmc1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hmmc1.Init.ClockDiv = 0;
	if (HAL_MMC_Init(&hmmc1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SDMMC1_Init 2 */

	/* USER CODE END SDMMC1_Init 2 */

}



/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
//static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	//hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	//hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
	//hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	//hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	//hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	//hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	//hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	//hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	//hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
	//hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	//hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	//if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
	//	Error_Handler();
	//}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

//}

/* FMC initialization function */
static void MX_FMC_Init(void) {

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = { 0 };

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 16;
	SdramTiming.ExitSelfRefreshDelay = 16;
	SdramTiming.SelfRefreshTime = 16;
	SdramTiming.RowCycleDelay = 16;
	SdramTiming.WriteRecoveryTime = 16;
	SdramTiming.RPDelay = 16;
	SdramTiming.RCDDelay = 16;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, MII_TX_ER_nINT_Pin | LCD_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PH15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_DISPD7_Pin */
	GPIO_InitStruct.Pin = LCD_DISPD7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_DISPD7_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PE5 PE4 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI4;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS2_ID_Pin OTG_FS2_PSO_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS2_ID_Pin | OTG_FS2_PSO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : audio_Int_Pin */
	GPIO_InitStruct.Pin = audio_Int_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
	HAL_GPIO_Init(audio_Int_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_INT_Pin */
	GPIO_InitStruct.Pin = LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS2_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS2_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS2_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MII_TX_ER_nINT_Pin LCD_RST_Pin */
	GPIO_InitStruct.Pin = MII_TX_ER_nINT_Pin | LCD_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LD1_Pin */
	GPIO_InitStruct.Pin = LD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart3, rx_buff, 10); //You need to toggle a breakpoint on this line!
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
