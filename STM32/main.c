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

#include <stdio.h>

#include "uart.h"
#include "retarget.h"
#include "dma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t max_speed;
    uint32_t current_speed;

    _Bool direction;
	_Bool direction_plus;
	_Bool direction_minus;

    uint16_t direction_pin;
    GPIO_TypeDef* direction_port;

    TIM_HandleTypeDef* timer;
    uint32_t timer_channel;
	uint32_t frequency;

    uint16_t motor_pin;
    GPIO_TypeDef* motor_port;

    uint32_t max_position; //in numbers of steps
	uint32_t starting_position;
    uint32_t position;
	_Bool running;

    _Bool reset_requested;
    _Bool reset_completed;

    uint16_t end_switch_pin;
    GPIO_TypeDef* end_switch_port;

	uint32_t unit_conversion; //number of steps per mm or deg

	uint32_t num_steps_per_turn; //number of steps per rotation (360°)

	uint32_t offset; // offset due to mechanical structure (spindle joint)

}motor_struct_t;
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

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;  // Example timer handles - adjust based on which timers you want to use
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

MMC_HandleTypeDef hmmc1;

//SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
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

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

motor_struct_t motors[4]; // Declaration only

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
//static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RNG_Init(void);
static void CPU_CACHE_Enable(void);
void StartDefaultTask(void *argument);

static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void stall(uint32_t duration_us);
void stop_all_motors(void);
void direction_change(uint8_t motor_number, _Bool direction);
void reset_motors(void);
void move_to_starting_position(uint8_t motor_number);
_Bool move_effector(uint32_t x, uint32_t y, uint32_t orientation);
void update_global_coordinates(void);
_Bool read_switch(uint8_t motor_number);
void run_motor(uint8_t motor_number);
void stop_motor(uint8_t motor_number);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void TIM1_UP_IRQHandler(void);
void TIM15_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void Test_USART3(void);

void USART1_IRQHandler(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

void izpis_v_serijc(char *sporocilo);
void uart_transmit(char *sporocilo);
char uart_receive(char *sporocilo);
void process_message(char message);

void uart_process_command(const char* command);
void uart_send_motor_status(void);

static void MPU_Config(void);
void USART3_Pin_Init(void);

void configure_end_switch_interrupts(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//void USART_write(int ch);

//void UART_Write_String(char *p);

//uint32_t Read_ADC(ADC_HandleTypeDef* hadc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TS_Init_t *hTSs;

#define BUFFER_SIZE 30

char rx_buff[BUFFER_SIZE];
//char rcv_buff[30];

uint32_t timing_uart = 0;
uint32_t limit_uart = 5; //mej osveževanja

uint8_t uart_rx_buffer[BUFFER_SIZE];
uint8_t uart_rx_index = 0;

uint8_t podatki;
char vnos[100];

_Bool uart_command_ready = false;

uint8_t num_of_motors=3; //število vseh motorjev

uint32_t effector_x=0;   //end effector x coordinate [mm]
uint32_t effector_y=0;   //end effector y coordinate [mm]
uint32_t effector_orientation=0;   //end effector orientation [deg]

uint32_t min_effector_y=0;
uint32_t max_effector_y=0;

uint32_t min_effector_x=0;
uint32_t max_effector_x=0;

uint32_t min_effector_orientation=1;
uint32_t max_effector_orientation=179;

uint32_t J1_offset_mm=0;
uint32_t J2_offset_deg=0;
uint32_t J3_offset_mm=0;
uint32_t J3_offset_base=30; //popravi
uint32_t J4_ammount_of_liquid=0;
uint32_t J4_volume_per_turn=0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	uint32_t RNG_PTR[2];
	for(uint8_t i=0;i<30;i++)rx_buff[i]='\0';
	for(uint8_t i=0;i<30;i++)uart_rx_buffer[i]='\0';

	/* USER CODE BEGIN 1 */
	 CPU_CACHE_Enable();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	// Add right after HAL_Init():
	/*
	ADC1->CR |= ADC_CR_ADEN | ADC_CR_BOOST;  // 0x10010001
	while(!(ADC1->ISR & ADC_ISR_ADRDY));

	ADC2->CR |= ADC_CR_ADEN | ADC_CR_BOOST;  // 0x10010001
	while(!(ADC2->ISR & ADC_ISR_ADRDY));

	ADC3->CR |= ADC_CR_ADEN | ADC_CR_BOOST;  // 0x10010001
	while(!(ADC3->ISR & ADC_ISR_ADRDY));
	*/
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	PeriphCommonClock_Config();

	/*
	__HAL_RCC_ADC12_CLK_ENABLE();
	__HAL_RCC_ADC3_CLK_ENABLE();
	HAL_Delay(10);

	// Manually power up the ADC
	ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0; // Set to HCLK/1 (or other suitable divider)
	ADC1->CR &= ~ADC_CR_DEEPPWD; // Disable deep power down
	ADC1->CR |= ADC_CR_ADVREGEN; // Enable voltage regulator
	HAL_Delay(1); // Wait for regulator to stabilize
	*/

	/* Configure the peripherals common clocks */
	//PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */


	//GPIO initialization
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	//HAL_UART_Receive_IT(&huart3, rx_buff_usb, 10);
	MX_USART1_UART_Init();
	// Clear any pending interrupts
	//__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_TCF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

	// Enable RX interrupt in NVIC
	//HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	//HAL_NVIC_EnableIRQ(USART1_IRQn);

	// Enable RX interrupt in USART peripheral
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	// Start receiving - THIS IS CRITICAL!
	HAL_UART_Receive_IT(&huart1, rx_buff, 30);  // Receive 1 byte at a time
    //Debug_USART1_Config();

	//Timer initialization
	MX_TIM1_Init();
	MX_TIM15_Init();
	MX_TIM3_Init();
	MX_TIM12_Init(); //VRŽE ERROR!

	/* USER CODE BEGIN 2 */
	// freq=100000 -> hitro
	// steps=40000 -> en obrat

	motors[0] = (motor_struct_t){
	    .max_speed = 10000,
	    .current_speed = 0,
	    .direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
	    .direction_pin = GPIO_PIN_3,
	    .direction_port = GPIOG,//D2
	    .timer = &htim3,
	    .timer_channel = TIM_CHANNEL_1,
		.frequency = 50000,
	    .motor_pin = GPIO_PIN_6,//D3
	    .motor_port = GPIOA,
	    .max_position = 100000,
		.starting_position=5000,
	    .position = 0,
		.running = false,
	    .reset_requested = false,
	    .reset_completed = false,
	    .end_switch_pin = GPIO_PIN_3,//D8
	    .end_switch_port = GPIOE,
	    //.end_switch2_pin = GPIO_PIN_15,//D9
	    //.end_switch2_port = GPIOH,
		.unit_conversion=100, //steps per mm
		.num_steps_per_turn=40000
	};
	motors[1] = (motor_struct_t){
		.max_speed = 10000,
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.direction_pin = GPIO_PIN_1,
		.direction_port = GPIOK,//D4
		.timer = &htim1,
		.timer_channel = TIM_CHANNEL_1,
		.frequency = 50000,
		.motor_pin = GPIO_PIN_8,//D5
		.motor_port = GPIOA,
		.max_position = 100000,
		.starting_position=5000,
		.position = 0,
		.running = false,
		.reset_requested = false,
		.reset_completed = false,
		.end_switch_pin = GPIO_PIN_15,//D9
		.end_switch_port = GPIOH,
		//.end_switch1_pin = GPIO_PIN_4,//D10
		//.end_switch1_port = GPIOB,
	    //.end_switch2_pin = GPIO_PIN_2,//D12
	    //.end_switch2_port = GPIOI,
		.unit_conversion=100, //steps per deg
		.num_steps_per_turn=40000
	};
	motors[2] = (motor_struct_t){
		.max_speed = 10000,
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.direction_pin = GPIO_PIN_8,
		.direction_port = GPIOI,//D7
		.timer = &htim15,
		.timer_channel = TIM_CHANNEL_2,
		.frequency = 50000,
		.motor_pin = GPIO_PIN_6,//D6
		.motor_port = GPIOE,
		.max_position = 100000,
		.starting_position=5000,
		.position = 0,
		.running = false,
		.reset_requested = false,
		.reset_completed = false,
		.end_switch_pin = GPIO_PIN_4,//D10
		.end_switch_port = GPIOB,
		//.end_switch1_pin = GPIO_PIN_13,//D14
		//.end_switch1_port = GPIOD,
		//.end_switch2_pin = GPIO_PIN_3,//D13
		//.end_switch2_port = GPIOD,
		.unit_conversion=100, //steps per mm
		.num_steps_per_turn=40000
	};
	motors[3] = (motor_struct_t){ //max 50000 freq
		.max_speed = 10000,
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.direction_pin = GPIO_PIN_12,
		.direction_port = GPIOD,//D15
		.timer = &htim12,
		.timer_channel = TIM_CHANNEL_2,
		.frequency = 500,
		.motor_pin = GPIO_PIN_15,//D11
		.motor_port = GPIOB,
		.max_position = 10000,
		.starting_position=5000,
		.position = 0,
		.running = false,

		//NE UPORABLJAJ = IGNORIRAJ:
		.reset_requested = false,
		.reset_completed = false,
		.end_switch_pin = GPIO_PIN_2,//D12
		.end_switch_port = GPIOI,
		//konc prepovedi

		.unit_conversion=100, //steps per mm
		.num_steps_per_turn=200
	};

	//usable IO: I2
	configure_end_switch_interrupts();

	/* Configure LED1 */
	//BSP_LED_Init(LED1);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/*
	//Start the PWMs
	HAL_TIM_PWM_Start(motors[0].timer, motors[0].timer_channel);
	HAL_TIM_PWM_Start(motors[1].timer, motors[1].timer_channel);
	HAL_TIM_PWM_Start(motors[2].timer, motors[2].timer_channel);

	// Starts interrupts (for position increments in callback functions)
	HAL_TIM_Base_Start_IT(motors[0].timer);
	HAL_TIM_Base_Start_IT(motors[1].timer);
	HAL_TIM_Base_Start_IT(motors[2].timer);
	*/
	stop_all_motors();

	//reset_motors();

	_Bool values[6]={0,0,0,0,0,0};

	//run_motor(0);

	//run_motor(2);

	char text[]="Nika\r\n";

	//uart_transmit(text);


	while (1) {
		/* USER CODE END WHILE */

		/*
		for (uint8_t neki=0;neki<num_of_motors;neki++)
		{
			values[neki*2+0]=read_switch1(neki);
			values[neki*2+1]=read_switch2(neki);
			HAL_GPIO_WritePin(motors[neki].direction_port,motors[neki].direction_pin,GPIO_PIN_SET);
		}
		*/
		//stall(5);


		/*
		for (uint8_t i=3;i<4;i++)
		{
			direction_change(i,motors[i].direction_plus);
			run_motor(i);
			if ((motors[i].position)==(motors[i].num_steps_per_turn))
			{
				stop_motor(i);
				HAL_Delay(1000);
			}
			direction_change(i,motors[i].direction_minus);
			run_motor(i);
			if ((motors[i].position)==0)
			{
				stop_motor(i);
				HAL_Delay(1000);
			}
		}*/




		//char neki[30];

		//uart_receive(uart_rx_buffer);
		//HAL_Delay(1);
		//uart_empty_buffer(uart_rx_buffer,BUFFER_SIZE);

	    // Transmit data via USART3

	    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	    //HAL_Delay(500);
	    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	    //HAL_Delay(500);

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}
	// Add these after your clock configuration
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM15_CLK_ENABLE();
	__HAL_RCC_TIM12_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	//__HAL_RCC_ADC123_CLK_ENABLE();
	__HAL_RCC_ADC12_CLK_ENABLE();  // For ADC1 and ADC2
	__HAL_RCC_ADC3_CLK_ENABLE();    // For ADC3
	HAL_Delay(1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
	//		| RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI
	//		| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48| RCC_OSCILLATORTYPE_HSI; //
	//RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	//RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; //
	//RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	//RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;//
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

	  // Ensure proper USART clock - add this after clock configuration
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	  PeriphClkInit.Usart16ClockSelection  = RCC_USART16CLKSOURCE_D2PCLK2;  // Important!
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
	      Error_Handler();
	  }
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

	//ADC_MultiModeTypeDef multimode = { 0 };
	//hadc1.Init.MultiMode = ADC_MODE_INDEPENDENT;
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // Recommended for H7
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;        // H7 supports 16-bit
	hadc1.Init.ScanConvMode = DISABLE;
	//hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	//hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	//hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	//hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	//hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;     // Changed from PRESERVED
	//hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	//hadc1.Init.OversamplingMode = DISABLE;
    //hadc1.Init.DataAlign = ADC_DATA_ALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	//hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/*
	HAL_Delay(10);
	// For STM32H7, you may also need to calibrate:
	uint32_t tickstart = HAL_GetTick();
	    while(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
	        if((HAL_GetTick() - tickstart) > 1000) {
	            Error_Handler();
	        }
	    }
	 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	sConfig.Channel = ADC_CHANNEL_10;        // PC0
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  // H7 has different timing
	//sConfig.SingleDiff = ADC_SINGLE_ENDED;
	//sConfig.OffsetNumber = ADC_OFFSET_NONE;
	//sConfig.Offset = 0;
	//sConfig.OffsetSignedSaturation = DISABLE;

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
	//hadc2.Init.MultiMode = ADC_MODE_INDEPENDENT;
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // Recommended for H7
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;        // H7 supports 16-bit
	hadc2.Init.ScanConvMode = DISABLE;
	//hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	//hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	//hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	//hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	//hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	//hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;     // Changed from PRESERVED
	//hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	//hadc2.Init.OversamplingMode = DISABLE;

    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    //hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    //hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    //hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    //hadc2.Init.OversamplingMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;

	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}


	// For STM32H7, you may also need to calibrate:
	if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}


	sConfig.Channel = ADC_CHANNEL_6;        // PC0,7
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  // H7 has different timing
	//sConfig.SingleDiff = ADC_SINGLE_ENDED;
	//sConfig.OffsetNumber = ADC_OFFSET_NONE;
	//sConfig.Offset = 0;
	//sConfig.OffsetSignedSaturation = DISABLE;

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
	//hadc3.Init.MultiMode = ADC_MODE_INDEPENDENT;
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */

	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;  // Recommended for H7
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;        // H7 supports 16-bit
	hadc3.Init.ScanConvMode = DISABLE;
	//hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	//hadc3.Init.ContinuousConvMode = DISABLE;
	//hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	//hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	//hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	//hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;     // Changed from PRESERVED
	//hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	//hadc3.Init.OversamplingMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	//hadc3.Init.DMAContinuousRequests = DISABLE;
	//hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//hadc3.Init.LowPowerAutoWait = DISABLE;
	//hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	// For STM32H7, you may also need to calibrate:
	if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_0;        // PC0,0
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  // H7 has different timing
	//sConfig.SingleDiff = ADC_SINGLE_ENDED;
	//sConfig.OffsetNumber = ADC_OFFSET_NONE;
	//sConfig.Offset = 0;
	//sConfig.OffsetSignedSaturation = DISABLE;

	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 699;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;  // Adjust for your desired frequency
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;  // Initial duty cycle (50% for 999 period)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // Add this right before HAL_TIM_MspPostInit():
  //htim1.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt
  //htim1.Instance->CR1 |= TIM_CR1_CEN;    // Enable timer

  htim1.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

  HAL_TIM_MspPostInit(&htim1);

  // Configure NVIC
  //HAL_NVIC_SetPriority(TIM1_UP_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  //htim1.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt
}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 399;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;  // Adjust for your desired frequency
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;  // Initial duty cycle (50% for 999 period)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)  // Note: Using CH2
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  htim15.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt
  //HAL_TIM_RegisterCallback(motors[2].timer, HAL_TIM_PERIOD_ELAPSED_CB_ID, HAL_TIM_PeriodElapsedCallback);

  HAL_NVIC_SetPriority(TIM15_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM15_IRQn);

  HAL_TIM_MspPostInit(&htim15);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
	  Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  htim3.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 79;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
	  Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  //htim12.Instance->DIER |= TIM_DIER_UIE;  // Enable update interrupt


  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

  HAL_TIM_MspPostInit(&htim12);

  HAL_TIM_Base_Start_IT(&htim12);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
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

static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */
    //__HAL_RCC_USART1_CLK_ENABLE();
    //__HAL_RCC_GPIOB_CLK_ENABLE();
    //HAL_Delay(1);
	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	//huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};
*/
	//__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure PB6 = TX, PB7 = RX */
/*
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	//GPIO_InitStruct.Pin = VCP_TX_Pin | VCP_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  // Alternate function 7 = USART3

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);
*/
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);  // Set appropriate priority
    HAL_NVIC_EnableIRQ(USART1_IRQn);


	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
/*
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/*
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);  // Set appropriate priority
    HAL_NVIC_EnableIRQ(USART1_IRQn);          // Enable USART1 interrupt
    */


	/* USER CODE BEGIN USART1_Init 2 */


    // Clear any pending interrupts
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_TCF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    // Enable RX interrupt in the peripheral
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	/* USER CODE END USART1_Init 2 */

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
	/*
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	*/
	/*
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	 */

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

	// inicializacija D7,D4,D2
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    //konc

    // inicializacija digital inputov (expansion board)
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	*/
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // Interrupt on rising edge
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
     */

    /*
    GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Push-pull output
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
     */

	/*
    GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Push-pull output
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	*/
    /*
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	*/

	/*
    GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Push-pull output
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	*/
    /*
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	*/

    /*
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
     */

	/*
    GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Push-pull output
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	*/
    /*
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
     */

    //konc

	/*Configure GPIO pins : USB_OTG_FS2_ID_Pin OTG_FS2_PSO_Pin */
	//GPIO_InitStruct.Pin = USB_OTG_FS2_ID_Pin | OTG_FS2_PSO_Pin;
	//GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

	//timers:

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;//tim13
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PE6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//konc timers

	//usart:
    /* Configure USART1 TX (PB6) and RX (PB7) pins */

	//__HAL_RCC_USART1_CLK_ENABLE();
	/*
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
*/
	GPIO_InitStruct.Pin = GPIO_PIN_6;
    //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  // USART1 uses AF7
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  // USART1 uses AF7
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/*
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	*/
    //konc usart

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
	// Configure ADC input pins as analog inputs
	//GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*
	// A0 (PC0) - ADC1_IN10
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// A1 (PF8) - ADC2_IN6
	GPIO_InitStruct.Pin = GPIO_PIN_8;//8
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIOF->MODER |= (3 << (2*8));  // Analog mode
	GPIOF->PUPDR &= ~(3 << (2*8)); // No pull
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	// A2 (PA0_C) - ADC3_IN10
	GPIO_InitStruct.Pin = GPIO_PIN_0;//0
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIOA->MODER |= (3 << (2*0));
	GPIOA->PUPDR &= ~(3 << (2*0));
	SYSCFG->PMCR |= SYSCFG_PMCR_PA0SO; // Critical for PA0_C!
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	*/


	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
    /* TIM1 CH1 on PA8 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(timHandle->Instance==TIM3)
  {
    /* TIM3 CH1 on PA6 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(timHandle->Instance==TIM15)
  {
    /* TIM15 CH2 on PE6 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;  // TIM15_CH2 uses AF4
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if(timHandle->Instance==TIM12)
  {
    /* TIM12 CH2 on PB15 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;  // TIM12_CH2 uses AF2
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
  * @brief  Stalls the processor for the defined time period in us.
  * @param  duration_us: the time that the CPU should wait
  * @retval None
  */
void stall(uint32_t duration_us)
{
	uint32_t stall_limit;

	stall_limit=duration_us*10000/250;
	for(uint32_t stall=0;stall<stall_limit;stall++){}
}

/**
  * @brief  Stops all motors (by stopping their timers).
  * @param  None
  * @retval None
  */
void stop_all_motors(void)
{
	stop_motor(0);
	stop_motor(1);
	stop_motor(2);
	stop_motor(3);
}

/**
  * @brief  Changes the direction of the chosen motor to the desires direction
			and flags it in the struct.
  * @param  motor_number: number of the motor (starting with index 0).
  *	@param	direction: boolean value of the direction of rotation.
  * @retval None
  */
void direction_change(uint8_t motor_number, _Bool direction)
{
	motors[motor_number].direction=direction;
	HAL_GPIO_WritePin(motors[motor_number].direction_port, motors[motor_number].direction_pin, motors[motor_number].direction);
}

/**
  * @brief  Resets all the motors and configures max positions,
			then moves the motors to their starting positions.
  * @param  None
  * @retval None
  */
void reset_motors(void)
{
	//motor 4 - pumpa (preskočil - nima reset)
	//motor 3 - J3 (trapezoidal thread axle)
	//motor 2 - J2 (snail)
	//motor 1 - J1 (pulley)

	for (uint8_t motor_num=0;motor_num<(num_of_motors-1);motor_num++)
	{

		motors[motor_num].reset_requested=true; //da ignorira pogoje za pozicijo
		motors[motor_num].reset_completed=false;

		direction_change(motor_num, 0);
		motors[motor_num].current_speed=10; // rot/s - ni še; raje frekvenco!

		if (motors[motor_num].running==false)
		{
			run_motor(motor_num);
		}

		if(motors[motor_num].reset_requested)
		{
			while(!(read_switch(motor_num)))
			{} //trenutno je vseeno kateri switch zadane

			stop_motor(motor_num);

			motors[motor_num].position=0; //mogoče bi lahko offsetal start position da ne udari v limit switch

			motors[motor_num].direction_minus=motors[motor_num].direction;

			direction_change(motor_num,!motors[motor_num].direction);

			motors[motor_num].direction_plus=motors[motor_num].direction;

			motors[motor_num].reset_completed=true; //da lahko zdaj spremlja korake

			run_motor(motor_num);

			while(!(read_switch(motor_num)))
			{}

			motors[motor_num].max_position=motors[motor_num].position;
			motors[motor_num].starting_position=motors[motor_num].max_position/2;

			motors[motor_num].reset_completed=false;
			motors[motor_num].reset_requested=false;
		}

		stop_motor(motor_num);

		move_to_starting_position(motor_num);
	}
}

/**
  * @brief  Manipulates the chosen motor to move to the struct
			defined starting position.
  * @param  motor_number: number of the motor (starting with index 0),
			whose second limit switch should be read.
  * @retval None
  */
void move_to_starting_position(uint8_t motor_number)
{
	if (motors[motor_number].position>motors[motor_number].starting_position)
		{
			direction_change(motor_number,motors[motor_number].direction_minus);
		}
		else if (motors[motor_number].position<motors[motor_number].starting_position)
		{
			direction_change(motor_number,motors[motor_number].direction_plus);
		}

	while(motors[motor_number].position!=motors[motor_number].starting_position)
	{
		if (motors[motor_number].running==false)
		{
			run_motor(motor_number);
		}
	}

}

/**
  * @brief  Moves the end effector (needle) to the given coordinates,
			taking in the account the desired orientation.
  *	@param	x: x coordinate of end effector position.
  * @param  y: x coordinate of end effector position.
  * @param  orientation: orientation (in degrees) with 0° being
			parralel to the pulley rail.s
  * @retval None
  */
bool move_effector(uint32_t target_x, uint32_t target_y, uint32_t target_orientation)
{
	stop_all_motors();

	//ce neke tocke ni mozno doseci pod doloceno orientacijo
	if (target_y<(motors[2].offset*sin(180-target_orientation))) //DOPOLNI!
	{	//PRENIZKO ZA TO ORIENTACIJO
		return false;
	}
	if (target_y>max_effector_y || target_x>max_effector_x)
	{
		//OUT OF BOUNDS
		return false;
	}

	if(target_orientation!=effector_orientation)
	{
		if(target_orientation>effector_orientation)
		{
			motors[1].direction=motors[1].direction_plus;
			start_motor(1);
		}
		else
		{
			motors[1].direction=motors[1].direction_minus;
			start_motor(1);
		}
	}

	if(target_x!=effector_x || target_y!=effector_y)
	{
		uint32_t cart_x_position_mm=motors[0].position;
		//x:
		if (target_x<(J1_offset_mm+J3_offset_mm*cos(180-target_orientation)))
		{
			motors[0].direction=motors[0].direction_minus;
			start_motor(0);
		}
		else if (target_x>(J1_offset_mm+J3_offset_mm*cos(180-target_orientation)))
		{
			motors[0].direction=motors[0].direction_plus;
			start_motor(0);
		}

		//y:
		if (target_y<(motors[2].offset*sin(180-J2_offset_deg)))
		{
			uint8_t neki=0;
		}
	}

}

/**
  * @brief  Updates the global end effector coordinate from the known struct
			data of the motors.
  * @param  None
  * @retval None
  */
void update_global_coordinates(void)
{		//PREVERI DELJENJE CELIH ŠTEVIL

	if (min_effector_x==0) //določanje skrajno leve točke, opcija je da bi bilo bolj levo od motorja
		min_effector_x=1;
	if (max_effector_x==0)
		max_effector_x=motors[0].position/motors[0].unit_conversion;


	//drugi index je hipotenuza (izteg); pri hipotenuzi je treba upostevat default dolzino
	effector_x=motors[0].position/motors[0].unit_conversion+(motors[2].position/motors[2].unit_conversion+motors[2].offset)*cos(motors[1].position/motors[1].unit_conversion);
	effector_y=(motors[2].position/motors[2].unit_conversion+motors[2].offset)*sin(motors[1].position/motors[1].unit_conversion);
	effector_orientation=motors[1].position/motors[1].unit_conversion;
}

/**
  * @brief  Reads the state of either the limit switch, of the chosen motor.
  * @param  motor_number: number of the motor (starting with index 0),
			whose first limit switch should be read.
  * @retval The state of the first limit switch.
  */
_Bool read_switch(uint8_t motor_number)
{
	return HAL_GPIO_ReadPin(motors[motor_number].end_switch_port, motors[motor_number].end_switch_pin);
}


/**
  * @brief  Runs the chosen motor with the struct defined speed/frequency.
  * @param  motor_number: the number of the motor that should be ran (starting with index 0)
  * @retval None
  */
void run_motor(uint8_t motor_number)
{
    // Stop PWM first
    //HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);
	//motors[motor_number].running=false;

	if (motors[motor_number].timer->State == HAL_TIM_STATE_RESET) {
	    // Timer not initialized, initialize it first
	    // You might need to call the specific MX_TIMx_Init function
	    return;
	}

	if (motors[motor_number].running == false)
	{
		// Stop timer first
		//HAL_TIM_Base_Stop_IT(motors[motor_number].timer);
		htim15.Instance->DIER &= ~(1);
		HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);

		uint32_t frequency_hz = motors[motor_number].frequency;
		frequency_hz = frequency_hz / 2;

		// Calculate prescaler and period
		uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
		if(motors[motor_number].timer->Instance == TIM1 ||
		   motors[motor_number].timer->Instance == TIM8 ||
		   motors[motor_number].timer->Instance == TIM12 ||
		   motors[motor_number].timer->Instance == TIM15 ||
		   motors[motor_number].timer->Instance == TIM16 ||
		   motors[motor_number].timer->Instance == TIM17) {
			timer_clock = HAL_RCC_GetPCLK2Freq();
		}

		uint32_t prescaler = (timer_clock / (frequency_hz * 1000)) - 1;
		uint32_t period = 999;

		// Configure timer registers
		motors[motor_number].timer->Instance->PSC = prescaler;
		motors[motor_number].timer->Instance->ARR = period;

		// Clear update flag
		motors[motor_number].timer->Instance->SR &= ~TIM_SR_UIF;

		// Restart timer
		//HAL_TIM_Base_Start_IT(motors[motor_number].timer);
		htim15.Instance->DIER |= TIM_DIER_UIE;
		motors[motor_number].timer->Instance->CR1 |= TIM_CR1_CEN;
		HAL_TIM_PWM_Start(motors[motor_number].timer, motors[motor_number].timer_channel);

		motors[motor_number].running = true;
	}

}

/**
  * @brief  Stops the chosen motor.
  * @param  motor_number: the number of the motor (starting with 0)
			that should be stopped.
  * @retval None
  */
void stop_motor(uint8_t motor_number)
{
	HAL_TIM_Base_Stop_IT(motors[motor_number].timer);
	HAL_TIM_Base_Stop(motors[motor_number].timer);
	HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);
	motors[motor_number].running=false;
}

/**
  * @brief  Runs the pump motor to squeeze out a specific ammount of liquid.
  * @param  ammount_of_liquid: the ammount of liquid (in mL) that should be
  				squeezed out of the needle.
  * @retval None
  */
void pump_liquid(uint32_t ammount_of_liquid)
{
	motors[3].position=0; //set the motor to track the ammount of liquid pumped out
	motors[3].direction=motors[3].direction_plus;
	J4_ammount_of_liquid=0;
	run_motor(3);
	while(J4_ammount_of_liquid!=ammount_of_liquid){}
	stop_motor(3);
}

/**
  * @brief  Timer interrupt function for incrementing the position
			of the stepper motor.
  * @param  htim: pointer to the designated timer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
    	//if (false)
		if ((((motors[0].position==motors[0].max_position && motors[0].direction==motors[0].direction_plus) || (motors[0].position==0 && motors[0].direction==motors[0].direction_minus)) && !motors[0].reset_requested ) && motors[0].running==true)
		{
			stop_motor(0);
			motors[0].running=false;
		}
		else if (motors[0].running=true && motors[0].direction==motors[0].direction_plus && (motors[0].reset_completed || motors[0].position<motors[0].max_position))
		{
			motors[0].position += 1;
			J1_offset_mm=motors[0].position*motors[0].unit_conversion/motors[0].num_steps_per_turn;
		}
		else if (motors[0].running=true && motors[0].direction==motors[0].direction_minus && (motors[0].reset_completed || motors[0].position>0))
		{
			motors[0].position -= 1;
			J1_offset_mm=motors[0].position*motors[0].unit_conversion/motors[0].num_steps_per_turn;
		}
	}
    else if (htim->Instance == TIM1)
    {
		if ((((motors[1].position==motors[1].max_position && motors[1].direction==motors[1].direction_plus) || (motors[1].position==0 && motors[1].direction==motors[1].direction_minus)) && !motors[1].reset_requested ) && motors[1].running==true)
		{
			stop_motor(1);
			motors[1].running=false;
		}
		else if (motors[1].running=true && motors[1].direction==motors[1].direction_plus && (motors[1].reset_completed || motors[1].position<motors[1].max_position))
		{
			motors[1].position += 1;
			//J2_offset_deg=motors[1].position*motors[1].unit_conversion/motors[1].num_steps_per_turn;
		}
		else if (motors[1].running=true && motors[1].direction==motors[1].direction_minus && (motors[1].reset_completed || motors[1].position>0))
		{
			motors[1].position -= 1;
			J2_offset_deg=motors[1].position*motors[1].unit_conversion/motors[1].num_steps_per_turn;
		}
    }
    else if (htim->Instance == TIM15)
    {
		if ((((motors[2].position==motors[2].max_position && motors[2].direction==motors[2].direction_plus) || (motors[2].position==0 && motors[2].direction==motors[2].direction_minus)) && !motors[2].reset_requested ) && motors[2].running==true)
		{
			stop_motor(2);
			motors[2].running=false;
		}
		else if (motors[2].running=true && motors[2].direction==motors[2].direction_plus && (motors[2].reset_completed || motors[2].position<motors[2].max_position))
		{
			motors[2].position += 1;
			J3_offset_mm=J3_offset_base+motors[2].position*motors[2].unit_conversion/motors[2].num_steps_per_turn;
		}
		else if (motors[2].running=true && motors[2].direction==motors[2].direction_minus && (motors[2].reset_completed || motors[2].position>0))
		{
			motors[2].position -= 1;
			J3_offset_mm=J3_offset_base+motors[2].position*motors[2].unit_conversion/motors[2].num_steps_per_turn;
		}
    }
    else if (htim->Instance == TIM12)
    {

		if (motors[3].running=true && motors[3].direction==motors[3].direction_plus)
		{
			motors[3].position += 1;
			//J4_ammount_of_liquid=J4_volume_per_turn*motors[3].position/motors[3].num_steps_per_turn;
		}
		else if (motors[3].running=true && motors[3].direction==motors[3].direction_minus)
		{
			motors[3].position -= 1;
		}
    }
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */
void TIM1_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */
void TIM15_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim15);
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim12);
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  // Disable the MPU
  HAL_MPU_Disable();

  // Configure the MPU attributes as Device Non Cacheable for SRAM1
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Enable the MPU
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

uint32_t Read_ADC2(ADC_HandleTypeDef* hadc) {
	    // Stop any ongoing conversion
	    HAL_ADC_Stop(hadc);

	    // Clear all flags
	    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_AWD1 |
	                          ADC_FLAG_AWD2 | ADC_FLAG_AWD3 | ADC_FLAG_JQOVF);

	    // Start conversion with timeout
	    uint32_t tickstart = HAL_GetTick();
	    while (HAL_ADC_Start(hadc) != HAL_OK) {
	        if ((HAL_GetTick() - tickstart) > 10) return 0xFFFF;
	    }

	    // Wait for conversion with timeout
	    tickstart = HAL_GetTick();
	    while (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
	        if ((HAL_GetTick() - tickstart) > 10) return 0xFFFF;
	    }

	    uint32_t value = HAL_ADC_GetValue(hadc);
	    HAL_ADC_Stop(hadc);

	    return value;
	}

uint32_t Read_ADC(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
}

// Diagnostic test - bypass all your existing ADC code
void TestADCs() {
    // 1. Enable clocks and power
    //RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN | RCC_AHB1ENR_ADC3EN;
    __DSB();

    // 2. Power up regulators
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC3->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC2->CR |= ADC_CR_ADVREGEN;
    ADC3->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(1);

    // 3. Calibrate (optional but recommended)
    if(ADC1->CR & ADC_CR_ADEN) ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADCAL; while(ADC1->CR & ADC_CR_ADCAL);

    // 4. Enable with boost for H7 high-speed operation
    ADC1->CR |= ADC_CR_ADEN | ADC_CR_BOOST;
    ADC2->CR |= ADC_CR_ADEN | ADC_CR_BOOST;
    ADC3->CR |= ADC_CR_ADEN | ADC_CR_BOOST;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while(!(ADC2->ISR & ADC_ISR_ADRDY));
    while(!(ADC3->ISR & ADC_ISR_ADRDY));

    // 5. Configure channels
    ADC1->SQR1 = (10 << 6) | (0 << 0);  // Ch10, 1 conversion
    ADC2->SQR1 = (6 << 6) | (0 << 0);   // Ch6, 1 conversion
    ADC3->SQR1 = (0 << 6) | (0 << 0);   // Ch0, 1 conversion

    // 6. Sample time configuration
    ADC1->SMPR1 = 7 << (3*0);  // 810.5 cycles for Ch0-9
    ADC2->SMPR1 = 7 << (3*0);
    ADC3->SMPR1 = 7 << (3*0);
}

void uart_process_command(const char* command) {
    char response[128];

    if (strncmp(command, "STATUS", 6) == 0) {
    	uart_send_motor_status();
    }
    else if (strncmp(command, "STOP", 4) == 0) {
        stop_all_motors();
        uart_transmit("OK:All motors stopped\r\n");
    }
    else if (strncmp(command, "START", 5) == 0) {
        // Parse motor number: START 0, START 1, etc.
        int motor_num = command[6] - '0';
        if (motor_num >= 0 && motor_num < 4) {
            run_motor(motor_num);
            snprintf(response, sizeof(response), "OK:Motor %d started\r\n", motor_num);
            uart_transmit(response);
        }
    }
    else if (strncmp(command, "MOVE", 4) == 0) {
        // Example: MOVE 0 50000 - move motor 0 to position 50000
        int motor_num, position;
        if (sscanf(command, "MOVE %d %d", &motor_num, &position) == 2) {
            if (motor_num >= 0 && motor_num < 4) {
                // Add your move logic here
                snprintf(response, sizeof(response), "OK:Moving motor %d to %d\r\n", motor_num, position);
                uart_transmit(response);
            }
        }
    }
    else if (strncmp(command, "RESET", 5) == 0) {
        reset_motors();
        UART_Send_Data("OK:Motors reset\r\n");
    }
    else {
        snprintf(response, sizeof(response), "ERROR:Unknown command: %s\r\n", command);
        uart_transmit(response);
    }
}

void uart_send_motor_status(void) {
    char status[256];
    snprintf(status, sizeof(status),
             "Motor Status:\r\n"
             "M0: Pos=%lu, Running=%d, Dir=%d\r\n"
             "M1: Pos=%lu, Running=%d, Dir=%d\r\n"
             "M2: Pos=%lu, Running=%d, Dir=%d\r\n"
             "M3: Pos=%lu, Running=%d, Dir=%d\r\n",
             motors[0].position, motors[0].running, motors[0].direction,
             motors[1].position, motors[1].running, motors[1].direction,
             motors[2].position, motors[2].running, motors[2].direction,
             motors[3].position, motors[3].running, motors[3].direction);
    uart_transmit(status);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(&huart1, rx_buff, 30);
}

void izpis_v_serijc(char *sporocilo)
{
	uint8_t X = 0;
	sprintf(sporocilo,sporocilo,X);
	HAL_UART_Transmit(&huart3, sporocilo, sizeof(sporocilo), 100);
}

void uart_transmit(char *sporocilo)
{
	//za preset sporocil:
	//char buffer[40]={'0'};
	//sprintf(buffer,"my variable is %s \r\n",sporocilo);

	HAL_UART_Transmit_IT(&huart1, sporocilo,strlen(sporocilo)-1);
	HAL_Delay(1);
}

char uart_receive(char *beseda)
{
	static char znak='\0'; 					//aktiven prebran znak
	static char znak_temp;

	//char beseda[30];
	for(uint8_t i=0;i<30;i++)beseda[i]='\0';
	uint8_t beseda_write_pos=0;

	static _Bool first_read=0;

	//timing_uart=HAL_GetTick();

	while(znak!='\n')
	{
		//if((HAL_GetTick()-timing_uart)>=limit_uart)
		//{
			timing_uart=HAL_GetTick();
			static uint8_t uart_read_pos=0;

			uint8_t read_state=0; 				//je 1 če smo prebrali nekaj novega, po defaultu 0 vsak loop

			if(uart_read_pos==30)uart_read_pos=0;

			if(first_read!=1 && rx_buff[0]!='\0')//prvo branje opravimo ko je prvi element različen od '\0', da ne listamo po nepotrebnem
			{
				first_read=1;
				//znak_temp=znak;
				//znak=rx_buff[uart_read_pos];
				//rx_buff[uart_read_pos]='\0'; //po branju zapišemo '\0' kot oznako da smo prebrali
				//uart_read_pos++;
				read_state=1;
			}

			else if(first_read==1 && rx_buff[uart_read_pos]!='\0') //če smo primer prvega branja opravili in če je kaj za prebrat
			{
				znak_temp=znak;
				znak=rx_buff[uart_read_pos];
				rx_buff[uart_read_pos]='\0'; //po branju zapišemo '\0' kot oznako da smo prebrali
				beseda[beseda_write_pos]=znak;
				beseda_write_pos++;
				uart_read_pos++;
				read_state=1;
			}
		//}
	}
	first_read=0;
	//return *beseda;
}

void uart_empty_buffer(char *buffer, uint8_t buffer_size)
{
	for(uint8_t buff_ptr=0;buff_ptr<buffer_size;buff_ptr++)
	{
		buffer[buff_ptr]='\0';
	}
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  // Enable USART3 clock
	  __HAL_RCC_USART3_CLK_ENABLE();
	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	/*
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

	*/
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  huart3.Instance = USART3;
	  huart3.Init.BaudRate = 115200;
	  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	  huart3.Init.StopBits = UART_STOPBITS_1;
	  huart3.Init.Parity = UART_PARITY_NONE;
	  huart3.Init.Mode = UART_MODE_TX_RX;
	  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

	  if (HAL_UART_Init(&huart3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

void configure_end_switch_interrupts(void)
{

		GPIO_InitTypeDef GPIO_InitStruct = {0};

	    // Enable SYSCFG clock
	    __HAL_RCC_SYSCFG_CLK_ENABLE();

	    // First, configure all pins as simple inputs
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	    GPIO_InitStruct.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); // PE3
	    GPIO_InitStruct.Pin = GPIO_PIN_15; HAL_GPIO_Init(GPIOH, &GPIO_InitStruct); // PH15
	    GPIO_InitStruct.Pin = GPIO_PIN_4;  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // PB4


	    // Now manually configure EXTI for each pin

	    // PE3 - EXTI3 (Motor 0 Switch 1)
	    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_Msk;
	    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PE;

	    // PH15 - EXTI15 (Motor 0 Switch 2)
	    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI15_Msk;
	    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PH;

	    // PB4 - EXTI4 (Motor 1 Switch 1)
	    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4_Msk;
	    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;


	    // Enable rising edge trigger for ALL lines
	    EXTI->RTSR1 |= (1 << 3) | (1 << 4) | (1 << 15);

	    // Enable interrupt mask for ALL lines
	    EXTI->IMR1 |= (1 << 3) | (1 << 4) | (1 << 15);

	    // Clear any pending interrupts
	    EXTI->PR1 = (1 << 3) | (1 << 4) | (1 << 15);


	    // Clear any pending interrupts
		//__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13 | GPIO_PIN_3 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_2);


	    HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
	    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	    HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
	    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


		EXTI->PR1 = (1 << 3) | (1 << 4) | (1 << 15);

}

void EXTI3_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
    }
}

void EXTI4_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_15);
    }
}

// Enhanced callback with proper shared line handling
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Debouncing - ignore interrupts within 50ms
    if(current_time - last_interrupt_time > 50) {
        last_interrupt_time = current_time;

        switch(GPIO_Pin) {
            case GPIO_PIN_3://motors[0].end_switch_pin:
				// Motor 0 Switch 1 (PE3)
				stop_motor(0);
				if (motors[0].direction=motors[0].direction_plus)
				{
					motors[0].position = motors[0].max_position;
				}
				else if (motors[0].direction=motors[0].direction_minus)
				{
					motors[0].position = 0;
				}
				motors[0].running = false;
				uart_transmit("M0: Switch (PE3) - STOPPED\r\n");
                break;

            case GPIO_PIN_15://motors[1].end_switch_pin:
                // Motor 0 (PH15)
                stop_motor(1);
            	if (motors[1].direction=motors[1].direction_plus)
				{
					motors[1].position = motors[1].max_position;
				}
				else if (motors[1].direction=motors[1].direction_minus)
				{
					motors[1].position = 0;
				}
				motors[1].running = false;
                uart_transmit("M1: Switch (PH15) - STOPPED\r\n");
                break;

            case GPIO_PIN_4://motors[2].end_switch_pin:
                // Motor 1 (PB4)
                stop_motor(2);
                if (motors[2].direction=motors[2].direction_plus)
                {
                	motors[2].position = motors[2].max_position;
                }
                else if (motors[2].direction=motors[2].direction_minus)
				{
					motors[2].position = 0;
				}
                motors[2].running = false;
                uart_transmit("M2: Switch (PB4) - STOPPED\r\n");
                break;

            default:
                // Unknown pin - this shouldn't happen
                char msg[50];
                snprintf(msg, sizeof(msg), "Unknown GPIO: %d\r\n", GPIO_Pin);
                uart_transmit(msg);
                break;
        }
    }
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


