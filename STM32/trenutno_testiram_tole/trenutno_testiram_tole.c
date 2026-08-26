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
	uint8_t allowed_direction;//2=any, sicer je limited=+/-

	uint32_t num_steps_for_switch_release;

    uint16_t direction_pin;
    GPIO_TypeDef* direction_port;

    TIM_HandleTypeDef* timer;
    uint32_t timer_channel;
	uint32_t frequency;

    uint16_t motor_pin;
    GPIO_TypeDef* motor_port;

    int max_position; //in numbers of steps
    int starting_position;
    int position;
    int target_position;
    int home_position;
    int offset;
	_Bool running;

    _Bool reset_requested;
    _Bool reset_completed;

    uint16_t end_switch_pin;
    GPIO_TypeDef* end_switch_port;
	_Bool end_switch_triggered;

	int unit_conversion; //number of step per mm or deg
	int travel_length; //maximum travel length distance of segment

	uint32_t num_steps_per_turn; //number of steps per rotation (360°)

	/*uint32_t num_turns_from_encoder;

	//encoder only one cable per channel due to lack of pins

	uint32_t encoder_A_state;
	uint16_t encoder_A_pin;
    GPIO_TypeDef* encoder_A_port;
	uint32_t encoder_B_state;
	uint16_t encoder_B_pin;
    GPIO_TypeDef* encoder_B_port;
	_Bool encoder_Z_state;
	uint16_t encoder_Z_pin;
    GPIO_TypeDef* encoder_Z_port;
    */

}motor_struct_t;

typedef struct {
    uint8_t pin_number;      // A0, A1, etc.
    uint32_t adc_channel;    // ADC channel number
    ADC_HandleTypeDef* hadc; // ADC handle to use
    uint32_t gpio_pin;       // GPIO pin
    GPIO_TypeDef* gpio_port; // GPIO port
    char* name;              // Pin name for display
} analog_pin_config_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L0X_ADDR 0x52  // 8-bitni naslov (0x29 << 1)
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG 0x01
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR 0x0B

/* ---- DC motor PWM controller (distance-regulated) ---- */

//podatki za tof pozicijo bolj nazaj
//do konca porinjena igla
//#define DC_DIST_MIN_MM      88U //83+35     /* below this: stop (unless reversing out) */

//#define DOLZINA_IGLE 20U
//#define MAX_IZTEG 71U

//pr tej razdalji se igla dotika debla
//#define DC_DIST_MAX_MM      108U//113+50=max reach//130U    /* above this: stop (unless forwarding back) */

//#define TOF_OFFSET 5U

/*
 * TOF senzor razdalje:
 * Od TOF do sredine konusa = 4.84+72.51+11 = 88.35 mm
 * Od TOF do konc igle = 4.84+72.51+30.61 = 107.96 mm
 * dolzina igle = 20 mm
 * max izteg igle = 71 mm
 */
/*
 * tof test:
 * les od konca bloka 31.5 mm (čis pr igli skor)
 * tof reading ~107 mm
 */
//----------------------------------------

//podatki za tof pozicijo bolj naprej
//do konca porinjena igla
#define DC_DIST_MIN_MM      66U//v resnici:67U     /* below this: stop (unless reversing out) */

#define DOLZINA_IGLE 19U//od sredine konusa
#define MAX_IZTEG 57U//124-86+19=57U//v resnici: 71U

//pr tej razdalji se igla dotika debla
#define DC_DIST_MAX_MM      87U   /* above this: stop (unless forwarding back) */

//#define TOF_OFFSET 5U

/*
 * TOF senzor razdalje:
 * Od TOF do sredine konusa = 4.32+50.25+11 = 65.57 mm
 * Od TOF do konc igle = 4.32+50.25+11+19 = 84.57 mm
 * dolzina igle = 19 mm
 * max izteg igle = 71 mm
 */
/*
 * tof test:
 * les od konca bloka 31.5 mm (čis pr igli)
 * tof reading ~87 mm
 */
//----------------------------------------


/* X-NUCLEO-IHM04A1 H-bridge pin mapping (STM32H750B-DK Arduino header)
 *
 *  A6 = PA6  → IN1: TIM13_CH1 (AF9)  – H-bridge input 1  (PWM)
 *  A1 = PF8  → IN2: GPIO output       – H-bridge input 2  (direction)
 *       Forward  : IN1 = PWM,  IN2 = LOW
 *       Reverse  : IN1 = 0,    IN2 = HIGH
 *
 *  A3 = PA1  → direction-change button, EXTI1, active LOW + pull-up
 *
 *  Note: TIM13_CH1 je na PA6 (AF9) — PF8 nima timer AF na STM32H750XB.
 *        PF8 se uporablja kot navaden GPIO za smer.
 */
#define DC_IN1_PORT         GPIOH
#define DC_IN1_PIN          GPIO_PIN_15
#define DC_IN2_PORT         GPIOB
#define DC_IN2_PIN          GPIO_PIN_4

#define DC_TIM_PERIOD       999U    /* ARR  → 1000 steps resolution              */
#define DC_TIM_PRESCALER    199U    /* 200 MHz / 200 / 1000 = 1 kHz PWM freq     */
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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

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

volatile _Bool manual_mode = false;
volatile uint8_t manual_motor_idx = 0; // Trenutno izbran motor (0-3)

I2C_HandleTypeDef hi2c4;

volatile float target_pressure = 0.0f;     // Ciljni tlak iz UART3
volatile uint8_t uart3_rx_buffer[32];      // Buffer za sprejem
volatile uint8_t uart3_rx_index = 0;       // Indeks v bufferju
volatile uint8_t uart3_command_ready = 0;  // Flag, da je ukaz prejet
volatile uint8_t uart3_new_data = 0;       // Flag za novo sporočilo

uint16_t timer_val_start, timer_val_end;
uint16_t elapsed_1st, elapsed_2nd, elapsed_3rd;

uint8_t time_str1[60];
uint8_t time_str2[40];

/*
int position_m2=0;
int dir_m2=0;
int position_m3=0;
int dir_m3=0;
*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim8;

motor_struct_t motors[4]; // Declaration only

analog_pin_config_t analog_pins[6];

typedef enum { DC_DIR_FORWARD = 0, DC_DIR_REVERSE = 1 } dc_direction_t;

static TIM_HandleTypeDef       htim2;   /* unused – kept to avoid linker errors */
//static TIM_HandleTypeDef       htim8;  /* TIM13_CH1 → IN1 on PA6 (A6) */
volatile dc_direction_t        dc_direction    = DC_DIR_FORWARD;
volatile uint8_t               dc_dir_changed  = 0;

/* Nastavitve za avtomatski časovni premor in logiko odmika */
uint32_t dc_wait_time_ms = 5000;         /* Čas čakanja na točki obrata (5 sekund) */
uint32_t dc_stop_timestamp = 0;         /* Časovna značka, kdaj se je motor ustavil */
uint32_t dc_time_elapsed = 0;

typedef enum {
    DC_STATE_REGULATED = 0,             /* Normalno delovanje: senzor regulira hitrost */
    DC_STATE_WAITING,                   /* Motor je dosegel min razdaljo in čaka 5 sekund */
    DC_STATE_RETRACTING                 /* Motor se umika nazaj do maks razdalje */
} dc_state_t;

volatile dc_state_t dc_current_state = DC_STATE_REGULATED;

uint16_t trenutna_razdalja=2;
uint8_t false_read_tof=0;

volatile uint8_t uart_blocks_disabled = 0;  /* 1 = IGNORIRAJ UART BLOKADE (samostojno delovanje), 0 = ČAKAJ NA UART */

_Bool zagon_izvedbe = false;

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
//_Bool move_effector(uint32_t x, uint32_t y, uint32_t orientation);
void update_global_coordinates(void);
_Bool read_switch(uint8_t motor_number);
void run_motor(uint8_t motor_number);
void stop_motor(uint8_t motor_number);
void pump_liquid(uint32_t ammount_of_liquid);
void test_motor(uint8_t motor_number);
void test_all_motors();
void demo_za_predstavitev();
void move_motor_2_end_switch(uint8_t motor_number, _Bool direction);
void calibrate_all_motors(void);

void process_encoder(uint8_t encoder_number, _Bool A, _Bool B, _Bool Z);

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
void motor_status(void);

void test_tipke_na_roke(void);
void receive_target_point(int *x_coordinate, int *y_coordinate, int* z_coordinate, int *phi);

static void MPU_Config(void);
void USART3_Pin_Init(void);

void configure_end_switch_interrupts(void);
//void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI1_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
//void USART_write(int ch);

//void UART_Write_String(char *p);

//uint32_t Read_ADC(ADC_HandleTypeDef* hadc);

void MX_ADC_Init_AnalogPins(void);
void configure_analog_pins(void);
uint16_t read_analog_pin(uint8_t pin_index);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float izmeri_pritisk();
float nastavi_pritisk();
_Bool reguliraj_pritisk(float izmerjen_tlak, float zeljen_tlak,float toleranca, uint8_t stevilka_motorja);

void serial_print_string(const char* text);
void serial_print_uint16(uint16_t number);
void serial_print_float(float number);
void serial_print_empty_screen(void);

void USART3_IRQHandler(void);
float parse_float_from_string(const char* str);

static void MX_I2C4_Init(void);
void VL53L0X_Init(void);
uint16_t VL53L0X_ReadDistance(void);
void VL53L0X_LoadTuningSettings(void);
static HAL_StatusTypeDef vl_write(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef vl_read(uint8_t reg, uint8_t *val);
static HAL_StatusTypeDef vl_read16(uint8_t reg, uint16_t *val);
static void VL53L0X_PerformSPADCalibration(void);
static void MX_TIM6_Init(void);
void TIM6_DAC_IRQHandler(void);
static void MX_TIM8_Init(void);
void VL53L0X_Diagnose(void);
void DC_Motor_Init(void);
void DC_Motor_Update(uint16_t distance_mm);
void DC_Motor_Set_Speed(int16_t speed);

void execute_robot_movement(void);
void pospravi_robota(void);
void robot_control_manually(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TS_Init_t *hTSs;

//#define BUFFER_SIZE 30

//char rx_buff[BUFFER_SIZE];
//char rcv_buff[30];

TS_Init_t *hTSs;

__attribute__((aligned(32))) char rx_buff[32];        // padded to cache line size
__attribute__((aligned(32))) uint8_t uart_rx_buffer[32];

uint32_t timing_uart = 0;
uint32_t limit_uart = 5; //mej osveževanja

#define UART_RX_BUFFER_SIZE 64
//char uart_rx_buffer[UART_RX_BUFFER_SIZE];
//uint16_t uart_rx_index = 0;
uint8_t uart_rx_index = 0;
uint8_t uart_single_byte; // Tukaj HAL shrani zadnji prejeti znak
extern UART_HandleTypeDef huart3;

uint8_t podatki;
char vnos[100];

_Bool uart_command_ready = false;

uint8_t num_of_motors=4; //število vseh motorjev

uint32_t effector_x=0;   //end effector x coordinate [mm]
uint32_t effector_y=0;   //end effector y coordinate [mm]
uint32_t effector_orientation=0;   //end effector orientation [deg]

int target_x_coordinate;
int target_y_coordinate;
int target_z_coordinate;
int target_phi;

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

uint32_t encoder_maximum=4096; //popravi!!!

volatile uint16_t vl53_distance_mm = 0;
volatile uint8_t  vl53_data_ready  = 0;
uint8_t vl53_stop_variable = 0;   /* used in Init and referenced via extern */

//_Bool end_switch0_triggered=0;
//_Bool end_switch1_triggered=0;
//_Bool end_switch2_triggered=0;


// Bounding Box definicija (v mm)
typedef struct {
    int32_t min_x;
    int32_t max_x;
    int32_t min_y;
    int32_t max_y;
} BoundingBox_t;

// Nastavi poljubne meje varnega območja v mm
// med stikali J1 60cm, voziček 10cm
volatile BoundingBox_t robot_bbox = {
    .min_x = -530/2,
    .max_x = 530/2,
    //.min_y = 90,     // Y je lahko samo pozitiven
    //.max_y = 90+105+150

	.min_y=300,//290,
	.max_y=400

};

// Željene ciljne koordinate (nastavljene preko UART)
volatile int32_t target_x = 0;
volatile int32_t target_y = 0;
volatile int32_t target_o = 0; // Orientacija v stopinjah

float current_x=0;
float current_y=90;
float current_o=0;
float izteg = 0;
float max_izteg=105;
_Bool premik_done=0;
_Bool lin_motor_running=0;

int16_t dc_motor_speed=0;

float igla_sklop_offset=219;//v mm
float offset_centr_vrtenja_do_kamera=110;//v mm

_Bool calibrate_flag=0;
_Bool switch_test=0;
_Bool tof_test=0;
_Bool spik_test=0;
_Bool pumpa_flag=0;
_Bool tlak_flag=0;
_Bool ciscenje_igle_requested=0;
_Bool regulacija_tlaka_flag=0;

_Bool tof_reset_requested=0;
_Bool tof_power_cycle=0;//flag da smo ga izklopili

_Bool manual_mode_flag=0;
_Bool manual_mode_array[8]={0,0,0,0,0,0,0,0};
//						{a,d,w,s,q,e,r,f}
// a,d (M1,LR) | w,s (M3,FB) | q,e (M2,ROT) | r,f (LIN,FB)

#define PI 3.141592654

uint8_t next_step=9; //spremenljivka za korake v meritvi
//0=segmentacija
//1=premik v predvbodno tocko
//2=premik tik do debla z iglo
//3=premik igle v deblo
//4=premik v zacetno lego

//-------za izračun pretoka----------
float notranji_polmer_cevi=3;//mm
float polmer_ovinka_cevi=16.625;//mm

float kolicina_precrpane_tekocine=0; //v micro litrih
float kolicina_tekocine_na_obrat=(2*PI*16.625)*(PI*pow(r,2));//obseg*presek
float target_kolicina_tekocine=1000; //v micro litrih
uint32_t pump_num_steps=0;
uint32_t pump_num_steps_per_turn=100;
//-----------------------------------

// Funkcija za izpis stanja spremenljivk nazaj na UART
void uart_print_current_targets(void) {
    char response[200];

    // Eksplicitno pretvorimo obe vrednosti v int32_t pred odštevanjem!
    int32_t pos_x = (int32_t)motors[0].position;
    int32_t home_x = (int32_t)motors[0].home_position;

    int32_t pos_y = (int32_t)motors[2].position;
    int32_t home_y = (int32_t)motors[2].home_position;

    int32_t pos_o = (int32_t)motors[1].position;
    int32_t home_o = (int32_t)motors[1].home_position;

    // Izračun relativnih korakov (sedaj so lahko zanesljivo negativni)
    int32_t relative_steps_x = pos_x - home_x;
    int32_t relative_steps_y = pos_y - home_y;
    int32_t relative_steps_o = pos_o - home_o;

    // Pretvorba v fizikalne enote
    //current_x = (motors[0].unit_conversion > 0.001f) ? ((float)relative_steps_x / motors[0].unit_conversion) : 0.0f;
    //current_y = (motors[2].unit_conversion > 0.001f) ? ((float)relative_steps_y / motors[2].unit_conversion) : 0.0f;
    //current_o = (motors[1].unit_conversion > 0.001f) ? ((float)relative_steps_o / motors[1].unit_conversion) : 0.0f;

    //izteg= (float)relative_steps_y / motors[2].unit_conversion+motors[2].offset;

    /*
    current_o=(float)relative_steps_o / motors[1].unit_conversion;
    current_x=(float)relative_steps_x / motors[0].unit_conversion+cos((90-current_o)*PI/180)*izteg;
    current_y=sin((90-current_o)*PI/180)*izteg;
	*/

    //float trenutni_izteg=motors[2].position/motors[2].unit_conversion-motors[2].offset;
    float trenutni_izteg=motors[2].position/motors[2].unit_conversion+motors[2].offset+igla_sklop_offset;

    current_o=motors[1].position/motors[1].unit_conversion-motors[1].travel_length/2;
    current_y=trenutni_izteg*sin((90-current_o)*PI/180);
    current_x=trenutni_izteg*cos((90-current_o)*PI/180)+motors[0].position/motors[0].unit_conversion-motors[0].travel_length/2;

    volatile int32_t print_target_y=target_y+motors[2].offset+igla_sklop_offset;

    // Izpis v terminal
    snprintf(response, sizeof(response),
             "\r\n[STATUS] Cilj: X=%d, Y=%d, O=%d \r\n[STATUS] Trenutna lega: X=%.2f mm, Y=%.2f mm, O=%.2f st.\r\n[STATUS] Delujoci motorji: 0:%d | 1:%d | 2:%d\r\n",
             target_x, print_target_y, target_o,
             current_x, current_y, current_o,
			 motors[0].running, motors[1].running, motors[2].running);

    HAL_UART_Transmit(&huart3, (uint8_t*)response, strlen(response), 100);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	CPU_CACHE_Enable();
	HAL_Init();

    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    __DSB();
    __ISB();

    SCnSCB->ACTLR |= (1UL << 1);
    __DSB();
    __ISB();

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);  /* highest priority */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4, 0);  /* lower than SysTick */


	uint32_t RNG_PTR[2];
	for(uint8_t i=0;i<30;i++)rx_buff[i]='\0';
	for(uint8_t i=0;i<30;i++)uart_rx_buffer[i]='\0';

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */


	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	PeriphCommonClock_Config();


	/* Configure the peripherals common clocks */
	//PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM15_CLK_ENABLE();
	__HAL_RCC_TIM12_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_ADC12_CLK_ENABLE();  // For ADC1 and ADC2
	__HAL_RCC_ADC3_CLK_ENABLE();    // For ADC3
	__DSB();
	__ISB();


	//GPIO initialization
	MX_GPIO_Init();
	if (!uart_blocks_disabled) {MX_USART3_UART_Init();}
	//HAL_UART_Receive_IT(&huart3, rx_buff_usb, 10);


	I2C4_BusRecovery();
	HAL_Delay(10);


	MX_I2C4_Init();      /* Ta funkcija nastavi PD12/PD13 v Alternate Function način */


	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // Poskusi za trenutek sprostiti CS


	//serial_print_string("Skeniram naslove na I2C4 (8-bit format)...\r\n");
	char msg[32];
	for(uint16_t i = 1; i < 255; i++) {
		// STM32 HAL skener preverja sode (pisanje) naslove
		if(HAL_I2C_IsDeviceReady(&hi2c4, i, 1, 10) == HAL_OK) {
			//serial_print_string("Najdena naprava na 8-bit naslovu: 0x");
			//sprintf(msg, "%02X", i);
			//serial_print_string(msg);
			//serial_print_string("\r\n");
		}
	}
	//serial_print_string("Skeniranje koncano.\r\n");

	if (HAL_I2C_IsDeviceReady(&hi2c4, VL53L0X_ADDR, 5, 100) == HAL_OK) {
		//serial_print_string("TOF senzor VL53L0X zaznan, inicializiram...\r\n");
		VL53L0X_Init();
		HAL_Delay(100);
		VL53L0X_Diagnose();
		MX_TIM6_Init();
		MX_TIM8_Init();
		//DC_Motor_Init();
	} else {
		serial_print_string("Senzorja na 0x52 ni. Preskakujem init, da preprecim HardFault.\r\n");
	}


	MX_USART1_UART_Init();


	// Start receiving - THIS IS CRITICAL!
	//HAL_UART_Receive_IT(&huart1, rx_buff, 30);  // Receive 1 byte at a time
	SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buff, 32);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buff, 30);  // Receive 1 byte at a time

	HAL_UART_Receive_IT(&huart3, &uart_single_byte, 1);
    //Debug_USART1_Config();

	//Timer initialization
	MX_TIM1_Init();
	MX_TIM15_Init();
	MX_TIM3_Init();
	MX_TIM12_Init();
	//HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

	// Nastavi časovnike na višjo prioriteto (npr. 3), da lahko prekinjajo UART
	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0); //?
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0); //?
	//HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 3, 0);

	/* USER CODE BEGIN 2 */
	// freq=100000 -> hitro
	// steps=40000 -> en obrat

	motors[0] = (motor_struct_t){
	    .max_speed = 10000,
	    .current_speed = 0,
	    .direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.allowed_direction=2,
		.num_steps_for_switch_release=1000, //spremeni
	    .direction_pin = GPIO_PIN_3,
	    .direction_port = GPIOG,//D2
	    .timer = &htim3,
	    .timer_channel = TIM_CHANNEL_1,
		.frequency = 30000,
	    .motor_pin = GPIO_PIN_6,//D3
	    .motor_port = GPIOA,
	    .max_position = 10000000,
		.starting_position = 5000,
	    .position = 0,
		.target_position=0,
		.offset=0,
		.running = false,
	    .reset_requested = false,
	    .reset_completed = false,
	    .end_switch_pin = GPIO_PIN_3,//D8
	    .end_switch_port = GPIOE,
		.end_switch_triggered = 0,
	    //.end_switch2_pin = GPIO_PIN_15,//D9
	    //.end_switch2_port = GPIOH,
		.unit_conversion=100, //steps per mm
		.travel_length=630, //mm
		.num_steps_per_turn=40000/*,
		.num_turns_from_encoder=0,
		.encoder_A_state = 0,
		.encoder_A_pin = GPIO_PIN_3,
	    .encoder_A_port = GPIOE,
		.encoder_B_state = 0,
		.encoder_B_pin = GPIO_PIN_3,
	    .encoder_B_port = GPIOE,
		.encoder_Z_state = 0,
		.encoder_Z_pin = GPIO_PIN_3,
	    .encoder_Z_port = GPIOE*/
	};
	motors[2] = (motor_struct_t){
		.max_speed = 10000,
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.allowed_direction=2,
		.num_steps_for_switch_release=1000, //spremeni
		.direction_pin = GPIO_PIN_1,
		.direction_port = GPIOK,//D4
		.timer = &htim1,
		.timer_channel = TIM_CHANNEL_1,
		.frequency = 30000,
		.motor_pin = GPIO_PIN_8,//D5
		.motor_port = GPIOA,
		.max_position = 10000000,
		.starting_position=5000,
		.position = 0,
		.target_position=0,
		.offset=90,//os vrtenja do konc spindla
		.running = false,
		.reset_requested = false,
		.reset_completed = false,
		.end_switch_pin = GPIO_PIN_3,//bilo 15
		.end_switch_port = GPIOD,//bilo H
		.end_switch_triggered = 0,
		//.end_switch1_pin = GPIO_PIN_4,//D10
		//.end_switch1_port = GPIOB,
	    //.end_switch2_pin = GPIO_PIN_2,//D12
	    //.end_switch2_port = GPIOI,
		.unit_conversion=100, //steps per mm
		.travel_length=105, //mm
		.num_steps_per_turn=40000/*,
		.num_turns_from_encoder=0,
		.encoder_A_state = 0,
		.encoder_A_pin = GPIO_PIN_3,
	    .encoder_A_port = GPIOE,
		.encoder_B_state = 0,
		.encoder_B_pin = GPIO_PIN_3,
	    .encoder_B_port = GPIOE,
		.encoder_Z_state = 0,
		.encoder_Z_pin = GPIO_PIN_3,
	    .encoder_Z_port = GPIOE*/
	};
	motors[1] = (motor_struct_t){
		.max_speed = 10000,
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.allowed_direction=2,
		.num_steps_for_switch_release=1000, //spremeni
		.direction_pin = GPIO_PIN_8,
		.direction_port = GPIOI,//D7
		.timer = &htim15,
		.timer_channel = TIM_CHANNEL_2,
		.frequency = 30000,
		.motor_pin = GPIO_PIN_6,//D6
		.motor_port = GPIOE,
		.max_position = 10000000,
		.starting_position=5000,
		.position = 0,
		.target_position=0,
		.offset=0,
		.running = false,
		.reset_requested = false,
		.reset_completed = false,
		.end_switch_pin = GPIO_PIN_2,//D10
		.end_switch_port = GPIOI,//prej PB4
		.end_switch_triggered = 0,
		//.end_switch1_pin = GPIO_PIN_13,//D14
		//.end_switch1_port = GPIOD,
		//.end_switch2_pin = GPIO_PIN_3,//D13
		//.end_switch2_port = GPIOD,
		.unit_conversion=100, //steps per deg
		.travel_length=60, //deg
		.num_steps_per_turn=40000/*,
		.num_turns_from_encoder=0,
		.encoder_A_state = 0,
		.encoder_A_pin = GPIO_PIN_3,
	    .encoder_A_port = GPIOE,
		.encoder_B_state = 0,
		.encoder_B_pin = GPIO_PIN_3,
	    .encoder_B_port = GPIOE,
		.encoder_Z_state = 0,
		.encoder_Z_pin = GPIO_PIN_3,
	    .encoder_Z_port = GPIOE*/
	};
	motors[3] = (motor_struct_t){ //max 50000 freq
		.max_speed = 10000, //pomembno za max pretok
		.current_speed = 0,
		.direction = 1,
		.direction_plus = 1,
		.direction_minus = 0,
		.allowed_direction = 2,
		.num_steps_for_switch_release=1000, //ne rabis tukaj
		.direction_pin = GPIO_PIN_12,
		.direction_port = GPIOD,//D15
		.timer = &htim12,
		.timer_channel = TIM_CHANNEL_2,
		.frequency = 100,
		.motor_pin = GPIO_PIN_15,//D11
		.motor_port = GPIOB,
		.max_position = 10000,
		.starting_position=5000,
		.position = 0,
		.target_position=0,
		.offset=0,//odmik od osi vrtenja v mm?
		.running = false,

		//NE UPORABLJAJ = IGNORIRAJ:
		.reset_requested = false,
		.reset_completed = false,


		//uporabljeno za dc motor:
		.end_switch_pin = GPIO_PIN_8,//D14
		.end_switch_port = GPIOF,
		.end_switch_triggered = 0,


		.unit_conversion=100, //steps per mm
		.travel_length=360, //deg
		.num_steps_per_turn=200/*,
		.num_turns_from_encoder=0,

		//NE UPORABLJAJ = IGNORIRAJ:
		.encoder_A_state = 0,
		.encoder_A_pin = GPIO_PIN_3,
	    .encoder_A_port = GPIOE,
		.encoder_B_state = 0,
		.encoder_B_pin = GPIO_PIN_3,
	    .encoder_B_port = GPIOE,
		.encoder_Z_state = 0,
		.encoder_Z_pin = GPIO_PIN_3,
	    .encoder_Z_port = GPIOE*/
		//konc prepovedi
	};

	// Initialize analog pins A0-A5
	configure_analog_pins();


	//usable IO: I2
	configure_end_switch_interrupts();

	/* Configure LED1 */
	//BSP_LED_Init(LED1);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/*

	*/
	stop_all_motors();

	//reset_motors();

	_Bool values[6]={0,0,0,0,0,0};

	//run_motor(0);

	//run_motor(2);


    // Počisti UART3 buffer
    memset((void*)uart3_rx_buffer, 0, 32);
    uart3_rx_index = 0;
    uart3_command_ready = 0;


/*
    char startup_menu[] =
    		"=============================================================================\r\n"
    		"   .-'''-. .-------. .-./`) .--.   .--.   _______       ,-----.  ,---------. \r\n"
    		"  / _     \\  _(`)_ \\ .-.')|  | _/  /   \  ____  \   .'  .-,  '.\          \\\r\n"
    		" (`' )/`--'| (_ o._)|/ `-' \| (`' ) /    | |    \ |  / ,-.|  \ _ \`--.  ,---'\r\n"
    		"(_ o _).   |  (_,_) / `-'`'`|(_ ()_)     | |____/ / ;  \  '_ /  | :  |   \   \r\n"
    		" (_,_). '. |   '-.-'  .---. | (_,_)   __ |   _ _ '. |  _`,/ \ _/  |  :_ _:   \r\n"
    		".---.  \  :|   |      |   | |  |\ \  |  ||  ( ' )  \: (  '\_/ \   ;  (_I_)   \r\n"
    		"\    `-'  ||   |      |   | |  | \ `'   /| (_{;}_) | \ `'/  \  ) /  (_(=)_)  \r\n"
    		" \       / /   )      |   | |  |  \    / |  (_,_)  /  '. \_/``'.'    (_I_)   \r\n"
    		"  `-...-'  `---'      '---' `--'   `'-'  /_______.'     '-----'      '---'   \r\n"
    		"=============================================================================\r\n";
*/
    char startup_menu[] =
    		"\r\n\n\n\n\n\n\n\n"
    		" ___________ _____ _   ________  _____ _____ \r\n"
    		"/  ___| ___ \\_   _| | / /| ___ \\|  _  |_   _|\r\n"
    		"\\ `--.| |_/ / | | | |/ / | |_/ /| | | | | |  \r\n"
    		" `--. \\  __/  | | |    \\ | ___ \\| | | | | |  \r\n"
    		"/\\__/ / |    _| |_| |\\  \\| |_/ /\\ \\_/ / | |  \r\n"
    		"\\____/\\_|    \\___/\\_| \\_/\\____/  \\___/  \\_/  \r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)startup_menu, strlen(startup_menu), 500);



    HAL_Delay(10);
	DC_Motor_Init(); //dodaj pull down upor
    DC_Motor_Set_Speed(0);



    char menu[] =
            "\r\n==================================================================\r\n"
            //" KALIBRACIJA USPESNO ZAKLJUCENA!\r\n"
            "======================================================================\r\n"
            " Navodila za vnos ukazov preko UART (vseeno male/VELIKE crke):\r\n"
            "  x=stevilka  -> Nastavi cilj X (pozitiven ali negativen)\r\n"
            "  y=stevilka  -> Nastavi cilj Y (samo pozitiven)\r\n"
            "  o=stevilka  -> Nastavi orientacijo O (omejitev od -30 do 30)\r\n"
            "  go          -> Sprozi socasen premik M0 in M1, nato sekvencno M2\r\n"
            "  exit        -> Pospravi robota iz koncne lege v zacetno\r\n"
    		"  manual      -> Nacin za krmiljenje s tipkovnico\r\n"
    		"  cc          -> Izklop nacina za krmiljenje s tipkovnico\r\n"
    		"  calibrate   -> Izvede kalibracijo\r\n"
    		"  stikala     -> Vklop nacina za preverjanje delovanja stikal\r\n"
    		"  sw          -> Izklop nacina za preverjanje delovanja stikal\r\n"
    		"  tof         -> Izpit desetih meritev TOF senzorja\r\n"
    		"  spik        -> Preklop nacina za preizkus spikanja\r\n"
    		"  pumpa       -> Vklopi/izklopi pumpo\r\n"
    		"  tlak        -> Zažene nekaj meritev tlaka\r\n"
    		"  regulacija  -> Vklopi/izklopi regulacijo pritiska na 1.8 bar\r\n"
    		"----------------------------------------------------------------------\r\n"
            " Vnesi ukaz za orientacijo -> y -> x; in pritisni ENTER:\r\n"
    		"----------------------------------------------------------------------\r\n\r\n";

	HAL_UART_Transmit(&huart3, (uint8_t*)menu, strlen(menu), 500);
	static _Bool post_pospravljanje=0;
	static uint16_t prej_step=10;
	while (1) {
		/* USER CODE END WHILE */



		 //---------------------MERITEV------------------

		 if((next_step==1) && premik_done==0)execute_robot_movement();
		 else if((next_step==2 || next_step==3 || next_step==5 || next_step==0)&&premik_done==1)
		 {
			 if(!tof_reset_requested)
			 {
			 	 	if(next_step==5 || next_step==0)
			 	 	{
			 	 		DC_Motor_Update(6);
			 	 	}
			 	 	else if ((vl53_data_ready && !motors[0].running && !motors[1].running && !motors[2].running && premik_done)||(vl53_data_ready&&spik_test))
 		            {
			 	 		static uint16_t last_valid_distance=0;
			 	 		static uint32_t distance_validate_timestamp=0;

 		            	const uint8_t num_samples=5;
						static uint16_t action_moving_average_array[5];
						static uint8_t action_moving_array_index=0;
						static _Bool moving_average_ready=0;
						uint32_t avrg_razdalja=0;

						static uint8_t num_fail_prints=0;

		                vl53_data_ready = 0;
		                trenutna_razdalja = VL53L0X_ReadDistance();

		                if(trenutna_razdalja==0 || trenutna_razdalja>65000)
		                {
		                	false_read_tof++;
		                }
		                else false_read_tof=0;

		                if ((false_read_tof>num_samples)&&(!tof_reset_requested))
		                {
		                	serial_print_string("Nenavadna razdalja, ustavljam...\r\n");
		                	DC_Motor_Set_Speed(0);
		                	num_fail_prints++;

							I2C4_BusRecovery();
							HAL_Delay(10);
							MX_I2C4_Init();       // ponovna inicializacija
							VL53L0X_Init();       // ponovni zagon senzorja

		                	if(num_fail_prints>=10)
		                	{
		                		num_fail_prints=0;
								tof_reset_requested=1;
								serial_print_string("Prosim resetirajte TOF senzor.\r\n");
								/*
								next_step=0;
		            			serial_print_string("\r\n");
		            			serial_print_string("MERITEV SENSOR FAIL");
		            			serial_print_string("\r\n");
		            			serial_print_string("MERITEV SENSOR FAIL");
		            			serial_print_string("\r\n");
		            			serial_print_string("MERITEV SENSOR FAIL");
		            			serial_print_string("\r\n");
								*/
		                	}
		                }

		                else
		                {
		                	num_fail_prints=0;
		                if (trenutna_razdalja!= 65535 && trenutna_razdalja!=0)
		                    {
		                		action_moving_average_array[action_moving_array_index]=trenutna_razdalja;
		                		action_moving_array_index++;
		                    	if(action_moving_array_index>(num_samples-1))
		                    	{
		                    		moving_average_ready=1;
		                    		action_moving_array_index=0;
		                    	}

							if(moving_average_ready)
								{
									avrg_razdalja=0;

									static uint32_t avrg_razdalja=0;

									for (uint8_t i=0;i<num_samples;i++)
									{
										avrg_razdalja+=action_moving_average_array[i];
									}
									avrg_razdalja/=(num_samples+1);

									/*
					                //če se sensor reading zatakne v hitri fazi
									if(distance_validate_timestamp==0)distance_validate_timestamp=HAL_GetTick();
					                if((HAL_GetTick()-distance_validate_timestamp)<1000 && dc_motor_speed==-900)
					                {
					                	if(sqrt(pow(trenutna_razdalja-last_valid_distance,2))>=2)
					                	{
					                		last_valid_distance=trenutna_razdalja; //če v času 1.5 s se premakne za 3 mm je ok
					                	}
					                	else
					                	{
					                		next_step=0;
					                	}
					                }
					                */

									DC_Motor_Update(avrg_razdalja);
									}
		                    	}
		                	}
 		            	}
			 }
			 else
			 {//če je zahtevan reset tofa počakamo prvo da se ugasne senzor in potem ponovni prižig

						//če je prižgan in vemo da smo ga ugasnili potem inicializiramo
						if ((HAL_I2C_IsDeviceReady(&hi2c4, VL53L0X_ADDR, 5, 100) == HAL_OK))
						{

								I2C4_BusRecovery();
								HAL_Delay(10);
								MX_I2C4_Init();
								HAL_Delay(10);

								VL53L0X_Init();       // ponovni zagon senzorja
								HAL_Delay(10);


								tof_reset_requested=0;
								tof_power_cycle=0;
						}
						else {
							tof_power_cycle=1;//ugasnjen
						}
			 }
		 }
		 else if(next_step==4)
		 {
			 float target_pressure=1.8; //zelimo 1.8 bara imeti v sistemu
			 float target_accuracy=0.05;
			 uint8_t pump_motor_number=3;

			 //vzpostavimo stabilen pritisk
			 while(!reguliraj_pritisk(izmeri_pritisk(),target_pressure,target_accuracy,pump_motor_number) && (next_step!=0)){}

			uint32_t pumpa_stop_timestamp = HAL_GetTick();
			uint32_t pumpa_wait_time_ms=10000; //zelimo da ohrani pritisk 10 sekund
			uint32_t pump_time_elapsed=0;


			 //nato držimo se kolikor dolgo je nastavljeno
			while((pump_time_elapsed < pumpa_wait_time_ms) && (next_step!=0))
			{
				if(reguliraj_pritisk(izmeri_pritisk(),target_pressure,target_accuracy,pump_motor_number))
				{
					pump_time_elapsed+=HAL_GetTick()-pumpa_stop_timestamp;
					pumpa_stop_timestamp=HAL_GetTick();
				}
				else
				{
					pumpa_stop_timestamp = HAL_GetTick();
				}
			}

			stop_motor(pump_motor_number);

			serial_print_string("\r\nCrpanje opravljeno.\r\n");
			serial_print_string("\r\n");
			serial_print_string("MERITEV 4 STOP");
			serial_print_string("\r\n");
			serial_print_string("MERITEV 4 STOP");
			serial_print_string("\r\n");
			serial_print_string("MERITEV 4 STOP");
			serial_print_string("\r\n");
		 }
		 else if((next_step==5 || next_step==0) && premik_done==0)
		 {
			 stop_motor(3);
			 pospravi_robota();
			 post_pospravljanje=1;

			 //ce smo izvedli vbod spucamo iglo:
			 if(ciscenje_igle_requested)
			 {
				 motors[3].frequency=500;
				 run_motor(3);
				 HAL_Delay(2000);
				 stop_motor(3);
				 ciscenje_igle_requested=0;
			 }
		 }

/*//debug:
if(post_pospravljanje && (prej_step!=next_step))
	{
	prej_step=next_step;
	serial_print_string("\r\n");
	serial_print_uint16((uint16_t)next_step);
	serial_print_string("\r\n");
	}
*/

//---------------------konec dela za MERITEV------------------------


//----------------------MANUAL MODE------------------------------------

if (manual_mode_flag==1)
{
	robot_control_manually();//interupti stikal, 1 za rewirat
}
//--------------------------END-------------------------------------

//-----------------------CALIBRATION MODE-----------------
if (calibrate_flag==1)
{
	calibrate_all_motors();//interupti stikal, 1 za rewirat
	calibrate_flag=0;
}
//-----------------------------END--------------------------

//-----------------------CALIBRATION MODE-----------------
if (switch_test==1)
{
   	_Bool sw0=read_switch(0);
	_Bool sw1=read_switch(1);
	_Bool sw2=read_switch(2);
	_Bool sw3=read_switch(3);
	char debug_msg[100];
	snprintf(debug_msg, sizeof(debug_msg), " SW0: %u | SW1: %u | SW2: %u | SW3: %u \r\n", sw0,sw1,sw2,sw3);
	serial_print_string(debug_msg);
	HAL_Delay(200);
}
//-----------------------------END--------------------------

//---------------------------tof test----------------------
if(tof_test || spik_test)
{
	if(!tof_reset_requested)
			 {
					if (vl53_data_ready)
					{
						static uint16_t last_valid_distance=0;
						static uint32_t distance_validate_timestamp=0;

						const uint8_t num_samples=5;
						static uint16_t action_moving_average_array[5];
						static uint8_t action_moving_array_index=0;
						static _Bool moving_average_ready=0;
						uint32_t avrg_razdalja=0;

						static uint8_t num_fail_prints=0;

						vl53_data_ready = 0;
						trenutna_razdalja = VL53L0X_ReadDistance();

						if(trenutna_razdalja==0 || trenutna_razdalja>65000)
						{
							false_read_tof++;
						}
						else false_read_tof=0;

						if ((false_read_tof>num_samples)&&(!tof_reset_requested))
						{
							serial_print_string("Nenavadna razdalja, ustavljam...\r\n");
							num_fail_prints++;

							I2C4_BusRecovery();
							HAL_Delay(10);
							MX_I2C4_Init();       // ponovna inicializacija
							VL53L0X_Init();       // ponovni zagon senzorja

							if(num_fail_prints>=10)
							{
								num_fail_prints=0;
								tof_reset_requested=1;
								serial_print_string("Prosim resetirajte TOF senzor.\r\n");
								/*
								next_step=0;
								serial_print_string("\r\n");
								serial_print_string("MERITEV SENSOR FAIL");
								serial_print_string("\r\n");
								serial_print_string("MERITEV SENSOR FAIL");
								serial_print_string("\r\n");
								serial_print_string("MERITEV SENSOR FAIL");
								serial_print_string("\r\n");
								*/
							}
						}

						else
						{
							num_fail_prints=0;
						if (trenutna_razdalja!= 65535 && trenutna_razdalja!=0)
							{
								action_moving_average_array[action_moving_array_index]=trenutna_razdalja;
								action_moving_array_index++;
								if(action_moving_array_index>(num_samples-1))
								{
									moving_average_ready=1;
									action_moving_array_index=0;
								}

							if(moving_average_ready)
								{
									avrg_razdalja=0;

									static uint32_t avrg_razdalja=0;

									for (uint8_t i=0;i<num_samples;i++)
									{
										avrg_razdalja+=action_moving_average_array[i];
									}
									avrg_razdalja/=(num_samples+1);

									char debug_msg[100];
									snprintf(debug_msg, sizeof(debug_msg), " Razdalja: %u \r\n", avrg_razdalja);
									serial_print_string(debug_msg);

									if(spik_test)
										{
										DC_Motor_Update(avrg_razdalja);
										}
									}
								}
							}
						}
			 }
			 else
			 {//če je zahtevan reset tofa počakamo prvo da se ugasne senzor in potem ponovni prižig

						//če je prižgan in vemo da smo ga ugasnili potem inicializiramo
						if ((HAL_I2C_IsDeviceReady(&hi2c4, VL53L0X_ADDR, 5, 100) == HAL_OK))
						{
							//if(tof_power_cycle)
							//{
								I2C4_BusRecovery();
								HAL_Delay(10);
								MX_I2C4_Init();
								HAL_Delay(10);

								VL53L0X_Init();       // ponovni zagon senzorja
								HAL_Delay(10);
								//I2C4_BusRecovery();
								//HAL_Delay(10);
								//MX_I2C4_Init();       // ponovna inicializacija
								//VL53L0X_Init();       // ponovni zagon senzorja

								tof_reset_requested=0;
								tof_power_cycle=0;
							//}
						}
						else {
							tof_power_cycle=1;//ugasnjen
						}
			 }
}
//------------------end tof test------------------

//-------------pumpa test---------------
// driver: Pololu DRV8825, delovna temp 60 °C
if(pumpa_flag && !motors[3].running)
{
	serial_print_string("\r\nZaganjam pumpo.\r\n");
	motors[3].frequency=500;
	run_motor(3);//zaženemo pumpo če ni že zagnana
}
else if(!pumpa_flag && !regulacija_tlaka_flag && motors[3].running)
{
	serial_print_string("\r\nUstavljam pumpo.\r\n");
	stop_motor(3);
}
//--------------konec pumpa test-----------------
//---------senzor za tlak test-------------
if(tlak_flag)
{
	for(uint8_t tlak_oneshot=0;tlak_oneshot<5;tlak_oneshot++)
	{
		serial_print_string("\r\nPovprečen izmerjen tlak:");
		serial_print_float(izmeri_pritisk());
		serial_print_string("\r\n");
	}
	tlak_flag=0;
}
//-----------konc tlak-------------------
//----------regulacija tlaka test-----------
if(regulacija_tlaka_flag)
{
	//vzpostavimo stabilen pritisk
	reguliraj_pritisk(izmeri_pritisk(),1.8,0.05,3);
}
//-----------konc tlak-------------------


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



static void MX_TIM8_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* 1. Initialize TIM8 Base Configuration */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0; // Adjust based on your APB2 clock to get your desired frequency
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 999;  // Matching your 0-999 speed range
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }

    /* 2. Configure Master/Slave Modes (Default) */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 3. Configure Channel 3 PWM Settings */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Starts at 0% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // Polarity for the 'N' channel
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    /* 4. Configure Break and Dead Time (Crucial for Advanced Timers like TIM8) */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    //sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // MOE Main Output Enable configuration
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Note: GPIO Initialization for PH15 (AF3) should be called here or inside HAL_TIM_PWM_MspInit */
}

/*
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
    if(htim_pwm->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();

    }
}
*/




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
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
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

  HAL_NVIC_SetPriority(TIM15_IRQn, 3, 0);
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
  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.State = HAL_TIM_STATE_RESET;
  htim12.Init.Prescaler = 79;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim12.Init.RepetitionCounter = 0;
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


  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

  HAL_TIM_MspPostInit(&htim12);

  //HAL_TIM_Base_Start_IT(&htim12);
  //HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
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

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
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

	/*
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Push-Pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;     // AF3 maps TIM8 to Port H
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
*/

	/* USER CODE END MX_GPIO_Init_2 */
}



/**
  * @brief  Prebere analogni pin (_C) in vrne digitalno stanje (true/false)
  * @param  hadc: Kazalec na ADC strukturo (npr. &hadc3)
  * @param  channel: ADC kanal (npr. ADC_CHANNEL_0 za PA0_C ali ADC_CHANNEL_1 za PA1_C)
  * @retval _Bool: true (HIGH) ali false (LOW)
  */
/*_Bool read_analog_pin_as_digital(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t raw_value = 0;

    // 1. Konfiguracija kanala za to specifično meritev
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_81CYCLES_5; // Dovolj dolg čas za stabilno meritev
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        // Napaka pri konfiguraciji
        return false;
    }

    // 2. Izvedba MERITEV
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        raw_value = HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);

    // 3. Primerjava vrednosti (16-bitni ADC: 0 do 65535)
    // Če je napetost bližje 3.3V (nad cca 2.5V), vrnemo true
    if (raw_value > 50000)
    {
        return true;  // Pin je HIGH
    }
    else
    {
        return false; // Pin je LOW (povezan na GND ali pull-down)
    }
}*/

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize all ADCs for analog pins A0-A5
 * @param None
 * @retval None
 */
void MX_ADC_Init_AnalogPins(void)
{
    // Configure ADC1 for A2 (PA0_C) and A3 (PA1_C)
    // Configure ADC3 for A1 (PF8), A4 (PC2_C), A5 (PC3_C)

    // ADC1 Configuration (for channels 0 and 1)
    __HAL_RCC_ADC12_CLK_ENABLE();

    // ADC3 Configuration (for channels 0,1,7)
    __HAL_RCC_ADC3_CLK_ENABLE();

    HAL_Delay(10);

    // Power up ADC1
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(1);

    // Power up ADC3
    ADC3->CR &= ~ADC_CR_DEEPPWD;
    ADC3->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(1);

    // Calibrate ADC1
    if(ADC1->CR & ADC_CR_ADEN) ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);

    // Calibrate ADC3
    if(ADC3->CR & ADC_CR_ADEN) ADC3->CR &= ~ADC_CR_ADEN;
    ADC3->CR |= ADC_CR_ADCAL;
    while(ADC3->CR & ADC_CR_ADCAL);

    // Enable ADCs with boost for high speed
    ADC1->CR |= ADC_CR_ADEN | ADC_CR_BOOST;
    ADC3->CR |= ADC_CR_ADEN | ADC_CR_BOOST;

    // Wait for ADCs ready
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while(!(ADC3->ISR & ADC_ISR_ADRDY));

    // Configure sample times (810.5 cycles for stability)
    ADC1->SMPR1 = (7 << (3*0)) | (7 << (3*1));  // Channels 0 and 1
    ADC3->SMPR1 = (7 << (3*0)) | (7 << (3*1));  // Channels 0 and 1
    ADC3->SMPR2 = (7 << (3*7));                 // Channel 7

    // Set number of conversions to 1 for each ADC
    ADC1->SQR1 = (0 << 0);  // 1 conversion
    ADC3->SQR1 = (0 << 0);  // 1 conversion
}

/**
 * @brief Configure all analog pins as analog inputs
 * @param None
 * @retval None
 */
void configure_analog_pins(void)
{

	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();

    // Configure PC0 as analog (A0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
/*
    // Configure PF8 as analog (A1)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Configure PA0_C as analog (A2)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // Enable PA0_C special function
    SYSCFG->PMCR |= SYSCFG_PMCR_PA0SO;

    // Configure PA1_C as analog (A3)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // Enable PA1_C special function
    SYSCFG->PMCR |= SYSCFG_PMCR_PA1SO;
*/



/*
        // 2. Nastavitev PF8 (ki že deluje)
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);



        // PF8 uporablja EXTI9_5_IRQn
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
*/



    // Configure PC2_C as analog (A4)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure PC3_C as analog (A5)
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize ADC peripherals
    MX_ADC_Init_AnalogPins();

    // Initialize the analog pins configuration structure
    analog_pins[0] = (analog_pin_config_t){
        .pin_number = 0,
        .adc_channel = ADC_CHANNEL_10,
        .hadc = &hadc1,  // Using ADC1 for PC0 (ADC123_IN10)
        .gpio_pin = GPIO_PIN_0,
        .gpio_port = GPIOC,
        .name = "A0"
    };

    /*
     analog_pins[1] = (analog_pin_config_t){
        .pin_number = 1,
        .adc_channel = ADC_CHANNEL_7,
        .hadc = &hadc3,  // Using ADC3 for PF8 (ADC3_IN7)
        .gpio_pin = GPIO_PIN_8,
        .gpio_port = GPIOF,
        .name = "A1"
    };

    analog_pins[2] = (analog_pin_config_t){
        .pin_number = 2,
        .adc_channel = ADC_CHANNEL_0,
        .hadc = &hadc1,  // Using ADC1 for PA0_C (ADC12_IN0)
        .gpio_pin = GPIO_PIN_0,
        .gpio_port = GPIOA,
        .name = "A2"
    };

    analog_pins[3] = (analog_pin_config_t){
        .pin_number = 3,
        .adc_channel = ADC_CHANNEL_1,
        .hadc = &hadc1,  // Using ADC1 for PA1_C (ADC12_IN1)
        .gpio_pin = GPIO_PIN_1,
        .gpio_port = GPIOA,
        .name = "A3"
    };
*/
    analog_pins[4] = (analog_pin_config_t){
        .pin_number = 4,
        .adc_channel = ADC_CHANNEL_0,
        .hadc = &hadc3,  // Using ADC3 for PC2_C (ADC3_IN0)
        .gpio_pin = GPIO_PIN_2,
        .gpio_port = GPIOC,
        .name = "A4"
    };

    analog_pins[5] = (analog_pin_config_t){
        .pin_number = 5,
        .adc_channel = ADC_CHANNEL_1,
        .hadc = &hadc3,  // Using ADC3 for PC3_C (ADC3_IN1)
        .gpio_pin = GPIO_PIN_3,
        .gpio_port = GPIOC,
        .name = "A5"
    };


/*
    // 2. Zapri analogna stikala, da povežeš PA0_C in PA1_C z ADC3 perifero
        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);
        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_CLOSE);

        // 3. Konfiguracija tvoje obstoječe strukture za PA0_C (Indeks 4)
        analog_pins[2] = (analog_pin_config_t){
            .pin_number = 0,
            .adc_channel = ADC_CHANNEL_0,
            .hadc = &hadc3,
            .gpio_pin = GPIO_PIN_0, // Za referenco, čeprav delamo preko ADC
            .gpio_port = GPIOA,
            .name = "PA0_C"
        };

        // 4. Konfiguracija tvoje obstoječe strukture za PA1_C (Indeks 5)
        analog_pins[3] = (analog_pin_config_t){
            .pin_number = 1,
            .adc_channel = ADC_CHANNEL_1,
            .hadc = &hadc3,
            .gpio_pin = GPIO_PIN_1,
            .gpio_port = GPIOA,
            .name = "PA1_C"
        };
*/

}

/**
 * @brief Direct register read for analog pins (most reliable)
 * @param pin_index: 0-5 for A0-A5
 * @retval 12-bit ADC value (0-4095)
 */
uint16_t read_analog_pin(uint8_t pin_index)
{
    if(pin_index >= 6) return 0;

    ADC_HandleTypeDef* hadc = analog_pins[pin_index].hadc;
    uint32_t channel = analog_pins[pin_index].adc_channel;

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;

    if(HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(hadc);

    if(HAL_ADC_PollForConversion(hadc, 100) != HAL_OK) {
        return 0;
    }

    uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return (uint16_t)value;  // HAL automatically handles resolution
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float izmeri_pritisk()
{
	uint8_t analog_pin_za_merjenje=0;//0 do 5
	float max_bari_senzor=8.2943; //vrednost pri 3.3V
	uint32_t analog_average=0;
	uint8_t num_samples_4_average=50;

	for(uint8_t a=0;a<num_samples_4_average;a++)analog_average+=read_analog_pin(analog_pin_za_merjenje);
	analog_average/=num_samples_4_average;

	return mapFloat(analog_average,973,4095,0.0,max_bari_senzor);
	//return mapFloat(read_analog_pin(analog_pin_za_merjenje),0,4095,0.0,max_bari_senzor);//adc 0-4096, pritisk 0-5?
}


//staro, prej s potenciometrom, zdaj preko uart:
/*float nastavi_pritisk()
{
	uint8_t analog_pin_za_merjenje=4;//0 do 5
	float max_bari_nastavljeni=8.2943;

	uint32_t analog_average=0;

	for(uint8_t a=0;a<10;a++)analog_average+=read_analog_pin(analog_pin_za_merjenje);
	analog_average/=10;

	return mapFloat(analog_average,973,4095,0.0,max_bari_nastavljeni);
	//return mapFloat(read_analog_pin(analog_pin_za_merjenje),0,4095,0.0,max_bari_nastavljeni);//adc 0-4096, pritisk 0-5?
}*/



/**
 * @brief Enostaven P regulator za uravnavanje tlaka
 * @param izmerjen_tlak: Trenutna izmerjena vrednost tlaka (float, v barih)
 * @param zeljen_tlak: Željena vrednost tlaka (uint16_t, celo število v barih)
 * @param smer_motorja: Kazalec na spremenljivko smeri motorja (1 = naprej, 0 = nazaj)
 * @param hitrost_motorja: Kazalec na spremenljivko hitrosti motorja (0-100%)
 * @param toleranca: Dovoljeno odstopanje od željene vrednosti (npr. 0.5 bara)
 * @return _Bool: true če je tlak ustaljen, false če regulacija še poteka
 */
_Bool reguliraj_pritisk(float izmerjen_tlak, float zeljen_tlak,
                        float toleranca, uint8_t stevilka_motorja)
{	static float napaka_old=0;
	static uint32_t frequency2Bset=0;

	float locljivost_napake=0.2;

	uint16_t pumpa_max_frequency=500;
	//6 rpm je 4000 frequency

    float napaka = izmerjen_tlak - zeljen_tlak;
    float kp = 2.0;  // Proporcionalni koeficient (po potrebi prilagodi)

    char debug_msg_pritisk[64];
	snprintf(debug_msg_pritisk, sizeof(debug_msg_pritisk), "\n\nPritisk: %f bar\r\n", izmerjen_tlak);
	serial_print_string(debug_msg_pritisk);

    // Preveri ali smo že v toleranci (ustaljen tlak)
    if((napaka > -toleranca) && (napaka < toleranca)) {

    	if (motors[stevilka_motorja].running)stop_motor(stevilka_motorja);  // Ustavi motor

    	return true;  // Tlak je ustaljen
    }

    /*
    serial_print_string("\r\nizmerjeno: ");
    serial_print_float(izmerjen_tlak);
    serial_print_string("\r\n");
    serial_print_string("nastavljeno: ");
    serial_print_float(zeljen_tlak);
    serial_print_string("\r\n\n");
    */

    //serial_print_string("----------------------\r\n\n");

    // Tlak je previsok - potrebno ga je znižati
    if((napaka >= 0)) {//&& ((napaka-napaka_old)>0.15)//

    	stop_motor(stevilka_motorja); //neprovratni ventil

    	/*

    	if (motors[stevilka_motorja].direction!=motors[stevilka_motorja].direction_plus)direction_change(stevilka_motorja,motors[stevilka_motorja].direction_plus);

        //int vrednost=(kp * napaka *(1-2*(napaka<0)) * 100);
        motors[stevilka_motorja].current_speed = (uint32_t)(kp * napaka *(1-2*(napaka<0)) * 100);;  // Hitrost sorazmerna napaki;
        //rabis ref hitrost pri znani frekvenci

        // Omeji hitrost na 0-100%
        if(motors[stevilka_motorja].current_speed > motors[stevilka_motorja].max_speed) motors[stevilka_motorja].current_speed = motors[stevilka_motorja].max_speed;

        //speed maping:
        frequency2Bset=motors[stevilka_motorja].current_speed*pumpa_max_frequency/500;//50000 je max frequency; 200 je pr kp 0.5 max speed
        if(frequency2Bset>pumpa_max_frequency)frequency2Bset=pumpa_max_frequency;

        if (abs(frequency2Bset-motors[stevilka_motorja].frequency)>500)
		{
        	stop_motor(stevilka_motorja);
        	motors[stevilka_motorja].frequency=frequency2Bset;
        	run_motor(stevilka_motorja);
		}

        napaka_old=napaka;

        */

        return false;  // Regulacija poteka
    }
    // Tlak je prenizek - potrebno ga je zvišati
    else if((napaka < 0)) {

    	//if (motors[stevilka_motorja].direction!=motors[stevilka_motorja].direction_minus)direction_change(stevilka_motorja,motors[stevilka_motorja].direction_minus);
        //motors[stevilka_motorja].current_speed = (uint32_t)(kp * napaka *(1+2*(napaka<0)) * 100);  // Hitrost sorazmerna napaki
        //rabis ref hitrost pri znani frekvenci

        // Omeji hitrost na 0-100%
        //if(motors[stevilka_motorja].current_speed > motors[stevilka_motorja].max_speed) motors[stevilka_motorja].current_speed = motors[stevilka_motorja].max_speed;

        //speed maping:
        //frequency2Bset=motors[stevilka_motorja].current_speed*pumpa_max_frequency/500;//50000 je max frequency
        //if(frequency2Bset>pumpa_max_frequency)frequency2Bset=pumpa_max_frequency;

    	serial_print_string("\r\nNapaka: ");
    	serial_print_float(napaka);
    	serial_print_string("\r\n");

    	//frequency2Bset=kp*abs(napaka)*pumpa_max_frequency/5;//5 je približno max napaka pri kp=2
		//if(frequency2Bset>pumpa_max_frequency)frequency2Bset=pumpa_max_frequency;

    	if((-napaka)>toleranca)
    		{
    			float set=(-napaka/toleranca)*100.0;
    			frequency2Bset=(uint32_t) set;
    		}

    	else
    		{
    			stop_motor(stevilka_motorja);
			}

    	if(frequency2Bset>pumpa_max_frequency)frequency2Bset=pumpa_max_frequency;

        if (abs(frequency2Bset-motors[stevilka_motorja].frequency)>90)
		{
        	stop_motor(stevilka_motorja);
        	motors[stevilka_motorja].frequency=frequency2Bset;
	    	serial_print_string("\rFreq: ");
	    	serial_print_float(frequency2Bset);
	    	serial_print_string("\r\n");
        	run_motor(stevilka_motorja);
		}

        napaka_old=napaka;
        return false;  // Regulacija poteka
    }

    return false;
}

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
    __HAL_RCC_GPIOB_CLK_ENABLE();
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
	stop_motor(motor_number);
	motors[motor_number].direction=direction;
	HAL_GPIO_WritePin(motors[motor_number].direction_port, motors[motor_number].direction_pin, motors[motor_number].direction);
}

/**
  * @brief  Resets all the motors and configures max positions,
			then moves the motors to their starting positions
			(half of maximum).
  * @param  None
  * @retval None
  */
// STARO
void reset_motors(void) //mislim da bi moralo delati :)
{
	//motor 4 - pumpa (preskočil - nima reset)
	//motor 3 - J3 (trapezoidal thread axle)
	//motor 2 - J2 (snail)
	//motor 1 - J1 (pulley)

	for (uint8_t motor_num=0;motor_num<(num_of_motors-1);motor_num++)
	{

		motors[motor_num].reset_requested=true; //da ignorira pogoje za pozicijo (pri timerjih)
		motors[motor_num].reset_completed=false;

		direction_change(motor_num, motors[motor_num].direction_minus);
		motors[motor_num].current_speed=10; // rot/s - ni še; raje frekvenco!

		if (motors[motor_num].running==false)
		{
			run_motor(motor_num);
		}

		if(motors[motor_num].reset_requested)
		{
			while(!motors[motor_num].end_switch_triggered){}

			stop_motor(motor_num);

			motors[motor_num].position=0; //mogoče bi lahko offsetal start position da ne udari v limit switch

			direction_change(motor_num,motors[motor_num].direction_plus);

			motors[motor_num].reset_completed=true; //da lahko zdaj spremlja korake (pri timerjih)

			run_motor(motor_num);

			while(motors[motor_num].end_switch_triggered){}//prvo pocakamo da se umakne stran od stikala
			while(!motors[motor_num].end_switch_triggered){}//potem pocakamo da zadane kontra stikalo

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
// STARO
void move_to_starting_position(uint8_t motor_number)//POPRAVI (DA CALLAŠ END EFFECTOR)
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
  * @brief  Updates the global end effector coordinate from the known struct
			data of the motors.
  * @param  None
  * @retval None
  */
void update_global_coordinates(void) //PREGLEJ!!
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
{	// za stabilnost počaka za 10 zaporednih "1". Čudno, ampak nujno za predstavitev :)
	_Bool sw_value=HAL_GPIO_ReadPin(motors[motor_number].end_switch_port, motors[motor_number].end_switch_pin);

	uint8_t swtich_stable_cnt=0;
	uint8_t proc_cycle_cnt=0;
	uint8_t successful_press_limit=5;
	while((swtich_stable_cnt<successful_press_limit)&&(proc_cycle_cnt<100))
	{
		sw_value=HAL_GPIO_ReadPin(motors[motor_number].end_switch_port, motors[motor_number].end_switch_pin);

		if(sw_value==1)	swtich_stable_cnt++;
		else swtich_stable_cnt=0;

		proc_cycle_cnt++;
		//HAL_Delay(1);
	}
	return (swtich_stable_cnt>=successful_press_limit);
}


/**
  * @brief  Runs the chosen motor with the struct defined speed/frequency.
  * @param  motor_number: the number of the motor that should be ran (starting with index 0)
  * @retval None
  */
void run_motor(uint8_t motor_number)
{

	/*
	if(motor_number==2)
	{
	__HAL_RCC_TIM1_FORCE_RESET();
	asm("nop"); asm("nop");
	__HAL_RCC_TIM1_RELEASE_RESET();
	MX_TIM1_Init();
	HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}


		if(motor_number==1)
	{
	__HAL_RCC_TIM15_FORCE_RESET();
	asm("nop"); asm("nop");
	__HAL_RCC_TIM15_RELEASE_RESET();
	MX_TIM15_Init();
HAL_NVIC_ClearPendingIRQ(TIM15_IRQn);
//HAL_NVIC_EnableIRQ(TIM15_CC_IRQn);
//HAL_NVIC_EnableIRQ(TIM15_UP_IRQn);
HAL_TIM_PWM_Start_IT(&htim15, TIM_CHANNEL_2);
	}
*/



    // Stop PWM first
    //HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);
	//motors[motor_number].running=false;

	if (motors[motor_number].timer->State == HAL_TIM_STATE_RESET) {
	    // Timer not initialized, initialize it first
	    // You might need to call the specific MX_TIMx_Init function
	    return;
	}

	//prepreči premikanje v napačno smer
	if(!((motors[motor_number].direction==motors[motor_number].allowed_direction)||(motors[motor_number].allowed_direction==2))) return;

	if (motors[motor_number].running == false)
	{
		// Stop timer first
		//HAL_TIM_Base_Stop_IT(motors[motor_number].timer);
		//htim15.Instance->DIER &= ~(1);
		HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);
		HAL_TIM_Base_Stop(motors[motor_number].timer);
		HAL_TIM_Base_Stop_IT(motors[motor_number].timer);

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

		uint32_t prescaler = (timer_clock / ((frequency_hz+1) * 1000)) - 1;
		uint32_t period = 999;

		// Configure timer registers
		motors[motor_number].timer->Instance->PSC = prescaler;
		motors[motor_number].timer->Instance->ARR = period;

		// Clear update flag
		motors[motor_number].timer->Instance->SR &= ~TIM_SR_UIF;

		// Restart timer
		//HAL_TIM_Base_Start_IT(motors[motor_number].timer);
		//htim15.Instance->DIER |= TIM_DIER_UIE;
		//motors[motor_number].timer->Instance->CR1 |= TIM_CR1_CEN;

		/*
        IRQn_Type irqn;
        if      (motors[motor_number].timer->Instance == TIM1)  irqn = TIM1_UP_IRQn;
        else if (motors[motor_number].timer->Instance == TIM3)  irqn = TIM3_IRQn;
        else if (motors[motor_number].timer->Instance == TIM15) irqn = TIM15_IRQn;
        else if (motors[motor_number].timer->Instance == TIM12) irqn = TIM8_BRK_TIM12_IRQn;
        else goto skip_nvic;
        HAL_NVIC_SetPriority(irqn, 5, 0);
        HAL_NVIC_EnableIRQ(irqn);
        skip_nvic:;
        */

		//__HAL_TIM_MOE_ENABLE(motors[motor_number].timer);
		//motors[motor_number].timer->State = HAL_TIM_STATE_READY;

		HAL_TIM_Base_Start(motors[motor_number].timer);
		//HAL_TIM_PWM_Start(motors[motor_number].timer, motors[motor_number].timer_channel);
		HAL_TIM_PWM_Start_IT(motors[motor_number].timer, motors[motor_number].timer_channel);
		HAL_TIM_Base_Start_IT(motors[motor_number].timer);
		motors[motor_number].timer->Instance->DIER |= TIM_DIER_UIE; // force it

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
	HAL_TIM_PWM_Stop(motors[motor_number].timer, motors[motor_number].timer_channel);
	//HAL_TIM_Base_Stop(motors[motor_number].timer);
	HAL_TIM_Base_Stop_IT(motors[motor_number].timer);
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

/*
void process_encoder(uint8_t encoder_number, _Bool A, _Bool B, _Bool Z)
{//POPRAVI
//vrednost 0 do max (začne se šele po kalibraciji z end switchi)

	static _Bool A_prej = 0;
	static _Bool B_prej = 0;
	static _Bool Z_prej = 0;

	if (!A_prej && A)
	{
		motors[encoder_number].encoder_A_state+=(!motors[encoder_number].direction)*(-1)+motors[encoder_number].direction;

		if (!A_prej && A && B_prej && (motors[encoder_number].direction))
		{
			motors[encoder_number].direction=motors[encoder_number].direction_plus;
		}
	}
	if (!B_prej && B)
	{
		motors[encoder_number].encoder_B_state+=(!motors[encoder_number].direction)*(-1)+motors[encoder_number].direction;

		if (!B_prej && B && A_prej && !(motors[encoder_number].direction))
		{
			motors[encoder_number].direction=motors[encoder_number].direction_minus;
		}
	}
	if (!Z_prej && Z)
	{
		motors[encoder_number].num_turns_from_encoder+=(!motors[encoder_number].direction)*(-1)+motors[encoder_number].direction;
		motors[encoder_number].encoder_B_state=0;
		motors[encoder_number].encoder_A_state=0;
	}

	//if (motors[encoder_number].encoder_A_state>encoder_maximum)motors[encoder_number].encoder_A_state=0;

	A_prej=A;
	B_prej=B;
	Z_prej=Z;
}
*/
void test_motor(uint8_t motor_number)
{
	//if(motors[motor_number].running)
	//{
		stop_motor(motor_number);
		HAL_Delay(1000);
		direction_change(motor_number,motors[motor_number].direction_minus);
		HAL_Delay(500);
		run_motor(motor_number);
		HAL_Delay(200);
		stop_motor(motor_number);
		HAL_Delay(1000);
		direction_change(motor_number,motors[motor_number].direction_plus);
		HAL_Delay(500);
		run_motor(motor_number);
		HAL_Delay(200);
		stop_motor(motor_number);
	//}
}

/**
  * @brief  Runs the 3 segment motors at the same time for the 3 seconds in each direction.
  * @param  None
  * @retval None
  */
void test_all_motors()
{
	stop_motor(0);
	stop_motor(1);
	stop_motor(2);
	HAL_Delay(1000);
	direction_change(0,motors[0].direction_minus);
	direction_change(1,motors[1].direction_minus);
	direction_change(2,motors[2].direction_minus);
	HAL_Delay(500);
	run_motor(0);
	run_motor(1);
	run_motor(2);
	HAL_Delay(1000);
	stop_motor(0);
	stop_motor(1);
	stop_motor(2);
	HAL_Delay(1000);
	direction_change(0,motors[0].direction_plus);
	direction_change(1,motors[1].direction_plus);
	direction_change(2,motors[2].direction_plus);
	HAL_Delay(500);
	run_motor(0);
	run_motor(1);
	run_motor(2);
	HAL_Delay(1000);
	stop_motor(0);
	stop_motor(1);
	stop_motor(2);
}


/**
 * @brief Uradno zaporedje za nalaganje nastavitev iz vl53l0x_tuning.h
 */
void VL53L0X_LoadTuningSettings(void) {
    // Definiramo tabelo kot statično, da ne obremenjujemo sklada (stack)
    static const uint8_t settings[][2] = {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
        {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
        {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
        {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
        {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
        {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
        {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
        {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
        {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
        {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
        {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
    };

    for(uint32_t i = 0; i < (sizeof(settings)/sizeof(settings[0])); i++) {
            uint8_t reg = settings[i][0];
            uint8_t data = settings[i][1];
            if (HAL_I2C_Mem_Write(&hi2c4, VL53L0X_ADDR, reg, 1, &data, 1, 10) != HAL_OK) {
                break; // Prekini ob napaki
            }
        }
}


void I2C4_BusRecovery(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // 1. Nastavi SCL (PD12) kot navaden izhodni pin v Open-Drain načinu
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 2. Nastavi SDA (PD13) kot vhod, da preveriš, če ga senzor drži nizko
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 3. Pošlji 9 urinih taktov (tako se sprosti I2C vodilo, če je senzor obtičal sredi branja)
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 4. Majhen zamik, da se linije umirijo
    HAL_Delay(5);
}
/*
void I2C4_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Reconfigure both pins as open-drain GPIO
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13; // SCL, SDA
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(1);

    // If SDA is stuck low, clock SCL up to 9 times
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET) // SDA pin
    {
        for (int i = 0; i < 9; i++)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // SCL low
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // SCL high
            HAL_Delay(1);
            if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
                break;
        }
    }

    // Generate a STOP condition: SDA low->high while SCL is high
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // SDA low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // SCL high
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // SDA high (STOP)
    HAL_Delay(1);
}
*/
static void MX_I2C4_Init(void) {
    // 1. Enable clocks FIRST
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_I2C4_CLK_ENABLE();
    __DSB();  /* <-- add after the clock enables */
    __ISB();

    // 2. Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 3. Then init the peripheral
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x00D0C7FF;//0x10808DD3
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_DeInit(&hi2c4);
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
        Error_Handler();
    }

    // 4. Optional noise filter
    HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0x0F);
}
/* ================================================================
 * VL53L0X minimal correct driver + periodic TIM interrupt
 * Target: STM32H750B-DK, I2C4, PD12=SCL, PD13=SDA
 * ================================================================ */

/* ---- register helpers (keep these as before) ---- */
static HAL_StatusTypeDef vl_write(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c4, VL53L0X_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 50);
}
static HAL_StatusTypeDef vl_read(uint8_t reg, uint8_t *out) {
    return HAL_I2C_Mem_Read(&hi2c4, VL53L0X_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, out, 1, 50);
}
static HAL_StatusTypeDef vl_read16(uint8_t reg, uint16_t *out) {
    uint8_t b[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(&hi2c4, VL53L0X_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, b, 2, 50);
    *out = ((uint16_t)b[0] << 8) | b[1];
    return s;
}

void VL53L0X_Diagnose(void) {
    uint8_t val;
    uint16_t val16;
    char buf[96];   // was 60 — must be at least 80, use 96 for safety

    if (HAL_I2C_IsDeviceReady(&hi2c4, VL53L0X_ADDR, 3, 50) != HAL_OK) {
        serial_print_string("DIAG: sensor not responding on I2C\r\n");
        return;
    }
    //serial_print_string("DIAG: sensor alive on I2C\r\n");

    vl_read(0xC0, &val);
    //snprintf(buf, sizeof(buf), "DIAG: model ID = 0x%02X (expect 0xEE)\r\n", val);
    //serial_print_string(buf);

    vl_read(0x00, &val);
    //snprintf(buf, sizeof(buf), "DIAG: SYSRANGE_START = 0x%02X\r\n", val);
    //serial_print_string(buf);

    vl_read(0x13, &val);
    //snprintf(buf, sizeof(buf), "DIAG: interrupt status reg 0x13 = 0x%02X\r\n", val);
    //serial_print_string(buf);

    vl_read(0x14, &val);
    uint8_t range_status = (val >> 3) & 0x1F;
    //snprintf(buf, sizeof(buf), "DIAG: range status nibble = %u\r\n", range_status);
    //serial_print_string(buf);

    vl_read16(0x1E, &val16);
    //snprintf(buf, sizeof(buf), "DIAG: raw range 0x1E:0x1F = %u mm\r\n", val16);
    //serial_print_string(buf);

    HAL_Delay(200);
    vl_read(0x13, &val);
    //snprintf(buf, sizeof(buf), "DIAG: interrupt status after 200ms = 0x%02X\r\n", val);
    //serial_print_string(buf);

    vl_read16(0x1E, &val16);
    //snprintf(buf, sizeof(buf), "DIAG: range after 200ms = %u mm\r\n", val16);
    //serial_print_string(buf);

    vl_write(0x0B, 0x01);   // clear interrupt → sensor queues next measurement
}

/* ================================================================
 * SPAD calibration – must run once before ranging
 * Reads the factory-programmed SPAD count/type from NVM and
 * applies them so the analog front-end has correct sensitivity.
 * ================================================================ */
static void VL53L0X_PerformSPADCalibration(void) {
    uint8_t val, spad_count, spad_type_is_aperture;
    uint8_t ref_spad_map[6];

    vl_write(0x80, 0x01); vl_write(0xFF, 0x01); vl_write(0x00, 0x00);
    vl_write(0xFF, 0x06);
    vl_read(0x83, &val); vl_write(0x83, val | 0x04);
    vl_write(0xFF, 0x07); vl_write(0x81, 0x01);
    vl_write(0x80, 0x01); vl_write(0x94, 0x6B); vl_write(0x83, 0x00);

    uint32_t t0 = HAL_GetTick();
    do { vl_read(0x83, &val); } while (val == 0 && (HAL_GetTick()-t0) < 100);
    vl_write(0x83, 0x01);

    vl_read(0x92, &val);
    spad_count            = val & 0x7F;
    spad_type_is_aperture = (val >> 7) & 0x01;

    vl_write(0x81, 0x00); vl_write(0xFF, 0x06);
    vl_read(0x83, &val); vl_write(0x83, val & ~0x04);
    vl_write(0xFF, 0x01); vl_write(0x00, 0x01);
    vl_write(0xFF, 0x00); vl_write(0x80, 0x00);

    /* read the 6-byte SPAD map — plain polling, no cache ops needed */
    HAL_I2C_Mem_Read(&hi2c4, VL53L0X_ADDR, 0xB0,
                     I2C_MEMADD_SIZE_8BIT, ref_spad_map, 6, 50);

    uint8_t first_spad = spad_type_is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad || enabled == spad_count)
            ref_spad_map[i/8] &= ~(1 << (i%8));
        else if (ref_spad_map[i/8] & (1 << (i%8)))
            enabled++;
    }
    HAL_I2C_Mem_Write(&hi2c4, VL53L0X_ADDR, 0xB0,
                      I2C_MEMADD_SIZE_8BIT, ref_spad_map, 6, 50);
}

static HAL_StatusTypeDef VL53L0X_PerformRefCalibration(void) {
    uint8_t val;

    /* VHV */
    vl_write(0x01, 0x01);
    vl_write(0x80, 0x01); vl_write(0xFF, 0x01); vl_write(0x00, 0x00);
    vl_write(0x91, vl53_stop_variable);
    vl_write(0x00, 0x01); vl_write(0xFF, 0x00); vl_write(0x80, 0x00);
    vl_write(0x00, 0x01);                        /* single shot */
    uint32_t t0 = HAL_GetTick();
    do {
        vl_read(0x13, &val);
        if ((HAL_GetTick()-t0) > 500) return HAL_TIMEOUT;
    } while ((val & 0x07) == 0);
    vl_write(0x0B, 0x01); vl_write(0x00, 0x00); /* clear + stop */

    /* phase */
    vl_write(0x01, 0x02);
    vl_write(0x00, 0x01);
    t0 = HAL_GetTick();
    do {
        vl_read(0x13, &val);
        if ((HAL_GetTick()-t0) > 500) return HAL_TIMEOUT;
    } while ((val & 0x07) == 0);
    vl_write(0x0B, 0x01); vl_write(0x00, 0x00);

    vl_write(0x01, 0xE8);                        /* restore all sequence steps */
    return HAL_OK;
}

/* ================================================================
 * VL53L0X_Init
 * ================================================================ */
void VL53L0X_Init(void) {
    uint8_t val;

    if (HAL_I2C_IsDeviceReady(&hi2c4, VL53L0X_ADDR, 3, 50) != HAL_OK) {
        serial_print_string("VL53L0X not found\r\n");
        return;
    }

    /* wake-up handshake */
    vl_write(0x88, 0x00);
    vl_write(0x80, 0x01); vl_write(0xFF, 0x01); vl_write(0x00, 0x00);
    vl_read(0x91, &val);
    vl53_stop_variable = val;
    vl_write(0x00, 0x01); vl_write(0xFF, 0x00); vl_write(0x80, 0x00);

    /* disable MSRC + TCC limit checks */
    vl_write(0x60, 0x00);

    /* signal rate limit 0.25 MCPS */
    vl_write(0x44, 0x00); vl_write(0x45, 0x20);

    /* load tuning settings */
    VL53L0X_LoadTuningSettings();

    /* SPAD calibration */
    VL53L0X_PerformSPADCalibration();

    /* GPIO: interrupt on new sample ready, active low; clear any pending */
    vl_write(0x0A, 0x04);
    vl_read(0x84, &val); vl_write(0x84, val & ~0x10);
    vl_write(0x0B, 0x01);

    /* reference calibration */
    if (VL53L0X_PerformRefCalibration() != HAL_OK) {
        serial_print_string("VL53L0X ref cal FAILED\r\n");
        return;
    }

    /* start continuous ranging */
    vl_write(0x80, 0x01); vl_write(0xFF, 0x01); vl_write(0x00, 0x00);
    vl_write(0x91, vl53_stop_variable);
    vl_write(0x00, 0x01); vl_write(0xFF, 0x00); vl_write(0x80, 0x00);
    vl_write(0x00, 0x02);   /* SYSRANGE_START = continuous back-to-back */

    HAL_Delay(100);         /* wait for first measurement to complete */
    vl_write(0x0B, 0x01);  /* clear that first interrupt so pipeline is clean */

    //serial_print_string("VL53L0X init OK\r\n");
}


/* ================================================================
 * VL53L0X_ReadDistance
 * Returns distance in mm, or 0xFFFF if result is invalid/timeout.
 * Call this from your TIM interrupt (or poll it in main).
 * ================================================================ */
/* Non-blocking version - safe to call from a TIM interrupt */
uint16_t VL53L0X_ReadDistance(void) {
    uint8_t status, range_status;
    uint16_t distance;

    if (vl_read(0x13, &status) != HAL_OK) return 0xFFFF;
    if ((status & 0x07) == 0) return 0xFFFF;   /* not ready */

    vl_read(0x14, &range_status);
    range_status = (range_status >> 3) & 0x1F;

    vl_read16(0x1E, &distance);
    vl_write(0x0B, 0x01);   /* clear → triggers next measurement */

    /* status 0  = valid
       status 11 = VCSEL continuity test — also valid for this sensor/SPAD config
       everything else = real error */
    if (range_status != 0 && range_status != 11) return 0xFFFF;

    return distance;
}

/* ================================================================
 * Periodic TIM interrupt – configure ONE free timer (e.g. TIM6)
 *
 * TIM6 is a basic timer, perfect for this – no PWM pins needed.
 * Period = (PSC+1)*(ARR+1) / TIM_CLK
 * With TIM_CLK = 200 MHz (D2 domain after PLL1):
 *   PSC = 9999  → 20 kHz tick
 *   ARR = 1999  → interrupt every 100 ms  (10 Hz)
 * Adjust ARR to taste.
 * ================================================================ */

static void MX_TIM6_Init(void) {
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Force the RCC write to complete before touching TIM6 registers.
       Without this, the AHB write buffer can still be pending when
       HAL_TIM_Base_Init writes TIM6->CR1, causing an imprecise bus fault. */
    __DSB();
    __ISB();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 9999;   /* 200 MHz / 10000 = 20 kHz */
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period             = 1999;   /* 20 kHz / 2000 = 10 Hz = 100 ms */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&htim6);   /* start immediately */
}

/* IRQ handler – put this with your other IRQ handlers */
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}



/**
  * @brief  Runs the motors for a specific time to reach a predetermined point.
  * @param  None
  * @retval None
  */
void demo_za_predstavitev()
{
	uint16_t motor0_move_time=3000;
	uint16_t motor1_move_time=700;
	uint16_t motor2_move_time=16000;
	uint16_t poke_time=2000;

	//ne rabim za zdaj
	//calibrate_motor(2);

	//BRANJE STIKAL ZELO NESTABILNO
	//rabim:
	move_motor_2_end_switch(2, motors[2].direction_plus);
	//za test samo:
	//while(!read_switch(2)){}

	receive_target_point(&target_x_coordinate,&target_y_coordinate,&target_z_coordinate,&target_phi);


	direction_change(0, motors[0].direction_plus);
	run_motor(0);
	HAL_Delay(motor0_move_time);
	stop_motor(0);

	direction_change(1, motors[1].direction_plus);
	run_motor(1);
	HAL_Delay(motor1_move_time);
	stop_motor(1);

	direction_change(2, motors[2].direction_minus);
	run_motor(2);
	HAL_Delay(motor2_move_time);
	stop_motor(2);


	//Sporocimo, da smo prispeli do spik lege
	char vbod_msg[]="Spikanje?\r\n";
	uart_transmit(vbod_msg);

	//pocakamo, da uporabnik na rpi vnese odgovor
	char move_msg[30];
	uart_receive(move_msg);

	run_motor(2);
	HAL_Delay(poke_time);
	stop_motor(2);

	HAL_Delay(2000);

	direction_change(2, motors[2].direction_plus);
	run_motor(2);
	HAL_Delay(poke_time);
	stop_motor(2);

	run_motor(2);
	HAL_Delay(motor2_move_time);
	stop_motor(2);

	direction_change(1, motors[1].direction_minus);
	run_motor(1);
	HAL_Delay(motor1_move_time);
	stop_motor(1);

	direction_change(0, motors[0].direction_minus);
	run_motor(0);
	HAL_Delay(motor0_move_time);
	stop_motor(0);
}

//funkcija ki upravlja motor s pomočjo 4ih tipk
// speed up, slow down, CW, CCW
void test_tipke_na_roke()
{
	const uint8_t stevilka_tipka_zelena_1=0; //E3, D8 (zgoraj)
	const uint8_t stevilka_tipka_zelena_2=1; //H15, D9 (zgoraj)
	const uint8_t stevilka_tipka_rdeca_1=3;  //B4, D10 (zgoraj)
	const uint8_t stevilka_tipka_rdeca_2=2;  //I2, D12 (zgoraj)

	//pobere stabilizirane vrednosti tipk, bere "end switche" motorjev
	//samo po en trenutno v funkciji, kasneje bosta 2 (upam)
	_Bool stanje_tipka_zelena_1 = read_switch(stevilka_tipka_zelena_1);
	_Bool stanje_tipka_zelena_2 = read_switch(stevilka_tipka_zelena_2);
	_Bool stanje_tipka_rdeca_1 = read_switch(stevilka_tipka_rdeca_1);
	_Bool stanje_tipka_rdeca_2 = read_switch(stevilka_tipka_rdeca_2);

	static _Bool stanje_tipka_rdeca_1_prej=0;
	static _Bool stanje_tipka_rdeca_2_prej=0;

	const uint8_t stevilka_motorja=2;

	//static uint32_t speed=10000;

	// če spustimo tipko naprej (in motor laufa) ustavi motor
	if(!stanje_tipka_zelena_1 && motors[stevilka_motorja].running && !stanje_tipka_zelena_2)
	{
		stop_motor(stevilka_motorja);
	}
	//tipka naprej (če držiš tipko in motor ne laufa, ga zalaufa)
	else if(stanje_tipka_zelena_1 && !motors[stevilka_motorja].running && !stanje_tipka_zelena_2)
	{
		direction_change(stevilka_motorja,motors[stevilka_motorja].direction_plus);
		run_motor(stevilka_motorja);
	}
	//tipka nazaj (če držiš tipko in motor ne laufa, ga zalaufa)
	else if(stanje_tipka_zelena_2 && !motors[stevilka_motorja].running && !stanje_tipka_zelena_1)
	{
		direction_change(stevilka_motorja,motors[stevilka_motorja].direction_minus);
		run_motor(stevilka_motorja);
	}


	if(stanje_tipka_rdeca_1 && !stanje_tipka_rdeca_1_prej && !stanje_tipka_zelena_1 && !stanje_tipka_zelena_2)
	{
		if (motors[stevilka_motorja].frequency<50000)
		{
			motors[stevilka_motorja].frequency+=10000;
		}
		//funkcija za izpis stanja update na ekranu?
	}
	else if(stanje_tipka_rdeca_2 && !stanje_tipka_rdeca_2_prej && !stanje_tipka_zelena_1 && !stanje_tipka_zelena_2)
	{
		if (motors[stevilka_motorja].frequency>10000)
		{
			motors[stevilka_motorja].frequency-=10000;
		}
		else if (motors[stevilka_motorja].frequency>1000)
		{
			motors[stevilka_motorja].frequency-=1000;
		}
		//funkcija za izpis stanja update na ekranu?
	}

	stanje_tipka_rdeca_1_prej=stanje_tipka_rdeca_1;
	stanje_tipka_rdeca_2_prej=stanje_tipka_rdeca_2;

}

void test_tipke_na_roke2()
{
	static _Bool initialized=0;

	if (!initialized)
	{

	}


	const uint8_t stevilka_tipka_zelena_1=0; //E3, D8 (zgoraj)
	const uint8_t stevilka_tipka_zelena_2=1; //H15, D9 (zgoraj)
	const uint8_t stevilka_tipka_rdeca_1=3;  //B4, D10 (zgoraj)
	const uint8_t stevilka_tipka_rdeca_2=2;  //I2, D12 (zgoraj)

	//pobere stabilizirane vrednosti tipk, bere "end switche" motorjev
	//samo po en trenutno v funkciji, kasneje bosta 2 (upam)
	_Bool stanje_tipka_zelena_1 = read_switch(stevilka_tipka_zelena_1);
	_Bool stanje_tipka_zelena_2 = read_switch(stevilka_tipka_zelena_2);
	_Bool stanje_tipka_rdeca_1 = read_switch(stevilka_tipka_rdeca_1);
	_Bool stanje_tipka_rdeca_2 = read_switch(stevilka_tipka_rdeca_2);

	static _Bool stanje_tipka_rdeca_1_prej=0;
	static _Bool stanje_tipka_rdeca_2_prej=0;

	const uint8_t stevilka_motorja=2;

	//static uint32_t speed=10000;

	// če spustimo tipko naprej (in motor laufa) ustavi motor
	if(!stanje_tipka_zelena_1 && motors[stevilka_motorja].running && !stanje_tipka_zelena_2)
	{
		stop_motor(stevilka_motorja);
	}
	//tipka naprej (če držiš tipko in motor ne laufa, ga zalaufa)
	else if(stanje_tipka_zelena_1 && !motors[stevilka_motorja].running && !stanje_tipka_zelena_2)
	{
		direction_change(stevilka_motorja,motors[stevilka_motorja].direction_plus);
		run_motor(stevilka_motorja);
	}
	//tipka nazaj (če držiš tipko in motor ne laufa, ga zalaufa)
	else if(stanje_tipka_zelena_2 && !motors[stevilka_motorja].running && !stanje_tipka_zelena_1)
	{
		direction_change(stevilka_motorja,motors[stevilka_motorja].direction_minus);
		run_motor(stevilka_motorja);
	}


	if(stanje_tipka_rdeca_1 && !stanje_tipka_rdeca_1_prej && !stanje_tipka_zelena_1 && !stanje_tipka_zelena_2)
	{
		if (motors[stevilka_motorja].frequency<50000)
		{
			motors[stevilka_motorja].frequency+=10000;
		}
		//funkcija za izpis stanja update na ekranu?
	}
	else if(stanje_tipka_rdeca_2 && !stanje_tipka_rdeca_2_prej && !stanje_tipka_zelena_1 && !stanje_tipka_zelena_2)
	{
		if (motors[stevilka_motorja].frequency>10000)
		{
			motors[stevilka_motorja].frequency-=10000;
		}
		else if (motors[stevilka_motorja].frequency>1000)
		{
			motors[stevilka_motorja].frequency-=1000;
		}
		//funkcija za izpis stanja update na ekranu?
	}

	stanje_tipka_rdeca_1_prej=stanje_tipka_rdeca_1;
	stanje_tipka_rdeca_2_prej=stanje_tipka_rdeca_2;
}

/**
  * @brief  Function for moving the segment to the end switch in the desired direction.
  * @param  motor_number: number of the motor that should be calibrated
  *			direction: direction of the segment movement
  * @retval None
  */
void move_motor_2_end_switch(uint8_t motor_number, _Bool direction)
{
	//if(!read_switch(motor_number))
	//{
		direction_change(motor_number,direction);
		run_motor(motor_number);
		while(!read_switch(motor_number)){}
		stop_motor(motor_number);
	//}
}

/**
  * @brief  Function for calibrating the chosen motor by reaching both end switches.
  *         Both switches are wired to the same input pin per motor.
  *         Procedure:
  *           1. Move in direction_minus until switch triggers (switch 1 hit).
  *           2. Reverse to direction_plus, run for at least 1 second blind
  *              (do not read switch) to clear the switch mechanically.
  *           3. Count steps while moving until the switch triggers again (switch 2 hit).
  *           4. The step count between the two switch events = max_position.
  *           5. Update unit_conversion and move to centre.
  *
  *         WARNING: If the mechanism starts already pressing a switch, it cannot
  *         determine which end it is at. Move it away from the switch manually
  *         before calling this function.
  *
  * @param  motor_number: number of the motor to calibrate (0, 1 or 2 — motor 3 unused)
  * @retval None
  */
void calibrate_motor(uint8_t motor_number)
{
	char msg[80];

	/* --- Safety check: switch already pressed at startup --- */
	if (read_switch(motor_number))
	{
		snprintf(msg, sizeof(msg), "CAL M%d: ERR - switch pressed at start, move away manually\r\n", motor_number);
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
		stop_motor(motor_number);
		return; /* Do not freeze — return so caller can handle it */
	}

	snprintf(msg, sizeof(msg), "CAL M%d: moving to switch 1 (dir_minus)...\r\n", motor_number);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	/* --- Phase 1: drive in direction_minus until switch 1 triggers --- */
	motors[motor_number].position = 0;
	motors[motor_number].end_switch_triggered = 0;
	motors[motor_number].allowed_direction = 2; /* allow both directions during calibration */

	direction_change(motor_number, motors[motor_number].direction_minus);
	HAL_Delay(100);
	run_motor(motor_number);

	while (!read_switch(motor_number)) { /* spin until switch fires */ }

	stop_motor(motor_number);

	snprintf(msg, sizeof(msg), "CAL M%d: switch 1 hit. Reversing...\r\n", motor_number);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	/* --- Phase 2: reverse direction and move BLIND for at least 1 second --- */
	direction_change(motor_number, motors[motor_number].direction_plus);
	HAL_Delay(100);

	motors[motor_number].position = 0; /* reset step counter at switch-1 position */
	motors[motor_number].end_switch_triggered = 0;

	run_motor(motor_number);

	/* Mandatory blind travel: do NOT read the switch for the first 1000 ms.
	   This ensures the mechanism has physically cleared the switch actuator. */
	HAL_Delay(1000);

	snprintf(msg, sizeof(msg), "CAL M%d: blind phase done, counting steps to switch 2...\r\n", motor_number);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	/* --- Phase 3: continue until switch 2 triggers, counting steps --- */
	/* position is incremented by the TIM IRQ callback while running */
	while (!read_switch(motor_number)) { /* spin until switch fires */ }
	uint32_t measured_steps;

	stop_motor(motor_number);

	measured_steps = motors[motor_number].position;

	snprintf(msg, sizeof(msg), "CAL M%d: switch 2 hit. Steps between switches: %lu\r\n",
	         motor_number, measured_steps);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	/* Sanity check: if we measured almost nothing the count is wrong */
	if (measured_steps < 100)
	{
		snprintf(msg, sizeof(msg), "CAL M%d: ERR - step count too low (%lu), aborting\r\n",
				 motor_number, measured_steps);
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	/* --- Phase 4: update motor parameters --- */
	motors[motor_number].max_position    = measured_steps;
	motors[motor_number].unit_conversion = measured_steps / motors[motor_number].travel_length;
	motors[motor_number].end_switch_triggered = 0;
	motors[motor_number].allowed_direction    = 2;

	snprintf(msg, sizeof(msg), "CAL M%d: max_pos=%lu, unit_conv=%lu. Moving to centre...\r\n",
			 motor_number,
			 motors[motor_number].max_position,
			 motors[motor_number].unit_conversion);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);


	/* --- Phase 5: drive back to the centre of the travel range --- */
	direction_change(motor_number, motors[motor_number].direction_minus);
	HAL_Delay(100);
	run_motor(motor_number);

	if(motor_number!=2)
	{
		while (motors[motor_number].position > (motors[motor_number].max_position / 2)) { /* wait */ }
	}
	else
	{
		while (motors[motor_number].position > (motors[motor_number].max_position * 1 / 10)) { /* wait */ }
	}

	stop_motor(motor_number);

	motors[motor_number].starting_position = motors[motor_number].position;
	motors[motor_number].unit_conversion=motors[motor_number].max_position/motors[motor_number].travel_length;

	snprintf(msg, sizeof(msg), "CAL M%d: done. Centre pos=%lu\r\n",
			 motor_number, motors[motor_number].position);
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

}

/**
  * @brief  Calibrates motors 0, 1 and 2 sequentially.
  *         Motor 3 (pump) is intentionally skipped — it has no end switches.
  *         Each motor is driven to its first end switch, then in reverse for
  *         at least 1 second (blind phase) before the second switch is detected
  *         and the full travel range is measured in steps.
  *         Results stored in motors[n].max_position and motors[n].unit_conversion.
  * @retval None
  */
void calibrate_all_motors(void)
{
	serial_print_string("CALIBRATION: starting motors 0, 1, 2...\r\n");

	for (uint8_t m = 0; m < 3; m++)
	{
		motors[m].reset_requested=1;
		serial_print_string("-----------------------\r\n");
		calibrate_motor(m);
		motors[m].reset_requested=0;
		HAL_Delay(500); /* short pause between motors */
	}

	serial_print_string("CALIBRATION: Linear Actuator\r\n");

	if(!read_switch(3))DC_Motor_Set_Speed(600);

	while(!read_switch(3)){}

	DC_Motor_Set_Speed(0);

	serial_print_string("CALIBRATION: complete.\r\n");

	for (uint8_t m = 0; m <= 2; m++) {
	    // Sredinska pozicija v korakih postane referenčna ničla ("Home")
	    motors[m].home_position = motors[m].position;
	}

	target_x = 0;
	target_y = 0;
	target_o = 0;
}

/**
  * @brief  Timer interrupt function for incrementing the position
			of the stepper motor.
  * @param  htim: pointer to the designated timer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // ---------- MOTOR 0 (TIM3) ----------
    if (htim->Instance == TIM3)
    {
        if (motors[0].running == true)
        {
            // Inkrement/Dekrement pozicije glede na trenutno smer
            if (motors[0].direction == motors[0].direction_plus && motors[0].position<motors[0].max_position)
            {
                motors[0].position += 1;
            }
            else if (motors[0].direction == motors[0].direction_minus && motors[0].position>0)
			{
                motors[0].position -= 1;
            }

            // Posodobitev globalnih koordinat za end-effector (opcijsko sproti)
            effector_x = motors[0].position / motors[0].unit_conversion;

            // DOSEŽEN CILJ: Če smo prispeli na target_position, ustavi motor!
            if (!motors[0].reset_requested && (motors[0].position == motors[0].target_position))
            {
                stop_motor(0);
            }
        }
    }

    // ---------- MOTOR 1 (TIM15) ----------
    else if (htim->Instance == TIM15)
    {
        if (motors[1].running == true)
        {
            if (motors[1].direction == motors[1].direction_plus && motors[1].position<motors[1].max_position)
            {
                motors[1].position += 1;
            }
            else if (motors[1].direction == motors[1].direction_minus && motors[1].position>0)
			{
                motors[1].position -= 1;
            }

            effector_y = motors[1].position / motors[1].unit_conversion;

            // DOSEŽEN CILJ
            if (!motors[1].reset_requested && (motors[1].position == motors[1].target_position))
            {
                stop_motor(1);
            }
        }
    }

    // ---------- MOTOR 2 (TIM1) ----------
    else if (htim->Instance == TIM1)
    {
        if (motors[2].running == true)
        {
            if (motors[2].direction == motors[2].direction_plus && motors[2].position<motors[2].max_position)
            {
                motors[2].position += 1;
            }
            else if (motors[2].direction == motors[2].direction_minus && motors[2].position>0)
			{
                motors[2].position -= 1;
            }

            effector_orientation = motors[2].position / mapFloat(motors[2].unit_conversion, 0, 1, 0, 1); // oz. vaša pretvorba stopinj

            // DOSEŽEN CILJ
            if (!motors[2].reset_requested && (motors[2].position == motors[2].target_position))
            {
                stop_motor(2);
            }
        }
    }

	//---------pumpa------------
	else if (htim->Instance == TIM12) {
	if (motors[3].running == true)
        {
		motors[3].position += 1;
		}
	}

	//----------tof---------
    else if (htim->Instance == TIM6) {
            vl53_data_ready  = 1;
    }
}
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t end_switch_reset_cnt[]={0,0,0};//[M0,M1,M2]


    if (htim->Instance == TIM3)
    {
		if (!motors[0].reset_requested&&(((motors[0].position==motors[0].max_position && motors[0].direction==motors[0].direction_plus) || (motors[0].position==0 && motors[0].direction==motors[0].direction_minus)) && !motors[0].reset_requested ) && motors[0].running==true)
		{
			stop_motor(0);
			motors[0].running=false;
		}
		else if (motors[0].running==true && motors[0].direction==motors[0].direction_plus)
		{
			motors[0].position += 1;
			J1_offset_mm=motors[0].position*motors[0].unit_conversion/motors[0].num_steps_per_turn;
		}
		else if (motors[0].running==true && motors[0].direction==motors[0].direction_minus)
		{
			if (motors[0].position > 0) motors[0].position -= 1;
			J1_offset_mm=motors[0].position*motors[0].unit_conversion/motors[0].num_steps_per_turn;
		}

		if (motors[0].end_switch_triggered)
		{
			if (read_switch(0))
			{
				end_switch_reset_cnt[0]=0;
			}
			else if(!read_switch(0))
			{
				end_switch_reset_cnt[0]++;
			}

			if (end_switch_reset_cnt[0]>1000)
			{
				motors[0].end_switch_triggered=0;
			}
		}

	}
    else if (htim->Instance == TIM15)
    {
		if (!motors[1].reset_requested&&(((motors[1].position==motors[1].max_position && motors[1].direction==motors[1].direction_plus) || (motors[1].position==0 && motors[1].direction==motors[1].direction_minus)) && !motors[1].reset_requested ) && motors[1].running==true)
		{
			stop_motor(1);
			motors[1].running=false;
		}
		else if (motors[1].running==true && motors[1].direction==motors[1].direction_plus)
		{
			motors[1].position += 1;
			//J2_offset_deg=motors[1].position*motors[1].unit_conversion/motors[1].num_steps_per_turn;
		}
		else if (motors[1].running==true && motors[1].direction==motors[1].direction_minus)
		{
			if (motors[1].position > 0) motors[1].position -= 1;
			J2_offset_deg=motors[1].position*motors[1].unit_conversion/motors[1].num_steps_per_turn;
		}

		if (motors[1].end_switch_triggered)
		{
			if (read_switch(1))
			{
				end_switch_reset_cnt[1]=0;
			}
			else if(!read_switch(1))
			{
				end_switch_reset_cnt[1]++;
			}

			if (end_switch_reset_cnt[1]>1000)
			{
				motors[1].end_switch_triggered=0;
			}
		}
    }
    else if (htim->Instance == TIM1)
    {
		if (!motors[2].reset_requested&&(((motors[2].position==motors[2].max_position && motors[2].direction==motors[2].direction_plus) || (motors[2].position==0 && motors[2].direction==motors[2].direction_minus)) && !motors[2].reset_requested ) && motors[2].running==true)
		{
			stop_motor(2);
			motors[2].running=false;
		}
		else if (motors[2].running==true && motors[2].direction==motors[2].direction_plus)
		{
			motors[2].position += 1;
			J3_offset_mm=J3_offset_base+motors[2].position*motors[2].unit_conversion/motors[2].num_steps_per_turn;
		}
		else if (motors[2].running==true && motors[2].direction==motors[2].direction_minus)
		{
			if (motors[2].position > 0) motors[2].position -= 1;
			J3_offset_mm=J3_offset_base+motors[2].position*motors[2].unit_conversion/motors[2].num_steps_per_turn;
		}

		if (motors[2].end_switch_triggered)
		{
			if (read_switch(2))
			{
				end_switch_reset_cnt[2]=0;
			}
			else if(!read_switch(2))
			{
				end_switch_reset_cnt[2]++;
			}

			if (end_switch_reset_cnt[2]>1000)
			{
				motors[2].end_switch_triggered=0;
			}
		}
    }
    else if (htim->Instance == TIM12)
    {

		if (motors[3].running==true && motors[3].direction==motors[3].direction_plus)
		{
			motors[3].position += 1;
			J4_ammount_of_liquid=J4_volume_per_turn*motors[3].position/motors[3].num_steps_per_turn;
		}
		else if (motors[3].running==true && motors[3].direction==motors[3].direction_minus)
		{
			if (motors[3].position > 0) motors[3].position -= 1;
		}
    }
}*/



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {

    	        if (motors[2].running == true)
    	        {
    	            if (motors[2].direction == motors[2].direction_plus && motors[2].position<motors[2].max_position)
    	            {
    	                motors[2].position += 1;
    	            }
    	            else if (motors[2].direction == motors[2].direction_minus && motors[2].position>0)
    				{
    	                motors[2].position -= 1;
    	            }

    	            effector_orientation = motors[2].position / mapFloat(motors[2].unit_conversion, 0, 1, 0, 1); // oz. vaša pretvorba stopinj

    	            // DOSEŽEN CILJ
    	            if (!motors[2].reset_requested && (motors[2].position == motors[2].target_position))
    	            {
    	                stop_motor(2);
    	            }
    	        }

    }




    if (htim->Instance == TIM15) {
    	        if (motors[1].running == true)
    	        {
    	            if (motors[1].direction == motors[1].direction_plus && motors[1].position<motors[1].max_position)
    	            {
    	                motors[1].position += 1;
    	            }
    	            else if (motors[1].direction == motors[1].direction_minus && motors[1].position>0)
    				{
    	                motors[1].position -= 1;
    	            }

    	            effector_y = motors[1].position / motors[1].unit_conversion;

    	            // DOSEŽEN CILJ
    	            if (!motors[1].reset_requested && (motors[1].position == motors[1].target_position))
    	            {
    	                stop_motor(1);
    	            }
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
    if((motors[2].num_steps_for_switch_release<motors[2].position && motors[2].position<(motors[2].max_position-motors[2].num_steps_for_switch_release))&&(motors[2].end_switch_triggered)&&(!read_switch(2)))
	{
		motors[2].end_switch_triggered=0;
		motors[2].allowed_direction=2;
	}
    if (motors[2].running == true)
    {
        if (motors[2].direction == motors[2].direction_plus && motors[2].position<motors[2].max_position)
        {
            motors[2].position += 1;
        }
        else if (motors[2].direction == motors[2].direction_minus && motors[2].position>0)
		{
            motors[2].position -= 1;
        }

        effector_orientation = motors[2].position / mapFloat(motors[2].unit_conversion, 0, 1, 0, 1); // oz. vaša pretvorba stopinj

        // DOSEŽEN CILJ
        if (!motors[2].reset_requested && (motors[2].position == motors[2].target_position))
        {
            stop_motor(2);
        }
    }
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */


void TIM15_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim15);
	if((motors[1].num_steps_for_switch_release<motors[1].position && motors[1].position<(motors[1].max_position-motors[1].num_steps_for_switch_release))&&(motors[1].end_switch_triggered)&&(!read_switch(1)))
	{
		motors[1].end_switch_triggered=0;
		motors[1].allowed_direction=2;
	}
    if (motors[1].running == true)
    {
        if (motors[1].direction == motors[1].direction_plus && motors[1].position<motors[1].max_position)
        {
            motors[1].position += 1;
        }
        else if (motors[1].direction == motors[1].direction_minus && motors[1].position>0)
		{
            motors[1].position -= 1;
        }

        effector_y = motors[1].position / motors[1].unit_conversion;

        // DOSEŽEN CILJ
        if (!motors[1].reset_requested && (motors[1].position == motors[1].target_position))
        {
            stop_motor(1);
        }
    }
}

/**
  * @brief  Makes the timer IRQ handler readable for the specific timer.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
	if((motors[0].num_steps_for_switch_release<motors[0].position && motors[0].position<(motors[0].max_position-motors[0].num_steps_for_switch_release))&&(motors[0].end_switch_triggered)&&(!read_switch(0)))
	{
		motors[0].end_switch_triggered=0;
		motors[0].allowed_direction=2;
	}
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

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
    if(htim_pwm->Instance == TIM8)
    {
        /* Enable Peripheral Clocks */
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();

        /* PH15 GPIO configuration happens here if using strict CubeMX structure */
    }
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

//ne rabim:
/*
void uart_process_command(const char* command) {
	if (strncmp(command, "x=", 2) == 0) {
	        int32_t val = (int32_t)atoi(command + 2);

	        // Preverjanje Bounding Boxa za X
	        if (val >= robot_bbox.min_x && val <= robot_bbox.max_x) {
	            target_x = val;
	            uart_print_current_targets();
	        } else {
	            uart_transmit("ERROR: X izven Bounding Boxa!\r\n");
	        }
	    }
	    else if (strncmp(command, "y=", 2) == 0) {
	        int32_t val = (int32_t)atoi(command + 2);

	        // Y mora biti pozitiven in znotraj Bounding Boxa
	        if (val < 0) {
	            uart_transmit("ERROR: Y mora biti pozitiven!\r\n");
	        } else if (val >= robot_bbox.min_y && val <= robot_bbox.max_y) {
	            target_y = val;
	            uart_print_current_targets();
	        } else {
	            uart_transmit("ERROR: Y izven Bounding Boxa!\r\n");
	        }
	    }
	    else if (strncmp(command, "O=", 2) == 0) {
	        int32_t val = (int32_t)atoi(command + 2);

	        // Omejitev orientacije na +- 30 stopinj
	        if (val >= -30 && val <= 30) {
	            target_o = val;
	            uart_print_current_targets();
	        } else {
	            uart_transmit("ERROR: Orientacija izven dovoljenega obmocja (+/- 30 st.)! Vnesi veljavne podatke.\r\n");
	        }
	    }
	    else if (strcmp(command, "go") == 0) {
	        uart_transmit("EXEC: Zacetek premika...\r\n");

	        // Pokličemo izvedbo giba
	        execute_robot_movement();
	    }
}
*/
//konc ne rabim

void execute_robot_movement(void)
{
    // Pomožna spremenljivka za časovno osveževanje izpisa (v milisekundah)
    uint32_t last_print_tick = 0;
    const uint32_t print_interval = 200; // Osveževanje na 200 ms (5-krat na sekundo)

    //uint32_t d_target = sqrt((target_x-current_x)^2+(target_y-current_y)^2);

    // 1. IZRAČUN CILJNIH KORAKOV GLEDE NA HOME POZICIJO
    //motors[0].target_position = (uint32_t)((int32_t)motors[0].home_position + (int32_t)(target_x * motors[0].unit_conversion));
    //motors[1].target_position = (uint32_t)((int32_t)motors[1].home_position + (int32_t)(target_o * motors[1].unit_conversion));
    //motors[2].target_position = (uint32_t)((int32_t)motors[2].home_position + (int32_t)(target_y * motors[2].unit_conversion));

    //float temp_target= target_x-cos((90-target_o)*PI/180)*izteg;//lokacija vozicka


    //izračun potrebnih kotov
    //izteg=sqrt(pow(target_y/cos(target_o*PI/180),2))-motors[2].offset;
    izteg=sqrt(pow(target_y/cos(target_o*PI/180),2))+motors[2].offset+igla_sklop_offset;

    //target_x,target_y,target_o=tocka!!! ločeno se spremeni v premik motorja
    motors[0].target_position = (int32_t)((target_x-cos((90+target_o)*PI/180)*izteg) * motors[0].unit_conversion+motors[0].max_position/2);

    motors[1].target_position = (int32_t)(target_o * motors[1].unit_conversion+motors[1].max_position/2);

    //staro://motors[2].target_position = (int32_t)((((target_y-motors[2].offset)/sin((90-target_o)*PI/180))) * motors[2].unit_conversion);
    motors[2].target_position = (int32_t)((((target_y)/sin((90-target_o)*PI/180))) * motors[2].unit_conversion+motors[2].home_position);


    // Varnostna omejitev (Saturation), da ne prebijemo kalibracijskih meja
    /*
    for (uint8_t m = 0; m <= 2; m++) {
        if ((int32_t)motors[m].target_position < 0) {
            motors[m].target_position = 0;
        }
        if (motors[m].target_position > motors[m].max_position) {
            motors[m].target_position = motors[m].max_position;
        }
    }
    */

    // 2. NASTAVITEV SMERI
    for (uint8_t m = 0; m <= 2; m++)
    {
        if (motors[m].target_position > motors[m].position) {
            direction_change(m, motors[m].direction_plus);
        }
        else if (motors[m].target_position < motors[m].position) {
            direction_change(m, motors[m].direction_minus);

        }
    }



    // 3. SOČASNI ZAGON: Motor 0 in Motor 1 (X in O)

    serial_print_string("Zagon M0 in M1\r\n");
    if (motors[0].position != motors[0].target_position) run_motor(0);
    if (motors[1].position != motors[1].target_position) run_motor(1);

    // ČAKANJE IN KONSTANTEN IZPIS MED PREMIKOM X IN Y
    while (motors[0].running || motors[1].running)
    {
        // Varnostni izhod ob sprožitvi stikal
        if (motors[0].end_switch_triggered || motors[1].end_switch_triggered || next_step==0) {
            stop_all_motors();
            serial_print_string("\r\nALERT: Koncno stikalo sprozeno med premikom X/O!\r\n");
            return;
        }


        // Periodični izpis trenutne lege vrha robota
        if (HAL_GetTick() - last_print_tick >= print_interval) {
            uart_print_current_targets();
            last_print_tick = HAL_GetTick();
        }



        if((motors[0].position>motors[0].target_position && motors[0].direction==motors[0].direction_plus)||(motors[0].position<motors[0].target_position && motors[0].direction==motors[0].direction_minus))
        	{
        		stop_motor(0);
        	}

        if((motors[1].position>motors[1].target_position && motors[1].direction==motors[1].direction_plus)||(motors[1].position<motors[1].target_position && motors[1].direction==motors[1].direction_minus))
        	{
        		stop_motor(1);
        	}

    }

    // 4. SEKVENČNI ZAGON: Motor 2 (Y), ko prva dva zaključita

    serial_print_string("Zagon M2\r\n");
    if (motors[2].position != motors[2].target_position)
    {
        run_motor(2);

        // ČAKANJE IN KONSTANTEN IZPIS MED PREMIKOM ORIENTACIJE
        while (motors[2].running)
        {
            if (motors[2].end_switch_triggered || next_step==0) {
                stop_motor(2);
                serial_print_string("\r\nALERT: Koncno stikalo sprozeno med premikom izteg!\r\n");
                return;
            }

            // Periodični izpis trenutne lege vrha robota
            if (HAL_GetTick() - last_print_tick >= print_interval) {
                uart_print_current_targets();
                last_print_tick = HAL_GetTick();
            }

            if((motors[2].position>motors[2].target_position && motors[2].direction==motors[2].direction_plus)||(motors[2].position<motors[2].target_position && motors[2].direction==motors[2].direction_minus))stop_motor(2);

        }
    }

    // Končni izpis ob uspešnem prihodu v točko
    serial_print_string("\r\nINFO: Premik uspesno zakljucen. Dosezena koncna tocka.\r\n");
    uart_print_current_targets();

    premik_done=1;
    serial_print_string("\r\n");
    serial_print_string("MERITEV 1 STOP");
    serial_print_string("\r\n");
	serial_print_string("MERITEV 1 STOP");
	serial_print_string("\r\n");
	serial_print_string("MERITEV 1 STOP");
    serial_print_string("\r\n");
}

void pospravi_robota(void)
{
	serial_print_string("Homing robot.\r\n");
	stop_all_motors();

	motors[0].target_position=0;
	motors[1].target_position=0;
	motors[2].target_position=motors[2].offset+motors[2].starting_position/motors[2].unit_conversion;

	target_x=0;
	target_y=motors[2].offset+motors[2].starting_position/motors[2].unit_conversion;
	target_o=0;

	for (uint8_t mo=0;mo<3;mo++)
	{
		if(motors[mo].position>motors[mo].home_position)
		{
			direction_change(mo,motors[mo].direction_minus);
		}
		else if(motors[mo].position<motors[mo].home_position)
		{
			direction_change(mo,motors[mo].direction_plus);
		}
	}

	run_motor(2);


	while(motors[2].position>motors[2].home_position)
	{
		uart_print_current_targets();
	}
	stop_motor(2);

	run_motor(1);
	run_motor(0);

	while(motors[0].running || motors[1].running || motors[2].running)
	{
		uart_print_current_targets();

		for(uint8_t num_motor=0;num_motor<2;num_motor++)
		{
			if(motors[num_motor].running)
			{
				if((motors[num_motor].position<motors[num_motor].home_position && motors[num_motor].direction==motors[num_motor].direction_minus)||(motors[num_motor].position>motors[num_motor].home_position && motors[num_motor].direction==motors[num_motor].direction_plus))
				{
					stop_motor(num_motor);
				}
			}
		}
	}
	serial_print_string("\r\nRobot je domaci poziciji.\r\n");
	serial_print_string("\r\n");
	serial_print_string("MERITEV 5 STOP");
	serial_print_string("\r\n");
	serial_print_string("MERITEV 5 STOP");
	serial_print_string("\r\n");
	serial_print_string("MERITEV 5 STOP");
	serial_print_string("\r\n");
	next_step=9; //ponastavitev na neuporabno vrednost
}



void robot_control_manually(void){


if (!manual_mode_array[0] && !manual_mode_array[1] && motors[0].running)stop_motor(0);
if (!manual_mode_array[2] && !manual_mode_array[3] && motors[1].running)stop_motor(1);
if (!manual_mode_array[4] && !manual_mode_array[5] && motors[2].running)stop_motor(2);
if (!manual_mode_array[6] && !manual_mode_array[7] && lin_motor_running)DC_Motor_Set_Speed(0);

if(manual_mode_array[0] && !motors[0].running)
{
	direction_change(0,motors[0].direction_plus);
	run_motor(0);
}
if(manual_mode_array[1] && !motors[0].running)
{
	direction_change(0,motors[0].direction_minus);
	run_motor(0);
}

if(manual_mode_array[2] && !motors[2].running)
{
	direction_change(2,motors[2].direction_plus);
	run_motor(2);
}
if(manual_mode_array[3] && !motors[2].running)
{
	direction_change(2,motors[2].direction_minus);
	run_motor(2);
}


if(manual_mode_array[5] && !motors[1].running)
{
	direction_change(1,motors[1].direction_plus);
	run_motor(1);
}
if(manual_mode_array[4] && !motors[1].running)
{
	direction_change(1,motors[1].direction_minus);
	run_motor(1);
}


if(manual_mode_array[6] && !lin_motor_running)
{
	dc_motor_speed=-600;
	DC_Motor_Set_Speed(dc_motor_speed);
}
if(manual_mode_array[7] && !lin_motor_running)
{
	dc_motor_speed=600;
	DC_Motor_Set_Speed(dc_motor_speed);
}

}






void motor_status(void) {
    char status[256];
    snprintf(status, sizeof(status),
             "Motor Status:\r\n"
             "M0: Pos=%lu, Running=%d, Dir=%d, Speed=%lu, Frequency=%lu\r\n"
             "M1: Pos=%lu, Running=%d, Dir=%d, Speed=%lu, Frequency=%lu\r\n"
             "M2: Pos=%lu, Running=%d, Dir=%d, Speed=%lu, Frequency=%lu\r\n"
             "M3: Pos=%lu, Running=%d, Dir=%d, Speed=%lu, Frequency=%lu\r\n"
    		 "--------------------------------------------------\r\n\n",
             motors[0].position, motors[0].running, motors[0].direction, motors[0].current_speed, motors[0].frequency,
             motors[1].position, motors[1].running, motors[1].direction, motors[1].current_speed, motors[1].frequency,
             motors[2].position, motors[2].running, motors[2].direction, motors[2].current_speed, motors[2].frequency,
             motors[3].position, motors[3].running, motors[3].direction, motors[3].current_speed, motors[3].frequency);
    HAL_UART_Transmit(&huart3, (uint8_t*)status, strlen(status), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) { // Prilagodi (npr. USART1), če uporabljaš drug port

	        // Preprečimo prekoračitev bufferja
	        if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1)) {

	            // Če je znak konec vrstice, zaključimo niz in obdelamo ukaz
	            if (uart_single_byte == '\n' || uart_single_byte == '\r') {
	                if (uart_rx_index > 0) { // Obdelaj le, če buffer ni prazen
	                    uart_rx_buffer[uart_rx_index] = '\0'; // Terminacija niza

	                    // Pokličemo procesiranje ukaza neposredno iz interrupta
	                    //uart_process_command((const char*)uart_rx_buffer);

	                    // Ponastavimo indeks za naslednji ukaz
	                    uart_rx_index = 0;
	                }
	            }
	            else {
	                // Če je navaden znak, ga dodaj v buffer
	                uart_rx_buffer[uart_rx_index++] = (char)uart_single_byte;
	            }
	        } else {
	            // Buffer se je prepolnil, ga za vsak slučaj počistimo
	            uart_rx_index = 0;
	        }

	        // KLJUČNO: Ponovno aktiviramo prekinitev za naslednji znak!
	        HAL_UART_Receive_IT(huart, &uart_single_byte, 1);
	    }
	else if (huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1, rx_buff, 30);
	}
}

//maybe rabim?
void izpis_v_serijc(char *sporocilo)
{
	uint8_t X = 0;
	sprintf(sporocilo,sporocilo,X);
	HAL_UART_Transmit(&huart3, sporocilo, sizeof(sporocilo), 100);
}
//konc ne rabim

//dela:
void serial_print_string(const char* text)
{
    char buffer[64];  // Buffer large enough for string + number
    sprintf(buffer, "%s", text);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
}
void serial_print_uint16(uint16_t number)
{
    char buffer[64];  // Buffer large enough for string + number
    sprintf(buffer, "%u", number);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
}
void serial_print_float(float number)
{
    char buffer[64];  // Buffer large enough for string + number
    sprintf(buffer, "%f", number);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
}
void serial_print_empty_screen()
{
    char buffer[64];  // Buffer large enough for string + number
    sprintf(buffer, "\n");
    for(uint8_t i=0;i<250;i++)HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
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

			if(first_read!=1 && rx_buff[uart_read_pos]!='\0')//prvo branje opravimo ko je prvi element različen od '\0', da ne listamo po nepotrebnem
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
	znak='\0';
	//return *beseda;
}

//ne rabim:
void uart_empty_buffer(char *buffer, uint8_t buffer_size)
{
	for(uint8_t buff_ptr=0;buff_ptr<buffer_size;buff_ptr++)
	{
		buffer[buff_ptr]='\0';
	}
}

void receive_target_point(int* x_coordinate, int* y_coordinate, int* z_coordinate, int* phi)
{
	_Bool receive_success=false;

	//char text[]="Nika\r\n";

	char prejeto_sporocilo[30];
	char potrditveno_sporocilo[30];

	while (!receive_success)//verjetno bi bilo pametno dodat timeout?
	{
		uart_receive(prejeto_sporocilo);
		uart_transmit(prejeto_sporocilo);
		uart_receive(potrditveno_sporocilo);

		char good_message[]="OK\n";

		//če prejmemo pravilno potrditveno sporočilo,
		//vemo da smo prejeli pravilne podatke za točki
		if(strcmp(potrditveno_sporocilo,"OK\n")==0)receive_success=true;
		else receive_success=false;
	}

	// Začasni spremenljivki za varno branje
	int temp_x = 0;
	int temp_y = 0;
	int temp_z = 0;
	int temp_rot = 0;

	// sscanf vrne število uspešno prebranih argumentov.
	// Format "X:%lu Y:%lu" preskoči fiksne znake in prebere številke.
	int parsed_count = sscanf(prejeto_sporocilo, "X:%d Y:%d Z:%d ROT:%d", &temp_x, &temp_y, &temp_z, &temp_rot);

	if (parsed_count == 4)
	{
		// Če smo uspešno prebrali obe številki, ju zapišemo na naslova
		*x_coordinate = temp_x;
		*y_coordinate = temp_y;
		*z_coordinate = temp_z;
		*phi = temp_rot;
	}
	else
	{
		// V primeru napake pri formatu lahko nastaviš privzeto vrednost
		*x_coordinate = 0;
		*y_coordinate = 0;
		*z_coordinate = 0;
		*phi = 0;
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
    // Inicializacija bufferja
    for(uint8_t i = 0; i < 32; i++) {
        uart3_rx_buffer[i] = '\0';
    }
    uart3_rx_index = 0;
    uart3_command_ready = 0;
    /* USER CODE END USART3_Init 1 */



    /*
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TX pin (PB10)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // RX pin (PD9)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
     */

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN USART3_Init 2 */
    // Omogoči RX interrupt
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

    // Nastavi prioriteto interrupta (nižja številka = višja prioriteta)
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    // Začni sprejem z interruptom
    HAL_UART_Receive_IT(&huart3, (uint8_t*)uart3_rx_buffer, 1);
    /* USER CODE END USART3_Init 2 */
}


/**
 * @brief USART3 interrupt handler - prejema znake in sestavlja sporočilo
 */
void USART3_IRQHandler(void)
{
	// 1. Preberemo register za stanje (ISR - Interrupt and Status Register)
	    //uint32_t isr_reg = USART3->ISR;

	    // 2. Če je prekinitev sprožil prejet znak (RXNE - Read Data Register Not Empty)
	    //if (isr_reg & USART_ISR_RXNE_RXFNE)
	    //{
	        // NUJNO: Branje registra RDR avtomatsko počisti RXNE zastavico!
	     //   volatile uint8_t received_char = (uint8_t)(USART3->RDR & 0xFF);

	//if (USART3->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE))
   // {
    //    USART3->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    //}

	    	static uint32_t old_target_x=0;
	    	static uint32_t old_target_o=0;
	    	static uint32_t old_target_y=0;

	        // Preveri, ali je interrupt od RXNE (prejet znak)
	        if((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) &&
	           (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) != RESET))
	        {
	            // Preberemo prejeti znak iz registra
	            uint8_t received_char = (uint8_t)(huart3.Instance->RDR & 0xFF);

	            // --- ECHO BACK ---
	            // Takoj pošljemo prejeti znak nazaj po UART, da uporabnik vidi, kaj tipka
	            // Pri '\r' pošljemo še '\n', da v terminalu skoči v novo vrstico

	            if(!manual_mode_flag)
	            	{
	            	HAL_UART_Transmit(&huart3, &received_char, 1, 10);
	            	}

	            if (received_char == '\r') {
	                uint8_t nl = '\n';
	                HAL_UART_Transmit(&huart3, &nl, 1, 10);
	            }



	            if (manual_mode_flag)
	            {
	            	if(received_char == 'a' || received_char == 'A')manual_mode_array[0]=1;
	            	else if(received_char == 'd' || received_char == 'D')manual_mode_array[1]=1;
	            	else if(received_char == 'w' || received_char == 'W')manual_mode_array[2]=1;
	            	else if(received_char == 's' || received_char == 'S')manual_mode_array[3]=1;
	            	else if(received_char == 'q' || received_char == 'Q')manual_mode_array[4]=1;
	            	else if(received_char == 'e' || received_char == 'E')manual_mode_array[5]=1;
	            	else if(received_char == 'r' || received_char == 'R')manual_mode_array[6]=1;
	            	else if(received_char == 'f' || received_char == 'F')manual_mode_array[7]=1;
	            	else if(received_char == 'x' || received_char == 'X')
	            		{
	            		for (uint8_t i=0;i<8;i++)manual_mode_array[i]=0;
	            		}

	            }



	            // Preveri, ali je prejet konec vrstice
	            if(received_char == '\n' || received_char == '\r')
	            {
	            	serial_print_string("\r\nnovo_uart_sporocilo\r\n");
	                if(uart3_rx_index > 0)
	                {
	                    // Zaključi string z null terminatorjem
	                    uart3_rx_buffer[uart3_rx_index] = '\0';

	                    uart3_command_ready = 1;
	                    uart3_new_data = 1;


	                    // --- OBDELAVA UKAZOV ZA ROBOTA (Ignorira velike/male črke) ---
	                    if(strncasecmp((char*)uart3_rx_buffer, "x=", 2) == 0)
	                    {//po določitvi željene orientacija določimo x
	                        int32_t val = (int32_t)atoi((char*)uart3_rx_buffer + 2);

	                        //float temp_target= val-cos((90-target_o)*PI/180)*izteg;//lokacija vozicka
	                        float temp_target= val;

	                        if(temp_target >= robot_bbox.min_x && temp_target <= robot_bbox.max_x)
	                        	{

	                        	target_x = temp_target;
	                        	//old_target_x=target_x;

	                            uart_print_current_targets();

	                        	}

	                    	else
	                    	{
	                            char err[] = "\r\nERROR: X izven Bounding Boxa!\r\n";
	                            HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
	                    	}
	                    }
	                    else if(strncasecmp((char*)uart3_rx_buffer, "y=", 2) == 0)
	                    {//po določitvi željene točke x določimo točko y
	                        int32_t val = (int32_t)atoi((char*)uart3_rx_buffer + 2);

	                        /*
	                        float distance = sqrt(pow(val+90,2)+pow(target_x,2));
	                        if (distance>max_izteg)
	                        {
	                        	char err[] = "\r\nERROR: Pri trenutnih pogojih je y komponenta predalec!\r\n";
	    						HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
	                        }
	                        */

	                        if(val < 0)
							{
								char err[] = "\r\nERROR: Y mora biti pozitiven! Popravljam\r\n";
								HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
								val=-val;
							}

	                        float temp_target=val+motors[2].offset+igla_sklop_offset;
	                        float x_component=(temp_target-0)/tan((90-target_o)*PI/180);


	                        if((x_component<robot_bbox.min_x) || (x_component>robot_bbox.max_x))
	                        {
	                        	char err[] = "\r\nERROR: Y izven Bounding Boxa (x component)!\r\n";
	    						HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
	                        }
	                        else if(temp_target >= robot_bbox.min_y && temp_target <= robot_bbox.max_y)
	                        {

	                            //target_y = temp_target;
	                        	target_y=val;

	                            //old_target_y=target_y;

	                            uart_print_current_targets();

	                        }
	                        else
	                        {
	                            char err[] = "\r\nERROR: Y izven Bounding Boxa!\r\n";
	                            HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
	                        }
	                    }
	                    else if(strncasecmp((char*)uart3_rx_buffer, "O=", 2) == 0)
	                    {//orientacijo določimo prvo
	                        int32_t val = (int32_t)atoi((char*)uart3_rx_buffer + 2);

	                        //float temp_x= target_x-cos((90-val)*PI/180)*izteg;
	                        //float temp_y= target_y-sin((90-val)*PI/180)*izteg;//90 JE OFFSET

	                        if((val >= -30 && val <= 30)) {/*&& (temp_y>robot_bbox.min_y && temp_y<robot_bbox.max_y) && (temp_x>robot_bbox.min_x && temp_x<robot_bbox.max_x)*/
	                            target_o = val;
	                            //target_x = old_target_x-cos((90-target_o)*PI/180)*izteg;
	                            //target_y = sin((90-target_o)*PI/180)*izteg;

	                            uart_print_current_targets();

	                        }
	                        else
	                        {
	                            char err[] = "\r\nERROR: Orientacija izven dovoljenega obmocja (+/- 30 st.)!\r\n";
	                            HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
	                        }
	                    }
	                    else if(strcasecmp((char*)uart3_rx_buffer, "go") == 0)
	                    {
	                    	manual_mode_flag=0;

	                        char msg[] = "\r\nEXEC: Zacetek premika robota...\r\n";
	                        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	                        // Izvedba premika
	                        execute_robot_movement();
	                    }
	                    else if(strcasecmp((char*)uart3_rx_buffer, "exit") == 0)
	                    {
	                    	manual_mode_flag=0;

	                        char msg[] = "\r\nEXIT: Pospravljam robota...\r\n";
	                        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

	                        // Izvedba premika
	                        pospravi_robota();
	                    }
	                    //------------manual mode-------------------
	                    else if((strcasecmp((char*)uart3_rx_buffer, "manual") == 0)||(strcasecmp((char*)uart3_rx_buffer, "cc") == 0))
	                    {
	                    	manual_mode_flag=!manual_mode_flag;

	                    	if(manual_mode_flag)
							{
	                        char menu[] =
	                                "\r\n==================================================\r\n"
	                                " NACIN KRMILJENJA S TIPKOVNICO\r\n"
	                                "==================================================\r\n"
	                                " Navodila za vnos ukazov preko UART (vseeno male/VELIKE crke):\r\n"
	                                "  A  -> Premik M1 LEVO\r\n"
	                                "  D  -> Premik M1 DESNO\r\n"
	                                "  W  -> Premik M3 NAPREJ\r\n"
	                                "  S  -> Premik M3 NAZAJ\r\n"
	                                "  Q  -> Premik M2 CW\r\n"
	                                "  E  -> Premik M2 CCW\r\n"
	                                "  R  -> Premik LIN NAPREJ\r\n"
	                                "  F  -> Premik LIN NAZAJ\r\n"
	                        		"  X  -> Zaustavitev vseh motorjev\r\n"
	                        		"--------------------------------------------------\r\n"
	                                " Motorji lahko delujejo sočasno.\r\n"
									" Pritisk tipke X ustavi vse motorje.\r\n\r\n";

	                    	HAL_UART_Transmit(&huart3, (uint8_t*)menu, strlen(menu), 500);
							}
	                    	else
	                    	{

								serial_print_string("\r\nManual mode izklopljen.\r\n");

							    char menu1[] =
							            "\r\n==================================================================\r\n"
							            //" KALIBRACIJA USPESNO ZAKLJUCENA!\r\n"
							            "======================================================================\r\n"
							            " Navodila za vnos ukazov preko UART (vseeno male/VELIKE crke):\r\n"
							            "  x=stevilka  -> Nastavi cilj X (pozitiven ali negativen)\r\n"
							            "  y=stevilka  -> Nastavi cilj Y (samo pozitiven)\r\n"
							            "  o=stevilka  -> Nastavi orientacijo O (omejitev od -30 do 30)\r\n"
							            "  go          -> Sprozi socasen premik M0 in M1, nato sekvencno M2\r\n"
							            "  exit        -> Pospravi robota iz koncne lege v zacetno\r\n"
							    		"  manual      -> Nacin za krmiljenje s tipkovnico\r\n"
							    		"  cc          -> Izklop nacina za krmiljenje s tipkovnico\r\n"
							    		"  calibrate   -> Izvede kalibracijo\r\n"
							    		"  stikala     -> Vklop nacina za preverjanje delovanja stikal\r\n"
							    		"  sw          -> Izklop nacina za preverjanje delovanja stikal\r\n"
							    		"  tof         -> Izpit desetih meritev TOF senzorja\r\n"
							    		"  spik        -> Preklop nacina za preizkus spikanja\r\n"
							    		"  pumpa       -> Vklopi/izklopi pumpo\r\n"
							    		"  tlak        -> Zažene nekaj meritev tlaka\r\n"
							    		"  regulacija  -> Vklopi/izklopi regulacijo pritiska na 1.8 bar\r\n"
							    		"----------------------------------------------------------------------\r\n"
							            " Vnesi ukaz za orientacijo -> y -> x; in pritisni ENTER:\r\n"
							    		"----------------------------------------------------------------------\r\n\r\n";

								HAL_UART_Transmit(&huart3, (uint8_t*)menu1, strlen(menu1), 500);

	                    	}
	                    }
						//konc
	                    else if((strcasecmp((char*)uart3_rx_buffer, "stikala") == 0)||(strcasecmp((char*)uart3_rx_buffer, "sw") == 0))
	                    {
	                        switch_test=!switch_test;

	                        if(switch_test==0)
	                        {
	                            char menu[] =
	                                    "\r\n==================================================================\r\n"
	                                    //" KALIBRACIJA USPESNO ZAKLJUCENA!\r\n"
	                                    "======================================================================\r\n"
	                                    " Navodila za vnos ukazov preko UART (vseeno male/VELIKE crke):\r\n"
	                                    "  x=stevilka  -> Nastavi cilj X (pozitiven ali negativen)\r\n"
	                                    "  y=stevilka  -> Nastavi cilj Y (samo pozitiven)\r\n"
	                                    "  o=stevilka  -> Nastavi orientacijo O (omejitev od -30 do 30)\r\n"
	                                    "  go          -> Sprozi socasen premik M0 in M1, nato sekvencno M2\r\n"
	                                    "  exit        -> Pospravi robota iz koncne lege v zacetno\r\n"
	                            		"  manual      -> Nacin za krmiljenje s tipkovnico\r\n"
	                            		"  cc          -> Izklop nacina za krmiljenje s tipkovnico\r\n"
	                            		"  calibrate   -> Izvede kalibracijo\r\n"
	                            		"  stikala     -> Vklop nacina za preverjanje delovanja stikal\r\n"
	                            		"  sw          -> Izklop nacina za preverjanje delovanja stikal\r\n"
	                            		"  tof         -> Izpit desetih meritev TOF senzorja\r\n"
	                            		"  spik        -> Preklop nacina za preizkus spikanja\r\n"
	                            		"  pumpa       -> Vklopi/izklopi pumpo\r\n"
	                            		"  tlak        -> Zažene nekaj meritev tlaka\r\n"
	                            		"  regulacija  -> Vklopi/izklopi regulacijo pritiska na 1.8 bar\r\n"
	                            		"----------------------------------------------------------------------\r\n"
	                                    " Vnesi ukaz za orientacijo -> y -> x; in pritisni ENTER:\r\n"
	                            		"----------------------------------------------------------------------\r\n\r\n";

	                        	HAL_UART_Transmit(&huart3, (uint8_t*)menu, strlen(menu), 500);
	                        }
	                    }
						//------------tof test---------------
	                    else if((strcasecmp((char*)uart3_rx_buffer, "tof") == 0))
						{
							tof_test=!tof_test;
						}
	                    //------------spikanje---------------
	                    else if((strcasecmp((char*)uart3_rx_buffer, "spik") == 0))
						{
							spik_test=!spik_test;
						}
	                    //----------kalibracija-------
	                    else if(strcasecmp((char*)uart3_rx_buffer, "calibrate") == 0)
	                    {

	                        char menu[] =
	                                "\r\nZacenjam kalibracijo.\r\n";

	                    	HAL_UART_Transmit(&huart3, (uint8_t*)menu, strlen(menu), 500);

	                        calibrate_flag=1;
	                    }

	                    //----------------pumpa---------------
	                    else if(strcasecmp((char*)uart3_rx_buffer, "pumpa") == 0)
	                    {
	                        pumpa_flag=!pumpa_flag;
	                    }

	                    //------------senzor za tlak----------------
						else if(strcasecmp((char*)uart3_rx_buffer, "tlak") == 0)
						{
							tlak_flag=!tlak_flag;
						}
	                    //------------regulacija tlaka----------------
						else if(strcasecmp((char*)uart3_rx_buffer, "regulacija") == 0)
						{
							regulacija_tlaka_flag=!regulacija_tlaka_flag;
						}
	                    //------------konc rocnih komand----------------



	                    //-------------ukazi iz pc za MERITEV----------
	                    //to omogoči vstop v določene dele kode, pred tem čaka
	                    //v primeru da je next step 0 prekine kodo in pospravi robota
	                    //ko opravi določen del sporoči "MERITEV X STOP"

	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV 1 START") == 0)
	                    {
	                       next_step=1;
	                       serial_print_string("Next step =1 \r\n");
	                    }
	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV 2 START") == 0)
						{
						   next_step=2;
						   serial_print_string("Next step =2 \r\n");
						}
	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV 3 START") == 0)
						{
						   next_step=3;
						   serial_print_string("Next step =3 \r\n");
						}
	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV 4 START") == 0)
						{
						   next_step=4;
						   serial_print_string("Next step =4 \r\n");
						}
	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV 5 START") == 0)
						{
						   next_step=5;
						   serial_print_string("Next step =5 \r\n");
						}
	                    else if(strcasecmp((char*)uart3_rx_buffer, "MERITEV KONEC") == 0)
						{
						   next_step=0;
						   serial_print_string("Next step =0 \r\n");
						}
	                    //----------------konc meritev-------------
	                    else
	                    {
	                        // Stara regulacija tlaka (opcijsko)

	                    	//target_pressure = parse_float_from_string((char*)uart3_rx_buffer);
	                        //char echo[64];
	                        //snprintf(echo, sizeof(echo), "\r\nPritisk set: %.2f\r\n", target_pressure);
	                        //HAL_UART_Transmit(&huart3, (uint8_t*)echo, strlen(echo), 100);
	                    }

	                    // Reset bufferja
	                    uart3_rx_index = 0;
	                    memset((void*)uart3_rx_buffer, 0, sizeof(uart3_rx_buffer));
	                }
	            }
	            else
	            {
	                // Če znak ni konec vrstice in ni Backspace, ga dodaj v buffer
	                if (received_char != '\b' && uart3_rx_index < (sizeof(uart3_rx_buffer) - 1))
	                {
	                    uart3_rx_buffer[uart3_rx_index++] = received_char;
	                }
	                // Podpora za Backspace (brisanje znaka v terminalu)
	                else if (received_char == '\b' && uart3_rx_index > 0)
	                {
	                    uart3_rx_index--;
	                }
	            }

	            // Počisti flag za RXNE
	            __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
	        }
	    //}

	    // 3. NUJNO ZA STM32H7: Preverjanje in čiščenje napak (Overrun, Framing, Noise)
	    // Če pride do napake (npr. Overrun - ORE), RXNE prekinitev zmrzne in nenehno sproža handler!
	    //if (isr_reg & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE))
	    //{
	        // Počistimo vse napake z vpisom v ICR (Interrupt Flag Clear Register)
	    //    USART3->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
	    //}







}


/**
 * @brief Pretvori string v float vrednost
 * @param str: string za pretvorbo
 * @retval float vrednost
 */
float parse_float_from_string(const char* str)
{
    float result = 0.0f;
    float decimal = 0.0f;
    int sign = 1;
    uint8_t i = 0;
    uint8_t decimal_places = 0;
    uint8_t is_decimal = 0;

    // Preveri predznak
    if(str[0] == '-')
    {
        sign = -1;
        i = 1;
    }

    // Pretvori string v float
    while(str[i] != '\0')
    {
        if(str[i] == '.')
        {
            is_decimal = 1;
            i++;
            continue;
        }

        if(str[i] >= '0' && str[i] <= '9')
        {
            if(is_decimal)
            {
                decimal = decimal * 10 + (str[i] - '0');
                decimal_places++;
            }
            else
            {
                result = result * 10 + (str[i] - '0');
            }
        }
        i++;
    }

    // Upoštevaj decimalna mesta
    while(decimal_places > 0)
    {
        decimal /= 10.0f;
        decimal_places--;
    }

    result = (result + decimal) * sign;

    // Omeji vrednost na realen obseg (0-4 bare)
    if(result < 0.0f) result = 0.0f;
    if(result > 4.0f) result = 4.0f;

    return result;
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

	    //GPIO_InitStruct.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); // PE3
	    GPIO_InitStruct.Pin = GPIO_PIN_13;  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // Pc13

	    GPIO_InitStruct.Pin = GPIO_PIN_2;  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct); // PI2

	    GPIO_InitStruct.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // PD3

	    GPIO_InitStruct.Pin = GPIO_PIN_8;  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); // PF8


	    // Now manually configure EXTI for each pin

/*
	    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_Msk;
	    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PI;

	    // PD3 -> EXTI3 (Če želite rajši PE3, zamenjajte _PD z _PE)
	    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_Msk;
	    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PD;

	    // PF8 -> EXTI8 (Vpišemo v EXTICR[2], ker pokriva pine 8-11)
	    SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI8_Msk;
	    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PF;

	    // PC13 -> EXTI13 (Vpišemo v EXTICR[3], ker pokriva pine 12-15)
		SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13_Msk;
		SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;


		// Omogoči Rising edge trigger za linije 2, 3, 8 in 13
		EXTI->RTSR1 |= (1 << 2) | (1 << 3) | (1 << 8) | (1 << 13);

		// Izklopi Falling edge trigger
		EXTI->FTSR1 &= ~((1 << 2) | (1 << 3) | (1 << 8) | (1 << 13));

		// Počisti morebitne čakajoče (pending) prekinitve pred vklopom maske
		EXTI->PR1 = (1 << 2) | (1 << 3) | (1 << 8) | (1 << 13);

		// Omogoči interrupt masko (Interrupt Mask Register) za linije 2, 3, 8 in 13
		//IMR1_ODREZAN_KOMENTAR // Če tvoj MCU uporablja starejši STM32 HAL, je to lahko le EXTI->IMR
		EXTI->IMR1 |= (1 << 2) | (1 << 3) | (1 << 8) | (1 << 13);


		// Nastavitve NVIC (Prekinitvenega kontrolerja)

		HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);

		HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);

		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		// Omogočeno za PC13 (spada pod skupno prekinitev od 10 do 15)
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
*/

		//EXTI->PR1 = (1 << 3) | (1 << 4) | (1 << 15);

}

void EXTI3_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
    }
}

void EXTI2_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_2);
    }
}

void EXTI9_5_IRQHandler(void)
{
    /* Preverimo, ali je prekinitev sprožil prav pin 8 */
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) {
        /* Počistimo zastavico za prekinitev na pinu 8 */
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

        /* Pokličemo HAL povratno funkcijo (Callback) */
        HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
    }
}

/*
void EXTI14_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_14);
    }
}
*/



void EXTI15_10_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
        HAL_GPIO_EXTI_Callback(GPIO_PIN_13);
    }

}


// Enhanced callback with proper shared line handling
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();



    /*
    _Bool pin_PF8  = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)  == GPIO_PIN_SET);
	_Bool pin_PB10 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET);
	_Bool pin_PB11 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET);
	// KORAK 1: Če so pritisnjene vse tri tipke hkrati -> Vstop v Manual Mode
	    if (pin_PF8 && pin_PB10 && pin_PB11)
	    {
	        stop_all_motors();
	        manual_mode = true;
	        manual_motor_idx = 0; // Ponastavimo na prvi motor

	        char msg[] = "\r\n[MANUAL] Vstop v rocni nacin! Vsi motorji zaustavljeni.\r\n";
	        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
	        return; // Prekinemo nadaljnjo obdelavo posameznih klikov v tem ciklu
	    }

	    // KORAK 2: Ukazi se izvršijo ŠELE, ko smo že v "manual mode"
	    if (manual_mode)
	    {
	        // PF8 -> Izbira motorja (vsak klik poveča indeks)
	        if (GPIO_Pin == GPIO_PIN_8 && pin_PF8)
	        {
	            stop_motor(manual_motor_idx); // Za vsak primer ustavimo prejšnjega
	            manual_motor_idx = (manual_motor_idx + 1) % num_of_motors; // num_of_motors je definiran kot 4

	            char msg[50];
	            snprintf(msg, sizeof(msg), "[MANUAL] Izbran motor indeks: %d\r\n", manual_motor_idx);
	            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
	        }

	        // PA0 -> Premik motorja v MINUS smer
	        else if (GPIO_Pin == GPIO_PIN_10 && pin_PB10)
	        {
	            direction_change(manual_motor_idx, 0); // 0 oz. false predstavlja minus smer glede na vaš motor_struct_t
	            run_motor(manual_motor_idx);

	            char msg[] = "[MANUAL] Premik v MINUS smer\r\n";
	            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
	        }

	        // PA1 -> Premik motorja v PLUS smer
	        else if (GPIO_Pin == GPIO_PIN_11 && pin_PB11)
	        {
	            direction_change(manual_motor_idx, 1); // 1 oz. true predstavlja plus smer
	            run_motor(manual_motor_idx);

	            char msg[] = "[MANUAL] Premik v PLUS smer\r\n";
	            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 10);
	        }

	        if (!pin_PB10 && !pin_PB11 && manual_mode) {
	            stop_motor(manual_motor_idx);
	        }
	    }
	    else{
*/

			// Debouncing - ignore interrupts within 50ms
			if((current_time - last_interrupt_time > 50)&&0) {
				last_interrupt_time = current_time;



				switch(GPIO_Pin) {

					case GPIO_PIN_3://motors[0].end_switch_pin:
						// Motor 0 Switch 1 (PE3)
						/*
							stop_motor(0);
							if (motors[0].direction==motors[0].direction_plus)
							{
								motors[0].position = motors[0].max_position;
								motors[0].allowed_direction=motors[0].direction_minus;
							}
							else if (motors[0].direction==motors[0].direction_minus)
							{
								motors[0].position = 0;
								motors[0].allowed_direction=motors[0].direction_plus;
							}
							motors[0].running = false;
							//uart_transmit("M0: Switch (PE3) - STOPPED\r\n");
							motors[0].end_switch_triggered=1;
							*/
					//	break;
					//case GPIO_PIN_15:


						if(!manual_mode_flag)
						{
							stop_motor(2);
							if (motors[2].direction==motors[2].direction_plus)
							{
								motors[2].position = motors[2].max_position;
								motors[2].allowed_direction=motors[2].direction_minus;
							}
							else if (motors[2].direction==motors[2].direction_minus)
							{
								motors[2].position = 0;
								motors[2].allowed_direction=motors[2].direction_plus;
							}
							motors[2].running = false;
							//uart_transmit("M2: Switch (PB4) - STOPPED\r\n");
							motors[2].end_switch_triggered=1;
						}

						break;

					case GPIO_PIN_2://motors[1].end_switch_pin:

						if(!manual_mode_flag)
						{
							stop_motor(1);
							if (motors[1].direction==motors[1].direction_plus)
							{
								motors[1].position = motors[1].max_position;
								motors[1].allowed_direction=motors[1].direction_minus;
							}
							else if (motors[1].direction==motors[1].direction_minus)
							{
								motors[1].position = 0;
								motors[1].allowed_direction=motors[1].direction_plus;
							}
							motors[1].running = false;
							//uart_transmit("M1: Switch (PH15) - STOPPED\r\n");
							motors[1].end_switch_triggered=1;
						}
						break;

					case GPIO_PIN_13://motors[1].end_switch_pin:

							if(!manual_mode_flag)
							{
								stop_motor(0);
								if (motors[0].direction==motors[0].direction_plus)
								{
									motors[0].position = motors[0].max_position;
									motors[0].allowed_direction=motors[0].direction_minus;
								}
								else if (motors[0].direction==motors[0].direction_minus)
								{
									motors[0].position = 0;
									motors[0].allowed_direction=motors[0].direction_plus;
								}
								motors[0].running = false;
								//uart_transmit("M1: Switch (PH15) - STOPPED\r\n");
								motors[0].end_switch_triggered=1;
							}
						break;

					case GPIO_PIN_8:

						if(!manual_mode_flag)
						{
							DC_Motor_Set_Speed(0);//POPRAVI!!!
						}

						break;

		/*
					case GPIO_PIN_4://motors[2].end_switch_pin:
						// Motor 1 (PB4)

						break;
		*/
					default:
						// Unknown pin - this shouldn't happen
						//char msg[50];
						//snprintf(msg, sizeof(msg), "Unknown GPIO: %d\r\n", GPIO_Pin);
						//uart_transmit(msg);
						break;
					}
				}
	   // }
}

/**
 * @brief  Nastavi hitrost in smer DC motorja.
 * @param  speed: Vrednost med -999 in 999.
 * Pozitivne vrednosti = naprej, negativne = nazaj, 0 = stop.
 * @retval None
 */
/**
 * @brief  Sets speed and direction of the DC motor using TIM8 (PH15) and PB4.
 * @param  speed: Value between -999 and 999.
 *                 Positive values = forward, negative = reverse, 0 = stop.
 */
void DC_Motor_Set_Speed(int16_t speed) {


		if (speed > 999)  speed = 999;
	    if (speed < -999) speed = -999;

	    if (speed==0)
	    {
	    	lin_motor_running=0;
	    }
	    else{
	    	lin_motor_running=1;
	    }

	    if (speed >= 0) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)speed);
	        return;
	    }

	    else {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // PB4 HIGH

	        int16_t inverse_speed = 999 - (-speed);
	        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t) inverse_speed);
	    }

}


/**
 * @brief  Posodablja stanje in hitrost DC motorja glede na razdaljo do ovire.
 * Deluje kot neblokirajoči avtomat stanj (State Machine).
 * @param  distance_mm: Trenutna razdalja iz senzorja v milimetrih.
 * @retval None
 */

void DC_Motor_Update(uint16_t distance_mm) {
    /* Preprečimo neveljavne MERITEV senzorja (npr. 0xFFFF ob napaki) */
    if (distance_mm == 0xFFFF || distance_mm == 0) {
        return;
    }

    static int16_t dc_motor_speed=0;
    static _Bool slowed_down=0;
    static _Bool panic_stop=0;

    static uint8_t num_too_far_reading=0;

    static uint32_t dc_sw_timestamp=0;

    if(next_step==0 || next_step==5)dc_current_state=DC_STATE_RETRACTING;//pospravimo v primeru prekinitve

    if(distance_mm<(DC_DIST_MAX_MM+MAX_IZTEG-DOLZINA_IGLE))//ce smo predalec da pridemo do drevesa pocakamo par meritev da se ustali, potem kenslamo
    {	num_too_far_reading=0;
		switch (dc_current_state) {

			case DC_STATE_REGULATED: {
				/* 1. Pogoj za ustavitev: dosežen minimum (preblizu ovire) */

				if(next_step==2 || next_step==3 || spik_test)
				{


					if(!panic_stop)
					{
					if (((distance_mm <= DC_DIST_MIN_MM)&&(next_step==3||spik_test))) {

						if(dc_motor_speed!=0)
						{
						dc_motor_speed=0;
						DC_Motor_Set_Speed(dc_motor_speed);  /* Takojšnja ustavitev */

						serial_print_string("\r\n");
						serial_print_string("MERITEV 3 STOP");
						serial_print_string("\r\n");
						serial_print_string("MERITEV 3 STOP");
						serial_print_string("\r\n");
						serial_print_string("MERITEV 3 STOP");
						serial_print_string("\r\n");
						dc_stop_timestamp = HAL_GetTick(); /* Shranimo trenutni čas ustavljanja */
						//dc_current_state = DC_STATE_WAITING;
						//dc_current_state = DC_STATE_RETRACTING;
						char debug_msg3[64];
						snprintf(debug_msg3, sizeof(debug_msg3), "\n\nRazdalja: %u mm \r\n", distance_mm);
						serial_print_string(debug_msg3);
						serial_print_string("Blizu ovire! Stop.\r\n");
						ciscenje_igle_requested=1;
						next_step=9;

						if(spik_test)
						{
							HAL_Delay(3000);
							ciscenje_igle_requested=0;
							dc_current_state=DC_STATE_RETRACTING;
						}

						}
						break;
					}
					else if (((distance_mm <= DC_DIST_MIN_MM)&&(next_step==2|| spik_test)) ) {
					dc_motor_speed=0;
					DC_Motor_Set_Speed(dc_motor_speed);  /* Takojšnja ustavitev */
					serial_print_string("\r\n");
					serial_print_string("MERITEV 2 STOP");
					serial_print_string("\r\n");
					serial_print_string("MERITEV 2 STOP");
					serial_print_string("\r\n");
					serial_print_string("MERITEV 2 STOP");
					serial_print_string("\r\n");
					}

						//ce se igla se ne dotika debla
						if((((distance_mm>(DC_DIST_MAX_MM))&&(next_step==2|| spik_test)) )&&!slowed_down) {

							//if(dc_motor_speed!=(-900))
							//{
								char debug_msg3[64];
								snprintf(debug_msg3, sizeof(debug_msg3), "\n\nHitro se priblizujem oviri. Razdalja: %u mm \r\n", distance_mm);
								serial_print_string(debug_msg3);
							//}
							dc_motor_speed=-900;
							DC_Motor_Set_Speed(dc_motor_speed);
							dc_sw_timestamp=HAL_GetTick();
						}
						else if((next_step==2 || (spik_test  && !slowed_down)) && dc_motor_speed!=0) {
							slowed_down=1;
							dc_motor_speed=0;
							DC_Motor_Set_Speed(dc_motor_speed);
							serial_print_string("\r\n");
							serial_print_string("MERITEV 2 STOP");
							serial_print_string("\r\n");
							serial_print_string("MERITEV 2 STOP");
							serial_print_string("\r\n");
							serial_print_string("MERITEV 2 STOP");
							serial_print_string("\r\n");
						}
						else if(next_step==3 || (spik_test && slowed_down))
						{
							char debug_msg3[64];
							snprintf(debug_msg3, sizeof(debug_msg3), "\n\nPocasi se priblizujem oviri. Razdalja: %u mm \r\n", distance_mm);
							serial_print_string(debug_msg3);

							dc_motor_speed=-600;
							DC_Motor_Set_Speed(dc_motor_speed);
							dc_sw_timestamp=HAL_GetTick();
						}
						else //if((DC_DIST_MAX_MM+MAX_IZTEG-DOLZINA_IGLE)<distance_mm)
						{
							dc_motor_speed=0;
							DC_Motor_Set_Speed(dc_motor_speed);
						}
					}
					/*
						if(read_switch(3) && dc_motor_speed<0 && (HAL_GetTick()-dc_sw_timestamp)>3000)//ce se zaleti v sprednje stikalo
						{
							dc_motor_speed=0;
							DC_Motor_Set_Speed(dc_motor_speed);

							panic_stop=1;//da ne pade v kak case
						}

						if(panic_stop==1)//premaknemo nazaj in prisilimo pospravljanje
						{
							dc_motor_speed=600;
							DC_Motor_Set_Speed(dc_motor_speed);
							HAL_Delay(500);
							dc_motor_speed=0;
							DC_Motor_Set_Speed(dc_motor_speed);

							serial_print_string("\r\n");
							serial_print_string("MERITEV ZAKLJUCENA");
							serial_print_string("\r\n");
							serial_print_string("MERITEV ZAKLJUCENA");
							serial_print_string("\r\n");
							serial_print_string("MERITEV ZAKLJUCENA");
							serial_print_string("\r\n");

							next_step=0;
						}
						*/

				}
				break;
			}
	/*
			case DC_STATE_WAITING: {

				if(dc_motor_speed!=0)
				{
					dc_motor_speed=0;
					DC_Motor_Set_Speed(dc_motor_speed);
				}

				if(next_step==5)
				{
					dc_stop_timestamp = HAL_GetTick();
					dc_wait_time_ms=2000; //zelimo da ohrani pritisk 5 sekund
					dc_time_elapsed=0;

					while(dc_time_elapsed < dc_wait_time_ms)
					{

									dc_time_elapsed+=HAL_GetTick()-dc_stop_timestamp;
									dc_stop_timestamp=HAL_GetTick();

					}

					//char debug_msg3[64];
					//snprintf(debug_msg3, sizeof(debug_msg3), "\n\nRazdalja: %u mm \r\n", distance_mm);
					//serial_print_string(debug_msg3);

					serial_print_string("Apliciranje opravljeno. Umikam motor nazaj...\r\n");
					dc_current_state = DC_STATE_RETRACTING;
				}
				break;
			}
	*/
			case DC_STATE_RETRACTING: {
				/* 1. Pogoj za konec umikanja: ko dosežemo želeno varnostno razdaljo (maksimum) */
				//if (distance_mm >= DC_DIST_MAX_MM) {
				if (read_switch(3)) {
					dc_motor_speed=0;
					DC_Motor_Set_Speed(dc_motor_speed);
					dc_current_state = DC_STATE_REGULATED;
					zagon_izvedbe = false;
					//char debug_msg3[64];
					//snprintf(debug_msg3, sizeof(debug_msg3), "\n\nRazdalja: %u mm \r\n", distance_mm);
					//serial_print_string(debug_msg3);
					serial_print_string("\r\nUmaknjen na varno razdaljo.\r\n");
					premik_done=0; //ponastavimo zastavico za kinematiko
					spik_test=0;
					slowed_down=0;
					panic_stop=0;
					break;
				}

				/* 2. Vzvratna vožnja s fiksno, varno konstantno hitrostjo (negativna vrednost) */
				//char debug_msg3[64];
				//snprintf(debug_msg3, sizeof(debug_msg3), "\n\nFura nazaj. Razdalja: %u mm \r\n", distance_mm);
				//serial_print_string(debug_msg3);

				if(dc_motor_speed!=900)
				{
					serial_print_string("\r\nFura nazaj.\r\n");
				}

				dc_motor_speed=900;
				DC_Motor_Set_Speed(dc_motor_speed);
				break;
			}

			default:
				dc_current_state = DC_STATE_REGULATED;
				break;
		}
	}
    else
    {
    	dc_motor_speed=0;
    	DC_Motor_Set_Speed(dc_motor_speed);
    	num_too_far_reading++;
    	if(num_too_far_reading>=10 && (dc_current_state!=DC_STATE_RETRACTING))
    	{
    		serial_print_string("Deblo je zal predalec :(");
			serial_print_string("\r\n");
			serial_print_string("MERITEV ZAKLJUCENA");
			serial_print_string("\r\n");
			serial_print_string("MERITEV ZAKLJUCENA");
			serial_print_string("\r\n");
			serial_print_string("MERITEV ZAKLJUCENA");
			serial_print_string("\r\n");
			num_too_far_reading=0;
			spik_test=0;

			if(!read_switch(3))
			{
				if(dc_motor_speed!=900)
				{
					serial_print_string("\r\nFura nazaj.\r\n");
				}

				dc_motor_speed=900;
				DC_Motor_Set_Speed(dc_motor_speed);
			}
			else
			{
				dc_motor_speed=0;
				DC_Motor_Set_Speed(dc_motor_speed);
			}

			while(!read_switch(3)){}

			dc_motor_speed=0;
			DC_Motor_Set_Speed(dc_motor_speed);
			dc_current_state = DC_STATE_REGULATED;

			zagon_izvedbe = false;
			serial_print_string("\r\nUmaknjen na varno razdaljo.\r\n");
			premik_done=0; //ponastavimo zastavico za kinematiko
			spik_test=0;
			slowed_down=0;
			panic_stop=0;


    	}
    }
}

void DC_Motor_Init(void) {
    /* 1. VKLOP UR ZA TIM13 IN PORT A (Kritično za H7!) */
    __HAL_RCC_TIM8_CLK_ENABLE();  // <-- TA VRSTICA JE MANJKALA!
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Kratek sistemski premor, da H7 uskladi registre ur (strojna specifika H7)
        __DSB();
        __ISB();

    /* 2. Konfiguracija smernega pina PF8 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* 3. Konfiguracija PWM pina PA6 (TIM13_CH1, AF9) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;  // AF9 je pravilen za TIM13 na PA6
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* 4. Konfiguracija časovnika TIM13 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 199; // 200 MHz / 200 = 1 MHz osnova
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 999;    // 1 kHz frekvenca PWM signala
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }

    /* 5. Konfiguracija PWM kanala 1 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0; // Začne z ustavljenim motorjem
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    /* 6. Zagon strojnega PWM-ja */
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    lin_motor_running=1;

    //serial_print_string("DC motor init.\r\n");
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
