/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern char rcv_buff[];
extern char rx_buff[];
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */

//void HardFault_Handler(void)
//{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
//  while (1)
//  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
//  }
//}
/* --- paste this entire block into stm32h7xx_it.c --- */

/* needs access to huart3 which is defined in main.c */
extern UART_HandleTypeDef huart3;

static void fault_print_str(const char *s) {
    while (*s) {
        while (!(huart3.Instance->ISR & USART_ISR_TXE_TXFNF));
        huart3.Instance->TDR = (uint8_t)*s++;
    }
}

static void fault_print_hex(uint32_t val) {
    char buf[11];
    buf[0] = '0'; buf[1] = 'x';
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (val >> (i * 4)) & 0xF;
        buf[9 - i] = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
    }
    buf[10] = '\0';
    for (int i = 0; buf[i]; i++) {
        while (!(huart3.Instance->ISR & USART_ISR_TXE_TXFNF));
        huart3.Instance->TDR = buf[i];
    }
}

void HardFault_Handler_C(uint32_t *fault_stack) {

    /* Force write buffer to drain so BFAR gets populated for imprecise faults */
    SCB->CFSR;  /* read to latch */
    /* Enable precise bus faults temporarily to catch the real address */
    //SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;  /* not helpful here, skip */

    /* Instead: check if this is a write-buffer fault by reading ACTLR */
    uint32_t actlr = SCnSCB->ACTLR;


    uint32_t pc   = fault_stack[6];
    uint32_t lr   = fault_stack[5];
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;
    uint32_t bfar = SCB->BFAR;
    uint32_t mmfar= SCB->MMFAR;

    fault_print_str("\r\n=== HARDFAULT ===\r\n");
    fault_print_str("PC   = "); fault_print_hex(pc);    fault_print_str("\r\n");
    fault_print_str("LR   = "); fault_print_hex(lr);    fault_print_str("\r\n");
    fault_print_str("CFSR = "); fault_print_hex(cfsr);  fault_print_str("\r\n");
    fault_print_str("HFSR = "); fault_print_hex(hfsr);  fault_print_str("\r\n");
    fault_print_str("BFAR = "); fault_print_hex(bfar);  fault_print_str("\r\n");
    fault_print_str("MMFAR= "); fault_print_hex(mmfar); fault_print_str("\r\n");
    fault_print_str("ACTLR= "); fault_print_hex(actlr); fault_print_str("\r\n");

    if (cfsr & 0x0002) fault_print_str("-> DACCVIOL: bad memory read/write\r\n");
    if (cfsr & 0x0200) fault_print_str("-> PRECISERR: precise bus error (BFAR valid)\r\n");
    if (cfsr & 0x0400) fault_print_str("-> IMPRECISERR: imprecise bus error\r\n");
    if (cfsr & 0x10000) fault_print_str("-> UNDEFINSTR: undefined instruction\r\n");
    if (cfsr & 0x20000) fault_print_str("-> INVSTATE: invalid CPU state\r\n");
    if (cfsr & 0x100000) fault_print_str("-> UNALIGNED: unaligned memory access\r\n");
    if (cfsr & 0x200000) fault_print_str("-> DIVBYZERO: divide by zero\r\n");
    if (hfsr & 0x40000000) fault_print_str("-> FORCED: escalated from configurable fault\r\n");

    fault_print_str("=================\r\n");
    __disable_irq();
    while (1);
}

/* naked so the compiler doesn't corrupt the stack before we save it */
__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile (
        "tst   lr, #4          \n"
        "ite   eq              \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "b     HardFault_Handler_C \n"
    );
}


/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */

	  /* USER CODE END USART1_IRQn 0 */
	  HAL_UART_IRQHandler(&huart1);

	  /* USER CODE BEGIN USART1_IRQn 1 */
	  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
