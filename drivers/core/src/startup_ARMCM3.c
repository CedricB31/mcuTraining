/**************************************************************************//**
 * @file     startup_ARMCM3.s
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM3 Device Series
 * @version  V5.00
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>


/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void( *pFunc )( void );


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
#ifndef __START
extern void  _start(void) __attribute__((noreturn));    /* PreeMain (C library entry point) */
#else
extern int  __START(void) __attribute__((noreturn));    /* main entry point */
#endif

#ifndef __NO_SYSTEM_INIT
extern void SystemInit (void);            /* CMSIS System Initialization      */
#endif


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);                          /* Default empty handler */
void Reset_Handler(void);                            /* Reset Handler */


/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
  #define	__STACK_SIZE  0x00000400
#endif
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

#ifndef __HEAP_SIZE
  #define	__HEAP_SIZE   0x00000C00
#endif
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif


/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Cortex-M3 Processor Exceptions */
void NMI_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler  (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));

/* ARMCM3 Specific Interrupts */
void WWDG_IRQHandler      		(void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler      		(void) __attribute__ ((weak, alias("Default_Handler")));
void TAMPER_STAMP_IRQHandler 	(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void DAC_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void LCD_IRQHandler				(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM9_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM10_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM11_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void USB_FS_WKUP_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler			(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler	(void) __attribute__ ((weak, alias("Default_Handler")));
void COMP_ACQ_IRQHandler		(void) __attribute__ ((weak, alias("Default_Handler")));
void BootRAM					(void) __attribute__ ((weak, alias("Default_Handler")));
/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__ ((section(".vectors"))) = {
  /* Cortex-M3 Exceptions Handler */
  (pFunc)((uint32_t)&__StackTop),           /*      Initial Stack Pointer     */
  Reset_Handler,                            /*      Reset Handler             */
  NMI_Handler,                              /*      NMI Handler               */
  HardFault_Handler,                        /*      Hard Fault Handler        */
  MemManage_Handler,                        /*      MPU Fault Handler         */
  BusFault_Handler,                         /*      Bus Fault Handler         */
  UsageFault_Handler,                       /*      Usage Fault Handler       */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  0,                                        /*      Reserved                  */
  SVC_Handler,                              /*      SVCall Handler            */
  DebugMon_Handler,                         /*      Debug Monitor Handler     */
  0,                                        /*      Reserved                  */
  PendSV_Handler,                           /*      PendSV Handler            */
  SysTick_Handler,                          /*      SysTick Handler           */

  /* External interrupts */
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_STAMP_IRQHandler,
  RTC_WKUP_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMA1_Channel1_IRQHandler,
  DMA1_Channel2_IRQHandler,
  DMA1_Channel3_IRQHandler,
  DMA1_Channel4_IRQHandler,
  DMA1_Channel5_IRQHandler,
  DMA1_Channel6_IRQHandler,
  DMA1_Channel7_IRQHandler,
  ADC1_IRQHandler,
  USB_HP_IRQHandler,
  USB_LP_IRQHandler,
  DAC_IRQHandler,
  COMP_IRQHandler,
  EXTI9_5_IRQHandler,
  LCD_IRQHandler,
  TIM9_IRQHandler,
  TIM10_IRQHandler,
  TIM11_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI15_10_IRQHandler,
  RTC_Alarm_IRQHandler,
  USB_FS_WKUP_IRQHandler,
  TIM6_IRQHandler,
  TIM7_IRQHandler,
  0,
  TIM5_IRQHandler,
  SPI3_IRQHandler,
  UART4_IRQHandler,
  UART5_IRQHandler,
  DMA2_Channel1_IRQHandler,
  DMA2_Channel2_IRQHandler,
  DMA2_Channel3_IRQHandler,
  DMA2_Channel4_IRQHandler,
  DMA2_Channel5_IRQHandler,
  0,
  COMP_ACQ_IRQHandler,
  0,
  0,
  0,
  0,
  0,
  BootRAM,          /* @0x108. This is for boot in RAM mode for STM32L152XE devices. */
};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) {
  uint32_t *pSrc, *pDest;
  uint32_t *pTable __attribute__((unused));

/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pTable = &__copy_table_start__;

  for (; pTable < &__copy_table_end__; pTable = pTable + 3) {
		pSrc  = (uint32_t*)*(pTable + 0);
		pDest = (uint32_t*)*(pTable + 1);
		for (; pDest < (uint32_t*)(*(pTable + 1) + *(pTable + 2)) ; ) {
      *pDest++ = *pSrc++;
		}
	}
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
  pSrc  = &__etext;
  pDest = &__data_start__;

  for ( ; pDest < &__data_end__ ; ) {
    *pDest++ = *pSrc++;
  }
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
  pTable = &__zero_table_start__;

  for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
		pDest = (uint32_t*)*(pTable + 0);
		for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
      *pDest++ = 0;
		}
	}
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
  pDest = &__bss_start__;

  for ( ; pDest < &__bss_end__ ; ) {
    *pDest++ = 0UL;
  }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
	SystemInit();
#endif

#ifndef __START
#define __START _start
#endif
	__START();

}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {

	while(1);
}
