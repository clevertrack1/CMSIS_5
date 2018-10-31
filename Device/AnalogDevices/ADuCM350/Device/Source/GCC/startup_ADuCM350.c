/**************************************************************************//**
 * @file     startup_ARMCM3.c
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM3 Device
 * @version  V5.3.1
 * @date     09. July 2018
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

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/

#include "ARMCM3.h"


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
extern void _start     (void) __attribute__((noreturn)); /* Pre-Main (C library entry point) */


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void) __attribute__ ((noreturn));
void Reset_Handler  (void) __attribute__ ((noreturn));


/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
//<h> Stack Configuration
//  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
//</h>
#define  __STACK_SIZE  0x00000400
static uint8_t stack[__STACK_SIZE] __attribute__ ((aligned(8), used, section(".stack")));

//<h> Heap Configuration
//  <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
//</h>
#define  __HEAP_SIZE   0x00000C00
#if __HEAP_SIZE > 0
static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif


/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));

void  WUT_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT0_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT1_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT2_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT3_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT4_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT5_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT6_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT7_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  EINT8_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  WDT_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void  TIMER0_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void  TIMER1_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void  FLASH0_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void  UART_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  SPI0_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  SPIH_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  I2CS_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  I2CM_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_ERR_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPIH_TX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPIH_RX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPI0_TX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPI0_RX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPI1_TX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_SPI1_RX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_UART_TX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_UART_RX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_I2CS_TX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_I2CS_RX_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_I2CM_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_AFE_TX_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_AFE_RX_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_CRC_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_PDI_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void  DMA_I2S_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void  USB_WAKEUP_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void  USB_CNTL_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void  USB_DMA_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void  I2S_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void  TIMER2_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void  SPI1_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  RTC_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void  BEEP_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  LCD_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void  GPIOA_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  GPIOB_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
void  AFE_CAPTURE_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void  AFE_GENERATE_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void  AFE_CMD_FIFO_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void  AFE_DATA_FIFO_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void  GP_FLASH_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void  RAND_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void  PDI_Handler     	      (void) __attribute__ ((weak, alias("Default_Handler")));


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern const pFunc __Vectors[240];
       const pFunc __Vectors[240] __attribute__ ((section(".vectors"))) = {
  (pFunc)(&__StackTop),                     /*     Initial Stack Pointer */
  Reset_Handler,                            /*     Reset Handler */
  NMI_Handler,                              /* -14 NMI Handler */
  HardFault_Handler,                        /* -13 Hard Fault Handler */
  MemManage_Handler,                        /* -12 MPU Fault Handler */
  BusFault_Handler,                         /* -11 Bus Fault Handler */
  UsageFault_Handler,                       /* -10 Usage Fault Handler */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  SVC_Handler,                              /*  -5 SVCall Handler */
  DebugMon_Handler,                         /*  -4 Debug Monitor Handler */
  0,                                        /*     Reserved */
  PendSV_Handler,                           /*  -2 PendSV Handler */
  SysTick_Handler,                          /*  -1 SysTick Handler */

  /* Interrupts */
    WUT_Handler,                /*    0  Wake Up Timer interrupt */ 
    EINT0_Handler,              /*    1  External Interrupt 0 */ 
    EINT1_Handler,              /*    2  External Interrupt 1 */ 
    EINT2_Handler,              /*    3  External Interrupt 2 */ 
    EINT3_Handler,              /*    4  External Interrupt 3 */ 
    EINT4_Handler,              /*    5  External Interrupt 4 */ 
    EINT5_Handler,              /*    6  External Interrupt 5 */ 
    EINT6_Handler,              /*    7  External Interrupt 6 */ 
    EINT7_Handler,              /*    8  External Interrupt 7 */ 
    EINT8_Handler,              /*    9  External Interrupt 8 */ 
    WDT_Handler,                /*    10 WDT Interrupt */ 
    TIMER0_Handler,             /*    11 Timer interrupt */ 
    TIMER1_Handler,             /*    12 Timer 1 Interrupt */ 
    FLASH0_Handler,             /*    13 Flash Controller Interrupt */ 
    UART_Handler,               /*    14 interrupt */ 
    SPI0_Handler,               /*    15 SPI 0 interrupt */ 
    SPIH_Handler,               /*    16 interrupt */ 
    I2CS_Handler,               /*    17 I2C 0 slave interrupt */ 
    I2CM_Handler,               /*    18 I2C 0 master interrupt */ 
    DMA_ERR_Handler,            /*    19 DMA interrupt */ 
    DMA_SPIH_TX_Handler,        /*    20 DMA Ch 0 interrupt */ 
    DMA_SPIH_RX_Handler,        /*    21 DMA Ch 1 interrupt */ 
    DMA_SPI0_TX_Handler,        /*    22 DMA Ch 2 interrupt */ 
    DMA_SPI0_RX_Handler,        /*    23 DMA Ch 3 interrupt */ 
    DMA_SPI1_TX_Handler,        /*    24 DMA Ch 4 interrupt */ 
    DMA_SPI1_RX_Handler,        /*    25 DMA Ch 5 interrupt */ 
    DMA_UART_TX_Handler,        /*    26 DMA Ch 6 interrupt */ 
    DMA_UART_RX_Handler,        /*    27 DMA Ch 7 interrupt */ 
    DMA_I2CS_TX_Handler,        /*    28 DMA Ch 8 interrupt */ 
    DMA_I2CS_RX_Handler,        /*    29 DMA Ch 9 interrupt */ 
    DMA_I2CM_Handler,           /*    30 DMA Ch 10 interrupt */
	DMA_AFE_TX_Handler,         /*    31 DMA Ch 11 interrupt */ 
    DMA_AFE_RX_Handler,         /*    32 DMA Ch 12 interrupt */ 
    DMA_CRC_Handler,            /*    33 DMA Ch 13 interrupt */ 
    DMA_PDI_Handler,            /*    34 DMA Ch 14 interrupt */ 
    DMA_I2S_Handler,            /*    35 DMA Ch 15 interrupt */ 
    USB_WAKEUP_Handler,         /*    36 USB Wakeup interrupt */ 
    USB_CNTL_Handler,           /*    37 USB Controller interrupt */ 
    USB_DMA_Handler,            /*    38 USB DMA interrupt */ 
    I2S_Handler,                /*    39 I2S interrupt */ 
    TIMER2_Handler,             /*    40 TIMER 2 interrupt */ 
    SPI1_Handler,               /*    42 interrupt */ 
    RTC_Handler,                /*    43 Real Time Clock interrupt */ 
    BEEP_Handler,               /*    45 Beep interrupt */ 
    LCD_Handler,                /*    46 LCD Controller interrupt */ 
    GPIOA_Handler,              /*    47 interrupt */ 
    GPIOB_Handler,              /*    48 interrupt */ 
    AFE_CAPTURE_Handler,        /*    50 Analog Front End Capture interrupt */ 
    AFE_GENERATE_Handler,       /*    51 Analog Front End Generation interrupt */ 
    AFE_CMD_FIFO_Handler,       /*    52 Analog Front End FIFO CMD interrupt */ 
    AFE_DATA_FIFO_Handler,      /*    53 Analog Front End FIFO DATA interrupt */ 
    GP_FLASH_Handler,           /*    55 Flash EEPROM interrupt */ 
    RAND_Handler,               /*    58 Random Bit Generator interrupt */ 
    PDI_Handler,                /*    59 Paraller Display Interface interrupt */ 
                                /* Interrupts 60 .. 224 are left out */
};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) {
  uint32_t *pSrc, *pDest;
  uint32_t *pTable __attribute__((unused));

/* Firstly it copies data from read only memory to RAM.
 * There are two schemes to copy. One can copy more than one sections.
 * Another can copy only one section. The former scheme needs more
 * instructions and read-only data to implement than the latter.
 * Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.
 */

#ifdef __STARTUP_COPY_MULTIPLE
/* Multiple sections scheme.
 *
 * Between symbol address __copy_table_start__ and __copy_table_end__,
 * there are array of triplets, each of which specify:
 *   offset 0: LMA of start of a section to copy from
 *   offset 4: VMA of start of a section to copy to
 *   offset 8: size of the section to copy. Must be multiply of 4
 *
 * All addresses must be aligned to 4 bytes boundary.
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
/* Single section scheme.
 *
 * The ranges of copy from/to are specified by following symbols
 *   __etext: LMA of start of the section to copy from. Usually end of text
 *   __data_start__: VMA of start of the section to copy to
 *   __data_end__: VMA of end of the section to copy to
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
  pSrc  = &__etext;
  pDest = &__data_start__;

  for ( ; pDest < &__data_end__ ; ) {
    *pDest++ = *pSrc++;
  }
#endif /*__STARTUP_COPY_MULTIPLE */

/* This part of work usually is done in C library startup code.
 * Otherwise, define this macro to enable it in this startup.
 *
 * There are two schemes too.
 * One can clear multiple BSS sections. Another can only clear one section.
 * The former is more size expensive than the latter.
 *
 * Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 * Otherwise define macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/* Multiple sections scheme.
 *
 * Between symbol address __copy_table_start__ and __copy_table_end__,
 * there are array of tuples specifying:
 *   offset 0: Start of a BSS section
 *   offset 4: Size of this BSS section. Must be multiply of 4
 */
  pTable = &__zero_table_start__;

  for (; pTable < &__zero_table_end__; pTable = pTable + 2) {
    pDest = (uint32_t*)*(pTable + 0);
    for (; pDest < (uint32_t*)(*(pTable + 0) + *(pTable + 1)) ; ) {
      *pDest++ = 0;
    }
  }
#elif defined (__STARTUP_CLEAR_BSS)
/* Single BSS section scheme.
 *
 * The BSS section is specified by following symbols
 *   __bss_start__: start of the BSS section.
 *   __bss_end__: end of the BSS section.
 *
 * Both addresses must be aligned to 4 bytes boundary.
 */
  pDest = &__bss_start__;

  for ( ; pDest < &__bss_end__ ; ) {
    *pDest++ = 0UL;
  }
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

  SystemInit();                             /* CMSIS System Initialization */
  _start();                                 /* Enter PreeMain (C library entry point) */
}


/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {

  while(1);
}
