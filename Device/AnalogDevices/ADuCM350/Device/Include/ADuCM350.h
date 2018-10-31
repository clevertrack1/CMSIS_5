/**************************************************************************//**
 * @file     ADuCM350.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           Analog Devices ADuCM350
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

#ifndef ADuCM350_H
#define ADuCM350_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup AnalogDevices
  * @{
  */


/** @addtogroup ADuCM350
  * @{
  */


/** @addtogroup Configuration_of_CMSIS
  * @{
  */



/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum IRQn
{
/* =======================================  ARM Cortex-M# Specific Interrupt Numbers  ======================================== */

/* use this Cortex interrupt numbers if your device is a Cortex-M3 / Cortex-M4 / Cortex-M7 device */
  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  MemoryManagement_IRQn     = -12,              /*!< -12  Memory Management, MPU mismatch, including Access Violation
                                                          and No Match                                                         */
  BusFault_IRQn             = -11,              /*!< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                          related Fault                                                        */
  UsageFault_IRQn           = -10,              /*!< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  DebugMonitor_IRQn         =  -4,              /*!< -4 Debug Monitor                                                          */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */

/* ===========================================  ADuCM350 Specific Interrupt Numbers  ========================================= */
/*  device specific external interrupt numbers according the interrupt handlers defined in startup_Device.s
         eg.: Interrupt for Timer#1       TIM1_IRQHandler   ->   TIM1_IRQn */
/* ==========================================  ADuCM350 Specific Interrupt Numbers  ========================================== */
  WUT_IRQn                  =   0,              /*!< 0  Wake Up Timer interrupt                                                */
  EINT0_IRQn                =   1,              /*!< 1  External Interrupt 0                                                   */
  EINT1_IRQn                =   2,              /*!< 2  External Interrupt 1                                                   */
  EINT2_IRQn                =   3,              /*!< 3  External Interrupt 2                                                   */
  EINT3_IRQn                =   4,              /*!< 4  External Interrupt 3                                                   */
  EINT4_IRQn                =   5,              /*!< 5  External Interrupt 4                                                   */
  EINT5_IRQn                =   6,              /*!< 6  External Interrupt 5                                                   */
  EINT6_IRQn                =   7,              /*!< 7  External Interrupt 6                                                   */
  EINT7_IRQn                =   8,              /*!< 8  External Interrupt 7                                                   */
  EINT8_IRQn                =   9,              /*!< 9  External Interrupt 8                                                   */
  WDT_IRQn                  =  10,              /*!< 10 WDT Interrupt                                                          */
  TIMER0_IRQn               =  11,              /*!< 11 Timer interrupt                                                        */
  TIMER1_IRQn               =  12,              /*!< 12 Timer 1 Interrupt                                                      */
  FLASH0_IRQn               =  13,              /*!< 13 Flash Controller Interrupt                                             */
  UART_IRQn                 =  14,              /*!< 14 interrupt                                                              */
  SPI0_IRQn                 =  15,              /*!< 15 SPI 0 interrupt                                                        */
  SPIH_IRQn                 =  16,              /*!< 16 interrupt                                                              */
  I2CS_IRQn                 =  17,              /*!< 17 I2C 0 slave interrupt                                                  */
  I2CM_IRQn                 =  18,              /*!< 18 I2C 0 master interrupt                                                 */
  DMA_ERR_IRQn              =  19,              /*!< 19 DMA interrupt                                                          */
  DMA_SPIH_TX_IRQn          =  20,              /*!< 20 DMA Ch 0 interrupt                                                     */
  DMA_SPIH_RX_IRQn          =  21,              /*!< 21 DMA Ch 1 interrupt                                                     */
  DMA_SPI0_TX_IRQn          =  22,              /*!< 22 DMA Ch 2 interrupt                                                     */
  DMA_SPI0_RX_IRQn          =  23,              /*!< 23 DMA Ch 3 interrupt                                                     */
  DMA_SPI1_TX_IRQn          =  24,              /*!< 24 DMA Ch 4 interrupt                                                     */
  DMA_SPI1_RX_IRQn          =  25,              /*!< 25 DMA Ch 5 interrupt                                                     */
  DMA_UART_TX_IRQn          =  26,              /*!< 26 DMA Ch 6 interrupt                                                     */
  DMA_UART_RX_IRQn          =  27,              /*!< 27 DMA Ch 7 interrupt                                                     */
  DMA_I2CS_TX_IRQn          =  28,              /*!< 28 DMA Ch 8 interrupt                                                     */
  DMA_I2CS_RX_IRQn          =  29,              /*!< 29 DMA Ch 9 interrupt                                                     */
  DMA_I2CM_IRQn             =  30,              /*!< 30 DMA Ch 10 interrupt                                                    */
  DMA_AFE_TX_IRQn           =  31,              /*!< 31 DMA Ch 11 interrupt                                                    */
  DMA_AFE_RX_IRQn           =  32,              /*!< 32 DMA Ch 12 interrupt                                                    */
  DMA_CRC_IRQn              =  33,              /*!< 33 DMA Ch 13 interrupt                                                    */
  DMA_PDI_IRQn              =  34,              /*!< 34 DMA Ch 14 interrupt                                                    */
  DMA_I2S_IRQn              =  35,              /*!< 35 DMA Ch 15 interrupt                                                    */
  USB_WAKEUP_IRQn           =  36,              /*!< 36 USB Wakeup interrupt                                                   */
  USB_CNTL_IRQn             =  37,              /*!< 37 USB Controller interrupt                                               */
  USB_DMA_IRQn              =  38,              /*!< 38 USB DMA interrupt                                                      */
  I2S_IRQn                  =  39,              /*!< 39 I2S interrupt                                                          */
  TIMER2_IRQn               =  40,              /*!< 40 TIMER 2 interrupt                                                      */
  SPI1_IRQn                 =  42,              /*!< 42 interrupt                                                              */
  RTC_IRQn                  =  43,              /*!< 43 Real Time Clock interrupt                                              */
  BEEP_IRQn                 =  45,              /*!< 45 Beep interrupt                                                         */
  LCD_IRQn                  =  46,              /*!< 46 LCD Controller interrupt                                               */
  GPIOA_IRQn                =  47,              /*!< 47 interrupt                                                              */
  GPIOB_IRQn                =  48,              /*!< 48 interrupt                                                              */
  AFE_CAPTURE_IRQn          =  50,              /*!< 50 Analog Front End Capture interrupt                                     */
  AFE_GENERATE_IRQn         =  51,              /*!< 51 Analog Front End Generation interrupt                                  */
  AFE_CMD_FIFO_IRQn         =  52,              /*!< 52 Analog Front End FIFO CMD interrupt                                    */
  AFE_DATA_FIFO_IRQn        =  53,              /*!< 53 Analog Front End FIFO DATA interrupt                                   */
  GP_FLASH_IRQn             =  55,              /*!< 55 Flash EEPROM interrupt                                                 */
  RAND_IRQn                 =  58,              /*!< 58 Random Bit Generator interrupt                                         */
  PDI_IRQn                  =  59               /*!< 59 Paraller Display Interface interrupt                                   */
} IRQn_Type;



/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* ===========================  Configuration of the Arm Cortex-M4 Processor and Core Peripherals  =========================== */
#define __CM3_REV                 0x0201    /*!< Core Revision r2p1 */
#define __MPU_PRESENT             0         /*!< Set to 1 if MPU is present */
#define __VTOR_PRESENT            0         /*!< Set to 1 if VTOR is present */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             0         /*!< Set to 1 if FPU is present */
#define __FPU_DP                  0         /*!< Set to 1 if FPU is double precision FPU (default is single precision FPU) */
#define __ICACHE_PRESENT          0         /*!< Set to 1 if I-Cache is present */
#define __DCACHE_PRESENT          0         /*!< Set to 1 if D-Cache is present */
#define __DTCM_PRESENT            0         /*!< Set to 1 if DTCM is present */


/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm3.h>                           /*!< Arm Cortex-M# processor and core peripherals */

#include "system_ADuCM350.h"                    /*!< ADuCM350 System */


/* ========================================  Start of section using anonymous unions  ======================================== */
#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                            Device Specific Peripheral Section                             ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripherals
  * @{
  */

/* ToDo: add here your device specific peripheral access structure typedefs
         following is an example for a timer */

/* =========================================================================================================================== */
/* ================                                            TMR                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Timer (TMR)
  */

typedef struct
{                                               /*!< (@ 0x40000000) TIM Structure                                              */
  __IOM uint32_t   TimerLoad;                   /*!< (@ 0x00000004) Timer Load                                                 */
  __IM  uint32_t   TimerValue;                  /*!< (@ 0x00000008) Timer Counter Current Value                                */
  __IOM uint32_t   TimerControl;                /*!< (@ 0x0000000C) Timer Control                                              */
  __OM  uint32_t   TimerIntClr;                 /*!< (@ 0x00000010) Timer Interrupt Clear                                      */
  __IM  uint32_t   TimerRIS;                    /*!< (@ 0x00000014) Timer Raw Interrupt Status                                 */
  __IM  uint32_t   TimerMIS;                    /*!< (@ 0x00000018) Timer Masked Interrupt Status                              */
  __IM  uint32_t   RESERVED[1];
  __IOM uint32_t   TimerBGLoad;                 /*!< (@ 0x00000020) Background Load Register                                   */
} ADI_ADuCM350_TMR_TypeDef;

/*@}*/ /* end of group ADuCM350_Peripherals */


/* =========================================  End of section using anonymous unions  ========================================= */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */


/* ToDo: add here your device peripherals base addresses
         following is an example for timer */
/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

/* Peripheral and SRAM base address */
#define ADI_ADuCM350_FLASH_BASE       (0x00000000UL)                              /*!< (FLASH     ) Base Address */
#define ADI_ADuCM350_SRAM_BASE        (0x20000000UL)                              /*!< (SRAM      ) Base Address */
#define ADI_ADuCM350_PERIPH_BASE      (0x40000000UL)                              /*!< (Peripheral) Base Address */

/* Peripheral memory map */
#define ADI_ADuCM350_TIM0_BASE         (ADI_ADuCM350_PERIPH_BASE)          /*!< (Timer0    ) Base Address */
#define ADI_ADuCM350_TIM1_BASE         (ADI_ADuCM350_PERIPH_BASE + 0x0800) /*!< (Timer1    ) Base Address */
#define ADI_ADuCM350_TIM2_BASE         (ADI_ADuCM350_PERIPH_BASE + 0x1000) /*!< (Timer2    ) Base Address */

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/* ToDo: add here your device peripherals pointer definitions
         following is an example for timer */
/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define ADI_ADuCM350_TIM0        ((ADI_ADuCM350_TMR_TypeDef *) ADI_ADuCM350TIM0_BASE)
#define ADI_ADuCM350_TIM1        ((ADI_ADuCM350_TMR_TypeDef *) ADI_ADuCM350TIM0_BASE)
#define ADI_ADuCM350_TIM2        ((ADI_ADuCM350_TMR_TypeDef *) ADI_ADuCM350TIM0_BASE)


/** @} */ /* End of group ADuCM350 */

/** @} */ /* End of group <Vendor> */

#ifdef __cplusplus
}
#endif

#endif  /* ADuCM350_H */
