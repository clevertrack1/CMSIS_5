/**************************************************************************//**
 * @file     startup_ADuCM350.S
 * @brief    CMSIS Cortex-M# Core Device Startup File for
 *           Device ADuCM350
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
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/

                .syntax  unified
                .arch    armv7-m


/*
;<h> Stack Configuration
;  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Stack_Size, 0x00000400

                .section .stack
                .align   3
	            .globl   __StackTop
	            .globl   __StackLimit
__StackLimit:
                .space   Stack_Size
                .size    __StackLimit, . - __StackLimit
__StackTop:
                .size    __StackTop, . - __StackTop


/*
;<h> Heap Configuration
;  <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Heap_Size, 0x00000C00

                .if      Heap_Size != 0                     /* Heap is provided */
                .section .heap
                .align   3
	            .globl   __HeapBase
	            .globl   __HeapLimit
__HeapBase:
                .space   Heap_Size
                .size    __HeapBase, . - __HeapBase
__HeapLimit:
                .size    __HeapLimit, . - __HeapLimit
                .endif


                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVCall Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

	/* External interrupts */
                .long        WUT_Handler                 /*    0  Wake Up Timer interrupt */ 
                .long        EINT0_Handler               /*    1  External Interrupt 0 */ 
                .long        EINT1_Handler               /*    2  External Interrupt 1 */ 
                .long        EINT2_Handler               /*    3  External Interrupt 2 */ 
                .long        EINT3_Handler               /*    4  External Interrupt 3 */ 
                .long        EINT4_Handler               /*    5  External Interrupt 4 */ 
                .long        EINT5_Handler               /*    6  External Interrupt 5 */ 
                .long        EINT6_Handler               /*    7  External Interrupt 6 */ 
                .long        EINT7_Handler               /*    8  External Interrupt 7 */ 
                .long        EINT8_Handler               /*    9  External Interrupt 8 */ 
                .long        WDT_Handler                 /*    10 WDT Interrupt */ 
                .long        TIMER0_Handler              /*    11 Timer interrupt */ 
                .long        TIMER1_Handler              /*    12 Timer 1 Interrupt */ 
                .long        FLASH0_Handler              /*    13 Flash Controller Interrupt */ 
                .long        UART_Handler                /*    14 interrupt */ 
                .long        SPI0_Handler                /*    15 SPI 0 interrupt */ 
                .long        SPIH_Handler                /*    16 interrupt */ 
                .long        I2CS_Handler                /*    17 I2C 0 slave interrupt */ 
                .long        I2CM_Handler                /*    18 I2C 0 master interrupt */ 
                .long        DMA_ERR_Handler             /*    19 DMA interrupt */ 
                .long        DMA_SPIH_TX_Handler         /*    20 DMA Ch 0 interrupt */ 
                .long        DMA_SPIH_RX_Handler         /*    21 DMA Ch 1 interrupt */ 
                .long        DMA_SPI0_TX_Handler         /*    22 DMA Ch 2 interrupt */ 
                .long        DMA_SPI0_RX_Handler         /*    23 DMA Ch 3 interrupt */ 
                .long        DMA_SPI1_TX_Handler         /*    24 DMA Ch 4 interrupt */ 
                .long        DMA_SPI1_RX_Handler         /*    25 DMA Ch 5 interrupt */ 
                .long        DMA_UART_TX_Handler         /*    26 DMA Ch 6 interrupt */ 
                .long        DMA_UART_RX_Handler         /*    27 DMA Ch 7 interrupt */ 
                .long        DMA_I2CS_TX_Handler         /*    28 DMA Ch 8 interrupt */ 
                .long        DMA_I2CS_RX_Handler         /*    29 DMA Ch 9 interrupt */ 
                .long        DMA_I2CM_Handler            /*    30 DMA Ch 10 interrupt */
                .long    	DMA_AFE_TX_Handler          /*    31 DMA Ch 11 interrupt */ 
                .long        DMA_AFE_RX_Handler          /*    32 DMA Ch 12 interrupt */ 
                .long        DMA_CRC_Handler             /*    33 DMA Ch 13 interrupt */ 
                .long        DMA_PDI_Handler             /*    34 DMA Ch 14 interrupt */ 
                .long        DMA_I2S_Handler             /*    35 DMA Ch 15 interrupt */ 
                .long        USB_WAKEUP_Handler          /*    36 USB Wakeup interrupt */ 
                .long        USB_CNTL_Handler            /*    37 USB Controller interrupt */ 
                .long        USB_DMA_Handler             /*    38 USB DMA interrupt */ 
                .long        I2S_Handler                 /*    39 I2S interrupt */ 
                .long        TIMER2_Handler              /*    40 TIMER 2 interrupt */ 
                .long        SPI1_Handler                /*    42 interrupt */ 
                .long        RTC_Handler                 /*    43 Real Time Clock interrupt */ 
                .long        BEEP_Handler                /*    45 Beep interrupt */ 
                .long        LCD_Handler                 /*    46 LCD Controller interrupt */ 
                .long        GPIOA_Handler               /*    47 interrupt */ 
                .long        GPIOB_Handler               /*    48 interrupt */ 
                .long        AFE_CAPTURE_Handler         /*    50 Analog Front End Capture interrupt */ 
                .long        AFE_GENERATE_Handler        /*    51 Analog Front End Generation interrupt */ 
                .long        AFE_CMD_FIFO_Handler        /*    52 Analog Front End FIFO CMD interrupt */ 
                .long        AFE_DATA_FIFO_Handler       /*    53 Analog Front End FIFO DATA interrupt */ 
                .long        GP_FLASH_Handler            /*    55 Flash EEPROM interrupt */ 
                .long        RAND_Handler                /*    58 Random Bit Generator interrupt */ 
                .long        PDI_Handler                 /*    59 Paraller Display Interface interrupt */ 

                .space   (163 * 4)                          /* Interrupts 10 .. 224 are left out */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
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
                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0

.L_loop0_done:
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
                ldr      r1, =__etext
                ldr      r2, =__data_start__
                ldr      r3, =__data_end__

.L_loop1:
                cmp      r2, r3
                ittt     lt
                ldrlt    r0, [r1], #4
                strlt    r0, [r2], #4
                blt      .L_loop1
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
                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/* Single BSS section scheme.
 *
 * The BSS section is specified by following symbols
 *   __bss_start__: start of the BSS section.
 *   __bss_end__: end of the BSS section.
 *
 * Both addresses must be aligned to 4 bytes boundary.
 */
                ldr      r1, =__bss_start__
                ldr      r2, =__bss_end__

                movs     r0, 0
.L_loop3:
                cmp      r1, r2
                itt      lt
                strlt    r0, [r1], #4
                blt      .L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

                bl       SystemInit
                bl       _start

                .fnend
                .size    Reset_Handler, . - Reset_Handler


                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  HardFault_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                Set_Default_Handler  WUT_Handler                 
                Set_Default_Handler  EINT0_Handler               
                Set_Default_Handler  EINT1_Handler               
                Set_Default_Handler  EINT2_Handler               
                Set_Default_Handler  EINT3_Handler               
                Set_Default_Handler  EINT4_Handler               
                Set_Default_Handler  EINT5_Handler               
                Set_Default_Handler  EINT6_Handler               
                Set_Default_Handler  EINT7_Handler               
                Set_Default_Handler  EINT8_Handler               
                Set_Default_Handler  WDT_Handler                 
                Set_Default_Handler  TIMER0_Handler              
                Set_Default_Handler  TIMER1_Handler              
                Set_Default_Handler  FLASH0_Handler              
                Set_Default_Handler  UART_Handler                
                Set_Default_Handler  SPI0_Handler                
                Set_Default_Handler  SPIH_Handler                
                Set_Default_Handler  I2CS_Handler                
                Set_Default_Handler  I2CM_Handler                
                Set_Default_Handler  DMA_ERR_Handler             
                Set_Default_Handler  DMA_SPIH_TX_Handler         
                Set_Default_Handler  DMA_SPIH_RX_Handler         
                Set_Default_Handler  DMA_SPI0_TX_Handler         
                Set_Default_Handler  DMA_SPI0_RX_Handler         
                Set_Default_Handler  DMA_SPI1_TX_Handler         
                Set_Default_Handler  DMA_SPI1_RX_Handler         
                Set_Default_Handler  DMA_UART_TX_Handler         
                Set_Default_Handler  DMA_UART_RX_Handler         
                Set_Default_Handler  DMA_I2CS_TX_Handler         
                Set_Default_Handler  DMA_I2CS_RX_Handler         
                Set_Default_Handler  DMA_I2CM_Handler            
                Set_Default_Handler  DMA_AFE_TX_Handler          
                Set_Default_Handler  DMA_AFE_RX_Handler          
                Set_Default_Handler  DMA_CRC_Handler             
                Set_Default_Handler  DMA_PDI_Handler             
                Set_Default_Handler  DMA_I2S_Handler             
                Set_Default_Handler  USB_WAKEUP_Handler          
                Set_Default_Handler  USB_CNTL_Handler            
                Set_Default_Handler  USB_DMA_Handler             
                Set_Default_Handler  I2S_Handler                 
                Set_Default_Handler  TIMER2_Handler              
                Set_Default_Handler  SPI1_Handler                
                Set_Default_Handler  RTC_Handler                 
                Set_Default_Handler  BEEP_Handler                
                Set_Default_Handler  LCD_Handler                 
                Set_Default_Handler  GPIOA_Handler               
                Set_Default_Handler  GPIOB_Handler               
                Set_Default_Handler  AFE_CAPTURE_Handler         
                Set_Default_Handler  AFE_GENERATE_Handler        
                Set_Default_Handler  AFE_CMD_FIFO_Handler        
                Set_Default_Handler  AFE_DATA_FIFO_Handler       
                Set_Default_Handler  GP_FLASH_Handler            
                Set_Default_Handler  RAND_Handler                
                Set_Default_Handler  PDI_Handler 


                .end
