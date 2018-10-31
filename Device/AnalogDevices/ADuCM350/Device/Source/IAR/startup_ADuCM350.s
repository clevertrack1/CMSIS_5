;/**************************************************************************//**
; * @file     startup_ADuCM350.s
; * @brief    CMSIS Cortex-M# Core Device Startup File for
; *           Device ADuCM350
; * @version  V5.3.1
; * @date     09. July 2018
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

                MODULE   ?cstartup

                ;; Forward declaration of sections.
                SECTION  CSTACK:DATA:NOROOT(3)

                SECTION  .intvec:CODE:NOROOT(2)

                EXTERN   __iar_program_start
                EXTERN   SystemInit
                PUBLIC   __vector_table
                PUBLIC   __vector_table_0x1c
                PUBLIC   __Vectors
                PUBLIC   __Vectors_End
                PUBLIC   __Vectors_Size

                DATA

__vector_table
                DCD      sfe(CSTACK)                         ;     Top of Stack
                DCD      Reset_Handler                       ;     Reset Handler
                DCD      NMI_Handler                         ; -14 NMI Handler
                DCD      HardFault_Handler                   ; -13 Hard Fault Handler
                DCD      MemManage_Handler                   ; -12 MPU Fault Handler
                DCD      BusFault_Handler                    ; -11 Bus Fault Handler
                DCD      UsageFault_Handler                  ; -10 Usage Fault Handler
__vector_table_0x1c
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      SVC_Handler                         ;  -5 SVCall Handler
                DCD      DebugMon_Handler                    ;  -4 Debug Monitor Handler
                DCD      0                                   ;     Reserved
                DCD      PendSV_Handler                      ;  -2 PendSV Handler
                DCD      SysTick_Handler                     ;  -1 SysTick Handler

        ; External Interrupts
                DCD  WUT_Handler;                
                DCD  EINT0_Handler;              
                DCD  EINT1_Handler;              
                DCD  EINT2_Handler;              
                DCD  EINT3_Handler;              
                DCD  EINT4_Handler;              
                DCD  EINT5_Handler;              
                DCD  EINT6_Handler;              
                DCD  EINT7_Handler;              
                DCD  EINT8_Handler;              
                DCD  WDT_Handler;                
                DCD  TIMER0_Handler;             
                DCD  TIMER1_Handler;             
                DCD  FLASH0_Handler;             
                DCD  UART_Handler;               
                DCD  SPI0_Handler;               
                DCD  SPIH_Handler;               
                DCD  I2CS_Handler;               
                DCD  I2CM_Handler;               
                DCD  DMA_ERR_Handler;            
                DCD  DMA_SPIH_TX_Handler;        
                DCD  DMA_SPIH_RX_Handler;        
                DCD  DMA_SPI0_TX_Handler;        
                DCD  DMA_SPI0_RX_Handler;        
                DCD  DMA_SPI1_TX_Handler;        
                DCD  DMA_SPI1_RX_Handler;        
                DCD  DMA_UART_TX_Handler;        
                DCD  DMA_UART_RX_Handler;        
                DCD  DMA_I2CS_TX_Handler;        
                DCD  DMA_I2CS_RX_Handler;        
                DCD  DMA_I2CM_Handler;           
                DCD  DMA_AFE_TX_Handler;         
                DCD  DMA_AFE_RX_Handler;         
                DCD  DMA_CRC_Handler;            
                DCD  DMA_PDI_Handler;            
                DCD  DMA_I2S_Handler;            
                DCD  USB_WAKEUP_Handler;         
                DCD  USB_CNTL_Handler;           
                DCD  USB_DMA_Handler;            
                DCD  I2S_Handler;                
                DCD  TIMER2_Handler;             
                DCD  SPI1_Handler;               
                DCD  RTC_Handler;                
                DCD  BEEP_Handler;               
                DCD  LCD_Handler;                
                DCD  GPIOA_Handler;              
                DCD  GPIOB_Handler;              
                DCD  AFE_CAPTURE_Handler;        
                DCD  AFE_GENERATE_Handler;       
                DCD  AFE_CMD_FIFO_Handler;       
                DCD  AFE_DATA_FIFO_Handler;      
                DCD  GP_FLASH_Handler;           
                DCD  RAND_Handler;               
                DCD  PDI_Handler;                

                DS32    (163)                                ; Interrupts 60 .. 224 are left out
__Vectors_End

__Vectors       EQU      __vector_table
__Vectors_Size  EQU      __Vectors_End - __Vectors


                THUMB

; Reset Handler

                PUBWEAK  Reset_Handler
                SECTION  .text:CODE:REORDER:NOROOT(2)
Reset_Handler
                LDR      R0, =SystemInit
                BLX      R0
                LDR      R0, =__iar_program_start
                BX       R0


                PUBWEAK NMI_Handler
                PUBWEAK HardFault_Handler
                PUBWEAK MemManage_Handler
                PUBWEAK BusFault_Handler
                PUBWEAK UsageFault_Handler
                PUBWEAK SVC_Handler
                PUBWEAK DebugMon_Handler
                PUBWEAK PendSV_Handler
                PUBWEAK SysTick_Handler

                PUBWEAK Interrupt0_Handler
                PUBWEAK Interrupt1_Handler
                PUBWEAK Interrupt2_Handler
                PUBWEAK Interrupt3_Handler
                PUBWEAK Interrupt4_Handler
                PUBWEAK Interrupt5_Handler
                PUBWEAK Interrupt6_Handler
                PUBWEAK Interrupt7_Handler
                PUBWEAK Interrupt8_Handler
                PUBWEAK Interrupt9_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler

Interrupt0_Handler
Interrupt1_Handler
Interrupt2_Handler
Interrupt3_Handler
Interrupt4_Handler
Interrupt5_Handler
Interrupt6_Handler
Interrupt7_Handler
Interrupt8_Handler
Interrupt9_Handler
Default_Handler
                B        .


                END
