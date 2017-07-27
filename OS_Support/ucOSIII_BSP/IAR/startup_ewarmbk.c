//*****************************************************************************
//
// startup_ewarm.c - Startup code for use with IAR's Embedded Workbench,
//                   version 5.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include  "lib_def.h"
#include  "bsp_int.h"
#include  <os.h>
//*****************************************************************************
//
// Enable the IAR extensions for this source file.
//
//*****************************************************************************
#pragma language=extended
//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
static  void  App_Reset_ISR       (void);

static  void  App_NMI_ISR         (void);

static  void  App_Fault_ISR       (void);

static  void  App_BusFault_ISR    (void);

static  void  App_UsageFault_ISR  (void);

static  void  App_MemFault_ISR    (void);

static  void  App_Spurious_ISR    (void);

extern  void  __iar_program_start (void);

//*****************************************************************************
//
// The entry point for the application startup code.
//
//*****************************************************************************
extern void __iar_program_start(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static uint32_t pui32Stack[64] @ ".noinit";

//*****************************************************************************
//
// A union that describes the entries of the vector table.  The union is needed
// since the first entry is the stack pointer and the remainder are function
// pointers.
//
//*****************************************************************************
typedef union
{
    void (*pfnHandler)(void);
    uint32_t ui32Ptr;
}
uVectorEntry;

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__root const uVectorEntry __vector_table[] @ ".intvec" =
{
    { .ui32Ptr = (uint32_t)pui32Stack + sizeof(pui32Stack) },
                                            // The initial stack pointer
    App_Reset_ISR,                               // The reset handler
    App_NMI_ISR,                                  // The NMI handler
    App_Fault_ISR,                               // The hard fault handler
    App_MemFault_ISR,                      // The MPU fault handler
    App_BusFault_ISR,                      // The bus fault handler
    App_UsageFault_ISR,                      // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    App_Spurious_ISR,                                      // SVCall handler
    App_Spurious_ISR,                                      // Debug monitor handler
    0,                                      // Reserved
    OS_CPU_PendSVHandler,                      // The PendSV handler
    OS_CPU_SysTickHandler,                      // The SysTick handler
    
    BSP_IntHandlerGPIOA,                      // GPIO Port A
    BSP_IntHandlerGPIOB,                      // GPIO Port B
    BSP_IntHandlerGPIOC,                      // GPIO Port C
    BSP_IntHandlerGPIOD,                      // GPIO Port D
    BSP_IntHandlerGPIOE,                      // GPIO Port E
    BSP_IntHandlerUART0,                      // UART0 Rx and Tx
    BSP_IntHandlerUART1,                      // UART1 Rx and Tx
    BSP_IntHandlerSSI0,                      // SSI0 Rx and Tx
    BSP_IntHandlerI2C0,                      // I2C0 Master and Slave
    BSP_IntHandlerPWM_FAULT,                      // PWM Fault
    BSP_IntHandlerPWM_GEN0,                      // PWM Generator 0
    BSP_IntHandlerPWM_GEN1,                      // PWM Generator 1
    BSP_IntHandlerPWM_GEN2,                      // PWM Generator 2
    BSP_IntHandlerQEI0,                      // Quadrature Encoder 0
    BSP_IntHandlerADC0_0,                      // ADC Sequence 0
    BSP_IntHandlerADC0_1,                      // ADC Sequence 1
    BSP_IntHandlerADC0_2,                      // ADC Sequence 2
    BSP_IntHandlerADC0_3,                      // ADC Sequence 3
    BSP_IntHandlerWDTO_WDT1,                      // Watchdog timer
    BSP_IntHandlerTMR0A,                      // Timer 0 subtimer A
    BSP_IntHandlerTMR0B,                      // Timer 0 subtimer B
    BSP_IntHandlerTMR1A,                      // Timer 1 subtimer A
    BSP_IntHandlerTMR1B,                      // Timer 1 subtimer B
    BSP_IntHandlerTMR2A,                      // Timer 2 subtimer A
    BSP_IntHandlerTMR2B,                      // Timer 2 subtimer B
    BSP_IntHandlerACOMP0,                      // Analog Comparator 0
    BSP_IntHandlerACOMP1,                      // Analog Comparator 1
    BSP_IntHandlerACOMP2,                      // Analog Comparator 2
    BSP_IntHandlerSYS_CTRL,                      // System Control (PLL, OSC, BO)
    BSP_IntHandlerFLASH,                      // FLASH Control
    BSP_IntHandlerGPIOF,                      // GPIO Port F
    App_Spurious_ISR,                      // GPIO Port G
    App_Spurious_ISR,                      // GPIO Port H
    BSP_IntHandlerUART2,                      // UART2 Rx and Tx
    BSP_IntHandlerSSI1,                      // SSI1 Rx and Tx
    BSP_IntHandlerTMR3A,                      // Timer 3 subtimer A
    BSP_IntHandlerTMR3B,                      // Timer 3 subtimer B
    BSP_IntHandlerI2C1,                      // I2C1 Master and Slave
    App_Spurious_ISR,                      // Quadrature Encoder 1
    BSP_IntHandlerCAN0,                      // CAN0
    BSP_IntHandlerCAN1,                      // CAN1
    0,                                      // Reserved
    0,                                      // Reserved
    BSP_IntHandlerHIB,                      // Hibernate
    BSP_IntHandlerUSB_MAC,                      // USB0
    BSP_IntHandlerPWM_GEN3,                      // PWM Generator 3
    BSP_IntHandlerUDMA0_SOFT,                      // uDMA Software Transfer
    BSP_IntHandlerUDAM0_ERR,                      // uDMA Error
    BSP_IntHandlerADC1_0,                      // ADC1 Sequence 0
    BSP_IntHandlerADC1_1,                      // ADC1 Sequence 1
    BSP_IntHandlerADC1_2,                      // ADC1 Sequence 2
    BSP_IntHandlerADC1_3,                      // ADC1 Sequence 3
   
    0,                                      // Reserved
    0,                                      // Reserved
    App_Spurious_ISR,                      // GPIO Port J
    App_Spurious_ISR,                      // GPIO Port K
    App_Spurious_ISR,                      // GPIO Port L
    
    BSP_IntHandlerSSI2,                      // SSI2 Rx and Tx
    BSP_IntHandlerSSI3,                      // SSI3 Rx and Tx
    BSP_IntHandlerUART3,                      // UART3 Rx and Tx
    BSP_IntHandlerUART4,                      // UART4 Rx and Tx
    BSP_IntHandlerUART5,                      // UART5 Rx and Tx
    BSP_IntHandlerUART6,                      // UART6 Rx and Tx
    BSP_IntHandlerUART7,                      // UART7 Rx and Tx
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    BSP_IntHandlerI2C2,                      // I2C2 Master and Slave
    BSP_IntHandlerI2C3,                      // I2C3 Master and Slave
    BSP_IntHandlerTMR4A,                      // Timer 4 subtimer A
    BSP_IntHandlerTMR4B,                      // Timer 4 subtimer B
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    BSP_IntHandlerTMR5A,                      // Timer 5 subtimer A
    BSP_IntHandlerTMR5B,                      // Timer 5 subtimer B
    App_Spurious_ISR,                      // Wide Timer 0 subtimer A
    App_Spurious_ISR,                      // Wide Timer 0 subtimer B
    App_Spurious_ISR,                      // Wide Timer 1 subtimer A
    App_Spurious_ISR,                      // Wide Timer 1 subtimer B
    App_Spurious_ISR,                      // Wide Timer 2 subtimer A
    App_Spurious_ISR,                      // Wide Timer 2 subtimer B
    App_Spurious_ISR,                      // Wide Timer 3 subtimer A
    App_Spurious_ISR,                      // Wide Timer 3 subtimer B
    App_Spurious_ISR,                      // Wide Timer 4 subtimer A
    App_Spurious_ISR,                      // Wide Timer 4 subtimer B
    App_Spurious_ISR,                      // Wide Timer 5 subtimer A
    App_Spurious_ISR,                      // Wide Timer 5 subtimer B
    App_Spurious_ISR,                      // FPU
    0,                                      // Reserved
    0,                                      // Reserved
    App_Spurious_ISR,                      // I2C4 Master and Slave
    App_Spurious_ISR,                      // I2C5 Master and Slave
    App_Spurious_ISR,                      // GPIO Port M
    App_Spurious_ISR,                      // GPIO Port N
    App_Spurious_ISR,                      // Quadrature Encoder 2
    0,                                      // Reserved
    0,                                      // Reserved
    App_Spurious_ISR,                      // GPIO Port P (Summary or P0)
    App_Spurious_ISR,                      // GPIO Port P1
    App_Spurious_ISR,                      // GPIO Port P2
    App_Spurious_ISR,                      // GPIO Port P3
    App_Spurious_ISR,                      // GPIO Port P4
    App_Spurious_ISR,                      // GPIO Port P5
    App_Spurious_ISR,                      // GPIO Port P6
    App_Spurious_ISR,                      // GPIO Port P7
    App_Spurious_ISR,                      // GPIO Port Q (Summary or Q0)
    App_Spurious_ISR,                      // GPIO Port Q1
    App_Spurious_ISR,                      // GPIO Port Q2
    App_Spurious_ISR,                      // GPIO Port Q3
    App_Spurious_ISR,                      // GPIO Port Q4
    App_Spurious_ISR,                      // GPIO Port Q5
    App_Spurious_ISR,                      // GPIO Port Q6
    App_Spurious_ISR,                      // GPIO Port Q7
    App_Spurious_ISR,                      // GPIO Port R
    App_Spurious_ISR,                      // GPIO Port S
    App_Spurious_ISR,                      // PWM 1 Generator 0
    App_Spurious_ISR,                      // PWM 1 Generator 1
    App_Spurious_ISR,                      // PWM 1 Generator 2
    App_Spurious_ISR,                      // PWM 1 Generator 3
    App_Spurious_ISR                       // PWM 1 Fault
};

/*
*********************************************************************************************************
*                                            App_Reset_ISR()
*
* Description : Handle Reset.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Reset_ISR (void)
{
#if __ARMVFP__                                                  /* Enable access to Floating-point coprocessor.         */
    CPU_REG_NVIC_CPACR = CPU_REG_NVIC_CPACR_CP10_FULL_ACCESS | CPU_REG_NVIC_CPACR_CP11_FULL_ACCESS;

    DEF_BIT_CLR(CPU_REG_SCB_FPCCR, DEF_BIT_31);                 /* Disable automatic FP register content                */
    DEF_BIT_CLR(CPU_REG_SCB_FPCCR, DEF_BIT_30);                 /* Disable Lazy context switch                          */
#endif

    __iar_program_start();
}

/*
*********************************************************************************************************
*                                            App_NMI_ISR()
*
* Description : Handle Non-Maskable Interrupt (NMI).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : (1) Since the NMI is not being used, this serves merely as a catch for a spurious
*                   exception.
*********************************************************************************************************
*/

static  void  App_NMI_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                             App_Fault_ISR()
*
* Description : Handle hard fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Fault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                           App_BusFault_ISR()
*
* Description : Handle bus fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_BusFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                          App_UsageFault_ISR()
*
* Description : Handle usage fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_UsageFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                           App_MemFault_ISR()
*
* Description : Handle memory fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_MemFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}

/*
*********************************************************************************************************
*                                           App_Spurious_ISR()
*
* Description : Handle spurious interrupt.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Spurious_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}
