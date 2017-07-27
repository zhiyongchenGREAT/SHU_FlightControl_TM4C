/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                         BOARD SUPPORT PACKAGE
*
*                                      Texas Instruments TM4C129x
*                                                on the
*
*                                             DK-TM4C129X
*                                           Development Kit
*
* Filename      : bsp_int.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_INT_MODULE

#include  <cpu.h>
#include  <os.h>
#include  <bsp_int.h>
#include  <lib_def.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/

static CPU_FNCT_VOID BSP_IntVectTbl[BSP_INT_ID_MAX];


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  BSP_IntHandlerDummy (void);


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          BSP_IntClr()
*
* Description : This function clears an interrupt
*
* Argument(s) : int_id        Interrupt to clear.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) An interrupt does not need to be cleared within the interrupt controller.
*********************************************************************************************************
*/

void  BSP_IntClr (CPU_INT08U  int_id)
{

}


/*
*********************************************************************************************************
*                                             BSP_IntDis()
*
* Description : This function disables an interrupt.
*
* Argument(s) : int_id        Interrupt to clear.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
* Note(s)     : Disable the specified interrupt by writing a 1 to the Interrupt
*               Mask Flag Register.
*********************************************************************************************************
*/

void  BSP_IntDis (CPU_INT08U  int_id)
{
                                                                /* -------------- ARGUMENTS CHECKING ---------------- */
    if (int_id < BSP_INT_ID_MAX) {
        CPU_IntSrcDis(int_id + 16u);
    }
}


/*
*********************************************************************************************************
*                                           BSP_IntDisAll()
*
* Description : Disable ALL interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntDisAll (void)
{
    CPU_IntDis();
}

/*
*********************************************************************************************************
*                                          BSP_IntEn()
*
* Description : This function enables an interrupt.
*
* Argument(s) : int_id        Interrupt to clear.
*
* Returns     : none.
*
* Caller(s)   : Application.
*
* Note(s)     : Enable interrupt by setting a 1 to the Interrupt Mask
*               Flag Register.
*********************************************************************************************************
*/

void  BSP_IntEn (CPU_INT08U  int_id)
{
    if (int_id < BSP_INT_ID_MAX) {
        CPU_IntSrcEn(int_id + 16u);
    }
}


/*
*********************************************************************************************************
*                                            BSP_IntVectSet()
*
* Description : Assign ISR handler.
*
* Argument(s) : int_id      Interrupt for which vector will be set.
*
*               prio        Interrupt priority level. 0(Highest) - 3(Lowest)
*
*               isr_fnct    Handler to assign
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntVectSet (CPU_INT08U     int_id,
                      CPU_FNCT_VOID  isr_fnct)
{
    CPU_SR_ALLOC();


    if (int_id < BSP_INT_ID_MAX) {
        CPU_CRITICAL_ENTER();
        BSP_IntVectTbl[int_id] = isr_fnct;                      /* Setup interrupt specified in the vector table        */
        CPU_CRITICAL_EXIT();
    }
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           INTERNAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              BSP_IntInit()
*
* Description : Initialize interrupts:
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntInit (void)
{
    CPU_INT32U  int_id;


    for (int_id = 0u; int_id < BSP_INT_ID_MAX; int_id++) {      /* Initialize each interrupt with Dummy Handler         */
        BSP_IntVectSet((CPU_INT08U   )int_id,
                       (CPU_FNCT_VOID)BSP_IntHandlerDummy);
    }
}

/*
*********************************************************************************************************
*                                        BSP_IntHandler####()
*
* Description : Handle an interrupt.
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

void  BSP_IntHandlerGPIOA     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOA);                 }
void  BSP_IntHandlerGPIOB     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOB);                 }
void  BSP_IntHandlerGPIOC     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOC);                 }
void  BSP_IntHandlerGPIOD     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOD);                 }
void  BSP_IntHandlerGPIOE     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOE);                 }
void  BSP_IntHandlerUART0     (void)  { BSP_IntHandler(BSP_INT_ID_UART0);                 }
void  BSP_IntHandlerUART1     (void)  { BSP_IntHandler(BSP_INT_ID_UART1);                 }
void  BSP_IntHandlerSSI0      (void)  { BSP_IntHandler(BSP_INT_ID_SSI0);                  }
void  BSP_IntHandlerI2C0      (void)  { BSP_IntHandler(BSP_INT_ID_I2C0);                  }
void  BSP_IntHandlerPWM_FAULT (void)  { BSP_IntHandler(BSP_INT_ID_PWM_FAULT);             }
void  BSP_IntHandlerPWM_GEN0  (void)  { BSP_IntHandler(BSP_INT_ID_PWM_GEN0);              }
void  BSP_IntHandlerPWM_GEN1  (void)  { BSP_IntHandler(BSP_INT_ID_PWM_GEN1);              }
void  BSP_IntHandlerPWM_GEN2  (void)  { BSP_IntHandler(BSP_INT_ID_PWM_GEN2);              }
void  BSP_IntHandlerQEI0      (void)  { BSP_IntHandler(BSP_INT_ID_QEI0);                  }
void  BSP_IntHandlerADC0_0    (void)  { BSP_IntHandler(BSP_INT_ID_ADC0_0);                }
void  BSP_IntHandlerADC0_1    (void)  { BSP_IntHandler(BSP_INT_ID_ADC0_1);                }
void  BSP_IntHandlerADC0_2    (void)  { BSP_IntHandler(BSP_INT_ID_ADC0_2);                }
void  BSP_IntHandlerADC0_3    (void)  { BSP_IntHandler(BSP_INT_ID_ADC0_3);                }
void  BSP_IntHandlerWDTO_WDT1 (void)  { BSP_IntHandler(BSP_INT_ID_WDTO_WDT1);             }
void  BSP_IntHandlerTMR0A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR0A);                 }
void  BSP_IntHandlerTMR0B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR0B);                 }
void  BSP_IntHandlerTMR1A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR1A);                 }
void  BSP_IntHandlerTMR1B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR1B);                 }
void  BSP_IntHandlerTMR2A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR2A);                 }
void  BSP_IntHandlerTMR2B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR2B);                 }
void  BSP_IntHandlerACOMP0    (void)  { BSP_IntHandler(BSP_INT_ID_ACOMP0);                }
void  BSP_IntHandlerACOMP1    (void)  { BSP_IntHandler(BSP_INT_ID_ACOMP1);                }
void  BSP_IntHandlerACOMP2    (void)  { BSP_IntHandler(BSP_INT_ID_ACOMP2);                }
void  BSP_IntHandlerSYS_CTRL  (void)  { BSP_IntHandler(BSP_INT_ID_SYS_CTRL);              }
void  BSP_IntHandlerFLASH     (void)  { BSP_IntHandler(BSP_INT_ID_FLASH);                 }
void  BSP_IntHandlerGPIOF     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOF);                 }
void  BSP_IntHandlerGPIOG     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOG);                 }
void  BSP_IntHandlerGPIOH     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOH);                 }
void  BSP_IntHandlerUART2     (void)  { BSP_IntHandler(BSP_INT_ID_UART2);                 }
void  BSP_IntHandlerSSI1      (void)  { BSP_IntHandler(BSP_INT_ID_SSI1);                  }
void  BSP_IntHandlerTMR3A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR3A);                 }
void  BSP_IntHandlerTMR3B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR3B);                 }
void  BSP_IntHandlerI2C1      (void)  { BSP_IntHandler(BSP_INT_ID_I2C1);                  }
void  BSP_IntHandlerCAN0      (void)  { BSP_IntHandler(BSP_INT_ID_CAN0);                  }
void  BSP_IntHandlerCAN1      (void)  { BSP_IntHandler(BSP_INT_ID_CAN1);                  }
void  BSP_IntHandlerETHER_MAC (void)  { BSP_IntHandler(BSP_INT_ID_ETHER_MAC);             }
void  BSP_IntHandlerHIB       (void)  { BSP_IntHandler(BSP_INT_ID_HIB);                   }
void  BSP_IntHandlerUSB_MAC   (void)  { BSP_IntHandler(BSP_INT_ID_USB_MAC);               }
void  BSP_IntHandlerPWM_GEN3  (void)  { BSP_IntHandler(BSP_INT_ID_PWM_GEN3);              }
void  BSP_IntHandlerUDMA0_SOFT(void)  { BSP_IntHandler(BSP_INT_ID_UDMA0_SOFT);            }
void  BSP_IntHandlerUDAM0_ERR (void)  { BSP_IntHandler(BSP_INT_ID_UDAM0_ERR);             }
void  BSP_IntHandlerADC1_0    (void)  { BSP_IntHandler(BSP_INT_ID_ADC1_0);                }
void  BSP_IntHandlerADC1_1    (void)  { BSP_IntHandler(BSP_INT_ID_ADC1_1);                }
void  BSP_IntHandlerADC1_2    (void)  { BSP_IntHandler(BSP_INT_ID_ADC1_2);                }
void  BSP_IntHandlerADC1_3    (void)  { BSP_IntHandler(BSP_INT_ID_ADC1_3);                }
void  BSP_IntHandlerEPI0      (void)  { BSP_IntHandler(BSP_INT_ID_EPI0);                  }
void  BSP_IntHandlerGPIOJ     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOJ);                 }
void  BSP_IntHandlerGPIOK     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOK);                 }
void  BSP_IntHandlerGPIOL     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOL);                 }
void  BSP_IntHandlerSSI2      (void)  { BSP_IntHandler(BSP_INT_ID_SSI2);                  }
void  BSP_IntHandlerSSI3      (void)  { BSP_IntHandler(BSP_INT_ID_SSI3);                  }
void  BSP_IntHandlerUART3     (void)  { BSP_IntHandler(BSP_INT_ID_UART3);                 }
void  BSP_IntHandlerUART4     (void)  { BSP_IntHandler(BSP_INT_ID_UART4);                 }
void  BSP_IntHandlerUART5     (void)  { BSP_IntHandler(BSP_INT_ID_UART5);                 }
void  BSP_IntHandlerUART6     (void)  { BSP_IntHandler(BSP_INT_ID_UART6);                 }
void  BSP_IntHandlerUART7     (void)  { BSP_IntHandler(BSP_INT_ID_UART7);                 }
void  BSP_IntHandlerI2C2      (void)  { BSP_IntHandler(BSP_INT_ID_I2C2);                  }
void  BSP_IntHandlerI2C3      (void)  { BSP_IntHandler(BSP_INT_ID_I2C3);                  }
void  BSP_IntHandlerTMR4A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR4A);                 }
void  BSP_IntHandlerTMR4B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR4B);                 }
void  BSP_IntHandlerTMR5A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR5A);                 }
void  BSP_IntHandlerTMR5B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR5B);                 }
void  BSP_IntHandlerFP        (void)  { BSP_IntHandler(BSP_INT_ID_FP);                    }

void  BSP_IntHandlerRSVD68    (void)  { BSP_IntHandler(BSP_INT_ID_RSVD68);                }
void  BSP_IntHandlerRSVD69    (void)  { BSP_IntHandler(BSP_INT_ID_RSVD69);                }

void  BSP_IntHandlerI2C4      (void)  { BSP_IntHandler(BSP_INT_ID_I2C4);                  }
void  BSP_IntHandlerI2C5      (void)  { BSP_IntHandler(BSP_INT_ID_I2C5);                  }
void  BSP_IntHandlerGPIOM     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOM);                 }
void  BSP_IntHandlerGPION     (void)  { BSP_IntHandler(BSP_INT_ID_GPION);                 }

void  BSP_IntHandlerRSVD74    (void)  { BSP_IntHandler(BSP_INT_ID_RSVD74);                }

void  BSP_IntHandlerTAMPER    (void)  { BSP_IntHandler(BSP_INT_ID_TAMPER);                }
void  BSP_IntHandlerGPIOP0    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP0);                }
void  BSP_IntHandlerGPIOP1    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP1);                }
void  BSP_IntHandlerGPIOP2    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP2);                }
void  BSP_IntHandlerGPIOP3    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP3);                }
void  BSP_IntHandlerGPIOP4    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP4);                }
void  BSP_IntHandlerGPIOP5    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP5);                }
void  BSP_IntHandlerGPIOP6    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP6);                }
void  BSP_IntHandlerGPIOP7    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOP7);                }
void  BSP_IntHandlerGPIOQ0    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ0);                }
void  BSP_IntHandlerGPIOQ1    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ1);                }
void  BSP_IntHandlerGPIOQ2    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ2);                }
void  BSP_IntHandlerGPIOQ3    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ3);                }
void  BSP_IntHandlerGPIOQ4    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ4);                }
void  BSP_IntHandlerGPIOQ5    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ5);                }
void  BSP_IntHandlerGPIOQ6    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ6);                }
void  BSP_IntHandlerGPIOQ7    (void)  { BSP_IntHandler(BSP_INT_ID_GPIOQ7);                }
void  BSP_IntHandlerGPIOR     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOR);                 }
void  BSP_IntHandlerGPIOS     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOS);                 }
void  BSP_IntHandlerSHA_MD5   (void)  { BSP_IntHandler(BSP_INT_ID_SHA_MD5);               }
void  BSP_IntHandlerAES       (void)  { BSP_IntHandler(BSP_INT_ID_AES);                   }
void  BSP_IntHandlerDES       (void)  { BSP_IntHandler(BSP_INT_ID_DES);                   }
void  BSP_IntHandlerLCD       (void)  { BSP_IntHandler(BSP_INT_ID_LCD);                   }
void  BSP_IntHandlerTMR6A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR6A);                 }
void  BSP_IntHandlerTMR6B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR6B);                 }
void  BSP_IntHandlerTMR7A     (void)  { BSP_IntHandler(BSP_INT_ID_TMR7A);                 }
void  BSP_IntHandlerTMR7B     (void)  { BSP_IntHandler(BSP_INT_ID_TMR7B);                 }
void  BSP_IntHandlerI2C6      (void)  { BSP_IntHandler(BSP_INT_ID_I2C6);                  }
void  BSP_IntHandlerI2C7      (void)  { BSP_IntHandler(BSP_INT_ID_I2C7);                  }

void  BSP_IntHandlerRSVD104   (void)  { BSP_IntHandler(BSP_INT_ID_RSVD104);               }

void  BSP_IntHandler1WIRE     (void)  { BSP_IntHandler(BSP_INT_ID_1WIRE);                 }

void  BSP_IntHandlerRSVD106   (void)  { BSP_IntHandler(BSP_INT_ID_RSVD106);               }
void  BSP_IntHandlerRSVD107   (void)  { BSP_IntHandler(BSP_INT_ID_RSVD107);               }
void  BSP_IntHandlerRSVD108   (void)  { BSP_IntHandler(BSP_INT_ID_RSVD108);               }

void  BSP_IntHandlerI2C8      (void)  { BSP_IntHandler(BSP_INT_ID_I2C8);                  }
void  BSP_IntHandlerI2C9      (void)  { BSP_IntHandler(BSP_INT_ID_I2C9);                  }
void  BSP_IntHandlerGPIOT     (void)  { BSP_IntHandler(BSP_INT_ID_GPIOT);                 }


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        BSP_IntHandlerDummy()
*
* Description : Dummy interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_IntHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void  BSP_IntHandlerDummy (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*****************************************s****************************************************************
*                                        BSP_IntHandler()
*
* Description : General Maskable Interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : CPU_IntHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntHandler (CPU_INT16U  src_nbr)
{
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */
    OSIntEnter();
    CPU_CRITICAL_EXIT();

    if (src_nbr < BSP_INT_ID_MAX) {
        isr = BSP_IntVectTbl[src_nbr];
        if (isr != (CPU_FNCT_VOID)0) {
            isr();
        }
    }

    OSIntExit();                                                /* Tell the OS that we are leaving the ISR            */
}
