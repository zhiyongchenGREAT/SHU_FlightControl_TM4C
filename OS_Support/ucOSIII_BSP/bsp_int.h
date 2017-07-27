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
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP_INT present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_INT_PRESENT
#define  BSP_INT_PRESENT


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/


#ifdef   BSP_INT_MODULE
#define  BSP_INT_EXT
#else
#define  BSP_INT_EXT  extern
#endif

/*
*********************************************************************************************************
*                                        DEFAULT CONFIGURATION
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

                                                                /* ------------ INTERRUPT PRIORITY DEFINES ---------- */
#define  BSP_INT_PRIO_LEVEL_MASK             DEF_BIT_MASK(7, 0)
#define  BSP_INT_PRIO_LEVEL_00               0u
#define  BSP_INT_PRIO_LEVEL_01               1u
#define  BSP_INT_PRIO_LEVEL_02               2u
#define  BSP_INT_PRIO_LEVEL_03               3u


/*
*********************************************************************************************************
*                                           INTERRUPT DEFINES
*********************************************************************************************************
*/

#define  BSP_INT_ID_GPIOA                            0u         /*  16, INTISR[  0]  GPIO Port A.                       */
#define  BSP_INT_ID_GPIOB                            1u         /*  17, INTISR[  1]  GPIO Port B.                       */
#define  BSP_INT_ID_GPIOC                            2u         /*  18, INTISR[  2]  GPIO Port C.                       */
#define  BSP_INT_ID_GPIOD                            3u         /*  19, INTISR[  3]  GPIO Port D.                       */
#define  BSP_INT_ID_GPIOE                            4u         /*  20, INTISR[  4]  GPIO Port E.                       */
#define  BSP_INT_ID_UART0                            5u         /*  21, INTISR[  5]  UART0.                             */
#define  BSP_INT_ID_UART1                            6u         /*  22, INTISR[  6]  UART1.                             */
#define  BSP_INT_ID_SSI0                             7u         /*  23, INTISR[  7]  SSI0.                              */
#define  BSP_INT_ID_I2C0                             8u         /*  24, INTISR[  8]  I2C0.                              */
#define  BSP_INT_ID_PWM_FAULT                        9u         /*  25, INTISR[  9]  PWM Fault.                         */
#define  BSP_INT_ID_PWM_GEN0                        10u         /*  26, INTISR[ 10]  PWM Generator 0.                   */
#define  BSP_INT_ID_PWM_GEN1                        11u         /*  27, INTISR[ 11]  PWM Generator 1.                   */
#define  BSP_INT_ID_PWM_GEN2                        12u         /*  28, INTISR[ 12]  PWM Generator 2.                   */
#define  BSP_INT_ID_QEI0                            13u         /*  29, INTISR[ 13]  QEI0.                              */
#define  BSP_INT_ID_ADC0_0                          14u         /*  30, INTISR[ 14]  ADC0 Sequence 0.                   */
#define  BSP_INT_ID_ADC0_1                          15u         /*  31, INTISR[ 15]  ADC0 Sequence 1.                   */
#define  BSP_INT_ID_ADC0_2                          16u         /*  32, INTISR[ 16]  ADC0 Sequence 2.                   */
#define  BSP_INT_ID_ADC0_3                          17u         /*  33, INTISR[ 17]  ADC0 Sequence 3.                   */
#define  BSP_INT_ID_WDTO_WDT1                       18u         /*  34, INTISR[ 18]  Watchdog Timers 0 and 1.           */
#define  BSP_INT_ID_TMR0A                           19u         /*  35, INTISR[ 19]  16/32-Bit Timer 0A.                */
#define  BSP_INT_ID_TMR0B                           20u         /*  36, INTISR[ 20]  16/32-Bit Timer 0B.                */
#define  BSP_INT_ID_TMR1A                           21u         /*  37, INTISR[ 21]  16/32-Bit Timer 1A.                */
#define  BSP_INT_ID_TMR1B                           22u         /*  38, INTISR[ 22]  16/32-Bit Timer 1B.                */
#define  BSP_INT_ID_TMR2A                           23u         /*  39, INTISR[ 23]  16/32-Bit Timer 2A.                */
#define  BSP_INT_ID_TMR2B                           24u         /*  40, INTISR[ 24]  16/32-Bit Timer 2B.                */
#define  BSP_INT_ID_ACOMP0                          25u         /*  41, INTISR[ 25]  Analog Comparator 0.               */
#define  BSP_INT_ID_ACOMP1                          26u         /*  42, INTISR[ 26]  Analog Comparator 1.               */
#define  BSP_INT_ID_ACOMP2                          27u         /*  43, INTISR[ 27]  Analog Comparator 2.               */
#define  BSP_INT_ID_SYS_CTRL                        28u         /*  44, INTISR[ 28]  System Control.                    */
#define  BSP_INT_ID_FLASH                           29u         /*  45, INTISR[ 29]  Flash Memory Control.              */
#define  BSP_INT_ID_GPIOF                           30u         /*  46, INTISR[ 30]  GPIO Port F.                       */
#define  BSP_INT_ID_GPIOG                           31u         /*  47, INTISR[ 31]  GPIO Port G.                       */
#define  BSP_INT_ID_GPIOH                           32u         /*  48, INTISR[ 32]  GPIO Port H.                       */
#define  BSP_INT_ID_UART2                           33u         /*  49, INTISR[ 33]  UART2.                             */
#define  BSP_INT_ID_SSI1                            34u         /*  50, INTISR[ 34]  SSI1.                              */
#define  BSP_INT_ID_TMR3A                           35u         /*  51, INTISR[ 35]  16/32-Bit Timer 3A.                */
#define  BSP_INT_ID_TMR3B                           36u         /*  52, INTISR[ 36]  16/32-Bit Timer 3B.                */
#define  BSP_INT_ID_I2C1                            37u         /*  53, INTISR[ 37]  I2C1.                              */
#define  BSP_INT_ID_CAN0                            38u         /*  54, INTISR[ 38]  CAN0.                              */
#define  BSP_INT_ID_CAN1                            39u         /*  55, INTISR[ 39]  CAN1.                              */
#define  BSP_INT_ID_ETHER_MAC                       40u         /*  56, INTISR[ 40]  Ethernet MAC.                      */
#define  BSP_INT_ID_HIB                             41u         /*  57, INTISR[ 41]  HIB(Power Island).                 */
#define  BSP_INT_ID_USB_MAC                         42u         /*  58, INTISR[ 42]  USB MAC.                           */
#define  BSP_INT_ID_PWM_GEN3                        43u         /*  59, INTISR[ 43]  PWM Generator 3.                   */
#define  BSP_INT_ID_UDMA0_SOFT                      44u         /*  60, INTISR[ 44]  uDMA 0 Software.                   */
#define  BSP_INT_ID_UDAM0_ERR                       45u         /*  61, INTISR[ 45]  uDMA 0 Error.                      */
#define  BSP_INT_ID_ADC1_0                          46u         /*  62, INTISR[ 46]  ADC1 Sequence 0.                   */
#define  BSP_INT_ID_ADC1_1                          47u         /*  63, INTISR[ 47]  ADC1 Sequence 1.                   */
#define  BSP_INT_ID_ADC1_2                          48u         /*  64, INTISR[ 48]  ADC1 Sequence 2.                   */
#define  BSP_INT_ID_ADC1_3                          49u         /*  65, INTISR[ 49]  ADC1 Sequence 3.                   */
#define  BSP_INT_ID_EPI0                            50u         /*  66, INTISR[ 50]  EPI0.                              */
#define  BSP_INT_ID_GPIOJ                           51u         /*  67, INTISR[ 51]  GPIO Port J.                       */
#define  BSP_INT_ID_GPIOK                           52u         /*  68, INTISR[ 52]  GPIO Port K.                       */
#define  BSP_INT_ID_GPIOL                           53u         /*  69, INTISR[ 53]  GPIO Port L.                       */
#define  BSP_INT_ID_SSI2                            54u         /*  70, INTISR[ 54]  SSI2.                              */
#define  BSP_INT_ID_SSI3                            55u         /*  71, INTISR[ 55]  SSI3.                              */
#define  BSP_INT_ID_UART3                           56u         /*  72, INTISR[ 56]  UART3.                             */
#define  BSP_INT_ID_UART4                           57u         /*  73, INTISR[ 57]  UART4.                             */
#define  BSP_INT_ID_UART5                           58u         /*  74, INTISR[ 58]  UART5.                             */
#define  BSP_INT_ID_UART6                           59u         /*  75, INTISR[ 59]  UART6.                             */
#define  BSP_INT_ID_UART7                           60u         /*  76, INTISR[ 60]  UART7.                             */
#define  BSP_INT_ID_I2C2                            61u         /*  77, INTISR[ 61]  I2C 2.                             */
#define  BSP_INT_ID_I2C3                            62u         /*  78, INTISR[ 62]  I2C 3.                             */
#define  BSP_INT_ID_TMR4A                           63u         /*  79, INTISR[ 63]  Timer 4A.                          */
#define  BSP_INT_ID_TMR4B                           64u         /*  80, INTISR[ 64]  Timer 4B.                          */
#define  BSP_INT_ID_TMR5A                           65u         /*  81, INTISR[ 65]  Timer 5A.                          */
#define  BSP_INT_ID_TMR5B                           66u         /*  82, INTISR[ 66]  Timer 5B.                          */
#define  BSP_INT_ID_FP                              67u         /*  83, INTISR[ 67]  FP Exception(imprecise).           */

#define  BSP_INT_ID_RSVD68                          68u         /*  84, INTISR[ 68]  Reserved.                          */
#define  BSP_INT_ID_RSVD69                          69u         /*  85, INTISR[ 69]  Reserved.                          */

#define  BSP_INT_ID_I2C4                            70u         /*  86, INTISR[ 70]  I2C 4.                             */
#define  BSP_INT_ID_I2C5                            71u         /*  87, INTISR[ 71]  I2C 5.                             */
#define  BSP_INT_ID_GPIOM                           72u         /*  88, INTISR[ 72]  GPIO Port M.                       */
#define  BSP_INT_ID_GPION                           73u         /*  89, INTISR[ 73]  GPIO Port N.                       */

#define  BSP_INT_ID_RSVD74                          74u         /*  90, INTISR[ 74]  Reserved.                          */

#define  BSP_INT_ID_TAMPER                          75u         /*  91, INTISR[ 75]  Tamper.                            */
#define  BSP_INT_ID_GPIOP0                          76u         /*  92, INTISR[ 76]  GPIO Port P(Summary or P0).        */
#define  BSP_INT_ID_GPIOP1                          77u         /*  93, INTISR[ 77]  GPIO Port P1.                      */
#define  BSP_INT_ID_GPIOP2                          78u         /*  94, INTISR[ 78]  GPIO Port P2.                      */
#define  BSP_INT_ID_GPIOP3                          79u         /*  95, INTISR[ 79]  GPIO Port P3.                      */
#define  BSP_INT_ID_GPIOP4                          80u         /*  96, INTISR[ 80]  GPIO Port P4.                      */
#define  BSP_INT_ID_GPIOP5                          81u         /*  97, INTISR[ 81]  GPIO Port P5.                      */
#define  BSP_INT_ID_GPIOP6                          82u         /*  98, INTISR[ 82]  GPIO Port P6.                      */
#define  BSP_INT_ID_GPIOP7                          83u         /*  99, INTISR[ 83]  GPIO Port P7.                      */
#define  BSP_INT_ID_GPIOQ0                          84u         /* 100, INTISR[ 84]  GPIO Port Q(Summary or Q0).        */
#define  BSP_INT_ID_GPIOQ1                          85u         /* 101, INTISR[ 85]  GPIO Port Q1.                      */
#define  BSP_INT_ID_GPIOQ2                          86u         /* 102, INTISR[ 86]  GPIO Port Q2.                      */
#define  BSP_INT_ID_GPIOQ3                          87u         /* 103, INTISR[ 87]  GPIO Port Q3.                      */
#define  BSP_INT_ID_GPIOQ4                          88u         /* 104, INTISR[ 88]  GPIO Port Q4.                      */
#define  BSP_INT_ID_GPIOQ5                          89u         /* 105, INTISR[ 89]  GPIO Port Q5.                      */
#define  BSP_INT_ID_GPIOQ6                          90u         /* 106, INTISR[ 90]  GPIO Port Q6.                      */
#define  BSP_INT_ID_GPIOQ7                          91u         /* 107, INTISR[ 91]  GPIO Port Q7.                      */
#define  BSP_INT_ID_GPIOR                           92u         /* 108, INTISR[ 92]  GPIO Port R.                       */
#define  BSP_INT_ID_GPIOS                           93u         /* 109, INTISR[ 93]  GPIO Port S.                       */
#define  BSP_INT_ID_SHA_MD5                         94u         /* 110, INTISR[ 94]  SHA/MD5.                           */
#define  BSP_INT_ID_AES                             95u         /* 111, INTISR[ 95]  AES.                               */
#define  BSP_INT_ID_DES                             96u         /* 112, INTISR[ 96]  DES.                               */
#define  BSP_INT_ID_LCD                             97u         /* 113, INTISR[ 97]  LCD.                               */
#define  BSP_INT_ID_TMR6A                           98u         /* 114, INTISR[ 98]  16/32-Bit Timer 6A.                */
#define  BSP_INT_ID_TMR6B                           99u         /* 115, INTISR[ 99]  16/32-Bit Timer 6B.                */
#define  BSP_INT_ID_TMR7A                          100u         /* 116, INTISR[100]  16/32-Bit Timer 7A.                */
#define  BSP_INT_ID_TMR7B                          101u         /* 117, INTISR[101]  16/32-Bit Timer 7B.                */
#define  BSP_INT_ID_I2C6                           102u         /* 118, INTISR[102]  I2C 6.                             */
#define  BSP_INT_ID_I2C7                           103u         /* 119, INTISR[103]  I2C 7.                             */

#define  BSP_INT_ID_RSVD104                        104u         /* 120, INTISR[104]  Reserved.                          */

#define  BSP_INT_ID_1WIRE                          105u         /* 121, INTISR[105]  1-Wire.                            */

#define  BSP_INT_ID_RSVD106                        106u         /* 122, INTISR[106]  Reserved.                          */
#define  BSP_INT_ID_RSVD107                        107u         /* 123, INTISR[107]  Reserved.                          */
#define  BSP_INT_ID_RSVD108                        108u         /* 124, INTISR[108]  Reserved.                          */

#define  BSP_INT_ID_I2C8                           109u         /* 125, INTISR[109]  I2C 8.                             */
#define  BSP_INT_ID_I2C9                           110u         /* 126, INTISR[110]  I2C 9.                             */
#define  BSP_INT_ID_GPIOT                          111u         /* 127, INTISR[111]  GPIO T.                            */

#define  BSP_INT_ID_MAX                            112u


/*
*********************************************************************************************************
*                                             PERIPH DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  BSP_IntClr              (CPU_INT08U     int_id);
void  BSP_IntDis              (CPU_INT08U     int_id);
void  BSP_IntDisAll           (void);
void  BSP_IntEn               (CPU_INT08U     int_id);
void  BSP_IntInit             (void);
void  BSP_IntVectSet          (CPU_INT08U     int_id,
                               CPU_FNCT_VOID  isr_fnct);
void  BSP_IntHandler          (CPU_INT16U     src_nbr);

void  BSP_IntHandlerGPIOA     (void);
void  BSP_IntHandlerGPIOB     (void);
void  BSP_IntHandlerGPIOC     (void);
void  BSP_IntHandlerGPIOD     (void);
void  BSP_IntHandlerGPIOE     (void);
void  BSP_IntHandlerUART0     (void);
void  BSP_IntHandlerUART1     (void);
void  BSP_IntHandlerSSI0      (void);
void  BSP_IntHandlerI2C0      (void);
void  BSP_IntHandlerPWM_FAULT (void);
void  BSP_IntHandlerPWM_GEN0  (void);
void  BSP_IntHandlerPWM_GEN1  (void);
void  BSP_IntHandlerPWM_GEN2  (void);
void  BSP_IntHandlerQEI0      (void);
void  BSP_IntHandlerADC0_0    (void);
void  BSP_IntHandlerADC0_1    (void);
void  BSP_IntHandlerADC0_2    (void);
void  BSP_IntHandlerADC0_3    (void);
void  BSP_IntHandlerWDTO_WDT1 (void);
void  BSP_IntHandlerTMR0A     (void);
void  BSP_IntHandlerTMR0B     (void);
void  BSP_IntHandlerTMR1A     (void);
void  BSP_IntHandlerTMR1B     (void);
void  BSP_IntHandlerTMR2A     (void);
void  BSP_IntHandlerTMR2B     (void);
void  BSP_IntHandlerACOMP0    (void);
void  BSP_IntHandlerACOMP1    (void);
void  BSP_IntHandlerACOMP2    (void);
void  BSP_IntHandlerSYS_CTRL  (void);
void  BSP_IntHandlerFLASH     (void);
void  BSP_IntHandlerGPIOF     (void);
void  BSP_IntHandlerGPIOG     (void);
void  BSP_IntHandlerGPIOH     (void);
void  BSP_IntHandlerUART2     (void);
void  BSP_IntHandlerSSI1      (void);
void  BSP_IntHandlerTMR3A     (void);
void  BSP_IntHandlerTMR3B     (void);
void  BSP_IntHandlerI2C1      (void);
void  BSP_IntHandlerCAN0      (void);
void  BSP_IntHandlerCAN1      (void);
void  BSP_IntHandlerETHER_MAC (void);
void  BSP_IntHandlerHIB       (void);
void  BSP_IntHandlerUSB_MAC   (void);
void  BSP_IntHandlerPWM_GEN3  (void);
void  BSP_IntHandlerUDMA0_SOFT(void);
void  BSP_IntHandlerUDAM0_ERR (void);
void  BSP_IntHandlerADC1_0    (void);
void  BSP_IntHandlerADC1_1    (void);
void  BSP_IntHandlerADC1_2    (void);
void  BSP_IntHandlerADC1_3    (void);
void  BSP_IntHandlerEPI0      (void);
void  BSP_IntHandlerGPIOJ     (void);
void  BSP_IntHandlerGPIOK     (void);
void  BSP_IntHandlerGPIOL     (void);
void  BSP_IntHandlerSSI2      (void);
void  BSP_IntHandlerSSI3      (void);
void  BSP_IntHandlerUART3     (void);
void  BSP_IntHandlerUART4     (void);
void  BSP_IntHandlerUART5     (void);
void  BSP_IntHandlerUART6     (void);
void  BSP_IntHandlerUART7     (void);
void  BSP_IntHandlerI2C2      (void);
void  BSP_IntHandlerI2C3      (void);
void  BSP_IntHandlerTMR4A     (void);
void  BSP_IntHandlerTMR4B     (void);
void  BSP_IntHandlerTMR5A     (void);
void  BSP_IntHandlerTMR5B     (void);
void  BSP_IntHandlerFP        (void);

void  BSP_IntHandlerRSVD68    (void);
void  BSP_IntHandlerRSVD69    (void);

void  BSP_IntHandlerI2C4      (void);
void  BSP_IntHandlerI2C5      (void);
void  BSP_IntHandlerGPIOM     (void);
void  BSP_IntHandlerGPION     (void);

void  BSP_IntHandlerRSVD74    (void);

void  BSP_IntHandlerTAMPER    (void);
void  BSP_IntHandlerGPIOP0    (void);
void  BSP_IntHandlerGPIOP1    (void);
void  BSP_IntHandlerGPIOP2    (void);
void  BSP_IntHandlerGPIOP3    (void);
void  BSP_IntHandlerGPIOP4    (void);
void  BSP_IntHandlerGPIOP5    (void);
void  BSP_IntHandlerGPIOP6    (void);
void  BSP_IntHandlerGPIOP7    (void);
void  BSP_IntHandlerGPIOQ0    (void);
void  BSP_IntHandlerGPIOQ1    (void);
void  BSP_IntHandlerGPIOQ2    (void);
void  BSP_IntHandlerGPIOQ3    (void);
void  BSP_IntHandlerGPIOQ4    (void);
void  BSP_IntHandlerGPIOQ5    (void);
void  BSP_IntHandlerGPIOQ6    (void);
void  BSP_IntHandlerGPIOQ7    (void);
void  BSP_IntHandlerGPIOR     (void);
void  BSP_IntHandlerGPIOS     (void);
void  BSP_IntHandlerSHA_MD5   (void);
void  BSP_IntHandlerAES       (void);
void  BSP_IntHandlerDES       (void);
void  BSP_IntHandlerLCD       (void);
void  BSP_IntHandlerTMR6A     (void);
void  BSP_IntHandlerTMR6B     (void);
void  BSP_IntHandlerTMR7A     (void);
void  BSP_IntHandlerTMR7B     (void);
void  BSP_IntHandlerI2C6      (void);
void  BSP_IntHandlerI2C7      (void);

void  BSP_IntHandlerRSVD104   (void);

void  BSP_IntHandler1WIRE     (void);

void  BSP_IntHandlerRSVD106   (void);
void  BSP_IntHandlerRSVD107   (void);
void  BSP_IntHandlerRSVD108   (void);

void  BSP_IntHandlerI2C8      (void);
void  BSP_IntHandlerI2C9      (void);
void  BSP_IntHandlerGPIOT     (void);

/*
*********************************************************************************************************
*                                              ERROR CHECKING
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of module include.                               */

