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
* Filename      : bsp.h
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

#ifndef  BSP_H_
#define  BSP_H_


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  BSP_GPIO_PORTA                                        0u
#define  BSP_GPIO_PORTB                                        1u
#define  BSP_GPIO_PORTC                                        2u
#define  BSP_GPIO_PORTD                                        3u
#define  BSP_GPIO_PORTE                                        4u
#define  BSP_GPIO_PORTF                                        5u
#define  BSP_GPIO_PORTG                                        6u
#define  BSP_GPIO_PORTH                                        7u
#define  BSP_GPIO_PORTJ                                        8u
#define  BSP_GPIO_PORTK                                        9u
#define  BSP_GPIO_PORTL                                       10u
#define  BSP_GPIO_PORTM                                       11u
#define  BSP_GPIO_PORTN                                       12u
#define  BSP_GPIO_PORTP                                       13u
#define  BSP_GPIO_PORTQ                                       14u
#define  BSP_GPIO_PORTR                                       15u
#define  BSP_GPIO_PORTS                                       16u
#define  BSP_GPIO_PORTT                                       17u


/*
*********************************************************************************************************
*                                             REGISTERS
*********************************************************************************************************
*/

#define  BSP_GPIO_BASE_ADDR                            0x40058000u
#define  BSP_GPIO_REG_DATA(port_nbr)              (*(CPU_REG32 *)(BSP_GPIO_BASE_ADDR + 0x1000 * (port_nbr) + 0x03FCu))
#define  BSP_GPIO_REG_DIR(port_nbr)               (*(CPU_REG32 *)(BSP_GPIO_BASE_ADDR + 0x1000 * (port_nbr) + 0x0400u))
#define  BSP_GPIO_REG_DEN(port_nbr)               (*(CPU_REG32 *)(BSP_GPIO_BASE_ADDR + 0x1000 * (port_nbr) + 0x051Cu))


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void  BSP_Init (void);

#endif
