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
* Filename      : bsp_sys.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDES
*********************************************************************************************************
*/

#include  <bsp_cfg.h>
#include  <lib_def.h>
#include  <bsp_sys.h>
////SAR Addition
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/sysctl.h"
////bicbrv Addition
#include <board_includes.h>

/*$PAGE*/
/*
*********************************************************************************************************
*                                         BSP SYSTEM INITIALIZATION
*
* Description: This function should be called early in the BSP initialization process.
*
* Argument(s): none.
*
* Return(s)  : none.
*
* Caller(s)  : Application.
*
* Note(s)    : 1) Ensure the main oscillator is enable because this is required by the PHY. The system
*                 must have a 25MHz crystal attached to the OSC pins. The SYSCTL_MOS_HIGHFREQ parameter
*                 is used when the crystal frequency is 10MHz or higher.
*
*              2) Depending on the CPU frequency, the application must program the Main Flash and EEPROM
*                 memory timing paremeters according to the following table:
*
*               +-----------------------+--------------------------+-------------+-----------+---------+
*               | CPU Freq. Range(F) in | Time Period Range (t) in | FBCHT/EBCHT | FBCE/EBCE | FWS/EWS |
*               |         MHz           |          ns              |             |           |         |
*               +-----------------------+--------------------------+-------------+-----------+---------+
*               |         16            |          62.5            |     0x0     |     1     |  0x0    |
*               |    16 < f <=  40      |    62.5  > f >= 25       |     0x2     |     0     |  0x1    |
*               |    40 < f <=  60      |    25    > f >= 16.67    |     0x3     |     0     |  0x2    |
*               |    60 < f <=  80      |    16.67 > f >= 12.5     |     0x4     |     0     |  0x3    |
*               |    80 < f <= 100      |    12.5  > f >= 10       |     0x5     |     0     |  0x4    |
*               |   100 < f <= 120      |    10    > f >=  8.33    |     0x6     |     0     |  0x5    |
*               +-----------------------+--------------------------+-------------+-----------+---------+
*********************************************************************************************************
*/

void  BSP_SysInit (void)
{
//  ROM_FPUEnable();
//  ROM_FPULazyStackingEnable();
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);		
}


/*
*********************************************************************************************************
*                                         SYSTEM CLOCK FREQUENCY
*
* Description: This function is used to retrieve system or CPU clock frequency.
*
* Arguments  : None
*
* Return     : System clock frequency in cycles.
*
* Caller(s)  : Application.
*
* Note(s)    : None
*********************************************************************************************************
*/

CPU_INT32U  BSP_SysClkFreqGet (void)
{
  return ROM_SysCtlClockGet(); 
}
