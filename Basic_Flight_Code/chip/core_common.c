/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_common.c
* By      : Bicbrv
* Note    : APP level declr & hard delay
*
* TERMS OF USE:
* ---------------
*           We provide ALL the source code for your convenience and to help you 
*           keep developing our flight control firmware.  
*
*           Please help us continue to provide our project with the finest software available.
*           Your dedicated work is greatly appreciated. Feel free to ameliorate any 
*           part of our code without any restriction to pursue maximum performance.
*
************************************************************************************************************************
*/

#include <core_common.h>

#pragma optimize=none 

void SysCtlDelay_MS(vuint32 MS)
{
  while(MS>0)
  {
    SysCtlDelay(16000);
    MS--;
  }
}