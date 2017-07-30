/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : core_common.h
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

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <board_includes.h>

/*
************************************************************************************************************************
*                                               application level data type decleration
************************************************************************************************************************
*/

typedef unsigned char       uint8;  /*  8 bits */
typedef unsigned short int  uint16; /* 16 bits */
typedef unsigned long int   uint32; /* 32 bits */
typedef unsigned long long  uint64; /* 64 bits */

typedef char                int8;   /*  8 bits */
typedef short int           int16;  /* 16 bits */
typedef long  int           int32;  /* 32 bits */
typedef long  long          int64;  /* 64 bits */

typedef volatile int8       vint8;  /*  8 bits */
typedef volatile int16      vint16; /* 16 bits */
typedef volatile int32      vint32; /* 32 bits */
typedef volatile int64      vint64; /* 64 bits */

typedef volatile uint8      vuint8;  /*  8 bits */
typedef volatile uint16     vuint16; /* 16 bits */
typedef volatile uint32     vuint32; /* 32 bits */
typedef volatile uint64     vuint64; /* 64 bits */


#define DELAY_SYS(LOOP) SysCtlDelay(LOOP)   
#define DELAY_US(US)  	SysCtlDelay(US*16)
#define DELAY_MS(MS)  	SysCtlDelay_MS(MS)
extern void SysCtlDelay_MS(vuint32 MS); 

#endif /* _COMMON_H_ */
