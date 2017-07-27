/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : includes.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  INCLUDES_MODULES_PRESENT
#define  INCLUDES_MODULES_PRESENT


/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/


#include  <stdio.h>
#include  <stdint.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>


/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/

#include  <os.h>
#include  <os_type.h>

/*
*********************************************************************************************************
*                                              LIBRARIES / CPU
*********************************************************************************************************
*/
#include  <lib_def.h>
#include  <lib_ascii.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <lib_str.h>

#include  <cpu.h>
#include  <cpu_core.h>
#include  <cpu_cache.h>
#include  <cpu_def.h>
/*
*********************************************************************************************************
*                                              CONFIG / BSP
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <bsp_cfg.h>
#include  <cpu_cfg.h>
#include  <lib_cfg.h>
#include  <os_app_hooks.h>
#include  <os_cfg.h>
#include  <os_cfg_app.h>

#include  <bsp_int.h>
#include  <bsp_sys.h>
#include  <bsp.h>

#include  <os_cpu.h>

#endif
