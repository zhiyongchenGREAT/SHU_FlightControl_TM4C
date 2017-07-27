/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,野火科技
*     All rights reserved.
*     技术讨论：野火初学论坛 http://www.chuxue123.com
*
*     除注明出处外，以下所有内容版权均属野火科技所有，未经允许，不得用于商业用途，
*     修改内容时必须保留野火科技的版权声明。
*
* @file       common.h
* @brief      野火K60 平台常用类型声明和宏定义
* @author     野火科技
* @version    v5.0
* @date       2013-06-26
*/

#ifndef _COMMON_H_
#define _COMMON_H_
//#include "chiplevel_includes.h"
#include <stdint.h>
#include <stdbool.h>   


#define true 1
#define false 0



/*
* 数据类型声明
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

/*
* 定义带位域的联合体类型
*/
typedef union
{
  uint32  DW;
  uint16  W[2];
  uint8   B[4];
  struct
  {
    uint32 b0: 1;
    uint32 b1: 1;
    uint32 b2: 1;
    uint32 b3: 1;
    uint32 b4: 1;
    uint32 b5: 1;
    uint32 b6: 1;
    uint32 b7: 1;
    uint32 b8: 1;
    uint32 b9: 1;
    uint32 b10: 1;
    uint32 b11: 1;
    uint32 b12: 1;
    uint32 b13: 1;
    uint32 b14: 1;
    uint32 b15: 1;
    uint32 b16: 1;
    uint32 b17: 1;
    uint32 b18: 1;
    uint32 b19: 1;
    uint32 b20: 1;
    uint32 b21: 1;
    uint32 b22: 1;
    uint32 b23: 1;
    uint32 b24: 1;
    uint32 b25: 1;
    uint32 b26: 1;
    uint32 b27: 1;
    uint32 b28: 1;
    uint32 b29: 1;
    uint32 b30: 1;
    uint32 b31: 1;
  };
} Dtype;    //sizeof(Dtype) 为 4

/*
* 定义坐标结构体
*/
typedef struct
{
  uint16 x;
  uint16 y;
} Site_t;

/*
* 定义矩形大小结构体
*/
typedef struct
{
  uint16 W;       //宽
  uint16 H;       //高
} Size_t;

#define TRUE    1
#define FALSE   0


/*
* 定义运行到RAM的函数
*/
#if defined(__ICCARM__)
//IAR 用 __ramfunc 来声明
#define     __RAMFUNC __ramfunc
#else
#define     __RAMFUN
#endif


/*
* 包含Cortex-M内核的通用头文件
*/
#include    <stdio.h>                       //printf
#include    <string.h>                      //memcpy
#include    <stdlib.h>                      //malloc
//#include "misc.h"

//#define   __NVIC_PRIO_BITS    2       /* interrupt priority shift*/
//#include "arm_math.h"





/*
* 包含常用头文件
*/
//#include    "system_MKL.h"                  //系统配置
//#include    "FIRE_PORT_cfg.h"               //管脚复用配置
//#include    "MKL_mcg.h"                    //K60 时钟模块

//#if (defined(IAR))
//#include "intrinsics.h"
//#endif




/**
*  @brief 变量的位清0和置1
*/
#define BIT_CLEAN(var,n)        (var) &= ~(1<<(n))   //变量var 的n位（即第n+1位）清0
#define BIT_SET(var,n)          (var) |=  (1<<(n))   //变量var 的n位（即第n+1位）置1
#define BIT_GET(var,n)          (((var)>>(n))&0x01)  //读取变量var 的n位（即第n+1位）

/**
*  @brief 仿二进制赋值
*/
#define  HEX__(n)   0x##n##UL
#define  B8__(x)   ( (x & 0x0000000FUL) ? 1:0 )\
+( (x & 0x000000F0UL) ? 2:0 )\
  +( (x & 0x00000F00UL) ? 4:0 )\
    +( (x & 0x0000F000UL) ? 8:0 )\
      +( (x & 0x000F0000UL) ? 16:0 )\
        +( (x & 0x00F00000UL) ? 32:0 )\
          +( (x & 0x0F000000UL) ? 64:0 )\
            +( (x & 0xF0000000UL) ? 128:0 )
#define  B8(x)                                     ((unsigned char)B8__(HEX__(x)))
#define  B16(x_msb,x_lsb)                (((unsigned int)B8(x_msb)<<8) + B8(x_lsb))
#define  B32(x_msb,x_2,x_3,x_lsb)   (((unsigned long)B8(x_msb)<<24) + ((unsigned long)B8(x_2)<<16) + ((unsigned long)B8(x_3)<<8) + B8(x_lsb))
              /* Sample usage:
              B8(01010101) = 85
              B16(10101010,01010101) = 43605
              B32(10000000,11111111,10101010,01010101) = 2164238933
              */
              
              
              /*
              * 求最大值和最小值
              */
#define MAX( x, y ) ( ((x) > (y)) ? (x) : (y) )
#define MIN( x, y ) ( ((x) < (y)) ? (x) : (y) )
              
              
              /*
              * 返回数组元素的个数
              */
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( ((a)[0]) ) )
              
              /*
              * 宏定义实现返回绝对值（x里不能有自加自减的语句，否则变量出错）
              */
#define ABS(x) (((x) > 0) ? (x) : (-x))
              
              /*
              * 获取结构体某成员偏移
              */
#define OFFSET(type, member)    (uint32)(&(((type *)0)->member))
              
              /*
              * 确保x的范围为 min~max
              */
#define RANGE(x,max,min)        ((uint8)((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))
              
              /*
              * 交换32位数据的4字节顺序
              */
#define SWAP32(data)    (uint32)((((uint32)(data) & (0xFFu<<0 ))<<24)      \
              | (((uint32)(data) & (0xFFu<<8 ))<<8)            \
                | (((uint32)(data) & (0xFFu<<16))>>8)            \
                  | (((uint32)(data) & (0xFFu<<24))>>24)           \
                    )

/*
* 交换16位数据的2字节顺序
*/
#define SWAP16(data)    (uint32)((((uint16)(data) & (0xFF<<0 ))<<8)      \
| (((uint32)(data) & (0xFF<<8))>>8)            \
  )

/*
* 交换 x, y 的值
*/
#define SWAP(x,y)           do{x^=y;y^=x;x^=y;}while(0)


/********************************************************************/

#endif /* _COMMON_H_ */
