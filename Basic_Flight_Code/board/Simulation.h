#ifndef __SIMULATION_H
#define __SIMULATION_H

#include "common.h"

#define ADNS3080_READ_REG        0x00  	  // ?¨¢??¡ä??¡Â??¨¢?
#define ADNS3080_WRITE_REG       0x80 	    // D¡ä??¡ä??¡Â??¨¢?

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60


// field of view of ADNS3080 sensor lenses
#define AP_OPTICALFLOW_ADNS3080_08_FOV 0.202458f        // 11.6 degrees¡ê??a??¨º?1a¨¢¡Â¡ä??D?¡Â¦Ì¡À?¡ã?1?¨¤??¦Ì?¨º¨®¨°¡ã???¨¨?¦Ì¡ê¡§???¨¨¡À¨ª¨º?¡ê?¡ê?¡ä¨®??¨º?11.6?¨¨

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define AP_OPTICALFLOW_ADNS3080_SCALER_400   1.1f       // when resolution set to 400
#define AP_OPTICALFLOW_ADNS3080_SCALER_1600  4.4f       // when resolution set to 1600

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X               30
#define ADNS3080_PIXELS_Y               30


/*NCS*/
#define SET_SPI_NCS  GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0xff)
#define CLR_SPI_NCS  GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0x00)
/*MISO*/
#define READ_MISO_DATA	GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5)
/*MOSI*/
#define SET_SPI_MOSI   GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,0xff)
#define CLR_SPI_MOSI  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,0x00)
/*SCLK*/
#define SET_SPI_SCL  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0xff)
#define CLR_SPI_SCL   GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0x00)


extern float conv_factor;
extern float radians_to_pixels_x,radians_to_pixels_y;

void Sim_SPI_GPIO_Init(uint8 CPOL,uint8 CPHA,uint32 Speed);
uint8 SPI_Basic_RW(uint8 data_write);
uint8 SPI_Read(uint8 reg);
void motion_read(uint8 * data_buffer);
void SPI_Write(uint8 reg,uint8 value);
void ADNS3080_Init(void);
//void SPI_Delay();
void SPI_Delay_us(uint16 us);
void SPI_Delay_ms(uint16 ms);
void delay(int32 count_delay);
void data_fix(void);

extern int8_t  x;
extern int8_t  y;
extern float SumX;
extern float SumY;
extern float High_Now;



#endif
