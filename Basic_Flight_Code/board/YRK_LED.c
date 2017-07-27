#include "common.h"
//#include "MKL_gpio.h"
//#include "MKL_i2c.h"
#include "YRK_LED.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
void led_init()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_1);  
}
