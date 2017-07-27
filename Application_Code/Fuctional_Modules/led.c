#include "led.h"

void led_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutputOD(GPIO_PORTA_BASE, GPIO_PIN_1);  
}
