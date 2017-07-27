#include <stdint.h>
#include <eepromqsj.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/eeprom.h"
int16 eeprom_readdate[9];
void eeprom_read(int select)                                                    //::note::eeprom_read(1) read from the address 0 of the eeprom; eeprom_read(2) read from address sizeof(pui32Read)+1
{                                                                               //seems only two eeprom address can be visited in this source file
  uint32_t pui32Read[9];
  

  //EEPROMMassErase();
  /*uint32_t pui32Data_init[3];
  pui32Data_init[0]=2000;
  pui32Data_init[1]=17000;
  pui32Data_init[2]=0;
  EEPROMProgram(pui32Data_init, 0x0, sizeof(pui32Data_init));*/
  if(select==1)
  EEPROMRead(pui32Read, 0x0, sizeof(pui32Read));
  else if(select==2)
  EEPROMRead(pui32Read, sizeof(pui32Read)+1, sizeof(pui32Read));
  
  eeprom_readdate[0]=pui32Read[0];
  eeprom_readdate[1]=pui32Read[1];
  eeprom_readdate[2]=pui32Read[2];
  eeprom_readdate[3]=pui32Read[3];
  eeprom_readdate[4]=pui32Read[4];
  eeprom_readdate[5]=pui32Read[5];
  eeprom_readdate[6]=pui32Read[6];
  eeprom_readdate[7]=pui32Read[7];
  eeprom_readdate[8]=pui32Read[8];
}
void eeprom_write(int select)                                                   //::note:: similar to the situation of eeprom_read, only two address can be visit
{ 
  uint32_t pui32Data[9];
  pui32Data[0] = Nrf_Buf_In.Data_int[0];
  pui32Data[1] = Nrf_Buf_In.Data_int[1];
  pui32Data[2] = Nrf_Buf_In.Data_int[2];
  pui32Data[3] = Nrf_Buf_In.Data_int[3];
  pui32Data[4] = Nrf_Buf_In.Data_int[4];
  pui32Data[5] = Nrf_Buf_In.Data_int[5];
  pui32Data[6] = Nrf_Buf_In.Data_int[6];
  pui32Data[7] = Nrf_Buf_In.Data_int[7];
  pui32Data[8] = Nrf_Buf_In.Data_int[8];
 // SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
 // EEPROMInit();
  if(select==1)
  EEPROMProgram(pui32Data, 0x0, sizeof(pui32Data));
  else if(select==2)
  EEPROMProgram(pui32Data, sizeof(pui32Data)+1, sizeof(pui32Data));
  
}  