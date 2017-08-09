#include "data_transfer.h"

uint16 nrf_flag = 0;
volatile uint16 nrf_irq_flag = 0;
int16 Nrf_in_switch[16];

static int16 flag_nrf=0;

void receive_date_check()
{
  if(flag_nrf==1)
  {
/* if flag_nrf == 1 transmmit data from address 1 and reset flag_nrf              */
/* 17: PID read from eeprom first address              */

    eeprom_read(1);                                                             
    Nrf_Buf_Out.Data_int[0]=(int16)(eeprom_readdate[0]);
    Nrf_Buf_Out.Data_int[1]=(int16)(eeprom_readdate[1]);                        
    Nrf_Buf_Out.Data_int[2]=(int16)(eeprom_readdate[2]);
    Nrf_Buf_Out.Data_int[3]=(int16)(eeprom_readdate[3]);
    Nrf_Buf_Out.Data_int[4]=(int16)(eeprom_readdate[4]); 
    Nrf_Buf_Out.Data_int[5]=(int16)(eeprom_readdate[5]);
    Nrf_Buf_Out.Data_int[6]=(int16)(eeprom_readdate[6]);
    Nrf_Buf_Out.Data_int[7]=(int16)(eeprom_readdate[7]); 
    Nrf_Buf_Out.Data_int[8]=(int16)(eeprom_readdate[8]);
    Nrf_Buf_Out.Data_int[11]=17;                                                
    nrf_tx(Nrf_Buf_Out.Data_char,NRF_DATA_PACKET);
    flag_nrf=0;
  }

/* Nrf_Buf_In is the union where nrf_getcmd() save the reveived nrf data              */
/* 0:pitch 1:roll 2:throttle 3:yaw 4:lock(mod)                   */ 
/* if 12: write Nrf_Buf_In[0~8] data to eeprom second position Seems to be PIDs              */
/* if 13: copy Nrf_Buf_In[0~10] data to uint16 Nrf_in_switch[0~10]              */
/* if 14: write Nrf_Buf_In[0~8] data to eeprom first position              */
/* if 15: read eeprom data to Nrf_Buf_Out being transmitted; only place to set flag_nrf to 1              */

  switch(Nrf_Buf_In.Data_int[11])
  {                                              
  case 12:                                                                      
    eeprom_write(2);break;                                                      
  case 13:                                                                               
    Nrf_in_switch[0]=Nrf_Buf_In.Data_int[0];                                    
    Nrf_in_switch[1]=Nrf_Buf_In.Data_int[1];                                    
    Nrf_in_switch[2]=Nrf_Buf_In.Data_int[2]; 
    Nrf_in_switch[3]=Nrf_Buf_In.Data_int[3];
    Nrf_in_switch[4]=Nrf_Buf_In.Data_int[4];
    Nrf_in_switch[5]=Nrf_Buf_In.Data_int[5];
    Nrf_in_switch[6]=Nrf_Buf_In.Data_int[6];
    Nrf_in_switch[7]=Nrf_Buf_In.Data_int[7];
    Nrf_in_switch[8]=Nrf_Buf_In.Data_int[8];
    Nrf_in_switch[9]=Nrf_Buf_In.Data_int[9];
    Nrf_in_switch[10]=Nrf_Buf_In.Data_int[10];
    break;
  case 14:
    eeprom_write(1);break;                                                      
  case 15:
    eeprom_read(2);                                                              
    Nrf_Buf_Out.Data_int[0]=(int16)(eeprom_readdate[0]);
    Nrf_Buf_Out.Data_int[1]=(int16)(eeprom_readdate[1]); 
    Nrf_Buf_Out.Data_int[2]=(int16)(eeprom_readdate[2]);
    Nrf_Buf_Out.Data_int[3]=(int16)(eeprom_readdate[3]);
    Nrf_Buf_Out.Data_int[4]=(int16)(eeprom_readdate[4]); 
    Nrf_Buf_Out.Data_int[5]=(int16)(eeprom_readdate[5]);
    Nrf_Buf_Out.Data_int[6]=(int16)(eeprom_readdate[6]);
    Nrf_Buf_Out.Data_int[7]=(int16)(eeprom_readdate[7]); 
    Nrf_Buf_Out.Data_int[8]=(int16)(eeprom_readdate[8]);
/* PID read from eeprom second address              */
    Nrf_Buf_Out.Data_int[11]=18;
    nrf_tx(Nrf_Buf_Out.Data_char,NRF_DATA_PACKET); 
    flag_nrf=1;
    break;   
  }
}


uint8 nrf_getcmd()
{
  uint8 relen=0;
/* nrf_irq_flag=1 asserted when an nrf int happen              */

  if(nrf_irq_flag>0)
  {
    nrf_irq_flag=0;
    nrf_handler();
  }
  
/* wait a packet and store in buff              */
/* #define NRF_DATA_PACKET NRF_DATA_PACKET_CHAR = 2*12               */

  relen = nrf_rx(Nrf_Buf_In.Data_char,NRF_DATA_PACKET);

/* Seems relen = third input(NRF_DATA_PACKET = 24). if third input is zero, relen = 0. if > DATA_PACKET(32), relen = 32              */

  if(relen != 0)
  {

/* nrf_flag maintain 400 if nrf data received(this varible is used only in IRQ_handler)              */

    nrf_flag=400;                                                               
  }
  else if(nrf_flag>0)

/* decrese nrf_flag if no nrf data received              */

    nrf_flag--;
  
  return relen;
}

void nrf_sendstate()
{

/* final data transmitted to ground terminal using nrf             */

  Nrf_Buf_Out.Data_int[0]=(int16)(attitudeActual.Pitch*100);
  Nrf_Buf_Out.Data_int[1]=(int16)(attitudeActual.Roll*100);
  Nrf_Buf_Out.Data_int[2]=(int16)(attitudeActual.Yaw*100);
//  Nrf_Buf_Out.Data_int[1]=(int16)(Xmm_Send);
//  Nrf_Buf_Out.Data_int[2]=(int16)(Ymm_Send);
  Nrf_Buf_Out.Data_int[3]=(int16)ks103_distance/10;
  
//  Nrf_Buf_Out.Data_int[4]=(int16)(SumX_amend);                       
//  Nrf_Buf_Out.Data_int[5]=(int16)(SumY_amend);
  Nrf_Buf_Out.Data_int[4]=(int16)(RENESAS.FLOW_X);                       
  Nrf_Buf_Out.Data_int[5]=(int16)(RENESAS.FLOW_Y);  		 
//  Nrf_Buf_Out.Data_int[6]=lowthrottle;     
/* 16: indicate a normal data transfer              */

  Nrf_Buf_Out.Data_int[11]=16;
  
/* all data send by nrf_tx from Nrf_Buf_Out               */
  
  nrf_tx(Nrf_Buf_Out.Data_char,NRF_DATA_PACKET);
}

