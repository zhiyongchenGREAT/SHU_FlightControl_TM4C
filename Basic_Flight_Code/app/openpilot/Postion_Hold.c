#include "Postion_Hold.h"

#include "core_uart.h"

#define pos_x_i_max 50000
#define pos_mov_i_max 800
#define angel_out_max 6.5f

float tot_x_cm,tot_y_cm;
float last_tot_x_cm,last_tot_y_cm;	
float angle_x_out,angle_y_out;
float pos_x_i=0,pos_y_i=0;
float PID_POS_XOUT,PID_POS_YOUT;
int Optical_Flow_Data_Update;
float flow_p,flow_i,flow_d;
float delta_sumx,delta_sumy;

void Control()
{
  tot_x_cm=SumX_amend;                                                          //SumX/Y as px4_data_fix() output
  tot_y_cm=SumY_amend;
  pos_x_i += SumX_amend;
  pos_y_i += SumY_amend;
  
  if(pos_x_i > pos_x_i_max)                                                     //pos_x_i_max 50000 is defined
    pos_x_i = pos_x_i_max;
  if(pos_x_i < -pos_x_i_max)
    pos_x_i = -pos_x_i_max;
  if(pos_y_i > pos_x_i_max)
    pos_y_i = pos_x_i_max;
  if(pos_y_i < -pos_x_i_max)
    pos_y_i = -pos_x_i_max;
  
  
  
  delta_sumx=tot_x_cm - last_tot_x_cm;
  delta_sumy=tot_y_cm - last_tot_y_cm;
  
  //PID_POS_XOUT  = 0.045*( tot_x_cm + 0.00714 * pos_x_i + 35* (tot_x_cm - last_tot_x_cm));
  //PID_POS_YOUT  = (eeprom_readdate[6]/1000.0) * (tot_y_cm + (eeprom_readdate[7]/100000.0) * pos_y_i + (eeprom_readdate[8]/100.0) * (tot_y_cm - last_tot_y_cm));
  
  //PID_POS_YOUT  = 0.045*( tot_y_cm + 0.0035 * pos_y_i + 37.6* (tot_y_cm - last_tot_y_cm));
  
//  PID_POS_XOUT  = 0.045*( tot_x_cm + 0.0035 * pos_x_i + 37.6 * (tot_x_cm - last_tot_x_cm));
//
//  PID_POS_YOUT  = 0.045*( tot_y_cm + 0.0035 * pos_y_i + 37.6 * (tot_y_cm - last_tot_y_cm));

////////////////////////////////////////////////////////////////////////////////
  PID_POS_XOUT  = PID.paraXA * ( tot_x_cm + PID.paraXB * pos_x_i + PID.paraXC * (tot_x_cm - last_tot_x_cm));

  PID_POS_YOUT  = PID.paraYA * ( tot_y_cm + PID.paraYB * pos_y_i + PID.paraYC * (tot_y_cm - last_tot_y_cm));
////////////////////////////////////////////////////////////////////////////////  

  //PID_POS_YOUT  = (eeprom_readdate[6]/1000.0) * (tot_y_cm + (eeprom_readdate[7]/100000.0) * pos_y_i + (eeprom_readdate[8]/100.0) * (tot_y_cm - last_tot_y_cm));
  
  
  if(ks103_distance<=200)
  {
    tot_y_cm=0;
    pos_y_i=0;
    tot_x_cm=0;
    pos_x_i=0;
  }
  
  angle_x_out = PID_POS_XOUT ;	
  angle_y_out = PID_POS_YOUT ;
  
  
  
  if(angle_x_out > angel_out_max)
    angle_x_out = angel_out_max;
  if(angle_x_out < -angel_out_max)
    angle_x_out = -angel_out_max;
  if(angle_y_out > angel_out_max)
    angle_y_out = angel_out_max;
  if(angle_y_out < -angel_out_max)
    angle_y_out = -angel_out_max;
  last_tot_x_cm = tot_x_cm;
  last_tot_y_cm = tot_y_cm;
}