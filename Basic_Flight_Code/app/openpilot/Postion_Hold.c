#include "Postion_Hold.h"

#define pos_x_i_max 50000
#define angel_out_max 6.5f

float angle_x_out,angle_y_out;
float delta_sumx,delta_sumy;

static float tot_x_cm,tot_y_cm;
static float last_tot_x_cm,last_tot_y_cm;	
static float pos_x_i=0,pos_y_i=0;
static float PID_POS_XOUT,PID_POS_YOUT;


void Control()
{
  
  tot_x_cm = SumX_amend;
  tot_y_cm = SumY_amend;
  pos_x_i += SumX_amend;
  pos_y_i += SumY_amend;
  
  if(pos_x_i > pos_x_i_max)
    pos_x_i = pos_x_i_max;
  if(pos_x_i < -pos_x_i_max)
    pos_x_i = -pos_x_i_max;
  if(pos_y_i > pos_x_i_max)
    pos_y_i = pos_x_i_max;
  if(pos_y_i < -pos_x_i_max)
    pos_y_i = -pos_x_i_max;
  
  delta_sumx=tot_x_cm - last_tot_x_cm;
  delta_sumy=tot_y_cm - last_tot_y_cm;
  
/* PID_POS_XOUT  = 0.045*( tot_x_cm + 0.00714 * pos_x_i + 35* (tot_x_cm - last_tot_x_cm));              */
/* PID_POS_YOUT  = 0.045*( tot_y_cm + 0.0035 * pos_y_i + 37.6* (tot_y_cm - last_tot_y_cm));              */

  PID_POS_XOUT  = UART_PIDadjust.FLOW_XP 
    * ( tot_x_cm + UART_PIDadjust.FLOW_XI * pos_x_i + UART_PIDadjust.FLOW_XD * (tot_x_cm - last_tot_x_cm));
  PID_POS_YOUT  = UART_PIDadjust.FLOW_YP 
    * ( tot_y_cm + UART_PIDadjust.FLOW_YI * pos_y_i + UART_PIDadjust.FLOW_YD * (tot_y_cm - last_tot_y_cm));
  
  if(ks103_distance<=220)
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