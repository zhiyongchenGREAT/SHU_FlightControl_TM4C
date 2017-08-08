#include "mixer.h"


void mixing(_Bool armed)
{
  float m1=0,m2=0,m3=0,m4=0;
  float t,r,p,y;

/*  testing              */
  
//  actuatorDesired.Roll = bound_sym(actuatorDesired.Roll,0.02f);
//  actuatorDesired.Pitch = bound_sym(actuatorDesired.Pitch,0.02f);
//  actuatorDesired.Yaw = bound_sym(actuatorDesired.Yaw,0.1f);
  
  t=actuatorDesired.Throttle/(100.0*0.9);
  r=actuatorDesired.Roll;
  p=actuatorDesired.Pitch;                                                      
  y=actuatorDesired.Yaw;
  m1=t+r-p+y;
  m2=t-r-p-y;
  m3=t-r+p+y;
  m4=t+r+p-y;
  motorspeed_set(0,armed,m1);
  motorspeed_set(1,armed,m2);
  motorspeed_set(2,armed,m3);
  motorspeed_set(3,armed,m4);
  
}