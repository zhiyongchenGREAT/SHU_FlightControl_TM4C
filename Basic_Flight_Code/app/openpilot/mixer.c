#include "mixer.h"
#include "include.h"
#include "motor_control.h"
extern float cos_lookup_deg(float angle);
void mixing(_Bool armed)
{
  float m1=0,m2=0,m3=0,m4=0;
  float t,r,p,y;
  t=actuatorDesired.Throttle/(100.0*0.9);//*cos_lookup_deg(fabs(attitudeActual.Pitch))*cos_lookup_deg(fabs(attitudeActual.Roll))  );
  r=actuatorDesired.Roll;
  p=actuatorDesired.Pitch;                                                      //::Note:: NO data transfered to p & y?
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