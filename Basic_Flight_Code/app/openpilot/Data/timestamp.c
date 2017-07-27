#include "timestamp.h"
/*  struct TimeData
    uint16 us;
    uint16 ms;
    uint8 s;
    uint8 m;
    uint8 h;  
    uint32 stamp0;
    uint32 stamp1;
    uint32 stamp2;
    uint32 stamp3;
    uint32 stamp4;
*/
uint32 timestampget(TimerData* timer)                                           //decode TimeData struct varible to unit ms
{
  return ((timer->h*60+timer->m)*60+timer->s)*1000+timer->ms;
}

void timer_tictok(TimerData* timer,uint16 stepus)                               //encode TimeData struct varible, stepus's unit is ms 
{
  timer->ms+=stepus/1000;
  if(timer->ms>=1000)
  {
    timer->ms-=1000;
    timer->s+=1;
    if(timer->s>=60)
    {
      timer->s-=60;
      timer->m+=1;
      if(timer->m>=60)
      {
       timer->m-=60;
       timer->h+=1;
      }
    }
  }
}
