#ifndef TIMESTAMP_H
#define TIMESTAMP_H
#include "common.h"
struct TimerData
{
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
};
typedef struct TimerData TimerData;

uint32 timestampget(TimerData* time);

void timer_tictok(TimerData* time,uint16 stepus);
#endif // TIMESTAMP_H
