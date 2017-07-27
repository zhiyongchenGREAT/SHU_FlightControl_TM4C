#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
#include "common.h"
#include "data_common.h"
#include "param_common.h"
//void AttitudeInitialize();
void  motorcontrol_init();
void motorspeed_set(uint8 chn,_Bool armed,float rate);
void motor_reset();
#endif // MOTORCONTROL_H
