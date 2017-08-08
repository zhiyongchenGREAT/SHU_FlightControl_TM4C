/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : param_common.h
* By      : Bicbrv
* Note    : Paras (manual....h, system....h unincluded)
*
* TERMS OF USE:
* ---------------
*           We provide ALL the source code for your convenience and to help you 
*           keep developing our flight control firmware.  
*
*           Please help us continue to provide our project with the finest software available.
*           Your dedicated work is greatly appreciated. Feel free to ameliorate any 
*           part of our code without any restriction to pursue maximum performance.
*
************************************************************************************************************************
*/

#ifndef PARAM_COMMON_H
#define PARAM_COMMON_H
#include "actuatorsettings.h"
#include "attitudesettings.h"
#include "mixersettings.h"
#include "motorsettings.h"
#include "relaytuningsettings.h"
#include "sensorsettings.h"
#include "stabilizationsettings.h"

#define CTL_RATE 2500
//#define CTL_RATE_STABILIZATION 50000

extern struct SensorSettingsData sensorSettings;
extern struct AttitudeSettingsData attitudeSettings;
extern struct StabilizationSettingsData stabilizationSettings;
extern struct RelayTuningSettingsData relaySettings;
extern struct ActuatorSettingsData actuatorSettings;
extern struct MixerSettingsData mixerSettings;
extern struct MotorSettingsData motorSettings;

void param_common_init();

#endif //PARAM_COMMON_H