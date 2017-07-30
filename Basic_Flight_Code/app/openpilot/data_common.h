/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : data_common.h
* By      : Bicbrv
* Note    : 
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

#ifndef DATA_COMMON_H
#define DATA_COMMON_H

#include "actuatordesired.h"
#include "accels.h"
#include "accessorydesired.h"
#include "actuatorcommand.h"
#include "attitudeactual.h"
#include "attitudesensor.h"
#include "flightbatterystate.h"
#include "flightstatus.h"
#include "gyros.h"
#include "gyrosbias.h"
#include "manualcontrolcommand.h"
#include "mixerstatus.h"
#include "nrf_data.h"
#include "ratedesired.h"
#include "relaytuning.h"
#include "stabilizationdesired.h"
#include "state_struct.h"
#include "timestamp.h"
#include "trimangles.h"
#include "uart_data.h"

extern struct AttitudeSensorData sensorData;
extern struct AttitudeActualData attitudeActual;
extern struct GlobalAttitudeVariables glblAtt;
extern struct GyrosBiasData gyrosBias;
extern struct FlightStatusData flightStatus;
extern struct TimerData timer;
extern struct RelayTuningData relay;
extern struct ActuatorDesiredData actuatorDesired;
extern struct StabilizationDesiredData stabDesired;
extern struct RateDesiredData rateDesired;
extern struct TrimAnglesData trimAngles;
extern struct GyrosData gyrosData;
extern struct AccelsData accelsData;
extern struct ActuatorCommandData actuatorCommand;
extern struct MixerStatusData mixerStatus;
extern struct AccessoryDesiredData accessoryDesired;
extern struct ManualControlCommandData manualControlCommand;

extern union NrfBuff Nrf_Buf_In;
extern union NrfBuff Nrf_Buf_Out;

extern union UART_Buff UART_Buff_In;//pc->mcu
extern union UART_Buff UART_Buff_Out;//muc->pc

void data_common_init();

#endif // DATA_COMMON_H
