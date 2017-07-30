/*
************************************************************************************************************************
*                                                TI-TM4C Flight Control
*                                               SCIE/Shanghai University
*                                              
* File    : data_common.c
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

#include "data_common.h"

struct AttitudeSensorData sensorData;
struct AttitudeActualData attitudeActual;
struct GlobalAttitudeVariables glblAtt;
struct GyrosBiasData gyrosBias;
struct FlightStatusData flightStatus;
struct TimerData timer;
struct RelayTuningData relay;
struct ActuatorDesiredData actuatorDesired;
struct StabilizationDesiredData stabDesired;
struct RateDesiredData rateDesired;
struct TrimAnglesData trimAngles;
struct GyrosData gyrosData;
struct AccelsData accelsData;
struct ActuatorCommandData actuatorCommand;
struct MixerStatusData mixerStatus;
struct AccessoryDesiredData accessoryDesired;
struct ManualControlCommandData manualControlCommand;

union NrfBuff Nrf_Buf_In;
union NrfBuff Nrf_Buf_Out;

union UART_Buff UART_Buff_In;
union UART_Buff UART_Buff_Out;

void data_common_init()
{
  gyrosBias.x=0;
  gyrosBias.y=0;
  gyrosBias.z=0;
  stabDesired.StabilizationMode[0]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
  stabDesired.StabilizationMode[1]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
  stabDesired.StabilizationMode[2]=STABILIZATIONDESIRED_STABILIZATIONMODE_AXISLOCK;
  stabDesired.StabilizationMode[3]=STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDEHOLD;
}