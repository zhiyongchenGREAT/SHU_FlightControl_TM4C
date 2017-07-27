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