#ifndef DATA_COMMON_H
#define DATA_COMMON_H

#include "accels.h"
#include "attitudeactual.h"
#include "actuatordesired.h"
#include "ratedesired.h"
#include "flightstatus.h"
#include "gyros.h"
#include "gyrosbias.h"
#include "attitudesensor.h"
#include "state_struct.h"
#include "stabilizationdesired.h"
#include "timestamp.h"
#include "relaytuning.h"
#include "trimangles.h"
#include "actuatorcommand.h"
#include "mixerstatus.h"
#include "accessorydesired.h"
#include "manualcontrolcommand.h"
#include "nrf_data.h"
#include "uart_data.h"
#include <math.h>



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
