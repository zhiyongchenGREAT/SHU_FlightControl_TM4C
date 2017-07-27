#ifndef FLIGHTBATTERYSTATE_H
#define FLIGHTBATTERYSTATE_H
#include "common.h"
struct FlightBatteryStateData
{
    float Voltage;
    float Current;
    float BoardSupplyVoltage;
    float PeakCurrent;
    float AvgCurrent;
    float ConsumedEnergy;
    float EstimatedFlightTime;
};
typedef struct FlightBatteryStateData FlightBatteryStateData;
#endif // FLIGHTBATTERYSTATE_H
