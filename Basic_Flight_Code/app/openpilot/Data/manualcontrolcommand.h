#ifndef MANUALCONTROLCOMMAND_H
#define MANUALCONTROLCOMMAND_H
#include "common.h"
struct ManualControlCommandData
{
    enum Connected
    {
        MANUALCONTROLCOMMAND_CONNECTED_FALSE,
        MANUALCONTROLCOMMAND_CONNECTED_TRUE
    }Connected;
    float Throttle;
    float Roll;
    float Pitch;
    float Yaw;
    float Mode;
    int16 Rssi;
    uint32 RawRssi;
    float Collective;
    float Channel;
};
typedef struct ManualControlCommandData ManualControlCommandData;
#endif // MANUALCONTROLCOMMAND_H
