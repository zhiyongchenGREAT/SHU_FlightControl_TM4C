#ifndef ACTUATORDESIRED_H
#define ACTUATORDESIRED_H
#include <core_common.h>
struct ActuatorDesiredData
{
    float Roll;
    float Pitch;
    float Yaw;
    float Throttle;
    float UpdateTime;
    float NumLongUpdates;
};
typedef struct ActuatorDesiredData ActuatorDesiredData;
#endif // ACTUATORDESIRED_H
