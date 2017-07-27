#ifndef ATTITUDE_SENSOR_H
#define ATTITUDE_SENSOR_H
#include "common.h"
struct AttitudeSensorData
{
    float G_X;
    float G_Y;
    float G_Z;
    float A_X;
    float A_Y;
    float A_Z;
    float M_X;
    float M_Y;
    float M_Z;
    float T;
};
typedef struct AttitudeSensorData AttitudeSensorData;
#endif // ATTITUDE_SENSOR_H
