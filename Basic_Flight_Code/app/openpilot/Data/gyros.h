#ifndef GYROS_H
#define GYROS_H
#include "common.h"
struct GyrosData
{
    float x;
    float y;
    float z;
    float temperature;
};
typedef struct GyrosData GyrosData;
#endif // GYROS_H
