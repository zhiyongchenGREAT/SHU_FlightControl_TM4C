#ifndef GYROS_H
#define GYROS_H
#include <core_common.h>
struct GyrosData
{
    float x;
    float y;
    float z;
    float temperature;
};
typedef struct GyrosData GyrosData;
#endif // GYROS_H
