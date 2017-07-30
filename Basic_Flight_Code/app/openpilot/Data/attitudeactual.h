#ifndef ATTITUDEACTUAL_H
#define ATTITUDEACTUAL_H
#include <core_common.h>
struct AttitudeActualData                                                       //::note::What's the definition of the member of this struct? What's the usage of the struct?
{
    float q1;
    float q2;
    float q3;
    float q4;
    float Roll;
    float Pitch;
    float Yaw;
};
typedef struct AttitudeActualData AttitudeActualData;
#endif // ATTITUDEACTUAL_H
