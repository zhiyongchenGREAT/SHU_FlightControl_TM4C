#ifndef SENSORSETTINGS_H
#define SENSORSETTINGS_H
#include <core_common.h>
#define SENSORSETTINGS_GYROSCALE_X 0
#define SENSORSETTINGS_GYROSCALE_Y 1
#define SENSORSETTINGS_GYROSCALE_Z 2

struct SensorSettingsData
{
    float AccelBias[3];
    float AccelScale[3];
    float GyroScale[3];
    float XGyroTempCoeff[4];
    float YGyroTempCoeff[4];
    float ZGyroTempCoeff[4];
    float MagBias[3];
    float MagScale[3];
    float ZAccelOffset;
};
typedef struct SensorSettingsData SensorSettingsData;
#endif // SENSORSETTINGS_H
