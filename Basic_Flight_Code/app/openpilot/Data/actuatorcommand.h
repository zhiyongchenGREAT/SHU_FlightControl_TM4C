#ifndef ACTUATORCOMMAND_H
#define ACTUATORCOMMAND_H
#include <core_common.h>
#define ACTUATORCOMMAND_CHANNEL_NUMELEM 10
struct ActuatorCommandData
{
    int16 Channel[ACTUATORCOMMAND_CHANNEL_NUMELEM];
    uint8 UpdateTime;
    uint16 MaxUpdateTime;
    uint8 NumFailedUpdates;
};
typedef struct ActuatorCommandData ActuatorCommandData;
#endif // ACTUATORCOMMAND_H
