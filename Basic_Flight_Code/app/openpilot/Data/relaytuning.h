#ifndef RELAYTUNING_H
#define RELAYTUNING_H
#include <core_common.h>
struct RelayTuningData
{
    float Period[3];
    float Gain[3];
};
typedef struct RelayTuningData RelayTuningData;
#endif // RELAYTUNING_H
