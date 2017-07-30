#ifndef RELAYTUNINGSETTINGS_H
#define RELAYTUNINGSETTINGS_H
#include <core_common.h>
struct RelayTuningSettingsData
{
    float RateGain;
    float AttitudeGain;
    float Amplitude;
    uint8 HysteresisThresh;
    enum Mode
    {
        RELAYTUNINGSETTINGS_MODE_RATE,
        RELAYTUNINGSETTINGS_MODE_ATTITUDE
    }Mode;
    enum Behavior
    {
        RELAYTUNINGSETTINGS_BEHAVIOR_MEASURE,
        RELAYTUNINGSETTINGS_BEHAVIOR_COMPUTE,
        RELAYTUNINGSETTINGS_BEHAVIOR_SAVE
    }Behavior;
};
typedef struct RelayTuningSettingsData RelayTuningSettingsData;
#endif // RELAYTUNINGSETTINGS_H
