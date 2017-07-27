#ifndef PARAM_COMMON_H
#define PARAM_COMMON_H

#include "sensorsettings.h"
#include "relaytuningsettings.h"
#include "attitudesettings.h"
#include "stabilizationsettings.h"
#include "actuatorsettings.h"
#include "mixersettings.h"
#include "motorsettings.h"


#define CTL_RATE 2500


extern struct SensorSettingsData sensorSettings;
extern struct AttitudeSettingsData attitudeSettings;
extern struct StabilizationSettingsData stabilizationSettings;
extern struct RelayTuningSettingsData relaySettings;
extern struct ActuatorSettingsData actuatorSettings;
extern struct MixerSettingsData mixerSettings;
extern struct MotorSettingsData motorSettings;
//extern struct ManualControlSettingsData manualControlSettings;

void param_common_init();

#endif //PARAM_COMMON_H