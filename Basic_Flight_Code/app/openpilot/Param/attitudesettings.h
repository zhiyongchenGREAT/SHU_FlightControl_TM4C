#ifndef ATTITUDESETTINGS_H
#define ATTITUDESETTINGS_H
#include <core_common.h>
struct AttitudeSettingsData
{
    int16 BoardRotation[3];
    float MagKp;
    float MagKi;
    float AccelKp;
    float AccelKi;
    float AccelTau;
    float VertPositionTau;
    float YawBiasRate;
    enum ZeroDuringArming
    {
        ATTITUDESETTINGS_ZERODURINGARMING_FALSE,
        ATTITUDESETTINGS_ZERODURINGARMING_TRUE
    }ZeroDuringArming;
    enum BiasCorrectGyro
    {
        ATTITUDESETTINGS_BIASCORRECTGYRO_FALSE,
        ATTITUDESETTINGS_BIASCORRECTGYRO_TRUE
    }BiasCorrectGyro;
    enum FilterChoice
    {
        ATTITUDESETTINGS_FILTERCHOICE_CCC,
        ATTITUDESETTINGS_FILTERCHOICE_PREMERLANI,
        ATTITUDESETTINGS_FILTERCHOICE_PREMERLANI_GPS
    }FilterChoice;
    enum TrimFlight
    {
        ATTITUDESETTINGS_TRIMFLIGHT_NORMAL,
        ATTITUDESETTINGS_TRIMFLIGHT_START,
        ATTITUDESETTINGS_TRIMFLIGHT_LOAD
    }TrimFlight;
};
typedef struct AttitudeSettingsData AttitudeSettingsData;
#endif // ATTITUDESETTINGS_H
