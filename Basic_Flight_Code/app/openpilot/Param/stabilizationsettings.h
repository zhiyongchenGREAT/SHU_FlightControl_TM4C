#ifndef STABILIZATIONSETTINGS_H
#define STABILIZATIONSETTINGS_H
#include <core_common.h>

struct StabilizationSettingsData
{
    uint8 RollMax;
    uint8 PitchMax;
    uint8 YawMax;
    float ManualRate[3];
    float MaximumRate[3];
    uint8 RateExpo[3];
    float PoiMaximumRate[3];
    float RollRatePID[4];
    float PitchRatePID[4];
    float YawRatePID[4];
    float RollPI[3];
    float PitchPI[3];
    float YawPI[3];
    float VbarSensitivity[3];
    float VbarRollPID[3];
    float VbarPitchPID[3];
    float VbarYawPID[3];
    float VbarTau;
    int8 VbarGyroSuppress;
    enum VbarPiroComp
    {
        STABILIZATIONSETTINGS_VBARPIROCOMP_FALSE,
        STABILIZATIONSETTINGS_VBARPIROCOMP_TRUE
    }VbarPiroComp;
    uint8 VbarMaxAngle;
    float GyroTau;
    uint8 DerivativeCutoff;
    float DerivativeGamma;
    uint8 MaxAxisLock;
    uint8 MaxAxisLockRate;
    float WeakLevelingKp;
    uint8 MaxWeakLevelingRate;
    enum LowThrottleZeroIntegral
    {
        STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_FALSE,
        STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE
    }LowThrottleZeroIntegral;
    float CoordinatedFlightYawPI[3];
};
typedef struct StabilizationSettingsData StabilizationSettingsData;
#endif // STABILIZATIONSETTINGS_H
