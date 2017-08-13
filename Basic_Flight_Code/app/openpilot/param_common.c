#include "param_common.h"


struct SensorSettingsData sensorSettings;
struct AttitudeSettingsData attitudeSettings;
struct StabilizationSettingsData stabilizationSettings;
struct RelayTuningSettingsData relaySettings;
struct ActuatorSettingsData actuatorSettings;
struct MixerSettingsData mixerSettings;
struct MotorSettingsData motorSettings;
//struct ManualControlSettingsData manualControlSettings;

void param_common_init()
{
  sensorSettings.AccelScale[0]=1;
  sensorSettings.AccelScale[1]=1;
  sensorSettings.AccelScale[2]=1;
  sensorSettings.GyroScale[0]=1000;
  sensorSettings.GyroScale[1]=1000;
  sensorSettings.GyroScale[2]=1000;
  motorSettings.Motor[0][MOTORSETTINGS_FREQ]=400;
  motorSettings.Motor[1][MOTORSETTINGS_FREQ]=400;
  motorSettings.Motor[2][MOTORSETTINGS_FREQ]=400;
  motorSettings.Motor[3][MOTORSETTINGS_FREQ]=400;
  
  motorSettings.Motor[0][MOTORSETTINGS_EDP_L]=1000;
  motorSettings.Motor[1][MOTORSETTINGS_EDP_L]=1000;
  motorSettings.Motor[2][MOTORSETTINGS_EDP_L]=1000;
  motorSettings.Motor[3][MOTORSETTINGS_EDP_L]=1000;
  
  motorSettings.Motor[0][MOTORSETTINGS_EDP_H]=1900;
  motorSettings.Motor[1][MOTORSETTINGS_EDP_H]=1900;
  motorSettings.Motor[2][MOTORSETTINGS_EDP_H]=1900;
  motorSettings.Motor[3][MOTORSETTINGS_EDP_H]=1900;
  
  motorSettings.Motor[0][MOTORSETTINGS_STP]=1050;
  motorSettings.Motor[1][MOTORSETTINGS_STP]=1050;
  motorSettings.Motor[2][MOTORSETTINGS_STP]=1050;
  motorSettings.Motor[3][MOTORSETTINGS_STP]=1050;
  
  attitudeSettings.AccelKp=0.05;
  attitudeSettings.AccelKi=0.0001f;
  attitudeSettings.AccelTau=0.1;
  attitudeSettings.VertPositionTau= 2;
  attitudeSettings.YawBiasRate=0.000001;
  attitudeSettings.ZeroDuringArming=ATTITUDESETTINGS_ZERODURINGARMING_TRUE;
  attitudeSettings.BiasCorrectGyro=ATTITUDESETTINGS_BIASCORRECTGYRO_TRUE;
  attitudeSettings.FilterChoice=ATTITUDESETTINGS_FILTERCHOICE_CCC;
  attitudeSettings.TrimFlight=ATTITUDESETTINGS_TRIMFLIGHT_NORMAL;
  
/* inner loop pid              */

  stabilizationSettings.RollRatePID[0]=0.0017;
  stabilizationSettings.RollRatePID[1]=0.0;
  stabilizationSettings.RollRatePID[2]=0.000072;
  stabilizationSettings.RollRatePID[3]=0.3;
  stabilizationSettings.PitchRatePID[0]=0.0017;
  stabilizationSettings.PitchRatePID[1]=0.0;
  stabilizationSettings.PitchRatePID[2]=0.000072;
  stabilizationSettings.PitchRatePID[3]=0.3;
  stabilizationSettings.YawRatePID[0]=0.0035;
  stabilizationSettings.YawRatePID[1]=0.0035;
  stabilizationSettings.YawRatePID[2]=0;
  stabilizationSettings.YawRatePID[3]=0.3;

/* outer loop pid              */
  
  stabilizationSettings.RollPI[0]=2.8;
  stabilizationSettings.RollPI[1]=0;
  stabilizationSettings.RollPI[2]=50;
  stabilizationSettings.PitchPI[0]=2.8;
  stabilizationSettings.PitchPI[1]=0;
  stabilizationSettings.PitchPI[2]=50;
  stabilizationSettings.YawPI[0]=2.8;
  stabilizationSettings.YawPI[1]=0;
  stabilizationSettings.YawPI[2]=50;
  
  stabilizationSettings.RollMax=90;
  stabilizationSettings.PitchMax=90;
  stabilizationSettings.YawMax=45;
  stabilizationSettings.ManualRate[0]=450;
  stabilizationSettings.ManualRate[1]=450;
  stabilizationSettings.ManualRate[2]=310;
  stabilizationSettings.MaximumRate[0]=500;
  stabilizationSettings.MaximumRate[1]=500;
  stabilizationSettings.MaximumRate[2]=500;
  
  stabilizationSettings.RateExpo[0]=0;
  stabilizationSettings.RateExpo[1]=0;
  stabilizationSettings.RateExpo[2]=0;
  stabilizationSettings.PoiMaximumRate[0]=30;
  stabilizationSettings.PoiMaximumRate[1]=30;
  stabilizationSettings.PoiMaximumRate[2]=30;
  //stabilizationSettings.PoiMaximumRate[0]=120;
  
  
  
  
  stabilizationSettings.VbarSensitivity[0]=0.5;
  stabilizationSettings.VbarSensitivity[1]=0.5;
  stabilizationSettings.VbarSensitivity[2]=0.5;
  stabilizationSettings.VbarRollPID[0]=0.005;
  stabilizationSettings.VbarRollPID[1]=0.002;
  stabilizationSettings.VbarRollPID[2]=0;
  stabilizationSettings.VbarPitchPID[0]=0.005;
  stabilizationSettings.VbarPitchPID[1]=0.002;
  stabilizationSettings.VbarPitchPID[2]=0;
  stabilizationSettings.VbarYawPID[0]=0.005;
  stabilizationSettings.VbarYawPID[1]=0.002;
  stabilizationSettings.VbarYawPID[2]=0;
  stabilizationSettings.VbarTau=0.5;
  stabilizationSettings.VbarGyroSuppress=30;
  stabilizationSettings.VbarPiroComp=STABILIZATIONSETTINGS_VBARPIROCOMP_FALSE;
  stabilizationSettings.VbarMaxAngle=10;
  stabilizationSettings.GyroTau=0.005;
  stabilizationSettings.DerivativeCutoff=20;
  stabilizationSettings.DerivativeGamma=1;
  stabilizationSettings.MaxAxisLock=30;
  stabilizationSettings.MaxAxisLockRate=5;
  stabilizationSettings.WeakLevelingKp=0.1;
  stabilizationSettings.MaxWeakLevelingRate=5;
  stabilizationSettings.LowThrottleZeroIntegral=STABILIZATIONSETTINGS_LOWTHROTTLEZEROINTEGRAL_TRUE;
  stabilizationSettings.CoordinatedFlightYawPI[0]=0;
  stabilizationSettings.CoordinatedFlightYawPI[1]=0.1;
  stabilizationSettings.CoordinatedFlightYawPI[2]=0.5;
}