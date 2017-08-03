#include "attitudesolving.h"

// Private constants
#define STACK_SIZE_BYTES 580
#define TASK_PRIORITY (tskIDLE_PRIORITY+3)

#define SENSOR_PERIOD 4
#define GYRO_NEUTRAL 1665

// Private types
enum complimentary_filter_status {
  CF_POWERON,
  CF_INITIALIZING,
  CF_ARMING,
  CF_NORMAL
};


// Private functions
void AttitudeTask(void *parameters);

float gyro_correct_int[3] = {0,0,0};

int32 updateSensorsCC3D(AccelsData * accelsData, GyrosData * gyrosData);
void updateAttitude(AccelsData *, GyrosData *);
void update_accels(AccelsData * accelsData);
void update_gyros(GyrosData * gyrosData);
void update_trimming(AccelsData * accelsData);
void updateTemperatureComp(float temperature, float *temp_bias);

//! Compute the mean gyro accumulated and assign the bias
void accumulate_gyro_compute();

//! Zero the gyro accumulators
void accumulate_gyro_zero();

//! Store a gyro sample
void accumulate_gyro(float gyros_out[3]);

void apply_accel_filter(const float * raw, float * filtered);

float accelKi = 0;
float accelKp = 0;
float accel_alpha = 0;
_Bool accel_filter_enabled = false;
float yawBiasRate = 0;
const float IDG_GYRO_GAIN = 0.42;
float q[4] = {1,0,0,0};
float Rsb[3][3];                                                                // Rotation matrix which transforms from the body frame to the sensor board frame
int8 rotate = 1;
_Bool zero_during_arming = false;
_Bool bias_correct_gyro = true;

// For computing the average gyro during arming
_Bool accumulating_gyro = false;
uint32 accumulated_gyro_samples = 0;
float accumulated_gyro[3];

// For running trim flights
volatile _Bool trim_requested = false;
volatile int32 trim_accels[3];
volatile int32 trim_samples;
int32 const MAX_TRIM_FLIGHT_SAMPLES = 65535;

#define ADXL345_ACCEL_SCALE  (GRAVITY * 0.004f)
/* 0.004f is gravity / LSB */
enum complimentary_filter_status complimentary_filter_status;


/**
* Initialise the module, called on startup
* \returns 0 on success or -1 if initialisation failed
*/
void AttitudeInitialize(void)
{
  //AttitudeActualInitialize();
  //SensorSettingsInitialize();
  //AttitudeSettingsInitialize();
  //AccelsInitialize();
  //GyrosInitialize();
  
  // Initialize quaternion
  //AttitudeActualData attitude;
  //AttitudeActualGet(&attitude);
  attitudeActual.q1 = 1;                                                        //attitudeActual struct {q1 ~ q4, Yaw, Row, Pitch} 
  attitudeActual.q2 = 0;
  attitudeActual.q3 = 0;
  attitudeActual.q4 = 0;
  //AttitudeActualSet(&attitude);
  
  // Cannot trust the values to init right above if BL runs
  gyro_correct_int[0] = 0;                                                      //float gyro_correct_int[3] = {0,0,0} initialized
  gyro_correct_int[1] = 0;
  gyro_correct_int[2] = 0;
  
  q[0] = 1;                                                                     //float q[4] = {1,0,0,0} initialized
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
  float rotationQuat[4] = {1,0,0,0};
  Quaternion2R(rotationQuat, Rsb);                                              //float Rsb[3][3] initialized ::note:: What is the usage of Rsb? No init to this varible?
  trim_requested = false;
  complimentary_filter_status = CF_POWERON;                                     //complimentary_filter_status = CF_POWERON, CF_INITIALIZING, CF_ARMING, CF_NORMAL
  ////////////////timer/////////////////////////
  //  timer.stamp0=timestampget(&timer);
  
  OS_ERR err;
  timer.stamp0=OSTimeGet(&err);
  /////////////////////////////////////////////  
  //AttitudeSettingsConnectCallback(&settingsUpdatedCb);
  //SensorSettingsConnectCallback(&settingsUpdatedCb);
  accelKp = attitudeSettings.AccelKp;                                           //float accelKp = 0 init; attitudeSettings.AccelKp=0.05 init;
  accelKi = attitudeSettings.AccelKi;                                           //float accelKi = 0 init; attitudeSettings.AccelKi=0.0001f init;  
  yawBiasRate = attitudeSettings.YawBiasRate;                                   //float yawBiasRate = 0 init; attitudeSettings.YawBiasRate=0.000001;
  
  // Calculate accel filter alpha, in the same way as for gyro data in stabilization module.
  const float fakeDt = 0.0025f;
  if(attitudeSettings.AccelTau < 0.0001f) 
  {                                     // attitudeSettings.AccelTau=0.1 init;                                     
    accel_alpha = 0;   // not trusting this to resolve to 0
    accel_filter_enabled = false;
  } 
  else 
  {
    accel_alpha = expf(-fakeDt  / attitudeSettings.AccelTau);
    accel_filter_enabled = true;
  }
  
  zero_during_arming = attitudeSettings.ZeroDuringArming                        //_Bool zero_during_arming = false;
    == ATTITUDESETTINGS_ZERODURINGARMING_TRUE;
  bias_correct_gyro = attitudeSettings.BiasCorrectGyro                          //_Bool bias_correct_gyro = true;                    
    == ATTITUDESETTINGS_BIASCORRECTGYRO_TRUE;
  
  gyro_correct_int[0] = 0;
  gyro_correct_int[1] = 0;
  gyro_correct_int[2] = 0;
  
  // Indicates not to expend cycles on rotation
  if(attitudeSettings.BoardRotation[0] == 0                                     //::note::What is the init value of attitudeSettings.BoardRotation?
     && attitudeSettings.BoardRotation[1] == 0
       && attitudeSettings.BoardRotation[2] == 0) 
  {
    rotate = 0;
    // Shouldn't be used but to be safe
    float rotationQuat[4] = {1,0,0,0};
    Quaternion2R(rotationQuat, Rsb);
  } 
  else 
  {
    float rotationQuat[4];
    const float rpy[3] = {attitudeSettings.BoardRotation[0] / 100.0f,
    attitudeSettings.BoardRotation[1] / 100.0f,
    attitudeSettings.BoardRotation[2] / 100.0f};
    RPY2Quaternion(rpy, rotationQuat);
    Quaternion2R(rotationQuat, Rsb);
    rotate = 1;
  }
  
  if (attitudeSettings.TrimFlight == ATTITUDESETTINGS_TRIMFLIGHT_START) 
  {       //::note:: init as ATTITUDESETTINGS_TRIMFLIGHT_NORMAL, What's the usage of this member?
    trim_accels[0] = 0;
    trim_accels[1] = 0;
    trim_accels[2] = 0;
    trim_samples = 0;
    trim_requested = true;
  } 
  else if (attitudeSettings.TrimFlight == ATTITUDESETTINGS_TRIMFLIGHT_LOAD) 
  {
    trim_requested = false;
    
    // Get sensor data  mean 
    int32 trim_samples_temp = trim_samples;    
    float a_body[3] = 
    { 
      trim_accels[0] / trim_samples_temp,                          //volatile int32 trim_accels[3]; volatile int32 trim_samples;
      trim_accels[1] / trim_samples_temp,                                              //::note::init of trim_accels & trim_samples
      trim_accels[2] / trim_samples_temp
    };
    
    // Inverse rotation of sensor data, from body frame into sensor frame
    float a_sensor[3];
    rot_mult(Rsb, a_body, a_sensor, false);
    
    // Temporary variables
    float psi, theta, phi;                                                      //::note::What psi, theta, phi stands for?
    
    psi = attitudeSettings.BoardRotation[2] * DEG2RAD / 100.0f;
    
    float cP = cosf(psi);                                                       
    float sP = sinf(psi);
    
    // In case psi is too small, we have to use a different equation to solve for theta
    if (fabsf(psi) > PI / 2)
      theta = atanf((a_sensor[1] + cP * (sP * a_sensor[0] -
					 cP * a_sensor[1])) / (sP * a_sensor[2]));
    else
      theta = atanf((a_sensor[0] - sP * (sP * a_sensor[0] -
					 cP * a_sensor[1])) / (cP * a_sensor[2]));
    
    phi = atan2f((sP * a_sensor[0] - cP * a_sensor[1]) / GRAVITY,
                 (a_sensor[2] / cosf(theta) / GRAVITY));
    
    attitudeSettings.BoardRotation[0] = (int16 )(phi * RAD2DEG * 100.0f);
    attitudeSettings.BoardRotation[1] = (int16 )(theta * RAD2DEG * 100.0f);
    
    attitudeSettings.TrimFlight = ATTITUDESETTINGS_TRIMFLIGHT_NORMAL;
  }
  
}

/**
* Module thread, should not return.
*/
void attsolving()
{  
  OS_ERR err;
#if 1
  uint32_t arming_count = 0;
  
  if (complimentary_filter_status == CF_POWERON) 
  {
/*         complimentary_filter_status = (timestampget(&timer)-timer.stamp0 > 1000) ? CF_INITIALIZING : CF_POWERON;              */
    complimentary_filter_status = (OSTimeGet(&err)-timer.stamp0 > 1000) ? 
  CF_INITIALIZING : CF_POWERON;
  } 
/*   else if(complimentary_filter_status == CF_INITIALIZING && (timestampget(&timer)-timer.stamp0 < 3000) && (timestampget(&timer)-timer.stamp0 > 1000))              */
  else if(complimentary_filter_status == CF_INITIALIZING &&
          (OSTimeGet(&err)-timer.stamp0 < 3000) && 
            (OSTimeGet(&err)-timer.stamp0 > 1000))    
  {
    
    // For first 7 seconds use accels to get gyro bias
    accelKp = 0.1f;// + 0.1f * (xTaskGetTickCount() < 4000);
    accelKi = 0.1f;
    yawBiasRate = 0.1;
    //accumulate_gyro_zero();
    accumulate_gyro_compute();
    accel_filter_enabled = false;
    
  } 
  else if (zero_during_arming && 
           (flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMING)) 
  {
    
    // Use a rapidly decrease accelKp to force the attitude to snap back
    // to level and then converge more smoothly
    if (arming_count < 20)
      accelKp = 1.7f;
    else if (accelKp > 0.1f)
      accelKp -= 0.01f;
    arming_count++;
    
    accelKi = 0.1f;
    yawBiasRate = 0.1f;
    accel_filter_enabled = false;
    
    // Indicate arming so that after arming it reloads
    // the normal settings
    if (complimentary_filter_status != CF_ARMING) 
    {
      accumulate_gyro_zero();
      complimentary_filter_status = CF_ARMING;
      accumulating_gyro = true;
    }
    
  } 
  else if (complimentary_filter_status == CF_ARMING ||
           complimentary_filter_status == CF_INITIALIZING) 
  {
    
    // Reload settings (all the rates)
    //AttitudeSettingsAccelKiGet(&accelKi);
    //AttitudeSettingsAccelKpGet(&accelKp);
    //AttitudeSettingsYawBiasRateGet(&yawBiasRate);
    accelKp = 0.05f;// + 0.1f * (xTaskGetTickCount() < 4000);
    accelKi = 0.0001f;
    yawBiasRate = 0.0f;
    if(accel_alpha > 0.0f)
      accel_filter_enabled = true;
    
    // If arming that means we were accumulating gyro
    // samples.  Compute new bias.
    if (complimentary_filter_status == CF_ARMING) 
    {
      accumulate_gyro_compute();
      accumulating_gyro = false;
      arming_count = 0;
    }
    
    // Indicate normal mode to prevent rerunning this code
    complimentary_filter_status = CF_NORMAL;
  }
#endif		
  //		PIOS_WDG_UpdateFlag(PIOS_WDG_ATTITUDE);
  
  //AccelsData accels;
  //GyrosData gyros;
  int32_t retval = 0;
  
  //if (cc3d)
  retval = updateSensorsCC3D(&accelsData, &gyrosData);
  //else
  //	retval = updateSensors(&accels, &gyros);
  
  // During power on set to angle from accel
  if (complimentary_filter_status == CF_POWERON) 
  {
    float RPY[3];
    float theta = atan2f(accelsData.x, -accelsData.z);
    RPY[1] = theta * RAD2DEG;
    RPY[0] = atan2f(-accelsData.y, -accelsData.z / cosf(theta)) * RAD2DEG;
    RPY[2] = 0;
    RPY2Quaternion(RPY, q);
  }
  
  // Only update attitude when sensor data is good
  if (retval != 0)
  {
    //	AlarmsSet(SYSTEMALARMS_ALARM_ATTITUDE, SYSTEMALARMS_ALARM_ERROR);
  }
  else 
  {
    // Do not update attitude data in simulation mode
    //	if (!AttitudeActualReadOnly())
    updateAttitude(&accelsData, &gyrosData);
    
    //AlarmsClear(SYSTEMALARMS_ALARM_ATTITUDE);
  }
  //}
}
/**
* Get an update from the sensors
* @param[in] attitudeRaw Populate the UAVO instead of saving right here
* @return 0 if successfull, -1 if not
*/
int32 updateSensorsCC3D(AccelsData * accelsData, GyrosData * gyrosData)
{
  getattdata();
  //queue = PIOS_SENSORS_GetQueue(PIOS_SENSOR_GYRO);
  //if(queue == NULL || xQueueReceive(queue, (void *) &gyros, 4) == errQUEUE_EMPTY) {
  //	return-1;
  //}
  
  // As it says below, because the rest of the code expects the accel to be ready when
  // the gyro is we must block here too
  //queue = PIOS_SENSORS_GetQueue(PIOS_SENSOR_ACCEL);
  //if(queue == NULL || xQueueReceive(queue, (void *) &accels, 1) == errQUEUE_EMPTY) {
  //	return -1;
  //}
  //else
  update_accels(accelsData);
  
  // Update gyros after the accels since the rest of the code expects
  // the accels to be available first
  update_gyros(gyrosData);
  
  update_trimming(accelsData);
  
  //GyrosSet(gyrosData);
  //AccelsSet(accelsData);
  
  return 0;
}

/**
* @brief Apply calibration and rotation to the raw accel data
* @param[in] accels The raw accel data
*/
void update_accels( AccelsData * accelsData)
{
  // Average and scale the accels before rotation
  float accels_out[3] = {sensorData.A_X * sensorSettings.AccelScale[0] - sensorSettings.AccelBias[0],
  sensorData.A_Y * sensorSettings.AccelScale[1] - sensorSettings.AccelBias[1],
  sensorData.A_Z* sensorSettings.AccelScale[2] - sensorSettings.AccelBias[2]};
  
  if (rotate) {
    float accel_rotated[3];
    rot_mult(Rsb, accels_out, accel_rotated, true);
    accelsData->x = accel_rotated[0];
    accelsData->y = accel_rotated[1];
    accelsData->z = accel_rotated[2];
  } else {
    accelsData->x = accels_out[0];
    accelsData->y = accels_out[1];
    accelsData->z = accels_out[2];
  }
  
  accelsData->temperature = sensorData.T;
}

/**
* @brief Apply calibration and rotation to the raw gyro data
* @param[in] gyros The raw gyro data
*/
float gyro_temp_bias[3] = {0,0,0};
void update_gyros(GyrosData * gyrosData)
{
  
  
  // Scale the gyros
  float gyros_out[3] = {sensorData.G_X * sensorSettings.GyroScale[0],
  sensorData.G_Y * sensorSettings.GyroScale[1],
  sensorData.G_Z * sensorSettings.GyroScale[2]};
  
  // Update the bias due to the temperature
  updateTemperatureComp(gyrosData->temperature, gyro_temp_bias);
  
  // Apply temperature bias correction before the rotation
  if (bias_correct_gyro) {
    gyros_out[0] -= gyro_temp_bias[0];
    gyros_out[1] -= gyro_temp_bias[1];
    gyros_out[2] -= gyro_temp_bias[2];
  }
  
  // When computing the bias accumulate samples
  accumulate_gyro(gyros_out);
  
  
  if (rotate) {
    float gyros[3];
    rot_mult(Rsb, gyros_out, gyros, true);
    gyrosData->x = gyros[0];
    gyrosData->y = gyros[1];
    gyrosData->z = gyros[2];
  } else {
    gyrosData->x = gyros_out[0];
    gyrosData->y = gyros_out[1];
    gyrosData->z = gyros_out[2];
  }
  
  if(bias_correct_gyro) {
    // Applying integral component here so it can be seen on the gyros and correct bias
    gyrosData->x -= gyro_correct_int[0];
    gyrosData->y -= gyro_correct_int[1];
    gyrosData->z -= gyro_correct_int[2];
  }
  
  // Because most crafts wont get enough information from gravity to zero yaw gyro, we try
  // and make it average zero (weakly)
  gyro_correct_int[2] += gyrosData->z * yawBiasRate;
  
  gyrosData->temperature = sensorData.T;
}

/**
* If accumulating data and enough samples acquired then recompute
* the gyro bias based on the mean accumulated
*/
void accumulate_gyro_compute()
{
  if (accumulating_gyro && 
      accumulated_gyro_samples > 100) {
        
        gyro_correct_int[0] = accumulated_gyro[0] / accumulated_gyro_samples;
        gyro_correct_int[1] = accumulated_gyro[1] / accumulated_gyro_samples;
        gyro_correct_int[2] = accumulated_gyro[2] / accumulated_gyro_samples;
        
        accumulate_gyro_zero();
        
        accumulating_gyro = false;
      }
}

/**
* Zero the accumulation of gyro data
*/

void accumulate_gyro_zero()
{
  accumulated_gyro_samples = 0;
  accumulated_gyro[0] = 0;
  accumulated_gyro[1] = 0;
  accumulated_gyro[2] = 0;
}

/**
* Accumulate a set of gyro samples for computing the
* bias
* @param [in] gyrosData The samples of data to accumulate
* @param [in] gyro_temp_bias The current temperature bias to account for
*/
void accumulate_gyro(float gyros_out[3])
{
  if (!accumulating_gyro)
    return;
  
  accumulated_gyro_samples++;
  accumulated_gyro[0] += gyros_out[0];
  accumulated_gyro[1] += gyros_out[1];
  accumulated_gyro[2] += gyros_out[2];
}

/**
* @brief If requested accumulate accel values to calculate level
* @param[in] accelsData the scaled and normalized accels
*/
void update_trimming(AccelsData * accelsData)
{
  if (trim_requested) {
    if (trim_samples >= MAX_TRIM_FLIGHT_SAMPLES) {
      trim_requested = false;
    } else {
      uint8 armed=0;
      float throttle=0;
      //FlightStatusArmedGet(&armed);
      //ManualControlCommandThrottleGet(&throttle);  // Until flight status indicates airborne
      if ((armed == FLIGHTSTATUS_ARMED_ARMED) && (throttle > 0)) {
        trim_samples++;
        // Store the digitally scaled version since that is what we use for bias
        trim_accels[0] += accelsData->x;
        trim_accels[1] += accelsData->y;
        trim_accels[2] += accelsData->z;
      }
    }
  }
}

void apply_accel_filter(const float * raw, float * filtered)
{
  if(accel_filter_enabled) {
    filtered[0] = filtered[0] * accel_alpha + raw[0] * (1 - accel_alpha);
    filtered[1] = filtered[1] * accel_alpha + raw[1] * (1 - accel_alpha);
    filtered[2] = filtered[2] * accel_alpha + raw[2] * (1 - accel_alpha);
  } else {
    filtered[0] = raw[0];
    filtered[1] = raw[1];
    filtered[2] = raw[2];
  }
}

void updateAttitude(AccelsData * accelsData, GyrosData * gyrosData)
{
  float dT;
  float accels_filtered[3] = {0,0,0};
  float grot_filtered[3] = {0,0,0};
  
  dT =CTL_RATE*1e-6f;
  //lastSysTime = thisSysTime;
  
  // Bad practice to assume structure order, but saves memory
  float * gyros = &gyrosData->x;
  float * accels = &accelsData->x;
  
  float grot[3];
  float accel_err[3];
  
  // Apply smoothing to accel values, to reduce vibration noise before main calculations.
  apply_accel_filter(accels,accels_filtered);
  
  // Rotate gravity to body frame, filter and cross with accels
  grot[0] = -(2 * (q[1] * q[3] - q[0] * q[2]));
  grot[1] = -(2 * (q[2] * q[3] + q[0] * q[1]));
  grot[2] = -(q[0] * q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
  
  // Apply same filtering to the rotated attitude to match delays
  apply_accel_filter(grot,grot_filtered);
  
  // Compute the error between the predicted direction of gravity and smoothed acceleration
  CrossProduct((const float *) accels_filtered, (const float *) grot_filtered, accel_err);
  
  // Account for accel magnitude
  float accel_mag = sqrtf(accels_filtered[0]*accels_filtered[0] + accels_filtered[1]*accels_filtered[1] + accels_filtered[2]*accels_filtered[2]);
  
  // Account for filtered gravity vector magnitude
  float grot_mag;
  
  if (accel_filter_enabled)
    grot_mag = sqrtf(grot_filtered[0]*grot_filtered[0] + grot_filtered[1]*grot_filtered[1] + grot_filtered[2]*grot_filtered[2]);
  else
    grot_mag = 1.0f;
  
  if (grot_mag > 1.0e-3f && accel_mag > 1.0e-3f) 
  {
    accel_err[0] /= (accel_mag*grot_mag);
    accel_err[1] /= (accel_mag*grot_mag);
    accel_err[2] /= (accel_mag*grot_mag);
    
    // Accumulate integral of error.  Scale here so that units are (deg/s) but Ki has units of s
    gyro_correct_int[0] -= accel_err[0] * accelKi;
    gyro_correct_int[1] -= accel_err[1] * accelKi;
    
    // Correct rates based on error, integral component dealt with in updateSensors
    gyros[0] += accel_err[0] * accelKp / dT;
    gyros[1] += accel_err[1] * accelKp / dT;
    gyros[2] += accel_err[2] * accelKp / dT;
  }
  
  // scoping variables to save memory
  // Work out time derivative from INSAlgo writeup
  // Also accounts for the fact that gyros are in deg/s
  float qdot[4];
  qdot[0] = (-q[1] * gyros[0] - q[2] * gyros[1] - q[3] * gyros[2]) * dT * DEG2RAD / 2;
  qdot[1] = (q[0] * gyros[0] - q[3] * gyros[1] + q[2] * gyros[2]) * dT * DEG2RAD / 2;
  qdot[2] = (q[3] * gyros[0] + q[0] * gyros[1] - q[1] * gyros[2]) * dT * DEG2RAD / 2;
  qdot[3] = (-q[2] * gyros[0] + q[1] * gyros[1] + q[0] * gyros[2]) * dT * DEG2RAD / 2;
  
  // Take a time step
  q[0] = q[0] + qdot[0];
  q[1] = q[1] + qdot[1];
  q[2] = q[2] + qdot[2];
  q[3] = q[3] + qdot[3];
  
  if(q[0] > 0) {
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
  }
  
  
  // Renomalize
  float qmag = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] = q[0] / qmag;
  q[1] = q[1] / qmag;
  q[2] = q[2] / qmag;
  q[3] = q[3] / qmag;
  
  // If quaternion has become inappropriately short or is nan reinit.
  // THIS SHOULD NEVER ACTUALLY HAPPEN
  if((fabs(qmag) < 1e-3) || (qmag != qmag)) {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
  
  //AttitudeActualData attitudeActual;
  //AttitudeActualGet(&attitudeActual);
  
  quat_copy(q, &attitudeActual.q1);
  
  // Convert into eueler degrees (makes assumptions about RPY order)
  Quaternion2RPY(&attitudeActual.q1,&attitudeActual.Roll);
  //attitudeActual.Yaw=      atan2f( 2 * q[0] * q[1] + q[2] * q[3], q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] )*57.3;//yaw
  //        attitudeActual.Pitch=      asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.3;
  //      attitudeActual.Roll =    atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.3; //roll
  //AttitudeActualSet(&attitudeActual);
}

/**
* Compute the bias expected from temperature variation for each gyro
* channel
*/
void updateTemperatureComp(float temperature, float *temp_bias)
{
  int temp_counter = 0;
  float temp_accum = 0;
  
  const float TEMP_MIN = -10;
  const float TEMP_MAX = 60;
  
  if (temperature < TEMP_MIN)
    temperature = TEMP_MIN;
  if (temperature > TEMP_MAX)
    temperature = TEMP_MAX;
  
  if (temp_counter < 500) {
    temp_accum += temperature;
    temp_counter ++;
  } else {
    float t = temp_accum / temp_counter;
    temp_accum = 0;
    temp_counter = 0;
    
    // Compute a third order polynomial for each chanel after each 500 samples
    temp_bias[0] = sensorSettings.XGyroTempCoeff[0] + 
      sensorSettings.XGyroTempCoeff[1] * t + 
        sensorSettings.XGyroTempCoeff[2] * powf(t,2) + 
          sensorSettings.XGyroTempCoeff[3] * powf(t,3);
    temp_bias[1] = sensorSettings.YGyroTempCoeff[0] + 
      sensorSettings.YGyroTempCoeff[1] * t + 
        sensorSettings.YGyroTempCoeff[2] * powf(t,2) + 
          sensorSettings.YGyroTempCoeff[3] * powf(t,3);
    temp_bias[2] = sensorSettings.ZGyroTempCoeff[0] + 
      sensorSettings.ZGyroTempCoeff[1] * t + 
        sensorSettings.ZGyroTempCoeff[2] * powf(t,2) + 
          sensorSettings.ZGyroTempCoeff[3] * powf(t,3);
  }
}
