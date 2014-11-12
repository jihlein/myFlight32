/*
 * File: tasks100Hz.h
 *
 * Code generated for Simulink model 'tasks100Hz'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Tue Nov 04 18:37:19 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_tasks100Hz_h_
#define RTW_HEADER_tasks100Hz_h_
#include "rtwtypes.h"
#include <math.h>
#ifndef tasks100Hz_COMMON_INCLUDES_
# define tasks100Hz_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* tasks100Hz_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_tasks100Hz_T RT_MODEL_tasks100Hz_T;

#ifndef _DEFINED_TYPEDEF_FOR_eepromConfig_t_
#define _DEFINED_TYPEDEF_FOR_eepromConfig_t_

typedef struct {
  uint8_T version;
  real32_T mpuTempMin;
  real32_T mpuTempMax;
  real32_T accelBiasPolynomial[15];
  real32_T accelScaleFactorPolynomial[15];
  real32_T gyroBiasPolynomial[15];
  real32_T magBias[6];
  real32_T accelCutoff;
  real32_T kpAcc;
  real32_T kpMag;
  real32_T earthAccel100HzHPtau;
  real32_T posEstA;
  real32_T posEstB;
  real32_T hEstA;
  real32_T hEstB;
  uint8_T dlpfSetting;
  uint8_T sensorOrientation;
  real32_T attTrim[2];
  real32_T rollAndPitchRateScaling;
  real32_T yawRateScaling;
  real32_T attitudeScaling;
  real32_T nDotEdotScaling;
  real32_T hDotScaling;
  uint8_T receiverType;
  uint8_T slaveSpektrum;
  uint8_T rcMap[8];
  uint16_T escPwmRate;
  uint16_T servoPwmRate;
  real32_T midCommand;
  real32_T minCheck;
  real32_T maxCheck;
  real32_T minThrottle;
  real32_T maxThrottle;
  uint8_T mixerConfiguration;
  real32_T yawDirection;
  uint16_T triYawServoPwmRate;
  real32_T triYawServoMin;
  real32_T triYawServoMid;
  real32_T triYawServoMax;
  real32_T triCopterYawCmd500HzLowPassTau;
  uint8_T freeMixMotors;
  real32_T freeMix[32];
  real32_T rateP[3];
  real32_T rateI[3];
  real32_T rateD[3];
  real32_T rateLimit[3];
  real32_T attitudeP[3];
  real32_T attitudeI[3];
  real32_T attitudeD[3];
  real32_T attitudeLimit[3];
  real32_T velocityP[3];
  real32_T velocityI[3];
  real32_T velocityD[3];
  real32_T velocityLimit[3];
  real32_T positionP[3];
  real32_T positionI[3];
  real32_T positionD[3];
  real32_T positionLimit[3];
  real32_T rollAttAltCompensationGain;
  real32_T rollAttAltCompensationLimit;
  real32_T pitchAttAltCompensationGain;
  real32_T pitchAttAltCompensationLimit;
  uint8_T batteryCells;
  real32_T voltageMonitorScale;
  real32_T voltageMonitorBias;
  real32_T batteryLow;
  real32_T batteryVeryLow;
  real32_T batteryMaxLow;
  uint8_T armCount;
  uint8_T disarmCount;
  uint16_T activeTelemetry;
  uint8_T mavlinkEnabled;
  uint8_T useGPS;
  uint8_T verticalVelocityHoldOnly;
  uint8_T externalHMC5883;
  uint8_T externalMS5611;
  uint8_T CRCFlags;
  uint32_T CRCAtEnd;
} eepromConfig_t;

#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S8>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_DSTATE;/* '<S8>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator2_DSTAT_p;/* '<S7>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_DSTAT_h;/* '<S7>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator2_DSTAT_a;/* '<S6>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_DSTAT_c;/* '<S6>/Discrete-Time Integrator3' */
  real32_T earthAccelHP_states[3];     /* '<S3>/earthAccelHP' */
  real32_T earthAccelHP_tmp[3];        /* '<S3>/earthAccelHP' */
  boolean_T DelayInput1_DSTATE;        /* '<S37>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_a;      /* '<S35>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_c;      /* '<S33>/Delay Input1' */
  int8_T DiscreteTimeIntegrator3_PrevRes;/* '<S8>/Discrete-Time Integrator3' */
  int8_T DiscreteTimeIntegrator3_PrevR_p;/* '<S7>/Discrete-Time Integrator3' */
  int8_T DiscreteTimeIntegrator3_PrevR_d;/* '<S6>/Discrete-Time Integrator3' */
  uint8_T DiscreteTimeIntegrator3_IC_LOAD;/* '<S8>/Discrete-Time Integrator3' */
  uint8_T DiscreteTimeIntegrator3_IC_LO_o;/* '<S7>/Discrete-Time Integrator3' */
  uint8_T DiscreteTimeIntegrator3_IC_LO_i;/* '<S6>/Discrete-Time Integrator3' */
} DW_tasks100Hz_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T q[4];                       /* '<Root>/q' */
  real32_T accel[3];                   /* '<Root>/accel' */
  real32_T accelOneG;                  /* '<Root>/accelOneG' */
  int32_T gpsLatitude;                 /* '<Root>/gpsLatitude' */
  int32_T homeLatitude;                /* '<Root>/homeLatitude' */
  int32_T gpsNdot;                     /* '<Root>/gpsNdot' */
  int32_T gpsLongitude;                /* '<Root>/gpsLongitude' */
  int32_T homeLongitude;               /* '<Root>/homeLongitude' */
  int32_T gpsEdot;                     /* '<Root>/gpsEdot' */
  real32_T pressureAlt;                /* '<Root>/pressureAlt' */
  uint8_T systemOperational;           /* '<Root>/systemOperational' */
  real32_T magVar;                     /* '<Root>/magVar' */
} ExtU_tasks100Hz_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T earthAxisAccels[3];         /* '<Root>/earthAxisAccels' */
  real32_T earthAxisAccelsNF[3];       /* '<Root>/earthAxisAccelsNF' */
  real32_T bodyVelocityEstimates[3];   /* '<Root>/bodyVelocityEstimates' */
  real32_T bodyPositionEstimates[3];   /* '<Root>/bodyPositionEstimates' */
  real32_T velocityEstimates[3];       /* '<Root>/velocityEstimates' */
  real32_T positionEstimates[3];       /* '<Root>/positionEstimates' */
} ExtY_tasks100Hz_T;

/* Real-time Model Data Structure */
struct tag_RTM_tasks100Hz_T {
  const char_T * volatile errorStatus;
};

/* Imported (extern) block parameters */
extern eepromConfig_t eepromConfig;    /* Variable: eepromConfig
                                        * Referenced by:
                                        *   '<S6>/Constant1'
                                        *   '<S6>/Constant2'
                                        *   '<S7>/Constant1'
                                        *   '<S7>/Constant2'
                                        *   '<S8>/Constant1'
                                        *   '<S8>/Constant2'
                                        */

/* Block states (auto storage) */
extern DW_tasks100Hz_T tasks100Hz_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_tasks100Hz_T tasks100Hz_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_tasks100Hz_T tasks100Hz_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T earthAccelDenHP[2];    /* Variable: earthAccelDenHP
                                        * Referenced by: '<S3>/earthAccelHP'
                                        */
extern real32_T earthAccelNumHP[2];    /* Variable: earthAccelNumHP
                                        * Referenced by: '<S3>/earthAccelHP'
                                        */

/* Model entry point functions */
extern void tasks100Hz_initialize(void);
extern void tasks100Hz_step(void);

/* Real-time Model object */
extern RT_MODEL_tasks100Hz_T *const tasks100Hz_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'tasks100Hz'
 * '<S1>'   : 'tasks100Hz/Quaternion Rotation1'
 * '<S2>'   : 'tasks100Hz/Quaternion Rotation2'
 * '<S3>'   : 'tasks100Hz/accelRotation'
 * '<S4>'   : 'tasks100Hz/calcDeltaE'
 * '<S5>'   : 'tasks100Hz/calcDeltaN'
 * '<S6>'   : 'tasks100Hz/eEstimates'
 * '<S7>'   : 'tasks100Hz/hEstimates'
 * '<S8>'   : 'tasks100Hz/nEstimates'
 * '<S9>'   : 'tasks100Hz/rotationCorrection'
 * '<S10>'  : 'tasks100Hz/Quaternion Rotation1/Quaternion Normalize'
 * '<S11>'  : 'tasks100Hz/Quaternion Rotation1/singleV1'
 * '<S12>'  : 'tasks100Hz/Quaternion Rotation1/singleV2'
 * '<S13>'  : 'tasks100Hz/Quaternion Rotation1/singleV3'
 * '<S14>'  : 'tasks100Hz/Quaternion Rotation1/Quaternion Normalize/Quaternion Modulus'
 * '<S15>'  : 'tasks100Hz/Quaternion Rotation1/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S16>'  : 'tasks100Hz/Quaternion Rotation2/Quaternion Normalize'
 * '<S17>'  : 'tasks100Hz/Quaternion Rotation2/singleV1'
 * '<S18>'  : 'tasks100Hz/Quaternion Rotation2/singleV2'
 * '<S19>'  : 'tasks100Hz/Quaternion Rotation2/singleV3'
 * '<S20>'  : 'tasks100Hz/Quaternion Rotation2/Quaternion Normalize/Quaternion Modulus'
 * '<S21>'  : 'tasks100Hz/Quaternion Rotation2/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S22>'  : 'tasks100Hz/accelRotation/Quaternion Conjugate'
 * '<S23>'  : 'tasks100Hz/accelRotation/Quaternion Rotation'
 * '<S24>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/Quaternion Normalize'
 * '<S25>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/singleV1'
 * '<S26>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/singleV2'
 * '<S27>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/singleV3'
 * '<S28>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/Quaternion Normalize/Quaternion Modulus'
 * '<S29>'  : 'tasks100Hz/accelRotation/Quaternion Rotation/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S30>'  : 'tasks100Hz/calcDeltaE/stdRadianLimit'
 * '<S31>'  : 'tasks100Hz/calcDeltaE/stdRadianLimit/Compare To Constant'
 * '<S32>'  : 'tasks100Hz/calcDeltaE/stdRadianLimit/Compare To Constant1'
 * '<S33>'  : 'tasks100Hz/eEstimates/Detect Rise Positive1'
 * '<S34>'  : 'tasks100Hz/eEstimates/Detect Rise Positive1/Positive'
 * '<S35>'  : 'tasks100Hz/hEstimates/Detect Rise Positive1'
 * '<S36>'  : 'tasks100Hz/hEstimates/Detect Rise Positive1/Positive'
 * '<S37>'  : 'tasks100Hz/nEstimates/Detect Rise Positive1'
 * '<S38>'  : 'tasks100Hz/nEstimates/Detect Rise Positive1/Positive'
 * '<S39>'  : 'tasks100Hz/rotationCorrection/Quaternions to Rotation Angles'
 * '<S40>'  : 'tasks100Hz/rotationCorrection/Rotation Angles to Quaternions Rotation Order: ZYX'
 * '<S41>'  : 'tasks100Hz/rotationCorrection/stdRadianLimit'
 * '<S42>'  : 'tasks100Hz/rotationCorrection/Quaternions to Rotation Angles/Quaternion Normalize'
 * '<S43>'  : 'tasks100Hz/rotationCorrection/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
 * '<S44>'  : 'tasks100Hz/rotationCorrection/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S45>'  : 'tasks100Hz/rotationCorrection/stdRadianLimit/Compare To Constant'
 * '<S46>'  : 'tasks100Hz/rotationCorrection/stdRadianLimit/Compare To Constant1'
 */
#endif                                 /* RTW_HEADER_tasks100Hz_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
