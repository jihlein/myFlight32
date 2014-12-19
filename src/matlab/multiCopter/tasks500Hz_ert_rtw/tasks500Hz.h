/*
 * File: tasks500Hz.h
 *
 * Code generated for Simulink model 'tasks500Hz'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Fri Dec 19 10:58:37 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_tasks500Hz_h_
#define RTW_HEADER_tasks500Hz_h_
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#ifndef tasks500Hz_COMMON_INCLUDES_
# define tasks500Hz_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* tasks500Hz_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_tasks500Hz_T RT_MODEL_tasks500Hz_T;

#ifndef _DEFINED_TYPEDEF_FOR_eepromConfig_t_
#define _DEFINED_TYPEDEF_FOR_eepromConfig_t_

typedef struct {
  uint8_T version;
  real32_T mpuTempMin;
  real32_T mpuTempMax;
  real32_T accelBiasPolynomial[12];
  real32_T accelScaleFactorPolynomial[12];
  real32_T gyroBiasPolynomial[12];
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
  real32_T DiscreteTimeIntegrator_DSTATE[3];/* '<S9>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE[3];/* '<S9>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_f[3];/* '<S14>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_f[3];/* '<S14>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_k[3];/* '<S5>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_i[3];/* '<S5>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_m[3];/* '<S10>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_k[3];/* '<S10>/Discrete-Time Integrator1' */
  real32_T UnitDelay_DSTATE;           /* '<S11>/Unit Delay' */
  real32_T triYawRate_states;          /* '<S22>/triYawRate' */
  real32_T accelConfidenceDecay;       /* '<S4>/mahonyAHRS_MC' */
  real32_T eAcc[3];                    /* '<S4>/mahonyAHRS_MC' */
  real32_T eMag[3];                    /* '<S4>/mahonyAHRS_MC' */
  real32_T pastAccelFilter;            /* '<S4>/mahonyAHRS_MC' */
  real32_T q[4];                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q1q1;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q1q2;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q1q3;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q1q4;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q2q2;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q2q3;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q2q4;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q3q3;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q3q4;                       /* '<S4>/mahonyAHRS_MC' */
  real32_T q4q4;                       /* '<S4>/mahonyAHRS_MC' */
  int8_T DiscreteTimeIntegrator_PrevRese[3];/* '<S9>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes[3];/* '<S9>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_c[3];/* '<S14>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_c[3];/* '<S14>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_a[3];/* '<S5>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_a[3];/* '<S5>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_h;/* '<S10>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_p;/* '<S10>/Discrete-Time Integrator1' */
  uint8_T ahrsFirstPass;               /* '<S4>/mahonyAHRS_MC' */
  uint8_T qValid;                      /* '<S4>/mahonyAHRS_MC' */
  uint8_T qValidCount;                 /* '<S4>/mahonyAHRS_MC' */
} DW_tasks500Hz_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Expression: mixTri
   * Referenced by: '<Root>/Constant1'
   */
  real32_T Constant1_Value[32];

  /* Expression: mixQuadX
   * Referenced by: '<Root>/Constant2'
   */
  real32_T Constant2_Value[32];

  /* Expression: mixHexX
   * Referenced by: '<Root>/Constant3'
   */
  real32_T Constant3_Value[32];

  /* Expression: mixY6
   * Referenced by: '<Root>/Constant9'
   */
  real32_T Constant9_Value[32];
} ConstP_tasks500Hz_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T gyro[3];                    /* '<Root>/gyro ' */
  real32_T accel[3];                   /* '<Root>/accel' */
  real32_T mag[3];                     /* '<Root>/mag' */
  real32_T dt;                         /* '<Root>/dt' */
  uint8_T gyroValid;                   /* '<Root>/gyroValid' */
  uint8_T accelValid;                  /* '<Root>/accelValid' */
  uint8_T magValid;                    /* '<Root>/magValid' */
  real32_T accelOneG;                  /* '<Root>/accelOneG' */
  uint8_T accelDataUpdate;             /* '<Root>/accelDataUpdate' */
  uint8_T magDataUpdate;               /* '<Root>/magDataUpdate' */
  real32_T magVar;                     /* '<Root>/magVar' */
  real32_T velocities[3];              /* '<Root>/velocities' */
  real32_T positions[3];               /* '<Root>/positions' */
  uint8_T rateModes[3];                /* '<Root>/rateModes' */
  real32_T rateCmds[3];                /* '<Root>/rateCmds' */
  uint8_T attModes[3];                 /* '<Root>/attModes' */
  real32_T attCmds[3];                 /* '<Root>/attCmds' */
  uint8_T velModes[3];                 /* '<Root>/velModes' */
  real32_T velCmds[3];                 /* '<Root>/velCmds' */
  uint8_T posModes[3];                 /* '<Root>/posModes' */
  real32_T posCmds[3];                 /* '<Root>/posCmds' */
  uint8_T resetPIDs;                   /* '<Root>/resetPIDs' */
} ExtU_tasks500Hz_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T axisCmds[4];                /* '<Root>/axisCmds' */
  real32_T motorCmds[8];               /* '<Root>/motorCmds' */
  real32_T triCopterServoCmd;          /* '<Root>/triCopterServoCmd' */
  real32_T q[4];                       /* '<Root>/q' */
  uint8_T attValid;                    /* '<Root>/attValid' */
  real32_T attitudes[3];               /* '<Root>/attitudes' */
} ExtY_tasks500Hz_T;

/* Real-time Model Data Structure */
struct tag_RTM_tasks500Hz_T {
  const char_T * volatile errorStatus;
};

/* Imported (extern) block parameters */
extern eepromConfig_t eepromConfig;    /* Variable: eepromConfig
                                        * Referenced by:
                                        *   '<Root>/Constant'
                                        *   '<Root>/Constant4'
                                        *   '<Root>/Constant5'
                                        *   '<Root>/Constant6'
                                        *   '<Root>/Constant7'
                                        *   '<Root>/Constant8'
                                        *   '<S5>/Constant'
                                        *   '<S5>/Gain'
                                        *   '<S5>/Gain1'
                                        *   '<S5>/Gain2'
                                        *   '<S8>/Gain1'
                                        *   '<S9>/Constant'
                                        *   '<S9>/Gain'
                                        *   '<S9>/Gain1'
                                        *   '<S9>/Gain2'
                                        *   '<S10>/Constant'
                                        *   '<S10>/Gain'
                                        *   '<S10>/Gain1'
                                        *   '<S10>/Gain2'
                                        *   '<S13>/Constant1'
                                        *   '<S13>/Constant5'
                                        *   '<S14>/Constant'
                                        *   '<S14>/Gain'
                                        *   '<S14>/Gain1'
                                        *   '<S14>/Gain2'
                                        *   '<S22>/Constant5'
                                        *   '<S22>/Saturation3'
                                        */

/* Block states (auto storage) */
extern DW_tasks500Hz_T tasks500Hz_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_tasks500Hz_T tasks500Hz_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_tasks500Hz_T tasks500Hz_Y;

/* Constant parameters (auto storage) */
extern const ConstP_tasks500Hz_T tasks500Hz_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T triYawRateDenLP[2];    /* Variable: triYawRateDenLP
                                        * Referenced by: '<S22>/triYawRate'
                                        */
extern real32_T triYawRateNumLP[2];    /* Variable: triYawRateNumLP
                                        * Referenced by: '<S22>/triYawRate'
                                        */

/* Model entry point functions */
extern void tasks500Hz_initialize(void);
extern void tasks500Hz_step(void);

/* Real-time Model object */
extern RT_MODEL_tasks500Hz_T *const tasks500Hz_M;

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
 * '<Root>' : 'tasks500Hz'
 * '<S1>'   : 'tasks500Hz/Compare To Zero'
 * '<S2>'   : 'tasks500Hz/Compare To Zero1'
 * '<S3>'   : 'tasks500Hz/Compare To Zero2'
 * '<S4>'   : 'tasks500Hz/Configurable Subsystem'
 * '<S5>'   : 'tasks500Hz/attitudePIDs'
 * '<S6>'   : 'tasks500Hz/invertQ'
 * '<S7>'   : 'tasks500Hz/invertTheta'
 * '<S8>'   : 'tasks500Hz/invertYaw'
 * '<S9>'   : 'tasks500Hz/positionPIDs'
 * '<S10>'  : 'tasks500Hz/ratePIDs'
 * '<S11>'  : 'tasks500Hz/sampleAndHold2'
 * '<S12>'  : 'tasks500Hz/stdRadianLimit'
 * '<S13>'  : 'tasks500Hz/triYawRate'
 * '<S14>'  : 'tasks500Hz/velocityPIDs'
 * '<S15>'  : 'tasks500Hz/Configurable Subsystem/mahonyAHRS_MC'
 * '<S16>'  : 'tasks500Hz/attitudePIDs/Saturation Dynamic'
 * '<S17>'  : 'tasks500Hz/positionPIDs/Saturation Dynamic'
 * '<S18>'  : 'tasks500Hz/ratePIDs/Saturation Dynamic'
 * '<S19>'  : 'tasks500Hz/stdRadianLimit/Compare To Constant'
 * '<S20>'  : 'tasks500Hz/stdRadianLimit/Compare To Constant1'
 * '<S21>'  : 'tasks500Hz/triYawRate/notTriCopter'
 * '<S22>'  : 'tasks500Hz/triYawRate/triCopter'
 * '<S23>'  : 'tasks500Hz/velocityPIDs/Saturation Dynamic'
 */
#endif                                 /* RTW_HEADER_tasks500Hz_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
