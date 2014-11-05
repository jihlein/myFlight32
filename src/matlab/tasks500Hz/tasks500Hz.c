/*
 * File: tasks500Hz.c
 *
 * Code generated for Simulink model 'tasks500Hz'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Tue Nov 04 18:36:44 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "tasks500Hz.h"

/* Exported block parameters */
real32_T triYawRateDenLP[2] = { 1.0F, -0.666666687F } ;/* Variable: triYawRateDenLP
                                                        * Referenced by: '<S22>/triYawRate'
                                                        */

real32_T triYawRateNumLP[2] = { 0.166666672F, 0.166666672F } ;/* Variable: triYawRateNumLP
                                                               * Referenced by: '<S22>/triYawRate'
                                                               */

/* Constant parameters (auto storage) */
const ConstP_tasks500Hz_T tasks500Hz_ConstP = {
  /* Expression: mixTri
   * Referenced by: '<Root>/Constant1'
   */
  { 1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.666667F, -0.666667F,
    1.33333302F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  /* Expression: mixQuadX
   * Referenced by: '<Root>/Constant2'
   */
  { 1.0F, -1.0F, -1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, -1.0F, -1.0F, 1.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  /* Expression: mixHexX
   * Referenced by: '<Root>/Constant3'
   */
  { 0.866025F, -0.866025F, -0.866025F, -0.866025F, 0.866025F, 0.866025F, 0.0F,
    0.0F, -1.0F, -1.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F, -1.0F,
    1.0F, -1.0F, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 0.0F,
    0.0F },

  /* Expression: mixY6
   * Referenced by: '<Root>/Constant9'
   */
  { 1.0F, -1.0F, 0.0F, 1.0F, -1.0F, 0.0F, 0.0F, 0.0F, -0.666667F, -0.666667F,
    1.33333302F, -0.666667F, -0.666667F, 1.33333302F, 0.0F, 0.0F, -1.0F, 1.0F,
    1.0F, 1.0F, -1.0F, -1.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    0.0F, 0.0F }
};

/* Block states (auto storage) */
DW_tasks500Hz_T tasks500Hz_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_tasks500Hz_T tasks500Hz_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_tasks500Hz_T tasks500Hz_Y;

/* Real-time model */
RT_MODEL_tasks500Hz_T tasks500Hz_M_;
RT_MODEL_tasks500Hz_T *const tasks500Hz_M = &tasks500Hz_M_;

/* Forward declaration for local functions */
static real32_T tasks500Hz_norm(const real32_T x[3]);

/* Function for MATLAB Function: '<S4>/mahonyAHRS_MC' */
static real32_T tasks500Hz_norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.17549435E-38F;
  absxk = (real32_T)fabs(x[0]);
  if (absxk > 1.17549435E-38F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.17549435E-38F;
    y = t * t;
  }

  absxk = (real32_T)fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = (real32_T)fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = ((y * t) * t) + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * ((real32_T)sqrt(y));
}

/* Model step function */
void tasks500Hz_step(void)
{
  /* local block i/o variables */
  boolean_T rtb_LogicalOperator[3];
  boolean_T rtb_LogicalOperator1[3];
  boolean_T rtb_LogicalOperator2[3];
  uint8_T qValidOut;
  real32_T initialRoll;
  real32_T initialPitch;
  real32_T cosRoll;
  real32_T sinRoll;
  real32_T sinPitch;
  real32_T initialHdg;
  real32_T u1;
  real32_T u2;
  real32_T u3;
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  real32_T rtb_MultiportSwitch4[32];
  int32_T i;
  real32_T rtb_Sum3;
  real32_T rtb_Sum2_j;
  real32_T rtb_Switch;
  real32_T rtb_Switch2_k;
  real32_T v_idx_0;
  real32_T v_idx_1;
  real32_T rtb_attitude_idx_2;
  real32_T rtb_attitude_idx_1;
  real32_T rtb_attitude_idx_0;
  real32_T rtb_Switch2_n_idx_2;
  real_T b_idx_0;
  real_T b_idx_1;
  real_T b_idx_2;
  real_T b_idx_3;
  real_T h_idx_3;
  real32_T rtb_qOut_idx_0;
  real32_T rtb_qOut_idx_1;
  real32_T rtb_qOut_idx_2;
  real32_T rtb_qOut_idx_3;
  real32_T rtb_Sum3_idx_0;
  real32_T rtb_Sum3_idx_1;
  real32_T rtb_Gain3_idx_0;
  real32_T rtb_Gain3_idx_1;
  real32_T rtb_Gain3_idx_2;
  real32_T rtb_Sum3_b_idx_0;
  real32_T rtb_Sum3_b_idx_1;
  real32_T rtb_Sum3_b_idx_2;
  real32_T rtb_Sum2_e_idx_0;
  real32_T rtb_Sum2_e_idx_1;
  real32_T rtb_Switch2_idx_0;
  real32_T rtb_Switch2_idx_1;
  real32_T rtb_Switch2_idx_2;
  real32_T rtb_Gain3_a_idx_0;
  real32_T rtb_Gain3_a_idx_1;
  real32_T rtb_Gain3_a_idx_2;
  real32_T rtb_Sum3_bo_idx_0;
  real32_T rtb_Sum3_bo_idx_1;
  real32_T rtb_Sum3_bo_idx_2;
  real32_T rtb_Switch_idx_0;
  real32_T rtb_Switch_idx_1;
  real32_T rtb_Switch2_n_idx_0;
  real32_T rtb_Switch2_n_idx_1;
  boolean_T rtb_Compare_b_idx_0;
  boolean_T rtb_Compare_b_idx_1;
  real32_T rtb_Gain3_h_idx_0;
  real32_T rtb_Gain3_h_idx_1;
  real32_T rtb_Gain3_h_idx_2;
  real32_T rtb_Sum3_h_idx_0;
  real32_T rtb_Sum3_h_idx_1;
  real32_T rtb_Sum3_h_idx_2;
  real32_T rtb_Sum_n_idx_0;
  real32_T rtb_Sum_n_idx_1;
  real32_T rtb_Switch2_h_idx_0;
  real32_T rtb_Switch2_h_idx_1;
  real32_T rtb_Switch2_h_idx_2;
  real32_T rtb_Switch2_f_idx_0;
  real32_T rtb_Switch2_f_idx_1;
  uint8_T tmp;

  /* MATLAB Function: '<S4>/mahonyAHRS_MC' incorporates:
   *  Constant: '<Root>/Constant5'
   *  Constant: '<Root>/Constant6'
   *  Constant: '<Root>/Constant7'
   *  Constant: '<Root>/Constant8'
   *  Inport: '<Root>/accel'
   *  Inport: '<Root>/accelDataUpdate'
   *  Inport: '<Root>/accelOneG'
   *  Inport: '<Root>/accelValid'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/gyro '
   *  Inport: '<Root>/gyroValid'
   *  Inport: '<Root>/mag'
   *  Inport: '<Root>/magDataUpdate'
   *  Inport: '<Root>/magValid'
   *  Inport: '<Root>/magVar'
   */
  /* MATLAB Function 'mahonyAHRS_MC': '<S15>:1' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  MAYHONYAHRS Madgwick's implementation of Mayhony's AHRS algorithm */
  /*  */
  /*    Date          Author          Notes */
  /*    28/09/2011    SOH Madgwick    Initial release */
  /*    25/08/2014    J Ihlein        Integrated into FF32 */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* '<S15>:1:49' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  if ((((((tasks500Hz_DW.ahrsFirstPass != 0) && (tasks500Hz_U.accelValid != 0)) &&
         (tasks500Hz_U.accelDataUpdate != 0)) && (tasks500Hz_U.gyroValid != 0)) &&
       (tasks500Hz_U.magValid != 0)) && (tasks500Hz_U.magDataUpdate != 0)) {
    /* '<S15>:1:54' */
    initialRoll = (real32_T)atan2(-tasks500Hz_U.accel[1], -tasks500Hz_U.accel[2]);

    /* '<S15>:1:55' */
    initialPitch = (real32_T)atan2(tasks500Hz_U.accel[0], -tasks500Hz_U.accel[2]);

    /* '<S15>:1:57' */
    cosRoll = (real32_T)cos(initialRoll);

    /* '<S15>:1:58' */
    sinRoll = (real32_T)sin(initialRoll);

    /* '<S15>:1:59' */
    /* '<S15>:1:60' */
    sinPitch = (real32_T)sin(initialPitch);

    /* '<S15>:1:62' */
    /* '<S15>:1:63' */
    /* '<S15>:1:65' */
    initialHdg = (real32_T)atan2(-((tasks500Hz_U.mag[1] * cosRoll) -
      (tasks500Hz_U.mag[2] * sinRoll)), (((tasks500Hz_U.mag[1] * sinRoll) *
      sinPitch) + (tasks500Hz_U.mag[0] * ((real32_T)cos(initialPitch)))) +
      ((tasks500Hz_U.mag[2] * cosRoll) * sinPitch));

    /* '<S15>:1:67' */
    u1 = (real32_T)sin(initialHdg / 2.0F);

    /* '<S15>:1:68' */
    u2 = (real32_T)sin(initialPitch / 2.0F);

    /* '<S15>:1:69' */
    u3 = (real32_T)sin(initialRoll / 2.0F);

    /* '<S15>:1:70' */
    sinPitch = (real32_T)cos(initialHdg / 2.0F);

    /* '<S15>:1:71' */
    sinRoll = (real32_T)cos(initialPitch / 2.0F);

    /* '<S15>:1:72' */
    cosRoll = (real32_T)cos(initialRoll / 2.0F);

    /* '<S15>:1:74' */
    tasks500Hz_DW.q[0] = ((sinPitch * sinRoll) * cosRoll) + ((u1 * u2) * u3);

    /* '<S15>:1:75' */
    tasks500Hz_DW.q[1] = ((sinPitch * sinRoll) * u3) - ((u1 * u2) * cosRoll);

    /* '<S15>:1:76' */
    tasks500Hz_DW.q[2] = ((sinPitch * u2) * cosRoll) + ((u1 * sinRoll) * u3);

    /* '<S15>:1:77' */
    tasks500Hz_DW.q[3] = ((u1 * sinRoll) * cosRoll) - ((sinPitch * u2) * u3);

    /* '<S15>:1:79' */
    tasks500Hz_DW.q1q1 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[0];

    /* '<S15>:1:80' */
    tasks500Hz_DW.q1q2 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[1];

    /* '<S15>:1:81' */
    tasks500Hz_DW.q1q3 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[2];

    /* '<S15>:1:82' */
    tasks500Hz_DW.q1q4 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[3];

    /* '<S15>:1:83' */
    tasks500Hz_DW.q2q2 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[1];

    /* '<S15>:1:84' */
    tasks500Hz_DW.q2q3 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[2];

    /* '<S15>:1:85' */
    tasks500Hz_DW.q2q4 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[3];

    /* '<S15>:1:86' */
    tasks500Hz_DW.q3q3 = tasks500Hz_DW.q[2] * tasks500Hz_DW.q[2];

    /* '<S15>:1:87' */
    tasks500Hz_DW.q3q4 = tasks500Hz_DW.q[2] * tasks500Hz_DW.q[3];

    /* '<S15>:1:88' */
    tasks500Hz_DW.q4q4 = tasks500Hz_DW.q[3] * tasks500Hz_DW.q[3];

    /* '<S15>:1:90' */
    tasks500Hz_DW.accelConfidenceDecay = 1.0F / ((real32_T)sqrt
      (eepromConfig.accelCutoff));

    /* '<S15>:1:92' */
    tasks500Hz_DW.ahrsFirstPass = 0U;
  }

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  if (!(tasks500Hz_DW.ahrsFirstPass != 0)) {
    /* '<S15>:1:97' */
    if (tasks500Hz_U.accelDataUpdate != 0) {
      /* Normalise accelerometer measurement */
      /* '<S15>:1:100' */
      sinRoll = tasks500Hz_norm(tasks500Hz_U.accel);

      /*  if(accelNorm == 0), return; end   % handle NaN */
      /* '<S15>:1:102' */
      /*  normalise magnitude */
      /* '<S15>:1:104' */
      /* '<S15>:1:105' */
      cosRoll = ((sinRoll / tasks500Hz_U.accelOneG) * 0.1F) + (0.9F *
        tasks500Hz_DW.pastAccelFilter);

      /* '<S15>:1:106' */
      tasks500Hz_DW.pastAccelFilter = cosRoll;

      /* '<S15>:1:107' */
      cosRoll = 1.0F - (((real32_T)sqrt((real32_T)fabs(cosRoll - 1.0F))) *
                        tasks500Hz_DW.accelConfidenceDecay);
      if (cosRoll > 1.0F) {
        /* '<S15>:1:109' */
        /* '<S15>:1:110' */
        cosRoll = 1.0F;
      } else {
        if (cosRoll < 0.0F) {
          /* '<S15>:1:111' */
          /* '<S15>:1:112' */
          cosRoll = 0.0F;
        }
      }

      /*  Estimated direction of gravity and magnetic field */
      /* '<S15>:1:116' */
      v_idx_0 = (tasks500Hz_DW.q2q4 - tasks500Hz_DW.q1q3) * 2.0F;
      v_idx_1 = (tasks500Hz_DW.q1q2 + tasks500Hz_DW.q3q4) * 2.0F;
      initialPitch = ((tasks500Hz_DW.q1q1 - tasks500Hz_DW.q2q2) -
                      tasks500Hz_DW.q3q3) + tasks500Hz_DW.q4q4;

      /*  Error is sum of cross product between estimated direction and measured direction of fields */
      /* '<S15>:1:121' */
      initialHdg = -(tasks500Hz_U.accel[0] / sinRoll);
      u1 = -(tasks500Hz_U.accel[1] / sinRoll);
      u2 = -(tasks500Hz_U.accel[2] / sinRoll);
      tasks500Hz_DW.eAcc[0] = (((u1 * initialPitch) - (u2 * v_idx_1)) *
        eepromConfig.kpAcc) * cosRoll;
      tasks500Hz_DW.eAcc[1] = (((u2 * v_idx_0) - (initialHdg * initialPitch)) *
        eepromConfig.kpAcc) * cosRoll;
      tasks500Hz_DW.eAcc[2] = (((initialHdg * v_idx_1) - (u1 * v_idx_0)) *
        eepromConfig.kpAcc) * cosRoll;
    }

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    if (tasks500Hz_U.magDataUpdate != 0) {
      /*  Normalise magnetometer measurement */
      /*  if(norm(mag) == 0), return; end     % handle NaN */
      /* '<S15>:1:129' */
      cosRoll = tasks500Hz_norm(tasks500Hz_U.mag);
      v_idx_0 = tasks500Hz_U.mag[0] / cosRoll;
      v_idx_1 = tasks500Hz_U.mag[1] / cosRoll;
      initialPitch = tasks500Hz_U.mag[2] / cosRoll;

      /*  normalise magnitude */
      /*  Reference direction of Earth's magnetic field */
      /* '<S15>:1:132' */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* QUATERNCONJ Converts a quaternion to its conjugate */
      /*  */
      /*    qConj = quaternConj(q) */
      /*  */
      /*    Converts a quaternion to its conjugate. */
      /*  */
      /*    Date          Author          Notes */
      /* 	27/09/2011    SOH Madgwick    Initial release */
      /*    25/08/2014    J Ihlein        Integrated into FF32 */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* '<S15>:1:252' */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* QUATERNPROD Calculates the quaternion product */
      /*  */
      /*    ab = quaternProd(a, b) */
      /*  */
      /*    Calculates the quaternion product of quaternion a and b. */
      /*  */
      /*    Date          Author          Notes */
      /* 	27/09/2011    SOH Madgwick    Initial release */
      /*    25/08/2014    J Ihlein        Integrated into FF32 */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* '<S15>:1:269' */
      /* '<S15>:1:271' */
      b_idx_0 = ((0.0F - (v_idx_0 * (-tasks500Hz_DW.q[1]))) - (v_idx_1 *
                  (-tasks500Hz_DW.q[2]))) - (initialPitch * (-tasks500Hz_DW.q[3]));

      /* '<S15>:1:272' */
      b_idx_1 = ((v_idx_0 * tasks500Hz_DW.q[0]) + (v_idx_1 * (-tasks500Hz_DW.q[3])))
        - (initialPitch * (-tasks500Hz_DW.q[2]));

      /* '<S15>:1:273' */
      b_idx_2 = ((0.0F - (v_idx_0 * (-tasks500Hz_DW.q[3]))) + (v_idx_1 *
                  tasks500Hz_DW.q[0])) + (initialPitch * (-tasks500Hz_DW.q[1]));

      /* '<S15>:1:274' */
      b_idx_3 = ((v_idx_0 * (-tasks500Hz_DW.q[2])) - (v_idx_1 *
                  (-tasks500Hz_DW.q[1]))) + (initialPitch * tasks500Hz_DW.q[0]);

      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* QUATERNPROD Calculates the quaternion product */
      /*  */
      /*    ab = quaternProd(a, b) */
      /*  */
      /*    Calculates the quaternion product of quaternion a and b. */
      /*  */
      /*    Date          Author          Notes */
      /* 	27/09/2011    SOH Madgwick    Initial release */
      /*    25/08/2014    J Ihlein        Integrated into FF32 */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* '<S15>:1:269' */
      /* '<S15>:1:271' */
      /* '<S15>:1:272' */
      /* '<S15>:1:273' */
      /* '<S15>:1:274' */
      h_idx_3 = (((tasks500Hz_DW.q[0] * ((real32_T)b_idx_3)) + (tasks500Hz_DW.q
        [1] * ((real32_T)b_idx_2))) - (tasks500Hz_DW.q[2] * ((real32_T)b_idx_1)))
        + (tasks500Hz_DW.q[3] * ((real32_T)b_idx_0));

      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
      /* '<S15>:1:133' */
      scale = 2.2250738585072014E-308;
      absxk = fabs((((tasks500Hz_DW.q[0] * ((real32_T)b_idx_1)) +
                     (tasks500Hz_DW.q[1] * ((real32_T)b_idx_0))) +
                    (tasks500Hz_DW.q[2] * ((real32_T)b_idx_3))) -
                   (tasks500Hz_DW.q[3] * ((real32_T)b_idx_2)));
      if (absxk > 2.2250738585072014E-308) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 2.2250738585072014E-308;
        y = t * t;
      }

      absxk = fabs((((tasks500Hz_DW.q[0] * ((real32_T)b_idx_2)) -
                     (tasks500Hz_DW.q[1] * ((real32_T)b_idx_3))) +
                    (tasks500Hz_DW.q[2] * ((real32_T)b_idx_0))) +
                   (tasks500Hz_DW.q[3] * ((real32_T)b_idx_1)));
      if (absxk > scale) {
        t = scale / absxk;
        y = ((y * t) * t) + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * sqrt(y);

      /* '<S15>:1:135' */
      initialHdg = (((0.5F - tasks500Hz_DW.q3q3) - tasks500Hz_DW.q4q4) *
                    ((real32_T)(2.0 * y))) + (((real32_T)(2.0 * h_idx_3)) *
        (tasks500Hz_DW.q2q4 - tasks500Hz_DW.q1q3));
      u1 = (((real32_T)(2.0 * y)) * (tasks500Hz_DW.q2q3 - tasks500Hz_DW.q1q4)) +
        (((real32_T)(2.0 * h_idx_3)) * (tasks500Hz_DW.q1q2 + tasks500Hz_DW.q3q4));
      u2 = (((0.5F - tasks500Hz_DW.q2q2) - tasks500Hz_DW.q3q3) * ((real32_T)(2.0
              * h_idx_3))) + (((real32_T)(2.0 * y)) * (tasks500Hz_DW.q1q3 +
        tasks500Hz_DW.q2q4));

      /*  Error is sum of cross product between estimated direction and measured direction of fields */
      /* '<S15>:1:140' */
      tasks500Hz_DW.eMag[0] = ((v_idx_1 * u2) - (initialPitch * u1)) *
        eepromConfig.kpMag;
      tasks500Hz_DW.eMag[1] = ((initialPitch * initialHdg) - (v_idx_0 * u2)) *
        eepromConfig.kpMag;
      tasks500Hz_DW.eMag[2] = ((v_idx_0 * u1) - (v_idx_1 * initialHdg)) *
        eepromConfig.kpMag;
    }

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Apply feedback terms */
    /* '<S15>:1:146' */
    v_idx_0 = (tasks500Hz_U.gyro[0] + tasks500Hz_DW.eAcc[0]) +
      tasks500Hz_DW.eMag[0];
    v_idx_1 = (tasks500Hz_U.gyro[1] + tasks500Hz_DW.eAcc[1]) +
      tasks500Hz_DW.eMag[1];
    initialPitch = (tasks500Hz_U.gyro[2] + tasks500Hz_DW.eAcc[2]) +
      tasks500Hz_DW.eMag[2];

    /*  Compute rate of change of quaternion */
    /* '<S15>:1:149' */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /* QUATERNPROD Calculates the quaternion product */
    /*  */
    /*    ab = quaternProd(a, b) */
    /*  */
    /*    Calculates the quaternion product of quaternion a and b. */
    /*  */
    /*    Date          Author          Notes */
    /* 	27/09/2011    SOH Madgwick    Initial release */
    /*    25/08/2014    J Ihlein        Integrated into FF32 */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /* '<S15>:1:269' */
    /* '<S15>:1:271' */
    /* '<S15>:1:272' */
    b_idx_0 = ((tasks500Hz_DW.q[0] * v_idx_0) + (tasks500Hz_DW.q[2] *
                initialPitch)) - (tasks500Hz_DW.q[3] * v_idx_1);

    /* '<S15>:1:273' */
    b_idx_1 = ((tasks500Hz_DW.q[0] * v_idx_1) - (tasks500Hz_DW.q[1] *
                initialPitch)) + (tasks500Hz_DW.q[3] * v_idx_0);

    /* '<S15>:1:274' */
    b_idx_2 = ((tasks500Hz_DW.q[0] * initialPitch) + (tasks500Hz_DW.q[1] *
                v_idx_1)) - (tasks500Hz_DW.q[2] * v_idx_0);

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /*  Integrate to yield quaternion */
    /* '<S15>:1:152' */
    tasks500Hz_DW.q[0] += ((real32_T)((((0.0F - (tasks500Hz_DW.q[1] * v_idx_0))
      - (tasks500Hz_DW.q[2] * v_idx_1)) - (tasks500Hz_DW.q[3] * initialPitch)) *
      0.5)) * tasks500Hz_U.dt;
    tasks500Hz_DW.q[1] += ((real32_T)(0.5 * b_idx_0)) * tasks500Hz_U.dt;
    tasks500Hz_DW.q[2] += ((real32_T)(0.5 * b_idx_1)) * tasks500Hz_U.dt;
    tasks500Hz_DW.q[3] += ((real32_T)(0.5 * b_idx_2)) * tasks500Hz_U.dt;

    /* '<S15>:1:153' */
    sinRoll = 1.17549435E-38F;
    sinPitch = (real32_T)fabs(tasks500Hz_DW.q[0]);
    if (sinPitch > 1.17549435E-38F) {
      cosRoll = 1.0F;
      sinRoll = sinPitch;
    } else {
      initialHdg = sinPitch / 1.17549435E-38F;
      cosRoll = initialHdg * initialHdg;
    }

    sinPitch = (real32_T)fabs(tasks500Hz_DW.q[1]);
    if (sinPitch > sinRoll) {
      initialHdg = sinRoll / sinPitch;
      cosRoll = ((cosRoll * initialHdg) * initialHdg) + 1.0F;
      sinRoll = sinPitch;
    } else {
      initialHdg = sinPitch / sinRoll;
      cosRoll += initialHdg * initialHdg;
    }

    sinPitch = (real32_T)fabs(tasks500Hz_DW.q[2]);
    if (sinPitch > sinRoll) {
      initialHdg = sinRoll / sinPitch;
      cosRoll = ((cosRoll * initialHdg) * initialHdg) + 1.0F;
      sinRoll = sinPitch;
    } else {
      initialHdg = sinPitch / sinRoll;
      cosRoll += initialHdg * initialHdg;
    }

    sinPitch = (real32_T)fabs(tasks500Hz_DW.q[3]);
    if (sinPitch > sinRoll) {
      initialHdg = sinRoll / sinPitch;
      cosRoll = ((cosRoll * initialHdg) * initialHdg) + 1.0F;
      sinRoll = sinPitch;
    } else {
      initialHdg = sinPitch / sinRoll;
      cosRoll += initialHdg * initialHdg;
    }

    cosRoll = sinRoll * ((real32_T)sqrt(cosRoll));
    tasks500Hz_DW.q[0] /= cosRoll;
    tasks500Hz_DW.q[1] /= cosRoll;
    tasks500Hz_DW.q[2] /= cosRoll;
    tasks500Hz_DW.q[3] /= cosRoll;

    /* '<S15>:1:155' */
    rtb_qOut_idx_0 = tasks500Hz_DW.q[0];
    rtb_qOut_idx_1 = tasks500Hz_DW.q[1];
    rtb_qOut_idx_2 = tasks500Hz_DW.q[2];
    rtb_qOut_idx_3 = tasks500Hz_DW.q[3];
    if (tasks500Hz_DW.qValid == 0) {
      /* '<S15>:1:157' */
      /* '<S15>:1:158' */
      i = (int32_T)(tasks500Hz_DW.qValidCount + 1U);
      if (((uint32_T)i) > 255U) {
        i = 255;
      }

      tasks500Hz_DW.qValidCount = (uint8_T)i;
    }

    if ((tasks500Hz_DW.qValidCount >= 20) && (tasks500Hz_DW.qValid == 0)) {
      /* '<S15>:1:161' */
      /* '<S15>:1:162' */
      tasks500Hz_DW.qValid = 1U;
    }

    /* '<S15>:1:165' */
    qValidOut = tasks500Hz_DW.qValid;

    /* '<S15>:1:167' */
    tasks500Hz_DW.q1q1 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[0];

    /* '<S15>:1:168' */
    tasks500Hz_DW.q1q2 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[1];

    /* '<S15>:1:169' */
    tasks500Hz_DW.q1q3 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[2];

    /* '<S15>:1:170' */
    tasks500Hz_DW.q1q4 = tasks500Hz_DW.q[0] * tasks500Hz_DW.q[3];

    /* '<S15>:1:171' */
    tasks500Hz_DW.q2q2 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[1];

    /* '<S15>:1:172' */
    tasks500Hz_DW.q2q3 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[2];

    /* '<S15>:1:173' */
    tasks500Hz_DW.q2q4 = tasks500Hz_DW.q[1] * tasks500Hz_DW.q[3];

    /* '<S15>:1:174' */
    tasks500Hz_DW.q3q3 = tasks500Hz_DW.q[2] * tasks500Hz_DW.q[2];

    /* '<S15>:1:175' */
    tasks500Hz_DW.q3q4 = tasks500Hz_DW.q[2] * tasks500Hz_DW.q[3];

    /* '<S15>:1:176' */
    tasks500Hz_DW.q4q4 = tasks500Hz_DW.q[3] * tasks500Hz_DW.q[3];

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    /* '<S15>:1:180' */
    rtb_attitude_idx_2 = ((real32_T)atan2((tasks500Hz_DW.q2q3 +
      tasks500Hz_DW.q1q4) * 2.0F, ((tasks500Hz_DW.q1q1 + tasks500Hz_DW.q2q2) -
      tasks500Hz_DW.q3q3) - tasks500Hz_DW.q4q4)) + tasks500Hz_U.magVar;

    /* '<S15>:1:182' */
    cosRoll = (tasks500Hz_DW.q2q4 - tasks500Hz_DW.q1q3) * -2.0F;
    if (cosRoll > 1.0F) {
      /* '<S15>:1:184' */
      /* '<S15>:1:185' */
      cosRoll = 1.0F;
    } else {
      if (cosRoll < -1.0F) {
        /* '<S15>:1:186' */
        /* '<S15>:1:187' */
        cosRoll = -1.0F;
      }
    }

    /* '<S15>:1:190' */
    rtb_attitude_idx_1 = ((real32_T)asin(cosRoll)) - eepromConfig.attTrim[1];

    /* '<S15>:1:192' */
    rtb_attitude_idx_0 = ((real32_T)atan2((tasks500Hz_DW.q3q4 +
      tasks500Hz_DW.q1q2) * 2.0F, ((tasks500Hz_DW.q1q1 - tasks500Hz_DW.q2q2) -
      tasks500Hz_DW.q3q3) + tasks500Hz_DW.q4q4)) - eepromConfig.attTrim[0];

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    if (rtb_attitude_idx_0 >= 3.1415926535897931) {
      /* '<S15>:1:196' */
      /* '<S15>:1:197' */
      rtb_attitude_idx_0 -= 6.28318548F;
    } else {
      if (rtb_attitude_idx_0 < -3.1415926535897931) {
        /* '<S15>:1:198' */
        /* '<S15>:1:199' */
        rtb_attitude_idx_0 += 6.28318548F;
      }
    }

    if (rtb_attitude_idx_1 >= 3.1415926535897931) {
      /* '<S15>:1:202' */
      /* '<S15>:1:203' */
      rtb_attitude_idx_1 -= 6.28318548F;
    } else {
      if (rtb_attitude_idx_1 < -3.1415926535897931) {
        /* '<S15>:1:204' */
        /* '<S15>:1:205' */
        rtb_attitude_idx_1 += 6.28318548F;
      }
    }

    if (rtb_attitude_idx_2 >= 3.1415926535897931) {
      /* '<S15>:1:208' */
      /* '<S15>:1:209' */
      rtb_attitude_idx_2 -= 6.28318548F;
    } else {
      if (rtb_attitude_idx_2 < -3.1415926535897931) {
        /* '<S15>:1:210' */
        /* '<S15>:1:211' */
        rtb_attitude_idx_2 += 6.28318548F;
      }
    }

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  } else {
    /* '<S15>:1:216' */
    rtb_qOut_idx_0 = 1.0F;
    rtb_qOut_idx_1 = 0.0F;
    rtb_qOut_idx_2 = 0.0F;
    rtb_qOut_idx_3 = 0.0F;

    /* '<S15>:1:218' */
    qValidOut = 0U;

    /* '<S15>:1:220' */
    tasks500Hz_DW.q1q1 = 1.0F;

    /* '<S15>:1:221' */
    tasks500Hz_DW.q1q2 = 0.0F;

    /* '<S15>:1:222' */
    tasks500Hz_DW.q1q3 = 0.0F;

    /* '<S15>:1:223' */
    tasks500Hz_DW.q1q4 = 0.0F;

    /* '<S15>:1:224' */
    tasks500Hz_DW.q2q2 = 0.0F;

    /* '<S15>:1:225' */
    tasks500Hz_DW.q2q3 = 0.0F;

    /* '<S15>:1:226' */
    tasks500Hz_DW.q2q4 = 0.0F;

    /* '<S15>:1:227' */
    tasks500Hz_DW.q3q3 = 0.0F;

    /* '<S15>:1:228' */
    tasks500Hz_DW.q3q4 = 0.0F;

    /* '<S15>:1:229' */
    tasks500Hz_DW.q4q4 = 0.0F;

    /* '<S15>:1:231' */
    rtb_attitude_idx_2 = 0.0F;

    /* '<S15>:1:232' */
    rtb_attitude_idx_1 = 0.0F;

    /* '<S15>:1:233' */
    rtb_attitude_idx_0 = 0.0F;
  }

  /* Saturate: '<Root>/Saturation1' incorporates:
   *  Inport: '<Root>/posModes'
   */
  if (tasks500Hz_U.posModes[0] <= 1) {
    tmp = tasks500Hz_U.posModes[0];
  } else {
    tmp = 1U;
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch3' incorporates:
   *  Inport: '<Root>/posCmds'
   *  Inport: '<Root>/positions'
   *  Saturate: '<Root>/Saturation1'
   */
  switch (tmp) {
   case 0:
    rtb_Sum3_idx_0 = tasks500Hz_U.positions[0];
    break;

   case 1:
    rtb_Sum3_idx_0 = tasks500Hz_U.posCmds[0];
    break;

   default:
    rtb_Sum3_idx_0 = tasks500Hz_U.posCmds[0];
    break;
  }

  /* Saturate: '<Root>/Saturation1' incorporates:
   *  Inport: '<Root>/posModes'
   */
  if (tasks500Hz_U.posModes[1] <= 1) {
    tmp = tasks500Hz_U.posModes[1];
  } else {
    tmp = 1U;
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch3' incorporates:
   *  Inport: '<Root>/posCmds'
   *  Inport: '<Root>/positions'
   *  Saturate: '<Root>/Saturation1'
   */
  switch (tmp) {
   case 0:
    rtb_Sum3_idx_1 = tasks500Hz_U.positions[1];
    break;

   case 1:
    rtb_Sum3_idx_1 = tasks500Hz_U.posCmds[1];
    break;

   default:
    rtb_Sum3_idx_1 = tasks500Hz_U.posCmds[1];
    break;
  }

  /* Saturate: '<Root>/Saturation1' incorporates:
   *  Inport: '<Root>/posModes'
   */
  if (tasks500Hz_U.posModes[2] <= 1) {
    tmp = tasks500Hz_U.posModes[2];
  } else {
    tmp = 1U;
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch3' incorporates:
   *  Inport: '<Root>/posCmds'
   *  Inport: '<Root>/positions'
   *  Saturate: '<Root>/Saturation1'
   */
  switch (tmp) {
   case 0:
    initialRoll = tasks500Hz_U.positions[2];
    break;

   case 1:
    initialRoll = tasks500Hz_U.posCmds[2];
    break;

   default:
    initialRoll = tasks500Hz_U.posCmds[2];
    break;
  }

  /* Sum: '<Root>/Sum3' incorporates:
   *  Inport: '<Root>/positions'
   */
  rtb_Sum3_idx_0 -= tasks500Hz_U.positions[0];
  rtb_Sum3_idx_1 -= tasks500Hz_U.positions[1];
  rtb_Sum3 = initialRoll - tasks500Hz_U.positions[2];

  /* Logic: '<Root>/Logical Operator' incorporates:
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/positions'
   *  Inport: '<Root>/resetPIDs'
   *  RelationalOperator: '<S1>/Compare'
   */
  rtb_LogicalOperator[0] = ((tasks500Hz_U.positions[0] == 0.0F) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator[1] = ((tasks500Hz_U.positions[1] == 0.0F) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator[2] = ((tasks500Hz_U.positions[2] == 0.0F) ||
    (tasks500Hz_U.resetPIDs != 0));

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  if ((rtb_LogicalOperator[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[0] = 0.0F;
  }

  if ((rtb_LogicalOperator[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[1] = 0.0F;
  }

  if ((rtb_LogicalOperator[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[2] = 0.0F;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator1' */
  if ((rtb_LogicalOperator[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[0] = 0.0F;
  }

  if ((rtb_LogicalOperator[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[1] = 0.0F;
  }

  if ((rtb_LogicalOperator[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[2] = 0.0F;
  }

  /* Gain: '<S9>/Gain3' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator1'
   *  Gain: '<S9>/Gain2'
   *  Sum: '<S9>/Sum1'
   */
  rtb_Gain3_idx_0 = ((eepromConfig.positionD[0] * rtb_Sum3_idx_0) -
                     tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[0]) * 100.0F;
  rtb_Gain3_idx_1 = ((eepromConfig.positionD[1] * rtb_Sum3_idx_1) -
                     tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[1]) * 100.0F;
  rtb_Gain3_idx_2 = ((eepromConfig.positionD[2] * rtb_Sum3) -
                     tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[2]) * 100.0F;

  /* Sum: '<S9>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Gain: '<S9>/Gain'
   */
  rtb_Sum3_b_idx_0 = ((eepromConfig.positionP[0] * rtb_Sum3_idx_0) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[0]) +
    rtb_Gain3_idx_0;
  rtb_Sum3_b_idx_1 = ((eepromConfig.positionP[1] * rtb_Sum3_idx_1) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[1]) +
    rtb_Gain3_idx_1;
  rtb_Sum3_b_idx_2 = ((eepromConfig.positionP[2] * rtb_Sum3) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[2]) +
    rtb_Gain3_idx_2;

  /* Gain: '<S9>/Gain5' incorporates:
   *  Constant: '<S9>/Constant'
   */
  rtb_Switch2_idx_2 = -eepromConfig.positionLimit[0];

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S9>/Constant'
   *  Gain: '<S9>/Gain5'
   *  RelationalOperator: '<S17>/UpperRelop'
   */
  if (!(rtb_Sum3_b_idx_0 < (-eepromConfig.positionLimit[0]))) {
    rtb_Switch2_idx_2 = rtb_Sum3_b_idx_0;
  }

  rtb_Sum2_e_idx_0 = rtb_Switch2_idx_2;

  /* Gain: '<S9>/Gain5' incorporates:
   *  Constant: '<S9>/Constant'
   */
  rtb_Switch2_idx_2 = -eepromConfig.positionLimit[1];

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S9>/Constant'
   *  Gain: '<S9>/Gain5'
   *  RelationalOperator: '<S17>/UpperRelop'
   */
  if (!(rtb_Sum3_b_idx_1 < (-eepromConfig.positionLimit[1]))) {
    rtb_Switch2_idx_2 = rtb_Sum3_b_idx_1;
  }

  rtb_Sum2_e_idx_1 = rtb_Switch2_idx_2;

  /* Gain: '<S9>/Gain5' incorporates:
   *  Constant: '<S9>/Constant'
   */
  rtb_Switch2_idx_2 = -eepromConfig.positionLimit[2];

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S9>/Constant'
   *  Gain: '<S9>/Gain5'
   *  RelationalOperator: '<S17>/UpperRelop'
   */
  if (!(rtb_Sum3_b_idx_2 < (-eepromConfig.positionLimit[2]))) {
    rtb_Switch2_idx_2 = rtb_Sum3_b_idx_2;
  }

  /* Switch: '<S17>/Switch2' incorporates:
   *  Constant: '<S9>/Constant'
   *  RelationalOperator: '<S17>/LowerRelop1'
   */
  if (rtb_Sum3_b_idx_0 > eepromConfig.positionLimit[0]) {
    rtb_Switch2_idx_0 = eepromConfig.positionLimit[0];
  } else {
    rtb_Switch2_idx_0 = rtb_Sum2_e_idx_0;
  }

  if (rtb_Sum3_b_idx_1 > eepromConfig.positionLimit[1]) {
    rtb_Switch2_idx_1 = eepromConfig.positionLimit[1];
  } else {
    rtb_Switch2_idx_1 = rtb_Sum2_e_idx_1;
  }

  if (rtb_Sum3_b_idx_2 > eepromConfig.positionLimit[2]) {
    rtb_Switch2_idx_2 = eepromConfig.positionLimit[2];
  }

  /* End of Switch: '<S17>/Switch2' */

  /* MultiPortSwitch: '<Root>/Multiport Switch2' incorporates:
   *  Inport: '<Root>/velCmds'
   *  Inport: '<Root>/velModes'
   *  Inport: '<Root>/velocities'
   */
  switch (tasks500Hz_U.velModes[0]) {
   case 0:
    rtb_Sum2_e_idx_0 = tasks500Hz_U.velocities[0];
    break;

   case 1:
    rtb_Sum2_e_idx_0 = tasks500Hz_U.velCmds[0];
    break;

   default:
    rtb_Sum2_e_idx_0 = rtb_Switch2_idx_0;
    break;
  }

  switch (tasks500Hz_U.velModes[1]) {
   case 0:
    rtb_Sum2_e_idx_1 = tasks500Hz_U.velocities[1];
    break;

   case 1:
    rtb_Sum2_e_idx_1 = tasks500Hz_U.velCmds[1];
    break;

   default:
    rtb_Sum2_e_idx_1 = rtb_Switch2_idx_1;
    break;
  }

  switch (tasks500Hz_U.velModes[2]) {
   case 0:
    initialRoll = tasks500Hz_U.velocities[2];
    break;

   case 1:
    initialRoll = tasks500Hz_U.velCmds[2];
    break;

   default:
    initialRoll = rtb_Switch2_idx_2;
    break;
  }

  /* End of MultiPortSwitch: '<Root>/Multiport Switch2' */

  /* Sum: '<Root>/Sum2' incorporates:
   *  Inport: '<Root>/velocities'
   */
  rtb_Sum2_e_idx_0 -= tasks500Hz_U.velocities[0];
  rtb_Sum2_e_idx_1 -= tasks500Hz_U.velocities[1];
  rtb_Sum2_j = initialRoll - tasks500Hz_U.velocities[2];

  /* Logic: '<Root>/Logical Operator1' incorporates:
   *  Constant: '<S2>/Constant'
   *  Inport: '<Root>/resetPIDs'
   *  Inport: '<Root>/velModes'
   *  RelationalOperator: '<S2>/Compare'
   */
  rtb_LogicalOperator1[0] = ((tasks500Hz_U.velModes[0] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator1[1] = ((tasks500Hz_U.velModes[1] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator1[2] = ((tasks500Hz_U.velModes[2] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));

  /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  if ((rtb_LogicalOperator1[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[0] = 0.0F;
  }

  if ((rtb_LogicalOperator1[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[1] = 0.0F;
  }

  if ((rtb_LogicalOperator1[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[2] = 0.0F;
  }

  /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
  if ((rtb_LogicalOperator1[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[0] = 0.0F;
  }

  if ((rtb_LogicalOperator1[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[1] = 0.0F;
  }

  if ((rtb_LogicalOperator1[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[2] = 0.0F;
  }

  /* Gain: '<S14>/Gain3' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator1'
   *  Gain: '<S14>/Gain2'
   *  Sum: '<S14>/Sum1'
   */
  rtb_Gain3_a_idx_0 = ((eepromConfig.velocityD[0] * rtb_Sum2_e_idx_0) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[0]) *
    100.0F;
  rtb_Gain3_a_idx_1 = ((eepromConfig.velocityD[1] * rtb_Sum2_e_idx_1) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[1]) *
    100.0F;
  rtb_Gain3_a_idx_2 = ((eepromConfig.velocityD[2] * rtb_Sum2_j) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[2]) *
    100.0F;

  /* Sum: '<S14>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   *  Gain: '<S14>/Gain'
   */
  rtb_Sum3_bo_idx_0 = ((eepromConfig.velocityP[0] * rtb_Sum2_e_idx_0) +
                       tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[0]) +
    rtb_Gain3_a_idx_0;
  rtb_Sum3_bo_idx_1 = ((eepromConfig.velocityP[1] * rtb_Sum2_e_idx_1) +
                       tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[1]) +
    rtb_Gain3_a_idx_1;
  rtb_Sum3_bo_idx_2 = ((eepromConfig.velocityP[2] * rtb_Sum2_j) +
                       tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[2]) +
    rtb_Gain3_a_idx_2;

  /* Gain: '<S14>/Gain5' incorporates:
   *  Constant: '<S14>/Constant'
   */
  rtb_Switch2_n_idx_2 = -eepromConfig.velocityLimit[0];

  /* Switch: '<S23>/Switch' incorporates:
   *  Constant: '<S14>/Constant'
   *  Gain: '<S14>/Gain5'
   *  RelationalOperator: '<S23>/UpperRelop'
   */
  if (!(rtb_Sum3_bo_idx_0 < (-eepromConfig.velocityLimit[0]))) {
    rtb_Switch2_n_idx_2 = rtb_Sum3_bo_idx_0;
  }

  rtb_Switch_idx_0 = rtb_Switch2_n_idx_2;

  /* Gain: '<S14>/Gain5' incorporates:
   *  Constant: '<S14>/Constant'
   */
  rtb_Switch2_n_idx_2 = -eepromConfig.velocityLimit[1];

  /* Switch: '<S23>/Switch' incorporates:
   *  Constant: '<S14>/Constant'
   *  Gain: '<S14>/Gain5'
   *  RelationalOperator: '<S23>/UpperRelop'
   */
  if (!(rtb_Sum3_bo_idx_1 < (-eepromConfig.velocityLimit[1]))) {
    rtb_Switch2_n_idx_2 = rtb_Sum3_bo_idx_1;
  }

  rtb_Switch_idx_1 = rtb_Switch2_n_idx_2;

  /* Gain: '<S14>/Gain5' incorporates:
   *  Constant: '<S14>/Constant'
   */
  rtb_Switch2_n_idx_2 = -eepromConfig.velocityLimit[2];

  /* Switch: '<S23>/Switch' incorporates:
   *  Constant: '<S14>/Constant'
   *  Gain: '<S14>/Gain5'
   *  RelationalOperator: '<S23>/UpperRelop'
   */
  if (!(rtb_Sum3_bo_idx_2 < (-eepromConfig.velocityLimit[2]))) {
    rtb_Switch2_n_idx_2 = rtb_Sum3_bo_idx_2;
  }

  /* Switch: '<S23>/Switch2' incorporates:
   *  Constant: '<S14>/Constant'
   *  RelationalOperator: '<S23>/LowerRelop1'
   */
  if (rtb_Sum3_bo_idx_0 > eepromConfig.velocityLimit[0]) {
    rtb_Switch2_n_idx_0 = eepromConfig.velocityLimit[0];
  } else {
    rtb_Switch2_n_idx_0 = rtb_Switch_idx_0;
  }

  if (rtb_Sum3_bo_idx_1 > eepromConfig.velocityLimit[1]) {
    rtb_Switch2_n_idx_1 = eepromConfig.velocityLimit[1];
  } else {
    rtb_Switch2_n_idx_1 = rtb_Switch_idx_1;
  }

  if (rtb_Sum3_bo_idx_2 > eepromConfig.velocityLimit[2]) {
    rtb_Switch2_n_idx_2 = eepromConfig.velocityLimit[2];
  }

  /* End of Switch: '<S23>/Switch2' */

  /* MultiPortSwitch: '<Root>/Multiport Switch1' incorporates:
   *  Gain: '<S7>/Gain'
   *  Inport: '<Root>/attCmds'
   *  Inport: '<Root>/attModes'
   */
  switch (tasks500Hz_U.attModes[0]) {
   case 0:
    rtb_Switch_idx_0 = rtb_attitude_idx_0;
    break;

   case 1:
    rtb_Switch_idx_0 = tasks500Hz_U.attCmds[0];
    break;

   default:
    rtb_Switch_idx_0 = rtb_Switch2_n_idx_0;
    break;
  }

  switch (tasks500Hz_U.attModes[1]) {
   case 0:
    rtb_Switch_idx_1 = -rtb_attitude_idx_1;
    break;

   case 1:
    rtb_Switch_idx_1 = tasks500Hz_U.attCmds[1];
    break;

   default:
    rtb_Switch_idx_1 = rtb_Switch2_n_idx_1;
    break;
  }

  switch (tasks500Hz_U.attModes[2]) {
   case 0:
    initialRoll = rtb_attitude_idx_2;
    break;

   case 1:
    initialRoll = tasks500Hz_U.attCmds[2];
    break;

   default:
    initialRoll = rtb_Switch2_n_idx_2;
    break;
  }

  /* End of MultiPortSwitch: '<Root>/Multiport Switch1' */

  /* Sum: '<Root>/Sum1' incorporates:
   *  Gain: '<S7>/Gain'
   */
  rtb_Switch_idx_0 -= rtb_attitude_idx_0;
  rtb_Switch_idx_1 -= -rtb_attitude_idx_1;
  initialRoll -= rtb_attitude_idx_2;

  /* Bias: '<S12>/Bias' */
  v_idx_0 = rtb_Switch_idx_0 + -6.28318548F;
  v_idx_1 = rtb_Switch_idx_1 + -6.28318548F;

  /* RelationalOperator: '<S19>/Compare' incorporates:
   *  Constant: '<S19>/Constant'
   */
  rtb_Compare_b_idx_0 = (rtb_Switch_idx_0 >= 3.14159274F);
  rtb_Compare_b_idx_1 = (rtb_Switch_idx_1 >= 3.14159274F);

  /* Switch: '<S12>/Switch1' incorporates:
   *  Bias: '<S12>/Bias1'
   *  Constant: '<S20>/Constant'
   *  RelationalOperator: '<S20>/Compare'
   */
  initialPitch = rtb_Switch_idx_0;
  if (rtb_Switch_idx_0 < -3.14159274F) {
    initialPitch = rtb_Switch_idx_0 + 6.28318548F;
  }

  rtb_Switch_idx_0 = initialPitch;
  initialPitch = rtb_Switch_idx_1;
  if (rtb_Switch_idx_1 < -3.14159274F) {
    initialPitch = rtb_Switch_idx_1 + 6.28318548F;
  }

  rtb_Switch_idx_1 = initialPitch;
  initialPitch = initialRoll;

  /* Switch: '<S12>/Switch1' incorporates:
   *  Bias: '<S12>/Bias1'
   *  Constant: '<S20>/Constant'
   *  RelationalOperator: '<S20>/Compare'
   */
  if (initialRoll < -3.14159274F) {
    initialPitch = initialRoll + 6.28318548F;
  }

  /* Switch: '<S12>/Switch' */
  rtb_Switch = rtb_Switch_idx_0;
  if (rtb_Compare_b_idx_0) {
    rtb_Switch = v_idx_0;
  }

  rtb_Switch_idx_0 = rtb_Switch;
  rtb_Switch = rtb_Switch_idx_1;
  if (rtb_Compare_b_idx_1) {
    rtb_Switch = v_idx_1;
  }

  rtb_Switch_idx_1 = rtb_Switch;
  rtb_Switch = initialPitch;

  /* Switch: '<S12>/Switch' incorporates:
   *  Bias: '<S12>/Bias'
   *  Constant: '<S19>/Constant'
   *  RelationalOperator: '<S19>/Compare'
   */
  if (initialRoll >= 3.14159274F) {
    rtb_Switch = initialRoll + -6.28318548F;
  }

  /* Logic: '<Root>/Logical Operator2' incorporates:
   *  Constant: '<S3>/Constant'
   *  Inport: '<Root>/attModes'
   *  Inport: '<Root>/resetPIDs'
   *  RelationalOperator: '<S3>/Compare'
   */
  rtb_LogicalOperator2[0] = ((tasks500Hz_U.attModes[0] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator2[1] = ((tasks500Hz_U.attModes[1] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));
  rtb_LogicalOperator2[2] = ((tasks500Hz_U.attModes[2] == 0) ||
    (tasks500Hz_U.resetPIDs != 0));

  /* DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  if ((rtb_LogicalOperator2[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[0] = 0.0F;
  }

  if ((rtb_LogicalOperator2[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[1] = 0.0F;
  }

  if ((rtb_LogicalOperator2[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[2] = 0.0F;
  }

  /* DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */
  if ((rtb_LogicalOperator2[0]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[0] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[0] = 0.0F;
  }

  if ((rtb_LogicalOperator2[1]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[1] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[1] = 0.0F;
  }

  if ((rtb_LogicalOperator2[2]) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[2] != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[2] = 0.0F;
  }

  /* Gain: '<S5>/Gain3' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator1'
   *  Gain: '<S5>/Gain2'
   *  Sum: '<S5>/Sum1'
   */
  rtb_Gain3_h_idx_0 = ((eepromConfig.attitudeD[0] * rtb_Switch_idx_0) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[0]) *
    100.0F;
  rtb_Gain3_h_idx_1 = ((eepromConfig.attitudeD[1] * rtb_Switch_idx_1) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[1]) *
    100.0F;
  rtb_Gain3_h_idx_2 = ((eepromConfig.attitudeD[2] * rtb_Switch) -
                       tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[2]) *
    100.0F;

  /* Sum: '<S5>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
   *  Gain: '<S5>/Gain'
   */
  rtb_Sum3_h_idx_0 = ((eepromConfig.attitudeP[0] * rtb_Switch_idx_0) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[0]) +
    rtb_Gain3_h_idx_0;
  rtb_Sum3_h_idx_1 = ((eepromConfig.attitudeP[1] * rtb_Switch_idx_1) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[1]) +
    rtb_Gain3_h_idx_1;
  rtb_Sum3_h_idx_2 = ((eepromConfig.attitudeP[2] * rtb_Switch) +
                      tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[2]) +
    rtb_Gain3_h_idx_2;

  /* Gain: '<S5>/Gain5' incorporates:
   *  Constant: '<S5>/Constant'
   */
  rtb_Switch2_h_idx_2 = -eepromConfig.attitudeLimit[0];

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S5>/Constant'
   *  Gain: '<S5>/Gain5'
   *  RelationalOperator: '<S16>/UpperRelop'
   */
  if (!(rtb_Sum3_h_idx_0 < (-eepromConfig.attitudeLimit[0]))) {
    rtb_Switch2_h_idx_2 = rtb_Sum3_h_idx_0;
  }

  rtb_Sum_n_idx_0 = rtb_Switch2_h_idx_2;

  /* Gain: '<S5>/Gain5' incorporates:
   *  Constant: '<S5>/Constant'
   */
  rtb_Switch2_h_idx_2 = -eepromConfig.attitudeLimit[1];

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S5>/Constant'
   *  Gain: '<S5>/Gain5'
   *  RelationalOperator: '<S16>/UpperRelop'
   */
  if (!(rtb_Sum3_h_idx_1 < (-eepromConfig.attitudeLimit[1]))) {
    rtb_Switch2_h_idx_2 = rtb_Sum3_h_idx_1;
  }

  rtb_Sum_n_idx_1 = rtb_Switch2_h_idx_2;

  /* Gain: '<S5>/Gain5' incorporates:
   *  Constant: '<S5>/Constant'
   */
  rtb_Switch2_h_idx_2 = -eepromConfig.attitudeLimit[2];

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S5>/Constant'
   *  Gain: '<S5>/Gain5'
   *  RelationalOperator: '<S16>/UpperRelop'
   */
  if (!(rtb_Sum3_h_idx_2 < (-eepromConfig.attitudeLimit[2]))) {
    rtb_Switch2_h_idx_2 = rtb_Sum3_h_idx_2;
  }

  /* Switch: '<S16>/Switch2' incorporates:
   *  Constant: '<S5>/Constant'
   *  RelationalOperator: '<S16>/LowerRelop1'
   */
  if (rtb_Sum3_h_idx_0 > eepromConfig.attitudeLimit[0]) {
    rtb_Switch2_h_idx_0 = eepromConfig.attitudeLimit[0];
  } else {
    rtb_Switch2_h_idx_0 = rtb_Sum_n_idx_0;
  }

  if (rtb_Sum3_h_idx_1 > eepromConfig.attitudeLimit[1]) {
    rtb_Switch2_h_idx_1 = eepromConfig.attitudeLimit[1];
  } else {
    rtb_Switch2_h_idx_1 = rtb_Sum_n_idx_1;
  }

  if (rtb_Sum3_h_idx_2 > eepromConfig.attitudeLimit[2]) {
    rtb_Switch2_h_idx_2 = eepromConfig.attitudeLimit[2];
  }

  /* End of Switch: '<S16>/Switch2' */

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/rateModes'
   */
  if (tasks500Hz_U.rateModes[0] > 2) {
    tmp = 2U;
  } else if (tasks500Hz_U.rateModes[0] < 1) {
    tmp = 1U;
  } else {
    tmp = tasks500Hz_U.rateModes[0];
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
   *  Inport: '<Root>/gyro '
   *  Inport: '<Root>/rateCmds'
   *  Saturate: '<Root>/Saturation'
   */
  switch (tmp) {
   case 0:
    rtb_Sum_n_idx_0 = tasks500Hz_U.gyro[0];
    break;

   case 1:
    rtb_Sum_n_idx_0 = tasks500Hz_U.rateCmds[0];
    break;

   default:
    rtb_Sum_n_idx_0 = rtb_Switch2_h_idx_0;
    break;
  }

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/rateModes'
   */
  if (tasks500Hz_U.rateModes[1] > 2) {
    tmp = 2U;
  } else if (tasks500Hz_U.rateModes[1] < 1) {
    tmp = 1U;
  } else {
    tmp = tasks500Hz_U.rateModes[1];
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
   *  Gain: '<S6>/Gain'
   *  Inport: '<Root>/gyro '
   *  Inport: '<Root>/rateCmds'
   *  Saturate: '<Root>/Saturation'
   */
  switch (tmp) {
   case 0:
    rtb_Sum_n_idx_1 = -tasks500Hz_U.gyro[1];
    break;

   case 1:
    rtb_Sum_n_idx_1 = tasks500Hz_U.rateCmds[1];
    break;

   default:
    rtb_Sum_n_idx_1 = rtb_Switch2_h_idx_1;
    break;
  }

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/rateModes'
   */
  if (tasks500Hz_U.rateModes[2] > 2) {
    tmp = 2U;
  } else if (tasks500Hz_U.rateModes[2] < 1) {
    tmp = 1U;
  } else {
    tmp = tasks500Hz_U.rateModes[2];
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
   *  Inport: '<Root>/gyro '
   *  Inport: '<Root>/rateCmds'
   *  Saturate: '<Root>/Saturation'
   */
  switch (tmp) {
   case 0:
    initialRoll = tasks500Hz_U.gyro[2];
    break;

   case 1:
    initialRoll = tasks500Hz_U.rateCmds[2];
    break;

   default:
    initialRoll = rtb_Switch2_h_idx_2;
    break;
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Gain: '<S6>/Gain'
   *  Inport: '<Root>/gyro '
   */
  rtb_Sum_n_idx_0 -= tasks500Hz_U.gyro[0];
  rtb_Sum_n_idx_1 -= -tasks500Hz_U.gyro[1];
  u3 = initialRoll - tasks500Hz_U.gyro[2];

  /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/resetPIDs'
   */
  if ((tasks500Hz_U.resetPIDs != 0) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_h != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[0] = 0.0F;
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[1] = 0.0F;
    tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[2] = 0.0F;
  }

  /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/resetPIDs'
   */
  if ((tasks500Hz_U.resetPIDs != 0) ||
      (tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_p != 0)) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[0] = 0.0F;
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[1] = 0.0F;
    tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[2] = 0.0F;
  }

  /* Gain: '<S10>/Gain3' incorporates:
   *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator1'
   *  Gain: '<S10>/Gain2'
   *  Sum: '<S10>/Sum1'
   */
  v_idx_0 = ((eepromConfig.rateD[0] * rtb_Sum_n_idx_0) -
             tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[0]) * 100.0F;
  v_idx_1 = ((eepromConfig.rateD[1] * rtb_Sum_n_idx_1) -
             tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[1]) * 100.0F;
  initialPitch = ((eepromConfig.rateD[2] * u3) -
                  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[2]) * 100.0F;

  /* Sum: '<S10>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator'
   *  Gain: '<S10>/Gain'
   */
  initialHdg = ((eepromConfig.rateP[0] * rtb_Sum_n_idx_0) +
                tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[0]) + v_idx_0;
  u1 = ((eepromConfig.rateP[1] * rtb_Sum_n_idx_1) +
        tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[1]) + v_idx_1;
  u2 = ((eepromConfig.rateP[2] * u3) +
        tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[2]) + initialPitch;

  /* Gain: '<S10>/Gain5' incorporates:
   *  Constant: '<S10>/Constant'
   */
  initialRoll = -eepromConfig.rateLimit[0];

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  Gain: '<S10>/Gain5'
   *  RelationalOperator: '<S18>/UpperRelop'
   */
  if (!(initialHdg < (-eepromConfig.rateLimit[0]))) {
    initialRoll = initialHdg;
  }

  rtb_Switch2_f_idx_0 = initialRoll;

  /* Gain: '<S10>/Gain5' incorporates:
   *  Constant: '<S10>/Constant'
   */
  initialRoll = -eepromConfig.rateLimit[1];

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  Gain: '<S10>/Gain5'
   *  RelationalOperator: '<S18>/UpperRelop'
   */
  if (!(u1 < (-eepromConfig.rateLimit[1]))) {
    initialRoll = u1;
  }

  rtb_Switch2_f_idx_1 = initialRoll;

  /* Gain: '<S10>/Gain5' incorporates:
   *  Constant: '<S10>/Constant'
   */
  initialRoll = -eepromConfig.rateLimit[2];

  /* Switch: '<S18>/Switch' incorporates:
   *  Constant: '<S10>/Constant'
   *  Gain: '<S10>/Gain5'
   *  RelationalOperator: '<S18>/UpperRelop'
   */
  if (!(u2 < (-eepromConfig.rateLimit[2]))) {
    initialRoll = u2;
  }

  /* Switch: '<S18>/Switch2' incorporates:
   *  Constant: '<S10>/Constant'
   *  RelationalOperator: '<S18>/LowerRelop1'
   */
  rtb_Switch2_k = rtb_Switch2_f_idx_0;
  if (initialHdg > eepromConfig.rateLimit[0]) {
    rtb_Switch2_k = eepromConfig.rateLimit[0];
  }

  rtb_Switch2_f_idx_0 = rtb_Switch2_k;
  rtb_Switch2_k = rtb_Switch2_f_idx_1;
  if (u1 > eepromConfig.rateLimit[1]) {
    rtb_Switch2_k = eepromConfig.rateLimit[1];
  }

  rtb_Switch2_f_idx_1 = rtb_Switch2_k;
  rtb_Switch2_k = initialRoll;

  /* Switch: '<S18>/Switch2' incorporates:
   *  Constant: '<S10>/Constant'
   *  RelationalOperator: '<S18>/LowerRelop1'
   */
  if (u2 > eepromConfig.rateLimit[2]) {
    rtb_Switch2_k = eepromConfig.rateLimit[2];
  }

  /* Gain: '<S8>/Gain1' */
  cosRoll = eepromConfig.yawDirection * rtb_Switch2_k;

  /* Switch: '<S11>/u2~=0' incorporates:
   *  Inport: '<Root>/velCmds'
   *  Inport: '<Root>/velModes'
   *  UnitDelay: '<S11>/Unit Delay'
   */
  if (tasks500Hz_U.velModes[2] != 0) {
    sinPitch = tasks500Hz_DW.UnitDelay_DSTATE;
  } else {
    sinPitch = tasks500Hz_U.velCmds[2];
  }

  /* End of Switch: '<S11>/u2~=0' */

  /* Sum: '<Root>/Sum4' */
  initialRoll = rtb_Switch2_n_idx_2 + sinPitch;

  /* Saturate: '<Root>/Saturation2' */
  if (initialRoll > 4000.0F) {
    /* SignalConversion: '<Root>/TmpSignal ConversionAtProductInport2' */
    initialRoll = 4000.0F;
  } else {
    if (initialRoll < 2000.0F) {
      /* SignalConversion: '<Root>/TmpSignal ConversionAtProductInport2' */
      initialRoll = 2000.0F;
    }
  }

  /* End of Saturate: '<Root>/Saturation2' */

  /* Outport: '<Root>/axisCmds' incorporates:
   *  SignalConversion: '<Root>/TmpSignal ConversionAtProductInport2'
   */
  tasks500Hz_Y.axisCmds[0] = rtb_Switch2_f_idx_0;
  tasks500Hz_Y.axisCmds[1] = rtb_Switch2_f_idx_1;
  tasks500Hz_Y.axisCmds[2] = cosRoll;
  tasks500Hz_Y.axisCmds[3] = initialRoll;

  /* MultiPortSwitch: '<Root>/Multiport Switch4' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/Constant1'
   *  Constant: '<Root>/Constant2'
   *  Constant: '<Root>/Constant3'
   *  Constant: '<Root>/Constant4'
   *  Constant: '<Root>/Constant9'
   */
  switch (eepromConfig.mixerConfiguration) {
   case 0:
    memcpy(&rtb_MultiportSwitch4[0], &tasks500Hz_ConstP.Constant1_Value[0],
           (sizeof(real32_T)) << 5U);
    break;

   case 1:
    memcpy(&rtb_MultiportSwitch4[0], &tasks500Hz_ConstP.Constant2_Value[0],
           (sizeof(real32_T)) << 5U);
    break;

   case 2:
    memcpy(&rtb_MultiportSwitch4[0], &tasks500Hz_ConstP.Constant3_Value[0],
           (sizeof(real32_T)) << 5U);
    break;

   case 3:
    memcpy(&rtb_MultiportSwitch4[0], &tasks500Hz_ConstP.Constant9_Value[0],
           (sizeof(real32_T)) << 5U);
    break;

   default:
    for (i = 0; i < 32; i++) {
      rtb_MultiportSwitch4[i] = eepromConfig.freeMix[i];
    }
    break;
  }

  /* End of MultiPortSwitch: '<Root>/Multiport Switch4' */

  /* Outport: '<Root>/motorCmds' incorporates:
   *  Product: '<Root>/Product'
   *  SignalConversion: '<Root>/TmpSignal ConversionAtProductInport2'
   */
  for (i = 0; i < 8; i++) {
    tasks500Hz_Y.motorCmds[i] = 0.0F;
    tasks500Hz_Y.motorCmds[i] += rtb_MultiportSwitch4[i] * rtb_Switch2_f_idx_0;
    tasks500Hz_Y.motorCmds[i] += rtb_MultiportSwitch4[i + 8] *
      rtb_Switch2_f_idx_1;
    tasks500Hz_Y.motorCmds[i] += rtb_MultiportSwitch4[i + 16] * cosRoll;
    tasks500Hz_Y.motorCmds[i] += rtb_MultiportSwitch4[i + 24] * initialRoll;
  }

  /* End of Outport: '<Root>/motorCmds' */

  /* If: '<S13>/If' incorporates:
   *  Constant: '<S13>/Constant1'
   */
  if (eepromConfig.mixerConfiguration == 0) {
    /* Outputs for IfAction SubSystem: '<S13>/triCopter' incorporates:
     *  ActionPort: '<S22>/Action Port'
     */
    /* Sum: '<S22>/Sum5' incorporates:
     *  Constant: '<S22>/Constant5'
     *  Gain: '<S22>/Gain1'
     */
    cosRoll = (-cosRoll) + eepromConfig.triYawServoMid;

    /* DiscreteTransferFcn: '<S22>/triYawRate' */
    sinRoll = (cosRoll - (triYawRateDenLP[1] * tasks500Hz_DW.triYawRate_states))
      / triYawRateDenLP[0];
    initialRoll = (triYawRateNumLP[0] * sinRoll) + (triYawRateNumLP[1] *
      tasks500Hz_DW.triYawRate_states);

    /* Saturate: '<S22>/Saturation3' */
    if (initialRoll > eepromConfig.triYawServoMax) {
      /* Outport: '<Root>/triCopterServoCmd' */
      tasks500Hz_Y.triCopterServoCmd = eepromConfig.triYawServoMax;
    } else if (initialRoll < eepromConfig.triYawServoMin) {
      /* Outport: '<Root>/triCopterServoCmd' */
      tasks500Hz_Y.triCopterServoCmd = eepromConfig.triYawServoMin;
    } else {
      /* Outport: '<Root>/triCopterServoCmd' */
      tasks500Hz_Y.triCopterServoCmd = initialRoll;
    }

    /* End of Saturate: '<S22>/Saturation3' */

    /* Update for DiscreteTransferFcn: '<S22>/triYawRate' */
    tasks500Hz_DW.triYawRate_states = sinRoll;

    /* End of Outputs for SubSystem: '<S13>/triCopter' */
  } else {
    /* Outputs for IfAction SubSystem: '<S13>/notTriCopter' incorporates:
     *  ActionPort: '<S21>/Action Port'
     */
    /* Outport: '<Root>/triCopterServoCmd' incorporates:
     *  Constant: '<S13>/Constant5'
     *  Inport: '<S21>/x'
     */
    tasks500Hz_Y.triCopterServoCmd = eepromConfig.triYawServoMid;

    /* End of Outputs for SubSystem: '<S13>/notTriCopter' */
  }

  /* End of If: '<S13>/If' */

  /* Outport: '<Root>/q' */
  tasks500Hz_Y.q[0] = rtb_qOut_idx_0;
  tasks500Hz_Y.q[1] = rtb_qOut_idx_1;
  tasks500Hz_Y.q[2] = rtb_qOut_idx_2;
  tasks500Hz_Y.q[3] = rtb_qOut_idx_3;

  /* Outport: '<Root>/attValid' incorporates:
   *  MATLAB Function: '<S4>/mahonyAHRS_MC'
   */
  tasks500Hz_Y.attValid = qValidOut;

  /* Outport: '<Root>/attitudes' */
  tasks500Hz_Y.attitudes[0] = rtb_attitude_idx_0;
  tasks500Hz_Y.attitudes[1] = rtb_attitude_idx_1;
  tasks500Hz_Y.attitudes[2] = rtb_attitude_idx_2;

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S9>/Gain1'
   *  Gain: '<S9>/Gain4'
   *  Sum: '<S9>/Sum'
   *  Sum: '<S9>/Sum2'
   */
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[0] += (((rtb_Switch2_idx_0 -
    rtb_Sum3_b_idx_0) * 100.0F) + (eepromConfig.positionI[0] * rtb_Sum3_idx_0)) *
    0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[1] += (((rtb_Switch2_idx_1 -
    rtb_Sum3_b_idx_1) * 100.0F) + (eepromConfig.positionI[1] * rtb_Sum3_idx_1)) *
    0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE[2] += (((rtb_Switch2_idx_2 -
    rtb_Sum3_b_idx_2) * 100.0F) + (eepromConfig.positionI[2] * rtb_Sum3)) *
    0.002F;
  if (rtb_LogicalOperator[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[0] = 0;
  }

  if (rtb_LogicalOperator[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[1] = 0;
  }

  if (rtb_LogicalOperator[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRese[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator1' */
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[0] += 0.002F * rtb_Gain3_idx_0;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[1] += 0.002F * rtb_Gain3_idx_1;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTATE[2] += 0.002F * rtb_Gain3_idx_2;
  if (rtb_LogicalOperator[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[0] = 0;
  }

  if (rtb_LogicalOperator[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[1] = 0;
  }

  if (rtb_LogicalOperator[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevRes[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator1' */

  /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S14>/Gain1'
   *  Gain: '<S14>/Gain4'
   *  Sum: '<S14>/Sum'
   *  Sum: '<S14>/Sum2'
   */
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[0] += (((rtb_Switch2_n_idx_0 -
    rtb_Sum3_bo_idx_0) * 100.0F) + (eepromConfig.velocityI[0] * rtb_Sum2_e_idx_0))
    * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[1] += (((rtb_Switch2_n_idx_1 -
    rtb_Sum3_bo_idx_1) * 100.0F) + (eepromConfig.velocityI[1] * rtb_Sum2_e_idx_1))
    * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_f[2] += (((rtb_Switch2_n_idx_2 -
    rtb_Sum3_bo_idx_2) * 100.0F) + (eepromConfig.velocityI[2] * rtb_Sum2_j)) *
    0.002F;
  if (rtb_LogicalOperator1[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[0] = 0;
  }

  if (rtb_LogicalOperator1[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[1] = 0;
  }

  if (rtb_LogicalOperator1[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_c[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[0] += 0.002F * rtb_Gain3_a_idx_0;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[1] += 0.002F * rtb_Gain3_a_idx_1;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_f[2] += 0.002F * rtb_Gain3_a_idx_2;
  if (rtb_LogicalOperator1[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[0] = 0;
  }

  if (rtb_LogicalOperator1[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[1] = 0;
  }

  if (rtb_LogicalOperator1[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_c[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */

  /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S5>/Gain1'
   *  Gain: '<S5>/Gain4'
   *  Sum: '<S5>/Sum'
   *  Sum: '<S5>/Sum2'
   */
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[0] += (((rtb_Switch2_h_idx_0 -
    rtb_Sum3_h_idx_0) * 100.0F) + (eepromConfig.attitudeI[0] * rtb_Switch_idx_0))
    * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[1] += (((rtb_Switch2_h_idx_1 -
    rtb_Sum3_h_idx_1) * 100.0F) + (eepromConfig.attitudeI[1] * rtb_Switch_idx_1))
    * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_k[2] += (((rtb_Switch2_h_idx_2 -
    rtb_Sum3_h_idx_2) * 100.0F) + (eepromConfig.attitudeI[2] * rtb_Switch)) *
    0.002F;
  if (rtb_LogicalOperator2[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[0] = 0;
  }

  if (rtb_LogicalOperator2[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[1] = 0;
  }

  if (rtb_LogicalOperator2[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_a[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[0] += 0.002F * rtb_Gain3_h_idx_0;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[1] += 0.002F * rtb_Gain3_h_idx_1;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_i[2] += 0.002F * rtb_Gain3_h_idx_2;
  if (rtb_LogicalOperator2[0]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[0] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[0] = 0;
  }

  if (rtb_LogicalOperator2[1]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[1] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[1] = 0;
  }

  if (rtb_LogicalOperator2[2]) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[2] = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_a[2] = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator1' */

  /* Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S10>/Gain1'
   *  Gain: '<S10>/Gain4'
   *  Inport: '<Root>/resetPIDs'
   *  Sum: '<S10>/Sum'
   *  Sum: '<S10>/Sum2'
   */
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[0] += (((rtb_Switch2_f_idx_0 -
    initialHdg) * 100.0F) + (eepromConfig.rateI[0] * rtb_Sum_n_idx_0)) * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[1] += (((rtb_Switch2_f_idx_1 -
    u1) * 100.0F) + (eepromConfig.rateI[1] * rtb_Sum_n_idx_1)) * 0.002F;
  tasks500Hz_DW.DiscreteTimeIntegrator_DSTATE_m[2] += (((rtb_Switch2_k - u2) *
    100.0F) + (eepromConfig.rateI[2] * u3)) * 0.002F;
  if (tasks500Hz_U.resetPIDs > 0) {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_h = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator_PrevRe_h = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/resetPIDs'
   */
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[0] += 0.002F * v_idx_0;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[1] += 0.002F * v_idx_1;
  tasks500Hz_DW.DiscreteTimeIntegrator1_DSTAT_k[2] += 0.002F * initialPitch;
  if (tasks500Hz_U.resetPIDs > 0) {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_p = 1;
  } else {
    tasks500Hz_DW.DiscreteTimeIntegrator1_PrevR_p = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' */

  /* Update for UnitDelay: '<S11>/Unit Delay' */
  tasks500Hz_DW.UnitDelay_DSTATE = sinPitch;
}

/* Model initialize function */
void tasks500Hz_initialize(void)
{
  /* InitializeConditions for MATLAB Function: '<S4>/mahonyAHRS_MC' */
  tasks500Hz_DW.q[0] = 1.0F;
  tasks500Hz_DW.q[1] = 0.0F;
  tasks500Hz_DW.q[2] = 0.0F;
  tasks500Hz_DW.q[3] = 0.0F;
  tasks500Hz_DW.ahrsFirstPass = 1U;
  tasks500Hz_DW.pastAccelFilter = 1.0F;
  tasks500Hz_DW.q1q1 = 1.0F;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
