/*
 * File: tasks100Hz.c
 *
 * Code generated for Simulink model 'tasks100Hz'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
 * C/C++ source code generated on : Fri Dec 19 11:01:21 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->STM32F4xx 32-bit Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "tasks100Hz.h"

/* Exported block parameters */
real32_T earthAccelDenHP[2] = { 1.0F, -0.997503102F } ;/* Variable: earthAccelDenHP
                                                        * Referenced by: '<S3>/earthAccelHP'
                                                        */

real32_T earthAccelNumHP[2] = { 0.998751581F, -0.998751581F } ;/* Variable: earthAccelNumHP
                                                                * Referenced by: '<S3>/earthAccelHP'
                                                                */

/* Block states (auto storage) */
DW_tasks100Hz_T tasks100Hz_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_tasks100Hz_T tasks100Hz_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_tasks100Hz_T tasks100Hz_Y;

/* Real-time model */
RT_MODEL_tasks100Hz_T tasks100Hz_M_;
RT_MODEL_tasks100Hz_T *const tasks100Hz_M = &tasks100Hz_M_;

/* Model step function */
void tasks100Hz_step(void)
{
  /* local block i/o variables */
  boolean_T rtb_FixPtRelationalOperator;
  boolean_T rtb_FixPtRelationalOperator_d;
  boolean_T rtb_FixPtRelationalOperator_f;
  real32_T rtb_Product3_d;
  real32_T rtb_DiscreteTimeIntegrator2;
  real32_T rtb_DiscreteTimeIntegrator3;
  real32_T rtb_DiscreteTimeIntegrator3_f;
  real32_T rtb_Product1_c1;
  real32_T rtb_Product2_i;
  real32_T rtb_Product3_b3;
  real32_T rtb_Sum2;
  boolean_T rtb_Compare_dq;
  real32_T rtb_Gain1_e_idx_0;
  real32_T rtb_Gain1_e_idx_1;
  real32_T rtb_Gain1_e_idx_2;
  real32_T rtb_earthAccelHP_idx_0;
  real32_T rtb_earthAccelHP_idx_1;
  real32_T rtb_earthAccelHP_idx_2;

  /* Outputs for Atomic SubSystem: '<Root>/accelRotation' */
  /* Sqrt: '<S28>/sqrt' incorporates:
   *  Inport: '<Root>/q'
   *  Product: '<S29>/Product'
   *  Product: '<S29>/Product1'
   *  Product: '<S29>/Product2'
   *  Product: '<S29>/Product3'
   *  Sum: '<S29>/Sum'
   *  UnaryMinus: '<S22>/Unary Minus'
   *  UnaryMinus: '<S22>/Unary Minus1'
   *  UnaryMinus: '<S22>/Unary Minus2'
   */
  rtb_Sum2 = (real32_T)sqrt((((tasks100Hz_U.q[0] * tasks100Hz_U.q[0]) +
    ((-tasks100Hz_U.q[1]) * (-tasks100Hz_U.q[1]))) + ((-tasks100Hz_U.q[2]) *
    (-tasks100Hz_U.q[2]))) + ((-tasks100Hz_U.q[3]) * (-tasks100Hz_U.q[3])));

  /* Product: '<S24>/Product2' incorporates:
   *  Inport: '<Root>/q'
   *  UnaryMinus: '<S22>/Unary Minus1'
   */
  rtb_Product2_i = (-tasks100Hz_U.q[2]) / rtb_Sum2;

  /* Product: '<S24>/Product3' incorporates:
   *  Inport: '<Root>/q'
   *  UnaryMinus: '<S22>/Unary Minus2'
   */
  rtb_Product3_b3 = (-tasks100Hz_U.q[3]) / rtb_Sum2;

  /* Product: '<S24>/Product1' incorporates:
   *  Inport: '<Root>/q'
   *  UnaryMinus: '<S22>/Unary Minus'
   */
  rtb_Product1_c1 = (-tasks100Hz_U.q[1]) / rtb_Sum2;

  /* Product: '<S24>/Product' incorporates:
   *  Inport: '<Root>/q'
   */
  rtb_Sum2 = tasks100Hz_U.q[0] / rtb_Sum2;

  /* Gain: '<S3>/Gain1' incorporates:
   *  Constant: '<S25>/Constant1'
   *  Constant: '<S26>/Constant1'
   *  Constant: '<S27>/Constant1'
   *  Gain: '<S25>/Gain'
   *  Gain: '<S25>/Gain1'
   *  Gain: '<S25>/Gain2'
   *  Gain: '<S26>/Gain'
   *  Gain: '<S26>/Gain1'
   *  Gain: '<S26>/Gain2'
   *  Gain: '<S27>/Gain'
   *  Gain: '<S27>/Gain1'
   *  Gain: '<S27>/Gain2'
   *  Inport: '<Root>/accel'
   *  Inport: '<Root>/accelOneG'
   *  Product: '<S25>/Product10'
   *  Product: '<S25>/Product2'
   *  Product: '<S25>/Product3'
   *  Product: '<S25>/Product4'
   *  Product: '<S25>/Product5'
   *  Product: '<S25>/Product6'
   *  Product: '<S25>/Product7'
   *  Product: '<S25>/Product8'
   *  Product: '<S25>/Product9'
   *  Product: '<S26>/Product10'
   *  Product: '<S26>/Product2'
   *  Product: '<S26>/Product3'
   *  Product: '<S26>/Product4'
   *  Product: '<S26>/Product5'
   *  Product: '<S26>/Product6'
   *  Product: '<S26>/Product7'
   *  Product: '<S26>/Product8'
   *  Product: '<S26>/Product9'
   *  Product: '<S27>/Product10'
   *  Product: '<S27>/Product2'
   *  Product: '<S27>/Product3'
   *  Product: '<S27>/Product4'
   *  Product: '<S27>/Product5'
   *  Product: '<S27>/Product6'
   *  Product: '<S27>/Product7'
   *  Product: '<S27>/Product8'
   *  Product: '<S27>/Product9'
   *  Sum: '<S25>/Sum4'
   *  Sum: '<S25>/Sum5'
   *  Sum: '<S25>/Sum6'
   *  Sum: '<S25>/Sum7'
   *  Sum: '<S26>/Sum4'
   *  Sum: '<S26>/Sum5'
   *  Sum: '<S26>/Sum6'
   *  Sum: '<S26>/Sum7'
   *  Sum: '<S27>/Sum4'
   *  Sum: '<S27>/Sum5'
   *  Sum: '<S27>/Sum6'
   *  Sum: '<S27>/Sum7'
   *  Sum: '<S3>/Sum2'
   */
  rtb_Gain1_e_idx_0 = (((((0.5F - (rtb_Product2_i * rtb_Product2_i)) -
    (rtb_Product3_b3 * rtb_Product3_b3)) * 2.0F) * tasks100Hz_U.accel[0]) +
                       ((((rtb_Product1_c1 * rtb_Product2_i) + (rtb_Sum2 *
    rtb_Product3_b3)) * 2.0F) * tasks100Hz_U.accel[1])) + ((((rtb_Product1_c1 *
    rtb_Product3_b3) - (rtb_Sum2 * rtb_Product2_i)) * 2.0F) *
    tasks100Hz_U.accel[2]);
  rtb_Gain1_e_idx_1 = (((((0.5F - (rtb_Product1_c1 * rtb_Product1_c1)) -
    (rtb_Product3_b3 * rtb_Product3_b3)) * 2.0F) * tasks100Hz_U.accel[1]) +
                       ((((rtb_Product1_c1 * rtb_Product2_i) - (rtb_Sum2 *
    rtb_Product3_b3)) * 2.0F) * tasks100Hz_U.accel[0])) + ((((rtb_Sum2 *
    rtb_Product1_c1) + (rtb_Product2_i * rtb_Product3_b3)) * 2.0F) *
    tasks100Hz_U.accel[2]);
  rtb_Gain1_e_idx_2 = -(((((((rtb_Product1_c1 * rtb_Product3_b3) + (rtb_Sum2 *
    rtb_Product2_i)) * 2.0F) * tasks100Hz_U.accel[0]) + ((((rtb_Product2_i *
    rtb_Product3_b3) - (rtb_Sum2 * rtb_Product1_c1)) * 2.0F) *
    tasks100Hz_U.accel[1])) + ((((0.5F - (rtb_Product1_c1 * rtb_Product1_c1)) -
                            (rtb_Product2_i * rtb_Product2_i)) * 2.0F) *
    tasks100Hz_U.accel[2])) + tasks100Hz_U.accelOneG);

  /* DiscreteTransferFcn: '<S3>/earthAccelHP' */
  tasks100Hz_DW.earthAccelHP_tmp[0U] = 0.0F;
  tasks100Hz_DW.earthAccelHP_tmp[0] = (rtb_Gain1_e_idx_0 - (earthAccelDenHP[1] *
    tasks100Hz_DW.earthAccelHP_states[0])) / earthAccelDenHP[0];
  rtb_earthAccelHP_idx_0 = (earthAccelNumHP[0] * tasks100Hz_DW.earthAccelHP_tmp
    [0]) + (earthAccelNumHP[1] * tasks100Hz_DW.earthAccelHP_states[0]);
  tasks100Hz_DW.earthAccelHP_tmp[1] = (rtb_Gain1_e_idx_1 - (earthAccelDenHP[1] *
    tasks100Hz_DW.earthAccelHP_states[1])) / earthAccelDenHP[0];
  rtb_earthAccelHP_idx_1 = (earthAccelNumHP[0] * tasks100Hz_DW.earthAccelHP_tmp
    [1]) + (earthAccelNumHP[1] * tasks100Hz_DW.earthAccelHP_states[1]);
  tasks100Hz_DW.earthAccelHP_tmp[2] = (rtb_Gain1_e_idx_2 - (earthAccelDenHP[1] *
    tasks100Hz_DW.earthAccelHP_states[2])) / earthAccelDenHP[0];
  rtb_Sum2 = (earthAccelNumHP[0] * tasks100Hz_DW.earthAccelHP_tmp[2]) +
    (earthAccelNumHP[1] * tasks100Hz_DW.earthAccelHP_states[2]);
  rtb_earthAccelHP_idx_2 = rtb_Sum2;

  /* Update for DiscreteTransferFcn: '<S3>/earthAccelHP' */
  tasks100Hz_DW.earthAccelHP_states[0] = tasks100Hz_DW.earthAccelHP_tmp[0];
  tasks100Hz_DW.earthAccelHP_states[1] = tasks100Hz_DW.earthAccelHP_tmp[1];
  tasks100Hz_DW.earthAccelHP_states[2] = tasks100Hz_DW.earthAccelHP_tmp[2];

  /* End of Outputs for SubSystem: '<Root>/accelRotation' */

  /* Outport: '<Root>/earthAxisAccels' incorporates:
   *  DiscreteTransferFcn: '<S3>/earthAccelHP'
   */
  tasks100Hz_Y.earthAxisAccels[0] = rtb_earthAccelHP_idx_0;
  tasks100Hz_Y.earthAxisAccels[1] = rtb_earthAccelHP_idx_1;

  /* Outputs for Atomic SubSystem: '<Root>/accelRotation' */
  tasks100Hz_Y.earthAxisAccels[2] = rtb_Sum2;

  /* End of Outputs for SubSystem: '<Root>/accelRotation' */

  /* Outport: '<Root>/earthAxisAccelsNF' */
  tasks100Hz_Y.earthAxisAccelsNF[0] = rtb_Gain1_e_idx_0;
  tasks100Hz_Y.earthAxisAccelsNF[1] = rtb_Gain1_e_idx_1;
  tasks100Hz_Y.earthAxisAccelsNF[2] = rtb_Gain1_e_idx_2;

  /* Outputs for Atomic SubSystem: '<Root>/calcDeltaN' */
  /* Gain: '<S5>/Gain' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   *  Inport: '<Root>/gpsLatitude'
   *  Inport: '<Root>/homeLatitude'
   *  Sum: '<S5>/Sum1'
   */
  rtb_Sum2 = ((real32_T)(tasks100Hz_U.gpsLatitude - tasks100Hz_U.homeLatitude)) *
    0.0111194896F;

  /* End of Outputs for SubSystem: '<Root>/calcDeltaN' */

  /* Outputs for Atomic SubSystem: '<Root>/nEstimates' */
  /* RelationalOperator: '<S38>/Compare' incorporates:
   *  Constant: '<S38>/Constant'
   *  Inport: '<Root>/systemOperational'
   */
  rtb_Compare_dq = (tasks100Hz_U.systemOperational > 0);

  /* RelationalOperator: '<S37>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S37>/Delay Input1'
   */
  rtb_FixPtRelationalOperator = (((int32_T)rtb_Compare_dq) > ((int32_T)
    tasks100Hz_DW.DelayInput1_DSTATE));

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' */
  rtb_DiscreteTimeIntegrator2 = tasks100Hz_DW.DiscreteTimeIntegrator2_DSTATE;

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator3' */
  if (tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LOAD != 0) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTATE = rtb_Sum2;
  }

  if (rtb_FixPtRelationalOperator &&
      (tasks100Hz_DW.DiscreteTimeIntegrator3_PrevRes <= 0)) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTATE = rtb_Sum2;
  }

  rtb_DiscreteTimeIntegrator3 = tasks100Hz_DW.DiscreteTimeIntegrator3_DSTATE;

  /* Sum: '<S8>/Sum5' incorporates:
   *  Constant: '<S8>/Constant2'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator3'
   *  Product: '<S8>/Product3'
   *  Sum: '<S8>/Sum6'
   */
  rtb_Sum2 = ((rtb_Sum2 - tasks100Hz_DW.DiscreteTimeIntegrator3_DSTATE) *
              eepromConfig.posEstA) +
    tasks100Hz_DW.DiscreteTimeIntegrator2_DSTATE;

  /* Update for UnitDelay: '<S37>/Delay Input1' */
  tasks100Hz_DW.DelayInput1_DSTATE = rtb_Compare_dq;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator2' incorporates:
   *  Constant: '<S8>/Constant1'
   *  DataTypeConversion: '<S8>/Data Type Conversion'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator2'
   *  Gain: '<S8>/Gain'
   *  Update for Inport: '<Root>/gpsNdot'
   *  Product: '<S8>/Product2'
   *  Sum: '<S8>/Sum1'
   *  Sum: '<S8>/Sum4'
   */
  tasks100Hz_DW.DiscreteTimeIntegrator2_DSTATE += ((((0.01F * ((real32_T)
    tasks100Hz_U.gpsNdot)) - tasks100Hz_DW.DiscreteTimeIntegrator2_DSTATE) *
    eepromConfig.posEstB) - rtb_earthAccelHP_idx_0) * 0.01F;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LOAD = 0U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_DSTATE += 0.01F * rtb_Sum2;
  if (rtb_FixPtRelationalOperator) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevRes = 1;
  } else {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevRes = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator3' */
  /* End of Outputs for SubSystem: '<Root>/nEstimates' */

  /* Outputs for Atomic SubSystem: '<Root>/rotationCorrection' */
  /* Sqrt: '<S43>/sqrt' incorporates:
   *  Inport: '<Root>/q'
   *  Product: '<S44>/Product'
   *  Product: '<S44>/Product1'
   *  Product: '<S44>/Product2'
   *  Product: '<S44>/Product3'
   *  Sum: '<S44>/Sum'
   */
  rtb_Sum2 = (real32_T)sqrt((((tasks100Hz_U.q[0] * tasks100Hz_U.q[0]) +
    (tasks100Hz_U.q[1] * tasks100Hz_U.q[1])) + (tasks100Hz_U.q[2] *
    tasks100Hz_U.q[2])) + (tasks100Hz_U.q[3] * tasks100Hz_U.q[3]));

  /* Product: '<S42>/Product' incorporates:
   *  Inport: '<Root>/q'
   */
  rtb_Product2_i = tasks100Hz_U.q[0] / rtb_Sum2;

  /* Product: '<S42>/Product1' incorporates:
   *  Inport: '<Root>/q'
   */
  rtb_Product3_b3 = tasks100Hz_U.q[1] / rtb_Sum2;

  /* Product: '<S42>/Product2' incorporates:
   *  Inport: '<Root>/q'
   */
  rtb_Product1_c1 = tasks100Hz_U.q[2] / rtb_Sum2;

  /* Product: '<S42>/Product3' incorporates:
   *  Inport: '<Root>/q'
   */
  rtb_Sum2 = tasks100Hz_U.q[3] / rtb_Sum2;

  /* Sum: '<S9>/Sum' incorporates:
   *  Fcn: '<S39>/fcn1'
   *  Fcn: '<S39>/fcn2'
   *  Inport: '<Root>/magVar'
   *  Trigonometry: '<S39>/Trigonometric Function1'
   */
  rtb_Gain1_e_idx_2 = ((real32_T)atan2(((rtb_Product3_b3 * rtb_Product1_c1) +
    (rtb_Product2_i * rtb_Sum2)) * 2.0F, (((rtb_Product2_i * rtb_Product2_i) +
    (rtb_Product3_b3 * rtb_Product3_b3)) - (rtb_Product1_c1 * rtb_Product1_c1))
    - (rtb_Sum2 * rtb_Sum2))) + tasks100Hz_U.magVar;

  /* Switch: '<S41>/Switch' incorporates:
   *  Bias: '<S41>/Bias'
   *  Constant: '<S45>/Constant'
   *  Constant: '<S46>/Constant'
   *  RelationalOperator: '<S45>/Compare'
   *  RelationalOperator: '<S46>/Compare'
   *  Switch: '<S41>/Switch1'
   */
  if (rtb_Gain1_e_idx_2 >= 3.14159274F) {
    rtb_Gain1_e_idx_2 += -6.28318548F;
  } else {
    if (rtb_Gain1_e_idx_2 < -3.14159274F) {
      /* Switch: '<S41>/Switch1' incorporates:
       *  Bias: '<S41>/Bias1'
       */
      rtb_Gain1_e_idx_2 += 6.28318548F;
    }
  }

  /* End of Switch: '<S41>/Switch' */

  /* Gain: '<S40>/1//2' */
  rtb_Gain1_e_idx_2 *= 0.5F;

  /* Trigonometry: '<S40>/Trigonometric Function' */
  rtb_earthAccelHP_idx_0 = (real32_T)sin(rtb_Gain1_e_idx_2);

  /* Trigonometry: '<S40>/Trigonometric Function1' */
  rtb_Gain1_e_idx_0 = (real32_T)cos(rtb_Gain1_e_idx_2);

  /* End of Outputs for SubSystem: '<Root>/rotationCorrection' */

  /* Sqrt: '<S20>/sqrt' incorporates:
   *  Product: '<S21>/Product'
   *  Product: '<S21>/Product3'
   *  Sum: '<S21>/Sum'
   */
  rtb_Gain1_e_idx_1 = (real32_T)sqrt((rtb_Gain1_e_idx_0 * rtb_Gain1_e_idx_0) +
    (rtb_earthAccelHP_idx_0 * rtb_earthAccelHP_idx_0));

  /* Product: '<S16>/Product3' */
  rtb_Product3_d = rtb_earthAccelHP_idx_0 / rtb_Gain1_e_idx_1;

  /* Product: '<S16>/Product' */
  rtb_Gain1_e_idx_1 = rtb_Gain1_e_idx_0 / rtb_Gain1_e_idx_1;

  /* Outputs for Atomic SubSystem: '<Root>/calcDeltaE' */
  /* DataTypeConversion: '<S4>/Data Type Conversion3' incorporates:
   *  Inport: '<Root>/gpsLongitude'
   *  Inport: '<Root>/homeLongitude'
   *  Sum: '<S4>/Sum2'
   */
  rtb_Sum2 = (real32_T)(tasks100Hz_U.gpsLongitude - tasks100Hz_U.homeLongitude);

  /* Switch: '<S30>/Switch' incorporates:
   *  Bias: '<S30>/Bias'
   *  Constant: '<S31>/Constant'
   *  Constant: '<S32>/Constant'
   *  DataTypeConversion: '<S4>/Data Type Conversion3'
   *  Inport: '<Root>/gpsLongitude'
   *  Inport: '<Root>/homeLongitude'
   *  RelationalOperator: '<S31>/Compare'
   *  RelationalOperator: '<S32>/Compare'
   *  Sum: '<S4>/Sum2'
   *  Switch: '<S30>/Switch1'
   */
  if (rtb_Sum2 >= 1.8E+9F) {
    rtb_Sum2 += -3.6E+9F;
  } else if (rtb_Sum2 < -1.8E+9F) {
    /* Switch: '<S30>/Switch1' incorporates:
     *  Bias: '<S30>/Bias1'
     */
    rtb_Sum2 += 3.6E+9F;
  } else {
    rtb_Sum2 = (real32_T)(tasks100Hz_U.gpsLongitude - tasks100Hz_U.homeLongitude);
  }

  /* End of Switch: '<S30>/Switch' */

  /* Product: '<S4>/Product' incorporates:
   *  DataTypeConversion: '<S4>/Data Type Conversion2'
   *  Gain: '<S4>/Gain1'
   *  Gain: '<S4>/Gain2'
   *  Inport: '<Root>/gpsLatitude'
   *  Trigonometry: '<S4>/Trigonometric Function'
   */
  rtb_Sum2 = ((real32_T)cos(1.74532921E-9F * ((real32_T)tasks100Hz_U.gpsLatitude)))
    * (0.0111194896F * rtb_Sum2);

  /* End of Outputs for SubSystem: '<Root>/calcDeltaE' */

  /* Outputs for Atomic SubSystem: '<Root>/eEstimates' */
  /* RelationalOperator: '<S34>/Compare' incorporates:
   *  Constant: '<S34>/Constant'
   *  Inport: '<Root>/systemOperational'
   */
  rtb_Compare_dq = (tasks100Hz_U.systemOperational > 0);

  /* RelationalOperator: '<S33>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S33>/Delay Input1'
   */
  rtb_FixPtRelationalOperator_f = (((int32_T)rtb_Compare_dq) > ((int32_T)
    tasks100Hz_DW.DelayInput1_DSTATE_c));

  /* DiscreteIntegrator: '<S6>/Discrete-Time Integrator2' */
  rtb_Gain1_e_idx_2 = tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_a;

  /* DiscreteIntegrator: '<S6>/Discrete-Time Integrator3' */
  if (tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_i != 0) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_c = rtb_Sum2;
  }

  if (rtb_FixPtRelationalOperator_f &&
      (tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_d <= 0)) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_c = rtb_Sum2;
  }

  rtb_DiscreteTimeIntegrator3_f = tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_c;

  /* Sum: '<S6>/Sum5' incorporates:
   *  Constant: '<S6>/Constant2'
   *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator3'
   *  Product: '<S6>/Product3'
   *  Sum: '<S6>/Sum6'
   */
  rtb_Sum2 = ((rtb_Sum2 - tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_c) *
              eepromConfig.posEstA) +
    tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_a;

  /* Update for UnitDelay: '<S33>/Delay Input1' */
  tasks100Hz_DW.DelayInput1_DSTATE_c = rtb_Compare_dq;

  /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator2' incorporates:
   *  Constant: '<S6>/Constant1'
   *  DataTypeConversion: '<S6>/Data Type Conversion'
   *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator2'
   *  Gain: '<S6>/Gain'
   *  Update for Inport: '<Root>/gpsEdot'
   *  Product: '<S6>/Product2'
   *  Sum: '<S6>/Sum1'
   *  Sum: '<S6>/Sum4'
   */
  tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_a += ((((0.01F * ((real32_T)
    tasks100Hz_U.gpsEdot)) - tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_a) *
    eepromConfig.posEstB) - rtb_earthAccelHP_idx_1) * 0.01F;

  /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_i = 0U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_c += 0.01F * rtb_Sum2;
  if (rtb_FixPtRelationalOperator_f) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_d = 1;
  } else {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_d = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3' */
  /* End of Outputs for SubSystem: '<Root>/eEstimates' */

  /* Outputs for Atomic SubSystem: '<Root>/hEstimates' */
  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   *  Inport: '<Root>/systemOperational'
   */
  rtb_Compare_dq = (tasks100Hz_U.systemOperational > 0);

  /* RelationalOperator: '<S35>/FixPt Relational Operator' incorporates:
   *  UnitDelay: '<S35>/Delay Input1'
   */
  rtb_FixPtRelationalOperator_d = (((int32_T)rtb_Compare_dq) > ((int32_T)
    tasks100Hz_DW.DelayInput1_DSTATE_a));

  /* DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' */
  rtb_Product3_b3 = tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_p;

  /* DiscreteIntegrator: '<S7>/Discrete-Time Integrator3' incorporates:
   *  Inport: '<Root>/pressureAlt'
   */
  if (tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_o != 0) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_h = tasks100Hz_U.pressureAlt;
  }

  if (rtb_FixPtRelationalOperator_d &&
      (tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_p <= 0)) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_h = tasks100Hz_U.pressureAlt;
  }

  rtb_Product1_c1 = tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_h;

  /* Sum: '<S7>/Sum6' incorporates:
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator3'
   *  Inport: '<Root>/pressureAlt'
   */
  rtb_Sum2 = tasks100Hz_U.pressureAlt -
    tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_h;

  /* Sum: '<S7>/Sum5' incorporates:
   *  Constant: '<S7>/Constant2'
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator2'
   *  Product: '<S7>/Product3'
   */
  rtb_Product2_i = (rtb_Sum2 * eepromConfig.hEstA) +
    tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_p;

  /* Update for UnitDelay: '<S35>/Delay Input1' */
  tasks100Hz_DW.DelayInput1_DSTATE_a = rtb_Compare_dq;

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator2' incorporates:
   *  Constant: '<S7>/Constant1'
   *  Product: '<S7>/Product2'
   *  Sum: '<S7>/Sum4'
   */
  tasks100Hz_DW.DiscreteTimeIntegrator2_DSTAT_p += ((rtb_Sum2 *
    eepromConfig.hEstB) + rtb_earthAccelHP_idx_2) * 0.01F;

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_o = 0U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_DSTAT_h += 0.01F * rtb_Product2_i;
  if (rtb_FixPtRelationalOperator_d) {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_p = 1;
  } else {
    tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_p = 0;
  }

  /* End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator3' */
  /* End of Outputs for SubSystem: '<Root>/hEstimates' */

  /* Outport: '<Root>/bodyVelocityEstimates' incorporates:
   *  Constant: '<S17>/Constant1'
   *  Constant: '<S18>/Constant1'
   *  Gain: '<S17>/Gain'
   *  Gain: '<S17>/Gain2'
   *  Gain: '<S18>/Gain'
   *  Gain: '<S18>/Gain2'
   *  Product: '<S17>/Product10'
   *  Product: '<S17>/Product3'
   *  Product: '<S17>/Product6'
   *  Product: '<S17>/Product9'
   *  Product: '<S18>/Product10'
   *  Product: '<S18>/Product3'
   *  Product: '<S18>/Product6'
   *  Product: '<S18>/Product9'
   *  Sum: '<S17>/Sum4'
   *  Sum: '<S17>/Sum7'
   *  Sum: '<S18>/Sum4'
   *  Sum: '<S18>/Sum7'
   */
  tasks100Hz_Y.bodyVelocityEstimates[0] = (((0.5F - (rtb_Product3_d *
    rtb_Product3_d)) * 2.0F) * rtb_DiscreteTimeIntegrator2) +
    (((rtb_Gain1_e_idx_1 * rtb_Product3_d) * 2.0F) * rtb_Gain1_e_idx_2);
  tasks100Hz_Y.bodyVelocityEstimates[1] = (((0.5F - (rtb_Product3_d *
    rtb_Product3_d)) * 2.0F) * rtb_Gain1_e_idx_2) + (((rtb_Gain1_e_idx_1 *
    rtb_Product3_d) * -2.0F) * rtb_DiscreteTimeIntegrator2);
  tasks100Hz_Y.bodyVelocityEstimates[2] = rtb_Product3_b3;

  /* Sqrt: '<S14>/sqrt' incorporates:
   *  Product: '<S15>/Product'
   *  Product: '<S15>/Product3'
   *  Sum: '<S15>/Sum'
   */
  rtb_Sum2 = (real32_T)sqrt((rtb_Gain1_e_idx_0 * rtb_Gain1_e_idx_0) +
    (rtb_earthAccelHP_idx_0 * rtb_earthAccelHP_idx_0));

  /* Product: '<S10>/Product3' */
  rtb_Product3_d = rtb_earthAccelHP_idx_0 / rtb_Sum2;

  /* Product: '<S10>/Product' */
  rtb_Sum2 = rtb_Gain1_e_idx_0 / rtb_Sum2;

  /* Outport: '<Root>/bodyPositionEstimates' incorporates:
   *  Constant: '<S11>/Constant1'
   *  Constant: '<S12>/Constant1'
   *  Gain: '<S11>/Gain'
   *  Gain: '<S11>/Gain2'
   *  Gain: '<S12>/Gain'
   *  Gain: '<S12>/Gain2'
   *  Product: '<S11>/Product10'
   *  Product: '<S11>/Product3'
   *  Product: '<S11>/Product6'
   *  Product: '<S11>/Product9'
   *  Product: '<S12>/Product10'
   *  Product: '<S12>/Product3'
   *  Product: '<S12>/Product6'
   *  Product: '<S12>/Product9'
   *  Sum: '<S11>/Sum4'
   *  Sum: '<S11>/Sum7'
   *  Sum: '<S12>/Sum4'
   *  Sum: '<S12>/Sum7'
   */
  tasks100Hz_Y.bodyPositionEstimates[0] = (((0.5F - (rtb_Product3_d *
    rtb_Product3_d)) * 2.0F) * rtb_DiscreteTimeIntegrator3) + (((rtb_Sum2 *
    rtb_Product3_d) * 2.0F) * rtb_DiscreteTimeIntegrator3_f);
  tasks100Hz_Y.bodyPositionEstimates[1] = (((0.5F - (rtb_Product3_d *
    rtb_Product3_d)) * 2.0F) * rtb_DiscreteTimeIntegrator3_f) + (((rtb_Sum2 *
    rtb_Product3_d) * -2.0F) * rtb_DiscreteTimeIntegrator3);
  tasks100Hz_Y.bodyPositionEstimates[2] = rtb_Product1_c1;

  /* Outport: '<Root>/velocityEstimates' */
  tasks100Hz_Y.velocityEstimates[0] = rtb_DiscreteTimeIntegrator2;
  tasks100Hz_Y.velocityEstimates[1] = rtb_Gain1_e_idx_2;
  tasks100Hz_Y.velocityEstimates[2] = rtb_Product3_b3;

  /* Outport: '<Root>/positionEstimates' */
  tasks100Hz_Y.positionEstimates[0] = rtb_DiscreteTimeIntegrator3;
  tasks100Hz_Y.positionEstimates[1] = rtb_DiscreteTimeIntegrator3_f;
  tasks100Hz_Y.positionEstimates[2] = rtb_Product1_c1;
}

/* Model initialize function */
void tasks100Hz_initialize(void)
{
  /* InitializeConditions for Atomic SubSystem: '<Root>/nEstimates' */
  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LOAD = 1U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_PrevRes = 2;

  /* End of InitializeConditions for SubSystem: '<Root>/nEstimates' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/eEstimates' */
  /* InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_i = 1U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_d = 2;

  /* End of InitializeConditions for SubSystem: '<Root>/eEstimates' */

  /* InitializeConditions for Atomic SubSystem: '<Root>/hEstimates' */
  /* InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator3' */
  tasks100Hz_DW.DiscreteTimeIntegrator3_IC_LO_o = 1U;
  tasks100Hz_DW.DiscreteTimeIntegrator3_PrevR_p = 2;

  /* End of InitializeConditions for SubSystem: '<Root>/hEstimates' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
