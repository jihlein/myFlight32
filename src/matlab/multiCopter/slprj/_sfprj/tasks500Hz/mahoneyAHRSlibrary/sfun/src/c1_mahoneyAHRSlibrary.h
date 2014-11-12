#ifndef __c1_mahoneyAHRSlibrary_h__
#define __c1_mahoneyAHRSlibrary_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_mahoneyAHRSlibraryInstanceStruct
#define typedef_SFc1_mahoneyAHRSlibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_mahoneyAHRSlibrary;
  real32_T c1_accelConfidenceDecay;
  boolean_T c1_accelConfidenceDecay_not_empty;
  uint8_T c1_ahrsFirstPass;
  boolean_T c1_ahrsFirstPass_not_empty;
  real32_T c1_eAcc[3];
  boolean_T c1_eAcc_not_empty;
  real32_T c1_eMag[3];
  boolean_T c1_eMag_not_empty;
  real32_T c1_pastAccelFilter;
  boolean_T c1_pastAccelFilter_not_empty;
  real32_T c1_q[4];
  boolean_T c1_q_not_empty;
  uint8_T c1_qValid;
  boolean_T c1_qValid_not_empty;
  uint8_T c1_qValidCount;
  boolean_T c1_qValidCount_not_empty;
  real32_T c1_q1q1;
  boolean_T c1_q1q1_not_empty;
  real32_T c1_q1q2;
  boolean_T c1_q1q2_not_empty;
  real32_T c1_q1q3;
  boolean_T c1_q1q3_not_empty;
  real32_T c1_q1q4;
  boolean_T c1_q1q4_not_empty;
  real32_T c1_q2q2;
  boolean_T c1_q2q2_not_empty;
  real32_T c1_q2q3;
  boolean_T c1_q2q3_not_empty;
  real32_T c1_q2q4;
  boolean_T c1_q2q4_not_empty;
  real32_T c1_q3q3;
  boolean_T c1_q3q3_not_empty;
  real32_T c1_q3q4;
  boolean_T c1_q3q4_not_empty;
  real32_T c1_q4q4;
  boolean_T c1_q4q4_not_empty;
} SFc1_mahoneyAHRSlibraryInstanceStruct;

#endif                                 /*typedef_SFc1_mahoneyAHRSlibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_mahoneyAHRSlibrary_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_mahoneyAHRSlibrary_get_check_sum(mxArray *plhs[]);
extern void c1_mahoneyAHRSlibrary_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
