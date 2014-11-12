/* Include files */

#include <stddef.h>
#include "blas.h"
#include "mahoneyAHRSlibrary_sfun.h"
#include "c1_mahoneyAHRSlibrary.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "mahoneyAHRSlibrary_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[63] = { "initialRoll", "initialPitch",
  "cosRoll", "sinRoll", "cosPitch", "sinPitch", "magX", "magY", "initialHdg",
  "u1", "u2", "u3", "u4", "u5", "u6", "accelNorm", "input", "output",
  "accelConfidence", "v", "h", "b", "w", "qDot", "asinPitch", "nargin",
  "nargout", "gyro", "accel", "mag", "dt", "gyroValid", "accelValid", "magValid",
  "accelOneG", "accelDataUpdate", "magDataUpdate", "magVar", "kpAcc", "kpMag",
  "accelCutoff", "attTrim", "qOut", "qValidOut", "attitude",
  "accelConfidenceDecay", "ahrsFirstPass", "eAcc", "eMag", "pastAccelFilter",
  "q", "qValid", "qValidCount", "q1q1", "q1q2", "q1q3", "q1q4", "q2q2", "q2q3",
  "q2q4", "q3q3", "q3q4", "q4q4" };

static const char * c1_b_debug_family_names[4] = { "nargin", "nargout", "q",
  "qConj" };

static const char * c1_c_debug_family_names[5] = { "nargin", "nargout", "a", "b",
  "ab" };

static const char * c1_d_debug_family_names[5] = { "nargin", "nargout", "a", "b",
  "ab" };

/* Function Declarations */
static void initialize_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void initialize_params_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void enable_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance);
static void disable_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct *
  chartInstance);
static void c1_update_debugger_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void set_sim_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance, const mxArray *c1_st);
static void finalize_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance);
static void sf_gateway_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void c1_chartstep_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void initSimStructsc1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real32_T c1_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q4q4, const char_T *c1_identifier);
static real32_T c1_b_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_c_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q3q4, const char_T *c1_identifier);
static real32_T c1_d_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_e_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q3q3, const char_T *c1_identifier);
static real32_T c1_f_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_g_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q4, const char_T *c1_identifier);
static real32_T c1_h_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_i_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q3, const char_T *c1_identifier);
static real32_T c1_j_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_k_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q2, const char_T *c1_identifier);
static real32_T c1_l_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_m_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q4, const char_T *c1_identifier);
static real32_T c1_n_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_o_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q3, const char_T *c1_identifier);
static real32_T c1_p_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_q_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q2, const char_T *c1_identifier);
static real32_T c1_r_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_s_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q1, const char_T *c1_identifier);
static real32_T c1_t_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_k_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_u_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_qValidCount, const char_T *c1_identifier);
static uint8_T c1_v_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_l_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_w_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_qValid, const char_T *c1_identifier);
static uint8_T c1_x_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_m_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_y_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q, const char_T *c1_identifier, real32_T
  c1_y[4]);
static void c1_ab_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[4]);
static void c1_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_n_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_bb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_pastAccelFilter, const char_T
  *c1_identifier);
static real32_T c1_cb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_o_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_db_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_eMag, const char_T *c1_identifier,
  real32_T c1_y[3]);
static void c1_eb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3]);
static void c1_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_p_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_fb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_eAcc, const char_T *c1_identifier,
  real32_T c1_y[3]);
static void c1_gb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3]);
static void c1_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_q_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_hb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_ahrsFirstPass, const char_T *c1_identifier);
static uint8_T c1_ib_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_r_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real32_T c1_jb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_accelConfidenceDecay, const char_T
  *c1_identifier);
static real32_T c1_kb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_s_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_lb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_attitude, const char_T *c1_identifier,
  real32_T c1_y[3]);
static void c1_mb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3]);
static void c1_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_t_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static uint8_T c1_nb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_qValidOut, const char_T *c1_identifier);
static uint8_T c1_ob_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_u_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_pb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_qOut, const char_T *c1_identifier, real32_T
  c1_y[4]);
static void c1_qb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[4]);
static void c1_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_v_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_w_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_x_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_y_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_rb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static real32_T c1_sb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_tb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[4]);
static void c1_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_ub_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3]);
static void c1_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(const mxArray **c1_info);
static const mxArray *c1_emlrt_marshallOut(const char * c1_u);
static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u);
static real32_T c1_atan2(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_y, real32_T c1_x);
static real32_T c1_sqrt(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x);
static void c1_eml_error(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static real32_T c1_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x[3]);
static void c1_eml_switch_helper(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance);
static void c1_realmin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static real32_T c1_abs(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x);
static void c1_cross(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                     real32_T c1_a[3], real32_T c1_b[3], real32_T c1_c[3]);
static void c1_quaternConj(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_b_q[4], real32_T c1_qConj[4]);
static void c1_quaternProd(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_a[4], real32_T c1_b[4], real_T c1_ab[4]);
static void c1_b_quaternProd(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, real32_T c1_a[4], real_T c1_b[4], real_T c1_ab[4]);
static real_T c1_b_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real_T c1_x[2]);
static real32_T c1_c_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x[4]);
static real32_T c1_asin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x);
static void c1_b_eml_error(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance);
static const mxArray *c1_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_vb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_b_sqrt(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                      real32_T *c1_x);
static void c1_b_asin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                      real32_T *c1_x);
static void init_dsm_address_info(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_accelConfidenceDecay_not_empty = false;
  chartInstance->c1_ahrsFirstPass_not_empty = false;
  chartInstance->c1_eAcc_not_empty = false;
  chartInstance->c1_eMag_not_empty = false;
  chartInstance->c1_pastAccelFilter_not_empty = false;
  chartInstance->c1_q_not_empty = false;
  chartInstance->c1_qValid_not_empty = false;
  chartInstance->c1_qValidCount_not_empty = false;
  chartInstance->c1_q1q1_not_empty = false;
  chartInstance->c1_q1q2_not_empty = false;
  chartInstance->c1_q1q3_not_empty = false;
  chartInstance->c1_q1q4_not_empty = false;
  chartInstance->c1_q2q2_not_empty = false;
  chartInstance->c1_q2q3_not_empty = false;
  chartInstance->c1_q2q4_not_empty = false;
  chartInstance->c1_q3q3_not_empty = false;
  chartInstance->c1_q3q4_not_empty = false;
  chartInstance->c1_q4q4_not_empty = false;
  chartInstance->c1_is_active_c1_mahoneyAHRSlibrary = 0U;
}

static void initialize_params_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct *
  chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  int32_T c1_i0;
  real32_T c1_u[3];
  const mxArray *c1_b_y = NULL;
  int32_T c1_i1;
  real32_T c1_b_u[4];
  const mxArray *c1_c_y = NULL;
  uint8_T c1_hoistedGlobal;
  uint8_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real32_T c1_b_hoistedGlobal;
  real32_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  uint8_T c1_c_hoistedGlobal;
  uint8_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  int32_T c1_i2;
  real32_T c1_f_u[3];
  const mxArray *c1_g_y = NULL;
  int32_T c1_i3;
  real32_T c1_g_u[3];
  const mxArray *c1_h_y = NULL;
  real32_T c1_d_hoistedGlobal;
  real32_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  int32_T c1_i4;
  real32_T c1_i_u[4];
  const mxArray *c1_j_y = NULL;
  real32_T c1_e_hoistedGlobal;
  real32_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real32_T c1_f_hoistedGlobal;
  real32_T c1_k_u;
  const mxArray *c1_l_y = NULL;
  real32_T c1_g_hoistedGlobal;
  real32_T c1_l_u;
  const mxArray *c1_m_y = NULL;
  real32_T c1_h_hoistedGlobal;
  real32_T c1_m_u;
  const mxArray *c1_n_y = NULL;
  real32_T c1_i_hoistedGlobal;
  real32_T c1_n_u;
  const mxArray *c1_o_y = NULL;
  real32_T c1_j_hoistedGlobal;
  real32_T c1_o_u;
  const mxArray *c1_p_y = NULL;
  real32_T c1_k_hoistedGlobal;
  real32_T c1_p_u;
  const mxArray *c1_q_y = NULL;
  real32_T c1_l_hoistedGlobal;
  real32_T c1_q_u;
  const mxArray *c1_r_y = NULL;
  real32_T c1_m_hoistedGlobal;
  real32_T c1_r_u;
  const mxArray *c1_s_y = NULL;
  real32_T c1_n_hoistedGlobal;
  real32_T c1_s_u;
  const mxArray *c1_t_y = NULL;
  uint8_T c1_o_hoistedGlobal;
  uint8_T c1_t_u;
  const mxArray *c1_u_y = NULL;
  uint8_T c1_p_hoistedGlobal;
  uint8_T c1_u_u;
  const mxArray *c1_v_y = NULL;
  uint8_T c1_q_hoistedGlobal;
  uint8_T c1_v_u;
  const mxArray *c1_w_y = NULL;
  uint8_T *c1_qValidOut;
  real32_T (*c1_qOut)[4];
  real32_T (*c1_attitude)[3];
  c1_attitude = (real32_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c1_qValidOut = (uint8_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_qOut = (real32_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(22, 1), false);
  for (c1_i0 = 0; c1_i0 < 3; c1_i0++) {
    c1_u[c1_i0] = (*c1_attitude)[c1_i0];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  for (c1_i1 = 0; c1_i1 < 4; c1_i1++) {
    c1_b_u[c1_i1] = (*c1_qOut)[c1_i1];
  }

  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", c1_b_u, 1, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_hoistedGlobal = *c1_qValidOut;
  c1_c_u = c1_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_b_hoistedGlobal = chartInstance->c1_accelConfidenceDecay;
  c1_d_u = c1_b_hoistedGlobal;
  c1_e_y = NULL;
  if (!chartInstance->c1_accelConfidenceDecay_not_empty) {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_c_hoistedGlobal = chartInstance->c1_ahrsFirstPass;
  c1_e_u = c1_c_hoistedGlobal;
  c1_f_y = NULL;
  if (!chartInstance->c1_ahrsFirstPass_not_empty) {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 4, c1_f_y);
  for (c1_i2 = 0; c1_i2 < 3; c1_i2++) {
    c1_f_u[c1_i2] = chartInstance->c1_eAcc[c1_i2];
  }

  c1_g_y = NULL;
  if (!chartInstance->c1_eAcc_not_empty) {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_f_u, 1, 0U, 1U, 0U, 1, 3),
                  false);
  }

  sf_mex_setcell(c1_y, 5, c1_g_y);
  for (c1_i3 = 0; c1_i3 < 3; c1_i3++) {
    c1_g_u[c1_i3] = chartInstance->c1_eMag[c1_i3];
  }

  c1_h_y = NULL;
  if (!chartInstance->c1_eMag_not_empty) {
    sf_mex_assign(&c1_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_h_y, sf_mex_create("y", c1_g_u, 1, 0U, 1U, 0U, 1, 3),
                  false);
  }

  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_d_hoistedGlobal = chartInstance->c1_pastAccelFilter;
  c1_h_u = c1_d_hoistedGlobal;
  c1_i_y = NULL;
  if (!chartInstance->c1_pastAccelFilter_not_empty) {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 7, c1_i_y);
  for (c1_i4 = 0; c1_i4 < 4; c1_i4++) {
    c1_i_u[c1_i4] = chartInstance->c1_q[c1_i4];
  }

  c1_j_y = NULL;
  if (!chartInstance->c1_q_not_empty) {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_j_y, sf_mex_create("y", c1_i_u, 1, 0U, 1U, 0U, 2, 1, 4),
                  false);
  }

  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_e_hoistedGlobal = chartInstance->c1_q1q1;
  c1_j_u = c1_e_hoistedGlobal;
  c1_k_y = NULL;
  if (!chartInstance->c1_q1q1_not_empty) {
    sf_mex_assign(&c1_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 9, c1_k_y);
  c1_f_hoistedGlobal = chartInstance->c1_q1q2;
  c1_k_u = c1_f_hoistedGlobal;
  c1_l_y = NULL;
  if (!chartInstance->c1_q1q2_not_empty) {
    sf_mex_assign(&c1_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_k_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 10, c1_l_y);
  c1_g_hoistedGlobal = chartInstance->c1_q1q3;
  c1_l_u = c1_g_hoistedGlobal;
  c1_m_y = NULL;
  if (!chartInstance->c1_q1q3_not_empty) {
    sf_mex_assign(&c1_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_l_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 11, c1_m_y);
  c1_h_hoistedGlobal = chartInstance->c1_q1q4;
  c1_m_u = c1_h_hoistedGlobal;
  c1_n_y = NULL;
  if (!chartInstance->c1_q1q4_not_empty) {
    sf_mex_assign(&c1_n_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_n_y, sf_mex_create("y", &c1_m_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 12, c1_n_y);
  c1_i_hoistedGlobal = chartInstance->c1_q2q2;
  c1_n_u = c1_i_hoistedGlobal;
  c1_o_y = NULL;
  if (!chartInstance->c1_q2q2_not_empty) {
    sf_mex_assign(&c1_o_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_o_y, sf_mex_create("y", &c1_n_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 13, c1_o_y);
  c1_j_hoistedGlobal = chartInstance->c1_q2q3;
  c1_o_u = c1_j_hoistedGlobal;
  c1_p_y = NULL;
  if (!chartInstance->c1_q2q3_not_empty) {
    sf_mex_assign(&c1_p_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_p_y, sf_mex_create("y", &c1_o_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 14, c1_p_y);
  c1_k_hoistedGlobal = chartInstance->c1_q2q4;
  c1_p_u = c1_k_hoistedGlobal;
  c1_q_y = NULL;
  if (!chartInstance->c1_q2q4_not_empty) {
    sf_mex_assign(&c1_q_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_q_y, sf_mex_create("y", &c1_p_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 15, c1_q_y);
  c1_l_hoistedGlobal = chartInstance->c1_q3q3;
  c1_q_u = c1_l_hoistedGlobal;
  c1_r_y = NULL;
  if (!chartInstance->c1_q3q3_not_empty) {
    sf_mex_assign(&c1_r_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_r_y, sf_mex_create("y", &c1_q_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 16, c1_r_y);
  c1_m_hoistedGlobal = chartInstance->c1_q3q4;
  c1_r_u = c1_m_hoistedGlobal;
  c1_s_y = NULL;
  if (!chartInstance->c1_q3q4_not_empty) {
    sf_mex_assign(&c1_s_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_s_y, sf_mex_create("y", &c1_r_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 17, c1_s_y);
  c1_n_hoistedGlobal = chartInstance->c1_q4q4;
  c1_s_u = c1_n_hoistedGlobal;
  c1_t_y = NULL;
  if (!chartInstance->c1_q4q4_not_empty) {
    sf_mex_assign(&c1_t_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_t_y, sf_mex_create("y", &c1_s_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 18, c1_t_y);
  c1_o_hoistedGlobal = chartInstance->c1_qValid;
  c1_t_u = c1_o_hoistedGlobal;
  c1_u_y = NULL;
  if (!chartInstance->c1_qValid_not_empty) {
    sf_mex_assign(&c1_u_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_u_y, sf_mex_create("y", &c1_t_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 19, c1_u_y);
  c1_p_hoistedGlobal = chartInstance->c1_qValidCount;
  c1_u_u = c1_p_hoistedGlobal;
  c1_v_y = NULL;
  if (!chartInstance->c1_qValidCount_not_empty) {
    sf_mex_assign(&c1_v_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c1_v_y, sf_mex_create("y", &c1_u_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c1_y, 20, c1_v_y);
  c1_q_hoistedGlobal = chartInstance->c1_is_active_c1_mahoneyAHRSlibrary;
  c1_v_u = c1_q_hoistedGlobal;
  c1_w_y = NULL;
  sf_mex_assign(&c1_w_y, sf_mex_create("y", &c1_v_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 21, c1_w_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real32_T c1_fv0[3];
  int32_T c1_i5;
  real32_T c1_fv1[4];
  int32_T c1_i6;
  real32_T c1_fv2[3];
  int32_T c1_i7;
  real32_T c1_fv3[3];
  int32_T c1_i8;
  real32_T c1_fv4[4];
  int32_T c1_i9;
  uint8_T *c1_qValidOut;
  real32_T (*c1_qOut)[4];
  real32_T (*c1_attitude)[3];
  c1_attitude = (real32_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c1_qValidOut = (uint8_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_qOut = (real32_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_lb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
    "attitude", c1_fv0);
  for (c1_i5 = 0; c1_i5 < 3; c1_i5++) {
    (*c1_attitude)[c1_i5] = c1_fv0[c1_i5];
  }

  c1_pb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
    "qOut", c1_fv1);
  for (c1_i6 = 0; c1_i6 < 4; c1_i6++) {
    (*c1_qOut)[c1_i6] = c1_fv1[c1_i6];
  }

  *c1_qValidOut = c1_nb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 2)), "qValidOut");
  chartInstance->c1_accelConfidenceDecay = c1_jb_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 3)), "accelConfidenceDecay");
  chartInstance->c1_ahrsFirstPass = c1_hb_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 4)), "ahrsFirstPass");
  c1_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 5)),
    "eAcc", c1_fv2);
  for (c1_i7 = 0; c1_i7 < 3; c1_i7++) {
    chartInstance->c1_eAcc[c1_i7] = c1_fv2[c1_i7];
  }

  c1_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 6)),
    "eMag", c1_fv3);
  for (c1_i8 = 0; c1_i8 < 3; c1_i8++) {
    chartInstance->c1_eMag[c1_i8] = c1_fv3[c1_i8];
  }

  chartInstance->c1_pastAccelFilter = c1_bb_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 7)), "pastAccelFilter");
  c1_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 8)), "q",
                        c1_fv4);
  for (c1_i9 = 0; c1_i9 < 4; c1_i9++) {
    chartInstance->c1_q[c1_i9] = c1_fv4[c1_i9];
  }

  chartInstance->c1_q1q1 = c1_s_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 9)), "q1q1");
  chartInstance->c1_q1q2 = c1_q_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 10)), "q1q2");
  chartInstance->c1_q1q3 = c1_o_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 11)), "q1q3");
  chartInstance->c1_q1q4 = c1_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 12)), "q1q4");
  chartInstance->c1_q2q2 = c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 13)), "q2q2");
  chartInstance->c1_q2q3 = c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 14)), "q2q3");
  chartInstance->c1_q2q4 = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 15)), "q2q4");
  chartInstance->c1_q3q3 = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 16)), "q3q3");
  chartInstance->c1_q3q4 = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 17)), "q3q4");
  chartInstance->c1_q4q4 = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 18)), "q4q4");
  chartInstance->c1_qValid = c1_w_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 19)), "qValid");
  chartInstance->c1_qValidCount = c1_u_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 20)), "qValidCount");
  chartInstance->c1_is_active_c1_mahoneyAHRSlibrary = c1_nb_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 21)),
     "is_active_c1_mahoneyAHRSlibrary");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_mahoneyAHRSlibrary(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_mahoneyAHRSlibrary(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  int32_T c1_i10;
  int32_T c1_i11;
  int32_T c1_i12;
  int32_T c1_i13;
  int32_T c1_i14;
  int32_T c1_i15;
  real32_T *c1_dt;
  uint8_T *c1_gyroValid;
  uint8_T *c1_accelValid;
  uint8_T *c1_magValid;
  real32_T *c1_accelOneG;
  uint8_T *c1_accelDataUpdate;
  uint8_T *c1_magDataUpdate;
  real32_T *c1_magVar;
  real32_T *c1_kpAcc;
  real32_T *c1_kpMag;
  uint8_T *c1_qValidOut;
  real32_T *c1_accelCutoff;
  real32_T (*c1_attTrim)[2];
  real32_T (*c1_attitude)[3];
  real32_T (*c1_mag)[3];
  real32_T (*c1_accel)[3];
  real32_T (*c1_qOut)[4];
  real32_T (*c1_gyro)[3];
  c1_attTrim = (real32_T (*)[2])ssGetInputPortSignal(chartInstance->S, 14);
  c1_attitude = (real32_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c1_accelCutoff = (real32_T *)ssGetInputPortSignal(chartInstance->S, 13);
  c1_qValidOut = (uint8_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_kpMag = (real32_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c1_kpAcc = (real32_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c1_magVar = (real32_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c1_magDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c1_accelDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c1_accelOneG = (real32_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c1_magValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c1_accelValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_gyroValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_dt = (real32_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_mag = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c1_accel = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c1_qOut = (real32_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_gyro = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i10 = 0; c1_i10 < 3; c1_i10++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_gyro)[c1_i10], 0U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_mahoneyAHRSlibrary(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_mahoneyAHRSlibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i11 = 0; c1_i11 < 4; c1_i11++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_qOut)[c1_i11], 1U);
  }

  for (c1_i12 = 0; c1_i12 < 3; c1_i12++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_accel)[c1_i12], 2U);
  }

  for (c1_i13 = 0; c1_i13 < 3; c1_i13++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_mag)[c1_i13], 3U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c1_dt, 4U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_gyroValid, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_accelValid, 6U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_magValid, 7U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_accelOneG, 8U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_accelDataUpdate, 9U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_magDataUpdate, 10U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_magVar, 11U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_kpAcc, 12U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_kpMag, 13U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_qValidOut, 14U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_accelCutoff, 15U);
  for (c1_i14 = 0; c1_i14 < 3; c1_i14++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_attitude)[c1_i14], 16U);
  }

  for (c1_i15 = 0; c1_i15 < 2; c1_i15++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c1_attTrim)[c1_i15], 17U);
  }
}

static void c1_chartstep_c1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  real32_T c1_hoistedGlobal;
  uint8_T c1_b_hoistedGlobal;
  uint8_T c1_c_hoistedGlobal;
  uint8_T c1_d_hoistedGlobal;
  real32_T c1_e_hoistedGlobal;
  uint8_T c1_f_hoistedGlobal;
  uint8_T c1_g_hoistedGlobal;
  real32_T c1_h_hoistedGlobal;
  real32_T c1_i_hoistedGlobal;
  real32_T c1_j_hoistedGlobal;
  real32_T c1_k_hoistedGlobal;
  int32_T c1_i16;
  real32_T c1_gyro[3];
  int32_T c1_i17;
  real32_T c1_accel[3];
  int32_T c1_i18;
  real32_T c1_mag[3];
  real32_T c1_dt;
  uint8_T c1_gyroValid;
  uint8_T c1_accelValid;
  uint8_T c1_magValid;
  real32_T c1_accelOneG;
  uint8_T c1_accelDataUpdate;
  uint8_T c1_magDataUpdate;
  real32_T c1_magVar;
  real32_T c1_kpAcc;
  real32_T c1_kpMag;
  real32_T c1_accelCutoff;
  int32_T c1_i19;
  real32_T c1_attTrim[2];
  uint32_T c1_debug_family_var_map[63];
  real32_T c1_initialRoll;
  real32_T c1_initialPitch;
  real32_T c1_cosRoll;
  real32_T c1_sinRoll;
  real32_T c1_cosPitch;
  real32_T c1_sinPitch;
  real32_T c1_magX;
  real32_T c1_magY;
  real32_T c1_initialHdg;
  real32_T c1_u1;
  real32_T c1_u2;
  real32_T c1_u3;
  real32_T c1_u4;
  real32_T c1_u5;
  real32_T c1_u6;
  real32_T c1_accelNorm;
  real32_T c1_input;
  real32_T c1_output;
  real32_T c1_accelConfidence;
  real32_T c1_v[3];
  real_T c1_h[4];
  real_T c1_b[4];
  real32_T c1_w[3];
  real_T c1_qDot[4];
  real32_T c1_asinPitch;
  real_T c1_nargin = 15.0;
  real_T c1_nargout = 3.0;
  real32_T c1_qOut[4];
  uint8_T c1_qValidOut;
  real32_T c1_attitude[3];
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  static real32_T c1_fv5[4] = { 1.0F, 0.0F, 0.0F, 0.0F };

  int32_T c1_i23;
  real32_T c1_x;
  real32_T c1_b_x;
  real32_T c1_c_x;
  real32_T c1_d_x;
  real32_T c1_e_x;
  real32_T c1_f_x;
  real32_T c1_g_x;
  real32_T c1_h_x;
  real32_T c1_A;
  real32_T c1_i_x;
  real32_T c1_j_x;
  real32_T c1_k_x;
  real32_T c1_y;
  real32_T c1_l_x;
  real32_T c1_m_x;
  real32_T c1_b_A;
  real32_T c1_n_x;
  real32_T c1_o_x;
  real32_T c1_p_x;
  real32_T c1_b_y;
  real32_T c1_q_x;
  real32_T c1_r_x;
  real32_T c1_c_A;
  real32_T c1_s_x;
  real32_T c1_t_x;
  real32_T c1_u_x;
  real32_T c1_c_y;
  real32_T c1_v_x;
  real32_T c1_w_x;
  real32_T c1_d_A;
  real32_T c1_x_x;
  real32_T c1_y_x;
  real32_T c1_ab_x;
  real32_T c1_d_y;
  real32_T c1_bb_x;
  real32_T c1_cb_x;
  real32_T c1_e_A;
  real32_T c1_db_x;
  real32_T c1_eb_x;
  real32_T c1_fb_x;
  real32_T c1_e_y;
  real32_T c1_gb_x;
  real32_T c1_hb_x;
  real32_T c1_f_A;
  real32_T c1_ib_x;
  real32_T c1_jb_x;
  real32_T c1_kb_x;
  real32_T c1_f_y;
  real32_T c1_lb_x;
  real32_T c1_mb_x;
  real32_T c1_B;
  real32_T c1_g_y;
  real32_T c1_h_y;
  real32_T c1_i_y;
  int32_T c1_i24;
  real32_T c1_b_accel[3];
  int32_T c1_i25;
  real32_T c1_g_A[3];
  real32_T c1_b_B;
  real32_T c1_j_y;
  real32_T c1_k_y;
  real32_T c1_l_y;
  int32_T c1_i26;
  real32_T c1_h_A;
  real32_T c1_c_B;
  real32_T c1_nb_x;
  real32_T c1_m_y;
  real32_T c1_ob_x;
  real32_T c1_n_y;
  real32_T c1_pb_x;
  real32_T c1_o_y;
  real32_T c1_b_b;
  real32_T c1_p_y;
  real32_T c1_l_hoistedGlobal;
  real32_T c1_c_b;
  real32_T c1_q_y;
  real32_T c1_f0;
  real32_T c1_m_hoistedGlobal;
  real32_T c1_n_hoistedGlobal;
  real32_T c1_d_b;
  real32_T c1_r_y;
  real32_T c1_o_hoistedGlobal;
  real32_T c1_p_hoistedGlobal;
  real32_T c1_e_b;
  real32_T c1_s_y;
  int32_T c1_i27;
  real32_T c1_c_accel[3];
  int32_T c1_i28;
  real32_T c1_b_v[3];
  real32_T c1_f_b;
  int32_T c1_i29;
  real32_T c1_g_b;
  int32_T c1_i30;
  int32_T c1_i31;
  int32_T c1_i32;
  real32_T c1_b_mag[3];
  real32_T c1_d_B;
  real32_T c1_t_y;
  real32_T c1_u_y;
  real32_T c1_v_y;
  int32_T c1_i33;
  int32_T c1_i34;
  real32_T c1_fv6[4];
  real32_T c1_w_y[4];
  real32_T c1_fv7[4];
  int32_T c1_i35;
  int32_T c1_i36;
  real32_T c1_x_y[4];
  real_T c1_h_b[4];
  int32_T c1_i37;
  real32_T c1_fv8[4];
  int32_T c1_i38;
  real_T c1_i_b[4];
  real_T c1_dv0[4];
  int32_T c1_i39;
  real_T c1_b_h[2];
  real_T c1_d0;
  real32_T c1_q_hoistedGlobal;
  real32_T c1_r_hoistedGlobal;
  real_T c1_a;
  real32_T c1_j_b;
  real32_T c1_y_y;
  real32_T c1_s_hoistedGlobal;
  real32_T c1_t_hoistedGlobal;
  real_T c1_b_a;
  real32_T c1_k_b;
  real32_T c1_ab_y;
  real32_T c1_u_hoistedGlobal;
  real32_T c1_v_hoistedGlobal;
  real_T c1_c_a;
  real32_T c1_l_b;
  real32_T c1_bb_y;
  real32_T c1_w_hoistedGlobal;
  real32_T c1_x_hoistedGlobal;
  real_T c1_d_a;
  real32_T c1_m_b;
  real32_T c1_cb_y;
  real32_T c1_y_hoistedGlobal;
  real32_T c1_ab_hoistedGlobal;
  real_T c1_e_a;
  real32_T c1_n_b;
  real32_T c1_db_y;
  real32_T c1_bb_hoistedGlobal;
  real32_T c1_cb_hoistedGlobal;
  real_T c1_f_a;
  real32_T c1_o_b;
  real32_T c1_eb_y;
  int32_T c1_i40;
  real32_T c1_c_mag[3];
  int32_T c1_i41;
  real32_T c1_b_w[3];
  real32_T c1_p_b;
  int32_T c1_i42;
  int32_T c1_i43;
  int32_T c1_i44;
  real32_T c1_db_hoistedGlobal[4];
  int32_T c1_i45;
  real32_T c1_eb_hoistedGlobal[4];
  real32_T c1_fv9[4];
  int32_T c1_i46;
  int32_T c1_i47;
  int32_T c1_i48;
  real32_T c1_q_b;
  int32_T c1_i49;
  int32_T c1_i50;
  int32_T c1_i51;
  int32_T c1_i52;
  int32_T c1_i53;
  real32_T c1_fb_y[4];
  real32_T c1_e_B;
  real32_T c1_gb_y;
  real32_T c1_hb_y;
  real32_T c1_ib_y;
  int32_T c1_i54;
  int32_T c1_i55;
  uint32_T c1_u0;
  real32_T c1_fb_hoistedGlobal;
  real32_T c1_gb_hoistedGlobal;
  real32_T c1_r_b;
  real32_T c1_jb_y;
  real32_T c1_hb_hoistedGlobal;
  real32_T c1_ib_hoistedGlobal;
  real32_T c1_s_b;
  real32_T c1_f1;
  real32_T c1_jb_hoistedGlobal;
  real32_T c1_kb_hoistedGlobal;
  real32_T c1_t_b;
  real32_T c1_kb_y;
  int32_T c1_i56;
  int32_T c1_i57;
  int32_T c1_i58;
  uint8_T *c1_b_qValidOut;
  real32_T *c1_b_accelCutoff;
  real32_T *c1_b_kpMag;
  real32_T *c1_b_kpAcc;
  real32_T *c1_b_magVar;
  uint8_T *c1_b_magDataUpdate;
  uint8_T *c1_b_accelDataUpdate;
  real32_T *c1_b_accelOneG;
  uint8_T *c1_b_magValid;
  uint8_T *c1_b_accelValid;
  uint8_T *c1_b_gyroValid;
  real32_T *c1_b_dt;
  real32_T (*c1_b_attitude)[3];
  real32_T (*c1_b_qOut)[4];
  real32_T (*c1_b_attTrim)[2];
  real32_T (*c1_d_mag)[3];
  real32_T (*c1_d_accel)[3];
  real32_T (*c1_b_gyro)[3];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  c1_b_attTrim = (real32_T (*)[2])ssGetInputPortSignal(chartInstance->S, 14);
  c1_b_attitude = (real32_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c1_b_accelCutoff = (real32_T *)ssGetInputPortSignal(chartInstance->S, 13);
  c1_b_qValidOut = (uint8_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_b_kpMag = (real32_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c1_b_kpAcc = (real32_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c1_b_magVar = (real32_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c1_b_magDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c1_b_accelDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c1_b_accelOneG = (real32_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c1_b_magValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c1_b_accelValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c1_b_gyroValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_dt = (real32_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_d_mag = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c1_d_accel = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_qOut = (real32_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_gyro = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_dt;
  c1_b_hoistedGlobal = *c1_b_gyroValid;
  c1_c_hoistedGlobal = *c1_b_accelValid;
  c1_d_hoistedGlobal = *c1_b_magValid;
  c1_e_hoistedGlobal = *c1_b_accelOneG;
  c1_f_hoistedGlobal = *c1_b_accelDataUpdate;
  c1_g_hoistedGlobal = *c1_b_magDataUpdate;
  c1_h_hoistedGlobal = *c1_b_magVar;
  c1_i_hoistedGlobal = *c1_b_kpAcc;
  c1_j_hoistedGlobal = *c1_b_kpMag;
  c1_k_hoistedGlobal = *c1_b_accelCutoff;
  for (c1_i16 = 0; c1_i16 < 3; c1_i16++) {
    c1_gyro[c1_i16] = (*c1_b_gyro)[c1_i16];
  }

  for (c1_i17 = 0; c1_i17 < 3; c1_i17++) {
    c1_accel[c1_i17] = (*c1_d_accel)[c1_i17];
  }

  for (c1_i18 = 0; c1_i18 < 3; c1_i18++) {
    c1_mag[c1_i18] = (*c1_d_mag)[c1_i18];
  }

  c1_dt = c1_hoistedGlobal;
  c1_gyroValid = c1_b_hoistedGlobal;
  c1_accelValid = c1_c_hoistedGlobal;
  c1_magValid = c1_d_hoistedGlobal;
  c1_accelOneG = c1_e_hoistedGlobal;
  c1_accelDataUpdate = c1_f_hoistedGlobal;
  c1_magDataUpdate = c1_g_hoistedGlobal;
  c1_magVar = c1_h_hoistedGlobal;
  c1_kpAcc = c1_i_hoistedGlobal;
  c1_kpMag = c1_j_hoistedGlobal;
  c1_accelCutoff = c1_k_hoistedGlobal;
  for (c1_i19 = 0; c1_i19 < 2; c1_i19++) {
    c1_attTrim[c1_i19] = (*c1_b_attTrim)[c1_i19];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 63U, 63U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_initialRoll, 0U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_initialPitch, 1U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_cosRoll, 2U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_sinRoll, 3U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_cosPitch, 4U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_sinPitch, 5U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_magX, 6U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_magY, 7U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_initialHdg, 8U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u1, 9U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u2, 10U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u3, 11U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u4, 12U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u5, 13U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_u6, 14U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_accelNorm, 15U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_input, 16U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_output, 17U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_accelConfidence, 18U,
    c1_w_sf_marshallOut, c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_v, 19U, c1_x_sf_marshallOut,
    c1_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_h, 20U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b, 21U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_w, 22U, c1_x_sf_marshallOut,
    c1_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_qDot, 23U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_asinPitch, 24U, c1_w_sf_marshallOut,
    c1_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 25U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 26U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_gyro, 27U, c1_x_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_accel, 28U, c1_x_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_mag, 29U, c1_x_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_dt, 30U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_gyroValid, 31U, c1_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_accelValid, 32U, c1_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_magValid, 33U, c1_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_accelOneG, 34U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_accelDataUpdate, 35U, c1_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_magDataUpdate, 36U, c1_t_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_magVar, 37U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_kpAcc, 38U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_kpMag, 39U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_accelCutoff, 40U, c1_w_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_attTrim, 41U, c1_v_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_qOut, 42U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_qValidOut, 43U, c1_t_sf_marshallOut,
    c1_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_attitude, 44U, c1_s_sf_marshallOut,
    c1_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_accelConfidenceDecay,
    45U, c1_r_sf_marshallOut, c1_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_ahrsFirstPass, 46U,
    c1_q_sf_marshallOut, c1_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_eAcc, 47U,
    c1_p_sf_marshallOut, c1_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_eMag, 48U,
    c1_o_sf_marshallOut, c1_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_pastAccelFilter, 49U,
    c1_n_sf_marshallOut, c1_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c1_q, 50U,
    c1_m_sf_marshallOut, c1_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_qValid, 51U,
    c1_l_sf_marshallOut, c1_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_qValidCount, 52U,
    c1_k_sf_marshallOut, c1_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q1q1, 53U,
    c1_j_sf_marshallOut, c1_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q1q2, 54U,
    c1_i_sf_marshallOut, c1_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q1q3, 55U,
    c1_h_sf_marshallOut, c1_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q1q4, 56U,
    c1_g_sf_marshallOut, c1_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q2q2, 57U,
    c1_f_sf_marshallOut, c1_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q2q3, 58U,
    c1_e_sf_marshallOut, c1_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q2q4, 59U,
    c1_d_sf_marshallOut, c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q3q3, 60U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q3q4, 61U,
    c1_b_sf_marshallOut, c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c1_q4q4, 62U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 25);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c1_ahrsFirstPass_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
    chartInstance->c1_accelConfidenceDecay = 0.0F;
    chartInstance->c1_accelConfidenceDecay_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 27);
    chartInstance->c1_ahrsFirstPass = 1U;
    chartInstance->c1_ahrsFirstPass_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
    for (c1_i20 = 0; c1_i20 < 3; c1_i20++) {
      chartInstance->c1_eAcc[c1_i20] = 0.0F;
    }

    chartInstance->c1_eAcc_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
    for (c1_i21 = 0; c1_i21 < 3; c1_i21++) {
      chartInstance->c1_eMag[c1_i21] = 0.0F;
    }

    chartInstance->c1_eMag_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
    chartInstance->c1_pastAccelFilter = 1.0F;
    chartInstance->c1_pastAccelFilter_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
    for (c1_i22 = 0; c1_i22 < 4; c1_i22++) {
      chartInstance->c1_q[c1_i22] = c1_fv5[c1_i22];
    }

    chartInstance->c1_q_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
    chartInstance->c1_qValid = 0U;
    chartInstance->c1_qValid_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
    chartInstance->c1_qValidCount = 0U;
    chartInstance->c1_qValidCount_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
    chartInstance->c1_q1q1 = 1.0F;
    chartInstance->c1_q1q1_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 36);
    chartInstance->c1_q1q2 = 0.0F;
    chartInstance->c1_q1q2_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 37);
    chartInstance->c1_q1q3 = 0.0F;
    chartInstance->c1_q1q3_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
    chartInstance->c1_q1q4 = 0.0F;
    chartInstance->c1_q1q4_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
    chartInstance->c1_q2q2 = 0.0F;
    chartInstance->c1_q2q2_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 40);
    chartInstance->c1_q2q3 = 0.0F;
    chartInstance->c1_q2q3_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 41);
    chartInstance->c1_q2q4 = 0.0F;
    chartInstance->c1_q2q4_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
    chartInstance->c1_q3q3 = 0.0F;
    chartInstance->c1_q3q3_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
    chartInstance->c1_q3q4 = 0.0F;
    chartInstance->c1_q3q4_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
    chartInstance->c1_q4q4 = 0.0F;
    chartInstance->c1_q4q4_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  for (c1_i23 = 0; c1_i23 < 3; c1_i23++) {
    c1_attitude[c1_i23] = 0.0F;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
  guard2 = false;
  guard3 = false;
  guard4 = false;
  guard5 = false;
  guard6 = false;
  if (CV_EML_COND(0, 1, 0, chartInstance->c1_ahrsFirstPass != 0)) {
    if (CV_EML_COND(0, 1, 1, c1_accelValid != 0)) {
      if (CV_EML_COND(0, 1, 2, c1_accelDataUpdate != 0)) {
        if (CV_EML_COND(0, 1, 3, c1_gyroValid != 0)) {
          if (CV_EML_COND(0, 1, 4, c1_magValid != 0)) {
            if (CV_EML_COND(0, 1, 5, c1_magDataUpdate != 0)) {
              CV_EML_MCDC(0, 1, 0, true);
              CV_EML_IF(0, 1, 1, true);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 54);
              c1_initialRoll = c1_atan2(chartInstance, -c1_accel[1], -c1_accel[2]);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
              c1_initialPitch = c1_atan2(chartInstance, c1_accel[0], -c1_accel[2]);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 57);
              c1_x = c1_initialRoll;
              c1_cosRoll = c1_x;
              c1_b_x = c1_cosRoll;
              c1_cosRoll = c1_b_x;
              c1_cosRoll = muSingleScalarCos(c1_cosRoll);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 58);
              c1_c_x = c1_initialRoll;
              c1_sinRoll = c1_c_x;
              c1_d_x = c1_sinRoll;
              c1_sinRoll = c1_d_x;
              c1_sinRoll = muSingleScalarSin(c1_sinRoll);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 59);
              c1_e_x = c1_initialPitch;
              c1_cosPitch = c1_e_x;
              c1_f_x = c1_cosPitch;
              c1_cosPitch = c1_f_x;
              c1_cosPitch = muSingleScalarCos(c1_cosPitch);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 60);
              c1_g_x = c1_initialPitch;
              c1_sinPitch = c1_g_x;
              c1_h_x = c1_sinPitch;
              c1_sinPitch = c1_h_x;
              c1_sinPitch = muSingleScalarSin(c1_sinPitch);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 62);
              c1_magX = (c1_mag[0] * c1_cosPitch + c1_mag[1] * c1_sinRoll *
                         c1_sinPitch) + c1_mag[2] * c1_cosRoll * c1_sinPitch;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
              c1_magY = c1_mag[1] * c1_cosRoll - c1_mag[2] * c1_sinRoll;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 65);
              c1_initialHdg = c1_atan2(chartInstance, -c1_magY, c1_magX);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
              c1_A = c1_initialHdg;
              c1_i_x = c1_A;
              c1_j_x = c1_i_x;
              c1_k_x = c1_j_x;
              c1_y = c1_k_x / 2.0F;
              c1_l_x = c1_y;
              c1_u1 = c1_l_x;
              c1_m_x = c1_u1;
              c1_u1 = c1_m_x;
              c1_u1 = muSingleScalarSin(c1_u1);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 68);
              c1_b_A = c1_initialPitch;
              c1_n_x = c1_b_A;
              c1_o_x = c1_n_x;
              c1_p_x = c1_o_x;
              c1_b_y = c1_p_x / 2.0F;
              c1_q_x = c1_b_y;
              c1_u2 = c1_q_x;
              c1_r_x = c1_u2;
              c1_u2 = c1_r_x;
              c1_u2 = muSingleScalarSin(c1_u2);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 69);
              c1_c_A = c1_initialRoll;
              c1_s_x = c1_c_A;
              c1_t_x = c1_s_x;
              c1_u_x = c1_t_x;
              c1_c_y = c1_u_x / 2.0F;
              c1_v_x = c1_c_y;
              c1_u3 = c1_v_x;
              c1_w_x = c1_u3;
              c1_u3 = c1_w_x;
              c1_u3 = muSingleScalarSin(c1_u3);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 70);
              c1_d_A = c1_initialHdg;
              c1_x_x = c1_d_A;
              c1_y_x = c1_x_x;
              c1_ab_x = c1_y_x;
              c1_d_y = c1_ab_x / 2.0F;
              c1_bb_x = c1_d_y;
              c1_u4 = c1_bb_x;
              c1_cb_x = c1_u4;
              c1_u4 = c1_cb_x;
              c1_u4 = muSingleScalarCos(c1_u4);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 71);
              c1_e_A = c1_initialPitch;
              c1_db_x = c1_e_A;
              c1_eb_x = c1_db_x;
              c1_fb_x = c1_eb_x;
              c1_e_y = c1_fb_x / 2.0F;
              c1_gb_x = c1_e_y;
              c1_u5 = c1_gb_x;
              c1_hb_x = c1_u5;
              c1_u5 = c1_hb_x;
              c1_u5 = muSingleScalarCos(c1_u5);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 72);
              c1_f_A = c1_initialRoll;
              c1_ib_x = c1_f_A;
              c1_jb_x = c1_ib_x;
              c1_kb_x = c1_jb_x;
              c1_f_y = c1_kb_x / 2.0F;
              c1_lb_x = c1_f_y;
              c1_u6 = c1_lb_x;
              c1_mb_x = c1_u6;
              c1_u6 = c1_mb_x;
              c1_u6 = muSingleScalarCos(c1_u6);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 74);
              chartInstance->c1_q[0] = c1_u4 * c1_u5 * c1_u6 + c1_u1 * c1_u2 *
                c1_u3;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 75);
              chartInstance->c1_q[1] = c1_u4 * c1_u5 * c1_u3 - c1_u1 * c1_u2 *
                c1_u6;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 76);
              chartInstance->c1_q[2] = c1_u4 * c1_u2 * c1_u6 + c1_u1 * c1_u5 *
                c1_u3;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 77);
              chartInstance->c1_q[3] = c1_u1 * c1_u5 * c1_u6 - c1_u4 * c1_u2 *
                c1_u3;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 79);
              chartInstance->c1_q1q1 = chartInstance->c1_q[0] *
                chartInstance->c1_q[0];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 80);
              chartInstance->c1_q1q2 = chartInstance->c1_q[0] *
                chartInstance->c1_q[1];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 81);
              chartInstance->c1_q1q3 = chartInstance->c1_q[0] *
                chartInstance->c1_q[2];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 82);
              chartInstance->c1_q1q4 = chartInstance->c1_q[0] *
                chartInstance->c1_q[3];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 83);
              chartInstance->c1_q2q2 = chartInstance->c1_q[1] *
                chartInstance->c1_q[1];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 84);
              chartInstance->c1_q2q3 = chartInstance->c1_q[1] *
                chartInstance->c1_q[2];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 85);
              chartInstance->c1_q2q4 = chartInstance->c1_q[1] *
                chartInstance->c1_q[3];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 86);
              chartInstance->c1_q3q3 = chartInstance->c1_q[2] *
                chartInstance->c1_q[2];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 87);
              chartInstance->c1_q3q4 = chartInstance->c1_q[2] *
                chartInstance->c1_q[3];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 88);
              chartInstance->c1_q4q4 = chartInstance->c1_q[3] *
                chartInstance->c1_q[3];
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 90);
              c1_B = c1_accelCutoff;
              c1_b_sqrt(chartInstance, &c1_B);
              c1_g_y = c1_B;
              c1_h_y = c1_g_y;
              c1_i_y = c1_h_y;
              chartInstance->c1_accelConfidenceDecay = 1.0F / c1_i_y;
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 92);
              chartInstance->c1_ahrsFirstPass = 0U;
            } else {
              guard2 = true;
            }
          } else {
            guard3 = true;
          }
        } else {
          guard4 = true;
        }
      } else {
        guard5 = true;
      }
    } else {
      guard6 = true;
    }
  } else {
    guard6 = true;
  }

  if (guard6 == true) {
    guard5 = true;
  }

  if (guard5 == true) {
    guard4 = true;
  }

  if (guard4 == true) {
    guard3 = true;
  }

  if (guard3 == true) {
    guard2 = true;
  }

  if (guard2 == true) {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 1, false);
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 97);
  if (CV_EML_IF(0, 1, 2, CV_EML_MCDC(0, 1, 1, !CV_EML_COND(0, 1, 6,
         chartInstance->c1_ahrsFirstPass != 0)))) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 98);
    if (CV_EML_IF(0, 1, 3, c1_accelDataUpdate != 0)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 100);
      for (c1_i24 = 0; c1_i24 < 3; c1_i24++) {
        c1_b_accel[c1_i24] = c1_accel[c1_i24];
      }

      c1_accelNorm = c1_norm(chartInstance, c1_b_accel);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 102);
      for (c1_i25 = 0; c1_i25 < 3; c1_i25++) {
        c1_g_A[c1_i25] = c1_accel[c1_i25];
      }

      c1_b_B = c1_accelNorm;
      c1_j_y = c1_b_B;
      c1_k_y = c1_j_y;
      c1_l_y = c1_k_y;
      for (c1_i26 = 0; c1_i26 < 3; c1_i26++) {
        c1_accel[c1_i26] = c1_g_A[c1_i26] / c1_l_y;
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 104);
      c1_h_A = c1_accelNorm;
      c1_c_B = c1_accelOneG;
      c1_nb_x = c1_h_A;
      c1_m_y = c1_c_B;
      c1_ob_x = c1_nb_x;
      c1_n_y = c1_m_y;
      c1_pb_x = c1_ob_x;
      c1_o_y = c1_n_y;
      c1_input = c1_pb_x / c1_o_y;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 105);
      c1_b_b = c1_input;
      c1_p_y = 0.1F * c1_b_b;
      c1_l_hoistedGlobal = chartInstance->c1_pastAccelFilter;
      c1_c_b = c1_l_hoistedGlobal;
      c1_q_y = 0.9F * c1_c_b;
      c1_output = c1_p_y + c1_q_y;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 106);
      chartInstance->c1_pastAccelFilter = c1_output;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 107);
      c1_f0 = c1_abs(chartInstance, c1_output - 1.0F);
      c1_b_sqrt(chartInstance, &c1_f0);
      c1_accelConfidence = 1.0F - chartInstance->c1_accelConfidenceDecay * c1_f0;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 109);
      if (CV_EML_IF(0, 1, 4, c1_accelConfidence > 1.0F)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 110);
        c1_accelConfidence = 1.0F;
      } else {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 111);
        if (CV_EML_IF(0, 1, 5, c1_accelConfidence < 0.0F)) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 112);
          c1_accelConfidence = 0.0F;
        }
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 116);
      c1_m_hoistedGlobal = chartInstance->c1_q2q4;
      c1_n_hoistedGlobal = chartInstance->c1_q1q3;
      c1_d_b = c1_m_hoistedGlobal - c1_n_hoistedGlobal;
      c1_r_y = 2.0F * c1_d_b;
      c1_o_hoistedGlobal = chartInstance->c1_q1q2;
      c1_p_hoistedGlobal = chartInstance->c1_q3q4;
      c1_e_b = c1_o_hoistedGlobal + c1_p_hoistedGlobal;
      c1_s_y = 2.0F * c1_e_b;
      c1_v[0] = c1_r_y;
      c1_v[1] = c1_s_y;
      c1_v[2] = ((chartInstance->c1_q1q1 - chartInstance->c1_q2q2) -
                 chartInstance->c1_q3q3) + chartInstance->c1_q4q4;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 121);
      for (c1_i27 = 0; c1_i27 < 3; c1_i27++) {
        c1_c_accel[c1_i27] = -c1_accel[c1_i27];
      }

      for (c1_i28 = 0; c1_i28 < 3; c1_i28++) {
        c1_b_v[c1_i28] = c1_v[c1_i28];
      }

      c1_cross(chartInstance, c1_c_accel, c1_b_v, c1_g_A);
      c1_f_b = c1_kpAcc;
      for (c1_i29 = 0; c1_i29 < 3; c1_i29++) {
        c1_g_A[c1_i29] *= c1_f_b;
      }

      c1_g_b = c1_accelConfidence;
      for (c1_i30 = 0; c1_i30 < 3; c1_i30++) {
        chartInstance->c1_eAcc[c1_i30] = c1_g_A[c1_i30] * c1_g_b;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 126);
    if (CV_EML_IF(0, 1, 6, c1_magDataUpdate != 0)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 129U);
      for (c1_i31 = 0; c1_i31 < 3; c1_i31++) {
        c1_g_A[c1_i31] = c1_mag[c1_i31];
      }

      for (c1_i32 = 0; c1_i32 < 3; c1_i32++) {
        c1_b_mag[c1_i32] = c1_mag[c1_i32];
      }

      c1_d_B = c1_norm(chartInstance, c1_b_mag);
      c1_t_y = c1_d_B;
      c1_u_y = c1_t_y;
      c1_v_y = c1_u_y;
      for (c1_i33 = 0; c1_i33 < 3; c1_i33++) {
        c1_mag[c1_i33] = c1_g_A[c1_i33] / c1_v_y;
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 132U);
      for (c1_i34 = 0; c1_i34 < 4; c1_i34++) {
        c1_fv6[c1_i34] = chartInstance->c1_q[c1_i34];
      }

      c1_quaternConj(chartInstance, c1_fv6, c1_w_y);
      c1_fv7[0] = 0.0F;
      for (c1_i35 = 0; c1_i35 < 3; c1_i35++) {
        c1_fv7[c1_i35 + 1] = c1_mag[c1_i35];
      }

      for (c1_i36 = 0; c1_i36 < 4; c1_i36++) {
        c1_x_y[c1_i36] = c1_w_y[c1_i36];
      }

      c1_quaternProd(chartInstance, c1_fv7, c1_x_y, c1_h_b);
      for (c1_i37 = 0; c1_i37 < 4; c1_i37++) {
        c1_fv8[c1_i37] = chartInstance->c1_q[c1_i37];
      }

      for (c1_i38 = 0; c1_i38 < 4; c1_i38++) {
        c1_i_b[c1_i38] = c1_h_b[c1_i38];
      }

      c1_b_quaternProd(chartInstance, c1_fv8, c1_i_b, c1_dv0);
      for (c1_i39 = 0; c1_i39 < 4; c1_i39++) {
        c1_h[c1_i39] = c1_dv0[c1_i39];
      }

      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 133U);
      c1_b_h[0] = c1_h[1];
      c1_b_h[1] = c1_h[2];
      c1_d0 = c1_b_norm(chartInstance, c1_b_h);
      c1_b[0] = 0.0;
      c1_b[1] = c1_d0;
      c1_b[2] = 0.0;
      c1_b[3] = c1_h[3];
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 135U);
      c1_q_hoistedGlobal = chartInstance->c1_q3q3;
      c1_r_hoistedGlobal = chartInstance->c1_q4q4;
      c1_a = 2.0 * c1_b[1];
      c1_j_b = (0.5F - c1_q_hoistedGlobal) - c1_r_hoistedGlobal;
      c1_y_y = (real32_T)c1_a * c1_j_b;
      c1_s_hoistedGlobal = chartInstance->c1_q2q4;
      c1_t_hoistedGlobal = chartInstance->c1_q1q3;
      c1_b_a = 2.0 * c1_b[3];
      c1_k_b = c1_s_hoistedGlobal - c1_t_hoistedGlobal;
      c1_ab_y = (real32_T)c1_b_a * c1_k_b;
      c1_u_hoistedGlobal = chartInstance->c1_q2q3;
      c1_v_hoistedGlobal = chartInstance->c1_q1q4;
      c1_c_a = 2.0 * c1_b[1];
      c1_l_b = c1_u_hoistedGlobal - c1_v_hoistedGlobal;
      c1_bb_y = (real32_T)c1_c_a * c1_l_b;
      c1_w_hoistedGlobal = chartInstance->c1_q1q2;
      c1_x_hoistedGlobal = chartInstance->c1_q3q4;
      c1_d_a = 2.0 * c1_b[3];
      c1_m_b = c1_w_hoistedGlobal + c1_x_hoistedGlobal;
      c1_cb_y = (real32_T)c1_d_a * c1_m_b;
      c1_y_hoistedGlobal = chartInstance->c1_q1q3;
      c1_ab_hoistedGlobal = chartInstance->c1_q2q4;
      c1_e_a = 2.0 * c1_b[1];
      c1_n_b = c1_y_hoistedGlobal + c1_ab_hoistedGlobal;
      c1_db_y = (real32_T)c1_e_a * c1_n_b;
      c1_bb_hoistedGlobal = chartInstance->c1_q2q2;
      c1_cb_hoistedGlobal = chartInstance->c1_q3q3;
      c1_f_a = 2.0 * c1_b[3];
      c1_o_b = (0.5F - c1_bb_hoistedGlobal) - c1_cb_hoistedGlobal;
      c1_eb_y = (real32_T)c1_f_a * c1_o_b;
      c1_w[0] = c1_y_y + c1_ab_y;
      c1_w[1] = c1_bb_y + c1_cb_y;
      c1_w[2] = c1_db_y + c1_eb_y;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 140U);
      for (c1_i40 = 0; c1_i40 < 3; c1_i40++) {
        c1_c_mag[c1_i40] = c1_mag[c1_i40];
      }

      for (c1_i41 = 0; c1_i41 < 3; c1_i41++) {
        c1_b_w[c1_i41] = c1_w[c1_i41];
      }

      c1_cross(chartInstance, c1_c_mag, c1_b_w, c1_g_A);
      c1_p_b = c1_kpMag;
      for (c1_i42 = 0; c1_i42 < 3; c1_i42++) {
        chartInstance->c1_eMag[c1_i42] = c1_g_A[c1_i42] * c1_p_b;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 146U);
    for (c1_i43 = 0; c1_i43 < 3; c1_i43++) {
      c1_gyro[c1_i43] = (c1_gyro[c1_i43] + chartInstance->c1_eAcc[c1_i43]) +
        chartInstance->c1_eMag[c1_i43];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 149U);
    for (c1_i44 = 0; c1_i44 < 4; c1_i44++) {
      c1_db_hoistedGlobal[c1_i44] = chartInstance->c1_q[c1_i44];
    }

    for (c1_i45 = 0; c1_i45 < 4; c1_i45++) {
      c1_eb_hoistedGlobal[c1_i45] = c1_db_hoistedGlobal[c1_i45];
    }

    c1_fv9[0] = 0.0F;
    c1_fv9[1] = c1_gyro[0];
    c1_fv9[2] = c1_gyro[1];
    c1_fv9[3] = c1_gyro[2];
    c1_quaternProd(chartInstance, c1_eb_hoistedGlobal, c1_fv9, c1_h_b);
    for (c1_i46 = 0; c1_i46 < 4; c1_i46++) {
      c1_qDot[c1_i46] = 0.5 * c1_h_b[c1_i46];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 152U);
    for (c1_i47 = 0; c1_i47 < 4; c1_i47++) {
      c1_db_hoistedGlobal[c1_i47] = chartInstance->c1_q[c1_i47];
    }

    for (c1_i48 = 0; c1_i48 < 4; c1_i48++) {
      c1_h_b[c1_i48] = c1_qDot[c1_i48];
    }

    c1_q_b = c1_dt;
    for (c1_i49 = 0; c1_i49 < 4; c1_i49++) {
      c1_w_y[c1_i49] = (real32_T)c1_h_b[c1_i49] * c1_q_b;
    }

    for (c1_i50 = 0; c1_i50 < 4; c1_i50++) {
      chartInstance->c1_q[c1_i50] = c1_db_hoistedGlobal[c1_i50] + c1_w_y[c1_i50];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 153U);
    for (c1_i51 = 0; c1_i51 < 4; c1_i51++) {
      c1_db_hoistedGlobal[c1_i51] = chartInstance->c1_q[c1_i51];
    }

    for (c1_i52 = 0; c1_i52 < 4; c1_i52++) {
      c1_w_y[c1_i52] = chartInstance->c1_q[c1_i52];
    }

    for (c1_i53 = 0; c1_i53 < 4; c1_i53++) {
      c1_fb_y[c1_i53] = c1_w_y[c1_i53];
    }

    c1_e_B = c1_c_norm(chartInstance, c1_fb_y);
    c1_gb_y = c1_e_B;
    c1_hb_y = c1_gb_y;
    c1_ib_y = c1_hb_y;
    for (c1_i54 = 0; c1_i54 < 4; c1_i54++) {
      chartInstance->c1_q[c1_i54] = c1_db_hoistedGlobal[c1_i54] / c1_ib_y;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 155U);
    for (c1_i55 = 0; c1_i55 < 4; c1_i55++) {
      c1_qOut[c1_i55] = chartInstance->c1_q[c1_i55];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 157U);
    if (CV_EML_IF(0, 1, 7, (real_T)chartInstance->c1_qValid == 0.0)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 158U);
      c1_u0 = (uint32_T)chartInstance->c1_qValidCount + 1U;
      if (CV_SATURATION_EVAL(4, 0, 0, 0, c1_u0 > 255U)) {
        c1_u0 = 255U;
      }

      chartInstance->c1_qValidCount = (uint8_T)c1_u0;
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 161U);
    guard1 = false;
    if (CV_EML_COND(0, 1, 7, (real_T)chartInstance->c1_qValidCount >= 20.0)) {
      if (CV_EML_COND(0, 1, 8, (real_T)chartInstance->c1_qValid == 0.0)) {
        CV_EML_MCDC(0, 1, 2, true);
        CV_EML_IF(0, 1, 8, true);
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 162U);
        chartInstance->c1_qValid = 1U;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 == true) {
      CV_EML_MCDC(0, 1, 2, false);
      CV_EML_IF(0, 1, 8, false);
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 165U);
    c1_qValidOut = chartInstance->c1_qValid;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 167U);
    chartInstance->c1_q1q1 = chartInstance->c1_q[0] * chartInstance->c1_q[0];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 168U);
    chartInstance->c1_q1q2 = chartInstance->c1_q[0] * chartInstance->c1_q[1];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 169U);
    chartInstance->c1_q1q3 = chartInstance->c1_q[0] * chartInstance->c1_q[2];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 170U);
    chartInstance->c1_q1q4 = chartInstance->c1_q[0] * chartInstance->c1_q[3];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 171U);
    chartInstance->c1_q2q2 = chartInstance->c1_q[1] * chartInstance->c1_q[1];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 172U);
    chartInstance->c1_q2q3 = chartInstance->c1_q[1] * chartInstance->c1_q[2];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 173U);
    chartInstance->c1_q2q4 = chartInstance->c1_q[1] * chartInstance->c1_q[3];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 174U);
    chartInstance->c1_q3q3 = chartInstance->c1_q[2] * chartInstance->c1_q[2];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 175U);
    chartInstance->c1_q3q4 = chartInstance->c1_q[2] * chartInstance->c1_q[3];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 176U);
    chartInstance->c1_q4q4 = chartInstance->c1_q[3] * chartInstance->c1_q[3];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 180U);
    c1_fb_hoistedGlobal = chartInstance->c1_q2q3;
    c1_gb_hoistedGlobal = chartInstance->c1_q1q4;
    c1_r_b = c1_fb_hoistedGlobal + c1_gb_hoistedGlobal;
    c1_jb_y = 2.0F * c1_r_b;
    c1_attitude[2] = c1_atan2(chartInstance, c1_jb_y, ((chartInstance->c1_q1q1 +
      chartInstance->c1_q2q2) - chartInstance->c1_q3q3) - chartInstance->c1_q4q4)
      + c1_magVar;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 182U);
    c1_hb_hoistedGlobal = chartInstance->c1_q2q4;
    c1_ib_hoistedGlobal = chartInstance->c1_q1q3;
    c1_s_b = c1_hb_hoistedGlobal - c1_ib_hoistedGlobal;
    c1_asinPitch = -2.0F * c1_s_b;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 184U);
    if (CV_EML_IF(0, 1, 9, c1_asinPitch > 1.0F)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 185U);
      c1_asinPitch = 1.0F;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 186U);
      if (CV_EML_IF(0, 1, 10, c1_asinPitch < -1.0F)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 187U);
        c1_asinPitch = -1.0F;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 190U);
    c1_f1 = c1_asinPitch;
    c1_b_asin(chartInstance, &c1_f1);
    c1_attitude[1] = c1_f1 - c1_attTrim[1];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 192U);
    c1_jb_hoistedGlobal = chartInstance->c1_q3q4;
    c1_kb_hoistedGlobal = chartInstance->c1_q1q2;
    c1_t_b = c1_jb_hoistedGlobal + c1_kb_hoistedGlobal;
    c1_kb_y = 2.0F * c1_t_b;
    c1_attitude[0] = c1_atan2(chartInstance, c1_kb_y, ((chartInstance->c1_q1q1 -
      chartInstance->c1_q2q2) - chartInstance->c1_q3q3) + chartInstance->c1_q4q4)
      - c1_attTrim[0];
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 196U);
    if (CV_EML_IF(0, 1, 11, (real_T)c1_attitude[0] >= 3.1415926535897931)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 197U);
      c1_attitude[0] -= 6.28318548F;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 198U);
      if (CV_EML_IF(0, 1, 12, (real_T)c1_attitude[0] < -3.1415926535897931)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 199U);
        c1_attitude[0] += 6.28318548F;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 202U);
    if (CV_EML_IF(0, 1, 13, (real_T)c1_attitude[1] >= 3.1415926535897931)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 203U);
      c1_attitude[1] -= 6.28318548F;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 204U);
      if (CV_EML_IF(0, 1, 14, (real_T)c1_attitude[1] < -3.1415926535897931)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 205U);
        c1_attitude[1] += 6.28318548F;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 208U);
    if (CV_EML_IF(0, 1, 15, (real_T)c1_attitude[2] >= 3.1415926535897931)) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 209U);
      c1_attitude[2] -= 6.28318548F;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 210U);
      if (CV_EML_IF(0, 1, 16, (real_T)c1_attitude[2] < -3.1415926535897931)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 211U);
        c1_attitude[2] += 6.28318548F;
      }
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 216U);
    for (c1_i56 = 0; c1_i56 < 4; c1_i56++) {
      c1_qOut[c1_i56] = c1_fv5[c1_i56];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 218U);
    c1_qValidOut = 0U;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 220U);
    chartInstance->c1_q1q1 = 1.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 221U);
    chartInstance->c1_q1q2 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 222U);
    chartInstance->c1_q1q3 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 223U);
    chartInstance->c1_q1q4 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 224U);
    chartInstance->c1_q2q2 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 225U);
    chartInstance->c1_q2q3 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 226U);
    chartInstance->c1_q2q4 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 227U);
    chartInstance->c1_q3q3 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 228U);
    chartInstance->c1_q3q4 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 229U);
    chartInstance->c1_q4q4 = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 231U);
    c1_attitude[2] = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 232U);
    c1_attitude[1] = 0.0F;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 233U);
    c1_attitude[0] = 0.0F;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -233);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_i57 = 0; c1_i57 < 4; c1_i57++) {
    (*c1_b_qOut)[c1_i57] = c1_qOut[c1_i57];
  }

  *c1_b_qValidOut = c1_qValidOut;
  for (c1_i58 = 0; c1_i58 < 3; c1_i58++) {
    (*c1_b_attitude)[c1_i58] = c1_attitude[c1_i58];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_mahoneyAHRSlibrary
  (SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q4q4_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q4q4, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q4q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q4q4);
  return c1_y;
}

static real32_T c1_b_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f2;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q4q4_not_empty = false;
  } else {
    chartInstance->c1_q4q4_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f2, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f2;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q4q4;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q4q4 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q4q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q4q4);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q3q4_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_c_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q3q4, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q3q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q3q4);
  return c1_y;
}

static real32_T c1_d_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f3;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q3q4_not_empty = false;
  } else {
    chartInstance->c1_q3q4_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f3, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f3;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q3q4;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q3q4 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q3q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q3q4);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q3q3_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_e_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q3q3, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q3q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q3q3);
  return c1_y;
}

static real32_T c1_f_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f4;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q3q3_not_empty = false;
  } else {
    chartInstance->c1_q3q3_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f4, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f4;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q3q3;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q3q3 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q3q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q3q3);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q2q4_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_g_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q4, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q4);
  return c1_y;
}

static real32_T c1_h_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f5;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q2q4_not_empty = false;
  } else {
    chartInstance->c1_q2q4_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f5, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f5;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q2q4;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q2q4 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q4);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q2q3_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_i_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q3, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q3);
  return c1_y;
}

static real32_T c1_j_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f6;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q2q3_not_empty = false;
  } else {
    chartInstance->c1_q2q3_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f6, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f6;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q2q3;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q2q3 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q3);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q2q2_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_k_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q2q2, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q2), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q2);
  return c1_y;
}

static real32_T c1_l_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f7;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q2q2_not_empty = false;
  } else {
    chartInstance->c1_q2q2_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f7, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f7;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q2q2;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q2q2 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q2q2), &c1_thisId);
  sf_mex_destroy(&c1_b_q2q2);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q1q4_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_m_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q4, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q4);
  return c1_y;
}

static real32_T c1_n_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f8;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q1q4_not_empty = false;
  } else {
    chartInstance->c1_q1q4_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f8, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f8;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q1q4;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q1q4 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q4), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q4);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_h_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q1q3_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_o_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q3, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q3);
  return c1_y;
}

static real32_T c1_p_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f9;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q1q3_not_empty = false;
  } else {
    chartInstance->c1_q1q3_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f9, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f9;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q1q3;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q1q3 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q3), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q3);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_i_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q1q2_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_q_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q2, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q2), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q2);
  return c1_y;
}

static real32_T c1_r_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f10;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q1q2_not_empty = false;
  } else {
    chartInstance->c1_q1q2_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f10, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f10;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q1q2;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q1q2 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q2), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q2);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_j_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_q1q1_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_s_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q1q1, const char_T *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q1), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q1);
  return c1_y;
}

static real32_T c1_t_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f11;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q1q1_not_empty = false;
  } else {
    chartInstance->c1_q1q1_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f11, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f11;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q1q1;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q1q1 = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q1q1), &c1_thisId);
  sf_mex_destroy(&c1_b_q1q1);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_k_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_qValidCount_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_u_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_qValidCount, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_qValidCount),
    &c1_thisId);
  sf_mex_destroy(&c1_b_qValidCount);
  return c1_y;
}

static uint8_T c1_v_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u1;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_qValidCount_not_empty = false;
  } else {
    chartInstance->c1_qValidCount_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u1, 1, 3, 0U, 0, 0U, 0);
    c1_y = c1_u1;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_qValidCount;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_qValidCount = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_qValidCount),
    &c1_thisId);
  sf_mex_destroy(&c1_b_qValidCount);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_l_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_qValid_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_w_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_qValid, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_qValid),
    &c1_thisId);
  sf_mex_destroy(&c1_b_qValid);
  return c1_y;
}

static uint8_T c1_x_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u2;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_qValid_not_empty = false;
  } else {
    chartInstance->c1_qValid_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u2, 1, 3, 0U, 0, 0U, 0);
    c1_y = c1_u2;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_qValid;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_qValid = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_qValid),
    &c1_thisId);
  sf_mex_destroy(&c1_b_qValid);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_m_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i59;
  real32_T c1_b_inData[4];
  int32_T c1_i60;
  real32_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i59 = 0; c1_i59 < 4; c1_i59++) {
    c1_b_inData[c1_i59] = (*(real32_T (*)[4])c1_inData)[c1_i59];
  }

  for (c1_i60 = 0; c1_i60 < 4; c1_i60++) {
    c1_u[c1_i60] = c1_b_inData[c1_i60];
  }

  c1_y = NULL;
  if (!chartInstance->c1_q_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 2, 1, 4), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_y_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_q, const char_T *c1_identifier, real32_T
  c1_y[4])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_q);
}

static void c1_ab_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[4])
{
  real32_T c1_fv10[4];
  int32_T c1_i61;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q_not_empty = false;
  } else {
    chartInstance->c1_q_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv10, 1, 1, 0U, 1, 0U, 2, 1,
                  4);
    for (c1_i61 = 0; c1_i61 < 4; c1_i61++) {
      c1_y[c1_i61] = c1_fv10[c1_i61];
    }
  }

  sf_mex_destroy(&c1_u);
}

static void c1_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[4];
  int32_T c1_i62;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_q = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_q);
  for (c1_i62 = 0; c1_i62 < 4; c1_i62++) {
    (*(real32_T (*)[4])c1_outData)[c1_i62] = c1_y[c1_i62];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_n_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_pastAccelFilter_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_bb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_pastAccelFilter, const char_T
  *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_pastAccelFilter),
    &c1_thisId);
  sf_mex_destroy(&c1_b_pastAccelFilter);
  return c1_y;
}

static real32_T c1_cb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f12;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_pastAccelFilter_not_empty = false;
  } else {
    chartInstance->c1_pastAccelFilter_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f12, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f12;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_pastAccelFilter;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_pastAccelFilter = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_pastAccelFilter),
    &c1_thisId);
  sf_mex_destroy(&c1_b_pastAccelFilter);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_o_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i63;
  real32_T c1_b_inData[3];
  int32_T c1_i64;
  real32_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i63 = 0; c1_i63 < 3; c1_i63++) {
    c1_b_inData[c1_i63] = (*(real32_T (*)[3])c1_inData)[c1_i63];
  }

  for (c1_i64 = 0; c1_i64 < 3; c1_i64++) {
    c1_u[c1_i64] = c1_b_inData[c1_i64];
  }

  c1_y = NULL;
  if (!chartInstance->c1_eMag_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 1, 3), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_db_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_eMag, const char_T *c1_identifier,
  real32_T c1_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_eMag), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_eMag);
}

static void c1_eb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3])
{
  real32_T c1_fv11[3];
  int32_T c1_i65;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_eMag_not_empty = false;
  } else {
    chartInstance->c1_eMag_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv11, 1, 1, 0U, 1, 0U, 1, 3);
    for (c1_i65 = 0; c1_i65 < 3; c1_i65++) {
      c1_y[c1_i65] = c1_fv11[c1_i65];
    }
  }

  sf_mex_destroy(&c1_u);
}

static void c1_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_eMag;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[3];
  int32_T c1_i66;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_eMag = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_eMag), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_eMag);
  for (c1_i66 = 0; c1_i66 < 3; c1_i66++) {
    (*(real32_T (*)[3])c1_outData)[c1_i66] = c1_y[c1_i66];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_p_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i67;
  real32_T c1_b_inData[3];
  int32_T c1_i68;
  real32_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i67 = 0; c1_i67 < 3; c1_i67++) {
    c1_b_inData[c1_i67] = (*(real32_T (*)[3])c1_inData)[c1_i67];
  }

  for (c1_i68 = 0; c1_i68 < 3; c1_i68++) {
    c1_u[c1_i68] = c1_b_inData[c1_i68];
  }

  c1_y = NULL;
  if (!chartInstance->c1_eAcc_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 1, 3), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_fb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_eAcc, const char_T *c1_identifier,
  real32_T c1_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_eAcc), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_eAcc);
}

static void c1_gb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3])
{
  real32_T c1_fv12[3];
  int32_T c1_i69;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_eAcc_not_empty = false;
  } else {
    chartInstance->c1_eAcc_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv12, 1, 1, 0U, 1, 0U, 1, 3);
    for (c1_i69 = 0; c1_i69 < 3; c1_i69++) {
      c1_y[c1_i69] = c1_fv12[c1_i69];
    }
  }

  sf_mex_destroy(&c1_u);
}

static void c1_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_eAcc;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[3];
  int32_T c1_i70;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_eAcc = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_eAcc), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_eAcc);
  for (c1_i70 = 0; c1_i70 < 3; c1_i70++) {
    (*(real32_T (*)[3])c1_outData)[c1_i70] = c1_y[c1_i70];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_q_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_ahrsFirstPass_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_hb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_ahrsFirstPass, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_ahrsFirstPass),
    &c1_thisId);
  sf_mex_destroy(&c1_b_ahrsFirstPass);
  return c1_y;
}

static uint8_T c1_ib_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u3;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_ahrsFirstPass_not_empty = false;
  } else {
    chartInstance->c1_ahrsFirstPass_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u3, 1, 3, 0U, 0, 0U, 0);
    c1_y = c1_u3;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_ahrsFirstPass;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_ahrsFirstPass = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_ahrsFirstPass),
    &c1_thisId);
  sf_mex_destroy(&c1_b_ahrsFirstPass);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_r_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_accelConfidenceDecay_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real32_T c1_jb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_b_accelConfidenceDecay, const char_T
  *c1_identifier)
{
  real32_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_kb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_accelConfidenceDecay), &c1_thisId);
  sf_mex_destroy(&c1_b_accelConfidenceDecay);
  return c1_y;
}

static real32_T c1_kb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f13;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_accelConfidenceDecay_not_empty = false;
  } else {
    chartInstance->c1_accelConfidenceDecay_not_empty = true;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f13, 1, 1, 0U, 0, 0U, 0);
    c1_y = c1_f13;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_accelConfidenceDecay;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_accelConfidenceDecay = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_kb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_accelConfidenceDecay), &c1_thisId);
  sf_mex_destroy(&c1_b_accelConfidenceDecay);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_s_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i71;
  real32_T c1_b_inData[3];
  int32_T c1_i72;
  real32_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i71 = 0; c1_i71 < 3; c1_i71++) {
    c1_b_inData[c1_i71] = (*(real32_T (*)[3])c1_inData)[c1_i71];
  }

  for (c1_i72 = 0; c1_i72 < 3; c1_i72++) {
    c1_u[c1_i72] = c1_b_inData[c1_i72];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_lb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_attitude, const char_T *c1_identifier,
  real32_T c1_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_attitude), &c1_thisId,
    c1_y);
  sf_mex_destroy(&c1_attitude);
}

static void c1_mb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3])
{
  real32_T c1_fv13[3];
  int32_T c1_i73;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv13, 1, 1, 0U, 1, 0U, 2, 1, 3);
  for (c1_i73 = 0; c1_i73 < 3; c1_i73++) {
    c1_y[c1_i73] = c1_fv13[c1_i73];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_attitude;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[3];
  int32_T c1_i74;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_attitude = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_attitude), &c1_thisId,
    c1_y);
  sf_mex_destroy(&c1_attitude);
  for (c1_i74 = 0; c1_i74 < 3; c1_i74++) {
    (*(real32_T (*)[3])c1_outData)[c1_i74] = c1_y[c1_i74];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_t_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  uint8_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(uint8_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static uint8_T c1_nb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_qValidOut, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_qValidOut),
    &c1_thisId);
  sf_mex_destroy(&c1_qValidOut);
  return c1_y;
}

static uint8_T c1_ob_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u4;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u4, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u4;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_qValidOut;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_qValidOut = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_qValidOut),
    &c1_thisId);
  sf_mex_destroy(&c1_qValidOut);
  *(uint8_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_u_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i75;
  real32_T c1_b_inData[4];
  int32_T c1_i76;
  real32_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i75 = 0; c1_i75 < 4; c1_i75++) {
    c1_b_inData[c1_i75] = (*(real32_T (*)[4])c1_inData)[c1_i75];
  }

  for (c1_i76 = 0; c1_i76 < 4; c1_i76++) {
    c1_u[c1_i76] = c1_b_inData[c1_i76];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_pb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_qOut, const char_T *c1_identifier, real32_T
  c1_y[4])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_qb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_qOut), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_qOut);
}

static void c1_qb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[4])
{
  real32_T c1_fv14[4];
  int32_T c1_i77;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv14, 1, 1, 0U, 1, 0U, 2, 1, 4);
  for (c1_i77 = 0; c1_i77 < 4; c1_i77++) {
    c1_y[c1_i77] = c1_fv14[c1_i77];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_qOut;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[4];
  int32_T c1_i78;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_qOut = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_qb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_qOut), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_qOut);
  for (c1_i78 = 0; c1_i78 < 4; c1_i78++) {
    (*(real32_T (*)[4])c1_outData)[c1_i78] = c1_y[c1_i78];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_v_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i79;
  real32_T c1_b_inData[2];
  int32_T c1_i80;
  real32_T c1_u[2];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i79 = 0; c1_i79 < 2; c1_i79++) {
    c1_b_inData[c1_i79] = (*(real32_T (*)[2])c1_inData)[c1_i79];
  }

  for (c1_i80 = 0; c1_i80 < 2; c1_i80++) {
    c1_u[c1_i80] = c1_b_inData[c1_i80];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_w_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_x_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i81;
  real32_T c1_b_inData[3];
  int32_T c1_i82;
  real32_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i81 = 0; c1_i81 < 3; c1_i81++) {
    c1_b_inData[c1_i81] = (*(real32_T (*)[3])c1_inData)[c1_i81];
  }

  for (c1_i82 = 0; c1_i82 < 3; c1_i82++) {
    c1_u[c1_i82] = c1_b_inData[c1_i82];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 1, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_y_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_rb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_rb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout),
    &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static real32_T c1_sb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real32_T c1_y;
  real32_T c1_f14;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_f14, 1, 1, 0U, 0, 0U, 0);
  c1_y = c1_f14;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_asinPitch;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_asinPitch = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_sb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_asinPitch),
    &c1_thisId);
  sf_mex_destroy(&c1_asinPitch);
  *(real32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i83;
  real_T c1_b_inData[4];
  int32_T c1_i84;
  real_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i83 = 0; c1_i83 < 4; c1_i83++) {
    c1_b_inData[c1_i83] = (*(real_T (*)[4])c1_inData)[c1_i83];
  }

  for (c1_i84 = 0; c1_i84 < 4; c1_i84++) {
    c1_u[c1_i84] = c1_b_inData[c1_i84];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_tb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[4])
{
  real_T c1_dv1[4];
  int32_T c1_i85;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv1, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c1_i85 = 0; c1_i85 < 4; c1_i85++) {
    c1_y[c1_i85] = c1_dv1[c1_i85];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_qDot;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[4];
  int32_T c1_i86;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_qDot = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_tb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_qDot), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_qDot);
  for (c1_i86 = 0; c1_i86 < 4; c1_i86++) {
    (*(real_T (*)[4])c1_outData)[c1_i86] = c1_y[c1_i86];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static void c1_ub_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real32_T c1_y[3])
{
  real32_T c1_fv15[3];
  int32_T c1_i87;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_fv15, 1, 1, 0U, 1, 0U, 1, 3);
  for (c1_i87 = 0; c1_i87 < 3; c1_i87++) {
    c1_y[c1_i87] = c1_fv15[c1_i87];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_w;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real32_T c1_y[3];
  int32_T c1_i88;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_w = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_ub_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_w), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_w);
  for (c1_i88 = 0; c1_i88 < 3; c1_i88++) {
    (*(real32_T (*)[3])c1_outData)[c1_i88] = c1_y[c1_i88];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_mahoneyAHRSlibrary_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_createstruct("structure", 2, 58, 1),
                false);
  c1_info_helper(&c1_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(const mxArray **c1_info)
{
  const mxArray *c1_rhs0 = NULL;
  const mxArray *c1_lhs0 = NULL;
  const mxArray *c1_rhs1 = NULL;
  const mxArray *c1_lhs1 = NULL;
  const mxArray *c1_rhs2 = NULL;
  const mxArray *c1_lhs2 = NULL;
  const mxArray *c1_rhs3 = NULL;
  const mxArray *c1_lhs3 = NULL;
  const mxArray *c1_rhs4 = NULL;
  const mxArray *c1_lhs4 = NULL;
  const mxArray *c1_rhs5 = NULL;
  const mxArray *c1_lhs5 = NULL;
  const mxArray *c1_rhs6 = NULL;
  const mxArray *c1_lhs6 = NULL;
  const mxArray *c1_rhs7 = NULL;
  const mxArray *c1_lhs7 = NULL;
  const mxArray *c1_rhs8 = NULL;
  const mxArray *c1_lhs8 = NULL;
  const mxArray *c1_rhs9 = NULL;
  const mxArray *c1_lhs9 = NULL;
  const mxArray *c1_rhs10 = NULL;
  const mxArray *c1_lhs10 = NULL;
  const mxArray *c1_rhs11 = NULL;
  const mxArray *c1_lhs11 = NULL;
  const mxArray *c1_rhs12 = NULL;
  const mxArray *c1_lhs12 = NULL;
  const mxArray *c1_rhs13 = NULL;
  const mxArray *c1_lhs13 = NULL;
  const mxArray *c1_rhs14 = NULL;
  const mxArray *c1_lhs14 = NULL;
  const mxArray *c1_rhs15 = NULL;
  const mxArray *c1_lhs15 = NULL;
  const mxArray *c1_rhs16 = NULL;
  const mxArray *c1_lhs16 = NULL;
  const mxArray *c1_rhs17 = NULL;
  const mxArray *c1_lhs17 = NULL;
  const mxArray *c1_rhs18 = NULL;
  const mxArray *c1_lhs18 = NULL;
  const mxArray *c1_rhs19 = NULL;
  const mxArray *c1_lhs19 = NULL;
  const mxArray *c1_rhs20 = NULL;
  const mxArray *c1_lhs20 = NULL;
  const mxArray *c1_rhs21 = NULL;
  const mxArray *c1_lhs21 = NULL;
  const mxArray *c1_rhs22 = NULL;
  const mxArray *c1_lhs22 = NULL;
  const mxArray *c1_rhs23 = NULL;
  const mxArray *c1_lhs23 = NULL;
  const mxArray *c1_rhs24 = NULL;
  const mxArray *c1_lhs24 = NULL;
  const mxArray *c1_rhs25 = NULL;
  const mxArray *c1_lhs25 = NULL;
  const mxArray *c1_rhs26 = NULL;
  const mxArray *c1_lhs26 = NULL;
  const mxArray *c1_rhs27 = NULL;
  const mxArray *c1_lhs27 = NULL;
  const mxArray *c1_rhs28 = NULL;
  const mxArray *c1_lhs28 = NULL;
  const mxArray *c1_rhs29 = NULL;
  const mxArray *c1_lhs29 = NULL;
  const mxArray *c1_rhs30 = NULL;
  const mxArray *c1_lhs30 = NULL;
  const mxArray *c1_rhs31 = NULL;
  const mxArray *c1_lhs31 = NULL;
  const mxArray *c1_rhs32 = NULL;
  const mxArray *c1_lhs32 = NULL;
  const mxArray *c1_rhs33 = NULL;
  const mxArray *c1_lhs33 = NULL;
  const mxArray *c1_rhs34 = NULL;
  const mxArray *c1_lhs34 = NULL;
  const mxArray *c1_rhs35 = NULL;
  const mxArray *c1_lhs35 = NULL;
  const mxArray *c1_rhs36 = NULL;
  const mxArray *c1_lhs36 = NULL;
  const mxArray *c1_rhs37 = NULL;
  const mxArray *c1_lhs37 = NULL;
  const mxArray *c1_rhs38 = NULL;
  const mxArray *c1_lhs38 = NULL;
  const mxArray *c1_rhs39 = NULL;
  const mxArray *c1_lhs39 = NULL;
  const mxArray *c1_rhs40 = NULL;
  const mxArray *c1_lhs40 = NULL;
  const mxArray *c1_rhs41 = NULL;
  const mxArray *c1_lhs41 = NULL;
  const mxArray *c1_rhs42 = NULL;
  const mxArray *c1_lhs42 = NULL;
  const mxArray *c1_rhs43 = NULL;
  const mxArray *c1_lhs43 = NULL;
  const mxArray *c1_rhs44 = NULL;
  const mxArray *c1_lhs44 = NULL;
  const mxArray *c1_rhs45 = NULL;
  const mxArray *c1_lhs45 = NULL;
  const mxArray *c1_rhs46 = NULL;
  const mxArray *c1_lhs46 = NULL;
  const mxArray *c1_rhs47 = NULL;
  const mxArray *c1_lhs47 = NULL;
  const mxArray *c1_rhs48 = NULL;
  const mxArray *c1_lhs48 = NULL;
  const mxArray *c1_rhs49 = NULL;
  const mxArray *c1_lhs49 = NULL;
  const mxArray *c1_rhs50 = NULL;
  const mxArray *c1_lhs50 = NULL;
  const mxArray *c1_rhs51 = NULL;
  const mxArray *c1_lhs51 = NULL;
  const mxArray *c1_rhs52 = NULL;
  const mxArray *c1_lhs52 = NULL;
  const mxArray *c1_rhs53 = NULL;
  const mxArray *c1_lhs53 = NULL;
  const mxArray *c1_rhs54 = NULL;
  const mxArray *c1_lhs54 = NULL;
  const mxArray *c1_rhs55 = NULL;
  const mxArray *c1_lhs55 = NULL;
  const mxArray *c1_rhs56 = NULL;
  const mxArray *c1_lhs56 = NULL;
  const mxArray *c1_rhs57 = NULL;
  const mxArray *c1_lhs57 = NULL;
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("atan2"), "name", "name", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859172U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c1_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009488U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c1_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336720U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c1_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009488U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c1_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336720U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c1_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847520U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c1_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("cos"), "name", "name", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859172U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c1_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847522U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c1_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("sin"), "name", "name", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859186U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c1_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847536U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c1_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("mrdivide"), "name", "name", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1388488896U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1370038686U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c1_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c1_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("rdivide"), "name", "name", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739080U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c1_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c1_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c1_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847596U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c1_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_div"), "name", "name", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009488U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c1_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336720U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c1_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("sqrt"), "name", "name", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859186U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c1_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_error"), "name", "name",
                  19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859158U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c1_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847538U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c1_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("norm"), "name", "name", 21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739068U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c1_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1323199378U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c1_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c1_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009492U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c1_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c1_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c1_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c1_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c1_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1381879100U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c1_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c1_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("realmin"), "name", "name", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1307680042U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c1_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_realmin"), "name", "name",
                  32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1307680044U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c1_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 33);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1326756796U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c1_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 34);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 34);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372611960U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c1_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 35);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 35);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 35);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372611960U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c1_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 36);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 36);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 36);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1372611960U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c1_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 37);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 37);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009488U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c1_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 38);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("intmax"), "name", "name", 38);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1362290682U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c1_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 39);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1381879100U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c1_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 40);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("abs"), "name", "name", 40);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739052U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c1_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 41);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c1_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 42);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847512U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c1_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 43);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 43);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1383906094U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c1_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 44);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 44);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c1_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 45);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 45);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c1_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 46);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("abs"), "name", "name", 46);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 46);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739052U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c1_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 47);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("cross"), "name", "name", 47);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/cross.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847642U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c1_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 48);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("norm"), "name", "name", 48);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739068U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c1_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 49);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  49);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1376009492U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c1_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 50);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 50);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c1_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 51);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 51);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1389336722U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c1_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 52);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("abs"), "name", "name", 52);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 52);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739052U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c1_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 53);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1363739756U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c1_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 54);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 54);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1286847512U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c1_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(""), "context", "context", 55);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("asin"), "name", "name", 55);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m"), "resolved",
                  "resolved", 55);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859170U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c1_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_error"), "name", "name",
                  56);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859158U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c1_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("eml_scalar_asin"), "name",
                  "name", 57);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c1_info, c1_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(1343859176U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c1_info, c1_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c1_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c1_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c1_info, sf_mex_duplicatearraysafe(&c1_lhs57), "lhs", "lhs",
                  57);
  sf_mex_destroy(&c1_rhs0);
  sf_mex_destroy(&c1_lhs0);
  sf_mex_destroy(&c1_rhs1);
  sf_mex_destroy(&c1_lhs1);
  sf_mex_destroy(&c1_rhs2);
  sf_mex_destroy(&c1_lhs2);
  sf_mex_destroy(&c1_rhs3);
  sf_mex_destroy(&c1_lhs3);
  sf_mex_destroy(&c1_rhs4);
  sf_mex_destroy(&c1_lhs4);
  sf_mex_destroy(&c1_rhs5);
  sf_mex_destroy(&c1_lhs5);
  sf_mex_destroy(&c1_rhs6);
  sf_mex_destroy(&c1_lhs6);
  sf_mex_destroy(&c1_rhs7);
  sf_mex_destroy(&c1_lhs7);
  sf_mex_destroy(&c1_rhs8);
  sf_mex_destroy(&c1_lhs8);
  sf_mex_destroy(&c1_rhs9);
  sf_mex_destroy(&c1_lhs9);
  sf_mex_destroy(&c1_rhs10);
  sf_mex_destroy(&c1_lhs10);
  sf_mex_destroy(&c1_rhs11);
  sf_mex_destroy(&c1_lhs11);
  sf_mex_destroy(&c1_rhs12);
  sf_mex_destroy(&c1_lhs12);
  sf_mex_destroy(&c1_rhs13);
  sf_mex_destroy(&c1_lhs13);
  sf_mex_destroy(&c1_rhs14);
  sf_mex_destroy(&c1_lhs14);
  sf_mex_destroy(&c1_rhs15);
  sf_mex_destroy(&c1_lhs15);
  sf_mex_destroy(&c1_rhs16);
  sf_mex_destroy(&c1_lhs16);
  sf_mex_destroy(&c1_rhs17);
  sf_mex_destroy(&c1_lhs17);
  sf_mex_destroy(&c1_rhs18);
  sf_mex_destroy(&c1_lhs18);
  sf_mex_destroy(&c1_rhs19);
  sf_mex_destroy(&c1_lhs19);
  sf_mex_destroy(&c1_rhs20);
  sf_mex_destroy(&c1_lhs20);
  sf_mex_destroy(&c1_rhs21);
  sf_mex_destroy(&c1_lhs21);
  sf_mex_destroy(&c1_rhs22);
  sf_mex_destroy(&c1_lhs22);
  sf_mex_destroy(&c1_rhs23);
  sf_mex_destroy(&c1_lhs23);
  sf_mex_destroy(&c1_rhs24);
  sf_mex_destroy(&c1_lhs24);
  sf_mex_destroy(&c1_rhs25);
  sf_mex_destroy(&c1_lhs25);
  sf_mex_destroy(&c1_rhs26);
  sf_mex_destroy(&c1_lhs26);
  sf_mex_destroy(&c1_rhs27);
  sf_mex_destroy(&c1_lhs27);
  sf_mex_destroy(&c1_rhs28);
  sf_mex_destroy(&c1_lhs28);
  sf_mex_destroy(&c1_rhs29);
  sf_mex_destroy(&c1_lhs29);
  sf_mex_destroy(&c1_rhs30);
  sf_mex_destroy(&c1_lhs30);
  sf_mex_destroy(&c1_rhs31);
  sf_mex_destroy(&c1_lhs31);
  sf_mex_destroy(&c1_rhs32);
  sf_mex_destroy(&c1_lhs32);
  sf_mex_destroy(&c1_rhs33);
  sf_mex_destroy(&c1_lhs33);
  sf_mex_destroy(&c1_rhs34);
  sf_mex_destroy(&c1_lhs34);
  sf_mex_destroy(&c1_rhs35);
  sf_mex_destroy(&c1_lhs35);
  sf_mex_destroy(&c1_rhs36);
  sf_mex_destroy(&c1_lhs36);
  sf_mex_destroy(&c1_rhs37);
  sf_mex_destroy(&c1_lhs37);
  sf_mex_destroy(&c1_rhs38);
  sf_mex_destroy(&c1_lhs38);
  sf_mex_destroy(&c1_rhs39);
  sf_mex_destroy(&c1_lhs39);
  sf_mex_destroy(&c1_rhs40);
  sf_mex_destroy(&c1_lhs40);
  sf_mex_destroy(&c1_rhs41);
  sf_mex_destroy(&c1_lhs41);
  sf_mex_destroy(&c1_rhs42);
  sf_mex_destroy(&c1_lhs42);
  sf_mex_destroy(&c1_rhs43);
  sf_mex_destroy(&c1_lhs43);
  sf_mex_destroy(&c1_rhs44);
  sf_mex_destroy(&c1_lhs44);
  sf_mex_destroy(&c1_rhs45);
  sf_mex_destroy(&c1_lhs45);
  sf_mex_destroy(&c1_rhs46);
  sf_mex_destroy(&c1_lhs46);
  sf_mex_destroy(&c1_rhs47);
  sf_mex_destroy(&c1_lhs47);
  sf_mex_destroy(&c1_rhs48);
  sf_mex_destroy(&c1_lhs48);
  sf_mex_destroy(&c1_rhs49);
  sf_mex_destroy(&c1_lhs49);
  sf_mex_destroy(&c1_rhs50);
  sf_mex_destroy(&c1_lhs50);
  sf_mex_destroy(&c1_rhs51);
  sf_mex_destroy(&c1_lhs51);
  sf_mex_destroy(&c1_rhs52);
  sf_mex_destroy(&c1_lhs52);
  sf_mex_destroy(&c1_rhs53);
  sf_mex_destroy(&c1_lhs53);
  sf_mex_destroy(&c1_rhs54);
  sf_mex_destroy(&c1_lhs54);
  sf_mex_destroy(&c1_rhs55);
  sf_mex_destroy(&c1_lhs55);
  sf_mex_destroy(&c1_rhs56);
  sf_mex_destroy(&c1_lhs56);
  sf_mex_destroy(&c1_rhs57);
  sf_mex_destroy(&c1_lhs57);
}

static const mxArray *c1_emlrt_marshallOut(const char * c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c1_u)), false);
  return c1_y;
}

static const mxArray *c1_b_emlrt_marshallOut(const uint32_T c1_u)
{
  const mxArray *c1_y = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 7, 0U, 0U, 0U, 0), false);
  return c1_y;
}

static real32_T c1_atan2(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_y, real32_T c1_x)
{
  real32_T c1_b_y;
  real32_T c1_b_x;
  (void)chartInstance;
  c1_b_y = c1_y;
  c1_b_x = c1_x;
  return muSingleScalarAtan2(c1_b_y, c1_b_x);
}

static real32_T c1_sqrt(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x)
{
  real32_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sqrt(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_eml_error(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  int32_T c1_i89;
  static char_T c1_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  int32_T c1_i90;
  static char_T c1_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c1_b_u[4];
  const mxArray *c1_b_y = NULL;
  (void)chartInstance;
  for (c1_i89 = 0; c1_i89 < 30; c1_i89++) {
    c1_u[c1_i89] = c1_cv0[c1_i89];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c1_i90 = 0; c1_i90 < 4; c1_i90++) {
    c1_b_u[c1_i90] = c1_cv1[c1_i90];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static real32_T c1_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x[3])
{
  real32_T c1_y;
  real32_T c1_scale;
  int32_T c1_k;
  int32_T c1_b_k;
  real32_T c1_b_x;
  real32_T c1_c_x;
  real32_T c1_absxk;
  real32_T c1_t;
  c1_eml_switch_helper(chartInstance);
  c1_y = 0.0F;
  c1_realmin(chartInstance);
  c1_scale = 1.17549435E-38F;
  for (c1_k = 1; c1_k < 4; c1_k++) {
    c1_b_k = c1_k;
    c1_b_x = c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c1_b_k), 1, 3, 1, 0) - 1];
    c1_c_x = c1_b_x;
    c1_absxk = muSingleScalarAbs(c1_c_x);
    if (c1_absxk > c1_scale) {
      c1_t = c1_scale / c1_absxk;
      c1_y = 1.0F + c1_y * c1_t * c1_t;
      c1_scale = c1_absxk;
    } else {
      c1_t = c1_absxk / c1_scale;
      c1_y += c1_t * c1_t;
    }
  }

  return c1_scale * muSingleScalarSqrt(c1_y);
}

static void c1_eml_switch_helper(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c1_realmin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real32_T c1_abs(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x)
{
  real32_T c1_b_x;
  (void)chartInstance;
  c1_b_x = c1_x;
  return muSingleScalarAbs(c1_b_x);
}

static void c1_cross(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                     real32_T c1_a[3], real32_T c1_b[3], real32_T c1_c[3])
{
  real32_T c1_c1;
  real32_T c1_c2;
  real32_T c1_c3;
  (void)chartInstance;
  c1_c1 = c1_a[1] * c1_b[2] - c1_a[2] * c1_b[1];
  c1_c2 = c1_a[2] * c1_b[0] - c1_a[0] * c1_b[2];
  c1_c3 = c1_a[0] * c1_b[1] - c1_a[1] * c1_b[0];
  c1_c[0] = c1_c1;
  c1_c[1] = c1_c2;
  c1_c[2] = c1_c3;
}

static void c1_quaternConj(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_b_q[4], real32_T c1_qConj[4])
{
  uint32_T c1_debug_family_var_map[4];
  real_T c1_nargin = 1.0;
  real_T c1_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c1_b_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_q, 2U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_qConj, 3U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 252U);
  c1_qConj[0] = c1_b_q[0];
  c1_qConj[1] = -c1_b_q[1];
  c1_qConj[2] = -c1_b_q[2];
  c1_qConj[3] = -c1_b_q[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -252);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c1_quaternProd(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_a[4], real32_T c1_b[4], real_T c1_ab[4])
{
  uint32_T c1_debug_family_var_map[5];
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 1.0;
  int32_T c1_i91;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c1_c_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a, 2U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b, 3U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_ab, 4U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 269);
  for (c1_i91 = 0; c1_i91 < 4; c1_i91++) {
    c1_ab[c1_i91] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 271);
  c1_ab[0] = ((c1_a[0] * c1_b[0] - c1_a[1] * c1_b[1]) - c1_a[2] * c1_b[2]) -
    c1_a[3] * c1_b[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 272);
  c1_ab[1] = ((c1_a[0] * c1_b[1] + c1_a[1] * c1_b[0]) + c1_a[2] * c1_b[3]) -
    c1_a[3] * c1_b[2];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 273);
  c1_ab[2] = ((c1_a[0] * c1_b[2] - c1_a[1] * c1_b[3]) + c1_a[2] * c1_b[0]) +
    c1_a[3] * c1_b[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 274);
  c1_ab[3] = ((c1_a[0] * c1_b[3] + c1_a[1] * c1_b[2]) - c1_a[2] * c1_b[1]) +
    c1_a[3] * c1_b[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -274);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c1_b_quaternProd(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, real32_T c1_a[4], real_T c1_b[4], real_T c1_ab[4])
{
  uint32_T c1_debug_family_var_map[5];
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 1.0;
  int32_T c1_i92;
  real32_T c1_b_a;
  real_T c1_b_b;
  real32_T c1_y;
  real32_T c1_c_a;
  real_T c1_c_b;
  real32_T c1_b_y;
  real32_T c1_d_a;
  real_T c1_d_b;
  real32_T c1_c_y;
  real32_T c1_e_a;
  real_T c1_e_b;
  real32_T c1_d_y;
  real32_T c1_f_a;
  real_T c1_f_b;
  real32_T c1_e_y;
  real32_T c1_g_a;
  real_T c1_g_b;
  real32_T c1_f_y;
  real32_T c1_h_a;
  real_T c1_h_b;
  real32_T c1_g_y;
  real32_T c1_i_a;
  real_T c1_i_b;
  real32_T c1_h_y;
  real32_T c1_j_a;
  real_T c1_j_b;
  real32_T c1_i_y;
  real32_T c1_k_a;
  real_T c1_k_b;
  real32_T c1_j_y;
  real32_T c1_l_a;
  real_T c1_l_b;
  real32_T c1_k_y;
  real32_T c1_m_a;
  real_T c1_m_b;
  real32_T c1_l_y;
  real32_T c1_n_a;
  real_T c1_n_b;
  real32_T c1_m_y;
  real32_T c1_o_a;
  real_T c1_o_b;
  real32_T c1_n_y;
  real32_T c1_p_a;
  real_T c1_p_b;
  real32_T c1_o_y;
  real32_T c1_q_a;
  real_T c1_q_b;
  real32_T c1_p_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c1_d_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_y_sf_marshallOut,
    c1_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_a, 2U, c1_u_sf_marshallOut,
    c1_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b, 3U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_ab, 4U, c1_ab_sf_marshallOut,
    c1_x_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 269);
  for (c1_i92 = 0; c1_i92 < 4; c1_i92++) {
    c1_ab[c1_i92] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 271);
  c1_b_a = c1_a[0];
  c1_b_b = c1_b[0];
  c1_y = c1_b_a * (real32_T)c1_b_b;
  c1_c_a = c1_a[1];
  c1_c_b = c1_b[1];
  c1_b_y = c1_c_a * (real32_T)c1_c_b;
  c1_d_a = c1_a[2];
  c1_d_b = c1_b[2];
  c1_c_y = c1_d_a * (real32_T)c1_d_b;
  c1_e_a = c1_a[3];
  c1_e_b = c1_b[3];
  c1_d_y = c1_e_a * (real32_T)c1_e_b;
  c1_ab[0] = ((c1_y - c1_b_y) - c1_c_y) - c1_d_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 272);
  c1_f_a = c1_a[0];
  c1_f_b = c1_b[1];
  c1_e_y = c1_f_a * (real32_T)c1_f_b;
  c1_g_a = c1_a[1];
  c1_g_b = c1_b[0];
  c1_f_y = c1_g_a * (real32_T)c1_g_b;
  c1_h_a = c1_a[2];
  c1_h_b = c1_b[3];
  c1_g_y = c1_h_a * (real32_T)c1_h_b;
  c1_i_a = c1_a[3];
  c1_i_b = c1_b[2];
  c1_h_y = c1_i_a * (real32_T)c1_i_b;
  c1_ab[1] = ((c1_e_y + c1_f_y) + c1_g_y) - c1_h_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 273);
  c1_j_a = c1_a[0];
  c1_j_b = c1_b[2];
  c1_i_y = c1_j_a * (real32_T)c1_j_b;
  c1_k_a = c1_a[1];
  c1_k_b = c1_b[3];
  c1_j_y = c1_k_a * (real32_T)c1_k_b;
  c1_l_a = c1_a[2];
  c1_l_b = c1_b[0];
  c1_k_y = c1_l_a * (real32_T)c1_l_b;
  c1_m_a = c1_a[3];
  c1_m_b = c1_b[1];
  c1_l_y = c1_m_a * (real32_T)c1_m_b;
  c1_ab[2] = ((c1_i_y - c1_j_y) + c1_k_y) + c1_l_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 274);
  c1_n_a = c1_a[0];
  c1_n_b = c1_b[3];
  c1_m_y = c1_n_a * (real32_T)c1_n_b;
  c1_o_a = c1_a[1];
  c1_o_b = c1_b[2];
  c1_n_y = c1_o_a * (real32_T)c1_o_b;
  c1_p_a = c1_a[2];
  c1_p_b = c1_b[1];
  c1_o_y = c1_p_a * (real32_T)c1_p_b;
  c1_q_a = c1_a[3];
  c1_q_b = c1_b[0];
  c1_p_y = c1_q_a * (real32_T)c1_q_b;
  c1_ab[3] = ((c1_m_y + c1_n_y) - c1_o_y) + c1_p_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -274);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c1_b_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real_T c1_x[2])
{
  real_T c1_y;
  real_T c1_scale;
  int32_T c1_k;
  int32_T c1_b_k;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_absxk;
  real_T c1_t;
  c1_eml_switch_helper(chartInstance);
  c1_y = 0.0;
  c1_scale = 2.2250738585072014E-308;
  for (c1_k = 1; c1_k < 3; c1_k++) {
    c1_b_k = c1_k;
    c1_b_x = c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c1_b_k), 1, 2, 1, 0) - 1];
    c1_c_x = c1_b_x;
    c1_absxk = muDoubleScalarAbs(c1_c_x);
    if (c1_absxk > c1_scale) {
      c1_t = c1_scale / c1_absxk;
      c1_y = 1.0 + c1_y * c1_t * c1_t;
      c1_scale = c1_absxk;
    } else {
      c1_t = c1_absxk / c1_scale;
      c1_y += c1_t * c1_t;
    }
  }

  return c1_scale * muDoubleScalarSqrt(c1_y);
}

static real32_T c1_c_norm(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x[4])
{
  real32_T c1_y;
  real32_T c1_scale;
  int32_T c1_k;
  int32_T c1_b_k;
  real32_T c1_b_x;
  real32_T c1_c_x;
  real32_T c1_absxk;
  real32_T c1_t;
  c1_eml_switch_helper(chartInstance);
  c1_y = 0.0F;
  c1_realmin(chartInstance);
  c1_scale = 1.17549435E-38F;
  for (c1_k = 1; c1_k < 5; c1_k++) {
    c1_b_k = c1_k;
    c1_b_x = c1_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c1_b_k), 1, 4, 1, 0) - 1];
    c1_c_x = c1_b_x;
    c1_absxk = muSingleScalarAbs(c1_c_x);
    if (c1_absxk > c1_scale) {
      c1_t = c1_scale / c1_absxk;
      c1_y = 1.0F + c1_y * c1_t * c1_t;
      c1_scale = c1_absxk;
    } else {
      c1_t = c1_absxk / c1_scale;
      c1_y += c1_t * c1_t;
    }
  }

  return c1_scale * muSingleScalarSqrt(c1_y);
}

static real32_T c1_asin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
  real32_T c1_x)
{
  real32_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_asin(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_b_eml_error(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance)
{
  int32_T c1_i93;
  static char_T c1_cv2[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  int32_T c1_i94;
  static char_T c1_cv3[4] = { 'a', 's', 'i', 'n' };

  char_T c1_b_u[4];
  const mxArray *c1_b_y = NULL;
  (void)chartInstance;
  for (c1_i93 = 0; c1_i93 < 30; c1_i93++) {
    c1_u[c1_i93] = c1_cv2[c1_i93];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c1_i94 = 0; c1_i94 < 4; c1_i94++) {
    c1_b_u[c1_i94] = c1_cv3[c1_i94];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static const mxArray *c1_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_vb_emlrt_marshallIn(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i95;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i95, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i95;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_vb_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static void c1_b_sqrt(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                      real32_T *c1_x)
{
  if (*c1_x < 0.0F) {
    c1_eml_error(chartInstance);
  }

  *c1_x = muSingleScalarSqrt(*c1_x);
}

static void c1_b_asin(SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance,
                      real32_T *c1_x)
{
  boolean_T guard1 = false;
  guard1 = false;
  if (*c1_x < -1.0F) {
    guard1 = true;
  } else {
    if (1.0F < *c1_x) {
      guard1 = true;
    }
  }

  if (guard1 == true) {
    c1_b_eml_error(chartInstance);
  }

  *c1_x = muSingleScalarAsin(*c1_x);
}

static void init_dsm_address_info(SFc1_mahoneyAHRSlibraryInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_mahoneyAHRSlibrary_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1715173720U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2873433242U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1384716085U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2524345422U);
}

mxArray *sf_c1_mahoneyAHRSlibrary_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("WTQQV2dmVYABUhFfr4X3hH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,15,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      pr[1] = (double)(1);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(4);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(3);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_mahoneyAHRSlibrary_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_mahoneyAHRSlibrary_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c1_mahoneyAHRSlibrary(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[19],T\"attitude\",},{M[1],M[5],T\"qOut\",},{M[1],M[11],T\"qValidOut\",},{M[4],M[0],T\"accelConfidenceDecay\",S'l','i','p'{{M1x2[848 868],M[0],}}},{M[4],M[0],T\"ahrsFirstPass\",S'l','i','p'{{M1x2[869 882],M[0],}}},{M[4],M[0],T\"eAcc\",S'l','i','p'{{M1x2[883 887],M[0],}}},{M[4],M[0],T\"eMag\",S'l','i','p'{{M1x2[888 892],M[0],}}},{M[4],M[0],T\"pastAccelFilter\",S'l','i','p'{{M1x2[893 908],M[0],}}},{M[4],M[0],T\"q\",S'l','i','p'{{M1x2[920 921],M[0],}}},{M[4],M[0],T\"q1q1\",S'l','i','p'{{M1x2[952 956],M[0],}}}}",
    "100 S1x10'type','srcId','name','auxInfo'{{M[4],M[0],T\"q1q2\",S'l','i','p'{{M1x2[957 961],M[0],}}},{M[4],M[0],T\"q1q3\",S'l','i','p'{{M1x2[962 966],M[0],}}},{M[4],M[0],T\"q1q4\",S'l','i','p'{{M1x2[967 971],M[0],}}},{M[4],M[0],T\"q2q2\",S'l','i','p'{{M1x2[984 988],M[0],}}},{M[4],M[0],T\"q2q3\",S'l','i','p'{{M1x2[989 993],M[0],}}},{M[4],M[0],T\"q2q4\",S'l','i','p'{{M1x2[994 998],M[0],}}},{M[4],M[0],T\"q3q3\",S'l','i','p'{{M1x2[1010 1014],M[0],}}},{M[4],M[0],T\"q3q4\",S'l','i','p'{{M1x2[1015 1019],M[0],}}},{M[4],M[0],T\"q4q4\",S'l','i','p'{{M1x2[1031 1035],M[0],}}},{M[4],M[0],T\"qValid\",S'l','i','p'{{M1x2[922 928],M[0],}}}}",
    "100 S1x2'type','srcId','name','auxInfo'{{M[4],M[0],T\"qValidCount\",S'l','i','p'{{M1x2[929 940],M[0],}}},{M[8],M[0],T\"is_active_c1_mahoneyAHRSlibrary\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 22, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_mahoneyAHRSlibrary_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _mahoneyAHRSlibraryMachineNumber_,
           1,
           1,
           1,
           0,
           18,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_mahoneyAHRSlibraryMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_mahoneyAHRSlibraryMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _mahoneyAHRSlibraryMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"gyro");
          _SFD_SET_DATA_PROPS(1,2,0,1,"qOut");
          _SFD_SET_DATA_PROPS(2,1,1,0,"accel");
          _SFD_SET_DATA_PROPS(3,1,1,0,"mag");
          _SFD_SET_DATA_PROPS(4,1,1,0,"dt");
          _SFD_SET_DATA_PROPS(5,1,1,0,"gyroValid");
          _SFD_SET_DATA_PROPS(6,1,1,0,"accelValid");
          _SFD_SET_DATA_PROPS(7,1,1,0,"magValid");
          _SFD_SET_DATA_PROPS(8,1,1,0,"accelOneG");
          _SFD_SET_DATA_PROPS(9,1,1,0,"accelDataUpdate");
          _SFD_SET_DATA_PROPS(10,1,1,0,"magDataUpdate");
          _SFD_SET_DATA_PROPS(11,1,1,0,"magVar");
          _SFD_SET_DATA_PROPS(12,1,1,0,"kpAcc");
          _SFD_SET_DATA_PROPS(13,1,1,0,"kpMag");
          _SFD_SET_DATA_PROPS(14,2,0,1,"qValidOut");
          _SFD_SET_DATA_PROPS(15,1,1,0,"accelCutoff");
          _SFD_SET_DATA_PROPS(16,2,0,1,"attitude");
          _SFD_SET_DATA_PROPS(17,1,1,0,"attTrim");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,3,17,0,1,0,0,0,9,3);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",360,-1,7044);
        _SFD_CV_INIT_EML_FCN(0,1,"quaternConj",7463,-1,7546);
        _SFD_CV_INIT_EML_FCN(0,2,"quaternProd",7977,-1,8316);
        _SFD_CV_INIT_EML_SATURATION(0,1,0,5225,-1,5240);
        _SFD_CV_INIT_EML_IF(0,1,0,1037,1062,-1,1614);
        _SFD_CV_INIT_EML_IF(0,1,1,1767,1858,-1,2987);
        _SFD_CV_INIT_EML_IF(0,1,2,3050,3067,6666,7039);
        _SFD_CV_INIT_EML_IF(0,1,3,3072,3090,-1,4023);
        _SFD_CV_INIT_EML_IF(0,1,4,3523,3545,3595,3621);
        _SFD_CV_INIT_EML_IF(0,1,5,3595,3621,-1,3621);
        _SFD_CV_INIT_EML_IF(0,1,6,4098,4114,-1,4834);
        _SFD_CV_INIT_EML_IF(0,1,7,5186,5202,-1,5249);
        _SFD_CV_INIT_EML_IF(0,1,8,5259,5300,-1,5335);
        _SFD_CV_INIT_EML_IF(0,1,9,5810,5826,5862,5883);
        _SFD_CV_INIT_EML_IF(0,1,10,5862,5883,-1,5883);
        _SFD_CV_INIT_EML_IF(0,1,11,6141,6161,6210,6234);
        _SFD_CV_INIT_EML_IF(0,1,12,6210,6234,-1,6234);
        _SFD_CV_INIT_EML_IF(0,1,13,6296,6316,6365,6389);
        _SFD_CV_INIT_EML_IF(0,1,14,6365,6389,-1,6389);
        _SFD_CV_INIT_EML_IF(0,1,15,6455,6475,6524,6548);
        _SFD_CV_INIT_EML_IF(0,1,16,6524,6548,-1,6548);

        {
          static int condStart[] = { 1770, 1787, 1801, 1820, 1833, 1845 };

          static int condEnd[] = { 1783, 1797, 1816, 1829, 1841, 1858 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3, 3, -3, 4, -3, 5, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,1770,1858,6,0,&(condStart[0]),&(condEnd[0]),
                                11,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3054 };

          static int condEnd[] = { 3067 };

          static int pfixExpr[] = { 0, -1 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,3053,3067,1,6,&(condStart[0]),&(condEnd[0]),
                                2,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 5264, 5287 };

          static int condEnd[] = { 5281, 5298 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,5263,5299,2,7,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_x_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_u_sf_marshallOut,(MexInFcnForType)
            c1_u_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_x_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_x_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_t_sf_marshallOut,(MexInFcnForType)c1_t_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_w_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(16,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_s_sf_marshallOut,(MexInFcnForType)
            c1_s_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(17,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_v_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real32_T *c1_dt;
          uint8_T *c1_gyroValid;
          uint8_T *c1_accelValid;
          uint8_T *c1_magValid;
          real32_T *c1_accelOneG;
          uint8_T *c1_accelDataUpdate;
          uint8_T *c1_magDataUpdate;
          real32_T *c1_magVar;
          real32_T *c1_kpAcc;
          real32_T *c1_kpMag;
          uint8_T *c1_qValidOut;
          real32_T *c1_accelCutoff;
          real32_T (*c1_gyro)[3];
          real32_T (*c1_qOut)[4];
          real32_T (*c1_accel)[3];
          real32_T (*c1_mag)[3];
          real32_T (*c1_attitude)[3];
          real32_T (*c1_attTrim)[2];
          c1_attTrim = (real32_T (*)[2])ssGetInputPortSignal(chartInstance->S,
            14);
          c1_attitude = (real32_T (*)[3])ssGetOutputPortSignal(chartInstance->S,
            3);
          c1_accelCutoff = (real32_T *)ssGetInputPortSignal(chartInstance->S, 13);
          c1_qValidOut = (uint8_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_kpMag = (real32_T *)ssGetInputPortSignal(chartInstance->S, 12);
          c1_kpAcc = (real32_T *)ssGetInputPortSignal(chartInstance->S, 11);
          c1_magVar = (real32_T *)ssGetInputPortSignal(chartInstance->S, 10);
          c1_magDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 9);
          c1_accelDataUpdate = (uint8_T *)ssGetInputPortSignal(chartInstance->S,
            8);
          c1_accelOneG = (real32_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c1_magValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c1_accelValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c1_gyroValid = (uint8_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_dt = (real32_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_mag = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c1_accel = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c1_qOut = (real32_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c1_gyro = (real32_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_gyro);
          _SFD_SET_DATA_VALUE_PTR(1U, *c1_qOut);
          _SFD_SET_DATA_VALUE_PTR(2U, *c1_accel);
          _SFD_SET_DATA_VALUE_PTR(3U, *c1_mag);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_dt);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_gyroValid);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_accelValid);
          _SFD_SET_DATA_VALUE_PTR(7U, c1_magValid);
          _SFD_SET_DATA_VALUE_PTR(8U, c1_accelOneG);
          _SFD_SET_DATA_VALUE_PTR(9U, c1_accelDataUpdate);
          _SFD_SET_DATA_VALUE_PTR(10U, c1_magDataUpdate);
          _SFD_SET_DATA_VALUE_PTR(11U, c1_magVar);
          _SFD_SET_DATA_VALUE_PTR(12U, c1_kpAcc);
          _SFD_SET_DATA_VALUE_PTR(13U, c1_kpMag);
          _SFD_SET_DATA_VALUE_PTR(14U, c1_qValidOut);
          _SFD_SET_DATA_VALUE_PTR(15U, c1_accelCutoff);
          _SFD_SET_DATA_VALUE_PTR(16U, *c1_attitude);
          _SFD_SET_DATA_VALUE_PTR(17U, *c1_attTrim);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _mahoneyAHRSlibraryMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "ePn4bcbmBsBMWWsNQeBuGB";
}

static void sf_opaque_initialize_c1_mahoneyAHRSlibrary(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
  initialize_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_mahoneyAHRSlibrary(void *chartInstanceVar)
{
  enable_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c1_mahoneyAHRSlibrary(void *chartInstanceVar)
{
  disable_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_mahoneyAHRSlibrary(void *chartInstanceVar)
{
  sf_gateway_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_mahoneyAHRSlibrary(SimStruct*
  S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_mahoneyAHRSlibrary
    ((SFc1_mahoneyAHRSlibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_mahoneyAHRSlibrary();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_mahoneyAHRSlibrary(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c1_mahoneyAHRSlibrary();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_mahoneyAHRSlibrary(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_mahoneyAHRSlibrary(S);
}

static void sf_opaque_set_sim_state_c1_mahoneyAHRSlibrary(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_mahoneyAHRSlibrary(S, st);
}

static void sf_opaque_terminate_c1_mahoneyAHRSlibrary(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_mahoneyAHRSlibraryInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_mahoneyAHRSlibrary_optimization_info();
    }

    finalize_c1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_mahoneyAHRSlibrary((SFc1_mahoneyAHRSlibraryInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_mahoneyAHRSlibrary(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c1_mahoneyAHRSlibrary
      ((SFc1_mahoneyAHRSlibraryInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_mahoneyAHRSlibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_mahoneyAHRSlibrary_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 13, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 14, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,15);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 15; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1424942908U));
  ssSetChecksum1(S,(2114929319U));
  ssSetChecksum2(S,(445893597U));
  ssSetChecksum3(S,(3853174052U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_mahoneyAHRSlibrary(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_mahoneyAHRSlibrary(SimStruct *S)
{
  SFc1_mahoneyAHRSlibraryInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc1_mahoneyAHRSlibraryInstanceStruct *)utMalloc(sizeof
    (SFc1_mahoneyAHRSlibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_mahoneyAHRSlibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_mahoneyAHRSlibrary;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_mahoneyAHRSlibrary_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_mahoneyAHRSlibrary(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_mahoneyAHRSlibrary(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_mahoneyAHRSlibrary(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_mahoneyAHRSlibrary_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
