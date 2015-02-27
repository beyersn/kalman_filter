/* Include files */

#include <stddef.h>
#include "blas.h"
#include "UKF_10hz_sfun.h"
#include "c7_UKF_10hz.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "UKF_10hz_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c7_debug_family_names[22] = { "Torque_c_temp", "Torque_c",
  "w_c_temp", "x_km1", "k1", "k2", "k3", "k4", "x_k", "w_s_temp", "nargin",
  "nargout", "I_c", "w_init_s", "Torque_s", "q0_i_s", "q_s_c", "Ts", "w_s",
  "q_i_s", "w_c_km1", "q_i_c_km1" };

static const char * c7_b_debug_family_names[5] = { "q_conj", "nargin", "nargout",
  "qin", "qinv" };

static const char * c7_c_debug_family_names[7] = { "vec", "scalar", "q", "r",
  "nargin", "nargout", "qres" };

static const char * c7_d_debug_family_names[4] = { "nargin", "nargout", "x",
  "output" };

static const char * c7_e_debug_family_names[10] = { "q", "w", "q_dot", "w_dot",
  "I", "nargin", "nargout", "x", "Torque_c", "results" };

/* Function Declarations */
static void initialize_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void initialize_params_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance);
static void enable_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void disable_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_update_debugger_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance);
static void set_sim_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_st);
static void finalize_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void sf_gateway_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_chartstep_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void initSimStructsc7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static void c7_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_q_i_c_km1, const char_T *c7_identifier, real_T c7_y[4]);
static void c7_b_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4]);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_w_c_km1, const char_T *c7_identifier, real_T c7_y[3]);
static void c7_d_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3]);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_e_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_q_i_s, const char_T *c7_identifier, real_T c7_y[4]);
static void c7_f_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4]);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_g_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_w_s, const char_T *c7_identifier, real_T c7_y[3]);
static void c7_h_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3]);
static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static real_T c7_i_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_j_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[7]);
static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_k_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4]);
static void c7_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_l_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3]);
static void c7_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_i_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_m_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[9]);
static void c7_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static void c7_info_helper(const mxArray **c7_info);
static const mxArray *c7_emlrt_marshallOut(const char * c7_u);
static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u);
static void c7_b_info_helper(const mxArray **c7_info);
static void c7_quatinv(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_qin[4], real_T c7_qinv[4]);
static void c7_eml_error(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_quatmultiply(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_q[4], real_T c7_r[4], real_T c7_qres[4]);
static void c7_Kinematics(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_x[7], real_T c7_I[3], real_T c7_Torque_c[3], real_T c7_results[7]);
static void c7_diag(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_v[3],
                    real_T c7_d[9]);
static void c7_skew_matrix(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_x[3], real_T c7_output[9]);
static void c7_eml_scalar_eg(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_threshold(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_mpower(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_a[9],
                      real_T c7_c[9]);
static void c7_inv3x3(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_x[9],
                      real_T c7_y[9]);
static real_T c7_norm(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_x[9]);
static void c7_eml_warning(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_b_eml_warning(SFc7_UKF_10hzInstanceStruct *chartInstance, char_T
  c7_varargin_2[14]);
static void c7_b_eml_scalar_eg(SFc7_UKF_10hzInstanceStruct *chartInstance);
static void c7_n_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_sprintf, const char_T *c7_identifier, char_T c7_y[14]);
static void c7_o_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, char_T c7_y[14]);
static const mxArray *c7_j_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_p_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_q_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_UKF_10hz, const char_T *c7_identifier);
static uint8_T c7_r_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void init_dsm_address_info(SFc7_UKF_10hzInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c7_w_c_km1_not_empty = false;
  chartInstance->c7_q_i_c_km1_not_empty = false;
  chartInstance->c7_is_active_c7_UKF_10hz = 0U;
}

static void initialize_params_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c7_update_debugger_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct
  *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  int32_T c7_i0;
  real_T c7_u[4];
  const mxArray *c7_b_y = NULL;
  int32_T c7_i1;
  real_T c7_b_u[3];
  const mxArray *c7_c_y = NULL;
  int32_T c7_i2;
  real_T c7_c_u[4];
  const mxArray *c7_d_y = NULL;
  int32_T c7_i3;
  real_T c7_d_u[3];
  const mxArray *c7_e_y = NULL;
  uint8_T c7_hoistedGlobal;
  uint8_T c7_e_u;
  const mxArray *c7_f_y = NULL;
  real_T (*c7_w_s)[3];
  real_T (*c7_q_i_s)[4];
  c7_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellmatrix(5, 1), false);
  for (c7_i0 = 0; c7_i0 < 4; c7_i0++) {
    c7_u[c7_i0] = (*c7_q_i_s)[c7_i0];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  for (c7_i1 = 0; c7_i1 < 3; c7_i1++) {
    c7_b_u[c7_i1] = (*c7_w_s)[c7_i1];
  }

  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", c7_b_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  for (c7_i2 = 0; c7_i2 < 4; c7_i2++) {
    c7_c_u[c7_i2] = chartInstance->c7_q_i_c_km1[c7_i2];
  }

  c7_d_y = NULL;
  if (!chartInstance->c7_q_i_c_km1_not_empty) {
    sf_mex_assign(&c7_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c7_d_y, sf_mex_create("y", c7_c_u, 0, 0U, 1U, 0U, 1, 4),
                  false);
  }

  sf_mex_setcell(c7_y, 2, c7_d_y);
  for (c7_i3 = 0; c7_i3 < 3; c7_i3++) {
    c7_d_u[c7_i3] = chartInstance->c7_w_c_km1[c7_i3];
  }

  c7_e_y = NULL;
  if (!chartInstance->c7_w_c_km1_not_empty) {
    sf_mex_assign(&c7_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c7_e_y, sf_mex_create("y", c7_d_u, 0, 0U, 1U, 0U, 1, 3),
                  false);
  }

  sf_mex_setcell(c7_y, 3, c7_e_y);
  c7_hoistedGlobal = chartInstance->c7_is_active_c7_UKF_10hz;
  c7_e_u = c7_hoistedGlobal;
  c7_f_y = NULL;
  sf_mex_assign(&c7_f_y, sf_mex_create("y", &c7_e_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 4, c7_f_y);
  sf_mex_assign(&c7_st, c7_y, false);
  return c7_st;
}

static void set_sim_state_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_st)
{
  const mxArray *c7_u;
  real_T c7_dv0[4];
  int32_T c7_i4;
  real_T c7_dv1[3];
  int32_T c7_i5;
  real_T c7_dv2[4];
  int32_T c7_i6;
  real_T c7_dv3[3];
  int32_T c7_i7;
  real_T (*c7_q_i_s)[4];
  real_T (*c7_w_s)[3];
  c7_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c7_doneDoubleBufferReInit = true;
  c7_u = sf_mex_dup(c7_st);
  c7_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 0)),
                        "q_i_s", c7_dv0);
  for (c7_i4 = 0; c7_i4 < 4; c7_i4++) {
    (*c7_q_i_s)[c7_i4] = c7_dv0[c7_i4];
  }

  c7_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 1)),
                        "w_s", c7_dv1);
  for (c7_i5 = 0; c7_i5 < 3; c7_i5++) {
    (*c7_w_s)[c7_i5] = c7_dv1[c7_i5];
  }

  c7_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 2)),
                      "q_i_c_km1", c7_dv2);
  for (c7_i6 = 0; c7_i6 < 4; c7_i6++) {
    chartInstance->c7_q_i_c_km1[c7_i6] = c7_dv2[c7_i6];
  }

  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 3)),
                        "w_c_km1", c7_dv3);
  for (c7_i7 = 0; c7_i7 < 3; c7_i7++) {
    chartInstance->c7_w_c_km1[c7_i7] = c7_dv3[c7_i7];
  }

  chartInstance->c7_is_active_c7_UKF_10hz = c7_q_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c7_u, 4)), "is_active_c7_UKF_10hz");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_UKF_10hz(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c7_i8;
  int32_T c7_i9;
  int32_T c7_i10;
  int32_T c7_i11;
  int32_T c7_i12;
  int32_T c7_i13;
  int32_T c7_i14;
  real_T *c7_Ts;
  real_T (*c7_q_s_c)[4];
  real_T (*c7_q_i_s)[4];
  real_T (*c7_w_s)[3];
  real_T (*c7_q0_i_s)[4];
  real_T (*c7_Torque_s)[3];
  real_T (*c7_w_init_s)[3];
  real_T (*c7_I_c)[3];
  c7_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c7_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
  c7_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c7_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c7_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c7_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c7_sfEvent);
  for (c7_i8 = 0; c7_i8 < 3; c7_i8++) {
    _SFD_DATA_RANGE_CHECK((*c7_I_c)[c7_i8], 0U);
  }

  for (c7_i9 = 0; c7_i9 < 3; c7_i9++) {
    _SFD_DATA_RANGE_CHECK((*c7_w_init_s)[c7_i9], 1U);
  }

  for (c7_i10 = 0; c7_i10 < 3; c7_i10++) {
    _SFD_DATA_RANGE_CHECK((*c7_Torque_s)[c7_i10], 2U);
  }

  for (c7_i11 = 0; c7_i11 < 4; c7_i11++) {
    _SFD_DATA_RANGE_CHECK((*c7_q0_i_s)[c7_i11], 3U);
  }

  chartInstance->c7_sfEvent = CALL_EVENT;
  c7_chartstep_c7_UKF_10hz(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_UKF_10hzMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c7_i12 = 0; c7_i12 < 3; c7_i12++) {
    _SFD_DATA_RANGE_CHECK((*c7_w_s)[c7_i12], 4U);
  }

  for (c7_i13 = 0; c7_i13 < 4; c7_i13++) {
    _SFD_DATA_RANGE_CHECK((*c7_q_i_s)[c7_i13], 5U);
  }

  for (c7_i14 = 0; c7_i14 < 4; c7_i14++) {
    _SFD_DATA_RANGE_CHECK((*c7_q_s_c)[c7_i14], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c7_Ts, 7U);
}

static void c7_chartstep_c7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  real_T c7_hoistedGlobal;
  int32_T c7_i15;
  real_T c7_I_c[3];
  int32_T c7_i16;
  real_T c7_w_init_s[3];
  int32_T c7_i17;
  real_T c7_Torque_s[3];
  int32_T c7_i18;
  real_T c7_q0_i_s[4];
  int32_T c7_i19;
  real_T c7_q_s_c[4];
  real_T c7_Ts;
  uint32_T c7_debug_family_var_map[22];
  real_T c7_Torque_c_temp[4];
  real_T c7_Torque_c[3];
  real_T c7_w_c_temp[4];
  real_T c7_x_km1[7];
  real_T c7_k1[7];
  real_T c7_k2[7];
  real_T c7_k3[7];
  real_T c7_k4[7];
  real_T c7_x_k[7];
  real_T c7_w_s_temp[4];
  real_T c7_nargin = 6.0;
  real_T c7_nargout = 2.0;
  real_T c7_w_s[3];
  real_T c7_q_i_s[4];
  int32_T c7_i20;
  real_T c7_b_q_s_c[4];
  real_T c7_dv4[4];
  int32_T c7_i21;
  real_T c7_dv5[4];
  real_T c7_dv6[4];
  int32_T c7_i22;
  int32_T c7_i23;
  real_T c7_dv7[4];
  int32_T c7_i24;
  real_T c7_c_q_s_c[4];
  real_T c7_dv8[4];
  int32_T c7_i25;
  int32_T c7_i26;
  int32_T c7_i27;
  int32_T c7_i28;
  int32_T c7_i29;
  real_T c7_d_q_s_c[4];
  int32_T c7_i30;
  real_T c7_dv9[4];
  real_T c7_dv10[4];
  int32_T c7_i31;
  int32_T c7_i32;
  real_T c7_dv11[4];
  int32_T c7_i33;
  real_T c7_e_q_s_c[4];
  real_T c7_dv12[4];
  int32_T c7_i34;
  int32_T c7_i35;
  int32_T c7_i36;
  real_T c7_b_q_i_s[4];
  int32_T c7_i37;
  real_T c7_f_q_s_c[4];
  real_T c7_dv13[4];
  int32_T c7_i38;
  int32_T c7_i39;
  int32_T c7_i40;
  int32_T c7_i41;
  real_T c7_b_x_km1[7];
  int32_T c7_i42;
  real_T c7_b_I_c[3];
  int32_T c7_i43;
  real_T c7_b_Torque_c[3];
  real_T c7_dv14[7];
  int32_T c7_i44;
  int32_T c7_i45;
  real_T c7_b[7];
  int32_T c7_i46;
  real_T c7_b_b;
  int32_T c7_i47;
  int32_T c7_i48;
  real_T c7_c_x_km1[7];
  int32_T c7_i49;
  real_T c7_c_I_c[3];
  int32_T c7_i50;
  real_T c7_c_Torque_c[3];
  real_T c7_dv15[7];
  int32_T c7_i51;
  int32_T c7_i52;
  int32_T c7_i53;
  real_T c7_c_b;
  int32_T c7_i54;
  int32_T c7_i55;
  real_T c7_d_x_km1[7];
  int32_T c7_i56;
  real_T c7_d_I_c[3];
  int32_T c7_i57;
  real_T c7_d_Torque_c[3];
  real_T c7_dv16[7];
  int32_T c7_i58;
  int32_T c7_i59;
  real_T c7_d_b;
  int32_T c7_i60;
  int32_T c7_i61;
  real_T c7_e_x_km1[7];
  int32_T c7_i62;
  real_T c7_e_I_c[3];
  int32_T c7_i63;
  real_T c7_e_Torque_c[3];
  real_T c7_dv17[7];
  int32_T c7_i64;
  int32_T c7_i65;
  int32_T c7_i66;
  int32_T c7_i67;
  real_T c7_e_b[7];
  int32_T c7_i68;
  int32_T c7_i69;
  real_T c7_f_b;
  int32_T c7_i70;
  int32_T c7_i71;
  int32_T c7_i72;
  int32_T c7_i73;
  int32_T c7_i74;
  int32_T c7_i75;
  real_T c7_g_q_s_c[4];
  int32_T c7_i76;
  real_T c7_dv18[4];
  int32_T c7_i77;
  real_T c7_dv19[4];
  real_T c7_dv20[4];
  int32_T c7_i78;
  int32_T c7_i79;
  real_T c7_h_q_s_c[4];
  real_T c7_dv21[4];
  int32_T c7_i80;
  int32_T c7_i81;
  real_T c7_i_q_s_c[4];
  real_T c7_dv22[4];
  int32_T c7_i82;
  real_T c7_dv23[4];
  int32_T c7_i83;
  real_T c7_dv24[4];
  real_T c7_dv25[4];
  int32_T c7_i84;
  int32_T c7_i85;
  int32_T c7_i86;
  int32_T c7_i87;
  real_T *c7_b_Ts;
  real_T (*c7_b_w_s)[3];
  real_T (*c7_c_q_i_s)[4];
  real_T (*c7_j_q_s_c)[4];
  real_T (*c7_b_q0_i_s)[4];
  real_T (*c7_b_Torque_s)[3];
  real_T (*c7_b_w_init_s)[3];
  real_T (*c7_f_I_c)[3];
  boolean_T guard1 = false;
  c7_b_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c7_j_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
  c7_c_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_b_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c7_b_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c7_b_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c7_f_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c7_sfEvent);
  c7_hoistedGlobal = *c7_b_Ts;
  for (c7_i15 = 0; c7_i15 < 3; c7_i15++) {
    c7_I_c[c7_i15] = (*c7_f_I_c)[c7_i15];
  }

  for (c7_i16 = 0; c7_i16 < 3; c7_i16++) {
    c7_w_init_s[c7_i16] = (*c7_b_w_init_s)[c7_i16];
  }

  for (c7_i17 = 0; c7_i17 < 3; c7_i17++) {
    c7_Torque_s[c7_i17] = (*c7_b_Torque_s)[c7_i17];
  }

  for (c7_i18 = 0; c7_i18 < 4; c7_i18++) {
    c7_q0_i_s[c7_i18] = (*c7_b_q0_i_s)[c7_i18];
  }

  for (c7_i19 = 0; c7_i19 < 4; c7_i19++) {
    c7_q_s_c[c7_i19] = (*c7_j_q_s_c)[c7_i19];
  }

  c7_Ts = c7_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 22U, 22U, c7_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_Torque_c_temp, 0U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_Torque_c, 1U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_w_c_temp, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x_km1, 3U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_k1, 4U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_k2, 5U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_k3, 6U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_k4, 7U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x_k, 8U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_w_s_temp, 9U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 10U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 11U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_I_c, 12U, c7_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_w_init_s, 13U, c7_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_Torque_s, 14U, c7_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_q0_i_s, 15U, c7_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_q_s_c, 16U, c7_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_Ts, 17U, c7_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_w_s, 18U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_q_i_s, 19U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c7_w_c_km1, 20U,
    c7_b_sf_marshallOut, c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c7_q_i_c_km1, 21U,
    c7_sf_marshallOut, c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 7);
  for (c7_i20 = 0; c7_i20 < 4; c7_i20++) {
    c7_b_q_s_c[c7_i20] = c7_q_s_c[c7_i20];
  }

  c7_quatinv(chartInstance, c7_b_q_s_c, c7_dv4);
  for (c7_i21 = 0; c7_i21 < 4; c7_i21++) {
    c7_dv5[c7_i21] = c7_dv4[c7_i21];
  }

  c7_dv6[0] = 0.0;
  for (c7_i22 = 0; c7_i22 < 3; c7_i22++) {
    c7_dv6[c7_i22 + 1] = c7_Torque_s[c7_i22];
  }

  c7_quatmultiply(chartInstance, c7_dv5, c7_dv6, c7_dv4);
  for (c7_i23 = 0; c7_i23 < 4; c7_i23++) {
    c7_dv7[c7_i23] = c7_dv4[c7_i23];
  }

  for (c7_i24 = 0; c7_i24 < 4; c7_i24++) {
    c7_c_q_s_c[c7_i24] = c7_q_s_c[c7_i24];
  }

  c7_quatmultiply(chartInstance, c7_dv7, c7_c_q_s_c, c7_dv8);
  for (c7_i25 = 0; c7_i25 < 4; c7_i25++) {
    c7_Torque_c_temp[c7_i25] = c7_dv8[c7_i25];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 8);
  for (c7_i26 = 0; c7_i26 < 3; c7_i26++) {
    c7_Torque_c[c7_i26] = c7_Torque_c_temp[c7_i26 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 10);
  guard1 = false;
  if (CV_EML_COND(0, 1, 0, !chartInstance->c7_w_c_km1_not_empty)) {
    guard1 = true;
  } else if (CV_EML_COND(0, 1, 1, !chartInstance->c7_q_i_c_km1_not_empty)) {
    guard1 = true;
  } else {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 0, false);
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 22);
    for (c7_i39 = 0; c7_i39 < 4; c7_i39++) {
      c7_x_km1[c7_i39] = chartInstance->c7_q_i_c_km1[c7_i39];
    }

    for (c7_i40 = 0; c7_i40 < 3; c7_i40++) {
      c7_x_km1[c7_i40 + 4] = chartInstance->c7_w_c_km1[c7_i40];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 30);
    for (c7_i41 = 0; c7_i41 < 7; c7_i41++) {
      c7_b_x_km1[c7_i41] = c7_x_km1[c7_i41];
    }

    for (c7_i42 = 0; c7_i42 < 3; c7_i42++) {
      c7_b_I_c[c7_i42] = c7_I_c[c7_i42];
    }

    for (c7_i43 = 0; c7_i43 < 3; c7_i43++) {
      c7_b_Torque_c[c7_i43] = c7_Torque_c[c7_i43];
    }

    c7_Kinematics(chartInstance, c7_b_x_km1, c7_b_I_c, c7_b_Torque_c, c7_dv14);
    for (c7_i44 = 0; c7_i44 < 7; c7_i44++) {
      c7_k1[c7_i44] = c7_dv14[c7_i44];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 31);
    for (c7_i45 = 0; c7_i45 < 7; c7_i45++) {
      c7_b[c7_i45] = c7_k1[c7_i45];
    }

    for (c7_i46 = 0; c7_i46 < 7; c7_i46++) {
      c7_b[c7_i46] *= 0.5;
    }

    c7_b_b = c7_Ts;
    for (c7_i47 = 0; c7_i47 < 7; c7_i47++) {
      c7_b[c7_i47] *= c7_b_b;
    }

    for (c7_i48 = 0; c7_i48 < 7; c7_i48++) {
      c7_c_x_km1[c7_i48] = c7_x_km1[c7_i48] + c7_b[c7_i48];
    }

    for (c7_i49 = 0; c7_i49 < 3; c7_i49++) {
      c7_c_I_c[c7_i49] = c7_I_c[c7_i49];
    }

    for (c7_i50 = 0; c7_i50 < 3; c7_i50++) {
      c7_c_Torque_c[c7_i50] = c7_Torque_c[c7_i50];
    }

    c7_Kinematics(chartInstance, c7_c_x_km1, c7_c_I_c, c7_c_Torque_c, c7_dv15);
    for (c7_i51 = 0; c7_i51 < 7; c7_i51++) {
      c7_k2[c7_i51] = c7_dv15[c7_i51];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 32);
    for (c7_i52 = 0; c7_i52 < 7; c7_i52++) {
      c7_b[c7_i52] = c7_k2[c7_i52];
    }

    for (c7_i53 = 0; c7_i53 < 7; c7_i53++) {
      c7_b[c7_i53] *= 0.5;
    }

    c7_c_b = c7_Ts;
    for (c7_i54 = 0; c7_i54 < 7; c7_i54++) {
      c7_b[c7_i54] *= c7_c_b;
    }

    for (c7_i55 = 0; c7_i55 < 7; c7_i55++) {
      c7_d_x_km1[c7_i55] = c7_x_km1[c7_i55] + c7_b[c7_i55];
    }

    for (c7_i56 = 0; c7_i56 < 3; c7_i56++) {
      c7_d_I_c[c7_i56] = c7_I_c[c7_i56];
    }

    for (c7_i57 = 0; c7_i57 < 3; c7_i57++) {
      c7_d_Torque_c[c7_i57] = c7_Torque_c[c7_i57];
    }

    c7_Kinematics(chartInstance, c7_d_x_km1, c7_d_I_c, c7_d_Torque_c, c7_dv16);
    for (c7_i58 = 0; c7_i58 < 7; c7_i58++) {
      c7_k3[c7_i58] = c7_dv16[c7_i58];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 33);
    for (c7_i59 = 0; c7_i59 < 7; c7_i59++) {
      c7_b[c7_i59] = c7_k3[c7_i59];
    }

    c7_d_b = c7_Ts;
    for (c7_i60 = 0; c7_i60 < 7; c7_i60++) {
      c7_b[c7_i60] *= c7_d_b;
    }

    for (c7_i61 = 0; c7_i61 < 7; c7_i61++) {
      c7_e_x_km1[c7_i61] = c7_x_km1[c7_i61] + c7_b[c7_i61];
    }

    for (c7_i62 = 0; c7_i62 < 3; c7_i62++) {
      c7_e_I_c[c7_i62] = c7_I_c[c7_i62];
    }

    for (c7_i63 = 0; c7_i63 < 3; c7_i63++) {
      c7_e_Torque_c[c7_i63] = c7_Torque_c[c7_i63];
    }

    c7_Kinematics(chartInstance, c7_e_x_km1, c7_e_I_c, c7_e_Torque_c, c7_dv17);
    for (c7_i64 = 0; c7_i64 < 7; c7_i64++) {
      c7_k4[c7_i64] = c7_dv17[c7_i64];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 35);
    for (c7_i65 = 0; c7_i65 < 7; c7_i65++) {
      c7_b[c7_i65] = c7_k2[c7_i65];
    }

    for (c7_i66 = 0; c7_i66 < 7; c7_i66++) {
      c7_b[c7_i66] *= 2.0;
    }

    for (c7_i67 = 0; c7_i67 < 7; c7_i67++) {
      c7_e_b[c7_i67] = c7_k3[c7_i67];
    }

    for (c7_i68 = 0; c7_i68 < 7; c7_i68++) {
      c7_e_b[c7_i68] *= 2.0;
    }

    for (c7_i69 = 0; c7_i69 < 7; c7_i69++) {
      c7_b[c7_i69] = ((c7_k1[c7_i69] + c7_b[c7_i69]) + c7_e_b[c7_i69]) +
        c7_k4[c7_i69];
    }

    c7_f_b = c7_Ts;
    for (c7_i70 = 0; c7_i70 < 7; c7_i70++) {
      c7_b[c7_i70] *= c7_f_b;
    }

    for (c7_i71 = 0; c7_i71 < 7; c7_i71++) {
      c7_b[c7_i71] /= 6.0;
    }

    for (c7_i72 = 0; c7_i72 < 7; c7_i72++) {
      c7_x_k[c7_i72] = c7_x_km1[c7_i72] + c7_b[c7_i72];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 41);
    for (c7_i73 = 0; c7_i73 < 4; c7_i73++) {
      chartInstance->c7_q_i_c_km1[c7_i73] = c7_x_k[c7_i73];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 42);
    for (c7_i74 = 0; c7_i74 < 3; c7_i74++) {
      chartInstance->c7_w_c_km1[c7_i74] = c7_x_k[c7_i74 + 4];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 45);
    for (c7_i75 = 0; c7_i75 < 4; c7_i75++) {
      c7_g_q_s_c[c7_i75] = c7_q_s_c[c7_i75];
    }

    c7_quatinv(chartInstance, c7_g_q_s_c, c7_dv4);
    for (c7_i76 = 0; c7_i76 < 4; c7_i76++) {
      c7_dv18[c7_i76] = chartInstance->c7_q_i_c_km1[c7_i76];
    }

    for (c7_i77 = 0; c7_i77 < 4; c7_i77++) {
      c7_dv19[c7_i77] = c7_dv4[c7_i77];
    }

    c7_quatmultiply(chartInstance, c7_dv18, c7_dv19, c7_dv20);
    for (c7_i78 = 0; c7_i78 < 4; c7_i78++) {
      c7_q_i_s[c7_i78] = c7_dv20[c7_i78];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 46);
    for (c7_i79 = 0; c7_i79 < 4; c7_i79++) {
      c7_h_q_s_c[c7_i79] = c7_q_s_c[c7_i79];
    }

    c7_dv21[0] = 0.0;
    for (c7_i80 = 0; c7_i80 < 3; c7_i80++) {
      c7_dv21[c7_i80 + 1] = chartInstance->c7_w_c_km1[c7_i80];
    }

    c7_quatmultiply(chartInstance, c7_h_q_s_c, c7_dv21, c7_dv4);
    for (c7_i81 = 0; c7_i81 < 4; c7_i81++) {
      c7_i_q_s_c[c7_i81] = c7_q_s_c[c7_i81];
    }

    c7_quatinv(chartInstance, c7_i_q_s_c, c7_dv22);
    for (c7_i82 = 0; c7_i82 < 4; c7_i82++) {
      c7_dv23[c7_i82] = c7_dv4[c7_i82];
    }

    for (c7_i83 = 0; c7_i83 < 4; c7_i83++) {
      c7_dv24[c7_i83] = c7_dv22[c7_i83];
    }

    c7_quatmultiply(chartInstance, c7_dv23, c7_dv24, c7_dv25);
    for (c7_i84 = 0; c7_i84 < 4; c7_i84++) {
      c7_w_s_temp[c7_i84] = c7_dv25[c7_i84];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 47);
    for (c7_i85 = 0; c7_i85 < 3; c7_i85++) {
      c7_w_s[c7_i85] = c7_w_s_temp[c7_i85 + 1];
    }
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, true);
    CV_EML_IF(0, 1, 0, true);
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 12);
    for (c7_i27 = 0; c7_i27 < 3; c7_i27++) {
      c7_w_s[c7_i27] = c7_w_init_s[c7_i27];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 13);
    for (c7_i28 = 0; c7_i28 < 4; c7_i28++) {
      c7_q_i_s[c7_i28] = c7_q0_i_s[c7_i28];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 16);
    for (c7_i29 = 0; c7_i29 < 4; c7_i29++) {
      c7_d_q_s_c[c7_i29] = c7_q_s_c[c7_i29];
    }

    c7_quatinv(chartInstance, c7_d_q_s_c, c7_dv4);
    for (c7_i30 = 0; c7_i30 < 4; c7_i30++) {
      c7_dv9[c7_i30] = c7_dv4[c7_i30];
    }

    c7_dv10[0] = 0.0;
    for (c7_i31 = 0; c7_i31 < 3; c7_i31++) {
      c7_dv10[c7_i31 + 1] = c7_w_s[c7_i31];
    }

    c7_quatmultiply(chartInstance, c7_dv9, c7_dv10, c7_dv4);
    for (c7_i32 = 0; c7_i32 < 4; c7_i32++) {
      c7_dv11[c7_i32] = c7_dv4[c7_i32];
    }

    for (c7_i33 = 0; c7_i33 < 4; c7_i33++) {
      c7_e_q_s_c[c7_i33] = c7_q_s_c[c7_i33];
    }

    c7_quatmultiply(chartInstance, c7_dv11, c7_e_q_s_c, c7_dv12);
    for (c7_i34 = 0; c7_i34 < 4; c7_i34++) {
      c7_w_c_temp[c7_i34] = c7_dv12[c7_i34];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 17);
    for (c7_i35 = 0; c7_i35 < 3; c7_i35++) {
      chartInstance->c7_w_c_km1[c7_i35] = c7_w_c_temp[c7_i35 + 1];
    }

    chartInstance->c7_w_c_km1_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 18);
    for (c7_i36 = 0; c7_i36 < 4; c7_i36++) {
      c7_b_q_i_s[c7_i36] = c7_q_i_s[c7_i36];
    }

    for (c7_i37 = 0; c7_i37 < 4; c7_i37++) {
      c7_f_q_s_c[c7_i37] = c7_q_s_c[c7_i37];
    }

    c7_quatmultiply(chartInstance, c7_b_q_i_s, c7_f_q_s_c, c7_dv13);
    for (c7_i38 = 0; c7_i38 < 4; c7_i38++) {
      chartInstance->c7_q_i_c_km1[c7_i38] = c7_dv13[c7_i38];
    }

    chartInstance->c7_q_i_c_km1_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -47);
  _SFD_SYMBOL_SCOPE_POP();
  for (c7_i86 = 0; c7_i86 < 3; c7_i86++) {
    (*c7_b_w_s)[c7_i86] = c7_w_s[c7_i86];
  }

  for (c7_i87 = 0; c7_i87 < 4; c7_i87++) {
    (*c7_c_q_i_s)[c7_i87] = c7_q_i_s[c7_i87];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c7_sfEvent);
}

static void initSimStructsc7_UKF_10hz(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber)
{
  (void)c7_machineNumber;
  (void)c7_chartNumber;
  (void)c7_instanceNumber;
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i88;
  real_T c7_b_inData[4];
  int32_T c7_i89;
  real_T c7_u[4];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i88 = 0; c7_i88 < 4; c7_i88++) {
    c7_b_inData[c7_i88] = (*(real_T (*)[4])c7_inData)[c7_i88];
  }

  for (c7_i89 = 0; c7_i89 < 4; c7_i89++) {
    c7_u[c7_i89] = c7_b_inData[c7_i89];
  }

  c7_y = NULL;
  if (!chartInstance->c7_q_i_c_km1_not_empty) {
    sf_mex_assign(&c7_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), false);
  }

  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_q_i_c_km1, const char_T *c7_identifier, real_T c7_y[4])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_q_i_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_q_i_c_km1);
}

static void c7_b_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4])
{
  real_T c7_dv26[4];
  int32_T c7_i90;
  if (mxIsEmpty(c7_u)) {
    chartInstance->c7_q_i_c_km1_not_empty = false;
  } else {
    chartInstance->c7_q_i_c_km1_not_empty = true;
    sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv26, 1, 0, 0U, 1, 0U, 1, 4);
    for (c7_i90 = 0; c7_i90 < 4; c7_i90++) {
      c7_y[c7_i90] = c7_dv26[c7_i90];
    }
  }

  sf_mex_destroy(&c7_u);
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_q_i_c_km1;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[4];
  int32_T c7_i91;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_b_q_i_c_km1 = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_q_i_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_q_i_c_km1);
  for (c7_i91 = 0; c7_i91 < 4; c7_i91++) {
    (*(real_T (*)[4])c7_outData)[c7_i91] = c7_y[c7_i91];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i92;
  real_T c7_b_inData[3];
  int32_T c7_i93;
  real_T c7_u[3];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i92 = 0; c7_i92 < 3; c7_i92++) {
    c7_b_inData[c7_i92] = (*(real_T (*)[3])c7_inData)[c7_i92];
  }

  for (c7_i93 = 0; c7_i93 < 3; c7_i93++) {
    c7_u[c7_i93] = c7_b_inData[c7_i93];
  }

  c7_y = NULL;
  if (!chartInstance->c7_w_c_km1_not_empty) {
    sf_mex_assign(&c7_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), false);
  }

  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_c_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_w_c_km1, const char_T *c7_identifier, real_T c7_y[3])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_w_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_w_c_km1);
}

static void c7_d_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3])
{
  real_T c7_dv27[3];
  int32_T c7_i94;
  if (mxIsEmpty(c7_u)) {
    chartInstance->c7_w_c_km1_not_empty = false;
  } else {
    chartInstance->c7_w_c_km1_not_empty = true;
    sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv27, 1, 0, 0U, 1, 0U, 1, 3);
    for (c7_i94 = 0; c7_i94 < 3; c7_i94++) {
      c7_y[c7_i94] = c7_dv27[c7_i94];
    }
  }

  sf_mex_destroy(&c7_u);
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_w_c_km1;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[3];
  int32_T c7_i95;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_b_w_c_km1 = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_w_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_w_c_km1);
  for (c7_i95 = 0; c7_i95 < 3; c7_i95++) {
    (*(real_T (*)[3])c7_outData)[c7_i95] = c7_y[c7_i95];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i96;
  real_T c7_b_inData[4];
  int32_T c7_i97;
  real_T c7_u[4];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i96 = 0; c7_i96 < 4; c7_i96++) {
    c7_b_inData[c7_i96] = (*(real_T (*)[4])c7_inData)[c7_i96];
  }

  for (c7_i97 = 0; c7_i97 < 4; c7_i97++) {
    c7_u[c7_i97] = c7_b_inData[c7_i97];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_e_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_q_i_s, const char_T *c7_identifier, real_T c7_y[4])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_q_i_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_q_i_s);
}

static void c7_f_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4])
{
  real_T c7_dv28[4];
  int32_T c7_i98;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv28, 1, 0, 0U, 1, 0U, 1, 4);
  for (c7_i98 = 0; c7_i98 < 4; c7_i98++) {
    c7_y[c7_i98] = c7_dv28[c7_i98];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_q_i_s;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[4];
  int32_T c7_i99;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_q_i_s = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_q_i_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_q_i_s);
  for (c7_i99 = 0; c7_i99 < 4; c7_i99++) {
    (*(real_T (*)[4])c7_outData)[c7_i99] = c7_y[c7_i99];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i100;
  real_T c7_b_inData[3];
  int32_T c7_i101;
  real_T c7_u[3];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i100 = 0; c7_i100 < 3; c7_i100++) {
    c7_b_inData[c7_i100] = (*(real_T (*)[3])c7_inData)[c7_i100];
  }

  for (c7_i101 = 0; c7_i101 < 3; c7_i101++) {
    c7_u[c7_i101] = c7_b_inData[c7_i101];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_g_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_w_s, const char_T *c7_identifier, real_T c7_y[3])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_w_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_w_s);
}

static void c7_h_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3])
{
  real_T c7_dv29[3];
  int32_T c7_i102;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv29, 1, 0, 0U, 1, 0U, 1, 3);
  for (c7_i102 = 0; c7_i102 < 3; c7_i102++) {
    c7_y[c7_i102] = c7_dv29[c7_i102];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_w_s;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[3];
  int32_T c7_i103;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_w_s = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_w_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_w_s);
  for (c7_i103 = 0; c7_i103 < 3; c7_i103++) {
    (*(real_T (*)[3])c7_outData)[c7_i103] = c7_y[c7_i103];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static real_T c7_i_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d0, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_nargout;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_nargout = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_nargout), &c7_thisId);
  sf_mex_destroy(&c7_nargout);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i104;
  real_T c7_b_inData[7];
  int32_T c7_i105;
  real_T c7_u[7];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i104 = 0; c7_i104 < 7; c7_i104++) {
    c7_b_inData[c7_i104] = (*(real_T (*)[7])c7_inData)[c7_i104];
  }

  for (c7_i105 = 0; c7_i105 < 7; c7_i105++) {
    c7_u[c7_i105] = c7_b_inData[c7_i105];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 7), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_j_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[7])
{
  real_T c7_dv30[7];
  int32_T c7_i106;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv30, 1, 0, 0U, 1, 0U, 1, 7);
  for (c7_i106 = 0; c7_i106 < 7; c7_i106++) {
    c7_y[c7_i106] = c7_dv30[c7_i106];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_x_k;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[7];
  int32_T c7_i107;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_x_k = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_x_k), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_x_k);
  for (c7_i107 = 0; c7_i107 < 7; c7_i107++) {
    (*(real_T (*)[7])c7_outData)[c7_i107] = c7_y[c7_i107];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i108;
  real_T c7_b_inData[4];
  int32_T c7_i109;
  real_T c7_u[4];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i108 = 0; c7_i108 < 4; c7_i108++) {
    c7_b_inData[c7_i108] = (*(real_T (*)[4])c7_inData)[c7_i108];
  }

  for (c7_i109 = 0; c7_i109 < 4; c7_i109++) {
    c7_u[c7_i109] = c7_b_inData[c7_i109];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_k_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[4])
{
  real_T c7_dv31[4];
  int32_T c7_i110;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv31, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c7_i110 = 0; c7_i110 < 4; c7_i110++) {
    c7_y[c7_i110] = c7_dv31[c7_i110];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_r;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[4];
  int32_T c7_i111;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_r = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_r), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_r);
  for (c7_i111 = 0; c7_i111 < 4; c7_i111++) {
    (*(real_T (*)[4])c7_outData)[c7_i111] = c7_y[c7_i111];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i112;
  real_T c7_b_inData[3];
  int32_T c7_i113;
  real_T c7_u[3];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i112 = 0; c7_i112 < 3; c7_i112++) {
    c7_b_inData[c7_i112] = (*(real_T (*)[3])c7_inData)[c7_i112];
  }

  for (c7_i113 = 0; c7_i113 < 3; c7_i113++) {
    c7_u[c7_i113] = c7_b_inData[c7_i113];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_l_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[3])
{
  real_T c7_dv32[3];
  int32_T c7_i114;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv32, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c7_i114 = 0; c7_i114 < 3; c7_i114++) {
    c7_y[c7_i114] = c7_dv32[c7_i114];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_vec;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[3];
  int32_T c7_i115;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_vec = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_vec), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_vec);
  for (c7_i115 = 0; c7_i115 < 3; c7_i115++) {
    (*(real_T (*)[3])c7_outData)[c7_i115] = c7_y[c7_i115];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_i_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i116;
  int32_T c7_i117;
  int32_T c7_i118;
  real_T c7_b_inData[9];
  int32_T c7_i119;
  int32_T c7_i120;
  int32_T c7_i121;
  real_T c7_u[9];
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i116 = 0;
  for (c7_i117 = 0; c7_i117 < 3; c7_i117++) {
    for (c7_i118 = 0; c7_i118 < 3; c7_i118++) {
      c7_b_inData[c7_i118 + c7_i116] = (*(real_T (*)[9])c7_inData)[c7_i118 +
        c7_i116];
    }

    c7_i116 += 3;
  }

  c7_i119 = 0;
  for (c7_i120 = 0; c7_i120 < 3; c7_i120++) {
    for (c7_i121 = 0; c7_i121 < 3; c7_i121++) {
      c7_u[c7_i121 + c7_i119] = c7_b_inData[c7_i121 + c7_i119];
    }

    c7_i119 += 3;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_m_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, real_T c7_y[9])
{
  real_T c7_dv33[9];
  int32_T c7_i122;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv33, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c7_i122 = 0; c7_i122 < 9; c7_i122++) {
    c7_y[c7_i122] = c7_dv33[c7_i122];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_output;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[9];
  int32_T c7_i123;
  int32_T c7_i124;
  int32_T c7_i125;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_output = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_output), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_output);
  c7_i123 = 0;
  for (c7_i124 = 0; c7_i124 < 3; c7_i124++) {
    for (c7_i125 = 0; c7_i125 < 3; c7_i125++) {
      (*(real_T (*)[9])c7_outData)[c7_i125 + c7_i123] = c7_y[c7_i125 + c7_i123];
    }

    c7_i123 += 3;
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

const mxArray *sf_c7_UKF_10hz_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  sf_mex_assign(&c7_nameCaptureInfo, sf_mex_createstruct("structure", 2, 77, 1),
                false);
  c7_info_helper(&c7_nameCaptureInfo);
  c7_b_info_helper(&c7_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs0 = NULL;
  const mxArray *c7_lhs0 = NULL;
  const mxArray *c7_rhs1 = NULL;
  const mxArray *c7_lhs1 = NULL;
  const mxArray *c7_rhs2 = NULL;
  const mxArray *c7_lhs2 = NULL;
  const mxArray *c7_rhs3 = NULL;
  const mxArray *c7_lhs3 = NULL;
  const mxArray *c7_rhs4 = NULL;
  const mxArray *c7_lhs4 = NULL;
  const mxArray *c7_rhs5 = NULL;
  const mxArray *c7_lhs5 = NULL;
  const mxArray *c7_rhs6 = NULL;
  const mxArray *c7_lhs6 = NULL;
  const mxArray *c7_rhs7 = NULL;
  const mxArray *c7_lhs7 = NULL;
  const mxArray *c7_rhs8 = NULL;
  const mxArray *c7_lhs8 = NULL;
  const mxArray *c7_rhs9 = NULL;
  const mxArray *c7_lhs9 = NULL;
  const mxArray *c7_rhs10 = NULL;
  const mxArray *c7_lhs10 = NULL;
  const mxArray *c7_rhs11 = NULL;
  const mxArray *c7_lhs11 = NULL;
  const mxArray *c7_rhs12 = NULL;
  const mxArray *c7_lhs12 = NULL;
  const mxArray *c7_rhs13 = NULL;
  const mxArray *c7_lhs13 = NULL;
  const mxArray *c7_rhs14 = NULL;
  const mxArray *c7_lhs14 = NULL;
  const mxArray *c7_rhs15 = NULL;
  const mxArray *c7_lhs15 = NULL;
  const mxArray *c7_rhs16 = NULL;
  const mxArray *c7_lhs16 = NULL;
  const mxArray *c7_rhs17 = NULL;
  const mxArray *c7_lhs17 = NULL;
  const mxArray *c7_rhs18 = NULL;
  const mxArray *c7_lhs18 = NULL;
  const mxArray *c7_rhs19 = NULL;
  const mxArray *c7_lhs19 = NULL;
  const mxArray *c7_rhs20 = NULL;
  const mxArray *c7_lhs20 = NULL;
  const mxArray *c7_rhs21 = NULL;
  const mxArray *c7_lhs21 = NULL;
  const mxArray *c7_rhs22 = NULL;
  const mxArray *c7_lhs22 = NULL;
  const mxArray *c7_rhs23 = NULL;
  const mxArray *c7_lhs23 = NULL;
  const mxArray *c7_rhs24 = NULL;
  const mxArray *c7_lhs24 = NULL;
  const mxArray *c7_rhs25 = NULL;
  const mxArray *c7_lhs25 = NULL;
  const mxArray *c7_rhs26 = NULL;
  const mxArray *c7_lhs26 = NULL;
  const mxArray *c7_rhs27 = NULL;
  const mxArray *c7_lhs27 = NULL;
  const mxArray *c7_rhs28 = NULL;
  const mxArray *c7_lhs28 = NULL;
  const mxArray *c7_rhs29 = NULL;
  const mxArray *c7_lhs29 = NULL;
  const mxArray *c7_rhs30 = NULL;
  const mxArray *c7_lhs30 = NULL;
  const mxArray *c7_rhs31 = NULL;
  const mxArray *c7_lhs31 = NULL;
  const mxArray *c7_rhs32 = NULL;
  const mxArray *c7_lhs32 = NULL;
  const mxArray *c7_rhs33 = NULL;
  const mxArray *c7_lhs33 = NULL;
  const mxArray *c7_rhs34 = NULL;
  const mxArray *c7_lhs34 = NULL;
  const mxArray *c7_rhs35 = NULL;
  const mxArray *c7_lhs35 = NULL;
  const mxArray *c7_rhs36 = NULL;
  const mxArray *c7_lhs36 = NULL;
  const mxArray *c7_rhs37 = NULL;
  const mxArray *c7_lhs37 = NULL;
  const mxArray *c7_rhs38 = NULL;
  const mxArray *c7_lhs38 = NULL;
  const mxArray *c7_rhs39 = NULL;
  const mxArray *c7_lhs39 = NULL;
  const mxArray *c7_rhs40 = NULL;
  const mxArray *c7_lhs40 = NULL;
  const mxArray *c7_rhs41 = NULL;
  const mxArray *c7_lhs41 = NULL;
  const mxArray *c7_rhs42 = NULL;
  const mxArray *c7_lhs42 = NULL;
  const mxArray *c7_rhs43 = NULL;
  const mxArray *c7_lhs43 = NULL;
  const mxArray *c7_rhs44 = NULL;
  const mxArray *c7_lhs44 = NULL;
  const mxArray *c7_rhs45 = NULL;
  const mxArray *c7_lhs45 = NULL;
  const mxArray *c7_rhs46 = NULL;
  const mxArray *c7_lhs46 = NULL;
  const mxArray *c7_rhs47 = NULL;
  const mxArray *c7_lhs47 = NULL;
  const mxArray *c7_rhs48 = NULL;
  const mxArray *c7_lhs48 = NULL;
  const mxArray *c7_rhs49 = NULL;
  const mxArray *c7_lhs49 = NULL;
  const mxArray *c7_rhs50 = NULL;
  const mxArray *c7_lhs50 = NULL;
  const mxArray *c7_rhs51 = NULL;
  const mxArray *c7_lhs51 = NULL;
  const mxArray *c7_rhs52 = NULL;
  const mxArray *c7_lhs52 = NULL;
  const mxArray *c7_rhs53 = NULL;
  const mxArray *c7_lhs53 = NULL;
  const mxArray *c7_rhs54 = NULL;
  const mxArray *c7_lhs54 = NULL;
  const mxArray *c7_rhs55 = NULL;
  const mxArray *c7_lhs55 = NULL;
  const mxArray *c7_rhs56 = NULL;
  const mxArray *c7_lhs56 = NULL;
  const mxArray *c7_rhs57 = NULL;
  const mxArray *c7_lhs57 = NULL;
  const mxArray *c7_rhs58 = NULL;
  const mxArray *c7_lhs58 = NULL;
  const mxArray *c7_rhs59 = NULL;
  const mxArray *c7_lhs59 = NULL;
  const mxArray *c7_rhs60 = NULL;
  const mxArray *c7_lhs60 = NULL;
  const mxArray *c7_rhs61 = NULL;
  const mxArray *c7_lhs61 = NULL;
  const mxArray *c7_rhs62 = NULL;
  const mxArray *c7_lhs62 = NULL;
  const mxArray *c7_rhs63 = NULL;
  const mxArray *c7_lhs63 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("power"), "name", "name", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c7_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c7_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c7_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c7_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c7_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c7_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("floor"), "name", "name", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c7_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c7_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840326U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c7_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c7_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sum"), "name", "name", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c7_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c7_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isequal"), "name", "name", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840358U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c7_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840386U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c7_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c7_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c7_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c7_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c7_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c7_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c7_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c7_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sqrt"), "name", "name", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c7_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_error"), "name", "name",
                  22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c7_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c7_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("rdivide"), "name", "name", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c7_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c7_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c7_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c7_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c7_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("diag"), "name", "name", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c7_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("ismatrix"), "name", "name", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c7_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c7_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c7_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c7_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1383898894U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c7_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c7_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c7_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c7_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c7_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c7_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c7_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c7_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c7_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c7_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c7_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c7_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("mpower"), "name", "name", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731878U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c7_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c7_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("ismatrix"), "name", "name", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c7_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840326U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c7_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c7_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("inv"), "name", "name", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1305339600U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c7_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c7_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c7_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c7_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c7_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c7_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c7_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c7_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("norm"), "name", "name", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731868U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c7_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c7_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c7_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isnan"), "name", "name", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c7_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c7_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c7_rhs0);
  sf_mex_destroy(&c7_lhs0);
  sf_mex_destroy(&c7_rhs1);
  sf_mex_destroy(&c7_lhs1);
  sf_mex_destroy(&c7_rhs2);
  sf_mex_destroy(&c7_lhs2);
  sf_mex_destroy(&c7_rhs3);
  sf_mex_destroy(&c7_lhs3);
  sf_mex_destroy(&c7_rhs4);
  sf_mex_destroy(&c7_lhs4);
  sf_mex_destroy(&c7_rhs5);
  sf_mex_destroy(&c7_lhs5);
  sf_mex_destroy(&c7_rhs6);
  sf_mex_destroy(&c7_lhs6);
  sf_mex_destroy(&c7_rhs7);
  sf_mex_destroy(&c7_lhs7);
  sf_mex_destroy(&c7_rhs8);
  sf_mex_destroy(&c7_lhs8);
  sf_mex_destroy(&c7_rhs9);
  sf_mex_destroy(&c7_lhs9);
  sf_mex_destroy(&c7_rhs10);
  sf_mex_destroy(&c7_lhs10);
  sf_mex_destroy(&c7_rhs11);
  sf_mex_destroy(&c7_lhs11);
  sf_mex_destroy(&c7_rhs12);
  sf_mex_destroy(&c7_lhs12);
  sf_mex_destroy(&c7_rhs13);
  sf_mex_destroy(&c7_lhs13);
  sf_mex_destroy(&c7_rhs14);
  sf_mex_destroy(&c7_lhs14);
  sf_mex_destroy(&c7_rhs15);
  sf_mex_destroy(&c7_lhs15);
  sf_mex_destroy(&c7_rhs16);
  sf_mex_destroy(&c7_lhs16);
  sf_mex_destroy(&c7_rhs17);
  sf_mex_destroy(&c7_lhs17);
  sf_mex_destroy(&c7_rhs18);
  sf_mex_destroy(&c7_lhs18);
  sf_mex_destroy(&c7_rhs19);
  sf_mex_destroy(&c7_lhs19);
  sf_mex_destroy(&c7_rhs20);
  sf_mex_destroy(&c7_lhs20);
  sf_mex_destroy(&c7_rhs21);
  sf_mex_destroy(&c7_lhs21);
  sf_mex_destroy(&c7_rhs22);
  sf_mex_destroy(&c7_lhs22);
  sf_mex_destroy(&c7_rhs23);
  sf_mex_destroy(&c7_lhs23);
  sf_mex_destroy(&c7_rhs24);
  sf_mex_destroy(&c7_lhs24);
  sf_mex_destroy(&c7_rhs25);
  sf_mex_destroy(&c7_lhs25);
  sf_mex_destroy(&c7_rhs26);
  sf_mex_destroy(&c7_lhs26);
  sf_mex_destroy(&c7_rhs27);
  sf_mex_destroy(&c7_lhs27);
  sf_mex_destroy(&c7_rhs28);
  sf_mex_destroy(&c7_lhs28);
  sf_mex_destroy(&c7_rhs29);
  sf_mex_destroy(&c7_lhs29);
  sf_mex_destroy(&c7_rhs30);
  sf_mex_destroy(&c7_lhs30);
  sf_mex_destroy(&c7_rhs31);
  sf_mex_destroy(&c7_lhs31);
  sf_mex_destroy(&c7_rhs32);
  sf_mex_destroy(&c7_lhs32);
  sf_mex_destroy(&c7_rhs33);
  sf_mex_destroy(&c7_lhs33);
  sf_mex_destroy(&c7_rhs34);
  sf_mex_destroy(&c7_lhs34);
  sf_mex_destroy(&c7_rhs35);
  sf_mex_destroy(&c7_lhs35);
  sf_mex_destroy(&c7_rhs36);
  sf_mex_destroy(&c7_lhs36);
  sf_mex_destroy(&c7_rhs37);
  sf_mex_destroy(&c7_lhs37);
  sf_mex_destroy(&c7_rhs38);
  sf_mex_destroy(&c7_lhs38);
  sf_mex_destroy(&c7_rhs39);
  sf_mex_destroy(&c7_lhs39);
  sf_mex_destroy(&c7_rhs40);
  sf_mex_destroy(&c7_lhs40);
  sf_mex_destroy(&c7_rhs41);
  sf_mex_destroy(&c7_lhs41);
  sf_mex_destroy(&c7_rhs42);
  sf_mex_destroy(&c7_lhs42);
  sf_mex_destroy(&c7_rhs43);
  sf_mex_destroy(&c7_lhs43);
  sf_mex_destroy(&c7_rhs44);
  sf_mex_destroy(&c7_lhs44);
  sf_mex_destroy(&c7_rhs45);
  sf_mex_destroy(&c7_lhs45);
  sf_mex_destroy(&c7_rhs46);
  sf_mex_destroy(&c7_lhs46);
  sf_mex_destroy(&c7_rhs47);
  sf_mex_destroy(&c7_lhs47);
  sf_mex_destroy(&c7_rhs48);
  sf_mex_destroy(&c7_lhs48);
  sf_mex_destroy(&c7_rhs49);
  sf_mex_destroy(&c7_lhs49);
  sf_mex_destroy(&c7_rhs50);
  sf_mex_destroy(&c7_lhs50);
  sf_mex_destroy(&c7_rhs51);
  sf_mex_destroy(&c7_lhs51);
  sf_mex_destroy(&c7_rhs52);
  sf_mex_destroy(&c7_lhs52);
  sf_mex_destroy(&c7_rhs53);
  sf_mex_destroy(&c7_lhs53);
  sf_mex_destroy(&c7_rhs54);
  sf_mex_destroy(&c7_lhs54);
  sf_mex_destroy(&c7_rhs55);
  sf_mex_destroy(&c7_lhs55);
  sf_mex_destroy(&c7_rhs56);
  sf_mex_destroy(&c7_lhs56);
  sf_mex_destroy(&c7_rhs57);
  sf_mex_destroy(&c7_lhs57);
  sf_mex_destroy(&c7_rhs58);
  sf_mex_destroy(&c7_lhs58);
  sf_mex_destroy(&c7_rhs59);
  sf_mex_destroy(&c7_lhs59);
  sf_mex_destroy(&c7_rhs60);
  sf_mex_destroy(&c7_lhs60);
  sf_mex_destroy(&c7_rhs61);
  sf_mex_destroy(&c7_lhs61);
  sf_mex_destroy(&c7_rhs62);
  sf_mex_destroy(&c7_lhs62);
  sf_mex_destroy(&c7_rhs63);
  sf_mex_destroy(&c7_lhs63);
}

static const mxArray *c7_emlrt_marshallOut(const char * c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c7_u)), false);
  return c7_y;
}

static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 7, 0U, 0U, 0U, 0), false);
  return c7_y;
}

static void c7_b_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs64 = NULL;
  const mxArray *c7_lhs64 = NULL;
  const mxArray *c7_rhs65 = NULL;
  const mxArray *c7_lhs65 = NULL;
  const mxArray *c7_rhs66 = NULL;
  const mxArray *c7_lhs66 = NULL;
  const mxArray *c7_rhs67 = NULL;
  const mxArray *c7_lhs67 = NULL;
  const mxArray *c7_rhs68 = NULL;
  const mxArray *c7_lhs68 = NULL;
  const mxArray *c7_rhs69 = NULL;
  const mxArray *c7_lhs69 = NULL;
  const mxArray *c7_rhs70 = NULL;
  const mxArray *c7_lhs70 = NULL;
  const mxArray *c7_rhs71 = NULL;
  const mxArray *c7_lhs71 = NULL;
  const mxArray *c7_rhs72 = NULL;
  const mxArray *c7_lhs72 = NULL;
  const mxArray *c7_rhs73 = NULL;
  const mxArray *c7_lhs73 = NULL;
  const mxArray *c7_rhs74 = NULL;
  const mxArray *c7_lhs74 = NULL;
  const mxArray *c7_rhs75 = NULL;
  const mxArray *c7_lhs75 = NULL;
  const mxArray *c7_rhs76 = NULL;
  const mxArray *c7_lhs76 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840376U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c7_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c7_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_warning"), "name", "name",
                  66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840402U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c7_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isnan"), "name", "name", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c7_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eps"), "name", "name", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c7_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c7_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_eps"), "name", "name", 70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c7_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c7_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_flt2str"), "name", "name",
                  72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "resolved",
                  "resolved", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1360303950U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c7_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "context",
                  "context", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "name", "name", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1319751568U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c7_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("mrdivide"), "name", "name", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1388481696U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c7_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c7_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("rdivide"), "name", "name", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c7_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs76), "lhs", "lhs",
                  76);
  sf_mex_destroy(&c7_rhs64);
  sf_mex_destroy(&c7_lhs64);
  sf_mex_destroy(&c7_rhs65);
  sf_mex_destroy(&c7_lhs65);
  sf_mex_destroy(&c7_rhs66);
  sf_mex_destroy(&c7_lhs66);
  sf_mex_destroy(&c7_rhs67);
  sf_mex_destroy(&c7_lhs67);
  sf_mex_destroy(&c7_rhs68);
  sf_mex_destroy(&c7_lhs68);
  sf_mex_destroy(&c7_rhs69);
  sf_mex_destroy(&c7_lhs69);
  sf_mex_destroy(&c7_rhs70);
  sf_mex_destroy(&c7_lhs70);
  sf_mex_destroy(&c7_rhs71);
  sf_mex_destroy(&c7_lhs71);
  sf_mex_destroy(&c7_rhs72);
  sf_mex_destroy(&c7_lhs72);
  sf_mex_destroy(&c7_rhs73);
  sf_mex_destroy(&c7_lhs73);
  sf_mex_destroy(&c7_rhs74);
  sf_mex_destroy(&c7_lhs74);
  sf_mex_destroy(&c7_rhs75);
  sf_mex_destroy(&c7_lhs75);
  sf_mex_destroy(&c7_rhs76);
  sf_mex_destroy(&c7_lhs76);
}

static void c7_quatinv(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_qin[4], real_T c7_qinv[4])
{
  uint32_T c7_debug_family_var_map[5];
  real_T c7_q_conj[4];
  real_T c7_nargin = 1.0;
  real_T c7_nargout = 1.0;
  int32_T c7_i126;
  int32_T c7_i127;
  real_T c7_a[4];
  int32_T c7_k;
  real_T c7_b_k;
  real_T c7_ak;
  real_T c7_b_a;
  real_T c7_y;
  real_T c7_b_y[4];
  real_T c7_c_y;
  int32_T c7_c_k;
  int32_T c7_d_k;
  real_T c7_x;
  real_T c7_b_x;
  int32_T c7_i128;
  real_T c7_d_y;
  real_T c7_e_y;
  real_T c7_f_y;
  int32_T c7_i129;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c7_b_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_q_conj, 0U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 2U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_qin, 3U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_qinv, 4U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 96);
  c7_q_conj[0] = c7_qin[0];
  for (c7_i126 = 0; c7_i126 < 3; c7_i126++) {
    c7_q_conj[c7_i126 + 1] = -c7_qin[c7_i126 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 97);
  for (c7_i127 = 0; c7_i127 < 4; c7_i127++) {
    c7_a[c7_i127] = c7_qin[c7_i127];
  }

  for (c7_k = 0; c7_k < 4; c7_k++) {
    c7_b_k = 1.0 + (real_T)c7_k;
    c7_ak = c7_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c7_b_k), 1, 4, 1, 0) - 1];
    c7_b_a = c7_ak;
    c7_y = c7_b_a * c7_b_a;
    c7_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c7_b_k), 1, 4, 1, 0) - 1] = c7_y;
  }

  c7_c_y = c7_b_y[0];
  for (c7_c_k = 2; c7_c_k < 5; c7_c_k++) {
    c7_d_k = c7_c_k;
    c7_c_y += c7_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c7_d_k), 1, 4, 1, 0) - 1];
  }

  c7_x = c7_c_y;
  c7_b_x = c7_x;
  if (c7_b_x < 0.0) {
    c7_eml_error(chartInstance);
  }

  c7_b_x = muDoubleScalarSqrt(c7_b_x);
  for (c7_i128 = 0; c7_i128 < 4; c7_i128++) {
    c7_a[c7_i128] = c7_q_conj[c7_i128];
  }

  c7_d_y = c7_b_x;
  c7_e_y = c7_d_y;
  c7_f_y = c7_e_y;
  for (c7_i129 = 0; c7_i129 < 4; c7_i129++) {
    c7_qinv[c7_i129] = c7_a[c7_i129] / c7_f_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -97);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c7_eml_error(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c7_i130;
  static char_T c7_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c7_u[30];
  const mxArray *c7_y = NULL;
  int32_T c7_i131;
  static char_T c7_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c7_b_u[4];
  const mxArray *c7_b_y = NULL;
  (void)chartInstance;
  for (c7_i130 = 0; c7_i130 < 30; c7_i130++) {
    c7_u[c7_i130] = c7_cv0[c7_i130];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c7_i131 = 0; c7_i131 < 4; c7_i131++) {
    c7_b_u[c7_i131] = c7_cv1[c7_i131];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c7_y, 14, c7_b_y));
}

static void c7_quatmultiply(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_q[4], real_T c7_r[4], real_T c7_qres[4])
{
  uint32_T c7_debug_family_var_map[7];
  real_T c7_vec[3];
  real_T c7_scalar;
  real_T c7_b_q[4];
  real_T c7_b_r[4];
  real_T c7_nargin = 2.0;
  real_T c7_nargout = 1.0;
  int32_T c7_i132;
  int32_T c7_i133;
  real_T c7_c_q[3];
  real_T c7_c_r[3];
  real_T c7_d_q[3];
  int32_T c7_i134;
  real_T c7_b_scalar[4];
  int32_T c7_i135;
  int32_T c7_i136;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 9U, c7_c_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_vec, 0U, c7_h_sf_marshallOut,
    c7_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_scalar, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_q, MAX_uint32_T, c7_g_sf_marshallOut,
    c7_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_r, MAX_uint32_T, c7_g_sf_marshallOut,
    c7_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 4U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 5U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_q, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_r, 3U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_qres, 6U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 76);
  for (c7_i132 = 0; c7_i132 < 4; c7_i132++) {
    c7_b_q[c7_i132] = c7_q[c7_i132];
  }

  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 77);
  for (c7_i133 = 0; c7_i133 < 4; c7_i133++) {
    c7_b_r[c7_i133] = c7_r[c7_i133];
  }

  _SFD_SYMBOL_SWITCH(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 80);
  c7_c_q[0] = c7_b_q[0] * c7_b_r[1];
  c7_c_q[1] = c7_b_q[0] * c7_b_r[2];
  c7_c_q[2] = c7_b_q[0] * c7_b_r[3];
  c7_c_r[0] = c7_b_r[0] * c7_b_q[1];
  c7_c_r[1] = c7_b_r[0] * c7_b_q[2];
  c7_c_r[2] = c7_b_r[0] * c7_b_q[3];
  c7_d_q[0] = c7_b_q[2] * c7_b_r[3] - c7_b_q[3] * c7_b_r[2];
  c7_d_q[1] = c7_b_q[3] * c7_b_r[1] - c7_b_q[1] * c7_b_r[3];
  c7_d_q[2] = c7_b_q[1] * c7_b_r[2] - c7_b_q[2] * c7_b_r[1];
  for (c7_i134 = 0; c7_i134 < 3; c7_i134++) {
    c7_vec[c7_i134] = (c7_c_q[c7_i134] + c7_c_r[c7_i134]) + c7_d_q[c7_i134];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 88);
  c7_scalar = ((c7_b_q[0] * c7_b_r[0] - c7_b_q[1] * c7_b_r[1]) - c7_b_q[2] *
               c7_b_r[2]) - c7_b_q[3] * c7_b_r[3];
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 91);
  c7_b_scalar[0] = c7_scalar;
  for (c7_i135 = 0; c7_i135 < 3; c7_i135++) {
    c7_b_scalar[c7_i135 + 1] = c7_vec[c7_i135];
  }

  for (c7_i136 = 0; c7_i136 < 4; c7_i136++) {
    c7_qres[c7_i136] = c7_b_scalar[c7_i136];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -91);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c7_Kinematics(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_x[7], real_T c7_I[3], real_T c7_Torque_c[3], real_T c7_results[7])
{
  uint32_T c7_debug_family_var_map[10];
  real_T c7_q[4];
  real_T c7_w[3];
  real_T c7_q_dot[4];
  real_T c7_w_dot[3];
  real_T c7_b_I[9];
  real_T c7_nargin = 3.0;
  real_T c7_nargout = 1.0;
  int32_T c7_i137;
  int32_T c7_i138;
  int32_T c7_i139;
  real_T c7_c_I[3];
  real_T c7_dv34[9];
  int32_T c7_i140;
  int32_T c7_i141;
  real_T c7_b_w[3];
  real_T c7_a[9];
  real_T c7_dv35[16];
  int32_T c7_i142;
  int32_T c7_i143;
  int32_T c7_i144;
  int32_T c7_i145;
  int32_T c7_i146;
  int32_T c7_i147;
  int32_T c7_i148;
  int32_T c7_i149;
  int32_T c7_i150;
  int32_T c7_i151;
  real_T c7_b_a[16];
  int32_T c7_i152;
  real_T c7_b[4];
  int32_T c7_i153;
  int32_T c7_i154;
  int32_T c7_i155;
  real_T c7_C[4];
  int32_T c7_i156;
  int32_T c7_i157;
  int32_T c7_i158;
  int32_T c7_i159;
  int32_T c7_i160;
  int32_T c7_i161;
  int32_T c7_i162;
  int32_T c7_i163;
  real_T c7_b_b[3];
  int32_T c7_i164;
  real_T c7_y[3];
  int32_T c7_i165;
  int32_T c7_i166;
  int32_T c7_i167;
  real_T c7_c_w[3];
  int32_T c7_i168;
  int32_T c7_i169;
  real_T c7_b_y[3];
  int32_T c7_i170;
  int32_T c7_i171;
  int32_T c7_i172;
  real_T c7_d_I[9];
  int32_T c7_i173;
  int32_T c7_i174;
  int32_T c7_i175;
  int32_T c7_i176;
  int32_T c7_i177;
  int32_T c7_i178;
  int32_T c7_i179;
  int32_T c7_i180;
  int32_T c7_i181;
  int32_T c7_i182;
  int32_T c7_i183;
  int32_T c7_i184;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 11U, c7_e_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_q, 0U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_w, 1U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_q_dot, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_w_dot, 3U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_I, MAX_uint32_T, c7_i_sf_marshallOut,
    c7_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 5U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 6U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x, 7U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_I, 4U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_Torque_c, 8U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_results, 9U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 61);
  for (c7_i137 = 0; c7_i137 < 4; c7_i137++) {
    c7_q[c7_i137] = c7_x[c7_i137];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 62);
  for (c7_i138 = 0; c7_i138 < 3; c7_i138++) {
    c7_w[c7_i138] = c7_x[c7_i138 + 4];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 63);
  for (c7_i139 = 0; c7_i139 < 3; c7_i139++) {
    c7_c_I[c7_i139] = c7_I[c7_i139];
  }

  c7_diag(chartInstance, c7_c_I, c7_dv34);
  for (c7_i140 = 0; c7_i140 < 9; c7_i140++) {
    c7_b_I[c7_i140] = c7_dv34[c7_i140];
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 65);
  for (c7_i141 = 0; c7_i141 < 3; c7_i141++) {
    c7_b_w[c7_i141] = c7_w[c7_i141];
  }

  c7_skew_matrix(chartInstance, c7_b_w, c7_a);
  c7_dv35[0] = 0.0;
  c7_i142 = 0;
  for (c7_i143 = 0; c7_i143 < 3; c7_i143++) {
    c7_dv35[c7_i142 + 4] = -c7_w[c7_i143];
    c7_i142 += 4;
  }

  for (c7_i144 = 0; c7_i144 < 3; c7_i144++) {
    c7_dv35[c7_i144 + 1] = c7_w[c7_i144];
  }

  c7_i145 = 0;
  c7_i146 = 0;
  for (c7_i147 = 0; c7_i147 < 3; c7_i147++) {
    for (c7_i148 = 0; c7_i148 < 3; c7_i148++) {
      c7_dv35[(c7_i148 + c7_i145) + 5] = -c7_a[c7_i148 + c7_i146];
    }

    c7_i145 += 4;
    c7_i146 += 3;
  }

  c7_i149 = 0;
  for (c7_i150 = 0; c7_i150 < 4; c7_i150++) {
    for (c7_i151 = 0; c7_i151 < 4; c7_i151++) {
      c7_b_a[c7_i151 + c7_i149] = 0.5 * c7_dv35[c7_i151 + c7_i149];
    }

    c7_i149 += 4;
  }

  for (c7_i152 = 0; c7_i152 < 4; c7_i152++) {
    c7_b[c7_i152] = c7_q[c7_i152];
  }

  c7_eml_scalar_eg(chartInstance);
  c7_eml_scalar_eg(chartInstance);
  for (c7_i153 = 0; c7_i153 < 4; c7_i153++) {
    c7_q_dot[c7_i153] = 0.0;
  }

  for (c7_i154 = 0; c7_i154 < 4; c7_i154++) {
    c7_q_dot[c7_i154] = 0.0;
  }

  for (c7_i155 = 0; c7_i155 < 4; c7_i155++) {
    c7_C[c7_i155] = c7_q_dot[c7_i155];
  }

  for (c7_i156 = 0; c7_i156 < 4; c7_i156++) {
    c7_q_dot[c7_i156] = c7_C[c7_i156];
  }

  c7_threshold(chartInstance);
  for (c7_i157 = 0; c7_i157 < 4; c7_i157++) {
    c7_C[c7_i157] = c7_q_dot[c7_i157];
  }

  for (c7_i158 = 0; c7_i158 < 4; c7_i158++) {
    c7_q_dot[c7_i158] = c7_C[c7_i158];
  }

  for (c7_i159 = 0; c7_i159 < 4; c7_i159++) {
    c7_q_dot[c7_i159] = 0.0;
    c7_i160 = 0;
    for (c7_i161 = 0; c7_i161 < 4; c7_i161++) {
      c7_q_dot[c7_i159] += c7_b_a[c7_i160 + c7_i159] * c7_b[c7_i161];
      c7_i160 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 66);
  for (c7_i162 = 0; c7_i162 < 9; c7_i162++) {
    c7_a[c7_i162] = c7_b_I[c7_i162];
  }

  for (c7_i163 = 0; c7_i163 < 3; c7_i163++) {
    c7_b_b[c7_i163] = c7_w[c7_i163];
  }

  c7_b_eml_scalar_eg(chartInstance);
  c7_b_eml_scalar_eg(chartInstance);
  c7_threshold(chartInstance);
  for (c7_i164 = 0; c7_i164 < 3; c7_i164++) {
    c7_y[c7_i164] = 0.0;
    c7_i165 = 0;
    for (c7_i166 = 0; c7_i166 < 3; c7_i166++) {
      c7_y[c7_i164] += c7_a[c7_i165 + c7_i164] * c7_b_b[c7_i166];
      c7_i165 += 3;
    }
  }

  for (c7_i167 = 0; c7_i167 < 3; c7_i167++) {
    c7_c_w[c7_i167] = c7_w[c7_i167];
  }

  c7_skew_matrix(chartInstance, c7_c_w, c7_a);
  for (c7_i168 = 0; c7_i168 < 9; c7_i168++) {
    c7_a[c7_i168] = -c7_a[c7_i168];
  }

  c7_b_eml_scalar_eg(chartInstance);
  c7_b_eml_scalar_eg(chartInstance);
  c7_threshold(chartInstance);
  for (c7_i169 = 0; c7_i169 < 3; c7_i169++) {
    c7_b_y[c7_i169] = 0.0;
    c7_i170 = 0;
    for (c7_i171 = 0; c7_i171 < 3; c7_i171++) {
      c7_b_y[c7_i169] += c7_a[c7_i170 + c7_i169] * c7_y[c7_i171];
      c7_i170 += 3;
    }
  }

  for (c7_i172 = 0; c7_i172 < 9; c7_i172++) {
    c7_d_I[c7_i172] = c7_b_I[c7_i172];
  }

  c7_mpower(chartInstance, c7_d_I, c7_a);
  for (c7_i173 = 0; c7_i173 < 3; c7_i173++) {
    c7_b_y[c7_i173] += c7_Torque_c[c7_i173];
  }

  c7_b_eml_scalar_eg(chartInstance);
  c7_b_eml_scalar_eg(chartInstance);
  for (c7_i174 = 0; c7_i174 < 3; c7_i174++) {
    c7_w_dot[c7_i174] = 0.0;
  }

  for (c7_i175 = 0; c7_i175 < 3; c7_i175++) {
    c7_w_dot[c7_i175] = 0.0;
  }

  for (c7_i176 = 0; c7_i176 < 3; c7_i176++) {
    c7_b_b[c7_i176] = c7_w_dot[c7_i176];
  }

  for (c7_i177 = 0; c7_i177 < 3; c7_i177++) {
    c7_w_dot[c7_i177] = c7_b_b[c7_i177];
  }

  c7_threshold(chartInstance);
  for (c7_i178 = 0; c7_i178 < 3; c7_i178++) {
    c7_b_b[c7_i178] = c7_w_dot[c7_i178];
  }

  for (c7_i179 = 0; c7_i179 < 3; c7_i179++) {
    c7_w_dot[c7_i179] = c7_b_b[c7_i179];
  }

  for (c7_i180 = 0; c7_i180 < 3; c7_i180++) {
    c7_w_dot[c7_i180] = 0.0;
    c7_i181 = 0;
    for (c7_i182 = 0; c7_i182 < 3; c7_i182++) {
      c7_w_dot[c7_i180] += c7_a[c7_i181 + c7_i180] * c7_b_y[c7_i182];
      c7_i181 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 72);
  for (c7_i183 = 0; c7_i183 < 4; c7_i183++) {
    c7_results[c7_i183] = c7_q_dot[c7_i183];
  }

  for (c7_i184 = 0; c7_i184 < 3; c7_i184++) {
    c7_results[c7_i184 + 4] = c7_w_dot[c7_i184];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -72);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c7_diag(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_v[3],
                    real_T c7_d[9])
{
  int32_T c7_i185;
  int32_T c7_j;
  int32_T c7_b_j;
  (void)chartInstance;
  for (c7_i185 = 0; c7_i185 < 9; c7_i185++) {
    c7_d[c7_i185] = 0.0;
  }

  for (c7_j = 1; c7_j < 4; c7_j++) {
    c7_b_j = c7_j;
    c7_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c7_b_j), 1, 3, 2, 0) - 1)) -
      1] = c7_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c7_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c7_skew_matrix(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T
  c7_x[3], real_T c7_output[9])
{
  uint32_T c7_debug_family_var_map[4];
  real_T c7_nargin = 1.0;
  real_T c7_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c7_d_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 0U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x, 2U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_output, 3U, c7_i_sf_marshallOut,
    c7_i_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 55);
  c7_output[0] = 0.0;
  c7_output[3] = -c7_x[2];
  c7_output[6] = c7_x[1];
  c7_output[1] = c7_x[2];
  c7_output[4] = 0.0;
  c7_output[7] = -c7_x[0];
  c7_output[2] = -c7_x[1];
  c7_output[5] = c7_x[0];
  c7_output[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -55);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c7_eml_scalar_eg(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_threshold(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_mpower(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_a[9],
                      real_T c7_c[9])
{
  int32_T c7_i186;
  real_T c7_b_a[9];
  int32_T c7_i187;
  real_T c7_c_a[9];
  real_T c7_n1x;
  int32_T c7_i188;
  real_T c7_b_c[9];
  real_T c7_n1xinv;
  real_T c7_rc;
  real_T c7_x;
  boolean_T c7_b;
  real_T c7_b_x;
  int32_T c7_i189;
  static char_T c7_cv2[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c7_u[8];
  const mxArray *c7_y = NULL;
  real_T c7_b_u;
  const mxArray *c7_b_y = NULL;
  real_T c7_c_u;
  const mxArray *c7_c_y = NULL;
  real_T c7_d_u;
  const mxArray *c7_d_y = NULL;
  char_T c7_str[14];
  int32_T c7_i190;
  char_T c7_b_str[14];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  for (c7_i186 = 0; c7_i186 < 9; c7_i186++) {
    c7_b_a[c7_i186] = c7_a[c7_i186];
  }

  c7_inv3x3(chartInstance, c7_b_a, c7_c);
  for (c7_i187 = 0; c7_i187 < 9; c7_i187++) {
    c7_c_a[c7_i187] = c7_a[c7_i187];
  }

  c7_n1x = c7_norm(chartInstance, c7_c_a);
  for (c7_i188 = 0; c7_i188 < 9; c7_i188++) {
    c7_b_c[c7_i188] = c7_c[c7_i188];
  }

  c7_n1xinv = c7_norm(chartInstance, c7_b_c);
  c7_rc = 1.0 / (c7_n1x * c7_n1xinv);
  guard1 = false;
  guard2 = false;
  if (c7_n1x == 0.0) {
    guard2 = true;
  } else if (c7_n1xinv == 0.0) {
    guard2 = true;
  } else if (c7_rc == 0.0) {
    guard1 = true;
  } else {
    c7_x = c7_rc;
    c7_b = muDoubleScalarIsNaN(c7_x);
    guard3 = false;
    if (c7_b) {
      guard3 = true;
    } else {
      if (c7_rc < 2.2204460492503131E-16) {
        guard3 = true;
      }
    }

    if (guard3 == true) {
      c7_b_x = c7_rc;
      for (c7_i189 = 0; c7_i189 < 8; c7_i189++) {
        c7_u[c7_i189] = c7_cv2[c7_i189];
      }

      c7_y = NULL;
      sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    false);
      c7_b_u = 14.0;
      c7_b_y = NULL;
      sf_mex_assign(&c7_b_y, sf_mex_create("y", &c7_b_u, 0, 0U, 0U, 0U, 0),
                    false);
      c7_c_u = 6.0;
      c7_c_y = NULL;
      sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_c_u, 0, 0U, 0U, 0U, 0),
                    false);
      c7_d_u = c7_b_x;
      c7_d_y = NULL;
      sf_mex_assign(&c7_d_y, sf_mex_create("y", &c7_d_u, 0, 0U, 0U, 0U, 0),
                    false);
      c7_n_emlrt_marshallIn(chartInstance, sf_mex_call_debug
                            (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14,
        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "sprintf", 1U, 3U, 14,
                          c7_y, 14, c7_b_y, 14, c7_c_y), 14, c7_d_y), "sprintf",
                            c7_str);
      for (c7_i190 = 0; c7_i190 < 14; c7_i190++) {
        c7_b_str[c7_i190] = c7_str[c7_i190];
      }

      c7_b_eml_warning(chartInstance, c7_b_str);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c7_eml_warning(chartInstance);
  }
}

static void c7_inv3x3(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_x[9],
                      real_T c7_y[9])
{
  int32_T c7_p1;
  int32_T c7_p2;
  int32_T c7_p3;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_absx11;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_absx21;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_absx31;
  real_T c7_t1;
  real_T c7_h_x;
  real_T c7_b_y;
  real_T c7_i_x;
  real_T c7_c_y;
  real_T c7_z;
  real_T c7_j_x;
  real_T c7_d_y;
  real_T c7_k_x;
  real_T c7_e_y;
  real_T c7_b_z;
  real_T c7_l_x;
  real_T c7_m_x;
  real_T c7_f_y;
  real_T c7_n_x;
  real_T c7_o_x;
  real_T c7_g_y;
  int32_T c7_itmp;
  real_T c7_p_x;
  real_T c7_h_y;
  real_T c7_q_x;
  real_T c7_i_y;
  real_T c7_c_z;
  real_T c7_r_x;
  real_T c7_j_y;
  real_T c7_s_x;
  real_T c7_k_y;
  real_T c7_t3;
  real_T c7_t_x;
  real_T c7_l_y;
  real_T c7_u_x;
  real_T c7_m_y;
  real_T c7_t2;
  int32_T c7_a;
  int32_T c7_b_a;
  int32_T c7_c;
  real_T c7_v_x;
  real_T c7_n_y;
  real_T c7_w_x;
  real_T c7_o_y;
  real_T c7_d_z;
  int32_T c7_c_a;
  int32_T c7_d_a;
  int32_T c7_b_c;
  int32_T c7_e_a;
  int32_T c7_f_a;
  int32_T c7_c_c;
  real_T c7_x_x;
  real_T c7_p_y;
  real_T c7_y_x;
  real_T c7_q_y;
  real_T c7_ab_x;
  real_T c7_r_y;
  real_T c7_bb_x;
  real_T c7_s_y;
  int32_T c7_g_a;
  int32_T c7_h_a;
  int32_T c7_d_c;
  real_T c7_cb_x;
  real_T c7_t_y;
  real_T c7_db_x;
  real_T c7_u_y;
  real_T c7_e_z;
  int32_T c7_i_a;
  int32_T c7_j_a;
  int32_T c7_e_c;
  int32_T c7_k_a;
  int32_T c7_l_a;
  int32_T c7_f_c;
  real_T c7_v_y;
  real_T c7_w_y;
  real_T c7_eb_x;
  real_T c7_x_y;
  real_T c7_fb_x;
  real_T c7_y_y;
  int32_T c7_m_a;
  int32_T c7_n_a;
  int32_T c7_g_c;
  real_T c7_gb_x;
  real_T c7_ab_y;
  real_T c7_hb_x;
  real_T c7_bb_y;
  real_T c7_f_z;
  int32_T c7_o_a;
  int32_T c7_p_a;
  int32_T c7_h_c;
  int32_T c7_q_a;
  int32_T c7_r_a;
  int32_T c7_i_c;
  boolean_T guard1 = false;
  (void)chartInstance;
  c7_p1 = 0;
  c7_p2 = 3;
  c7_p3 = 6;
  c7_b_x = c7_x[0];
  c7_c_x = c7_b_x;
  c7_absx11 = muDoubleScalarAbs(c7_c_x);
  c7_d_x = c7_x[1];
  c7_e_x = c7_d_x;
  c7_absx21 = muDoubleScalarAbs(c7_e_x);
  c7_f_x = c7_x[2];
  c7_g_x = c7_f_x;
  c7_absx31 = muDoubleScalarAbs(c7_g_x);
  guard1 = false;
  if (c7_absx21 > c7_absx11) {
    if (c7_absx21 > c7_absx31) {
      c7_p1 = 3;
      c7_p2 = 0;
      c7_t1 = c7_x[0];
      c7_x[0] = c7_x[1];
      c7_x[1] = c7_t1;
      c7_t1 = c7_x[3];
      c7_x[3] = c7_x[4];
      c7_x[4] = c7_t1;
      c7_t1 = c7_x[6];
      c7_x[6] = c7_x[7];
      c7_x[7] = c7_t1;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    if (c7_absx31 > c7_absx11) {
      c7_p1 = 6;
      c7_p3 = 0;
      c7_t1 = c7_x[0];
      c7_x[0] = c7_x[2];
      c7_x[2] = c7_t1;
      c7_t1 = c7_x[3];
      c7_x[3] = c7_x[5];
      c7_x[5] = c7_t1;
      c7_t1 = c7_x[6];
      c7_x[6] = c7_x[8];
      c7_x[8] = c7_t1;
    }
  }

  c7_h_x = c7_x[1];
  c7_b_y = c7_x[0];
  c7_i_x = c7_h_x;
  c7_c_y = c7_b_y;
  c7_z = c7_i_x / c7_c_y;
  c7_x[1] = c7_z;
  c7_j_x = c7_x[2];
  c7_d_y = c7_x[0];
  c7_k_x = c7_j_x;
  c7_e_y = c7_d_y;
  c7_b_z = c7_k_x / c7_e_y;
  c7_x[2] = c7_b_z;
  c7_x[4] -= c7_x[1] * c7_x[3];
  c7_x[5] -= c7_x[2] * c7_x[3];
  c7_x[7] -= c7_x[1] * c7_x[6];
  c7_x[8] -= c7_x[2] * c7_x[6];
  c7_l_x = c7_x[5];
  c7_m_x = c7_l_x;
  c7_f_y = muDoubleScalarAbs(c7_m_x);
  c7_n_x = c7_x[4];
  c7_o_x = c7_n_x;
  c7_g_y = muDoubleScalarAbs(c7_o_x);
  if (c7_f_y > c7_g_y) {
    c7_itmp = c7_p2;
    c7_p2 = c7_p3;
    c7_p3 = c7_itmp;
    c7_t1 = c7_x[1];
    c7_x[1] = c7_x[2];
    c7_x[2] = c7_t1;
    c7_t1 = c7_x[4];
    c7_x[4] = c7_x[5];
    c7_x[5] = c7_t1;
    c7_t1 = c7_x[7];
    c7_x[7] = c7_x[8];
    c7_x[8] = c7_t1;
  }

  c7_p_x = c7_x[5];
  c7_h_y = c7_x[4];
  c7_q_x = c7_p_x;
  c7_i_y = c7_h_y;
  c7_c_z = c7_q_x / c7_i_y;
  c7_x[5] = c7_c_z;
  c7_x[8] -= c7_x[5] * c7_x[7];
  c7_r_x = c7_x[5] * c7_x[1] - c7_x[2];
  c7_j_y = c7_x[8];
  c7_s_x = c7_r_x;
  c7_k_y = c7_j_y;
  c7_t3 = c7_s_x / c7_k_y;
  c7_t_x = -(c7_x[1] + c7_x[7] * c7_t3);
  c7_l_y = c7_x[4];
  c7_u_x = c7_t_x;
  c7_m_y = c7_l_y;
  c7_t2 = c7_u_x / c7_m_y;
  c7_a = c7_p1;
  c7_b_a = c7_a + 1;
  c7_c = c7_b_a;
  c7_v_x = (1.0 - c7_x[3] * c7_t2) - c7_x[6] * c7_t3;
  c7_n_y = c7_x[0];
  c7_w_x = c7_v_x;
  c7_o_y = c7_n_y;
  c7_d_z = c7_w_x / c7_o_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_c), 1, 9, 1, 0) - 1] = c7_d_z;
  c7_c_a = c7_p1;
  c7_d_a = c7_c_a + 2;
  c7_b_c = c7_d_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_b_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_e_a = c7_p1;
  c7_f_a = c7_e_a + 3;
  c7_c_c = c7_f_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_c_c), 1, 9, 1, 0) - 1] = c7_t3;
  c7_x_x = -c7_x[5];
  c7_p_y = c7_x[8];
  c7_y_x = c7_x_x;
  c7_q_y = c7_p_y;
  c7_t3 = c7_y_x / c7_q_y;
  c7_ab_x = 1.0 - c7_x[7] * c7_t3;
  c7_r_y = c7_x[4];
  c7_bb_x = c7_ab_x;
  c7_s_y = c7_r_y;
  c7_t2 = c7_bb_x / c7_s_y;
  c7_g_a = c7_p2;
  c7_h_a = c7_g_a + 1;
  c7_d_c = c7_h_a;
  c7_cb_x = -(c7_x[3] * c7_t2 + c7_x[6] * c7_t3);
  c7_t_y = c7_x[0];
  c7_db_x = c7_cb_x;
  c7_u_y = c7_t_y;
  c7_e_z = c7_db_x / c7_u_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_d_c), 1, 9, 1, 0) - 1] = c7_e_z;
  c7_i_a = c7_p2;
  c7_j_a = c7_i_a + 2;
  c7_e_c = c7_j_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_e_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_k_a = c7_p2;
  c7_l_a = c7_k_a + 3;
  c7_f_c = c7_l_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_f_c), 1, 9, 1, 0) - 1] = c7_t3;
  c7_v_y = c7_x[8];
  c7_w_y = c7_v_y;
  c7_t3 = 1.0 / c7_w_y;
  c7_eb_x = -c7_x[7] * c7_t3;
  c7_x_y = c7_x[4];
  c7_fb_x = c7_eb_x;
  c7_y_y = c7_x_y;
  c7_t2 = c7_fb_x / c7_y_y;
  c7_m_a = c7_p3;
  c7_n_a = c7_m_a + 1;
  c7_g_c = c7_n_a;
  c7_gb_x = -(c7_x[3] * c7_t2 + c7_x[6] * c7_t3);
  c7_ab_y = c7_x[0];
  c7_hb_x = c7_gb_x;
  c7_bb_y = c7_ab_y;
  c7_f_z = c7_hb_x / c7_bb_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_g_c), 1, 9, 1, 0) - 1] = c7_f_z;
  c7_o_a = c7_p3;
  c7_p_a = c7_o_a + 2;
  c7_h_c = c7_p_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_h_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_q_a = c7_p3;
  c7_r_a = c7_q_a + 3;
  c7_i_c = c7_r_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_i_c), 1, 9, 1, 0) - 1] = c7_t3;
}

static real_T c7_norm(SFc7_UKF_10hzInstanceStruct *chartInstance, real_T c7_x[9])
{
  real_T c7_y;
  int32_T c7_j;
  real_T c7_b_j;
  real_T c7_s;
  int32_T c7_i;
  real_T c7_b_i;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_b_y;
  real_T c7_d_x;
  boolean_T c7_b;
  boolean_T exitg1;
  (void)chartInstance;
  c7_y = 0.0;
  c7_j = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c7_j < 3)) {
    c7_b_j = 1.0 + (real_T)c7_j;
    c7_s = 0.0;
    for (c7_i = 0; c7_i < 3; c7_i++) {
      c7_b_i = 1.0 + (real_T)c7_i;
      c7_b_x = c7_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c7_b_i), 1, 3, 1, 0) + 3 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c7_b_j), 1, 3, 2, 0) - 1)) - 1];
      c7_c_x = c7_b_x;
      c7_b_y = muDoubleScalarAbs(c7_c_x);
      c7_s += c7_b_y;
    }

    c7_d_x = c7_s;
    c7_b = muDoubleScalarIsNaN(c7_d_x);
    if (c7_b) {
      c7_y = rtNaN;
      exitg1 = true;
    } else {
      if (c7_s > c7_y) {
        c7_y = c7_s;
      }

      c7_j++;
    }
  }

  return c7_y;
}

static void c7_eml_warning(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c7_i191;
  static char_T c7_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c7_u[27];
  const mxArray *c7_y = NULL;
  (void)chartInstance;
  for (c7_i191 = 0; c7_i191 < 27; c7_i191++) {
    c7_u[c7_i191] = c7_varargin_1[c7_i191];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c7_y));
}

static void c7_b_eml_warning(SFc7_UKF_10hzInstanceStruct *chartInstance, char_T
  c7_varargin_2[14])
{
  int32_T c7_i192;
  static char_T c7_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c7_u[33];
  const mxArray *c7_y = NULL;
  int32_T c7_i193;
  char_T c7_b_u[14];
  const mxArray *c7_b_y = NULL;
  (void)chartInstance;
  for (c7_i192 = 0; c7_i192 < 33; c7_i192++) {
    c7_u[c7_i192] = c7_varargin_1[c7_i192];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  for (c7_i193 = 0; c7_i193 < 14; c7_i193++) {
    c7_b_u[c7_i193] = c7_varargin_2[c7_i193];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c7_y, 14, c7_b_y));
}

static void c7_b_eml_scalar_eg(SFc7_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_n_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_sprintf, const char_T *c7_identifier, char_T c7_y[14])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_sprintf), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_sprintf);
}

static void c7_o_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId, char_T c7_y[14])
{
  char_T c7_cv3[14];
  int32_T c7_i194;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_cv3, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c7_i194 = 0; c7_i194 < 14; c7_i194++) {
    c7_y[c7_i194] = c7_cv3[c7_i194];
  }

  sf_mex_destroy(&c7_u);
}

static const mxArray *c7_j_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static int32_T c7_p_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i195;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i195, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i195;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_q_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_b_is_active_c7_UKF_10hz, const char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_UKF_10hz), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_UKF_10hz);
  return c7_y;
}

static uint8_T c7_r_emlrt_marshallIn(SFc7_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void init_dsm_address_info(SFc7_UKF_10hzInstanceStruct *chartInstance)
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

void sf_c7_UKF_10hz_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1252554727U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2708909537U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1598354459U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1023797696U);
}

mxArray *sf_c7_UKF_10hz_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("B4RAqXnj7v1LZkByIwbGEC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c7_UKF_10hz_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c7_UKF_10hz_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c7_UKF_10hz(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[11],T\"q_i_s\",},{M[1],M[10],T\"w_s\",},{M[4],M[0],T\"q_i_c_km1\",S'l','i','p'{{M1x2[118 127],M[0],}}},{M[4],M[0],T\"w_c_km1\",S'l','i','p'{{M1x2[110 117],M[0],}}},{M[8],M[0],T\"is_active_c7_UKF_10hz\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_UKF_10hz_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_UKF_10hzInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc7_UKF_10hzInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _UKF_10hzMachineNumber_,
           7,
           1,
           1,
           0,
           8,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_UKF_10hzMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_UKF_10hzMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _UKF_10hzMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"I_c");
          _SFD_SET_DATA_PROPS(1,1,1,0,"w_init_s");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Torque_s");
          _SFD_SET_DATA_PROPS(3,1,1,0,"q0_i_s");
          _SFD_SET_DATA_PROPS(4,2,0,1,"w_s");
          _SFD_SET_DATA_PROPS(5,2,0,1,"q_i_s");
          _SFD_SET_DATA_PROPS(6,1,1,0,"q_s_c");
          _SFD_SET_DATA_PROPS(7,1,1,0,"Ts");
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
        _SFD_CV_INIT_EML(0,1,5,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1417);
        _SFD_CV_INIT_EML_FCN(0,1,"skew_matrix",1419,-1,1579);
        _SFD_CV_INIT_EML_FCN(0,2,"Kinematics",1581,-1,2017);
        _SFD_CV_INIT_EML_FCN(0,3,"quatmultiply",2019,-1,2612);
        _SFD_CV_INIT_EML_FCN(0,4,"quatinv",2614,-1,2712);
        _SFD_CV_INIT_EML_IF(0,1,0,297,339,631,1412);

        {
          static int condStart[] = { 301, 321 };

          static int condEnd[] = { 317, 339 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,301,339,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)
            c7_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_c_sf_marshallOut,(MexInFcnForType)
            c7_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_e_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c7_Ts;
          real_T (*c7_I_c)[3];
          real_T (*c7_w_init_s)[3];
          real_T (*c7_Torque_s)[3];
          real_T (*c7_q0_i_s)[4];
          real_T (*c7_w_s)[3];
          real_T (*c7_q_i_s)[4];
          real_T (*c7_q_s_c)[4];
          c7_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c7_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
          c7_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
          c7_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
          c7_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
          c7_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c7_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c7_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c7_I_c);
          _SFD_SET_DATA_VALUE_PTR(1U, *c7_w_init_s);
          _SFD_SET_DATA_VALUE_PTR(2U, *c7_Torque_s);
          _SFD_SET_DATA_VALUE_PTR(3U, *c7_q0_i_s);
          _SFD_SET_DATA_VALUE_PTR(4U, *c7_w_s);
          _SFD_SET_DATA_VALUE_PTR(5U, *c7_q_i_s);
          _SFD_SET_DATA_VALUE_PTR(6U, *c7_q_s_c);
          _SFD_SET_DATA_VALUE_PTR(7U, c7_Ts);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _UKF_10hzMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "RUBn5C8lxKYbHdAHK5PsCG";
}

static void sf_opaque_initialize_c7_UKF_10hz(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
  initialize_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c7_UKF_10hz(void *chartInstanceVar)
{
  enable_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c7_UKF_10hz(void *chartInstanceVar)
{
  disable_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c7_UKF_10hz(void *chartInstanceVar)
{
  sf_gateway_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_UKF_10hz(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_UKF_10hz();/* state var info */
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

extern void sf_internal_set_sim_state_c7_UKF_10hz(SimStruct* S, const mxArray
  *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c7_UKF_10hz();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_UKF_10hz(SimStruct* S)
{
  return sf_internal_get_sim_state_c7_UKF_10hz(S);
}

static void sf_opaque_set_sim_state_c7_UKF_10hz(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c7_UKF_10hz(S, st);
}

static void sf_opaque_terminate_c7_UKF_10hz(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_UKF_10hz_optimization_info();
    }

    finalize_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_UKF_10hz(SimStruct *S)
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
    initialize_params_c7_UKF_10hz((SFc7_UKF_10hzInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_UKF_10hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_UKF_10hz_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,7,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,7);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(317838308U));
  ssSetChecksum1(S,(2889282638U));
  ssSetChecksum2(S,(2227111656U));
  ssSetChecksum3(S,(3900886227U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_UKF_10hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_UKF_10hz(SimStruct *S)
{
  SFc7_UKF_10hzInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc7_UKF_10hzInstanceStruct *)utMalloc(sizeof
    (SFc7_UKF_10hzInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_UKF_10hzInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c7_UKF_10hz;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c7_UKF_10hz;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c7_UKF_10hz;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c7_UKF_10hz;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c7_UKF_10hz;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c7_UKF_10hz;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c7_UKF_10hz;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c7_UKF_10hz;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_UKF_10hz;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_UKF_10hz;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c7_UKF_10hz;
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

void c7_UKF_10hz_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_UKF_10hz(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_UKF_10hz(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_UKF_10hz(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_UKF_10hz_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
