/* Include files */

#include "blascompat32.h"
#include "UKF_1hz_sfun.h"
#include "c1_UKF_1hz.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "UKF_1hz_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[12] = { "W", "k1", "k2", "k3", "k4",
  "nargin", "nargout", "Initial_Attitude_C", "w_cont", "Attitude_I_C", "q", "Ts"
};

/* Function Declarations */
static void initialize_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void initialize_params_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance);
static void enable_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void disable_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance);
static void set_sim_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_st);
static void finalize_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void sf_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void c1_chartstep_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void initSimStructsc1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_Ts, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_c_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_q, const char_T *c1_identifier, real_T c1_y[4]);
static void c1_d_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[4]);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_e_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_Attitude_I_C, const char_T *c1_identifier, real_T c1_y[4]);
static void c1_f_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[4]);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_g_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_h_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[16]);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_eml_scalar_eg(SFc1_UKF_1hzInstanceStruct *chartInstance);
static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_i_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_j_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_UKF_1hz, const char_T *c1_identifier);
static uint8_T c1_k_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_UKF_1hzInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_q_not_empty = FALSE;
  chartInstance->c1_Ts_not_empty = FALSE;
  chartInstance->c1_is_active_c1_UKF_1hz = 0U;
}

static void initialize_params_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance)
{
}

static void enable_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  int32_T c1_i0;
  real_T c1_u[4];
  const mxArray *c1_b_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  int32_T c1_i1;
  real_T c1_c_u[4];
  const mxArray *c1_d_y = NULL;
  uint8_T c1_b_hoistedGlobal;
  uint8_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T (*c1_Attitude_I_C)[4];
  c1_Attitude_I_C = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(4), FALSE);
  for (c1_i0 = 0; c1_i0 < 4; c1_i0++) {
    c1_u[c1_i0] = (*c1_Attitude_I_C)[c1_i0];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_hoistedGlobal = chartInstance->c1_Ts;
  c1_b_u = c1_hoistedGlobal;
  c1_c_y = NULL;
  if (!chartInstance->c1_Ts_not_empty) {
    sf_mex_assign(&c1_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c1_y, 1, c1_c_y);
  for (c1_i1 = 0; c1_i1 < 4; c1_i1++) {
    c1_c_u[c1_i1] = chartInstance->c1_q[c1_i1];
  }

  c1_d_y = NULL;
  if (!chartInstance->c1_q_not_empty) {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", c1_c_u, 0, 0U, 1U, 0U, 1, 4),
                  FALSE);
  }

  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_b_hoistedGlobal = chartInstance->c1_is_active_c1_UKF_1hz;
  c1_d_u = c1_b_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[4];
  int32_T c1_i2;
  real_T c1_dv1[4];
  int32_T c1_i3;
  real_T (*c1_Attitude_I_C)[4];
  c1_Attitude_I_C = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
                        "Attitude_I_C", c1_dv0);
  for (c1_i2 = 0; c1_i2 < 4; c1_i2++) {
    (*c1_Attitude_I_C)[c1_i2] = c1_dv0[c1_i2];
  }

  chartInstance->c1_Ts = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 1)), "Ts");
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 2)), "q",
                        c1_dv1);
  for (c1_i3 = 0; c1_i3 < 4; c1_i3++) {
    chartInstance->c1_q[c1_i3] = c1_dv1[c1_i3];
  }

  chartInstance->c1_is_active_c1_UKF_1hz = c1_j_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 3)), "is_active_c1_UKF_1hz");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_UKF_1hz(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
}

static void sf_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
  int32_T c1_i4;
  int32_T c1_i5;
  int32_T c1_i6;
  real_T (*c1_w_cont)[3];
  real_T (*c1_Attitude_I_C)[4];
  real_T (*c1_Initial_Attitude_C)[4];
  c1_w_cont = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c1_Attitude_I_C = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_Initial_Attitude_C = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S,
    0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i4 = 0; c1_i4 < 4; c1_i4++) {
    _SFD_DATA_RANGE_CHECK((*c1_Initial_Attitude_C)[c1_i4], 0U);
  }

  for (c1_i5 = 0; c1_i5 < 4; c1_i5++) {
    _SFD_DATA_RANGE_CHECK((*c1_Attitude_I_C)[c1_i5], 1U);
  }

  for (c1_i6 = 0; c1_i6 < 3; c1_i6++) {
    _SFD_DATA_RANGE_CHECK((*c1_w_cont)[c1_i6], 2U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_UKF_1hz(chartInstance);
  sf_debug_check_for_state_inconsistency(_UKF_1hzMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
  int32_T c1_i7;
  real_T c1_Initial_Attitude_C[4];
  int32_T c1_i8;
  real_T c1_w_cont[3];
  uint32_T c1_debug_family_var_map[12];
  real_T c1_W[16];
  real_T c1_k1[4];
  real_T c1_k2[4];
  real_T c1_k3[4];
  real_T c1_k4[4];
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 1.0;
  real_T c1_Attitude_I_C[4];
  int32_T c1_i9;
  int32_T c1_i10;
  real_T c1_b[16];
  int32_T c1_i11;
  int32_T c1_i12;
  real_T c1_hoistedGlobal[4];
  int32_T c1_i13;
  int32_T c1_i14;
  int32_T c1_i15;
  int32_T c1_i16;
  real_T c1_C[4];
  int32_T c1_i17;
  int32_T c1_i18;
  int32_T c1_i19;
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  int32_T c1_i23;
  int32_T c1_i24;
  int32_T c1_i25;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_b;
  int32_T c1_i26;
  int32_T c1_i27;
  int32_T c1_i28;
  int32_T c1_i29;
  int32_T c1_i30;
  int32_T c1_i31;
  int32_T c1_i32;
  int32_T c1_i33;
  int32_T c1_i34;
  int32_T c1_i35;
  int32_T c1_i36;
  int32_T c1_i37;
  int32_T c1_i38;
  int32_T c1_i39;
  int32_T c1_i40;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_b;
  int32_T c1_i41;
  int32_T c1_i42;
  int32_T c1_i43;
  int32_T c1_i44;
  int32_T c1_i45;
  int32_T c1_i46;
  int32_T c1_i47;
  int32_T c1_i48;
  int32_T c1_i49;
  int32_T c1_i50;
  int32_T c1_i51;
  int32_T c1_i52;
  int32_T c1_i53;
  real_T c1_d_hoistedGlobal;
  int32_T c1_i54;
  real_T c1_d_b;
  int32_T c1_i55;
  int32_T c1_i56;
  int32_T c1_i57;
  int32_T c1_i58;
  int32_T c1_i59;
  int32_T c1_i60;
  int32_T c1_i61;
  int32_T c1_i62;
  int32_T c1_i63;
  int32_T c1_i64;
  int32_T c1_i65;
  int32_T c1_i66;
  int32_T c1_i67;
  int32_T c1_i68;
  int32_T c1_i69;
  int32_T c1_i70;
  real_T c1_e_b[4];
  int32_T c1_i71;
  real_T c1_e_hoistedGlobal;
  int32_T c1_i72;
  real_T c1_f_b;
  int32_T c1_i73;
  int32_T c1_i74;
  int32_T c1_i75;
  int32_T c1_i76;
  int32_T c1_i77;
  real_T (*c1_b_Attitude_I_C)[4];
  real_T (*c1_b_w_cont)[3];
  real_T (*c1_b_Initial_Attitude_C)[4];
  c1_b_w_cont = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_Attitude_I_C = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_Initial_Attitude_C = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S,
    0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i7 = 0; c1_i7 < 4; c1_i7++) {
    c1_Initial_Attitude_C[c1_i7] = (*c1_b_Initial_Attitude_C)[c1_i7];
  }

  for (c1_i8 = 0; c1_i8 < 3; c1_i8++) {
    c1_w_cont[c1_i8] = (*c1_b_w_cont)[c1_i8];
  }

  sf_debug_symbol_scope_push_eml(0U, 12U, 12U, c1_debug_family_names,
    c1_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c1_W, 0U, c1_f_sf_marshallOut,
    c1_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c1_k1, 1U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c1_k2, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c1_k3, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c1_k4, 4U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargin, 5U, c1_e_sf_marshallOut,
    c1_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargout, 6U, c1_e_sf_marshallOut,
    c1_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c1_Initial_Attitude_C, 7U, c1_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c1_w_cont, 8U, c1_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c1_Attitude_I_C, 9U,
    c1_c_sf_marshallOut, c1_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c1_q, 10U,
    c1_b_sf_marshallOut, c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c1_Ts, 11U,
    c1_sf_marshallOut, c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c1_q_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
    for (c1_i9 = 0; c1_i9 < 4; c1_i9++) {
      chartInstance->c1_q[c1_i9] = c1_Initial_Attitude_C[c1_i9];
    }

    chartInstance->c1_q_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
    for (c1_i10 = 0; c1_i10 < 4; c1_i10++) {
      c1_Attitude_I_C[c1_i10] = chartInstance->c1_q[c1_i10];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
    chartInstance->c1_Ts = 1.0;
    chartInstance->c1_Ts_not_empty = TRUE;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
    c1_b[0] = 0.0;
    c1_b[4] = -c1_w_cont[0];
    c1_b[8] = -c1_w_cont[1];
    c1_b[12] = -c1_w_cont[2];
    c1_b[1] = c1_w_cont[0];
    c1_b[5] = 0.0;
    c1_b[9] = c1_w_cont[2];
    c1_b[13] = -c1_w_cont[1];
    c1_b[2] = c1_w_cont[1];
    c1_b[6] = -c1_w_cont[2];
    c1_b[10] = 0.0;
    c1_b[14] = c1_w_cont[0];
    c1_b[3] = c1_w_cont[2];
    c1_b[7] = c1_w_cont[1];
    c1_b[11] = -c1_w_cont[0];
    c1_b[15] = 0.0;
    for (c1_i11 = 0; c1_i11 < 16; c1_i11++) {
      c1_W[c1_i11] = 0.5 * c1_b[c1_i11];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
    for (c1_i12 = 0; c1_i12 < 4; c1_i12++) {
      c1_hoistedGlobal[c1_i12] = chartInstance->c1_q[c1_i12];
    }

    for (c1_i13 = 0; c1_i13 < 16; c1_i13++) {
      c1_b[c1_i13] = c1_W[c1_i13];
    }

    c1_eml_scalar_eg(chartInstance);
    c1_eml_scalar_eg(chartInstance);
    for (c1_i14 = 0; c1_i14 < 4; c1_i14++) {
      c1_k1[c1_i14] = 0.0;
    }

    for (c1_i15 = 0; c1_i15 < 4; c1_i15++) {
      c1_k1[c1_i15] = 0.0;
    }

    for (c1_i16 = 0; c1_i16 < 4; c1_i16++) {
      c1_C[c1_i16] = c1_k1[c1_i16];
    }

    for (c1_i17 = 0; c1_i17 < 4; c1_i17++) {
      c1_k1[c1_i17] = c1_C[c1_i17];
    }

    for (c1_i18 = 0; c1_i18 < 4; c1_i18++) {
      c1_C[c1_i18] = c1_k1[c1_i18];
    }

    for (c1_i19 = 0; c1_i19 < 4; c1_i19++) {
      c1_k1[c1_i19] = c1_C[c1_i19];
    }

    for (c1_i20 = 0; c1_i20 < 4; c1_i20++) {
      c1_k1[c1_i20] = 0.0;
      c1_i21 = 0;
      for (c1_i22 = 0; c1_i22 < 4; c1_i22++) {
        c1_k1[c1_i20] += c1_b[c1_i21 + c1_i20] * c1_hoistedGlobal[c1_i22];
        c1_i21 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
    for (c1_i23 = 0; c1_i23 < 4; c1_i23++) {
      c1_hoistedGlobal[c1_i23] = chartInstance->c1_q[c1_i23];
    }

    for (c1_i24 = 0; c1_i24 < 4; c1_i24++) {
      c1_C[c1_i24] = c1_k1[c1_i24];
    }

    for (c1_i25 = 0; c1_i25 < 4; c1_i25++) {
      c1_C[c1_i25] *= 0.5;
    }

    c1_b_hoistedGlobal = chartInstance->c1_Ts;
    c1_b_b = c1_b_hoistedGlobal;
    for (c1_i26 = 0; c1_i26 < 4; c1_i26++) {
      c1_C[c1_i26] *= c1_b_b;
    }

    for (c1_i27 = 0; c1_i27 < 16; c1_i27++) {
      c1_b[c1_i27] = c1_W[c1_i27];
    }

    for (c1_i28 = 0; c1_i28 < 4; c1_i28++) {
      c1_hoistedGlobal[c1_i28] += c1_C[c1_i28];
    }

    c1_eml_scalar_eg(chartInstance);
    c1_eml_scalar_eg(chartInstance);
    for (c1_i29 = 0; c1_i29 < 4; c1_i29++) {
      c1_k2[c1_i29] = 0.0;
    }

    for (c1_i30 = 0; c1_i30 < 4; c1_i30++) {
      c1_k2[c1_i30] = 0.0;
    }

    for (c1_i31 = 0; c1_i31 < 4; c1_i31++) {
      c1_C[c1_i31] = c1_k2[c1_i31];
    }

    for (c1_i32 = 0; c1_i32 < 4; c1_i32++) {
      c1_k2[c1_i32] = c1_C[c1_i32];
    }

    for (c1_i33 = 0; c1_i33 < 4; c1_i33++) {
      c1_C[c1_i33] = c1_k2[c1_i33];
    }

    for (c1_i34 = 0; c1_i34 < 4; c1_i34++) {
      c1_k2[c1_i34] = c1_C[c1_i34];
    }

    for (c1_i35 = 0; c1_i35 < 4; c1_i35++) {
      c1_k2[c1_i35] = 0.0;
      c1_i36 = 0;
      for (c1_i37 = 0; c1_i37 < 4; c1_i37++) {
        c1_k2[c1_i35] += c1_b[c1_i36 + c1_i35] * c1_hoistedGlobal[c1_i37];
        c1_i36 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
    for (c1_i38 = 0; c1_i38 < 4; c1_i38++) {
      c1_hoistedGlobal[c1_i38] = chartInstance->c1_q[c1_i38];
    }

    for (c1_i39 = 0; c1_i39 < 4; c1_i39++) {
      c1_C[c1_i39] = c1_k2[c1_i39];
    }

    for (c1_i40 = 0; c1_i40 < 4; c1_i40++) {
      c1_C[c1_i40] *= 0.5;
    }

    c1_c_hoistedGlobal = chartInstance->c1_Ts;
    c1_c_b = c1_c_hoistedGlobal;
    for (c1_i41 = 0; c1_i41 < 4; c1_i41++) {
      c1_C[c1_i41] *= c1_c_b;
    }

    for (c1_i42 = 0; c1_i42 < 16; c1_i42++) {
      c1_b[c1_i42] = c1_W[c1_i42];
    }

    for (c1_i43 = 0; c1_i43 < 4; c1_i43++) {
      c1_hoistedGlobal[c1_i43] += c1_C[c1_i43];
    }

    c1_eml_scalar_eg(chartInstance);
    c1_eml_scalar_eg(chartInstance);
    for (c1_i44 = 0; c1_i44 < 4; c1_i44++) {
      c1_k3[c1_i44] = 0.0;
    }

    for (c1_i45 = 0; c1_i45 < 4; c1_i45++) {
      c1_k3[c1_i45] = 0.0;
    }

    for (c1_i46 = 0; c1_i46 < 4; c1_i46++) {
      c1_C[c1_i46] = c1_k3[c1_i46];
    }

    for (c1_i47 = 0; c1_i47 < 4; c1_i47++) {
      c1_k3[c1_i47] = c1_C[c1_i47];
    }

    for (c1_i48 = 0; c1_i48 < 4; c1_i48++) {
      c1_C[c1_i48] = c1_k3[c1_i48];
    }

    for (c1_i49 = 0; c1_i49 < 4; c1_i49++) {
      c1_k3[c1_i49] = c1_C[c1_i49];
    }

    for (c1_i50 = 0; c1_i50 < 4; c1_i50++) {
      c1_k3[c1_i50] = 0.0;
      c1_i51 = 0;
      for (c1_i52 = 0; c1_i52 < 4; c1_i52++) {
        c1_k3[c1_i50] += c1_b[c1_i51 + c1_i50] * c1_hoistedGlobal[c1_i52];
        c1_i51 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
    for (c1_i53 = 0; c1_i53 < 4; c1_i53++) {
      c1_hoistedGlobal[c1_i53] = chartInstance->c1_q[c1_i53];
    }

    c1_d_hoistedGlobal = chartInstance->c1_Ts;
    for (c1_i54 = 0; c1_i54 < 4; c1_i54++) {
      c1_C[c1_i54] = c1_k3[c1_i54];
    }

    c1_d_b = c1_d_hoistedGlobal;
    for (c1_i55 = 0; c1_i55 < 4; c1_i55++) {
      c1_C[c1_i55] *= c1_d_b;
    }

    for (c1_i56 = 0; c1_i56 < 16; c1_i56++) {
      c1_b[c1_i56] = c1_W[c1_i56];
    }

    for (c1_i57 = 0; c1_i57 < 4; c1_i57++) {
      c1_hoistedGlobal[c1_i57] += c1_C[c1_i57];
    }

    c1_eml_scalar_eg(chartInstance);
    c1_eml_scalar_eg(chartInstance);
    for (c1_i58 = 0; c1_i58 < 4; c1_i58++) {
      c1_k4[c1_i58] = 0.0;
    }

    for (c1_i59 = 0; c1_i59 < 4; c1_i59++) {
      c1_k4[c1_i59] = 0.0;
    }

    for (c1_i60 = 0; c1_i60 < 4; c1_i60++) {
      c1_C[c1_i60] = c1_k4[c1_i60];
    }

    for (c1_i61 = 0; c1_i61 < 4; c1_i61++) {
      c1_k4[c1_i61] = c1_C[c1_i61];
    }

    for (c1_i62 = 0; c1_i62 < 4; c1_i62++) {
      c1_C[c1_i62] = c1_k4[c1_i62];
    }

    for (c1_i63 = 0; c1_i63 < 4; c1_i63++) {
      c1_k4[c1_i63] = c1_C[c1_i63];
    }

    for (c1_i64 = 0; c1_i64 < 4; c1_i64++) {
      c1_k4[c1_i64] = 0.0;
      c1_i65 = 0;
      for (c1_i66 = 0; c1_i66 < 4; c1_i66++) {
        c1_k4[c1_i64] += c1_b[c1_i65 + c1_i64] * c1_hoistedGlobal[c1_i66];
        c1_i65 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
    for (c1_i67 = 0; c1_i67 < 4; c1_i67++) {
      c1_hoistedGlobal[c1_i67] = chartInstance->c1_q[c1_i67];
    }

    for (c1_i68 = 0; c1_i68 < 4; c1_i68++) {
      c1_C[c1_i68] = c1_k2[c1_i68];
    }

    for (c1_i69 = 0; c1_i69 < 4; c1_i69++) {
      c1_C[c1_i69] *= 2.0;
    }

    for (c1_i70 = 0; c1_i70 < 4; c1_i70++) {
      c1_e_b[c1_i70] = c1_k3[c1_i70];
    }

    for (c1_i71 = 0; c1_i71 < 4; c1_i71++) {
      c1_e_b[c1_i71] *= 2.0;
    }

    c1_e_hoistedGlobal = chartInstance->c1_Ts;
    for (c1_i72 = 0; c1_i72 < 4; c1_i72++) {
      c1_C[c1_i72] = ((c1_k1[c1_i72] + c1_C[c1_i72]) + c1_e_b[c1_i72]) +
        c1_k4[c1_i72];
    }

    c1_f_b = c1_e_hoistedGlobal;
    for (c1_i73 = 0; c1_i73 < 4; c1_i73++) {
      c1_C[c1_i73] *= c1_f_b;
    }

    for (c1_i74 = 0; c1_i74 < 4; c1_i74++) {
      c1_C[c1_i74] /= 6.0;
    }

    for (c1_i75 = 0; c1_i75 < 4; c1_i75++) {
      c1_Attitude_I_C[c1_i75] = c1_hoistedGlobal[c1_i75] + c1_C[c1_i75];
    }

    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
    for (c1_i76 = 0; c1_i76 < 4; c1_i76++) {
      chartInstance->c1_q[c1_i76] = c1_Attitude_I_C[c1_i76];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -22);
  sf_debug_symbol_scope_pop();
  for (c1_i77 = 0; c1_i77 < 4; c1_i77++) {
    (*c1_b_Attitude_I_C)[c1_i77] = c1_Attitude_I_C[c1_i77];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_UKF_1hz(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  if (!chartInstance->c1_Ts_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_Ts, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Ts), &c1_thisId);
  sf_mex_destroy(&c1_b_Ts);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_Ts_not_empty = FALSE;
  } else {
    chartInstance->c1_Ts_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
    c1_y = c1_d0;
  }

  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_Ts;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_b_Ts = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Ts), &c1_thisId);
  sf_mex_destroy(&c1_b_Ts);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i78;
  real_T c1_b_inData[4];
  int32_T c1_i79;
  real_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i78 = 0; c1_i78 < 4; c1_i78++) {
    c1_b_inData[c1_i78] = (*(real_T (*)[4])c1_inData)[c1_i78];
  }

  for (c1_i79 = 0; c1_i79 < 4; c1_i79++) {
    c1_u[c1_i79] = c1_b_inData[c1_i79];
  }

  c1_y = NULL;
  if (!chartInstance->c1_q_not_empty) {
    sf_mex_assign(&c1_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  }

  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_c_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_q, const char_T *c1_identifier, real_T c1_y[4])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_q);
}

static void c1_d_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[4])
{
  real_T c1_dv2[4];
  int32_T c1_i80;
  if (mxIsEmpty(c1_u)) {
    chartInstance->c1_q_not_empty = FALSE;
  } else {
    chartInstance->c1_q_not_empty = TRUE;
    sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv2, 1, 0, 0U, 1, 0U, 1, 4);
    for (c1_i80 = 0; c1_i80 < 4; c1_i80++) {
      c1_y[c1_i80] = c1_dv2[c1_i80];
    }
  }

  sf_mex_destroy(&c1_u);
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_q;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[4];
  int32_T c1_i81;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_b_q = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_q), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_q);
  for (c1_i81 = 0; c1_i81 < 4; c1_i81++) {
    (*(real_T (*)[4])c1_outData)[c1_i81] = c1_y[c1_i81];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i82;
  real_T c1_b_inData[4];
  int32_T c1_i83;
  real_T c1_u[4];
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i82 = 0; c1_i82 < 4; c1_i82++) {
    c1_b_inData[c1_i82] = (*(real_T (*)[4])c1_inData)[c1_i82];
  }

  for (c1_i83 = 0; c1_i83 < 4; c1_i83++) {
    c1_u[c1_i83] = c1_b_inData[c1_i83];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_e_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_Attitude_I_C, const char_T *c1_identifier, real_T c1_y[4])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_Attitude_I_C), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_Attitude_I_C);
}

static void c1_f_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[4])
{
  real_T c1_dv3[4];
  int32_T c1_i84;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv3, 1, 0, 0U, 1, 0U, 1, 4);
  for (c1_i84 = 0; c1_i84 < 4; c1_i84++) {
    c1_y[c1_i84] = c1_dv3[c1_i84];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_Attitude_I_C;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[4];
  int32_T c1_i85;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_Attitude_I_C = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_Attitude_I_C), &c1_thisId,
                        c1_y);
  sf_mex_destroy(&c1_Attitude_I_C);
  for (c1_i85 = 0; c1_i85 < 4; c1_i85++) {
    (*(real_T (*)[4])c1_outData)[c1_i85] = c1_y[c1_i85];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i86;
  real_T c1_b_inData[3];
  int32_T c1_i87;
  real_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i86 = 0; c1_i86 < 3; c1_i86++) {
    c1_b_inData[c1_i86] = (*(real_T (*)[3])c1_inData)[c1_i86];
  }

  for (c1_i87 = 0; c1_i87 < 3; c1_i87++) {
    c1_u[c1_i87] = c1_b_inData[c1_i87];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_g_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d1;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d1;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i88;
  int32_T c1_i89;
  int32_T c1_i90;
  real_T c1_b_inData[16];
  int32_T c1_i91;
  int32_T c1_i92;
  int32_T c1_i93;
  real_T c1_u[16];
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i88 = 0;
  for (c1_i89 = 0; c1_i89 < 4; c1_i89++) {
    for (c1_i90 = 0; c1_i90 < 4; c1_i90++) {
      c1_b_inData[c1_i90 + c1_i88] = (*(real_T (*)[16])c1_inData)[c1_i90 +
        c1_i88];
    }

    c1_i88 += 4;
  }

  c1_i91 = 0;
  for (c1_i92 = 0; c1_i92 < 4; c1_i92++) {
    for (c1_i93 = 0; c1_i93 < 4; c1_i93++) {
      c1_u[c1_i93 + c1_i91] = c1_b_inData[c1_i93 + c1_i91];
    }

    c1_i91 += 4;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_h_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[16])
{
  real_T c1_dv4[16];
  int32_T c1_i94;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv4, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c1_i94 = 0; c1_i94 < 16; c1_i94++) {
    c1_y[c1_i94] = c1_dv4[c1_i94];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_W;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[16];
  int32_T c1_i95;
  int32_T c1_i96;
  int32_T c1_i97;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_W = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_W), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_W);
  c1_i95 = 0;
  for (c1_i96 = 0; c1_i96 < 4; c1_i96++) {
    for (c1_i97 = 0; c1_i97 < 4; c1_i97++) {
      (*(real_T (*)[16])c1_outData)[c1_i97 + c1_i95] = c1_y[c1_i97 + c1_i95];
    }

    c1_i95 += 4;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_UKF_1hz_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[12];
  c1_ResolvedFunctionInfo (*c1_b_info)[12];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i98;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_b_info = (c1_ResolvedFunctionInfo (*)[12])c1_info;
  (*c1_b_info)[0].context = "";
  (*c1_b_info)[0].name = "mtimes";
  (*c1_b_info)[0].dominantType = "double";
  (*c1_b_info)[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[0].fileTimeLo = 1289541292U;
  (*c1_b_info)[0].fileTimeHi = 0U;
  (*c1_b_info)[0].mFileTimeLo = 0U;
  (*c1_b_info)[0].mFileTimeHi = 0U;
  (*c1_b_info)[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[1].name = "eml_index_class";
  (*c1_b_info)[1].dominantType = "";
  (*c1_b_info)[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c1_b_info)[1].fileTimeLo = 1323192178U;
  (*c1_b_info)[1].fileTimeHi = 0U;
  (*c1_b_info)[1].mFileTimeLo = 0U;
  (*c1_b_info)[1].mFileTimeHi = 0U;
  (*c1_b_info)[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[2].name = "eml_scalar_eg";
  (*c1_b_info)[2].dominantType = "double";
  (*c1_b_info)[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c1_b_info)[2].fileTimeLo = 1286840396U;
  (*c1_b_info)[2].fileTimeHi = 0U;
  (*c1_b_info)[2].mFileTimeLo = 0U;
  (*c1_b_info)[2].mFileTimeHi = 0U;
  (*c1_b_info)[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[3].name = "eml_xgemm";
  (*c1_b_info)[3].dominantType = "char";
  (*c1_b_info)[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c1_b_info)[3].fileTimeLo = 1299098372U;
  (*c1_b_info)[3].fileTimeHi = 0U;
  (*c1_b_info)[3].mFileTimeLo = 0U;
  (*c1_b_info)[3].mFileTimeHi = 0U;
  (*c1_b_info)[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c1_b_info)[4].name = "eml_blas_inline";
  (*c1_b_info)[4].dominantType = "";
  (*c1_b_info)[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  (*c1_b_info)[4].fileTimeLo = 1299098368U;
  (*c1_b_info)[4].fileTimeHi = 0U;
  (*c1_b_info)[4].mFileTimeLo = 0U;
  (*c1_b_info)[4].mFileTimeHi = 0U;
  (*c1_b_info)[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  (*c1_b_info)[5].name = "mtimes";
  (*c1_b_info)[5].dominantType = "double";
  (*c1_b_info)[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[5].fileTimeLo = 1289541292U;
  (*c1_b_info)[5].fileTimeHi = 0U;
  (*c1_b_info)[5].mFileTimeLo = 0U;
  (*c1_b_info)[5].mFileTimeHi = 0U;
  (*c1_b_info)[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c1_b_info)[6].name = "eml_index_class";
  (*c1_b_info)[6].dominantType = "";
  (*c1_b_info)[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c1_b_info)[6].fileTimeLo = 1323192178U;
  (*c1_b_info)[6].fileTimeHi = 0U;
  (*c1_b_info)[6].mFileTimeLo = 0U;
  (*c1_b_info)[6].mFileTimeHi = 0U;
  (*c1_b_info)[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c1_b_info)[7].name = "eml_scalar_eg";
  (*c1_b_info)[7].dominantType = "double";
  (*c1_b_info)[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c1_b_info)[7].fileTimeLo = 1286840396U;
  (*c1_b_info)[7].fileTimeHi = 0U;
  (*c1_b_info)[7].mFileTimeLo = 0U;
  (*c1_b_info)[7].mFileTimeHi = 0U;
  (*c1_b_info)[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c1_b_info)[8].name = "eml_refblas_xgemm";
  (*c1_b_info)[8].dominantType = "char";
  (*c1_b_info)[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  (*c1_b_info)[8].fileTimeLo = 1299098374U;
  (*c1_b_info)[8].fileTimeHi = 0U;
  (*c1_b_info)[8].mFileTimeLo = 0U;
  (*c1_b_info)[8].mFileTimeHi = 0U;
  (*c1_b_info)[9].context = "";
  (*c1_b_info)[9].name = "mrdivide";
  (*c1_b_info)[9].dominantType = "double";
  (*c1_b_info)[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c1_b_info)[9].fileTimeLo = 1342832544U;
  (*c1_b_info)[9].fileTimeHi = 0U;
  (*c1_b_info)[9].mFileTimeLo = 1319751566U;
  (*c1_b_info)[9].mFileTimeHi = 0U;
  (*c1_b_info)[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  (*c1_b_info)[10].name = "rdivide";
  (*c1_b_info)[10].dominantType = "double";
  (*c1_b_info)[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c1_b_info)[10].fileTimeLo = 1286840444U;
  (*c1_b_info)[10].fileTimeHi = 0U;
  (*c1_b_info)[10].mFileTimeLo = 0U;
  (*c1_b_info)[10].mFileTimeHi = 0U;
  (*c1_b_info)[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  (*c1_b_info)[11].name = "eml_div";
  (*c1_b_info)[11].dominantType = "double";
  (*c1_b_info)[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  (*c1_b_info)[11].fileTimeLo = 1313369410U;
  (*c1_b_info)[11].fileTimeHi = 0U;
  (*c1_b_info)[11].mFileTimeLo = 0U;
  (*c1_b_info)[11].mFileTimeHi = 0U;
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 12), FALSE);
  for (c1_i98 = 0; c1_i98 < 12; c1_i98++) {
    c1_r0 = &c1_info[c1_i98];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i98);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i98);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_eml_scalar_eg(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
}

static const mxArray *c1_g_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_i_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i99;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i99, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i99;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_j_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_UKF_1hz, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_UKF_1hz), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_UKF_1hz);
  return c1_y;
}

static uint8_T c1_k_emlrt_marshallIn(SFc1_UKF_1hzInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_UKF_1hzInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c1_UKF_1hz_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1021025563U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2783943448U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2499125294U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1820335195U);
}

mxArray *sf_c1_UKF_1hz_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("qPYzifZjfORlifRzCMxeR");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c1_UKF_1hz(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"Attitude_I_C\",},{M[4],M[0],T\"Ts\",S'l','i','p'{{M1x2[95 97],M[0],}}},{M[4],M[0],T\"q\",S'l','i','p'{{M1x2[93 94],M[0],}}},{M[8],M[0],T\"is_active_c1_UKF_1hz\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_UKF_1hz_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_UKF_1hzInstanceStruct *chartInstance;
    chartInstance = (SFc1_UKF_1hzInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_UKF_1hzMachineNumber_,
          1,
          1,
          1,
          3,
          0,
          0,
          0,
          0,
          0,
          &(chartInstance->chartNumber),
          &(chartInstance->instanceNumber),
          ssGetPath(S),
          (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_UKF_1hzMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_UKF_1hzMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_UKF_1hzMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Initial_Attitude_C");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Attitude_I_C");
          _SFD_SET_DATA_PROPS(2,1,1,0,"w_cont");
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
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,608);
        _SFD_CV_INIT_EML_IF(0,1,0,98,111,224,608);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)
            c1_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c1_Initial_Attitude_C)[4];
          real_T (*c1_Attitude_I_C)[4];
          real_T (*c1_w_cont)[3];
          c1_w_cont = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c1_Attitude_I_C = (real_T (*)[4])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c1_Initial_Attitude_C = (real_T (*)[4])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_Initial_Attitude_C);
          _SFD_SET_DATA_VALUE_PTR(1U, *c1_Attitude_I_C);
          _SFD_SET_DATA_VALUE_PTR(2U, *c1_w_cont);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_UKF_1hzMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "uXgvOZNQANaq1RVyryw9aH";
}

static void sf_opaque_initialize_c1_UKF_1hz(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar)->S,
    0);
  initialize_params_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
  initialize_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_UKF_1hz(void *chartInstanceVar)
{
  enable_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_UKF_1hz(void *chartInstanceVar)
{
  disable_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_UKF_1hz(void *chartInstanceVar)
{
  sf_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_UKF_1hz(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_UKF_1hz();/* state var info */
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

extern void sf_internal_set_sim_state_c1_UKF_1hz(SimStruct* S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_UKF_1hz();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*)chartInfo->chartInstance,
    mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_UKF_1hz(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_UKF_1hz(S);
}

static void sf_opaque_set_sim_state_c1_UKF_1hz(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_UKF_1hz(S, st);
}

static void sf_opaque_terminate_c1_UKF_1hz(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_UKF_1hz_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_UKF_1hz(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_UKF_1hz((SFc1_UKF_1hzInstanceStruct*)(((ChartInfoStruct
      *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_UKF_1hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_UKF_1hz_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2826233862U));
  ssSetChecksum1(S,(1670347304U));
  ssSetChecksum2(S,(3514440421U));
  ssSetChecksum3(S,(3000338617U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_UKF_1hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_UKF_1hz(SimStruct *S)
{
  SFc1_UKF_1hzInstanceStruct *chartInstance;
  chartInstance = (SFc1_UKF_1hzInstanceStruct *)malloc(sizeof
    (SFc1_UKF_1hzInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_UKF_1hzInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_UKF_1hz;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_UKF_1hz;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_UKF_1hz;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_UKF_1hz;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_UKF_1hz;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_UKF_1hz;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_UKF_1hz;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_UKF_1hz;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_UKF_1hz;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_UKF_1hz;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_UKF_1hz;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_UKF_1hz_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_UKF_1hz(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_UKF_1hz(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_UKF_1hz(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_UKF_1hz_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
