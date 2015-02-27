/* Include files */

#include "blascompat32.h"
#include "BDotControllerDiscrete_sfun.h"
#include "c8_BDotControllerDiscrete.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "BDotControllerDiscrete_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c8_debug_family_names[10] = { "Dipole_Gain", "nargin",
  "nargout", "B_s", "Command_Trigger", "I_c", "Ts", "Dipole_Command_s", "Bint",
  "Bdot" };

/* Function Declarations */
static void initialize_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void initialize_params_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void enable_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void disable_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void c8_update_debugger_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void set_sim_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance, const mxArray
   *c8_st);
static void finalize_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void sf_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void c8_chartstep_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void initSimStructsc8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static void c8_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_Bdot, const char_T *c8_identifier, real_T
  c8_y[3]);
static void c8_b_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_c_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_Bint, const char_T *c8_identifier, real_T
  c8_y[3]);
static void c8_d_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3]);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_e_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_Dipole_Command_s, const char_T
  *c8_identifier, real_T c8_y[3]);
static void c8_f_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3]);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static real_T c8_g_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[16]);
static void c8_eml_scalar_eg(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance);
static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_h_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_i_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_BDotControllerDiscrete, const
  char_T *c8_identifier);
static uint8_T c8_j_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c8_Bint_not_empty = FALSE;
  chartInstance->c8_Bdot_not_empty = FALSE;
  chartInstance->c8_is_active_c8_BDotControllerDiscrete = 0U;
}

static void initialize_params_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
}

static void enable_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c8_update_debugger_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i0;
  real_T c8_u[3];
  const mxArray *c8_b_y = NULL;
  int32_T c8_i1;
  real_T c8_b_u[3];
  const mxArray *c8_c_y = NULL;
  int32_T c8_i2;
  real_T c8_c_u[3];
  const mxArray *c8_d_y = NULL;
  uint8_T c8_hoistedGlobal;
  uint8_T c8_d_u;
  const mxArray *c8_e_y = NULL;
  real_T (*c8_Dipole_Command_s)[3];
  c8_Dipole_Command_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellarray(4), FALSE);
  for (c8_i0 = 0; c8_i0 < 3; c8_i0++) {
    c8_u[c8_i0] = (*c8_Dipole_Command_s)[c8_i0];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  for (c8_i1 = 0; c8_i1 < 3; c8_i1++) {
    c8_b_u[c8_i1] = chartInstance->c8_Bdot[c8_i1];
  }

  c8_c_y = NULL;
  if (!chartInstance->c8_Bdot_not_empty) {
    sf_mex_assign(&c8_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c8_c_y, sf_mex_create("y", c8_b_u, 0, 0U, 1U, 0U, 1, 3),
                  FALSE);
  }

  sf_mex_setcell(c8_y, 1, c8_c_y);
  for (c8_i2 = 0; c8_i2 < 3; c8_i2++) {
    c8_c_u[c8_i2] = chartInstance->c8_Bint[c8_i2];
  }

  c8_d_y = NULL;
  if (!chartInstance->c8_Bint_not_empty) {
    sf_mex_assign(&c8_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c8_d_y, sf_mex_create("y", c8_c_u, 0, 0U, 1U, 0U, 1, 3),
                  FALSE);
  }

  sf_mex_setcell(c8_y, 2, c8_d_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_BDotControllerDiscrete;
  c8_d_u = c8_hoistedGlobal;
  c8_e_y = NULL;
  sf_mex_assign(&c8_e_y, sf_mex_create("y", &c8_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c8_y, 3, c8_e_y);
  sf_mex_assign(&c8_st, c8_y, FALSE);
  return c8_st;
}

static void set_sim_state_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance, const mxArray
   *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv0[3];
  int32_T c8_i3;
  real_T c8_dv1[3];
  int32_T c8_i4;
  real_T c8_dv2[3];
  int32_T c8_i5;
  real_T (*c8_Dipole_Command_s)[3];
  c8_Dipole_Command_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = TRUE;
  c8_u = sf_mex_dup(c8_st);
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                        "Dipole_Command_s", c8_dv0);
  for (c8_i3 = 0; c8_i3 < 3; c8_i3++) {
    (*c8_Dipole_Command_s)[c8_i3] = c8_dv0[c8_i3];
  }

  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)), "Bdot",
                      c8_dv1);
  for (c8_i4 = 0; c8_i4 < 3; c8_i4++) {
    chartInstance->c8_Bdot[c8_i4] = c8_dv1[c8_i4];
  }

  c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 2)),
                        "Bint", c8_dv2);
  for (c8_i5 = 0; c8_i5 < 3; c8_i5++) {
    chartInstance->c8_Bint[c8_i5] = c8_dv2[c8_i5];
  }

  chartInstance->c8_is_active_c8_BDotControllerDiscrete = c8_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 3)),
     "is_active_c8_BDotControllerDiscrete");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_BDotControllerDiscrete(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
}

static void sf_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  int32_T c8_i6;
  int32_T c8_i7;
  int32_T c8_i8;
  real_T *c8_Command_Trigger;
  real_T *c8_Ts;
  real_T (*c8_I_c)[3];
  real_T (*c8_Dipole_Command_s)[3];
  real_T (*c8_B_s)[3];
  c8_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c8_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c8_Command_Trigger = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c8_Dipole_Command_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_B_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c8_sfEvent);
  for (c8_i6 = 0; c8_i6 < 3; c8_i6++) {
    _SFD_DATA_RANGE_CHECK((*c8_B_s)[c8_i6], 0U);
  }

  for (c8_i7 = 0; c8_i7 < 3; c8_i7++) {
    _SFD_DATA_RANGE_CHECK((*c8_Dipole_Command_s)[c8_i7], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c8_Command_Trigger, 2U);
  for (c8_i8 = 0; c8_i8 < 3; c8_i8++) {
    _SFD_DATA_RANGE_CHECK((*c8_I_c)[c8_i8], 3U);
  }

  _SFD_DATA_RANGE_CHECK(*c8_Ts, 4U);
  chartInstance->c8_sfEvent = CALL_EVENT;
  c8_chartstep_c8_BDotControllerDiscrete(chartInstance);
  sf_debug_check_for_state_inconsistency(_BDotControllerDiscreteMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c8_chartstep_c8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
  real_T c8_hoistedGlobal;
  real_T c8_b_hoistedGlobal;
  int32_T c8_i9;
  real_T c8_B_s[3];
  real_T c8_Command_Trigger;
  int32_T c8_i10;
  real_T c8_I_c[3];
  real_T c8_Ts;
  uint32_T c8_debug_family_var_map[10];
  real_T c8_Dipole_Gain;
  real_T c8_nargin = 4.0;
  real_T c8_nargout = 1.0;
  real_T c8_Dipole_Command_s[3];
  int32_T c8_i11;
  int32_T c8_i12;
  real_T c8_c_hoistedGlobal[3];
  int32_T c8_i13;
  int32_T c8_i14;
  int32_T c8_i15;
  real_T c8_b;
  int32_T c8_i16;
  int32_T c8_i17;
  int32_T c8_i18;
  int32_T c8_i19;
  int32_T c8_i20;
  int32_T c8_i21;
  int32_T c8_i22;
  real_T c8_C[3];
  int32_T c8_i23;
  int32_T c8_i24;
  int32_T c8_i25;
  int32_T c8_i26;
  int32_T c8_i27;
  int32_T c8_i28;
  static real_T c8_a[9] = { -2.15E+7, -0.0, -0.0, -0.0, -2.15E+7, -0.0, -0.0,
    -0.0, -1.95E+7 };

  int32_T c8_i29;
  real_T *c8_b_Command_Trigger;
  real_T *c8_b_Ts;
  real_T (*c8_b_Dipole_Command_s)[3];
  real_T (*c8_b_I_c)[3];
  real_T (*c8_b_B_s)[3];
  boolean_T guard1 = FALSE;
  c8_b_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c8_b_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c8_b_Command_Trigger = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c8_b_Dipole_Command_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S,
    1);
  c8_b_B_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c8_sfEvent);
  c8_hoistedGlobal = *c8_b_Command_Trigger;
  c8_b_hoistedGlobal = *c8_b_Ts;
  for (c8_i9 = 0; c8_i9 < 3; c8_i9++) {
    c8_B_s[c8_i9] = (*c8_b_B_s)[c8_i9];
  }

  c8_Command_Trigger = c8_hoistedGlobal;
  for (c8_i10 = 0; c8_i10 < 3; c8_i10++) {
    c8_I_c[c8_i10] = (*c8_b_I_c)[c8_i10];
  }

  c8_Ts = c8_b_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 10U, 10U, c8_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c8_Dipole_Gain, 0U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 1U, c8_d_sf_marshallOut,
    c8_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 2U, c8_d_sf_marshallOut,
    c8_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_B_s, 3U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c8_Command_Trigger, 4U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_I_c, 5U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c8_Ts, 6U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c8_Dipole_Command_s, 7U,
    c8_c_sf_marshallOut, c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c8_Bint, 8U,
    c8_b_sf_marshallOut, c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c8_Bdot, 9U,
    c8_sf_marshallOut, c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 10);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  c8_Dipole_Gain = -5.0E+6;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 13);
  guard1 = FALSE;
  if (CV_EML_COND(0, 1, 0, c8_Command_Trigger == 1.0)) {
    guard1 = TRUE;
  } else if (CV_EML_COND(0, 1, 1, !chartInstance->c8_Bdot_not_empty)) {
    guard1 = TRUE;
  } else {
    CV_EML_MCDC(0, 1, 0, FALSE);
    CV_EML_IF(0, 1, 0, FALSE);
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 27);
    for (c8_i19 = 0; c8_i19 < 3; c8_i19++) {
      c8_c_hoistedGlobal[c8_i19] = chartInstance->c8_Bdot[c8_i19];
    }

    c8_eml_scalar_eg(chartInstance);
    c8_eml_scalar_eg(chartInstance);
    for (c8_i20 = 0; c8_i20 < 3; c8_i20++) {
      c8_Dipole_Command_s[c8_i20] = 0.0;
    }

    for (c8_i21 = 0; c8_i21 < 3; c8_i21++) {
      c8_Dipole_Command_s[c8_i21] = 0.0;
    }

    for (c8_i22 = 0; c8_i22 < 3; c8_i22++) {
      c8_C[c8_i22] = c8_Dipole_Command_s[c8_i22];
    }

    for (c8_i23 = 0; c8_i23 < 3; c8_i23++) {
      c8_Dipole_Command_s[c8_i23] = c8_C[c8_i23];
    }

    for (c8_i24 = 0; c8_i24 < 3; c8_i24++) {
      c8_C[c8_i24] = c8_Dipole_Command_s[c8_i24];
    }

    for (c8_i25 = 0; c8_i25 < 3; c8_i25++) {
      c8_Dipole_Command_s[c8_i25] = c8_C[c8_i25];
    }

    for (c8_i26 = 0; c8_i26 < 3; c8_i26++) {
      c8_Dipole_Command_s[c8_i26] = 0.0;
      c8_i27 = 0;
      for (c8_i28 = 0; c8_i28 < 3; c8_i28++) {
        c8_Dipole_Command_s[c8_i26] += c8_a[c8_i27 + c8_i26] *
          c8_c_hoistedGlobal[c8_i28];
        c8_i27 += 3;
      }
    }
  }

  if (guard1 == TRUE) {
    CV_EML_MCDC(0, 1, 0, TRUE);
    CV_EML_IF(0, 1, 0, TRUE);
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 15);
    if (CV_EML_IF(0, 1, 1, !chartInstance->c8_Bint_not_empty)) {
      _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 16);
      for (c8_i11 = 0; c8_i11 < 3; c8_i11++) {
        chartInstance->c8_Bint[c8_i11] = 0.0;
      }

      chartInstance->c8_Bint_not_empty = TRUE;
    }

    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 21);
    for (c8_i12 = 0; c8_i12 < 3; c8_i12++) {
      c8_c_hoistedGlobal[c8_i12] = chartInstance->c8_Bint[c8_i12];
    }

    for (c8_i13 = 0; c8_i13 < 3; c8_i13++) {
      c8_c_hoistedGlobal[c8_i13] = c8_B_s[c8_i13] - c8_c_hoistedGlobal[c8_i13];
    }

    for (c8_i14 = 0; c8_i14 < 3; c8_i14++) {
      chartInstance->c8_Bdot[c8_i14] = 0.8 * c8_c_hoistedGlobal[c8_i14];
    }

    chartInstance->c8_Bdot_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 22);
    for (c8_i15 = 0; c8_i15 < 3; c8_i15++) {
      c8_c_hoistedGlobal[c8_i15] = chartInstance->c8_Bdot[c8_i15];
    }

    c8_b = c8_Ts;
    for (c8_i16 = 0; c8_i16 < 3; c8_i16++) {
      c8_c_hoistedGlobal[c8_i16] *= c8_b;
    }

    for (c8_i17 = 0; c8_i17 < 3; c8_i17++) {
      chartInstance->c8_Bint[c8_i17] += c8_c_hoistedGlobal[c8_i17];
    }

    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 23);
    for (c8_i18 = 0; c8_i18 < 3; c8_i18++) {
      c8_Dipole_Command_s[c8_i18] = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -27);
  sf_debug_symbol_scope_pop();
  for (c8_i29 = 0; c8_i29 < 3; c8_i29++) {
    (*c8_b_Dipole_Command_s)[c8_i29] = c8_Dipole_Command_s[c8_i29];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c8_sfEvent);
}

static void initSimStructsc8_BDotControllerDiscrete
  (SFc8_BDotControllerDiscreteInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber)
{
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i30;
  real_T c8_b_inData[3];
  int32_T c8_i31;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i30 = 0; c8_i30 < 3; c8_i30++) {
    c8_b_inData[c8_i30] = (*(real_T (*)[3])c8_inData)[c8_i30];
  }

  for (c8_i31 = 0; c8_i31 < 3; c8_i31++) {
    c8_u[c8_i31] = c8_b_inData[c8_i31];
  }

  c8_y = NULL;
  if (!chartInstance->c8_Bdot_not_empty) {
    sf_mex_assign(&c8_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  }

  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_Bdot, const char_T *c8_identifier, real_T
  c8_y[3])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_Bdot), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_Bdot);
}

static void c8_b_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3])
{
  real_T c8_dv3[3];
  int32_T c8_i32;
  if (mxIsEmpty(c8_u)) {
    chartInstance->c8_Bdot_not_empty = FALSE;
  } else {
    chartInstance->c8_Bdot_not_empty = TRUE;
    sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv3, 1, 0, 0U, 1, 0U, 1, 3);
    for (c8_i32 = 0; c8_i32 < 3; c8_i32++) {
      c8_y[c8_i32] = c8_dv3[c8_i32];
    }
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_Bdot;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[3];
  int32_T c8_i33;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_b_Bdot = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_Bdot), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_Bdot);
  for (c8_i33 = 0; c8_i33 < 3; c8_i33++) {
    (*(real_T (*)[3])c8_outData)[c8_i33] = c8_y[c8_i33];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i34;
  real_T c8_b_inData[3];
  int32_T c8_i35;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i34 = 0; c8_i34 < 3; c8_i34++) {
    c8_b_inData[c8_i34] = (*(real_T (*)[3])c8_inData)[c8_i34];
  }

  for (c8_i35 = 0; c8_i35 < 3; c8_i35++) {
    c8_u[c8_i35] = c8_b_inData[c8_i35];
  }

  c8_y = NULL;
  if (!chartInstance->c8_Bint_not_empty) {
    sf_mex_assign(&c8_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  }

  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_c_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_Bint, const char_T *c8_identifier, real_T
  c8_y[3])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_Bint), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_Bint);
}

static void c8_d_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3])
{
  real_T c8_dv4[3];
  int32_T c8_i36;
  if (mxIsEmpty(c8_u)) {
    chartInstance->c8_Bint_not_empty = FALSE;
  } else {
    chartInstance->c8_Bint_not_empty = TRUE;
    sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv4, 1, 0, 0U, 1, 0U, 1, 3);
    for (c8_i36 = 0; c8_i36 < 3; c8_i36++) {
      c8_y[c8_i36] = c8_dv4[c8_i36];
    }
  }

  sf_mex_destroy(&c8_u);
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_Bint;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[3];
  int32_T c8_i37;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_b_Bint = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_Bint), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_Bint);
  for (c8_i37 = 0; c8_i37 < 3; c8_i37++) {
    (*(real_T (*)[3])c8_outData)[c8_i37] = c8_y[c8_i37];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i38;
  real_T c8_b_inData[3];
  int32_T c8_i39;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i38 = 0; c8_i38 < 3; c8_i38++) {
    c8_b_inData[c8_i38] = (*(real_T (*)[3])c8_inData)[c8_i38];
  }

  for (c8_i39 = 0; c8_i39 < 3; c8_i39++) {
    c8_u[c8_i39] = c8_b_inData[c8_i39];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_e_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_Dipole_Command_s, const char_T
  *c8_identifier, real_T c8_y[3])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_Dipole_Command_s),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_Dipole_Command_s);
}

static void c8_f_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3])
{
  real_T c8_dv5[3];
  int32_T c8_i40;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv5, 1, 0, 0U, 1, 0U, 1, 3);
  for (c8_i40 = 0; c8_i40 < 3; c8_i40++) {
    c8_y[c8_i40] = c8_dv5[c8_i40];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_Dipole_Command_s;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[3];
  int32_T c8_i41;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_Dipole_Command_s = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_Dipole_Command_s),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_Dipole_Command_s);
  for (c8_i41 = 0; c8_i41 < 3; c8_i41++) {
    (*(real_T (*)[3])c8_outData)[c8_i41] = c8_y[c8_i41];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static real_T c8_g_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_BDotControllerDiscrete_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo;
  c8_ResolvedFunctionInfo c8_info[16];
  const mxArray *c8_m0 = NULL;
  int32_T c8_i42;
  c8_ResolvedFunctionInfo *c8_r0;
  c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  c8_info_helper(c8_info);
  sf_mex_assign(&c8_m0, sf_mex_createstruct("nameCaptureInfo", 1, 16), FALSE);
  for (c8_i42 = 0; c8_i42 < 16; c8_i42++) {
    c8_r0 = &c8_info[c8_i42];
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->context)), "context", "nameCaptureInfo",
                    c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c8_r0->name)), "name", "nameCaptureInfo", c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c8_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->resolved)), "resolved", "nameCaptureInfo",
                    c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c8_i42);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c8_i42);
  }

  sf_mex_assign(&c8_nameCaptureInfo, c8_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[16])
{
  c8_info[0].context = "";
  c8_info[0].name = "mtimes";
  c8_info[0].dominantType = "double";
  c8_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[0].fileTimeLo = 1289541292U;
  c8_info[0].fileTimeHi = 0U;
  c8_info[0].mFileTimeLo = 0U;
  c8_info[0].mFileTimeHi = 0U;
  c8_info[1].context = "";
  c8_info[1].name = "diag";
  c8_info[1].dominantType = "double";
  c8_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c8_info[1].fileTimeLo = 1286840286U;
  c8_info[1].fileTimeHi = 0U;
  c8_info[1].mFileTimeLo = 0U;
  c8_info[1].mFileTimeHi = 0U;
  c8_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c8_info[2].name = "eml_index_class";
  c8_info[2].dominantType = "";
  c8_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[2].fileTimeLo = 1323192178U;
  c8_info[2].fileTimeHi = 0U;
  c8_info[2].mFileTimeLo = 0U;
  c8_info[2].mFileTimeHi = 0U;
  c8_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c8_info[3].name = "eml_index_plus";
  c8_info[3].dominantType = "coder.internal.indexInt";
  c8_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[3].fileTimeLo = 1286840378U;
  c8_info[3].fileTimeHi = 0U;
  c8_info[3].mFileTimeLo = 0U;
  c8_info[3].mFileTimeHi = 0U;
  c8_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[4].name = "eml_index_class";
  c8_info[4].dominantType = "";
  c8_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[4].fileTimeLo = 1323192178U;
  c8_info[4].fileTimeHi = 0U;
  c8_info[4].mFileTimeLo = 0U;
  c8_info[4].mFileTimeHi = 0U;
  c8_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c8_info[5].name = "eml_scalar_eg";
  c8_info[5].dominantType = "double";
  c8_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[5].fileTimeLo = 1286840396U;
  c8_info[5].fileTimeHi = 0U;
  c8_info[5].mFileTimeLo = 0U;
  c8_info[5].mFileTimeHi = 0U;
  c8_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c8_info[6].name = "eml_int_forloop_overflow_check";
  c8_info[6].dominantType = "";
  c8_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[6].fileTimeLo = 1332186672U;
  c8_info[6].fileTimeHi = 0U;
  c8_info[6].mFileTimeLo = 0U;
  c8_info[6].mFileTimeHi = 0U;
  c8_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c8_info[7].name = "intmax";
  c8_info[7].dominantType = "char";
  c8_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c8_info[7].fileTimeLo = 1311276916U;
  c8_info[7].fileTimeHi = 0U;
  c8_info[7].mFileTimeLo = 0U;
  c8_info[7].mFileTimeHi = 0U;
  c8_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[8].name = "eml_index_class";
  c8_info[8].dominantType = "";
  c8_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[8].fileTimeLo = 1323192178U;
  c8_info[8].fileTimeHi = 0U;
  c8_info[8].mFileTimeLo = 0U;
  c8_info[8].mFileTimeHi = 0U;
  c8_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[9].name = "eml_scalar_eg";
  c8_info[9].dominantType = "double";
  c8_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[9].fileTimeLo = 1286840396U;
  c8_info[9].fileTimeHi = 0U;
  c8_info[9].mFileTimeLo = 0U;
  c8_info[9].mFileTimeHi = 0U;
  c8_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[10].name = "eml_xgemm";
  c8_info[10].dominantType = "char";
  c8_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c8_info[10].fileTimeLo = 1299098372U;
  c8_info[10].fileTimeHi = 0U;
  c8_info[10].mFileTimeLo = 0U;
  c8_info[10].mFileTimeHi = 0U;
  c8_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c8_info[11].name = "eml_blas_inline";
  c8_info[11].dominantType = "";
  c8_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[11].fileTimeLo = 1299098368U;
  c8_info[11].fileTimeHi = 0U;
  c8_info[11].mFileTimeLo = 0U;
  c8_info[11].mFileTimeHi = 0U;
  c8_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c8_info[12].name = "mtimes";
  c8_info[12].dominantType = "double";
  c8_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[12].fileTimeLo = 1289541292U;
  c8_info[12].fileTimeHi = 0U;
  c8_info[12].mFileTimeLo = 0U;
  c8_info[12].mFileTimeHi = 0U;
  c8_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[13].name = "eml_index_class";
  c8_info[13].dominantType = "";
  c8_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[13].fileTimeLo = 1323192178U;
  c8_info[13].fileTimeHi = 0U;
  c8_info[13].mFileTimeLo = 0U;
  c8_info[13].mFileTimeHi = 0U;
  c8_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[14].name = "eml_scalar_eg";
  c8_info[14].dominantType = "double";
  c8_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[14].fileTimeLo = 1286840396U;
  c8_info[14].fileTimeHi = 0U;
  c8_info[14].mFileTimeLo = 0U;
  c8_info[14].mFileTimeHi = 0U;
  c8_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[15].name = "eml_refblas_xgemm";
  c8_info[15].dominantType = "char";
  c8_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c8_info[15].fileTimeLo = 1299098374U;
  c8_info[15].fileTimeHi = 0U;
  c8_info[15].mFileTimeLo = 0U;
  c8_info[15].mFileTimeHi = 0U;
}

static void c8_eml_scalar_eg(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance)
{
}

static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static int32_T c8_h_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i43;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i43, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i43;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_i_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_BDotControllerDiscrete, const
  char_T *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_BDotControllerDiscrete), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_BDotControllerDiscrete);
  return c8_y;
}

static uint8_T c8_j_emlrt_marshallIn(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_BDotControllerDiscreteInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c8_BDotControllerDiscrete_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3043460674U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3961175812U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3641478403U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1102159433U);
}

mxArray *sf_c8_BDotControllerDiscrete_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("RYmXw0zAoKvHoRKVaGusWF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      pr[0] = (double)(1);
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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c8_BDotControllerDiscrete(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"Dipole_Command_s\",},{M[4],M[0],T\"Bdot\",S'l','i','p'{{M1x2[251 255],M[0],}}},{M[4],M[0],T\"Bint\",S'l','i','p'{{M1x2[246 250],M[0],}}},{M[8],M[0],T\"is_active_c8_BDotControllerDiscrete\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_BDotControllerDiscrete_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
    chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_BDotControllerDiscreteMachineNumber_,
           8,
           1,
           1,
           5,
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
          init_script_number_translation(_BDotControllerDiscreteMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_BDotControllerDiscreteMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds
            (_BDotControllerDiscreteMachineNumber_,
             chartInstance->chartNumber,
             0,
             0,
             0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"B_s");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Dipole_Command_s");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Command_Trigger");
          _SFD_SET_DATA_PROPS(3,1,1,0,"I_c");
          _SFD_SET_DATA_PROPS(4,1,1,0,"Ts");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,651);
        _SFD_CV_INIT_EML_IF(0,1,0,277,317,566,650);
        _SFD_CV_INIT_EML_IF(0,1,1,378,394,426,459);

        {
          static int condStart[] = { 280, 304 };

          static int condEnd[] = { 300, 317 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,280,317,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

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
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c8_Command_Trigger;
          real_T *c8_Ts;
          real_T (*c8_B_s)[3];
          real_T (*c8_Dipole_Command_s)[3];
          real_T (*c8_I_c)[3];
          c8_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c8_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c8_Command_Trigger = (real_T *)ssGetInputPortSignal(chartInstance->S,
            1);
          c8_Dipole_Command_s = (real_T (*)[3])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c8_B_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_B_s);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_Dipole_Command_s);
          _SFD_SET_DATA_VALUE_PTR(2U, c8_Command_Trigger);
          _SFD_SET_DATA_VALUE_PTR(3U, *c8_I_c);
          _SFD_SET_DATA_VALUE_PTR(4U, c8_Ts);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_BDotControllerDiscreteMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "Y59oUbGQ5Hk7okoKvgxtd";
}

static void sf_opaque_initialize_c8_BDotControllerDiscrete(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_BDotControllerDiscreteInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c8_BDotControllerDiscrete
    ((SFc8_BDotControllerDiscreteInstanceStruct*) chartInstanceVar);
  initialize_c8_BDotControllerDiscrete
    ((SFc8_BDotControllerDiscreteInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c8_BDotControllerDiscrete(void *chartInstanceVar)
{
  enable_c8_BDotControllerDiscrete((SFc8_BDotControllerDiscreteInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c8_BDotControllerDiscrete(void *chartInstanceVar)
{
  disable_c8_BDotControllerDiscrete((SFc8_BDotControllerDiscreteInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c8_BDotControllerDiscrete(void *chartInstanceVar)
{
  sf_c8_BDotControllerDiscrete((SFc8_BDotControllerDiscreteInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_BDotControllerDiscrete
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_BDotControllerDiscrete
    ((SFc8_BDotControllerDiscreteInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_BDotControllerDiscrete();/* state var info */
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

extern void sf_internal_set_sim_state_c8_BDotControllerDiscrete(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_BDotControllerDiscrete();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_BDotControllerDiscrete
    ((SFc8_BDotControllerDiscreteInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_BDotControllerDiscrete
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c8_BDotControllerDiscrete(S);
}

static void sf_opaque_set_sim_state_c8_BDotControllerDiscrete(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c8_BDotControllerDiscrete(S, st);
}

static void sf_opaque_terminate_c8_BDotControllerDiscrete(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_BDotControllerDiscreteInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c8_BDotControllerDiscrete
      ((SFc8_BDotControllerDiscreteInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_BDotControllerDiscrete_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_BDotControllerDiscrete
    ((SFc8_BDotControllerDiscreteInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_BDotControllerDiscrete(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c8_BDotControllerDiscrete
      ((SFc8_BDotControllerDiscreteInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_BDotControllerDiscrete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_BDotControllerDiscrete_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,8,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2278290670U));
  ssSetChecksum1(S,(3545931051U));
  ssSetChecksum2(S,(3469086077U));
  ssSetChecksum3(S,(1410716463U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c8_BDotControllerDiscrete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_BDotControllerDiscrete(SimStruct *S)
{
  SFc8_BDotControllerDiscreteInstanceStruct *chartInstance;
  chartInstance = (SFc8_BDotControllerDiscreteInstanceStruct *)malloc(sizeof
    (SFc8_BDotControllerDiscreteInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_BDotControllerDiscreteInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_BDotControllerDiscrete;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_BDotControllerDiscrete;
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

void c8_BDotControllerDiscrete_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_BDotControllerDiscrete(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_BDotControllerDiscrete(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_BDotControllerDiscrete(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_BDotControllerDiscrete_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
