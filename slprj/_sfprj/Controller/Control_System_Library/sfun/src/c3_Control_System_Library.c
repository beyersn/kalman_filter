/* Include files */

#include "blascompat32.h"
#include "Control_System_Library_sfun.h"
#include "c3_Control_System_Library.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Control_System_Library_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c3_debug_family_names[13] = { "muC", "Cd", "R_mag", "v",
  "rho", "r_cg", "r_cp", "r_cpcg", "A", "nargin", "nargout", "R", "ATM_Torque" };

/* Function Declarations */
static void initialize_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void initialize_params_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void enable_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void disable_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void c3_update_debugger_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void set_sim_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c3_st);
static void finalize_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void sf_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void c3_chartstep_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void initSimStructsc3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_ATM_Torque, const char_T *c3_identifier,
  real_T c3_y[3]);
static void c3_b_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_c_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[22]);
static void c3_eml_scalar_eg(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c3_check_forloop_overflow_error
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance);
static void c3_eml_error(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_d_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_e_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_Control_System_Library, const
  char_T *c3_identifier);
static uint8_T c3_f_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void init_dsm_address_info(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_Control_System_Library = 0U;
}

static void initialize_params_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void enable_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[3];
  const mxArray *c3_b_y = NULL;
  uint8_T c3_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T (*c3_ATM_Torque)[3];
  c3_ATM_Torque = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(2), FALSE);
  for (c3_i0 = 0; c3_i0 < 3; c3_i0++) {
    c3_u[c3_i0] = (*c3_ATM_Torque)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_hoistedGlobal = chartInstance->c3_is_active_c3_Control_System_Library;
  c3_b_u = c3_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[3];
  int32_T c3_i1;
  real_T (*c3_ATM_Torque)[3];
  c3_ATM_Torque = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)),
                      "ATM_Torque", c3_dv0);
  for (c3_i1 = 0; c3_i1 < 3; c3_i1++) {
    (*c3_ATM_Torque)[c3_i1] = c3_dv0[c3_i1];
  }

  chartInstance->c3_is_active_c3_Control_System_Library = c3_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
     "is_active_c3_Control_System_Library");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_Control_System_Library(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void sf_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c3_i2;
  int32_T c3_i3;
  real_T (*c3_R)[3];
  real_T (*c3_ATM_Torque)[3];
  c3_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  c3_ATM_Torque = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  for (c3_i2 = 0; c3_i2 < 3; c3_i2++) {
    _SFD_DATA_RANGE_CHECK((*c3_ATM_Torque)[c3_i2], 0U);
  }

  for (c3_i3 = 0; c3_i3 < 3; c3_i3++) {
    _SFD_DATA_RANGE_CHECK((*c3_R)[c3_i3], 1U);
  }

  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_Control_System_Library(chartInstance);
  sf_debug_check_for_state_inconsistency(_Control_System_LibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c3_chartstep_c3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c3_i4;
  real_T c3_R[3];
  uint32_T c3_debug_family_var_map[13];
  real_T c3_muC;
  real_T c3_Cd;
  real_T c3_R_mag;
  real_T c3_v;
  real_T c3_rho;
  real_T c3_r_cg[3];
  real_T c3_r_cp[3];
  real_T c3_r_cpcg[3];
  real_T c3_A[3];
  real_T c3_nargin = 1.0;
  real_T c3_nargout = 1.0;
  real_T c3_ATM_Torque[3];
  int32_T c3_i5;
  real_T c3_a[3];
  int32_T c3_k;
  real_T c3_b_k;
  real_T c3_ak;
  real_T c3_y[3];
  real_T c3_b_y;
  int32_T c3_c_k;
  int32_T c3_d_k;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_B;
  real_T c3_c_y;
  real_T c3_d_y;
  real_T c3_e_y;
  real_T c3_c_x;
  real_T c3_d_x;
  int32_T c3_i6;
  static real_T c3_dv1[3] = { 0.2054, 0.21, 0.2984 };

  int32_T c3_i7;
  static real_T c3_dv2[3] = { 0.2075, 0.2075, 0.3714 };

  int32_T c3_i8;
  static real_T c3_dv3[3] = { 0.0020999999999999908, -0.0025000000000000022,
    0.073000000000000009 };

  int32_T c3_i9;
  static real_T c3_dv4[3] = { 0.309, 0.309, 0.658 };

  real_T c3_b_a;
  real_T c3_c_a;
  real_T c3_d_a;
  real_T c3_b_ak;
  real_T c3_c;
  real_T c3_b;
  int32_T c3_i10;
  static real_T c3_e_a[3] = { 8.1112499999999649E-17, -9.656250000000009E-17,
    6.0042500000000013E-15 };

  int32_T c3_i11;
  real_T (*c3_b_ATM_Torque)[3];
  real_T (*c3_b_R)[3];
  c3_b_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  c3_b_ATM_Torque = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
  for (c3_i4 = 0; c3_i4 < 3; c3_i4++) {
    c3_R[c3_i4] = (*c3_b_R)[c3_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 13U, 13U, c3_debug_family_names,
    c3_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c3_muC, 0U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c3_Cd, 1U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c3_R_mag, 2U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_v, 3U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c3_rho, 4U, c3_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_r_cg, 5U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_r_cp, 6U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_r_cpcg, 7U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c3_A, 8U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargin, 9U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c3_nargout, 10U, c3_b_sf_marshallOut,
    c3_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c3_R, 11U, c3_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c3_ATM_Torque, 12U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
  c3_muC = 3.986004418E+14;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 12);
  c3_Cd = 2.5;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 13);
  for (c3_i5 = 0; c3_i5 < 3; c3_i5++) {
    c3_a[c3_i5] = c3_R[c3_i5];
  }

  for (c3_k = 0; c3_k < 3; c3_k++) {
    c3_b_k = 1.0 + (real_T)c3_k;
    c3_ak = c3_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c3_b_k), 1, 3, 1, 0) - 1];
    c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c3_b_k),
      1, 3, 1, 0) - 1] = muDoubleScalarPower(c3_ak, 2.0);
  }

  c3_b_y = c3_y[0];
  c3_check_forloop_overflow_error(chartInstance);
  for (c3_c_k = 2; c3_c_k < 4; c3_c_k++) {
    c3_d_k = c3_c_k;
    c3_b_y += c3_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c3_d_k), 1, 3, 1, 0) - 1];
  }

  c3_x = c3_b_y;
  c3_R_mag = c3_x;
  if (c3_R_mag < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_b_x = c3_R_mag;
  c3_R_mag = c3_b_x;
  c3_R_mag = muDoubleScalarSqrt(c3_R_mag);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
  c3_B = c3_R_mag;
  c3_c_y = c3_B;
  c3_d_y = c3_c_y;
  c3_e_y = 3.986004418E+14 / c3_d_y;
  c3_c_x = c3_e_y;
  c3_v = c3_c_x;
  if (c3_v < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_d_x = c3_v;
  c3_v = c3_d_x;
  c3_v = muDoubleScalarSqrt(c3_v);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 15);
  c3_rho = 1.0E-13;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
  for (c3_i6 = 0; c3_i6 < 3; c3_i6++) {
    c3_r_cg[c3_i6] = c3_dv1[c3_i6];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 17);
  for (c3_i7 = 0; c3_i7 < 3; c3_i7++) {
    c3_r_cp[c3_i7] = c3_dv2[c3_i7];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
  for (c3_i8 = 0; c3_i8 < 3; c3_i8++) {
    c3_r_cpcg[c3_i8] = c3_dv3[c3_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
  for (c3_i9 = 0; c3_i9 < 3; c3_i9++) {
    c3_A[c3_i9] = c3_dv4[c3_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 21);
  c3_b_a = c3_v;
  c3_c_a = c3_b_a;
  c3_d_a = c3_c_a;
  c3_eml_scalar_eg(chartInstance);
  c3_b_ak = c3_d_a;
  c3_c = muDoubleScalarPower(c3_b_ak, 2.0);
  c3_b = c3_c;
  for (c3_i10 = 0; c3_i10 < 3; c3_i10++) {
    c3_ATM_Torque[c3_i10] = c3_e_a[c3_i10] * c3_b;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -21);
  sf_debug_symbol_scope_pop();
  for (c3_i11 = 0; c3_i11 < 3; c3_i11++) {
    (*c3_b_ATM_Torque)[c3_i11] = c3_ATM_Torque[c3_i11];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_Control_System_Library
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i12;
  real_T c3_b_inData[3];
  int32_T c3_i13;
  real_T c3_u[3];
  const mxArray *c3_y = NULL;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i12 = 0; c3_i12 < 3; c3_i12++) {
    c3_b_inData[c3_i12] = (*(real_T (*)[3])c3_inData)[c3_i12];
  }

  for (c3_i13 = 0; c3_i13 < 3; c3_i13++) {
    c3_u[c3_i13] = c3_b_inData[c3_i13];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static void c3_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_ATM_Torque, const char_T *c3_identifier,
  real_T c3_y[3])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_ATM_Torque), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_ATM_Torque);
}

static void c3_b_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[3])
{
  real_T c3_dv5[3];
  int32_T c3_i14;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv5, 1, 0, 0U, 1, 0U, 1, 3);
  for (c3_i14 = 0; c3_i14 < 3; c3_i14++) {
    c3_y[c3_i14] = c3_dv5[c3_i14];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_ATM_Torque;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[3];
  int32_T c3_i15;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_ATM_Torque = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_ATM_Torque), &c3_thisId,
                        c3_y);
  sf_mex_destroy(&c3_ATM_Torque);
  for (c3_i15 = 0; c3_i15 < 3; c3_i15++) {
    (*(real_T (*)[3])c3_outData)[c3_i15] = c3_y[c3_i15];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_c_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_Control_System_Library_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo;
  c3_ResolvedFunctionInfo c3_info[22];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i16;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 22), FALSE);
  for (c3_i16 = 0; c3_i16 < 22; c3_i16++) {
    c3_r0 = &c3_info[c3_i16];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context", "nameCaptureInfo",
                    c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name", "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved", "nameCaptureInfo",
                    c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c3_i16);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c3_i16);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[22])
{
  c3_info[0].context = "";
  c3_info[0].name = "mpower";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[0].fileTimeLo = 1286840442U;
  c3_info[0].fileTimeHi = 0U;
  c3_info[0].mFileTimeLo = 0U;
  c3_info[0].mFileTimeHi = 0U;
  c3_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c3_info[1].name = "power";
  c3_info[1].dominantType = "double";
  c3_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[1].fileTimeLo = 1336543696U;
  c3_info[1].fileTimeHi = 0U;
  c3_info[1].mFileTimeLo = 0U;
  c3_info[1].mFileTimeHi = 0U;
  c3_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[2].name = "eml_scalar_eg";
  c3_info[2].dominantType = "double";
  c3_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[2].fileTimeLo = 1286840396U;
  c3_info[2].fileTimeHi = 0U;
  c3_info[2].mFileTimeLo = 0U;
  c3_info[2].mFileTimeHi = 0U;
  c3_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[3].name = "eml_scalexp_alloc";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[3].fileTimeLo = 1330630034U;
  c3_info[3].fileTimeHi = 0U;
  c3_info[3].mFileTimeLo = 0U;
  c3_info[3].mFileTimeHi = 0U;
  c3_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c3_info[4].name = "floor";
  c3_info[4].dominantType = "double";
  c3_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c3_info[4].fileTimeLo = 1286840342U;
  c3_info[4].fileTimeHi = 0U;
  c3_info[4].mFileTimeLo = 0U;
  c3_info[4].mFileTimeHi = 0U;
  c3_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c3_info[5].name = "eml_scalar_floor";
  c3_info[5].dominantType = "double";
  c3_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c3_info[5].fileTimeLo = 1286840326U;
  c3_info[5].fileTimeHi = 0U;
  c3_info[5].mFileTimeLo = 0U;
  c3_info[5].mFileTimeHi = 0U;
  c3_info[6].context = "";
  c3_info[6].name = "mtimes";
  c3_info[6].dominantType = "double";
  c3_info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[6].fileTimeLo = 1289541292U;
  c3_info[6].fileTimeHi = 0U;
  c3_info[6].mFileTimeLo = 0U;
  c3_info[6].mFileTimeHi = 0U;
  c3_info[7].context = "";
  c3_info[7].name = "power";
  c3_info[7].dominantType = "double";
  c3_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c3_info[7].fileTimeLo = 1336543696U;
  c3_info[7].fileTimeHi = 0U;
  c3_info[7].mFileTimeLo = 0U;
  c3_info[7].mFileTimeHi = 0U;
  c3_info[8].context = "";
  c3_info[8].name = "sum";
  c3_info[8].dominantType = "double";
  c3_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[8].fileTimeLo = 1314758212U;
  c3_info[8].fileTimeHi = 0U;
  c3_info[8].mFileTimeLo = 0U;
  c3_info[8].mFileTimeHi = 0U;
  c3_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[9].name = "isequal";
  c3_info[9].dominantType = "double";
  c3_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c3_info[9].fileTimeLo = 1286840358U;
  c3_info[9].fileTimeHi = 0U;
  c3_info[9].mFileTimeLo = 0U;
  c3_info[9].mFileTimeHi = 0U;
  c3_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c3_info[10].name = "eml_isequal_core";
  c3_info[10].dominantType = "double";
  c3_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c3_info[10].fileTimeLo = 1286840386U;
  c3_info[10].fileTimeHi = 0U;
  c3_info[10].mFileTimeLo = 0U;
  c3_info[10].mFileTimeHi = 0U;
  c3_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[11].name = "eml_const_nonsingleton_dim";
  c3_info[11].dominantType = "double";
  c3_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c3_info[11].fileTimeLo = 1286840296U;
  c3_info[11].fileTimeHi = 0U;
  c3_info[11].mFileTimeLo = 0U;
  c3_info[11].mFileTimeHi = 0U;
  c3_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[12].name = "eml_scalar_eg";
  c3_info[12].dominantType = "double";
  c3_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[12].fileTimeLo = 1286840396U;
  c3_info[12].fileTimeHi = 0U;
  c3_info[12].mFileTimeLo = 0U;
  c3_info[12].mFileTimeHi = 0U;
  c3_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[13].name = "eml_index_class";
  c3_info[13].dominantType = "";
  c3_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[13].fileTimeLo = 1323192178U;
  c3_info[13].fileTimeHi = 0U;
  c3_info[13].mFileTimeLo = 0U;
  c3_info[13].mFileTimeHi = 0U;
  c3_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c3_info[14].name = "eml_int_forloop_overflow_check";
  c3_info[14].dominantType = "";
  c3_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c3_info[14].fileTimeLo = 1332186672U;
  c3_info[14].fileTimeHi = 0U;
  c3_info[14].mFileTimeLo = 0U;
  c3_info[14].mFileTimeHi = 0U;
  c3_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c3_info[15].name = "intmax";
  c3_info[15].dominantType = "char";
  c3_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c3_info[15].fileTimeLo = 1311276916U;
  c3_info[15].fileTimeHi = 0U;
  c3_info[15].mFileTimeLo = 0U;
  c3_info[15].mFileTimeHi = 0U;
  c3_info[16].context = "";
  c3_info[16].name = "sqrt";
  c3_info[16].dominantType = "double";
  c3_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[16].fileTimeLo = 1286840352U;
  c3_info[16].fileTimeHi = 0U;
  c3_info[16].mFileTimeLo = 0U;
  c3_info[16].mFileTimeHi = 0U;
  c3_info[17].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[17].name = "eml_error";
  c3_info[17].dominantType = "char";
  c3_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[17].fileTimeLo = 1305339600U;
  c3_info[17].fileTimeHi = 0U;
  c3_info[17].mFileTimeLo = 0U;
  c3_info[17].mFileTimeHi = 0U;
  c3_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[18].name = "eml_scalar_sqrt";
  c3_info[18].dominantType = "double";
  c3_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c3_info[18].fileTimeLo = 1286840338U;
  c3_info[18].fileTimeHi = 0U;
  c3_info[18].mFileTimeLo = 0U;
  c3_info[18].mFileTimeHi = 0U;
  c3_info[19].context = "";
  c3_info[19].name = "mrdivide";
  c3_info[19].dominantType = "double";
  c3_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[19].fileTimeLo = 1342832544U;
  c3_info[19].fileTimeHi = 0U;
  c3_info[19].mFileTimeLo = 1319751566U;
  c3_info[19].mFileTimeHi = 0U;
  c3_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[20].name = "rdivide";
  c3_info[20].dominantType = "double";
  c3_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[20].fileTimeLo = 1286840444U;
  c3_info[20].fileTimeHi = 0U;
  c3_info[20].mFileTimeLo = 0U;
  c3_info[20].mFileTimeHi = 0U;
  c3_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[21].name = "eml_div";
  c3_info[21].dominantType = "double";
  c3_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[21].fileTimeLo = 1313369410U;
  c3_info[21].fileTimeHi = 0U;
  c3_info[21].mFileTimeLo = 0U;
  c3_info[21].mFileTimeHi = 0U;
}

static void c3_eml_scalar_eg(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c3_check_forloop_overflow_error
  (SFc3_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void c3_eml_error(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance)
{
  int32_T c3_i17;
  static char_T c3_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  for (c3_i17 = 0; c3_i17 < 30; c3_i17++) {
    c3_u[c3_i17] = c3_varargin_1[c3_i17];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c3_y));
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_d_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i18;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i18, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i18;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_Control_System_Library, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_Control_System_Library), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_Control_System_Library);
  return c3_y;
}

static uint8_T c3_f_emlrt_marshallIn(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void init_dsm_address_info(SFc3_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c3_Control_System_Library_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2286077517U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1506408336U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(112518601U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1918494290U);
}

mxArray *sf_c3_Control_System_Library_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("agz7MMBn5pkgrd5QVWCnYD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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

static const mxArray *sf_get_sim_state_info_c3_Control_System_Library(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"ATM_Torque\",},{M[8],M[0],T\"is_active_c3_Control_System_Library\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_Control_System_Library_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_Control_System_LibraryInstanceStruct *chartInstance;
    chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_Control_System_LibraryMachineNumber_,
           3,
           1,
           1,
           2,
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
          init_script_number_translation(_Control_System_LibraryMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_Control_System_LibraryMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds
            (_Control_System_LibraryMachineNumber_,
             chartInstance->chartNumber,
             0,
             0,
             0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"ATM_Torque");
          _SFD_SET_DATA_PROPS(1,1,1,0,"R");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,892);
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
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c3_ATM_Torque)[3];
          real_T (*c3_R)[3];
          c3_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          c3_ATM_Torque = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S,
            1);
          _SFD_SET_DATA_VALUE_PTR(0U, *c3_ATM_Torque);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_R);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_Control_System_LibraryMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "GjeS24AuOCHqnYuKNfs86C";
}

static void sf_opaque_initialize_c3_Control_System_Library(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_Control_System_LibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_Control_System_Library
    ((SFc3_Control_System_LibraryInstanceStruct*) chartInstanceVar);
  initialize_c3_Control_System_Library
    ((SFc3_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_Control_System_Library(void *chartInstanceVar)
{
  enable_c3_Control_System_Library((SFc3_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_Control_System_Library(void *chartInstanceVar)
{
  disable_c3_Control_System_Library((SFc3_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_Control_System_Library(void *chartInstanceVar)
{
  sf_c3_Control_System_Library((SFc3_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_Control_System_Library
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_Control_System_Library
    ((SFc3_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_Control_System_Library();/* state var info */
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

extern void sf_internal_set_sim_state_c3_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_Control_System_Library();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_Control_System_Library
    ((SFc3_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_Control_System_Library
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c3_Control_System_Library(S);
}

static void sf_opaque_set_sim_state_c3_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c3_Control_System_Library(S, st);
}

static void sf_opaque_terminate_c3_Control_System_Library(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_Control_System_LibraryInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c3_Control_System_Library
      ((SFc3_Control_System_LibraryInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Control_System_Library_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_Control_System_Library
    ((SFc3_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_Control_System_Library(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_Control_System_Library
      ((SFc3_Control_System_LibraryInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Control_System_Library_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3100102515U));
  ssSetChecksum1(S,(2528720445U));
  ssSetChecksum2(S,(483482874U));
  ssSetChecksum3(S,(2011606972U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_Control_System_Library(SimStruct *S)
{
  SFc3_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc3_Control_System_LibraryInstanceStruct *)malloc(sizeof
    (SFc3_Control_System_LibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_Control_System_LibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_Control_System_Library;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_Control_System_Library;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_Control_System_Library;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c3_Control_System_Library;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_Control_System_Library;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_Control_System_Library;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_Control_System_Library;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_Control_System_Library;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_Control_System_Library;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_Control_System_Library;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_Control_System_Library;
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

void c3_Control_System_Library_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_Control_System_Library(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_Control_System_Library(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_Control_System_Library(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_Control_System_Library_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
