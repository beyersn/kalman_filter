/* Include files */

#include "blascompat32.h"
#include "Control_System_Library_sfun.h"
#include "c5_Control_System_Library.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Control_System_Library_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c5_debug_family_names[10] = { "muC", "R_avg_earth", "Ixx",
  "Iyy", "Izz", "R_sat", "nargin", "nargout", "R", "G_grav" };

/* Function Declarations */
static void initialize_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void initialize_params_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void enable_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void disable_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void set_sim_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c5_st);
static void finalize_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void sf_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void c5_chartstep_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void initSimStructsc5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static void c5_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_G_grav, const char_T *c5_identifier, real_T
  c5_y[3]);
static void c5_b_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3]);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_c_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[19]);
static void c5_check_forloop_overflow_error
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_d_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_e_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_Control_System_Library, const
  char_T *c5_identifier);
static uint8_T c5_f_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_Control_System_Library = 0U;
}

static void initialize_params_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void enable_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  int32_T c5_i0;
  real_T c5_u[3];
  const mxArray *c5_b_y = NULL;
  uint8_T c5_hoistedGlobal;
  uint8_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T (*c5_G_grav)[3];
  c5_G_grav = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(2), FALSE);
  for (c5_i0 = 0; c5_i0 < 3; c5_i0++) {
    c5_u[c5_i0] = (*c5_G_grav)[c5_i0];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_hoistedGlobal = chartInstance->c5_is_active_c5_Control_System_Library;
  c5_b_u = c5_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  sf_mex_assign(&c5_st, c5_y, FALSE);
  return c5_st;
}

static void set_sim_state_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c5_st)
{
  const mxArray *c5_u;
  real_T c5_dv0[3];
  int32_T c5_i1;
  real_T (*c5_G_grav)[3];
  c5_G_grav = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
                      "G_grav", c5_dv0);
  for (c5_i1 = 0; c5_i1 < 3; c5_i1++) {
    (*c5_G_grav)[c5_i1] = c5_dv0[c5_i1];
  }

  chartInstance->c5_is_active_c5_Control_System_Library = c5_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
     "is_active_c5_Control_System_Library");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_Control_System_Library(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void sf_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c5_i2;
  int32_T c5_i3;
  real_T (*c5_G_grav)[3];
  real_T (*c5_R)[3];
  c5_G_grav = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  for (c5_i2 = 0; c5_i2 < 3; c5_i2++) {
    _SFD_DATA_RANGE_CHECK((*c5_R)[c5_i2], 0U);
  }

  for (c5_i3 = 0; c5_i3 < 3; c5_i3++) {
    _SFD_DATA_RANGE_CHECK((*c5_G_grav)[c5_i3], 1U);
  }

  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_chartstep_c5_Control_System_Library(chartInstance);
  sf_debug_check_for_state_inconsistency(_Control_System_LibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c5_chartstep_c5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c5_i4;
  real_T c5_R[3];
  uint32_T c5_debug_family_var_map[10];
  real_T c5_muC;
  real_T c5_R_avg_earth;
  real_T c5_Ixx;
  real_T c5_Iyy;
  real_T c5_Izz;
  real_T c5_R_sat[3];
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 1.0;
  real_T c5_G_grav[3];
  int32_T c5_i5;
  int32_T c5_i6;
  real_T c5_a[3];
  int32_T c5_k;
  real_T c5_b_k;
  real_T c5_ak;
  real_T c5_y[3];
  real_T c5_b_y;
  int32_T c5_c_k;
  int32_T c5_d_k;
  real_T c5_B;
  real_T c5_c_y;
  real_T c5_d_y;
  real_T c5_e_y;
  real_T c5_b;
  real_T c5_f_y;
  real_T c5_b_a;
  real_T c5_b_b;
  real_T c5_g_y;
  real_T c5_c_b;
  real_T c5_h_y;
  real_T c5_c_a;
  real_T c5_d_b;
  real_T c5_i_y;
  real_T c5_e_b;
  real_T c5_j_y;
  real_T c5_d_a;
  real_T c5_f_b;
  real_T c5_k_y;
  real_T c5_e_a;
  int32_T c5_i7;
  int32_T c5_i8;
  real_T (*c5_b_G_grav)[3];
  real_T (*c5_b_R)[3];
  c5_b_G_grav = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  for (c5_i4 = 0; c5_i4 < 3; c5_i4++) {
    c5_R[c5_i4] = (*c5_b_R)[c5_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 10U, 10U, c5_debug_family_names,
    c5_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c5_muC, 0U, c5_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c5_R_avg_earth, 1U,
    c5_b_sf_marshallOut, c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c5_Ixx, 2U, c5_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_Iyy, 3U, c5_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c5_Izz, 4U, c5_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c5_R_sat, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_nargin, 6U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c5_nargout, 7U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c5_R, 8U, c5_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c5_G_grav, 9U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 11);
  c5_muC = 3.986004418E+14;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
  c5_R_avg_earth = 6.731E+6;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 13);
  c5_Ixx = 4.3;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 14);
  c5_Iyy = 4.3;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 15);
  c5_Izz = 1.9;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 17);
  for (c5_i5 = 0; c5_i5 < 3; c5_i5++) {
    c5_R_sat[c5_i5] = c5_R_avg_earth + c5_R[c5_i5];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 20);
  for (c5_i6 = 0; c5_i6 < 3; c5_i6++) {
    c5_a[c5_i6] = c5_R_sat[c5_i6];
  }

  for (c5_k = 0; c5_k < 3; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    c5_ak = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 3, 1, 0) - 1];
    c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c5_b_k),
      1, 3, 1, 0) - 1] = muDoubleScalarPower(c5_ak, 5.0);
  }

  c5_b_y = c5_y[0];
  c5_check_forloop_overflow_error(chartInstance);
  for (c5_c_k = 2; c5_c_k < 4; c5_c_k++) {
    c5_d_k = c5_c_k;
    c5_b_y += c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c5_d_k), 1, 3, 1, 0) - 1];
  }

  c5_B = c5_b_y;
  c5_c_y = c5_B;
  c5_d_y = c5_c_y;
  c5_e_y = 1.1958013254E+15 / c5_d_y;
  c5_b = c5_R_sat[1];
  c5_f_y = -2.4 * c5_b;
  c5_b_a = c5_f_y;
  c5_b_b = c5_R_sat[2];
  c5_g_y = c5_b_a * c5_b_b;
  c5_c_b = c5_R_sat[2];
  c5_h_y = -2.4 * c5_c_b;
  c5_c_a = c5_h_y;
  c5_d_b = c5_R_sat[0];
  c5_i_y = c5_c_a * c5_d_b;
  c5_e_b = c5_R_sat[0];
  c5_j_y = 0.0 * c5_e_b;
  c5_d_a = c5_j_y;
  c5_f_b = c5_R_sat[2];
  c5_k_y = c5_d_a * c5_f_b;
  c5_e_a = c5_e_y;
  c5_a[0] = c5_g_y;
  c5_a[1] = c5_i_y;
  c5_a[2] = c5_k_y;
  for (c5_i7 = 0; c5_i7 < 3; c5_i7++) {
    c5_G_grav[c5_i7] = c5_e_a * c5_a[c5_i7];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -20);
  sf_debug_symbol_scope_pop();
  for (c5_i8 = 0; c5_i8 < 3; c5_i8++) {
    (*c5_b_G_grav)[c5_i8] = c5_G_grav[c5_i8];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
}

static void initSimStructsc5_Control_System_Library
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i9;
  real_T c5_b_inData[3];
  int32_T c5_i10;
  real_T c5_u[3];
  const mxArray *c5_y = NULL;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i9 = 0; c5_i9 < 3; c5_i9++) {
    c5_b_inData[c5_i9] = (*(real_T (*)[3])c5_inData)[c5_i9];
  }

  for (c5_i10 = 0; c5_i10 < 3; c5_i10++) {
    c5_u[c5_i10] = c5_b_inData[c5_i10];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_G_grav, const char_T *c5_identifier, real_T
  c5_y[3])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_G_grav), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_G_grav);
}

static void c5_b_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3])
{
  real_T c5_dv1[3];
  int32_T c5_i11;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c5_i11 = 0; c5_i11 < 3; c5_i11++) {
    c5_y[c5_i11] = c5_dv1[c5_i11];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_G_grav;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[3];
  int32_T c5_i12;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_G_grav = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_G_grav), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_G_grav);
  for (c5_i12 = 0; c5_i12 < 3; c5_i12++) {
    (*(real_T (*)[3])c5_outData)[c5_i12] = c5_y[c5_i12];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static real_T c5_c_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_Control_System_Library_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo;
  c5_ResolvedFunctionInfo c5_info[19];
  const mxArray *c5_m0 = NULL;
  int32_T c5_i13;
  c5_ResolvedFunctionInfo *c5_r0;
  c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  c5_info_helper(c5_info);
  sf_mex_assign(&c5_m0, sf_mex_createstruct("nameCaptureInfo", 1, 19), FALSE);
  for (c5_i13 = 0; c5_i13 < 19; c5_i13++) {
    c5_r0 = &c5_info[c5_i13];
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->context)), "context", "nameCaptureInfo",
                    c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c5_r0->name)), "name", "nameCaptureInfo", c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c5_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", c5_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c5_r0->resolved)), "resolved", "nameCaptureInfo",
                    c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c5_i13);
    sf_mex_addfield(c5_m0, sf_mex_create("nameCaptureInfo", &c5_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c5_i13);
  }

  sf_mex_assign(&c5_nameCaptureInfo, c5_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(c5_ResolvedFunctionInfo c5_info[19])
{
  c5_info[0].context = "";
  c5_info[0].name = "mpower";
  c5_info[0].dominantType = "double";
  c5_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c5_info[0].fileTimeLo = 1286840442U;
  c5_info[0].fileTimeHi = 0U;
  c5_info[0].mFileTimeLo = 0U;
  c5_info[0].mFileTimeHi = 0U;
  c5_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c5_info[1].name = "power";
  c5_info[1].dominantType = "double";
  c5_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[1].fileTimeLo = 1336543696U;
  c5_info[1].fileTimeHi = 0U;
  c5_info[1].mFileTimeLo = 0U;
  c5_info[1].mFileTimeHi = 0U;
  c5_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c5_info[2].name = "eml_scalar_eg";
  c5_info[2].dominantType = "double";
  c5_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[2].fileTimeLo = 1286840396U;
  c5_info[2].fileTimeHi = 0U;
  c5_info[2].mFileTimeLo = 0U;
  c5_info[2].mFileTimeHi = 0U;
  c5_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c5_info[3].name = "eml_scalexp_alloc";
  c5_info[3].dominantType = "double";
  c5_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c5_info[3].fileTimeLo = 1330630034U;
  c5_info[3].fileTimeHi = 0U;
  c5_info[3].mFileTimeLo = 0U;
  c5_info[3].mFileTimeHi = 0U;
  c5_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c5_info[4].name = "floor";
  c5_info[4].dominantType = "double";
  c5_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c5_info[4].fileTimeLo = 1286840342U;
  c5_info[4].fileTimeHi = 0U;
  c5_info[4].mFileTimeLo = 0U;
  c5_info[4].mFileTimeHi = 0U;
  c5_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c5_info[5].name = "eml_scalar_floor";
  c5_info[5].dominantType = "double";
  c5_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c5_info[5].fileTimeLo = 1286840326U;
  c5_info[5].fileTimeHi = 0U;
  c5_info[5].mFileTimeLo = 0U;
  c5_info[5].mFileTimeHi = 0U;
  c5_info[6].context = "";
  c5_info[6].name = "mtimes";
  c5_info[6].dominantType = "double";
  c5_info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c5_info[6].fileTimeLo = 1289541292U;
  c5_info[6].fileTimeHi = 0U;
  c5_info[6].mFileTimeLo = 0U;
  c5_info[6].mFileTimeHi = 0U;
  c5_info[7].context = "";
  c5_info[7].name = "power";
  c5_info[7].dominantType = "double";
  c5_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c5_info[7].fileTimeLo = 1336543696U;
  c5_info[7].fileTimeHi = 0U;
  c5_info[7].mFileTimeLo = 0U;
  c5_info[7].mFileTimeHi = 0U;
  c5_info[8].context = "";
  c5_info[8].name = "sum";
  c5_info[8].dominantType = "double";
  c5_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[8].fileTimeLo = 1314758212U;
  c5_info[8].fileTimeHi = 0U;
  c5_info[8].mFileTimeLo = 0U;
  c5_info[8].mFileTimeHi = 0U;
  c5_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[9].name = "isequal";
  c5_info[9].dominantType = "double";
  c5_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c5_info[9].fileTimeLo = 1286840358U;
  c5_info[9].fileTimeHi = 0U;
  c5_info[9].mFileTimeLo = 0U;
  c5_info[9].mFileTimeHi = 0U;
  c5_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c5_info[10].name = "eml_isequal_core";
  c5_info[10].dominantType = "double";
  c5_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c5_info[10].fileTimeLo = 1286840386U;
  c5_info[10].fileTimeHi = 0U;
  c5_info[10].mFileTimeLo = 0U;
  c5_info[10].mFileTimeHi = 0U;
  c5_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[11].name = "eml_const_nonsingleton_dim";
  c5_info[11].dominantType = "double";
  c5_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c5_info[11].fileTimeLo = 1286840296U;
  c5_info[11].fileTimeHi = 0U;
  c5_info[11].mFileTimeLo = 0U;
  c5_info[11].mFileTimeHi = 0U;
  c5_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[12].name = "eml_scalar_eg";
  c5_info[12].dominantType = "double";
  c5_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c5_info[12].fileTimeLo = 1286840396U;
  c5_info[12].fileTimeHi = 0U;
  c5_info[12].mFileTimeLo = 0U;
  c5_info[12].mFileTimeHi = 0U;
  c5_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[13].name = "eml_index_class";
  c5_info[13].dominantType = "";
  c5_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c5_info[13].fileTimeLo = 1323192178U;
  c5_info[13].fileTimeHi = 0U;
  c5_info[13].mFileTimeLo = 0U;
  c5_info[13].mFileTimeHi = 0U;
  c5_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c5_info[14].name = "eml_int_forloop_overflow_check";
  c5_info[14].dominantType = "";
  c5_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c5_info[14].fileTimeLo = 1332186672U;
  c5_info[14].fileTimeHi = 0U;
  c5_info[14].mFileTimeLo = 0U;
  c5_info[14].mFileTimeHi = 0U;
  c5_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c5_info[15].name = "intmax";
  c5_info[15].dominantType = "char";
  c5_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c5_info[15].fileTimeLo = 1311276916U;
  c5_info[15].fileTimeHi = 0U;
  c5_info[15].mFileTimeLo = 0U;
  c5_info[15].mFileTimeHi = 0U;
  c5_info[16].context = "";
  c5_info[16].name = "mrdivide";
  c5_info[16].dominantType = "double";
  c5_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c5_info[16].fileTimeLo = 1342832544U;
  c5_info[16].fileTimeHi = 0U;
  c5_info[16].mFileTimeLo = 1319751566U;
  c5_info[16].mFileTimeHi = 0U;
  c5_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c5_info[17].name = "rdivide";
  c5_info[17].dominantType = "double";
  c5_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c5_info[17].fileTimeLo = 1286840444U;
  c5_info[17].fileTimeHi = 0U;
  c5_info[17].mFileTimeLo = 0U;
  c5_info[17].mFileTimeHi = 0U;
  c5_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c5_info[18].name = "eml_div";
  c5_info[18].dominantType = "double";
  c5_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c5_info[18].fileTimeLo = 1313369410U;
  c5_info[18].fileTimeHi = 0U;
  c5_info[18].mFileTimeLo = 0U;
  c5_info[18].mFileTimeHi = 0U;
}

static void c5_check_forloop_overflow_error
  (SFc5_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static int32_T c5_d_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i14;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i14, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i14;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_e_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_Control_System_Library, const
  char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_Control_System_Library), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_Control_System_Library);
  return c5_y;
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c5_Control_System_Library_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1328489457U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(972550739U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1583030857U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1089255614U);
}

mxArray *sf_c5_Control_System_Library_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("IU7mIkQiXp7vyeMrx7FvsE");
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

static const mxArray *sf_get_sim_state_info_c5_Control_System_Library(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"G_grav\",},{M[8],M[0],T\"is_active_c5_Control_System_Library\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_Control_System_Library_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_Control_System_LibraryInstanceStruct *chartInstance;
    chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_Control_System_LibraryMachineNumber_,
           5,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"R");
          _SFD_SET_DATA_PROPS(1,2,0,1,"G_grav");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,546);
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
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)
            c5_sf_marshallIn);
        }

        {
          real_T (*c5_R)[3];
          real_T (*c5_G_grav)[3];
          c5_G_grav = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
          c5_R = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c5_R);
          _SFD_SET_DATA_VALUE_PTR(1U, *c5_G_grav);
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
  return "AadCheIOECr4NVYgcvLMcF";
}

static void sf_opaque_initialize_c5_Control_System_Library(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_Control_System_LibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_Control_System_Library
    ((SFc5_Control_System_LibraryInstanceStruct*) chartInstanceVar);
  initialize_c5_Control_System_Library
    ((SFc5_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c5_Control_System_Library(void *chartInstanceVar)
{
  enable_c5_Control_System_Library((SFc5_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_Control_System_Library(void *chartInstanceVar)
{
  disable_c5_Control_System_Library((SFc5_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_Control_System_Library(void *chartInstanceVar)
{
  sf_c5_Control_System_Library((SFc5_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_Control_System_Library
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_Control_System_Library
    ((SFc5_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_Control_System_Library();/* state var info */
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

extern void sf_internal_set_sim_state_c5_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_Control_System_Library();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_Control_System_Library
    ((SFc5_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_Control_System_Library
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c5_Control_System_Library(S);
}

static void sf_opaque_set_sim_state_c5_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c5_Control_System_Library(S, st);
}

static void sf_opaque_terminate_c5_Control_System_Library(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_Control_System_LibraryInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c5_Control_System_Library
      ((SFc5_Control_System_LibraryInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Control_System_Library_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_Control_System_Library
    ((SFc5_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_Control_System_Library(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_Control_System_Library
      ((SFc5_Control_System_LibraryInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Control_System_Library_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,5,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1191413570U));
  ssSetChecksum1(S,(3804074168U));
  ssSetChecksum2(S,(1265601594U));
  ssSetChecksum3(S,(3251476340U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_Control_System_Library(SimStruct *S)
{
  SFc5_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc5_Control_System_LibraryInstanceStruct *)malloc(sizeof
    (SFc5_Control_System_LibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_Control_System_LibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_Control_System_Library;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_Control_System_Library;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_Control_System_Library;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_Control_System_Library;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_Control_System_Library;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_Control_System_Library;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_Control_System_Library;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_Control_System_Library;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_Control_System_Library;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_Control_System_Library;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_Control_System_Library;
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

void c5_Control_System_Library_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_Control_System_Library(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_Control_System_Library(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_Control_System_Library(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_Control_System_Library_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
