/* Include files */

#include "blascompat32.h"
#include "Control_System_Library_sfun.h"
#include "c7_Control_System_Library.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Control_System_Library_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
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
static void initialize_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void initialize_params_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void enable_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void disable_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void c7_update_debugger_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void set_sim_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c7_st);
static void finalize_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void sf_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void initSimStructsc7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static void c7_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_q_i_c_km1, const char_T *c7_identifier,
  real_T c7_y[4]);
static void c7_b_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4]);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_w_c_km1, const char_T *c7_identifier,
  real_T c7_y[3]);
static void c7_d_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3]);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_e_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_q_i_s, const char_T *c7_identifier, real_T
  c7_y[4]);
static void c7_f_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4]);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_g_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_w_s, const char_T *c7_identifier, real_T
  c7_y[3]);
static void c7_h_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3]);
static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static real_T c7_i_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_j_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[7]);
static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_k_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4]);
static void c7_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_l_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3]);
static void c7_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_i_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_m_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[9]);
static void c7_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[64]);
static void c7_quatinv(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
  real_T c7_qin[4], real_T c7_qinv[4]);
static void c7_check_forloop_overflow_error
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance);
static void c7_eml_error(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_quatmultiply(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_q[4], real_T c7_r[4], real_T c7_qres[4]);
static void c7_Kinematics(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_x[7], real_T c7_I[3], real_T c7_Torque_c[3], real_T
  c7_results[7]);
static void c7_diag(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                    real_T c7_v[3], real_T c7_d[9]);
static void c7_skew_matrix(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_x[3], real_T c7_output[9]);
static void c7_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_mpower(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_a[9], real_T c7_c[9]);
static void c7_matrix_to_integer_power(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_a[9], real_T c7_b, real_T c7_c[9]);
static void c7_b_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_c_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_inv3x3(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_x[9], real_T c7_y[9]);
static real_T c7_norm(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_x[9]);
static void c7_eml_warning(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_b_eml_warning(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, char_T c7_varargin_2[14]);
static void c7_d_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c7_n_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_sprintf, const char_T *c7_identifier, char_T
  c7_y[14]);
static void c7_o_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  char_T c7_y[14]);
static const mxArray *c7_j_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_p_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_q_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_Control_System_Library, const
  char_T *c7_identifier);
static uint8_T c7_r_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void init_dsm_address_info(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c7_w_c_km1_not_empty = FALSE;
  chartInstance->c7_q_i_c_km1_not_empty = FALSE;
  chartInstance->c7_is_active_c7_Control_System_Library = 0U;
}

static void initialize_params_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void enable_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c7_update_debugger_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
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
  sf_mex_assign(&c7_y, sf_mex_createcellarray(5), FALSE);
  for (c7_i0 = 0; c7_i0 < 4; c7_i0++) {
    c7_u[c7_i0] = (*c7_q_i_s)[c7_i0];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  for (c7_i1 = 0; c7_i1 < 3; c7_i1++) {
    c7_b_u[c7_i1] = (*c7_w_s)[c7_i1];
  }

  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", c7_b_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  for (c7_i2 = 0; c7_i2 < 4; c7_i2++) {
    c7_c_u[c7_i2] = chartInstance->c7_q_i_c_km1[c7_i2];
  }

  c7_d_y = NULL;
  if (!chartInstance->c7_q_i_c_km1_not_empty) {
    sf_mex_assign(&c7_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c7_d_y, sf_mex_create("y", c7_c_u, 0, 0U, 1U, 0U, 1, 4),
                  FALSE);
  }

  sf_mex_setcell(c7_y, 2, c7_d_y);
  for (c7_i3 = 0; c7_i3 < 3; c7_i3++) {
    c7_d_u[c7_i3] = chartInstance->c7_w_c_km1[c7_i3];
  }

  c7_e_y = NULL;
  if (!chartInstance->c7_w_c_km1_not_empty) {
    sf_mex_assign(&c7_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c7_e_y, sf_mex_create("y", c7_d_u, 0, 0U, 1U, 0U, 1, 3),
                  FALSE);
  }

  sf_mex_setcell(c7_y, 3, c7_e_y);
  c7_hoistedGlobal = chartInstance->c7_is_active_c7_Control_System_Library;
  c7_e_u = c7_hoistedGlobal;
  c7_f_y = NULL;
  sf_mex_assign(&c7_f_y, sf_mex_create("y", &c7_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c7_y, 4, c7_f_y);
  sf_mex_assign(&c7_st, c7_y, FALSE);
  return c7_st;
}

static void set_sim_state_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c7_st)
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
  chartInstance->c7_doneDoubleBufferReInit = TRUE;
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

  chartInstance->c7_is_active_c7_Control_System_Library = c7_q_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 4)),
     "is_active_c7_Control_System_Library");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_Control_System_Library(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void sf_c7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c7_i8;
  int32_T c7_i9;
  int32_T c7_i10;
  int32_T c7_i11;
  int32_T c7_i12;
  int32_T c7_i13;
  int32_T c7_i14;
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
  int32_T c7_i80;
  real_T c7_i_q_s_c[4];
  real_T c7_dv21[4];
  int32_T c7_i81;
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
  boolean_T guard1 = FALSE;
  c7_b_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c7_j_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
  c7_c_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_b_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c7_b_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c7_b_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c7_f_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  for (c7_i8 = 0; c7_i8 < 3; c7_i8++) {
    _SFD_DATA_RANGE_CHECK((*c7_f_I_c)[c7_i8], 0U);
  }

  for (c7_i9 = 0; c7_i9 < 3; c7_i9++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_w_init_s)[c7_i9], 1U);
  }

  for (c7_i10 = 0; c7_i10 < 3; c7_i10++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_Torque_s)[c7_i10], 2U);
  }

  for (c7_i11 = 0; c7_i11 < 4; c7_i11++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_q0_i_s)[c7_i11], 3U);
  }

  for (c7_i12 = 0; c7_i12 < 3; c7_i12++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_w_s)[c7_i12], 4U);
  }

  for (c7_i13 = 0; c7_i13 < 4; c7_i13++) {
    _SFD_DATA_RANGE_CHECK((*c7_c_q_i_s)[c7_i13], 5U);
  }

  for (c7_i14 = 0; c7_i14 < 4; c7_i14++) {
    _SFD_DATA_RANGE_CHECK((*c7_j_q_s_c)[c7_i14], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c7_b_Ts, 7U);
  chartInstance->c7_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
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
  sf_debug_symbol_scope_push_eml(0U, 22U, 22U, c7_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c7_Torque_c_temp, 0U,
    c7_c_sf_marshallOut, c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_Torque_c, 1U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_w_c_temp, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_x_km1, 3U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_k1, 4U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_k2, 5U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_k3, 6U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_k4, 7U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_x_k, 8U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_w_s_temp, 9U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 10U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 11U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c7_I_c, 12U, c7_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c7_w_init_s, 13U, c7_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c7_Torque_s, 14U, c7_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c7_q0_i_s, 15U, c7_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c7_q_s_c, 16U, c7_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c7_Ts, 17U, c7_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c7_w_s, 18U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_q_i_s, 19U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c7_w_c_km1, 20U,
    c7_b_sf_marshallOut, c7_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c7_q_i_c_km1, 21U,
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
  guard1 = FALSE;
  if (CV_EML_COND(0, 1, 0, !chartInstance->c7_w_c_km1_not_empty)) {
    guard1 = TRUE;
  } else if (CV_EML_COND(0, 1, 1, !chartInstance->c7_q_i_c_km1_not_empty)) {
    guard1 = TRUE;
  } else {
    CV_EML_MCDC(0, 1, 0, FALSE);
    CV_EML_IF(0, 1, 0, FALSE);
    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 22);
    for (c7_i39 = 0; c7_i39 < 4; c7_i39++) {
      c7_x_km1[c7_i39] = chartInstance->c7_q_i_c_km1[c7_i39];
    }

    for (c7_i40 = 0; c7_i40 < 3; c7_i40++) {
      c7_x_km1[c7_i40 + 4] = chartInstance->c7_w_c_km1[c7_i40];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 25);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 26);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 27);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 28);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 30);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 33);
    for (c7_i73 = 0; c7_i73 < 4; c7_i73++) {
      chartInstance->c7_q_i_c_km1[c7_i73] = c7_x_k[c7_i73];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 34);
    for (c7_i74 = 0; c7_i74 < 3; c7_i74++) {
      chartInstance->c7_w_c_km1[c7_i74] = c7_x_k[c7_i74 + 4];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 37);
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

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 38);
    for (c7_i79 = 0; c7_i79 < 4; c7_i79++) {
      c7_h_q_s_c[c7_i79] = c7_q_s_c[c7_i79];
    }

    c7_quatinv(chartInstance, c7_h_q_s_c, c7_dv4);
    for (c7_i80 = 0; c7_i80 < 4; c7_i80++) {
      c7_i_q_s_c[c7_i80] = c7_q_s_c[c7_i80];
    }

    c7_dv21[0] = 0.0;
    for (c7_i81 = 0; c7_i81 < 3; c7_i81++) {
      c7_dv21[c7_i81 + 1] = chartInstance->c7_w_c_km1[c7_i81];
    }

    c7_quatmultiply(chartInstance, c7_i_q_s_c, c7_dv21, c7_dv22);
    for (c7_i82 = 0; c7_i82 < 4; c7_i82++) {
      c7_dv23[c7_i82] = c7_dv22[c7_i82];
    }

    for (c7_i83 = 0; c7_i83 < 4; c7_i83++) {
      c7_dv24[c7_i83] = c7_dv4[c7_i83];
    }

    c7_quatmultiply(chartInstance, c7_dv23, c7_dv24, c7_dv25);
    for (c7_i84 = 0; c7_i84 < 4; c7_i84++) {
      c7_w_s_temp[c7_i84] = c7_dv25[c7_i84];
    }

    _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 39);
    for (c7_i85 = 0; c7_i85 < 3; c7_i85++) {
      c7_w_s[c7_i85] = c7_w_s_temp[c7_i85 + 1];
    }
  }

  if (guard1 == TRUE) {
    CV_EML_MCDC(0, 1, 0, TRUE);
    CV_EML_IF(0, 1, 0, TRUE);
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

    chartInstance->c7_w_c_km1_not_empty = TRUE;
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

    chartInstance->c7_q_i_c_km1_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -39);
  sf_debug_symbol_scope_pop();
  for (c7_i86 = 0; c7_i86 < 3; c7_i86++) {
    (*c7_b_w_s)[c7_i86] = c7_w_s[c7_i86];
  }

  for (c7_i87 = 0; c7_i87 < 4; c7_i87++) {
    (*c7_c_q_i_s)[c7_i87] = c7_q_i_s[c7_i87];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  sf_debug_check_for_state_inconsistency(_Control_System_LibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc7_Control_System_Library
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber)
{
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i88;
  real_T c7_b_inData[4];
  int32_T c7_i89;
  real_T c7_u[4];
  const mxArray *c7_y = NULL;
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i88 = 0; c7_i88 < 4; c7_i88++) {
    c7_b_inData[c7_i88] = (*(real_T (*)[4])c7_inData)[c7_i88];
  }

  for (c7_i89 = 0; c7_i89 < 4; c7_i89++) {
    c7_u[c7_i89] = c7_b_inData[c7_i89];
  }

  c7_y = NULL;
  if (!chartInstance->c7_q_i_c_km1_not_empty) {
    sf_mex_assign(&c7_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  }

  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_q_i_c_km1, const char_T *c7_identifier,
  real_T c7_y[4])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_q_i_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_q_i_c_km1);
}

static void c7_b_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4])
{
  real_T c7_dv26[4];
  int32_T c7_i90;
  if (mxIsEmpty(c7_u)) {
    chartInstance->c7_q_i_c_km1_not_empty = FALSE;
  } else {
    chartInstance->c7_q_i_c_km1_not_empty = TRUE;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i92 = 0; c7_i92 < 3; c7_i92++) {
    c7_b_inData[c7_i92] = (*(real_T (*)[3])c7_inData)[c7_i92];
  }

  for (c7_i93 = 0; c7_i93 < 3; c7_i93++) {
    c7_u[c7_i93] = c7_b_inData[c7_i93];
  }

  c7_y = NULL;
  if (!chartInstance->c7_w_c_km1_not_empty) {
    sf_mex_assign(&c7_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  }

  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_c_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_w_c_km1, const char_T *c7_identifier,
  real_T c7_y[3])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_w_c_km1), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_b_w_c_km1);
}

static void c7_d_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3])
{
  real_T c7_dv27[3];
  int32_T c7_i94;
  if (mxIsEmpty(c7_u)) {
    chartInstance->c7_w_c_km1_not_empty = FALSE;
  } else {
    chartInstance->c7_w_c_km1_not_empty = TRUE;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i96 = 0; c7_i96 < 4; c7_i96++) {
    c7_b_inData[c7_i96] = (*(real_T (*)[4])c7_inData)[c7_i96];
  }

  for (c7_i97 = 0; c7_i97 < 4; c7_i97++) {
    c7_u[c7_i97] = c7_b_inData[c7_i97];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_e_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_q_i_s, const char_T *c7_identifier, real_T
  c7_y[4])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_q_i_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_q_i_s);
}

static void c7_f_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4])
{
  real_T c7_dv28[4];
  int32_T c7_i98;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i100 = 0; c7_i100 < 3; c7_i100++) {
    c7_b_inData[c7_i100] = (*(real_T (*)[3])c7_inData)[c7_i100];
  }

  for (c7_i101 = 0; c7_i101 < 3; c7_i101++) {
    c7_u[c7_i101] = c7_b_inData[c7_i101];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_g_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_w_s, const char_T *c7_identifier, real_T
  c7_y[3])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_w_s), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_w_s);
}

static void c7_h_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3])
{
  real_T c7_dv29[3];
  int32_T c7_i102;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static real_T c7_i_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i104 = 0; c7_i104 < 7; c7_i104++) {
    c7_b_inData[c7_i104] = (*(real_T (*)[7])c7_inData)[c7_i104];
  }

  for (c7_i105 = 0; c7_i105 < 7; c7_i105++) {
    c7_u[c7_i105] = c7_b_inData[c7_i105];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 7), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_j_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[7])
{
  real_T c7_dv30[7];
  int32_T c7_i106;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i108 = 0; c7_i108 < 4; c7_i108++) {
    c7_b_inData[c7_i108] = (*(real_T (*)[4])c7_inData)[c7_i108];
  }

  for (c7_i109 = 0; c7_i109 < 4; c7_i109++) {
    c7_u[c7_i109] = c7_b_inData[c7_i109];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_k_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[4])
{
  real_T c7_dv31[4];
  int32_T c7_i110;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i112 = 0; c7_i112 < 3; c7_i112++) {
    c7_b_inData[c7_i112] = (*(real_T (*)[3])c7_inData)[c7_i112];
  }

  for (c7_i113 = 0; c7_i113 < 3; c7_i113++) {
    c7_u[c7_i113] = c7_b_inData[c7_i113];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_l_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[3])
{
  real_T c7_dv32[3];
  int32_T c7_i114;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_m_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[9])
{
  real_T c7_dv33[9];
  int32_T c7_i122;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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

const mxArray *sf_c7_Control_System_Library_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo;
  c7_ResolvedFunctionInfo c7_info[64];
  const mxArray *c7_m0 = NULL;
  int32_T c7_i126;
  c7_ResolvedFunctionInfo *c7_r0;
  c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  c7_info_helper(c7_info);
  sf_mex_assign(&c7_m0, sf_mex_createstruct("nameCaptureInfo", 1, 64), FALSE);
  for (c7_i126 = 0; c7_i126 < 64; c7_i126++) {
    c7_r0 = &c7_info[c7_i126];
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->context)), "context", "nameCaptureInfo",
                    c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c7_r0->name)), "name", "nameCaptureInfo", c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c7_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->resolved)), "resolved", "nameCaptureInfo",
                    c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c7_i126);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c7_i126);
  }

  sf_mex_assign(&c7_nameCaptureInfo, c7_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(c7_ResolvedFunctionInfo c7_info[64])
{
  c7_info[0].context = "";
  c7_info[0].name = "power";
  c7_info[0].dominantType = "double";
  c7_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c7_info[0].fileTimeLo = 1336543696U;
  c7_info[0].fileTimeHi = 0U;
  c7_info[0].mFileTimeLo = 0U;
  c7_info[0].mFileTimeHi = 0U;
  c7_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c7_info[1].name = "eml_scalar_eg";
  c7_info[1].dominantType = "double";
  c7_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[1].fileTimeLo = 1286840396U;
  c7_info[1].fileTimeHi = 0U;
  c7_info[1].mFileTimeLo = 0U;
  c7_info[1].mFileTimeHi = 0U;
  c7_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c7_info[2].name = "eml_scalexp_alloc";
  c7_info[2].dominantType = "double";
  c7_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c7_info[2].fileTimeLo = 1330630034U;
  c7_info[2].fileTimeHi = 0U;
  c7_info[2].mFileTimeLo = 0U;
  c7_info[2].mFileTimeHi = 0U;
  c7_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c7_info[3].name = "floor";
  c7_info[3].dominantType = "double";
  c7_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[3].fileTimeLo = 1286840342U;
  c7_info[3].fileTimeHi = 0U;
  c7_info[3].mFileTimeLo = 0U;
  c7_info[3].mFileTimeHi = 0U;
  c7_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c7_info[4].name = "eml_scalar_floor";
  c7_info[4].dominantType = "double";
  c7_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c7_info[4].fileTimeLo = 1286840326U;
  c7_info[4].fileTimeHi = 0U;
  c7_info[4].mFileTimeLo = 0U;
  c7_info[4].mFileTimeHi = 0U;
  c7_info[5].context = "";
  c7_info[5].name = "sum";
  c7_info[5].dominantType = "double";
  c7_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[5].fileTimeLo = 1314758212U;
  c7_info[5].fileTimeHi = 0U;
  c7_info[5].mFileTimeLo = 0U;
  c7_info[5].mFileTimeHi = 0U;
  c7_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[6].name = "isequal";
  c7_info[6].dominantType = "double";
  c7_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c7_info[6].fileTimeLo = 1286840358U;
  c7_info[6].fileTimeHi = 0U;
  c7_info[6].mFileTimeLo = 0U;
  c7_info[6].mFileTimeHi = 0U;
  c7_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c7_info[7].name = "eml_isequal_core";
  c7_info[7].dominantType = "double";
  c7_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c7_info[7].fileTimeLo = 1286840386U;
  c7_info[7].fileTimeHi = 0U;
  c7_info[7].mFileTimeLo = 0U;
  c7_info[7].mFileTimeHi = 0U;
  c7_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[8].name = "eml_const_nonsingleton_dim";
  c7_info[8].dominantType = "double";
  c7_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c7_info[8].fileTimeLo = 1286840296U;
  c7_info[8].fileTimeHi = 0U;
  c7_info[8].mFileTimeLo = 0U;
  c7_info[8].mFileTimeHi = 0U;
  c7_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[9].name = "eml_scalar_eg";
  c7_info[9].dominantType = "double";
  c7_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[9].fileTimeLo = 1286840396U;
  c7_info[9].fileTimeHi = 0U;
  c7_info[9].mFileTimeLo = 0U;
  c7_info[9].mFileTimeHi = 0U;
  c7_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[10].name = "eml_index_class";
  c7_info[10].dominantType = "";
  c7_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[10].fileTimeLo = 1323192178U;
  c7_info[10].fileTimeHi = 0U;
  c7_info[10].mFileTimeLo = 0U;
  c7_info[10].mFileTimeHi = 0U;
  c7_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c7_info[11].name = "eml_int_forloop_overflow_check";
  c7_info[11].dominantType = "";
  c7_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[11].fileTimeLo = 1332186672U;
  c7_info[11].fileTimeHi = 0U;
  c7_info[11].mFileTimeLo = 0U;
  c7_info[11].mFileTimeHi = 0U;
  c7_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c7_info[12].name = "intmax";
  c7_info[12].dominantType = "char";
  c7_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c7_info[12].fileTimeLo = 1311276916U;
  c7_info[12].fileTimeHi = 0U;
  c7_info[12].mFileTimeLo = 0U;
  c7_info[12].mFileTimeHi = 0U;
  c7_info[13].context = "";
  c7_info[13].name = "sqrt";
  c7_info[13].dominantType = "double";
  c7_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c7_info[13].fileTimeLo = 1286840352U;
  c7_info[13].fileTimeHi = 0U;
  c7_info[13].mFileTimeLo = 0U;
  c7_info[13].mFileTimeHi = 0U;
  c7_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c7_info[14].name = "eml_error";
  c7_info[14].dominantType = "char";
  c7_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c7_info[14].fileTimeLo = 1305339600U;
  c7_info[14].fileTimeHi = 0U;
  c7_info[14].mFileTimeLo = 0U;
  c7_info[14].mFileTimeHi = 0U;
  c7_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c7_info[15].name = "eml_scalar_sqrt";
  c7_info[15].dominantType = "double";
  c7_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c7_info[15].fileTimeLo = 1286840338U;
  c7_info[15].fileTimeHi = 0U;
  c7_info[15].mFileTimeLo = 0U;
  c7_info[15].mFileTimeHi = 0U;
  c7_info[16].context = "";
  c7_info[16].name = "rdivide";
  c7_info[16].dominantType = "double";
  c7_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[16].fileTimeLo = 1286840444U;
  c7_info[16].fileTimeHi = 0U;
  c7_info[16].mFileTimeLo = 0U;
  c7_info[16].mFileTimeHi = 0U;
  c7_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[17].name = "eml_div";
  c7_info[17].dominantType = "double";
  c7_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c7_info[17].fileTimeLo = 1313369410U;
  c7_info[17].fileTimeHi = 0U;
  c7_info[17].mFileTimeLo = 0U;
  c7_info[17].mFileTimeHi = 0U;
  c7_info[18].context = "";
  c7_info[18].name = "diag";
  c7_info[18].dominantType = "double";
  c7_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c7_info[18].fileTimeLo = 1286840286U;
  c7_info[18].fileTimeHi = 0U;
  c7_info[18].mFileTimeLo = 0U;
  c7_info[18].mFileTimeHi = 0U;
  c7_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c7_info[19].name = "eml_index_class";
  c7_info[19].dominantType = "";
  c7_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[19].fileTimeLo = 1323192178U;
  c7_info[19].fileTimeHi = 0U;
  c7_info[19].mFileTimeLo = 0U;
  c7_info[19].mFileTimeHi = 0U;
  c7_info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c7_info[20].name = "eml_index_plus";
  c7_info[20].dominantType = "coder.internal.indexInt";
  c7_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[20].fileTimeLo = 1286840378U;
  c7_info[20].fileTimeHi = 0U;
  c7_info[20].mFileTimeLo = 0U;
  c7_info[20].mFileTimeHi = 0U;
  c7_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[21].name = "eml_index_class";
  c7_info[21].dominantType = "";
  c7_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[21].fileTimeLo = 1323192178U;
  c7_info[21].fileTimeHi = 0U;
  c7_info[21].mFileTimeLo = 0U;
  c7_info[21].mFileTimeHi = 0U;
  c7_info[22].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c7_info[22].name = "eml_scalar_eg";
  c7_info[22].dominantType = "double";
  c7_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[22].fileTimeLo = 1286840396U;
  c7_info[22].fileTimeHi = 0U;
  c7_info[22].mFileTimeLo = 0U;
  c7_info[22].mFileTimeHi = 0U;
  c7_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c7_info[23].name = "eml_int_forloop_overflow_check";
  c7_info[23].dominantType = "";
  c7_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[23].fileTimeLo = 1332186672U;
  c7_info[23].fileTimeHi = 0U;
  c7_info[23].mFileTimeLo = 0U;
  c7_info[23].mFileTimeHi = 0U;
  c7_info[24].context = "";
  c7_info[24].name = "mtimes";
  c7_info[24].dominantType = "double";
  c7_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[24].fileTimeLo = 1289541292U;
  c7_info[24].fileTimeHi = 0U;
  c7_info[24].mFileTimeLo = 0U;
  c7_info[24].mFileTimeHi = 0U;
  c7_info[25].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[25].name = "eml_index_class";
  c7_info[25].dominantType = "";
  c7_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[25].fileTimeLo = 1323192178U;
  c7_info[25].fileTimeHi = 0U;
  c7_info[25].mFileTimeLo = 0U;
  c7_info[25].mFileTimeHi = 0U;
  c7_info[26].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[26].name = "eml_scalar_eg";
  c7_info[26].dominantType = "double";
  c7_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[26].fileTimeLo = 1286840396U;
  c7_info[26].fileTimeHi = 0U;
  c7_info[26].mFileTimeLo = 0U;
  c7_info[26].mFileTimeHi = 0U;
  c7_info[27].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[27].name = "eml_xgemm";
  c7_info[27].dominantType = "char";
  c7_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c7_info[27].fileTimeLo = 1299098372U;
  c7_info[27].fileTimeHi = 0U;
  c7_info[27].mFileTimeLo = 0U;
  c7_info[27].mFileTimeHi = 0U;
  c7_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c7_info[28].name = "eml_blas_inline";
  c7_info[28].dominantType = "";
  c7_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c7_info[28].fileTimeLo = 1299098368U;
  c7_info[28].fileTimeHi = 0U;
  c7_info[28].mFileTimeLo = 0U;
  c7_info[28].mFileTimeHi = 0U;
  c7_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c7_info[29].name = "mtimes";
  c7_info[29].dominantType = "double";
  c7_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[29].fileTimeLo = 1289541292U;
  c7_info[29].fileTimeHi = 0U;
  c7_info[29].mFileTimeLo = 0U;
  c7_info[29].mFileTimeHi = 0U;
  c7_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[30].name = "eml_index_class";
  c7_info[30].dominantType = "";
  c7_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[30].fileTimeLo = 1323192178U;
  c7_info[30].fileTimeHi = 0U;
  c7_info[30].mFileTimeLo = 0U;
  c7_info[30].mFileTimeHi = 0U;
  c7_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[31].name = "eml_scalar_eg";
  c7_info[31].dominantType = "double";
  c7_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[31].fileTimeLo = 1286840396U;
  c7_info[31].fileTimeHi = 0U;
  c7_info[31].mFileTimeLo = 0U;
  c7_info[31].mFileTimeHi = 0U;
  c7_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c7_info[32].name = "eml_refblas_xgemm";
  c7_info[32].dominantType = "char";
  c7_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c7_info[32].fileTimeLo = 1299098374U;
  c7_info[32].fileTimeHi = 0U;
  c7_info[32].mFileTimeLo = 0U;
  c7_info[32].mFileTimeHi = 0U;
  c7_info[33].context = "";
  c7_info[33].name = "mpower";
  c7_info[33].dominantType = "double";
  c7_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c7_info[33].fileTimeLo = 1286840442U;
  c7_info[33].fileTimeHi = 0U;
  c7_info[33].mFileTimeLo = 0U;
  c7_info[33].mFileTimeHi = 0U;
  c7_info[34].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c7_info[34].name = "eml_scalar_floor";
  c7_info[34].dominantType = "double";
  c7_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c7_info[34].fileTimeLo = 1286840326U;
  c7_info[34].fileTimeHi = 0U;
  c7_info[34].mFileTimeLo = 0U;
  c7_info[34].mFileTimeHi = 0U;
  c7_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[35].name = "eml_index_class";
  c7_info[35].dominantType = "";
  c7_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[35].fileTimeLo = 1323192178U;
  c7_info[35].fileTimeHi = 0U;
  c7_info[35].mFileTimeLo = 0U;
  c7_info[35].mFileTimeHi = 0U;
  c7_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[36].name = "eml_scalar_eg";
  c7_info[36].dominantType = "double";
  c7_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c7_info[36].fileTimeLo = 1286840396U;
  c7_info[36].fileTimeHi = 0U;
  c7_info[36].mFileTimeLo = 0U;
  c7_info[36].mFileTimeHi = 0U;
  c7_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[37].name = "eml_scalar_abs";
  c7_info[37].dominantType = "double";
  c7_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c7_info[37].fileTimeLo = 1286840312U;
  c7_info[37].fileTimeHi = 0U;
  c7_info[37].mFileTimeLo = 0U;
  c7_info[37].mFileTimeHi = 0U;
  c7_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[38].name = "eml_scalar_floor";
  c7_info[38].dominantType = "double";
  c7_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c7_info[38].fileTimeLo = 1286840326U;
  c7_info[38].fileTimeHi = 0U;
  c7_info[38].mFileTimeLo = 0U;
  c7_info[38].mFileTimeHi = 0U;
  c7_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[39].name = "mtimes";
  c7_info[39].dominantType = "double";
  c7_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[39].fileTimeLo = 1289541292U;
  c7_info[39].fileTimeHi = 0U;
  c7_info[39].mFileTimeLo = 0U;
  c7_info[39].mFileTimeHi = 0U;
  c7_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[40].name = "inv";
  c7_info[40].dominantType = "double";
  c7_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m";
  c7_info[40].fileTimeLo = 1305339600U;
  c7_info[40].fileTimeHi = 0U;
  c7_info[40].mFileTimeLo = 0U;
  c7_info[40].mFileTimeHi = 0U;
  c7_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c7_info[41].name = "eml_index_class";
  c7_info[41].dominantType = "";
  c7_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c7_info[41].fileTimeLo = 1323192178U;
  c7_info[41].fileTimeHi = 0U;
  c7_info[41].mFileTimeLo = 0U;
  c7_info[41].mFileTimeHi = 0U;
  c7_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c7_info[42].name = "abs";
  c7_info[42].dominantType = "double";
  c7_info[42].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[42].fileTimeLo = 1286840294U;
  c7_info[42].fileTimeHi = 0U;
  c7_info[42].mFileTimeLo = 0U;
  c7_info[42].mFileTimeHi = 0U;
  c7_info[43].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[43].name = "eml_scalar_abs";
  c7_info[43].dominantType = "double";
  c7_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c7_info[43].fileTimeLo = 1286840312U;
  c7_info[43].fileTimeHi = 0U;
  c7_info[43].mFileTimeLo = 0U;
  c7_info[43].mFileTimeHi = 0U;
  c7_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c7_info[44].name = "eml_div";
  c7_info[44].dominantType = "double";
  c7_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c7_info[44].fileTimeLo = 1313369410U;
  c7_info[44].fileTimeHi = 0U;
  c7_info[44].mFileTimeLo = 0U;
  c7_info[44].mFileTimeHi = 0U;
  c7_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c7_info[45].name = "mtimes";
  c7_info[45].dominantType = "double";
  c7_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[45].fileTimeLo = 1289541292U;
  c7_info[45].fileTimeHi = 0U;
  c7_info[45].mFileTimeLo = 0U;
  c7_info[45].mFileTimeHi = 0U;
  c7_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c7_info[46].name = "eml_index_plus";
  c7_info[46].dominantType = "double";
  c7_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c7_info[46].fileTimeLo = 1286840378U;
  c7_info[46].fileTimeHi = 0U;
  c7_info[46].mFileTimeLo = 0U;
  c7_info[46].mFileTimeHi = 0U;
  c7_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[47].name = "norm";
  c7_info[47].dominantType = "double";
  c7_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c7_info[47].fileTimeLo = 1336543694U;
  c7_info[47].fileTimeHi = 0U;
  c7_info[47].mFileTimeLo = 0U;
  c7_info[47].mFileTimeHi = 0U;
  c7_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c7_info[48].name = "abs";
  c7_info[48].dominantType = "double";
  c7_info[48].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c7_info[48].fileTimeLo = 1286840294U;
  c7_info[48].fileTimeHi = 0U;
  c7_info[48].mFileTimeLo = 0U;
  c7_info[48].mFileTimeHi = 0U;
  c7_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c7_info[49].name = "isnan";
  c7_info[49].dominantType = "double";
  c7_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c7_info[49].fileTimeLo = 1286840360U;
  c7_info[49].fileTimeHi = 0U;
  c7_info[49].mFileTimeLo = 0U;
  c7_info[49].mFileTimeHi = 0U;
  c7_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c7_info[50].name = "eml_guarded_nan";
  c7_info[50].dominantType = "char";
  c7_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c7_info[50].fileTimeLo = 1286840376U;
  c7_info[50].fileTimeHi = 0U;
  c7_info[50].mFileTimeLo = 0U;
  c7_info[50].mFileTimeHi = 0U;
  c7_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c7_info[51].name = "eml_is_float_class";
  c7_info[51].dominantType = "char";
  c7_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c7_info[51].fileTimeLo = 1286840382U;
  c7_info[51].fileTimeHi = 0U;
  c7_info[51].mFileTimeLo = 0U;
  c7_info[51].mFileTimeHi = 0U;
  c7_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[52].name = "mtimes";
  c7_info[52].dominantType = "double";
  c7_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c7_info[52].fileTimeLo = 1289541292U;
  c7_info[52].fileTimeHi = 0U;
  c7_info[52].mFileTimeLo = 0U;
  c7_info[52].mFileTimeHi = 0U;
  c7_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[53].name = "eml_warning";
  c7_info[53].dominantType = "char";
  c7_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c7_info[53].fileTimeLo = 1286840402U;
  c7_info[53].fileTimeHi = 0U;
  c7_info[53].mFileTimeLo = 0U;
  c7_info[53].mFileTimeHi = 0U;
  c7_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[54].name = "isnan";
  c7_info[54].dominantType = "double";
  c7_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c7_info[54].fileTimeLo = 1286840360U;
  c7_info[54].fileTimeHi = 0U;
  c7_info[54].mFileTimeLo = 0U;
  c7_info[54].mFileTimeHi = 0U;
  c7_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[55].name = "eps";
  c7_info[55].dominantType = "char";
  c7_info[55].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[55].fileTimeLo = 1326749596U;
  c7_info[55].fileTimeHi = 0U;
  c7_info[55].mFileTimeLo = 0U;
  c7_info[55].mFileTimeHi = 0U;
  c7_info[56].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[56].name = "eml_is_float_class";
  c7_info[56].dominantType = "char";
  c7_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c7_info[56].fileTimeLo = 1286840382U;
  c7_info[56].fileTimeHi = 0U;
  c7_info[56].mFileTimeLo = 0U;
  c7_info[56].mFileTimeHi = 0U;
  c7_info[57].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c7_info[57].name = "eml_eps";
  c7_info[57].dominantType = "char";
  c7_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c7_info[57].fileTimeLo = 1326749596U;
  c7_info[57].fileTimeHi = 0U;
  c7_info[57].mFileTimeLo = 0U;
  c7_info[57].mFileTimeHi = 0U;
  c7_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c7_info[58].name = "eml_float_model";
  c7_info[58].dominantType = "char";
  c7_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c7_info[58].fileTimeLo = 1326749596U;
  c7_info[58].fileTimeHi = 0U;
  c7_info[58].mFileTimeLo = 0U;
  c7_info[58].mFileTimeHi = 0U;
  c7_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c7_info[59].name = "eml_flt2str";
  c7_info[59].dominantType = "double";
  c7_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c7_info[59].fileTimeLo = 1309472796U;
  c7_info[59].fileTimeHi = 0U;
  c7_info[59].mFileTimeLo = 0U;
  c7_info[59].mFileTimeHi = 0U;
  c7_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c7_info[60].name = "char";
  c7_info[60].dominantType = "double";
  c7_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m";
  c7_info[60].fileTimeLo = 1319751568U;
  c7_info[60].fileTimeHi = 0U;
  c7_info[60].mFileTimeLo = 0U;
  c7_info[60].mFileTimeHi = 0U;
  c7_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c7_info[61].name = "eml_int_forloop_overflow_check";
  c7_info[61].dominantType = "";
  c7_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c7_info[61].fileTimeLo = 1332186672U;
  c7_info[61].fileTimeHi = 0U;
  c7_info[61].mFileTimeLo = 0U;
  c7_info[61].mFileTimeHi = 0U;
  c7_info[62].context = "";
  c7_info[62].name = "mrdivide";
  c7_info[62].dominantType = "double";
  c7_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c7_info[62].fileTimeLo = 1342832544U;
  c7_info[62].fileTimeHi = 0U;
  c7_info[62].mFileTimeLo = 1319751566U;
  c7_info[62].mFileTimeHi = 0U;
  c7_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c7_info[63].name = "rdivide";
  c7_info[63].dominantType = "double";
  c7_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c7_info[63].fileTimeLo = 1286840444U;
  c7_info[63].fileTimeHi = 0U;
  c7_info[63].mFileTimeLo = 0U;
  c7_info[63].mFileTimeHi = 0U;
}

static void c7_quatinv(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
  real_T c7_qin[4], real_T c7_qinv[4])
{
  uint32_T c7_debug_family_var_map[5];
  real_T c7_q_conj[4];
  real_T c7_nargin = 1.0;
  real_T c7_nargout = 1.0;
  int32_T c7_i127;
  int32_T c7_i128;
  real_T c7_a[4];
  int32_T c7_k;
  real_T c7_b_k;
  real_T c7_ak;
  real_T c7_y[4];
  real_T c7_b_y;
  int32_T c7_c_k;
  int32_T c7_d_k;
  real_T c7_x;
  real_T c7_b_x;
  int32_T c7_i129;
  real_T c7_c_y;
  real_T c7_d_y;
  int32_T c7_i130;
  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c7_b_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c7_q_conj, 0U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 2U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_qin, 3U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_qinv, 4U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 88);
  c7_q_conj[0] = c7_qin[0];
  for (c7_i127 = 0; c7_i127 < 3; c7_i127++) {
    c7_q_conj[c7_i127 + 1] = -c7_qin[c7_i127 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 89);
  for (c7_i128 = 0; c7_i128 < 4; c7_i128++) {
    c7_a[c7_i128] = c7_qin[c7_i128];
  }

  for (c7_k = 0; c7_k < 4; c7_k++) {
    c7_b_k = 1.0 + (real_T)c7_k;
    c7_ak = c7_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c7_b_k), 1, 4, 1, 0) - 1];
    c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c7_b_k),
      1, 4, 1, 0) - 1] = muDoubleScalarPower(c7_ak, 2.0);
  }

  c7_b_y = c7_y[0];
  c7_check_forloop_overflow_error(chartInstance);
  for (c7_c_k = 2; c7_c_k < 5; c7_c_k++) {
    c7_d_k = c7_c_k;
    c7_b_y += c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c7_d_k), 1, 4, 1, 0) - 1];
  }

  c7_x = c7_b_y;
  c7_b_x = c7_x;
  if (c7_b_x < 0.0) {
    c7_eml_error(chartInstance);
  }

  c7_b_x = muDoubleScalarSqrt(c7_b_x);
  for (c7_i129 = 0; c7_i129 < 4; c7_i129++) {
    c7_a[c7_i129] = c7_q_conj[c7_i129];
  }

  c7_c_y = c7_b_x;
  c7_d_y = c7_c_y;
  for (c7_i130 = 0; c7_i130 < 4; c7_i130++) {
    c7_qinv[c7_i130] = c7_a[c7_i130] / c7_d_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -89);
  sf_debug_symbol_scope_pop();
}

static void c7_check_forloop_overflow_error
  (SFc7_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void c7_eml_error(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
  int32_T c7_i131;
  static char_T c7_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c7_u[30];
  const mxArray *c7_y = NULL;
  for (c7_i131 = 0; c7_i131 < 30; c7_i131++) {
    c7_u[c7_i131] = c7_varargin_1[c7_i131];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c7_y));
}

static void c7_quatmultiply(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_q[4], real_T c7_r[4], real_T c7_qres[4])
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
  sf_debug_symbol_scope_push_eml(0U, 7U, 9U, c7_c_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c7_vec, 0U, c7_h_sf_marshallOut,
    c7_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_scalar, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_b_q, MAX_uint32_T,
    c7_g_sf_marshallOut, c7_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_b_r, MAX_uint32_T,
    c7_g_sf_marshallOut, c7_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 4U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 5U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_q, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_r, 3U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_qres, 6U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 68);
  for (c7_i132 = 0; c7_i132 < 4; c7_i132++) {
    c7_b_q[c7_i132] = c7_q[c7_i132];
  }

  sf_debug_symbol_switch(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 69);
  for (c7_i133 = 0; c7_i133 < 4; c7_i133++) {
    c7_b_r[c7_i133] = c7_r[c7_i133];
  }

  sf_debug_symbol_switch(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 72);
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

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 80);
  c7_scalar = ((c7_b_q[0] * c7_b_r[0] - c7_b_q[1] * c7_b_r[1]) - c7_b_q[2] *
               c7_b_r[2]) - c7_b_q[3] * c7_b_r[3];
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 83);
  c7_b_scalar[0] = c7_scalar;
  for (c7_i135 = 0; c7_i135 < 3; c7_i135++) {
    c7_b_scalar[c7_i135 + 1] = c7_vec[c7_i135];
  }

  for (c7_i136 = 0; c7_i136 < 4; c7_i136++) {
    c7_qres[c7_i136] = c7_b_scalar[c7_i136];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -83);
  sf_debug_symbol_scope_pop();
}

static void c7_Kinematics(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_x[7], real_T c7_I[3], real_T c7_Torque_c[3], real_T
  c7_results[7])
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
  sf_debug_symbol_scope_push_eml(0U, 10U, 11U, c7_e_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c7_q, 0U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_w, 1U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_q_dot, 2U, c7_c_sf_marshallOut,
    c7_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_w_dot, 3U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_b_I, MAX_uint32_T,
    c7_i_sf_marshallOut, c7_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 5U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 6U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_x, 7U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_I, 4U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_Torque_c, 8U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_results, 9U, c7_f_sf_marshallOut,
    c7_f_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 53);
  for (c7_i137 = 0; c7_i137 < 4; c7_i137++) {
    c7_q[c7_i137] = c7_x[c7_i137];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 54);
  for (c7_i138 = 0; c7_i138 < 3; c7_i138++) {
    c7_w[c7_i138] = c7_x[c7_i138 + 4];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 55);
  for (c7_i139 = 0; c7_i139 < 3; c7_i139++) {
    c7_c_I[c7_i139] = c7_I[c7_i139];
  }

  c7_diag(chartInstance, c7_c_I, c7_dv34);
  for (c7_i140 = 0; c7_i140 < 9; c7_i140++) {
    c7_b_I[c7_i140] = c7_dv34[c7_i140];
  }

  sf_debug_symbol_switch(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 57);
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

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 58);
  for (c7_i162 = 0; c7_i162 < 9; c7_i162++) {
    c7_a[c7_i162] = c7_b_I[c7_i162];
  }

  for (c7_i163 = 0; c7_i163 < 3; c7_i163++) {
    c7_b_b[c7_i163] = c7_w[c7_i163];
  }

  c7_d_eml_scalar_eg(chartInstance);
  c7_d_eml_scalar_eg(chartInstance);
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

  c7_d_eml_scalar_eg(chartInstance);
  c7_d_eml_scalar_eg(chartInstance);
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

  c7_d_eml_scalar_eg(chartInstance);
  c7_d_eml_scalar_eg(chartInstance);
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

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 64);
  for (c7_i183 = 0; c7_i183 < 4; c7_i183++) {
    c7_results[c7_i183] = c7_q_dot[c7_i183];
  }

  for (c7_i184 = 0; c7_i184 < 3; c7_i184++) {
    c7_results[c7_i184 + 4] = c7_w_dot[c7_i184];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -64);
  sf_debug_symbol_scope_pop();
}

static void c7_diag(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                    real_T c7_v[3], real_T c7_d[9])
{
  int32_T c7_i185;
  int32_T c7_j;
  int32_T c7_b_j;
  int32_T c7_a;
  int32_T c7_c;
  for (c7_i185 = 0; c7_i185 < 9; c7_i185++) {
    c7_d[c7_i185] = 0.0;
  }

  c7_check_forloop_overflow_error(chartInstance);
  for (c7_j = 1; c7_j < 4; c7_j++) {
    c7_b_j = c7_j;
    c7_a = c7_b_j;
    c7_c = c7_a;
    c7_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c7_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c7_c), 1, 3, 2, 0) - 1)) - 1]
      = c7_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c7_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c7_skew_matrix(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_x[3], real_T c7_output[9])
{
  uint32_T c7_debug_family_var_map[4];
  real_T c7_nargin = 1.0;
  real_T c7_nargout = 1.0;
  sf_debug_symbol_scope_push_eml(0U, 4U, 4U, c7_d_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 0U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 1U, c7_e_sf_marshallOut,
    c7_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_x, 2U, c7_d_sf_marshallOut,
    c7_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c7_output, 3U, c7_i_sf_marshallOut,
    c7_i_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 47);
  c7_output[0] = 0.0;
  c7_output[3] = -c7_x[2];
  c7_output[6] = c7_x[1];
  c7_output[1] = c7_x[2];
  c7_output[4] = 0.0;
  c7_output[7] = -c7_x[0];
  c7_output[2] = -c7_x[1];
  c7_output[5] = c7_x[0];
  c7_output[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -47);
  sf_debug_symbol_scope_pop();
}

static void c7_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c7_mpower(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_a[9], real_T c7_c[9])
{
  int32_T c7_i186;
  real_T c7_b_a[9];
  for (c7_i186 = 0; c7_i186 < 9; c7_i186++) {
    c7_b_a[c7_i186] = c7_a[c7_i186];
  }

  c7_matrix_to_integer_power(chartInstance, c7_b_a, -1.0, c7_c);
}

static void c7_matrix_to_integer_power(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c7_a[9], real_T c7_b, real_T c7_c[9])
{
  real_T c7_x;
  real_T c7_e;
  boolean_T c7_firstmult;
  real_T c7_b_x;
  real_T c7_ed2;
  real_T c7_b_b;
  real_T c7_y;
  int32_T c7_i187;
  int32_T c7_i188;
  real_T c7_A[9];
  int32_T c7_i189;
  int32_T c7_i190;
  int32_T c7_i191;
  int32_T c7_i192;
  int32_T c7_i193;
  int32_T c7_i194;
  int32_T c7_i195;
  real_T c7_b_A[9];
  int32_T c7_i196;
  real_T c7_c_A[9];
  real_T c7_n1x;
  int32_T c7_i197;
  real_T c7_b_c[9];
  real_T c7_n1xinv;
  real_T c7_b_a;
  real_T c7_c_b;
  real_T c7_b_y;
  real_T c7_rc;
  real_T c7_c_x;
  boolean_T c7_d_b;
  real_T c7_d_x;
  int32_T c7_i198;
  static char_T c7_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c7_u[8];
  const mxArray *c7_c_y = NULL;
  real_T c7_b_u;
  const mxArray *c7_d_y = NULL;
  real_T c7_c_u;
  const mxArray *c7_e_y = NULL;
  real_T c7_d_u;
  const mxArray *c7_f_y = NULL;
  char_T c7_str[14];
  int32_T c7_i199;
  char_T c7_b_str[14];
  int32_T c7_i200;
  int32_T c7_i201;
  int32_T c7_i202;
  int32_T c7_i203;
  int32_T c7_i204;
  int32_T c7_i205;
  int32_T c7_i206;
  int32_T c7_k;
  int32_T c7_b_k;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  int32_T exitg1;
  c7_b_eml_scalar_eg(chartInstance);
  c7_x = c7_b;
  c7_e = muDoubleScalarAbs(c7_x);
  if (c7_e > 0.0) {
    c7_firstmult = TRUE;
    do {
      exitg1 = 0;
      c7_b_x = c7_e / 2.0;
      c7_ed2 = c7_b_x;
      c7_ed2 = muDoubleScalarFloor(c7_ed2);
      c7_b_b = c7_ed2;
      c7_y = 2.0 * c7_b_b;
      if (c7_y != c7_e) {
        if (c7_firstmult) {
          for (c7_i187 = 0; c7_i187 < 9; c7_i187++) {
            c7_c[c7_i187] = c7_a[c7_i187];
          }

          c7_firstmult = FALSE;
        } else {
          c7_c_eml_scalar_eg(chartInstance);
          c7_c_eml_scalar_eg(chartInstance);
          for (c7_i188 = 0; c7_i188 < 9; c7_i188++) {
            c7_A[c7_i188] = c7_c[c7_i188];
          }

          for (c7_i189 = 0; c7_i189 < 3; c7_i189++) {
            c7_i190 = 0;
            for (c7_i191 = 0; c7_i191 < 3; c7_i191++) {
              c7_c[c7_i190 + c7_i189] = 0.0;
              c7_i192 = 0;
              for (c7_i193 = 0; c7_i193 < 3; c7_i193++) {
                c7_c[c7_i190 + c7_i189] += c7_A[c7_i192 + c7_i189] *
                  c7_a[c7_i193 + c7_i190];
                c7_i192 += 3;
              }

              c7_i190 += 3;
            }
          }
        }
      }

      if (c7_ed2 == 0.0) {
        exitg1 = 1;
      } else {
        c7_e = c7_ed2;
        c7_c_eml_scalar_eg(chartInstance);
        c7_c_eml_scalar_eg(chartInstance);
        for (c7_i200 = 0; c7_i200 < 9; c7_i200++) {
          c7_A[c7_i200] = c7_a[c7_i200];
        }

        for (c7_i201 = 0; c7_i201 < 3; c7_i201++) {
          c7_i202 = 0;
          for (c7_i203 = 0; c7_i203 < 3; c7_i203++) {
            c7_a[c7_i202 + c7_i201] = 0.0;
            c7_i204 = 0;
            for (c7_i205 = 0; c7_i205 < 3; c7_i205++) {
              c7_a[c7_i202 + c7_i201] += c7_A[c7_i204 + c7_i201] * c7_A[c7_i205
                + c7_i202];
              c7_i204 += 3;
            }

            c7_i202 += 3;
          }
        }
      }
    } while (exitg1 == 0);

    if (c7_b < 0.0) {
      for (c7_i194 = 0; c7_i194 < 9; c7_i194++) {
        c7_A[c7_i194] = c7_c[c7_i194];
      }

      for (c7_i195 = 0; c7_i195 < 9; c7_i195++) {
        c7_b_A[c7_i195] = c7_A[c7_i195];
      }

      c7_inv3x3(chartInstance, c7_b_A, c7_c);
      for (c7_i196 = 0; c7_i196 < 9; c7_i196++) {
        c7_c_A[c7_i196] = c7_A[c7_i196];
      }

      c7_n1x = c7_norm(chartInstance, c7_c_A);
      for (c7_i197 = 0; c7_i197 < 9; c7_i197++) {
        c7_b_c[c7_i197] = c7_c[c7_i197];
      }

      c7_n1xinv = c7_norm(chartInstance, c7_b_c);
      c7_b_a = c7_n1x;
      c7_c_b = c7_n1xinv;
      c7_b_y = c7_b_a * c7_c_b;
      c7_rc = 1.0 / c7_b_y;
      guard1 = FALSE;
      guard2 = FALSE;
      if (c7_n1x == 0.0) {
        guard2 = TRUE;
      } else if (c7_n1xinv == 0.0) {
        guard2 = TRUE;
      } else if (c7_rc == 0.0) {
        guard1 = TRUE;
      } else {
        c7_c_x = c7_rc;
        c7_d_b = muDoubleScalarIsNaN(c7_c_x);
        guard3 = FALSE;
        if (c7_d_b) {
          guard3 = TRUE;
        } else {
          if (c7_rc < 2.2204460492503131E-16) {
            guard3 = TRUE;
          }
        }

        if (guard3 == TRUE) {
          c7_d_x = c7_rc;
          for (c7_i198 = 0; c7_i198 < 8; c7_i198++) {
            c7_u[c7_i198] = c7_cv0[c7_i198];
          }

          c7_c_y = NULL;
          sf_mex_assign(&c7_c_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1,
            8), FALSE);
          c7_b_u = 14.0;
          c7_d_y = NULL;
          sf_mex_assign(&c7_d_y, sf_mex_create("y", &c7_b_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c7_c_u = 6.0;
          c7_e_y = NULL;
          sf_mex_assign(&c7_e_y, sf_mex_create("y", &c7_c_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c7_d_u = c7_d_x;
          c7_f_y = NULL;
          sf_mex_assign(&c7_f_y, sf_mex_create("y", &c7_d_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c7_n_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U,
            2U, 14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c7_c_y, 14, c7_d_y,
            14, c7_e_y), 14, c7_f_y), "sprintf", c7_str);
          for (c7_i199 = 0; c7_i199 < 14; c7_i199++) {
            c7_b_str[c7_i199] = c7_str[c7_i199];
          }

          c7_b_eml_warning(chartInstance, c7_b_str);
        }
      }

      if (guard2 == TRUE) {
        guard1 = TRUE;
      }

      if (guard1 == TRUE) {
        c7_eml_warning(chartInstance);
      }
    }
  } else {
    for (c7_i206 = 0; c7_i206 < 9; c7_i206++) {
      c7_c[c7_i206] = 0.0;
    }

    c7_check_forloop_overflow_error(chartInstance);
    for (c7_k = 1; c7_k < 4; c7_k++) {
      c7_b_k = c7_k;
      c7_c[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c7_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c7_b_k), 1, 3, 2, 0) - 1))
        - 1] = 1.0;
    }
  }
}

static void c7_b_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c7_c_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c7_inv3x3(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_x[9], real_T c7_y[9])
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
  real_T c7_z;
  real_T c7_i_x;
  real_T c7_c_y;
  real_T c7_b_z;
  real_T c7_a;
  real_T c7_b;
  real_T c7_d_y;
  real_T c7_b_a;
  real_T c7_b_b;
  real_T c7_e_y;
  real_T c7_c_a;
  real_T c7_c_b;
  real_T c7_f_y;
  real_T c7_d_a;
  real_T c7_d_b;
  real_T c7_g_y;
  real_T c7_j_x;
  real_T c7_k_x;
  real_T c7_h_y;
  real_T c7_l_x;
  real_T c7_m_x;
  real_T c7_i_y;
  int32_T c7_itmp;
  real_T c7_n_x;
  real_T c7_j_y;
  real_T c7_c_z;
  real_T c7_e_a;
  real_T c7_e_b;
  real_T c7_k_y;
  real_T c7_f_a;
  real_T c7_f_b;
  real_T c7_l_y;
  real_T c7_o_x;
  real_T c7_m_y;
  real_T c7_t3;
  real_T c7_g_a;
  real_T c7_g_b;
  real_T c7_n_y;
  real_T c7_p_x;
  real_T c7_o_y;
  real_T c7_t2;
  int32_T c7_h_a;
  int32_T c7_c;
  real_T c7_i_a;
  real_T c7_h_b;
  real_T c7_p_y;
  real_T c7_j_a;
  real_T c7_i_b;
  real_T c7_q_y;
  real_T c7_q_x;
  real_T c7_r_y;
  real_T c7_d_z;
  int32_T c7_k_a;
  int32_T c7_b_c;
  int32_T c7_l_a;
  int32_T c7_c_c;
  real_T c7_r_x;
  real_T c7_s_y;
  real_T c7_m_a;
  real_T c7_j_b;
  real_T c7_t_y;
  real_T c7_s_x;
  real_T c7_u_y;
  int32_T c7_n_a;
  int32_T c7_d_c;
  real_T c7_o_a;
  real_T c7_k_b;
  real_T c7_v_y;
  real_T c7_p_a;
  real_T c7_l_b;
  real_T c7_w_y;
  real_T c7_t_x;
  real_T c7_x_y;
  real_T c7_e_z;
  int32_T c7_q_a;
  int32_T c7_e_c;
  int32_T c7_r_a;
  int32_T c7_f_c;
  real_T c7_y_y;
  real_T c7_s_a;
  real_T c7_m_b;
  real_T c7_ab_y;
  real_T c7_u_x;
  real_T c7_bb_y;
  int32_T c7_t_a;
  int32_T c7_g_c;
  real_T c7_u_a;
  real_T c7_n_b;
  real_T c7_cb_y;
  real_T c7_v_a;
  real_T c7_o_b;
  real_T c7_db_y;
  real_T c7_v_x;
  real_T c7_eb_y;
  real_T c7_f_z;
  int32_T c7_w_a;
  int32_T c7_h_c;
  int32_T c7_x_a;
  int32_T c7_i_c;
  boolean_T guard1 = FALSE;
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
  guard1 = FALSE;
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
      guard1 = TRUE;
    }
  } else {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
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
  c7_z = c7_h_x / c7_b_y;
  c7_x[1] = c7_z;
  c7_i_x = c7_x[2];
  c7_c_y = c7_x[0];
  c7_b_z = c7_i_x / c7_c_y;
  c7_x[2] = c7_b_z;
  c7_a = c7_x[1];
  c7_b = c7_x[3];
  c7_d_y = c7_a * c7_b;
  c7_x[4] -= c7_d_y;
  c7_b_a = c7_x[2];
  c7_b_b = c7_x[3];
  c7_e_y = c7_b_a * c7_b_b;
  c7_x[5] -= c7_e_y;
  c7_c_a = c7_x[1];
  c7_c_b = c7_x[6];
  c7_f_y = c7_c_a * c7_c_b;
  c7_x[7] -= c7_f_y;
  c7_d_a = c7_x[2];
  c7_d_b = c7_x[6];
  c7_g_y = c7_d_a * c7_d_b;
  c7_x[8] -= c7_g_y;
  c7_j_x = c7_x[5];
  c7_k_x = c7_j_x;
  c7_h_y = muDoubleScalarAbs(c7_k_x);
  c7_l_x = c7_x[4];
  c7_m_x = c7_l_x;
  c7_i_y = muDoubleScalarAbs(c7_m_x);
  if (c7_h_y > c7_i_y) {
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

  c7_n_x = c7_x[5];
  c7_j_y = c7_x[4];
  c7_c_z = c7_n_x / c7_j_y;
  c7_x[5] = c7_c_z;
  c7_e_a = c7_x[5];
  c7_e_b = c7_x[7];
  c7_k_y = c7_e_a * c7_e_b;
  c7_x[8] -= c7_k_y;
  c7_f_a = c7_x[5];
  c7_f_b = c7_x[1];
  c7_l_y = c7_f_a * c7_f_b;
  c7_o_x = c7_l_y - c7_x[2];
  c7_m_y = c7_x[8];
  c7_t3 = c7_o_x / c7_m_y;
  c7_g_a = c7_x[7];
  c7_g_b = c7_t3;
  c7_n_y = c7_g_a * c7_g_b;
  c7_p_x = -(c7_x[1] + c7_n_y);
  c7_o_y = c7_x[4];
  c7_t2 = c7_p_x / c7_o_y;
  c7_h_a = c7_p1 + 1;
  c7_c = c7_h_a;
  c7_i_a = c7_x[3];
  c7_h_b = c7_t2;
  c7_p_y = c7_i_a * c7_h_b;
  c7_j_a = c7_x[6];
  c7_i_b = c7_t3;
  c7_q_y = c7_j_a * c7_i_b;
  c7_q_x = (1.0 - c7_p_y) - c7_q_y;
  c7_r_y = c7_x[0];
  c7_d_z = c7_q_x / c7_r_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_c), 1, 9, 1, 0) - 1] = c7_d_z;
  c7_k_a = c7_p1 + 2;
  c7_b_c = c7_k_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_b_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_l_a = c7_p1 + 3;
  c7_c_c = c7_l_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_c_c), 1, 9, 1, 0) - 1] = c7_t3;
  c7_r_x = -c7_x[5];
  c7_s_y = c7_x[8];
  c7_t3 = c7_r_x / c7_s_y;
  c7_m_a = c7_x[7];
  c7_j_b = c7_t3;
  c7_t_y = c7_m_a * c7_j_b;
  c7_s_x = 1.0 - c7_t_y;
  c7_u_y = c7_x[4];
  c7_t2 = c7_s_x / c7_u_y;
  c7_n_a = c7_p2 + 1;
  c7_d_c = c7_n_a;
  c7_o_a = c7_x[3];
  c7_k_b = c7_t2;
  c7_v_y = c7_o_a * c7_k_b;
  c7_p_a = c7_x[6];
  c7_l_b = c7_t3;
  c7_w_y = c7_p_a * c7_l_b;
  c7_t_x = -(c7_v_y + c7_w_y);
  c7_x_y = c7_x[0];
  c7_e_z = c7_t_x / c7_x_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_d_c), 1, 9, 1, 0) - 1] = c7_e_z;
  c7_q_a = c7_p2 + 2;
  c7_e_c = c7_q_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_e_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_r_a = c7_p2 + 3;
  c7_f_c = c7_r_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_f_c), 1, 9, 1, 0) - 1] = c7_t3;
  c7_y_y = c7_x[8];
  c7_t3 = 1.0 / c7_y_y;
  c7_s_a = -c7_x[7];
  c7_m_b = c7_t3;
  c7_ab_y = c7_s_a * c7_m_b;
  c7_u_x = c7_ab_y;
  c7_bb_y = c7_x[4];
  c7_t2 = c7_u_x / c7_bb_y;
  c7_t_a = c7_p3 + 1;
  c7_g_c = c7_t_a;
  c7_u_a = c7_x[3];
  c7_n_b = c7_t2;
  c7_cb_y = c7_u_a * c7_n_b;
  c7_v_a = c7_x[6];
  c7_o_b = c7_t3;
  c7_db_y = c7_v_a * c7_o_b;
  c7_v_x = -(c7_cb_y + c7_db_y);
  c7_eb_y = c7_x[0];
  c7_f_z = c7_v_x / c7_eb_y;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_g_c), 1, 9, 1, 0) - 1] = c7_f_z;
  c7_w_a = c7_p3 + 2;
  c7_h_c = c7_w_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_h_c), 1, 9, 1, 0) - 1] = c7_t2;
  c7_x_a = c7_p3 + 3;
  c7_i_c = c7_x_a;
  c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c7_i_c), 1, 9, 1, 0) - 1] = c7_t3;
}

static real_T c7_norm(SFc7_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c7_x[9])
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
  c7_y = 0.0;
  c7_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c7_j < 3)) {
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
      exitg1 = TRUE;
    } else {
      if (c7_s > c7_y) {
        c7_y = c7_s;
      }

      c7_j++;
    }
  }

  return c7_y;
}

static void c7_eml_warning(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
  int32_T c7_i207;
  static char_T c7_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c7_u[27];
  const mxArray *c7_y = NULL;
  for (c7_i207 = 0; c7_i207 < 27; c7_i207++) {
    c7_u[c7_i207] = c7_varargin_1[c7_i207];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c7_y));
}

static void c7_b_eml_warning(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, char_T c7_varargin_2[14])
{
  int32_T c7_i208;
  static char_T c7_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c7_u[33];
  const mxArray *c7_y = NULL;
  int32_T c7_i209;
  char_T c7_b_u[14];
  const mxArray *c7_b_y = NULL;
  for (c7_i208 = 0; c7_i208 < 33; c7_i208++) {
    c7_u[c7_i208] = c7_varargin_1[c7_i208];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c7_i209 = 0; c7_i209 < 14; c7_i209++) {
    c7_b_u[c7_i209] = c7_varargin_2[c7_i209];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c7_y, 14, c7_b_y));
}

static void c7_d_eml_scalar_eg(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c7_n_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_sprintf, const char_T *c7_identifier, char_T
  c7_y[14])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_sprintf), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_sprintf);
}

static void c7_o_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  char_T c7_y[14])
{
  char_T c7_cv1[14];
  int32_T c7_i210;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c7_i210 = 0; c7_i210 < 14; c7_i210++) {
    c7_y[c7_i210] = c7_cv1[c7_i210];
  }

  sf_mex_destroy(&c7_u);
}

static const mxArray *c7_j_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static int32_T c7_p_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i211;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i211, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i211;
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
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
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

static uint8_T c7_q_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_Control_System_Library, const
  char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_Control_System_Library), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_Control_System_Library);
  return c7_y;
}

static uint8_T c7_r_emlrt_marshallIn(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void init_dsm_address_info(SFc7_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c7_Control_System_Library_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3914795468U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2766591190U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3511175920U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(727197934U);
}

mxArray *sf_c7_Control_System_Library_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("nVqRykQWyE2CcjOkbG7sYG");
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

static const mxArray *sf_get_sim_state_info_c7_Control_System_Library(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[11],T\"q_i_s\",},{M[1],M[10],T\"w_s\",},{M[4],M[0],T\"q_i_c_km1\",S'l','i','p'{{M1x2[118 127],M[0],}}},{M[4],M[0],T\"w_c_km1\",S'l','i','p'{{M1x2[110 117],M[0],}}},{M[8],M[0],T\"is_active_c7_Control_System_Library\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_Control_System_Library_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_Control_System_LibraryInstanceStruct *chartInstance;
    chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_Control_System_LibraryMachineNumber_,
           7,
           1,
           1,
           8,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1287);
        _SFD_CV_INIT_EML_FCN(0,1,"skew_matrix",1289,-1,1449);
        _SFD_CV_INIT_EML_FCN(0,2,"Kinematics",1451,-1,1887);
        _SFD_CV_INIT_EML_FCN(0,3,"quatmultiply",1889,-1,2482);
        _SFD_CV_INIT_EML_FCN(0,4,"quatinv",2484,-1,2582);
        _SFD_CV_INIT_EML_IF(0,1,0,297,339,631,1282);

        {
          static int condStart[] = { 301, 321 };

          static int condEnd[] = { 317, 339 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,301,339,2,0,&(condStart[0]),&(condEnd[0]),
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
      sf_debug_reset_current_state_configuration
        (_Control_System_LibraryMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "c8JwK4vRDyYdMmjyiOtqYE";
}

static void sf_opaque_initialize_c7_Control_System_Library(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_Control_System_LibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c7_Control_System_Library
    ((SFc7_Control_System_LibraryInstanceStruct*) chartInstanceVar);
  initialize_c7_Control_System_Library
    ((SFc7_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c7_Control_System_Library(void *chartInstanceVar)
{
  enable_c7_Control_System_Library((SFc7_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c7_Control_System_Library(void *chartInstanceVar)
{
  disable_c7_Control_System_Library((SFc7_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c7_Control_System_Library(void *chartInstanceVar)
{
  sf_c7_Control_System_Library((SFc7_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_Control_System_Library
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_Control_System_Library
    ((SFc7_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_Control_System_Library();/* state var info */
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

extern void sf_internal_set_sim_state_c7_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_Control_System_Library();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_Control_System_Library
    ((SFc7_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_Control_System_Library
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c7_Control_System_Library(S);
}

static void sf_opaque_set_sim_state_c7_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c7_Control_System_Library(S, st);
}

static void sf_opaque_terminate_c7_Control_System_Library(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_Control_System_LibraryInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c7_Control_System_Library
      ((SFc7_Control_System_LibraryInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Control_System_Library_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_Control_System_Library
    ((SFc7_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_Control_System_Library(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c7_Control_System_Library
      ((SFc7_Control_System_LibraryInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Control_System_Library_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,7,
      "gatewayCannotBeInlinedMultipleTimes"));
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(187941185U));
  ssSetChecksum1(S,(1853517190U));
  ssSetChecksum2(S,(2774474011U));
  ssSetChecksum3(S,(363148451U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_Control_System_Library(SimStruct *S)
{
  SFc7_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc7_Control_System_LibraryInstanceStruct *)malloc(sizeof
    (SFc7_Control_System_LibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_Control_System_LibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c7_Control_System_Library;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c7_Control_System_Library;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c7_Control_System_Library;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c7_Control_System_Library;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c7_Control_System_Library;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c7_Control_System_Library;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c7_Control_System_Library;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c7_Control_System_Library;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_Control_System_Library;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_Control_System_Library;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c7_Control_System_Library;
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

void c7_Control_System_Library_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_Control_System_Library(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_Control_System_Library(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_Control_System_Library(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_Control_System_Library_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
