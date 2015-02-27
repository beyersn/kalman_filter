/* Include files */

#include "blascompat32.h"
#include "Control_System_Library_sfun.h"
#include "c6_Control_System_Library.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Control_System_Library_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c6_debug_family_names[20] = { "w_c_temp", "x_km1", "k1",
  "k2", "k3", "k4", "x_k", "w_s_temp", "nargin", "nargout", "I_c", "w_init_s",
  "Torque_c", "q0_i_s", "q_s_c", "Ts", "w_s", "q_i_s", "w_c_km1", "q_i_c_km1" };

static const char * c6_b_debug_family_names[5] = { "q_conj", "nargin", "nargout",
  "qin", "qinv" };

static const char * c6_c_debug_family_names[7] = { "vec", "scalar", "q", "r",
  "nargin", "nargout", "qres" };

static const char * c6_d_debug_family_names[4] = { "nargin", "nargout", "x",
  "output" };

static const char * c6_e_debug_family_names[10] = { "q", "w", "q_dot", "w_dot",
  "I", "nargin", "nargout", "x", "torque_c", "results" };

/* Function Declarations */
static void initialize_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void initialize_params_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void enable_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void disable_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void set_sim_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c6_st);
static void finalize_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void sf_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void initSimStructsc6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_q_i_c_km1, const char_T *c6_identifier,
  real_T c6_y[4]);
static void c6_b_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_w_c_km1, const char_T *c6_identifier,
  real_T c6_y[3]);
static void c6_d_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_e_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_q_i_s, const char_T *c6_identifier, real_T
  c6_y[4]);
static void c6_f_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_g_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_w_s, const char_T *c6_identifier, real_T
  c6_y[3]);
static void c6_h_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_i_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_j_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[7]);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_k_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4]);
static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_l_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_m_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9]);
static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[64]);
static void c6_quatinv(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
  real_T c6_qin[4], real_T c6_qinv[4]);
static void c6_check_forloop_overflow_error
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance);
static void c6_eml_error(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_quatmultiply(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_q[4], real_T c6_r[4], real_T c6_qres[4]);
static void c6_Kinematics(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_x[7], real_T c6_I[3], real_T c6_torque_c[3], real_T
  c6_results[7]);
static void c6_diag(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                    real_T c6_v[3], real_T c6_d[9]);
static void c6_skew_matrix(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_x[3], real_T c6_output[9]);
static void c6_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_mpower(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_a[9], real_T c6_c[9]);
static void c6_matrix_to_integer_power(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_a[9], real_T c6_b, real_T c6_c[9]);
static void c6_b_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_c_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_inv3x3(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_x[9], real_T c6_y[9]);
static real_T c6_norm(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_x[9]);
static void c6_eml_warning(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_b_eml_warning(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, char_T c6_varargin_2[14]);
static void c6_d_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);
static void c6_n_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_sprintf, const char_T *c6_identifier, char_T
  c6_y[14]);
static void c6_o_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  char_T c6_y[14]);
static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_p_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_q_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_Control_System_Library, const
  char_T *c6_identifier);
static uint8_T c6_r_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_w_c_km1_not_empty = FALSE;
  chartInstance->c6_q_i_c_km1_not_empty = FALSE;
  chartInstance->c6_is_active_c6_Control_System_Library = 0U;
}

static void initialize_params_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void enable_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[4];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i1;
  real_T c6_b_u[3];
  const mxArray *c6_c_y = NULL;
  int32_T c6_i2;
  real_T c6_c_u[4];
  const mxArray *c6_d_y = NULL;
  int32_T c6_i3;
  real_T c6_d_u[3];
  const mxArray *c6_e_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_e_u;
  const mxArray *c6_f_y = NULL;
  real_T (*c6_w_s)[3];
  real_T (*c6_q_i_s)[4];
  c6_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(5), FALSE);
  for (c6_i0 = 0; c6_i0 < 4; c6_i0++) {
    c6_u[c6_i0] = (*c6_q_i_s)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  for (c6_i1 = 0; c6_i1 < 3; c6_i1++) {
    c6_b_u[c6_i1] = (*c6_w_s)[c6_i1];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  for (c6_i2 = 0; c6_i2 < 4; c6_i2++) {
    c6_c_u[c6_i2] = chartInstance->c6_q_i_c_km1[c6_i2];
  }

  c6_d_y = NULL;
  if (!chartInstance->c6_q_i_c_km1_not_empty) {
    sf_mex_assign(&c6_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_c_u, 0, 0U, 1U, 0U, 1, 4),
                  FALSE);
  }

  sf_mex_setcell(c6_y, 2, c6_d_y);
  for (c6_i3 = 0; c6_i3 < 3; c6_i3++) {
    c6_d_u[c6_i3] = chartInstance->c6_w_c_km1[c6_i3];
  }

  c6_e_y = NULL;
  if (!chartInstance->c6_w_c_km1_not_empty) {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 3),
                  FALSE);
  }

  sf_mex_setcell(c6_y, 3, c6_e_y);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_Control_System_Library;
  c6_e_u = c6_hoistedGlobal;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 4, c6_f_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance, const mxArray
   *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[4];
  int32_T c6_i4;
  real_T c6_dv1[3];
  int32_T c6_i5;
  real_T c6_dv2[4];
  int32_T c6_i6;
  real_T c6_dv3[3];
  int32_T c6_i7;
  real_T (*c6_q_i_s)[4];
  real_T (*c6_w_s)[3];
  c6_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
                        "q_i_s", c6_dv0);
  for (c6_i4 = 0; c6_i4 < 4; c6_i4++) {
    (*c6_q_i_s)[c6_i4] = c6_dv0[c6_i4];
  }

  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
                        "w_s", c6_dv1);
  for (c6_i5 = 0; c6_i5 < 3; c6_i5++) {
    (*c6_w_s)[c6_i5] = c6_dv1[c6_i5];
  }

  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 2)),
                      "q_i_c_km1", c6_dv2);
  for (c6_i6 = 0; c6_i6 < 4; c6_i6++) {
    chartInstance->c6_q_i_c_km1[c6_i6] = c6_dv2[c6_i6];
  }

  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)),
                        "w_c_km1", c6_dv3);
  for (c6_i7 = 0; c6_i7 < 3; c6_i7++) {
    chartInstance->c6_w_c_km1[c6_i7] = c6_dv3[c6_i7];
  }

  chartInstance->c6_is_active_c6_Control_System_Library = c6_q_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 4)),
     "is_active_c6_Control_System_Library");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_Control_System_Library(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void sf_c6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
  int32_T c6_i8;
  int32_T c6_i9;
  int32_T c6_i10;
  int32_T c6_i11;
  int32_T c6_i12;
  int32_T c6_i13;
  int32_T c6_i14;
  real_T c6_hoistedGlobal;
  int32_T c6_i15;
  real_T c6_I_c[3];
  int32_T c6_i16;
  real_T c6_w_init_s[3];
  int32_T c6_i17;
  real_T c6_Torque_c[3];
  int32_T c6_i18;
  real_T c6_q0_i_s[4];
  int32_T c6_i19;
  real_T c6_q_s_c[4];
  real_T c6_Ts;
  uint32_T c6_debug_family_var_map[20];
  real_T c6_w_c_temp[4];
  real_T c6_x_km1[7];
  real_T c6_k1[7];
  real_T c6_k2[7];
  real_T c6_k3[7];
  real_T c6_k4[7];
  real_T c6_x_k[7];
  real_T c6_w_s_temp[4];
  real_T c6_nargin = 6.0;
  real_T c6_nargout = 2.0;
  real_T c6_w_s[3];
  real_T c6_q_i_s[4];
  int32_T c6_i20;
  int32_T c6_i21;
  int32_T c6_i22;
  real_T c6_b_q_s_c[4];
  real_T c6_dv4[4];
  int32_T c6_i23;
  real_T c6_dv5[4];
  real_T c6_dv6[4];
  int32_T c6_i24;
  int32_T c6_i25;
  real_T c6_dv7[4];
  int32_T c6_i26;
  real_T c6_c_q_s_c[4];
  real_T c6_dv8[4];
  int32_T c6_i27;
  int32_T c6_i28;
  int32_T c6_i29;
  real_T c6_b_q_i_s[4];
  int32_T c6_i30;
  real_T c6_d_q_s_c[4];
  real_T c6_dv9[4];
  int32_T c6_i31;
  int32_T c6_i32;
  int32_T c6_i33;
  int32_T c6_i34;
  real_T c6_b_x_km1[7];
  int32_T c6_i35;
  real_T c6_b_I_c[3];
  int32_T c6_i36;
  real_T c6_b_Torque_c[3];
  real_T c6_dv10[7];
  int32_T c6_i37;
  int32_T c6_i38;
  real_T c6_b[7];
  int32_T c6_i39;
  real_T c6_b_b;
  int32_T c6_i40;
  int32_T c6_i41;
  real_T c6_c_x_km1[7];
  int32_T c6_i42;
  real_T c6_c_I_c[3];
  int32_T c6_i43;
  real_T c6_c_Torque_c[3];
  real_T c6_dv11[7];
  int32_T c6_i44;
  int32_T c6_i45;
  int32_T c6_i46;
  real_T c6_c_b;
  int32_T c6_i47;
  int32_T c6_i48;
  real_T c6_d_x_km1[7];
  int32_T c6_i49;
  real_T c6_d_I_c[3];
  int32_T c6_i50;
  real_T c6_d_Torque_c[3];
  real_T c6_dv12[7];
  int32_T c6_i51;
  int32_T c6_i52;
  real_T c6_d_b;
  int32_T c6_i53;
  int32_T c6_i54;
  real_T c6_e_x_km1[7];
  int32_T c6_i55;
  real_T c6_e_I_c[3];
  int32_T c6_i56;
  real_T c6_e_Torque_c[3];
  real_T c6_dv13[7];
  int32_T c6_i57;
  int32_T c6_i58;
  int32_T c6_i59;
  int32_T c6_i60;
  real_T c6_e_b[7];
  int32_T c6_i61;
  int32_T c6_i62;
  real_T c6_f_b;
  int32_T c6_i63;
  int32_T c6_i64;
  int32_T c6_i65;
  int32_T c6_i66;
  int32_T c6_i67;
  int32_T c6_i68;
  real_T c6_e_q_s_c[4];
  int32_T c6_i69;
  real_T c6_dv14[4];
  int32_T c6_i70;
  real_T c6_dv15[4];
  real_T c6_dv16[4];
  int32_T c6_i71;
  int32_T c6_i72;
  real_T c6_f_q_s_c[4];
  int32_T c6_i73;
  real_T c6_g_q_s_c[4];
  real_T c6_dv17[4];
  int32_T c6_i74;
  real_T c6_dv18[4];
  int32_T c6_i75;
  real_T c6_dv19[4];
  int32_T c6_i76;
  real_T c6_dv20[4];
  real_T c6_dv21[4];
  int32_T c6_i77;
  int32_T c6_i78;
  int32_T c6_i79;
  int32_T c6_i80;
  real_T *c6_b_Ts;
  real_T (*c6_b_w_s)[3];
  real_T (*c6_c_q_i_s)[4];
  real_T (*c6_h_q_s_c)[4];
  real_T (*c6_b_q0_i_s)[4];
  real_T (*c6_f_Torque_c)[3];
  real_T (*c6_b_w_init_s)[3];
  real_T (*c6_f_I_c)[3];
  boolean_T guard1 = FALSE;
  c6_b_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c6_h_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
  c6_c_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_b_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c6_f_Torque_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c6_f_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  for (c6_i8 = 0; c6_i8 < 3; c6_i8++) {
    _SFD_DATA_RANGE_CHECK((*c6_f_I_c)[c6_i8], 0U);
  }

  for (c6_i9 = 0; c6_i9 < 3; c6_i9++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_w_init_s)[c6_i9], 1U);
  }

  for (c6_i10 = 0; c6_i10 < 3; c6_i10++) {
    _SFD_DATA_RANGE_CHECK((*c6_f_Torque_c)[c6_i10], 2U);
  }

  for (c6_i11 = 0; c6_i11 < 4; c6_i11++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_q0_i_s)[c6_i11], 3U);
  }

  for (c6_i12 = 0; c6_i12 < 3; c6_i12++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_w_s)[c6_i12], 4U);
  }

  for (c6_i13 = 0; c6_i13 < 4; c6_i13++) {
    _SFD_DATA_RANGE_CHECK((*c6_c_q_i_s)[c6_i13], 5U);
  }

  for (c6_i14 = 0; c6_i14 < 4; c6_i14++) {
    _SFD_DATA_RANGE_CHECK((*c6_h_q_s_c)[c6_i14], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_b_Ts, 7U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_Ts;
  for (c6_i15 = 0; c6_i15 < 3; c6_i15++) {
    c6_I_c[c6_i15] = (*c6_f_I_c)[c6_i15];
  }

  for (c6_i16 = 0; c6_i16 < 3; c6_i16++) {
    c6_w_init_s[c6_i16] = (*c6_b_w_init_s)[c6_i16];
  }

  for (c6_i17 = 0; c6_i17 < 3; c6_i17++) {
    c6_Torque_c[c6_i17] = (*c6_f_Torque_c)[c6_i17];
  }

  for (c6_i18 = 0; c6_i18 < 4; c6_i18++) {
    c6_q0_i_s[c6_i18] = (*c6_b_q0_i_s)[c6_i18];
  }

  for (c6_i19 = 0; c6_i19 < 4; c6_i19++) {
    c6_q_s_c[c6_i19] = (*c6_h_q_s_c)[c6_i19];
  }

  c6_Ts = c6_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 20U, 20U, c6_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_w_c_temp, 0U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_x_km1, 1U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_k1, 2U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_k2, 3U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_k3, 4U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_k4, 5U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_x_k, 6U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_w_s_temp, 7U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 8U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 9U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c6_I_c, 10U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_w_init_s, 11U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_Torque_c, 12U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_q0_i_s, 13U, c6_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_q_s_c, 14U, c6_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c6_Ts, 15U, c6_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_w_s, 16U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_q_i_s, 17U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c6_w_c_km1, 18U,
    c6_b_sf_marshallOut, c6_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c6_q_i_c_km1, 19U,
    c6_sf_marshallOut, c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 6);
  guard1 = FALSE;
  if (CV_EML_COND(0, 1, 0, !chartInstance->c6_w_c_km1_not_empty)) {
    guard1 = TRUE;
  } else if (CV_EML_COND(0, 1, 1, !chartInstance->c6_q_i_c_km1_not_empty)) {
    guard1 = TRUE;
  } else {
    CV_EML_MCDC(0, 1, 0, FALSE);
    CV_EML_IF(0, 1, 0, FALSE);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 18);
    for (c6_i32 = 0; c6_i32 < 4; c6_i32++) {
      c6_x_km1[c6_i32] = chartInstance->c6_q_i_c_km1[c6_i32];
    }

    for (c6_i33 = 0; c6_i33 < 3; c6_i33++) {
      c6_x_km1[c6_i33 + 4] = chartInstance->c6_w_c_km1[c6_i33];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 21);
    for (c6_i34 = 0; c6_i34 < 7; c6_i34++) {
      c6_b_x_km1[c6_i34] = c6_x_km1[c6_i34];
    }

    for (c6_i35 = 0; c6_i35 < 3; c6_i35++) {
      c6_b_I_c[c6_i35] = c6_I_c[c6_i35];
    }

    for (c6_i36 = 0; c6_i36 < 3; c6_i36++) {
      c6_b_Torque_c[c6_i36] = c6_Torque_c[c6_i36];
    }

    c6_Kinematics(chartInstance, c6_b_x_km1, c6_b_I_c, c6_b_Torque_c, c6_dv10);
    for (c6_i37 = 0; c6_i37 < 7; c6_i37++) {
      c6_k1[c6_i37] = c6_dv10[c6_i37];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 22);
    for (c6_i38 = 0; c6_i38 < 7; c6_i38++) {
      c6_b[c6_i38] = c6_k1[c6_i38];
    }

    for (c6_i39 = 0; c6_i39 < 7; c6_i39++) {
      c6_b[c6_i39] *= 0.5;
    }

    c6_b_b = c6_Ts;
    for (c6_i40 = 0; c6_i40 < 7; c6_i40++) {
      c6_b[c6_i40] *= c6_b_b;
    }

    for (c6_i41 = 0; c6_i41 < 7; c6_i41++) {
      c6_c_x_km1[c6_i41] = c6_x_km1[c6_i41] + c6_b[c6_i41];
    }

    for (c6_i42 = 0; c6_i42 < 3; c6_i42++) {
      c6_c_I_c[c6_i42] = c6_I_c[c6_i42];
    }

    for (c6_i43 = 0; c6_i43 < 3; c6_i43++) {
      c6_c_Torque_c[c6_i43] = c6_Torque_c[c6_i43];
    }

    c6_Kinematics(chartInstance, c6_c_x_km1, c6_c_I_c, c6_c_Torque_c, c6_dv11);
    for (c6_i44 = 0; c6_i44 < 7; c6_i44++) {
      c6_k2[c6_i44] = c6_dv11[c6_i44];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 23);
    for (c6_i45 = 0; c6_i45 < 7; c6_i45++) {
      c6_b[c6_i45] = c6_k2[c6_i45];
    }

    for (c6_i46 = 0; c6_i46 < 7; c6_i46++) {
      c6_b[c6_i46] *= 0.5;
    }

    c6_c_b = c6_Ts;
    for (c6_i47 = 0; c6_i47 < 7; c6_i47++) {
      c6_b[c6_i47] *= c6_c_b;
    }

    for (c6_i48 = 0; c6_i48 < 7; c6_i48++) {
      c6_d_x_km1[c6_i48] = c6_x_km1[c6_i48] + c6_b[c6_i48];
    }

    for (c6_i49 = 0; c6_i49 < 3; c6_i49++) {
      c6_d_I_c[c6_i49] = c6_I_c[c6_i49];
    }

    for (c6_i50 = 0; c6_i50 < 3; c6_i50++) {
      c6_d_Torque_c[c6_i50] = c6_Torque_c[c6_i50];
    }

    c6_Kinematics(chartInstance, c6_d_x_km1, c6_d_I_c, c6_d_Torque_c, c6_dv12);
    for (c6_i51 = 0; c6_i51 < 7; c6_i51++) {
      c6_k3[c6_i51] = c6_dv12[c6_i51];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 24);
    for (c6_i52 = 0; c6_i52 < 7; c6_i52++) {
      c6_b[c6_i52] = c6_k3[c6_i52];
    }

    c6_d_b = c6_Ts;
    for (c6_i53 = 0; c6_i53 < 7; c6_i53++) {
      c6_b[c6_i53] *= c6_d_b;
    }

    for (c6_i54 = 0; c6_i54 < 7; c6_i54++) {
      c6_e_x_km1[c6_i54] = c6_x_km1[c6_i54] + c6_b[c6_i54];
    }

    for (c6_i55 = 0; c6_i55 < 3; c6_i55++) {
      c6_e_I_c[c6_i55] = c6_I_c[c6_i55];
    }

    for (c6_i56 = 0; c6_i56 < 3; c6_i56++) {
      c6_e_Torque_c[c6_i56] = c6_Torque_c[c6_i56];
    }

    c6_Kinematics(chartInstance, c6_e_x_km1, c6_e_I_c, c6_e_Torque_c, c6_dv13);
    for (c6_i57 = 0; c6_i57 < 7; c6_i57++) {
      c6_k4[c6_i57] = c6_dv13[c6_i57];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 26);
    for (c6_i58 = 0; c6_i58 < 7; c6_i58++) {
      c6_b[c6_i58] = c6_k2[c6_i58];
    }

    for (c6_i59 = 0; c6_i59 < 7; c6_i59++) {
      c6_b[c6_i59] *= 2.0;
    }

    for (c6_i60 = 0; c6_i60 < 7; c6_i60++) {
      c6_e_b[c6_i60] = c6_k3[c6_i60];
    }

    for (c6_i61 = 0; c6_i61 < 7; c6_i61++) {
      c6_e_b[c6_i61] *= 2.0;
    }

    for (c6_i62 = 0; c6_i62 < 7; c6_i62++) {
      c6_b[c6_i62] = ((c6_k1[c6_i62] + c6_b[c6_i62]) + c6_e_b[c6_i62]) +
        c6_k4[c6_i62];
    }

    c6_f_b = c6_Ts;
    for (c6_i63 = 0; c6_i63 < 7; c6_i63++) {
      c6_b[c6_i63] *= c6_f_b;
    }

    for (c6_i64 = 0; c6_i64 < 7; c6_i64++) {
      c6_b[c6_i64] /= 6.0;
    }

    for (c6_i65 = 0; c6_i65 < 7; c6_i65++) {
      c6_x_k[c6_i65] = c6_x_km1[c6_i65] + c6_b[c6_i65];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 29);
    for (c6_i66 = 0; c6_i66 < 4; c6_i66++) {
      chartInstance->c6_q_i_c_km1[c6_i66] = c6_x_k[c6_i66];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 30);
    for (c6_i67 = 0; c6_i67 < 3; c6_i67++) {
      chartInstance->c6_w_c_km1[c6_i67] = c6_x_k[c6_i67 + 4];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 33);
    for (c6_i68 = 0; c6_i68 < 4; c6_i68++) {
      c6_e_q_s_c[c6_i68] = c6_q_s_c[c6_i68];
    }

    c6_quatinv(chartInstance, c6_e_q_s_c, c6_dv4);
    for (c6_i69 = 0; c6_i69 < 4; c6_i69++) {
      c6_dv14[c6_i69] = chartInstance->c6_q_i_c_km1[c6_i69];
    }

    for (c6_i70 = 0; c6_i70 < 4; c6_i70++) {
      c6_dv15[c6_i70] = c6_dv4[c6_i70];
    }

    c6_quatmultiply(chartInstance, c6_dv14, c6_dv15, c6_dv16);
    for (c6_i71 = 0; c6_i71 < 4; c6_i71++) {
      c6_q_i_s[c6_i71] = c6_dv16[c6_i71];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 34);
    for (c6_i72 = 0; c6_i72 < 4; c6_i72++) {
      c6_f_q_s_c[c6_i72] = c6_q_s_c[c6_i72];
    }

    c6_quatinv(chartInstance, c6_f_q_s_c, c6_dv4);
    for (c6_i73 = 0; c6_i73 < 4; c6_i73++) {
      c6_g_q_s_c[c6_i73] = c6_q_s_c[c6_i73];
    }

    c6_dv17[0] = 0.0;
    for (c6_i74 = 0; c6_i74 < 3; c6_i74++) {
      c6_dv17[c6_i74 + 1] = chartInstance->c6_w_c_km1[c6_i74];
    }

    c6_quatmultiply(chartInstance, c6_g_q_s_c, c6_dv17, c6_dv18);
    for (c6_i75 = 0; c6_i75 < 4; c6_i75++) {
      c6_dv19[c6_i75] = c6_dv18[c6_i75];
    }

    for (c6_i76 = 0; c6_i76 < 4; c6_i76++) {
      c6_dv20[c6_i76] = c6_dv4[c6_i76];
    }

    c6_quatmultiply(chartInstance, c6_dv19, c6_dv20, c6_dv21);
    for (c6_i77 = 0; c6_i77 < 4; c6_i77++) {
      c6_w_s_temp[c6_i77] = c6_dv21[c6_i77];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 35);
    for (c6_i78 = 0; c6_i78 < 3; c6_i78++) {
      c6_w_s[c6_i78] = c6_w_s_temp[c6_i78 + 1];
    }
  }

  if (guard1 == TRUE) {
    CV_EML_MCDC(0, 1, 0, TRUE);
    CV_EML_IF(0, 1, 0, TRUE);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
    for (c6_i20 = 0; c6_i20 < 3; c6_i20++) {
      c6_w_s[c6_i20] = c6_w_init_s[c6_i20];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 9);
    for (c6_i21 = 0; c6_i21 < 4; c6_i21++) {
      c6_q_i_s[c6_i21] = c6_q0_i_s[c6_i21];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 12);
    for (c6_i22 = 0; c6_i22 < 4; c6_i22++) {
      c6_b_q_s_c[c6_i22] = c6_q_s_c[c6_i22];
    }

    c6_quatinv(chartInstance, c6_b_q_s_c, c6_dv4);
    for (c6_i23 = 0; c6_i23 < 4; c6_i23++) {
      c6_dv5[c6_i23] = c6_dv4[c6_i23];
    }

    c6_dv6[0] = 0.0;
    for (c6_i24 = 0; c6_i24 < 3; c6_i24++) {
      c6_dv6[c6_i24 + 1] = c6_w_s[c6_i24];
    }

    c6_quatmultiply(chartInstance, c6_dv5, c6_dv6, c6_dv4);
    for (c6_i25 = 0; c6_i25 < 4; c6_i25++) {
      c6_dv7[c6_i25] = c6_dv4[c6_i25];
    }

    for (c6_i26 = 0; c6_i26 < 4; c6_i26++) {
      c6_c_q_s_c[c6_i26] = c6_q_s_c[c6_i26];
    }

    c6_quatmultiply(chartInstance, c6_dv7, c6_c_q_s_c, c6_dv8);
    for (c6_i27 = 0; c6_i27 < 4; c6_i27++) {
      c6_w_c_temp[c6_i27] = c6_dv8[c6_i27];
    }

    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 13);
    for (c6_i28 = 0; c6_i28 < 3; c6_i28++) {
      chartInstance->c6_w_c_km1[c6_i28] = c6_w_c_temp[c6_i28 + 1];
    }

    chartInstance->c6_w_c_km1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 14);
    for (c6_i29 = 0; c6_i29 < 4; c6_i29++) {
      c6_b_q_i_s[c6_i29] = c6_q_i_s[c6_i29];
    }

    for (c6_i30 = 0; c6_i30 < 4; c6_i30++) {
      c6_d_q_s_c[c6_i30] = c6_q_s_c[c6_i30];
    }

    c6_quatmultiply(chartInstance, c6_b_q_i_s, c6_d_q_s_c, c6_dv9);
    for (c6_i31 = 0; c6_i31 < 4; c6_i31++) {
      chartInstance->c6_q_i_c_km1[c6_i31] = c6_dv9[c6_i31];
    }

    chartInstance->c6_q_i_c_km1_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -35);
  sf_debug_symbol_scope_pop();
  for (c6_i79 = 0; c6_i79 < 3; c6_i79++) {
    (*c6_b_w_s)[c6_i79] = c6_w_s[c6_i79];
  }

  for (c6_i80 = 0; c6_i80 < 4; c6_i80++) {
    (*c6_c_q_i_s)[c6_i80] = c6_q_i_s[c6_i80];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c6_sfEvent);
  sf_debug_check_for_state_inconsistency(_Control_System_LibraryMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc6_Control_System_Library
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i81;
  real_T c6_b_inData[4];
  int32_T c6_i82;
  real_T c6_u[4];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i81 = 0; c6_i81 < 4; c6_i81++) {
    c6_b_inData[c6_i81] = (*(real_T (*)[4])c6_inData)[c6_i81];
  }

  for (c6_i82 = 0; c6_i82 < 4; c6_i82++) {
    c6_u[c6_i82] = c6_b_inData[c6_i82];
  }

  c6_y = NULL;
  if (!chartInstance->c6_q_i_c_km1_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_q_i_c_km1, const char_T *c6_identifier,
  real_T c6_y[4])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_q_i_c_km1), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_q_i_c_km1);
}

static void c6_b_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4])
{
  real_T c6_dv22[4];
  int32_T c6_i83;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_q_i_c_km1_not_empty = FALSE;
  } else {
    chartInstance->c6_q_i_c_km1_not_empty = TRUE;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv22, 1, 0, 0U, 1, 0U, 1, 4);
    for (c6_i83 = 0; c6_i83 < 4; c6_i83++) {
      c6_y[c6_i83] = c6_dv22[c6_i83];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_q_i_c_km1;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[4];
  int32_T c6_i84;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_b_q_i_c_km1 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_q_i_c_km1), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_q_i_c_km1);
  for (c6_i84 = 0; c6_i84 < 4; c6_i84++) {
    (*(real_T (*)[4])c6_outData)[c6_i84] = c6_y[c6_i84];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i85;
  real_T c6_b_inData[3];
  int32_T c6_i86;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i85 = 0; c6_i85 < 3; c6_i85++) {
    c6_b_inData[c6_i85] = (*(real_T (*)[3])c6_inData)[c6_i85];
  }

  for (c6_i86 = 0; c6_i86 < 3; c6_i86++) {
    c6_u[c6_i86] = c6_b_inData[c6_i86];
  }

  c6_y = NULL;
  if (!chartInstance->c6_w_c_km1_not_empty) {
    sf_mex_assign(&c6_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_w_c_km1, const char_T *c6_identifier,
  real_T c6_y[3])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_w_c_km1), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_w_c_km1);
}

static void c6_d_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv23[3];
  int32_T c6_i87;
  if (mxIsEmpty(c6_u)) {
    chartInstance->c6_w_c_km1_not_empty = FALSE;
  } else {
    chartInstance->c6_w_c_km1_not_empty = TRUE;
    sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv23, 1, 0, 0U, 1, 0U, 1, 3);
    for (c6_i87 = 0; c6_i87 < 3; c6_i87++) {
      c6_y[c6_i87] = c6_dv23[c6_i87];
    }
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_w_c_km1;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i88;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_b_w_c_km1 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_w_c_km1), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_b_w_c_km1);
  for (c6_i88 = 0; c6_i88 < 3; c6_i88++) {
    (*(real_T (*)[3])c6_outData)[c6_i88] = c6_y[c6_i88];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i89;
  real_T c6_b_inData[4];
  int32_T c6_i90;
  real_T c6_u[4];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i89 = 0; c6_i89 < 4; c6_i89++) {
    c6_b_inData[c6_i89] = (*(real_T (*)[4])c6_inData)[c6_i89];
  }

  for (c6_i90 = 0; c6_i90 < 4; c6_i90++) {
    c6_u[c6_i90] = c6_b_inData[c6_i90];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_e_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_q_i_s, const char_T *c6_identifier, real_T
  c6_y[4])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_i_s), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_i_s);
}

static void c6_f_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4])
{
  real_T c6_dv24[4];
  int32_T c6_i91;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv24, 1, 0, 0U, 1, 0U, 1, 4);
  for (c6_i91 = 0; c6_i91 < 4; c6_i91++) {
    c6_y[c6_i91] = c6_dv24[c6_i91];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_q_i_s;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[4];
  int32_T c6_i92;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_q_i_s = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_i_s), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_i_s);
  for (c6_i92 = 0; c6_i92 < 4; c6_i92++) {
    (*(real_T (*)[4])c6_outData)[c6_i92] = c6_y[c6_i92];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i93;
  real_T c6_b_inData[3];
  int32_T c6_i94;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i93 = 0; c6_i93 < 3; c6_i93++) {
    c6_b_inData[c6_i93] = (*(real_T (*)[3])c6_inData)[c6_i93];
  }

  for (c6_i94 = 0; c6_i94 < 3; c6_i94++) {
    c6_u[c6_i94] = c6_b_inData[c6_i94];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_g_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_w_s, const char_T *c6_identifier, real_T
  c6_y[3])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_w_s), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_w_s);
}

static void c6_h_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv25[3];
  int32_T c6_i95;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv25, 1, 0, 0U, 1, 0U, 1, 3);
  for (c6_i95 = 0; c6_i95 < 3; c6_i95++) {
    c6_y[c6_i95] = c6_dv25[c6_i95];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_w_s;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i96;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_w_s = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_w_s), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_w_s);
  for (c6_i96 = 0; c6_i96 < 3; c6_i96++) {
    (*(real_T (*)[3])c6_outData)[c6_i96] = c6_y[c6_i96];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_i_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i97;
  real_T c6_b_inData[7];
  int32_T c6_i98;
  real_T c6_u[7];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i97 = 0; c6_i97 < 7; c6_i97++) {
    c6_b_inData[c6_i97] = (*(real_T (*)[7])c6_inData)[c6_i97];
  }

  for (c6_i98 = 0; c6_i98 < 7; c6_i98++) {
    c6_u[c6_i98] = c6_b_inData[c6_i98];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 7), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_j_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[7])
{
  real_T c6_dv26[7];
  int32_T c6_i99;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv26, 1, 0, 0U, 1, 0U, 1, 7);
  for (c6_i99 = 0; c6_i99 < 7; c6_i99++) {
    c6_y[c6_i99] = c6_dv26[c6_i99];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_x_k;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[7];
  int32_T c6_i100;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_x_k = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_x_k), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_x_k);
  for (c6_i100 = 0; c6_i100 < 7; c6_i100++) {
    (*(real_T (*)[7])c6_outData)[c6_i100] = c6_y[c6_i100];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i101;
  real_T c6_b_inData[4];
  int32_T c6_i102;
  real_T c6_u[4];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i101 = 0; c6_i101 < 4; c6_i101++) {
    c6_b_inData[c6_i101] = (*(real_T (*)[4])c6_inData)[c6_i101];
  }

  for (c6_i102 = 0; c6_i102 < 4; c6_i102++) {
    c6_u[c6_i102] = c6_b_inData[c6_i102];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_k_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[4])
{
  real_T c6_dv27[4];
  int32_T c6_i103;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv27, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c6_i103 = 0; c6_i103 < 4; c6_i103++) {
    c6_y[c6_i103] = c6_dv27[c6_i103];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_r;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[4];
  int32_T c6_i104;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_r = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_r), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_r);
  for (c6_i104 = 0; c6_i104 < 4; c6_i104++) {
    (*(real_T (*)[4])c6_outData)[c6_i104] = c6_y[c6_i104];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i105;
  real_T c6_b_inData[3];
  int32_T c6_i106;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i105 = 0; c6_i105 < 3; c6_i105++) {
    c6_b_inData[c6_i105] = (*(real_T (*)[3])c6_inData)[c6_i105];
  }

  for (c6_i106 = 0; c6_i106 < 3; c6_i106++) {
    c6_u[c6_i106] = c6_b_inData[c6_i106];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_l_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv28[3];
  int32_T c6_i107;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv28, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c6_i107 = 0; c6_i107 < 3; c6_i107++) {
    c6_y[c6_i107] = c6_dv28[c6_i107];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_vec;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i108;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_vec = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_vec), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_vec);
  for (c6_i108 = 0; c6_i108 < 3; c6_i108++) {
    (*(real_T (*)[3])c6_outData)[c6_i108] = c6_y[c6_i108];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i109;
  int32_T c6_i110;
  int32_T c6_i111;
  real_T c6_b_inData[9];
  int32_T c6_i112;
  int32_T c6_i113;
  int32_T c6_i114;
  real_T c6_u[9];
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i109 = 0;
  for (c6_i110 = 0; c6_i110 < 3; c6_i110++) {
    for (c6_i111 = 0; c6_i111 < 3; c6_i111++) {
      c6_b_inData[c6_i111 + c6_i109] = (*(real_T (*)[9])c6_inData)[c6_i111 +
        c6_i109];
    }

    c6_i109 += 3;
  }

  c6_i112 = 0;
  for (c6_i113 = 0; c6_i113 < 3; c6_i113++) {
    for (c6_i114 = 0; c6_i114 < 3; c6_i114++) {
      c6_u[c6_i114 + c6_i112] = c6_b_inData[c6_i114 + c6_i112];
    }

    c6_i112 += 3;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_m_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9])
{
  real_T c6_dv29[9];
  int32_T c6_i115;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv29, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c6_i115 = 0; c6_i115 < 9; c6_i115++) {
    c6_y[c6_i115] = c6_dv29[c6_i115];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_output;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[9];
  int32_T c6_i116;
  int32_T c6_i117;
  int32_T c6_i118;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_output = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_output), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_output);
  c6_i116 = 0;
  for (c6_i117 = 0; c6_i117 < 3; c6_i117++) {
    for (c6_i118 = 0; c6_i118 < 3; c6_i118++) {
      (*(real_T (*)[9])c6_outData)[c6_i118 + c6_i116] = c6_y[c6_i118 + c6_i116];
    }

    c6_i116 += 3;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_Control_System_Library_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[64];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i119;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 64), FALSE);
  for (c6_i119 = 0; c6_i119 < 64; c6_i119++) {
    c6_r0 = &c6_info[c6_i119];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context", "nameCaptureInfo",
                    c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name", "nameCaptureInfo", c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved", "nameCaptureInfo",
                    c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i119);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i119);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[64])
{
  c6_info[0].context = "";
  c6_info[0].name = "power";
  c6_info[0].dominantType = "double";
  c6_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[0].fileTimeLo = 1336543696U;
  c6_info[0].fileTimeHi = 0U;
  c6_info[0].mFileTimeLo = 0U;
  c6_info[0].mFileTimeHi = 0U;
  c6_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[1].name = "eml_scalar_eg";
  c6_info[1].dominantType = "double";
  c6_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[1].fileTimeLo = 1286840396U;
  c6_info[1].fileTimeHi = 0U;
  c6_info[1].mFileTimeLo = 0U;
  c6_info[1].mFileTimeHi = 0U;
  c6_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[2].name = "eml_scalexp_alloc";
  c6_info[2].dominantType = "double";
  c6_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[2].fileTimeLo = 1330630034U;
  c6_info[2].fileTimeHi = 0U;
  c6_info[2].mFileTimeLo = 0U;
  c6_info[2].mFileTimeHi = 0U;
  c6_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c6_info[3].name = "floor";
  c6_info[3].dominantType = "double";
  c6_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[3].fileTimeLo = 1286840342U;
  c6_info[3].fileTimeHi = 0U;
  c6_info[3].mFileTimeLo = 0U;
  c6_info[3].mFileTimeHi = 0U;
  c6_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[4].name = "eml_scalar_floor";
  c6_info[4].dominantType = "double";
  c6_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[4].fileTimeLo = 1286840326U;
  c6_info[4].fileTimeHi = 0U;
  c6_info[4].mFileTimeLo = 0U;
  c6_info[4].mFileTimeHi = 0U;
  c6_info[5].context = "";
  c6_info[5].name = "sum";
  c6_info[5].dominantType = "double";
  c6_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[5].fileTimeLo = 1314758212U;
  c6_info[5].fileTimeHi = 0U;
  c6_info[5].mFileTimeLo = 0U;
  c6_info[5].mFileTimeHi = 0U;
  c6_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[6].name = "isequal";
  c6_info[6].dominantType = "double";
  c6_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[6].fileTimeLo = 1286840358U;
  c6_info[6].fileTimeHi = 0U;
  c6_info[6].mFileTimeLo = 0U;
  c6_info[6].mFileTimeHi = 0U;
  c6_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[7].name = "eml_isequal_core";
  c6_info[7].dominantType = "double";
  c6_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[7].fileTimeLo = 1286840386U;
  c6_info[7].fileTimeHi = 0U;
  c6_info[7].mFileTimeLo = 0U;
  c6_info[7].mFileTimeHi = 0U;
  c6_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[8].name = "eml_const_nonsingleton_dim";
  c6_info[8].dominantType = "double";
  c6_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c6_info[8].fileTimeLo = 1286840296U;
  c6_info[8].fileTimeHi = 0U;
  c6_info[8].mFileTimeLo = 0U;
  c6_info[8].mFileTimeHi = 0U;
  c6_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[9].name = "eml_scalar_eg";
  c6_info[9].dominantType = "double";
  c6_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[9].fileTimeLo = 1286840396U;
  c6_info[9].fileTimeHi = 0U;
  c6_info[9].mFileTimeLo = 0U;
  c6_info[9].mFileTimeHi = 0U;
  c6_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[10].name = "eml_index_class";
  c6_info[10].dominantType = "";
  c6_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[10].fileTimeLo = 1323192178U;
  c6_info[10].fileTimeHi = 0U;
  c6_info[10].mFileTimeLo = 0U;
  c6_info[10].mFileTimeHi = 0U;
  c6_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[11].name = "eml_int_forloop_overflow_check";
  c6_info[11].dominantType = "";
  c6_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[11].fileTimeLo = 1332186672U;
  c6_info[11].fileTimeHi = 0U;
  c6_info[11].mFileTimeLo = 0U;
  c6_info[11].mFileTimeHi = 0U;
  c6_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[12].name = "intmax";
  c6_info[12].dominantType = "char";
  c6_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[12].fileTimeLo = 1311276916U;
  c6_info[12].fileTimeHi = 0U;
  c6_info[12].mFileTimeLo = 0U;
  c6_info[12].mFileTimeHi = 0U;
  c6_info[13].context = "";
  c6_info[13].name = "sqrt";
  c6_info[13].dominantType = "double";
  c6_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c6_info[13].fileTimeLo = 1286840352U;
  c6_info[13].fileTimeHi = 0U;
  c6_info[13].mFileTimeLo = 0U;
  c6_info[13].mFileTimeHi = 0U;
  c6_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c6_info[14].name = "eml_error";
  c6_info[14].dominantType = "char";
  c6_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c6_info[14].fileTimeLo = 1305339600U;
  c6_info[14].fileTimeHi = 0U;
  c6_info[14].mFileTimeLo = 0U;
  c6_info[14].mFileTimeHi = 0U;
  c6_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c6_info[15].name = "eml_scalar_sqrt";
  c6_info[15].dominantType = "double";
  c6_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c6_info[15].fileTimeLo = 1286840338U;
  c6_info[15].fileTimeHi = 0U;
  c6_info[15].mFileTimeLo = 0U;
  c6_info[15].mFileTimeHi = 0U;
  c6_info[16].context = "";
  c6_info[16].name = "rdivide";
  c6_info[16].dominantType = "double";
  c6_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[16].fileTimeLo = 1286840444U;
  c6_info[16].fileTimeHi = 0U;
  c6_info[16].mFileTimeLo = 0U;
  c6_info[16].mFileTimeHi = 0U;
  c6_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[17].name = "eml_div";
  c6_info[17].dominantType = "double";
  c6_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[17].fileTimeLo = 1313369410U;
  c6_info[17].fileTimeHi = 0U;
  c6_info[17].mFileTimeLo = 0U;
  c6_info[17].mFileTimeHi = 0U;
  c6_info[18].context = "";
  c6_info[18].name = "diag";
  c6_info[18].dominantType = "double";
  c6_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c6_info[18].fileTimeLo = 1286840286U;
  c6_info[18].fileTimeHi = 0U;
  c6_info[18].mFileTimeLo = 0U;
  c6_info[18].mFileTimeHi = 0U;
  c6_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c6_info[19].name = "eml_index_class";
  c6_info[19].dominantType = "";
  c6_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[19].fileTimeLo = 1323192178U;
  c6_info[19].fileTimeHi = 0U;
  c6_info[19].mFileTimeLo = 0U;
  c6_info[19].mFileTimeHi = 0U;
  c6_info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c6_info[20].name = "eml_index_plus";
  c6_info[20].dominantType = "coder.internal.indexInt";
  c6_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[20].fileTimeLo = 1286840378U;
  c6_info[20].fileTimeHi = 0U;
  c6_info[20].mFileTimeLo = 0U;
  c6_info[20].mFileTimeHi = 0U;
  c6_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[21].name = "eml_index_class";
  c6_info[21].dominantType = "";
  c6_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[21].fileTimeLo = 1323192178U;
  c6_info[21].fileTimeHi = 0U;
  c6_info[21].mFileTimeLo = 0U;
  c6_info[21].mFileTimeHi = 0U;
  c6_info[22].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c6_info[22].name = "eml_scalar_eg";
  c6_info[22].dominantType = "double";
  c6_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[22].fileTimeLo = 1286840396U;
  c6_info[22].fileTimeHi = 0U;
  c6_info[22].mFileTimeLo = 0U;
  c6_info[22].mFileTimeHi = 0U;
  c6_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c6_info[23].name = "eml_int_forloop_overflow_check";
  c6_info[23].dominantType = "";
  c6_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[23].fileTimeLo = 1332186672U;
  c6_info[23].fileTimeHi = 0U;
  c6_info[23].mFileTimeLo = 0U;
  c6_info[23].mFileTimeHi = 0U;
  c6_info[24].context = "";
  c6_info[24].name = "mtimes";
  c6_info[24].dominantType = "double";
  c6_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[24].fileTimeLo = 1289541292U;
  c6_info[24].fileTimeHi = 0U;
  c6_info[24].mFileTimeLo = 0U;
  c6_info[24].mFileTimeHi = 0U;
  c6_info[25].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[25].name = "eml_index_class";
  c6_info[25].dominantType = "";
  c6_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[25].fileTimeLo = 1323192178U;
  c6_info[25].fileTimeHi = 0U;
  c6_info[25].mFileTimeLo = 0U;
  c6_info[25].mFileTimeHi = 0U;
  c6_info[26].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[26].name = "eml_scalar_eg";
  c6_info[26].dominantType = "double";
  c6_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[26].fileTimeLo = 1286840396U;
  c6_info[26].fileTimeHi = 0U;
  c6_info[26].mFileTimeLo = 0U;
  c6_info[26].mFileTimeHi = 0U;
  c6_info[27].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[27].name = "eml_xgemm";
  c6_info[27].dominantType = "char";
  c6_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[27].fileTimeLo = 1299098372U;
  c6_info[27].fileTimeHi = 0U;
  c6_info[27].mFileTimeLo = 0U;
  c6_info[27].mFileTimeHi = 0U;
  c6_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[28].name = "eml_blas_inline";
  c6_info[28].dominantType = "";
  c6_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[28].fileTimeLo = 1299098368U;
  c6_info[28].fileTimeHi = 0U;
  c6_info[28].mFileTimeLo = 0U;
  c6_info[28].mFileTimeHi = 0U;
  c6_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c6_info[29].name = "mtimes";
  c6_info[29].dominantType = "double";
  c6_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[29].fileTimeLo = 1289541292U;
  c6_info[29].fileTimeHi = 0U;
  c6_info[29].mFileTimeLo = 0U;
  c6_info[29].mFileTimeHi = 0U;
  c6_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[30].name = "eml_index_class";
  c6_info[30].dominantType = "";
  c6_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[30].fileTimeLo = 1323192178U;
  c6_info[30].fileTimeHi = 0U;
  c6_info[30].mFileTimeLo = 0U;
  c6_info[30].mFileTimeHi = 0U;
  c6_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[31].name = "eml_scalar_eg";
  c6_info[31].dominantType = "double";
  c6_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[31].fileTimeLo = 1286840396U;
  c6_info[31].fileTimeHi = 0U;
  c6_info[31].mFileTimeLo = 0U;
  c6_info[31].mFileTimeHi = 0U;
  c6_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[32].name = "eml_refblas_xgemm";
  c6_info[32].dominantType = "char";
  c6_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c6_info[32].fileTimeLo = 1299098374U;
  c6_info[32].fileTimeHi = 0U;
  c6_info[32].mFileTimeLo = 0U;
  c6_info[32].mFileTimeHi = 0U;
  c6_info[33].context = "";
  c6_info[33].name = "mpower";
  c6_info[33].dominantType = "double";
  c6_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c6_info[33].fileTimeLo = 1286840442U;
  c6_info[33].fileTimeHi = 0U;
  c6_info[33].mFileTimeLo = 0U;
  c6_info[33].mFileTimeHi = 0U;
  c6_info[34].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c6_info[34].name = "eml_scalar_floor";
  c6_info[34].dominantType = "double";
  c6_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[34].fileTimeLo = 1286840326U;
  c6_info[34].fileTimeHi = 0U;
  c6_info[34].mFileTimeLo = 0U;
  c6_info[34].mFileTimeHi = 0U;
  c6_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[35].name = "eml_index_class";
  c6_info[35].dominantType = "";
  c6_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[35].fileTimeLo = 1323192178U;
  c6_info[35].fileTimeHi = 0U;
  c6_info[35].mFileTimeLo = 0U;
  c6_info[35].mFileTimeHi = 0U;
  c6_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[36].name = "eml_scalar_eg";
  c6_info[36].dominantType = "double";
  c6_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[36].fileTimeLo = 1286840396U;
  c6_info[36].fileTimeHi = 0U;
  c6_info[36].mFileTimeLo = 0U;
  c6_info[36].mFileTimeHi = 0U;
  c6_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[37].name = "eml_scalar_abs";
  c6_info[37].dominantType = "double";
  c6_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[37].fileTimeLo = 1286840312U;
  c6_info[37].fileTimeHi = 0U;
  c6_info[37].mFileTimeLo = 0U;
  c6_info[37].mFileTimeHi = 0U;
  c6_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[38].name = "eml_scalar_floor";
  c6_info[38].dominantType = "double";
  c6_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[38].fileTimeLo = 1286840326U;
  c6_info[38].fileTimeHi = 0U;
  c6_info[38].mFileTimeLo = 0U;
  c6_info[38].mFileTimeHi = 0U;
  c6_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[39].name = "mtimes";
  c6_info[39].dominantType = "double";
  c6_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[39].fileTimeLo = 1289541292U;
  c6_info[39].fileTimeHi = 0U;
  c6_info[39].mFileTimeLo = 0U;
  c6_info[39].mFileTimeHi = 0U;
  c6_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[40].name = "inv";
  c6_info[40].dominantType = "double";
  c6_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m";
  c6_info[40].fileTimeLo = 1305339600U;
  c6_info[40].fileTimeHi = 0U;
  c6_info[40].mFileTimeLo = 0U;
  c6_info[40].mFileTimeHi = 0U;
  c6_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c6_info[41].name = "eml_index_class";
  c6_info[41].dominantType = "";
  c6_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[41].fileTimeLo = 1323192178U;
  c6_info[41].fileTimeHi = 0U;
  c6_info[41].mFileTimeLo = 0U;
  c6_info[41].mFileTimeHi = 0U;
  c6_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c6_info[42].name = "abs";
  c6_info[42].dominantType = "double";
  c6_info[42].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[42].fileTimeLo = 1286840294U;
  c6_info[42].fileTimeHi = 0U;
  c6_info[42].mFileTimeLo = 0U;
  c6_info[42].mFileTimeHi = 0U;
  c6_info[43].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[43].name = "eml_scalar_abs";
  c6_info[43].dominantType = "double";
  c6_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[43].fileTimeLo = 1286840312U;
  c6_info[43].fileTimeHi = 0U;
  c6_info[43].mFileTimeLo = 0U;
  c6_info[43].mFileTimeHi = 0U;
  c6_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c6_info[44].name = "eml_div";
  c6_info[44].dominantType = "double";
  c6_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[44].fileTimeLo = 1313369410U;
  c6_info[44].fileTimeHi = 0U;
  c6_info[44].mFileTimeLo = 0U;
  c6_info[44].mFileTimeHi = 0U;
  c6_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c6_info[45].name = "mtimes";
  c6_info[45].dominantType = "double";
  c6_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[45].fileTimeLo = 1289541292U;
  c6_info[45].fileTimeHi = 0U;
  c6_info[45].mFileTimeLo = 0U;
  c6_info[45].mFileTimeHi = 0U;
  c6_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3";
  c6_info[46].name = "eml_index_plus";
  c6_info[46].dominantType = "double";
  c6_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[46].fileTimeLo = 1286840378U;
  c6_info[46].fileTimeHi = 0U;
  c6_info[46].mFileTimeLo = 0U;
  c6_info[46].mFileTimeHi = 0U;
  c6_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[47].name = "norm";
  c6_info[47].dominantType = "double";
  c6_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c6_info[47].fileTimeLo = 1336543694U;
  c6_info[47].fileTimeHi = 0U;
  c6_info[47].mFileTimeLo = 0U;
  c6_info[47].mFileTimeHi = 0U;
  c6_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[48].name = "abs";
  c6_info[48].dominantType = "double";
  c6_info[48].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[48].fileTimeLo = 1286840294U;
  c6_info[48].fileTimeHi = 0U;
  c6_info[48].mFileTimeLo = 0U;
  c6_info[48].mFileTimeHi = 0U;
  c6_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[49].name = "isnan";
  c6_info[49].dominantType = "double";
  c6_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[49].fileTimeLo = 1286840360U;
  c6_info[49].fileTimeHi = 0U;
  c6_info[49].mFileTimeLo = 0U;
  c6_info[49].mFileTimeHi = 0U;
  c6_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[50].name = "eml_guarded_nan";
  c6_info[50].dominantType = "char";
  c6_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[50].fileTimeLo = 1286840376U;
  c6_info[50].fileTimeHi = 0U;
  c6_info[50].mFileTimeLo = 0U;
  c6_info[50].mFileTimeHi = 0U;
  c6_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[51].name = "eml_is_float_class";
  c6_info[51].dominantType = "char";
  c6_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[51].fileTimeLo = 1286840382U;
  c6_info[51].fileTimeHi = 0U;
  c6_info[51].mFileTimeLo = 0U;
  c6_info[51].mFileTimeHi = 0U;
  c6_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[52].name = "mtimes";
  c6_info[52].dominantType = "double";
  c6_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[52].fileTimeLo = 1289541292U;
  c6_info[52].fileTimeHi = 0U;
  c6_info[52].mFileTimeLo = 0U;
  c6_info[52].mFileTimeHi = 0U;
  c6_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[53].name = "eml_warning";
  c6_info[53].dominantType = "char";
  c6_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[53].fileTimeLo = 1286840402U;
  c6_info[53].fileTimeHi = 0U;
  c6_info[53].mFileTimeLo = 0U;
  c6_info[53].mFileTimeHi = 0U;
  c6_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[54].name = "isnan";
  c6_info[54].dominantType = "double";
  c6_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[54].fileTimeLo = 1286840360U;
  c6_info[54].fileTimeHi = 0U;
  c6_info[54].mFileTimeLo = 0U;
  c6_info[54].mFileTimeHi = 0U;
  c6_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[55].name = "eps";
  c6_info[55].dominantType = "char";
  c6_info[55].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[55].fileTimeLo = 1326749596U;
  c6_info[55].fileTimeHi = 0U;
  c6_info[55].mFileTimeLo = 0U;
  c6_info[55].mFileTimeHi = 0U;
  c6_info[56].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[56].name = "eml_is_float_class";
  c6_info[56].dominantType = "char";
  c6_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[56].fileTimeLo = 1286840382U;
  c6_info[56].fileTimeHi = 0U;
  c6_info[56].mFileTimeLo = 0U;
  c6_info[56].mFileTimeHi = 0U;
  c6_info[57].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[57].name = "eml_eps";
  c6_info[57].dominantType = "char";
  c6_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[57].fileTimeLo = 1326749596U;
  c6_info[57].fileTimeHi = 0U;
  c6_info[57].mFileTimeLo = 0U;
  c6_info[57].mFileTimeHi = 0U;
  c6_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[58].name = "eml_float_model";
  c6_info[58].dominantType = "char";
  c6_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[58].fileTimeLo = 1326749596U;
  c6_info[58].fileTimeHi = 0U;
  c6_info[58].mFileTimeLo = 0U;
  c6_info[58].mFileTimeHi = 0U;
  c6_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[59].name = "eml_flt2str";
  c6_info[59].dominantType = "double";
  c6_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c6_info[59].fileTimeLo = 1309472796U;
  c6_info[59].fileTimeHi = 0U;
  c6_info[59].mFileTimeLo = 0U;
  c6_info[59].mFileTimeHi = 0U;
  c6_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c6_info[60].name = "char";
  c6_info[60].dominantType = "double";
  c6_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m";
  c6_info[60].fileTimeLo = 1319751568U;
  c6_info[60].fileTimeHi = 0U;
  c6_info[60].mFileTimeLo = 0U;
  c6_info[60].mFileTimeHi = 0U;
  c6_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power";
  c6_info[61].name = "eml_int_forloop_overflow_check";
  c6_info[61].dominantType = "";
  c6_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[61].fileTimeLo = 1332186672U;
  c6_info[61].fileTimeHi = 0U;
  c6_info[61].mFileTimeLo = 0U;
  c6_info[61].mFileTimeHi = 0U;
  c6_info[62].context = "";
  c6_info[62].name = "mrdivide";
  c6_info[62].dominantType = "double";
  c6_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[62].fileTimeLo = 1342832544U;
  c6_info[62].fileTimeHi = 0U;
  c6_info[62].mFileTimeLo = 1319751566U;
  c6_info[62].mFileTimeHi = 0U;
  c6_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[63].name = "rdivide";
  c6_info[63].dominantType = "double";
  c6_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[63].fileTimeLo = 1286840444U;
  c6_info[63].fileTimeHi = 0U;
  c6_info[63].mFileTimeLo = 0U;
  c6_info[63].mFileTimeHi = 0U;
}

static void c6_quatinv(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
  real_T c6_qin[4], real_T c6_qinv[4])
{
  uint32_T c6_debug_family_var_map[5];
  real_T c6_q_conj[4];
  real_T c6_nargin = 1.0;
  real_T c6_nargout = 1.0;
  int32_T c6_i120;
  int32_T c6_i121;
  real_T c6_a[4];
  int32_T c6_k;
  real_T c6_b_k;
  real_T c6_ak;
  real_T c6_y[4];
  real_T c6_b_y;
  int32_T c6_c_k;
  int32_T c6_d_k;
  real_T c6_x;
  real_T c6_b_x;
  int32_T c6_i122;
  real_T c6_c_y;
  real_T c6_d_y;
  int32_T c6_i123;
  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c6_b_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_q_conj, 0U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 1U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 2U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_qin, 3U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_qinv, 4U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 80);
  c6_q_conj[0] = c6_qin[0];
  for (c6_i120 = 0; c6_i120 < 3; c6_i120++) {
    c6_q_conj[c6_i120 + 1] = -c6_qin[c6_i120 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 81);
  for (c6_i121 = 0; c6_i121 < 4; c6_i121++) {
    c6_a[c6_i121] = c6_qin[c6_i121];
  }

  for (c6_k = 0; c6_k < 4; c6_k++) {
    c6_b_k = 1.0 + (real_T)c6_k;
    c6_ak = c6_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c6_b_k), 1, 4, 1, 0) - 1];
    c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c6_b_k),
      1, 4, 1, 0) - 1] = muDoubleScalarPower(c6_ak, 2.0);
  }

  c6_b_y = c6_y[0];
  c6_check_forloop_overflow_error(chartInstance);
  for (c6_c_k = 2; c6_c_k < 5; c6_c_k++) {
    c6_d_k = c6_c_k;
    c6_b_y += c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_d_k), 1, 4, 1, 0) - 1];
  }

  c6_x = c6_b_y;
  c6_b_x = c6_x;
  if (c6_b_x < 0.0) {
    c6_eml_error(chartInstance);
  }

  c6_b_x = muDoubleScalarSqrt(c6_b_x);
  for (c6_i122 = 0; c6_i122 < 4; c6_i122++) {
    c6_a[c6_i122] = c6_q_conj[c6_i122];
  }

  c6_c_y = c6_b_x;
  c6_d_y = c6_c_y;
  for (c6_i123 = 0; c6_i123 < 4; c6_i123++) {
    c6_qinv[c6_i123] = c6_a[c6_i123] / c6_d_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -81);
  sf_debug_symbol_scope_pop();
}

static void c6_check_forloop_overflow_error
  (SFc6_Control_System_LibraryInstanceStruct *chartInstance)
{
}

static void c6_eml_error(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
  int32_T c6_i124;
  static char_T c6_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c6_u[30];
  const mxArray *c6_y = NULL;
  for (c6_i124 = 0; c6_i124 < 30; c6_i124++) {
    c6_u[c6_i124] = c6_varargin_1[c6_i124];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c6_y));
}

static void c6_quatmultiply(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_q[4], real_T c6_r[4], real_T c6_qres[4])
{
  uint32_T c6_debug_family_var_map[7];
  real_T c6_vec[3];
  real_T c6_scalar;
  real_T c6_b_q[4];
  real_T c6_b_r[4];
  real_T c6_nargin = 2.0;
  real_T c6_nargout = 1.0;
  int32_T c6_i125;
  int32_T c6_i126;
  real_T c6_c_q[3];
  real_T c6_c_r[3];
  real_T c6_d_q[3];
  int32_T c6_i127;
  real_T c6_b_scalar[4];
  int32_T c6_i128;
  int32_T c6_i129;
  sf_debug_symbol_scope_push_eml(0U, 7U, 9U, c6_c_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_vec, 0U, c6_h_sf_marshallOut,
    c6_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_scalar, 1U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_q, MAX_uint32_T,
    c6_g_sf_marshallOut, c6_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_r, MAX_uint32_T,
    c6_g_sf_marshallOut, c6_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 4U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 5U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_q, 2U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_r, 3U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_qres, 6U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 60);
  for (c6_i125 = 0; c6_i125 < 4; c6_i125++) {
    c6_b_q[c6_i125] = c6_q[c6_i125];
  }

  sf_debug_symbol_switch(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 61);
  for (c6_i126 = 0; c6_i126 < 4; c6_i126++) {
    c6_b_r[c6_i126] = c6_r[c6_i126];
  }

  sf_debug_symbol_switch(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 64);
  c6_c_q[0] = c6_b_q[0] * c6_b_r[1];
  c6_c_q[1] = c6_b_q[0] * c6_b_r[2];
  c6_c_q[2] = c6_b_q[0] * c6_b_r[3];
  c6_c_r[0] = c6_b_r[0] * c6_b_q[1];
  c6_c_r[1] = c6_b_r[0] * c6_b_q[2];
  c6_c_r[2] = c6_b_r[0] * c6_b_q[3];
  c6_d_q[0] = c6_b_q[2] * c6_b_r[3] - c6_b_q[3] * c6_b_r[2];
  c6_d_q[1] = c6_b_q[3] * c6_b_r[1] - c6_b_q[1] * c6_b_r[3];
  c6_d_q[2] = c6_b_q[1] * c6_b_r[2] - c6_b_q[2] * c6_b_r[1];
  for (c6_i127 = 0; c6_i127 < 3; c6_i127++) {
    c6_vec[c6_i127] = (c6_c_q[c6_i127] + c6_c_r[c6_i127]) + c6_d_q[c6_i127];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 72);
  c6_scalar = ((c6_b_q[0] * c6_b_r[0] - c6_b_q[1] * c6_b_r[1]) - c6_b_q[2] *
               c6_b_r[2]) - c6_b_q[3] * c6_b_r[3];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 75);
  c6_b_scalar[0] = c6_scalar;
  for (c6_i128 = 0; c6_i128 < 3; c6_i128++) {
    c6_b_scalar[c6_i128 + 1] = c6_vec[c6_i128];
  }

  for (c6_i129 = 0; c6_i129 < 4; c6_i129++) {
    c6_qres[c6_i129] = c6_b_scalar[c6_i129];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -75);
  sf_debug_symbol_scope_pop();
}

static void c6_Kinematics(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_x[7], real_T c6_I[3], real_T c6_torque_c[3], real_T
  c6_results[7])
{
  uint32_T c6_debug_family_var_map[10];
  real_T c6_q[4];
  real_T c6_w[3];
  real_T c6_q_dot[4];
  real_T c6_w_dot[3];
  real_T c6_b_I[9];
  real_T c6_nargin = 3.0;
  real_T c6_nargout = 1.0;
  int32_T c6_i130;
  int32_T c6_i131;
  int32_T c6_i132;
  real_T c6_c_I[3];
  real_T c6_dv30[9];
  int32_T c6_i133;
  int32_T c6_i134;
  real_T c6_b_w[3];
  real_T c6_a[9];
  real_T c6_dv31[16];
  int32_T c6_i135;
  int32_T c6_i136;
  int32_T c6_i137;
  int32_T c6_i138;
  int32_T c6_i139;
  int32_T c6_i140;
  int32_T c6_i141;
  int32_T c6_i142;
  int32_T c6_i143;
  int32_T c6_i144;
  real_T c6_b_a[16];
  int32_T c6_i145;
  real_T c6_b[4];
  int32_T c6_i146;
  int32_T c6_i147;
  int32_T c6_i148;
  real_T c6_C[4];
  int32_T c6_i149;
  int32_T c6_i150;
  int32_T c6_i151;
  int32_T c6_i152;
  int32_T c6_i153;
  int32_T c6_i154;
  int32_T c6_i155;
  int32_T c6_i156;
  real_T c6_b_b[3];
  int32_T c6_i157;
  real_T c6_y[3];
  int32_T c6_i158;
  int32_T c6_i159;
  int32_T c6_i160;
  real_T c6_c_w[3];
  int32_T c6_i161;
  int32_T c6_i162;
  real_T c6_b_y[3];
  int32_T c6_i163;
  int32_T c6_i164;
  int32_T c6_i165;
  real_T c6_d_I[9];
  int32_T c6_i166;
  int32_T c6_i167;
  int32_T c6_i168;
  int32_T c6_i169;
  int32_T c6_i170;
  int32_T c6_i171;
  int32_T c6_i172;
  int32_T c6_i173;
  int32_T c6_i174;
  int32_T c6_i175;
  int32_T c6_i176;
  int32_T c6_i177;
  sf_debug_symbol_scope_push_eml(0U, 10U, 11U, c6_e_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_q, 0U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_w, 1U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_q_dot, 2U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_w_dot, 3U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_I, MAX_uint32_T,
    c6_i_sf_marshallOut, c6_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 5U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 6U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_x, 7U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_I, 4U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_torque_c, 8U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_results, 9U, c6_f_sf_marshallOut,
    c6_f_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 49);
  for (c6_i130 = 0; c6_i130 < 4; c6_i130++) {
    c6_q[c6_i130] = c6_x[c6_i130];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 50);
  for (c6_i131 = 0; c6_i131 < 3; c6_i131++) {
    c6_w[c6_i131] = c6_x[c6_i131 + 4];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 51);
  for (c6_i132 = 0; c6_i132 < 3; c6_i132++) {
    c6_c_I[c6_i132] = c6_I[c6_i132];
  }

  c6_diag(chartInstance, c6_c_I, c6_dv30);
  for (c6_i133 = 0; c6_i133 < 9; c6_i133++) {
    c6_b_I[c6_i133] = c6_dv30[c6_i133];
  }

  sf_debug_symbol_switch(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 53);
  for (c6_i134 = 0; c6_i134 < 3; c6_i134++) {
    c6_b_w[c6_i134] = c6_w[c6_i134];
  }

  c6_skew_matrix(chartInstance, c6_b_w, c6_a);
  c6_dv31[0] = 0.0;
  c6_i135 = 0;
  for (c6_i136 = 0; c6_i136 < 3; c6_i136++) {
    c6_dv31[c6_i135 + 4] = -c6_w[c6_i136];
    c6_i135 += 4;
  }

  for (c6_i137 = 0; c6_i137 < 3; c6_i137++) {
    c6_dv31[c6_i137 + 1] = c6_w[c6_i137];
  }

  c6_i138 = 0;
  c6_i139 = 0;
  for (c6_i140 = 0; c6_i140 < 3; c6_i140++) {
    for (c6_i141 = 0; c6_i141 < 3; c6_i141++) {
      c6_dv31[(c6_i141 + c6_i138) + 5] = -c6_a[c6_i141 + c6_i139];
    }

    c6_i138 += 4;
    c6_i139 += 3;
  }

  c6_i142 = 0;
  for (c6_i143 = 0; c6_i143 < 4; c6_i143++) {
    for (c6_i144 = 0; c6_i144 < 4; c6_i144++) {
      c6_b_a[c6_i144 + c6_i142] = 0.5 * c6_dv31[c6_i144 + c6_i142];
    }

    c6_i142 += 4;
  }

  for (c6_i145 = 0; c6_i145 < 4; c6_i145++) {
    c6_b[c6_i145] = c6_q[c6_i145];
  }

  c6_eml_scalar_eg(chartInstance);
  c6_eml_scalar_eg(chartInstance);
  for (c6_i146 = 0; c6_i146 < 4; c6_i146++) {
    c6_q_dot[c6_i146] = 0.0;
  }

  for (c6_i147 = 0; c6_i147 < 4; c6_i147++) {
    c6_q_dot[c6_i147] = 0.0;
  }

  for (c6_i148 = 0; c6_i148 < 4; c6_i148++) {
    c6_C[c6_i148] = c6_q_dot[c6_i148];
  }

  for (c6_i149 = 0; c6_i149 < 4; c6_i149++) {
    c6_q_dot[c6_i149] = c6_C[c6_i149];
  }

  for (c6_i150 = 0; c6_i150 < 4; c6_i150++) {
    c6_C[c6_i150] = c6_q_dot[c6_i150];
  }

  for (c6_i151 = 0; c6_i151 < 4; c6_i151++) {
    c6_q_dot[c6_i151] = c6_C[c6_i151];
  }

  for (c6_i152 = 0; c6_i152 < 4; c6_i152++) {
    c6_q_dot[c6_i152] = 0.0;
    c6_i153 = 0;
    for (c6_i154 = 0; c6_i154 < 4; c6_i154++) {
      c6_q_dot[c6_i152] += c6_b_a[c6_i153 + c6_i152] * c6_b[c6_i154];
      c6_i153 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 54);
  for (c6_i155 = 0; c6_i155 < 9; c6_i155++) {
    c6_a[c6_i155] = c6_b_I[c6_i155];
  }

  for (c6_i156 = 0; c6_i156 < 3; c6_i156++) {
    c6_b_b[c6_i156] = c6_w[c6_i156];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  for (c6_i157 = 0; c6_i157 < 3; c6_i157++) {
    c6_y[c6_i157] = 0.0;
    c6_i158 = 0;
    for (c6_i159 = 0; c6_i159 < 3; c6_i159++) {
      c6_y[c6_i157] += c6_a[c6_i158 + c6_i157] * c6_b_b[c6_i159];
      c6_i158 += 3;
    }
  }

  for (c6_i160 = 0; c6_i160 < 3; c6_i160++) {
    c6_c_w[c6_i160] = c6_w[c6_i160];
  }

  c6_skew_matrix(chartInstance, c6_c_w, c6_a);
  for (c6_i161 = 0; c6_i161 < 9; c6_i161++) {
    c6_a[c6_i161] = -c6_a[c6_i161];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  for (c6_i162 = 0; c6_i162 < 3; c6_i162++) {
    c6_b_y[c6_i162] = 0.0;
    c6_i163 = 0;
    for (c6_i164 = 0; c6_i164 < 3; c6_i164++) {
      c6_b_y[c6_i162] += c6_a[c6_i163 + c6_i162] * c6_y[c6_i164];
      c6_i163 += 3;
    }
  }

  for (c6_i165 = 0; c6_i165 < 9; c6_i165++) {
    c6_d_I[c6_i165] = c6_b_I[c6_i165];
  }

  c6_mpower(chartInstance, c6_d_I, c6_a);
  for (c6_i166 = 0; c6_i166 < 3; c6_i166++) {
    c6_b_y[c6_i166] += c6_torque_c[c6_i166];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  for (c6_i167 = 0; c6_i167 < 3; c6_i167++) {
    c6_w_dot[c6_i167] = 0.0;
  }

  for (c6_i168 = 0; c6_i168 < 3; c6_i168++) {
    c6_w_dot[c6_i168] = 0.0;
  }

  for (c6_i169 = 0; c6_i169 < 3; c6_i169++) {
    c6_b_b[c6_i169] = c6_w_dot[c6_i169];
  }

  for (c6_i170 = 0; c6_i170 < 3; c6_i170++) {
    c6_w_dot[c6_i170] = c6_b_b[c6_i170];
  }

  for (c6_i171 = 0; c6_i171 < 3; c6_i171++) {
    c6_b_b[c6_i171] = c6_w_dot[c6_i171];
  }

  for (c6_i172 = 0; c6_i172 < 3; c6_i172++) {
    c6_w_dot[c6_i172] = c6_b_b[c6_i172];
  }

  for (c6_i173 = 0; c6_i173 < 3; c6_i173++) {
    c6_w_dot[c6_i173] = 0.0;
    c6_i174 = 0;
    for (c6_i175 = 0; c6_i175 < 3; c6_i175++) {
      c6_w_dot[c6_i173] += c6_a[c6_i174 + c6_i173] * c6_b_y[c6_i175];
      c6_i174 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 56);
  for (c6_i176 = 0; c6_i176 < 4; c6_i176++) {
    c6_results[c6_i176] = c6_q_dot[c6_i176];
  }

  for (c6_i177 = 0; c6_i177 < 3; c6_i177++) {
    c6_results[c6_i177 + 4] = c6_w_dot[c6_i177];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -56);
  sf_debug_symbol_scope_pop();
}

static void c6_diag(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                    real_T c6_v[3], real_T c6_d[9])
{
  int32_T c6_i178;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_a;
  int32_T c6_c;
  for (c6_i178 = 0; c6_i178 < 9; c6_i178++) {
    c6_d[c6_i178] = 0.0;
  }

  c6_check_forloop_overflow_error(chartInstance);
  for (c6_j = 1; c6_j < 4; c6_j++) {
    c6_b_j = c6_j;
    c6_a = c6_b_j;
    c6_c = c6_a;
    c6_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 3, 2, 0) - 1)) - 1]
      = c6_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c6_skew_matrix(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_x[3], real_T c6_output[9])
{
  uint32_T c6_debug_family_var_map[4];
  real_T c6_nargin = 1.0;
  real_T c6_nargout = 1.0;
  sf_debug_symbol_scope_push_eml(0U, 4U, 4U, c6_d_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 0U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 1U, c6_e_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_x, 2U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_output, 3U, c6_i_sf_marshallOut,
    c6_i_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 43);
  c6_output[0] = 0.0;
  c6_output[3] = -c6_x[2];
  c6_output[6] = c6_x[1];
  c6_output[1] = c6_x[2];
  c6_output[4] = 0.0;
  c6_output[7] = -c6_x[0];
  c6_output[2] = -c6_x[1];
  c6_output[5] = c6_x[0];
  c6_output[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -43);
  sf_debug_symbol_scope_pop();
}

static void c6_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c6_mpower(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_a[9], real_T c6_c[9])
{
  int32_T c6_i179;
  real_T c6_b_a[9];
  for (c6_i179 = 0; c6_i179 < 9; c6_i179++) {
    c6_b_a[c6_i179] = c6_a[c6_i179];
  }

  c6_matrix_to_integer_power(chartInstance, c6_b_a, -1.0, c6_c);
}

static void c6_matrix_to_integer_power(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, real_T c6_a[9], real_T c6_b, real_T c6_c[9])
{
  real_T c6_x;
  real_T c6_e;
  boolean_T c6_firstmult;
  real_T c6_b_x;
  real_T c6_ed2;
  real_T c6_b_b;
  real_T c6_y;
  int32_T c6_i180;
  int32_T c6_i181;
  real_T c6_A[9];
  int32_T c6_i182;
  int32_T c6_i183;
  int32_T c6_i184;
  int32_T c6_i185;
  int32_T c6_i186;
  int32_T c6_i187;
  int32_T c6_i188;
  real_T c6_b_A[9];
  int32_T c6_i189;
  real_T c6_c_A[9];
  real_T c6_n1x;
  int32_T c6_i190;
  real_T c6_b_c[9];
  real_T c6_n1xinv;
  real_T c6_b_a;
  real_T c6_c_b;
  real_T c6_b_y;
  real_T c6_rc;
  real_T c6_c_x;
  boolean_T c6_d_b;
  real_T c6_d_x;
  int32_T c6_i191;
  static char_T c6_cv0[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c6_u[8];
  const mxArray *c6_c_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_d_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_e_y = NULL;
  real_T c6_d_u;
  const mxArray *c6_f_y = NULL;
  char_T c6_str[14];
  int32_T c6_i192;
  char_T c6_b_str[14];
  int32_T c6_i193;
  int32_T c6_i194;
  int32_T c6_i195;
  int32_T c6_i196;
  int32_T c6_i197;
  int32_T c6_i198;
  int32_T c6_i199;
  int32_T c6_k;
  int32_T c6_b_k;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  int32_T exitg1;
  c6_b_eml_scalar_eg(chartInstance);
  c6_x = c6_b;
  c6_e = muDoubleScalarAbs(c6_x);
  if (c6_e > 0.0) {
    c6_firstmult = TRUE;
    do {
      exitg1 = 0;
      c6_b_x = c6_e / 2.0;
      c6_ed2 = c6_b_x;
      c6_ed2 = muDoubleScalarFloor(c6_ed2);
      c6_b_b = c6_ed2;
      c6_y = 2.0 * c6_b_b;
      if (c6_y != c6_e) {
        if (c6_firstmult) {
          for (c6_i180 = 0; c6_i180 < 9; c6_i180++) {
            c6_c[c6_i180] = c6_a[c6_i180];
          }

          c6_firstmult = FALSE;
        } else {
          c6_c_eml_scalar_eg(chartInstance);
          c6_c_eml_scalar_eg(chartInstance);
          for (c6_i181 = 0; c6_i181 < 9; c6_i181++) {
            c6_A[c6_i181] = c6_c[c6_i181];
          }

          for (c6_i182 = 0; c6_i182 < 3; c6_i182++) {
            c6_i183 = 0;
            for (c6_i184 = 0; c6_i184 < 3; c6_i184++) {
              c6_c[c6_i183 + c6_i182] = 0.0;
              c6_i185 = 0;
              for (c6_i186 = 0; c6_i186 < 3; c6_i186++) {
                c6_c[c6_i183 + c6_i182] += c6_A[c6_i185 + c6_i182] *
                  c6_a[c6_i186 + c6_i183];
                c6_i185 += 3;
              }

              c6_i183 += 3;
            }
          }
        }
      }

      if (c6_ed2 == 0.0) {
        exitg1 = 1;
      } else {
        c6_e = c6_ed2;
        c6_c_eml_scalar_eg(chartInstance);
        c6_c_eml_scalar_eg(chartInstance);
        for (c6_i193 = 0; c6_i193 < 9; c6_i193++) {
          c6_A[c6_i193] = c6_a[c6_i193];
        }

        for (c6_i194 = 0; c6_i194 < 3; c6_i194++) {
          c6_i195 = 0;
          for (c6_i196 = 0; c6_i196 < 3; c6_i196++) {
            c6_a[c6_i195 + c6_i194] = 0.0;
            c6_i197 = 0;
            for (c6_i198 = 0; c6_i198 < 3; c6_i198++) {
              c6_a[c6_i195 + c6_i194] += c6_A[c6_i197 + c6_i194] * c6_A[c6_i198
                + c6_i195];
              c6_i197 += 3;
            }

            c6_i195 += 3;
          }
        }
      }
    } while (exitg1 == 0);

    if (c6_b < 0.0) {
      for (c6_i187 = 0; c6_i187 < 9; c6_i187++) {
        c6_A[c6_i187] = c6_c[c6_i187];
      }

      for (c6_i188 = 0; c6_i188 < 9; c6_i188++) {
        c6_b_A[c6_i188] = c6_A[c6_i188];
      }

      c6_inv3x3(chartInstance, c6_b_A, c6_c);
      for (c6_i189 = 0; c6_i189 < 9; c6_i189++) {
        c6_c_A[c6_i189] = c6_A[c6_i189];
      }

      c6_n1x = c6_norm(chartInstance, c6_c_A);
      for (c6_i190 = 0; c6_i190 < 9; c6_i190++) {
        c6_b_c[c6_i190] = c6_c[c6_i190];
      }

      c6_n1xinv = c6_norm(chartInstance, c6_b_c);
      c6_b_a = c6_n1x;
      c6_c_b = c6_n1xinv;
      c6_b_y = c6_b_a * c6_c_b;
      c6_rc = 1.0 / c6_b_y;
      guard1 = FALSE;
      guard2 = FALSE;
      if (c6_n1x == 0.0) {
        guard2 = TRUE;
      } else if (c6_n1xinv == 0.0) {
        guard2 = TRUE;
      } else if (c6_rc == 0.0) {
        guard1 = TRUE;
      } else {
        c6_c_x = c6_rc;
        c6_d_b = muDoubleScalarIsNaN(c6_c_x);
        guard3 = FALSE;
        if (c6_d_b) {
          guard3 = TRUE;
        } else {
          if (c6_rc < 2.2204460492503131E-16) {
            guard3 = TRUE;
          }
        }

        if (guard3 == TRUE) {
          c6_d_x = c6_rc;
          for (c6_i191 = 0; c6_i191 < 8; c6_i191++) {
            c6_u[c6_i191] = c6_cv0[c6_i191];
          }

          c6_c_y = NULL;
          sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1,
            8), FALSE);
          c6_b_u = 14.0;
          c6_d_y = NULL;
          sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c6_c_u = 6.0;
          c6_e_y = NULL;
          sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c6_d_u = c6_d_x;
          c6_f_y = NULL;
          sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_d_u, 0, 0U, 0U, 0U, 0),
                        FALSE);
          c6_n_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U,
            2U, 14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c6_c_y, 14, c6_d_y,
            14, c6_e_y), 14, c6_f_y), "sprintf", c6_str);
          for (c6_i192 = 0; c6_i192 < 14; c6_i192++) {
            c6_b_str[c6_i192] = c6_str[c6_i192];
          }

          c6_b_eml_warning(chartInstance, c6_b_str);
        }
      }

      if (guard2 == TRUE) {
        guard1 = TRUE;
      }

      if (guard1 == TRUE) {
        c6_eml_warning(chartInstance);
      }
    }
  } else {
    for (c6_i199 = 0; c6_i199 < 9; c6_i199++) {
      c6_c[c6_i199] = 0.0;
    }

    c6_check_forloop_overflow_error(chartInstance);
    for (c6_k = 1; c6_k < 4; c6_k++) {
      c6_b_k = c6_k;
      c6_c[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 3, 2, 0) - 1))
        - 1] = 1.0;
    }
  }
}

static void c6_b_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c6_c_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c6_inv3x3(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_x[9], real_T c6_y[9])
{
  int32_T c6_p1;
  int32_T c6_p2;
  int32_T c6_p3;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_absx11;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_absx21;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_absx31;
  real_T c6_t1;
  real_T c6_h_x;
  real_T c6_b_y;
  real_T c6_z;
  real_T c6_i_x;
  real_T c6_c_y;
  real_T c6_b_z;
  real_T c6_a;
  real_T c6_b;
  real_T c6_d_y;
  real_T c6_b_a;
  real_T c6_b_b;
  real_T c6_e_y;
  real_T c6_c_a;
  real_T c6_c_b;
  real_T c6_f_y;
  real_T c6_d_a;
  real_T c6_d_b;
  real_T c6_g_y;
  real_T c6_j_x;
  real_T c6_k_x;
  real_T c6_h_y;
  real_T c6_l_x;
  real_T c6_m_x;
  real_T c6_i_y;
  int32_T c6_itmp;
  real_T c6_n_x;
  real_T c6_j_y;
  real_T c6_c_z;
  real_T c6_e_a;
  real_T c6_e_b;
  real_T c6_k_y;
  real_T c6_f_a;
  real_T c6_f_b;
  real_T c6_l_y;
  real_T c6_o_x;
  real_T c6_m_y;
  real_T c6_t3;
  real_T c6_g_a;
  real_T c6_g_b;
  real_T c6_n_y;
  real_T c6_p_x;
  real_T c6_o_y;
  real_T c6_t2;
  int32_T c6_h_a;
  int32_T c6_c;
  real_T c6_i_a;
  real_T c6_h_b;
  real_T c6_p_y;
  real_T c6_j_a;
  real_T c6_i_b;
  real_T c6_q_y;
  real_T c6_q_x;
  real_T c6_r_y;
  real_T c6_d_z;
  int32_T c6_k_a;
  int32_T c6_b_c;
  int32_T c6_l_a;
  int32_T c6_c_c;
  real_T c6_r_x;
  real_T c6_s_y;
  real_T c6_m_a;
  real_T c6_j_b;
  real_T c6_t_y;
  real_T c6_s_x;
  real_T c6_u_y;
  int32_T c6_n_a;
  int32_T c6_d_c;
  real_T c6_o_a;
  real_T c6_k_b;
  real_T c6_v_y;
  real_T c6_p_a;
  real_T c6_l_b;
  real_T c6_w_y;
  real_T c6_t_x;
  real_T c6_x_y;
  real_T c6_e_z;
  int32_T c6_q_a;
  int32_T c6_e_c;
  int32_T c6_r_a;
  int32_T c6_f_c;
  real_T c6_y_y;
  real_T c6_s_a;
  real_T c6_m_b;
  real_T c6_ab_y;
  real_T c6_u_x;
  real_T c6_bb_y;
  int32_T c6_t_a;
  int32_T c6_g_c;
  real_T c6_u_a;
  real_T c6_n_b;
  real_T c6_cb_y;
  real_T c6_v_a;
  real_T c6_o_b;
  real_T c6_db_y;
  real_T c6_v_x;
  real_T c6_eb_y;
  real_T c6_f_z;
  int32_T c6_w_a;
  int32_T c6_h_c;
  int32_T c6_x_a;
  int32_T c6_i_c;
  boolean_T guard1 = FALSE;
  c6_p1 = 0;
  c6_p2 = 3;
  c6_p3 = 6;
  c6_b_x = c6_x[0];
  c6_c_x = c6_b_x;
  c6_absx11 = muDoubleScalarAbs(c6_c_x);
  c6_d_x = c6_x[1];
  c6_e_x = c6_d_x;
  c6_absx21 = muDoubleScalarAbs(c6_e_x);
  c6_f_x = c6_x[2];
  c6_g_x = c6_f_x;
  c6_absx31 = muDoubleScalarAbs(c6_g_x);
  guard1 = FALSE;
  if (c6_absx21 > c6_absx11) {
    if (c6_absx21 > c6_absx31) {
      c6_p1 = 3;
      c6_p2 = 0;
      c6_t1 = c6_x[0];
      c6_x[0] = c6_x[1];
      c6_x[1] = c6_t1;
      c6_t1 = c6_x[3];
      c6_x[3] = c6_x[4];
      c6_x[4] = c6_t1;
      c6_t1 = c6_x[6];
      c6_x[6] = c6_x[7];
      c6_x[7] = c6_t1;
    } else {
      guard1 = TRUE;
    }
  } else {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    if (c6_absx31 > c6_absx11) {
      c6_p1 = 6;
      c6_p3 = 0;
      c6_t1 = c6_x[0];
      c6_x[0] = c6_x[2];
      c6_x[2] = c6_t1;
      c6_t1 = c6_x[3];
      c6_x[3] = c6_x[5];
      c6_x[5] = c6_t1;
      c6_t1 = c6_x[6];
      c6_x[6] = c6_x[8];
      c6_x[8] = c6_t1;
    }
  }

  c6_h_x = c6_x[1];
  c6_b_y = c6_x[0];
  c6_z = c6_h_x / c6_b_y;
  c6_x[1] = c6_z;
  c6_i_x = c6_x[2];
  c6_c_y = c6_x[0];
  c6_b_z = c6_i_x / c6_c_y;
  c6_x[2] = c6_b_z;
  c6_a = c6_x[1];
  c6_b = c6_x[3];
  c6_d_y = c6_a * c6_b;
  c6_x[4] -= c6_d_y;
  c6_b_a = c6_x[2];
  c6_b_b = c6_x[3];
  c6_e_y = c6_b_a * c6_b_b;
  c6_x[5] -= c6_e_y;
  c6_c_a = c6_x[1];
  c6_c_b = c6_x[6];
  c6_f_y = c6_c_a * c6_c_b;
  c6_x[7] -= c6_f_y;
  c6_d_a = c6_x[2];
  c6_d_b = c6_x[6];
  c6_g_y = c6_d_a * c6_d_b;
  c6_x[8] -= c6_g_y;
  c6_j_x = c6_x[5];
  c6_k_x = c6_j_x;
  c6_h_y = muDoubleScalarAbs(c6_k_x);
  c6_l_x = c6_x[4];
  c6_m_x = c6_l_x;
  c6_i_y = muDoubleScalarAbs(c6_m_x);
  if (c6_h_y > c6_i_y) {
    c6_itmp = c6_p2;
    c6_p2 = c6_p3;
    c6_p3 = c6_itmp;
    c6_t1 = c6_x[1];
    c6_x[1] = c6_x[2];
    c6_x[2] = c6_t1;
    c6_t1 = c6_x[4];
    c6_x[4] = c6_x[5];
    c6_x[5] = c6_t1;
    c6_t1 = c6_x[7];
    c6_x[7] = c6_x[8];
    c6_x[8] = c6_t1;
  }

  c6_n_x = c6_x[5];
  c6_j_y = c6_x[4];
  c6_c_z = c6_n_x / c6_j_y;
  c6_x[5] = c6_c_z;
  c6_e_a = c6_x[5];
  c6_e_b = c6_x[7];
  c6_k_y = c6_e_a * c6_e_b;
  c6_x[8] -= c6_k_y;
  c6_f_a = c6_x[5];
  c6_f_b = c6_x[1];
  c6_l_y = c6_f_a * c6_f_b;
  c6_o_x = c6_l_y - c6_x[2];
  c6_m_y = c6_x[8];
  c6_t3 = c6_o_x / c6_m_y;
  c6_g_a = c6_x[7];
  c6_g_b = c6_t3;
  c6_n_y = c6_g_a * c6_g_b;
  c6_p_x = -(c6_x[1] + c6_n_y);
  c6_o_y = c6_x[4];
  c6_t2 = c6_p_x / c6_o_y;
  c6_h_a = c6_p1 + 1;
  c6_c = c6_h_a;
  c6_i_a = c6_x[3];
  c6_h_b = c6_t2;
  c6_p_y = c6_i_a * c6_h_b;
  c6_j_a = c6_x[6];
  c6_i_b = c6_t3;
  c6_q_y = c6_j_a * c6_i_b;
  c6_q_x = (1.0 - c6_p_y) - c6_q_y;
  c6_r_y = c6_x[0];
  c6_d_z = c6_q_x / c6_r_y;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_c), 1, 9, 1, 0) - 1] = c6_d_z;
  c6_k_a = c6_p1 + 2;
  c6_b_c = c6_k_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_b_c), 1, 9, 1, 0) - 1] = c6_t2;
  c6_l_a = c6_p1 + 3;
  c6_c_c = c6_l_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_c_c), 1, 9, 1, 0) - 1] = c6_t3;
  c6_r_x = -c6_x[5];
  c6_s_y = c6_x[8];
  c6_t3 = c6_r_x / c6_s_y;
  c6_m_a = c6_x[7];
  c6_j_b = c6_t3;
  c6_t_y = c6_m_a * c6_j_b;
  c6_s_x = 1.0 - c6_t_y;
  c6_u_y = c6_x[4];
  c6_t2 = c6_s_x / c6_u_y;
  c6_n_a = c6_p2 + 1;
  c6_d_c = c6_n_a;
  c6_o_a = c6_x[3];
  c6_k_b = c6_t2;
  c6_v_y = c6_o_a * c6_k_b;
  c6_p_a = c6_x[6];
  c6_l_b = c6_t3;
  c6_w_y = c6_p_a * c6_l_b;
  c6_t_x = -(c6_v_y + c6_w_y);
  c6_x_y = c6_x[0];
  c6_e_z = c6_t_x / c6_x_y;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_d_c), 1, 9, 1, 0) - 1] = c6_e_z;
  c6_q_a = c6_p2 + 2;
  c6_e_c = c6_q_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_e_c), 1, 9, 1, 0) - 1] = c6_t2;
  c6_r_a = c6_p2 + 3;
  c6_f_c = c6_r_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_f_c), 1, 9, 1, 0) - 1] = c6_t3;
  c6_y_y = c6_x[8];
  c6_t3 = 1.0 / c6_y_y;
  c6_s_a = -c6_x[7];
  c6_m_b = c6_t3;
  c6_ab_y = c6_s_a * c6_m_b;
  c6_u_x = c6_ab_y;
  c6_bb_y = c6_x[4];
  c6_t2 = c6_u_x / c6_bb_y;
  c6_t_a = c6_p3 + 1;
  c6_g_c = c6_t_a;
  c6_u_a = c6_x[3];
  c6_n_b = c6_t2;
  c6_cb_y = c6_u_a * c6_n_b;
  c6_v_a = c6_x[6];
  c6_o_b = c6_t3;
  c6_db_y = c6_v_a * c6_o_b;
  c6_v_x = -(c6_cb_y + c6_db_y);
  c6_eb_y = c6_x[0];
  c6_f_z = c6_v_x / c6_eb_y;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_g_c), 1, 9, 1, 0) - 1] = c6_f_z;
  c6_w_a = c6_p3 + 2;
  c6_h_c = c6_w_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_h_c), 1, 9, 1, 0) - 1] = c6_t2;
  c6_x_a = c6_p3 + 3;
  c6_i_c = c6_x_a;
  c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c6_i_c), 1, 9, 1, 0) - 1] = c6_t3;
}

static real_T c6_norm(SFc6_Control_System_LibraryInstanceStruct *chartInstance,
                      real_T c6_x[9])
{
  real_T c6_y;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_s;
  int32_T c6_i;
  real_T c6_b_i;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_b_y;
  real_T c6_d_x;
  boolean_T c6_b;
  boolean_T exitg1;
  c6_y = 0.0;
  c6_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c6_j < 3)) {
    c6_b_j = 1.0 + (real_T)c6_j;
    c6_s = 0.0;
    for (c6_i = 0; c6_i < 3; c6_i++) {
      c6_b_i = 1.0 + (real_T)c6_i;
      c6_b_x = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c6_b_i), 1, 3, 1, 0) + 3 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c6_b_j), 1, 3, 2, 0) - 1)) - 1];
      c6_c_x = c6_b_x;
      c6_b_y = muDoubleScalarAbs(c6_c_x);
      c6_s += c6_b_y;
    }

    c6_d_x = c6_s;
    c6_b = muDoubleScalarIsNaN(c6_d_x);
    if (c6_b) {
      c6_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c6_s > c6_y) {
        c6_y = c6_s;
      }

      c6_j++;
    }
  }

  return c6_y;
}

static void c6_eml_warning(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
  int32_T c6_i200;
  static char_T c6_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c6_u[27];
  const mxArray *c6_y = NULL;
  for (c6_i200 = 0; c6_i200 < 27; c6_i200++) {
    c6_u[c6_i200] = c6_varargin_1[c6_i200];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c6_y));
}

static void c6_b_eml_warning(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, char_T c6_varargin_2[14])
{
  int32_T c6_i201;
  static char_T c6_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c6_u[33];
  const mxArray *c6_y = NULL;
  int32_T c6_i202;
  char_T c6_b_u[14];
  const mxArray *c6_b_y = NULL;
  for (c6_i201 = 0; c6_i201 < 33; c6_i201++) {
    c6_u[c6_i201] = c6_varargin_1[c6_i201];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c6_i202 = 0; c6_i202 < 14; c6_i202++) {
    c6_b_u[c6_i202] = c6_varargin_2[c6_i202];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c6_y, 14, c6_b_y));
}

static void c6_d_eml_scalar_eg(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

static void c6_n_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_sprintf, const char_T *c6_identifier, char_T
  c6_y[14])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_sprintf), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_sprintf);
}

static void c6_o_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  char_T c6_y[14])
{
  char_T c6_cv1[14];
  int32_T c6_i203;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_cv1, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c6_i203 = 0; c6_i203 < 14; c6_i203++) {
    c6_y[c6_i203] = c6_cv1[c6_i203];
  }

  sf_mex_destroy(&c6_u);
}

static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_p_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i204;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i204, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i204;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_q_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_Control_System_Library, const
  char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_Control_System_Library), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_Control_System_Library);
  return c6_y;
}

static uint8_T c6_r_emlrt_marshallIn(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_Control_System_LibraryInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c6_Control_System_Library_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2189049019U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3400699352U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3248092251U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2834678953U);
}

mxArray *sf_c6_Control_System_Library_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("d90y3Bwm2SXuhV6HMPbnPF");
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

static const mxArray *sf_get_sim_state_info_c6_Control_System_Library(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[11],T\"q_i_s\",},{M[1],M[10],T\"w_s\",},{M[4],M[0],T\"q_i_c_km1\",S'l','i','p'{{M1x2[118 127],M[0],}}},{M[4],M[0],T\"w_c_km1\",S'l','i','p'{{M1x2[110 117],M[0],}}},{M[8],M[0],T\"is_active_c6_Control_System_Library\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_Control_System_Library_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_Control_System_LibraryInstanceStruct *chartInstance;
    chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_Control_System_LibraryMachineNumber_,
           6,
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
          _SFD_SET_DATA_PROPS(2,1,1,0,"Torque_c");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1119);
        _SFD_CV_INIT_EML_FCN(0,1,"skew_matrix",1121,-1,1281);
        _SFD_CV_INIT_EML_FCN(0,2,"Kinematics",1283,-1,1491);
        _SFD_CV_INIT_EML_FCN(0,3,"quatmultiply",1493,-1,2086);
        _SFD_CV_INIT_EML_FCN(0,4,"quatinv",2088,-1,2186);
        _SFD_CV_INIT_EML_IF(0,1,0,129,171,463,1114);

        {
          static int condStart[] = { 133, 153 };

          static int condEnd[] = { 149, 171 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,133,171,2,0,&(condStart[0]),&(condEnd[0]),
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
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)
            c6_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c6_Ts;
          real_T (*c6_I_c)[3];
          real_T (*c6_w_init_s)[3];
          real_T (*c6_Torque_c)[3];
          real_T (*c6_q0_i_s)[4];
          real_T (*c6_w_s)[3];
          real_T (*c6_q_i_s)[4];
          real_T (*c6_q_s_c)[4];
          c6_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c6_q_s_c = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 4);
          c6_q_i_s = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 2);
          c6_w_s = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_q0_i_s = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
          c6_Torque_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c6_w_init_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c6_I_c = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_I_c);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_w_init_s);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_Torque_c);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_q0_i_s);
          _SFD_SET_DATA_VALUE_PTR(4U, *c6_w_s);
          _SFD_SET_DATA_VALUE_PTR(5U, *c6_q_i_s);
          _SFD_SET_DATA_VALUE_PTR(6U, *c6_q_s_c);
          _SFD_SET_DATA_VALUE_PTR(7U, c6_Ts);
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
  return "veORVMYmH4lCYQAF3pRIpF";
}

static void sf_opaque_initialize_c6_Control_System_Library(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_Control_System_LibraryInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_Control_System_Library
    ((SFc6_Control_System_LibraryInstanceStruct*) chartInstanceVar);
  initialize_c6_Control_System_Library
    ((SFc6_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c6_Control_System_Library(void *chartInstanceVar)
{
  enable_c6_Control_System_Library((SFc6_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_Control_System_Library(void *chartInstanceVar)
{
  disable_c6_Control_System_Library((SFc6_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_Control_System_Library(void *chartInstanceVar)
{
  sf_c6_Control_System_Library((SFc6_Control_System_LibraryInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_Control_System_Library
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_Control_System_Library
    ((SFc6_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_Control_System_Library();/* state var info */
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

extern void sf_internal_set_sim_state_c6_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_Control_System_Library();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_Control_System_Library
    ((SFc6_Control_System_LibraryInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_Control_System_Library
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c6_Control_System_Library(S);
}

static void sf_opaque_set_sim_state_c6_Control_System_Library(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c6_Control_System_Library(S, st);
}

static void sf_opaque_terminate_c6_Control_System_Library(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_Control_System_LibraryInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c6_Control_System_Library
      ((SFc6_Control_System_LibraryInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_Control_System_Library_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_Control_System_Library
    ((SFc6_Control_System_LibraryInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_Control_System_Library(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_Control_System_Library
      ((SFc6_Control_System_LibraryInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Control_System_Library_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,6,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,2);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1313734112U));
  ssSetChecksum1(S,(3425997935U));
  ssSetChecksum2(S,(1639507803U));
  ssSetChecksum3(S,(1257572342U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_Control_System_Library(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_Control_System_Library(SimStruct *S)
{
  SFc6_Control_System_LibraryInstanceStruct *chartInstance;
  chartInstance = (SFc6_Control_System_LibraryInstanceStruct *)malloc(sizeof
    (SFc6_Control_System_LibraryInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_Control_System_LibraryInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_Control_System_Library;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_Control_System_Library;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_Control_System_Library;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c6_Control_System_Library;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_Control_System_Library;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_Control_System_Library;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_Control_System_Library;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_Control_System_Library;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_Control_System_Library;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_Control_System_Library;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_Control_System_Library;
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

void c6_Control_System_Library_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_Control_System_Library(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_Control_System_Library(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_Control_System_Library(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_Control_System_Library_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
