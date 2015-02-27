/* Include files */

#include <stddef.h>
#include "blas.h"
#include "UKF_10hz_sfun.h"
#include "c2_UKF_10hz.h"
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
static const char * c2_debug_family_names[49] = { "P_kk", "L", "w_bias",
  "DX_sigma", "dX_km1km1", "X_km1km1_temp", "X_km1km1", "Torque_c", "X_kkm1",
  "W0_m", "W0_c", "Wi_cm", "x_kkm1", "dX_kkm1_temp", "dX_kkm1", "dx_kkm1",
  "P_kkm1", "i", "Z_kkm1_control", "z_kkm1_control", "P_zkzk", "P_xkzk", "K_k",
  "B_control", "w_control", "dx_kk", "nargin", "nargout", "B_ECI", "B_sat",
  "w_gyro", "Torque_s", "Ts", "Attitude_sensor", "w_sensor", "w_bias_sensor",
  "Kalman_Gain", "noise_covarience", "P_km1km1", "x_km1km1", "x_kk", "R_k",
  "Q_k", "Lambda", "alpha", "K", "Beta", "q_s_c", "I_c" };

static const char * c2_b_debug_family_names[5] = { "q_conj", "nargin", "nargout",
  "qin", "qinv" };

static const char * c2_c_debug_family_names[7] = { "vec", "scalar", "q", "r",
  "nargin", "nargout", "qres" };

static const char * c2_d_debug_family_names[6] = { "rvec_temp", "nargin",
  "nargout", "vec", "q_s_c", "rvec" };

static const char * c2_e_debug_family_names[7] = { "vec", "scalar", "q", "r",
  "nargin", "nargout", "qres" };

static const char * c2_f_debug_family_names[4] = { "nargin", "nargout", "x",
  "output" };

static const char * c2_g_debug_family_names[11] = { "q", "w", "q_dot", "w_dot",
  "w_bias_dot", "I", "nargin", "nargout", "x", "torque_c", "results" };

static const char * c2_h_debug_family_names[12] = { "idx", "k1", "k2", "k3",
  "k4", "nargin", "nargout", "X_km1km1", "I_c", "Torque_c", "Ts", "X_kkm1" };

static const char * c2_i_debug_family_names[7] = { "idx", "Z_kkm1_temp",
  "nargin", "nargout", "X_kkm1", "B_ref", "Z_kkm1" };

static const char * c2_j_debug_family_names[6] = { "rvec_temp", "nargin",
  "nargout", "vec", "q_s_c", "rvec" };

/* Function Declarations */
static void initialize_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void initialize_params_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance);
static void enable_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void disable_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance);
static void set_sim_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_st);
static void finalize_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void sf_gateway_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_chartstep_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void initSimStructsc2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_I_c, const char_T *c2_identifier, real_T c2_y[3]);
static void c2_b_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_q_s_c, const char_T *c2_identifier, real_T c2_y[4]);
static void c2_d_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_e_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Beta, const char_T *c2_identifier);
static real_T c2_f_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_g_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_K, const char_T *c2_identifier);
static real_T c2_h_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_i_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_alpha, const char_T *c2_identifier);
static real_T c2_j_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_k_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Lambda, const char_T *c2_identifier);
static real_T c2_l_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_m_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Q_k, const char_T *c2_identifier, real_T c2_y[81]);
static void c2_n_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_o_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_R_k, const char_T *c2_identifier, real_T c2_y[36]);
static void c2_p_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_q_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_x_kk, const char_T *c2_identifier, real_T c2_y[10]);
static void c2_r_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_s_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_x_km1km1, const char_T *c2_identifier, real_T c2_y[10]);
static void c2_t_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_u_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_P_km1km1, const char_T *c2_identifier, real_T c2_y[81]);
static void c2_v_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_w_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_noise_covarience, const char_T *c2_identifier, real_T c2_y
  [36]);
static void c2_x_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_y_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_Kalman_Gain, const char_T *c2_identifier, real_T c2_y[54]);
static void c2_ab_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[54]);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_bb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_w_bias_sensor, const char_T *c2_identifier, real_T c2_y[3]);
static void c2_cb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_db_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_Attitude_sensor, const char_T *c2_identifier, real_T c2_y[4]);
static void c2_eb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_fb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_gb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9]);
static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_hb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6]);
static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_ib_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[114]);
static void c2_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_t_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_jb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81]);
static void c2_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_u_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_kb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[171]);
static void c2_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_v_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_lb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[76]);
static void c2_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_w_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_mb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10]);
static void c2_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_x_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_nb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[190]);
static void c2_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_y_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_ob_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4]);
static void c2_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_pb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_qb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[76]);
static void c2_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_rb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[19]);
static void c2_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_db_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_sb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[57]);
static void c2_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_tb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9]);
static void c2_eb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u);
static void c2_b_info_helper(const mxArray **c2_info);
static void c2_c_info_helper(const mxArray **c2_info);
static void c2_d_info_helper(const mxArray **c2_info);
static void c2_e_info_helper(const mxArray **c2_info);
static void c2_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a[3],
                     real_T c2_y[3]);
static void c2_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static real_T c2_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[3]);
static void c2_eml_switch_helper(SFc2_UKF_10hzInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x);
static void c2_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_rdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[3],
  real_T c2_y, real_T c2_z[3]);
static void c2_RotateVecSensor2Cont(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_vec[3], real_T c2_b_q_s_c[4], real_T c2_rvec[3]);
static void c2_quatinv(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_qin[4], real_T c2_qinv[4]);
static void c2_b_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a[4],
  real_T c2_y[4]);
static real_T c2_b_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[4]);
static void c2_b_rdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[4], real_T c2_y, real_T c2_z[4]);
static void c2_quatmultiply(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_q[4], real_T c2_r[4], real_T c2_qres[4]);
static real_T c2_mpower(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a);
static void c2_b_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_eml_matlab_zpotrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_b_A[81], int32_T *c2_info);
static real_T c2_eml_xdotc(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[81], int32_T c2_ix0, real_T c2_y[81], int32_T c2_iy0);
static void c2_check_forloop_overflow_error(SFc2_UKF_10hzInstanceStruct
  *chartInstance, boolean_T c2_overflow);
static void c2_eml_xgemv(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0, real_T c2_y[81], int32_T
  c2_iy0, real_T c2_b_y[81]);
static void c2_below_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_b_below_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_c_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_c_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a
  [57], real_T c2_y[57]);
static void c2_c_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[57],
                     real_T c2_y[19]);
static void c2_b_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[19],
                      real_T c2_b_x[19]);
static void c2_b_quatmultiply(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_q[76], real_T c2_r[4], real_T c2_qres[76]);
static void c2_RK4(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
                   c2_X_km1km1[190], real_T c2_b_I_c[3], real_T c2_Torque_c[3],
                   real_T c2_Ts, real_T c2_X_kkm1[190]);
static void c2_Kinematics(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[10], real_T c2_I[3], real_T c2_torque_c[3], real_T c2_results[10]);
static void c2_diag(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_v[3],
                    real_T c2_d[9]);
static void c2_skew_matrix(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[3], real_T c2_output[9]);
static void c2_b_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_b_mpower(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a
  [9], real_T c2_c[9]);
static void c2_inv3x3(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[9],
                      real_T c2_y[9]);
static real_T c2_norm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[9]);
static void c2_eml_warning(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_eps(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_b_eml_warning(SFc2_UKF_10hzInstanceStruct *chartInstance, char_T
  c2_varargin_2[14]);
static void c2_c_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_d_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[190],
                     real_T c2_y[10]);
static void c2_e_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[171],
                     real_T c2_y[9]);
static void c2_SensorModel(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_X_kkm1[190], real_T c2_B_ref[3], real_T c2_Z_kkm1[114]);
static void c2_f_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[114],
                     real_T c2_y[6]);
static void c2_mrdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_A
  [54], real_T c2_B[36], real_T c2_y[54]);
static void c2_eml_matlab_zgetrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[36], real_T c2_b_A[36], int32_T c2_ipiv[6], int32_T *c2_info);
static int32_T c2_eml_ixamax(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[36], int32_T c2_ix0);
static void c2_b_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_eml_xgeru(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[36], int32_T c2_ia0, real_T c2_b_A[36]);
static void c2_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54], real_T c2_b_B[54]);
static void c2_c_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_scalarEg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_b_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54], real_T c2_b_B[54]);
static void c2_d_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_e_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_f_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance);
static void c2_RotateVecCont2Sensor(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_vec[3], real_T c2_b_q_s_c[4], real_T c2_rvec[3]);
static void c2_ub_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14]);
static void c2_vb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14]);
static const mxArray *c2_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_wb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_fb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_xb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_UKF_10hz, const char_T *c2_identifier);
static uint8_T c2_yb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T *c2_x);
static int32_T c2_b_eml_matlab_zpotrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[81]);
static void c2_b_eml_xgemv(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0, real_T c2_y[81], int32_T
  c2_iy0);
static void c2_d_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[19]);
static void c2_b_eml_matlab_zgetrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[36], int32_T c2_ipiv[6], int32_T *c2_info);
static void c2_b_eml_xgeru(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[36], int32_T c2_ia0);
static void c2_c_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54]);
static void c2_d_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54]);
static void init_dsm_address_info(SFc2_UKF_10hzInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_P_km1km1_not_empty = false;
  chartInstance->c2_x_km1km1_not_empty = false;
  chartInstance->c2_x_kk_not_empty = false;
  chartInstance->c2_R_k_not_empty = false;
  chartInstance->c2_Q_k_not_empty = false;
  chartInstance->c2_Lambda_not_empty = false;
  chartInstance->c2_alpha_not_empty = false;
  chartInstance->c2_K_not_empty = false;
  chartInstance->c2_Beta_not_empty = false;
  chartInstance->c2_q_s_c_not_empty = false;
  chartInstance->c2_I_c_not_empty = false;
  chartInstance->c2_is_active_c2_UKF_10hz = 0U;
}

static void initialize_params_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[4];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i1;
  real_T c2_b_u[54];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i2;
  real_T c2_c_u[36];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i3;
  real_T c2_d_u[3];
  const mxArray *c2_e_y = NULL;
  int32_T c2_i4;
  real_T c2_e_u[3];
  const mxArray *c2_f_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  int32_T c2_i5;
  real_T c2_g_u[3];
  const mxArray *c2_h_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  int32_T c2_i6;
  real_T c2_j_u[81];
  const mxArray *c2_k_y = NULL;
  int32_T c2_i7;
  real_T c2_k_u[81];
  const mxArray *c2_l_y = NULL;
  int32_T c2_i8;
  real_T c2_l_u[36];
  const mxArray *c2_m_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_m_u;
  const mxArray *c2_n_y = NULL;
  int32_T c2_i9;
  real_T c2_n_u[4];
  const mxArray *c2_o_y = NULL;
  int32_T c2_i10;
  real_T c2_o_u[10];
  const mxArray *c2_p_y = NULL;
  int32_T c2_i11;
  real_T c2_p_u[10];
  const mxArray *c2_q_y = NULL;
  uint8_T c2_e_hoistedGlobal;
  uint8_T c2_q_u;
  const mxArray *c2_r_y = NULL;
  real_T (*c2_w_sensor)[3];
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_noise_covarience)[36];
  real_T (*c2_Kalman_Gain)[54];
  real_T (*c2_Attitude_sensor)[4];
  c2_noise_covarience = (real_T (*)[36])ssGetOutputPortSignal(chartInstance->S,
    5);
  c2_Kalman_Gain = (real_T (*)[54])ssGetOutputPortSignal(chartInstance->S, 4);
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(17, 1), false);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    c2_u[c2_i0] = (*c2_Attitude_sensor)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i1 = 0; c2_i1 < 54; c2_i1++) {
    c2_b_u[c2_i1] = (*c2_Kalman_Gain)[c2_i1];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 2, 9, 6),
                false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i2 = 0; c2_i2 < 36; c2_i2++) {
    c2_c_u[c2_i2] = (*c2_noise_covarience)[c2_i2];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 2, 6, 6),
                false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    c2_d_u[c2_i3] = (*c2_w_bias_sensor)[c2_i3];
  }

  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_d_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_e_u[c2_i4] = (*c2_w_sensor)[c2_i4];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_e_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_hoistedGlobal = chartInstance->c2_Beta;
  c2_f_u = c2_hoistedGlobal;
  c2_g_y = NULL;
  if (!chartInstance->c2_Beta_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    c2_g_u[c2_i5] = chartInstance->c2_I_c[c2_i5];
  }

  c2_h_y = NULL;
  if (!chartInstance->c2_I_c_not_empty) {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", c2_g_u, 0, 0U, 1U, 0U, 2, 1, 3),
                  false);
  }

  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_b_hoistedGlobal = chartInstance->c2_K;
  c2_h_u = c2_b_hoistedGlobal;
  c2_i_y = NULL;
  if (!chartInstance->c2_K_not_empty) {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_c_hoistedGlobal = chartInstance->c2_Lambda;
  c2_i_u = c2_c_hoistedGlobal;
  c2_j_y = NULL;
  if (!chartInstance->c2_Lambda_not_empty) {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 8, c2_j_y);
  for (c2_i6 = 0; c2_i6 < 81; c2_i6++) {
    c2_j_u[c2_i6] = chartInstance->c2_P_km1km1[c2_i6];
  }

  c2_k_y = NULL;
  if (!chartInstance->c2_P_km1km1_not_empty) {
    sf_mex_assign(&c2_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_k_y, sf_mex_create("y", c2_j_u, 0, 0U, 1U, 0U, 2, 9, 9),
                  false);
  }

  sf_mex_setcell(c2_y, 9, c2_k_y);
  for (c2_i7 = 0; c2_i7 < 81; c2_i7++) {
    c2_k_u[c2_i7] = chartInstance->c2_Q_k[c2_i7];
  }

  c2_l_y = NULL;
  if (!chartInstance->c2_Q_k_not_empty) {
    sf_mex_assign(&c2_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_l_y, sf_mex_create("y", c2_k_u, 0, 0U, 1U, 0U, 2, 9, 9),
                  false);
  }

  sf_mex_setcell(c2_y, 10, c2_l_y);
  for (c2_i8 = 0; c2_i8 < 36; c2_i8++) {
    c2_l_u[c2_i8] = chartInstance->c2_R_k[c2_i8];
  }

  c2_m_y = NULL;
  if (!chartInstance->c2_R_k_not_empty) {
    sf_mex_assign(&c2_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_m_y, sf_mex_create("y", c2_l_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  false);
  }

  sf_mex_setcell(c2_y, 11, c2_m_y);
  c2_d_hoistedGlobal = chartInstance->c2_alpha;
  c2_m_u = c2_d_hoistedGlobal;
  c2_n_y = NULL;
  if (!chartInstance->c2_alpha_not_empty) {
    sf_mex_assign(&c2_n_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_m_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c2_y, 12, c2_n_y);
  for (c2_i9 = 0; c2_i9 < 4; c2_i9++) {
    c2_n_u[c2_i9] = chartInstance->c2_q_s_c[c2_i9];
  }

  c2_o_y = NULL;
  if (!chartInstance->c2_q_s_c_not_empty) {
    sf_mex_assign(&c2_o_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_o_y, sf_mex_create("y", c2_n_u, 0, 0U, 1U, 0U, 1, 4),
                  false);
  }

  sf_mex_setcell(c2_y, 13, c2_o_y);
  for (c2_i10 = 0; c2_i10 < 10; c2_i10++) {
    c2_o_u[c2_i10] = chartInstance->c2_x_kk[c2_i10];
  }

  c2_p_y = NULL;
  if (!chartInstance->c2_x_kk_not_empty) {
    sf_mex_assign(&c2_p_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_p_y, sf_mex_create("y", c2_o_u, 0, 0U, 1U, 0U, 1, 10),
                  false);
  }

  sf_mex_setcell(c2_y, 14, c2_p_y);
  for (c2_i11 = 0; c2_i11 < 10; c2_i11++) {
    c2_p_u[c2_i11] = chartInstance->c2_x_km1km1[c2_i11];
  }

  c2_q_y = NULL;
  if (!chartInstance->c2_x_km1km1_not_empty) {
    sf_mex_assign(&c2_q_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_q_y, sf_mex_create("y", c2_p_u, 0, 0U, 1U, 0U, 1, 10),
                  false);
  }

  sf_mex_setcell(c2_y, 15, c2_q_y);
  c2_e_hoistedGlobal = chartInstance->c2_is_active_c2_UKF_10hz;
  c2_q_u = c2_e_hoistedGlobal;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_q_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 16, c2_r_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i12;
  real_T c2_dv1[54];
  int32_T c2_i13;
  real_T c2_dv2[36];
  int32_T c2_i14;
  real_T c2_dv3[3];
  int32_T c2_i15;
  real_T c2_dv4[3];
  int32_T c2_i16;
  real_T c2_dv5[3];
  int32_T c2_i17;
  real_T c2_dv6[81];
  int32_T c2_i18;
  real_T c2_dv7[81];
  int32_T c2_i19;
  real_T c2_dv8[36];
  int32_T c2_i20;
  real_T c2_dv9[4];
  int32_T c2_i21;
  real_T c2_dv10[10];
  int32_T c2_i22;
  real_T c2_dv11[10];
  int32_T c2_i23;
  real_T (*c2_Attitude_sensor)[4];
  real_T (*c2_Kalman_Gain)[54];
  real_T (*c2_noise_covarience)[36];
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_w_sensor)[3];
  c2_noise_covarience = (real_T (*)[36])ssGetOutputPortSignal(chartInstance->S,
    5);
  c2_Kalman_Gain = (real_T (*)[54])ssGetOutputPortSignal(chartInstance->S, 4);
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_db_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
    "Attitude_sensor", c2_dv0);
  for (c2_i12 = 0; c2_i12 < 4; c2_i12++) {
    (*c2_Attitude_sensor)[c2_i12] = c2_dv0[c2_i12];
  }

  c2_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "Kalman_Gain", c2_dv1);
  for (c2_i13 = 0; c2_i13 < 54; c2_i13++) {
    (*c2_Kalman_Gain)[c2_i13] = c2_dv1[c2_i13];
  }

  c2_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
                        "noise_covarience", c2_dv2);
  for (c2_i14 = 0; c2_i14 < 36; c2_i14++) {
    (*c2_noise_covarience)[c2_i14] = c2_dv2[c2_i14];
  }

  c2_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)),
    "w_bias_sensor", c2_dv3);
  for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
    (*c2_w_bias_sensor)[c2_i15] = c2_dv3[c2_i15];
  }

  c2_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
    "w_sensor", c2_dv4);
  for (c2_i16 = 0; c2_i16 < 3; c2_i16++) {
    (*c2_w_sensor)[c2_i16] = c2_dv4[c2_i16];
  }

  chartInstance->c2_Beta = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 5)), "Beta");
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 6)), "I_c",
                      c2_dv5);
  for (c2_i17 = 0; c2_i17 < 3; c2_i17++) {
    chartInstance->c2_I_c[c2_i17] = c2_dv5[c2_i17];
  }

  chartInstance->c2_K = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 7)), "K");
  chartInstance->c2_Lambda = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 8)), "Lambda");
  c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)),
                        "P_km1km1", c2_dv6);
  for (c2_i18 = 0; c2_i18 < 81; c2_i18++) {
    chartInstance->c2_P_km1km1[c2_i18] = c2_dv6[c2_i18];
  }

  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 10)),
                        "Q_k", c2_dv7);
  for (c2_i19 = 0; c2_i19 < 81; c2_i19++) {
    chartInstance->c2_Q_k[c2_i19] = c2_dv7[c2_i19];
  }

  c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 11)),
                        "R_k", c2_dv8);
  for (c2_i20 = 0; c2_i20 < 36; c2_i20++) {
    chartInstance->c2_R_k[c2_i20] = c2_dv8[c2_i20];
  }

  chartInstance->c2_alpha = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 12)), "alpha");
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 13)),
                        "q_s_c", c2_dv9);
  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    chartInstance->c2_q_s_c[c2_i21] = c2_dv9[c2_i21];
  }

  c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 14)),
                        "x_kk", c2_dv10);
  for (c2_i22 = 0; c2_i22 < 10; c2_i22++) {
    chartInstance->c2_x_kk[c2_i22] = c2_dv10[c2_i22];
  }

  c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 15)),
                        "x_km1km1", c2_dv11);
  for (c2_i23 = 0; c2_i23 < 10; c2_i23++) {
    chartInstance->c2_x_km1km1[c2_i23] = c2_dv11[c2_i23];
  }

  chartInstance->c2_is_active_c2_UKF_10hz = c2_xb_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 16)), "is_active_c2_UKF_10hz");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_UKF_10hz(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  int32_T c2_i32;
  real_T *c2_Ts;
  real_T (*c2_noise_covarience)[36];
  real_T (*c2_Kalman_Gain)[54];
  real_T (*c2_Torque_s)[3];
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_w_gyro)[3];
  real_T (*c2_B_sat)[3];
  real_T (*c2_w_sensor)[3];
  real_T (*c2_Attitude_sensor)[4];
  real_T (*c2_B_ECI)[3];
  c2_noise_covarience = (real_T (*)[36])ssGetOutputPortSignal(chartInstance->S,
    5);
  c2_Kalman_Gain = (real_T (*)[54])ssGetOutputPortSignal(chartInstance->S, 4);
  c2_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_B_ECI = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
    _SFD_DATA_RANGE_CHECK((*c2_B_ECI)[c2_i24], 0U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_UKF_10hz(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_UKF_10hzMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i25 = 0; c2_i25 < 4; c2_i25++) {
    _SFD_DATA_RANGE_CHECK((*c2_Attitude_sensor)[c2_i25], 1U);
  }

  for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_sensor)[c2_i26], 2U);
  }

  for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
    _SFD_DATA_RANGE_CHECK((*c2_B_sat)[c2_i27], 3U);
  }

  for (c2_i28 = 0; c2_i28 < 3; c2_i28++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_gyro)[c2_i28], 4U);
  }

  for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_bias_sensor)[c2_i29], 5U);
  }

  for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
    _SFD_DATA_RANGE_CHECK((*c2_Torque_s)[c2_i30], 6U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_Ts, 7U);
  for (c2_i31 = 0; c2_i31 < 54; c2_i31++) {
    _SFD_DATA_RANGE_CHECK((*c2_Kalman_Gain)[c2_i31], 8U);
  }

  for (c2_i32 = 0; c2_i32 < 36; c2_i32++) {
    _SFD_DATA_RANGE_CHECK((*c2_noise_covarience)[c2_i32], 9U);
  }
}

static void c2_chartstep_c2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  int32_T c2_i33;
  real_T c2_B_ECI[3];
  int32_T c2_i34;
  real_T c2_B_sat[3];
  int32_T c2_i35;
  real_T c2_w_gyro[3];
  int32_T c2_i36;
  real_T c2_Torque_s[3];
  real_T c2_Ts;
  uint32_T c2_debug_family_var_map[49];
  real_T c2_P_kk[81];
  real_T c2_L;
  real_T c2_w_bias[3];
  real_T c2_DX_sigma[81];
  real_T c2_dX_km1km1[171];
  real_T c2_X_km1km1_temp[76];
  real_T c2_X_km1km1[190];
  real_T c2_Torque_c[3];
  real_T c2_X_kkm1[190];
  real_T c2_W0_m;
  real_T c2_W0_c;
  real_T c2_Wi_cm;
  real_T c2_x_kkm1[10];
  real_T c2_dX_kkm1_temp[76];
  real_T c2_dX_kkm1[171];
  real_T c2_dx_kkm1[9];
  real_T c2_P_kkm1[81];
  real_T c2_i;
  real_T c2_Z_kkm1_control[114];
  real_T c2_z_kkm1_control[6];
  real_T c2_P_zkzk[36];
  real_T c2_P_xkzk[54];
  real_T c2_K_k[54];
  real_T c2_B_control[3];
  real_T c2_w_control[3];
  real_T c2_dx_kk[9];
  real_T c2_nargin = 5.0;
  real_T c2_nargout = 5.0;
  real_T c2_Attitude_sensor[4];
  real_T c2_w_sensor[3];
  real_T c2_w_bias_sensor[3];
  real_T c2_Kalman_Gain[54];
  real_T c2_noise_covarience[36];
  int32_T c2_i37;
  real_T c2_b_B_sat[3];
  real_T c2_a[3];
  int32_T c2_i38;
  real_T c2_b_a[3];
  real_T c2_d0;
  int32_T c2_i39;
  real_T c2_c_B_sat[3];
  real_T c2_dv12[3];
  int32_T c2_i40;
  int32_T c2_i41;
  real_T c2_b_B_ECI[3];
  int32_T c2_i42;
  real_T c2_c_a[3];
  real_T c2_d1;
  int32_T c2_i43;
  real_T c2_c_B_ECI[3];
  real_T c2_dv13[3];
  int32_T c2_i44;
  int32_T c2_i45;
  static real_T c2_dv14[4] = { 1.0, 0.0, 0.0, 0.0 };

  int32_T c2_i46;
  static real_T c2_dv15[3] = { 3.4, 3.4, 1.9 };

  int32_T c2_i47;
  static real_T c2_dv16[81] = { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  int32_T c2_i51;
  real_T c2_b_w_gyro[3];
  int32_T c2_i52;
  real_T c2_dv17[4];
  real_T c2_dv18[3];
  int32_T c2_i53;
  int32_T c2_i54;
  int32_T c2_i55;
  int32_T c2_i56;
  static real_T c2_dv19[36] = { 5.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5 };

  int32_T c2_i57;
  static real_T c2_dv20[81] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8 };

  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  real_T c2_b_hoistedGlobal;
  int32_T c2_i63;
  real_T c2_c_hoistedGlobal[81];
  real_T c2_d_a;
  int32_T c2_i64;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_info;
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_jmax;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_b_jmax;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_c_j;
  int32_T c2_g_a;
  int32_T c2_h_a;
  int32_T c2_i65;
  int32_T c2_c_jmax;
  int32_T c2_i_a;
  int32_T c2_c_b;
  int32_T c2_j_a;
  int32_T c2_d_b;
  boolean_T c2_b_overflow;
  int32_T c2_b_i;
  int32_T c2_c_i;
  int32_T c2_i66;
  int32_T c2_i67;
  real_T c2_e_b[9];
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  real_T c2_b_dX_km1km1[57];
  real_T c2_y[57];
  int32_T c2_i80;
  real_T c2_b_y[57];
  real_T c2_dv21[19];
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  real_T c2_d_hoistedGlobal[10];
  int32_T c2_i89;
  real_T c2_e_hoistedGlobal[10];
  int32_T c2_i90;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  real_T c2_c_y[57];
  int32_T c2_i99;
  real_T c2_b_X_km1km1_temp[76];
  int32_T c2_i100;
  real_T c2_f_hoistedGlobal[4];
  real_T c2_dv22[76];
  int32_T c2_i101;
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  int32_T c2_i110;
  int32_T c2_i111;
  int32_T c2_i112;
  int32_T c2_i113;
  int32_T c2_i114;
  int32_T c2_i115;
  real_T c2_b_Torque_s[3];
  int32_T c2_i116;
  real_T c2_dv23[4];
  real_T c2_dv24[3];
  int32_T c2_i117;
  int32_T c2_i118;
  real_T c2_b_X_km1km1[190];
  int32_T c2_i119;
  real_T c2_dv25[3];
  int32_T c2_i120;
  real_T c2_b_Torque_c[3];
  real_T c2_dv26[190];
  int32_T c2_i121;
  real_T c2_g_hoistedGlobal;
  real_T c2_h_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_d_y;
  real_T c2_b_x;
  real_T c2_e_y;
  real_T c2_c_x;
  real_T c2_f_y;
  real_T c2_i_hoistedGlobal;
  real_T c2_j_hoistedGlobal;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_g_y;
  real_T c2_e_x;
  real_T c2_h_y;
  real_T c2_f_x;
  real_T c2_i_y;
  real_T c2_j_y;
  real_T c2_k_hoistedGlobal;
  real_T c2_c_B;
  real_T c2_k_y;
  real_T c2_l_y;
  real_T c2_m_y;
  real_T c2_k_a;
  int32_T c2_i122;
  int32_T c2_i123;
  real_T c2_l_a;
  int32_T c2_i124;
  int32_T c2_i125;
  int32_T c2_i126;
  real_T c2_f_b[180];
  int32_T c2_i127;
  int32_T c2_i128;
  real_T c2_l_hoistedGlobal[190];
  int32_T c2_i129;
  int32_T c2_i130;
  int32_T c2_i131;
  real_T c2_dv27[10];
  int32_T c2_i132;
  int32_T c2_i133;
  real_T c2_b_x_kkm1[4];
  real_T c2_dv28[4];
  int32_T c2_i134;
  real_T c2_dv29[4];
  real_T c2_d2;
  int32_T c2_i135;
  real_T c2_c_x_kkm1[4];
  real_T c2_dv30[4];
  int32_T c2_i136;
  int32_T c2_i137;
  int32_T c2_i138;
  real_T c2_d_x_kkm1[4];
  int32_T c2_i139;
  int32_T c2_i140;
  int32_T c2_i141;
  int32_T c2_i142;
  real_T c2_b_X_kkm1[76];
  int32_T c2_i143;
  real_T c2_dv31[4];
  real_T c2_dv32[76];
  int32_T c2_i144;
  int32_T c2_i145;
  real_T c2_m_a[6];
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  real_T c2_n_y[114];
  int32_T c2_i149;
  int32_T c2_i150;
  int32_T c2_i151;
  int32_T c2_i152;
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  int32_T c2_i156;
  int32_T c2_i157;
  real_T c2_n_a;
  int32_T c2_i158;
  int32_T c2_i159;
  real_T c2_o_a;
  int32_T c2_i160;
  int32_T c2_i161;
  int32_T c2_i162;
  real_T c2_g_b[162];
  int32_T c2_i163;
  int32_T c2_i164;
  real_T c2_h_b[171];
  int32_T c2_i165;
  int32_T c2_i166;
  int32_T c2_i167;
  real_T c2_dv33[9];
  int32_T c2_i168;
  int32_T c2_i169;
  int32_T c2_d_i;
  real_T c2_p_a;
  int32_T c2_e_i;
  int32_T c2_i170;
  int32_T c2_i171;
  int32_T c2_f_i;
  int32_T c2_i172;
  real_T c2_i_b[9];
  int32_T c2_i173;
  int32_T c2_i174;
  int32_T c2_i175;
  int32_T c2_i176;
  real_T c2_q_a;
  int32_T c2_g_i;
  int32_T c2_i177;
  int32_T c2_i178;
  int32_T c2_h_i;
  int32_T c2_i179;
  int32_T c2_i180;
  int32_T c2_i181;
  int32_T c2_i182;
  int32_T c2_i183;
  int32_T c2_i184;
  int32_T c2_i185;
  int32_T c2_i186;
  int32_T c2_i187;
  real_T c2_c_X_kkm1[190];
  int32_T c2_i188;
  real_T c2_d_B_ECI[3];
  real_T c2_dv34[114];
  int32_T c2_i189;
  real_T c2_r_a;
  int32_T c2_i190;
  int32_T c2_i191;
  real_T c2_s_a;
  int32_T c2_i192;
  int32_T c2_i193;
  int32_T c2_i194;
  real_T c2_j_b[108];
  int32_T c2_i195;
  int32_T c2_i196;
  real_T c2_t_a[114];
  int32_T c2_i197;
  int32_T c2_i198;
  int32_T c2_i199;
  real_T c2_dv35[6];
  int32_T c2_i200;
  int32_T c2_i201;
  int32_T c2_i202;
  int32_T c2_i_i;
  real_T c2_u_a;
  int32_T c2_j_i;
  int32_T c2_i203;
  int32_T c2_i204;
  int32_T c2_k_i;
  int32_T c2_i205;
  real_T c2_k_b[6];
  int32_T c2_i206;
  int32_T c2_i207;
  int32_T c2_i208;
  real_T c2_l_b[36];
  int32_T c2_i209;
  real_T c2_v_a;
  int32_T c2_l_i;
  int32_T c2_i210;
  int32_T c2_i211;
  int32_T c2_m_i;
  int32_T c2_i212;
  int32_T c2_i213;
  int32_T c2_i214;
  int32_T c2_i215;
  real_T c2_o_y[54];
  int32_T c2_i216;
  real_T c2_w_a;
  int32_T c2_n_i;
  int32_T c2_i217;
  int32_T c2_i218;
  int32_T c2_o_i;
  int32_T c2_i219;
  int32_T c2_i220;
  int32_T c2_i221;
  int32_T c2_i222;
  int32_T c2_i223;
  real_T c2_x_a;
  int32_T c2_p_i;
  int32_T c2_i224;
  int32_T c2_i225;
  int32_T c2_q_i;
  int32_T c2_i226;
  int32_T c2_i227;
  int32_T c2_i228;
  int32_T c2_i229;
  int32_T c2_i230;
  int32_T c2_i231;
  int32_T c2_i232;
  int32_T c2_i233;
  real_T c2_b_P_xkzk[54];
  int32_T c2_i234;
  real_T c2_b_P_zkzk[36];
  real_T c2_dv36[54];
  int32_T c2_i235;
  int32_T c2_i236;
  int32_T c2_i237;
  real_T c2_b_z_kkm1_control[3];
  int32_T c2_i238;
  real_T c2_y_a[3];
  real_T c2_d3;
  int32_T c2_i239;
  real_T c2_c_z_kkm1_control[3];
  int32_T c2_i240;
  int32_T c2_i241;
  real_T c2_d_B_sat[3];
  int32_T c2_i242;
  real_T c2_dv37[4];
  real_T c2_dv38[3];
  int32_T c2_i243;
  int32_T c2_i244;
  real_T c2_c_w_gyro[3];
  int32_T c2_i245;
  real_T c2_dv39[4];
  real_T c2_dv40[3];
  int32_T c2_i246;
  int32_T c2_i247;
  real_T c2_ab_a[54];
  int32_T c2_i248;
  real_T c2_b_B_control[6];
  int32_T c2_i249;
  int32_T c2_i250;
  int32_T c2_i251;
  int32_T c2_i252;
  int32_T c2_i253;
  int32_T c2_i254;
  int32_T c2_i255;
  int32_T c2_i256;
  int32_T c2_i257;
  int32_T c2_i258;
  int32_T c2_i259;
  int32_T c2_i260;
  real_T c2_b_dx_kk[3];
  int32_T c2_i261;
  real_T c2_bb_a[3];
  real_T c2_d4;
  real_T c2_dv41[4];
  int32_T c2_i262;
  int32_T c2_i263;
  real_T c2_e_x_kkm1[4];
  int32_T c2_i264;
  int32_T c2_i265;
  int32_T c2_i266;
  int32_T c2_i267;
  int32_T c2_i268;
  int32_T c2_i269;
  int32_T c2_i270;
  int32_T c2_i271;
  int32_T c2_i272;
  int32_T c2_i273;
  int32_T c2_i274;
  int32_T c2_i275;
  int32_T c2_i276;
  int32_T c2_i277;
  int32_T c2_i278;
  real_T c2_m_b[54];
  int32_T c2_i279;
  int32_T c2_i280;
  int32_T c2_i281;
  int32_T c2_i282;
  int32_T c2_i283;
  int32_T c2_i284;
  int32_T c2_i285;
  int32_T c2_i286;
  int32_T c2_i287;
  int32_T c2_i288;
  real_T c2_dv42[4];
  int32_T c2_i289;
  real_T c2_dv43[4];
  int32_T c2_i290;
  real_T c2_dv44[4];
  real_T c2_dv45[4];
  int32_T c2_i291;
  int32_T c2_i292;
  real_T c2_dv46[3];
  int32_T c2_i293;
  real_T c2_dv47[4];
  real_T c2_dv48[3];
  int32_T c2_i294;
  int32_T c2_i295;
  real_T c2_dv49[3];
  int32_T c2_i296;
  real_T c2_dv50[4];
  real_T c2_dv51[3];
  int32_T c2_i297;
  int32_T c2_i298;
  int32_T c2_i299;
  int32_T c2_i300;
  int32_T c2_i301;
  int32_T c2_i302;
  real_T (*c2_b_noise_covarience)[36];
  real_T (*c2_b_Kalman_Gain)[54];
  real_T (*c2_b_w_bias_sensor)[3];
  real_T (*c2_b_w_sensor)[3];
  real_T (*c2_b_Attitude_sensor)[4];
  real_T *c2_b_Ts;
  real_T (*c2_c_Torque_s)[3];
  real_T (*c2_d_w_gyro)[3];
  real_T (*c2_e_B_sat)[3];
  real_T (*c2_e_B_ECI)[3];
  c2_b_noise_covarience = (real_T (*)[36])ssGetOutputPortSignal(chartInstance->S,
    5);
  c2_b_Kalman_Gain = (real_T (*)[54])ssGetOutputPortSignal(chartInstance->S, 4);
  c2_b_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_c_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_d_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_e_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S,
    1);
  c2_e_B_ECI = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_Ts;
  for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
    c2_B_ECI[c2_i33] = (*c2_e_B_ECI)[c2_i33];
  }

  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_B_sat[c2_i34] = (*c2_e_B_sat)[c2_i34];
  }

  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    c2_w_gyro[c2_i35] = (*c2_d_w_gyro)[c2_i35];
  }

  for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
    c2_Torque_s[c2_i36] = (*c2_c_Torque_s)[c2_i36];
  }

  c2_Ts = c2_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 49U, 49U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P_kk, 0U, c2_t_sf_marshallOut,
    c2_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_L, 1U, c2_p_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_bias, 2U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_DX_sigma, 3U, c2_t_sf_marshallOut,
    c2_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dX_km1km1, 4U, c2_u_sf_marshallOut,
    c2_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_km1km1_temp, 5U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_km1km1, 6U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Torque_c, 7U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_kkm1, 8U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_W0_m, 9U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_W0_c, 10U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Wi_cm, 11U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_x_kkm1, 12U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dX_kkm1_temp, 13U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dX_kkm1, 14U, c2_u_sf_marshallOut,
    c2_u_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx_kkm1, 15U, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P_kkm1, 16U, c2_t_sf_marshallOut,
    c2_t_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_i, 17U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Z_kkm1_control, 18U,
    c2_s_sf_marshallOut, c2_s_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_z_kkm1_control, 19U,
    c2_r_sf_marshallOut, c2_r_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P_zkzk, 20U, c2_l_sf_marshallOut,
    c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P_xkzk, 21U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_K_k, 22U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_B_control, 23U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_control, 24U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx_kk, 25U, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 26U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 27U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_B_ECI, 28U, c2_n_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_B_sat, 29U, c2_n_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_w_gyro, 30U, c2_n_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Torque_s, 31U, c2_n_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Ts, 32U, c2_p_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Attitude_sensor, 33U,
    c2_o_sf_marshallOut, c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_sensor, 34U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_bias_sensor, 35U,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Kalman_Gain, 36U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_noise_covarience, 37U,
    c2_l_sf_marshallOut, c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_P_km1km1, 38U,
    c2_k_sf_marshallOut, c2_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_x_km1km1, 39U,
    c2_j_sf_marshallOut, c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_x_kk, 40U,
    c2_i_sf_marshallOut, c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_R_k, 41U,
    c2_h_sf_marshallOut, c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_Q_k, 42U,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_Lambda, 43U,
    c2_f_sf_marshallOut, c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_alpha, 44U,
    c2_e_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_K, 45U,
    c2_d_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_Beta, 46U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_q_s_c, 47U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_I_c, 48U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  for (c2_i37 = 0; c2_i37 < 3; c2_i37++) {
    c2_b_B_sat[c2_i37] = c2_B_sat[c2_i37];
  }

  c2_power(chartInstance, c2_b_B_sat, c2_a);
  for (c2_i38 = 0; c2_i38 < 3; c2_i38++) {
    c2_b_a[c2_i38] = c2_a[c2_i38];
  }

  c2_d0 = c2_sum(chartInstance, c2_b_a);
  c2_c_sqrt(chartInstance, &c2_d0);
  for (c2_i39 = 0; c2_i39 < 3; c2_i39++) {
    c2_c_B_sat[c2_i39] = c2_B_sat[c2_i39];
  }

  c2_rdivide(chartInstance, c2_c_B_sat, c2_d0, c2_dv12);
  for (c2_i40 = 0; c2_i40 < 3; c2_i40++) {
    c2_B_sat[c2_i40] = c2_dv12[c2_i40];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  for (c2_i41 = 0; c2_i41 < 3; c2_i41++) {
    c2_b_B_ECI[c2_i41] = c2_B_ECI[c2_i41];
  }

  c2_power(chartInstance, c2_b_B_ECI, c2_a);
  for (c2_i42 = 0; c2_i42 < 3; c2_i42++) {
    c2_c_a[c2_i42] = c2_a[c2_i42];
  }

  c2_d1 = c2_sum(chartInstance, c2_c_a);
  c2_c_sqrt(chartInstance, &c2_d1);
  for (c2_i43 = 0; c2_i43 < 3; c2_i43++) {
    c2_c_B_ECI[c2_i43] = c2_B_ECI[c2_i43];
  }

  c2_rdivide(chartInstance, c2_c_B_ECI, c2_d1, c2_dv13);
  for (c2_i44 = 0; c2_i44 < 3; c2_i44++) {
    c2_B_ECI[c2_i44] = c2_dv13[c2_i44];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c2_x_km1km1_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
    for (c2_i45 = 0; c2_i45 < 4; c2_i45++) {
      chartInstance->c2_q_s_c[c2_i45] = c2_dv14[c2_i45];
    }

    chartInstance->c2_q_s_c_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
    for (c2_i46 = 0; c2_i46 < 3; c2_i46++) {
      chartInstance->c2_I_c[c2_i46] = c2_dv15[c2_i46];
    }

    chartInstance->c2_I_c_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
    for (c2_i47 = 0; c2_i47 < 81; c2_i47++) {
      chartInstance->c2_P_km1km1[c2_i47] = c2_dv16[c2_i47];
    }

    chartInstance->c2_P_km1km1_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
    for (c2_i48 = 0; c2_i48 < 81; c2_i48++) {
      c2_P_kk[c2_i48] = chartInstance->c2_P_km1km1[c2_i48];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
    for (c2_i49 = 0; c2_i49 < 10; c2_i49++) {
      chartInstance->c2_x_km1km1[c2_i49] = 0.0;
    }

    chartInstance->c2_x_km1km1_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
    for (c2_i50 = 0; c2_i50 < 4; c2_i50++) {
      chartInstance->c2_x_km1km1[c2_i50] = c2_dv14[c2_i50];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
    for (c2_i51 = 0; c2_i51 < 3; c2_i51++) {
      c2_b_w_gyro[c2_i51] = c2_w_gyro[c2_i51];
    }

    for (c2_i52 = 0; c2_i52 < 4; c2_i52++) {
      c2_dv17[c2_i52] = chartInstance->c2_q_s_c[c2_i52];
    }

    c2_RotateVecSensor2Cont(chartInstance, c2_b_w_gyro, c2_dv17, c2_dv18);
    for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
      chartInstance->c2_x_km1km1[c2_i53 + 4] = c2_dv18[c2_i53];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
    for (c2_i54 = 0; c2_i54 < 3; c2_i54++) {
      chartInstance->c2_x_km1km1[c2_i54 + 7] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
    for (c2_i55 = 0; c2_i55 < 10; c2_i55++) {
      chartInstance->c2_x_kk[c2_i55] = chartInstance->c2_x_km1km1[c2_i55];
    }

    chartInstance->c2_x_kk_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
    c2_L = 9.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 40);
    chartInstance->c2_alpha = 3.0;
    c2_c_sqrt(chartInstance, &chartInstance->c2_alpha);
    chartInstance->c2_alpha_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 41);
    chartInstance->c2_K = 0.0;
    chartInstance->c2_K_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 42);
    chartInstance->c2_Lambda = c2_mpower(chartInstance, chartInstance->c2_alpha)
      * (c2_L + chartInstance->c2_K) - c2_L;
    chartInstance->c2_Lambda_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
    for (c2_i56 = 0; c2_i56 < 36; c2_i56++) {
      chartInstance->c2_R_k[c2_i56] = c2_dv19[c2_i56];
    }

    chartInstance->c2_R_k_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
    for (c2_i57 = 0; c2_i57 < 81; c2_i57++) {
      chartInstance->c2_Q_k[c2_i57] = c2_dv20[c2_i57];
    }

    chartInstance->c2_Q_k_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 45);
    chartInstance->c2_Beta = 2.0;
    chartInstance->c2_Beta_not_empty = true;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
    for (c2_i58 = 0; c2_i58 < 54; c2_i58++) {
      c2_Kalman_Gain[c2_i58] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 47);
    for (c2_i59 = 0; c2_i59 < 36; c2_i59++) {
      c2_noise_covarience[c2_i59] = 0.0;
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
    c2_L = 9.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
    for (c2_i60 = 0; c2_i60 < 10; c2_i60++) {
      chartInstance->c2_x_kk[c2_i60] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
    for (c2_i61 = 0; c2_i61 < 4; c2_i61++) {
      c2_Attitude_sensor[c2_i61] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
    for (c2_i62 = 0; c2_i62 < 3; c2_i62++) {
      c2_w_bias[c2_i62] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
    c2_b_hoistedGlobal = chartInstance->c2_Lambda;
    for (c2_i63 = 0; c2_i63 < 81; c2_i63++) {
      c2_c_hoistedGlobal[c2_i63] = chartInstance->c2_P_km1km1[c2_i63];
    }

    c2_d_a = c2_L + c2_b_hoistedGlobal;
    for (c2_i64 = 0; c2_i64 < 81; c2_i64++) {
      c2_c_hoistedGlobal[c2_i64] *= c2_d_a;
    }

    for (c2_j = 1; c2_j < 10; c2_j++) {
      c2_b_j = c2_j;
    }

    c2_info = c2_b_eml_matlab_zpotrf(chartInstance, c2_c_hoistedGlobal);
    c2_b_info = c2_info;
    c2_c_info = c2_b_info;
    c2_d_info = c2_c_info;
    if (c2_d_info == 0) {
      c2_jmax = 9;
    } else {
      c2_c_eml_error(chartInstance);
      c2_e_a = c2_d_info;
      c2_f_a = c2_e_a - 1;
      c2_jmax = c2_f_a;
    }

    c2_b_jmax = c2_jmax;
    c2_b = c2_b_jmax;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_c_j = 1; c2_c_j <= c2_b_jmax; c2_c_j++) {
      c2_b_j = c2_c_j;
      c2_g_a = c2_b_j;
      c2_h_a = c2_g_a + 1;
      c2_i65 = c2_h_a;
      c2_c_jmax = c2_jmax;
      c2_i_a = c2_i65;
      c2_c_b = c2_c_jmax;
      c2_j_a = c2_i_a;
      c2_d_b = c2_c_b;
      if (c2_j_a > c2_d_b) {
        c2_b_overflow = false;
      } else {
        c2_eml_switch_helper(chartInstance);
        c2_b_overflow = (c2_d_b > 2147483646);
      }

      if (c2_b_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      }

      for (c2_b_i = c2_i65; c2_b_i <= c2_c_jmax; c2_b_i++) {
        c2_c_i = c2_b_i;
        c2_c_hoistedGlobal[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_c_i), 1, 9, 1, 0) + 9 *
                            (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = 0.0;
      }
    }

    for (c2_i66 = 0; c2_i66 < 81; c2_i66++) {
      c2_DX_sigma[c2_i66] = c2_c_hoistedGlobal[c2_i66];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
    for (c2_i67 = 0; c2_i67 < 9; c2_i67++) {
      c2_e_b[c2_i67] = 0.0;
    }

    for (c2_i68 = 0; c2_i68 < 9; c2_i68++) {
      c2_dX_km1km1[c2_i68] = c2_e_b[c2_i68];
    }

    c2_i69 = 0;
    for (c2_i70 = 0; c2_i70 < 9; c2_i70++) {
      for (c2_i71 = 0; c2_i71 < 9; c2_i71++) {
        c2_dX_km1km1[(c2_i71 + c2_i69) + 9] = -c2_DX_sigma[c2_i71 + c2_i69];
      }

      c2_i69 += 9;
    }

    c2_i72 = 0;
    for (c2_i73 = 0; c2_i73 < 9; c2_i73++) {
      for (c2_i74 = 0; c2_i74 < 9; c2_i74++) {
        c2_dX_km1km1[(c2_i74 + c2_i72) + 90] = c2_DX_sigma[c2_i74 + c2_i72];
      }

      c2_i72 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 62);
    for (c2_i75 = 0; c2_i75 < 76; c2_i75++) {
      c2_X_km1km1_temp[c2_i75] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
    c2_i76 = 0;
    c2_i77 = 0;
    for (c2_i78 = 0; c2_i78 < 19; c2_i78++) {
      for (c2_i79 = 0; c2_i79 < 3; c2_i79++) {
        c2_b_dX_km1km1[c2_i79 + c2_i76] = c2_dX_km1km1[c2_i79 + c2_i77];
      }

      c2_i76 += 3;
      c2_i77 += 9;
    }

    c2_c_power(chartInstance, c2_b_dX_km1km1, c2_y);
    for (c2_i80 = 0; c2_i80 < 57; c2_i80++) {
      c2_b_y[c2_i80] = c2_y[c2_i80];
    }

    c2_c_sum(chartInstance, c2_b_y, c2_dv21);
    for (c2_i81 = 0; c2_i81 < 19; c2_i81++) {
      c2_dv21[c2_i81] = 1.0 - c2_dv21[c2_i81];
    }

    c2_d_sqrt(chartInstance, c2_dv21);
    c2_i82 = 0;
    for (c2_i83 = 0; c2_i83 < 19; c2_i83++) {
      c2_X_km1km1_temp[c2_i82] = c2_dv21[c2_i83];
      c2_i82 += 4;
    }

    c2_i84 = 0;
    c2_i85 = 0;
    for (c2_i86 = 0; c2_i86 < 19; c2_i86++) {
      for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
        c2_X_km1km1_temp[(c2_i87 + c2_i84) + 1] = c2_dX_km1km1[c2_i87 + c2_i85];
      }

      c2_i84 += 4;
      c2_i85 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 64);
    for (c2_i88 = 0; c2_i88 < 10; c2_i88++) {
      c2_d_hoistedGlobal[c2_i88] = chartInstance->c2_x_km1km1[c2_i88];
    }

    for (c2_i89 = 0; c2_i89 < 10; c2_i89++) {
      c2_e_hoistedGlobal[c2_i89] = chartInstance->c2_x_km1km1[c2_i89];
    }

    for (c2_i90 = 0; c2_i90 < 3; c2_i90++) {
      c2_a[c2_i90] = c2_e_hoistedGlobal[c2_i90 + 4];
    }

    for (c2_i91 = 0; c2_i91 < 3; c2_i91++) {
      c2_i92 = 0;
      for (c2_i93 = 0; c2_i93 < 19; c2_i93++) {
        c2_y[c2_i92 + c2_i91] = c2_a[c2_i91];
        c2_i92 += 3;
      }
    }

    for (c2_i94 = 0; c2_i94 < 10; c2_i94++) {
      c2_e_hoistedGlobal[c2_i94] = chartInstance->c2_x_km1km1[c2_i94];
    }

    for (c2_i95 = 0; c2_i95 < 3; c2_i95++) {
      c2_a[c2_i95] = c2_e_hoistedGlobal[c2_i95 + 7];
    }

    for (c2_i96 = 0; c2_i96 < 3; c2_i96++) {
      c2_i97 = 0;
      for (c2_i98 = 0; c2_i98 < 19; c2_i98++) {
        c2_c_y[c2_i97 + c2_i96] = c2_a[c2_i96];
        c2_i97 += 3;
      }
    }

    for (c2_i99 = 0; c2_i99 < 76; c2_i99++) {
      c2_b_X_km1km1_temp[c2_i99] = c2_X_km1km1_temp[c2_i99];
    }

    for (c2_i100 = 0; c2_i100 < 4; c2_i100++) {
      c2_f_hoistedGlobal[c2_i100] = c2_d_hoistedGlobal[c2_i100];
    }

    c2_b_quatmultiply(chartInstance, c2_b_X_km1km1_temp, c2_f_hoistedGlobal,
                      c2_dv22);
    c2_i101 = 0;
    c2_i102 = 0;
    for (c2_i103 = 0; c2_i103 < 19; c2_i103++) {
      for (c2_i104 = 0; c2_i104 < 4; c2_i104++) {
        c2_X_km1km1[c2_i104 + c2_i101] = c2_dv22[c2_i104 + c2_i102];
      }

      c2_i101 += 10;
      c2_i102 += 4;
    }

    c2_i105 = 0;
    c2_i106 = 0;
    c2_i107 = 0;
    for (c2_i108 = 0; c2_i108 < 19; c2_i108++) {
      for (c2_i109 = 0; c2_i109 < 3; c2_i109++) {
        c2_X_km1km1[(c2_i109 + c2_i105) + 4] = c2_y[c2_i109 + c2_i106] +
          c2_dX_km1km1[(c2_i109 + c2_i107) + 3];
      }

      c2_i105 += 10;
      c2_i106 += 3;
      c2_i107 += 9;
    }

    c2_i110 = 0;
    c2_i111 = 0;
    c2_i112 = 0;
    for (c2_i113 = 0; c2_i113 < 19; c2_i113++) {
      for (c2_i114 = 0; c2_i114 < 3; c2_i114++) {
        c2_X_km1km1[(c2_i114 + c2_i110) + 7] = c2_c_y[c2_i114 + c2_i111] +
          c2_dX_km1km1[(c2_i114 + c2_i112) + 6];
      }

      c2_i110 += 10;
      c2_i111 += 3;
      c2_i112 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 70);
    for (c2_i115 = 0; c2_i115 < 3; c2_i115++) {
      c2_b_Torque_s[c2_i115] = c2_Torque_s[c2_i115];
    }

    for (c2_i116 = 0; c2_i116 < 4; c2_i116++) {
      c2_dv23[c2_i116] = chartInstance->c2_q_s_c[c2_i116];
    }

    c2_RotateVecSensor2Cont(chartInstance, c2_b_Torque_s, c2_dv23, c2_dv24);
    for (c2_i117 = 0; c2_i117 < 3; c2_i117++) {
      c2_Torque_c[c2_i117] = c2_dv24[c2_i117];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 71);
    for (c2_i118 = 0; c2_i118 < 190; c2_i118++) {
      c2_b_X_km1km1[c2_i118] = c2_X_km1km1[c2_i118];
    }

    for (c2_i119 = 0; c2_i119 < 3; c2_i119++) {
      c2_dv25[c2_i119] = chartInstance->c2_I_c[c2_i119];
    }

    for (c2_i120 = 0; c2_i120 < 3; c2_i120++) {
      c2_b_Torque_c[c2_i120] = c2_Torque_c[c2_i120];
    }

    c2_RK4(chartInstance, c2_b_X_km1km1, c2_dv25, c2_b_Torque_c, c2_Ts, c2_dv26);
    for (c2_i121 = 0; c2_i121 < 190; c2_i121++) {
      c2_X_kkm1[c2_i121] = c2_dv26[c2_i121];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 74);
    c2_g_hoistedGlobal = chartInstance->c2_Lambda;
    c2_h_hoistedGlobal = chartInstance->c2_Lambda;
    c2_A = c2_g_hoistedGlobal;
    c2_B = c2_L + c2_h_hoistedGlobal;
    c2_x = c2_A;
    c2_d_y = c2_B;
    c2_b_x = c2_x;
    c2_e_y = c2_d_y;
    c2_c_x = c2_b_x;
    c2_f_y = c2_e_y;
    c2_W0_m = c2_c_x / c2_f_y;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 75);
    c2_i_hoistedGlobal = chartInstance->c2_Lambda;
    c2_j_hoistedGlobal = chartInstance->c2_Lambda;
    c2_b_A = c2_i_hoistedGlobal;
    c2_b_B = c2_L + c2_j_hoistedGlobal;
    c2_d_x = c2_b_A;
    c2_g_y = c2_b_B;
    c2_e_x = c2_d_x;
    c2_h_y = c2_g_y;
    c2_f_x = c2_e_x;
    c2_i_y = c2_h_y;
    c2_j_y = c2_f_x / c2_i_y;
    c2_W0_c = c2_j_y + ((1.0 - c2_mpower(chartInstance, chartInstance->c2_alpha))
                        + chartInstance->c2_Beta);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 76);
    c2_k_hoistedGlobal = chartInstance->c2_Lambda;
    c2_c_B = 2.0 * (c2_L + c2_k_hoistedGlobal);
    c2_k_y = c2_c_B;
    c2_l_y = c2_k_y;
    c2_m_y = c2_l_y;
    c2_Wi_cm = 1.0 / c2_m_y;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 77);
    c2_k_a = c2_W0_m;
    for (c2_i122 = 0; c2_i122 < 10; c2_i122++) {
      c2_d_hoistedGlobal[c2_i122] = c2_X_kkm1[c2_i122];
    }

    for (c2_i123 = 0; c2_i123 < 10; c2_i123++) {
      c2_d_hoistedGlobal[c2_i123] *= c2_k_a;
    }

    c2_l_a = c2_Wi_cm;
    c2_i124 = 0;
    for (c2_i125 = 0; c2_i125 < 18; c2_i125++) {
      for (c2_i126 = 0; c2_i126 < 10; c2_i126++) {
        c2_f_b[c2_i126 + c2_i124] = c2_X_kkm1[(c2_i126 + c2_i124) + 10];
      }

      c2_i124 += 10;
    }

    for (c2_i127 = 0; c2_i127 < 180; c2_i127++) {
      c2_f_b[c2_i127] *= c2_l_a;
    }

    for (c2_i128 = 0; c2_i128 < 10; c2_i128++) {
      c2_l_hoistedGlobal[c2_i128] = c2_d_hoistedGlobal[c2_i128];
    }

    c2_i129 = 0;
    for (c2_i130 = 0; c2_i130 < 18; c2_i130++) {
      for (c2_i131 = 0; c2_i131 < 10; c2_i131++) {
        c2_l_hoistedGlobal[(c2_i131 + c2_i129) + 10] = c2_f_b[c2_i131 + c2_i129];
      }

      c2_i129 += 10;
    }

    c2_d_sum(chartInstance, c2_l_hoistedGlobal, c2_dv27);
    for (c2_i132 = 0; c2_i132 < 10; c2_i132++) {
      c2_x_kkm1[c2_i132] = c2_dv27[c2_i132];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 78);
    for (c2_i133 = 0; c2_i133 < 4; c2_i133++) {
      c2_b_x_kkm1[c2_i133] = c2_x_kkm1[c2_i133];
    }

    c2_b_power(chartInstance, c2_b_x_kkm1, c2_dv28);
    for (c2_i134 = 0; c2_i134 < 4; c2_i134++) {
      c2_dv29[c2_i134] = c2_dv28[c2_i134];
    }

    c2_d2 = c2_b_sum(chartInstance, c2_dv29);
    c2_c_sqrt(chartInstance, &c2_d2);
    for (c2_i135 = 0; c2_i135 < 4; c2_i135++) {
      c2_c_x_kkm1[c2_i135] = c2_x_kkm1[c2_i135];
    }

    c2_b_rdivide(chartInstance, c2_c_x_kkm1, c2_d2, c2_dv30);
    for (c2_i136 = 0; c2_i136 < 4; c2_i136++) {
      c2_x_kkm1[c2_i136] = c2_dv30[c2_i136];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 82);
    for (c2_i137 = 0; c2_i137 < 76; c2_i137++) {
      c2_dX_kkm1_temp[c2_i137] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 83);
    for (c2_i138 = 0; c2_i138 < 4; c2_i138++) {
      c2_d_x_kkm1[c2_i138] = c2_x_kkm1[c2_i138];
    }

    c2_quatinv(chartInstance, c2_d_x_kkm1, c2_dv28);
    c2_i139 = 0;
    c2_i140 = 0;
    for (c2_i141 = 0; c2_i141 < 19; c2_i141++) {
      for (c2_i142 = 0; c2_i142 < 4; c2_i142++) {
        c2_b_X_kkm1[c2_i142 + c2_i139] = c2_X_kkm1[c2_i142 + c2_i140];
      }

      c2_i139 += 4;
      c2_i140 += 10;
    }

    for (c2_i143 = 0; c2_i143 < 4; c2_i143++) {
      c2_dv31[c2_i143] = c2_dv28[c2_i143];
    }

    c2_b_quatmultiply(chartInstance, c2_b_X_kkm1, c2_dv31, c2_dv32);
    for (c2_i144 = 0; c2_i144 < 76; c2_i144++) {
      c2_dX_kkm1_temp[c2_i144] = c2_dv32[c2_i144];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 84);
    for (c2_i145 = 0; c2_i145 < 6; c2_i145++) {
      c2_m_a[c2_i145] = c2_x_kkm1[c2_i145 + 4];
    }

    for (c2_i146 = 0; c2_i146 < 6; c2_i146++) {
      c2_i147 = 0;
      for (c2_i148 = 0; c2_i148 < 19; c2_i148++) {
        c2_n_y[c2_i147 + c2_i146] = c2_m_a[c2_i146];
        c2_i147 += 6;
      }
    }

    c2_i149 = 0;
    c2_i150 = 0;
    for (c2_i151 = 0; c2_i151 < 19; c2_i151++) {
      for (c2_i152 = 0; c2_i152 < 3; c2_i152++) {
        c2_dX_kkm1[c2_i152 + c2_i149] = c2_dX_kkm1_temp[(c2_i152 + c2_i150) + 1];
      }

      c2_i149 += 9;
      c2_i150 += 4;
    }

    c2_i153 = 0;
    c2_i154 = 0;
    c2_i155 = 0;
    for (c2_i156 = 0; c2_i156 < 19; c2_i156++) {
      for (c2_i157 = 0; c2_i157 < 6; c2_i157++) {
        c2_dX_kkm1[(c2_i157 + c2_i153) + 3] = c2_X_kkm1[(c2_i157 + c2_i154) + 4]
          - c2_n_y[c2_i157 + c2_i155];
      }

      c2_i153 += 9;
      c2_i154 += 10;
      c2_i155 += 6;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 88);
    c2_n_a = c2_W0_m;
    for (c2_i158 = 0; c2_i158 < 9; c2_i158++) {
      c2_e_b[c2_i158] = c2_dX_kkm1[c2_i158];
    }

    for (c2_i159 = 0; c2_i159 < 9; c2_i159++) {
      c2_e_b[c2_i159] *= c2_n_a;
    }

    c2_o_a = c2_Wi_cm;
    c2_i160 = 0;
    for (c2_i161 = 0; c2_i161 < 18; c2_i161++) {
      for (c2_i162 = 0; c2_i162 < 9; c2_i162++) {
        c2_g_b[c2_i162 + c2_i160] = c2_dX_kkm1[(c2_i162 + c2_i160) + 9];
      }

      c2_i160 += 9;
    }

    for (c2_i163 = 0; c2_i163 < 162; c2_i163++) {
      c2_g_b[c2_i163] *= c2_o_a;
    }

    for (c2_i164 = 0; c2_i164 < 9; c2_i164++) {
      c2_h_b[c2_i164] = c2_e_b[c2_i164];
    }

    c2_i165 = 0;
    for (c2_i166 = 0; c2_i166 < 18; c2_i166++) {
      for (c2_i167 = 0; c2_i167 < 9; c2_i167++) {
        c2_h_b[(c2_i167 + c2_i165) + 9] = c2_g_b[c2_i167 + c2_i165];
      }

      c2_i165 += 9;
    }

    c2_e_sum(chartInstance, c2_h_b, c2_dv33);
    for (c2_i168 = 0; c2_i168 < 9; c2_i168++) {
      c2_dx_kkm1[c2_i168] = c2_dv33[c2_i168];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 91);
    for (c2_i169 = 0; c2_i169 < 81; c2_i169++) {
      c2_P_kkm1[c2_i169] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 92);
    c2_i = 1.0;
    c2_d_i = 0;
    while (c2_d_i < 19) {
      c2_i = 1.0 + (real_T)c2_d_i;
      CV_EML_FOR(0, 1, 0, 1);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 93);
      if (CV_EML_IF(0, 1, 1, c2_i == 1.0)) {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 94);
        c2_p_a = c2_W0_c;
        c2_e_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i170 = 0; c2_i170 < 9; c2_i170++) {
          c2_e_b[c2_i170] = c2_dX_kkm1[c2_i170 + 9 * c2_e_i] -
            c2_dx_kkm1[c2_i170];
        }

        for (c2_i171 = 0; c2_i171 < 9; c2_i171++) {
          c2_e_b[c2_i171] *= c2_p_a;
        }

        c2_f_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i172 = 0; c2_i172 < 9; c2_i172++) {
          c2_i_b[c2_i172] = c2_dX_kkm1[c2_i172 + 9 * c2_f_i] -
            c2_dx_kkm1[c2_i172];
        }

        for (c2_i173 = 0; c2_i173 < 9; c2_i173++) {
          c2_i174 = 0;
          for (c2_i175 = 0; c2_i175 < 9; c2_i175++) {
            c2_c_hoistedGlobal[c2_i174 + c2_i173] = c2_e_b[c2_i173] *
              c2_i_b[c2_i175];
            c2_i174 += 9;
          }
        }

        for (c2_i176 = 0; c2_i176 < 81; c2_i176++) {
          c2_P_kkm1[c2_i176] += c2_c_hoistedGlobal[c2_i176];
        }
      } else {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 97);
        c2_q_a = c2_Wi_cm;
        c2_g_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i177 = 0; c2_i177 < 9; c2_i177++) {
          c2_e_b[c2_i177] = c2_dX_kkm1[c2_i177 + 9 * c2_g_i] -
            c2_dx_kkm1[c2_i177];
        }

        for (c2_i178 = 0; c2_i178 < 9; c2_i178++) {
          c2_e_b[c2_i178] *= c2_q_a;
        }

        c2_h_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i179 = 0; c2_i179 < 9; c2_i179++) {
          c2_i_b[c2_i179] = c2_dX_kkm1[c2_i179 + 9 * c2_h_i] -
            c2_dx_kkm1[c2_i179];
        }

        for (c2_i180 = 0; c2_i180 < 9; c2_i180++) {
          c2_i181 = 0;
          for (c2_i182 = 0; c2_i182 < 9; c2_i182++) {
            c2_c_hoistedGlobal[c2_i181 + c2_i180] = c2_e_b[c2_i180] *
              c2_i_b[c2_i182];
            c2_i181 += 9;
          }
        }

        for (c2_i183 = 0; c2_i183 < 81; c2_i183++) {
          c2_P_kkm1[c2_i183] += c2_c_hoistedGlobal[c2_i183];
        }
      }

      c2_d_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 0, 0);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 101);
    for (c2_i184 = 0; c2_i184 < 81; c2_i184++) {
      c2_P_kkm1[c2_i184] += chartInstance->c2_Q_k[c2_i184];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 106);
    for (c2_i185 = 0; c2_i185 < 114; c2_i185++) {
      c2_Z_kkm1_control[c2_i185] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 107);
    for (c2_i186 = 0; c2_i186 < 6; c2_i186++) {
      c2_z_kkm1_control[c2_i186] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 108);
    for (c2_i187 = 0; c2_i187 < 190; c2_i187++) {
      c2_c_X_kkm1[c2_i187] = c2_X_kkm1[c2_i187];
    }

    for (c2_i188 = 0; c2_i188 < 3; c2_i188++) {
      c2_d_B_ECI[c2_i188] = c2_B_ECI[c2_i188];
    }

    c2_SensorModel(chartInstance, c2_c_X_kkm1, c2_d_B_ECI, c2_dv34);
    for (c2_i189 = 0; c2_i189 < 114; c2_i189++) {
      c2_Z_kkm1_control[c2_i189] = c2_dv34[c2_i189];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 109);
    c2_r_a = c2_W0_m;
    for (c2_i190 = 0; c2_i190 < 6; c2_i190++) {
      c2_m_a[c2_i190] = c2_Z_kkm1_control[c2_i190];
    }

    for (c2_i191 = 0; c2_i191 < 6; c2_i191++) {
      c2_m_a[c2_i191] *= c2_r_a;
    }

    c2_s_a = c2_Wi_cm;
    c2_i192 = 0;
    for (c2_i193 = 0; c2_i193 < 18; c2_i193++) {
      for (c2_i194 = 0; c2_i194 < 6; c2_i194++) {
        c2_j_b[c2_i194 + c2_i192] = c2_Z_kkm1_control[(c2_i194 + c2_i192) + 6];
      }

      c2_i192 += 6;
    }

    for (c2_i195 = 0; c2_i195 < 108; c2_i195++) {
      c2_j_b[c2_i195] *= c2_s_a;
    }

    for (c2_i196 = 0; c2_i196 < 6; c2_i196++) {
      c2_t_a[c2_i196] = c2_m_a[c2_i196];
    }

    c2_i197 = 0;
    for (c2_i198 = 0; c2_i198 < 18; c2_i198++) {
      for (c2_i199 = 0; c2_i199 < 6; c2_i199++) {
        c2_t_a[(c2_i199 + c2_i197) + 6] = c2_j_b[c2_i199 + c2_i197];
      }

      c2_i197 += 6;
    }

    c2_f_sum(chartInstance, c2_t_a, c2_dv35);
    for (c2_i200 = 0; c2_i200 < 6; c2_i200++) {
      c2_z_kkm1_control[c2_i200] = c2_dv35[c2_i200];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 113);
    for (c2_i201 = 0; c2_i201 < 36; c2_i201++) {
      c2_P_zkzk[c2_i201] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 114);
    for (c2_i202 = 0; c2_i202 < 54; c2_i202++) {
      c2_P_xkzk[c2_i202] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 115);
    c2_i = 1.0;
    c2_i_i = 0;
    while (c2_i_i < 19) {
      c2_i = 1.0 + (real_T)c2_i_i;
      CV_EML_FOR(0, 1, 1, 1);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 116);
      if (CV_EML_IF(0, 1, 2, c2_i == 1.0)) {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 117);
        c2_u_a = c2_W0_c;
        c2_j_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i203 = 0; c2_i203 < 6; c2_i203++) {
          c2_m_a[c2_i203] = c2_Z_kkm1_control[c2_i203 + 6 * c2_j_i] -
            c2_z_kkm1_control[c2_i203];
        }

        for (c2_i204 = 0; c2_i204 < 6; c2_i204++) {
          c2_m_a[c2_i204] *= c2_u_a;
        }

        c2_k_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i205 = 0; c2_i205 < 6; c2_i205++) {
          c2_k_b[c2_i205] = c2_Z_kkm1_control[c2_i205 + 6 * c2_k_i] -
            c2_z_kkm1_control[c2_i205];
        }

        for (c2_i206 = 0; c2_i206 < 6; c2_i206++) {
          c2_i207 = 0;
          for (c2_i208 = 0; c2_i208 < 6; c2_i208++) {
            c2_l_b[c2_i207 + c2_i206] = c2_m_a[c2_i206] * c2_k_b[c2_i208];
            c2_i207 += 6;
          }
        }

        for (c2_i209 = 0; c2_i209 < 36; c2_i209++) {
          c2_P_zkzk[c2_i209] += c2_l_b[c2_i209];
        }

        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 119);
        c2_v_a = c2_W0_c;
        c2_l_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i210 = 0; c2_i210 < 9; c2_i210++) {
          c2_e_b[c2_i210] = c2_dX_kkm1[c2_i210 + 9 * c2_l_i] -
            c2_dx_kkm1[c2_i210];
        }

        for (c2_i211 = 0; c2_i211 < 9; c2_i211++) {
          c2_e_b[c2_i211] *= c2_v_a;
        }

        c2_m_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i212 = 0; c2_i212 < 6; c2_i212++) {
          c2_k_b[c2_i212] = c2_Z_kkm1_control[c2_i212 + 6 * c2_m_i] -
            c2_z_kkm1_control[c2_i212];
        }

        for (c2_i213 = 0; c2_i213 < 9; c2_i213++) {
          c2_i214 = 0;
          for (c2_i215 = 0; c2_i215 < 6; c2_i215++) {
            c2_o_y[c2_i214 + c2_i213] = c2_e_b[c2_i213] * c2_k_b[c2_i215];
            c2_i214 += 9;
          }
        }

        for (c2_i216 = 0; c2_i216 < 54; c2_i216++) {
          c2_P_xkzk[c2_i216] += c2_o_y[c2_i216];
        }
      } else {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 122);
        c2_w_a = c2_Wi_cm;
        c2_n_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i217 = 0; c2_i217 < 6; c2_i217++) {
          c2_m_a[c2_i217] = c2_Z_kkm1_control[c2_i217 + 6 * c2_n_i] -
            c2_z_kkm1_control[c2_i217];
        }

        for (c2_i218 = 0; c2_i218 < 6; c2_i218++) {
          c2_m_a[c2_i218] *= c2_w_a;
        }

        c2_o_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i219 = 0; c2_i219 < 6; c2_i219++) {
          c2_k_b[c2_i219] = c2_Z_kkm1_control[c2_i219 + 6 * c2_o_i] -
            c2_z_kkm1_control[c2_i219];
        }

        for (c2_i220 = 0; c2_i220 < 6; c2_i220++) {
          c2_i221 = 0;
          for (c2_i222 = 0; c2_i222 < 6; c2_i222++) {
            c2_l_b[c2_i221 + c2_i220] = c2_m_a[c2_i220] * c2_k_b[c2_i222];
            c2_i221 += 6;
          }
        }

        for (c2_i223 = 0; c2_i223 < 36; c2_i223++) {
          c2_P_zkzk[c2_i223] += c2_l_b[c2_i223];
        }

        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 124);
        c2_x_a = c2_Wi_cm;
        c2_p_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i224 = 0; c2_i224 < 9; c2_i224++) {
          c2_e_b[c2_i224] = c2_dX_kkm1[c2_i224 + 9 * c2_p_i] -
            c2_dx_kkm1[c2_i224];
        }

        for (c2_i225 = 0; c2_i225 < 9; c2_i225++) {
          c2_e_b[c2_i225] *= c2_x_a;
        }

        c2_q_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 19, 2, 0) - 1;
        for (c2_i226 = 0; c2_i226 < 6; c2_i226++) {
          c2_k_b[c2_i226] = c2_Z_kkm1_control[c2_i226 + 6 * c2_q_i] -
            c2_z_kkm1_control[c2_i226];
        }

        for (c2_i227 = 0; c2_i227 < 9; c2_i227++) {
          c2_i228 = 0;
          for (c2_i229 = 0; c2_i229 < 6; c2_i229++) {
            c2_o_y[c2_i228 + c2_i227] = c2_e_b[c2_i227] * c2_k_b[c2_i229];
            c2_i228 += 9;
          }
        }

        for (c2_i230 = 0; c2_i230 < 54; c2_i230++) {
          c2_P_xkzk[c2_i230] += c2_o_y[c2_i230];
        }
      }

      c2_i_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 1, 0);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 129U);
    for (c2_i231 = 0; c2_i231 < 36; c2_i231++) {
      c2_P_zkzk[c2_i231] += chartInstance->c2_R_k[c2_i231];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 130U);
    for (c2_i232 = 0; c2_i232 < 36; c2_i232++) {
      c2_noise_covarience[c2_i232] = chartInstance->c2_R_k[c2_i232];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 133U);
    for (c2_i233 = 0; c2_i233 < 54; c2_i233++) {
      c2_b_P_xkzk[c2_i233] = c2_P_xkzk[c2_i233];
    }

    for (c2_i234 = 0; c2_i234 < 36; c2_i234++) {
      c2_b_P_zkzk[c2_i234] = c2_P_zkzk[c2_i234];
    }

    c2_mrdivide(chartInstance, c2_b_P_xkzk, c2_b_P_zkzk, c2_dv36);
    for (c2_i235 = 0; c2_i235 < 54; c2_i235++) {
      c2_K_k[c2_i235] = c2_dv36[c2_i235];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 134U);
    for (c2_i236 = 0; c2_i236 < 54; c2_i236++) {
      c2_Kalman_Gain[c2_i236] = c2_K_k[c2_i236];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 137U);
    for (c2_i237 = 0; c2_i237 < 3; c2_i237++) {
      c2_b_z_kkm1_control[c2_i237] = c2_z_kkm1_control[c2_i237];
    }

    c2_power(chartInstance, c2_b_z_kkm1_control, c2_a);
    for (c2_i238 = 0; c2_i238 < 3; c2_i238++) {
      c2_y_a[c2_i238] = c2_a[c2_i238];
    }

    c2_d3 = c2_sum(chartInstance, c2_y_a);
    c2_c_sqrt(chartInstance, &c2_d3);
    for (c2_i239 = 0; c2_i239 < 3; c2_i239++) {
      c2_c_z_kkm1_control[c2_i239] = c2_z_kkm1_control[c2_i239];
    }

    c2_rdivide(chartInstance, c2_c_z_kkm1_control, c2_d3, c2_a);
    for (c2_i240 = 0; c2_i240 < 3; c2_i240++) {
      c2_z_kkm1_control[c2_i240] = c2_a[c2_i240];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 140U);
    for (c2_i241 = 0; c2_i241 < 3; c2_i241++) {
      c2_d_B_sat[c2_i241] = c2_B_sat[c2_i241];
    }

    for (c2_i242 = 0; c2_i242 < 4; c2_i242++) {
      c2_dv37[c2_i242] = chartInstance->c2_q_s_c[c2_i242];
    }

    c2_RotateVecSensor2Cont(chartInstance, c2_d_B_sat, c2_dv37, c2_dv38);
    for (c2_i243 = 0; c2_i243 < 3; c2_i243++) {
      c2_B_control[c2_i243] = c2_dv38[c2_i243];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 141U);
    for (c2_i244 = 0; c2_i244 < 3; c2_i244++) {
      c2_c_w_gyro[c2_i244] = c2_w_gyro[c2_i244];
    }

    for (c2_i245 = 0; c2_i245 < 4; c2_i245++) {
      c2_dv39[c2_i245] = chartInstance->c2_q_s_c[c2_i245];
    }

    c2_RotateVecSensor2Cont(chartInstance, c2_c_w_gyro, c2_dv39, c2_dv40);
    for (c2_i246 = 0; c2_i246 < 3; c2_i246++) {
      c2_w_control[c2_i246] = c2_dv40[c2_i246];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 142U);
    for (c2_i247 = 0; c2_i247 < 54; c2_i247++) {
      c2_ab_a[c2_i247] = c2_K_k[c2_i247];
    }

    for (c2_i248 = 0; c2_i248 < 3; c2_i248++) {
      c2_b_B_control[c2_i248] = c2_B_control[c2_i248];
    }

    for (c2_i249 = 0; c2_i249 < 3; c2_i249++) {
      c2_b_B_control[c2_i249 + 3] = c2_w_control[c2_i249];
    }

    for (c2_i250 = 0; c2_i250 < 6; c2_i250++) {
      c2_m_a[c2_i250] = c2_b_B_control[c2_i250] - c2_z_kkm1_control[c2_i250];
    }

    c2_d_eml_scalar_eg(chartInstance);
    c2_d_eml_scalar_eg(chartInstance);
    for (c2_i251 = 0; c2_i251 < 9; c2_i251++) {
      c2_dx_kk[c2_i251] = 0.0;
    }

    for (c2_i252 = 0; c2_i252 < 9; c2_i252++) {
      c2_dx_kk[c2_i252] = 0.0;
    }

    for (c2_i253 = 0; c2_i253 < 9; c2_i253++) {
      c2_e_b[c2_i253] = c2_dx_kk[c2_i253];
    }

    for (c2_i254 = 0; c2_i254 < 9; c2_i254++) {
      c2_dx_kk[c2_i254] = c2_e_b[c2_i254];
    }

    c2_threshold(chartInstance);
    for (c2_i255 = 0; c2_i255 < 9; c2_i255++) {
      c2_e_b[c2_i255] = c2_dx_kk[c2_i255];
    }

    for (c2_i256 = 0; c2_i256 < 9; c2_i256++) {
      c2_dx_kk[c2_i256] = c2_e_b[c2_i256];
    }

    for (c2_i257 = 0; c2_i257 < 9; c2_i257++) {
      c2_dx_kk[c2_i257] = 0.0;
      c2_i258 = 0;
      for (c2_i259 = 0; c2_i259 < 6; c2_i259++) {
        c2_dx_kk[c2_i257] += c2_ab_a[c2_i258 + c2_i257] * c2_m_a[c2_i259];
        c2_i258 += 9;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 145U);
    for (c2_i260 = 0; c2_i260 < 3; c2_i260++) {
      c2_b_dx_kk[c2_i260] = c2_dx_kk[c2_i260];
    }

    c2_power(chartInstance, c2_b_dx_kk, c2_a);
    for (c2_i261 = 0; c2_i261 < 3; c2_i261++) {
      c2_bb_a[c2_i261] = c2_a[c2_i261];
    }

    c2_d4 = 1.0 - c2_sum(chartInstance, c2_bb_a);
    c2_c_sqrt(chartInstance, &c2_d4);
    c2_dv41[0] = c2_d4;
    for (c2_i262 = 0; c2_i262 < 3; c2_i262++) {
      c2_dv41[c2_i262 + 1] = c2_dx_kk[c2_i262];
    }

    for (c2_i263 = 0; c2_i263 < 4; c2_i263++) {
      c2_e_x_kkm1[c2_i263] = c2_x_kkm1[c2_i263];
    }

    c2_quatmultiply(chartInstance, c2_dv41, c2_e_x_kkm1, c2_dv28);
    for (c2_i264 = 0; c2_i264 < 4; c2_i264++) {
      chartInstance->c2_x_kk[c2_i264] = c2_dv28[c2_i264];
    }

    for (c2_i265 = 0; c2_i265 < 3; c2_i265++) {
      chartInstance->c2_x_kk[c2_i265 + 4] = c2_x_kkm1[c2_i265 + 4] +
        c2_dx_kk[c2_i265 + 3];
    }

    for (c2_i266 = 0; c2_i266 < 3; c2_i266++) {
      chartInstance->c2_x_kk[c2_i266 + 7] = c2_x_kkm1[c2_i266 + 7] +
        c2_dx_kk[c2_i266 + 6];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 150U);
    for (c2_i267 = 0; c2_i267 < 54; c2_i267++) {
      c2_ab_a[c2_i267] = c2_K_k[c2_i267];
    }

    for (c2_i268 = 0; c2_i268 < 36; c2_i268++) {
      c2_l_b[c2_i268] = c2_P_zkzk[c2_i268];
    }

    c2_e_eml_scalar_eg(chartInstance);
    c2_e_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i269 = 0; c2_i269 < 9; c2_i269++) {
      c2_i270 = 0;
      c2_i271 = 0;
      for (c2_i272 = 0; c2_i272 < 6; c2_i272++) {
        c2_o_y[c2_i270 + c2_i269] = 0.0;
        c2_i273 = 0;
        for (c2_i274 = 0; c2_i274 < 6; c2_i274++) {
          c2_o_y[c2_i270 + c2_i269] += c2_ab_a[c2_i273 + c2_i269] *
            c2_l_b[c2_i274 + c2_i271];
          c2_i273 += 9;
        }

        c2_i270 += 9;
        c2_i271 += 6;
      }
    }

    c2_i275 = 0;
    for (c2_i276 = 0; c2_i276 < 9; c2_i276++) {
      c2_i277 = 0;
      for (c2_i278 = 0; c2_i278 < 6; c2_i278++) {
        c2_m_b[c2_i278 + c2_i275] = c2_K_k[c2_i277 + c2_i276];
        c2_i277 += 9;
      }

      c2_i275 += 6;
    }

    c2_f_eml_scalar_eg(chartInstance);
    c2_f_eml_scalar_eg(chartInstance);
    c2_threshold(chartInstance);
    for (c2_i279 = 0; c2_i279 < 9; c2_i279++) {
      c2_i280 = 0;
      c2_i281 = 0;
      for (c2_i282 = 0; c2_i282 < 9; c2_i282++) {
        c2_c_hoistedGlobal[c2_i280 + c2_i279] = 0.0;
        c2_i283 = 0;
        for (c2_i284 = 0; c2_i284 < 6; c2_i284++) {
          c2_c_hoistedGlobal[c2_i280 + c2_i279] += c2_o_y[c2_i283 + c2_i279] *
            c2_m_b[c2_i284 + c2_i281];
          c2_i283 += 9;
        }

        c2_i280 += 9;
        c2_i281 += 6;
      }
    }

    for (c2_i285 = 0; c2_i285 < 81; c2_i285++) {
      c2_P_kk[c2_i285] = c2_P_kkm1[c2_i285] - c2_c_hoistedGlobal[c2_i285];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 155U);
  for (c2_i286 = 0; c2_i286 < 81; c2_i286++) {
    chartInstance->c2_P_km1km1[c2_i286] = c2_P_kk[c2_i286];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 156U);
  for (c2_i287 = 0; c2_i287 < 10; c2_i287++) {
    chartInstance->c2_x_km1km1[c2_i287] = chartInstance->c2_x_kk[c2_i287];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 159U);
  for (c2_i288 = 0; c2_i288 < 4; c2_i288++) {
    c2_dv42[c2_i288] = chartInstance->c2_q_s_c[c2_i288];
  }

  c2_quatinv(chartInstance, c2_dv42, c2_dv28);
  for (c2_i289 = 0; c2_i289 < 4; c2_i289++) {
    c2_dv43[c2_i289] = chartInstance->c2_x_kk[c2_i289];
  }

  for (c2_i290 = 0; c2_i290 < 4; c2_i290++) {
    c2_dv44[c2_i290] = c2_dv28[c2_i290];
  }

  c2_quatmultiply(chartInstance, c2_dv43, c2_dv44, c2_dv45);
  for (c2_i291 = 0; c2_i291 < 4; c2_i291++) {
    c2_Attitude_sensor[c2_i291] = c2_dv45[c2_i291];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 160U);
  for (c2_i292 = 0; c2_i292 < 3; c2_i292++) {
    c2_dv46[c2_i292] = chartInstance->c2_x_kk[c2_i292 + 4];
  }

  for (c2_i293 = 0; c2_i293 < 4; c2_i293++) {
    c2_dv47[c2_i293] = chartInstance->c2_q_s_c[c2_i293];
  }

  c2_RotateVecCont2Sensor(chartInstance, c2_dv46, c2_dv47, c2_dv48);
  for (c2_i294 = 0; c2_i294 < 3; c2_i294++) {
    c2_w_sensor[c2_i294] = c2_dv48[c2_i294];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 161U);
  for (c2_i295 = 0; c2_i295 < 3; c2_i295++) {
    c2_dv49[c2_i295] = chartInstance->c2_x_kk[c2_i295 + 7];
  }

  for (c2_i296 = 0; c2_i296 < 4; c2_i296++) {
    c2_dv50[c2_i296] = chartInstance->c2_q_s_c[c2_i296];
  }

  c2_RotateVecCont2Sensor(chartInstance, c2_dv49, c2_dv50, c2_dv51);
  for (c2_i297 = 0; c2_i297 < 3; c2_i297++) {
    c2_w_bias_sensor[c2_i297] = c2_dv51[c2_i297];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -161);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i298 = 0; c2_i298 < 4; c2_i298++) {
    (*c2_b_Attitude_sensor)[c2_i298] = c2_Attitude_sensor[c2_i298];
  }

  for (c2_i299 = 0; c2_i299 < 3; c2_i299++) {
    (*c2_b_w_sensor)[c2_i299] = c2_w_sensor[c2_i299];
  }

  for (c2_i300 = 0; c2_i300 < 3; c2_i300++) {
    (*c2_b_w_bias_sensor)[c2_i300] = c2_w_bias_sensor[c2_i300];
  }

  for (c2_i301 = 0; c2_i301 < 54; c2_i301++) {
    (*c2_b_Kalman_Gain)[c2_i301] = c2_Kalman_Gain[c2_i301];
  }

  for (c2_i302 = 0; c2_i302 < 36; c2_i302++) {
    (*c2_b_noise_covarience)[c2_i302] = c2_noise_covarience[c2_i302];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_UKF_10hz(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i303;
  real_T c2_b_inData[3];
  int32_T c2_i304;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i303 = 0; c2_i303 < 3; c2_i303++) {
    c2_b_inData[c2_i303] = (*(real_T (*)[3])c2_inData)[c2_i303];
  }

  for (c2_i304 = 0; c2_i304 < 3; c2_i304++) {
    c2_u[c2_i304] = c2_b_inData[c2_i304];
  }

  c2_y = NULL;
  if (!chartInstance->c2_I_c_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_I_c, const char_T *c2_identifier, real_T c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_I_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_I_c);
}

static void c2_b_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv52[3];
  int32_T c2_i305;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_I_c_not_empty = false;
  } else {
    chartInstance->c2_I_c_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv52, 1, 0, 0U, 1, 0U, 2, 1,
                  3);
    for (c2_i305 = 0; c2_i305 < 3; c2_i305++) {
      c2_y[c2_i305] = c2_dv52[c2_i305];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_I_c;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i306;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_I_c = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_I_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_I_c);
  for (c2_i306 = 0; c2_i306 < 3; c2_i306++) {
    (*(real_T (*)[3])c2_outData)[c2_i306] = c2_y[c2_i306];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i307;
  real_T c2_b_inData[4];
  int32_T c2_i308;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i307 = 0; c2_i307 < 4; c2_i307++) {
    c2_b_inData[c2_i307] = (*(real_T (*)[4])c2_inData)[c2_i307];
  }

  for (c2_i308 = 0; c2_i308 < 4; c2_i308++) {
    c2_u[c2_i308] = c2_b_inData[c2_i308];
  }

  c2_y = NULL;
  if (!chartInstance->c2_q_s_c_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_q_s_c, const char_T *c2_identifier, real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_q_s_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_q_s_c);
}

static void c2_d_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv53[4];
  int32_T c2_i309;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_q_s_c_not_empty = false;
  } else {
    chartInstance->c2_q_s_c_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv53, 1, 0, 0U, 1, 0U, 1, 4);
    for (c2_i309 = 0; c2_i309 < 4; c2_i309++) {
      c2_y[c2_i309] = c2_dv53[c2_i309];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_q_s_c;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i310;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_q_s_c = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_q_s_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_q_s_c);
  for (c2_i310 = 0; c2_i310 < 4; c2_i310++) {
    (*(real_T (*)[4])c2_outData)[c2_i310] = c2_y[c2_i310];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Beta_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_e_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Beta, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Beta), &c2_thisId);
  sf_mex_destroy(&c2_b_Beta);
  return c2_y;
}

static real_T c2_f_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d5;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Beta_not_empty = false;
  } else {
    chartInstance->c2_Beta_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d5, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d5;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Beta;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_Beta = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Beta), &c2_thisId);
  sf_mex_destroy(&c2_b_Beta);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_K_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_g_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_K, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K), &c2_thisId);
  sf_mex_destroy(&c2_b_K);
  return c2_y;
}

static real_T c2_h_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d6;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_K_not_empty = false;
  } else {
    chartInstance->c2_K_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d6, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d6;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_K;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_K = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K), &c2_thisId);
  sf_mex_destroy(&c2_b_K);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_alpha_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_i_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_alpha, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_alpha), &c2_thisId);
  sf_mex_destroy(&c2_b_alpha);
  return c2_y;
}

static real_T c2_j_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d7;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_alpha_not_empty = false;
  } else {
    chartInstance->c2_alpha_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d7, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d7;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_alpha;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_alpha = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_alpha), &c2_thisId);
  sf_mex_destroy(&c2_b_alpha);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Lambda_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_k_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Lambda, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Lambda),
    &c2_thisId);
  sf_mex_destroy(&c2_b_Lambda);
  return c2_y;
}

static real_T c2_l_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d8;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Lambda_not_empty = false;
  } else {
    chartInstance->c2_Lambda_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d8, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d8;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Lambda;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_Lambda = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Lambda),
    &c2_thisId);
  sf_mex_destroy(&c2_b_Lambda);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i311;
  int32_T c2_i312;
  int32_T c2_i313;
  real_T c2_b_inData[81];
  int32_T c2_i314;
  int32_T c2_i315;
  int32_T c2_i316;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i311 = 0;
  for (c2_i312 = 0; c2_i312 < 9; c2_i312++) {
    for (c2_i313 = 0; c2_i313 < 9; c2_i313++) {
      c2_b_inData[c2_i313 + c2_i311] = (*(real_T (*)[81])c2_inData)[c2_i313 +
        c2_i311];
    }

    c2_i311 += 9;
  }

  c2_i314 = 0;
  for (c2_i315 = 0; c2_i315 < 9; c2_i315++) {
    for (c2_i316 = 0; c2_i316 < 9; c2_i316++) {
      c2_u[c2_i316 + c2_i314] = c2_b_inData[c2_i316 + c2_i314];
    }

    c2_i314 += 9;
  }

  c2_y = NULL;
  if (!chartInstance->c2_Q_k_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_m_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_Q_k, const char_T *c2_identifier, real_T c2_y[81])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_Q_k);
}

static void c2_n_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81])
{
  real_T c2_dv54[81];
  int32_T c2_i317;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Q_k_not_empty = false;
  } else {
    chartInstance->c2_Q_k_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv54, 1, 0, 0U, 1, 0U, 2, 9,
                  9);
    for (c2_i317 = 0; c2_i317 < 81; c2_i317++) {
      c2_y[c2_i317] = c2_dv54[c2_i317];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Q_k;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i318;
  int32_T c2_i319;
  int32_T c2_i320;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_Q_k = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_Q_k);
  c2_i318 = 0;
  for (c2_i319 = 0; c2_i319 < 9; c2_i319++) {
    for (c2_i320 = 0; c2_i320 < 9; c2_i320++) {
      (*(real_T (*)[81])c2_outData)[c2_i320 + c2_i318] = c2_y[c2_i320 + c2_i318];
    }

    c2_i318 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i321;
  int32_T c2_i322;
  int32_T c2_i323;
  real_T c2_b_inData[36];
  int32_T c2_i324;
  int32_T c2_i325;
  int32_T c2_i326;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i321 = 0;
  for (c2_i322 = 0; c2_i322 < 6; c2_i322++) {
    for (c2_i323 = 0; c2_i323 < 6; c2_i323++) {
      c2_b_inData[c2_i323 + c2_i321] = (*(real_T (*)[36])c2_inData)[c2_i323 +
        c2_i321];
    }

    c2_i321 += 6;
  }

  c2_i324 = 0;
  for (c2_i325 = 0; c2_i325 < 6; c2_i325++) {
    for (c2_i326 = 0; c2_i326 < 6; c2_i326++) {
      c2_u[c2_i326 + c2_i324] = c2_b_inData[c2_i326 + c2_i324];
    }

    c2_i324 += 6;
  }

  c2_y = NULL;
  if (!chartInstance->c2_R_k_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_o_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_R_k, const char_T *c2_identifier, real_T c2_y[36])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_R_k);
}

static void c2_p_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36])
{
  real_T c2_dv55[36];
  int32_T c2_i327;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_R_k_not_empty = false;
  } else {
    chartInstance->c2_R_k_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv55, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c2_i327 = 0; c2_i327 < 36; c2_i327++) {
      c2_y[c2_i327] = c2_dv55[c2_i327];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_R_k;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i328;
  int32_T c2_i329;
  int32_T c2_i330;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_R_k = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_R_k);
  c2_i328 = 0;
  for (c2_i329 = 0; c2_i329 < 6; c2_i329++) {
    for (c2_i330 = 0; c2_i330 < 6; c2_i330++) {
      (*(real_T (*)[36])c2_outData)[c2_i330 + c2_i328] = c2_y[c2_i330 + c2_i328];
    }

    c2_i328 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i331;
  real_T c2_b_inData[10];
  int32_T c2_i332;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i331 = 0; c2_i331 < 10; c2_i331++) {
    c2_b_inData[c2_i331] = (*(real_T (*)[10])c2_inData)[c2_i331];
  }

  for (c2_i332 = 0; c2_i332 < 10; c2_i332++) {
    c2_u[c2_i332] = c2_b_inData[c2_i332];
  }

  c2_y = NULL;
  if (!chartInstance->c2_x_kk_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_q_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_x_kk, const char_T *c2_identifier, real_T c2_y[10])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_x_kk);
}

static void c2_r_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10])
{
  real_T c2_dv56[10];
  int32_T c2_i333;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_x_kk_not_empty = false;
  } else {
    chartInstance->c2_x_kk_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv56, 1, 0, 0U, 1, 0U, 1, 10);
    for (c2_i333 = 0; c2_i333 < 10; c2_i333++) {
      c2_y[c2_i333] = c2_dv56[c2_i333];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_x_kk;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[10];
  int32_T c2_i334;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_x_kk = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_x_kk);
  for (c2_i334 = 0; c2_i334 < 10; c2_i334++) {
    (*(real_T (*)[10])c2_outData)[c2_i334] = c2_y[c2_i334];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i335;
  real_T c2_b_inData[10];
  int32_T c2_i336;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i335 = 0; c2_i335 < 10; c2_i335++) {
    c2_b_inData[c2_i335] = (*(real_T (*)[10])c2_inData)[c2_i335];
  }

  for (c2_i336 = 0; c2_i336 < 10; c2_i336++) {
    c2_u[c2_i336] = c2_b_inData[c2_i336];
  }

  c2_y = NULL;
  if (!chartInstance->c2_x_km1km1_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_s_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_x_km1km1, const char_T *c2_identifier, real_T c2_y[10])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_x_km1km1);
}

static void c2_t_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10])
{
  real_T c2_dv57[10];
  int32_T c2_i337;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_x_km1km1_not_empty = false;
  } else {
    chartInstance->c2_x_km1km1_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv57, 1, 0, 0U, 1, 0U, 1, 10);
    for (c2_i337 = 0; c2_i337 < 10; c2_i337++) {
      c2_y[c2_i337] = c2_dv57[c2_i337];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_x_km1km1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[10];
  int32_T c2_i338;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_x_km1km1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_x_km1km1);
  for (c2_i338 = 0; c2_i338 < 10; c2_i338++) {
    (*(real_T (*)[10])c2_outData)[c2_i338] = c2_y[c2_i338];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i339;
  int32_T c2_i340;
  int32_T c2_i341;
  real_T c2_b_inData[81];
  int32_T c2_i342;
  int32_T c2_i343;
  int32_T c2_i344;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i339 = 0;
  for (c2_i340 = 0; c2_i340 < 9; c2_i340++) {
    for (c2_i341 = 0; c2_i341 < 9; c2_i341++) {
      c2_b_inData[c2_i341 + c2_i339] = (*(real_T (*)[81])c2_inData)[c2_i341 +
        c2_i339];
    }

    c2_i339 += 9;
  }

  c2_i342 = 0;
  for (c2_i343 = 0; c2_i343 < 9; c2_i343++) {
    for (c2_i344 = 0; c2_i344 < 9; c2_i344++) {
      c2_u[c2_i344 + c2_i342] = c2_b_inData[c2_i344 + c2_i342];
    }

    c2_i342 += 9;
  }

  c2_y = NULL;
  if (!chartInstance->c2_P_km1km1_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), false);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_u_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_P_km1km1, const char_T *c2_identifier, real_T c2_y[81])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_P_km1km1);
}

static void c2_v_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81])
{
  real_T c2_dv58[81];
  int32_T c2_i345;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_P_km1km1_not_empty = false;
  } else {
    chartInstance->c2_P_km1km1_not_empty = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv58, 1, 0, 0U, 1, 0U, 2, 9,
                  9);
    for (c2_i345 = 0; c2_i345 < 81; c2_i345++) {
      c2_y[c2_i345] = c2_dv58[c2_i345];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_P_km1km1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i346;
  int32_T c2_i347;
  int32_T c2_i348;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_P_km1km1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_P_km1km1);
  c2_i346 = 0;
  for (c2_i347 = 0; c2_i347 < 9; c2_i347++) {
    for (c2_i348 = 0; c2_i348 < 9; c2_i348++) {
      (*(real_T (*)[81])c2_outData)[c2_i348 + c2_i346] = c2_y[c2_i348 + c2_i346];
    }

    c2_i346 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i349;
  int32_T c2_i350;
  int32_T c2_i351;
  real_T c2_b_inData[36];
  int32_T c2_i352;
  int32_T c2_i353;
  int32_T c2_i354;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i349 = 0;
  for (c2_i350 = 0; c2_i350 < 6; c2_i350++) {
    for (c2_i351 = 0; c2_i351 < 6; c2_i351++) {
      c2_b_inData[c2_i351 + c2_i349] = (*(real_T (*)[36])c2_inData)[c2_i351 +
        c2_i349];
    }

    c2_i349 += 6;
  }

  c2_i352 = 0;
  for (c2_i353 = 0; c2_i353 < 6; c2_i353++) {
    for (c2_i354 = 0; c2_i354 < 6; c2_i354++) {
      c2_u[c2_i354 + c2_i352] = c2_b_inData[c2_i354 + c2_i352];
    }

    c2_i352 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_w_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_noise_covarience, const char_T *c2_identifier, real_T c2_y
  [36])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_noise_covarience),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_noise_covarience);
}

static void c2_x_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[36])
{
  real_T c2_dv59[36];
  int32_T c2_i355;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv59, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c2_i355 = 0; c2_i355 < 36; c2_i355++) {
    c2_y[c2_i355] = c2_dv59[c2_i355];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_noise_covarience;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i356;
  int32_T c2_i357;
  int32_T c2_i358;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_noise_covarience = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_noise_covarience),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_noise_covarience);
  c2_i356 = 0;
  for (c2_i357 = 0; c2_i357 < 6; c2_i357++) {
    for (c2_i358 = 0; c2_i358 < 6; c2_i358++) {
      (*(real_T (*)[36])c2_outData)[c2_i358 + c2_i356] = c2_y[c2_i358 + c2_i356];
    }

    c2_i356 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i359;
  int32_T c2_i360;
  int32_T c2_i361;
  real_T c2_b_inData[54];
  int32_T c2_i362;
  int32_T c2_i363;
  int32_T c2_i364;
  real_T c2_u[54];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i359 = 0;
  for (c2_i360 = 0; c2_i360 < 6; c2_i360++) {
    for (c2_i361 = 0; c2_i361 < 9; c2_i361++) {
      c2_b_inData[c2_i361 + c2_i359] = (*(real_T (*)[54])c2_inData)[c2_i361 +
        c2_i359];
    }

    c2_i359 += 9;
  }

  c2_i362 = 0;
  for (c2_i363 = 0; c2_i363 < 6; c2_i363++) {
    for (c2_i364 = 0; c2_i364 < 9; c2_i364++) {
      c2_u[c2_i364 + c2_i362] = c2_b_inData[c2_i364 + c2_i362];
    }

    c2_i362 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_y_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_Kalman_Gain, const char_T *c2_identifier, real_T c2_y[54])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Kalman_Gain), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_Kalman_Gain);
}

static void c2_ab_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[54])
{
  real_T c2_dv60[54];
  int32_T c2_i365;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv60, 1, 0, 0U, 1, 0U, 2, 9, 6);
  for (c2_i365 = 0; c2_i365 < 54; c2_i365++) {
    c2_y[c2_i365] = c2_dv60[c2_i365];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Kalman_Gain;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[54];
  int32_T c2_i366;
  int32_T c2_i367;
  int32_T c2_i368;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_Kalman_Gain = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Kalman_Gain), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_Kalman_Gain);
  c2_i366 = 0;
  for (c2_i367 = 0; c2_i367 < 6; c2_i367++) {
    for (c2_i368 = 0; c2_i368 < 9; c2_i368++) {
      (*(real_T (*)[54])c2_outData)[c2_i368 + c2_i366] = c2_y[c2_i368 + c2_i366];
    }

    c2_i366 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i369;
  real_T c2_b_inData[3];
  int32_T c2_i370;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i369 = 0; c2_i369 < 3; c2_i369++) {
    c2_b_inData[c2_i369] = (*(real_T (*)[3])c2_inData)[c2_i369];
  }

  for (c2_i370 = 0; c2_i370 < 3; c2_i370++) {
    c2_u[c2_i370] = c2_b_inData[c2_i370];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_bb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_w_bias_sensor, const char_T *c2_identifier, real_T c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_bias_sensor), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_w_bias_sensor);
}

static void c2_cb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv61[3];
  int32_T c2_i371;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv61, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i371 = 0; c2_i371 < 3; c2_i371++) {
    c2_y[c2_i371] = c2_dv61[c2_i371];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_w_bias_sensor;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i372;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_w_bias_sensor = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_bias_sensor), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_w_bias_sensor);
  for (c2_i372 = 0; c2_i372 < 3; c2_i372++) {
    (*(real_T (*)[3])c2_outData)[c2_i372] = c2_y[c2_i372];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i373;
  real_T c2_b_inData[4];
  int32_T c2_i374;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i373 = 0; c2_i373 < 4; c2_i373++) {
    c2_b_inData[c2_i373] = (*(real_T (*)[4])c2_inData)[c2_i373];
  }

  for (c2_i374 = 0; c2_i374 < 4; c2_i374++) {
    c2_u[c2_i374] = c2_b_inData[c2_i374];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_db_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_Attitude_sensor, const char_T *c2_identifier, real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Attitude_sensor),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Attitude_sensor);
}

static void c2_eb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv62[4];
  int32_T c2_i375;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv62, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i375 = 0; c2_i375 < 4; c2_i375++) {
    c2_y[c2_i375] = c2_dv62[c2_i375];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Attitude_sensor;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i376;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_Attitude_sensor = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Attitude_sensor),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Attitude_sensor);
  for (c2_i376 = 0; c2_i376 < 4; c2_i376++) {
    (*(real_T (*)[4])c2_outData)[c2_i376] = c2_y[c2_i376];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_fb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d9;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d9, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d9;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout),
    &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i377;
  real_T c2_b_inData[9];
  int32_T c2_i378;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i377 = 0; c2_i377 < 9; c2_i377++) {
    c2_b_inData[c2_i377] = (*(real_T (*)[9])c2_inData)[c2_i377];
  }

  for (c2_i378 = 0; c2_i378 < 9; c2_i378++) {
    c2_u[c2_i378] = c2_b_inData[c2_i378];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 9), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_gb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9])
{
  real_T c2_dv63[9];
  int32_T c2_i379;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv63, 1, 0, 0U, 1, 0U, 1, 9);
  for (c2_i379 = 0; c2_i379 < 9; c2_i379++) {
    c2_y[c2_i379] = c2_dv63[c2_i379];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dx_kk;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i380;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_dx_kk = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dx_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dx_kk);
  for (c2_i380 = 0; c2_i380 < 9; c2_i380++) {
    (*(real_T (*)[9])c2_outData)[c2_i380] = c2_y[c2_i380];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i381;
  real_T c2_b_inData[6];
  int32_T c2_i382;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i381 = 0; c2_i381 < 6; c2_i381++) {
    c2_b_inData[c2_i381] = (*(real_T (*)[6])c2_inData)[c2_i381];
  }

  for (c2_i382 = 0; c2_i382 < 6; c2_i382++) {
    c2_u[c2_i382] = c2_b_inData[c2_i382];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_hb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[6])
{
  real_T c2_dv64[6];
  int32_T c2_i383;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv64, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i383 = 0; c2_i383 < 6; c2_i383++) {
    c2_y[c2_i383] = c2_dv64[c2_i383];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_z_kkm1_control;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i384;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_z_kkm1_control = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_hb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_z_kkm1_control),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_z_kkm1_control);
  for (c2_i384 = 0; c2_i384 < 6; c2_i384++) {
    (*(real_T (*)[6])c2_outData)[c2_i384] = c2_y[c2_i384];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i385;
  int32_T c2_i386;
  int32_T c2_i387;
  real_T c2_b_inData[114];
  int32_T c2_i388;
  int32_T c2_i389;
  int32_T c2_i390;
  real_T c2_u[114];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i385 = 0;
  for (c2_i386 = 0; c2_i386 < 19; c2_i386++) {
    for (c2_i387 = 0; c2_i387 < 6; c2_i387++) {
      c2_b_inData[c2_i387 + c2_i385] = (*(real_T (*)[114])c2_inData)[c2_i387 +
        c2_i385];
    }

    c2_i385 += 6;
  }

  c2_i388 = 0;
  for (c2_i389 = 0; c2_i389 < 19; c2_i389++) {
    for (c2_i390 = 0; c2_i390 < 6; c2_i390++) {
      c2_u[c2_i390 + c2_i388] = c2_b_inData[c2_i390 + c2_i388];
    }

    c2_i388 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 19), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_ib_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[114])
{
  real_T c2_dv65[114];
  int32_T c2_i391;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv65, 1, 0, 0U, 1, 0U, 2, 6,
                19);
  for (c2_i391 = 0; c2_i391 < 114; c2_i391++) {
    c2_y[c2_i391] = c2_dv65[c2_i391];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Z_kkm1_control;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[114];
  int32_T c2_i392;
  int32_T c2_i393;
  int32_T c2_i394;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_Z_kkm1_control = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Z_kkm1_control),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Z_kkm1_control);
  c2_i392 = 0;
  for (c2_i393 = 0; c2_i393 < 19; c2_i393++) {
    for (c2_i394 = 0; c2_i394 < 6; c2_i394++) {
      (*(real_T (*)[114])c2_outData)[c2_i394 + c2_i392] = c2_y[c2_i394 + c2_i392];
    }

    c2_i392 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_t_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i395;
  int32_T c2_i396;
  int32_T c2_i397;
  real_T c2_b_inData[81];
  int32_T c2_i398;
  int32_T c2_i399;
  int32_T c2_i400;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i395 = 0;
  for (c2_i396 = 0; c2_i396 < 9; c2_i396++) {
    for (c2_i397 = 0; c2_i397 < 9; c2_i397++) {
      c2_b_inData[c2_i397 + c2_i395] = (*(real_T (*)[81])c2_inData)[c2_i397 +
        c2_i395];
    }

    c2_i395 += 9;
  }

  c2_i398 = 0;
  for (c2_i399 = 0; c2_i399 < 9; c2_i399++) {
    for (c2_i400 = 0; c2_i400 < 9; c2_i400++) {
      c2_u[c2_i400 + c2_i398] = c2_b_inData[c2_i400 + c2_i398];
    }

    c2_i398 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_jb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[81])
{
  real_T c2_dv66[81];
  int32_T c2_i401;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv66, 1, 0, 0U, 1, 0U, 2, 9, 9);
  for (c2_i401 = 0; c2_i401 < 81; c2_i401++) {
    c2_y[c2_i401] = c2_dv66[c2_i401];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_P_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i402;
  int32_T c2_i403;
  int32_T c2_i404;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_P_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_jb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_P_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_P_kkm1);
  c2_i402 = 0;
  for (c2_i403 = 0; c2_i403 < 9; c2_i403++) {
    for (c2_i404 = 0; c2_i404 < 9; c2_i404++) {
      (*(real_T (*)[81])c2_outData)[c2_i404 + c2_i402] = c2_y[c2_i404 + c2_i402];
    }

    c2_i402 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_u_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i405;
  int32_T c2_i406;
  int32_T c2_i407;
  real_T c2_b_inData[171];
  int32_T c2_i408;
  int32_T c2_i409;
  int32_T c2_i410;
  real_T c2_u[171];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i405 = 0;
  for (c2_i406 = 0; c2_i406 < 19; c2_i406++) {
    for (c2_i407 = 0; c2_i407 < 9; c2_i407++) {
      c2_b_inData[c2_i407 + c2_i405] = (*(real_T (*)[171])c2_inData)[c2_i407 +
        c2_i405];
    }

    c2_i405 += 9;
  }

  c2_i408 = 0;
  for (c2_i409 = 0; c2_i409 < 19; c2_i409++) {
    for (c2_i410 = 0; c2_i410 < 9; c2_i410++) {
      c2_u[c2_i410 + c2_i408] = c2_b_inData[c2_i410 + c2_i408];
    }

    c2_i408 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 19), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_kb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[171])
{
  real_T c2_dv67[171];
  int32_T c2_i411;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv67, 1, 0, 0U, 1, 0U, 2, 9,
                19);
  for (c2_i411 = 0; c2_i411 < 171; c2_i411++) {
    c2_y[c2_i411] = c2_dv67[c2_i411];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dX_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[171];
  int32_T c2_i412;
  int32_T c2_i413;
  int32_T c2_i414;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_dX_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dX_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dX_kkm1);
  c2_i412 = 0;
  for (c2_i413 = 0; c2_i413 < 19; c2_i413++) {
    for (c2_i414 = 0; c2_i414 < 9; c2_i414++) {
      (*(real_T (*)[171])c2_outData)[c2_i414 + c2_i412] = c2_y[c2_i414 + c2_i412];
    }

    c2_i412 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_v_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i415;
  int32_T c2_i416;
  int32_T c2_i417;
  real_T c2_b_inData[76];
  int32_T c2_i418;
  int32_T c2_i419;
  int32_T c2_i420;
  real_T c2_u[76];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i415 = 0;
  for (c2_i416 = 0; c2_i416 < 19; c2_i416++) {
    for (c2_i417 = 0; c2_i417 < 4; c2_i417++) {
      c2_b_inData[c2_i417 + c2_i415] = (*(real_T (*)[76])c2_inData)[c2_i417 +
        c2_i415];
    }

    c2_i415 += 4;
  }

  c2_i418 = 0;
  for (c2_i419 = 0; c2_i419 < 19; c2_i419++) {
    for (c2_i420 = 0; c2_i420 < 4; c2_i420++) {
      c2_u[c2_i420 + c2_i418] = c2_b_inData[c2_i420 + c2_i418];
    }

    c2_i418 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 19), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_lb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[76])
{
  real_T c2_dv68[76];
  int32_T c2_i421;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv68, 1, 0, 0U, 1, 0U, 2, 4,
                19);
  for (c2_i421 = 0; c2_i421 < 76; c2_i421++) {
    c2_y[c2_i421] = c2_dv68[c2_i421];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dX_kkm1_temp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[76];
  int32_T c2_i422;
  int32_T c2_i423;
  int32_T c2_i424;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_dX_kkm1_temp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_lb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dX_kkm1_temp), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_dX_kkm1_temp);
  c2_i422 = 0;
  for (c2_i423 = 0; c2_i423 < 19; c2_i423++) {
    for (c2_i424 = 0; c2_i424 < 4; c2_i424++) {
      (*(real_T (*)[76])c2_outData)[c2_i424 + c2_i422] = c2_y[c2_i424 + c2_i422];
    }

    c2_i422 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_w_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i425;
  real_T c2_b_inData[10];
  int32_T c2_i426;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i425 = 0; c2_i425 < 10; c2_i425++) {
    c2_b_inData[c2_i425] = (*(real_T (*)[10])c2_inData)[c2_i425];
  }

  for (c2_i426 = 0; c2_i426 < 10; c2_i426++) {
    c2_u[c2_i426] = c2_b_inData[c2_i426];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_mb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[10])
{
  real_T c2_dv69[10];
  int32_T c2_i427;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv69, 1, 0, 0U, 1, 0U, 1, 10);
  for (c2_i427 = 0; c2_i427 < 10; c2_i427++) {
    c2_y[c2_i427] = c2_dv69[c2_i427];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_x_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[10];
  int32_T c2_i428;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_x_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_x_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_x_kkm1);
  for (c2_i428 = 0; c2_i428 < 10; c2_i428++) {
    (*(real_T (*)[10])c2_outData)[c2_i428] = c2_y[c2_i428];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_x_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i429;
  int32_T c2_i430;
  int32_T c2_i431;
  real_T c2_b_inData[190];
  int32_T c2_i432;
  int32_T c2_i433;
  int32_T c2_i434;
  real_T c2_u[190];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i429 = 0;
  for (c2_i430 = 0; c2_i430 < 19; c2_i430++) {
    for (c2_i431 = 0; c2_i431 < 10; c2_i431++) {
      c2_b_inData[c2_i431 + c2_i429] = (*(real_T (*)[190])c2_inData)[c2_i431 +
        c2_i429];
    }

    c2_i429 += 10;
  }

  c2_i432 = 0;
  for (c2_i433 = 0; c2_i433 < 19; c2_i433++) {
    for (c2_i434 = 0; c2_i434 < 10; c2_i434++) {
      c2_u[c2_i434 + c2_i432] = c2_b_inData[c2_i434 + c2_i432];
    }

    c2_i432 += 10;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 10, 19), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_nb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[190])
{
  real_T c2_dv70[190];
  int32_T c2_i435;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv70, 1, 0, 0U, 1, 0U, 2, 10,
                19);
  for (c2_i435 = 0; c2_i435 < 190; c2_i435++) {
    c2_y[c2_i435] = c2_dv70[c2_i435];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_X_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[190];
  int32_T c2_i436;
  int32_T c2_i437;
  int32_T c2_i438;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_X_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_nb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_X_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_X_kkm1);
  c2_i436 = 0;
  for (c2_i437 = 0; c2_i437 < 19; c2_i437++) {
    for (c2_i438 = 0; c2_i438 < 10; c2_i438++) {
      (*(real_T (*)[190])c2_outData)[c2_i438 + c2_i436] = c2_y[c2_i438 + c2_i436];
    }

    c2_i436 += 10;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_y_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i439;
  real_T c2_b_inData[4];
  int32_T c2_i440;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i439 = 0; c2_i439 < 4; c2_i439++) {
    c2_b_inData[c2_i439] = (*(real_T (*)[4])c2_inData)[c2_i439];
  }

  for (c2_i440 = 0; c2_i440 < 4; c2_i440++) {
    c2_u[c2_i440] = c2_b_inData[c2_i440];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_ob_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[4])
{
  real_T c2_dv71[4];
  int32_T c2_i441;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv71, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c2_i441 = 0; c2_i441 < 4; c2_i441++) {
    c2_y[c2_i441] = c2_dv71[c2_i441];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_r;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i442;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_r = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_r), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_r);
  for (c2_i442 = 0; c2_i442 < 4; c2_i442++) {
    (*(real_T (*)[4])c2_outData)[c2_i442] = c2_y[c2_i442];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i443;
  real_T c2_b_inData[3];
  int32_T c2_i444;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i443 = 0; c2_i443 < 3; c2_i443++) {
    c2_b_inData[c2_i443] = (*(real_T (*)[3])c2_inData)[c2_i443];
  }

  for (c2_i444 = 0; c2_i444 < 3; c2_i444++) {
    c2_u[c2_i444] = c2_b_inData[c2_i444];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_pb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv72[3];
  int32_T c2_i445;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv72, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c2_i445 = 0; c2_i445 < 3; c2_i445++) {
    c2_y[c2_i445] = c2_dv72[c2_i445];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_vec;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i446;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_vec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_pb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_vec), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_vec);
  for (c2_i446 = 0; c2_i446 < 3; c2_i446++) {
    (*(real_T (*)[3])c2_outData)[c2_i446] = c2_y[c2_i446];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i447;
  int32_T c2_i448;
  int32_T c2_i449;
  real_T c2_b_inData[76];
  int32_T c2_i450;
  int32_T c2_i451;
  int32_T c2_i452;
  real_T c2_u[76];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i447 = 0;
  for (c2_i448 = 0; c2_i448 < 4; c2_i448++) {
    for (c2_i449 = 0; c2_i449 < 19; c2_i449++) {
      c2_b_inData[c2_i449 + c2_i447] = (*(real_T (*)[76])c2_inData)[c2_i449 +
        c2_i447];
    }

    c2_i447 += 19;
  }

  c2_i450 = 0;
  for (c2_i451 = 0; c2_i451 < 4; c2_i451++) {
    for (c2_i452 = 0; c2_i452 < 19; c2_i452++) {
      c2_u[c2_i452 + c2_i450] = c2_b_inData[c2_i452 + c2_i450];
    }

    c2_i450 += 19;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 19, 4), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_qb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[76])
{
  real_T c2_dv73[76];
  int32_T c2_i453;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv73, 1, 0, 0U, 1, 0U, 2, 19,
                4);
  for (c2_i453 = 0; c2_i453 < 76; c2_i453++) {
    c2_y[c2_i453] = c2_dv73[c2_i453];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_q;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[76];
  int32_T c2_i454;
  int32_T c2_i455;
  int32_T c2_i456;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_q = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_qb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_q);
  c2_i454 = 0;
  for (c2_i455 = 0; c2_i455 < 4; c2_i455++) {
    for (c2_i456 = 0; c2_i456 < 19; c2_i456++) {
      (*(real_T (*)[76])c2_outData)[c2_i456 + c2_i454] = c2_y[c2_i456 + c2_i454];
    }

    c2_i454 += 19;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i457;
  real_T c2_b_inData[19];
  int32_T c2_i458;
  real_T c2_u[19];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i457 = 0; c2_i457 < 19; c2_i457++) {
    c2_b_inData[c2_i457] = (*(real_T (*)[19])c2_inData)[c2_i457];
  }

  for (c2_i458 = 0; c2_i458 < 19; c2_i458++) {
    c2_u[c2_i458] = c2_b_inData[c2_i458];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 19), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_rb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[19])
{
  real_T c2_dv74[19];
  int32_T c2_i459;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv74, 1, 0, 0U, 1, 0U, 1, 19);
  for (c2_i459 = 0; c2_i459 < 19; c2_i459++) {
    c2_y[c2_i459] = c2_dv74[c2_i459];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_scalar;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[19];
  int32_T c2_i460;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_scalar = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_rb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_scalar), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_scalar);
  for (c2_i460 = 0; c2_i460 < 19; c2_i460++) {
    (*(real_T (*)[19])c2_outData)[c2_i460] = c2_y[c2_i460];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_db_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i461;
  int32_T c2_i462;
  int32_T c2_i463;
  real_T c2_b_inData[57];
  int32_T c2_i464;
  int32_T c2_i465;
  int32_T c2_i466;
  real_T c2_u[57];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i461 = 0;
  for (c2_i462 = 0; c2_i462 < 3; c2_i462++) {
    for (c2_i463 = 0; c2_i463 < 19; c2_i463++) {
      c2_b_inData[c2_i463 + c2_i461] = (*(real_T (*)[57])c2_inData)[c2_i463 +
        c2_i461];
    }

    c2_i461 += 19;
  }

  c2_i464 = 0;
  for (c2_i465 = 0; c2_i465 < 3; c2_i465++) {
    for (c2_i466 = 0; c2_i466 < 19; c2_i466++) {
      c2_u[c2_i466 + c2_i464] = c2_b_inData[c2_i466 + c2_i464];
    }

    c2_i464 += 19;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 19, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_sb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[57])
{
  real_T c2_dv75[57];
  int32_T c2_i467;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv75, 1, 0, 0U, 1, 0U, 2, 19,
                3);
  for (c2_i467 = 0; c2_i467 < 57; c2_i467++) {
    c2_y[c2_i467] = c2_dv75[c2_i467];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_vec;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[57];
  int32_T c2_i468;
  int32_T c2_i469;
  int32_T c2_i470;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_vec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_sb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_vec), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_vec);
  c2_i468 = 0;
  for (c2_i469 = 0; c2_i469 < 3; c2_i469++) {
    for (c2_i470 = 0; c2_i470 < 19; c2_i470++) {
      (*(real_T (*)[57])c2_outData)[c2_i470 + c2_i468] = c2_y[c2_i470 + c2_i468];
    }

    c2_i468 += 19;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i471;
  int32_T c2_i472;
  int32_T c2_i473;
  real_T c2_b_inData[9];
  int32_T c2_i474;
  int32_T c2_i475;
  int32_T c2_i476;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i471 = 0;
  for (c2_i472 = 0; c2_i472 < 3; c2_i472++) {
    for (c2_i473 = 0; c2_i473 < 3; c2_i473++) {
      c2_b_inData[c2_i473 + c2_i471] = (*(real_T (*)[9])c2_inData)[c2_i473 +
        c2_i471];
    }

    c2_i471 += 3;
  }

  c2_i474 = 0;
  for (c2_i475 = 0; c2_i475 < 3; c2_i475++) {
    for (c2_i476 = 0; c2_i476 < 3; c2_i476++) {
      c2_u[c2_i476 + c2_i474] = c2_b_inData[c2_i476 + c2_i474];
    }

    c2_i474 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_tb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9])
{
  real_T c2_dv76[9];
  int32_T c2_i477;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv76, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i477 = 0; c2_i477 < 9; c2_i477++) {
    c2_y[c2_i477] = c2_dv76[c2_i477];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_eb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_output;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i478;
  int32_T c2_i479;
  int32_T c2_i480;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_output = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_tb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_output), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_output);
  c2_i478 = 0;
  for (c2_i479 = 0; c2_i479 < 3; c2_i479++) {
    for (c2_i480 = 0; c2_i480 < 3; c2_i480++) {
      (*(real_T (*)[9])c2_outData)[c2_i480 + c2_i478] = c2_y[c2_i480 + c2_i478];
    }

    c2_i478 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_UKF_10hz_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 257, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  c2_b_info_helper(&c2_nameCaptureInfo);
  c2_c_info_helper(&c2_nameCaptureInfo);
  c2_d_info_helper(&c2_nameCaptureInfo);
  c2_e_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  const mxArray *c2_rhs42 = NULL;
  const mxArray *c2_lhs42 = NULL;
  const mxArray *c2_rhs43 = NULL;
  const mxArray *c2_lhs43 = NULL;
  const mxArray *c2_rhs44 = NULL;
  const mxArray *c2_lhs44 = NULL;
  const mxArray *c2_rhs45 = NULL;
  const mxArray *c2_lhs45 = NULL;
  const mxArray *c2_rhs46 = NULL;
  const mxArray *c2_lhs46 = NULL;
  const mxArray *c2_rhs47 = NULL;
  const mxArray *c2_lhs47 = NULL;
  const mxArray *c2_rhs48 = NULL;
  const mxArray *c2_lhs48 = NULL;
  const mxArray *c2_rhs49 = NULL;
  const mxArray *c2_lhs49 = NULL;
  const mxArray *c2_rhs50 = NULL;
  const mxArray *c2_lhs50 = NULL;
  const mxArray *c2_rhs51 = NULL;
  const mxArray *c2_lhs51 = NULL;
  const mxArray *c2_rhs52 = NULL;
  const mxArray *c2_lhs52 = NULL;
  const mxArray *c2_rhs53 = NULL;
  const mxArray *c2_lhs53 = NULL;
  const mxArray *c2_rhs54 = NULL;
  const mxArray *c2_lhs54 = NULL;
  const mxArray *c2_rhs55 = NULL;
  const mxArray *c2_lhs55 = NULL;
  const mxArray *c2_rhs56 = NULL;
  const mxArray *c2_lhs56 = NULL;
  const mxArray *c2_rhs57 = NULL;
  const mxArray *c2_lhs57 = NULL;
  const mxArray *c2_rhs58 = NULL;
  const mxArray *c2_lhs58 = NULL;
  const mxArray *c2_rhs59 = NULL;
  const mxArray *c2_lhs59 = NULL;
  const mxArray *c2_rhs60 = NULL;
  const mxArray *c2_lhs60 = NULL;
  const mxArray *c2_rhs61 = NULL;
  const mxArray *c2_lhs61 = NULL;
  const mxArray *c2_rhs62 = NULL;
  const mxArray *c2_lhs62 = NULL;
  const mxArray *c2_rhs63 = NULL;
  const mxArray *c2_lhs63 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840326U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sum"), "name", "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isequal"), "name", "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840358U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840386U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sqrt"), "name", "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343851986U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_error"), "name", "name",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840338U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840396U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("diag"), "name", "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383898894U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mpower"), "name", "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731878U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("chol"), "name", "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1344493634U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c2_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c2_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_error"), "name", "name",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343851958U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c2_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xpotrf"), "name", "name",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xpotrf.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840408U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c2_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xpotrf.m"),
                  "context", "context", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_lapack_xpotrf"), "name",
                  "name", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xpotrf.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840412U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c2_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xpotrf.m"),
                  "context", "context", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matlab_zpotrf"), "name",
                  "name", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840424U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c2_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c2_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c2_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c2_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c2_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c2_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c2_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c2_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdotc"), "name", "name",
                  55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c2_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c2_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xdotc"),
                  "name", "name", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c2_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "context", "context", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c2_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c2_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c2_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c2_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c2_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c2_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
  sf_mex_destroy(&c2_rhs42);
  sf_mex_destroy(&c2_lhs42);
  sf_mex_destroy(&c2_rhs43);
  sf_mex_destroy(&c2_lhs43);
  sf_mex_destroy(&c2_rhs44);
  sf_mex_destroy(&c2_lhs44);
  sf_mex_destroy(&c2_rhs45);
  sf_mex_destroy(&c2_lhs45);
  sf_mex_destroy(&c2_rhs46);
  sf_mex_destroy(&c2_lhs46);
  sf_mex_destroy(&c2_rhs47);
  sf_mex_destroy(&c2_lhs47);
  sf_mex_destroy(&c2_rhs48);
  sf_mex_destroy(&c2_lhs48);
  sf_mex_destroy(&c2_rhs49);
  sf_mex_destroy(&c2_lhs49);
  sf_mex_destroy(&c2_rhs50);
  sf_mex_destroy(&c2_lhs50);
  sf_mex_destroy(&c2_rhs51);
  sf_mex_destroy(&c2_lhs51);
  sf_mex_destroy(&c2_rhs52);
  sf_mex_destroy(&c2_lhs52);
  sf_mex_destroy(&c2_rhs53);
  sf_mex_destroy(&c2_lhs53);
  sf_mex_destroy(&c2_rhs54);
  sf_mex_destroy(&c2_lhs54);
  sf_mex_destroy(&c2_rhs55);
  sf_mex_destroy(&c2_lhs55);
  sf_mex_destroy(&c2_rhs56);
  sf_mex_destroy(&c2_lhs56);
  sf_mex_destroy(&c2_rhs57);
  sf_mex_destroy(&c2_lhs57);
  sf_mex_destroy(&c2_rhs58);
  sf_mex_destroy(&c2_lhs58);
  sf_mex_destroy(&c2_rhs59);
  sf_mex_destroy(&c2_lhs59);
  sf_mex_destroy(&c2_rhs60);
  sf_mex_destroy(&c2_lhs60);
  sf_mex_destroy(&c2_rhs61);
  sf_mex_destroy(&c2_lhs61);
  sf_mex_destroy(&c2_rhs62);
  sf_mex_destroy(&c2_lhs62);
  sf_mex_destroy(&c2_rhs63);
  sf_mex_destroy(&c2_lhs63);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), false);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  return c2_y;
}

static void c2_b_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs64 = NULL;
  const mxArray *c2_lhs64 = NULL;
  const mxArray *c2_rhs65 = NULL;
  const mxArray *c2_lhs65 = NULL;
  const mxArray *c2_rhs66 = NULL;
  const mxArray *c2_lhs66 = NULL;
  const mxArray *c2_rhs67 = NULL;
  const mxArray *c2_lhs67 = NULL;
  const mxArray *c2_rhs68 = NULL;
  const mxArray *c2_lhs68 = NULL;
  const mxArray *c2_rhs69 = NULL;
  const mxArray *c2_lhs69 = NULL;
  const mxArray *c2_rhs70 = NULL;
  const mxArray *c2_lhs70 = NULL;
  const mxArray *c2_rhs71 = NULL;
  const mxArray *c2_lhs71 = NULL;
  const mxArray *c2_rhs72 = NULL;
  const mxArray *c2_lhs72 = NULL;
  const mxArray *c2_rhs73 = NULL;
  const mxArray *c2_lhs73 = NULL;
  const mxArray *c2_rhs74 = NULL;
  const mxArray *c2_lhs74 = NULL;
  const mxArray *c2_rhs75 = NULL;
  const mxArray *c2_lhs75 = NULL;
  const mxArray *c2_rhs76 = NULL;
  const mxArray *c2_lhs76 = NULL;
  const mxArray *c2_rhs77 = NULL;
  const mxArray *c2_lhs77 = NULL;
  const mxArray *c2_rhs78 = NULL;
  const mxArray *c2_lhs78 = NULL;
  const mxArray *c2_rhs79 = NULL;
  const mxArray *c2_lhs79 = NULL;
  const mxArray *c2_rhs80 = NULL;
  const mxArray *c2_lhs80 = NULL;
  const mxArray *c2_rhs81 = NULL;
  const mxArray *c2_lhs81 = NULL;
  const mxArray *c2_rhs82 = NULL;
  const mxArray *c2_lhs82 = NULL;
  const mxArray *c2_rhs83 = NULL;
  const mxArray *c2_lhs83 = NULL;
  const mxArray *c2_rhs84 = NULL;
  const mxArray *c2_lhs84 = NULL;
  const mxArray *c2_rhs85 = NULL;
  const mxArray *c2_lhs85 = NULL;
  const mxArray *c2_rhs86 = NULL;
  const mxArray *c2_lhs86 = NULL;
  const mxArray *c2_rhs87 = NULL;
  const mxArray *c2_lhs87 = NULL;
  const mxArray *c2_rhs88 = NULL;
  const mxArray *c2_lhs88 = NULL;
  const mxArray *c2_rhs89 = NULL;
  const mxArray *c2_lhs89 = NULL;
  const mxArray *c2_rhs90 = NULL;
  const mxArray *c2_lhs90 = NULL;
  const mxArray *c2_rhs91 = NULL;
  const mxArray *c2_lhs91 = NULL;
  const mxArray *c2_rhs92 = NULL;
  const mxArray *c2_lhs92 = NULL;
  const mxArray *c2_rhs93 = NULL;
  const mxArray *c2_lhs93 = NULL;
  const mxArray *c2_rhs94 = NULL;
  const mxArray *c2_lhs94 = NULL;
  const mxArray *c2_rhs95 = NULL;
  const mxArray *c2_lhs95 = NULL;
  const mxArray *c2_rhs96 = NULL;
  const mxArray *c2_lhs96 = NULL;
  const mxArray *c2_rhs97 = NULL;
  const mxArray *c2_lhs97 = NULL;
  const mxArray *c2_rhs98 = NULL;
  const mxArray *c2_lhs98 = NULL;
  const mxArray *c2_rhs99 = NULL;
  const mxArray *c2_lhs99 = NULL;
  const mxArray *c2_rhs100 = NULL;
  const mxArray *c2_lhs100 = NULL;
  const mxArray *c2_rhs101 = NULL;
  const mxArray *c2_lhs101 = NULL;
  const mxArray *c2_rhs102 = NULL;
  const mxArray *c2_lhs102 = NULL;
  const mxArray *c2_rhs103 = NULL;
  const mxArray *c2_lhs103 = NULL;
  const mxArray *c2_rhs104 = NULL;
  const mxArray *c2_lhs104 = NULL;
  const mxArray *c2_rhs105 = NULL;
  const mxArray *c2_lhs105 = NULL;
  const mxArray *c2_rhs106 = NULL;
  const mxArray *c2_lhs106 = NULL;
  const mxArray *c2_rhs107 = NULL;
  const mxArray *c2_lhs107 = NULL;
  const mxArray *c2_rhs108 = NULL;
  const mxArray *c2_lhs108 = NULL;
  const mxArray *c2_rhs109 = NULL;
  const mxArray *c2_lhs109 = NULL;
  const mxArray *c2_rhs110 = NULL;
  const mxArray *c2_lhs110 = NULL;
  const mxArray *c2_rhs111 = NULL;
  const mxArray *c2_lhs111 = NULL;
  const mxArray *c2_rhs112 = NULL;
  const mxArray *c2_lhs112 = NULL;
  const mxArray *c2_rhs113 = NULL;
  const mxArray *c2_lhs113 = NULL;
  const mxArray *c2_rhs114 = NULL;
  const mxArray *c2_lhs114 = NULL;
  const mxArray *c2_rhs115 = NULL;
  const mxArray *c2_lhs115 = NULL;
  const mxArray *c2_rhs116 = NULL;
  const mxArray *c2_lhs116 = NULL;
  const mxArray *c2_rhs117 = NULL;
  const mxArray *c2_lhs117 = NULL;
  const mxArray *c2_rhs118 = NULL;
  const mxArray *c2_lhs118 = NULL;
  const mxArray *c2_rhs119 = NULL;
  const mxArray *c2_lhs119 = NULL;
  const mxArray *c2_rhs120 = NULL;
  const mxArray *c2_lhs120 = NULL;
  const mxArray *c2_rhs121 = NULL;
  const mxArray *c2_lhs121 = NULL;
  const mxArray *c2_rhs122 = NULL;
  const mxArray *c2_lhs122 = NULL;
  const mxArray *c2_rhs123 = NULL;
  const mxArray *c2_lhs123 = NULL;
  const mxArray *c2_rhs124 = NULL;
  const mxArray *c2_lhs124 = NULL;
  const mxArray *c2_rhs125 = NULL;
  const mxArray *c2_lhs125 = NULL;
  const mxArray *c2_rhs126 = NULL;
  const mxArray *c2_lhs126 = NULL;
  const mxArray *c2_rhs127 = NULL;
  const mxArray *c2_lhs127 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c2_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xdotx"),
                  "name", "name", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c2_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c2_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c2_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c2_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c2_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c2_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemv"), "name", "name",
                  71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c2_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c2_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xgemv"),
                  "name", "name", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c2_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c2_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c2_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c2_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c2_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c2_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c2_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xgemv"),
                  "name", "name", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c2_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c2_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c2_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c2_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c2_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c2_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c2_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!ceval_xgemv"),
                  "context", "context", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c2_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!ceval_xgemv"),
                  "context", "context", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c2_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c2_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m"),
                  "context", "context", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xscal"), "name", "name",
                  90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c2_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c2_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c2_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c2_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c2_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c2_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c2_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xscal"),
                  "name", "name", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c2_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c2_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c2_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c2_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c2_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c2_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m!cholesky"),
                  "context", "context", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c2_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c2_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c2_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c2_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c2_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c2_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c2_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360303950U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c2_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360304188U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c2_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360303950U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c2_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360304188U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c2_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c2_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c2_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c2_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c2_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c2_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c2_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c2_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c2_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c2_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c2_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c2_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840326U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c2_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c2_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m!matrix_to_integer_power"),
                  "context", "context", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("inv"), "name", "name", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m"), "resolved",
                  "resolved", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1305339600U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c2_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c2_rhs64);
  sf_mex_destroy(&c2_lhs64);
  sf_mex_destroy(&c2_rhs65);
  sf_mex_destroy(&c2_lhs65);
  sf_mex_destroy(&c2_rhs66);
  sf_mex_destroy(&c2_lhs66);
  sf_mex_destroy(&c2_rhs67);
  sf_mex_destroy(&c2_lhs67);
  sf_mex_destroy(&c2_rhs68);
  sf_mex_destroy(&c2_lhs68);
  sf_mex_destroy(&c2_rhs69);
  sf_mex_destroy(&c2_lhs69);
  sf_mex_destroy(&c2_rhs70);
  sf_mex_destroy(&c2_lhs70);
  sf_mex_destroy(&c2_rhs71);
  sf_mex_destroy(&c2_lhs71);
  sf_mex_destroy(&c2_rhs72);
  sf_mex_destroy(&c2_lhs72);
  sf_mex_destroy(&c2_rhs73);
  sf_mex_destroy(&c2_lhs73);
  sf_mex_destroy(&c2_rhs74);
  sf_mex_destroy(&c2_lhs74);
  sf_mex_destroy(&c2_rhs75);
  sf_mex_destroy(&c2_lhs75);
  sf_mex_destroy(&c2_rhs76);
  sf_mex_destroy(&c2_lhs76);
  sf_mex_destroy(&c2_rhs77);
  sf_mex_destroy(&c2_lhs77);
  sf_mex_destroy(&c2_rhs78);
  sf_mex_destroy(&c2_lhs78);
  sf_mex_destroy(&c2_rhs79);
  sf_mex_destroy(&c2_lhs79);
  sf_mex_destroy(&c2_rhs80);
  sf_mex_destroy(&c2_lhs80);
  sf_mex_destroy(&c2_rhs81);
  sf_mex_destroy(&c2_lhs81);
  sf_mex_destroy(&c2_rhs82);
  sf_mex_destroy(&c2_lhs82);
  sf_mex_destroy(&c2_rhs83);
  sf_mex_destroy(&c2_lhs83);
  sf_mex_destroy(&c2_rhs84);
  sf_mex_destroy(&c2_lhs84);
  sf_mex_destroy(&c2_rhs85);
  sf_mex_destroy(&c2_lhs85);
  sf_mex_destroy(&c2_rhs86);
  sf_mex_destroy(&c2_lhs86);
  sf_mex_destroy(&c2_rhs87);
  sf_mex_destroy(&c2_lhs87);
  sf_mex_destroy(&c2_rhs88);
  sf_mex_destroy(&c2_lhs88);
  sf_mex_destroy(&c2_rhs89);
  sf_mex_destroy(&c2_lhs89);
  sf_mex_destroy(&c2_rhs90);
  sf_mex_destroy(&c2_lhs90);
  sf_mex_destroy(&c2_rhs91);
  sf_mex_destroy(&c2_lhs91);
  sf_mex_destroy(&c2_rhs92);
  sf_mex_destroy(&c2_lhs92);
  sf_mex_destroy(&c2_rhs93);
  sf_mex_destroy(&c2_lhs93);
  sf_mex_destroy(&c2_rhs94);
  sf_mex_destroy(&c2_lhs94);
  sf_mex_destroy(&c2_rhs95);
  sf_mex_destroy(&c2_lhs95);
  sf_mex_destroy(&c2_rhs96);
  sf_mex_destroy(&c2_lhs96);
  sf_mex_destroy(&c2_rhs97);
  sf_mex_destroy(&c2_lhs97);
  sf_mex_destroy(&c2_rhs98);
  sf_mex_destroy(&c2_lhs98);
  sf_mex_destroy(&c2_rhs99);
  sf_mex_destroy(&c2_lhs99);
  sf_mex_destroy(&c2_rhs100);
  sf_mex_destroy(&c2_lhs100);
  sf_mex_destroy(&c2_rhs101);
  sf_mex_destroy(&c2_lhs101);
  sf_mex_destroy(&c2_rhs102);
  sf_mex_destroy(&c2_lhs102);
  sf_mex_destroy(&c2_rhs103);
  sf_mex_destroy(&c2_lhs103);
  sf_mex_destroy(&c2_rhs104);
  sf_mex_destroy(&c2_lhs104);
  sf_mex_destroy(&c2_rhs105);
  sf_mex_destroy(&c2_lhs105);
  sf_mex_destroy(&c2_rhs106);
  sf_mex_destroy(&c2_lhs106);
  sf_mex_destroy(&c2_rhs107);
  sf_mex_destroy(&c2_lhs107);
  sf_mex_destroy(&c2_rhs108);
  sf_mex_destroy(&c2_lhs108);
  sf_mex_destroy(&c2_rhs109);
  sf_mex_destroy(&c2_lhs109);
  sf_mex_destroy(&c2_rhs110);
  sf_mex_destroy(&c2_lhs110);
  sf_mex_destroy(&c2_rhs111);
  sf_mex_destroy(&c2_lhs111);
  sf_mex_destroy(&c2_rhs112);
  sf_mex_destroy(&c2_lhs112);
  sf_mex_destroy(&c2_rhs113);
  sf_mex_destroy(&c2_lhs113);
  sf_mex_destroy(&c2_rhs114);
  sf_mex_destroy(&c2_lhs114);
  sf_mex_destroy(&c2_rhs115);
  sf_mex_destroy(&c2_lhs115);
  sf_mex_destroy(&c2_rhs116);
  sf_mex_destroy(&c2_lhs116);
  sf_mex_destroy(&c2_rhs117);
  sf_mex_destroy(&c2_lhs117);
  sf_mex_destroy(&c2_rhs118);
  sf_mex_destroy(&c2_lhs118);
  sf_mex_destroy(&c2_rhs119);
  sf_mex_destroy(&c2_lhs119);
  sf_mex_destroy(&c2_rhs120);
  sf_mex_destroy(&c2_lhs120);
  sf_mex_destroy(&c2_rhs121);
  sf_mex_destroy(&c2_lhs121);
  sf_mex_destroy(&c2_rhs122);
  sf_mex_destroy(&c2_lhs122);
  sf_mex_destroy(&c2_rhs123);
  sf_mex_destroy(&c2_lhs123);
  sf_mex_destroy(&c2_rhs124);
  sf_mex_destroy(&c2_lhs124);
  sf_mex_destroy(&c2_rhs125);
  sf_mex_destroy(&c2_lhs125);
  sf_mex_destroy(&c2_rhs126);
  sf_mex_destroy(&c2_lhs126);
  sf_mex_destroy(&c2_rhs127);
  sf_mex_destroy(&c2_lhs127);
}

static void c2_c_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs128 = NULL;
  const mxArray *c2_lhs128 = NULL;
  const mxArray *c2_rhs129 = NULL;
  const mxArray *c2_lhs129 = NULL;
  const mxArray *c2_rhs130 = NULL;
  const mxArray *c2_lhs130 = NULL;
  const mxArray *c2_rhs131 = NULL;
  const mxArray *c2_lhs131 = NULL;
  const mxArray *c2_rhs132 = NULL;
  const mxArray *c2_lhs132 = NULL;
  const mxArray *c2_rhs133 = NULL;
  const mxArray *c2_lhs133 = NULL;
  const mxArray *c2_rhs134 = NULL;
  const mxArray *c2_lhs134 = NULL;
  const mxArray *c2_rhs135 = NULL;
  const mxArray *c2_lhs135 = NULL;
  const mxArray *c2_rhs136 = NULL;
  const mxArray *c2_lhs136 = NULL;
  const mxArray *c2_rhs137 = NULL;
  const mxArray *c2_lhs137 = NULL;
  const mxArray *c2_rhs138 = NULL;
  const mxArray *c2_lhs138 = NULL;
  const mxArray *c2_rhs139 = NULL;
  const mxArray *c2_lhs139 = NULL;
  const mxArray *c2_rhs140 = NULL;
  const mxArray *c2_lhs140 = NULL;
  const mxArray *c2_rhs141 = NULL;
  const mxArray *c2_lhs141 = NULL;
  const mxArray *c2_rhs142 = NULL;
  const mxArray *c2_lhs142 = NULL;
  const mxArray *c2_rhs143 = NULL;
  const mxArray *c2_lhs143 = NULL;
  const mxArray *c2_rhs144 = NULL;
  const mxArray *c2_lhs144 = NULL;
  const mxArray *c2_rhs145 = NULL;
  const mxArray *c2_lhs145 = NULL;
  const mxArray *c2_rhs146 = NULL;
  const mxArray *c2_lhs146 = NULL;
  const mxArray *c2_rhs147 = NULL;
  const mxArray *c2_lhs147 = NULL;
  const mxArray *c2_rhs148 = NULL;
  const mxArray *c2_lhs148 = NULL;
  const mxArray *c2_rhs149 = NULL;
  const mxArray *c2_lhs149 = NULL;
  const mxArray *c2_rhs150 = NULL;
  const mxArray *c2_lhs150 = NULL;
  const mxArray *c2_rhs151 = NULL;
  const mxArray *c2_lhs151 = NULL;
  const mxArray *c2_rhs152 = NULL;
  const mxArray *c2_lhs152 = NULL;
  const mxArray *c2_rhs153 = NULL;
  const mxArray *c2_lhs153 = NULL;
  const mxArray *c2_rhs154 = NULL;
  const mxArray *c2_lhs154 = NULL;
  const mxArray *c2_rhs155 = NULL;
  const mxArray *c2_lhs155 = NULL;
  const mxArray *c2_rhs156 = NULL;
  const mxArray *c2_lhs156 = NULL;
  const mxArray *c2_rhs157 = NULL;
  const mxArray *c2_lhs157 = NULL;
  const mxArray *c2_rhs158 = NULL;
  const mxArray *c2_lhs158 = NULL;
  const mxArray *c2_rhs159 = NULL;
  const mxArray *c2_lhs159 = NULL;
  const mxArray *c2_rhs160 = NULL;
  const mxArray *c2_lhs160 = NULL;
  const mxArray *c2_rhs161 = NULL;
  const mxArray *c2_lhs161 = NULL;
  const mxArray *c2_rhs162 = NULL;
  const mxArray *c2_lhs162 = NULL;
  const mxArray *c2_rhs163 = NULL;
  const mxArray *c2_lhs163 = NULL;
  const mxArray *c2_rhs164 = NULL;
  const mxArray *c2_lhs164 = NULL;
  const mxArray *c2_rhs165 = NULL;
  const mxArray *c2_lhs165 = NULL;
  const mxArray *c2_rhs166 = NULL;
  const mxArray *c2_lhs166 = NULL;
  const mxArray *c2_rhs167 = NULL;
  const mxArray *c2_lhs167 = NULL;
  const mxArray *c2_rhs168 = NULL;
  const mxArray *c2_lhs168 = NULL;
  const mxArray *c2_rhs169 = NULL;
  const mxArray *c2_lhs169 = NULL;
  const mxArray *c2_rhs170 = NULL;
  const mxArray *c2_lhs170 = NULL;
  const mxArray *c2_rhs171 = NULL;
  const mxArray *c2_lhs171 = NULL;
  const mxArray *c2_rhs172 = NULL;
  const mxArray *c2_lhs172 = NULL;
  const mxArray *c2_rhs173 = NULL;
  const mxArray *c2_lhs173 = NULL;
  const mxArray *c2_rhs174 = NULL;
  const mxArray *c2_lhs174 = NULL;
  const mxArray *c2_rhs175 = NULL;
  const mxArray *c2_lhs175 = NULL;
  const mxArray *c2_rhs176 = NULL;
  const mxArray *c2_lhs176 = NULL;
  const mxArray *c2_rhs177 = NULL;
  const mxArray *c2_lhs177 = NULL;
  const mxArray *c2_rhs178 = NULL;
  const mxArray *c2_lhs178 = NULL;
  const mxArray *c2_rhs179 = NULL;
  const mxArray *c2_lhs179 = NULL;
  const mxArray *c2_rhs180 = NULL;
  const mxArray *c2_lhs180 = NULL;
  const mxArray *c2_rhs181 = NULL;
  const mxArray *c2_lhs181 = NULL;
  const mxArray *c2_rhs182 = NULL;
  const mxArray *c2_lhs182 = NULL;
  const mxArray *c2_rhs183 = NULL;
  const mxArray *c2_lhs183 = NULL;
  const mxArray *c2_rhs184 = NULL;
  const mxArray *c2_lhs184 = NULL;
  const mxArray *c2_rhs185 = NULL;
  const mxArray *c2_lhs185 = NULL;
  const mxArray *c2_rhs186 = NULL;
  const mxArray *c2_lhs186 = NULL;
  const mxArray *c2_rhs187 = NULL;
  const mxArray *c2_lhs187 = NULL;
  const mxArray *c2_rhs188 = NULL;
  const mxArray *c2_lhs188 = NULL;
  const mxArray *c2_rhs189 = NULL;
  const mxArray *c2_lhs189 = NULL;
  const mxArray *c2_rhs190 = NULL;
  const mxArray *c2_lhs190 = NULL;
  const mxArray *c2_rhs191 = NULL;
  const mxArray *c2_lhs191 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c2_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c2_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c2_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c2_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c2_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!inv3x3"), "context",
                  "context", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c2_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("norm"), "name", "name", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731868U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c2_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "context",
                  "context", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c2_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c2_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c2_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c2_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm"),
                  "context", "context", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840376U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c2_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c2_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_warning"), "name", "name",
                  141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840402U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c2_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731858U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c2_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eps"), "name", "name", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c2_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840382U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c2_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_eps"), "name", "name", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c2_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c2_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/inv.m!checkcond"),
                  "context", "context", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_flt2str"), "name", "name",
                  147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "resolved",
                  "resolved", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360303950U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c2_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m"), "context",
                  "context", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "name", "name", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m"), "resolved",
                  "resolved", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319751568U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c2_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name",
                  149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1388481696U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370031486U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c2_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c2_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c2_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name",
                  152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331326458U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c2_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_lusolve"), "name", "name",
                  153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m"), "resolved",
                  "resolved", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370031486U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c2_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN"),
                  "context", "context", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgetrf"), "name", "name",
                  154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840406U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c2_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m"),
                  "context", "context", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_lapack_xgetrf"), "name",
                  "name", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840410U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c2_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m"),
                  "context", "context", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matlab_zgetrf"), "name",
                  "name", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1302710594U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c2_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("realmin"), "name", "name", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307672842U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c2_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmin"), "name", "name",
                  158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307672844U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c2_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c2_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eps"), "name", "name", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326749596U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c2_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("min"), "name", "name", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c2_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378317584U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c2_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c2_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c2_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c2_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c2_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c2_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c2_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c2_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("colon"), "name", "name", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378317588U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c2_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("colon"), "name", "name", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1378317588U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c2_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c2_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c2_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731854U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c2_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c2_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c2_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c2_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c2_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c2_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c2_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "context",
                  "context", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c2_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_unsigned_class"), "name",
                  "name", 182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c2_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "context", "context", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.unsignedClass"),
                  "name", "name", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "resolved", "resolved", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c2_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381871900U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c2_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c2_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c2_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c2_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c2_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs188), "lhs", "lhs",
                  188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 189);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 189);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c2_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs189), "rhs", "rhs",
                  189);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs189), "lhs", "lhs",
                  189);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon"),
                  "context", "context", 190);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 190);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 190);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 190);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c2_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs190), "rhs", "rhs",
                  190);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs190), "lhs", "lhs",
                  190);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 191);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 191);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 191);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323192178U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c2_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs191), "rhs", "rhs",
                  191);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs191), "lhs", "lhs",
                  191);
  sf_mex_destroy(&c2_rhs128);
  sf_mex_destroy(&c2_lhs128);
  sf_mex_destroy(&c2_rhs129);
  sf_mex_destroy(&c2_lhs129);
  sf_mex_destroy(&c2_rhs130);
  sf_mex_destroy(&c2_lhs130);
  sf_mex_destroy(&c2_rhs131);
  sf_mex_destroy(&c2_lhs131);
  sf_mex_destroy(&c2_rhs132);
  sf_mex_destroy(&c2_lhs132);
  sf_mex_destroy(&c2_rhs133);
  sf_mex_destroy(&c2_lhs133);
  sf_mex_destroy(&c2_rhs134);
  sf_mex_destroy(&c2_lhs134);
  sf_mex_destroy(&c2_rhs135);
  sf_mex_destroy(&c2_lhs135);
  sf_mex_destroy(&c2_rhs136);
  sf_mex_destroy(&c2_lhs136);
  sf_mex_destroy(&c2_rhs137);
  sf_mex_destroy(&c2_lhs137);
  sf_mex_destroy(&c2_rhs138);
  sf_mex_destroy(&c2_lhs138);
  sf_mex_destroy(&c2_rhs139);
  sf_mex_destroy(&c2_lhs139);
  sf_mex_destroy(&c2_rhs140);
  sf_mex_destroy(&c2_lhs140);
  sf_mex_destroy(&c2_rhs141);
  sf_mex_destroy(&c2_lhs141);
  sf_mex_destroy(&c2_rhs142);
  sf_mex_destroy(&c2_lhs142);
  sf_mex_destroy(&c2_rhs143);
  sf_mex_destroy(&c2_lhs143);
  sf_mex_destroy(&c2_rhs144);
  sf_mex_destroy(&c2_lhs144);
  sf_mex_destroy(&c2_rhs145);
  sf_mex_destroy(&c2_lhs145);
  sf_mex_destroy(&c2_rhs146);
  sf_mex_destroy(&c2_lhs146);
  sf_mex_destroy(&c2_rhs147);
  sf_mex_destroy(&c2_lhs147);
  sf_mex_destroy(&c2_rhs148);
  sf_mex_destroy(&c2_lhs148);
  sf_mex_destroy(&c2_rhs149);
  sf_mex_destroy(&c2_lhs149);
  sf_mex_destroy(&c2_rhs150);
  sf_mex_destroy(&c2_lhs150);
  sf_mex_destroy(&c2_rhs151);
  sf_mex_destroy(&c2_lhs151);
  sf_mex_destroy(&c2_rhs152);
  sf_mex_destroy(&c2_lhs152);
  sf_mex_destroy(&c2_rhs153);
  sf_mex_destroy(&c2_lhs153);
  sf_mex_destroy(&c2_rhs154);
  sf_mex_destroy(&c2_lhs154);
  sf_mex_destroy(&c2_rhs155);
  sf_mex_destroy(&c2_lhs155);
  sf_mex_destroy(&c2_rhs156);
  sf_mex_destroy(&c2_lhs156);
  sf_mex_destroy(&c2_rhs157);
  sf_mex_destroy(&c2_lhs157);
  sf_mex_destroy(&c2_rhs158);
  sf_mex_destroy(&c2_lhs158);
  sf_mex_destroy(&c2_rhs159);
  sf_mex_destroy(&c2_lhs159);
  sf_mex_destroy(&c2_rhs160);
  sf_mex_destroy(&c2_lhs160);
  sf_mex_destroy(&c2_rhs161);
  sf_mex_destroy(&c2_lhs161);
  sf_mex_destroy(&c2_rhs162);
  sf_mex_destroy(&c2_lhs162);
  sf_mex_destroy(&c2_rhs163);
  sf_mex_destroy(&c2_lhs163);
  sf_mex_destroy(&c2_rhs164);
  sf_mex_destroy(&c2_lhs164);
  sf_mex_destroy(&c2_rhs165);
  sf_mex_destroy(&c2_lhs165);
  sf_mex_destroy(&c2_rhs166);
  sf_mex_destroy(&c2_lhs166);
  sf_mex_destroy(&c2_rhs167);
  sf_mex_destroy(&c2_lhs167);
  sf_mex_destroy(&c2_rhs168);
  sf_mex_destroy(&c2_lhs168);
  sf_mex_destroy(&c2_rhs169);
  sf_mex_destroy(&c2_lhs169);
  sf_mex_destroy(&c2_rhs170);
  sf_mex_destroy(&c2_lhs170);
  sf_mex_destroy(&c2_rhs171);
  sf_mex_destroy(&c2_lhs171);
  sf_mex_destroy(&c2_rhs172);
  sf_mex_destroy(&c2_lhs172);
  sf_mex_destroy(&c2_rhs173);
  sf_mex_destroy(&c2_lhs173);
  sf_mex_destroy(&c2_rhs174);
  sf_mex_destroy(&c2_lhs174);
  sf_mex_destroy(&c2_rhs175);
  sf_mex_destroy(&c2_lhs175);
  sf_mex_destroy(&c2_rhs176);
  sf_mex_destroy(&c2_lhs176);
  sf_mex_destroy(&c2_rhs177);
  sf_mex_destroy(&c2_lhs177);
  sf_mex_destroy(&c2_rhs178);
  sf_mex_destroy(&c2_lhs178);
  sf_mex_destroy(&c2_rhs179);
  sf_mex_destroy(&c2_lhs179);
  sf_mex_destroy(&c2_rhs180);
  sf_mex_destroy(&c2_lhs180);
  sf_mex_destroy(&c2_rhs181);
  sf_mex_destroy(&c2_lhs181);
  sf_mex_destroy(&c2_rhs182);
  sf_mex_destroy(&c2_lhs182);
  sf_mex_destroy(&c2_rhs183);
  sf_mex_destroy(&c2_lhs183);
  sf_mex_destroy(&c2_rhs184);
  sf_mex_destroy(&c2_lhs184);
  sf_mex_destroy(&c2_rhs185);
  sf_mex_destroy(&c2_lhs185);
  sf_mex_destroy(&c2_rhs186);
  sf_mex_destroy(&c2_lhs186);
  sf_mex_destroy(&c2_rhs187);
  sf_mex_destroy(&c2_lhs187);
  sf_mex_destroy(&c2_rhs188);
  sf_mex_destroy(&c2_lhs188);
  sf_mex_destroy(&c2_rhs189);
  sf_mex_destroy(&c2_lhs189);
  sf_mex_destroy(&c2_rhs190);
  sf_mex_destroy(&c2_lhs190);
  sf_mex_destroy(&c2_rhs191);
  sf_mex_destroy(&c2_lhs191);
}

static void c2_d_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs192 = NULL;
  const mxArray *c2_lhs192 = NULL;
  const mxArray *c2_rhs193 = NULL;
  const mxArray *c2_lhs193 = NULL;
  const mxArray *c2_rhs194 = NULL;
  const mxArray *c2_lhs194 = NULL;
  const mxArray *c2_rhs195 = NULL;
  const mxArray *c2_lhs195 = NULL;
  const mxArray *c2_rhs196 = NULL;
  const mxArray *c2_lhs196 = NULL;
  const mxArray *c2_rhs197 = NULL;
  const mxArray *c2_lhs197 = NULL;
  const mxArray *c2_rhs198 = NULL;
  const mxArray *c2_lhs198 = NULL;
  const mxArray *c2_rhs199 = NULL;
  const mxArray *c2_lhs199 = NULL;
  const mxArray *c2_rhs200 = NULL;
  const mxArray *c2_lhs200 = NULL;
  const mxArray *c2_rhs201 = NULL;
  const mxArray *c2_lhs201 = NULL;
  const mxArray *c2_rhs202 = NULL;
  const mxArray *c2_lhs202 = NULL;
  const mxArray *c2_rhs203 = NULL;
  const mxArray *c2_lhs203 = NULL;
  const mxArray *c2_rhs204 = NULL;
  const mxArray *c2_lhs204 = NULL;
  const mxArray *c2_rhs205 = NULL;
  const mxArray *c2_lhs205 = NULL;
  const mxArray *c2_rhs206 = NULL;
  const mxArray *c2_lhs206 = NULL;
  const mxArray *c2_rhs207 = NULL;
  const mxArray *c2_lhs207 = NULL;
  const mxArray *c2_rhs208 = NULL;
  const mxArray *c2_lhs208 = NULL;
  const mxArray *c2_rhs209 = NULL;
  const mxArray *c2_lhs209 = NULL;
  const mxArray *c2_rhs210 = NULL;
  const mxArray *c2_lhs210 = NULL;
  const mxArray *c2_rhs211 = NULL;
  const mxArray *c2_lhs211 = NULL;
  const mxArray *c2_rhs212 = NULL;
  const mxArray *c2_lhs212 = NULL;
  const mxArray *c2_rhs213 = NULL;
  const mxArray *c2_lhs213 = NULL;
  const mxArray *c2_rhs214 = NULL;
  const mxArray *c2_lhs214 = NULL;
  const mxArray *c2_rhs215 = NULL;
  const mxArray *c2_lhs215 = NULL;
  const mxArray *c2_rhs216 = NULL;
  const mxArray *c2_lhs216 = NULL;
  const mxArray *c2_rhs217 = NULL;
  const mxArray *c2_lhs217 = NULL;
  const mxArray *c2_rhs218 = NULL;
  const mxArray *c2_lhs218 = NULL;
  const mxArray *c2_rhs219 = NULL;
  const mxArray *c2_lhs219 = NULL;
  const mxArray *c2_rhs220 = NULL;
  const mxArray *c2_lhs220 = NULL;
  const mxArray *c2_rhs221 = NULL;
  const mxArray *c2_lhs221 = NULL;
  const mxArray *c2_rhs222 = NULL;
  const mxArray *c2_lhs222 = NULL;
  const mxArray *c2_rhs223 = NULL;
  const mxArray *c2_lhs223 = NULL;
  const mxArray *c2_rhs224 = NULL;
  const mxArray *c2_lhs224 = NULL;
  const mxArray *c2_rhs225 = NULL;
  const mxArray *c2_lhs225 = NULL;
  const mxArray *c2_rhs226 = NULL;
  const mxArray *c2_lhs226 = NULL;
  const mxArray *c2_rhs227 = NULL;
  const mxArray *c2_lhs227 = NULL;
  const mxArray *c2_rhs228 = NULL;
  const mxArray *c2_lhs228 = NULL;
  const mxArray *c2_rhs229 = NULL;
  const mxArray *c2_lhs229 = NULL;
  const mxArray *c2_rhs230 = NULL;
  const mxArray *c2_lhs230 = NULL;
  const mxArray *c2_rhs231 = NULL;
  const mxArray *c2_lhs231 = NULL;
  const mxArray *c2_rhs232 = NULL;
  const mxArray *c2_lhs232 = NULL;
  const mxArray *c2_rhs233 = NULL;
  const mxArray *c2_lhs233 = NULL;
  const mxArray *c2_rhs234 = NULL;
  const mxArray *c2_lhs234 = NULL;
  const mxArray *c2_rhs235 = NULL;
  const mxArray *c2_lhs235 = NULL;
  const mxArray *c2_rhs236 = NULL;
  const mxArray *c2_lhs236 = NULL;
  const mxArray *c2_rhs237 = NULL;
  const mxArray *c2_lhs237 = NULL;
  const mxArray *c2_rhs238 = NULL;
  const mxArray *c2_lhs238 = NULL;
  const mxArray *c2_rhs239 = NULL;
  const mxArray *c2_lhs239 = NULL;
  const mxArray *c2_rhs240 = NULL;
  const mxArray *c2_lhs240 = NULL;
  const mxArray *c2_rhs241 = NULL;
  const mxArray *c2_lhs241 = NULL;
  const mxArray *c2_rhs242 = NULL;
  const mxArray *c2_lhs242 = NULL;
  const mxArray *c2_rhs243 = NULL;
  const mxArray *c2_lhs243 = NULL;
  const mxArray *c2_rhs244 = NULL;
  const mxArray *c2_lhs244 = NULL;
  const mxArray *c2_rhs245 = NULL;
  const mxArray *c2_lhs245 = NULL;
  const mxArray *c2_rhs246 = NULL;
  const mxArray *c2_lhs246 = NULL;
  const mxArray *c2_rhs247 = NULL;
  const mxArray *c2_lhs247 = NULL;
  const mxArray *c2_rhs248 = NULL;
  const mxArray *c2_lhs248 = NULL;
  const mxArray *c2_rhs249 = NULL;
  const mxArray *c2_lhs249 = NULL;
  const mxArray *c2_rhs250 = NULL;
  const mxArray *c2_lhs250 = NULL;
  const mxArray *c2_rhs251 = NULL;
  const mxArray *c2_lhs251 = NULL;
  const mxArray *c2_rhs252 = NULL;
  const mxArray *c2_lhs252 = NULL;
  const mxArray *c2_rhs253 = NULL;
  const mxArray *c2_lhs253 = NULL;
  const mxArray *c2_rhs254 = NULL;
  const mxArray *c2_lhs254 = NULL;
  const mxArray *c2_rhs255 = NULL;
  const mxArray *c2_lhs255 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 192);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 192);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 192);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c2_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs192), "rhs", "rhs",
                  192);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs192), "lhs", "lhs",
                  192);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 193);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 193);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c2_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs193), "rhs", "rhs",
                  193);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs193), "lhs", "lhs",
                  193);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 194);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 194);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 194);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c2_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs194), "rhs", "rhs",
                  194);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs194), "lhs", "lhs",
                  194);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 195);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 195);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 195);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 195);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c2_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs195), "rhs", "rhs",
                  195);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs195), "lhs", "lhs",
                  195);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 196);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 196);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 196);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c2_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs196), "rhs", "rhs",
                  196);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs196), "lhs", "lhs",
                  196);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 197);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 197);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 197);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 197);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c2_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs197), "rhs", "rhs",
                  197);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs197), "lhs", "lhs",
                  197);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 198);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 198);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 198);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604016U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c2_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs198), "rhs", "rhs",
                  198);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs198), "lhs", "lhs",
                  198);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 199);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_ixamax"), "name", "name",
                  199);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "resolved", "resolved", 199);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c2_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs199), "rhs", "rhs",
                  199);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs199), "lhs", "lhs",
                  199);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 200);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 200);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c2_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs200), "rhs", "rhs",
                  200);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs200), "lhs", "lhs",
                  200);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 201);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.ixamax"),
                  "name", "name", 201);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 201);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "resolved", "resolved", 201);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c2_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs201), "rhs", "rhs",
                  201);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs201), "lhs", "lhs",
                  201);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 202);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 202);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 202);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c2_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs202), "rhs", "rhs",
                  202);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs202), "lhs", "lhs",
                  202);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 203);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 203);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 203);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c2_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs203), "rhs", "rhs",
                  203);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs203), "lhs", "lhs",
                  203);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 204);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 204);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 204);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303167806U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c2_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs204), "rhs", "rhs",
                  204);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs204), "lhs", "lhs",
                  204);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 205);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.ixamax"),
                  "name", "name", 205);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "resolved", "resolved", 205);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c2_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs205), "rhs", "rhs",
                  205);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs205), "lhs", "lhs",
                  205);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 206);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 206);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 206);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 206);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c2_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs206), "rhs", "rhs",
                  206);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs206), "lhs", "lhs",
                  206);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 207);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 207);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 207);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 207);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c2_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs207), "rhs", "rhs",
                  207);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs207), "lhs", "lhs",
                  207);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 208);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 208);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c2_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs208), "rhs", "rhs",
                  208);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs208), "lhs", "lhs",
                  208);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 209);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 209);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 209);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c2_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs209), "rhs", "rhs",
                  209);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs209), "lhs", "lhs",
                  209);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 210);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xswap"), "name", "name",
                  210);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 210);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 210);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 210);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 210);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 210);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 210);
  sf_mex_assign(&c2_rhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs210), "rhs", "rhs",
                  210);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs210), "lhs", "lhs",
                  210);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 211);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 211);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 211);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 211);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 211);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 211);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 211);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 211);
  sf_mex_assign(&c2_rhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs211), "rhs", "rhs",
                  211);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs211), "lhs", "lhs",
                  211);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 212);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 212);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 212);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 212);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 212);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 212);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 212);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 212);
  sf_mex_assign(&c2_rhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs212), "rhs", "rhs",
                  212);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs212), "lhs", "lhs",
                  212);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 213);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 213);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 213);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 213);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 213);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 213);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 213);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 213);
  sf_mex_assign(&c2_rhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs213), "rhs", "rhs",
                  213);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs213), "lhs", "lhs",
                  213);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 214);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 214);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 214);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 214);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 214);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 214);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 214);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 214);
  sf_mex_assign(&c2_rhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs214), "rhs", "rhs",
                  214);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs214), "lhs", "lhs",
                  214);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 215);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xswap"),
                  "name", "name", 215);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 215);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 215);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 215);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 215);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 215);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 215);
  sf_mex_assign(&c2_rhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs215), "rhs", "rhs",
                  215);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs215), "lhs", "lhs",
                  215);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 216);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 216);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 216);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 216);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 216);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 216);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 216);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 216);
  sf_mex_assign(&c2_rhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs216), "rhs", "rhs",
                  216);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs216), "lhs", "lhs",
                  216);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 217);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 217);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 217);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 217);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 217);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 217);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 217);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 217);
  sf_mex_assign(&c2_rhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs217), "rhs", "rhs",
                  217);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs217), "lhs", "lhs",
                  217);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 218);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 218);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 218);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 218);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840312U), "fileTimeLo",
                  "fileTimeLo", 218);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 218);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 218);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 218);
  sf_mex_assign(&c2_rhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs218), "rhs", "rhs",
                  218);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs218), "lhs", "lhs",
                  218);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 219);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 219);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 219);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 219);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 219);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 219);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 219);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 219);
  sf_mex_assign(&c2_rhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs219), "rhs", "rhs",
                  219);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs219), "lhs", "lhs",
                  219);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 220);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 220);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 220);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 220);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 220);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 220);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 220);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 220);
  sf_mex_assign(&c2_rhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs220), "rhs", "rhs",
                  220);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs220), "lhs", "lhs",
                  220);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 221);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 221);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 221);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 221);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 221);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 221);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 221);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 221);
  sf_mex_assign(&c2_rhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs221), "rhs", "rhs",
                  221);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs221), "lhs", "lhs",
                  221);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m"),
                  "context", "context", 222);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgeru"), "name", "name",
                  222);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 222);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"),
                  "resolved", "resolved", 222);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002290U), "fileTimeLo",
                  "fileTimeLo", 222);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 222);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 222);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 222);
  sf_mex_assign(&c2_rhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs222), "rhs", "rhs",
                  222);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs222), "lhs", "lhs",
                  222);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"), "context",
                  "context", 223);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 223);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 223);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 223);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 223);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 223);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 223);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 223);
  sf_mex_assign(&c2_rhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs223), "rhs", "rhs",
                  223);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs223), "lhs", "lhs",
                  223);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m"), "context",
                  "context", 224);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xgeru"),
                  "name", "name", 224);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 224);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgeru.p"),
                  "resolved", "resolved", 224);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 224);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 224);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 224);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 224);
  sf_mex_assign(&c2_rhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs224), "rhs", "rhs",
                  224);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs224), "lhs", "lhs",
                  224);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgeru.p"),
                  "context", "context", 225);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xger"),
                  "name", "name", 225);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 225);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "resolved", "resolved", 225);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 225);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 225);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 225);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 225);
  sf_mex_assign(&c2_rhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs225), "rhs", "rhs",
                  225);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs225), "lhs", "lhs",
                  225);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 226);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 226);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 226);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 226);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 226);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 226);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 226);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 226);
  sf_mex_assign(&c2_rhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs226), "rhs", "rhs",
                  226);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs226), "lhs", "lhs",
                  226);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 227);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 227);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 227);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 227);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 227);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 227);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 227);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 227);
  sf_mex_assign(&c2_rhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs227), "rhs", "rhs",
                  227);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs227), "lhs", "lhs",
                  227);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 228);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 228);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 228);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 228);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 228);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 228);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 228);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 228);
  sf_mex_assign(&c2_rhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs228), "rhs", "rhs",
                  228);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs228), "lhs", "lhs",
                  228);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 229);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 229);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 229);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 229);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 229);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 229);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 229);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 229);
  sf_mex_assign(&c2_rhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs229), "rhs", "rhs",
                  229);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs229), "lhs", "lhs",
                  229);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 230);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("min"), "name", "name", 230);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 230);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 230);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1311276918U), "fileTimeLo",
                  "fileTimeLo", 230);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 230);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 230);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 230);
  sf_mex_assign(&c2_rhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs230), "rhs", "rhs",
                  230);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs230), "lhs", "lhs",
                  230);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 231);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 231);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 231);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 231);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 231);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 231);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 231);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 231);
  sf_mex_assign(&c2_rhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs231), "rhs", "rhs",
                  231);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs231), "lhs", "lhs",
                  231);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 232);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 232);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 232);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 232);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 232);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 232);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 232);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 232);
  sf_mex_assign(&c2_rhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs232), "rhs", "rhs",
                  232);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs232), "lhs", "lhs",
                  232);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 233);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 233);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 233);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 233);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 233);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 233);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 233);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 233);
  sf_mex_assign(&c2_rhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs233), "rhs", "rhs",
                  233);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs233), "lhs", "lhs",
                  233);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 234);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 234);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 234);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 234);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 234);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 234);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 234);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 234);
  sf_mex_assign(&c2_rhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs234), "rhs", "rhs",
                  234);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs234), "lhs", "lhs",
                  234);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 235);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xger"),
                  "name", "name", 235);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 235);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "resolved", "resolved", 235);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 235);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 235);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 235);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 235);
  sf_mex_assign(&c2_rhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs235), "rhs", "rhs",
                  235);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs235), "lhs", "lhs",
                  235);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "context", "context", 236);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xgerx"),
                  "name", "name", 236);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 236);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "resolved", "resolved", 236);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 236);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 236);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 236);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 236);
  sf_mex_assign(&c2_rhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs236), "rhs", "rhs",
                  236);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs236), "lhs", "lhs",
                  236);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 237);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 237);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 237);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 237);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731852U), "fileTimeLo",
                  "fileTimeLo", 237);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 237);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 237);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 237);
  sf_mex_assign(&c2_rhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs237), "rhs", "rhs",
                  237);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs237), "lhs", "lhs",
                  237);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 238);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 238);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 238);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 238);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 238);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 238);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 238);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 238);
  sf_mex_assign(&c2_rhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs238), "rhs", "rhs",
                  238);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs238), "lhs", "lhs",
                  238);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 239);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 239);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 239);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 239);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 239);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 239);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 239);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 239);
  sf_mex_assign(&c2_rhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs239), "rhs", "rhs",
                  239);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs239), "lhs", "lhs",
                  239);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 240);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 240);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 240);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 240);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 240);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 240);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 240);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 240);
  sf_mex_assign(&c2_rhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs240), "rhs", "rhs",
                  240);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs240), "lhs", "lhs",
                  240);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 241);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 241);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 241);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 241);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372604760U), "fileTimeLo",
                  "fileTimeLo", 241);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 241);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 241);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 241);
  sf_mex_assign(&c2_rhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs241), "rhs", "rhs",
                  241);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs241), "lhs", "lhs",
                  241);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular"),
                  "context", "context", 242);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_warning"), "name", "name",
                  242);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 242);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 242);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286840402U), "fileTimeLo",
                  "fileTimeLo", 242);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 242);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 242);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 242);
  sf_mex_assign(&c2_rhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs242), "rhs", "rhs",
                  242);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs242), "lhs", "lhs",
                  242);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN"),
                  "context", "context", 243);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 243);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 243);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 243);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 243);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 243);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 243);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 243);
  sf_mex_assign(&c2_rhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs243), "rhs", "rhs",
                  243);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs243), "lhs", "lhs",
                  243);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN"),
                  "context", "context", 244);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xtrsm"), "name", "name",
                  244);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 244);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"),
                  "resolved", "resolved", 244);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002292U), "fileTimeLo",
                  "fileTimeLo", 244);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 244);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 244);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 244);
  sf_mex_assign(&c2_rhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs244), "rhs", "rhs",
                  244);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs244), "lhs", "lhs",
                  244);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"), "context",
                  "context", 245);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 245);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 245);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 245);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 245);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 245);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 245);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 245);
  sf_mex_assign(&c2_rhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs245), "rhs", "rhs",
                  245);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs245), "lhs", "lhs",
                  245);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m"), "context",
                  "context", 246);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xtrsm"),
                  "name", "name", 246);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 246);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "resolved", "resolved", 246);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 246);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 246);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 246);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 246);
  sf_mex_assign(&c2_rhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs246), "rhs", "rhs",
                  246);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs246), "lhs", "lhs",
                  246);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 247);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 247);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 247);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 247);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 247);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 247);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 247);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 247);
  sf_mex_assign(&c2_rhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs247), "rhs", "rhs",
                  247);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs247), "lhs", "lhs",
                  247);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p!below_threshold"),
                  "context", "context", 248);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 248);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 248);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 248);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 248);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 248);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 248);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 248);
  sf_mex_assign(&c2_rhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs248), "rhs", "rhs",
                  248);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs248), "lhs", "lhs",
                  248);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 249);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 249);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 249);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 249);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 249);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 249);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 249);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 249);
  sf_mex_assign(&c2_rhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs249), "rhs", "rhs",
                  249);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs249), "lhs", "lhs",
                  249);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xtrsm.p"),
                  "context", "context", 250);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xtrsm"),
                  "name", "name", 250);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 250);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "resolved", "resolved", 250);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329522U), "fileTimeLo",
                  "fileTimeLo", 250);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 250);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 250);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 250);
  sf_mex_assign(&c2_rhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs250), "rhs", "rhs",
                  250);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs250), "lhs", "lhs",
                  250);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 251);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 251);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 251);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 251);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363732556U), "fileTimeLo",
                  "fileTimeLo", 251);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 251);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 251);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 251);
  sf_mex_assign(&c2_rhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs251), "rhs", "rhs",
                  251);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs251), "lhs", "lhs",
                  251);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 252);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 252);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 252);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 252);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389329520U), "fileTimeLo",
                  "fileTimeLo", 252);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 252);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 252);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 252);
  sf_mex_assign(&c2_rhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs252), "rhs", "rhs",
                  252);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs252), "lhs", "lhs",
                  252);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 253);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 253);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 253);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 253);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 253);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 253);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 253);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 253);
  sf_mex_assign(&c2_rhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs253), "rhs", "rhs",
                  253);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs253), "lhs", "lhs",
                  253);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xtrsm.p"),
                  "context", "context", 254);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 254);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 254);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 254);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363731880U), "fileTimeLo",
                  "fileTimeLo", 254);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 254);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 254);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 254);
  sf_mex_assign(&c2_rhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs254), "rhs", "rhs",
                  254);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs254), "lhs", "lhs",
                  254);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 255);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 255);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 255);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 255);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362283482U), "fileTimeLo",
                  "fileTimeLo", 255);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 255);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 255);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 255);
  sf_mex_assign(&c2_rhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs255), "rhs", "rhs",
                  255);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs255), "lhs", "lhs",
                  255);
  sf_mex_destroy(&c2_rhs192);
  sf_mex_destroy(&c2_lhs192);
  sf_mex_destroy(&c2_rhs193);
  sf_mex_destroy(&c2_lhs193);
  sf_mex_destroy(&c2_rhs194);
  sf_mex_destroy(&c2_lhs194);
  sf_mex_destroy(&c2_rhs195);
  sf_mex_destroy(&c2_lhs195);
  sf_mex_destroy(&c2_rhs196);
  sf_mex_destroy(&c2_lhs196);
  sf_mex_destroy(&c2_rhs197);
  sf_mex_destroy(&c2_lhs197);
  sf_mex_destroy(&c2_rhs198);
  sf_mex_destroy(&c2_lhs198);
  sf_mex_destroy(&c2_rhs199);
  sf_mex_destroy(&c2_lhs199);
  sf_mex_destroy(&c2_rhs200);
  sf_mex_destroy(&c2_lhs200);
  sf_mex_destroy(&c2_rhs201);
  sf_mex_destroy(&c2_lhs201);
  sf_mex_destroy(&c2_rhs202);
  sf_mex_destroy(&c2_lhs202);
  sf_mex_destroy(&c2_rhs203);
  sf_mex_destroy(&c2_lhs203);
  sf_mex_destroy(&c2_rhs204);
  sf_mex_destroy(&c2_lhs204);
  sf_mex_destroy(&c2_rhs205);
  sf_mex_destroy(&c2_lhs205);
  sf_mex_destroy(&c2_rhs206);
  sf_mex_destroy(&c2_lhs206);
  sf_mex_destroy(&c2_rhs207);
  sf_mex_destroy(&c2_lhs207);
  sf_mex_destroy(&c2_rhs208);
  sf_mex_destroy(&c2_lhs208);
  sf_mex_destroy(&c2_rhs209);
  sf_mex_destroy(&c2_lhs209);
  sf_mex_destroy(&c2_rhs210);
  sf_mex_destroy(&c2_lhs210);
  sf_mex_destroy(&c2_rhs211);
  sf_mex_destroy(&c2_lhs211);
  sf_mex_destroy(&c2_rhs212);
  sf_mex_destroy(&c2_lhs212);
  sf_mex_destroy(&c2_rhs213);
  sf_mex_destroy(&c2_lhs213);
  sf_mex_destroy(&c2_rhs214);
  sf_mex_destroy(&c2_lhs214);
  sf_mex_destroy(&c2_rhs215);
  sf_mex_destroy(&c2_lhs215);
  sf_mex_destroy(&c2_rhs216);
  sf_mex_destroy(&c2_lhs216);
  sf_mex_destroy(&c2_rhs217);
  sf_mex_destroy(&c2_lhs217);
  sf_mex_destroy(&c2_rhs218);
  sf_mex_destroy(&c2_lhs218);
  sf_mex_destroy(&c2_rhs219);
  sf_mex_destroy(&c2_lhs219);
  sf_mex_destroy(&c2_rhs220);
  sf_mex_destroy(&c2_lhs220);
  sf_mex_destroy(&c2_rhs221);
  sf_mex_destroy(&c2_lhs221);
  sf_mex_destroy(&c2_rhs222);
  sf_mex_destroy(&c2_lhs222);
  sf_mex_destroy(&c2_rhs223);
  sf_mex_destroy(&c2_lhs223);
  sf_mex_destroy(&c2_rhs224);
  sf_mex_destroy(&c2_lhs224);
  sf_mex_destroy(&c2_rhs225);
  sf_mex_destroy(&c2_lhs225);
  sf_mex_destroy(&c2_rhs226);
  sf_mex_destroy(&c2_lhs226);
  sf_mex_destroy(&c2_rhs227);
  sf_mex_destroy(&c2_lhs227);
  sf_mex_destroy(&c2_rhs228);
  sf_mex_destroy(&c2_lhs228);
  sf_mex_destroy(&c2_rhs229);
  sf_mex_destroy(&c2_lhs229);
  sf_mex_destroy(&c2_rhs230);
  sf_mex_destroy(&c2_lhs230);
  sf_mex_destroy(&c2_rhs231);
  sf_mex_destroy(&c2_lhs231);
  sf_mex_destroy(&c2_rhs232);
  sf_mex_destroy(&c2_lhs232);
  sf_mex_destroy(&c2_rhs233);
  sf_mex_destroy(&c2_lhs233);
  sf_mex_destroy(&c2_rhs234);
  sf_mex_destroy(&c2_lhs234);
  sf_mex_destroy(&c2_rhs235);
  sf_mex_destroy(&c2_lhs235);
  sf_mex_destroy(&c2_rhs236);
  sf_mex_destroy(&c2_lhs236);
  sf_mex_destroy(&c2_rhs237);
  sf_mex_destroy(&c2_lhs237);
  sf_mex_destroy(&c2_rhs238);
  sf_mex_destroy(&c2_lhs238);
  sf_mex_destroy(&c2_rhs239);
  sf_mex_destroy(&c2_lhs239);
  sf_mex_destroy(&c2_rhs240);
  sf_mex_destroy(&c2_lhs240);
  sf_mex_destroy(&c2_rhs241);
  sf_mex_destroy(&c2_lhs241);
  sf_mex_destroy(&c2_rhs242);
  sf_mex_destroy(&c2_lhs242);
  sf_mex_destroy(&c2_rhs243);
  sf_mex_destroy(&c2_lhs243);
  sf_mex_destroy(&c2_rhs244);
  sf_mex_destroy(&c2_lhs244);
  sf_mex_destroy(&c2_rhs245);
  sf_mex_destroy(&c2_lhs245);
  sf_mex_destroy(&c2_rhs246);
  sf_mex_destroy(&c2_lhs246);
  sf_mex_destroy(&c2_rhs247);
  sf_mex_destroy(&c2_lhs247);
  sf_mex_destroy(&c2_rhs248);
  sf_mex_destroy(&c2_lhs248);
  sf_mex_destroy(&c2_rhs249);
  sf_mex_destroy(&c2_lhs249);
  sf_mex_destroy(&c2_rhs250);
  sf_mex_destroy(&c2_lhs250);
  sf_mex_destroy(&c2_rhs251);
  sf_mex_destroy(&c2_lhs251);
  sf_mex_destroy(&c2_rhs252);
  sf_mex_destroy(&c2_lhs252);
  sf_mex_destroy(&c2_rhs253);
  sf_mex_destroy(&c2_lhs253);
  sf_mex_destroy(&c2_rhs254);
  sf_mex_destroy(&c2_lhs254);
  sf_mex_destroy(&c2_rhs255);
  sf_mex_destroy(&c2_lhs255);
}

static void c2_e_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs256 = NULL;
  const mxArray *c2_lhs256 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN"),
                  "context", "context", 256);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 256);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 256);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 256);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1376002288U), "fileTimeLo",
                  "fileTimeLo", 256);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 256);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 256);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 256);
  sf_mex_assign(&c2_rhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs256), "rhs", "rhs",
                  256);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs256), "lhs", "lhs",
                  256);
  sf_mex_destroy(&c2_rhs256);
  sf_mex_destroy(&c2_lhs256);
}

static void c2_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a[3],
                     real_T c2_y[3])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  real_T c2_b_a;
  real_T c2_b_y;
  for (c2_k = 0; c2_k < 3; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 3, 1, 0) - 1];
    c2_b_a = c2_ak;
    c2_eml_scalar_eg(chartInstance);
    c2_b_y = c2_b_a * c2_b_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 3, 1, 0) - 1] = c2_b_y;
  }
}

static void c2_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[3])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  (void)chartInstance;
  c2_y = c2_x[0];
  for (c2_k = 2; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_y += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 3, 1, 0) - 1];
  }

  return c2_y;
}

static void c2_eml_switch_helper(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_c_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c2_i481;
  static char_T c2_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i482;
  static char_T c2_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  (void)chartInstance;
  for (c2_i481 = 0; c2_i481 < 30; c2_i481++) {
    c2_u[c2_i481] = c2_cv0[c2_i481];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c2_i482 = 0; c2_i482 < 4; c2_i482++) {
    c2_b_u[c2_i482] = c2_cv1[c2_i482];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static void c2_rdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[3],
  real_T c2_y, real_T c2_z[3])
{
  real_T c2_b_y;
  real_T c2_c_y;
  int32_T c2_i483;
  (void)chartInstance;
  c2_b_y = c2_y;
  c2_c_y = c2_b_y;
  for (c2_i483 = 0; c2_i483 < 3; c2_i483++) {
    c2_z[c2_i483] = c2_x[c2_i483] / c2_c_y;
  }
}

static void c2_RotateVecSensor2Cont(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_vec[3], real_T c2_b_q_s_c[4], real_T c2_rvec[3])
{
  uint32_T c2_debug_family_var_map[6];
  real_T c2_rvec_temp[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i484;
  real_T c2_c_q_s_c[4];
  real_T c2_dv77[4];
  int32_T c2_i485;
  real_T c2_dv78[4];
  real_T c2_dv79[4];
  int32_T c2_i486;
  int32_T c2_i487;
  real_T c2_dv80[4];
  int32_T c2_i488;
  real_T c2_d_q_s_c[4];
  real_T c2_dv81[4];
  int32_T c2_i489;
  int32_T c2_i490;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_d_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rvec_temp, 0U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_vec, 3U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q_s_c, 4U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rvec, 5U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  CV_EML_FCN(0, 7);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 262);
  for (c2_i484 = 0; c2_i484 < 4; c2_i484++) {
    c2_c_q_s_c[c2_i484] = c2_b_q_s_c[c2_i484];
  }

  c2_quatinv(chartInstance, c2_c_q_s_c, c2_dv77);
  for (c2_i485 = 0; c2_i485 < 4; c2_i485++) {
    c2_dv78[c2_i485] = c2_dv77[c2_i485];
  }

  c2_dv79[0] = 0.0;
  for (c2_i486 = 0; c2_i486 < 3; c2_i486++) {
    c2_dv79[c2_i486 + 1] = c2_vec[c2_i486];
  }

  c2_quatmultiply(chartInstance, c2_dv78, c2_dv79, c2_dv77);
  for (c2_i487 = 0; c2_i487 < 4; c2_i487++) {
    c2_dv80[c2_i487] = c2_dv77[c2_i487];
  }

  for (c2_i488 = 0; c2_i488 < 4; c2_i488++) {
    c2_d_q_s_c[c2_i488] = c2_b_q_s_c[c2_i488];
  }

  c2_quatmultiply(chartInstance, c2_dv80, c2_d_q_s_c, c2_dv81);
  for (c2_i489 = 0; c2_i489 < 4; c2_i489++) {
    c2_rvec_temp[c2_i489] = c2_dv81[c2_i489];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 263);
  for (c2_i490 = 0; c2_i490 < 3; c2_i490++) {
    c2_rvec[c2_i490] = c2_rvec_temp[c2_i490 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -263);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_quatinv(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_qin[4], real_T c2_qinv[4])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_q_conj[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i491;
  int32_T c2_i492;
  real_T c2_b_qin[4];
  real_T c2_x[4];
  int32_T c2_i493;
  real_T c2_b_x[4];
  real_T c2_c_x;
  real_T c2_d_x;
  int32_T c2_i494;
  real_T c2_y;
  real_T c2_b_y;
  real_T c2_c_y;
  int32_T c2_i495;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_conj, 0U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qin, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qinv, 4U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  CV_EML_FCN(0, 6);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, MAX_uint8_T);
  c2_q_conj[0] = c2_qin[0];
  for (c2_i491 = 0; c2_i491 < 3; c2_i491++) {
    c2_q_conj[c2_i491 + 1] = -c2_qin[c2_i491 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 256);
  for (c2_i492 = 0; c2_i492 < 4; c2_i492++) {
    c2_b_qin[c2_i492] = c2_qin[c2_i492];
  }

  c2_b_power(chartInstance, c2_b_qin, c2_x);
  for (c2_i493 = 0; c2_i493 < 4; c2_i493++) {
    c2_b_x[c2_i493] = c2_x[c2_i493];
  }

  c2_c_x = c2_b_sum(chartInstance, c2_b_x);
  c2_d_x = c2_c_x;
  if (c2_d_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_d_x = muDoubleScalarSqrt(c2_d_x);
  for (c2_i494 = 0; c2_i494 < 4; c2_i494++) {
    c2_x[c2_i494] = c2_q_conj[c2_i494];
  }

  c2_y = c2_d_x;
  c2_b_y = c2_y;
  c2_c_y = c2_b_y;
  for (c2_i495 = 0; c2_i495 < 4; c2_i495++) {
    c2_qinv[c2_i495] = c2_x[c2_i495] / c2_c_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -256);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_b_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a[4],
  real_T c2_y[4])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  real_T c2_b_a;
  real_T c2_b_y;
  for (c2_k = 0; c2_k < 4; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 4, 1, 0) - 1];
    c2_b_a = c2_ak;
    c2_eml_scalar_eg(chartInstance);
    c2_b_y = c2_b_a * c2_b_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 4, 1, 0) - 1] = c2_b_y;
  }
}

static real_T c2_b_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[4])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  (void)chartInstance;
  c2_y = c2_x[0];
  for (c2_k = 2; c2_k < 5; c2_k++) {
    c2_b_k = c2_k;
    c2_y += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 4, 1, 0) - 1];
  }

  return c2_y;
}

static void c2_b_rdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[4], real_T c2_y, real_T c2_z[4])
{
  real_T c2_b_y;
  real_T c2_c_y;
  int32_T c2_i496;
  (void)chartInstance;
  c2_b_y = c2_y;
  c2_c_y = c2_b_y;
  for (c2_i496 = 0; c2_i496 < 4; c2_i496++) {
    c2_z[c2_i496] = c2_x[c2_i496] / c2_c_y;
  }
}

static void c2_quatmultiply(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_q[4], real_T c2_r[4], real_T c2_qres[4])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_vec[3];
  real_T c2_scalar;
  real_T c2_b_q[4];
  real_T c2_b_r[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i497;
  int32_T c2_i498;
  real_T c2_c_q[3];
  real_T c2_c_r[3];
  real_T c2_d_q[3];
  int32_T c2_i499;
  real_T c2_b_scalar[4];
  int32_T c2_i500;
  int32_T c2_i501;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 9U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_vec, 0U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_scalar, 1U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q, MAX_uint32_T, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_r, MAX_uint32_T, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qres, 6U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 235U);
  for (c2_i497 = 0; c2_i497 < 4; c2_i497++) {
    c2_b_q[c2_i497] = c2_q[c2_i497];
  }

  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 236U);
  for (c2_i498 = 0; c2_i498 < 4; c2_i498++) {
    c2_b_r[c2_i498] = c2_r[c2_i498];
  }

  _SFD_SYMBOL_SWITCH(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 239U);
  c2_c_q[0] = c2_b_q[0] * c2_b_r[1];
  c2_c_q[1] = c2_b_q[0] * c2_b_r[2];
  c2_c_q[2] = c2_b_q[0] * c2_b_r[3];
  c2_c_r[0] = c2_b_r[0] * c2_b_q[1];
  c2_c_r[1] = c2_b_r[0] * c2_b_q[2];
  c2_c_r[2] = c2_b_r[0] * c2_b_q[3];
  c2_d_q[0] = c2_b_q[2] * c2_b_r[3] - c2_b_q[3] * c2_b_r[2];
  c2_d_q[1] = c2_b_q[3] * c2_b_r[1] - c2_b_q[1] * c2_b_r[3];
  c2_d_q[2] = c2_b_q[1] * c2_b_r[2] - c2_b_q[2] * c2_b_r[1];
  for (c2_i499 = 0; c2_i499 < 3; c2_i499++) {
    c2_vec[c2_i499] = (c2_c_q[c2_i499] + c2_c_r[c2_i499]) + c2_d_q[c2_i499];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  c2_scalar = ((c2_b_q[0] * c2_b_r[0] - c2_b_q[1] * c2_b_r[1]) - c2_b_q[2] *
               c2_b_r[2]) - c2_b_q[3] * c2_b_r[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 250U);
  c2_b_scalar[0] = c2_scalar;
  for (c2_i500 = 0; c2_i500 < 3; c2_i500++) {
    c2_b_scalar[c2_i500 + 1] = c2_vec[c2_i500];
  }

  for (c2_i501 = 0; c2_i501 < 4; c2_i501++) {
    c2_qres[c2_i501] = c2_b_scalar[c2_i501];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -250);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c2_mpower(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_eml_scalar_eg(chartInstance);
  return c2_d_a * c2_d_a;
}

static void c2_b_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c2_i502;
  static char_T c2_cv2[48] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'c', 'h', 'o', 'l', '_', 'm', 'a', 't', 'r', 'i', 'x', 'M',
    'u', 's', 't', 'B', 'e', 'P', 'o', 's', 'D', 'e', 'f', 'W', 'i', 't', 'h',
    'R', 'e', 'a', 'l', 'D', 'i', 'a', 'g' };

  char_T c2_u[48];
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  for (c2_i502 = 0; c2_i502 < 48; c2_i502++) {
    c2_u[c2_i502] = c2_cv2[c2_i502];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 48), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static void c2_eml_matlab_zpotrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[81], real_T c2_b_A[81], int32_T *c2_info)
{
  int32_T c2_i503;
  for (c2_i503 = 0; c2_i503 < 81; c2_i503++) {
    c2_b_A[c2_i503] = c2_A[c2_i503];
  }

  *c2_info = c2_b_eml_matlab_zpotrf(chartInstance, c2_b_A);
}

static real_T c2_eml_xdotc(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[81], int32_T c2_ix0, real_T c2_y[81], int32_T c2_iy0)
{
  real_T c2_d;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_d_n;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_e_n;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_d_n = c2_c_n;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_e_n = c2_d_n;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_d = 0.0;
  if (c2_e_n < 1) {
  } else {
    c2_ix = c2_e_ix0;
    c2_iy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
      c2_d += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 81, 1, 0) - 1] * c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 81, 1, 0) - 1];
      c2_a = c2_ix + 1;
      c2_ix = c2_a;
      c2_b_a = c2_iy + 1;
      c2_iy = c2_b_a;
    }
  }

  return c2_d;
}

static void c2_check_forloop_overflow_error(SFc2_UKF_10hzInstanceStruct
  *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i504;
  static char_T c2_cv3[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i505;
  static char_T c2_cv4[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  (void)chartInstance;
  if (!c2_overflow) {
  } else {
    for (c2_i504 = 0; c2_i504 < 34; c2_i504++) {
      c2_u[c2_i504] = c2_cv3[c2_i504];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c2_i505 = 0; c2_i505 < 23; c2_i505++) {
      c2_b_u[c2_i505] = c2_cv4[c2_i505];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c2_y, 14, c2_b_y));
  }
}

static void c2_eml_xgemv(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0, real_T c2_y[81], int32_T
  c2_iy0, real_T c2_b_y[81])
{
  int32_T c2_i506;
  for (c2_i506 = 0; c2_i506 < 81; c2_i506++) {
    c2_b_y[c2_i506] = c2_y[c2_i506];
  }

  c2_b_eml_xgemv(chartInstance, c2_m, c2_n, c2_ia0, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_below_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_below_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_c_eml_error(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c2_i507;
  static char_T c2_cv5[19] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'p', 'o', 's', 'd', 'e', 'f' };

  char_T c2_u[19];
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  for (c2_i507 = 0; c2_i507 < 19; c2_i507++) {
    c2_u[c2_i507] = c2_cv5[c2_i507];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 19), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static void c2_c_power(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a
  [57], real_T c2_y[57])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  real_T c2_b_a;
  real_T c2_b_y;
  for (c2_k = 0; c2_k < 57; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 57, 1, 0) - 1];
    c2_b_a = c2_ak;
    c2_eml_scalar_eg(chartInstance);
    c2_b_y = c2_b_a * c2_b_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 57, 1, 0) - 1] = c2_b_y;
  }
}

static void c2_c_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[57],
                     real_T c2_y[19])
{
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_i;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_b_a;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_e_a;
  int32_T c2_f_a;
  (void)chartInstance;
  c2_ix = 0;
  c2_iy = 0;
  for (c2_i = 1; c2_i < 20; c2_i++) {
    c2_ixstart = c2_ix;
    c2_a = c2_ixstart;
    c2_b_a = c2_a;
    c2_ixstart = c2_b_a;
    c2_ix = c2_ixstart + 1;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 57, 1, 0) - 1];
    for (c2_k = 2; c2_k < 4; c2_k++) {
      c2_c_a = c2_ix;
      c2_d_a = c2_c_a + 1;
      c2_ix = c2_d_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 57, 1, 0) - 1];
    }

    c2_e_a = c2_iy;
    c2_f_a = c2_e_a + 1;
    c2_iy = c2_f_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 19, 1, 0) - 1] = c2_s;
  }
}

static void c2_b_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[19],
                      real_T c2_b_x[19])
{
  int32_T c2_i508;
  for (c2_i508 = 0; c2_i508 < 19; c2_i508++) {
    c2_b_x[c2_i508] = c2_x[c2_i508];
  }

  c2_d_sqrt(chartInstance, c2_b_x);
}

static void c2_b_quatmultiply(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_q[76], real_T c2_r[4], real_T c2_qres[76])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_vec[57];
  real_T c2_scalar[19];
  real_T c2_b_q[76];
  real_T c2_b_r[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i509;
  int32_T c2_i510;
  int32_T c2_i511;
  int32_T c2_i512;
  int32_T c2_i513;
  real_T c2_c_r;
  int32_T c2_i514;
  real_T c2_c_q[19];
  real_T c2_d_r;
  int32_T c2_i515;
  real_T c2_d_q[19];
  real_T c2_e_r;
  int32_T c2_i516;
  real_T c2_e_q[19];
  real_T c2_f_r;
  int32_T c2_i517;
  real_T c2_g_r[19];
  real_T c2_h_r;
  int32_T c2_i518;
  real_T c2_i_r[19];
  real_T c2_j_r;
  int32_T c2_i519;
  real_T c2_k_r[19];
  real_T c2_l_r;
  real_T c2_m_r;
  int32_T c2_i520;
  real_T c2_f_q[19];
  real_T c2_n_r;
  real_T c2_o_r;
  int32_T c2_i521;
  real_T c2_g_q[19];
  real_T c2_p_r;
  real_T c2_q_r;
  int32_T c2_i522;
  real_T c2_h_q[19];
  int32_T c2_i523;
  real_T c2_i_q[57];
  int32_T c2_i524;
  int32_T c2_i525;
  int32_T c2_i526;
  real_T c2_r_r[57];
  int32_T c2_i527;
  int32_T c2_i528;
  int32_T c2_i529;
  real_T c2_j_q[57];
  int32_T c2_i530;
  int32_T c2_i531;
  int32_T c2_i532;
  int32_T c2_i533;
  int32_T c2_i534;
  real_T c2_s_r;
  real_T c2_t_r;
  real_T c2_u_r;
  real_T c2_v_r;
  int32_T c2_i535;
  int32_T c2_i536;
  int32_T c2_i537;
  int32_T c2_i538;
  int32_T c2_i539;
  int32_T c2_i540;
  int32_T c2_i541;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 9U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_vec, 0U, c2_db_sf_marshallOut,
    c2_db_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_scalar, 1U, c2_cb_sf_marshallOut,
    c2_cb_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q, MAX_uint32_T,
    c2_bb_sf_marshallOut, c2_bb_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_r, MAX_uint32_T, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 2U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_r, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_qres, 6U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  CV_EML_FCN(0, 5);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 235U);
  c2_i509 = 0;
  for (c2_i510 = 0; c2_i510 < 4; c2_i510++) {
    c2_i511 = 0;
    for (c2_i512 = 0; c2_i512 < 19; c2_i512++) {
      c2_b_q[c2_i512 + c2_i509] = c2_q[c2_i511 + c2_i510];
      c2_i511 += 4;
    }

    c2_i509 += 19;
  }

  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 236U);
  for (c2_i513 = 0; c2_i513 < 4; c2_i513++) {
    c2_b_r[c2_i513] = c2_r[c2_i513];
  }

  _SFD_SYMBOL_SWITCH(3U, 3U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 239U);
  c2_c_r = c2_b_r[1];
  for (c2_i514 = 0; c2_i514 < 19; c2_i514++) {
    c2_c_q[c2_i514] = c2_b_q[c2_i514] * c2_c_r;
  }

  c2_d_r = c2_b_r[2];
  for (c2_i515 = 0; c2_i515 < 19; c2_i515++) {
    c2_d_q[c2_i515] = c2_b_q[c2_i515] * c2_d_r;
  }

  c2_e_r = c2_b_r[3];
  for (c2_i516 = 0; c2_i516 < 19; c2_i516++) {
    c2_e_q[c2_i516] = c2_b_q[c2_i516] * c2_e_r;
  }

  c2_f_r = c2_b_r[0];
  for (c2_i517 = 0; c2_i517 < 19; c2_i517++) {
    c2_g_r[c2_i517] = c2_f_r * c2_b_q[c2_i517 + 19];
  }

  c2_h_r = c2_b_r[0];
  for (c2_i518 = 0; c2_i518 < 19; c2_i518++) {
    c2_i_r[c2_i518] = c2_h_r * c2_b_q[c2_i518 + 38];
  }

  c2_j_r = c2_b_r[0];
  for (c2_i519 = 0; c2_i519 < 19; c2_i519++) {
    c2_k_r[c2_i519] = c2_j_r * c2_b_q[c2_i519 + 57];
  }

  c2_l_r = c2_b_r[3];
  c2_m_r = c2_b_r[2];
  for (c2_i520 = 0; c2_i520 < 19; c2_i520++) {
    c2_f_q[c2_i520] = c2_b_q[c2_i520 + 38] * c2_l_r - c2_b_q[c2_i520 + 57] *
      c2_m_r;
  }

  c2_n_r = c2_b_r[1];
  c2_o_r = c2_b_r[3];
  for (c2_i521 = 0; c2_i521 < 19; c2_i521++) {
    c2_g_q[c2_i521] = c2_b_q[c2_i521 + 57] * c2_n_r - c2_b_q[c2_i521 + 19] *
      c2_o_r;
  }

  c2_p_r = c2_b_r[2];
  c2_q_r = c2_b_r[1];
  for (c2_i522 = 0; c2_i522 < 19; c2_i522++) {
    c2_h_q[c2_i522] = c2_b_q[c2_i522 + 19] * c2_p_r - c2_b_q[c2_i522 + 38] *
      c2_q_r;
  }

  for (c2_i523 = 0; c2_i523 < 19; c2_i523++) {
    c2_i_q[c2_i523] = c2_c_q[c2_i523];
  }

  for (c2_i524 = 0; c2_i524 < 19; c2_i524++) {
    c2_i_q[c2_i524 + 19] = c2_d_q[c2_i524];
  }

  for (c2_i525 = 0; c2_i525 < 19; c2_i525++) {
    c2_i_q[c2_i525 + 38] = c2_e_q[c2_i525];
  }

  for (c2_i526 = 0; c2_i526 < 19; c2_i526++) {
    c2_r_r[c2_i526] = c2_g_r[c2_i526];
  }

  for (c2_i527 = 0; c2_i527 < 19; c2_i527++) {
    c2_r_r[c2_i527 + 19] = c2_i_r[c2_i527];
  }

  for (c2_i528 = 0; c2_i528 < 19; c2_i528++) {
    c2_r_r[c2_i528 + 38] = c2_k_r[c2_i528];
  }

  for (c2_i529 = 0; c2_i529 < 19; c2_i529++) {
    c2_j_q[c2_i529] = c2_f_q[c2_i529];
  }

  for (c2_i530 = 0; c2_i530 < 19; c2_i530++) {
    c2_j_q[c2_i530 + 19] = c2_g_q[c2_i530];
  }

  for (c2_i531 = 0; c2_i531 < 19; c2_i531++) {
    c2_j_q[c2_i531 + 38] = c2_h_q[c2_i531];
  }

  c2_i532 = 0;
  for (c2_i533 = 0; c2_i533 < 3; c2_i533++) {
    for (c2_i534 = 0; c2_i534 < 19; c2_i534++) {
      c2_vec[c2_i534 + c2_i532] = (c2_i_q[c2_i534 + c2_i532] + c2_r_r[c2_i534 +
        c2_i532]) + c2_j_q[c2_i534 + c2_i532];
    }

    c2_i532 += 19;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  c2_s_r = c2_b_r[0];
  c2_t_r = c2_b_r[1];
  c2_u_r = c2_b_r[2];
  c2_v_r = c2_b_r[3];
  for (c2_i535 = 0; c2_i535 < 19; c2_i535++) {
    c2_scalar[c2_i535] = ((c2_b_q[c2_i535] * c2_s_r - c2_b_q[c2_i535 + 19] *
      c2_t_r) - c2_b_q[c2_i535 + 38] * c2_u_r) - c2_b_q[c2_i535 + 57] * c2_v_r;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 250U);
  c2_i536 = 0;
  for (c2_i537 = 0; c2_i537 < 19; c2_i537++) {
    c2_qres[c2_i536] = c2_scalar[c2_i537];
    c2_i536 += 4;
  }

  c2_i538 = 0;
  for (c2_i539 = 0; c2_i539 < 19; c2_i539++) {
    c2_i540 = 0;
    for (c2_i541 = 0; c2_i541 < 3; c2_i541++) {
      c2_qres[(c2_i541 + c2_i538) + 1] = c2_vec[c2_i540 + c2_i539];
      c2_i540 += 19;
    }

    c2_i538 += 4;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -250);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_RK4(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
                   c2_X_km1km1[190], real_T c2_b_I_c[3], real_T c2_Torque_c[3],
                   real_T c2_Ts, real_T c2_X_kkm1[190])
{
  uint32_T c2_debug_family_var_map[12];
  real_T c2_idx;
  real_T c2_k1[10];
  real_T c2_k2[10];
  real_T c2_k3[10];
  real_T c2_k4[10];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i542;
  int32_T c2_b_idx;
  int32_T c2_c_idx;
  int32_T c2_i543;
  real_T c2_b_X_km1km1[10];
  int32_T c2_i544;
  real_T c2_c_I_c[3];
  int32_T c2_i545;
  real_T c2_b_Torque_c[3];
  real_T c2_dv82[10];
  int32_T c2_i546;
  int32_T c2_i547;
  real_T c2_b[10];
  int32_T c2_i548;
  real_T c2_b_b;
  int32_T c2_i549;
  int32_T c2_d_idx;
  int32_T c2_i550;
  real_T c2_c_X_km1km1[10];
  int32_T c2_i551;
  real_T c2_d_I_c[3];
  int32_T c2_i552;
  real_T c2_c_Torque_c[3];
  real_T c2_dv83[10];
  int32_T c2_i553;
  int32_T c2_i554;
  int32_T c2_i555;
  real_T c2_c_b;
  int32_T c2_i556;
  int32_T c2_e_idx;
  int32_T c2_i557;
  real_T c2_d_X_km1km1[10];
  int32_T c2_i558;
  real_T c2_e_I_c[3];
  int32_T c2_i559;
  real_T c2_d_Torque_c[3];
  real_T c2_dv84[10];
  int32_T c2_i560;
  int32_T c2_i561;
  real_T c2_d_b;
  int32_T c2_i562;
  int32_T c2_f_idx;
  int32_T c2_i563;
  real_T c2_e_X_km1km1[10];
  int32_T c2_i564;
  real_T c2_f_I_c[3];
  int32_T c2_i565;
  real_T c2_e_Torque_c[3];
  real_T c2_dv85[10];
  int32_T c2_i566;
  int32_T c2_i567;
  int32_T c2_i568;
  int32_T c2_i569;
  real_T c2_e_b[10];
  int32_T c2_i570;
  int32_T c2_i571;
  real_T c2_f_b;
  int32_T c2_i572;
  int32_T c2_i573;
  int32_T c2_g_idx;
  int32_T c2_h_idx;
  int32_T c2_i574;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c2_h_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_idx, 0U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_k1, 1U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_k2, 2U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_k3, 3U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_k4, 4U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 5U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 6U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_km1km1, 7U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_I_c, 8U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Torque_c, 9U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Ts, 10U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_kkm1, 11U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 174U);
  for (c2_i542 = 0; c2_i542 < 190; c2_i542++) {
    c2_X_kkm1[c2_i542] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 175U);
  c2_idx = 1.0;
  c2_b_idx = 0;
  while (c2_b_idx < 19) {
    c2_idx = 1.0 + (real_T)c2_b_idx;
    CV_EML_FOR(0, 1, 2, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 179U);
    c2_c_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i543 = 0; c2_i543 < 10; c2_i543++) {
      c2_b_X_km1km1[c2_i543] = c2_X_km1km1[c2_i543 + 10 * c2_c_idx];
    }

    for (c2_i544 = 0; c2_i544 < 3; c2_i544++) {
      c2_c_I_c[c2_i544] = c2_b_I_c[c2_i544];
    }

    for (c2_i545 = 0; c2_i545 < 3; c2_i545++) {
      c2_b_Torque_c[c2_i545] = c2_Torque_c[c2_i545];
    }

    c2_Kinematics(chartInstance, c2_b_X_km1km1, c2_c_I_c, c2_b_Torque_c, c2_dv82);
    for (c2_i546 = 0; c2_i546 < 10; c2_i546++) {
      c2_k1[c2_i546] = c2_dv82[c2_i546];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 180U);
    for (c2_i547 = 0; c2_i547 < 10; c2_i547++) {
      c2_b[c2_i547] = c2_k1[c2_i547];
    }

    for (c2_i548 = 0; c2_i548 < 10; c2_i548++) {
      c2_b[c2_i548] *= 0.5;
    }

    c2_b_b = c2_Ts;
    for (c2_i549 = 0; c2_i549 < 10; c2_i549++) {
      c2_b[c2_i549] *= c2_b_b;
    }

    c2_d_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i550 = 0; c2_i550 < 10; c2_i550++) {
      c2_c_X_km1km1[c2_i550] = c2_X_km1km1[c2_i550 + 10 * c2_d_idx] +
        c2_b[c2_i550];
    }

    for (c2_i551 = 0; c2_i551 < 3; c2_i551++) {
      c2_d_I_c[c2_i551] = c2_b_I_c[c2_i551];
    }

    for (c2_i552 = 0; c2_i552 < 3; c2_i552++) {
      c2_c_Torque_c[c2_i552] = c2_Torque_c[c2_i552];
    }

    c2_Kinematics(chartInstance, c2_c_X_km1km1, c2_d_I_c, c2_c_Torque_c, c2_dv83);
    for (c2_i553 = 0; c2_i553 < 10; c2_i553++) {
      c2_k2[c2_i553] = c2_dv83[c2_i553];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 181U);
    for (c2_i554 = 0; c2_i554 < 10; c2_i554++) {
      c2_b[c2_i554] = c2_k2[c2_i554];
    }

    for (c2_i555 = 0; c2_i555 < 10; c2_i555++) {
      c2_b[c2_i555] *= 0.5;
    }

    c2_c_b = c2_Ts;
    for (c2_i556 = 0; c2_i556 < 10; c2_i556++) {
      c2_b[c2_i556] *= c2_c_b;
    }

    c2_e_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i557 = 0; c2_i557 < 10; c2_i557++) {
      c2_d_X_km1km1[c2_i557] = c2_X_km1km1[c2_i557 + 10 * c2_e_idx] +
        c2_b[c2_i557];
    }

    for (c2_i558 = 0; c2_i558 < 3; c2_i558++) {
      c2_e_I_c[c2_i558] = c2_b_I_c[c2_i558];
    }

    for (c2_i559 = 0; c2_i559 < 3; c2_i559++) {
      c2_d_Torque_c[c2_i559] = c2_Torque_c[c2_i559];
    }

    c2_Kinematics(chartInstance, c2_d_X_km1km1, c2_e_I_c, c2_d_Torque_c, c2_dv84);
    for (c2_i560 = 0; c2_i560 < 10; c2_i560++) {
      c2_k3[c2_i560] = c2_dv84[c2_i560];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 182U);
    for (c2_i561 = 0; c2_i561 < 10; c2_i561++) {
      c2_b[c2_i561] = c2_k3[c2_i561];
    }

    c2_d_b = c2_Ts;
    for (c2_i562 = 0; c2_i562 < 10; c2_i562++) {
      c2_b[c2_i562] *= c2_d_b;
    }

    c2_f_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i563 = 0; c2_i563 < 10; c2_i563++) {
      c2_e_X_km1km1[c2_i563] = c2_X_km1km1[c2_i563 + 10 * c2_f_idx] +
        c2_b[c2_i563];
    }

    for (c2_i564 = 0; c2_i564 < 3; c2_i564++) {
      c2_f_I_c[c2_i564] = c2_b_I_c[c2_i564];
    }

    for (c2_i565 = 0; c2_i565 < 3; c2_i565++) {
      c2_e_Torque_c[c2_i565] = c2_Torque_c[c2_i565];
    }

    c2_Kinematics(chartInstance, c2_e_X_km1km1, c2_f_I_c, c2_e_Torque_c, c2_dv85);
    for (c2_i566 = 0; c2_i566 < 10; c2_i566++) {
      c2_k4[c2_i566] = c2_dv85[c2_i566];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 184U);
    for (c2_i567 = 0; c2_i567 < 10; c2_i567++) {
      c2_b[c2_i567] = c2_k2[c2_i567];
    }

    for (c2_i568 = 0; c2_i568 < 10; c2_i568++) {
      c2_b[c2_i568] *= 2.0;
    }

    for (c2_i569 = 0; c2_i569 < 10; c2_i569++) {
      c2_e_b[c2_i569] = c2_k3[c2_i569];
    }

    for (c2_i570 = 0; c2_i570 < 10; c2_i570++) {
      c2_e_b[c2_i570] *= 2.0;
    }

    for (c2_i571 = 0; c2_i571 < 10; c2_i571++) {
      c2_b[c2_i571] = ((c2_k1[c2_i571] + c2_b[c2_i571]) + c2_e_b[c2_i571]) +
        c2_k4[c2_i571];
    }

    c2_f_b = c2_Ts;
    for (c2_i572 = 0; c2_i572 < 10; c2_i572++) {
      c2_b[c2_i572] *= c2_f_b;
    }

    for (c2_i573 = 0; c2_i573 < 10; c2_i573++) {
      c2_b[c2_i573] /= 6.0;
    }

    c2_g_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    c2_h_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i574 = 0; c2_i574 < 10; c2_i574++) {
      c2_X_kkm1[c2_i574 + 10 * c2_g_idx] = c2_X_km1km1[c2_i574 + 10 * c2_h_idx]
        + c2_b[c2_i574];
    }

    c2_b_idx++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 2, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -184);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_Kinematics(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[10], real_T c2_I[3], real_T c2_torque_c[3], real_T c2_results[10])
{
  uint32_T c2_debug_family_var_map[11];
  real_T c2_q[4];
  real_T c2_w[3];
  real_T c2_q_dot[4];
  real_T c2_w_dot[3];
  real_T c2_w_bias_dot[3];
  real_T c2_b_I[9];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i575;
  int32_T c2_i576;
  int32_T c2_i577;
  real_T c2_c_I[3];
  real_T c2_dv86[9];
  int32_T c2_i578;
  int32_T c2_i579;
  real_T c2_b_w[3];
  real_T c2_a[9];
  real_T c2_dv87[16];
  int32_T c2_i580;
  int32_T c2_i581;
  int32_T c2_i582;
  int32_T c2_i583;
  int32_T c2_i584;
  int32_T c2_i585;
  int32_T c2_i586;
  int32_T c2_i587;
  int32_T c2_i588;
  int32_T c2_i589;
  real_T c2_b_a[16];
  int32_T c2_i590;
  real_T c2_b[4];
  int32_T c2_i591;
  int32_T c2_i592;
  int32_T c2_i593;
  real_T c2_C[4];
  int32_T c2_i594;
  int32_T c2_i595;
  int32_T c2_i596;
  int32_T c2_i597;
  int32_T c2_i598;
  int32_T c2_i599;
  int32_T c2_i600;
  int32_T c2_i601;
  real_T c2_b_b[3];
  int32_T c2_i602;
  real_T c2_y[3];
  int32_T c2_i603;
  int32_T c2_i604;
  int32_T c2_i605;
  real_T c2_c_w[3];
  int32_T c2_i606;
  int32_T c2_i607;
  real_T c2_b_y[3];
  int32_T c2_i608;
  int32_T c2_i609;
  int32_T c2_i610;
  real_T c2_d_I[9];
  int32_T c2_i611;
  int32_T c2_i612;
  int32_T c2_i613;
  int32_T c2_i614;
  int32_T c2_i615;
  int32_T c2_i616;
  int32_T c2_i617;
  int32_T c2_i618;
  int32_T c2_i619;
  int32_T c2_i620;
  int32_T c2_i621;
  int32_T c2_i622;
  int32_T c2_i623;
  int32_T c2_i624;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 12U, c2_g_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 0U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w, 1U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_dot, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_dot, 3U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w_bias_dot, 4U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_I, MAX_uint32_T,
    c2_eb_sf_marshallOut, c2_eb_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 6U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 7U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_x, 8U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_I, 5U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_torque_c, 9U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_results, 10U, c2_w_sf_marshallOut,
    c2_w_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 215U);
  for (c2_i575 = 0; c2_i575 < 4; c2_i575++) {
    c2_q[c2_i575] = c2_x[c2_i575];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 216U);
  for (c2_i576 = 0; c2_i576 < 3; c2_i576++) {
    c2_w[c2_i576] = c2_x[c2_i576 + 4];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 217U);
  for (c2_i577 = 0; c2_i577 < 3; c2_i577++) {
    c2_c_I[c2_i577] = c2_I[c2_i577];
  }

  c2_diag(chartInstance, c2_c_I, c2_dv86);
  for (c2_i578 = 0; c2_i578 < 9; c2_i578++) {
    c2_b_I[c2_i578] = c2_dv86[c2_i578];
  }

  _SFD_SYMBOL_SWITCH(5U, 5U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 219U);
  for (c2_i579 = 0; c2_i579 < 3; c2_i579++) {
    c2_b_w[c2_i579] = c2_w[c2_i579];
  }

  c2_skew_matrix(chartInstance, c2_b_w, c2_a);
  c2_dv87[0] = 0.0;
  c2_i580 = 0;
  for (c2_i581 = 0; c2_i581 < 3; c2_i581++) {
    c2_dv87[c2_i580 + 4] = -c2_w[c2_i581];
    c2_i580 += 4;
  }

  for (c2_i582 = 0; c2_i582 < 3; c2_i582++) {
    c2_dv87[c2_i582 + 1] = c2_w[c2_i582];
  }

  c2_i583 = 0;
  c2_i584 = 0;
  for (c2_i585 = 0; c2_i585 < 3; c2_i585++) {
    for (c2_i586 = 0; c2_i586 < 3; c2_i586++) {
      c2_dv87[(c2_i586 + c2_i583) + 5] = -c2_a[c2_i586 + c2_i584];
    }

    c2_i583 += 4;
    c2_i584 += 3;
  }

  c2_i587 = 0;
  for (c2_i588 = 0; c2_i588 < 4; c2_i588++) {
    for (c2_i589 = 0; c2_i589 < 4; c2_i589++) {
      c2_b_a[c2_i589 + c2_i587] = 0.5 * c2_dv87[c2_i589 + c2_i587];
    }

    c2_i587 += 4;
  }

  for (c2_i590 = 0; c2_i590 < 4; c2_i590++) {
    c2_b[c2_i590] = c2_q[c2_i590];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i591 = 0; c2_i591 < 4; c2_i591++) {
    c2_q_dot[c2_i591] = 0.0;
  }

  for (c2_i592 = 0; c2_i592 < 4; c2_i592++) {
    c2_q_dot[c2_i592] = 0.0;
  }

  for (c2_i593 = 0; c2_i593 < 4; c2_i593++) {
    c2_C[c2_i593] = c2_q_dot[c2_i593];
  }

  for (c2_i594 = 0; c2_i594 < 4; c2_i594++) {
    c2_q_dot[c2_i594] = c2_C[c2_i594];
  }

  c2_threshold(chartInstance);
  for (c2_i595 = 0; c2_i595 < 4; c2_i595++) {
    c2_C[c2_i595] = c2_q_dot[c2_i595];
  }

  for (c2_i596 = 0; c2_i596 < 4; c2_i596++) {
    c2_q_dot[c2_i596] = c2_C[c2_i596];
  }

  for (c2_i597 = 0; c2_i597 < 4; c2_i597++) {
    c2_q_dot[c2_i597] = 0.0;
    c2_i598 = 0;
    for (c2_i599 = 0; c2_i599 < 4; c2_i599++) {
      c2_q_dot[c2_i597] += c2_b_a[c2_i598 + c2_i597] * c2_b[c2_i599];
      c2_i598 += 4;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 220U);
  for (c2_i600 = 0; c2_i600 < 9; c2_i600++) {
    c2_a[c2_i600] = c2_b_I[c2_i600];
  }

  for (c2_i601 = 0; c2_i601 < 3; c2_i601++) {
    c2_b_b[c2_i601] = c2_w[c2_i601];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i602 = 0; c2_i602 < 3; c2_i602++) {
    c2_y[c2_i602] = 0.0;
    c2_i603 = 0;
    for (c2_i604 = 0; c2_i604 < 3; c2_i604++) {
      c2_y[c2_i602] += c2_a[c2_i603 + c2_i602] * c2_b_b[c2_i604];
      c2_i603 += 3;
    }
  }

  for (c2_i605 = 0; c2_i605 < 3; c2_i605++) {
    c2_c_w[c2_i605] = c2_w[c2_i605];
  }

  c2_skew_matrix(chartInstance, c2_c_w, c2_a);
  for (c2_i606 = 0; c2_i606 < 9; c2_i606++) {
    c2_a[c2_i606] = -c2_a[c2_i606];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_threshold(chartInstance);
  for (c2_i607 = 0; c2_i607 < 3; c2_i607++) {
    c2_b_y[c2_i607] = 0.0;
    c2_i608 = 0;
    for (c2_i609 = 0; c2_i609 < 3; c2_i609++) {
      c2_b_y[c2_i607] += c2_a[c2_i608 + c2_i607] * c2_y[c2_i609];
      c2_i608 += 3;
    }
  }

  for (c2_i610 = 0; c2_i610 < 9; c2_i610++) {
    c2_d_I[c2_i610] = c2_b_I[c2_i610];
  }

  c2_b_mpower(chartInstance, c2_d_I, c2_a);
  for (c2_i611 = 0; c2_i611 < 3; c2_i611++) {
    c2_b_y[c2_i611] += c2_torque_c[c2_i611];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i612 = 0; c2_i612 < 3; c2_i612++) {
    c2_w_dot[c2_i612] = 0.0;
  }

  for (c2_i613 = 0; c2_i613 < 3; c2_i613++) {
    c2_w_dot[c2_i613] = 0.0;
  }

  for (c2_i614 = 0; c2_i614 < 3; c2_i614++) {
    c2_b_b[c2_i614] = c2_w_dot[c2_i614];
  }

  for (c2_i615 = 0; c2_i615 < 3; c2_i615++) {
    c2_w_dot[c2_i615] = c2_b_b[c2_i615];
  }

  c2_threshold(chartInstance);
  for (c2_i616 = 0; c2_i616 < 3; c2_i616++) {
    c2_b_b[c2_i616] = c2_w_dot[c2_i616];
  }

  for (c2_i617 = 0; c2_i617 < 3; c2_i617++) {
    c2_w_dot[c2_i617] = c2_b_b[c2_i617];
  }

  for (c2_i618 = 0; c2_i618 < 3; c2_i618++) {
    c2_w_dot[c2_i618] = 0.0;
    c2_i619 = 0;
    for (c2_i620 = 0; c2_i620 < 3; c2_i620++) {
      c2_w_dot[c2_i618] += c2_a[c2_i619 + c2_i618] * c2_b_y[c2_i620];
      c2_i619 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 221U);
  for (c2_i621 = 0; c2_i621 < 3; c2_i621++) {
    c2_w_bias_dot[c2_i621] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 223U);
  for (c2_i622 = 0; c2_i622 < 4; c2_i622++) {
    c2_results[c2_i622] = c2_q_dot[c2_i622];
  }

  for (c2_i623 = 0; c2_i623 < 3; c2_i623++) {
    c2_results[c2_i623 + 4] = c2_w_dot[c2_i623];
  }

  for (c2_i624 = 0; c2_i624 < 3; c2_i624++) {
    c2_results[c2_i624 + 7] = c2_w_bias_dot[c2_i624];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -223);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_diag(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_v[3],
                    real_T c2_d[9])
{
  int32_T c2_i625;
  int32_T c2_j;
  int32_T c2_b_j;
  (void)chartInstance;
  for (c2_i625 = 0; c2_i625 < 9; c2_i625++) {
    c2_d[c2_i625] = 0.0;
  }

  for (c2_j = 1; c2_j < 4; c2_j++) {
    c2_b_j = c2_j;
    c2_d[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 3, 2, 0) - 1)) -
      1] = c2_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_j), 1, 3, 1, 0) - 1];
  }
}

static void c2_skew_matrix(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_x[3], real_T c2_output[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c2_f_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_x, 2U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_output, 3U, c2_eb_sf_marshallOut,
    c2_eb_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 229U);
  c2_output[0] = 0.0;
  c2_output[3] = -c2_x[2];
  c2_output[6] = c2_x[1];
  c2_output[1] = c2_x[2];
  c2_output[4] = 0.0;
  c2_output[7] = -c2_x[0];
  c2_output[2] = -c2_x[1];
  c2_output[5] = c2_x[0];
  c2_output[8] = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -229);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_b_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_mpower(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_a
  [9], real_T c2_c[9])
{
  int32_T c2_i626;
  real_T c2_b_a[9];
  int32_T c2_i627;
  real_T c2_c_a[9];
  real_T c2_n1x;
  int32_T c2_i628;
  real_T c2_b_c[9];
  real_T c2_n1xinv;
  real_T c2_rc;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_x;
  int32_T c2_i629;
  static char_T c2_cv6[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c2_u[8];
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  char_T c2_str[14];
  int32_T c2_i630;
  char_T c2_b_str[14];
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  for (c2_i626 = 0; c2_i626 < 9; c2_i626++) {
    c2_b_a[c2_i626] = c2_a[c2_i626];
  }

  c2_inv3x3(chartInstance, c2_b_a, c2_c);
  for (c2_i627 = 0; c2_i627 < 9; c2_i627++) {
    c2_c_a[c2_i627] = c2_a[c2_i627];
  }

  c2_n1x = c2_norm(chartInstance, c2_c_a);
  for (c2_i628 = 0; c2_i628 < 9; c2_i628++) {
    c2_b_c[c2_i628] = c2_c[c2_i628];
  }

  c2_n1xinv = c2_norm(chartInstance, c2_b_c);
  c2_rc = 1.0 / (c2_n1x * c2_n1xinv);
  guard1 = false;
  guard2 = false;
  if (c2_n1x == 0.0) {
    guard2 = true;
  } else if (c2_n1xinv == 0.0) {
    guard2 = true;
  } else if (c2_rc == 0.0) {
    guard1 = true;
  } else {
    c2_x = c2_rc;
    c2_b = muDoubleScalarIsNaN(c2_x);
    guard3 = false;
    if (c2_b) {
      guard3 = true;
    } else {
      c2_eps(chartInstance);
      if (c2_rc < 2.2204460492503131E-16) {
        guard3 = true;
      }
    }

    if (guard3 == true) {
      c2_b_x = c2_rc;
      for (c2_i629 = 0; c2_i629 < 8; c2_i629++) {
        c2_u[c2_i629] = c2_cv6[c2_i629];
      }

      c2_y = NULL;
      sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    false);
      c2_b_u = 14.0;
      c2_b_y = NULL;
      sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0),
                    false);
      c2_c_u = 6.0;
      c2_c_y = NULL;
      sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0),
                    false);
      c2_d_u = c2_b_x;
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0),
                    false);
      c2_ub_emlrt_marshallIn(chartInstance, sf_mex_call_debug
        (sfGlobalDebugInstanceStruct, "sprintf", 1U, 2U, 14, sf_mex_call_debug
         (sfGlobalDebugInstanceStruct, "sprintf", 1U, 3U, 14, c2_y, 14, c2_b_y,
          14, c2_c_y), 14, c2_d_y), "sprintf", c2_str);
      for (c2_i630 = 0; c2_i630 < 14; c2_i630++) {
        c2_b_str[c2_i630] = c2_str[c2_i630];
      }

      c2_b_eml_warning(chartInstance, c2_b_str);
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c2_eml_warning(chartInstance);
  }
}

static void c2_inv3x3(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[9],
                      real_T c2_y[9])
{
  int32_T c2_p1;
  int32_T c2_p2;
  int32_T c2_p3;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_absx11;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absx21;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_absx31;
  real_T c2_t1;
  real_T c2_h_x;
  real_T c2_b_y;
  real_T c2_i_x;
  real_T c2_c_y;
  real_T c2_z;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_b_z;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_f_y;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_g_y;
  int32_T c2_itmp;
  real_T c2_p_x;
  real_T c2_h_y;
  real_T c2_q_x;
  real_T c2_i_y;
  real_T c2_c_z;
  real_T c2_r_x;
  real_T c2_j_y;
  real_T c2_s_x;
  real_T c2_k_y;
  real_T c2_t3;
  real_T c2_t_x;
  real_T c2_l_y;
  real_T c2_u_x;
  real_T c2_m_y;
  real_T c2_t2;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_c;
  real_T c2_v_x;
  real_T c2_n_y;
  real_T c2_w_x;
  real_T c2_o_y;
  real_T c2_d_z;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_c_c;
  real_T c2_x_x;
  real_T c2_p_y;
  real_T c2_y_x;
  real_T c2_q_y;
  real_T c2_ab_x;
  real_T c2_r_y;
  real_T c2_bb_x;
  real_T c2_s_y;
  int32_T c2_g_a;
  int32_T c2_h_a;
  int32_T c2_d_c;
  real_T c2_cb_x;
  real_T c2_t_y;
  real_T c2_db_x;
  real_T c2_u_y;
  real_T c2_e_z;
  int32_T c2_i_a;
  int32_T c2_j_a;
  int32_T c2_e_c;
  int32_T c2_k_a;
  int32_T c2_l_a;
  int32_T c2_f_c;
  real_T c2_v_y;
  real_T c2_w_y;
  real_T c2_eb_x;
  real_T c2_x_y;
  real_T c2_fb_x;
  real_T c2_y_y;
  int32_T c2_m_a;
  int32_T c2_n_a;
  int32_T c2_g_c;
  real_T c2_gb_x;
  real_T c2_ab_y;
  real_T c2_hb_x;
  real_T c2_bb_y;
  real_T c2_f_z;
  int32_T c2_o_a;
  int32_T c2_p_a;
  int32_T c2_h_c;
  int32_T c2_q_a;
  int32_T c2_r_a;
  int32_T c2_i_c;
  boolean_T guard1 = false;
  (void)chartInstance;
  c2_p1 = 0;
  c2_p2 = 3;
  c2_p3 = 6;
  c2_b_x = c2_x[0];
  c2_c_x = c2_b_x;
  c2_absx11 = muDoubleScalarAbs(c2_c_x);
  c2_d_x = c2_x[1];
  c2_e_x = c2_d_x;
  c2_absx21 = muDoubleScalarAbs(c2_e_x);
  c2_f_x = c2_x[2];
  c2_g_x = c2_f_x;
  c2_absx31 = muDoubleScalarAbs(c2_g_x);
  guard1 = false;
  if (c2_absx21 > c2_absx11) {
    if (c2_absx21 > c2_absx31) {
      c2_p1 = 3;
      c2_p2 = 0;
      c2_t1 = c2_x[0];
      c2_x[0] = c2_x[1];
      c2_x[1] = c2_t1;
      c2_t1 = c2_x[3];
      c2_x[3] = c2_x[4];
      c2_x[4] = c2_t1;
      c2_t1 = c2_x[6];
      c2_x[6] = c2_x[7];
      c2_x[7] = c2_t1;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    if (c2_absx31 > c2_absx11) {
      c2_p1 = 6;
      c2_p3 = 0;
      c2_t1 = c2_x[0];
      c2_x[0] = c2_x[2];
      c2_x[2] = c2_t1;
      c2_t1 = c2_x[3];
      c2_x[3] = c2_x[5];
      c2_x[5] = c2_t1;
      c2_t1 = c2_x[6];
      c2_x[6] = c2_x[8];
      c2_x[8] = c2_t1;
    }
  }

  c2_h_x = c2_x[1];
  c2_b_y = c2_x[0];
  c2_i_x = c2_h_x;
  c2_c_y = c2_b_y;
  c2_z = c2_i_x / c2_c_y;
  c2_x[1] = c2_z;
  c2_j_x = c2_x[2];
  c2_d_y = c2_x[0];
  c2_k_x = c2_j_x;
  c2_e_y = c2_d_y;
  c2_b_z = c2_k_x / c2_e_y;
  c2_x[2] = c2_b_z;
  c2_x[4] -= c2_x[1] * c2_x[3];
  c2_x[5] -= c2_x[2] * c2_x[3];
  c2_x[7] -= c2_x[1] * c2_x[6];
  c2_x[8] -= c2_x[2] * c2_x[6];
  c2_l_x = c2_x[5];
  c2_m_x = c2_l_x;
  c2_f_y = muDoubleScalarAbs(c2_m_x);
  c2_n_x = c2_x[4];
  c2_o_x = c2_n_x;
  c2_g_y = muDoubleScalarAbs(c2_o_x);
  if (c2_f_y > c2_g_y) {
    c2_itmp = c2_p2;
    c2_p2 = c2_p3;
    c2_p3 = c2_itmp;
    c2_t1 = c2_x[1];
    c2_x[1] = c2_x[2];
    c2_x[2] = c2_t1;
    c2_t1 = c2_x[4];
    c2_x[4] = c2_x[5];
    c2_x[5] = c2_t1;
    c2_t1 = c2_x[7];
    c2_x[7] = c2_x[8];
    c2_x[8] = c2_t1;
  }

  c2_p_x = c2_x[5];
  c2_h_y = c2_x[4];
  c2_q_x = c2_p_x;
  c2_i_y = c2_h_y;
  c2_c_z = c2_q_x / c2_i_y;
  c2_x[5] = c2_c_z;
  c2_x[8] -= c2_x[5] * c2_x[7];
  c2_r_x = c2_x[5] * c2_x[1] - c2_x[2];
  c2_j_y = c2_x[8];
  c2_s_x = c2_r_x;
  c2_k_y = c2_j_y;
  c2_t3 = c2_s_x / c2_k_y;
  c2_t_x = -(c2_x[1] + c2_x[7] * c2_t3);
  c2_l_y = c2_x[4];
  c2_u_x = c2_t_x;
  c2_m_y = c2_l_y;
  c2_t2 = c2_u_x / c2_m_y;
  c2_a = c2_p1;
  c2_b_a = c2_a + 1;
  c2_c = c2_b_a;
  c2_v_x = (1.0 - c2_x[3] * c2_t2) - c2_x[6] * c2_t3;
  c2_n_y = c2_x[0];
  c2_w_x = c2_v_x;
  c2_o_y = c2_n_y;
  c2_d_z = c2_w_x / c2_o_y;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_c), 1, 9, 1, 0) - 1] = c2_d_z;
  c2_c_a = c2_p1;
  c2_d_a = c2_c_a + 2;
  c2_b_c = c2_d_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_b_c), 1, 9, 1, 0) - 1] = c2_t2;
  c2_e_a = c2_p1;
  c2_f_a = c2_e_a + 3;
  c2_c_c = c2_f_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_c_c), 1, 9, 1, 0) - 1] = c2_t3;
  c2_x_x = -c2_x[5];
  c2_p_y = c2_x[8];
  c2_y_x = c2_x_x;
  c2_q_y = c2_p_y;
  c2_t3 = c2_y_x / c2_q_y;
  c2_ab_x = 1.0 - c2_x[7] * c2_t3;
  c2_r_y = c2_x[4];
  c2_bb_x = c2_ab_x;
  c2_s_y = c2_r_y;
  c2_t2 = c2_bb_x / c2_s_y;
  c2_g_a = c2_p2;
  c2_h_a = c2_g_a + 1;
  c2_d_c = c2_h_a;
  c2_cb_x = -(c2_x[3] * c2_t2 + c2_x[6] * c2_t3);
  c2_t_y = c2_x[0];
  c2_db_x = c2_cb_x;
  c2_u_y = c2_t_y;
  c2_e_z = c2_db_x / c2_u_y;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_d_c), 1, 9, 1, 0) - 1] = c2_e_z;
  c2_i_a = c2_p2;
  c2_j_a = c2_i_a + 2;
  c2_e_c = c2_j_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_e_c), 1, 9, 1, 0) - 1] = c2_t2;
  c2_k_a = c2_p2;
  c2_l_a = c2_k_a + 3;
  c2_f_c = c2_l_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_f_c), 1, 9, 1, 0) - 1] = c2_t3;
  c2_v_y = c2_x[8];
  c2_w_y = c2_v_y;
  c2_t3 = 1.0 / c2_w_y;
  c2_eb_x = -c2_x[7] * c2_t3;
  c2_x_y = c2_x[4];
  c2_fb_x = c2_eb_x;
  c2_y_y = c2_x_y;
  c2_t2 = c2_fb_x / c2_y_y;
  c2_m_a = c2_p3;
  c2_n_a = c2_m_a + 1;
  c2_g_c = c2_n_a;
  c2_gb_x = -(c2_x[3] * c2_t2 + c2_x[6] * c2_t3);
  c2_ab_y = c2_x[0];
  c2_hb_x = c2_gb_x;
  c2_bb_y = c2_ab_y;
  c2_f_z = c2_hb_x / c2_bb_y;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_g_c), 1, 9, 1, 0) - 1] = c2_f_z;
  c2_o_a = c2_p3;
  c2_p_a = c2_o_a + 2;
  c2_h_c = c2_p_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_h_c), 1, 9, 1, 0) - 1] = c2_t2;
  c2_q_a = c2_p3;
  c2_r_a = c2_q_a + 3;
  c2_i_c = c2_r_a;
  c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_i_c), 1, 9, 1, 0) - 1] = c2_t3;
}

static real_T c2_norm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[9])
{
  real_T c2_y;
  int32_T c2_j;
  real_T c2_b_j;
  real_T c2_s;
  int32_T c2_i;
  real_T c2_b_i;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_b_y;
  real_T c2_d_x;
  boolean_T c2_b;
  boolean_T exitg1;
  (void)chartInstance;
  c2_y = 0.0;
  c2_j = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_j < 3)) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_s = 0.0;
    for (c2_i = 0; c2_i < 3; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_b_x = c2_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c2_b_i), 1, 3, 1, 0) + 3 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c2_b_j), 1, 3, 2, 0) - 1)) - 1];
      c2_c_x = c2_b_x;
      c2_b_y = muDoubleScalarAbs(c2_c_x);
      c2_s += c2_b_y;
    }

    c2_d_x = c2_s;
    c2_b = muDoubleScalarIsNaN(c2_d_x);
    if (c2_b) {
      c2_y = rtNaN;
      exitg1 = true;
    } else {
      if (c2_s > c2_y) {
        c2_y = c2_s;
      }

      c2_j++;
    }
  }

  return c2_y;
}

static void c2_eml_warning(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  int32_T c2_i631;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  for (c2_i631 = 0; c2_i631 < 27; c2_i631++) {
    c2_u[c2_i631] = c2_varargin_1[c2_i631];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static void c2_eps(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_eml_warning(SFc2_UKF_10hzInstanceStruct *chartInstance, char_T
  c2_varargin_2[14])
{
  int32_T c2_i632;
  static char_T c2_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c2_u[33];
  const mxArray *c2_y = NULL;
  int32_T c2_i633;
  char_T c2_b_u[14];
  const mxArray *c2_b_y = NULL;
  (void)chartInstance;
  for (c2_i632 = 0; c2_i632 < 33; c2_i632++) {
    c2_u[c2_i632] = c2_varargin_1[c2_i632];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  for (c2_i633 = 0; c2_i633 < 14; c2_i633++) {
    c2_b_u[c2_i633] = c2_varargin_2[c2_i633];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static void c2_c_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[190],
                     real_T c2_y[10])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_e_a;
  int32_T c2_f_a;
  (void)chartInstance;
  c2_iy = 0;
  c2_ixstart = 0;
  for (c2_j = 1; c2_j < 11; c2_j++) {
    c2_a = c2_ixstart;
    c2_b_a = c2_a + 1;
    c2_ixstart = c2_b_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 190, 1, 0) - 1];
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_c_a = c2_ix;
      c2_d_a = c2_c_a + 10;
      c2_ix = c2_d_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 190, 1, 0) - 1];
    }

    c2_e_a = c2_iy;
    c2_f_a = c2_e_a + 1;
    c2_iy = c2_f_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 10, 1, 0) - 1] = c2_s;
  }
}

static void c2_e_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[171],
                     real_T c2_y[9])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_e_a;
  int32_T c2_f_a;
  (void)chartInstance;
  c2_iy = 0;
  c2_ixstart = 0;
  for (c2_j = 1; c2_j < 10; c2_j++) {
    c2_a = c2_ixstart;
    c2_b_a = c2_a + 1;
    c2_ixstart = c2_b_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 171, 1, 0) - 1];
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_c_a = c2_ix;
      c2_d_a = c2_c_a + 9;
      c2_ix = c2_d_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 171, 1, 0) - 1];
    }

    c2_e_a = c2_iy;
    c2_f_a = c2_e_a + 1;
    c2_iy = c2_f_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 9, 1, 0) - 1] = c2_s;
  }
}

static void c2_SensorModel(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_X_kkm1[190], real_T c2_B_ref[3], real_T c2_Z_kkm1[114])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_idx;
  real_T c2_Z_kkm1_temp[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i634;
  int32_T c2_b_idx;
  int32_T c2_c_idx;
  int32_T c2_i635;
  real_T c2_b_X_kkm1[4];
  real_T c2_dv88[4];
  int32_T c2_i636;
  real_T c2_dv89[4];
  real_T c2_dv90[4];
  int32_T c2_i637;
  int32_T c2_i638;
  real_T c2_dv91[4];
  int32_T c2_d_idx;
  int32_T c2_i639;
  real_T c2_c_X_kkm1[4];
  real_T c2_dv92[4];
  int32_T c2_i640;
  int32_T c2_e_idx;
  int32_T c2_i641;
  int32_T c2_f_idx;
  int32_T c2_g_idx;
  int32_T c2_h_idx;
  int32_T c2_i642;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c2_i_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_idx, 0U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Z_kkm1_temp, 1U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 2U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 3U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_X_kkm1, 4U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_B_ref, 5U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Z_kkm1, 6U, c2_s_sf_marshallOut,
    c2_s_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 200U);
  for (c2_i634 = 0; c2_i634 < 114; c2_i634++) {
    c2_Z_kkm1[c2_i634] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 202U);
  c2_idx = 1.0;
  c2_b_idx = 0;
  while (c2_b_idx < 19) {
    c2_idx = 1.0 + (real_T)c2_b_idx;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 205U);
    c2_c_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i635 = 0; c2_i635 < 4; c2_i635++) {
      c2_b_X_kkm1[c2_i635] = c2_X_kkm1[c2_i635 + 10 * c2_c_idx];
    }

    c2_quatinv(chartInstance, c2_b_X_kkm1, c2_dv88);
    for (c2_i636 = 0; c2_i636 < 4; c2_i636++) {
      c2_dv89[c2_i636] = c2_dv88[c2_i636];
    }

    c2_dv90[0] = 0.0;
    for (c2_i637 = 0; c2_i637 < 3; c2_i637++) {
      c2_dv90[c2_i637 + 1] = c2_B_ref[c2_i637];
    }

    c2_quatmultiply(chartInstance, c2_dv89, c2_dv90, c2_dv88);
    for (c2_i638 = 0; c2_i638 < 4; c2_i638++) {
      c2_dv91[c2_i638] = c2_dv88[c2_i638];
    }

    c2_d_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i639 = 0; c2_i639 < 4; c2_i639++) {
      c2_c_X_kkm1[c2_i639] = c2_X_kkm1[c2_i639 + 10 * c2_d_idx];
    }

    c2_quatmultiply(chartInstance, c2_dv91, c2_c_X_kkm1, c2_dv92);
    for (c2_i640 = 0; c2_i640 < 4; c2_i640++) {
      c2_Z_kkm1_temp[c2_i640] = c2_dv92[c2_i640];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 206U);
    c2_e_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i641 = 0; c2_i641 < 3; c2_i641++) {
      c2_Z_kkm1[c2_i641 + 6 * c2_e_idx] = c2_Z_kkm1_temp[1 + c2_i641];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 209U);
    c2_f_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    c2_g_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    c2_h_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_idx), 1, 19, 2, 0) - 1;
    for (c2_i642 = 0; c2_i642 < 3; c2_i642++) {
      c2_Z_kkm1[(c2_i642 + 6 * c2_f_idx) + 3] = c2_X_kkm1[(c2_i642 + 10 *
        c2_g_idx) + 4] + c2_X_kkm1[(c2_i642 + 10 * c2_h_idx) + 7];
    }

    c2_b_idx++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -209);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_f_sum(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[114],
                     real_T c2_y[6])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_e_a;
  int32_T c2_f_a;
  (void)chartInstance;
  c2_iy = 0;
  c2_ixstart = 0;
  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_a = c2_ixstart;
    c2_b_a = c2_a + 1;
    c2_ixstart = c2_b_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 114, 1, 0) - 1];
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_c_a = c2_ix;
      c2_d_a = c2_c_a + 6;
      c2_ix = c2_d_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 114, 1, 0) - 1];
    }

    c2_e_a = c2_iy;
    c2_f_a = c2_e_a + 1;
    c2_iy = c2_f_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 6, 1, 0) - 1] = c2_s;
  }
}

static void c2_mrdivide(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_A
  [54], real_T c2_B[36], real_T c2_y[54])
{
  int32_T c2_i643;
  real_T c2_b_A[36];
  int32_T c2_info;
  int32_T c2_ipiv[6];
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_i644;
  int32_T c2_i645;
  real_T c2_c_A[36];
  int32_T c2_i646;
  real_T c2_d_A[36];
  int32_T c2_xj;
  int32_T c2_b_xj;
  int32_T c2_jp;
  int32_T c2_xi;
  int32_T c2_b_xi;
  real_T c2_temp;
  for (c2_i643 = 0; c2_i643 < 36; c2_i643++) {
    c2_b_A[c2_i643] = c2_B[c2_i643];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, &c2_info);
  c2_b_info = c2_info;
  c2_c_info = c2_b_info;
  c2_d_info = c2_c_info;
  if (c2_d_info > 0) {
    c2_eml_warning(chartInstance);
  }

  for (c2_i644 = 0; c2_i644 < 54; c2_i644++) {
    c2_y[c2_i644] = c2_A[c2_i644];
  }

  for (c2_i645 = 0; c2_i645 < 36; c2_i645++) {
    c2_c_A[c2_i645] = c2_b_A[c2_i645];
  }

  c2_c_eml_xtrsm(chartInstance, c2_c_A, c2_y);
  for (c2_i646 = 0; c2_i646 < 36; c2_i646++) {
    c2_d_A[c2_i646] = c2_b_A[c2_i646];
  }

  c2_d_eml_xtrsm(chartInstance, c2_d_A, c2_y);
  for (c2_xj = 5; c2_xj > 0; c2_xj--) {
    c2_b_xj = c2_xj;
    if (c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_xj), 1, 6, 1, 0) - 1] != c2_b_xj) {
      c2_jp = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_xj), 1, 6, 1, 0) - 1];
      for (c2_xi = 1; c2_xi < 10; c2_xi++) {
        c2_b_xi = c2_xi;
        c2_temp = c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_xi), 1, 9, 1, 0) + 9 *
                        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_xj), 1, 6, 2, 0) - 1)) - 1];
        c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_xi), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_xj), 1, 6, 2, 0)
               - 1)) - 1] = c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_xi), 1, 9, 1, 0) + 9 *
          (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jp), 1, 6, 2, 0) - 1)) - 1];
        c2_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_xi), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK
               ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_jp), 1, 6, 2, 0)
               - 1)) - 1] = c2_temp;
      }
    }
  }
}

static void c2_eml_matlab_zgetrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[36], real_T c2_b_A[36], int32_T c2_ipiv[6], int32_T *c2_info)
{
  int32_T c2_i647;
  for (c2_i647 = 0; c2_i647 < 36; c2_i647++) {
    c2_b_A[c2_i647] = c2_A[c2_i647];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static int32_T c2_eml_ixamax(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_n, real_T c2_x[36], int32_T c2_ix0)
{
  int32_T c2_idxmax;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_ix;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_y;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_c_y;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_d_y;
  real_T c2_s;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  if (c2_c_n < 1) {
    c2_idxmax = 0;
  } else {
    c2_idxmax = 1;
    if (c2_c_n > 1) {
      c2_ix = c2_c_ix0;
      c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 36, 1, 0) - 1];
      c2_c_x = c2_b_x;
      c2_d_x = c2_c_x;
      c2_y = muDoubleScalarAbs(c2_d_x);
      c2_e_x = 0.0;
      c2_f_x = c2_e_x;
      c2_b_y = muDoubleScalarAbs(c2_f_x);
      c2_smax = c2_y + c2_b_y;
      c2_d_n = c2_c_n;
      c2_b = c2_d_n;
      c2_b_b = c2_b;
      if (2 > c2_b_b) {
        c2_overflow = false;
      } else {
        c2_eml_switch_helper(chartInstance);
        c2_overflow = (c2_b_b > 2147483646);
      }

      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
        c2_b_k = c2_k;
        c2_a = c2_ix + 1;
        c2_ix = c2_a;
        c2_g_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 36, 1, 0) - 1];
        c2_h_x = c2_g_x;
        c2_i_x = c2_h_x;
        c2_c_y = muDoubleScalarAbs(c2_i_x);
        c2_j_x = 0.0;
        c2_k_x = c2_j_x;
        c2_d_y = muDoubleScalarAbs(c2_k_x);
        c2_s = c2_c_y + c2_d_y;
        if (c2_s > c2_smax) {
          c2_idxmax = c2_b_k;
          c2_smax = c2_s;
        }
      }
    }
  }

  return c2_idxmax;
}

static void c2_b_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_eml_xgeru(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[36], int32_T c2_ia0, real_T c2_b_A[36])
{
  int32_T c2_i648;
  for (c2_i648 = 0; c2_i648 < 36; c2_i648++) {
    c2_b_A[c2_i648] = c2_A[c2_i648];
  }

  c2_b_eml_xgeru(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                 c2_ia0);
}

static void c2_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54], real_T c2_b_B[54])
{
  int32_T c2_i649;
  int32_T c2_i650;
  real_T c2_b_A[36];
  for (c2_i649 = 0; c2_i649 < 54; c2_i649++) {
    c2_b_B[c2_i649] = c2_B[c2_i649];
  }

  for (c2_i650 = 0; c2_i650 < 36; c2_i650++) {
    c2_b_A[c2_i650] = c2_A[c2_i650];
  }

  c2_c_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_c_threshold(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_scalarEg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54], real_T c2_b_B[54])
{
  int32_T c2_i651;
  int32_T c2_i652;
  real_T c2_b_A[36];
  for (c2_i651 = 0; c2_i651 < 54; c2_i651++) {
    c2_b_B[c2_i651] = c2_B[c2_i651];
  }

  for (c2_i652 = 0; c2_i652 < 36; c2_i652++) {
    c2_b_A[c2_i652] = c2_A[c2_i652];
  }

  c2_d_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_d_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_e_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_f_eml_scalar_eg(SFc2_UKF_10hzInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_RotateVecCont2Sensor(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_vec[3], real_T c2_b_q_s_c[4], real_T c2_rvec[3])
{
  uint32_T c2_debug_family_var_map[6];
  real_T c2_rvec_temp[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i653;
  real_T c2_c_q_s_c[4];
  real_T c2_dv93[4];
  int32_T c2_i654;
  real_T c2_dv94[4];
  int32_T c2_i655;
  real_T c2_d_q_s_c[4];
  real_T c2_dv95[4];
  int32_T c2_i656;
  real_T c2_dv96[4];
  int32_T c2_i657;
  real_T c2_dv97[4];
  real_T c2_dv98[4];
  int32_T c2_i658;
  int32_T c2_i659;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_j_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rvec_temp, 0U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_vec, 3U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_q_s_c, 4U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_rvec, 5U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  CV_EML_FCN(0, 8);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 269);
  for (c2_i653 = 0; c2_i653 < 4; c2_i653++) {
    c2_c_q_s_c[c2_i653] = c2_b_q_s_c[c2_i653];
  }

  c2_dv93[0] = 0.0;
  for (c2_i654 = 0; c2_i654 < 3; c2_i654++) {
    c2_dv93[c2_i654 + 1] = c2_vec[c2_i654];
  }

  c2_quatmultiply(chartInstance, c2_c_q_s_c, c2_dv93, c2_dv94);
  for (c2_i655 = 0; c2_i655 < 4; c2_i655++) {
    c2_d_q_s_c[c2_i655] = c2_b_q_s_c[c2_i655];
  }

  c2_quatinv(chartInstance, c2_d_q_s_c, c2_dv95);
  for (c2_i656 = 0; c2_i656 < 4; c2_i656++) {
    c2_dv96[c2_i656] = c2_dv94[c2_i656];
  }

  for (c2_i657 = 0; c2_i657 < 4; c2_i657++) {
    c2_dv97[c2_i657] = c2_dv95[c2_i657];
  }

  c2_quatmultiply(chartInstance, c2_dv96, c2_dv97, c2_dv98);
  for (c2_i658 = 0; c2_i658 < 4; c2_i658++) {
    c2_rvec_temp[c2_i658] = c2_dv98[c2_i658];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 270);
  for (c2_i659 = 0; c2_i659 < 3; c2_i659++) {
    c2_rvec[c2_i659] = c2_rvec_temp[c2_i659 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -270);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_ub_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_sprintf, const char_T *c2_identifier, char_T c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_vb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_sprintf), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_sprintf);
}

static void c2_vb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, char_T c2_y[14])
{
  char_T c2_cv7[14];
  int32_T c2_i660;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv7, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c2_i660 = 0; c2_i660 < 14; c2_i660++) {
    c2_y[c2_i660] = c2_cv7[c2_i660];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_wb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i661;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i661, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i661;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_fb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_wb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_xb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_UKF_10hz, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_yb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_UKF_10hz), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_UKF_10hz);
  return c2_y;
}

static uint8_T c2_yb_emlrt_marshallIn(SFc2_UKF_10hzInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static int32_T c2_b_eml_matlab_zpotrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[81])
{
  int32_T c2_info;
  int32_T c2_colj;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_jm1;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_i662;
  int32_T c2_i663;
  int32_T c2_i664;
  real_T c2_b_A[81];
  int32_T c2_i665;
  int32_T c2_i666;
  int32_T c2_i667;
  real_T c2_c_A[81];
  real_T c2_ajj;
  int32_T c2_c_b;
  int32_T c2_d_b;
  int32_T c2_nmj;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_jjp1;
  int32_T c2_g_a;
  int32_T c2_h_a;
  int32_T c2_coljp1;
  int32_T c2_b_jm1;
  int32_T c2_e_b;
  int32_T c2_f_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_jm1;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_b_overflow;
  int32_T c2_c_k;
  real_T c2_y;
  real_T c2_b_y;
  real_T c2_z;
  int32_T c2_n;
  real_T c2_i_a;
  int32_T c2_ix0;
  int32_T c2_b_n;
  real_T c2_j_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_k_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_l_a;
  int32_T c2_c;
  int32_T c2_i_b;
  int32_T c2_b_c;
  int32_T c2_m_a;
  int32_T c2_j_b;
  int32_T c2_i668;
  int32_T c2_n_a;
  int32_T c2_k_b;
  int32_T c2_o_a;
  int32_T c2_l_b;
  boolean_T c2_c_overflow;
  int32_T c2_d_k;
  int32_T c2_e_k;
  boolean_T exitg1;
  c2_info = 0;
  c2_colj = 1;
  c2_j = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c2_j < 10)) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_b_a = c2_a - 1;
    c2_jm1 = c2_b_a;
    c2_c_a = c2_colj;
    c2_b = c2_jm1;
    c2_d_a = c2_c_a;
    c2_b_b = c2_b;
    c2_jj = c2_d_a + c2_b_b;
    c2_i662 = 0;
    for (c2_i663 = 0; c2_i663 < 9; c2_i663++) {
      for (c2_i664 = 0; c2_i664 < 9; c2_i664++) {
        c2_b_A[c2_i664 + c2_i662] = c2_A[c2_i664 + c2_i662];
      }

      c2_i662 += 9;
    }

    c2_i665 = 0;
    for (c2_i666 = 0; c2_i666 < 9; c2_i666++) {
      for (c2_i667 = 0; c2_i667 < 9; c2_i667++) {
        c2_c_A[c2_i667 + c2_i665] = c2_A[c2_i667 + c2_i665];
      }

      c2_i665 += 9;
    }

    c2_ajj = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_jj), 1, 81, 1, 0) - 1] - c2_eml_xdotc(chartInstance, c2_jm1,
      c2_b_A, c2_colj, c2_c_A, c2_colj);
    if (c2_ajj > 0.0) {
      c2_ajj = muDoubleScalarSqrt(c2_ajj);
      c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_jj), 1, 81, 1, 0) - 1] = c2_ajj;
      if (c2_b_j < 9) {
        c2_c_b = c2_b_j;
        c2_d_b = c2_c_b;
        c2_nmj = 9 - c2_d_b;
        c2_e_a = c2_jj;
        c2_f_a = c2_e_a + 9;
        c2_jjp1 = c2_f_a;
        c2_g_a = c2_colj;
        c2_h_a = c2_g_a + 9;
        c2_coljp1 = c2_h_a;
        c2_b_jm1 = c2_jm1;
        c2_e_b = c2_b_jm1;
        c2_f_b = c2_e_b;
        if (1 > c2_f_b) {
          c2_overflow = false;
        } else {
          c2_eml_switch_helper(chartInstance);
          c2_overflow = (c2_f_b > 2147483646);
        }

        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_k = 1; c2_k <= c2_b_jm1; c2_k++) {
          c2_b_k = c2_k;
          c2_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_k), 1, 9, 1, 0) + 9 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_k), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0)
               - 1)) - 1];
        }

        c2_b_eml_xgemv(chartInstance, c2_jm1, c2_nmj, c2_coljp1, c2_colj, c2_A,
                       c2_jjp1);
        c2_c_jm1 = c2_jm1;
        c2_g_b = c2_c_jm1;
        c2_h_b = c2_g_b;
        if (1 > c2_h_b) {
          c2_b_overflow = false;
        } else {
          c2_eml_switch_helper(chartInstance);
          c2_b_overflow = (c2_h_b > 2147483646);
        }

        if (c2_b_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        }

        for (c2_c_k = 1; c2_c_k <= c2_c_jm1; c2_c_k++) {
          c2_b_k = c2_c_k;
          c2_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_k), 1, 9, 1, 0) + 9 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_k), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0)
               - 1)) - 1];
        }

        c2_y = c2_ajj;
        c2_b_y = c2_y;
        c2_z = 1.0 / c2_b_y;
        c2_n = c2_nmj;
        c2_i_a = c2_z;
        c2_ix0 = c2_jjp1;
        c2_b_n = c2_n;
        c2_j_a = c2_i_a;
        c2_b_ix0 = c2_ix0;
        c2_b_below_threshold(chartInstance);
        c2_c_n = c2_b_n;
        c2_k_a = c2_j_a;
        c2_c_ix0 = c2_b_ix0;
        c2_d_ix0 = c2_c_ix0;
        c2_l_a = c2_c_n;
        c2_c = c2_l_a;
        c2_i_b = c2_c - 1;
        c2_b_c = 9 * c2_i_b;
        c2_m_a = c2_c_ix0;
        c2_j_b = c2_b_c;
        c2_i668 = c2_m_a + c2_j_b;
        c2_n_a = c2_d_ix0;
        c2_k_b = c2_i668;
        c2_o_a = c2_n_a;
        c2_l_b = c2_k_b;
        if (c2_o_a > c2_l_b) {
          c2_c_overflow = false;
        } else {
          c2_eml_switch_helper(chartInstance);
          c2_c_overflow = (c2_l_b > 2147483638);
        }

        if (c2_c_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        }

        for (c2_d_k = c2_d_ix0; c2_d_k <= c2_i668; c2_d_k += 9) {
          c2_e_k = c2_d_k;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_k), 1, 81, 1, 0) - 1] = c2_k_a *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_k), 1, 81, 1, 0) - 1];
        }

        c2_colj = c2_coljp1;
      }

      c2_j++;
    } else {
      c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_jj), 1, 81, 1, 0) - 1] = c2_ajj;
      c2_info = c2_b_j;
      exitg1 = true;
    }
  }

  return c2_info;
}

static void c2_b_eml_xgemv(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0, real_T c2_y[81], int32_T
  c2_iy0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  int32_T c2_b_ia0;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_alpha1;
  int32_T c2_c_ia0;
  int32_T c2_c_ix0;
  real_T c2_beta1;
  int32_T c2_c_iy0;
  char_T c2_TRANSA;
  int32_T c2_var;
  ptrdiff_t c2_m_t;
  int32_T c2_b_var;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_incx_t;
  ptrdiff_t c2_incy_t;
  double * c2_alpha1_t;
  double * c2_beta1_t;
  double * c2_yiy0_t;
  double * c2_yix0_t;
  double * c2_yia0_t;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_ia0 = c2_ia0;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_below_threshold(chartInstance);
  if (c2_b_m < 1) {
  } else if (c2_b_n < 1) {
  } else {
    c2_c_m = c2_b_m;
    c2_c_n = c2_b_n;
    c2_alpha1 = -1.0;
    c2_c_ia0 = c2_b_ia0;
    c2_c_ix0 = c2_b_ix0;
    c2_beta1 = 1.0;
    c2_c_iy0 = c2_b_iy0;
    c2_TRANSA = 'T';
    c2_var = c2_c_m;
    c2_m_t = (ptrdiff_t)(c2_var);
    c2_b_var = c2_c_n;
    c2_n_t = (ptrdiff_t)(c2_b_var);
    c2_lda_t = (ptrdiff_t)(9);
    c2_incx_t = (ptrdiff_t)(1);
    c2_incy_t = (ptrdiff_t)(9);
    c2_alpha1_t = (double *)(&c2_alpha1);
    c2_beta1_t = (double *)(&c2_beta1);
    c2_yiy0_t = (double *)(&c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_c_iy0), 1, 81, 1, 0) - 1]);
    c2_yix0_t = (double *)(&c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_c_ix0), 1, 81, 1, 0) - 1]);
    c2_yia0_t = (double *)(&c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_c_ia0), 1, 81, 1, 0) - 1]);
    dgemv(&c2_TRANSA, &c2_m_t, &c2_n_t, c2_alpha1_t, c2_yia0_t, &c2_lda_t,
          c2_yix0_t, &c2_incx_t, c2_beta1_t, c2_yiy0_t, &c2_incy_t);
  }
}

static void c2_d_sqrt(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T c2_x[19])
{
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_b_x;
  real_T c2_c_x;
  for (c2_k = 0; c2_k < 19; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    if (c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c2_b_k), 1, 19, 1, 0) - 1] < 0.0) {
      c2_eml_error(chartInstance);
    }
  }

  for (c2_c_k = 0; c2_c_k < 19; c2_c_k++) {
    c2_b_k = 1.0 + (real_T)c2_c_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 19, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_c_x = muDoubleScalarSqrt(c2_c_x);
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 19, 1, 0) - 1] = c2_c_x;
  }
}

static void c2_b_eml_matlab_zgetrf(SFc2_UKF_10hzInstanceStruct *chartInstance,
  real_T c2_A[36], int32_T c2_ipiv[6], int32_T *c2_info)
{
  int32_T c2_i669;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_b_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_b_b;
  int32_T c2_mmj;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_c_b;
  int32_T c2_d_b;
  int32_T c2_jj;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_jp1j;
  int32_T c2_g_a;
  int32_T c2_h_a;
  int32_T c2_b_c;
  int32_T c2_i670;
  int32_T c2_i671;
  int32_T c2_i672;
  real_T c2_b_A[36];
  int32_T c2_i_a;
  int32_T c2_j_a;
  int32_T c2_jpiv_offset;
  int32_T c2_k_a;
  int32_T c2_e_b;
  int32_T c2_l_a;
  int32_T c2_f_b;
  int32_T c2_jpiv;
  int32_T c2_m_a;
  int32_T c2_g_b;
  int32_T c2_n_a;
  int32_T c2_h_b;
  int32_T c2_c_c;
  int32_T c2_i_b;
  int32_T c2_j_b;
  int32_T c2_jrow;
  int32_T c2_o_a;
  int32_T c2_k_b;
  int32_T c2_p_a;
  int32_T c2_l_b;
  int32_T c2_jprow;
  int32_T c2_ix0;
  int32_T c2_iy0;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_q_a;
  int32_T c2_r_a;
  int32_T c2_b_jp1j;
  int32_T c2_s_a;
  int32_T c2_t_a;
  int32_T c2_d_c;
  int32_T c2_u_a;
  int32_T c2_m_b;
  int32_T c2_v_a;
  int32_T c2_n_b;
  int32_T c2_i673;
  int32_T c2_w_a;
  int32_T c2_o_b;
  int32_T c2_x_a;
  int32_T c2_p_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_z;
  int32_T c2_q_b;
  int32_T c2_r_b;
  int32_T c2_e_c;
  int32_T c2_y_a;
  int32_T c2_ab_a;
  int32_T c2_f_c;
  int32_T c2_bb_a;
  int32_T c2_cb_a;
  int32_T c2_g_c;
  real_T c2_d10;
  c2_eps(chartInstance);
  for (c2_i669 = 0; c2_i669 < 6; c2_i669++) {
    c2_ipiv[c2_i669] = 1 + c2_i669;
  }

  *c2_info = 0;
  for (c2_j = 1; c2_j < 6; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_b_a = c2_a - 1;
    c2_jm1 = c2_b_a;
    c2_b = c2_b_j;
    c2_b_b = c2_b;
    c2_mmj = 6 - c2_b_b;
    c2_c_a = c2_jm1;
    c2_d_a = c2_c_a;
    c2_c = c2_d_a * 7;
    c2_c_b = c2_c;
    c2_d_b = c2_c_b + 1;
    c2_jj = c2_d_b;
    c2_e_a = c2_jj;
    c2_f_a = c2_e_a + 1;
    c2_jp1j = c2_f_a;
    c2_g_a = c2_mmj;
    c2_h_a = c2_g_a;
    c2_b_c = c2_h_a;
    c2_i670 = 0;
    for (c2_i671 = 0; c2_i671 < 6; c2_i671++) {
      for (c2_i672 = 0; c2_i672 < 6; c2_i672++) {
        c2_b_A[c2_i672 + c2_i670] = c2_A[c2_i672 + c2_i670];
      }

      c2_i670 += 6;
    }

    c2_i_a = c2_eml_ixamax(chartInstance, c2_b_c + 1, c2_b_A, c2_jj);
    c2_j_a = c2_i_a - 1;
    c2_jpiv_offset = c2_j_a;
    c2_k_a = c2_jj;
    c2_e_b = c2_jpiv_offset;
    c2_l_a = c2_k_a;
    c2_f_b = c2_e_b;
    c2_jpiv = c2_l_a + c2_f_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if (c2_jpiv_offset != 0) {
        c2_m_a = c2_b_j;
        c2_g_b = c2_jpiv_offset;
        c2_n_a = c2_m_a;
        c2_h_b = c2_g_b;
        c2_c_c = c2_n_a + c2_h_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 6, 1, 0) - 1] = c2_c_c;
        c2_i_b = c2_jm1;
        c2_j_b = c2_i_b + 1;
        c2_jrow = c2_j_b;
        c2_o_a = c2_jrow;
        c2_k_b = c2_jpiv_offset;
        c2_p_a = c2_o_a;
        c2_l_b = c2_k_b;
        c2_jprow = c2_p_a + c2_l_b;
        c2_ix0 = c2_jrow;
        c2_iy0 = c2_jprow;
        c2_b_ix0 = c2_ix0;
        c2_b_iy0 = c2_iy0;
        c2_b_threshold(chartInstance);
        c2_c_ix0 = c2_b_ix0;
        c2_c_iy0 = c2_b_iy0;
        c2_ix = c2_c_ix0;
        c2_iy = c2_c_iy0;
        for (c2_k = 1; c2_k < 7; c2_k++) {
          c2_temp = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 36, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 36, 1, 0) - 1] = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK
            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 36, 1, 0) -
            1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 36, 1, 0) - 1] = c2_temp;
          c2_q_a = c2_ix + 6;
          c2_ix = c2_q_a;
          c2_r_a = c2_iy + 6;
          c2_iy = c2_r_a;
        }
      }

      c2_b_jp1j = c2_jp1j;
      c2_s_a = c2_mmj;
      c2_t_a = c2_s_a;
      c2_d_c = c2_t_a;
      c2_u_a = c2_jp1j;
      c2_m_b = c2_d_c - 1;
      c2_v_a = c2_u_a;
      c2_n_b = c2_m_b;
      c2_i673 = c2_v_a + c2_n_b;
      c2_w_a = c2_b_jp1j;
      c2_o_b = c2_i673;
      c2_x_a = c2_w_a;
      c2_p_b = c2_o_b;
      if (c2_x_a > c2_p_b) {
        c2_overflow = false;
      } else {
        c2_eml_switch_helper(chartInstance);
        c2_overflow = (c2_p_b > 2147483646);
      }

      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_i = c2_b_jp1j; c2_i <= c2_i673; c2_i++) {
        c2_b_i = c2_i;
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_b_i), 1, 36, 1, 0) - 1];
        c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_jj), 1, 36, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_b_y = c2_y;
        c2_z = c2_b_x / c2_b_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 36, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_q_b = c2_b_j;
    c2_r_b = c2_q_b;
    c2_e_c = 6 - c2_r_b;
    c2_y_a = c2_jj;
    c2_ab_a = c2_y_a;
    c2_f_c = c2_ab_a;
    c2_bb_a = c2_jj;
    c2_cb_a = c2_bb_a;
    c2_g_c = c2_cb_a;
    c2_d10 = -1.0;
    c2_b_eml_xgeru(chartInstance, c2_mmj, c2_e_c, c2_d10, c2_jp1j, c2_f_c + 6,
                   c2_A, c2_g_c + 7);
  }

  if (*c2_info == 0) {
    if (!(c2_A[35] != 0.0)) {
      *c2_info = 6;
    }
  }
}

static void c2_b_eml_xgeru(SFc2_UKF_10hzInstanceStruct *chartInstance, int32_T
  c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0, real_T
  c2_A[36], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_c_ia0;
  int32_T c2_d_m;
  int32_T c2_d_n;
  real_T c2_d_alpha1;
  int32_T c2_d_ix0;
  int32_T c2_d_iy0;
  int32_T c2_d_ia0;
  int32_T c2_e_m;
  int32_T c2_e_n;
  real_T c2_e_alpha1;
  int32_T c2_e_ix0;
  int32_T c2_e_iy0;
  int32_T c2_e_ia0;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_f_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_c_b;
  int32_T c2_i674;
  int32_T c2_b_a;
  int32_T c2_d_b;
  int32_T c2_i675;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_d_a;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_ijA;
  int32_T c2_b_ijA;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_g_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_b_ia0 = c2_ia0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_alpha1 = c2_b_alpha1;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_c_ia0 = c2_b_ia0;
  c2_d_m = c2_c_m;
  c2_d_n = c2_c_n;
  c2_d_alpha1 = c2_c_alpha1;
  c2_d_ix0 = c2_c_ix0;
  c2_d_iy0 = c2_c_iy0;
  c2_d_ia0 = c2_c_ia0;
  c2_e_m = c2_d_m;
  c2_e_n = c2_d_n;
  c2_e_alpha1 = c2_d_alpha1;
  c2_e_ix0 = c2_d_ix0;
  c2_e_iy0 = c2_d_iy0;
  c2_e_ia0 = c2_d_ia0;
  if (c2_e_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_e_ix0;
    c2_a = c2_e_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = c2_e_iy0;
    c2_f_n = c2_e_n;
    c2_b = c2_f_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_f_n; c2_j++) {
      c2_yjy = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 36, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_e_alpha1;
        c2_ix = c2_ixstart;
        c2_c_b = c2_jA + 1;
        c2_i674 = c2_c_b;
        c2_b_a = c2_e_m;
        c2_d_b = c2_jA;
        c2_i675 = c2_b_a + c2_d_b;
        c2_c_a = c2_i674;
        c2_e_b = c2_i675;
        c2_d_a = c2_c_a;
        c2_f_b = c2_e_b;
        if (c2_d_a > c2_f_b) {
          c2_b_overflow = false;
        } else {
          c2_eml_switch_helper(chartInstance);
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        if (c2_b_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        }

        for (c2_ijA = c2_i674; c2_ijA <= c2_i675; c2_ijA++) {
          c2_b_ijA = c2_ijA;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 36, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 36, 1, 0) - 1] +
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 36, 1, 0) - 1] * c2_temp;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
        }
      }

      c2_f_a = c2_jy + 6;
      c2_jy = c2_f_a;
      c2_g_a = c2_jA + 6;
      c2_jA = c2_g_a;
    }
  }
}

static void c2_c_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_jBcol;
  int32_T c2_jAcol;
  int32_T c2_i676;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_kBcol;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_y;
  real_T c2_b_y;
  real_T c2_c_y;
  real_T c2_temp;
  int32_T c2_c_i;
  c2_c_threshold(chartInstance);
  c2_scalarEg(chartInstance);
  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_jBcol = 9 * c2_b_j;
    c2_jAcol = 6 * c2_b_j;
    c2_i676 = c2_b_j;
    c2_b = c2_i676;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_k = 1; c2_k <= c2_i676; c2_k++) {
      c2_b_k = c2_k;
      c2_kBcol = 9 * (c2_b_k - 1);
      if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_k + c2_jAcol)), 1, 36, 1, 0) - 1] != 0.0) {
        for (c2_i = 1; c2_i < 10; c2_i++) {
          c2_b_i = c2_i;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1] -
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_k + c2_jAcol)), 1, 36, 1, 0) - 1] *
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_kBcol)), 1, 54, 1, 0) - 1];
        }
      }
    }

    c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)((c2_b_j + c2_jAcol) + 1)), 1, 36, 1, 0) - 1];
    c2_b_y = c2_y;
    c2_c_y = c2_b_y;
    c2_temp = 1.0 / c2_c_y;
    for (c2_c_i = 1; c2_c_i < 10; c2_c_i++) {
      c2_b_i = c2_c_i;
      c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1] = c2_temp *
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1];
    }
  }
}

static void c2_d_eml_xtrsm(SFc2_UKF_10hzInstanceStruct *chartInstance, real_T
  c2_A[36], real_T c2_B[54])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_jBcol;
  int32_T c2_jAcol;
  int32_T c2_i677;
  int32_T c2_a;
  int32_T c2_b_a;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_kBcol;
  int32_T c2_i;
  int32_T c2_b_i;
  c2_c_threshold(chartInstance);
  c2_scalarEg(chartInstance);
  for (c2_j = 6; c2_j > 0; c2_j--) {
    c2_b_j = c2_j - 1;
    c2_jBcol = 9 * c2_b_j;
    c2_jAcol = 6 * c2_b_j;
    c2_i677 = c2_b_j + 2;
    c2_a = c2_i677;
    c2_b_a = c2_a;
    if (c2_b_a > 6) {
      c2_overflow = false;
    } else {
      c2_eml_switch_helper(chartInstance);
      c2_overflow = false;
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_k = c2_i677; c2_k < 7; c2_k++) {
      c2_b_k = c2_k;
      c2_kBcol = 9 * (c2_b_k - 1);
      if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_k + c2_jAcol)), 1, 36, 1, 0) - 1] != 0.0) {
        for (c2_i = 1; c2_i < 10; c2_i++) {
          c2_b_i = c2_i;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_jBcol)), 1, 54, 1, 0) - 1] -
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_k + c2_jAcol)), 1, 36, 1, 0) - 1] *
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)(c2_b_i + c2_kBcol)), 1, 54, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc2_UKF_10hzInstanceStruct *chartInstance)
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

void sf_c2_UKF_10hz_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(814773437U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2785596431U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1061583140U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(134324565U);
}

mxArray *sf_c2_UKF_10hz_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("YPKCb8qgfXarf3Fu5LGaIH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      pr[0] = (double)(3);
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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      pr[0] = (double)(9);
      pr[1] = (double)(6);
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
      pr[0] = (double)(6);
      pr[1] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_UKF_10hz_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_UKF_10hz_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_UKF_10hz(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"Attitude_sensor\",},{M[1],M[15],T\"Kalman_Gain\",},{M[1],M[17],T\"noise_covarience\",},{M[1],M[9],T\"w_bias_sensor\",},{M[1],M[10],T\"w_sensor\",},{M[4],M[0],T\"Beta\",S'l','i','p'{{M1x2[831 835],M[0],}}},{M[4],M[0],T\"I_c\",S'l','i','p'{{M1x2[842 845],M[0],}}},{M[4],M[0],T\"K\",S'l','i','p'{{M1x2[829 830],M[0],}}},{M[4],M[0],T\"Lambda\",S'l','i','p'{{M1x2[816 822],M[0],}}},{M[4],M[0],T\"P_km1km1\",S'l','i','p'{{M1x2[785 793],M[0],}}}}",
    "100 S1x7'type','srcId','name','auxInfo'{{M[4],M[0],T\"Q_k\",S'l','i','p'{{M1x2[812 815],M[0],}}},{M[4],M[0],T\"R_k\",S'l','i','p'{{M1x2[808 811],M[0],}}},{M[4],M[0],T\"alpha\",S'l','i','p'{{M1x2[823 828],M[0],}}},{M[4],M[0],T\"q_s_c\",S'l','i','p'{{M1x2[836 841],M[0],}}},{M[4],M[0],T\"x_kk\",S'l','i','p'{{M1x2[803 807],M[0],}}},{M[4],M[0],T\"x_km1km1\",S'l','i','p'{{M1x2[794 802],M[0],}}},{M[8],M[0],T\"is_active_c2_UKF_10hz\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 17, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_UKF_10hz_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_UKF_10hzInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_UKF_10hzInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _UKF_10hzMachineNumber_,
           2,
           1,
           1,
           0,
           10,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"B_ECI");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Attitude_sensor");
          _SFD_SET_DATA_PROPS(2,2,0,1,"w_sensor");
          _SFD_SET_DATA_PROPS(3,1,1,0,"B_sat");
          _SFD_SET_DATA_PROPS(4,1,1,0,"w_gyro");
          _SFD_SET_DATA_PROPS(5,2,0,1,"w_bias_sensor");
          _SFD_SET_DATA_PROPS(6,1,1,0,"Torque_s");
          _SFD_SET_DATA_PROPS(7,1,1,0,"Ts");
          _SFD_SET_DATA_PROPS(8,2,0,1,"Kalman_Gain");
          _SFD_SET_DATA_PROPS(9,2,0,1,"noise_covarience");
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
        _SFD_CV_INIT_EML(0,1,9,3,0,0,0,4,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,6183);
        _SFD_CV_INIT_EML_FCN(0,1,"RK4",6185,-1,7120);
        _SFD_CV_INIT_EML_FCN(0,2,"SensorModel",7122,-1,7736);
        _SFD_CV_INIT_EML_FCN(0,3,"Kinematics",7738,-1,7982);
        _SFD_CV_INIT_EML_FCN(0,4,"skew_matrix",7984,-1,8144);
        _SFD_CV_INIT_EML_FCN(0,5,"quatmultiply",8146,-1,8739);
        _SFD_CV_INIT_EML_FCN(0,6,"quatinv",8741,-1,8839);
        _SFD_CV_INIT_EML_FCN(0,7,"RotateVecSensor2Cont",8841,-1,8993);
        _SFD_CV_INIT_EML_FCN(0,8,"RotateVecCont2Sensor",8995,-1,9147);
        _SFD_CV_INIT_EML_IF(0,1,0,895,915,1757,5838);
        _SFD_CV_INIT_EML_IF(0,1,1,3540,3549,3666,3790);
        _SFD_CV_INIT_EML_IF(0,1,2,4449,4458,4724,4998);
        _SFD_CV_INIT_EML_FOR(0,1,0,3516,3532,3798);
        _SFD_CV_INIT_EML_FOR(0,1,1,4425,4441,5006);
        _SFD_CV_INIT_EML_FOR(0,1,2,6586,6615,7103);
        _SFD_CV_INIT_EML_FOR(0,1,3,7422,7449,7732);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_o_sf_marshallOut,(MexInFcnForType)
            c2_o_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)
            c2_n_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)
            c2_n_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_p_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 9;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)
            c2_m_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_l_sf_marshallOut,(MexInFcnForType)
            c2_l_sf_marshallIn);
        }

        {
          real_T *c2_Ts;
          real_T (*c2_B_ECI)[3];
          real_T (*c2_Attitude_sensor)[4];
          real_T (*c2_w_sensor)[3];
          real_T (*c2_B_sat)[3];
          real_T (*c2_w_gyro)[3];
          real_T (*c2_w_bias_sensor)[3];
          real_T (*c2_Torque_s)[3];
          real_T (*c2_Kalman_Gain)[54];
          real_T (*c2_noise_covarience)[36];
          c2_noise_covarience = (real_T (*)[36])ssGetOutputPortSignal
            (chartInstance->S, 5);
          c2_Kalman_Gain = (real_T (*)[54])ssGetOutputPortSignal
            (chartInstance->S, 4);
          c2_Ts = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_Torque_s = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal
            (chartInstance->S, 3);
          c2_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c2_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c2_B_ECI = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_B_ECI);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Attitude_sensor);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_w_sensor);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_B_sat);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_w_gyro);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_w_bias_sensor);
          _SFD_SET_DATA_VALUE_PTR(6U, *c2_Torque_s);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_Ts);
          _SFD_SET_DATA_VALUE_PTR(8U, *c2_Kalman_Gain);
          _SFD_SET_DATA_VALUE_PTR(9U, *c2_noise_covarience);
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
  return "Vg1EQ99RHiPkvsFP0b3jkC";
}

static void sf_opaque_initialize_c2_UKF_10hz(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
  initialize_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_UKF_10hz(void *chartInstanceVar)
{
  enable_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_UKF_10hz(void *chartInstanceVar)
{
  disable_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_UKF_10hz(void *chartInstanceVar)
{
  sf_gateway_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_UKF_10hz(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_UKF_10hz();/* state var info */
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

extern void sf_internal_set_sim_state_c2_UKF_10hz(SimStruct* S, const mxArray
  *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c2_UKF_10hz();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_UKF_10hz(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_UKF_10hz(S);
}

static void sf_opaque_set_sim_state_c2_UKF_10hz(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_UKF_10hz(S, st);
}

static void sf_opaque_terminate_c2_UKF_10hz(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_UKF_10hz_optimization_info();
    }

    finalize_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_UKF_10hz(SimStruct *S)
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
    initialize_params_c2_UKF_10hz((SFc2_UKF_10hzInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_UKF_10hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_UKF_10hz_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=5; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3786094931U));
  ssSetChecksum1(S,(2677680116U));
  ssSetChecksum2(S,(3615056805U));
  ssSetChecksum3(S,(3459374142U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_UKF_10hz(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_UKF_10hz(SimStruct *S)
{
  SFc2_UKF_10hzInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_UKF_10hzInstanceStruct *)utMalloc(sizeof
    (SFc2_UKF_10hzInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_UKF_10hzInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_UKF_10hz;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_UKF_10hz;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_UKF_10hz;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_UKF_10hz;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_UKF_10hz;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_UKF_10hz;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_UKF_10hz;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_UKF_10hz;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_UKF_10hz;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_UKF_10hz;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_UKF_10hz;
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

void c2_UKF_10hz_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_UKF_10hz(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_UKF_10hz(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_UKF_10hz(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_UKF_10hz_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
