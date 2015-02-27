/* Include files */

#include "blascompat32.h"
#include "SatelliteControlSystem_sfun.h"
#include "c2_SatelliteControlSystem.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SatelliteControlSystem_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[46] = { "P_kk", "L", "w_bias",
  "DX_sigma", "dX_km1km1", "X_km1km1_temp", "X_km1km1", "X_kkm1", "W0_m", "W0_c",
  "Wi_cm", "x_kkm1", "dX_kkm1_temp", "dX_kkm1", "dx_kkm1", "P_kkm1",
  "Z_kkm1_control", "z_kkm1_control", "P_zkzk", "P_xkzk", "K_k", "B_control",
  "w_control", "dx_kk", "w_sensor_temp", "w_bias_sensor_temp", "nargin",
  "nargout", "B_ref", "B_sat", "w_gyro", "Attitude_sensor", "w_sensor",
  "w_bias_sensor", "P_km1km1", "x_km1km1", "q_s_c", "x_kk", "DCM_s2c", "Ts",
  "R_k", "Q_k", "Lambda", "alpha", "K", "Beta" };

static const char * c2_b_debug_family_names[4] = { "nargin", "nargout", "qin",
  "dcm" };

static const char * c2_c_debug_family_names[7] = { "vec", "scalar", "nargin",
  "nargout", "q", "r", "qout" };

static const char * c2_d_debug_family_names[13] = { "DCM", "q", "w_cont", "W",
  "k1", "k2", "k3", "k4", "nargin", "nargout", "X_km1km1", "Ts", "X_kkm1" };

static const char * c2_e_debug_family_names[5] = { "q_conj", "nargin", "nargout",
  "qin", "qinv" };

static const char * c2_f_debug_family_names[7] = { "vec", "scalar", "nargin",
  "nargout", "q", "r", "qout" };

static const char * c2_g_debug_family_names[7] = { "DCM_i2c", "Z_kkm1_temp",
  "nargin", "nargout", "X_kkm1", "B_ref", "Z_kkm1" };

/* Function Declarations */
static void initialize_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void initialize_params_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void enable_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void disable_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void set_sim_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance, const mxArray
   *c2_st);
static void finalize_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void sf_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void c2_chartstep_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void initSimStructsc2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Beta, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_K, const char_T *c2_identifier);
static real_T c2_d_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_e_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_alpha, const char_T *c2_identifier);
static real_T c2_f_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_g_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Lambda, const char_T *c2_identifier);
static real_T c2_h_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Q_k, const char_T *c2_identifier, real_T
  c2_y[81]);
static void c2_j_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_k_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_R_k, const char_T *c2_identifier, real_T
  c2_y[36]);
static void c2_l_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_m_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Ts, const char_T *c2_identifier);
static real_T c2_n_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_o_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_DCM_s2c, const char_T *c2_identifier,
  real_T c2_y[9]);
static void c2_p_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_q_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_x_kk, const char_T *c2_identifier, real_T
  c2_y[10]);
static void c2_r_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_s_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_q_s_c, const char_T *c2_identifier, real_T
  c2_y[4]);
static void c2_t_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_u_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_x_km1km1, const char_T *c2_identifier,
  real_T c2_y[10]);
static void c2_v_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_w_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_P_km1km1, const char_T *c2_identifier,
  real_T c2_y[81]);
static void c2_x_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_y_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_w_bias_sensor, const char_T *c2_identifier,
  real_T c2_y[3]);
static void c2_ab_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_bb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_Attitude_sensor, const char_T *c2_identifier,
  real_T c2_y[4]);
static void c2_cb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_db_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_eb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_fb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[54]);
static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_gb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36]);
static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_hb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6]);
static void c2_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_t_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_ib_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[114]);
static void c2_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_u_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_jb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81]);
static void c2_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_v_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_kb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[171]);
static void c2_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_w_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_lb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[76]);
static void c2_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_x_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_mb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10]);
static void c2_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_y_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_nb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[190]);
static void c2_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_ob_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_pb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_qb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[76]);
static void c2_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_db_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_rb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[19]);
static void c2_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_sb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[57]);
static void c2_eb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_tb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_fb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_gb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_ub_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_gb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[214]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[214]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[214]);
static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[214]);
static void c2_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_y[3]);
static real_T c2_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[3]);
static void c2_check_forloop_overflow_error
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x);
static void c2_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_rdivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[3], real_T c2_y, real_T c2_z[3]);
static void c2_quat2dcm(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_qin[4], real_T c2_dcm[9]);
static void c2_b_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[4], real_T c2_y[4]);
static real_T c2_b_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[4]);
static real_T c2_c_power(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_a);
static void c2_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_mldivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_A[9], real_T c2_B[3], real_T c2_Y[3]);
static void c2_eml_warning(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static real_T c2_mpower(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_chol(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                    real_T c2_A[81], real_T c2_b_A[81]);
static void c2_b_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_eml_matlab_zpotrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[81], real_T c2_b_A[81], int32_T *c2_info);
static void c2_b_check_forloop_overflow_error
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance, boolean_T
   c2_overflow);
static void c2_eml_xgemv(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0,
  real_T c2_y[81], int32_T c2_iy0, real_T c2_b_y[81]);
static void c2_c_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_d_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[57], real_T c2_y[57]);
static void c2_c_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[57], real_T c2_y[19]);
static void c2_b_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x[19], real_T c2_b_x[19]);
static void c2_quatmultiply(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_q[76], real_T c2_r[4], real_T c2_qout[76]);
static void c2_RK4(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                   real_T c2_X_km1km1[190], real_T c2_b_Ts, real_T c2_X_kkm1[190]);
static void c2_c_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_b_rdivide(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_x[4], real_T c2_y, real_T c2_z[4]);
static void c2_d_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[190], real_T c2_y[10]);
static void c2_e_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[4], real_T c2_y[4]);
static real_T c2_e_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[4]);
static void c2_quatinv(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_qin[4], real_T c2_qinv[4]);
static void c2_f_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[171], real_T c2_y[9]);
static void c2_SensorModel(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_X_kkm1[190], real_T c2_B_ref[3], real_T c2_Z_kkm1
  [114]);
static void c2_b_quatmultiply(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_q[4], real_T c2_r[4], real_T c2_qout[4]);
static void c2_g_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[114], real_T c2_y[6]);
static void c2_mrdivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_A[54], real_T c2_B[36], real_T c2_y[54]);
static void c2_realmin(SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void c2_eps(SFc2_SatelliteControlSystemInstanceStruct *chartInstance);
static void c2_eml_matlab_zgetrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_b_A[36], int32_T c2_ipiv[6],
  int32_T *c2_info);
static void c2_eml_xger(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[36], int32_T c2_ia0, real_T c2_b_A[36]);
static void c2_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54], real_T c2_b_B[54]);
static void c2_below_threshold(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_d_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_b_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54], real_T c2_b_B[54]);
static void c2_e_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_f_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static void c2_g_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);
static const mxArray *c2_hb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_vb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_hb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_wb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_b_is_active_c2_SatelliteControlSystem, const
  char_T *c2_identifier);
static uint8_T c2_xb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T *c2_x);
static void c2_b_chol(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_A[81]);
static int32_T c2_b_eml_matlab_zpotrf(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, real_T c2_A[81]);
static void c2_b_eml_xgemv(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0,
  real_T c2_y[81], int32_T c2_iy0);
static void c2_d_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x[19]);
static void c2_b_eml_matlab_zgetrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], int32_T c2_ipiv[6], int32_T *c2_info);
static void c2_b_eml_xger(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[36], int32_T c2_ia0);
static void c2_c_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54]);
static void c2_d_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54]);
static void init_dsm_address_info(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_P_km1km1_not_empty = FALSE;
  chartInstance->c2_x_km1km1_not_empty = FALSE;
  chartInstance->c2_q_s_c_not_empty = FALSE;
  chartInstance->c2_x_kk_not_empty = FALSE;
  chartInstance->c2_DCM_s2c_not_empty = FALSE;
  chartInstance->c2_Ts_not_empty = FALSE;
  chartInstance->c2_R_k_not_empty = FALSE;
  chartInstance->c2_Q_k_not_empty = FALSE;
  chartInstance->c2_Lambda_not_empty = FALSE;
  chartInstance->c2_alpha_not_empty = FALSE;
  chartInstance->c2_K_not_empty = FALSE;
  chartInstance->c2_Beta_not_empty = FALSE;
  chartInstance->c2_is_active_c2_SatelliteControlSystem = 0U;
}

static void initialize_params_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static void enable_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[4];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i1;
  real_T c2_b_u[3];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i2;
  real_T c2_c_u[3];
  const mxArray *c2_d_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_i3;
  real_T c2_e_u[9];
  const mxArray *c2_f_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  int32_T c2_i4;
  real_T c2_h_u[81];
  const mxArray *c2_i_y = NULL;
  int32_T c2_i5;
  real_T c2_i_u[81];
  const mxArray *c2_j_y = NULL;
  int32_T c2_i6;
  real_T c2_j_u[36];
  const mxArray *c2_k_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  real_T c2_e_hoistedGlobal;
  real_T c2_l_u;
  const mxArray *c2_m_y = NULL;
  int32_T c2_i7;
  real_T c2_m_u[4];
  const mxArray *c2_n_y = NULL;
  int32_T c2_i8;
  real_T c2_n_u[10];
  const mxArray *c2_o_y = NULL;
  int32_T c2_i9;
  real_T c2_o_u[10];
  const mxArray *c2_p_y = NULL;
  uint8_T c2_f_hoistedGlobal;
  uint8_T c2_p_u;
  const mxArray *c2_q_y = NULL;
  real_T (*c2_w_sensor)[3];
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_Attitude_sensor)[4];
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(16), FALSE);
  for (c2_i0 = 0; c2_i0 < 4; c2_i0++) {
    c2_u[c2_i0] = (*c2_Attitude_sensor)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i1 = 0; c2_i1 < 3; c2_i1++) {
    c2_b_u[c2_i1] = (*c2_w_bias_sensor)[c2_i1];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i2 = 0; c2_i2 < 3; c2_i2++) {
    c2_c_u[c2_i2] = (*c2_w_sensor)[c2_i2];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_hoistedGlobal = chartInstance->c2_Beta;
  c2_d_u = c2_hoistedGlobal;
  c2_e_y = NULL;
  if (!chartInstance->c2_Beta_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i3 = 0; c2_i3 < 9; c2_i3++) {
    c2_e_u[c2_i3] = chartInstance->c2_DCM_s2c[c2_i3];
  }

  c2_f_y = NULL;
  if (!chartInstance->c2_DCM_s2c_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_b_hoistedGlobal = chartInstance->c2_K;
  c2_f_u = c2_b_hoistedGlobal;
  c2_g_y = NULL;
  if (!chartInstance->c2_K_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_c_hoistedGlobal = chartInstance->c2_Lambda;
  c2_g_u = c2_c_hoistedGlobal;
  c2_h_y = NULL;
  if (!chartInstance->c2_Lambda_not_empty) {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 6, c2_h_y);
  for (c2_i4 = 0; c2_i4 < 81; c2_i4++) {
    c2_h_u[c2_i4] = chartInstance->c2_P_km1km1[c2_i4];
  }

  c2_i_y = NULL;
  if (!chartInstance->c2_P_km1km1_not_empty) {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", c2_h_u, 0, 0U, 1U, 0U, 2, 9, 9),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 7, c2_i_y);
  for (c2_i5 = 0; c2_i5 < 81; c2_i5++) {
    c2_i_u[c2_i5] = chartInstance->c2_Q_k[c2_i5];
  }

  c2_j_y = NULL;
  if (!chartInstance->c2_Q_k_not_empty) {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_j_y, sf_mex_create("y", c2_i_u, 0, 0U, 1U, 0U, 2, 9, 9),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 8, c2_j_y);
  for (c2_i6 = 0; c2_i6 < 36; c2_i6++) {
    c2_j_u[c2_i6] = chartInstance->c2_R_k[c2_i6];
  }

  c2_k_y = NULL;
  if (!chartInstance->c2_R_k_not_empty) {
    sf_mex_assign(&c2_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_k_y, sf_mex_create("y", c2_j_u, 0, 0U, 1U, 0U, 2, 6, 6),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 9, c2_k_y);
  c2_d_hoistedGlobal = chartInstance->c2_Ts;
  c2_k_u = c2_d_hoistedGlobal;
  c2_l_y = NULL;
  if (!chartInstance->c2_Ts_not_empty) {
    sf_mex_assign(&c2_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 10, c2_l_y);
  c2_e_hoistedGlobal = chartInstance->c2_alpha;
  c2_l_u = c2_e_hoistedGlobal;
  c2_m_y = NULL;
  if (!chartInstance->c2_alpha_not_empty) {
    sf_mex_assign(&c2_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_l_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 11, c2_m_y);
  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    c2_m_u[c2_i7] = chartInstance->c2_q_s_c[c2_i7];
  }

  c2_n_y = NULL;
  if (!chartInstance->c2_q_s_c_not_empty) {
    sf_mex_assign(&c2_n_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_n_y, sf_mex_create("y", c2_m_u, 0, 0U, 1U, 0U, 1, 4),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 12, c2_n_y);
  for (c2_i8 = 0; c2_i8 < 10; c2_i8++) {
    c2_n_u[c2_i8] = chartInstance->c2_x_kk[c2_i8];
  }

  c2_o_y = NULL;
  if (!chartInstance->c2_x_kk_not_empty) {
    sf_mex_assign(&c2_o_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_o_y, sf_mex_create("y", c2_n_u, 0, 0U, 1U, 0U, 1, 10),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 13, c2_o_y);
  for (c2_i9 = 0; c2_i9 < 10; c2_i9++) {
    c2_o_u[c2_i9] = chartInstance->c2_x_km1km1[c2_i9];
  }

  c2_p_y = NULL;
  if (!chartInstance->c2_x_km1km1_not_empty) {
    sf_mex_assign(&c2_p_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_p_y, sf_mex_create("y", c2_o_u, 0, 0U, 1U, 0U, 1, 10),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 14, c2_p_y);
  c2_f_hoistedGlobal = chartInstance->c2_is_active_c2_SatelliteControlSystem;
  c2_p_u = c2_f_hoistedGlobal;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_p_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 15, c2_q_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance, const mxArray
   *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[4];
  int32_T c2_i10;
  real_T c2_dv1[3];
  int32_T c2_i11;
  real_T c2_dv2[3];
  int32_T c2_i12;
  real_T c2_dv3[9];
  int32_T c2_i13;
  real_T c2_dv4[81];
  int32_T c2_i14;
  real_T c2_dv5[81];
  int32_T c2_i15;
  real_T c2_dv6[36];
  int32_T c2_i16;
  real_T c2_dv7[4];
  int32_T c2_i17;
  real_T c2_dv8[10];
  int32_T c2_i18;
  real_T c2_dv9[10];
  int32_T c2_i19;
  real_T (*c2_Attitude_sensor)[4];
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_w_sensor)[3];
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_bb_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
    "Attitude_sensor", c2_dv0);
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    (*c2_Attitude_sensor)[c2_i10] = c2_dv0[c2_i10];
  }

  c2_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "w_bias_sensor", c2_dv1);
  for (c2_i11 = 0; c2_i11 < 3; c2_i11++) {
    (*c2_w_bias_sensor)[c2_i11] = c2_dv1[c2_i11];
  }

  c2_y_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
                        "w_sensor", c2_dv2);
  for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
    (*c2_w_sensor)[c2_i12] = c2_dv2[c2_i12];
  }

  chartInstance->c2_Beta = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 3)), "Beta");
  c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
                        "DCM_s2c", c2_dv3);
  for (c2_i13 = 0; c2_i13 < 9; c2_i13++) {
    chartInstance->c2_DCM_s2c[c2_i13] = c2_dv3[c2_i13];
  }

  chartInstance->c2_K = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 5)), "K");
  chartInstance->c2_Lambda = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 6)), "Lambda");
  c2_w_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 7)),
                        "P_km1km1", c2_dv4);
  for (c2_i14 = 0; c2_i14 < 81; c2_i14++) {
    chartInstance->c2_P_km1km1[c2_i14] = c2_dv4[c2_i14];
  }

  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 8)),
                        "Q_k", c2_dv5);
  for (c2_i15 = 0; c2_i15 < 81; c2_i15++) {
    chartInstance->c2_Q_k[c2_i15] = c2_dv5[c2_i15];
  }

  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)),
                        "R_k", c2_dv6);
  for (c2_i16 = 0; c2_i16 < 36; c2_i16++) {
    chartInstance->c2_R_k[c2_i16] = c2_dv6[c2_i16];
  }

  chartInstance->c2_Ts = c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 10)), "Ts");
  chartInstance->c2_alpha = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 11)), "alpha");
  c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 12)),
                        "q_s_c", c2_dv7);
  for (c2_i17 = 0; c2_i17 < 4; c2_i17++) {
    chartInstance->c2_q_s_c[c2_i17] = c2_dv7[c2_i17];
  }

  c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 13)),
                        "x_kk", c2_dv8);
  for (c2_i18 = 0; c2_i18 < 10; c2_i18++) {
    chartInstance->c2_x_kk[c2_i18] = c2_dv8[c2_i18];
  }

  c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 14)),
                        "x_km1km1", c2_dv9);
  for (c2_i19 = 0; c2_i19 < 10; c2_i19++) {
    chartInstance->c2_x_km1km1[c2_i19] = c2_dv9[c2_i19];
  }

  chartInstance->c2_is_active_c2_SatelliteControlSystem = c2_wb_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 15)),
     "is_active_c2_SatelliteControlSystem");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_SatelliteControlSystem(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static void sf_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T (*c2_w_bias_sensor)[3];
  real_T (*c2_w_gyro)[3];
  real_T (*c2_B_sat)[3];
  real_T (*c2_w_sensor)[3];
  real_T (*c2_Attitude_sensor)[4];
  real_T (*c2_B_ref)[3];
  c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_B_ref = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i20 = 0; c2_i20 < 3; c2_i20++) {
    _SFD_DATA_RANGE_CHECK((*c2_B_ref)[c2_i20], 0U);
  }

  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    _SFD_DATA_RANGE_CHECK((*c2_Attitude_sensor)[c2_i21], 1U);
  }

  for (c2_i22 = 0; c2_i22 < 3; c2_i22++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_sensor)[c2_i22], 2U);
  }

  for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
    _SFD_DATA_RANGE_CHECK((*c2_B_sat)[c2_i23], 3U);
  }

  for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_gyro)[c2_i24], 4U);
  }

  for (c2_i25 = 0; c2_i25 < 3; c2_i25++) {
    _SFD_DATA_RANGE_CHECK((*c2_w_bias_sensor)[c2_i25], 5U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_SatelliteControlSystem(chartInstance);
  sf_debug_check_for_state_inconsistency(_SatelliteControlSystemMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
  int32_T c2_i26;
  real_T c2_B_ref[3];
  int32_T c2_i27;
  real_T c2_B_sat[3];
  int32_T c2_i28;
  real_T c2_w_gyro[3];
  uint32_T c2_debug_family_var_map[46];
  real_T c2_P_kk[81];
  real_T c2_L;
  real_T c2_w_bias[3];
  real_T c2_DX_sigma[81];
  real_T c2_dX_km1km1[171];
  real_T c2_X_km1km1_temp[76];
  real_T c2_X_km1km1[190];
  real_T c2_X_kkm1[190];
  real_T c2_W0_m;
  real_T c2_W0_c;
  real_T c2_Wi_cm;
  real_T c2_x_kkm1[10];
  real_T c2_dX_kkm1_temp[76];
  real_T c2_dX_kkm1[171];
  real_T c2_dx_kkm1[9];
  real_T c2_P_kkm1[81];
  real_T c2_Z_kkm1_control[114];
  real_T c2_z_kkm1_control[6];
  real_T c2_P_zkzk[36];
  real_T c2_P_xkzk[54];
  real_T c2_K_k[54];
  real_T c2_B_control[4];
  real_T c2_w_control[4];
  real_T c2_dx_kk[9];
  real_T c2_w_sensor_temp[4];
  real_T c2_w_bias_sensor_temp[4];
  real_T c2_b_B_control[3];
  real_T c2_b_w_control[3];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 3.0;
  real_T c2_Attitude_sensor[4];
  real_T c2_w_sensor[3];
  real_T c2_w_bias_sensor[3];
  int32_T c2_i29;
  real_T c2_b_B_sat[3];
  real_T c2_a[3];
  int32_T c2_i30;
  real_T c2_b_a[3];
  real_T c2_d0;
  int32_T c2_i31;
  real_T c2_c_B_sat[3];
  real_T c2_dv10[3];
  int32_T c2_i32;
  int32_T c2_i33;
  real_T c2_b_B_ref[3];
  int32_T c2_i34;
  real_T c2_c_a[3];
  real_T c2_d1;
  int32_T c2_i35;
  real_T c2_c_B_ref[3];
  real_T c2_dv11[3];
  int32_T c2_i36;
  int32_T c2_i37;
  static real_T c2_dv12[81] = { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

  int32_T c2_i38;
  int32_T c2_i39;
  static real_T c2_dv13[4] = { 1.0, 0.0, 0.0, 0.0 };

  int32_T c2_i40;
  int32_T c2_i41;
  real_T c2_dv14[4];
  real_T c2_dv15[9];
  int32_T c2_i42;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  real_T c2_dv16[9];
  int32_T c2_i46;
  real_T c2_b_w_gyro[3];
  real_T c2_dv17[3];
  int32_T c2_i47;
  int32_T c2_i48;
  int32_T c2_i49;
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_d_a;
  real_T c2_b;
  real_T c2_y;
  int32_T c2_i50;
  static real_T c2_dv18[36] = { 1.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5 };

  int32_T c2_i51;
  static real_T c2_dv19[81] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8 };

  int32_T c2_i52;
  int32_T c2_i53;
  int32_T c2_i54;
  real_T c2_c_hoistedGlobal;
  int32_T c2_i55;
  real_T c2_d_hoistedGlobal[81];
  real_T c2_e_a;
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  real_T c2_b_b[9];
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  real_T c2_b_dX_km1km1[57];
  real_T c2_b_y[57];
  int32_T c2_i71;
  real_T c2_c_y[57];
  real_T c2_dv20[19];
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  real_T c2_e_hoistedGlobal[10];
  int32_T c2_i80;
  real_T c2_f_hoistedGlobal[10];
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_i89;
  real_T c2_d_y[57];
  int32_T c2_i90;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  real_T c2_b_X_km1km1_temp[76];
  int32_T c2_i94;
  real_T c2_g_hoistedGlobal[4];
  real_T c2_dv21[76];
  int32_T c2_i95;
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  real_T c2_b_X_km1km1[190];
  real_T c2_dv22[190];
  int32_T c2_i110;
  real_T c2_h_hoistedGlobal;
  real_T c2_i_hoistedGlobal;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_e_y;
  real_T c2_b_x;
  real_T c2_f_y;
  real_T c2_j_hoistedGlobal;
  real_T c2_k_hoistedGlobal;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_c_x;
  real_T c2_g_y;
  real_T c2_d_x;
  real_T c2_h_y;
  real_T c2_i_y;
  real_T c2_l_hoistedGlobal;
  real_T c2_c_b;
  real_T c2_j_y;
  real_T c2_c_B;
  real_T c2_k_y;
  real_T c2_l_y;
  real_T c2_f_a;
  int32_T c2_i111;
  int32_T c2_i112;
  real_T c2_g_a;
  int32_T c2_i113;
  int32_T c2_i114;
  int32_T c2_i115;
  real_T c2_d_b[180];
  int32_T c2_i116;
  int32_T c2_i117;
  real_T c2_m_hoistedGlobal[190];
  int32_T c2_i118;
  int32_T c2_i119;
  int32_T c2_i120;
  real_T c2_dv23[10];
  int32_T c2_i121;
  int32_T c2_i122;
  real_T c2_b_x_kkm1[4];
  real_T c2_dv24[4];
  int32_T c2_i123;
  real_T c2_dv25[4];
  real_T c2_d2;
  int32_T c2_i124;
  real_T c2_c_x_kkm1[4];
  real_T c2_dv26[4];
  int32_T c2_i125;
  int32_T c2_i126;
  int32_T c2_i127;
  real_T c2_d_x_kkm1[4];
  real_T c2_dv27[4];
  int32_T c2_i128;
  int32_T c2_i129;
  int32_T c2_i130;
  int32_T c2_i131;
  real_T c2_b_X_kkm1[76];
  int32_T c2_i132;
  real_T c2_dv28[4];
  int32_T c2_i133;
  int32_T c2_i134;
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  real_T c2_h_a[6];
  int32_T c2_i138;
  int32_T c2_i139;
  int32_T c2_i140;
  real_T c2_m_y[114];
  int32_T c2_i141;
  int32_T c2_i142;
  int32_T c2_i143;
  int32_T c2_i144;
  int32_T c2_i145;
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  int32_T c2_i149;
  real_T c2_i_a;
  int32_T c2_i150;
  int32_T c2_i151;
  real_T c2_j_a;
  int32_T c2_i152;
  int32_T c2_i153;
  int32_T c2_i154;
  real_T c2_e_b[162];
  int32_T c2_i155;
  int32_T c2_i156;
  real_T c2_f_b[171];
  int32_T c2_i157;
  int32_T c2_i158;
  int32_T c2_i159;
  real_T c2_dv29[9];
  int32_T c2_i160;
  int32_T c2_i161;
  int32_T c2_i;
  real_T c2_b_i;
  real_T c2_k_a;
  int32_T c2_c_i;
  int32_T c2_i162;
  int32_T c2_i163;
  int32_T c2_d_i;
  int32_T c2_i164;
  real_T c2_g_b[9];
  int32_T c2_i165;
  int32_T c2_i166;
  int32_T c2_i167;
  int32_T c2_i168;
  real_T c2_l_a;
  int32_T c2_e_i;
  int32_T c2_i169;
  int32_T c2_i170;
  int32_T c2_f_i;
  int32_T c2_i171;
  int32_T c2_i172;
  int32_T c2_i173;
  int32_T c2_i174;
  int32_T c2_i175;
  int32_T c2_i176;
  int32_T c2_i177;
  int32_T c2_i178;
  int32_T c2_i179;
  real_T c2_c_X_kkm1[190];
  int32_T c2_i180;
  real_T c2_d_B_ref[3];
  real_T c2_dv30[114];
  int32_T c2_i181;
  real_T c2_m_a;
  int32_T c2_i182;
  int32_T c2_i183;
  real_T c2_n_a;
  int32_T c2_i184;
  int32_T c2_i185;
  int32_T c2_i186;
  real_T c2_h_b[108];
  int32_T c2_i187;
  int32_T c2_i188;
  real_T c2_o_a[114];
  int32_T c2_i189;
  int32_T c2_i190;
  int32_T c2_i191;
  real_T c2_dv31[6];
  int32_T c2_i192;
  int32_T c2_i193;
  int32_T c2_i194;
  int32_T c2_g_i;
  real_T c2_p_a;
  int32_T c2_h_i;
  int32_T c2_i195;
  int32_T c2_i196;
  int32_T c2_i_i;
  int32_T c2_i197;
  real_T c2_i_b[6];
  int32_T c2_i198;
  int32_T c2_i199;
  int32_T c2_i200;
  real_T c2_j_b[36];
  int32_T c2_i201;
  real_T c2_q_a;
  int32_T c2_j_i;
  int32_T c2_i202;
  int32_T c2_i203;
  int32_T c2_k_i;
  int32_T c2_i204;
  int32_T c2_i205;
  int32_T c2_i206;
  int32_T c2_i207;
  real_T c2_n_y[54];
  int32_T c2_i208;
  real_T c2_r_a;
  int32_T c2_l_i;
  int32_T c2_i209;
  int32_T c2_i210;
  int32_T c2_m_i;
  int32_T c2_i211;
  int32_T c2_i212;
  int32_T c2_i213;
  int32_T c2_i214;
  int32_T c2_i215;
  real_T c2_s_a;
  int32_T c2_n_i;
  int32_T c2_i216;
  int32_T c2_i217;
  int32_T c2_o_i;
  int32_T c2_i218;
  int32_T c2_i219;
  int32_T c2_i220;
  int32_T c2_i221;
  int32_T c2_i222;
  int32_T c2_i223;
  int32_T c2_i224;
  real_T c2_b_P_xkzk[54];
  int32_T c2_i225;
  real_T c2_b_P_zkzk[36];
  real_T c2_dv32[54];
  int32_T c2_i226;
  int32_T c2_i227;
  real_T c2_b_z_kkm1_control[3];
  int32_T c2_i228;
  real_T c2_t_a[3];
  real_T c2_d3;
  int32_T c2_i229;
  real_T c2_c_z_kkm1_control[3];
  int32_T c2_i230;
  int32_T c2_i231;
  real_T c2_dv33[4];
  int32_T c2_i232;
  real_T c2_dv34[4];
  real_T c2_dv35[4];
  int32_T c2_i233;
  int32_T c2_i234;
  real_T c2_dv36[4];
  int32_T c2_i235;
  real_T c2_dv37[4];
  int32_T c2_i236;
  int32_T c2_i237;
  int32_T c2_i238;
  real_T c2_dv38[4];
  int32_T c2_i239;
  real_T c2_dv39[4];
  real_T c2_dv40[4];
  int32_T c2_i240;
  int32_T c2_i241;
  real_T c2_dv41[4];
  int32_T c2_i242;
  real_T c2_dv42[4];
  int32_T c2_i243;
  int32_T c2_i244;
  int32_T c2_i245;
  real_T c2_u_a[54];
  int32_T c2_i246;
  real_T c2_c_B_control[6];
  int32_T c2_i247;
  int32_T c2_i248;
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
  real_T c2_b_dx_kk[3];
  int32_T c2_i259;
  real_T c2_v_a[3];
  real_T c2_d4;
  real_T c2_dv43[4];
  int32_T c2_i260;
  int32_T c2_i261;
  real_T c2_e_x_kkm1[4];
  int32_T c2_i262;
  int32_T c2_i263;
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
  real_T c2_k_b[54];
  int32_T c2_i277;
  int32_T c2_i278;
  int32_T c2_i279;
  int32_T c2_i280;
  int32_T c2_i281;
  int32_T c2_i282;
  int32_T c2_i283;
  int32_T c2_i284;
  int32_T c2_i285;
  int32_T c2_i286;
  real_T c2_dv44[4];
  int32_T c2_i287;
  real_T c2_dv45[4];
  int32_T c2_i288;
  real_T c2_dv46[4];
  int32_T c2_i289;
  int32_T c2_i290;
  real_T c2_dv47[4];
  real_T c2_dv48[4];
  int32_T c2_i291;
  int32_T c2_i292;
  real_T c2_dv49[4];
  real_T c2_dv50[4];
  int32_T c2_i293;
  real_T c2_dv51[4];
  int32_T c2_i294;
  real_T c2_dv52[4];
  int32_T c2_i295;
  int32_T c2_i296;
  int32_T c2_i297;
  real_T c2_dv53[4];
  real_T c2_dv54[4];
  int32_T c2_i298;
  int32_T c2_i299;
  real_T c2_dv55[4];
  int32_T c2_i300;
  real_T c2_dv56[4];
  int32_T c2_i301;
  real_T c2_dv57[4];
  int32_T c2_i302;
  int32_T c2_i303;
  int32_T c2_i304;
  int32_T c2_i305;
  int32_T c2_i306;
  real_T (*c2_b_Attitude_sensor)[4];
  real_T (*c2_b_w_sensor)[3];
  real_T (*c2_b_w_bias_sensor)[3];
  real_T (*c2_c_w_gyro)[3];
  real_T (*c2_d_B_sat)[3];
  real_T (*c2_e_B_ref)[3];
  c2_b_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 3);
  c2_c_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
  c2_d_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S,
    1);
  c2_e_B_ref = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
    c2_B_ref[c2_i26] = (*c2_e_B_ref)[c2_i26];
  }

  for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
    c2_B_sat[c2_i27] = (*c2_d_B_sat)[c2_i27];
  }

  for (c2_i28 = 0; c2_i28 < 3; c2_i28++) {
    c2_w_gyro[c2_i28] = (*c2_c_w_gyro)[c2_i28];
  }

  sf_debug_symbol_scope_push_eml(0U, 46U, 48U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_P_kk, 0U, c2_u_sf_marshallOut,
    c2_u_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c2_L, 1U, c2_o_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_w_bias, 2U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_DX_sigma, 3U, c2_u_sf_marshallOut,
    c2_u_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dX_km1km1, 4U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_km1km1_temp, 5U,
    c2_w_sf_marshallOut, c2_w_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_km1km1, 6U, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_kkm1, 7U, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_W0_m, 8U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_W0_c, 9U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_Wi_cm, 10U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_x_kkm1, 11U, c2_x_sf_marshallOut,
    c2_x_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dX_kkm1_temp, 12U,
    c2_w_sf_marshallOut, c2_w_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dX_kkm1, 13U, c2_v_sf_marshallOut,
    c2_v_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dx_kkm1, 14U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_P_kkm1, 15U, c2_u_sf_marshallOut,
    c2_u_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Z_kkm1_control, 16U,
    c2_t_sf_marshallOut, c2_t_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_z_kkm1_control, 17U,
    c2_s_sf_marshallOut, c2_s_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_P_zkzk, 18U, c2_r_sf_marshallOut,
    c2_r_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_P_xkzk, 19U, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_K_k, 20U, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_B_control, MAX_uint32_T,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_control, MAX_uint32_T,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dx_kk, 23U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_sensor_temp, 24U,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_bias_sensor_temp, 25U,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_b_B_control, MAX_uint32_T,
    c2_m_sf_marshallOut, c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_b_w_control, MAX_uint32_T,
    c2_m_sf_marshallOut, c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 26U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 27U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_B_ref, 28U, c2_m_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_B_sat, 29U, c2_m_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c2_w_gyro, 30U, c2_m_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_Attitude_sensor, 31U,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_sensor, 32U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_bias_sensor, 33U,
    c2_m_sf_marshallOut, c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_P_km1km1, 34U,
    c2_l_sf_marshallOut, c2_l_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_x_km1km1, 35U,
    c2_k_sf_marshallOut, c2_k_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_q_s_c, 36U,
    c2_j_sf_marshallOut, c2_j_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_x_kk, 37U,
    c2_i_sf_marshallOut, c2_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_DCM_s2c, 38U,
    c2_h_sf_marshallOut, c2_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_Ts, 39U,
    c2_g_sf_marshallOut, c2_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_R_k, 40U,
    c2_f_sf_marshallOut, c2_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(chartInstance->c2_Q_k, 41U,
    c2_e_sf_marshallOut, c2_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_Lambda, 42U,
    c2_d_sf_marshallOut, c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_alpha, 43U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_K, 44U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&chartInstance->c2_Beta, 45U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
    c2_b_B_sat[c2_i29] = c2_B_sat[c2_i29];
  }

  c2_power(chartInstance, c2_b_B_sat, c2_a);
  for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
    c2_b_a[c2_i30] = c2_a[c2_i30];
  }

  c2_d0 = c2_sum(chartInstance, c2_b_a);
  c2_c_sqrt(chartInstance, &c2_d0);
  for (c2_i31 = 0; c2_i31 < 3; c2_i31++) {
    c2_c_B_sat[c2_i31] = c2_B_sat[c2_i31];
  }

  c2_rdivide(chartInstance, c2_c_B_sat, c2_d0, c2_dv10);
  for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
    c2_B_sat[c2_i32] = c2_dv10[c2_i32];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
    c2_b_B_ref[c2_i33] = c2_B_ref[c2_i33];
  }

  c2_power(chartInstance, c2_b_B_ref, c2_a);
  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_c_a[c2_i34] = c2_a[c2_i34];
  }

  c2_d1 = c2_sum(chartInstance, c2_c_a);
  c2_c_sqrt(chartInstance, &c2_d1);
  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    c2_c_B_ref[c2_i35] = c2_B_ref[c2_i35];
  }

  c2_rdivide(chartInstance, c2_c_B_ref, c2_d1, c2_dv11);
  for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
    c2_B_ref[c2_i36] = c2_dv11[c2_i36];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c2_x_km1km1_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
    for (c2_i37 = 0; c2_i37 < 81; c2_i37++) {
      chartInstance->c2_P_km1km1[c2_i37] = c2_dv12[c2_i37];
    }

    chartInstance->c2_P_km1km1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
    for (c2_i38 = 0; c2_i38 < 81; c2_i38++) {
      c2_P_kk[c2_i38] = chartInstance->c2_P_km1km1[c2_i38];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
    for (c2_i39 = 0; c2_i39 < 4; c2_i39++) {
      chartInstance->c2_q_s_c[c2_i39] = c2_dv13[c2_i39];
    }

    chartInstance->c2_q_s_c_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
    for (c2_i40 = 0; c2_i40 < 9; c2_i40++) {
      chartInstance->c2_DCM_s2c[c2_i40] = 0.0;
    }

    chartInstance->c2_DCM_s2c_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
    for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
      c2_dv14[c2_i41] = chartInstance->c2_q_s_c[c2_i41];
    }

    c2_quat2dcm(chartInstance, c2_dv14, c2_dv15);
    for (c2_i42 = 0; c2_i42 < 9; c2_i42++) {
      chartInstance->c2_DCM_s2c[c2_i42] = c2_dv15[c2_i42];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
    for (c2_i43 = 0; c2_i43 < 10; c2_i43++) {
      chartInstance->c2_x_km1km1[c2_i43] = 0.0;
    }

    chartInstance->c2_x_km1km1_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
    for (c2_i44 = 0; c2_i44 < 4; c2_i44++) {
      chartInstance->c2_x_km1km1[c2_i44] = c2_dv13[c2_i44];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
    for (c2_i45 = 0; c2_i45 < 9; c2_i45++) {
      c2_dv16[c2_i45] = chartInstance->c2_DCM_s2c[c2_i45];
    }

    for (c2_i46 = 0; c2_i46 < 3; c2_i46++) {
      c2_b_w_gyro[c2_i46] = c2_w_gyro[c2_i46];
    }

    c2_mldivide(chartInstance, c2_dv16, c2_b_w_gyro, c2_dv17);
    for (c2_i47 = 0; c2_i47 < 3; c2_i47++) {
      chartInstance->c2_x_km1km1[c2_i47 + 4] = c2_dv17[c2_i47];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
    for (c2_i48 = 0; c2_i48 < 3; c2_i48++) {
      chartInstance->c2_x_km1km1[c2_i48 + 7] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 40);
    for (c2_i49 = 0; c2_i49 < 10; c2_i49++) {
      chartInstance->c2_x_kk[c2_i49] = chartInstance->c2_x_km1km1[c2_i49];
    }

    chartInstance->c2_x_kk_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
    chartInstance->c2_Ts = 0.1;
    chartInstance->c2_Ts_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 46);
    c2_L = 9.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 47);
    chartInstance->c2_alpha = 3.0;
    c2_c_sqrt(chartInstance, &chartInstance->c2_alpha);
    chartInstance->c2_alpha_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
    chartInstance->c2_K = 0.0;
    chartInstance->c2_K_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 49);
    c2_hoistedGlobal = chartInstance->c2_alpha;
    c2_b_hoistedGlobal = chartInstance->c2_K;
    c2_d_a = c2_mpower(chartInstance, c2_hoistedGlobal);
    c2_b = c2_L + c2_b_hoistedGlobal;
    c2_y = c2_d_a * c2_b;
    chartInstance->c2_Lambda = c2_y - c2_L;
    chartInstance->c2_Lambda_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
    for (c2_i50 = 0; c2_i50 < 36; c2_i50++) {
      chartInstance->c2_R_k[c2_i50] = c2_dv18[c2_i50];
    }

    chartInstance->c2_R_k_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
    for (c2_i51 = 0; c2_i51 < 81; c2_i51++) {
      chartInstance->c2_Q_k[c2_i51] = c2_dv19[c2_i51];
    }

    chartInstance->c2_Q_k_not_empty = TRUE;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
    chartInstance->c2_Beta = 2.0;
    chartInstance->c2_Beta_not_empty = TRUE;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
    c2_L = 9.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 57);
    for (c2_i52 = 0; c2_i52 < 10; c2_i52++) {
      chartInstance->c2_x_kk[c2_i52] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
    for (c2_i53 = 0; c2_i53 < 4; c2_i53++) {
      c2_Attitude_sensor[c2_i53] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
    for (c2_i54 = 0; c2_i54 < 3; c2_i54++) {
      c2_w_bias[c2_i54] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
    c2_c_hoistedGlobal = chartInstance->c2_Lambda;
    for (c2_i55 = 0; c2_i55 < 81; c2_i55++) {
      c2_d_hoistedGlobal[c2_i55] = chartInstance->c2_P_km1km1[c2_i55];
    }

    c2_e_a = c2_L + c2_c_hoistedGlobal;
    for (c2_i56 = 0; c2_i56 < 81; c2_i56++) {
      c2_d_hoistedGlobal[c2_i56] *= c2_e_a;
    }

    for (c2_i57 = 0; c2_i57 < 81; c2_i57++) {
      c2_DX_sigma[c2_i57] = c2_d_hoistedGlobal[c2_i57];
    }

    c2_b_chol(chartInstance, c2_DX_sigma);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 64);
    for (c2_i58 = 0; c2_i58 < 9; c2_i58++) {
      c2_b_b[c2_i58] = 0.0;
    }

    for (c2_i59 = 0; c2_i59 < 9; c2_i59++) {
      c2_dX_km1km1[c2_i59] = c2_b_b[c2_i59];
    }

    c2_i60 = 0;
    for (c2_i61 = 0; c2_i61 < 9; c2_i61++) {
      for (c2_i62 = 0; c2_i62 < 9; c2_i62++) {
        c2_dX_km1km1[(c2_i62 + c2_i60) + 9] = -c2_DX_sigma[c2_i62 + c2_i60];
      }

      c2_i60 += 9;
    }

    c2_i63 = 0;
    for (c2_i64 = 0; c2_i64 < 9; c2_i64++) {
      for (c2_i65 = 0; c2_i65 < 9; c2_i65++) {
        c2_dX_km1km1[(c2_i65 + c2_i63) + 90] = c2_DX_sigma[c2_i65 + c2_i63];
      }

      c2_i63 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 67);
    for (c2_i66 = 0; c2_i66 < 76; c2_i66++) {
      c2_X_km1km1_temp[c2_i66] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 68);
    c2_i67 = 0;
    c2_i68 = 0;
    for (c2_i69 = 0; c2_i69 < 19; c2_i69++) {
      for (c2_i70 = 0; c2_i70 < 3; c2_i70++) {
        c2_b_dX_km1km1[c2_i70 + c2_i67] = c2_dX_km1km1[c2_i70 + c2_i68];
      }

      c2_i67 += 3;
      c2_i68 += 9;
    }

    c2_d_power(chartInstance, c2_b_dX_km1km1, c2_b_y);
    for (c2_i71 = 0; c2_i71 < 57; c2_i71++) {
      c2_c_y[c2_i71] = c2_b_y[c2_i71];
    }

    c2_c_sum(chartInstance, c2_c_y, c2_dv20);
    for (c2_i72 = 0; c2_i72 < 19; c2_i72++) {
      c2_dv20[c2_i72] = 1.0 - c2_dv20[c2_i72];
    }

    c2_d_sqrt(chartInstance, c2_dv20);
    c2_i73 = 0;
    for (c2_i74 = 0; c2_i74 < 19; c2_i74++) {
      c2_X_km1km1_temp[c2_i73] = c2_dv20[c2_i74];
      c2_i73 += 4;
    }

    c2_i75 = 0;
    c2_i76 = 0;
    for (c2_i77 = 0; c2_i77 < 19; c2_i77++) {
      for (c2_i78 = 0; c2_i78 < 3; c2_i78++) {
        c2_X_km1km1_temp[(c2_i78 + c2_i75) + 1] = c2_dX_km1km1[c2_i78 + c2_i76];
      }

      c2_i75 += 4;
      c2_i76 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 69);
    for (c2_i79 = 0; c2_i79 < 10; c2_i79++) {
      c2_e_hoistedGlobal[c2_i79] = chartInstance->c2_x_km1km1[c2_i79];
    }

    for (c2_i80 = 0; c2_i80 < 10; c2_i80++) {
      c2_f_hoistedGlobal[c2_i80] = chartInstance->c2_x_km1km1[c2_i80];
    }

    for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
      c2_a[c2_i81] = c2_f_hoistedGlobal[c2_i81 + 4];
    }

    for (c2_i82 = 0; c2_i82 < 3; c2_i82++) {
      c2_i83 = 0;
      for (c2_i84 = 0; c2_i84 < 19; c2_i84++) {
        c2_b_y[c2_i83 + c2_i82] = c2_a[c2_i82];
        c2_i83 += 3;
      }
    }

    for (c2_i85 = 0; c2_i85 < 10; c2_i85++) {
      c2_f_hoistedGlobal[c2_i85] = chartInstance->c2_x_km1km1[c2_i85];
    }

    for (c2_i86 = 0; c2_i86 < 3; c2_i86++) {
      c2_a[c2_i86] = c2_f_hoistedGlobal[c2_i86 + 7];
    }

    for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
      c2_i88 = 0;
      for (c2_i89 = 0; c2_i89 < 19; c2_i89++) {
        c2_d_y[c2_i88 + c2_i87] = c2_a[c2_i87];
        c2_i88 += 3;
      }
    }

    c2_i90 = 0;
    for (c2_i91 = 0; c2_i91 < 4; c2_i91++) {
      c2_i92 = 0;
      for (c2_i93 = 0; c2_i93 < 19; c2_i93++) {
        c2_b_X_km1km1_temp[c2_i93 + c2_i90] = c2_X_km1km1_temp[c2_i92 + c2_i91];
        c2_i92 += 4;
      }

      c2_i90 += 19;
    }

    for (c2_i94 = 0; c2_i94 < 4; c2_i94++) {
      c2_g_hoistedGlobal[c2_i94] = c2_e_hoistedGlobal[c2_i94];
    }

    c2_quatmultiply(chartInstance, c2_b_X_km1km1_temp, c2_g_hoistedGlobal,
                    c2_dv21);
    c2_i95 = 0;
    for (c2_i96 = 0; c2_i96 < 19; c2_i96++) {
      c2_i97 = 0;
      for (c2_i98 = 0; c2_i98 < 4; c2_i98++) {
        c2_X_km1km1[c2_i98 + c2_i95] = c2_dv21[c2_i97 + c2_i96];
        c2_i97 += 19;
      }

      c2_i95 += 10;
    }

    c2_i99 = 0;
    c2_i100 = 0;
    c2_i101 = 0;
    for (c2_i102 = 0; c2_i102 < 19; c2_i102++) {
      for (c2_i103 = 0; c2_i103 < 3; c2_i103++) {
        c2_X_km1km1[(c2_i103 + c2_i99) + 4] = c2_b_y[c2_i103 + c2_i100] +
          c2_dX_km1km1[(c2_i103 + c2_i101) + 3];
      }

      c2_i99 += 10;
      c2_i100 += 3;
      c2_i101 += 9;
    }

    c2_i104 = 0;
    c2_i105 = 0;
    c2_i106 = 0;
    for (c2_i107 = 0; c2_i107 < 19; c2_i107++) {
      for (c2_i108 = 0; c2_i108 < 3; c2_i108++) {
        c2_X_km1km1[(c2_i108 + c2_i104) + 7] = c2_d_y[c2_i108 + c2_i105] +
          c2_dX_km1km1[(c2_i108 + c2_i106) + 6];
      }

      c2_i104 += 10;
      c2_i105 += 3;
      c2_i106 += 9;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 75);
    for (c2_i109 = 0; c2_i109 < 190; c2_i109++) {
      c2_b_X_km1km1[c2_i109] = c2_X_km1km1[c2_i109];
    }

    c2_RK4(chartInstance, c2_b_X_km1km1, chartInstance->c2_Ts, c2_dv22);
    for (c2_i110 = 0; c2_i110 < 190; c2_i110++) {
      c2_X_kkm1[c2_i110] = c2_dv22[c2_i110];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 78);
    c2_h_hoistedGlobal = chartInstance->c2_Lambda;
    c2_i_hoistedGlobal = chartInstance->c2_Lambda;
    c2_A = c2_h_hoistedGlobal;
    c2_B = c2_L + c2_i_hoistedGlobal;
    c2_x = c2_A;
    c2_e_y = c2_B;
    c2_b_x = c2_x;
    c2_f_y = c2_e_y;
    c2_W0_m = c2_b_x / c2_f_y;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 79);
    c2_j_hoistedGlobal = chartInstance->c2_Lambda;
    c2_k_hoistedGlobal = chartInstance->c2_Lambda;
    c2_b_A = c2_j_hoistedGlobal;
    c2_b_B = c2_L + c2_k_hoistedGlobal;
    c2_c_x = c2_b_A;
    c2_g_y = c2_b_B;
    c2_d_x = c2_c_x;
    c2_h_y = c2_g_y;
    c2_i_y = c2_d_x / c2_h_y;
    c2_W0_c = c2_i_y + ((1.0 - c2_mpower(chartInstance, chartInstance->c2_alpha))
                        + chartInstance->c2_Beta);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 80);
    c2_l_hoistedGlobal = chartInstance->c2_Lambda;
    c2_c_b = c2_L + c2_l_hoistedGlobal;
    c2_j_y = 2.0 * c2_c_b;
    c2_c_B = c2_j_y;
    c2_k_y = c2_c_B;
    c2_l_y = c2_k_y;
    c2_Wi_cm = 1.0 / c2_l_y;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 81);
    c2_f_a = c2_W0_m;
    for (c2_i111 = 0; c2_i111 < 10; c2_i111++) {
      c2_e_hoistedGlobal[c2_i111] = c2_X_kkm1[c2_i111];
    }

    for (c2_i112 = 0; c2_i112 < 10; c2_i112++) {
      c2_e_hoistedGlobal[c2_i112] *= c2_f_a;
    }

    c2_g_a = c2_Wi_cm;
    c2_i113 = 0;
    for (c2_i114 = 0; c2_i114 < 18; c2_i114++) {
      for (c2_i115 = 0; c2_i115 < 10; c2_i115++) {
        c2_d_b[c2_i115 + c2_i113] = c2_X_kkm1[(c2_i115 + c2_i113) + 10];
      }

      c2_i113 += 10;
    }

    for (c2_i116 = 0; c2_i116 < 180; c2_i116++) {
      c2_d_b[c2_i116] *= c2_g_a;
    }

    for (c2_i117 = 0; c2_i117 < 10; c2_i117++) {
      c2_m_hoistedGlobal[c2_i117] = c2_e_hoistedGlobal[c2_i117];
    }

    c2_i118 = 0;
    for (c2_i119 = 0; c2_i119 < 18; c2_i119++) {
      for (c2_i120 = 0; c2_i120 < 10; c2_i120++) {
        c2_m_hoistedGlobal[(c2_i120 + c2_i118) + 10] = c2_d_b[c2_i120 + c2_i118];
      }

      c2_i118 += 10;
    }

    c2_d_sum(chartInstance, c2_m_hoistedGlobal, c2_dv23);
    for (c2_i121 = 0; c2_i121 < 10; c2_i121++) {
      c2_x_kkm1[c2_i121] = c2_dv23[c2_i121];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 82);
    for (c2_i122 = 0; c2_i122 < 4; c2_i122++) {
      c2_b_x_kkm1[c2_i122] = c2_x_kkm1[c2_i122];
    }

    c2_e_power(chartInstance, c2_b_x_kkm1, c2_dv24);
    for (c2_i123 = 0; c2_i123 < 4; c2_i123++) {
      c2_dv25[c2_i123] = c2_dv24[c2_i123];
    }

    c2_d2 = c2_e_sum(chartInstance, c2_dv25);
    c2_c_sqrt(chartInstance, &c2_d2);
    for (c2_i124 = 0; c2_i124 < 4; c2_i124++) {
      c2_c_x_kkm1[c2_i124] = c2_x_kkm1[c2_i124];
    }

    c2_b_rdivide(chartInstance, c2_c_x_kkm1, c2_d2, c2_dv26);
    for (c2_i125 = 0; c2_i125 < 4; c2_i125++) {
      c2_x_kkm1[c2_i125] = c2_dv26[c2_i125];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 86);
    for (c2_i126 = 0; c2_i126 < 76; c2_i126++) {
      c2_dX_kkm1_temp[c2_i126] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 87);
    for (c2_i127 = 0; c2_i127 < 4; c2_i127++) {
      c2_d_x_kkm1[c2_i127] = c2_x_kkm1[c2_i127];
    }

    c2_quatinv(chartInstance, c2_d_x_kkm1, c2_dv27);
    c2_i128 = 0;
    for (c2_i129 = 0; c2_i129 < 4; c2_i129++) {
      c2_i130 = 0;
      for (c2_i131 = 0; c2_i131 < 19; c2_i131++) {
        c2_b_X_kkm1[c2_i131 + c2_i128] = c2_X_kkm1[c2_i130 + c2_i129];
        c2_i130 += 10;
      }

      c2_i128 += 19;
    }

    for (c2_i132 = 0; c2_i132 < 4; c2_i132++) {
      c2_dv28[c2_i132] = c2_dv27[c2_i132];
    }

    c2_quatmultiply(chartInstance, c2_b_X_kkm1, c2_dv28, c2_dv21);
    c2_i133 = 0;
    for (c2_i134 = 0; c2_i134 < 19; c2_i134++) {
      c2_i135 = 0;
      for (c2_i136 = 0; c2_i136 < 4; c2_i136++) {
        c2_dX_kkm1_temp[c2_i136 + c2_i133] = c2_dv21[c2_i135 + c2_i134];
        c2_i135 += 19;
      }

      c2_i133 += 4;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 88);
    for (c2_i137 = 0; c2_i137 < 6; c2_i137++) {
      c2_h_a[c2_i137] = c2_x_kkm1[c2_i137 + 4];
    }

    for (c2_i138 = 0; c2_i138 < 6; c2_i138++) {
      c2_i139 = 0;
      for (c2_i140 = 0; c2_i140 < 19; c2_i140++) {
        c2_m_y[c2_i139 + c2_i138] = c2_h_a[c2_i138];
        c2_i139 += 6;
      }
    }

    c2_i141 = 0;
    c2_i142 = 0;
    for (c2_i143 = 0; c2_i143 < 19; c2_i143++) {
      for (c2_i144 = 0; c2_i144 < 3; c2_i144++) {
        c2_dX_kkm1[c2_i144 + c2_i141] = c2_dX_kkm1_temp[(c2_i144 + c2_i142) + 1];
      }

      c2_i141 += 9;
      c2_i142 += 4;
    }

    c2_i145 = 0;
    c2_i146 = 0;
    c2_i147 = 0;
    for (c2_i148 = 0; c2_i148 < 19; c2_i148++) {
      for (c2_i149 = 0; c2_i149 < 6; c2_i149++) {
        c2_dX_kkm1[(c2_i149 + c2_i145) + 3] = c2_X_kkm1[(c2_i149 + c2_i146) + 4]
          - c2_m_y[c2_i149 + c2_i147];
      }

      c2_i145 += 9;
      c2_i146 += 10;
      c2_i147 += 6;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 92);
    c2_i_a = c2_W0_m;
    for (c2_i150 = 0; c2_i150 < 9; c2_i150++) {
      c2_b_b[c2_i150] = c2_dX_kkm1[c2_i150];
    }

    for (c2_i151 = 0; c2_i151 < 9; c2_i151++) {
      c2_b_b[c2_i151] *= c2_i_a;
    }

    c2_j_a = c2_Wi_cm;
    c2_i152 = 0;
    for (c2_i153 = 0; c2_i153 < 18; c2_i153++) {
      for (c2_i154 = 0; c2_i154 < 9; c2_i154++) {
        c2_e_b[c2_i154 + c2_i152] = c2_dX_kkm1[(c2_i154 + c2_i152) + 9];
      }

      c2_i152 += 9;
    }

    for (c2_i155 = 0; c2_i155 < 162; c2_i155++) {
      c2_e_b[c2_i155] *= c2_j_a;
    }

    for (c2_i156 = 0; c2_i156 < 9; c2_i156++) {
      c2_f_b[c2_i156] = c2_b_b[c2_i156];
    }

    c2_i157 = 0;
    for (c2_i158 = 0; c2_i158 < 18; c2_i158++) {
      for (c2_i159 = 0; c2_i159 < 9; c2_i159++) {
        c2_f_b[(c2_i159 + c2_i157) + 9] = c2_e_b[c2_i159 + c2_i157];
      }

      c2_i157 += 9;
    }

    c2_f_sum(chartInstance, c2_f_b, c2_dv29);
    for (c2_i160 = 0; c2_i160 < 9; c2_i160++) {
      c2_dx_kkm1[c2_i160] = c2_dv29[c2_i160];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 95);
    for (c2_i161 = 0; c2_i161 < 81; c2_i161++) {
      c2_P_kkm1[c2_i161] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 96);
    c2_i = 0;
    while (c2_i < 19) {
      c2_b_i = 1.0 + (real_T)c2_i;
      CV_EML_FOR(0, 1, 0, 1);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 97);
      if (CV_EML_IF(0, 1, 1, c2_b_i == 1.0)) {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 98);
        c2_k_a = c2_W0_c;
        c2_c_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i162 = 0; c2_i162 < 9; c2_i162++) {
          c2_b_b[c2_i162] = c2_dX_kkm1[c2_i162 + 9 * c2_c_i] -
            c2_dx_kkm1[c2_i162];
        }

        for (c2_i163 = 0; c2_i163 < 9; c2_i163++) {
          c2_b_b[c2_i163] *= c2_k_a;
        }

        c2_d_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i164 = 0; c2_i164 < 9; c2_i164++) {
          c2_g_b[c2_i164] = c2_dX_kkm1[c2_i164 + 9 * c2_d_i] -
            c2_dx_kkm1[c2_i164];
        }

        for (c2_i165 = 0; c2_i165 < 9; c2_i165++) {
          c2_i166 = 0;
          for (c2_i167 = 0; c2_i167 < 9; c2_i167++) {
            c2_d_hoistedGlobal[c2_i166 + c2_i165] = c2_b_b[c2_i165] *
              c2_g_b[c2_i167];
            c2_i166 += 9;
          }
        }

        for (c2_i168 = 0; c2_i168 < 81; c2_i168++) {
          c2_P_kkm1[c2_i168] += c2_d_hoistedGlobal[c2_i168];
        }
      } else {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 101);
        c2_l_a = c2_Wi_cm;
        c2_e_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i169 = 0; c2_i169 < 9; c2_i169++) {
          c2_b_b[c2_i169] = c2_dX_kkm1[c2_i169 + 9 * c2_e_i] -
            c2_dx_kkm1[c2_i169];
        }

        for (c2_i170 = 0; c2_i170 < 9; c2_i170++) {
          c2_b_b[c2_i170] *= c2_l_a;
        }

        c2_f_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i171 = 0; c2_i171 < 9; c2_i171++) {
          c2_g_b[c2_i171] = c2_dX_kkm1[c2_i171 + 9 * c2_f_i] -
            c2_dx_kkm1[c2_i171];
        }

        for (c2_i172 = 0; c2_i172 < 9; c2_i172++) {
          c2_i173 = 0;
          for (c2_i174 = 0; c2_i174 < 9; c2_i174++) {
            c2_d_hoistedGlobal[c2_i173 + c2_i172] = c2_b_b[c2_i172] *
              c2_g_b[c2_i174];
            c2_i173 += 9;
          }
        }

        for (c2_i175 = 0; c2_i175 < 81; c2_i175++) {
          c2_P_kkm1[c2_i175] += c2_d_hoistedGlobal[c2_i175];
        }
      }

      c2_i++;
      sf_mex_listen_for_ctrl_c(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 0, 0);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 105);
    for (c2_i176 = 0; c2_i176 < 81; c2_i176++) {
      c2_P_kkm1[c2_i176] += chartInstance->c2_Q_k[c2_i176];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 110);
    for (c2_i177 = 0; c2_i177 < 114; c2_i177++) {
      c2_Z_kkm1_control[c2_i177] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 111);
    for (c2_i178 = 0; c2_i178 < 6; c2_i178++) {
      c2_z_kkm1_control[c2_i178] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 112);
    for (c2_i179 = 0; c2_i179 < 190; c2_i179++) {
      c2_c_X_kkm1[c2_i179] = c2_X_kkm1[c2_i179];
    }

    for (c2_i180 = 0; c2_i180 < 3; c2_i180++) {
      c2_d_B_ref[c2_i180] = c2_B_ref[c2_i180];
    }

    c2_SensorModel(chartInstance, c2_c_X_kkm1, c2_d_B_ref, c2_dv30);
    for (c2_i181 = 0; c2_i181 < 114; c2_i181++) {
      c2_Z_kkm1_control[c2_i181] = c2_dv30[c2_i181];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 113);
    c2_m_a = c2_W0_m;
    for (c2_i182 = 0; c2_i182 < 6; c2_i182++) {
      c2_h_a[c2_i182] = c2_Z_kkm1_control[c2_i182];
    }

    for (c2_i183 = 0; c2_i183 < 6; c2_i183++) {
      c2_h_a[c2_i183] *= c2_m_a;
    }

    c2_n_a = c2_Wi_cm;
    c2_i184 = 0;
    for (c2_i185 = 0; c2_i185 < 18; c2_i185++) {
      for (c2_i186 = 0; c2_i186 < 6; c2_i186++) {
        c2_h_b[c2_i186 + c2_i184] = c2_Z_kkm1_control[(c2_i186 + c2_i184) + 6];
      }

      c2_i184 += 6;
    }

    for (c2_i187 = 0; c2_i187 < 108; c2_i187++) {
      c2_h_b[c2_i187] *= c2_n_a;
    }

    for (c2_i188 = 0; c2_i188 < 6; c2_i188++) {
      c2_o_a[c2_i188] = c2_h_a[c2_i188];
    }

    c2_i189 = 0;
    for (c2_i190 = 0; c2_i190 < 18; c2_i190++) {
      for (c2_i191 = 0; c2_i191 < 6; c2_i191++) {
        c2_o_a[(c2_i191 + c2_i189) + 6] = c2_h_b[c2_i191 + c2_i189];
      }

      c2_i189 += 6;
    }

    c2_g_sum(chartInstance, c2_o_a, c2_dv31);
    for (c2_i192 = 0; c2_i192 < 6; c2_i192++) {
      c2_z_kkm1_control[c2_i192] = c2_dv31[c2_i192];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 117);
    for (c2_i193 = 0; c2_i193 < 36; c2_i193++) {
      c2_P_zkzk[c2_i193] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 118);
    for (c2_i194 = 0; c2_i194 < 54; c2_i194++) {
      c2_P_xkzk[c2_i194] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 119);
    c2_g_i = 0;
    while (c2_g_i < 19) {
      c2_b_i = 1.0 + (real_T)c2_g_i;
      CV_EML_FOR(0, 1, 1, 1);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 120);
      if (CV_EML_IF(0, 1, 2, c2_b_i == 1.0)) {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 121);
        c2_p_a = c2_W0_c;
        c2_h_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i195 = 0; c2_i195 < 6; c2_i195++) {
          c2_h_a[c2_i195] = c2_Z_kkm1_control[c2_i195 + 6 * c2_h_i] -
            c2_z_kkm1_control[c2_i195];
        }

        for (c2_i196 = 0; c2_i196 < 6; c2_i196++) {
          c2_h_a[c2_i196] *= c2_p_a;
        }

        c2_i_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i197 = 0; c2_i197 < 6; c2_i197++) {
          c2_i_b[c2_i197] = c2_Z_kkm1_control[c2_i197 + 6 * c2_i_i] -
            c2_z_kkm1_control[c2_i197];
        }

        for (c2_i198 = 0; c2_i198 < 6; c2_i198++) {
          c2_i199 = 0;
          for (c2_i200 = 0; c2_i200 < 6; c2_i200++) {
            c2_j_b[c2_i199 + c2_i198] = c2_h_a[c2_i198] * c2_i_b[c2_i200];
            c2_i199 += 6;
          }
        }

        for (c2_i201 = 0; c2_i201 < 36; c2_i201++) {
          c2_P_zkzk[c2_i201] += c2_j_b[c2_i201];
        }

        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 123);
        c2_q_a = c2_W0_c;
        c2_j_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i202 = 0; c2_i202 < 9; c2_i202++) {
          c2_b_b[c2_i202] = c2_dX_kkm1[c2_i202 + 9 * c2_j_i] -
            c2_dx_kkm1[c2_i202];
        }

        for (c2_i203 = 0; c2_i203 < 9; c2_i203++) {
          c2_b_b[c2_i203] *= c2_q_a;
        }

        c2_k_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i204 = 0; c2_i204 < 6; c2_i204++) {
          c2_i_b[c2_i204] = c2_Z_kkm1_control[c2_i204 + 6 * c2_k_i] -
            c2_z_kkm1_control[c2_i204];
        }

        for (c2_i205 = 0; c2_i205 < 9; c2_i205++) {
          c2_i206 = 0;
          for (c2_i207 = 0; c2_i207 < 6; c2_i207++) {
            c2_n_y[c2_i206 + c2_i205] = c2_b_b[c2_i205] * c2_i_b[c2_i207];
            c2_i206 += 9;
          }
        }

        for (c2_i208 = 0; c2_i208 < 54; c2_i208++) {
          c2_P_xkzk[c2_i208] += c2_n_y[c2_i208];
        }
      } else {
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 126);
        c2_r_a = c2_Wi_cm;
        c2_l_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i209 = 0; c2_i209 < 6; c2_i209++) {
          c2_h_a[c2_i209] = c2_Z_kkm1_control[c2_i209 + 6 * c2_l_i] -
            c2_z_kkm1_control[c2_i209];
        }

        for (c2_i210 = 0; c2_i210 < 6; c2_i210++) {
          c2_h_a[c2_i210] *= c2_r_a;
        }

        c2_m_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i211 = 0; c2_i211 < 6; c2_i211++) {
          c2_i_b[c2_i211] = c2_Z_kkm1_control[c2_i211 + 6 * c2_m_i] -
            c2_z_kkm1_control[c2_i211];
        }

        for (c2_i212 = 0; c2_i212 < 6; c2_i212++) {
          c2_i213 = 0;
          for (c2_i214 = 0; c2_i214 < 6; c2_i214++) {
            c2_j_b[c2_i213 + c2_i212] = c2_h_a[c2_i212] * c2_i_b[c2_i214];
            c2_i213 += 6;
          }
        }

        for (c2_i215 = 0; c2_i215 < 36; c2_i215++) {
          c2_P_zkzk[c2_i215] += c2_j_b[c2_i215];
        }

        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 128U);
        c2_s_a = c2_Wi_cm;
        c2_n_i = _SFD_EML_ARRAY_BOUNDS_CHECK("dX_kkm1", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i216 = 0; c2_i216 < 9; c2_i216++) {
          c2_b_b[c2_i216] = c2_dX_kkm1[c2_i216 + 9 * c2_n_i] -
            c2_dx_kkm1[c2_i216];
        }

        for (c2_i217 = 0; c2_i217 < 9; c2_i217++) {
          c2_b_b[c2_i217] *= c2_s_a;
        }

        c2_o_i = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1_control", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_b_i), 1, 19, 2, 0) - 1;
        for (c2_i218 = 0; c2_i218 < 6; c2_i218++) {
          c2_i_b[c2_i218] = c2_Z_kkm1_control[c2_i218 + 6 * c2_o_i] -
            c2_z_kkm1_control[c2_i218];
        }

        for (c2_i219 = 0; c2_i219 < 9; c2_i219++) {
          c2_i220 = 0;
          for (c2_i221 = 0; c2_i221 < 6; c2_i221++) {
            c2_n_y[c2_i220 + c2_i219] = c2_b_b[c2_i219] * c2_i_b[c2_i221];
            c2_i220 += 9;
          }
        }

        for (c2_i222 = 0; c2_i222 < 54; c2_i222++) {
          c2_P_xkzk[c2_i222] += c2_n_y[c2_i222];
        }
      }

      c2_g_i++;
      sf_mex_listen_for_ctrl_c(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 1, 0);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 132U);
    for (c2_i223 = 0; c2_i223 < 36; c2_i223++) {
      c2_P_zkzk[c2_i223] += chartInstance->c2_R_k[c2_i223];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 135U);
    for (c2_i224 = 0; c2_i224 < 54; c2_i224++) {
      c2_b_P_xkzk[c2_i224] = c2_P_xkzk[c2_i224];
    }

    for (c2_i225 = 0; c2_i225 < 36; c2_i225++) {
      c2_b_P_zkzk[c2_i225] = c2_P_zkzk[c2_i225];
    }

    c2_mrdivide(chartInstance, c2_b_P_xkzk, c2_b_P_zkzk, c2_dv32);
    for (c2_i226 = 0; c2_i226 < 54; c2_i226++) {
      c2_K_k[c2_i226] = c2_dv32[c2_i226];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 138U);
    for (c2_i227 = 0; c2_i227 < 3; c2_i227++) {
      c2_b_z_kkm1_control[c2_i227] = c2_z_kkm1_control[c2_i227];
    }

    c2_power(chartInstance, c2_b_z_kkm1_control, c2_a);
    for (c2_i228 = 0; c2_i228 < 3; c2_i228++) {
      c2_t_a[c2_i228] = c2_a[c2_i228];
    }

    c2_d3 = c2_sum(chartInstance, c2_t_a);
    c2_c_sqrt(chartInstance, &c2_d3);
    for (c2_i229 = 0; c2_i229 < 3; c2_i229++) {
      c2_c_z_kkm1_control[c2_i229] = c2_z_kkm1_control[c2_i229];
    }

    c2_rdivide(chartInstance, c2_c_z_kkm1_control, c2_d3, c2_a);
    for (c2_i230 = 0; c2_i230 < 3; c2_i230++) {
      c2_z_kkm1_control[c2_i230] = c2_a[c2_i230];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 144U);
    for (c2_i231 = 0; c2_i231 < 4; c2_i231++) {
      c2_dv33[c2_i231] = chartInstance->c2_q_s_c[c2_i231];
    }

    c2_quatinv(chartInstance, c2_dv33, c2_dv27);
    for (c2_i232 = 0; c2_i232 < 4; c2_i232++) {
      c2_dv34[c2_i232] = c2_dv27[c2_i232];
    }

    c2_dv35[0] = 0.0;
    for (c2_i233 = 0; c2_i233 < 3; c2_i233++) {
      c2_dv35[c2_i233 + 1] = c2_B_sat[c2_i233];
    }

    c2_b_quatmultiply(chartInstance, c2_dv34, c2_dv35, c2_dv27);
    for (c2_i234 = 0; c2_i234 < 4; c2_i234++) {
      c2_dv36[c2_i234] = c2_dv27[c2_i234];
    }

    for (c2_i235 = 0; c2_i235 < 4; c2_i235++) {
      c2_dv37[c2_i235] = chartInstance->c2_q_s_c[c2_i235];
    }

    c2_b_quatmultiply(chartInstance, c2_dv36, c2_dv37, c2_dv27);
    for (c2_i236 = 0; c2_i236 < 4; c2_i236++) {
      c2_B_control[c2_i236] = c2_dv27[c2_i236];
    }

    sf_debug_symbol_switch(21U, 21U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 145U);
    for (c2_i237 = 0; c2_i237 < 3; c2_i237++) {
      c2_b_B_control[c2_i237] = c2_B_control[c2_i237 + 1];
    }

    sf_debug_symbol_switch(21U, 26U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 147U);
    for (c2_i238 = 0; c2_i238 < 4; c2_i238++) {
      c2_dv38[c2_i238] = chartInstance->c2_q_s_c[c2_i238];
    }

    c2_quatinv(chartInstance, c2_dv38, c2_dv27);
    for (c2_i239 = 0; c2_i239 < 4; c2_i239++) {
      c2_dv39[c2_i239] = c2_dv27[c2_i239];
    }

    c2_dv40[0] = 0.0;
    for (c2_i240 = 0; c2_i240 < 3; c2_i240++) {
      c2_dv40[c2_i240 + 1] = c2_w_gyro[c2_i240];
    }

    c2_b_quatmultiply(chartInstance, c2_dv39, c2_dv40, c2_dv27);
    for (c2_i241 = 0; c2_i241 < 4; c2_i241++) {
      c2_dv41[c2_i241] = c2_dv27[c2_i241];
    }

    for (c2_i242 = 0; c2_i242 < 4; c2_i242++) {
      c2_dv42[c2_i242] = chartInstance->c2_q_s_c[c2_i242];
    }

    c2_b_quatmultiply(chartInstance, c2_dv41, c2_dv42, c2_dv27);
    for (c2_i243 = 0; c2_i243 < 4; c2_i243++) {
      c2_w_control[c2_i243] = c2_dv27[c2_i243];
    }

    sf_debug_symbol_switch(22U, 22U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 148U);
    for (c2_i244 = 0; c2_i244 < 3; c2_i244++) {
      c2_b_w_control[c2_i244] = c2_w_control[c2_i244 + 1];
    }

    sf_debug_symbol_switch(22U, 27U);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 149U);
    for (c2_i245 = 0; c2_i245 < 54; c2_i245++) {
      c2_u_a[c2_i245] = c2_K_k[c2_i245];
    }

    for (c2_i246 = 0; c2_i246 < 3; c2_i246++) {
      c2_c_B_control[c2_i246] = c2_b_B_control[c2_i246];
    }

    for (c2_i247 = 0; c2_i247 < 3; c2_i247++) {
      c2_c_B_control[c2_i247 + 3] = c2_b_w_control[c2_i247];
    }

    for (c2_i248 = 0; c2_i248 < 6; c2_i248++) {
      c2_h_a[c2_i248] = c2_c_B_control[c2_i248] - c2_z_kkm1_control[c2_i248];
    }

    c2_e_eml_scalar_eg(chartInstance);
    c2_e_eml_scalar_eg(chartInstance);
    for (c2_i249 = 0; c2_i249 < 9; c2_i249++) {
      c2_dx_kk[c2_i249] = 0.0;
    }

    for (c2_i250 = 0; c2_i250 < 9; c2_i250++) {
      c2_dx_kk[c2_i250] = 0.0;
    }

    for (c2_i251 = 0; c2_i251 < 9; c2_i251++) {
      c2_b_b[c2_i251] = c2_dx_kk[c2_i251];
    }

    for (c2_i252 = 0; c2_i252 < 9; c2_i252++) {
      c2_dx_kk[c2_i252] = c2_b_b[c2_i252];
    }

    for (c2_i253 = 0; c2_i253 < 9; c2_i253++) {
      c2_b_b[c2_i253] = c2_dx_kk[c2_i253];
    }

    for (c2_i254 = 0; c2_i254 < 9; c2_i254++) {
      c2_dx_kk[c2_i254] = c2_b_b[c2_i254];
    }

    for (c2_i255 = 0; c2_i255 < 9; c2_i255++) {
      c2_dx_kk[c2_i255] = 0.0;
      c2_i256 = 0;
      for (c2_i257 = 0; c2_i257 < 6; c2_i257++) {
        c2_dx_kk[c2_i255] += c2_u_a[c2_i256 + c2_i255] * c2_h_a[c2_i257];
        c2_i256 += 9;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 152U);
    for (c2_i258 = 0; c2_i258 < 3; c2_i258++) {
      c2_b_dx_kk[c2_i258] = c2_dx_kk[c2_i258];
    }

    c2_power(chartInstance, c2_b_dx_kk, c2_a);
    for (c2_i259 = 0; c2_i259 < 3; c2_i259++) {
      c2_v_a[c2_i259] = c2_a[c2_i259];
    }

    c2_d4 = 1.0 - c2_sum(chartInstance, c2_v_a);
    c2_c_sqrt(chartInstance, &c2_d4);
    c2_dv43[0] = c2_d4;
    for (c2_i260 = 0; c2_i260 < 3; c2_i260++) {
      c2_dv43[c2_i260 + 1] = c2_dx_kk[c2_i260];
    }

    for (c2_i261 = 0; c2_i261 < 4; c2_i261++) {
      c2_e_x_kkm1[c2_i261] = c2_x_kkm1[c2_i261];
    }

    c2_b_quatmultiply(chartInstance, c2_dv43, c2_e_x_kkm1, c2_dv27);
    for (c2_i262 = 0; c2_i262 < 4; c2_i262++) {
      chartInstance->c2_x_kk[c2_i262] = c2_dv27[c2_i262];
    }

    for (c2_i263 = 0; c2_i263 < 3; c2_i263++) {
      chartInstance->c2_x_kk[c2_i263 + 4] = c2_x_kkm1[c2_i263 + 4] +
        c2_dx_kk[c2_i263 + 3];
    }

    for (c2_i264 = 0; c2_i264 < 3; c2_i264++) {
      chartInstance->c2_x_kk[c2_i264 + 7] = c2_x_kkm1[c2_i264 + 7] +
        c2_dx_kk[c2_i264 + 6];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 157U);
    for (c2_i265 = 0; c2_i265 < 54; c2_i265++) {
      c2_u_a[c2_i265] = c2_K_k[c2_i265];
    }

    for (c2_i266 = 0; c2_i266 < 36; c2_i266++) {
      c2_j_b[c2_i266] = c2_P_zkzk[c2_i266];
    }

    c2_f_eml_scalar_eg(chartInstance);
    c2_f_eml_scalar_eg(chartInstance);
    for (c2_i267 = 0; c2_i267 < 9; c2_i267++) {
      c2_i268 = 0;
      c2_i269 = 0;
      for (c2_i270 = 0; c2_i270 < 6; c2_i270++) {
        c2_n_y[c2_i268 + c2_i267] = 0.0;
        c2_i271 = 0;
        for (c2_i272 = 0; c2_i272 < 6; c2_i272++) {
          c2_n_y[c2_i268 + c2_i267] += c2_u_a[c2_i271 + c2_i267] *
            c2_j_b[c2_i272 + c2_i269];
          c2_i271 += 9;
        }

        c2_i268 += 9;
        c2_i269 += 6;
      }
    }

    c2_i273 = 0;
    for (c2_i274 = 0; c2_i274 < 9; c2_i274++) {
      c2_i275 = 0;
      for (c2_i276 = 0; c2_i276 < 6; c2_i276++) {
        c2_k_b[c2_i276 + c2_i273] = c2_K_k[c2_i275 + c2_i274];
        c2_i275 += 9;
      }

      c2_i273 += 6;
    }

    c2_g_eml_scalar_eg(chartInstance);
    c2_g_eml_scalar_eg(chartInstance);
    for (c2_i277 = 0; c2_i277 < 9; c2_i277++) {
      c2_i278 = 0;
      c2_i279 = 0;
      for (c2_i280 = 0; c2_i280 < 9; c2_i280++) {
        c2_d_hoistedGlobal[c2_i278 + c2_i277] = 0.0;
        c2_i281 = 0;
        for (c2_i282 = 0; c2_i282 < 6; c2_i282++) {
          c2_d_hoistedGlobal[c2_i278 + c2_i277] += c2_n_y[c2_i281 + c2_i277] *
            c2_k_b[c2_i282 + c2_i279];
          c2_i281 += 9;
        }

        c2_i278 += 9;
        c2_i279 += 6;
      }
    }

    for (c2_i283 = 0; c2_i283 < 81; c2_i283++) {
      c2_P_kk[c2_i283] = c2_P_kkm1[c2_i283] - c2_d_hoistedGlobal[c2_i283];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 162U);
  for (c2_i284 = 0; c2_i284 < 81; c2_i284++) {
    chartInstance->c2_P_km1km1[c2_i284] = c2_P_kk[c2_i284];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 163U);
  for (c2_i285 = 0; c2_i285 < 10; c2_i285++) {
    chartInstance->c2_x_km1km1[c2_i285] = chartInstance->c2_x_kk[c2_i285];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 166U);
  for (c2_i286 = 0; c2_i286 < 4; c2_i286++) {
    c2_dv44[c2_i286] = chartInstance->c2_q_s_c[c2_i286];
  }

  c2_quatinv(chartInstance, c2_dv44, c2_dv27);
  for (c2_i287 = 0; c2_i287 < 4; c2_i287++) {
    c2_dv45[c2_i287] = chartInstance->c2_x_kk[c2_i287];
  }

  for (c2_i288 = 0; c2_i288 < 4; c2_i288++) {
    c2_dv46[c2_i288] = c2_dv27[c2_i288];
  }

  c2_b_quatmultiply(chartInstance, c2_dv45, c2_dv46, c2_dv27);
  for (c2_i289 = 0; c2_i289 < 4; c2_i289++) {
    c2_Attitude_sensor[c2_i289] = c2_dv27[c2_i289];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 168U);
  for (c2_i290 = 0; c2_i290 < 4; c2_i290++) {
    c2_dv47[c2_i290] = chartInstance->c2_q_s_c[c2_i290];
  }

  c2_dv48[0] = 0.0;
  for (c2_i291 = 0; c2_i291 < 3; c2_i291++) {
    c2_dv48[c2_i291 + 1] = chartInstance->c2_x_kk[c2_i291 + 4];
  }

  c2_b_quatmultiply(chartInstance, c2_dv47, c2_dv48, c2_dv27);
  for (c2_i292 = 0; c2_i292 < 4; c2_i292++) {
    c2_dv49[c2_i292] = chartInstance->c2_q_s_c[c2_i292];
  }

  c2_quatinv(chartInstance, c2_dv49, c2_dv50);
  for (c2_i293 = 0; c2_i293 < 4; c2_i293++) {
    c2_dv51[c2_i293] = c2_dv27[c2_i293];
  }

  for (c2_i294 = 0; c2_i294 < 4; c2_i294++) {
    c2_dv52[c2_i294] = c2_dv50[c2_i294];
  }

  c2_b_quatmultiply(chartInstance, c2_dv51, c2_dv52, c2_dv27);
  for (c2_i295 = 0; c2_i295 < 4; c2_i295++) {
    c2_w_sensor_temp[c2_i295] = c2_dv27[c2_i295];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 169U);
  for (c2_i296 = 0; c2_i296 < 3; c2_i296++) {
    c2_w_sensor[c2_i296] = c2_w_sensor_temp[c2_i296 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 171U);
  for (c2_i297 = 0; c2_i297 < 4; c2_i297++) {
    c2_dv53[c2_i297] = chartInstance->c2_q_s_c[c2_i297];
  }

  c2_dv54[0] = 0.0;
  for (c2_i298 = 0; c2_i298 < 3; c2_i298++) {
    c2_dv54[c2_i298 + 1] = chartInstance->c2_x_kk[c2_i298 + 7];
  }

  c2_b_quatmultiply(chartInstance, c2_dv53, c2_dv54, c2_dv27);
  for (c2_i299 = 0; c2_i299 < 4; c2_i299++) {
    c2_dv55[c2_i299] = chartInstance->c2_q_s_c[c2_i299];
  }

  c2_quatinv(chartInstance, c2_dv55, c2_dv50);
  for (c2_i300 = 0; c2_i300 < 4; c2_i300++) {
    c2_dv56[c2_i300] = c2_dv27[c2_i300];
  }

  for (c2_i301 = 0; c2_i301 < 4; c2_i301++) {
    c2_dv57[c2_i301] = c2_dv50[c2_i301];
  }

  c2_b_quatmultiply(chartInstance, c2_dv56, c2_dv57, c2_dv27);
  for (c2_i302 = 0; c2_i302 < 4; c2_i302++) {
    c2_w_bias_sensor_temp[c2_i302] = c2_dv27[c2_i302];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 172U);
  for (c2_i303 = 0; c2_i303 < 3; c2_i303++) {
    c2_w_bias_sensor[c2_i303] = c2_w_bias_sensor_temp[c2_i303 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -172);
  sf_debug_symbol_scope_pop();
  for (c2_i304 = 0; c2_i304 < 4; c2_i304++) {
    (*c2_b_Attitude_sensor)[c2_i304] = c2_Attitude_sensor[c2_i304];
  }

  for (c2_i305 = 0; c2_i305 < 3; c2_i305++) {
    (*c2_b_w_sensor)[c2_i305] = c2_w_sensor[c2_i305];
  }

  for (c2_i306 = 0; c2_i306 < 3; c2_i306++) {
    (*c2_b_w_bias_sensor)[c2_i306] = c2_w_bias_sensor[c2_i306];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_SatelliteControlSystem
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Beta_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Beta, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Beta), &c2_thisId);
  sf_mex_destroy(&c2_b_Beta);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d5;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Beta_not_empty = FALSE;
  } else {
    chartInstance->c2_Beta_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d5, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d5;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Beta;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_Beta = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Beta), &c2_thisId);
  sf_mex_destroy(&c2_b_Beta);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_K_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_K, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K), &c2_thisId);
  sf_mex_destroy(&c2_b_K);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d6;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_K_not_empty = FALSE;
  } else {
    chartInstance->c2_K_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d6, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d6;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_K;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_K = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_K), &c2_thisId);
  sf_mex_destroy(&c2_b_K);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_alpha_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_e_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_alpha, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_alpha), &c2_thisId);
  sf_mex_destroy(&c2_b_alpha);
  return c2_y;
}

static real_T c2_f_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d7;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_alpha_not_empty = FALSE;
  } else {
    chartInstance->c2_alpha_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d7, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d7;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_alpha;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_alpha = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_alpha), &c2_thisId);
  sf_mex_destroy(&c2_b_alpha);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Lambda_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_g_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Lambda, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Lambda),
    &c2_thisId);
  sf_mex_destroy(&c2_b_Lambda);
  return c2_y;
}

static real_T c2_h_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d8;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Lambda_not_empty = FALSE;
  } else {
    chartInstance->c2_Lambda_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d8, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d8;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Lambda;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_Lambda = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Lambda),
    &c2_thisId);
  sf_mex_destroy(&c2_b_Lambda);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i307;
  int32_T c2_i308;
  int32_T c2_i309;
  real_T c2_b_inData[81];
  int32_T c2_i310;
  int32_T c2_i311;
  int32_T c2_i312;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i307 = 0;
  for (c2_i308 = 0; c2_i308 < 9; c2_i308++) {
    for (c2_i309 = 0; c2_i309 < 9; c2_i309++) {
      c2_b_inData[c2_i309 + c2_i307] = (*(real_T (*)[81])c2_inData)[c2_i309 +
        c2_i307];
    }

    c2_i307 += 9;
  }

  c2_i310 = 0;
  for (c2_i311 = 0; c2_i311 < 9; c2_i311++) {
    for (c2_i312 = 0; c2_i312 < 9; c2_i312++) {
      c2_u[c2_i312 + c2_i310] = c2_b_inData[c2_i312 + c2_i310];
    }

    c2_i310 += 9;
  }

  c2_y = NULL;
  if (!chartInstance->c2_Q_k_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Q_k, const char_T *c2_identifier, real_T
  c2_y[81])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_Q_k);
}

static void c2_j_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81])
{
  real_T c2_dv58[81];
  int32_T c2_i313;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Q_k_not_empty = FALSE;
  } else {
    chartInstance->c2_Q_k_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv58, 1, 0, 0U, 1, 0U, 2, 9,
                  9);
    for (c2_i313 = 0; c2_i313 < 81; c2_i313++) {
      c2_y[c2_i313] = c2_dv58[c2_i313];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Q_k;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i314;
  int32_T c2_i315;
  int32_T c2_i316;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_Q_k = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Q_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_Q_k);
  c2_i314 = 0;
  for (c2_i315 = 0; c2_i315 < 9; c2_i315++) {
    for (c2_i316 = 0; c2_i316 < 9; c2_i316++) {
      (*(real_T (*)[81])c2_outData)[c2_i316 + c2_i314] = c2_y[c2_i316 + c2_i314];
    }

    c2_i314 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i317;
  int32_T c2_i318;
  int32_T c2_i319;
  real_T c2_b_inData[36];
  int32_T c2_i320;
  int32_T c2_i321;
  int32_T c2_i322;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i317 = 0;
  for (c2_i318 = 0; c2_i318 < 6; c2_i318++) {
    for (c2_i319 = 0; c2_i319 < 6; c2_i319++) {
      c2_b_inData[c2_i319 + c2_i317] = (*(real_T (*)[36])c2_inData)[c2_i319 +
        c2_i317];
    }

    c2_i317 += 6;
  }

  c2_i320 = 0;
  for (c2_i321 = 0; c2_i321 < 6; c2_i321++) {
    for (c2_i322 = 0; c2_i322 < 6; c2_i322++) {
      c2_u[c2_i322 + c2_i320] = c2_b_inData[c2_i322 + c2_i320];
    }

    c2_i320 += 6;
  }

  c2_y = NULL;
  if (!chartInstance->c2_R_k_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_k_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_R_k, const char_T *c2_identifier, real_T
  c2_y[36])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_R_k);
}

static void c2_l_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36])
{
  real_T c2_dv59[36];
  int32_T c2_i323;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_R_k_not_empty = FALSE;
  } else {
    chartInstance->c2_R_k_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv59, 1, 0, 0U, 1, 0U, 2, 6,
                  6);
    for (c2_i323 = 0; c2_i323 < 36; c2_i323++) {
      c2_y[c2_i323] = c2_dv59[c2_i323];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_R_k;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i324;
  int32_T c2_i325;
  int32_T c2_i326;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_R_k = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_R_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_R_k);
  c2_i324 = 0;
  for (c2_i325 = 0; c2_i325 < 6; c2_i325++) {
    for (c2_i326 = 0; c2_i326 < 6; c2_i326++) {
      (*(real_T (*)[36])c2_outData)[c2_i326 + c2_i324] = c2_y[c2_i326 + c2_i324];
    }

    c2_i324 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_Ts_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_m_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_Ts, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Ts), &c2_thisId);
  sf_mex_destroy(&c2_b_Ts);
  return c2_y;
}

static real_T c2_n_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d9;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_Ts_not_empty = FALSE;
  } else {
    chartInstance->c2_Ts_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d9, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d9;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_Ts;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_Ts = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Ts), &c2_thisId);
  sf_mex_destroy(&c2_b_Ts);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i327;
  int32_T c2_i328;
  int32_T c2_i329;
  real_T c2_b_inData[9];
  int32_T c2_i330;
  int32_T c2_i331;
  int32_T c2_i332;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i327 = 0;
  for (c2_i328 = 0; c2_i328 < 3; c2_i328++) {
    for (c2_i329 = 0; c2_i329 < 3; c2_i329++) {
      c2_b_inData[c2_i329 + c2_i327] = (*(real_T (*)[9])c2_inData)[c2_i329 +
        c2_i327];
    }

    c2_i327 += 3;
  }

  c2_i330 = 0;
  for (c2_i331 = 0; c2_i331 < 3; c2_i331++) {
    for (c2_i332 = 0; c2_i332 < 3; c2_i332++) {
      c2_u[c2_i332 + c2_i330] = c2_b_inData[c2_i332 + c2_i330];
    }

    c2_i330 += 3;
  }

  c2_y = NULL;
  if (!chartInstance->c2_DCM_s2c_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_o_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_DCM_s2c, const char_T *c2_identifier,
  real_T c2_y[9])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_DCM_s2c), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_DCM_s2c);
}

static void c2_p_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv60[9];
  int32_T c2_i333;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_DCM_s2c_not_empty = FALSE;
  } else {
    chartInstance->c2_DCM_s2c_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv60, 1, 0, 0U, 1, 0U, 2, 3,
                  3);
    for (c2_i333 = 0; c2_i333 < 9; c2_i333++) {
      c2_y[c2_i333] = c2_dv60[c2_i333];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_DCM_s2c;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i334;
  int32_T c2_i335;
  int32_T c2_i336;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_DCM_s2c = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_DCM_s2c), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_DCM_s2c);
  c2_i334 = 0;
  for (c2_i335 = 0; c2_i335 < 3; c2_i335++) {
    for (c2_i336 = 0; c2_i336 < 3; c2_i336++) {
      (*(real_T (*)[9])c2_outData)[c2_i336 + c2_i334] = c2_y[c2_i336 + c2_i334];
    }

    c2_i334 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i337;
  real_T c2_b_inData[10];
  int32_T c2_i338;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i337 = 0; c2_i337 < 10; c2_i337++) {
    c2_b_inData[c2_i337] = (*(real_T (*)[10])c2_inData)[c2_i337];
  }

  for (c2_i338 = 0; c2_i338 < 10; c2_i338++) {
    c2_u[c2_i338] = c2_b_inData[c2_i338];
  }

  c2_y = NULL;
  if (!chartInstance->c2_x_kk_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_q_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_x_kk, const char_T *c2_identifier, real_T
  c2_y[10])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_x_kk);
}

static void c2_r_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10])
{
  real_T c2_dv61[10];
  int32_T c2_i339;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_x_kk_not_empty = FALSE;
  } else {
    chartInstance->c2_x_kk_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv61, 1, 0, 0U, 1, 0U, 1, 10);
    for (c2_i339 = 0; c2_i339 < 10; c2_i339++) {
      c2_y[c2_i339] = c2_dv61[c2_i339];
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
  int32_T c2_i340;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_x_kk = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_x_kk);
  for (c2_i340 = 0; c2_i340 < 10; c2_i340++) {
    (*(real_T (*)[10])c2_outData)[c2_i340] = c2_y[c2_i340];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i341;
  real_T c2_b_inData[4];
  int32_T c2_i342;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i341 = 0; c2_i341 < 4; c2_i341++) {
    c2_b_inData[c2_i341] = (*(real_T (*)[4])c2_inData)[c2_i341];
  }

  for (c2_i342 = 0; c2_i342 < 4; c2_i342++) {
    c2_u[c2_i342] = c2_b_inData[c2_i342];
  }

  c2_y = NULL;
  if (!chartInstance->c2_q_s_c_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_s_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_q_s_c, const char_T *c2_identifier, real_T
  c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_q_s_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_q_s_c);
}

static void c2_t_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv62[4];
  int32_T c2_i343;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_q_s_c_not_empty = FALSE;
  } else {
    chartInstance->c2_q_s_c_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv62, 1, 0, 0U, 1, 0U, 1, 4);
    for (c2_i343 = 0; c2_i343 < 4; c2_i343++) {
      c2_y[c2_i343] = c2_dv62[c2_i343];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_q_s_c;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i344;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_q_s_c = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_q_s_c), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_q_s_c);
  for (c2_i344 = 0; c2_i344 < 4; c2_i344++) {
    (*(real_T (*)[4])c2_outData)[c2_i344] = c2_y[c2_i344];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i345;
  real_T c2_b_inData[10];
  int32_T c2_i346;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i345 = 0; c2_i345 < 10; c2_i345++) {
    c2_b_inData[c2_i345] = (*(real_T (*)[10])c2_inData)[c2_i345];
  }

  for (c2_i346 = 0; c2_i346 < 10; c2_i346++) {
    c2_u[c2_i346] = c2_b_inData[c2_i346];
  }

  c2_y = NULL;
  if (!chartInstance->c2_x_km1km1_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_u_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_x_km1km1, const char_T *c2_identifier,
  real_T c2_y[10])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_x_km1km1);
}

static void c2_v_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10])
{
  real_T c2_dv63[10];
  int32_T c2_i347;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_x_km1km1_not_empty = FALSE;
  } else {
    chartInstance->c2_x_km1km1_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv63, 1, 0, 0U, 1, 0U, 1, 10);
    for (c2_i347 = 0; c2_i347 < 10; c2_i347++) {
      c2_y[c2_i347] = c2_dv63[c2_i347];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_x_km1km1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[10];
  int32_T c2_i348;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_x_km1km1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_x_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_x_km1km1);
  for (c2_i348 = 0; c2_i348 < 10; c2_i348++) {
    (*(real_T (*)[10])c2_outData)[c2_i348] = c2_y[c2_i348];
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
  real_T c2_b_inData[81];
  int32_T c2_i352;
  int32_T c2_i353;
  int32_T c2_i354;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i349 = 0;
  for (c2_i350 = 0; c2_i350 < 9; c2_i350++) {
    for (c2_i351 = 0; c2_i351 < 9; c2_i351++) {
      c2_b_inData[c2_i351 + c2_i349] = (*(real_T (*)[81])c2_inData)[c2_i351 +
        c2_i349];
    }

    c2_i349 += 9;
  }

  c2_i352 = 0;
  for (c2_i353 = 0; c2_i353 < 9; c2_i353++) {
    for (c2_i354 = 0; c2_i354 < 9; c2_i354++) {
      c2_u[c2_i354 + c2_i352] = c2_b_inData[c2_i354 + c2_i352];
    }

    c2_i352 += 9;
  }

  c2_y = NULL;
  if (!chartInstance->c2_P_km1km1_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_w_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_b_P_km1km1, const char_T *c2_identifier,
  real_T c2_y[81])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_P_km1km1);
}

static void c2_x_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81])
{
  real_T c2_dv64[81];
  int32_T c2_i355;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_P_km1km1_not_empty = FALSE;
  } else {
    chartInstance->c2_P_km1km1_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv64, 1, 0, 0U, 1, 0U, 2, 9,
                  9);
    for (c2_i355 = 0; c2_i355 < 81; c2_i355++) {
      c2_y[c2_i355] = c2_dv64[c2_i355];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_P_km1km1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i356;
  int32_T c2_i357;
  int32_T c2_i358;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_P_km1km1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_x_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P_km1km1), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_P_km1km1);
  c2_i356 = 0;
  for (c2_i357 = 0; c2_i357 < 9; c2_i357++) {
    for (c2_i358 = 0; c2_i358 < 9; c2_i358++) {
      (*(real_T (*)[81])c2_outData)[c2_i358 + c2_i356] = c2_y[c2_i358 + c2_i356];
    }

    c2_i356 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i359;
  real_T c2_b_inData[3];
  int32_T c2_i360;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i359 = 0; c2_i359 < 3; c2_i359++) {
    c2_b_inData[c2_i359] = (*(real_T (*)[3])c2_inData)[c2_i359];
  }

  for (c2_i360 = 0; c2_i360 < 3; c2_i360++) {
    c2_u[c2_i360] = c2_b_inData[c2_i360];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_y_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_w_bias_sensor, const char_T *c2_identifier,
  real_T c2_y[3])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_bias_sensor), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_w_bias_sensor);
}

static void c2_ab_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv65[3];
  int32_T c2_i361;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv65, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i361 = 0; c2_i361 < 3; c2_i361++) {
    c2_y[c2_i361] = c2_dv65[c2_i361];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_w_bias_sensor;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i362;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_w_bias_sensor = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ab_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_w_bias_sensor), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_w_bias_sensor);
  for (c2_i362 = 0; c2_i362 < 3; c2_i362++) {
    (*(real_T (*)[3])c2_outData)[c2_i362] = c2_y[c2_i362];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i363;
  real_T c2_b_inData[4];
  int32_T c2_i364;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i363 = 0; c2_i363 < 4; c2_i363++) {
    c2_b_inData[c2_i363] = (*(real_T (*)[4])c2_inData)[c2_i363];
  }

  for (c2_i364 = 0; c2_i364 < 4; c2_i364++) {
    c2_u[c2_i364] = c2_b_inData[c2_i364];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_bb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_Attitude_sensor, const char_T *c2_identifier,
  real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Attitude_sensor),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Attitude_sensor);
}

static void c2_cb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv66[4];
  int32_T c2_i365;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv66, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i365 = 0; c2_i365 < 4; c2_i365++) {
    c2_y[c2_i365] = c2_dv66[c2_i365];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Attitude_sensor;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i366;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_Attitude_sensor = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_cb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Attitude_sensor),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Attitude_sensor);
  for (c2_i366 = 0; c2_i366 < 4; c2_i366++) {
    (*(real_T (*)[4])c2_outData)[c2_i366] = c2_y[c2_i366];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_db_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d10;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d10, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d10;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_db_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout),
    &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i367;
  real_T c2_b_inData[9];
  int32_T c2_i368;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i367 = 0; c2_i367 < 9; c2_i367++) {
    c2_b_inData[c2_i367] = (*(real_T (*)[9])c2_inData)[c2_i367];
  }

  for (c2_i368 = 0; c2_i368 < 9; c2_i368++) {
    c2_u[c2_i368] = c2_b_inData[c2_i368];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_eb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv67[9];
  int32_T c2_i369;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv67, 1, 0, 0U, 1, 0U, 1, 9);
  for (c2_i369 = 0; c2_i369 < 9; c2_i369++) {
    c2_y[c2_i369] = c2_dv67[c2_i369];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dx_kk;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i370;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_dx_kk = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_eb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dx_kk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dx_kk);
  for (c2_i370 = 0; c2_i370 < 9; c2_i370++) {
    (*(real_T (*)[9])c2_outData)[c2_i370] = c2_y[c2_i370];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i371;
  int32_T c2_i372;
  int32_T c2_i373;
  real_T c2_b_inData[54];
  int32_T c2_i374;
  int32_T c2_i375;
  int32_T c2_i376;
  real_T c2_u[54];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i371 = 0;
  for (c2_i372 = 0; c2_i372 < 6; c2_i372++) {
    for (c2_i373 = 0; c2_i373 < 9; c2_i373++) {
      c2_b_inData[c2_i373 + c2_i371] = (*(real_T (*)[54])c2_inData)[c2_i373 +
        c2_i371];
    }

    c2_i371 += 9;
  }

  c2_i374 = 0;
  for (c2_i375 = 0; c2_i375 < 6; c2_i375++) {
    for (c2_i376 = 0; c2_i376 < 9; c2_i376++) {
      c2_u[c2_i376 + c2_i374] = c2_b_inData[c2_i376 + c2_i374];
    }

    c2_i374 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_fb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[54])
{
  real_T c2_dv68[54];
  int32_T c2_i377;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv68, 1, 0, 0U, 1, 0U, 2, 9, 6);
  for (c2_i377 = 0; c2_i377 < 54; c2_i377++) {
    c2_y[c2_i377] = c2_dv68[c2_i377];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_K_k;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[54];
  int32_T c2_i378;
  int32_T c2_i379;
  int32_T c2_i380;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_K_k = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_fb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_K_k), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_K_k);
  c2_i378 = 0;
  for (c2_i379 = 0; c2_i379 < 6; c2_i379++) {
    for (c2_i380 = 0; c2_i380 < 9; c2_i380++) {
      (*(real_T (*)[54])c2_outData)[c2_i380 + c2_i378] = c2_y[c2_i380 + c2_i378];
    }

    c2_i378 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i381;
  int32_T c2_i382;
  int32_T c2_i383;
  real_T c2_b_inData[36];
  int32_T c2_i384;
  int32_T c2_i385;
  int32_T c2_i386;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i381 = 0;
  for (c2_i382 = 0; c2_i382 < 6; c2_i382++) {
    for (c2_i383 = 0; c2_i383 < 6; c2_i383++) {
      c2_b_inData[c2_i383 + c2_i381] = (*(real_T (*)[36])c2_inData)[c2_i383 +
        c2_i381];
    }

    c2_i381 += 6;
  }

  c2_i384 = 0;
  for (c2_i385 = 0; c2_i385 < 6; c2_i385++) {
    for (c2_i386 = 0; c2_i386 < 6; c2_i386++) {
      c2_u[c2_i386 + c2_i384] = c2_b_inData[c2_i386 + c2_i384];
    }

    c2_i384 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_gb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36])
{
  real_T c2_dv69[36];
  int32_T c2_i387;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv69, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c2_i387 = 0; c2_i387 < 36; c2_i387++) {
    c2_y[c2_i387] = c2_dv69[c2_i387];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_P_zkzk;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i388;
  int32_T c2_i389;
  int32_T c2_i390;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_P_zkzk = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_gb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_P_zkzk), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_P_zkzk);
  c2_i388 = 0;
  for (c2_i389 = 0; c2_i389 < 6; c2_i389++) {
    for (c2_i390 = 0; c2_i390 < 6; c2_i390++) {
      (*(real_T (*)[36])c2_outData)[c2_i390 + c2_i388] = c2_y[c2_i390 + c2_i388];
    }

    c2_i388 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_s_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i391;
  real_T c2_b_inData[6];
  int32_T c2_i392;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i391 = 0; c2_i391 < 6; c2_i391++) {
    c2_b_inData[c2_i391] = (*(real_T (*)[6])c2_inData)[c2_i391];
  }

  for (c2_i392 = 0; c2_i392 < 6; c2_i392++) {
    c2_u[c2_i392] = c2_b_inData[c2_i392];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_hb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6])
{
  real_T c2_dv70[6];
  int32_T c2_i393;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv70, 1, 0, 0U, 1, 0U, 1, 6);
  for (c2_i393 = 0; c2_i393 < 6; c2_i393++) {
    c2_y[c2_i393] = c2_dv70[c2_i393];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_s_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_z_kkm1_control;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i394;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_z_kkm1_control = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_hb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_z_kkm1_control),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_z_kkm1_control);
  for (c2_i394 = 0; c2_i394 < 6; c2_i394++) {
    (*(real_T (*)[6])c2_outData)[c2_i394] = c2_y[c2_i394];
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
  real_T c2_b_inData[114];
  int32_T c2_i398;
  int32_T c2_i399;
  int32_T c2_i400;
  real_T c2_u[114];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i395 = 0;
  for (c2_i396 = 0; c2_i396 < 19; c2_i396++) {
    for (c2_i397 = 0; c2_i397 < 6; c2_i397++) {
      c2_b_inData[c2_i397 + c2_i395] = (*(real_T (*)[114])c2_inData)[c2_i397 +
        c2_i395];
    }

    c2_i395 += 6;
  }

  c2_i398 = 0;
  for (c2_i399 = 0; c2_i399 < 19; c2_i399++) {
    for (c2_i400 = 0; c2_i400 < 6; c2_i400++) {
      c2_u[c2_i400 + c2_i398] = c2_b_inData[c2_i400 + c2_i398];
    }

    c2_i398 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 19), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_ib_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[114])
{
  real_T c2_dv71[114];
  int32_T c2_i401;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv71, 1, 0, 0U, 1, 0U, 2, 6,
                19);
  for (c2_i401 = 0; c2_i401 < 114; c2_i401++) {
    c2_y[c2_i401] = c2_dv71[c2_i401];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_t_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Z_kkm1_control;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[114];
  int32_T c2_i402;
  int32_T c2_i403;
  int32_T c2_i404;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_Z_kkm1_control = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ib_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Z_kkm1_control),
    &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Z_kkm1_control);
  c2_i402 = 0;
  for (c2_i403 = 0; c2_i403 < 19; c2_i403++) {
    for (c2_i404 = 0; c2_i404 < 6; c2_i404++) {
      (*(real_T (*)[114])c2_outData)[c2_i404 + c2_i402] = c2_y[c2_i404 + c2_i402];
    }

    c2_i402 += 6;
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
  real_T c2_b_inData[81];
  int32_T c2_i408;
  int32_T c2_i409;
  int32_T c2_i410;
  real_T c2_u[81];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i405 = 0;
  for (c2_i406 = 0; c2_i406 < 9; c2_i406++) {
    for (c2_i407 = 0; c2_i407 < 9; c2_i407++) {
      c2_b_inData[c2_i407 + c2_i405] = (*(real_T (*)[81])c2_inData)[c2_i407 +
        c2_i405];
    }

    c2_i405 += 9;
  }

  c2_i408 = 0;
  for (c2_i409 = 0; c2_i409 < 9; c2_i409++) {
    for (c2_i410 = 0; c2_i410 < 9; c2_i410++) {
      c2_u[c2_i410 + c2_i408] = c2_b_inData[c2_i410 + c2_i408];
    }

    c2_i408 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_jb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[81])
{
  real_T c2_dv72[81];
  int32_T c2_i411;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv72, 1, 0, 0U, 1, 0U, 2, 9, 9);
  for (c2_i411 = 0; c2_i411 < 81; c2_i411++) {
    c2_y[c2_i411] = c2_dv72[c2_i411];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_u_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_P_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[81];
  int32_T c2_i412;
  int32_T c2_i413;
  int32_T c2_i414;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_P_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_jb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_P_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_P_kkm1);
  c2_i412 = 0;
  for (c2_i413 = 0; c2_i413 < 9; c2_i413++) {
    for (c2_i414 = 0; c2_i414 < 9; c2_i414++) {
      (*(real_T (*)[81])c2_outData)[c2_i414 + c2_i412] = c2_y[c2_i414 + c2_i412];
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
  real_T c2_b_inData[171];
  int32_T c2_i418;
  int32_T c2_i419;
  int32_T c2_i420;
  real_T c2_u[171];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i415 = 0;
  for (c2_i416 = 0; c2_i416 < 19; c2_i416++) {
    for (c2_i417 = 0; c2_i417 < 9; c2_i417++) {
      c2_b_inData[c2_i417 + c2_i415] = (*(real_T (*)[171])c2_inData)[c2_i417 +
        c2_i415];
    }

    c2_i415 += 9;
  }

  c2_i418 = 0;
  for (c2_i419 = 0; c2_i419 < 19; c2_i419++) {
    for (c2_i420 = 0; c2_i420 < 9; c2_i420++) {
      c2_u[c2_i420 + c2_i418] = c2_b_inData[c2_i420 + c2_i418];
    }

    c2_i418 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 19), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_kb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[171])
{
  real_T c2_dv73[171];
  int32_T c2_i421;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv73, 1, 0, 0U, 1, 0U, 2, 9,
                19);
  for (c2_i421 = 0; c2_i421 < 171; c2_i421++) {
    c2_y[c2_i421] = c2_dv73[c2_i421];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_v_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dX_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[171];
  int32_T c2_i422;
  int32_T c2_i423;
  int32_T c2_i424;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_dX_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_kb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dX_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dX_kkm1);
  c2_i422 = 0;
  for (c2_i423 = 0; c2_i423 < 19; c2_i423++) {
    for (c2_i424 = 0; c2_i424 < 9; c2_i424++) {
      (*(real_T (*)[171])c2_outData)[c2_i424 + c2_i422] = c2_y[c2_i424 + c2_i422];
    }

    c2_i422 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_w_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i425;
  int32_T c2_i426;
  int32_T c2_i427;
  real_T c2_b_inData[76];
  int32_T c2_i428;
  int32_T c2_i429;
  int32_T c2_i430;
  real_T c2_u[76];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i425 = 0;
  for (c2_i426 = 0; c2_i426 < 19; c2_i426++) {
    for (c2_i427 = 0; c2_i427 < 4; c2_i427++) {
      c2_b_inData[c2_i427 + c2_i425] = (*(real_T (*)[76])c2_inData)[c2_i427 +
        c2_i425];
    }

    c2_i425 += 4;
  }

  c2_i428 = 0;
  for (c2_i429 = 0; c2_i429 < 19; c2_i429++) {
    for (c2_i430 = 0; c2_i430 < 4; c2_i430++) {
      c2_u[c2_i430 + c2_i428] = c2_b_inData[c2_i430 + c2_i428];
    }

    c2_i428 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 19), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_lb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[76])
{
  real_T c2_dv74[76];
  int32_T c2_i431;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv74, 1, 0, 0U, 1, 0U, 2, 4,
                19);
  for (c2_i431 = 0; c2_i431 < 76; c2_i431++) {
    c2_y[c2_i431] = c2_dv74[c2_i431];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_w_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dX_kkm1_temp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[76];
  int32_T c2_i432;
  int32_T c2_i433;
  int32_T c2_i434;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_dX_kkm1_temp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_lb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dX_kkm1_temp), &c2_thisId,
    c2_y);
  sf_mex_destroy(&c2_dX_kkm1_temp);
  c2_i432 = 0;
  for (c2_i433 = 0; c2_i433 < 19; c2_i433++) {
    for (c2_i434 = 0; c2_i434 < 4; c2_i434++) {
      (*(real_T (*)[76])c2_outData)[c2_i434 + c2_i432] = c2_y[c2_i434 + c2_i432];
    }

    c2_i432 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_x_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i435;
  real_T c2_b_inData[10];
  int32_T c2_i436;
  real_T c2_u[10];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i435 = 0; c2_i435 < 10; c2_i435++) {
    c2_b_inData[c2_i435] = (*(real_T (*)[10])c2_inData)[c2_i435];
  }

  for (c2_i436 = 0; c2_i436 < 10; c2_i436++) {
    c2_u[c2_i436] = c2_b_inData[c2_i436];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 10), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_mb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[10])
{
  real_T c2_dv75[10];
  int32_T c2_i437;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv75, 1, 0, 0U, 1, 0U, 1, 10);
  for (c2_i437 = 0; c2_i437 < 10; c2_i437++) {
    c2_y[c2_i437] = c2_dv75[c2_i437];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_x_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_x_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[10];
  int32_T c2_i438;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_x_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_mb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_x_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_x_kkm1);
  for (c2_i438 = 0; c2_i438 < 10; c2_i438++) {
    (*(real_T (*)[10])c2_outData)[c2_i438] = c2_y[c2_i438];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_y_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i439;
  int32_T c2_i440;
  int32_T c2_i441;
  real_T c2_b_inData[190];
  int32_T c2_i442;
  int32_T c2_i443;
  int32_T c2_i444;
  real_T c2_u[190];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i439 = 0;
  for (c2_i440 = 0; c2_i440 < 19; c2_i440++) {
    for (c2_i441 = 0; c2_i441 < 10; c2_i441++) {
      c2_b_inData[c2_i441 + c2_i439] = (*(real_T (*)[190])c2_inData)[c2_i441 +
        c2_i439];
    }

    c2_i439 += 10;
  }

  c2_i442 = 0;
  for (c2_i443 = 0; c2_i443 < 19; c2_i443++) {
    for (c2_i444 = 0; c2_i444 < 10; c2_i444++) {
      c2_u[c2_i444 + c2_i442] = c2_b_inData[c2_i444 + c2_i442];
    }

    c2_i442 += 10;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 10, 19), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_nb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[190])
{
  real_T c2_dv76[190];
  int32_T c2_i445;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv76, 1, 0, 0U, 1, 0U, 2, 10,
                19);
  for (c2_i445 = 0; c2_i445 < 190; c2_i445++) {
    c2_y[c2_i445] = c2_dv76[c2_i445];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_y_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_X_kkm1;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[190];
  int32_T c2_i446;
  int32_T c2_i447;
  int32_T c2_i448;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_X_kkm1 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_nb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_X_kkm1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_X_kkm1);
  c2_i446 = 0;
  for (c2_i447 = 0; c2_i447 < 19; c2_i447++) {
    for (c2_i448 = 0; c2_i448 < 10; c2_i448++) {
      (*(real_T (*)[190])c2_outData)[c2_i448 + c2_i446] = c2_y[c2_i448 + c2_i446];
    }

    c2_i446 += 10;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_ab_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i449;
  int32_T c2_i450;
  int32_T c2_i451;
  real_T c2_b_inData[9];
  int32_T c2_i452;
  int32_T c2_i453;
  int32_T c2_i454;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i449 = 0;
  for (c2_i450 = 0; c2_i450 < 3; c2_i450++) {
    for (c2_i451 = 0; c2_i451 < 3; c2_i451++) {
      c2_b_inData[c2_i451 + c2_i449] = (*(real_T (*)[9])c2_inData)[c2_i451 +
        c2_i449];
    }

    c2_i449 += 3;
  }

  c2_i452 = 0;
  for (c2_i453 = 0; c2_i453 < 3; c2_i453++) {
    for (c2_i454 = 0; c2_i454 < 3; c2_i454++) {
      c2_u[c2_i454 + c2_i452] = c2_b_inData[c2_i454 + c2_i452];
    }

    c2_i452 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_ob_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv77[9];
  int32_T c2_i455;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv77, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i455 = 0; c2_i455 < 9; c2_i455++) {
    c2_y[c2_i455] = c2_dv77[c2_i455];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_ab_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dcm;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i456;
  int32_T c2_i457;
  int32_T c2_i458;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_dcm = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ob_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dcm), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dcm);
  c2_i456 = 0;
  for (c2_i457 = 0; c2_i457 < 3; c2_i457++) {
    for (c2_i458 = 0; c2_i458 < 3; c2_i458++) {
      (*(real_T (*)[9])c2_outData)[c2_i458 + c2_i456] = c2_y[c2_i458 + c2_i456];
    }

    c2_i456 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_bb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i459;
  real_T c2_b_inData[4];
  int32_T c2_i460;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i459 = 0; c2_i459 < 4; c2_i459++) {
    c2_b_inData[c2_i459] = (*(real_T (*)[4])c2_inData)[c2_i459];
  }

  for (c2_i460 = 0; c2_i460 < 4; c2_i460++) {
    c2_u[c2_i460] = c2_b_inData[c2_i460];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_pb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv78[4];
  int32_T c2_i461;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv78, 1, 0, 0U, 1, 0U, 2, 1, 4);
  for (c2_i461 = 0; c2_i461 < 4; c2_i461++) {
    c2_y[c2_i461] = c2_dv78[c2_i461];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_bb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_qin;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i462;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_qin = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_pb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_qin), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_qin);
  for (c2_i462 = 0; c2_i462 < 4; c2_i462++) {
    (*(real_T (*)[4])c2_outData)[c2_i462] = c2_y[c2_i462];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_cb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i463;
  int32_T c2_i464;
  int32_T c2_i465;
  real_T c2_b_inData[76];
  int32_T c2_i466;
  int32_T c2_i467;
  int32_T c2_i468;
  real_T c2_u[76];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i463 = 0;
  for (c2_i464 = 0; c2_i464 < 4; c2_i464++) {
    for (c2_i465 = 0; c2_i465 < 19; c2_i465++) {
      c2_b_inData[c2_i465 + c2_i463] = (*(real_T (*)[76])c2_inData)[c2_i465 +
        c2_i463];
    }

    c2_i463 += 19;
  }

  c2_i466 = 0;
  for (c2_i467 = 0; c2_i467 < 4; c2_i467++) {
    for (c2_i468 = 0; c2_i468 < 19; c2_i468++) {
      c2_u[c2_i468 + c2_i466] = c2_b_inData[c2_i468 + c2_i466];
    }

    c2_i466 += 19;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 19, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_qb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[76])
{
  real_T c2_dv79[76];
  int32_T c2_i469;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv79, 1, 0, 0U, 1, 0U, 2, 19,
                4);
  for (c2_i469 = 0; c2_i469 < 76; c2_i469++) {
    c2_y[c2_i469] = c2_dv79[c2_i469];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_cb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_qout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[76];
  int32_T c2_i470;
  int32_T c2_i471;
  int32_T c2_i472;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_qout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_qb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_qout), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_qout);
  c2_i470 = 0;
  for (c2_i471 = 0; c2_i471 < 4; c2_i471++) {
    for (c2_i472 = 0; c2_i472 < 19; c2_i472++) {
      (*(real_T (*)[76])c2_outData)[c2_i472 + c2_i470] = c2_y[c2_i472 + c2_i470];
    }

    c2_i470 += 19;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_db_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i473;
  real_T c2_b_inData[19];
  int32_T c2_i474;
  real_T c2_u[19];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i473 = 0; c2_i473 < 19; c2_i473++) {
    c2_b_inData[c2_i473] = (*(real_T (*)[19])c2_inData)[c2_i473];
  }

  for (c2_i474 = 0; c2_i474 < 19; c2_i474++) {
    c2_u[c2_i474] = c2_b_inData[c2_i474];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 19), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_rb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[19])
{
  real_T c2_dv80[19];
  int32_T c2_i475;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv80, 1, 0, 0U, 1, 0U, 1, 19);
  for (c2_i475 = 0; c2_i475 < 19; c2_i475++) {
    c2_y[c2_i475] = c2_dv80[c2_i475];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_db_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_scalar;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[19];
  int32_T c2_i476;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_scalar = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_rb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_scalar), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_scalar);
  for (c2_i476 = 0; c2_i476 < 19; c2_i476++) {
    (*(real_T (*)[19])c2_outData)[c2_i476] = c2_y[c2_i476];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_eb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i477;
  int32_T c2_i478;
  int32_T c2_i479;
  real_T c2_b_inData[57];
  int32_T c2_i480;
  int32_T c2_i481;
  int32_T c2_i482;
  real_T c2_u[57];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i477 = 0;
  for (c2_i478 = 0; c2_i478 < 3; c2_i478++) {
    for (c2_i479 = 0; c2_i479 < 19; c2_i479++) {
      c2_b_inData[c2_i479 + c2_i477] = (*(real_T (*)[57])c2_inData)[c2_i479 +
        c2_i477];
    }

    c2_i477 += 19;
  }

  c2_i480 = 0;
  for (c2_i481 = 0; c2_i481 < 3; c2_i481++) {
    for (c2_i482 = 0; c2_i482 < 19; c2_i482++) {
      c2_u[c2_i482 + c2_i480] = c2_b_inData[c2_i482 + c2_i480];
    }

    c2_i480 += 19;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 19, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_sb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[57])
{
  real_T c2_dv81[57];
  int32_T c2_i483;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv81, 1, 0, 0U, 1, 0U, 2, 19,
                3);
  for (c2_i483 = 0; c2_i483 < 57; c2_i483++) {
    c2_y[c2_i483] = c2_dv81[c2_i483];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_eb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_vec;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[57];
  int32_T c2_i484;
  int32_T c2_i485;
  int32_T c2_i486;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_vec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_sb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_vec), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_vec);
  c2_i484 = 0;
  for (c2_i485 = 0; c2_i485 < 3; c2_i485++) {
    for (c2_i486 = 0; c2_i486 < 19; c2_i486++) {
      (*(real_T (*)[57])c2_outData)[c2_i486 + c2_i484] = c2_y[c2_i486 + c2_i484];
    }

    c2_i484 += 19;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_fb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i487;
  int32_T c2_i488;
  int32_T c2_i489;
  real_T c2_b_inData[16];
  int32_T c2_i490;
  int32_T c2_i491;
  int32_T c2_i492;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i487 = 0;
  for (c2_i488 = 0; c2_i488 < 4; c2_i488++) {
    for (c2_i489 = 0; c2_i489 < 4; c2_i489++) {
      c2_b_inData[c2_i489 + c2_i487] = (*(real_T (*)[16])c2_inData)[c2_i489 +
        c2_i487];
    }

    c2_i487 += 4;
  }

  c2_i490 = 0;
  for (c2_i491 = 0; c2_i491 < 4; c2_i491++) {
    for (c2_i492 = 0; c2_i492 < 4; c2_i492++) {
      c2_u[c2_i492 + c2_i490] = c2_b_inData[c2_i492 + c2_i490];
    }

    c2_i490 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_tb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv82[16];
  int32_T c2_i493;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv82, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c2_i493 = 0; c2_i493 < 16; c2_i493++) {
    c2_y[c2_i493] = c2_dv82[c2_i493];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_fb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_W;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i494;
  int32_T c2_i495;
  int32_T c2_i496;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_W = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_tb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_W), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_W);
  c2_i494 = 0;
  for (c2_i495 = 0; c2_i495 < 4; c2_i495++) {
    for (c2_i496 = 0; c2_i496 < 4; c2_i496++) {
      (*(real_T (*)[16])c2_outData)[c2_i496 + c2_i494] = c2_y[c2_i496 + c2_i494];
    }

    c2_i494 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_gb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i497;
  real_T c2_b_inData[3];
  int32_T c2_i498;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i497 = 0; c2_i497 < 3; c2_i497++) {
    c2_b_inData[c2_i497] = (*(real_T (*)[3])c2_inData)[c2_i497];
  }

  for (c2_i498 = 0; c2_i498 < 3; c2_i498++) {
    c2_u[c2_i498] = c2_b_inData[c2_i498];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_ub_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv83[3];
  int32_T c2_i499;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv83, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c2_i499 = 0; c2_i499 < 3; c2_i499++) {
    c2_y[c2_i499] = c2_dv83[c2_i499];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_gb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_vec;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i500;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_vec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_ub_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_vec), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_vec);
  for (c2_i500 = 0; c2_i500 < 3; c2_i500++) {
    (*(real_T (*)[3])c2_outData)[c2_i500] = c2_y[c2_i500];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_SatelliteControlSystem_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[214];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i501;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  c2_d_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 214), FALSE);
  for (c2_i501 = 0; c2_i501 < 214; c2_i501++) {
    c2_r0 = &c2_info[c2_i501];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i501);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i501);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[214])
{
  c2_info[0].context = "";
  c2_info[0].name = "power";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[0].fileTimeLo = 1336543696U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[1].name = "eml_scalar_eg";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[1].fileTimeLo = 1286840396U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[2].name = "eml_scalexp_alloc";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[2].fileTimeLo = 1330630034U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[3].name = "floor";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[3].fileTimeLo = 1286840342U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[4].name = "eml_scalar_floor";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[4].fileTimeLo = 1286840326U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "";
  c2_info[5].name = "sum";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[5].fileTimeLo = 1314758212U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[6].name = "isequal";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[6].fileTimeLo = 1286840358U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[7].name = "eml_isequal_core";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[7].fileTimeLo = 1286840386U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[8].name = "eml_const_nonsingleton_dim";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c2_info[8].fileTimeLo = 1286840296U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[9].name = "eml_scalar_eg";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[9].fileTimeLo = 1286840396U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[10].name = "eml_index_class";
  c2_info[10].dominantType = "";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[10].fileTimeLo = 1323192178U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[11].name = "eml_int_forloop_overflow_check";
  c2_info[11].dominantType = "";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[11].fileTimeLo = 1332186672U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[12].name = "intmax";
  c2_info[12].dominantType = "char";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[12].fileTimeLo = 1311276916U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context = "";
  c2_info[13].name = "sqrt";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[13].fileTimeLo = 1286840352U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[14].name = "eml_error";
  c2_info[14].dominantType = "char";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[14].fileTimeLo = 1305339600U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[15].name = "eml_scalar_sqrt";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[15].fileTimeLo = 1286840338U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context = "";
  c2_info[16].name = "rdivide";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[16].fileTimeLo = 1286840444U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[17].name = "eml_div";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[17].fileTimeLo = 1313369410U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context = "";
  c2_info[18].name = "diag";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[18].fileTimeLo = 1286840286U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[19].name = "eml_index_class";
  c2_info[19].dominantType = "";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[19].fileTimeLo = 1323192178U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[20].name = "eml_index_plus";
  c2_info[20].dominantType = "coder.internal.indexInt";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[20].fileTimeLo = 1286840378U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[21].name = "eml_index_class";
  c2_info[21].dominantType = "";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[21].fileTimeLo = 1323192178U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[22].name = "eml_scalar_eg";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[22].fileTimeLo = 1286840396U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[23].name = "eml_int_forloop_overflow_check";
  c2_info[23].dominantType = "";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[23].fileTimeLo = 1332186672U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context = "";
  c2_info[24].name = "mtimes";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[24].fileTimeLo = 1289541292U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context = "";
  c2_info[25].name = "mldivide";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[25].fileTimeLo = 1342832544U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 1319751566U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[26].name = "eml_lusolve";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[26].fileTimeLo = 1309472796U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m";
  c2_info[27].name = "eml_index_class";
  c2_info[27].dominantType = "";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[27].fileTimeLo = 1323192178U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3";
  c2_info[28].name = "eml_index_class";
  c2_info[28].dominantType = "";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[28].fileTimeLo = 1323192178U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3";
  c2_info[29].name = "eml_xcabs1";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[29].fileTimeLo = 1286840306U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[30].name = "abs";
  c2_info[30].dominantType = "double";
  c2_info[30].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[30].fileTimeLo = 1286840294U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[31].name = "eml_scalar_abs";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[31].fileTimeLo = 1286840312U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3";
  c2_info[32].name = "eml_div";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[32].fileTimeLo = 1313369410U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3";
  c2_info[33].name = "mtimes";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[33].fileTimeLo = 1289541292U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!warn_singular";
  c2_info[34].name = "eml_warning";
  c2_info[34].dominantType = "char";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[34].fileTimeLo = 1286840402U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolve3x3";
  c2_info[35].name = "eml_scalar_eg";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[35].fileTimeLo = 1286840396U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context = "";
  c2_info[36].name = "mpower";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[36].fileTimeLo = 1286840442U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[37].name = "power";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[37].fileTimeLo = 1336543696U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context = "";
  c2_info[38].name = "chol";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[38].fileTimeLo = 1286840422U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[39].name = "eml_index_class";
  c2_info[39].dominantType = "";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[39].fileTimeLo = 1323192178U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[40].name = "eml_int_forloop_overflow_check";
  c2_info[40].dominantType = "";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[40].fileTimeLo = 1332186672U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[41].name = "eml_error";
  c2_info[41].dominantType = "char";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[41].fileTimeLo = 1305339600U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[42].name = "eml_xpotrf";
  c2_info[42].dominantType = "char";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xpotrf.m";
  c2_info[42].fileTimeLo = 1286840408U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xpotrf.m";
  c2_info[43].name = "eml_lapack_xpotrf";
  c2_info[43].dominantType = "char";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xpotrf.m";
  c2_info[43].fileTimeLo = 1286840412U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xpotrf.m";
  c2_info[44].name = "eml_matlab_zpotrf";
  c2_info[44].dominantType = "char";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[44].fileTimeLo = 1286840424U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[45].name = "eml_index_class";
  c2_info[45].dominantType = "";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[45].fileTimeLo = 1323192178U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[46].name = "eml_scalar_eg";
  c2_info[46].dominantType = "double";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[46].fileTimeLo = 1286840396U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[47].name = "eml_int_forloop_overflow_check";
  c2_info[47].dominantType = "";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[47].fileTimeLo = 1332186672U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[48].name = "eml_index_minus";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[48].fileTimeLo = 1286840378U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[49].name = "eml_index_class";
  c2_info[49].dominantType = "";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[49].fileTimeLo = 1323192178U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[50].name = "eml_index_plus";
  c2_info[50].dominantType = "coder.internal.indexInt";
  c2_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[50].fileTimeLo = 1286840378U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[51].name = "eml_xdotc";
  c2_info[51].dominantType = "double";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[51].fileTimeLo = 1299098372U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[52].name = "eml_blas_inline";
  c2_info[52].dominantType = "";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[52].fileTimeLo = 1299098368U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[53].name = "eml_xdot";
  c2_info[53].dominantType = "double";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[53].fileTimeLo = 1299098372U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[54].name = "eml_blas_inline";
  c2_info[54].dominantType = "";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[54].fileTimeLo = 1299098368U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c2_info[55].name = "length";
  c2_info[55].dominantType = "double";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[55].fileTimeLo = 1303167806U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[56].name = "eml_index_class";
  c2_info[56].dominantType = "";
  c2_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[56].fileTimeLo = 1323192178U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[57].name = "eml_index_class";
  c2_info[57].dominantType = "";
  c2_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[57].fileTimeLo = 1323192178U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[58].name = "eml_refblas_xdot";
  c2_info[58].dominantType = "double";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[58].fileTimeLo = 1299098372U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[59].name = "eml_refblas_xdotx";
  c2_info[59].dominantType = "char";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[59].fileTimeLo = 1299098374U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[60].name = "eml_scalar_eg";
  c2_info[60].dominantType = "double";
  c2_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[60].fileTimeLo = 1286840396U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[61].name = "eml_index_class";
  c2_info[61].dominantType = "";
  c2_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[61].fileTimeLo = 1323192178U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[62].name = "eml_int_forloop_overflow_check";
  c2_info[62].dominantType = "";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[62].fileTimeLo = 1332186672U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[63].name = "eml_index_plus";
  c2_info[63].dominantType = "coder.internal.indexInt";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[63].fileTimeLo = 1286840378U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[214])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[64].name = "eml_index_minus";
  c2_info[64].dominantType = "coder.internal.indexInt";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[64].fileTimeLo = 1286840378U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[65].name = "eml_xgemv";
  c2_info[65].dominantType = "char";
  c2_info[65].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m";
  c2_info[65].fileTimeLo = 1299098374U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m";
  c2_info[66].name = "eml_blas_inline";
  c2_info[66].dominantType = "";
  c2_info[66].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[66].fileTimeLo = 1299098368U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[67].name = "length";
  c2_info[67].dominantType = "double";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[67].fileTimeLo = 1303167806U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[68].name = "intmax";
  c2_info[68].dominantType = "char";
  c2_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[68].fileTimeLo = 1311276916U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[69].name = "mtimes";
  c2_info[69].dominantType = "double";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[69].fileTimeLo = 1289541292U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[70].name = "eml_index_class";
  c2_info[70].dominantType = "";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[70].fileTimeLo = 1323192178U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[71].name = "eml_scalar_eg";
  c2_info[71].dominantType = "double";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[71].fileTimeLo = 1286840396U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[72].name = "eml_refblas_xgemv";
  c2_info[72].dominantType = "char";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[72].fileTimeLo = 1299098376U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[73].name = "eml_index_minus";
  c2_info[73].dominantType = "double";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[73].fileTimeLo = 1286840378U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[74].name = "eml_index_class";
  c2_info[74].dominantType = "";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[74].fileTimeLo = 1323192178U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[75].name = "eml_index_times";
  c2_info[75].dominantType = "coder.internal.indexInt";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[75].fileTimeLo = 1286840380U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[76].name = "eml_index_class";
  c2_info[76].dominantType = "";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[76].fileTimeLo = 1323192178U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[77].name = "eml_index_plus";
  c2_info[77].dominantType = "coder.internal.indexInt";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[77].fileTimeLo = 1286840378U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[78].name = "eml_int_forloop_overflow_check";
  c2_info[78].dominantType = "";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[78].fileTimeLo = 1332186672U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[79].name = "eml_scalar_eg";
  c2_info[79].dominantType = "double";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[79].fileTimeLo = 1286840396U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[80].name = "eml_div";
  c2_info[80].dominantType = "double";
  c2_info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[80].fileTimeLo = 1313369410U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zpotrf.m";
  c2_info[81].name = "eml_xscal";
  c2_info[81].dominantType = "double";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[81].fileTimeLo = 1299098376U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[82].name = "eml_blas_inline";
  c2_info[82].dominantType = "";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[82].fileTimeLo = 1299098368U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c2_info[83].name = "length";
  c2_info[83].dominantType = "double";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[83].fileTimeLo = 1303167806U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[84].name = "eml_index_class";
  c2_info[84].dominantType = "";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[84].fileTimeLo = 1323192178U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[85].name = "eml_scalar_eg";
  c2_info[85].dominantType = "double";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[85].fileTimeLo = 1286840396U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[86].name = "eml_refblas_xscal";
  c2_info[86].dominantType = "double";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[86].fileTimeLo = 1299098384U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[87].name = "eml_index_class";
  c2_info[87].dominantType = "";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[87].fileTimeLo = 1323192178U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[88].name = "eml_index_minus";
  c2_info[88].dominantType = "double";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[88].fileTimeLo = 1286840378U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[89].name = "eml_index_times";
  c2_info[89].dominantType = "coder.internal.indexInt";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[89].fileTimeLo = 1286840380U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[90].name = "eml_index_plus";
  c2_info[90].dominantType = "coder.internal.indexInt";
  c2_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[90].fileTimeLo = 1286840378U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[91].name = "eml_int_forloop_overflow_check";
  c2_info[91].dominantType = "";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[91].fileTimeLo = 1332186672U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[92].name = "eml_index_minus";
  c2_info[92].dominantType = "double";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[92].fileTimeLo = 1286840378U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/chol.m";
  c2_info[93].name = "eml_index_plus";
  c2_info[93].dominantType = "double";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[93].fileTimeLo = 1286840378U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[94].name = "eml_assert_valid_dim";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[94].fileTimeLo = 1286840294U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[95].name = "eml_scalar_floor";
  c2_info[95].dominantType = "double";
  c2_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[95].fileTimeLo = 1286840326U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[96].name = "eml_index_class";
  c2_info[96].dominantType = "";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[96].fileTimeLo = 1323192178U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[97].name = "intmax";
  c2_info[97].dominantType = "char";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[97].fileTimeLo = 1311276916U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[98].name = "eml_matrix_vstride";
  c2_info[98].dominantType = "double";
  c2_info[98].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m";
  c2_info[98].fileTimeLo = 1286840388U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m";
  c2_info[99].name = "eml_index_minus";
  c2_info[99].dominantType = "double";
  c2_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[99].fileTimeLo = 1286840378U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m";
  c2_info[100].name = "eml_index_class";
  c2_info[100].dominantType = "";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[100].fileTimeLo = 1323192178U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m";
  c2_info[101].name = "eml_size_prod";
  c2_info[101].dominantType = "double";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_prod.m";
  c2_info[101].fileTimeLo = 1286840398U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_prod.m";
  c2_info[102].name = "eml_index_class";
  c2_info[102].dominantType = "";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[102].fileTimeLo = 1323192178U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[103].name = "eml_matrix_npages";
  c2_info[103].dominantType = "double";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m";
  c2_info[103].fileTimeLo = 1286840386U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m";
  c2_info[104].name = "eml_index_plus";
  c2_info[104].dominantType = "double";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[104].fileTimeLo = 1286840378U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m";
  c2_info[105].name = "eml_index_class";
  c2_info[105].dominantType = "";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[105].fileTimeLo = 1323192178U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m";
  c2_info[106].name = "eml_size_prod";
  c2_info[106].dominantType = "double";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_prod.m";
  c2_info[106].fileTimeLo = 1286840398U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_prod.m";
  c2_info[107].name = "eml_index_times";
  c2_info[107].dominantType = "double";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[107].fileTimeLo = 1286840380U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[108].name = "eml_index_plus";
  c2_info[108].dominantType = "double";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[108].fileTimeLo = 1286840378U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[109].name = "eml_index_plus";
  c2_info[109].dominantType = "coder.internal.indexInt";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[109].fileTimeLo = 1286840378U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[110].name = "eml_index_class";
  c2_info[110].dominantType = "";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[110].fileTimeLo = 1323192178U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[111].name = "eml_scalar_eg";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[111].fileTimeLo = 1286840396U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[112].name = "eml_xgemm";
  c2_info[112].dominantType = "char";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[112].fileTimeLo = 1299098372U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[113].name = "eml_blas_inline";
  c2_info[113].dominantType = "";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[113].fileTimeLo = 1299098368U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[114].name = "mtimes";
  c2_info[114].dominantType = "double";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[114].fileTimeLo = 1289541292U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[115].name = "eml_index_class";
  c2_info[115].dominantType = "";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[115].fileTimeLo = 1323192178U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[116].name = "eml_scalar_eg";
  c2_info[116].dominantType = "double";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[116].fileTimeLo = 1286840396U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[117].name = "eml_refblas_xgemm";
  c2_info[117].dominantType = "char";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[117].fileTimeLo = 1299098374U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context = "";
  c2_info[118].name = "mrdivide";
  c2_info[118].dominantType = "double";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[118].fileTimeLo = 1342832544U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 1319751566U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[119].name = "rdivide";
  c2_info[119].dominantType = "double";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[119].fileTimeLo = 1286840444U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[120].name = "mldivide";
  c2_info[120].dominantType = "double";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[120].fileTimeLo = 1342832544U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 1319751566U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[121].name = "eml_index_class";
  c2_info[121].dominantType = "";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[121].fileTimeLo = 1323192178U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[122].name = "eml_xgetrf";
  c2_info[122].dominantType = "double";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[122].fileTimeLo = 1286840406U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c2_info[123].name = "eml_lapack_xgetrf";
  c2_info[123].dominantType = "double";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[123].fileTimeLo = 1286840410U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c2_info[124].name = "eml_matlab_zgetrf";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[124].fileTimeLo = 1302710594U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[125].name = "realmin";
  c2_info[125].dominantType = "char";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[125].fileTimeLo = 1307672842U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[126].name = "eml_realmin";
  c2_info[126].dominantType = "char";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[126].fileTimeLo = 1307672844U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[127].name = "eml_float_model";
  c2_info[127].dominantType = "char";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[127].fileTimeLo = 1326749596U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[214])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[128].name = "eps";
  c2_info[128].dominantType = "char";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[128].fileTimeLo = 1326749596U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[129].name = "eml_is_float_class";
  c2_info[129].dominantType = "char";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[129].fileTimeLo = 1286840382U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[130].name = "eml_eps";
  c2_info[130].dominantType = "char";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[130].fileTimeLo = 1326749596U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[131].name = "eml_float_model";
  c2_info[131].dominantType = "char";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[131].fileTimeLo = 1326749596U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[132].name = "min";
  c2_info[132].dominantType = "coder.internal.indexInt";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[132].fileTimeLo = 1311276918U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[133].name = "eml_min_or_max";
  c2_info[133].dominantType = "char";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[133].fileTimeLo = 1334093090U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[134].name = "eml_scalar_eg";
  c2_info[134].dominantType = "coder.internal.indexInt";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[134].fileTimeLo = 1286840396U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[135].name = "eml_scalexp_alloc";
  c2_info[135].dominantType = "coder.internal.indexInt";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[135].fileTimeLo = 1330630034U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[136].name = "eml_index_class";
  c2_info[136].dominantType = "";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[136].fileTimeLo = 1323192178U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[137].name = "eml_scalar_eg";
  c2_info[137].dominantType = "coder.internal.indexInt";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[137].fileTimeLo = 1286840396U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[138].name = "colon";
  c2_info[138].dominantType = "double";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[138].fileTimeLo = 1311276918U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 0U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[139].name = "floor";
  c2_info[139].dominantType = "double";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[139].fileTimeLo = 1286840342U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[140].name = "intmin";
  c2_info[140].dominantType = "char";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[140].fileTimeLo = 1311276918U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[141].name = "intmax";
  c2_info[141].dominantType = "char";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[141].fileTimeLo = 1311276916U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[142].name = "intmin";
  c2_info[142].dominantType = "char";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[142].fileTimeLo = 1311276918U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[143].name = "intmax";
  c2_info[143].dominantType = "char";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[143].fileTimeLo = 1311276916U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[144].name = "eml_isa_uint";
  c2_info[144].dominantType = "coder.internal.indexInt";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[144].fileTimeLo = 1286840384U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[145].name = "eml_unsigned_class";
  c2_info[145].dominantType = "char";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[145].fileTimeLo = 1323192180U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 0U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[146].name = "eml_index_class";
  c2_info[146].dominantType = "";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[146].fileTimeLo = 1323192178U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
  c2_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[147].name = "eml_index_class";
  c2_info[147].dominantType = "";
  c2_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[147].fileTimeLo = 1323192178U;
  c2_info[147].fileTimeHi = 0U;
  c2_info[147].mFileTimeLo = 0U;
  c2_info[147].mFileTimeHi = 0U;
  c2_info[148].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[148].name = "intmax";
  c2_info[148].dominantType = "char";
  c2_info[148].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[148].fileTimeLo = 1311276916U;
  c2_info[148].fileTimeHi = 0U;
  c2_info[148].mFileTimeLo = 0U;
  c2_info[148].mFileTimeHi = 0U;
  c2_info[149].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[149].name = "eml_isa_uint";
  c2_info[149].dominantType = "coder.internal.indexInt";
  c2_info[149].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[149].fileTimeLo = 1286840384U;
  c2_info[149].fileTimeHi = 0U;
  c2_info[149].mFileTimeLo = 0U;
  c2_info[149].mFileTimeHi = 0U;
  c2_info[150].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[150].name = "eml_index_plus";
  c2_info[150].dominantType = "double";
  c2_info[150].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[150].fileTimeLo = 1286840378U;
  c2_info[150].fileTimeHi = 0U;
  c2_info[150].mFileTimeLo = 0U;
  c2_info[150].mFileTimeHi = 0U;
  c2_info[151].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[151].name = "eml_int_forloop_overflow_check";
  c2_info[151].dominantType = "";
  c2_info[151].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[151].fileTimeLo = 1332186672U;
  c2_info[151].fileTimeHi = 0U;
  c2_info[151].mFileTimeLo = 0U;
  c2_info[151].mFileTimeHi = 0U;
  c2_info[152].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[152].name = "eml_index_class";
  c2_info[152].dominantType = "";
  c2_info[152].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[152].fileTimeLo = 1323192178U;
  c2_info[152].fileTimeHi = 0U;
  c2_info[152].mFileTimeLo = 0U;
  c2_info[152].mFileTimeHi = 0U;
  c2_info[153].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[153].name = "eml_index_plus";
  c2_info[153].dominantType = "double";
  c2_info[153].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[153].fileTimeLo = 1286840378U;
  c2_info[153].fileTimeHi = 0U;
  c2_info[153].mFileTimeLo = 0U;
  c2_info[153].mFileTimeHi = 0U;
  c2_info[154].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[154].name = "eml_int_forloop_overflow_check";
  c2_info[154].dominantType = "";
  c2_info[154].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[154].fileTimeLo = 1332186672U;
  c2_info[154].fileTimeHi = 0U;
  c2_info[154].mFileTimeLo = 0U;
  c2_info[154].mFileTimeHi = 0U;
  c2_info[155].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[155].name = "eml_index_minus";
  c2_info[155].dominantType = "double";
  c2_info[155].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[155].fileTimeLo = 1286840378U;
  c2_info[155].fileTimeHi = 0U;
  c2_info[155].mFileTimeLo = 0U;
  c2_info[155].mFileTimeHi = 0U;
  c2_info[156].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[156].name = "eml_index_minus";
  c2_info[156].dominantType = "coder.internal.indexInt";
  c2_info[156].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[156].fileTimeLo = 1286840378U;
  c2_info[156].fileTimeHi = 0U;
  c2_info[156].mFileTimeLo = 0U;
  c2_info[156].mFileTimeHi = 0U;
  c2_info[157].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[157].name = "eml_index_times";
  c2_info[157].dominantType = "coder.internal.indexInt";
  c2_info[157].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[157].fileTimeLo = 1286840380U;
  c2_info[157].fileTimeHi = 0U;
  c2_info[157].mFileTimeLo = 0U;
  c2_info[157].mFileTimeHi = 0U;
  c2_info[158].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[158].name = "eml_index_plus";
  c2_info[158].dominantType = "coder.internal.indexInt";
  c2_info[158].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[158].fileTimeLo = 1286840378U;
  c2_info[158].fileTimeHi = 0U;
  c2_info[158].mFileTimeLo = 0U;
  c2_info[158].mFileTimeHi = 0U;
  c2_info[159].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[159].name = "eml_ixamax";
  c2_info[159].dominantType = "double";
  c2_info[159].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[159].fileTimeLo = 1299098370U;
  c2_info[159].fileTimeHi = 0U;
  c2_info[159].mFileTimeLo = 0U;
  c2_info[159].mFileTimeHi = 0U;
  c2_info[160].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[160].name = "eml_blas_inline";
  c2_info[160].dominantType = "";
  c2_info[160].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[160].fileTimeLo = 1299098368U;
  c2_info[160].fileTimeHi = 0U;
  c2_info[160].mFileTimeLo = 0U;
  c2_info[160].mFileTimeHi = 0U;
  c2_info[161].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[161].name = "length";
  c2_info[161].dominantType = "double";
  c2_info[161].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[161].fileTimeLo = 1303167806U;
  c2_info[161].fileTimeHi = 0U;
  c2_info[161].mFileTimeLo = 0U;
  c2_info[161].mFileTimeHi = 0U;
  c2_info[162].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[162].name = "eml_index_class";
  c2_info[162].dominantType = "";
  c2_info[162].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[162].fileTimeLo = 1323192178U;
  c2_info[162].fileTimeHi = 0U;
  c2_info[162].mFileTimeLo = 0U;
  c2_info[162].mFileTimeHi = 0U;
  c2_info[163].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[163].name = "eml_refblas_ixamax";
  c2_info[163].dominantType = "double";
  c2_info[163].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[163].fileTimeLo = 1299098370U;
  c2_info[163].fileTimeHi = 0U;
  c2_info[163].mFileTimeLo = 0U;
  c2_info[163].mFileTimeHi = 0U;
  c2_info[164].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[164].name = "eml_index_class";
  c2_info[164].dominantType = "";
  c2_info[164].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[164].fileTimeLo = 1323192178U;
  c2_info[164].fileTimeHi = 0U;
  c2_info[164].mFileTimeLo = 0U;
  c2_info[164].mFileTimeHi = 0U;
  c2_info[165].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[165].name = "eml_xcabs1";
  c2_info[165].dominantType = "double";
  c2_info[165].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[165].fileTimeLo = 1286840306U;
  c2_info[165].fileTimeHi = 0U;
  c2_info[165].mFileTimeLo = 0U;
  c2_info[165].mFileTimeHi = 0U;
  c2_info[166].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[166].name = "eml_int_forloop_overflow_check";
  c2_info[166].dominantType = "";
  c2_info[166].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[166].fileTimeLo = 1332186672U;
  c2_info[166].fileTimeHi = 0U;
  c2_info[166].mFileTimeLo = 0U;
  c2_info[166].mFileTimeHi = 0U;
  c2_info[167].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[167].name = "eml_index_plus";
  c2_info[167].dominantType = "coder.internal.indexInt";
  c2_info[167].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[167].fileTimeLo = 1286840378U;
  c2_info[167].fileTimeHi = 0U;
  c2_info[167].mFileTimeLo = 0U;
  c2_info[167].mFileTimeHi = 0U;
  c2_info[168].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[168].name = "eml_xswap";
  c2_info[168].dominantType = "double";
  c2_info[168].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[168].fileTimeLo = 1299098378U;
  c2_info[168].fileTimeHi = 0U;
  c2_info[168].mFileTimeLo = 0U;
  c2_info[168].mFileTimeHi = 0U;
  c2_info[169].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[169].name = "eml_blas_inline";
  c2_info[169].dominantType = "";
  c2_info[169].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[169].fileTimeLo = 1299098368U;
  c2_info[169].fileTimeHi = 0U;
  c2_info[169].mFileTimeLo = 0U;
  c2_info[169].mFileTimeHi = 0U;
  c2_info[170].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[170].name = "eml_index_class";
  c2_info[170].dominantType = "";
  c2_info[170].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[170].fileTimeLo = 1323192178U;
  c2_info[170].fileTimeHi = 0U;
  c2_info[170].mFileTimeLo = 0U;
  c2_info[170].mFileTimeHi = 0U;
  c2_info[171].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[171].name = "eml_refblas_xswap";
  c2_info[171].dominantType = "double";
  c2_info[171].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[171].fileTimeLo = 1299098386U;
  c2_info[171].fileTimeHi = 0U;
  c2_info[171].mFileTimeLo = 0U;
  c2_info[171].mFileTimeHi = 0U;
  c2_info[172].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[172].name = "eml_index_class";
  c2_info[172].dominantType = "";
  c2_info[172].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[172].fileTimeLo = 1323192178U;
  c2_info[172].fileTimeHi = 0U;
  c2_info[172].mFileTimeLo = 0U;
  c2_info[172].mFileTimeHi = 0U;
  c2_info[173].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[173].name = "abs";
  c2_info[173].dominantType = "coder.internal.indexInt";
  c2_info[173].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[173].fileTimeLo = 1286840294U;
  c2_info[173].fileTimeHi = 0U;
  c2_info[173].mFileTimeLo = 0U;
  c2_info[173].mFileTimeHi = 0U;
  c2_info[174].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[174].name = "eml_scalar_abs";
  c2_info[174].dominantType = "coder.internal.indexInt";
  c2_info[174].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[174].fileTimeLo = 1286840312U;
  c2_info[174].fileTimeHi = 0U;
  c2_info[174].mFileTimeLo = 0U;
  c2_info[174].mFileTimeHi = 0U;
  c2_info[175].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[175].name = "eml_int_forloop_overflow_check";
  c2_info[175].dominantType = "";
  c2_info[175].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[175].fileTimeLo = 1332186672U;
  c2_info[175].fileTimeHi = 0U;
  c2_info[175].mFileTimeLo = 0U;
  c2_info[175].mFileTimeHi = 0U;
  c2_info[176].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[176].name = "eml_index_plus";
  c2_info[176].dominantType = "coder.internal.indexInt";
  c2_info[176].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[176].fileTimeLo = 1286840378U;
  c2_info[176].fileTimeHi = 0U;
  c2_info[176].mFileTimeLo = 0U;
  c2_info[176].mFileTimeHi = 0U;
  c2_info[177].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[177].name = "eml_div";
  c2_info[177].dominantType = "double";
  c2_info[177].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[177].fileTimeLo = 1313369410U;
  c2_info[177].fileTimeHi = 0U;
  c2_info[177].mFileTimeLo = 0U;
  c2_info[177].mFileTimeHi = 0U;
  c2_info[178].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c2_info[178].name = "eml_xgeru";
  c2_info[178].dominantType = "double";
  c2_info[178].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[178].fileTimeLo = 1299098374U;
  c2_info[178].fileTimeHi = 0U;
  c2_info[178].mFileTimeLo = 0U;
  c2_info[178].mFileTimeHi = 0U;
  c2_info[179].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[179].name = "eml_blas_inline";
  c2_info[179].dominantType = "";
  c2_info[179].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[179].fileTimeLo = 1299098368U;
  c2_info[179].fileTimeHi = 0U;
  c2_info[179].mFileTimeLo = 0U;
  c2_info[179].mFileTimeHi = 0U;
  c2_info[180].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c2_info[180].name = "eml_xger";
  c2_info[180].dominantType = "double";
  c2_info[180].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[180].fileTimeLo = 1299098374U;
  c2_info[180].fileTimeHi = 0U;
  c2_info[180].mFileTimeLo = 0U;
  c2_info[180].mFileTimeHi = 0U;
  c2_info[181].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[181].name = "eml_blas_inline";
  c2_info[181].dominantType = "";
  c2_info[181].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[181].fileTimeLo = 1299098368U;
  c2_info[181].fileTimeHi = 0U;
  c2_info[181].mFileTimeLo = 0U;
  c2_info[181].mFileTimeHi = 0U;
  c2_info[182].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[182].name = "intmax";
  c2_info[182].dominantType = "char";
  c2_info[182].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[182].fileTimeLo = 1311276916U;
  c2_info[182].fileTimeHi = 0U;
  c2_info[182].mFileTimeLo = 0U;
  c2_info[182].mFileTimeHi = 0U;
  c2_info[183].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[183].name = "min";
  c2_info[183].dominantType = "double";
  c2_info[183].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[183].fileTimeLo = 1311276918U;
  c2_info[183].fileTimeHi = 0U;
  c2_info[183].mFileTimeLo = 0U;
  c2_info[183].mFileTimeHi = 0U;
  c2_info[184].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[184].name = "eml_scalar_eg";
  c2_info[184].dominantType = "double";
  c2_info[184].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[184].fileTimeLo = 1286840396U;
  c2_info[184].fileTimeHi = 0U;
  c2_info[184].mFileTimeLo = 0U;
  c2_info[184].mFileTimeHi = 0U;
  c2_info[185].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[185].name = "eml_scalexp_alloc";
  c2_info[185].dominantType = "double";
  c2_info[185].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[185].fileTimeLo = 1330630034U;
  c2_info[185].fileTimeHi = 0U;
  c2_info[185].mFileTimeLo = 0U;
  c2_info[185].mFileTimeHi = 0U;
  c2_info[186].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[186].name = "eml_scalar_eg";
  c2_info[186].dominantType = "double";
  c2_info[186].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[186].fileTimeLo = 1286840396U;
  c2_info[186].fileTimeHi = 0U;
  c2_info[186].mFileTimeLo = 0U;
  c2_info[186].mFileTimeHi = 0U;
  c2_info[187].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[187].name = "mtimes";
  c2_info[187].dominantType = "double";
  c2_info[187].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[187].fileTimeLo = 1289541292U;
  c2_info[187].fileTimeHi = 0U;
  c2_info[187].mFileTimeLo = 0U;
  c2_info[187].mFileTimeHi = 0U;
  c2_info[188].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[188].name = "eml_index_class";
  c2_info[188].dominantType = "";
  c2_info[188].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[188].fileTimeLo = 1323192178U;
  c2_info[188].fileTimeHi = 0U;
  c2_info[188].mFileTimeLo = 0U;
  c2_info[188].mFileTimeHi = 0U;
  c2_info[189].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[189].name = "eml_refblas_xger";
  c2_info[189].dominantType = "double";
  c2_info[189].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[189].fileTimeLo = 1299098376U;
  c2_info[189].fileTimeHi = 0U;
  c2_info[189].mFileTimeLo = 0U;
  c2_info[189].mFileTimeHi = 0U;
  c2_info[190].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[190].name = "eml_refblas_xgerx";
  c2_info[190].dominantType = "char";
  c2_info[190].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[190].fileTimeLo = 1299098378U;
  c2_info[190].fileTimeHi = 0U;
  c2_info[190].mFileTimeLo = 0U;
  c2_info[190].mFileTimeHi = 0U;
  c2_info[191].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[191].name = "eml_index_class";
  c2_info[191].dominantType = "";
  c2_info[191].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[191].fileTimeLo = 1323192178U;
  c2_info[191].fileTimeHi = 0U;
  c2_info[191].mFileTimeLo = 0U;
  c2_info[191].mFileTimeHi = 0U;
}

static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[214])
{
  c2_info[192].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[192].name = "abs";
  c2_info[192].dominantType = "coder.internal.indexInt";
  c2_info[192].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[192].fileTimeLo = 1286840294U;
  c2_info[192].fileTimeHi = 0U;
  c2_info[192].mFileTimeLo = 0U;
  c2_info[192].mFileTimeHi = 0U;
  c2_info[193].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[193].name = "eml_index_minus";
  c2_info[193].dominantType = "double";
  c2_info[193].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[193].fileTimeLo = 1286840378U;
  c2_info[193].fileTimeHi = 0U;
  c2_info[193].mFileTimeLo = 0U;
  c2_info[193].mFileTimeHi = 0U;
  c2_info[194].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[194].name = "eml_int_forloop_overflow_check";
  c2_info[194].dominantType = "";
  c2_info[194].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[194].fileTimeLo = 1332186672U;
  c2_info[194].fileTimeHi = 0U;
  c2_info[194].mFileTimeLo = 0U;
  c2_info[194].mFileTimeHi = 0U;
  c2_info[195].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[195].name = "eml_index_plus";
  c2_info[195].dominantType = "double";
  c2_info[195].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[195].fileTimeLo = 1286840378U;
  c2_info[195].fileTimeHi = 0U;
  c2_info[195].mFileTimeLo = 0U;
  c2_info[195].mFileTimeHi = 0U;
  c2_info[196].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[196].name = "eml_index_plus";
  c2_info[196].dominantType = "coder.internal.indexInt";
  c2_info[196].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[196].fileTimeLo = 1286840378U;
  c2_info[196].fileTimeHi = 0U;
  c2_info[196].mFileTimeLo = 0U;
  c2_info[196].mFileTimeHi = 0U;
  c2_info[197].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[197].name = "eml_scalar_eg";
  c2_info[197].dominantType = "double";
  c2_info[197].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[197].fileTimeLo = 1286840396U;
  c2_info[197].fileTimeHi = 0U;
  c2_info[197].mFileTimeLo = 0U;
  c2_info[197].mFileTimeHi = 0U;
  c2_info[198].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[198].name = "eml_int_forloop_overflow_check";
  c2_info[198].dominantType = "";
  c2_info[198].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[198].fileTimeLo = 1332186672U;
  c2_info[198].fileTimeHi = 0U;
  c2_info[198].mFileTimeLo = 0U;
  c2_info[198].mFileTimeHi = 0U;
  c2_info[199].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_lusolve.m!lusolveNxN";
  c2_info[199].name = "eml_xtrsm";
  c2_info[199].dominantType = "char";
  c2_info[199].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[199].fileTimeLo = 1299098378U;
  c2_info[199].fileTimeHi = 0U;
  c2_info[199].mFileTimeLo = 0U;
  c2_info[199].mFileTimeHi = 0U;
  c2_info[200].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c2_info[200].name = "eml_blas_inline";
  c2_info[200].dominantType = "";
  c2_info[200].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[200].fileTimeLo = 1299098368U;
  c2_info[200].fileTimeHi = 0U;
  c2_info[200].mFileTimeLo = 0U;
  c2_info[200].mFileTimeHi = 0U;
  c2_info[201].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c2_info[201].name = "mtimes";
  c2_info[201].dominantType = "double";
  c2_info[201].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[201].fileTimeLo = 1289541292U;
  c2_info[201].fileTimeHi = 0U;
  c2_info[201].mFileTimeLo = 0U;
  c2_info[201].mFileTimeHi = 0U;
  c2_info[202].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[202].name = "eml_index_class";
  c2_info[202].dominantType = "";
  c2_info[202].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[202].fileTimeLo = 1323192178U;
  c2_info[202].fileTimeHi = 0U;
  c2_info[202].mFileTimeLo = 0U;
  c2_info[202].mFileTimeHi = 0U;
  c2_info[203].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[203].name = "eml_scalar_eg";
  c2_info[203].dominantType = "double";
  c2_info[203].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[203].fileTimeLo = 1286840396U;
  c2_info[203].fileTimeHi = 0U;
  c2_info[203].mFileTimeLo = 0U;
  c2_info[203].mFileTimeHi = 0U;
  c2_info[204].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c2_info[204].name = "eml_refblas_xtrsm";
  c2_info[204].dominantType = "char";
  c2_info[204].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[204].fileTimeLo = 1299098386U;
  c2_info[204].fileTimeHi = 0U;
  c2_info[204].mFileTimeLo = 0U;
  c2_info[204].mFileTimeHi = 0U;
  c2_info[205].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[205].name = "eml_scalar_eg";
  c2_info[205].dominantType = "double";
  c2_info[205].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[205].fileTimeLo = 1286840396U;
  c2_info[205].fileTimeHi = 0U;
  c2_info[205].mFileTimeLo = 0U;
  c2_info[205].mFileTimeHi = 0U;
  c2_info[206].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[206].name = "eml_index_minus";
  c2_info[206].dominantType = "double";
  c2_info[206].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[206].fileTimeLo = 1286840378U;
  c2_info[206].fileTimeHi = 0U;
  c2_info[206].mFileTimeLo = 0U;
  c2_info[206].mFileTimeHi = 0U;
  c2_info[207].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[207].name = "eml_index_class";
  c2_info[207].dominantType = "";
  c2_info[207].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[207].fileTimeLo = 1323192178U;
  c2_info[207].fileTimeHi = 0U;
  c2_info[207].mFileTimeLo = 0U;
  c2_info[207].mFileTimeHi = 0U;
  c2_info[208].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[208].name = "eml_int_forloop_overflow_check";
  c2_info[208].dominantType = "";
  c2_info[208].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[208].fileTimeLo = 1332186672U;
  c2_info[208].fileTimeHi = 0U;
  c2_info[208].mFileTimeLo = 0U;
  c2_info[208].mFileTimeHi = 0U;
  c2_info[209].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[209].name = "eml_index_times";
  c2_info[209].dominantType = "coder.internal.indexInt";
  c2_info[209].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[209].fileTimeLo = 1286840380U;
  c2_info[209].fileTimeHi = 0U;
  c2_info[209].mFileTimeLo = 0U;
  c2_info[209].mFileTimeHi = 0U;
  c2_info[210].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[210].name = "eml_index_plus";
  c2_info[210].dominantType = "coder.internal.indexInt";
  c2_info[210].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[210].fileTimeLo = 1286840378U;
  c2_info[210].fileTimeHi = 0U;
  c2_info[210].mFileTimeLo = 0U;
  c2_info[210].mFileTimeHi = 0U;
  c2_info[211].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[211].name = "eml_index_plus";
  c2_info[211].dominantType = "double";
  c2_info[211].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[211].fileTimeLo = 1286840378U;
  c2_info[211].fileTimeHi = 0U;
  c2_info[211].mFileTimeLo = 0U;
  c2_info[211].mFileTimeHi = 0U;
  c2_info[212].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[212].name = "intmin";
  c2_info[212].dominantType = "char";
  c2_info[212].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[212].fileTimeLo = 1311276918U;
  c2_info[212].fileTimeHi = 0U;
  c2_info[212].mFileTimeLo = 0U;
  c2_info[212].mFileTimeHi = 0U;
  c2_info[213].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c2_info[213].name = "eml_div";
  c2_info[213].dominantType = "double";
  c2_info[213].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[213].fileTimeLo = 1313369410U;
  c2_info[213].fileTimeHi = 0U;
  c2_info[213].mFileTimeLo = 0U;
  c2_info[213].mFileTimeHi = 0U;
}

static void c2_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_y[3])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  for (c2_k = 0; c2_k < 3; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 3, 1, 0) - 1];
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 3, 1, 0) - 1] = muDoubleScalarPower(c2_ak, 2.0);
  }
}

static real_T c2_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[3])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_y = c2_x[0];
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 2; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_y += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 3, 1, 0) - 1];
  }

  return c2_y;
}

static void c2_check_forloop_overflow_error
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static real_T c2_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_c_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
  int32_T c2_i502;
  static char_T c2_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  for (c2_i502 = 0; c2_i502 < 30; c2_i502++) {
    c2_u[c2_i502] = c2_varargin_1[c2_i502];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_rdivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[3], real_T c2_y, real_T c2_z[3])
{
  real_T c2_b_y;
  int32_T c2_i503;
  c2_b_y = c2_y;
  for (c2_i503 = 0; c2_i503 < 3; c2_i503++) {
    c2_z[c2_i503] = c2_x[c2_i503] / c2_b_y;
  }
}

static void c2_quat2dcm(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_qin[4], real_T c2_dcm[9])
{
  uint32_T c2_debug_family_var_map[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i504;
  real_T c2_b_qin[4];
  real_T c2_x[4];
  int32_T c2_i505;
  real_T c2_b_x[4];
  real_T c2_c_x;
  real_T c2_d_x;
  int32_T c2_i506;
  real_T c2_y;
  real_T c2_b_y;
  int32_T c2_i507;
  int32_T c2_i508;
  sf_debug_symbol_scope_push_eml(0U, 4U, 4U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 0U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 1U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_qin, 2U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_dcm, 3U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 254U);
  for (c2_i504 = 0; c2_i504 < 4; c2_i504++) {
    c2_b_qin[c2_i504] = c2_qin[c2_i504];
  }

  c2_b_power(chartInstance, c2_b_qin, c2_x);
  for (c2_i505 = 0; c2_i505 < 4; c2_i505++) {
    c2_b_x[c2_i505] = c2_x[c2_i505];
  }

  c2_c_x = c2_b_sum(chartInstance, c2_b_x);
  c2_d_x = c2_c_x;
  if (c2_d_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_d_x = muDoubleScalarSqrt(c2_d_x);
  for (c2_i506 = 0; c2_i506 < 4; c2_i506++) {
    c2_x[c2_i506] = c2_qin[c2_i506];
  }

  c2_y = c2_d_x;
  c2_b_y = c2_y;
  for (c2_i507 = 0; c2_i507 < 4; c2_i507++) {
    c2_qin[c2_i507] = c2_x[c2_i507] / c2_b_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 256);
  for (c2_i508 = 0; c2_i508 < 9; c2_i508++) {
    c2_dcm[c2_i508] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 257);
  c2_dcm[0] = ((c2_c_power(chartInstance, c2_qin[0]) + c2_c_power(chartInstance,
    c2_qin[1])) - c2_c_power(chartInstance, c2_qin[2])) - c2_c_power
    (chartInstance, c2_qin[3]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 258);
  c2_dcm[3] = 2.0 * (c2_qin[1] * c2_qin[2] + c2_qin[0] * c2_qin[3]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 259);
  c2_dcm[6] = 2.0 * (c2_qin[1] * c2_qin[3] - c2_qin[0] * c2_qin[2]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 260);
  c2_dcm[1] = 2.0 * (c2_qin[1] * c2_qin[2] - c2_qin[0] * c2_qin[3]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 261);
  c2_dcm[4] = ((c2_c_power(chartInstance, c2_qin[0]) - c2_c_power(chartInstance,
    c2_qin[1])) + c2_c_power(chartInstance, c2_qin[2])) - c2_c_power
    (chartInstance, c2_qin[3]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 262);
  c2_dcm[7] = 2.0 * (c2_qin[2] * c2_qin[3] + c2_qin[0] * c2_qin[1]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 263);
  c2_dcm[2] = 2.0 * (c2_qin[1] * c2_qin[3] + c2_qin[0] * c2_qin[2]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 264);
  c2_dcm[5] = 2.0 * (c2_qin[2] * c2_qin[3] - c2_qin[0] * c2_qin[1]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 265);
  c2_dcm[8] = ((c2_c_power(chartInstance, c2_qin[0]) - c2_c_power(chartInstance,
    c2_qin[1])) - c2_c_power(chartInstance, c2_qin[2])) + c2_c_power
    (chartInstance, c2_qin[3]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -265);
  sf_debug_symbol_scope_pop();
}

static void c2_b_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[4], real_T c2_y[4])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  for (c2_k = 0; c2_k < 4; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 4, 1, 0) - 1];
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 4, 1, 0) - 1] = muDoubleScalarPower(c2_ak, 2.0);
  }
}

static real_T c2_b_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[4])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_y = c2_x[0];
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 2; c2_k < 5; c2_k++) {
    c2_b_k = c2_k;
    c2_y += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 4, 1, 0) - 1];
  }

  return c2_y;
}

static real_T c2_c_power(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_ak;
  c2_b_a = c2_a;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_b_a;
  return muDoubleScalarPower(c2_ak, 2.0);
}

static void c2_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_mldivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_A[9], real_T c2_B[3], real_T c2_Y[3])
{
  int32_T c2_i509;
  real_T c2_b_A[9];
  int32_T c2_r1;
  int32_T c2_r2;
  int32_T c2_r3;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_maxval;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_a21;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_e_y;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_f_y;
  real_T c2_d;
  real_T c2_p_x;
  real_T c2_g_y;
  real_T c2_z;
  real_T c2_q_x;
  real_T c2_h_y;
  real_T c2_b_z;
  real_T c2_a;
  real_T c2_b;
  real_T c2_i_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_j_y;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_k_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_l_y;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_m_y;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_n_y;
  real_T c2_b_d;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_y_x;
  real_T c2_o_y;
  real_T c2_ab_x;
  real_T c2_bb_x;
  real_T c2_p_y;
  real_T c2_c_d;
  int32_T c2_rtemp;
  real_T c2_cb_x;
  real_T c2_q_y;
  real_T c2_c_z;
  real_T c2_e_a;
  real_T c2_e_b;
  real_T c2_r_y;
  real_T c2_f_a;
  real_T c2_f_b;
  real_T c2_s_y;
  real_T c2_g_a;
  real_T c2_g_b;
  real_T c2_t_y;
  real_T c2_h_a;
  real_T c2_h_b;
  real_T c2_u_y;
  real_T c2_db_x;
  real_T c2_v_y;
  real_T c2_d_z;
  real_T c2_i_a;
  real_T c2_i_b;
  real_T c2_w_y;
  real_T c2_j_a;
  real_T c2_j_b;
  real_T c2_x_y;
  real_T c2_eb_x;
  real_T c2_y_y;
  real_T c2_e_z;
  real_T c2_k_a;
  real_T c2_k_b;
  real_T c2_ab_y;
  real_T c2_fb_x;
  real_T c2_bb_y;
  real_T c2_f_z;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  for (c2_i509 = 0; c2_i509 < 9; c2_i509++) {
    c2_b_A[c2_i509] = c2_A[c2_i509];
  }

  c2_r1 = 1;
  c2_r2 = 2;
  c2_r3 = 3;
  c2_x = c2_b_A[0];
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_y = muDoubleScalarAbs(c2_c_x);
  c2_d_x = 0.0;
  c2_e_x = c2_d_x;
  c2_b_y = muDoubleScalarAbs(c2_e_x);
  c2_maxval = c2_y + c2_b_y;
  c2_f_x = c2_b_A[1];
  c2_g_x = c2_f_x;
  c2_h_x = c2_g_x;
  c2_c_y = muDoubleScalarAbs(c2_h_x);
  c2_i_x = 0.0;
  c2_j_x = c2_i_x;
  c2_d_y = muDoubleScalarAbs(c2_j_x);
  c2_a21 = c2_c_y + c2_d_y;
  if (c2_a21 > c2_maxval) {
    c2_maxval = c2_a21;
    c2_r1 = 2;
    c2_r2 = 1;
  }

  c2_k_x = c2_b_A[2];
  c2_l_x = c2_k_x;
  c2_m_x = c2_l_x;
  c2_e_y = muDoubleScalarAbs(c2_m_x);
  c2_n_x = 0.0;
  c2_o_x = c2_n_x;
  c2_f_y = muDoubleScalarAbs(c2_o_x);
  c2_d = c2_e_y + c2_f_y;
  if (c2_d > c2_maxval) {
    c2_r1 = 3;
    c2_r2 = 2;
    c2_r3 = 1;
  }

  c2_p_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) - 1];
  c2_g_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) - 1];
  c2_z = c2_p_x / c2_g_y;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r2), 1, 3, 1, 0) - 1] = c2_z;
  c2_q_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) - 1];
  c2_h_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) - 1];
  c2_b_z = c2_q_x / c2_h_y;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r3), 1, 3, 1, 0) - 1] = c2_b_z;
  c2_a = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) - 1];
  c2_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 2];
  c2_i_y = c2_a * c2_b;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r2), 1, 3, 1, 0) + 2] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c2_r2), 1, 3, 1, 0) + 2] - c2_i_y;
  c2_b_a = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) - 1];
  c2_b_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 2];
  c2_j_y = c2_b_a * c2_b_b;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r3), 1, 3, 1, 0) + 2] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c2_r3), 1, 3, 1, 0) + 2] - c2_j_y;
  c2_c_a = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) - 1];
  c2_c_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 5];
  c2_k_y = c2_c_a * c2_c_b;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r2), 1, 3, 1, 0) + 5] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c2_r2), 1, 3, 1, 0) + 5] - c2_k_y;
  c2_d_a = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) - 1];
  c2_d_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 5];
  c2_l_y = c2_d_a * c2_d_b;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r3), 1, 3, 1, 0) + 5] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c2_r3), 1, 3, 1, 0) + 5] - c2_l_y;
  c2_r_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) + 2];
  c2_s_x = c2_r_x;
  c2_t_x = c2_s_x;
  c2_m_y = muDoubleScalarAbs(c2_t_x);
  c2_u_x = 0.0;
  c2_v_x = c2_u_x;
  c2_n_y = muDoubleScalarAbs(c2_v_x);
  c2_b_d = c2_m_y + c2_n_y;
  c2_w_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) + 2];
  c2_x_x = c2_w_x;
  c2_y_x = c2_x_x;
  c2_o_y = muDoubleScalarAbs(c2_y_x);
  c2_ab_x = 0.0;
  c2_bb_x = c2_ab_x;
  c2_p_y = muDoubleScalarAbs(c2_bb_x);
  c2_c_d = c2_o_y + c2_p_y;
  if (c2_b_d > c2_c_d) {
    c2_rtemp = c2_r2;
    c2_r2 = c2_r3;
    c2_r3 = c2_rtemp;
  }

  c2_cb_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c2_r3), 1, 3, 1, 0) + 2];
  c2_q_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) + 2];
  c2_c_z = c2_cb_x / c2_q_y;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r3), 1, 3, 1, 0) + 2] = c2_c_z;
  c2_e_a = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) + 2];
  c2_e_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) + 5];
  c2_r_y = c2_e_a * c2_e_b;
  c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
    c2_r3), 1, 3, 1, 0) + 5] = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
    _SFD_INTEGER_CHECK("", (real_T)c2_r3), 1, 3, 1, 0) + 5] - c2_r_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_r1), 1, 3, 1, 0) - 1] == 0.0) {
    guard2 = TRUE;
  } else if (c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
               "", (real_T)c2_r2), 1, 3, 1, 0) + 2] == 0.0) {
    guard2 = TRUE;
  } else {
    if (c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_r3), 1, 3, 1, 0) + 5] == 0.0) {
      guard1 = TRUE;
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c2_eml_warning(chartInstance);
  }

  c2_Y[0] = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) - 1];
  c2_f_a = c2_Y[0];
  c2_f_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) - 1];
  c2_s_y = c2_f_a * c2_f_b;
  c2_Y[1] = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) - 1] - c2_s_y;
  c2_g_a = c2_Y[0];
  c2_g_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) - 1];
  c2_t_y = c2_g_a * c2_g_b;
  c2_h_a = c2_Y[1];
  c2_h_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) + 2];
  c2_u_y = c2_h_a * c2_h_b;
  c2_Y[2] = (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c2_r3), 1, 3, 1, 0) - 1] - c2_t_y) - c2_u_y;
  c2_db_x = c2_Y[2];
  c2_v_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r3), 1, 3, 1, 0) + 5];
  c2_d_z = c2_db_x / c2_v_y;
  c2_Y[2] = c2_d_z;
  c2_i_a = c2_Y[2];
  c2_i_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 5];
  c2_w_y = c2_i_a * c2_i_b;
  c2_Y[0] -= c2_w_y;
  c2_j_a = c2_Y[2];
  c2_j_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) + 5];
  c2_x_y = c2_j_a * c2_j_b;
  c2_Y[1] -= c2_x_y;
  c2_eb_x = c2_Y[1];
  c2_y_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r2), 1, 3, 1, 0) + 2];
  c2_e_z = c2_eb_x / c2_y_y;
  c2_Y[1] = c2_e_z;
  c2_k_a = c2_Y[1];
  c2_k_b = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c2_r1), 1, 3, 1, 0) + 2];
  c2_ab_y = c2_k_a * c2_k_b;
  c2_Y[0] -= c2_ab_y;
  c2_fb_x = c2_Y[0];
  c2_bb_y = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c2_r1), 1, 3, 1, 0) - 1];
  c2_f_z = c2_fb_x / c2_bb_y;
  c2_Y[0] = c2_f_z;
}

static void c2_eml_warning(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
  int32_T c2_i510;
  static char_T c2_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c2_u[27];
  const mxArray *c2_y = NULL;
  for (c2_i510 = 0; c2_i510 < 27; c2_i510++) {
    c2_u[c2_i510] = c2_varargin_1[c2_i510];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c2_y));
}

static real_T c2_mpower(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a)
{
  return c2_c_power(chartInstance, c2_a);
}

static void c2_chol(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                    real_T c2_A[81], real_T c2_b_A[81])
{
  int32_T c2_i511;
  for (c2_i511 = 0; c2_i511 < 81; c2_i511++) {
    c2_b_A[c2_i511] = c2_A[c2_i511];
  }

  c2_b_chol(chartInstance, c2_b_A);
}

static void c2_b_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
  int32_T c2_i512;
  static char_T c2_varargin_1[48] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'c', 'h', 'o', 'l', '_', 'm', 'a', 't', 'r', 'i',
    'x', 'M', 'u', 's', 't', 'B', 'e', 'P', 'o', 's', 'D', 'e', 'f', 'W', 'i',
    't', 'h', 'R', 'e', 'a', 'l', 'D', 'i', 'a', 'g' };

  char_T c2_u[48];
  const mxArray *c2_y = NULL;
  for (c2_i512 = 0; c2_i512 < 48; c2_i512++) {
    c2_u[c2_i512] = c2_varargin_1[c2_i512];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 48), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_b_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_eml_matlab_zpotrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[81], real_T c2_b_A[81], int32_T *c2_info)
{
  int32_T c2_i513;
  for (c2_i513 = 0; c2_i513 < 81; c2_i513++) {
    c2_b_A[c2_i513] = c2_A[c2_i513];
  }

  *c2_info = c2_b_eml_matlab_zpotrf(chartInstance, c2_b_A);
}

static void c2_b_check_forloop_overflow_error
  (SFc2_SatelliteControlSystemInstanceStruct *chartInstance, boolean_T
   c2_overflow)
{
  int32_T c2_i514;
  static char_T c2_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i515;
  static char_T c2_cv1[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i514 = 0; c2_i514 < 34; c2_i514++) {
      c2_u[c2_i514] = c2_cv0[c2_i514];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i515 = 0; c2_i515 < 23; c2_i515++) {
      c2_b_u[c2_i515] = c2_cv1[c2_i515];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static void c2_eml_xgemv(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0,
  real_T c2_y[81], int32_T c2_iy0, real_T c2_b_y[81])
{
  int32_T c2_i516;
  for (c2_i516 = 0; c2_i516 < 81; c2_i516++) {
    c2_b_y[c2_i516] = c2_y[c2_i516];
  }

  c2_b_eml_xgemv(chartInstance, c2_m, c2_n, c2_ia0, c2_ix0, c2_b_y, c2_iy0);
}

static void c2_c_eml_error(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
  int32_T c2_i517;
  static char_T c2_varargin_1[19] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'p', 'o', 's', 'd', 'e', 'f' };

  char_T c2_u[19];
  const mxArray *c2_y = NULL;
  for (c2_i517 = 0; c2_i517 < 19; c2_i517++) {
    c2_u[c2_i517] = c2_varargin_1[c2_i517];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 19), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_d_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[57], real_T c2_y[57])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  for (c2_k = 0; c2_k < 57; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 57, 1, 0) - 1];
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 57, 1, 0) - 1] = muDoubleScalarPower(c2_ak, 2.0);
  }
}

static void c2_c_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[57], real_T c2_y[19])
{
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_i;
  int32_T c2_ixstart;
  int32_T c2_a;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_b_a;
  int32_T c2_c_a;
  c2_ix = 0;
  c2_iy = 0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_i = 1; c2_i < 20; c2_i++) {
    c2_ixstart = c2_ix;
    c2_a = c2_ixstart + 1;
    c2_ixstart = c2_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 57, 1, 0) - 1];
    c2_check_forloop_overflow_error(chartInstance);
    for (c2_k = 2; c2_k < 4; c2_k++) {
      c2_b_a = c2_ix + 1;
      c2_ix = c2_b_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 57, 1, 0) - 1];
    }

    c2_c_a = c2_iy + 1;
    c2_iy = c2_c_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 19, 1, 0) - 1] = c2_s;
  }
}

static void c2_b_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x[19], real_T c2_b_x[19])
{
  int32_T c2_i518;
  for (c2_i518 = 0; c2_i518 < 19; c2_i518++) {
    c2_b_x[c2_i518] = c2_x[c2_i518];
  }

  c2_d_sqrt(chartInstance, c2_b_x);
}

static void c2_quatmultiply(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_q[76], real_T c2_r[4], real_T c2_qout[76])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_vec[57];
  real_T c2_scalar[19];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  real_T c2_b_r;
  int32_T c2_i519;
  real_T c2_b_q[19];
  real_T c2_c_r;
  int32_T c2_i520;
  real_T c2_c_q[19];
  real_T c2_d_r;
  int32_T c2_i521;
  real_T c2_d_q[19];
  real_T c2_e_r;
  int32_T c2_i522;
  real_T c2_f_r[19];
  real_T c2_g_r;
  int32_T c2_i523;
  real_T c2_h_r[19];
  real_T c2_i_r;
  int32_T c2_i524;
  real_T c2_j_r[19];
  real_T c2_k_r;
  real_T c2_l_r;
  int32_T c2_i525;
  real_T c2_e_q[19];
  real_T c2_m_r;
  real_T c2_n_r;
  int32_T c2_i526;
  real_T c2_f_q[19];
  real_T c2_o_r;
  real_T c2_p_r;
  int32_T c2_i527;
  real_T c2_g_q[19];
  int32_T c2_i528;
  real_T c2_h_q[57];
  int32_T c2_i529;
  int32_T c2_i530;
  int32_T c2_i531;
  real_T c2_q_r[57];
  int32_T c2_i532;
  int32_T c2_i533;
  int32_T c2_i534;
  real_T c2_i_q[57];
  int32_T c2_i535;
  int32_T c2_i536;
  int32_T c2_i537;
  int32_T c2_i538;
  int32_T c2_i539;
  real_T c2_r_r;
  real_T c2_s_r;
  real_T c2_t_r;
  real_T c2_u_r;
  int32_T c2_i540;
  int32_T c2_i541;
  int32_T c2_i542;
  int32_T c2_i543;
  int32_T c2_i544;
  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_vec, 0U, c2_eb_sf_marshallOut,
    c2_eb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_scalar, 1U, c2_db_sf_marshallOut,
    c2_db_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_q, 4U, c2_cb_sf_marshallOut,
    c2_cb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_r, 5U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_qout, 6U, c2_cb_sf_marshallOut,
    c2_cb_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 237U);
  c2_b_r = c2_r[1];
  for (c2_i519 = 0; c2_i519 < 19; c2_i519++) {
    c2_b_q[c2_i519] = c2_q[c2_i519] * c2_b_r;
  }

  c2_c_r = c2_r[2];
  for (c2_i520 = 0; c2_i520 < 19; c2_i520++) {
    c2_c_q[c2_i520] = c2_q[c2_i520] * c2_c_r;
  }

  c2_d_r = c2_r[3];
  for (c2_i521 = 0; c2_i521 < 19; c2_i521++) {
    c2_d_q[c2_i521] = c2_q[c2_i521] * c2_d_r;
  }

  c2_e_r = c2_r[0];
  for (c2_i522 = 0; c2_i522 < 19; c2_i522++) {
    c2_f_r[c2_i522] = c2_e_r * c2_q[c2_i522 + 19];
  }

  c2_g_r = c2_r[0];
  for (c2_i523 = 0; c2_i523 < 19; c2_i523++) {
    c2_h_r[c2_i523] = c2_g_r * c2_q[c2_i523 + 38];
  }

  c2_i_r = c2_r[0];
  for (c2_i524 = 0; c2_i524 < 19; c2_i524++) {
    c2_j_r[c2_i524] = c2_i_r * c2_q[c2_i524 + 57];
  }

  c2_k_r = c2_r[3];
  c2_l_r = c2_r[2];
  for (c2_i525 = 0; c2_i525 < 19; c2_i525++) {
    c2_e_q[c2_i525] = c2_q[c2_i525 + 38] * c2_k_r - c2_q[c2_i525 + 57] * c2_l_r;
  }

  c2_m_r = c2_r[1];
  c2_n_r = c2_r[3];
  for (c2_i526 = 0; c2_i526 < 19; c2_i526++) {
    c2_f_q[c2_i526] = c2_q[c2_i526 + 57] * c2_m_r - c2_q[c2_i526 + 19] * c2_n_r;
  }

  c2_o_r = c2_r[2];
  c2_p_r = c2_r[1];
  for (c2_i527 = 0; c2_i527 < 19; c2_i527++) {
    c2_g_q[c2_i527] = c2_q[c2_i527 + 19] * c2_o_r - c2_q[c2_i527 + 38] * c2_p_r;
  }

  for (c2_i528 = 0; c2_i528 < 19; c2_i528++) {
    c2_h_q[c2_i528] = c2_b_q[c2_i528];
  }

  for (c2_i529 = 0; c2_i529 < 19; c2_i529++) {
    c2_h_q[c2_i529 + 19] = c2_c_q[c2_i529];
  }

  for (c2_i530 = 0; c2_i530 < 19; c2_i530++) {
    c2_h_q[c2_i530 + 38] = c2_d_q[c2_i530];
  }

  for (c2_i531 = 0; c2_i531 < 19; c2_i531++) {
    c2_q_r[c2_i531] = c2_f_r[c2_i531];
  }

  for (c2_i532 = 0; c2_i532 < 19; c2_i532++) {
    c2_q_r[c2_i532 + 19] = c2_h_r[c2_i532];
  }

  for (c2_i533 = 0; c2_i533 < 19; c2_i533++) {
    c2_q_r[c2_i533 + 38] = c2_j_r[c2_i533];
  }

  for (c2_i534 = 0; c2_i534 < 19; c2_i534++) {
    c2_i_q[c2_i534] = c2_e_q[c2_i534];
  }

  for (c2_i535 = 0; c2_i535 < 19; c2_i535++) {
    c2_i_q[c2_i535 + 19] = c2_f_q[c2_i535];
  }

  for (c2_i536 = 0; c2_i536 < 19; c2_i536++) {
    c2_i_q[c2_i536 + 38] = c2_g_q[c2_i536];
  }

  c2_i537 = 0;
  for (c2_i538 = 0; c2_i538 < 3; c2_i538++) {
    for (c2_i539 = 0; c2_i539 < 19; c2_i539++) {
      c2_vec[c2_i539 + c2_i537] = (c2_h_q[c2_i539 + c2_i537] + c2_q_r[c2_i539 +
        c2_i537]) + c2_i_q[c2_i539 + c2_i537];
    }

    c2_i537 += 19;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 245U);
  c2_r_r = c2_r[0];
  c2_s_r = c2_r[1];
  c2_t_r = c2_r[2];
  c2_u_r = c2_r[3];
  for (c2_i540 = 0; c2_i540 < 19; c2_i540++) {
    c2_scalar[c2_i540] = ((c2_q[c2_i540] * c2_r_r - c2_q[c2_i540 + 19] * c2_s_r)
                          - c2_q[c2_i540 + 38] * c2_t_r) - c2_q[c2_i540 + 57] *
      c2_u_r;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  for (c2_i541 = 0; c2_i541 < 19; c2_i541++) {
    c2_qout[c2_i541] = c2_scalar[c2_i541];
  }

  c2_i542 = 0;
  for (c2_i543 = 0; c2_i543 < 3; c2_i543++) {
    for (c2_i544 = 0; c2_i544 < 19; c2_i544++) {
      c2_qout[(c2_i544 + c2_i542) + 19] = c2_vec[c2_i544 + c2_i542];
    }

    c2_i542 += 19;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -248);
  sf_debug_symbol_scope_pop();
}

static void c2_RK4(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                   real_T c2_X_km1km1[190], real_T c2_b_Ts, real_T c2_X_kkm1[190])
{
  uint32_T c2_debug_family_var_map[13];
  real_T c2_DCM[9];
  real_T c2_q[4];
  real_T c2_w_cont[3];
  real_T c2_W[16];
  real_T c2_k1[4];
  real_T c2_k2[4];
  real_T c2_k3[4];
  real_T c2_k4[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i545;
  int32_T c2_idx;
  real_T c2_b_idx;
  int32_T c2_i546;
  int32_T c2_c_idx;
  int32_T c2_i547;
  int32_T c2_d_idx;
  int32_T c2_i548;
  real_T c2_b[16];
  int32_T c2_i549;
  int32_T c2_i550;
  int32_T c2_i551;
  real_T c2_b_b[4];
  int32_T c2_i552;
  int32_T c2_i553;
  int32_T c2_i554;
  real_T c2_C[4];
  int32_T c2_i555;
  int32_T c2_i556;
  int32_T c2_i557;
  int32_T c2_i558;
  int32_T c2_i559;
  int32_T c2_i560;
  int32_T c2_i561;
  int32_T c2_i562;
  real_T c2_c_b;
  int32_T c2_i563;
  int32_T c2_i564;
  int32_T c2_i565;
  int32_T c2_i566;
  int32_T c2_i567;
  int32_T c2_i568;
  int32_T c2_i569;
  int32_T c2_i570;
  int32_T c2_i571;
  int32_T c2_i572;
  int32_T c2_i573;
  int32_T c2_i574;
  int32_T c2_i575;
  int32_T c2_i576;
  real_T c2_d_b;
  int32_T c2_i577;
  int32_T c2_i578;
  int32_T c2_i579;
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
  real_T c2_a[4];
  real_T c2_e_b;
  int32_T c2_i590;
  int32_T c2_i591;
  int32_T c2_i592;
  int32_T c2_i593;
  int32_T c2_i594;
  int32_T c2_i595;
  int32_T c2_i596;
  int32_T c2_i597;
  int32_T c2_i598;
  int32_T c2_i599;
  int32_T c2_i600;
  int32_T c2_i601;
  int32_T c2_i602;
  int32_T c2_i603;
  int32_T c2_i604;
  int32_T c2_i605;
  int32_T c2_i606;
  real_T c2_f_b;
  int32_T c2_i607;
  int32_T c2_i608;
  int32_T c2_e_idx;
  int32_T c2_i609;
  int32_T c2_f_idx;
  int32_T c2_g_idx;
  int32_T c2_i610;
  int32_T c2_h_idx;
  int32_T c2_i_idx;
  int32_T c2_i611;
  sf_debug_symbol_scope_push_eml(0U, 13U, 13U, c2_d_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_DCM, 0U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_q, 1U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_w_cont, 2U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_W, 3U, c2_fb_sf_marshallOut,
    c2_fb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_k1, 4U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_k2, 5U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_k3, 6U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_k4, 7U, c2_n_sf_marshallOut,
    c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 8U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 9U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_km1km1, 10U, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_b_Ts, 11U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_kkm1, 12U, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 184U);
  for (c2_i545 = 0; c2_i545 < 190; c2_i545++) {
    c2_X_kkm1[c2_i545] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 185U);
  c2_idx = 0;
  while (c2_idx < 19) {
    c2_b_idx = 1.0 + (real_T)c2_idx;
    CV_EML_FOR(0, 1, 2, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 186U);
    for (c2_i546 = 0; c2_i546 < 9; c2_i546++) {
      c2_DCM[c2_i546] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 189U);
    c2_c_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i547 = 0; c2_i547 < 4; c2_i547++) {
      c2_q[c2_i547] = c2_X_km1km1[c2_i547 + 10 * c2_c_idx];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 190U);
    c2_d_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i548 = 0; c2_i548 < 3; c2_i548++) {
      c2_w_cont[c2_i548] = c2_X_km1km1[(c2_i548 + 10 * c2_d_idx) + 4];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 192U);
    c2_b[0] = 0.0;
    c2_b[4] = -c2_w_cont[0];
    c2_b[8] = -c2_w_cont[1];
    c2_b[12] = -c2_w_cont[2];
    c2_b[1] = c2_w_cont[0];
    c2_b[5] = 0.0;
    c2_b[9] = c2_w_cont[2];
    c2_b[13] = -c2_w_cont[1];
    c2_b[2] = c2_w_cont[1];
    c2_b[6] = -c2_w_cont[2];
    c2_b[10] = 0.0;
    c2_b[14] = c2_w_cont[0];
    c2_b[3] = c2_w_cont[2];
    c2_b[7] = c2_w_cont[1];
    c2_b[11] = -c2_w_cont[0];
    c2_b[15] = 0.0;
    for (c2_i549 = 0; c2_i549 < 16; c2_i549++) {
      c2_W[c2_i549] = 0.5 * c2_b[c2_i549];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 197U);
    for (c2_i550 = 0; c2_i550 < 16; c2_i550++) {
      c2_b[c2_i550] = c2_W[c2_i550];
    }

    for (c2_i551 = 0; c2_i551 < 4; c2_i551++) {
      c2_b_b[c2_i551] = c2_q[c2_i551];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i552 = 0; c2_i552 < 4; c2_i552++) {
      c2_k1[c2_i552] = 0.0;
    }

    for (c2_i553 = 0; c2_i553 < 4; c2_i553++) {
      c2_k1[c2_i553] = 0.0;
    }

    for (c2_i554 = 0; c2_i554 < 4; c2_i554++) {
      c2_C[c2_i554] = c2_k1[c2_i554];
    }

    for (c2_i555 = 0; c2_i555 < 4; c2_i555++) {
      c2_k1[c2_i555] = c2_C[c2_i555];
    }

    for (c2_i556 = 0; c2_i556 < 4; c2_i556++) {
      c2_C[c2_i556] = c2_k1[c2_i556];
    }

    for (c2_i557 = 0; c2_i557 < 4; c2_i557++) {
      c2_k1[c2_i557] = c2_C[c2_i557];
    }

    for (c2_i558 = 0; c2_i558 < 4; c2_i558++) {
      c2_k1[c2_i558] = 0.0;
      c2_i559 = 0;
      for (c2_i560 = 0; c2_i560 < 4; c2_i560++) {
        c2_k1[c2_i558] += c2_b[c2_i559 + c2_i558] * c2_b_b[c2_i560];
        c2_i559 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 198U);
    for (c2_i561 = 0; c2_i561 < 4; c2_i561++) {
      c2_b_b[c2_i561] = c2_k1[c2_i561];
    }

    for (c2_i562 = 0; c2_i562 < 4; c2_i562++) {
      c2_b_b[c2_i562] *= 0.5;
    }

    c2_c_b = c2_b_Ts;
    for (c2_i563 = 0; c2_i563 < 4; c2_i563++) {
      c2_b_b[c2_i563] *= c2_c_b;
    }

    for (c2_i564 = 0; c2_i564 < 16; c2_i564++) {
      c2_b[c2_i564] = c2_W[c2_i564];
    }

    for (c2_i565 = 0; c2_i565 < 4; c2_i565++) {
      c2_b_b[c2_i565] += c2_q[c2_i565];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i566 = 0; c2_i566 < 4; c2_i566++) {
      c2_k2[c2_i566] = 0.0;
    }

    for (c2_i567 = 0; c2_i567 < 4; c2_i567++) {
      c2_k2[c2_i567] = 0.0;
    }

    for (c2_i568 = 0; c2_i568 < 4; c2_i568++) {
      c2_C[c2_i568] = c2_k2[c2_i568];
    }

    for (c2_i569 = 0; c2_i569 < 4; c2_i569++) {
      c2_k2[c2_i569] = c2_C[c2_i569];
    }

    for (c2_i570 = 0; c2_i570 < 4; c2_i570++) {
      c2_C[c2_i570] = c2_k2[c2_i570];
    }

    for (c2_i571 = 0; c2_i571 < 4; c2_i571++) {
      c2_k2[c2_i571] = c2_C[c2_i571];
    }

    for (c2_i572 = 0; c2_i572 < 4; c2_i572++) {
      c2_k2[c2_i572] = 0.0;
      c2_i573 = 0;
      for (c2_i574 = 0; c2_i574 < 4; c2_i574++) {
        c2_k2[c2_i572] += c2_b[c2_i573 + c2_i572] * c2_b_b[c2_i574];
        c2_i573 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 199U);
    for (c2_i575 = 0; c2_i575 < 4; c2_i575++) {
      c2_b_b[c2_i575] = c2_k2[c2_i575];
    }

    for (c2_i576 = 0; c2_i576 < 4; c2_i576++) {
      c2_b_b[c2_i576] *= 0.5;
    }

    c2_d_b = c2_b_Ts;
    for (c2_i577 = 0; c2_i577 < 4; c2_i577++) {
      c2_b_b[c2_i577] *= c2_d_b;
    }

    for (c2_i578 = 0; c2_i578 < 16; c2_i578++) {
      c2_b[c2_i578] = c2_W[c2_i578];
    }

    for (c2_i579 = 0; c2_i579 < 4; c2_i579++) {
      c2_b_b[c2_i579] += c2_q[c2_i579];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i580 = 0; c2_i580 < 4; c2_i580++) {
      c2_k3[c2_i580] = 0.0;
    }

    for (c2_i581 = 0; c2_i581 < 4; c2_i581++) {
      c2_k3[c2_i581] = 0.0;
    }

    for (c2_i582 = 0; c2_i582 < 4; c2_i582++) {
      c2_C[c2_i582] = c2_k3[c2_i582];
    }

    for (c2_i583 = 0; c2_i583 < 4; c2_i583++) {
      c2_k3[c2_i583] = c2_C[c2_i583];
    }

    for (c2_i584 = 0; c2_i584 < 4; c2_i584++) {
      c2_C[c2_i584] = c2_k3[c2_i584];
    }

    for (c2_i585 = 0; c2_i585 < 4; c2_i585++) {
      c2_k3[c2_i585] = c2_C[c2_i585];
    }

    for (c2_i586 = 0; c2_i586 < 4; c2_i586++) {
      c2_k3[c2_i586] = 0.0;
      c2_i587 = 0;
      for (c2_i588 = 0; c2_i588 < 4; c2_i588++) {
        c2_k3[c2_i586] += c2_b[c2_i587 + c2_i586] * c2_b_b[c2_i588];
        c2_i587 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 200U);
    for (c2_i589 = 0; c2_i589 < 4; c2_i589++) {
      c2_a[c2_i589] = c2_k3[c2_i589];
    }

    c2_e_b = c2_b_Ts;
    for (c2_i590 = 0; c2_i590 < 4; c2_i590++) {
      c2_a[c2_i590] *= c2_e_b;
    }

    for (c2_i591 = 0; c2_i591 < 16; c2_i591++) {
      c2_b[c2_i591] = c2_W[c2_i591];
    }

    for (c2_i592 = 0; c2_i592 < 4; c2_i592++) {
      c2_a[c2_i592] += c2_q[c2_i592];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i593 = 0; c2_i593 < 4; c2_i593++) {
      c2_k4[c2_i593] = 0.0;
    }

    for (c2_i594 = 0; c2_i594 < 4; c2_i594++) {
      c2_k4[c2_i594] = 0.0;
    }

    for (c2_i595 = 0; c2_i595 < 4; c2_i595++) {
      c2_C[c2_i595] = c2_k4[c2_i595];
    }

    for (c2_i596 = 0; c2_i596 < 4; c2_i596++) {
      c2_k4[c2_i596] = c2_C[c2_i596];
    }

    for (c2_i597 = 0; c2_i597 < 4; c2_i597++) {
      c2_C[c2_i597] = c2_k4[c2_i597];
    }

    for (c2_i598 = 0; c2_i598 < 4; c2_i598++) {
      c2_k4[c2_i598] = c2_C[c2_i598];
    }

    for (c2_i599 = 0; c2_i599 < 4; c2_i599++) {
      c2_k4[c2_i599] = 0.0;
      c2_i600 = 0;
      for (c2_i601 = 0; c2_i601 < 4; c2_i601++) {
        c2_k4[c2_i599] += c2_b[c2_i600 + c2_i599] * c2_a[c2_i601];
        c2_i600 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 201U);
    for (c2_i602 = 0; c2_i602 < 4; c2_i602++) {
      c2_b_b[c2_i602] = c2_k2[c2_i602];
    }

    for (c2_i603 = 0; c2_i603 < 4; c2_i603++) {
      c2_b_b[c2_i603] *= 2.0;
    }

    for (c2_i604 = 0; c2_i604 < 4; c2_i604++) {
      c2_a[c2_i604] = c2_k3[c2_i604];
    }

    for (c2_i605 = 0; c2_i605 < 4; c2_i605++) {
      c2_a[c2_i605] *= 2.0;
    }

    for (c2_i606 = 0; c2_i606 < 4; c2_i606++) {
      c2_b_b[c2_i606] = ((c2_k1[c2_i606] + c2_b_b[c2_i606]) + c2_a[c2_i606]) +
        c2_k4[c2_i606];
    }

    c2_f_b = c2_b_Ts;
    for (c2_i607 = 0; c2_i607 < 4; c2_i607++) {
      c2_b_b[c2_i607] *= c2_f_b;
    }

    for (c2_i608 = 0; c2_i608 < 4; c2_i608++) {
      c2_b_b[c2_i608] /= 6.0;
    }

    c2_e_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i609 = 0; c2_i609 < 4; c2_i609++) {
      c2_X_kkm1[c2_i609 + 10 * c2_e_idx] = c2_q[c2_i609] + c2_b_b[c2_i609];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 202U);
    c2_f_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    c2_g_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i610 = 0; c2_i610 < 3; c2_i610++) {
      c2_X_kkm1[(c2_i610 + 10 * c2_f_idx) + 4] = c2_X_km1km1[(c2_i610 + 10 *
        c2_g_idx) + 4];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 203U);
    c2_h_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    c2_i_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_km1km1", (int32_T)
      _SFD_INTEGER_CHECK("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i611 = 0; c2_i611 < 3; c2_i611++) {
      c2_X_kkm1[(c2_i611 + 10 * c2_h_idx) + 7] = c2_X_km1km1[(c2_i611 + 10 *
        c2_i_idx) + 7];
    }

    c2_idx++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 2, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -203);
  sf_debug_symbol_scope_pop();
}

static void c2_c_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_b_rdivide(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_x[4], real_T c2_y, real_T c2_z[4])
{
  real_T c2_b_y;
  int32_T c2_i612;
  c2_b_y = c2_y;
  for (c2_i612 = 0; c2_i612 < 4; c2_i612++) {
    c2_z[c2_i612] = c2_x[c2_i612] / c2_b_y;
  }
}

static void c2_d_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[190], real_T c2_y[10])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_b_a;
  int32_T c2_c_a;
  c2_iy = 0;
  c2_ixstart = 0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_j = 1; c2_j < 11; c2_j++) {
    c2_a = c2_ixstart + 1;
    c2_ixstart = c2_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 190, 1, 0) - 1];
    c2_b_check_forloop_overflow_error(chartInstance, FALSE);
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_b_a = c2_ix + 10;
      c2_ix = c2_b_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 190, 1, 0) - 1];
    }

    c2_c_a = c2_iy + 1;
    c2_iy = c2_c_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 10, 1, 0) - 1] = c2_s;
  }
}

static void c2_e_power(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_a[4], real_T c2_y[4])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  for (c2_k = 0; c2_k < 4; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_k), 1, 4, 1, 0) - 1];
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_b_k),
      1, 4, 1, 0) - 1] = muDoubleScalarPower(c2_ak, 2.0);
  }
}

static real_T c2_e_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_x[4])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_y = c2_x[0];
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_k = 2; c2_k < 5; c2_k++) {
    c2_b_k = c2_k;
    c2_y += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 4, 1, 0) - 1];
  }

  return c2_y;
}

static void c2_quatinv(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_qin[4], real_T c2_qinv[4])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_q_conj[4];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i613;
  int32_T c2_i614;
  real_T c2_b_qin[4];
  real_T c2_x[4];
  int32_T c2_i615;
  real_T c2_b_x[4];
  real_T c2_c_x;
  real_T c2_d_x;
  int32_T c2_i616;
  real_T c2_y;
  real_T c2_b_y;
  int32_T c2_i617;
  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_q_conj, 0U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 1U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_qin, 3U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_qinv, 4U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  CV_EML_FCN(0, 6);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 285);
  c2_q_conj[0] = c2_qin[0];
  for (c2_i613 = 0; c2_i613 < 3; c2_i613++) {
    c2_q_conj[c2_i613 + 1] = -c2_qin[c2_i613 + 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 286);
  for (c2_i614 = 0; c2_i614 < 4; c2_i614++) {
    c2_b_qin[c2_i614] = c2_qin[c2_i614];
  }

  c2_b_power(chartInstance, c2_b_qin, c2_x);
  for (c2_i615 = 0; c2_i615 < 4; c2_i615++) {
    c2_b_x[c2_i615] = c2_x[c2_i615];
  }

  c2_c_x = c2_b_sum(chartInstance, c2_b_x);
  c2_d_x = c2_c_x;
  if (c2_d_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  c2_d_x = muDoubleScalarSqrt(c2_d_x);
  for (c2_i616 = 0; c2_i616 < 4; c2_i616++) {
    c2_x[c2_i616] = c2_q_conj[c2_i616];
  }

  c2_y = c2_d_x;
  c2_b_y = c2_y;
  for (c2_i617 = 0; c2_i617 < 4; c2_i617++) {
    c2_qinv[c2_i617] = c2_x[c2_i617] / c2_b_y;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -286);
  sf_debug_symbol_scope_pop();
}

static void c2_f_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[171], real_T c2_y[9])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_b_a;
  int32_T c2_c_a;
  c2_iy = 0;
  c2_ixstart = 0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 10; c2_j++) {
    c2_a = c2_ixstart + 1;
    c2_ixstart = c2_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 171, 1, 0) - 1];
    c2_b_check_forloop_overflow_error(chartInstance, FALSE);
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_b_a = c2_ix + 9;
      c2_ix = c2_b_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 171, 1, 0) - 1];
    }

    c2_c_a = c2_iy + 1;
    c2_iy = c2_c_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 9, 1, 0) - 1] = c2_s;
  }
}

static void c2_SensorModel(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_X_kkm1[190], real_T c2_B_ref[3], real_T c2_Z_kkm1
  [114])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_DCM_i2c[9];
  real_T c2_Z_kkm1_temp[4];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i618;
  int32_T c2_i619;
  int32_T c2_idx;
  real_T c2_b_idx;
  int32_T c2_c_idx;
  int32_T c2_i620;
  real_T c2_b_X_kkm1[4];
  real_T c2_dv84[4];
  int32_T c2_i621;
  real_T c2_dv85[4];
  real_T c2_dv86[4];
  int32_T c2_i622;
  int32_T c2_i623;
  real_T c2_dv87[4];
  int32_T c2_d_idx;
  int32_T c2_i624;
  real_T c2_c_X_kkm1[4];
  int32_T c2_i625;
  int32_T c2_e_idx;
  int32_T c2_i626;
  int32_T c2_f_idx;
  int32_T c2_g_idx;
  int32_T c2_h_idx;
  int32_T c2_i627;
  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c2_g_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_DCM_i2c, 0U, c2_ab_sf_marshallOut,
    c2_ab_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Z_kkm1_temp, 1U,
    c2_n_sf_marshallOut, c2_n_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_X_kkm1, 4U, c2_y_sf_marshallOut,
    c2_y_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_B_ref, 5U, c2_m_sf_marshallOut,
    c2_m_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_Z_kkm1, 6U, c2_t_sf_marshallOut,
    c2_t_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 215U);
  for (c2_i618 = 0; c2_i618 < 114; c2_i618++) {
    c2_Z_kkm1[c2_i618] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 216U);
  for (c2_i619 = 0; c2_i619 < 9; c2_i619++) {
    c2_DCM_i2c[c2_i619] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 218U);
  c2_idx = 0;
  while (c2_idx < 19) {
    c2_b_idx = 1.0 + (real_T)c2_idx;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 224U);
    c2_c_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i620 = 0; c2_i620 < 4; c2_i620++) {
      c2_b_X_kkm1[c2_i620] = c2_X_kkm1[c2_i620 + 10 * c2_c_idx];
    }

    c2_quatinv(chartInstance, c2_b_X_kkm1, c2_dv84);
    for (c2_i621 = 0; c2_i621 < 4; c2_i621++) {
      c2_dv85[c2_i621] = c2_dv84[c2_i621];
    }

    c2_dv86[0] = 0.0;
    for (c2_i622 = 0; c2_i622 < 3; c2_i622++) {
      c2_dv86[c2_i622 + 1] = c2_B_ref[c2_i622];
    }

    c2_b_quatmultiply(chartInstance, c2_dv85, c2_dv86, c2_dv84);
    for (c2_i623 = 0; c2_i623 < 4; c2_i623++) {
      c2_dv87[c2_i623] = c2_dv84[c2_i623];
    }

    c2_d_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i624 = 0; c2_i624 < 4; c2_i624++) {
      c2_c_X_kkm1[c2_i624] = c2_X_kkm1[c2_i624 + 10 * c2_d_idx];
    }

    c2_b_quatmultiply(chartInstance, c2_dv87, c2_c_X_kkm1, c2_dv84);
    for (c2_i625 = 0; c2_i625 < 4; c2_i625++) {
      c2_Z_kkm1_temp[c2_i625] = c2_dv84[c2_i625];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 225U);
    c2_e_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i626 = 0; c2_i626 < 3; c2_i626++) {
      c2_Z_kkm1[c2_i626 + 6 * c2_e_idx] = c2_Z_kkm1_temp[1 + c2_i626];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 228U);
    c2_f_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("Z_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    c2_g_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    c2_h_idx = _SFD_EML_ARRAY_BOUNDS_CHECK("X_kkm1", (int32_T)_SFD_INTEGER_CHECK
      ("idx", c2_b_idx), 1, 19, 2, 0) - 1;
    for (c2_i627 = 0; c2_i627 < 3; c2_i627++) {
      c2_Z_kkm1[(c2_i627 + 6 * c2_f_idx) + 3] = c2_X_kkm1[(c2_i627 + 10 *
        c2_g_idx) + 4] + c2_X_kkm1[(c2_i627 + 10 * c2_h_idx) + 7];
    }

    c2_idx++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -228);
  sf_debug_symbol_scope_pop();
}

static void c2_b_quatmultiply(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_q[4], real_T c2_r[4], real_T c2_qout[4])
{
  uint32_T c2_debug_family_var_map[7];
  real_T c2_vec[3];
  real_T c2_scalar;
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  real_T c2_b_q[3];
  real_T c2_b_r[3];
  real_T c2_c_q[3];
  int32_T c2_i628;
  int32_T c2_i629;
  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c2_f_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_vec, 0U, c2_gb_sf_marshallOut,
    c2_gb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_scalar, 1U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 2U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 3U, c2_o_sf_marshallOut,
    c2_o_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_q, 4U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_r, 5U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c2_qout, 6U, c2_bb_sf_marshallOut,
    c2_bb_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 237U);
  c2_b_q[0] = c2_q[0] * c2_r[1];
  c2_b_q[1] = c2_q[0] * c2_r[2];
  c2_b_q[2] = c2_q[0] * c2_r[3];
  c2_b_r[0] = c2_r[0] * c2_q[1];
  c2_b_r[1] = c2_r[0] * c2_q[2];
  c2_b_r[2] = c2_r[0] * c2_q[3];
  c2_c_q[0] = c2_q[2] * c2_r[3] - c2_q[3] * c2_r[2];
  c2_c_q[1] = c2_q[3] * c2_r[1] - c2_q[1] * c2_r[3];
  c2_c_q[2] = c2_q[1] * c2_r[2] - c2_q[2] * c2_r[1];
  for (c2_i628 = 0; c2_i628 < 3; c2_i628++) {
    c2_vec[c2_i628] = (c2_b_q[c2_i628] + c2_b_r[c2_i628]) + c2_c_q[c2_i628];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 245U);
  c2_scalar = ((c2_q[0] * c2_r[0] - c2_q[1] * c2_r[1]) - c2_q[2] * c2_r[2]) -
    c2_q[3] * c2_r[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  c2_qout[0] = c2_scalar;
  for (c2_i629 = 0; c2_i629 < 3; c2_i629++) {
    c2_qout[c2_i629 + 1] = c2_vec[c2_i629];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -248);
  sf_debug_symbol_scope_pop();
}

static void c2_g_sum(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                     real_T c2_x[114], real_T c2_y[6])
{
  int32_T c2_iy;
  int32_T c2_ixstart;
  int32_T c2_j;
  int32_T c2_a;
  int32_T c2_ix;
  real_T c2_s;
  int32_T c2_k;
  int32_T c2_b_a;
  int32_T c2_c_a;
  c2_iy = 0;
  c2_ixstart = 0;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_a = c2_ixstart + 1;
    c2_ixstart = c2_a;
    c2_ix = c2_ixstart;
    c2_s = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_ix), 1, 114, 1, 0) - 1];
    c2_b_check_forloop_overflow_error(chartInstance, FALSE);
    for (c2_k = 2; c2_k < 20; c2_k++) {
      c2_b_a = c2_ix + 6;
      c2_ix = c2_b_a;
      c2_s += c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 114, 1, 0) - 1];
    }

    c2_c_a = c2_iy + 1;
    c2_iy = c2_c_a;
    c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 6, 1, 0) - 1] = c2_s;
  }
}

static void c2_mrdivide(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  real_T c2_A[54], real_T c2_B[36], real_T c2_y[54])
{
  int32_T c2_i630;
  int32_T c2_i631;
  int32_T c2_i632;
  int32_T c2_i633;
  real_T c2_b_A[36];
  int32_T c2_i634;
  int32_T c2_i635;
  int32_T c2_i636;
  int32_T c2_i637;
  real_T c2_b_B[54];
  int32_T c2_info;
  int32_T c2_ipiv[6];
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_ip;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_temp;
  int32_T c2_i638;
  real_T c2_c_A[36];
  int32_T c2_i639;
  real_T c2_d_A[36];
  int32_T c2_i640;
  int32_T c2_i641;
  int32_T c2_i642;
  int32_T c2_i643;
  c2_i630 = 0;
  for (c2_i631 = 0; c2_i631 < 6; c2_i631++) {
    c2_i632 = 0;
    for (c2_i633 = 0; c2_i633 < 6; c2_i633++) {
      c2_b_A[c2_i633 + c2_i630] = c2_B[c2_i632 + c2_i631];
      c2_i632 += 6;
    }

    c2_i630 += 6;
  }

  c2_i634 = 0;
  for (c2_i635 = 0; c2_i635 < 9; c2_i635++) {
    c2_i636 = 0;
    for (c2_i637 = 0; c2_i637 < 6; c2_i637++) {
      c2_b_B[c2_i637 + c2_i634] = c2_A[c2_i636 + c2_i635];
      c2_i636 += 9;
    }

    c2_i634 += 6;
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, &c2_info);
  c2_b_info = c2_info;
  c2_c_info = c2_b_info;
  c2_d_info = c2_c_info;
  if (c2_d_info > 0) {
    c2_eml_warning(chartInstance);
  }

  c2_check_forloop_overflow_error(chartInstance);
  for (c2_i = 1; c2_i < 7; c2_i++) {
    c2_b_i = c2_i;
    if (c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 6, 1, 0) - 1] != c2_b_i) {
      c2_ip = c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 6, 1, 0) - 1];
      c2_check_forloop_overflow_error(chartInstance);
      for (c2_j = 1; c2_j < 10; c2_j++) {
        c2_b_j = c2_j;
        c2_temp = c2_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 6, 1, 0) + 6 *
                          (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1];
        c2_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_b_i), 1, 6, 1, 0) + 6 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = c2_b_B
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_ip), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0) - 1))
          - 1];
        c2_b_B[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_ip), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK
                 ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2,
                  0) - 1)) - 1] = c2_temp;
      }
    }
  }

  for (c2_i638 = 0; c2_i638 < 36; c2_i638++) {
    c2_c_A[c2_i638] = c2_b_A[c2_i638];
  }

  c2_c_eml_xtrsm(chartInstance, c2_c_A, c2_b_B);
  for (c2_i639 = 0; c2_i639 < 36; c2_i639++) {
    c2_d_A[c2_i639] = c2_b_A[c2_i639];
  }

  c2_d_eml_xtrsm(chartInstance, c2_d_A, c2_b_B);
  c2_i640 = 0;
  for (c2_i641 = 0; c2_i641 < 6; c2_i641++) {
    c2_i642 = 0;
    for (c2_i643 = 0; c2_i643 < 9; c2_i643++) {
      c2_y[c2_i643 + c2_i640] = c2_b_B[c2_i642 + c2_i641];
      c2_i642 += 6;
    }

    c2_i640 += 9;
  }
}

static void c2_realmin(SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static void c2_eps(SFc2_SatelliteControlSystemInstanceStruct *chartInstance)
{
}

static void c2_eml_matlab_zgetrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_b_A[36], int32_T c2_ipiv[6],
  int32_T *c2_info)
{
  int32_T c2_i644;
  for (c2_i644 = 0; c2_i644 < 36; c2_i644++) {
    c2_b_A[c2_i644] = c2_A[c2_i644];
  }

  c2_b_eml_matlab_zgetrf(chartInstance, c2_b_A, c2_ipiv, c2_info);
}

static void c2_eml_xger(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, int32_T c2_iy0,
  real_T c2_A[36], int32_T c2_ia0, real_T c2_b_A[36])
{
  int32_T c2_i645;
  for (c2_i645 = 0; c2_i645 < 36; c2_i645++) {
    c2_b_A[c2_i645] = c2_A[c2_i645];
  }

  c2_b_eml_xger(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_iy0, c2_b_A,
                c2_ia0);
}

static void c2_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54], real_T c2_b_B[54])
{
  int32_T c2_i646;
  int32_T c2_i647;
  real_T c2_b_A[36];
  for (c2_i646 = 0; c2_i646 < 54; c2_i646++) {
    c2_b_B[c2_i646] = c2_B[c2_i646];
  }

  for (c2_i647 = 0; c2_i647 < 36; c2_i647++) {
    c2_b_A[c2_i647] = c2_A[c2_i647];
  }

  c2_c_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_below_threshold(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_d_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54], real_T c2_b_B[54])
{
  int32_T c2_i648;
  int32_T c2_i649;
  real_T c2_b_A[36];
  for (c2_i648 = 0; c2_i648 < 54; c2_i648++) {
    c2_b_B[c2_i648] = c2_B[c2_i648];
  }

  for (c2_i649 = 0; c2_i649 < 36; c2_i649++) {
    c2_b_A[c2_i649] = c2_A[c2_i649];
  }

  c2_d_eml_xtrsm(chartInstance, c2_b_A, c2_b_B);
}

static void c2_e_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_f_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static void c2_g_eml_scalar_eg(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

static const mxArray *c2_hb_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_vb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i650;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i650, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i650;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_hb_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_vb_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_wb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_b_is_active_c2_SatelliteControlSystem, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_xb_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_SatelliteControlSystem), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_SatelliteControlSystem);
  return c2_y;
}

static uint8_T c2_xb_emlrt_marshallIn(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_chol(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_A[81])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_info;
  int32_T c2_b_info;
  int32_T c2_c_info;
  int32_T c2_d_info;
  int32_T c2_jmax;
  int32_T c2_a;
  int32_T c2_b_jmax;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_c_j;
  int32_T c2_b_a;
  int32_T c2_i651;
  int32_T c2_c_jmax;
  int32_T c2_c_a;
  int32_T c2_c_b;
  int32_T c2_d_a;
  int32_T c2_d_b;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 10; c2_j++) {
    c2_b_j = c2_j;
  }

  c2_info = c2_b_eml_matlab_zpotrf(chartInstance, c2_A);
  c2_b_info = c2_info;
  c2_c_info = c2_b_info;
  c2_d_info = c2_c_info;
  if (c2_d_info == 0) {
    c2_jmax = 9;
  } else {
    c2_c_eml_error(chartInstance);
    c2_a = c2_d_info - 1;
    c2_jmax = c2_a;
  }

  c2_b_jmax = c2_jmax;
  c2_b = c2_b_jmax;
  c2_b_b = c2_b;
  if (1 > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
  for (c2_c_j = 1; c2_c_j <= c2_b_jmax; c2_c_j++) {
    c2_b_j = c2_c_j;
    c2_b_a = c2_b_j + 1;
    c2_i651 = c2_b_a;
    c2_c_jmax = c2_jmax;
    c2_c_a = c2_i651;
    c2_c_b = c2_c_jmax;
    c2_d_a = c2_c_a;
    c2_d_b = c2_c_b;
    if (c2_d_a > c2_d_b) {
      c2_b_overflow = FALSE;
    } else {
      c2_b_overflow = (c2_d_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    for (c2_i = c2_i651; c2_i <= c2_c_jmax; c2_i++) {
      c2_b_i = c2_i;
      c2_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_b_i), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0) - 1))
        - 1] = 0.0;
    }
  }
}

static int32_T c2_b_eml_matlab_zpotrf(SFc2_SatelliteControlSystemInstanceStruct *
  chartInstance, real_T c2_A[81])
{
  int32_T c2_info;
  int32_T c2_colj;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b_a;
  int32_T c2_b;
  int32_T c2_jj;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_iy0;
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
  real_T c2_d;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_f_n;
  int32_T c2_b_b;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_c_a;
  int32_T c2_d_a;
  real_T c2_ajj;
  int32_T c2_d_b;
  int32_T c2_nmj;
  int32_T c2_e_a;
  int32_T c2_jjp1;
  int32_T c2_f_a;
  int32_T c2_coljp1;
  int32_T c2_b_jm1;
  int32_T c2_e_b;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_b_k;
  int32_T c2_c_k;
  int32_T c2_c_jm1;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  int32_T c2_d_k;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_g_n;
  real_T c2_g_a;
  int32_T c2_f_ix0;
  int32_T c2_h_n;
  real_T c2_h_a;
  int32_T c2_g_ix0;
  int32_T c2_i_n;
  real_T c2_i_a;
  int32_T c2_h_ix0;
  int32_T c2_i_ix0;
  int32_T c2_j_a;
  int32_T c2_c;
  int32_T c2_i_b;
  int32_T c2_b_c;
  int32_T c2_k_a;
  int32_T c2_j_b;
  int32_T c2_i652;
  int32_T c2_l_a;
  int32_T c2_k_b;
  int32_T c2_m_a;
  int32_T c2_l_b;
  boolean_T c2_d_overflow;
  int32_T c2_e_k;
  int32_T c2_f_k;
  boolean_T exitg1;
  c2_info = 0;
  c2_b_eml_scalar_eg(chartInstance);
  c2_colj = 1;
  c2_check_forloop_overflow_error(chartInstance);
  c2_j = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_j < 10)) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b_a = c2_colj;
    c2_b = c2_jm1;
    c2_jj = c2_b_a + c2_b;
    c2_n = c2_jm1;
    c2_ix0 = c2_colj;
    c2_iy0 = c2_colj;
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
      c2_b_b = c2_f_n;
      c2_c_b = c2_b_b;
      if (1 > c2_c_b) {
        c2_overflow = FALSE;
      } else {
        c2_overflow = (c2_c_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
      for (c2_k = 1; c2_k <= c2_f_n; c2_k++) {
        c2_d += c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
          ("", (real_T)c2_ix), 1, 81, 1, 0) - 1] *
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_iy), 1, 81, 1, 0) - 1];
        c2_c_a = c2_ix + 1;
        c2_ix = c2_c_a;
        c2_d_a = c2_iy + 1;
        c2_iy = c2_d_a;
      }
    }

    c2_ajj = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_jj), 1, 81, 1, 0) - 1] - c2_d;
    if (c2_ajj > 0.0) {
      c2_ajj = muDoubleScalarSqrt(c2_ajj);
      c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_jj), 1, 81, 1, 0) - 1] = c2_ajj;
      if (c2_b_j < 9) {
        c2_d_b = c2_b_j;
        c2_nmj = 9 - c2_d_b;
        c2_e_a = c2_jj + 9;
        c2_jjp1 = c2_e_a;
        c2_f_a = c2_colj + 9;
        c2_coljp1 = c2_f_a;
        c2_b_jm1 = c2_jm1;
        c2_e_b = c2_b_jm1;
        c2_f_b = c2_e_b;
        if (1 > c2_f_b) {
          c2_b_overflow = FALSE;
        } else {
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        for (c2_b_k = 1; c2_b_k <= c2_b_jm1; c2_b_k++) {
          c2_c_k = c2_b_k;
          c2_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_c_k), 1, 9, 1, 0) + 9 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_c_k), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0)
               - 1)) - 1];
        }

        c2_b_eml_xgemv(chartInstance, c2_jm1, c2_nmj, c2_coljp1, c2_colj, c2_A,
                       c2_jjp1);
        c2_c_jm1 = c2_jm1;
        c2_g_b = c2_c_jm1;
        c2_h_b = c2_g_b;
        if (1 > c2_h_b) {
          c2_c_overflow = FALSE;
        } else {
          c2_c_overflow = (c2_h_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        for (c2_d_k = 1; c2_d_k <= c2_c_jm1; c2_d_k++) {
          c2_c_k = c2_d_k;
          c2_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c2_c_k), 1, 9, 1, 0) + 9 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c2_b_j), 1, 9, 2, 0) - 1)) - 1] = c2_A
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_c_k), 1, 9, 1, 0) + 9 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 9, 2, 0)
               - 1)) - 1];
        }

        c2_y = c2_ajj;
        c2_z = 1.0 / c2_y;
        c2_g_n = c2_nmj;
        c2_g_a = c2_z;
        c2_f_ix0 = c2_jjp1;
        c2_h_n = c2_g_n;
        c2_h_a = c2_g_a;
        c2_g_ix0 = c2_f_ix0;
        c2_i_n = c2_h_n;
        c2_i_a = c2_h_a;
        c2_h_ix0 = c2_g_ix0;
        c2_i_ix0 = c2_h_ix0;
        c2_j_a = c2_i_n;
        c2_c = c2_j_a;
        c2_i_b = c2_c - 1;
        c2_b_c = 9 * c2_i_b;
        c2_k_a = c2_h_ix0;
        c2_j_b = c2_b_c;
        c2_i652 = c2_k_a + c2_j_b;
        c2_l_a = c2_i_ix0;
        c2_k_b = c2_i652;
        c2_m_a = c2_l_a;
        c2_l_b = c2_k_b;
        if (c2_m_a > c2_l_b) {
          c2_d_overflow = FALSE;
        } else {
          c2_d_overflow = (c2_l_b > 2147483638);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_d_overflow);
        for (c2_e_k = c2_i_ix0; c2_e_k <= c2_i652; c2_e_k += 9) {
          c2_f_k = c2_e_k;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_k), 1, 81, 1, 0) - 1] = c2_i_a *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_k), 1, 81, 1, 0) - 1];
        }

        c2_colj = c2_coljp1;
      }

      c2_j++;
    } else {
      c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_jj), 1, 81, 1, 0) - 1] = c2_ajj;
      c2_info = c2_b_j;
      exitg1 = TRUE;
    }
  }

  return c2_info;
}

static void c2_b_eml_xgemv(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_ia0, int32_T c2_ix0,
  real_T c2_y[81], int32_T c2_iy0)
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
  int32_T c2_lda;
  int32_T c2_c_ix0;
  int32_T c2_incx;
  real_T c2_beta1;
  int32_T c2_c_iy0;
  int32_T c2_incy;
  char_T c2_TRANSA;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_ia0 = c2_ia0;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  if (c2_b_m < 1) {
  } else if (c2_b_n < 1) {
  } else {
    c2_c_m = c2_b_m;
    c2_c_n = c2_b_n;
    c2_alpha1 = -1.0;
    c2_c_ia0 = c2_b_ia0;
    c2_lda = 9;
    c2_c_ix0 = c2_b_ix0;
    c2_incx = 1;
    c2_beta1 = 1.0;
    c2_c_iy0 = c2_b_iy0;
    c2_incy = 9;
    c2_TRANSA = 'T';
    dgemv32(&c2_TRANSA, &c2_c_m, &c2_c_n, &c2_alpha1,
            &c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_c_ia0), 1, 81, 1, 0) - 1], &c2_lda,
            &c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_c_ix0), 1, 81, 1, 0) - 1], &c2_incx, &c2_beta1,
            &c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_c_iy0), 1, 81, 1, 0) - 1], &c2_incy);
  }
}

static void c2_d_sqrt(SFc2_SatelliteControlSystemInstanceStruct *chartInstance,
                      real_T c2_x[19])
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

static void c2_b_eml_matlab_zgetrf(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], int32_T c2_ipiv[6], int32_T *c2_info)
{
  int32_T c2_i653;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_jm1;
  int32_T c2_b;
  int32_T c2_mmj;
  int32_T c2_b_a;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_jj;
  int32_T c2_c_a;
  int32_T c2_jp1j;
  int32_T c2_d_a;
  int32_T c2_b_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_idxmax;
  int32_T c2_ix;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_e_a;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_d_y;
  real_T c2_s;
  int32_T c2_f_a;
  int32_T c2_jpiv_offset;
  int32_T c2_g_a;
  int32_T c2_e_b;
  int32_T c2_jpiv;
  int32_T c2_h_a;
  int32_T c2_f_b;
  int32_T c2_c_c;
  int32_T c2_g_b;
  int32_T c2_jrow;
  int32_T c2_i_a;
  int32_T c2_h_b;
  int32_T c2_jprow;
  int32_T c2_d_ix0;
  int32_T c2_iy0;
  int32_T c2_e_ix0;
  int32_T c2_b_iy0;
  int32_T c2_f_ix0;
  int32_T c2_c_iy0;
  int32_T c2_b_ix;
  int32_T c2_iy;
  int32_T c2_c_k;
  real_T c2_temp;
  int32_T c2_j_a;
  int32_T c2_k_a;
  int32_T c2_b_jp1j;
  int32_T c2_l_a;
  int32_T c2_d_c;
  int32_T c2_m_a;
  int32_T c2_i_b;
  int32_T c2_i654;
  int32_T c2_n_a;
  int32_T c2_j_b;
  int32_T c2_o_a;
  int32_T c2_k_b;
  boolean_T c2_b_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_k_x;
  real_T c2_e_y;
  real_T c2_z;
  int32_T c2_l_b;
  int32_T c2_e_c;
  int32_T c2_p_a;
  int32_T c2_f_c;
  int32_T c2_q_a;
  int32_T c2_g_c;
  int32_T c2_m;
  int32_T c2_e_n;
  int32_T c2_g_ix0;
  int32_T c2_d_iy0;
  int32_T c2_ia0;
  real_T c2_d11;
  c2_realmin(chartInstance);
  c2_eps(chartInstance);
  for (c2_i653 = 0; c2_i653 < 6; c2_i653++) {
    c2_ipiv[c2_i653] = 1 + c2_i653;
  }

  *c2_info = 0;
  c2_b_check_forloop_overflow_error(chartInstance, FALSE);
  for (c2_j = 1; c2_j < 6; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j - 1;
    c2_jm1 = c2_a;
    c2_b = c2_b_j;
    c2_mmj = 6 - c2_b;
    c2_b_a = c2_jm1;
    c2_c = c2_b_a * 7;
    c2_b_b = c2_c + 1;
    c2_jj = c2_b_b;
    c2_c_a = c2_jj + 1;
    c2_jp1j = c2_c_a;
    c2_d_a = c2_mmj;
    c2_b_c = c2_d_a;
    c2_n = c2_b_c + 1;
    c2_ix0 = c2_jj;
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
        c2_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_ix), 1, 36, 1, 0) - 1];
        c2_b_x = c2_x;
        c2_c_x = c2_b_x;
        c2_y = muDoubleScalarAbs(c2_c_x);
        c2_d_x = 0.0;
        c2_e_x = c2_d_x;
        c2_b_y = muDoubleScalarAbs(c2_e_x);
        c2_smax = c2_y + c2_b_y;
        c2_d_n = c2_c_n;
        c2_c_b = c2_d_n;
        c2_d_b = c2_c_b;
        if (2 > c2_d_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_d_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
        for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
          c2_b_k = c2_k;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
          c2_f_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 36, 1, 0) - 1];
          c2_g_x = c2_f_x;
          c2_h_x = c2_g_x;
          c2_c_y = muDoubleScalarAbs(c2_h_x);
          c2_i_x = 0.0;
          c2_j_x = c2_i_x;
          c2_d_y = muDoubleScalarAbs(c2_j_x);
          c2_s = c2_c_y + c2_d_y;
          if (c2_s > c2_smax) {
            c2_idxmax = c2_b_k;
            c2_smax = c2_s;
          }
        }
      }
    }

    c2_f_a = c2_idxmax - 1;
    c2_jpiv_offset = c2_f_a;
    c2_g_a = c2_jj;
    c2_e_b = c2_jpiv_offset;
    c2_jpiv = c2_g_a + c2_e_b;
    if (c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if (c2_jpiv_offset != 0) {
        c2_h_a = c2_b_j;
        c2_f_b = c2_jpiv_offset;
        c2_c_c = c2_h_a + c2_f_b;
        c2_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 6, 1, 0) - 1] = c2_c_c;
        c2_g_b = c2_jm1 + 1;
        c2_jrow = c2_g_b;
        c2_i_a = c2_jrow;
        c2_h_b = c2_jpiv_offset;
        c2_jprow = c2_i_a + c2_h_b;
        c2_d_ix0 = c2_jrow;
        c2_iy0 = c2_jprow;
        c2_e_ix0 = c2_d_ix0;
        c2_b_iy0 = c2_iy0;
        c2_f_ix0 = c2_e_ix0;
        c2_c_iy0 = c2_b_iy0;
        c2_b_ix = c2_f_ix0;
        c2_iy = c2_c_iy0;
        c2_check_forloop_overflow_error(chartInstance);
        for (c2_c_k = 1; c2_c_k < 7; c2_c_k++) {
          c2_temp = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_ix), 1, 36, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ix), 1, 36, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 36, 1, 0) - 1];
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_iy), 1, 36, 1, 0) - 1] = c2_temp;
          c2_j_a = c2_b_ix + 6;
          c2_b_ix = c2_j_a;
          c2_k_a = c2_iy + 6;
          c2_iy = c2_k_a;
        }
      }

      c2_b_jp1j = c2_jp1j;
      c2_l_a = c2_mmj;
      c2_d_c = c2_l_a;
      c2_m_a = c2_jp1j;
      c2_i_b = c2_d_c - 1;
      c2_i654 = c2_m_a + c2_i_b;
      c2_n_a = c2_b_jp1j;
      c2_j_b = c2_i654;
      c2_o_a = c2_n_a;
      c2_k_b = c2_j_b;
      if (c2_o_a > c2_k_b) {
        c2_b_overflow = FALSE;
      } else {
        c2_b_overflow = (c2_k_b > 2147483646);
      }

      c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      for (c2_i = c2_b_jp1j; c2_i <= c2_i654; c2_i++) {
        c2_b_i = c2_i;
        c2_k_x = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 36, 1, 0) - 1];
        c2_e_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_jj), 1, 36, 1, 0) - 1];
        c2_z = c2_k_x / c2_e_y;
        c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_i), 1, 36, 1, 0) - 1] = c2_z;
      }
    } else {
      *c2_info = c2_b_j;
    }

    c2_l_b = c2_b_j;
    c2_e_c = 6 - c2_l_b;
    c2_p_a = c2_jj;
    c2_f_c = c2_p_a;
    c2_q_a = c2_jj;
    c2_g_c = c2_q_a;
    c2_m = c2_mmj;
    c2_e_n = c2_e_c;
    c2_g_ix0 = c2_jp1j;
    c2_d_iy0 = c2_f_c + 6;
    c2_ia0 = c2_g_c + 7;
    c2_d11 = -1.0;
    c2_b_eml_xger(chartInstance, c2_m, c2_e_n, c2_d11, c2_g_ix0, c2_d_iy0, c2_A,
                  c2_ia0);
  }

  if (*c2_info == 0) {
    if (!(c2_A[35] != 0.0)) {
      *c2_info = 6;
    }
  }
}

static void c2_b_eml_xger(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0,
  int32_T c2_iy0, real_T c2_A[36], int32_T c2_ia0)
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
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_e_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_c_b;
  int32_T c2_i655;
  int32_T c2_b_a;
  int32_T c2_d_b;
  int32_T c2_i656;
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
  if (c2_d_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_d_ix0;
    c2_a = c2_d_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = c2_d_iy0;
    c2_e_n = c2_d_n;
    c2_b = c2_e_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
    for (c2_j = 1; c2_j <= c2_e_n; c2_j++) {
      c2_yjy = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 36, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_d_alpha1;
        c2_ix = c2_ixstart;
        c2_c_b = c2_jA + 1;
        c2_i655 = c2_c_b;
        c2_b_a = c2_d_m;
        c2_d_b = c2_jA;
        c2_i656 = c2_b_a + c2_d_b;
        c2_c_a = c2_i655;
        c2_e_b = c2_i656;
        c2_d_a = c2_c_a;
        c2_f_b = c2_e_b;
        if (c2_d_a > c2_f_b) {
          c2_b_overflow = FALSE;
        } else {
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        for (c2_ijA = c2_i655; c2_ijA <= c2_i656; c2_ijA++) {
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

static void c2_c_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_jBcol;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_b;
  int32_T c2_d_c;
  int32_T c2_d_b;
  int32_T c2_kAcol;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_e_c;
  int32_T c2_d_a;
  int32_T c2_i657;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_e_a;
  int32_T c2_f_b;
  int32_T c2_f_c;
  int32_T c2_f_a;
  int32_T c2_g_b;
  int32_T c2_g_c;
  int32_T c2_g_a;
  int32_T c2_h_b;
  int32_T c2_h_c;
  int32_T c2_h_a;
  int32_T c2_i_b;
  int32_T c2_i_c;
  c2_below_threshold(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 10; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = 6 * c2_b;
    c2_b_b = c2_b_c;
    c2_jBcol = c2_b_b;
    c2_check_forloop_overflow_error(chartInstance);
    for (c2_k = 1; c2_k < 7; c2_k++) {
      c2_b_k = c2_k;
      c2_b_a = c2_b_k;
      c2_c_c = c2_b_a;
      c2_c_b = c2_c_c - 1;
      c2_d_c = 6 * c2_c_b;
      c2_d_b = c2_d_c;
      c2_kAcol = c2_d_b;
      c2_c_a = c2_b_k;
      c2_e_b = c2_jBcol;
      c2_e_c = c2_c_a + c2_e_b;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_c), 1, 54, 1, 0) - 1] != 0.0) {
        c2_d_a = c2_b_k;
        c2_i657 = c2_d_a;
        c2_overflow = FALSE;
        c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
        for (c2_i = c2_i657 + 1; c2_i < 7; c2_i++) {
          c2_b_i = c2_i;
          c2_e_a = c2_b_i;
          c2_f_b = c2_jBcol;
          c2_f_c = c2_e_a + c2_f_b;
          c2_f_a = c2_b_i;
          c2_g_b = c2_jBcol;
          c2_g_c = c2_f_a + c2_g_b;
          c2_g_a = c2_b_k;
          c2_h_b = c2_jBcol;
          c2_h_c = c2_g_a + c2_h_b;
          c2_h_a = c2_b_i;
          c2_i_b = c2_kAcol;
          c2_i_c = c2_h_a + c2_i_b;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_f_c), 1, 54, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_g_c), 1, 54, 1, 0) - 1] -
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_h_c), 1, 54, 1, 0) - 1] *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c2_d_eml_xtrsm(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance, real_T c2_A[36], real_T c2_B[54])
{
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_b;
  int32_T c2_jBcol;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_b_a;
  int32_T c2_c_c;
  int32_T c2_c_b;
  int32_T c2_d_c;
  int32_T c2_d_b;
  int32_T c2_kAcol;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_e_c;
  int32_T c2_d_a;
  int32_T c2_f_b;
  int32_T c2_f_c;
  int32_T c2_e_a;
  int32_T c2_g_b;
  int32_T c2_g_c;
  int32_T c2_f_a;
  int32_T c2_h_b;
  int32_T c2_h_c;
  real_T c2_x;
  real_T c2_y;
  real_T c2_z;
  int32_T c2_g_a;
  int32_T c2_i658;
  int32_T c2_i_b;
  int32_T c2_j_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_h_a;
  int32_T c2_k_b;
  int32_T c2_i_c;
  int32_T c2_i_a;
  int32_T c2_l_b;
  int32_T c2_j_c;
  int32_T c2_j_a;
  int32_T c2_m_b;
  int32_T c2_k_c;
  int32_T c2_k_a;
  int32_T c2_n_b;
  int32_T c2_l_c;
  c2_below_threshold(chartInstance);
  c2_d_eml_scalar_eg(chartInstance);
  c2_check_forloop_overflow_error(chartInstance);
  for (c2_j = 1; c2_j < 10; c2_j++) {
    c2_b_j = c2_j;
    c2_a = c2_b_j;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = 6 * c2_b;
    c2_b_b = c2_b_c;
    c2_jBcol = c2_b_b;
    c2_check_forloop_overflow_error(chartInstance);
    for (c2_k = 6; c2_k > 0; c2_k--) {
      c2_b_k = c2_k;
      c2_b_a = c2_b_k;
      c2_c_c = c2_b_a;
      c2_c_b = c2_c_c - 1;
      c2_d_c = 6 * c2_c_b;
      c2_d_b = c2_d_c;
      c2_kAcol = c2_d_b;
      c2_c_a = c2_b_k;
      c2_e_b = c2_jBcol;
      c2_e_c = c2_c_a + c2_e_b;
      if (c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_e_c), 1, 54, 1, 0) - 1] != 0.0) {
        c2_d_a = c2_b_k;
        c2_f_b = c2_jBcol;
        c2_f_c = c2_d_a + c2_f_b;
        c2_e_a = c2_b_k;
        c2_g_b = c2_jBcol;
        c2_g_c = c2_e_a + c2_g_b;
        c2_f_a = c2_b_k;
        c2_h_b = c2_kAcol;
        c2_h_c = c2_f_a + c2_h_b;
        c2_x = c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_g_c), 1, 54, 1, 0) - 1];
        c2_y = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c2_h_c), 1, 36, 1, 0) - 1];
        c2_z = c2_x / c2_y;
        c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_f_c), 1, 54, 1, 0) - 1] = c2_z;
        c2_g_a = c2_b_k - 1;
        c2_i658 = c2_g_a;
        c2_i_b = c2_i658;
        c2_j_b = c2_i_b;
        if (1 > c2_j_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_j_b > 2147483646);
        }

        c2_b_check_forloop_overflow_error(chartInstance, c2_overflow);
        for (c2_i = 1; c2_i <= c2_i658; c2_i++) {
          c2_b_i = c2_i;
          c2_h_a = c2_b_i;
          c2_k_b = c2_jBcol;
          c2_i_c = c2_h_a + c2_k_b;
          c2_i_a = c2_b_i;
          c2_l_b = c2_jBcol;
          c2_j_c = c2_i_a + c2_l_b;
          c2_j_a = c2_b_k;
          c2_m_b = c2_jBcol;
          c2_k_c = c2_j_a + c2_m_b;
          c2_k_a = c2_b_i;
          c2_n_b = c2_kAcol;
          c2_l_c = c2_k_a + c2_n_b;
          c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i_c), 1, 54, 1, 0) - 1] =
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_j_c), 1, 54, 1, 0) - 1] -
            c2_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_k_c), 1, 54, 1, 0) - 1] *
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_l_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void init_dsm_address_info(SFc2_SatelliteControlSystemInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_SatelliteControlSystem_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2915128976U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(440766821U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2015799754U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2695053520U);
}

mxArray *sf_c2_SatelliteControlSystem_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("H0cIFiKNSSFBaw5vcmOoc");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c2_SatelliteControlSystem(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"Attitude_sensor\",},{M[1],M[9],T\"w_bias_sensor\",},{M[1],M[10],T\"w_sensor\",},{M[4],M[0],T\"Beta\",S'l','i','p'{{M1x2[790 794],M[0],}}},{M[4],M[0],T\"DCM_s2c\",S'l','i','p'{{M1x2[756 763],M[0],}}},{M[4],M[0],T\"K\",S'l','i','p'{{M1x2[788 789],M[0],}}},{M[4],M[0],T\"Lambda\",S'l','i','p'{{M1x2[775 781],M[0],}}},{M[4],M[0],T\"P_km1km1\",S'l','i','p'{{M1x2[727 735],M[0],}}},{M[4],M[0],T\"Q_k\",S'l','i','p'{{M1x2[771 774],M[0],}}},{M[4],M[0],T\"R_k\",S'l','i','p'{{M1x2[767 770],M[0],}}}}",
    "100 S1x6'type','srcId','name','auxInfo'{{M[4],M[0],T\"Ts\",S'l','i','p'{{M1x2[764 766],M[0],}}},{M[4],M[0],T\"alpha\",S'l','i','p'{{M1x2[782 787],M[0],}}},{M[4],M[0],T\"q_s_c\",S'l','i','p'{{M1x2[745 750],M[0],}}},{M[4],M[0],T\"x_kk\",S'l','i','p'{{M1x2[751 755],M[0],}}},{M[4],M[0],T\"x_km1km1\",S'l','i','p'{{M1x2[736 744],M[0],}}},{M[8],M[0],T\"is_active_c2_SatelliteControlSystem\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 16, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_SatelliteControlSystem_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
    chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_SatelliteControlSystemMachineNumber_,
           2,
           1,
           1,
           6,
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
          init_script_number_translation(_SatelliteControlSystemMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_SatelliteControlSystemMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds
            (_SatelliteControlSystemMachineNumber_,
             chartInstance->chartNumber,
             0,
             0,
             0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"B_ref");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Attitude_sensor");
          _SFD_SET_DATA_PROPS(2,2,0,1,"w_sensor");
          _SFD_SET_DATA_PROPS(3,1,1,0,"B_sat");
          _SFD_SET_DATA_PROPS(4,1,1,0,"w_gyro");
          _SFD_SET_DATA_PROPS(5,2,0,1,"w_bias_sensor");
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
        _SFD_CV_INIT_EML(0,1,7,3,0,0,0,4,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,6583);
        _SFD_CV_INIT_EML_FCN(0,1,"RK4",6585,-1,7695);
        _SFD_CV_INIT_EML_FCN(0,2,"SensorModel",7697,-1,8529);
        _SFD_CV_INIT_EML_FCN(0,3,"quatmultiply",8531,-1,9073);
        _SFD_CV_INIT_EML_FCN(0,4,"quat2dcm",9075,-1,9703);
        _SFD_CV_INIT_EML_FCN(0,5,"dcm2quat",9705,-1,9986);
        _SFD_CV_INIT_EML_FCN(0,6,"quatinv",9988,-1,10085);
        _SFD_CV_INIT_EML_IF(0,1,0,830,850,1781,6015);
        _SFD_CV_INIT_EML_IF(0,1,1,3501,3510,3627,3751);
        _SFD_CV_INIT_EML_IF(0,1,2,4410,4419,4685,4959);
        _SFD_CV_INIT_EML_FOR(0,1,0,3477,3493,3759);
        _SFD_CV_INIT_EML_FOR(0,1,1,4386,4402,4967);
        _SFD_CV_INIT_EML_FOR(0,1,2,6971,7000,7677);
        _SFD_CV_INIT_EML_FOR(0,1,3,8064,8091,8525);
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
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_n_sf_marshallOut,(MexInFcnForType)
            c2_n_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)
            c2_m_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_m_sf_marshallOut,(MexInFcnForType)
            c2_m_sf_marshallIn);
        }

        {
          real_T (*c2_B_ref)[3];
          real_T (*c2_Attitude_sensor)[4];
          real_T (*c2_w_sensor)[3];
          real_T (*c2_B_sat)[3];
          real_T (*c2_w_gyro)[3];
          real_T (*c2_w_bias_sensor)[3];
          c2_w_bias_sensor = (real_T (*)[3])ssGetOutputPortSignal
            (chartInstance->S, 3);
          c2_w_gyro = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 2);
          c2_B_sat = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_w_sensor = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_Attitude_sensor = (real_T (*)[4])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c2_B_ref = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_B_ref);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Attitude_sensor);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_w_sensor);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_B_sat);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_w_gyro);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_w_bias_sensor);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_SatelliteControlSystemMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "2kwO2L27aWhIWVXtRU1kz";
}

static void sf_opaque_initialize_c2_SatelliteControlSystem(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_SatelliteControlSystemInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_SatelliteControlSystem
    ((SFc2_SatelliteControlSystemInstanceStruct*) chartInstanceVar);
  initialize_c2_SatelliteControlSystem
    ((SFc2_SatelliteControlSystemInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_SatelliteControlSystem(void *chartInstanceVar)
{
  enable_c2_SatelliteControlSystem((SFc2_SatelliteControlSystemInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_SatelliteControlSystem(void *chartInstanceVar)
{
  disable_c2_SatelliteControlSystem((SFc2_SatelliteControlSystemInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_SatelliteControlSystem(void *chartInstanceVar)
{
  sf_c2_SatelliteControlSystem((SFc2_SatelliteControlSystemInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_SatelliteControlSystem
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_SatelliteControlSystem
    ((SFc2_SatelliteControlSystemInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_SatelliteControlSystem();/* state var info */
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

extern void sf_internal_set_sim_state_c2_SatelliteControlSystem(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_SatelliteControlSystem();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_SatelliteControlSystem
    ((SFc2_SatelliteControlSystemInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_SatelliteControlSystem
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c2_SatelliteControlSystem(S);
}

static void sf_opaque_set_sim_state_c2_SatelliteControlSystem(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c2_SatelliteControlSystem(S, st);
}

static void sf_opaque_terminate_c2_SatelliteControlSystem(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_SatelliteControlSystemInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_SatelliteControlSystem
      ((SFc2_SatelliteControlSystemInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_SatelliteControlSystem_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_SatelliteControlSystem
    ((SFc2_SatelliteControlSystemInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_SatelliteControlSystem(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_SatelliteControlSystem
      ((SFc2_SatelliteControlSystemInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_SatelliteControlSystem(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SatelliteControlSystem_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4158739534U));
  ssSetChecksum1(S,(4063975001U));
  ssSetChecksum2(S,(2983946543U));
  ssSetChecksum3(S,(3876489117U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_SatelliteControlSystem(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_SatelliteControlSystem(SimStruct *S)
{
  SFc2_SatelliteControlSystemInstanceStruct *chartInstance;
  chartInstance = (SFc2_SatelliteControlSystemInstanceStruct *)malloc(sizeof
    (SFc2_SatelliteControlSystemInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_SatelliteControlSystemInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_SatelliteControlSystem;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_SatelliteControlSystem;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_SatelliteControlSystem;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_SatelliteControlSystem;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_SatelliteControlSystem;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_SatelliteControlSystem;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_SatelliteControlSystem;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_SatelliteControlSystem;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_SatelliteControlSystem;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_SatelliteControlSystem;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_SatelliteControlSystem;
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

void c2_SatelliteControlSystem_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_SatelliteControlSystem(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_SatelliteControlSystem(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_SatelliteControlSystem(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_SatelliteControlSystem_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
