#ifndef __c2_UKF_10hz_h__
#define __c2_UKF_10hz_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_UKF_10hzInstanceStruct
#define typedef_SFc2_UKF_10hzInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_UKF_10hz;
  real_T c2_P_km1km1[81];
  boolean_T c2_P_km1km1_not_empty;
  real_T c2_x_km1km1[10];
  boolean_T c2_x_km1km1_not_empty;
  real_T c2_x_kk[10];
  boolean_T c2_x_kk_not_empty;
  real_T c2_R_k[36];
  boolean_T c2_R_k_not_empty;
  real_T c2_Q_k[81];
  boolean_T c2_Q_k_not_empty;
  real_T c2_Lambda;
  boolean_T c2_Lambda_not_empty;
  real_T c2_alpha;
  boolean_T c2_alpha_not_empty;
  real_T c2_K;
  boolean_T c2_K_not_empty;
  real_T c2_Beta;
  boolean_T c2_Beta_not_empty;
  real_T c2_q_s_c[4];
  boolean_T c2_q_s_c_not_empty;
  real_T c2_I_c[3];
  boolean_T c2_I_c_not_empty;
} SFc2_UKF_10hzInstanceStruct;

#endif                                 /*typedef_SFc2_UKF_10hzInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_UKF_10hz_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_UKF_10hz_get_check_sum(mxArray *plhs[]);
extern void c2_UKF_10hz_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
