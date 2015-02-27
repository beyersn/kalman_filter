#ifndef __c2_UKF_1hz_h__
#define __c2_UKF_1hz_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_c2_ResolvedFunctionInfo
#define typedef_c2_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c2_ResolvedFunctionInfo;

#endif                                 /*typedef_c2_ResolvedFunctionInfo*/

#ifndef typedef_SFc2_UKF_1hzInstanceStruct
#define typedef_SFc2_UKF_1hzInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_UKF_1hz;
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
} SFc2_UKF_1hzInstanceStruct;

#endif                                 /*typedef_SFc2_UKF_1hzInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_UKF_1hz_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_UKF_1hz_get_check_sum(mxArray *plhs[]);
extern void c2_UKF_1hz_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
