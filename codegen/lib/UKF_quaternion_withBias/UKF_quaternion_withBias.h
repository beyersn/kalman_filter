/*
 * UKF_quaternion_withBias.h
 *
 * Code generation for function 'UKF_quaternion_withBias'
 *
 * C source code generated on: Thu Aug 15 19:39:28 2013
 *
 */

#ifndef __UKF_QUATERNION_WITHBIAS_H__
#define __UKF_QUATERNION_WITHBIAS_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "UKF_quaternion_withBias_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void UKF_quaternion_withBias(real_T B_ref[3], real_T B_sat[3], const real_T w_gyro[3], const real_T Torque_s[3], real_T Ts, real_T Attitude_sensor[4], real_T w_sensor[3], real_T w_bias_sensor[3]);
extern void UKF_quaternion_withBias_initialize(void);
extern void UKF_quaternion_withBias_terminate(void);
#endif
/* End of code generation (UKF_quaternion_withBias.h) */
