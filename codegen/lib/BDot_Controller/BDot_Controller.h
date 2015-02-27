/*
 * BDot_Controller.h
 *
 * Code generation for function 'BDot_Controller'
 *
 * C source code generated on: Mon Oct 14 18:58:10 2013
 *
 */

#ifndef __BDOT_CONTROLLER_H__
#define __BDOT_CONTROLLER_H__
/* Include files */
#include <stddef.h>
#include <stdlib.h>

#include "rtwtypes.h"
#include "BDot_Controller_types.h"

/* Function Declarations */
extern void BDot_Controller(const real_T B_s[3], real32_T Command_Trigger, real_T Ts, real_T Dipole_Command_s[3]);
extern void BDot_Controller_initialize(void);
extern void BDot_Controller_terminate(void);
#endif
/* End of code generation (BDot_Controller.h) */
