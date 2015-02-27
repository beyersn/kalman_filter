/*
 * SGP4_Setup_terminate.c
 *
 * Code generation for function 'SGP4_Setup_terminate'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "SGP4_Setup_terminate.h"
#include <stdio.h>

/* Function Definitions */
void SGP4_Setup_atexit(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void SGP4_Setup_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (SGP4_Setup_terminate.c) */
