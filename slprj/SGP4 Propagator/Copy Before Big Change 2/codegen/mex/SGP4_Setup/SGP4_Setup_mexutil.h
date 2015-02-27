/*
 * SGP4_Setup_mexutil.h
 *
 * Code generation for function 'SGP4_Setup_mexutil'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

#ifndef __SGP4_SETUP_MEXUTIL_H__
#define __SGP4_SETUP_MEXUTIL_H__
/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "SGP4_Setup_types.h"

/* Function Declarations */
extern real_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *parentId);
#ifdef __WATCOMC__
#pragma aux b_emlrt_marshallIn value [8087];
#endif
extern const mxArray *b_emlrt_marshallOut(const struct_T *u);
extern const mxArray *c_emlrt_marshallOut(const b_struct_T u);
extern void e_emlrt_marshallIn(const mxArray *c_satrec, const char_T *identifier, struct_T *y);
extern void emlrt_checkEscapedGlobals(void);
extern void emlrt_synchGlobalsFromML(void);
extern void emlrt_synchGlobalsToML(void);
extern b_struct_T h_emlrt_marshallIn(const mxArray *c_gravc, const char_T *identifier);
#endif
/* End of code generation (SGP4_Setup_mexutil.h) */
