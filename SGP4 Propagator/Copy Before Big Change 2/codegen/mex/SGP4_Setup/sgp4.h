/*
 * sgp4.h
 *
 * Code generation for function 'sgp4'
 *
 * C source code generated on: Thu Jul 11 14:30:48 2013
 *
 */

#ifndef __SGP4_H__
#define __SGP4_H__
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
extern void b_eml_error(void);
extern void eml_error(void);
extern void sgp4(real_T tsince, real_T r[3], real_T v[3]);
#endif
/* End of code generation (sgp4.h) */
