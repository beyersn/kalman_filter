/*
 * sgp4.h
 *
 * Code generation for function 'sgp4'
 *
 * C source code generated on: Sat Oct 26 12:43:25 2013
 *
 */

#ifndef __SGP4_H__
#define __SGP4_H__
/* Include files */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "rtwtypes.h"
#include "sgp4_types.h"

/* Function Declarations */
extern void sgp4(real_T tsince, real_T r_data[3], int32_T r_size[2]);
extern void sgp4_initialize(void);
extern void sgp4_terminate(void);
#endif
/* End of code generation (sgp4.h) */
