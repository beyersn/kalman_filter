/*
 * getgravc.c
 *
 * Code generation for function 'getgravc'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "getgravc.h"
#include "sgp4.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo db_emlrtRSI = { 49, "getgravc",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m" };

/* Function Definitions */
void getgravc(void)
{
  real_T y;

  /*  ----------------------------------------------------------------------------- */
  /*  */
  /*                            function getgravc */
  /*  */
  /*   this function gets constants for the propagator. note that gravc.mu is identified to */
  /*     facilitiate comparisons with newer models. */
  /*  */
  /*   author        : david vallado                  719-573-2600   21 jul 2006 */
  /*  */
  /*   inputs        : */
  /*     whichconst  - which set of constants to use  721, 72, 84 */
  /*  */
  /*   outputs       : */
  /*     tumin       - minutes in one time unit */
  /*     mu          - earth gravitational parameter */
  /*     radiusearthkm - radius of the earth in km */
  /*     xke         - reciprocal of gravc.tumin */
  /*     j2, j3, j4  - un-normalized zonal harmonic values */
  /*     j3oj2       - j3 divided by j2 */
  /*  */
  /*   locals        : */
  /*  */
  /*   coupling      : */
  /*  */
  /*   references    : */
  /*     norad spacetrack report #3 */
  /*     vallado, crawford, hujsak, kelso  2006 */
  /*  [gravc.tumin, gravc.mu, gravc.radiusearthkm, gravc.xke, j2, j3, j4, j3oj2] = getgravc(whichconst); */
  /*   --------------------------------------------------------------------------- $/ */
  /*  ------------ wgs-72 constants ------------ */
  gravc.mu = 398600.8;
  gravc_dirty |= 1U;

  /* // in km3 / s2 */
  gravc.radiusearthkm = 6378.135;
  gravc_dirty |= 1U;

  /* // km */
  emlrtPushRtStackR2012b(&db_emlrtRSI, emlrtRootTLSGlobal);
  y = gravc.radiusearthkm * gravc.radiusearthkm * gravc.radiusearthkm / gravc.mu;
  if (y < 0.0) {
    emlrtPushRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
    eml_error();
    emlrtPopRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
  }

  gravc.xke = 60.0 / muDoubleScalarSqrt(y);
  gravc_dirty |= 1U;
  emlrtPopRtStackR2012b(&db_emlrtRSI, emlrtRootTLSGlobal);
  gravc.tumin = 1.0 / gravc.xke;
  gravc_dirty |= 1U;
  gravc.j2 = 0.001082616;
  gravc_dirty |= 1U;
  gravc.j3 = -2.53881E-6;
  gravc_dirty |= 1U;
  gravc.j4 = -1.65597E-6;
  gravc_dirty |= 1U;
  gravc.j3oj2 = gravc.j3 / gravc.j2;
  gravc_dirty |= 1U;

  /*  case */
}

/* End of code generation (getgravc.c) */
