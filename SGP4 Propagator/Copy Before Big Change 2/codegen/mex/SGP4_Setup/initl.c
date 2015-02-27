/*
 * initl.c
 *
 * Code generation for function 'initl'
 *
 * C source code generated on: Thu Jul 11 14:30:48 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "initl.h"
#include "sgp4.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo kb_emlrtRSI = { 70, "initl",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m" };

static emlrtRSInfo lb_emlrtRSI = { 75, "initl",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m" };

static emlrtRSInfo mb_emlrtRSI = { 83, "initl",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m" };

/* Function Definitions */
void initl(real_T *ainv, real_T *ao, real_T *con42, real_T *cosio, real_T
           *cosio2, real_T *einv, real_T *eccsq, real_T *omeosq, real_T *posq,
           real_T *rp, real_T *rteosq, real_T *sinio)
{
  real_T ak;
  real_T d1;
  real_T del;

  /*  ----------------------------------------------------------------------------- */
  /*  */
  /*                             procedure initl */
  /*  */
  /*    this procedure initializes the spg4 propagator. all the initialization is */
  /*      consolidated here instead of having multiple loops inside other routines. */
  /*  */
  /*  Author:  */
  /*    Jeff Beck  */
  /*    beckja@alumni.lehigh.edu */
  /*    1.0 (aug 7, 2006) - update for paper dav */
  /*    1.1 nov 16, 2007 - update for better compliance */
  /*  original comments from Vallado C++ version: */
  /*    author        : david vallado                  719-573-2600   28 jun 2005 */
  /*  */
  /*    inputs        : */
  /*      satrec.ecco        - eccentricity                           0.0 - 1.0 */
  /*      epoch       - epoch time in days from jan 0, 1950. 0 hr */
  /*      satrec.inclo       - inclination of satellite */
  /*      satrec.no          - mean motion of satellite */
  /*      satrec.satnum        - satellite number */
  /*  */
  /*    outputs       : */
  /*      ainv        - 1.0 / a */
  /*      ao          - semi major axis */
  /*      satrec.con41       - */
  /*      con42       - 1.0 - 5.0 cos(i) */
  /*      cosio       - cosine of inclination */
  /*      cosio2      - cosio squared */
  /*      einv        - 1.0 / e */
  /*      eccsq       - eccentricity squared */
  /*      satrec.method      - flag for deep space                    'd', 'n' */
  /*      omeosq      - 1.0 - satrec.ecco * satrec.ecco */
  /*      posq        - semi-parameter squared */
  /*      rp          - radius of perigee */
  /*      rteosq      - square root of (1.0 - satrec.ecco*satrec.ecco) */
  /*      sinio       - sine of inclination */
  /*      satrec.gsto        - gst at time of observation               rad */
  /*      satrec.no          - mean motion of satellite */
  /*  */
  /*    locals        : */
  /*      ak          - */
  /*      d1          - */
  /*      del         - */
  /*      adel        - */
  /*      po          - */
  /*  */
  /*    coupling      : */
  /*      gstime      - find greenwich sidereal time from the julian date */
  /*  */
  /*    references    : */
  /*      hoots, roehrich, norad spacetrack report #3 1980 */
  /*      hoots, norad spacetrack report #6 1986 */
  /*      hoots, schumacher and glover 2004 */
  /*      vallado, crawford, hujsak, kelso  2006 */
  /*   ----------------------------------------------------------------------------$/ */
  /*  /$ -------------------- wgs-72 earth constants ----------------- $/ */
  /*      // sgp4fix identify constants and allow alternate values */
  /*  /$ ------------- calculate auxillary epoch quantities ---------- $/ */
  *eccsq = satrec.ecco * satrec.ecco;
  *omeosq = 1.0 - *eccsq;
  emlrtPushRtStackR2012b(&kb_emlrtRSI, emlrtRootTLSGlobal);
  if (1.0 - *eccsq < 0.0) {
    emlrtPushRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
    eml_error();
    emlrtPopRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
  }

  *rteosq = muDoubleScalarSqrt(1.0 - *eccsq);
  emlrtPopRtStackR2012b(&kb_emlrtRSI, emlrtRootTLSGlobal);
  *cosio = muDoubleScalarCos(satrec.inclo);
  *cosio2 = *cosio * *cosio;

  /*  /$ ------------------ un-kozai the mean motion ----------------- $/ */
  emlrtPushRtStackR2012b(&lb_emlrtRSI, emlrtRootTLSGlobal);
  ak = gravc.xke / satrec.no;
  emlrtPushRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  if (ak < 0.0) {
    emlrtPushRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
    b_eml_error();
    emlrtPopRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
  }

  ak = muDoubleScalarPower(ak, 0.66666666666666663);
  emlrtPopRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&lb_emlrtRSI, emlrtRootTLSGlobal);
  d1 = 0.75 * gravc.j2 * (3.0 * *cosio2 - 1.0) / (*rteosq * (1.0 - *eccsq));
  del = d1 / (ak * ak);
  ak *= (1.0 - del * del) - del * (0.33333333333333331 + 134.0 * del * del /
    81.0);
  satrec.no /= 1.0 + d1 / (ak * ak);
  satrec_dirty |= 1U;
  emlrtPushRtStackR2012b(&mb_emlrtRSI, emlrtRootTLSGlobal);
  ak = gravc.xke / satrec.no;
  emlrtPushRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  if (ak < 0.0) {
    emlrtPushRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
    b_eml_error();
    emlrtPopRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
  }

  *ao = muDoubleScalarPower(ak, 0.66666666666666663);
  emlrtPopRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&mb_emlrtRSI, emlrtRootTLSGlobal);
  *sinio = muDoubleScalarSin(satrec.inclo);
  ak = *ao * (1.0 - *eccsq);
  *con42 = 1.0 - 5.0 * *cosio2;
  satrec.con41 = (-(1.0 - 5.0 * *cosio2) - *cosio2) - *cosio2;
  satrec_dirty |= 1U;
  *ainv = 1.0 / *ao;
  *einv = 1.0 / satrec.ecco;
  *posq = ak * ak;
  *rp = *ao * (1.0 - satrec.ecco);
  satrec.method = 'n';
  satrec_dirty |= 1U;

  /*  sgp4fix modern approach to finding sidereal time */
  /*  ----------------------------------------------------------------------------- */
  /*  */
  /*                            function gstime */
  /*  */
  /*   this function finds the greenwich sidereal time (iau-82). */
  /*  */
  /*   author        : david vallado                  719-573-2600    7 jun 2002 */
  /*  */
  /*   revisions */
  /*                 - */
  /*  */
  /*   inputs          description                    range / units */
  /*     jdut1       - julian date of ut1             days from 4713 bc */
  /*  */
  /*   outputs       : */
  /*     gst         - greenwich sidereal time        0 to 2pi rad */
  /*  */
  /*   locals        : */
  /*     temp        - temporary variable for reals   rad */
  /*     tut1        - julian centuries from the */
  /*                   jan 1, 2000 12 h epoch (ut1) */
  /*  */
  /*   coupling      : */
  /*  */
  /*   references    : */
  /*     vallado       2007, 193, Eq 3-43 */
  /*  */
  /*  gst = gstime(jdut1); */
  /*  ----------------------------------------------------------------------------- */
  /*  ------------------------  implementation   ------------------ */
  ak = (satrec.jdsatepoch - 2.451545E+6) / 36525.0;

  /*  360/86400 = 1/240, to deg, to rad */
  ak = muDoubleScalarRem((((-6.2E-6 * ak * ak * ak + 0.093104 * ak * ak) +
    3.1644001848128662E+9 * ak) + 67310.54841) * 0.017453292519943295 / 240.0,
    6.2831853071795862);

  /*  ------------------------ check quadrants -------------------- */
  if (ak < 0.0) {
    ak += 6.2831853071795862;
  }

  satrec.gsto = ak;
  satrec_dirty |= 1U;

  /*     global idebug dbgfile */
  /*     if isempty(idebug) */
  /*         idebug = 0; */
  /*     elseif idebug */
  /*         debug5; */
  /*     end */
}

/* End of code generation (initl.c) */
