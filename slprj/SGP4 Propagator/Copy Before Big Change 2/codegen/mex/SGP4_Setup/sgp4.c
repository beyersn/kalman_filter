/*
 * sgp4.c
 *
 * Code generation for function 'sgp4'
 *
 * C source code generated on: Thu Jul 11 14:30:48 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "sgp4.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo cb_emlrtRSI = { 20, "eml_error",
  "C:/Program Files/MATLAB/R2013a/toolbox/eml/lib/matlab/eml/eml_error.m" };

static emlrtRSInfo nb_emlrtRSI = { 156, "sgp4",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m" };

static emlrtRSInfo ob_emlrtRSI = { 157, "sgp4",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m" };

static emlrtRSInfo pb_emlrtRSI = { 236, "sgp4",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m" };

static emlrtRSInfo qb_emlrtRSI = { 237, "sgp4",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m" };

static emlrtRSInfo rb_emlrtRSI = { 238, "sgp4",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m" };

static emlrtRTEInfo emlrtRTEI = { 20, 5, "eml_error",
  "C:/Program Files/MATLAB/R2013a/toolbox/eml/lib/matlab/eml/eml_error.m" };

/* Function Definitions */
void b_eml_error(void)
{
  emlrtPushRtStackR2012b(&cb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtErrorWithMessageIdR2012b(emlrtRootTLSGlobal, &emlrtRTEI,
    "Coder:toolbox:power_domainError", 0);
  emlrtPopRtStackR2012b(&cb_emlrtRSI, emlrtRootTLSGlobal);
}

void eml_error(void)
{
  static char_T cv0[4][1] = { { 's' }, { 'q' }, { 'r' }, { 't' } };

  emlrtPushRtStackR2012b(&cb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtErrorWithMessageIdR2012b(emlrtRootTLSGlobal, &emlrtRTEI,
    "Coder:toolbox:ElFunDomainError", 3, 4, 4, cv0);
  emlrtPopRtStackR2012b(&cb_emlrtRSI, emlrtRootTLSGlobal);
}

void sgp4(real_T tsince, real_T r[3], real_T v[3])
{
  real_T vkmpersec;
  real_T mrt;
  real_T xmdf;
  real_T argpdf;
  real_T t2;
  real_T nodem;
  real_T tempa;
  real_T tempe;
  real_T templ;
  real_T temp;
  real_T t3;
  real_T t4;
  real_T nm;
  real_T inclm;
  real_T axnl;
  real_T am;
  real_T cosim;
  int32_T ktr;
  real_T esine;
  real_T rl;
  real_T sinu;
  real_T ROT3[9];
  real_T b_ROT3[3];
  static const int8_T iv17[3] = { 0, 0, 1 };

  int32_T i0;
  real_T c_ROT3[3];
  real_T dv0[3];

  /*  ----------------------------------------------------------------------------- */
  /*  */
  /*                               procedure sgp4 */
  /*  */
  /*   this procedure is the sgp4 prediction model from space command. this is an */
  /*     updated and combined version of sgp4 and sdp4, which were originally */
  /*     published separately in spacetrack report #3. this version follows the */
  /*     methodology from the aiaa paper (2006) describing the history and */
  /*     development of the code. */
  /*  */
  /*  Author: */
  /*    Jeff Beck */
  /*    beckja@alumni.lehigh.edu */
  /*     current : */
  /*                7 may 08  david vallado */
  /*                            update small eccentricity check */
  /*     changes : */
  /*               16 nov 07  david vallado */
  /*                            misc fixes for better compliance */
  /*    1.0 (aug 7, 2006) - update for paper dav */
  /*  original comments from Vallado C++ version: */
  /*    author        : david vallado                  719-573-2600   28 jun 2005 */
  /*  */
  /*    inputs        : */
  /*      satrec    - initialised structure from sgp4init() call. */
  /*      tsince    - time eince epoch (minutes) */
  /*  */
  /*    outputs       : */
  /*      r           - position vector                     km */
  /*      v           - velocity                            km/sec */
  /*      return code - non-zero on error. */
  /*                     1 - mean elements, ecc >= 1.0 or ecc < -0.001 or a < 0.95 er */
  /*                     2 - mean motion less than 0.0 */
  /*                     3 - pert elements, ecc < 0.0  or  ecc > 1.0 */
  /*                     4 - semi-latus rectum < 0.0 */
  /*                     5 - epoch elements are sub-orbital */
  /*                     6 - satellite has decayed */
  /*  */
  /*    locals        : */
  /*      am          - */
  /*      axnl, aynl        - */
  /*      betal       - */
  /*      COSIM   , SINIM   , COSOMM  , SINOMM  , Cnod    , Snod    , Cos2u   , */
  /*      Sin2u   , Coseo1  , Sineo1  , Cosi    , Sini    , Cosip   , Sinip   , */
  /*      Cosisq  , Cossu   , Sinsu   , Cosu    , Sinu */
  /*      Delm        - */
  /*      Delomg      - */
  /*      Dndt        - */
  /*      Eccm        - */
  /*      EMSQ        - */
  /*      Ecose       - */
  /*      El2         - */
  /*      Eo1         - */
  /*      Eccp        - */
  /*      Esine       - */
  /*      Argpm       - */
  /*      Argpp       - */
  /*      Omgadf      - */
  /*      Pl          - */
  /*      R           - */
  /*      RTEMSQ      - */
  /*      Rdotl       - */
  /*      Rl          - */
  /*      Rvdot       - */
  /*      Rvdotl      - */
  /*      Su          - */
  /*      T2  , T3   , T4    , Tc */
  /*      Tem5, Temp , Temp1 , Temp2  , Tempa  , Tempe  , Templ */
  /*      U   , Ux   , Uy    , Uz     , Vx     , Vy     , Vz */
  /*      inclm       - inclination */
  /*      mm          - mean anomaly */
  /*      nm          - mean motion */
  /*      nodem      - longi of ascending node */
  /*      xinc        - */
  /*      xincp       - */
  /*      xl          - */
  /*      xlm         - */
  /*      mp          - */
  /*      xmdf        - */
  /*      xmx         - */
  /*      xmy         - */
  /*      nodedf     - */
  /*      xnode       - */
  /*      nodep      - */
  /*      np          - */
  /*  */
  /*    coupling      : */
  /*      getgravconst */
  /*      dpper */
  /*      dspace */
  /*  */
  /*    references    : */
  /*      hoots, roehrich, norad spacetrack report #3 1980 */
  /*      hoots, norad spacetrack report #6 1986 */
  /*      hoots, schumacher and glover 2004 */
  /*      vallado, crawford, hujsak, kelso  2006 */
  /*   ----------------------------------------------------------------------------$/ */
  /* % Define Constants */
  /*  sgp4fix divisor for divide by zero check on inclination */
  /*  the old check used 1.0 + cos(pi-1.0e-9), but then compared it to */
  /*  1.5 e-12, so the threshold was changed to 1.5e-12 for consistancy */
  /*   sgp4fix identify constants and allow alternate values */
  vkmpersec = gravc.radiusearthkm * gravc.xke / 60.0;

  /*  Clear sgp4 error flag */
  satrec.t = tsince;
  satrec_dirty |= 1U;
  satrec.error = 0.0;
  satrec_dirty |= 1U;
  mrt = 0.0;

  /*  Update for secular gravity and atmospheric drag */
  xmdf = satrec.mo + satrec.mdot * satrec.t;
  argpdf = satrec.argpo + satrec.argpdot * satrec.t;
  t2 = satrec.t * satrec.t;
  nodem = (satrec.nodeo + satrec.nodedot * satrec.t) + satrec.nodecf * t2;
  tempa = 1.0 - satrec.cc1 * satrec.t;
  tempe = satrec.bstar * satrec.cc4 * satrec.t;
  templ = satrec.t2cof * t2;
  if (satrec.isimp != 1.0) {
    /*  Not deep space model */
    temp = satrec.omgcof * satrec.t + satrec.xmcof * (muDoubleScalarPower(1.0 +
      satrec.eta * muDoubleScalarCos(xmdf), 3.0) - satrec.delmo);
    xmdf += temp;
    argpdf -= temp;
    t3 = t2 * satrec.t;
    t4 = t3 * satrec.t;
    tempa = (((1.0 - satrec.cc1 * satrec.t) - satrec.d2 * t2) - satrec.d3 * t3)
      - satrec.d4 * t4;
    tempe += satrec.bstar * satrec.cc5 * (muDoubleScalarSin(xmdf) -
      satrec.sinmao);
    templ = (templ + satrec.t3cof * t3) + t4 * (satrec.t4cof + satrec.t *
      satrec.t5cof);
  }

  nm = satrec.no;
  t4 = satrec.ecco;
  inclm = satrec.inclo;
  if (satrec.no <= 0.0) {
    satrec.error = 2.0;
    satrec_dirty |= 1U;
  }

  emlrtPushRtStackR2012b(&nb_emlrtRSI, emlrtRootTLSGlobal);
  axnl = gravc.xke / nm;
  emlrtPushRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  if (axnl < 0.0) {
    emlrtPushRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
    b_eml_error();
    emlrtPopRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
  }

  emlrtPopRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  am = muDoubleScalarPower(axnl, 0.66666666666666663) * tempa * tempa;
  emlrtPopRtStackR2012b(&nb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&ob_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  if (am < 0.0) {
    emlrtPushRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
    b_eml_error();
    emlrtPopRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
  }

  emlrtPopRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  nm = gravc.xke / muDoubleScalarPower(am, 1.5);
  emlrtPopRtStackR2012b(&ob_emlrtRSI, emlrtRootTLSGlobal);
  t4 -= tempe;

  /*  fix tolerance for error recognition */
  if ((t4 >= 1.0) || (t4 < -0.001) || (am < 0.95)) {
    satrec.error = 1.0;
    satrec_dirty |= 1U;
  }

  /*    sgp4fix change test condition for eccentricity */
  if (t4 < 1.0E-6) {
    t4 = 1.0E-6;
  }

  xmdf += satrec.no * templ;
  t3 = (xmdf + argpdf) + nodem;
  nodem = muDoubleScalarRem(nodem, 6.2831853071795862);
  argpdf = muDoubleScalarRem(argpdf, 6.2831853071795862);
  t3 = muDoubleScalarRem(t3, 6.2831853071795862);

  /*  compute extra mean quantities */
  cosim = muDoubleScalarCos(inclm);

  /*  add lunar-solar periodics */
  /*  long period periodics */
  axnl = t4 * muDoubleScalarCos(argpdf);
  temp = 1.0 / (am * (1.0 - t4 * t4));
  templ = t4 * muDoubleScalarSin(argpdf) + temp * satrec.aycof;

  /*  solve kepler's equation */
  t3 = muDoubleScalarRem((((muDoubleScalarRem((t3 - argpdf) - nodem,
    6.2831853071795862) + argpdf) + nodem) + temp * satrec.xlcof * axnl) - nodem,
    6.2831853071795862);
  t4 = t3;
  t2 = 9999.9;
  ktr = 1;

  /*  sgp4fix for kepler iteration */
  /*  the following iteration needs better limits on corrections */
  /*  To please the compiler, the following variable are defined here so that */
  /*  they are not undefined for some execution paths. */
  tempa = 0.0;
  xmdf = 0.0;
  while ((muDoubleScalarAbs(t2) >= 1.0E-12) && (ktr <= 10)) {
    tempa = muDoubleScalarSin(t4);
    xmdf = muDoubleScalarCos(t4);
    t2 = (((t3 - templ * xmdf) + axnl * tempa) - t4) / ((1.0 - xmdf * axnl) -
      tempa * templ);
    if (muDoubleScalarAbs(t2) >= 0.95) {
      if (t2 > 0.0) {
        t2 = 0.95;
      } else {
        t2 = -0.95;
      }
    }

    t4 += t2;
    ktr++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  /*  Short period preliminary quantities */
  esine = axnl * tempa - templ * xmdf;
  t3 = axnl * axnl + templ * templ;
  argpdf = am * (1.0 - t3);
  if (argpdf < 0.0) {
    satrec.error = 4.0;
    satrec_dirty |= 1U;
    for (ktr = 0; ktr < 3; ktr++) {
      r[ktr] = 0.0;
      v[ktr] = 0.0;
    }
  } else {
    rl = am * (1.0 - (axnl * xmdf + templ * tempa));
    emlrtPushRtStackR2012b(&pb_emlrtRSI, emlrtRootTLSGlobal);
    emlrtPopRtStackR2012b(&pb_emlrtRSI, emlrtRootTLSGlobal);
    emlrtPushRtStackR2012b(&qb_emlrtRSI, emlrtRootTLSGlobal);
    emlrtPopRtStackR2012b(&qb_emlrtRSI, emlrtRootTLSGlobal);
    emlrtPushRtStackR2012b(&rb_emlrtRSI, emlrtRootTLSGlobal);
    if (1.0 - t3 < 0.0) {
      emlrtPushRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
      eml_error();
      emlrtPopRtStackR2012b(&bb_emlrtRSI, emlrtRootTLSGlobal);
    }

    tempe = muDoubleScalarSqrt(1.0 - t3);
    emlrtPopRtStackR2012b(&rb_emlrtRSI, emlrtRootTLSGlobal);
    temp = esine / (1.0 + tempe);
    sinu = am / rl * ((tempa - templ) - axnl * temp);
    t3 = am / rl * ((xmdf - axnl) + templ * temp);
    tempa = (t3 + t3) * sinu;
    axnl = 2.0 * sinu;
    t2 = 1.0 - axnl * sinu;
    temp = 1.0 / argpdf;
    templ = 0.5 * gravc.j2 * temp;
    t4 = templ * temp;

    /*  update for short period periodics */
    mrt = rl * (1.0 - 1.5 * t4 * tempe * satrec.con41) + 0.5 * templ *
      satrec.x1mth2 * t2;
    xmdf = muDoubleScalarAtan2(sinu, t3) - 0.25 * t4 * satrec.x7thm1 * tempa;
    tempe = nodem + 1.5 * t4 * cosim * tempa;
    t3 = inclm + 1.5 * t4 * cosim * muDoubleScalarSin(inclm) * t2;
    temp = muDoubleScalarSqrt(am) * esine / rl - nm * templ * satrec.x1mth2 *
      tempa / gravc.xke;
    sinu = muDoubleScalarSqrt(argpdf) / rl + nm * templ * (satrec.x1mth2 * (1.0
      - axnl * sinu) + 1.5 * satrec.con41) / gravc.xke;

    /*  orientation vectors */
    esine = muDoubleScalarSin(xmdf);
    argpdf = muDoubleScalarCos(xmdf);
    axnl = muDoubleScalarSin(tempe);
    tempe = muDoubleScalarCos(tempe);
    templ = muDoubleScalarSin(t3);
    t3 = muDoubleScalarCos(t3);
    xmdf = -axnl * t3;
    t3 *= tempe;
    t4 = xmdf * esine + tempe * argpdf;
    t2 = t3 * esine + axnl * argpdf;
    tempa = templ * esine;

    /*  position and velocity (in km and km/sec) */
    r[0] = mrt * t4 * gravc.radiusearthkm;
    r[1] = mrt * t2 * gravc.radiusearthkm;
    r[2] = mrt * tempa * gravc.radiusearthkm;
    v[0] = (temp * t4 + sinu * (xmdf * argpdf - tempe * esine)) * vkmpersec;
    v[1] = (temp * t2 + sinu * (t3 * argpdf - axnl * esine)) * vkmpersec;
    v[2] = (temp * tempa + sinu * (templ * argpdf)) * vkmpersec;

    /*  Calculate the GMST1982 angle */
    t3 = ((satrec.jdsatepoch + tsince / 1440.0) - 2.451545E+6) / 36525.0;

    /*  1440 minutes in a day */
    t3 = ((67310.54841 + 3.1644001848128662E+9 * t3) + 0.093104 * (t3 * t3)) -
      6.2E-6 * muDoubleScalarPower(t3, 3.0);
    t3 = (t3 - muDoubleScalarFloor(t3 / 86400.0) * 86400.0) / 240.0 *
      3.1415926535897931 / 180.0;

    /*  1 sec = 1/240 degrees */
    ROT3[0] = muDoubleScalarCos(t3);
    ROT3[3] = -muDoubleScalarSin(t3);
    ROT3[6] = 0.0;
    ROT3[1] = muDoubleScalarSin(t3);
    ROT3[4] = muDoubleScalarCos(t3);
    ROT3[7] = 0.0;
    for (ktr = 0; ktr < 3; ktr++) {
      ROT3[2 + 3 * ktr] = iv17[ktr];
      b_ROT3[ktr] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        b_ROT3[ktr] += ROT3[i0 + 3 * ktr] * r[i0];
      }
    }

    /*  rad/solar second */
    for (ktr = 0; ktr < 3; ktr++) {
      r[ktr] = b_ROT3[ktr];
      c_ROT3[ktr] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        c_ROT3[ktr] += ROT3[i0 + 3 * ktr] * v[i0];
      }
    }

    dv0[0] = 0.0 * r[2] - 7.2921158553E-5 * r[1];
    dv0[1] = 7.2921158553E-5 * r[0] - 0.0 * r[2];
    dv0[2] = 0.0 * r[1] - 0.0 * r[0];
    for (ktr = 0; ktr < 3; ktr++) {
      v[ktr] = c_ROT3[ktr] + dv0[ktr];
    }
  }

  /*  // if pl > 0 */
  /*  // sgp4fix for decaying satellites */
  if (mrt < 1.0) {
    satrec.error = 6.0;
    satrec_dirty |= 1U;
  }

  /*  global idebug dbgfile */
  /*  if idebug */
  /*      debug7; */
  /*  end */
}

/* End of code generation (sgp4.c) */
