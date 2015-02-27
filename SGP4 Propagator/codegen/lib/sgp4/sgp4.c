/*
 * sgp4.c
 *
 * Code generation for function 'sgp4'
 *
 * C source code generated on: Sat Oct 26 12:43:25 2013
 *
 */

/* Include files */
#include "sgp4.h"

/* Type Definitions */
#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  real_T mu;
  real_T radiusearthkm;
  real_T xke;
  real_T tumin;
  real_T j2;
  real_T j3;
  real_T j4;
  real_T j3oj2;
} b_struct_T;

#endif                                 /*typedef_b_struct_T*/

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  real_T error;
  real_T satnum;
  real_T epochyr;
  real_T epochdays;
  real_T ndot;
  real_T nddot;
  real_T bstar;
  real_T inclo;
  real_T nodeo;
  real_T ecco;
  real_T argpo;
  real_T mo;
  real_T no;
  real_T a;
  real_T alta;
  real_T altp;
  real_T jdsatepoch;
  real_T isimp;
  char_T method;
  real_T aycof;
  real_T con41;
  real_T cc1;
  real_T cc4;
  real_T cc5;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T delmo;
  real_T eta;
  real_T argpdot;
  real_T omgcof;
  real_T sinmao;
  real_T t;
  real_T t2cof;
  real_T t3cof;
  real_T t4cof;
  real_T t5cof;
  real_T x1mth2;
  real_T x7thm1;
  real_T mdot;
  real_T nodedot;
  real_T xlcof;
  real_T xmcof;
  real_T nodecf;
  char_T init;
  real_T gsto;
} struct_T;

#endif                                 /*typedef_struct_T*/

/* Variable Definitions */
static b_struct_T gravc;
static uint32_T gravc_dirty;
static struct_T satrec;
static uint32_T satrec_dirty;

/* Function Declarations */
static real_T rt_remd(real_T u0, real_T u1);
static real_T rt_roundd(real_T u);

/* Function Definitions */
static real_T rt_remd(real_T u0, real_T u1)
{
  real_T y;
  real_T tr;
  if (u1 < 0.0) {
    y = ceil(u1);
  } else {
    y = floor(u1);
  }

  if ((u1 != 0.0) && (u1 != y)) {
    tr = u0 / u1;
    if (fabs(tr - rt_roundd(tr)) <= DBL_EPSILON * fabs(tr)) {
      y = 0.0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
  }

  return y;
}

static real_T rt_roundd(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void sgp4(real_T tsince, real_T r_data[3], int32_T r_size[2])
{
  real_T mrt;
  real_T xmdf;
  real_T argpdf;
  real_T t2;
  real_T nodem;
  real_T tempa;
  real_T tempe;
  real_T templ;
  real_T xlm;
  real_T t4;
  real_T temp;
  real_T t3;
  real_T inclm;
  real_T am;
  real_T cosim;
  real_T axnl;
  int32_T ktr;
  real_T a[9];
  static const int8_T iv0[3] = { 0, 0, 1 };

  real_T b_data[3];
  int32_T ib;
  int32_T ia;
  int32_T ic;

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
  /*  Clear sgp4 error flag */
  satrec.t = tsince;
  satrec.error = 0.0;
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
    xlm = 1.0 + satrec.eta * cos(xmdf);
    t4 = pow(xlm, 3.0);
    temp = satrec.omgcof * satrec.t + satrec.xmcof * (t4 - satrec.delmo);
    xmdf += temp;
    argpdf -= temp;
    t3 = t2 * satrec.t;
    t4 = t3 * satrec.t;
    tempa = (((1.0 - satrec.cc1 * satrec.t) - satrec.d2 * t2) - satrec.d3 * t3)
      - satrec.d4 * t4;
    tempe += satrec.bstar * satrec.cc5 * (sin(xmdf) - satrec.sinmao);
    templ = (templ + satrec.t3cof * t3) + t4 * (satrec.t4cof + satrec.t *
      satrec.t5cof);
  }

  t2 = satrec.no;
  t3 = satrec.ecco;
  inclm = satrec.inclo;
  if (satrec.no <= 0.0) {
    satrec.error = 2.0;
  }

  xlm = gravc.xke / t2;
  t4 = pow(xlm, 0.66666666666666663);
  am = t4 * tempa * tempa;
  t3 -= tempe;

  /*  fix tolerance for error recognition */
  if ((t3 >= 1.0) || (t3 < -0.001) || (am < 0.95)) {
    satrec.error = 1.0;
  }

  /*    sgp4fix change test condition for eccentricity */
  if (t3 < 1.0E-6) {
    t3 = 1.0E-6;
  }

  xmdf += satrec.no * templ;
  xlm = (xmdf + argpdf) + nodem;
  nodem = rt_remd(nodem, 6.2831853071795862);
  argpdf = rt_remd(argpdf, 6.2831853071795862);
  xlm = rt_remd(xlm, 6.2831853071795862);

  /*  compute extra mean quantities */
  cosim = cos(inclm);

  /*  add lunar-solar periodics */
  /*  long period periodics */
  axnl = t3 * cos(argpdf);
  temp = 1.0 / (am * (1.0 - t3 * t3));
  t2 = t3 * sin(argpdf) + temp * satrec.aycof;

  /*  solve kepler's equation */
  tempa = rt_remd((((rt_remd((xlm - argpdf) - nodem, 6.2831853071795862) +
                     argpdf) + nodem) + temp * satrec.xlcof * axnl) - nodem,
                  6.2831853071795862);
  xmdf = tempa;
  t3 = 9999.9;
  ktr = 1;

  /*  sgp4fix for kepler iteration */
  /*  the following iteration needs better limits on corrections */
  /*  To please the compiler, the following variable are defined here so that */
  /*  they are not undefined for some execution paths. */
  t4 = 0.0;
  xlm = 0.0;
  while ((fabs(t3) >= 1.0E-12) && (ktr <= 10)) {
    t4 = sin(xmdf);
    xlm = cos(xmdf);
    t3 = (((tempa - t2 * xlm) + axnl * t4) - xmdf) / ((1.0 - xlm * axnl) - t4 *
      t2);
    if (fabs(t3) >= 0.95) {
      if (t3 > 0.0) {
        t3 = 0.95;
      } else {
        t3 = -0.95;
      }
    }

    xmdf += t3;
    ktr++;
  }

  /*  Short period preliminary quantities */
  t3 = axnl * axnl + t2 * t2;
  tempa = am * (1.0 - t3);
  if (tempa < 0.0) {
    satrec.error = 4.0;
    r_size[0] = 3;
    r_size[1] = 1;
    for (ktr = 0; ktr < 3; ktr++) {
      r_data[ktr] = 0.0;
    }
  } else {
    argpdf = am * (1.0 - (axnl * xlm + t2 * t4));
    tempe = sqrt(1.0 - t3);
    temp = (axnl * t4 - t2 * xlm) / (1.0 + tempe);
    templ = am / argpdf * ((t4 - t2) - axnl * temp);
    t4 = am / argpdf * ((xlm - axnl) + t2 * temp);
    xmdf = (t4 + t4) * templ;
    t2 = 1.0 - 2.0 * templ * templ;
    temp = 1.0 / tempa;
    t3 = 0.5 * gravc.j2 * temp;
    xlm = t3 * temp;

    /*  update for short period periodics */
    mrt = argpdf * (1.0 - 1.5 * xlm * tempe * satrec.con41) + 0.5 * t3 *
      satrec.x1mth2 * t2;
    t4 = atan2(templ, t4);
    t3 = t4 - 0.25 * xlm * satrec.x7thm1 * xmdf;
    t4 = nodem + 1.5 * xlm * cosim * xmdf;
    tempa = inclm + 1.5 * xlm * cosim * sin(inclm) * t2;

    /*  orientation vectors */
    xmdf = sin(t3);
    t2 = cos(t3);
    xlm = sin(t4);
    t3 = cos(t4);
    t4 = cos(tempa);

    /*  position and velocity (in km and km/sec) */
    r_data[0] = mrt * (-xlm * t4 * xmdf + t3 * t2) * gravc.radiusearthkm;
    r_data[1] = mrt * (t3 * t4 * xmdf + xlm * t2) * gravc.radiusearthkm;
    r_data[2] = mrt * (sin(tempa) * xmdf) * gravc.radiusearthkm;

    /*      v(1,1) = (mvt * ux + rvdot * vx) * vkmpersec; */
    /*      v(2,1) = (mvt * uy + rvdot * vy) * vkmpersec; */
    /*      v(3,1) = (mvt * uz + rvdot * vz) * vkmpersec; */
    /*  Calculate the GMST1982 angle */
    t3 = ((satrec.jdsatepoch + tsince / 1440.0) - 2.451545E+6) / 36525.0;

    /*  1440 minutes in a day */
    t4 = pow(t3, 3.0);
    t3 = ((67310.54841 + 3.1644001848128662E+9 * t3) + 0.093104 * (t3 * t3)) -
      6.2E-6 * t4;
    t3 = (t3 - floor(t3 / 86400.0) * 86400.0) / 240.0 * 3.1415926535897931 /
      180.0;

    /*  1 sec = 1/240 degrees */
    a[0] = cos(t3);
    a[1] = -sin(t3);
    a[2] = 0.0;
    a[3] = sin(t3);
    a[4] = cos(t3);
    a[5] = 0.0;
    for (ktr = 0; ktr < 3; ktr++) {
      a[6 + ktr] = iv0[ktr];
    }

    for (ktr = 0; ktr < 3; ktr++) {
      b_data[ktr] = r_data[ktr];
    }

    r_size[0] = 3;
    r_size[1] = 1;
    for (ktr = 0; ktr < 3; ktr++) {
      r_data[ktr] = 0.0;
      r_data[ktr] = 0.0;
    }

    ktr = 0;
    for (ib = 0; ib + 1 < 4; ib++) {
      if (b_data[ib] != 0.0) {
        ia = ktr;
        for (ic = 0; ic < 3; ic++) {
          ia++;
          r_data[ic] += b_data[ib] * a[ia - 1];
        }
      }

      ktr += 3;
    }

    /*      LLA = convert_ecef2lla(r'*1000); */
    /*      w_e = [0; 0; 0.00007292115855300]; % rad/solar second */
    /*      v = ROT3'*v + cross(w_e,r); */
  }

  /*  // if pl > 0 */
  /*  // sgp4fix for decaying satellites */
  if (mrt < 1.0) {
    satrec.error = 6.0;
  }

  /*  global idebug dbgfile */
  /*  if idebug */
  /*      debug7; */
  /*  end */
  /*  function LLA = convert_ecef2lla(r) */
  /*  Re = 6371*1000; % m */
  /*  R = sqrt(sum(r.^2)); */
  /*  xy = sqrt(r(1)^2 + r(2)^2); */
  /*  lon = -acosd(r(1)/xy); */
  /*  lat = acosd(xy/R); */
  /*  alt = R - Re; */
  /*  LLA = [lat lon alt]; */
  /*  end */
}

void sgp4_initialize(void)
{
  satrec.error = 0.0;
  satrec.satnum = 25544.0;
  satrec.epochyr = 13.0;
  satrec.epochdays = 292.23040836999996;
  satrec.ndot = 4.1187952295762027E-10;
  satrec.nddot = 0.0;
  satrec.bstar = 0.00024790000000000006;
  satrec.inclo = 0.901490483923103;
  satrec.nodeo = 3.7299222004860577;
  satrec.ecco = 0.0002923;
  satrec.argpo = 0.75124556059442327;
  satrec.mo = 1.9598965395760086;
  satrec.no = 0.06761307489166285;
  satrec.a = 1.0654522464091658;
  satrec.alta = 0.06576367810079109;
  satrec.altp = 0.0651408147175403;
  satrec.jdsatepoch = 2.45658473040837E+6;
  satrec.isimp = 0.0;
  satrec.method = 'n';
  satrec.aycof = 0.000919563440851769;
  satrec.con41 = 0.15484386591348254;
  satrec.cc1 = 4.197971013306204E-9;
  satrec.cc4 = 6.9448725506776572E-7;
  satrec.cc5 = 0.00049653605005586885;
  satrec.d2 = 1.409173116793821E-15;
  satrec.d3 = 7.0757405568256644E-22;
  satrec.d4 = 4.1425018812670106E-28;
  satrec.delmo = 0.99336471599207787;
  satrec.eta = 0.005843247776587011;
  satrec.argpdot = 4.4630998073520835E-5;
  satrec.omgcof = 6.7369209500540515E-7;
  satrec.sinmao = 0.925250774076808;
  satrec.t = 10751.465663909912;
  satrec.t2cof = 6.296956519959306E-9;
  satrec.t3cof = 1.4444190380509392E-15;
  satrec.t4cof = 5.4861249714773562E-22;
  satrec.t5cof = 2.5821187094534938E-28;
  satrec.x1mth2 = 0.61505204469550578;
  satrec.x7thm1 = 1.6946356871314596;
  satrec.mdot = 0.067620571684069389;
  satrec.nodedot = -6.0033923452098432E-5;
  satrec.xlcof = 0.0017314315883144471;
  satrec.xmcof = -0.022540430366343008;
  satrec.nodecf = -8.8160247245169434E-13;
  satrec.init = 'n';
  satrec.gsto = 1.9339058429068672;
  gravc.mu = 398600.79964;
  gravc.radiusearthkm = 6378.135;
  gravc.xke = 0.0743669161;
  gravc.tumin = 13.446839702957643;
  gravc.j2 = 0.001082616;
  gravc.j3 = -2.53881E-6;
  gravc.j4 = -1.65597E-6;
  gravc.j3oj2 = -0.0023450697200115278;
  satrec_dirty = 0U;
  gravc_dirty = 0U;
}

void sgp4_terminate(void)
{
  /* (no terminate code required) */
}

/* End of code generation (sgp4.c) */
