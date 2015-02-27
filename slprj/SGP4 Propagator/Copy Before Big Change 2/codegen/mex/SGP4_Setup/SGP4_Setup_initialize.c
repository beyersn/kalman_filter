/*
 * SGP4_Setup_initialize.c
 *
 * Code generation for function 'SGP4_Setup_initialize'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "SGP4_Setup_initialize.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Function Declarations */
static void SGP4_Setup_once(void);

/* Function Definitions */
static void SGP4_Setup_once(void)
{
  gravc.mu = 398600.8;
  gravc.radiusearthkm = 6378.135;
  gravc.xke = 0.074366916133173422;
  gravc.tumin = 13.446839696959309;
  gravc.j2 = 0.001082616;
  gravc.j3 = -2.53881E-6;
  gravc.j4 = -1.65597E-6;
  gravc.j3oj2 = -0.0023450697200115278;
  satrec.error = 0.0;
  satrec.satnum = 25544.0;
  satrec.epochyr = 13.0;
  satrec.epochdays = 192.54519036;
  satrec.ndot = 7.6176349644335848E-11;
  satrec.nddot = 0.0;
  satrec.bstar = 5.2449000000000006E-5;
  satrec.inclo = 0.90145034135030722;
  satrec.nodeo = 6.0668150944805967;
  satrec.ecco = 0.0005392;
  satrec.argpo = 3.0050485648892691;
  satrec.mo = 4.0566513271179;
  satrec.no = 0.067613655762034527;
  satrec.a = 1.0654460840223445;
  satrec.alta = 0.066020572550849366;
  satrec.altp = 0.0648715954938397;
  satrec.jdsatepoch = 2.45648504519036E+6;
  satrec.isimp = 0.0;
  satrec.method = 'n';
  satrec.aycof = 0.00091953423680780325;
  satrec.con41 = 0.1549610631894891;
  satrec.cc1 = 8.8897523682672347E-10;
  satrec.cc4 = 1.0311300154088273E-6;
  satrec.cc5 = 0.00049702101495686056;
  satrec.d2 = 6.3199267140551318E-17;
  satrec.d3 = 6.7207414725048437E-24;
  satrec.d4 = 8.3330814962105672E-31;
  satrec.delmo = 0.98041003496706181;
  satrec.eta = 0.010780095480089449;
  satrec.argpdot = 4.4641379363496083E-5;
  satrec.omgcof = -1.0479885737597259E-7;
  satrec.sinmao = -0.79259836407506989;
  satrec.t = 268.13118159770966;
  satrec.t2cof = 1.3334628552400851E-9;
  satrec.t3cof = 6.4779821083933572E-17;
  satrec.t4cof = 5.2108601964444432E-24;
  satrec.t5cof = 5.1941838436013781E-31;
  satrec.x1mth2 = 0.61501297893683693;
  satrec.x7thm1 = 1.6949091474421416;
  satrec.mdot = 0.067621158377329418;
  satrec.nodedot = -6.0038222524212988E-5;
  satrec.xlcof = 0.0017313876246722571;
  satrec.xmcof = -0.0014019496363263187;
  satrec.nodecf = -1.867041011825626E-13;
  satrec.init = 'n';
  satrec.gsto = 2.19687536029177;
  gravc_dirty = 1U;
  satrec_dirty = 1U;
}

void SGP4_Setup_initialize(emlrtContext *aContext)
{
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, FALSE, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    SGP4_Setup_once();
  }
}

/* End of code generation (SGP4_Setup_initialize.c) */
