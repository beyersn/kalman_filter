/*
 * SGP4_Setup.c
 *
 * Code generation for function 'SGP4_Setup'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "sgp4init.h"
#include "days2mdh.h"
#include "sgp4.h"
#include "SGP4_Setup_mexutil.h"
#include "getgravc.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 124, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo b_emlrtRSI = { 125, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo c_emlrtRSI = { 97, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo d_emlrtRSI = { 98, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo e_emlrtRSI = { 101, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo f_emlrtRSI = { 102, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo g_emlrtRSI = { 103, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo h_emlrtRSI = { 104, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo i_emlrtRSI = { 105, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo j_emlrtRSI = { 106, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo k_emlrtRSI = { 107, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo l_emlrtRSI = { 108, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo m_emlrtRSI = { 109, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo n_emlrtRSI = { 112, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo o_emlrtRSI = { 113, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo p_emlrtRSI = { 114, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo q_emlrtRSI = { 115, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo r_emlrtRSI = { 116, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo s_emlrtRSI = { 117, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo t_emlrtRSI = { 118, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo u_emlrtRSI = { 119, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo v_emlrtRSI = { 120, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo w_emlrtRSI = { 34, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo x_emlrtRSI = { 128, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo y_emlrtRSI = { 153, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtRSInfo ab_emlrtRSI = { 158, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo emlrtMCI = { 98, 21, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo b_emlrtMCI = { 101, 22, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo c_emlrtMCI = { 102, 24, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo d_emlrtMCI = { 103, 19, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo e_emlrtMCI = { 104, 20, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo f_emlrtMCI = { 105, 12, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo g_emlrtMCI = { 106, 20, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo h_emlrtMCI = { 107, 13, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo i_emlrtMCI = { 113, 21, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo j_emlrtMCI = { 114, 20, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo k_emlrtMCI = { 115, 20, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo l_emlrtMCI = { 116, 19, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo m_emlrtMCI = { 117, 20, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo n_emlrtMCI = { 118, 17, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo o_emlrtMCI = { 119, 17, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo p_emlrtMCI = { 124, 34, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo q_emlrtMCI = { 124, 19, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo r_emlrtMCI = { 125, 34, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo s_emlrtMCI = { 125, 19, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo t_emlrtMCI = { 97, 15, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo u_emlrtMCI = { 108, 12, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo v_emlrtMCI = { 112, 16, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo w_emlrtMCI = { 109, 13, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

static emlrtMCInfo x_emlrtMCI = { 120, 14, "SGP4_Setup",
  "//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m" };

/* Function Declarations */
static void b_str2num(const mxArray *b, emlrtMCInfo *location);
static real_T emlrt_marshallIn(const mxArray *c_str2num, const char_T
  *identifier);
static const mxArray *mpower(const mxArray *b, const mxArray *c, emlrtMCInfo
  *location);
static const mxArray *mtimes(const mxArray *b, const mxArray *c, emlrtMCInfo
  *location);
static const mxArray *str2num(const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static void b_str2num(const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "str2num", TRUE,
                        location);
}

static real_T emlrt_marshallIn(const mxArray *c_str2num, const char_T
  *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(emlrtAlias(c_str2num), &thisId);
  emlrtDestroyArray(&c_str2num);
  return y;
}

static const mxArray *mpower(const mxArray *b, const mxArray *c, emlrtMCInfo
  *location)
{
  const mxArray *pArrays[2];
  const mxArray *m5;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 1, &m5, 2, pArrays, "mpower",
    TRUE, location);
}

static const mxArray *mtimes(const mxArray *b, const mxArray *c, emlrtMCInfo
  *location)
{
  const mxArray *pArrays[2];
  const mxArray *m6;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 1, &m6, 2, pArrays, "mtimes",
    TRUE, location);
}

static const mxArray *str2num(const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m4;
  pArray = b;
  return emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 1, &m4, 1, &pArray, "str2num",
    TRUE, location);
}

void SGP4_Setup(char_T longstr1[69], char_T longstr2[69])
{
  int32_T j;
  const mxArray *y;
  const mxArray *m0;
  const mxArray *b_y;
  static const int32_T iv0[2] = { 1, 5 };

  char_T b_longstr1[5];
  const mxArray *c_y;
  static const int32_T iv1[2] = { 1, 2 };

  char_T c_longstr1[2];
  const mxArray *d_y;
  static const int32_T iv2[2] = { 1, 12 };

  char_T d_longstr1[12];
  const mxArray *e_y;
  static const int32_T iv3[2] = { 1, 10 };

  char_T e_longstr1[10];
  const mxArray *f_y;
  static const int32_T iv4[2] = { 1, 7 };

  char_T f_longstr1[7];
  const mxArray *g_y;
  static const int32_T iv5[2] = { 1, 2 };

  const mxArray *nexp = NULL;
  const mxArray *h_y;
  static const int32_T iv6[2] = { 1, 7 };

  const mxArray *i_y;
  static const int32_T iv7[2] = { 1, 2 };

  const mxArray *ibexp = NULL;
  const mxArray *j_y;
  const mxArray *k_y;
  static const int32_T iv8[2] = { 1, 4 };

  char_T g_longstr1[4];
  const mxArray *l_y;
  const mxArray *m_y;
  static const int32_T iv9[2] = { 1, 5 };

  const mxArray *n_y;
  static const int32_T iv10[2] = { 1, 9 };

  char_T b_longstr2[9];
  const mxArray *o_y;
  static const int32_T iv11[2] = { 1, 9 };

  const mxArray *p_y;
  static const int32_T iv12[2] = { 1, 8 };

  char_T c_longstr2[8];
  const mxArray *q_y;
  static const int32_T iv13[2] = { 1, 9 };

  const mxArray *r_y;
  static const int32_T iv14[2] = { 1, 9 };

  const mxArray *s_y;
  static const int32_T iv15[2] = { 1, 12 };

  const mxArray *t_y;
  static const int32_T iv16[2] = { 1, 5 };

  const mxArray *u_y;
  const mxArray *v_y;
  const mxArray *w_y;
  const mxArray *x_y;
  real_T year;
  real_T sec;
  real_T minute;
  real_T hr;
  real_T day;
  real_T mon;

  /*  SGP4_Setup */
  /*  Brandon Jackson */
  /*  bajackso@mtu.edu */
  /*  9th July 2013 */
  /*  */
  /*  */
  /*  Inputs: */
  /*  longstr1: TLE character String Line 1 */
  /*  longstr2: TLE character Sring Line 2 */
  /*  */
  /*  Outputs: */
  /*  satrec: structure containing all of the SGP4 satellite information  */
  /*  */
  /*  Coupling: */
  /*  getgravconst */
  /*  days2mdhms */
  /*  jday */
  /*  sgp4init */
  /*  */
  /*  References: */
  /*  Norad Spacetrack Report #3 */
  /*  Vallado, Crawford, Hujsak, Kelso 2006 */
  /* % Define Global Variables */
  /* % Include extrinsic functions */
  /* % WGS-72 Earth Constants */
  /*  sgp4fix identify constants and allow alternate values */
  /*  Options 721 72 84 */
  emlrtPushRtStackR2012b(&w_emlrtRSI, emlrtRootTLSGlobal);
  getgravc();
  emlrtPopRtStackR2012b(&w_emlrtRSI, emlrtRootTLSGlobal);

  /* % Define Constants */
  /*   0.01745329251994330;  % [deg/rad] */
  /*  229.1831180523293;  % [rev/day]/[rad/min]   */
  satrec.error = 0.0;
  satrec_dirty |= 1U;

  /* % Parse TLE */
  /*  Set the implied decimal points since doing a formated read fixes for bad */
  /*  input data values (missing, ...) */
  for (j = 0; j < 6; j++) {
    if (longstr1[10 + j] == ' ') {
      longstr1[10 + j] = '_';
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  if (longstr1[44] != ' ') {
    longstr1[43] = longstr1[44];
  }

  longstr1[44] = '.';
  if (longstr1[7] == ' ') {
    longstr1[7] = 'U';
  }

  if (longstr1[9] == ' ') {
    longstr1[9] = '.';
  }

  for (j = 0; j < 5; j++) {
    if (longstr1[45 + j] == ' ') {
      longstr1[45 + j] = '0';
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  if (longstr1[51] == ' ') {
    longstr1[51] = '0';
  }

  if (longstr1[53] != ' ') {
    longstr1[52] = longstr1[53];
  }

  longstr1[53] = '.';
  longstr2[25] = '.';
  for (j = 0; j < 7; j++) {
    if (longstr2[26 + j] == ' ') {
      longstr2[26 + j] = '0';
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  if (longstr1[62] == ' ') {
    longstr1[62] = '0';
  }

  if (longstr1[67] == ' ') {
    longstr1[67] = '0';
  }

  /*  parse first line */
  emlrtPushRtStackR2012b(&c_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  y = NULL;
  m0 = emlrtCreateString1(longstr1[0]);
  emlrtAssign(&y, m0);
  b_str2num(y, &t_emlrtMCI);
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&c_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&d_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  b_y = NULL;
  m0 = mxCreateCharArray(2, iv0);
  for (j = 0; j < 5; j++) {
    b_longstr1[j] = longstr1[2 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 5, m0, b_longstr1);
  emlrtAssign(&b_y, m0);
  satrec.satnum = emlrt_marshallIn(str2num(b_y, &emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&d_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&e_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  c_y = NULL;
  m0 = mxCreateCharArray(2, iv1);
  for (j = 0; j < 2; j++) {
    c_longstr1[j] = longstr1[18 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 2, m0, c_longstr1);
  emlrtAssign(&c_y, m0);
  satrec.epochyr = emlrt_marshallIn(str2num(c_y, &b_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&e_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&f_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  d_y = NULL;
  m0 = mxCreateCharArray(2, iv2);
  for (j = 0; j < 12; j++) {
    d_longstr1[j] = longstr1[20 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 12, m0, d_longstr1);
  emlrtAssign(&d_y, m0);
  satrec.epochdays = emlrt_marshallIn(str2num(d_y, &c_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&f_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&g_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  e_y = NULL;
  m0 = mxCreateCharArray(2, iv3);
  for (j = 0; j < 10; j++) {
    e_longstr1[j] = longstr1[33 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 10, m0, e_longstr1);
  emlrtAssign(&e_y, m0);
  satrec.ndot = emlrt_marshallIn(str2num(e_y, &d_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&g_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&h_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  f_y = NULL;
  m0 = mxCreateCharArray(2, iv4);
  for (j = 0; j < 7; j++) {
    f_longstr1[j] = longstr1[43 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 7, m0, f_longstr1);
  emlrtAssign(&f_y, m0);
  satrec.nddot = emlrt_marshallIn(str2num(f_y, &e_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&h_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&i_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  g_y = NULL;
  m0 = mxCreateCharArray(2, iv5);
  for (j = 0; j < 2; j++) {
    c_longstr1[j] = longstr1[50 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 2, m0, c_longstr1);
  emlrtAssign(&g_y, m0);
  emlrtAssign(&nexp, str2num(g_y, &f_emlrtMCI));
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&i_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&j_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  h_y = NULL;
  m0 = mxCreateCharArray(2, iv6);
  for (j = 0; j < 7; j++) {
    f_longstr1[j] = longstr1[52 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 7, m0, f_longstr1);
  emlrtAssign(&h_y, m0);
  satrec.bstar = emlrt_marshallIn(str2num(h_y, &g_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&j_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&k_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  i_y = NULL;
  m0 = mxCreateCharArray(2, iv7);
  for (j = 0; j < 2; j++) {
    c_longstr1[j] = longstr1[59 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 2, m0, c_longstr1);
  emlrtAssign(&i_y, m0);
  emlrtAssign(&ibexp, str2num(i_y, &h_emlrtMCI));
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&k_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&l_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  j_y = NULL;
  m0 = emlrtCreateString1(longstr1[62]);
  emlrtAssign(&j_y, m0);
  b_str2num(j_y, &u_emlrtMCI);
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&l_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&m_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  k_y = NULL;
  m0 = mxCreateCharArray(2, iv8);
  for (j = 0; j < 4; j++) {
    g_longstr1[j] = longstr1[64 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 4, m0, g_longstr1);
  emlrtAssign(&k_y, m0);
  emlrt_marshallIn(str2num(k_y, &w_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&m_emlrtRSI, emlrtRootTLSGlobal);

  /*  parse second line */
  emlrtPushRtStackR2012b(&n_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  l_y = NULL;
  m0 = emlrtCreateString1(longstr2[0]);
  emlrtAssign(&l_y, m0);
  b_str2num(l_y, &v_emlrtMCI);
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&n_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&o_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  m_y = NULL;
  m0 = mxCreateCharArray(2, iv9);
  for (j = 0; j < 5; j++) {
    b_longstr1[j] = longstr2[2 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 5, m0, b_longstr1);
  emlrtAssign(&m_y, m0);
  satrec.satnum = emlrt_marshallIn(str2num(m_y, &i_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&o_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&p_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  n_y = NULL;
  m0 = mxCreateCharArray(2, iv10);
  for (j = 0; j < 9; j++) {
    b_longstr2[j] = longstr2[7 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 9, m0, b_longstr2);
  emlrtAssign(&n_y, m0);
  satrec.inclo = emlrt_marshallIn(str2num(n_y, &j_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&p_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&q_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  o_y = NULL;
  m0 = mxCreateCharArray(2, iv11);
  for (j = 0; j < 9; j++) {
    b_longstr2[j] = longstr2[16 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 9, m0, b_longstr2);
  emlrtAssign(&o_y, m0);
  satrec.nodeo = emlrt_marshallIn(str2num(o_y, &k_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&q_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&r_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  p_y = NULL;
  m0 = mxCreateCharArray(2, iv12);
  for (j = 0; j < 8; j++) {
    c_longstr2[j] = longstr2[25 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 8, m0, c_longstr2);
  emlrtAssign(&p_y, m0);
  satrec.ecco = emlrt_marshallIn(str2num(p_y, &l_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&r_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&s_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  q_y = NULL;
  m0 = mxCreateCharArray(2, iv13);
  for (j = 0; j < 9; j++) {
    b_longstr2[j] = longstr2[33 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 9, m0, b_longstr2);
  emlrtAssign(&q_y, m0);
  satrec.argpo = emlrt_marshallIn(str2num(q_y, &m_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&s_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&t_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  r_y = NULL;
  m0 = mxCreateCharArray(2, iv14);
  for (j = 0; j < 9; j++) {
    b_longstr2[j] = longstr2[42 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 9, m0, b_longstr2);
  emlrtAssign(&r_y, m0);
  satrec.mo = emlrt_marshallIn(str2num(r_y, &n_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&t_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&u_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  s_y = NULL;
  m0 = mxCreateCharArray(2, iv15);
  for (j = 0; j < 12; j++) {
    d_longstr1[j] = longstr2[51 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 12, m0, d_longstr1);
  emlrtAssign(&s_y, m0);
  satrec.no = emlrt_marshallIn(str2num(s_y, &o_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&u_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&v_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  t_y = NULL;
  m0 = mxCreateCharArray(2, iv16);
  for (j = 0; j < 5; j++) {
    b_longstr1[j] = longstr2[63 + j];
  }

  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 5, m0, b_longstr1);
  emlrtAssign(&t_y, m0);
  emlrt_marshallIn(str2num(t_y, &x_emlrtMCI), "str2num");
  emlrt_synchGlobalsFromML();
  emlrtPopRtStackR2012b(&v_emlrtRSI, emlrtRootTLSGlobal);

  /*  find no, ndot, nddot */
  satrec.no /= 229.18311805232929;
  satrec_dirty |= 1U;

  /* //$ rad/min */
  emlrtPushRtStackR2012b(&emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  u_y = NULL;
  m0 = mxCreateDoubleScalar(satrec.nddot);
  emlrtAssign(&u_y, m0);
  v_y = NULL;
  m0 = mxCreateDoubleScalar(10.0);
  emlrtAssign(&v_y, m0);
  satrec.nddot = emlrt_marshallIn(mtimes(u_y, mpower(v_y, emlrtAlias(nexp),
    &p_emlrtMCI), &q_emlrtMCI), "mtimes");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&b_emlrtRSI, emlrtRootTLSGlobal);
  emlrt_synchGlobalsToML();
  w_y = NULL;
  m0 = mxCreateDoubleScalar(satrec.bstar);
  emlrtAssign(&w_y, m0);
  x_y = NULL;
  m0 = mxCreateDoubleScalar(10.0);
  emlrtAssign(&x_y, m0);
  satrec.bstar = emlrt_marshallIn(mtimes(w_y, mpower(x_y, emlrtAlias(ibexp),
    &r_emlrtMCI), &s_emlrtMCI), "mtimes");
  emlrt_synchGlobalsFromML();
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&b_emlrtRSI, emlrtRootTLSGlobal);

  /*  convert to sgp4 units */
  emlrtPushRtStackR2012b(&x_emlrtRSI, emlrtRootTLSGlobal);
  year = satrec.no * gravc.tumin;
  emlrtPushRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPushRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  if (year < 0.0) {
    emlrtPushRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
    b_eml_error();
    emlrtPopRtStackR2012b(&gb_emlrtRSI, emlrtRootTLSGlobal);
  }

  emlrtPopRtStackR2012b(&fb_emlrtRSI, emlrtRootTLSGlobal);
  emlrtPopRtStackR2012b(&eb_emlrtRSI, emlrtRootTLSGlobal);
  satrec.a = muDoubleScalarPower(year, -0.66666666666666663);
  satrec_dirty |= 1U;
  emlrtPopRtStackR2012b(&x_emlrtRSI, emlrtRootTLSGlobal);

  /*  [er] */
  satrec.ndot /= 330023.68999535416;
  satrec_dirty |= 1U;

  /*  [rad/min^2] */
  satrec.nddot /= 4.7523411359331E+8;
  satrec_dirty |= 1U;

  /*  [rad/min^3] */
  /*  find standard orbital elements */
  satrec.inclo *= 0.017453292519943295;
  satrec_dirty |= 1U;
  satrec.nodeo *= 0.017453292519943295;
  satrec_dirty |= 1U;
  satrec.argpo *= 0.017453292519943295;
  satrec_dirty |= 1U;
  satrec.mo *= 0.017453292519943295;
  satrec_dirty |= 1U;
  satrec.alta = satrec.a * (1.0 + satrec.ecco) - 1.0;
  satrec_dirty |= 1U;
  satrec.altp = satrec.a * (1.0 - satrec.ecco) - 1.0;
  satrec_dirty |= 1U;

  /* % Find SGP4 Epoch Time of element set */
  /*   Remember that sgp4 uses units of days from 0 jan 1950 (sgp4epoch) and */
  /*   minutes from the epoch (time) */
  /*  Temp fix for years 1957-2056 */
  /*  Correct fix will occur when year is 7-digits in 21e */
  if (satrec.epochyr < 57.0) {
    year = satrec.epochyr + 2000.0;
  } else {
    year = satrec.epochyr + 1900.0;
  }

  emlrtPushRtStackR2012b(&y_emlrtRSI, emlrtRootTLSGlobal);
  days2mdh(year, satrec.epochdays, &mon, &day, &hr, &minute, &sec);
  emlrtPopRtStackR2012b(&y_emlrtRSI, emlrtRootTLSGlobal);

  /*  ----------------------------------------------------------------------------- */
  /*  */
  /*                            function jday.m */
  /*  */
  /*   this function finds the julian date given the year, month, day, and time. */
  /*  */
  /*   author        : david vallado                  719-573-2600   27 may 2002 */
  /*  */
  /*   revisions */
  /*                 - */
  /*  */
  /*   inputs          description                    range / units */
  /*     year        - year                           1900 .. 2100 */
  /*     mon         - month                          1 .. 12 */
  /*     day         - day                            1 .. 28,29,30,31 */
  /*     hr          - universal time hour            0 .. 23 */
  /*     min         - universal time min             0 .. 59 */
  /*     sec         - universal time sec             0.0 .. 59.999 */
  /*     whichtype   - julian .or. gregorian calender   'j' .or. 'g' */
  /*  */
  /*   outputs       : */
  /*     jd          - julian date                    days from 4713 bc */
  /*  */
  /*   locals        : */
  /*     none. */
  /*  */
  /*   coupling      : */
  /*     none. */
  /*  */
  /*   references    : */
  /*     vallado       2007, 189, alg 14, ex 3-14 */
  /*  */
  /*  jd = jday(yr, mon, day, hr, min, sec) */
  /*  ----------------------------------------------------------------------------- */
  /*  ------------------------  implementation   ------------------ */
  /*   - 0.5 * sign(100.0 * yr + mon - 190002.5) + 0.5; */
  satrec.jdsatepoch = ((((367.0 * year - muDoubleScalarFloor(7.0 * (year +
    muDoubleScalarFloor((mon + 9.0) / 12.0)) * 0.25)) + muDoubleScalarFloor
    (275.0 * mon / 9.0)) + day) + 1.7210135E+6) + ((sec / 60.0 + minute) / 60.0
    + hr) / 24.0;
  satrec_dirty |= 1U;

  /*  %% Initialize the orbit at SGP4 Epoch     */
  /*  days since 0 Jan 1950 */
  emlrtPushRtStackR2012b(&ab_emlrtRSI, emlrtRootTLSGlobal);
  sgp4init();
  emlrtPopRtStackR2012b(&ab_emlrtRSI, emlrtRootTLSGlobal);
  emlrtDestroyArray(&nexp);
  emlrtDestroyArray(&ibexp);
}

/* End of code generation (SGP4_Setup.c) */
