/*
 * SGP4_Setup_api.c
 *
 * Code generation for function 'SGP4_Setup_api'
 *
 * C source code generated on: Thu Jul 11 14:30:48 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "SGP4_Setup_api.h"
#include "SGP4_Setup_mexutil.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Function Declarations */
static void b_info_helper(ResolvedFunctionInfo info[96]);
static void c_emlrt_marshallIn(const mxArray *longstr1, const char_T *identifier,
  char_T y[69]);
static void d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[69]);
static const mxArray *emlrt_marshallOut(ResolvedFunctionInfo u[96]);
static void info_helper(ResolvedFunctionInfo info[96]);
static void k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[69]);

/* Function Definitions */
static void b_info_helper(ResolvedFunctionInfo info[96])
{
  info[64].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[64].name = "mpower";
  info[64].dominantType = "double";
  info[64].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  info[64].fileTimeLo = 1286840442U;
  info[64].fileTimeHi = 0U;
  info[64].mFileTimeLo = 0U;
  info[64].mFileTimeHi = 0U;
  info[65].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[65].name = "sin";
  info[65].dominantType = "double";
  info[65].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  info[65].fileTimeLo = 1343851986U;
  info[65].fileTimeHi = 0U;
  info[65].mFileTimeLo = 0U;
  info[65].mFileTimeHi = 0U;
  info[66].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[66].name = "rem";
  info[66].dominantType = "double";
  info[66].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m";
  info[66].fileTimeLo = 1343851984U;
  info[66].fileTimeHi = 0U;
  info[66].mFileTimeLo = 0U;
  info[66].mFileTimeHi = 0U;
  info[67].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[67].name = "abs";
  info[67].dominantType = "double";
  info[67].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  info[67].fileTimeLo = 1343851966U;
  info[67].fileTimeHi = 0U;
  info[67].mFileTimeLo = 0U;
  info[67].mFileTimeHi = 0U;
  info[68].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[68].name = "sqrt";
  info[68].dominantType = "double";
  info[68].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  info[68].fileTimeLo = 1343851986U;
  info[68].fileTimeHi = 0U;
  info[68].mFileTimeLo = 0U;
  info[68].mFileTimeHi = 0U;
  info[69].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[69].name = "atan2";
  info[69].dominantType = "double";
  info[69].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  info[69].fileTimeLo = 1343851972U;
  info[69].fileTimeHi = 0U;
  info[69].mFileTimeLo = 0U;
  info[69].mFileTimeHi = 0U;
  info[70].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  info[70].name = "eml_scalar_eg";
  info[70].dominantType = "double";
  info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[70].fileTimeLo = 1286840396U;
  info[70].fileTimeHi = 0U;
  info[70].mFileTimeLo = 0U;
  info[70].mFileTimeHi = 0U;
  info[71].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  info[71].name = "eml_scalexp_alloc";
  info[71].dominantType = "double";
  info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  info[71].fileTimeLo = 1352446460U;
  info[71].fileTimeHi = 0U;
  info[71].mFileTimeLo = 0U;
  info[71].mFileTimeHi = 0U;
  info[72].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m";
  info[72].name = "eml_scalar_atan2";
  info[72].dominantType = "double";
  info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  info[72].fileTimeLo = 1286840320U;
  info[72].fileTimeHi = 0U;
  info[72].mFileTimeLo = 0U;
  info[72].mFileTimeHi = 0U;
  info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  info[73].name = "mtimes";
  info[73].dominantType = "double";
  info[73].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[73].fileTimeLo = 1289541292U;
  info[73].fileTimeHi = 0U;
  info[73].mFileTimeLo = 0U;
  info[73].mFileTimeHi = 0U;
  info[74].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[74].name = "mod";
  info[74].dominantType = "double";
  info[74].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  info[74].fileTimeLo = 1343851982U;
  info[74].fileTimeHi = 0U;
  info[74].mFileTimeLo = 0U;
  info[74].mFileTimeHi = 0U;
  info[75].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  info[75].name = "eml_scalar_eg";
  info[75].dominantType = "double";
  info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[75].fileTimeLo = 1286840396U;
  info[75].fileTimeHi = 0U;
  info[75].mFileTimeLo = 0U;
  info[75].mFileTimeHi = 0U;
  info[76].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  info[76].name = "eml_scalexp_alloc";
  info[76].dominantType = "double";
  info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  info[76].fileTimeLo = 1352446460U;
  info[76].fileTimeHi = 0U;
  info[76].mFileTimeLo = 0U;
  info[76].mFileTimeHi = 0U;
  info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[77].name = "eml_scalar_eg";
  info[77].dominantType = "double";
  info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[77].fileTimeLo = 1286840396U;
  info[77].fileTimeHi = 0U;
  info[77].mFileTimeLo = 0U;
  info[77].mFileTimeHi = 0U;
  info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[78].name = "eml_scalar_floor";
  info[78].dominantType = "double";
  info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  info[78].fileTimeLo = 1286840326U;
  info[78].fileTimeHi = 0U;
  info[78].mFileTimeLo = 0U;
  info[78].mFileTimeHi = 0U;
  info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[79].name = "eml_scalar_round";
  info[79].dominantType = "double";
  info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  info[79].fileTimeLo = 1307672838U;
  info[79].fileTimeHi = 0U;
  info[79].mFileTimeLo = 0U;
  info[79].mFileTimeHi = 0U;
  info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[80].name = "eml_scalar_abs";
  info[80].dominantType = "double";
  info[80].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  info[80].fileTimeLo = 1286840312U;
  info[80].fileTimeHi = 0U;
  info[80].mFileTimeLo = 0U;
  info[80].mFileTimeHi = 0U;
  info[81].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[81].name = "eps";
  info[81].dominantType = "char";
  info[81].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  info[81].fileTimeLo = 1326749596U;
  info[81].fileTimeHi = 0U;
  info[81].mFileTimeLo = 0U;
  info[81].mFileTimeHi = 0U;
  info[82].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  info[82].name = "eml_is_float_class";
  info[82].dominantType = "char";
  info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  info[82].fileTimeLo = 1286840382U;
  info[82].fileTimeHi = 0U;
  info[82].mFileTimeLo = 0U;
  info[82].mFileTimeHi = 0U;
  info[83].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  info[83].name = "eml_eps";
  info[83].dominantType = "char";
  info[83].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  info[83].fileTimeLo = 1326749596U;
  info[83].fileTimeHi = 0U;
  info[83].mFileTimeLo = 0U;
  info[83].mFileTimeHi = 0U;
  info[84].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  info[84].name = "eml_float_model";
  info[84].dominantType = "char";
  info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  info[84].fileTimeLo = 1326749596U;
  info[84].fileTimeHi = 0U;
  info[84].mFileTimeLo = 0U;
  info[84].mFileTimeHi = 0U;
  info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m!floatmod";
  info[85].name = "mtimes";
  info[85].dominantType = "double";
  info[85].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[85].fileTimeLo = 1289541292U;
  info[85].fileTimeHi = 0U;
  info[85].mFileTimeLo = 0U;
  info[85].mFileTimeHi = 0U;
  info[86].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[86].name = "eml_index_class";
  info[86].dominantType = "";
  info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  info[86].fileTimeLo = 1323192178U;
  info[86].fileTimeHi = 0U;
  info[86].mFileTimeLo = 0U;
  info[86].mFileTimeHi = 0U;
  info[87].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[87].name = "eml_scalar_eg";
  info[87].dominantType = "double";
  info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[87].fileTimeLo = 1286840396U;
  info[87].fileTimeHi = 0U;
  info[87].mFileTimeLo = 0U;
  info[87].mFileTimeHi = 0U;
  info[88].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[88].name = "eml_xgemm";
  info[88].dominantType = "char";
  info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  info[88].fileTimeLo = 1299098372U;
  info[88].fileTimeHi = 0U;
  info[88].mFileTimeLo = 0U;
  info[88].mFileTimeHi = 0U;
  info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  info[89].name = "eml_blas_inline";
  info[89].dominantType = "";
  info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  info[89].fileTimeLo = 1299098368U;
  info[89].fileTimeHi = 0U;
  info[89].mFileTimeLo = 0U;
  info[89].mFileTimeHi = 0U;
  info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  info[90].name = "mtimes";
  info[90].dominantType = "double";
  info[90].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[90].fileTimeLo = 1289541292U;
  info[90].fileTimeHi = 0U;
  info[90].mFileTimeLo = 0U;
  info[90].mFileTimeHi = 0U;
  info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  info[91].name = "eml_index_class";
  info[91].dominantType = "";
  info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  info[91].fileTimeLo = 1323192178U;
  info[91].fileTimeHi = 0U;
  info[91].mFileTimeLo = 0U;
  info[91].mFileTimeHi = 0U;
  info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  info[92].name = "eml_scalar_eg";
  info[92].dominantType = "double";
  info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[92].fileTimeLo = 1286840396U;
  info[92].fileTimeHi = 0U;
  info[92].mFileTimeLo = 0U;
  info[92].mFileTimeHi = 0U;
  info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  info[93].name = "eml_refblas_xgemm";
  info[93].dominantType = "char";
  info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  info[93].fileTimeLo = 1299098374U;
  info[93].fileTimeHi = 0U;
  info[93].mFileTimeLo = 0U;
  info[93].mFileTimeHi = 0U;
  info[94].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[94].name = "cross";
  info[94].dominantType = "double";
  info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/cross.m";
  info[94].fileTimeLo = 1286840442U;
  info[94].fileTimeHi = 0U;
  info[94].mFileTimeLo = 0U;
  info[94].mFileTimeHi = 0U;
  info[95].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/cross.m";
  info[95].name = "mtimes";
  info[95].dominantType = "double";
  info[95].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[95].fileTimeLo = 1289541292U;
  info[95].fileTimeHi = 0U;
  info[95].mFileTimeLo = 0U;
  info[95].mFileTimeHi = 0U;
}

static void c_emlrt_marshallIn(const mxArray *longstr1, const char_T *identifier,
  char_T y[69])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  d_emlrt_marshallIn(emlrtAlias(longstr1), &thisId, y);
  emlrtDestroyArray(&longstr1);
}

static void d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[69])
{
  k_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *emlrt_marshallOut(ResolvedFunctionInfo u[96])
{
  const mxArray *y;
  int32_T iv18[1];
  int32_T i1;
  ResolvedFunctionInfo *r0;
  const char * b_u;
  const mxArray *b_y;
  const mxArray *m1;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  uint32_T c_u;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  y = NULL;
  iv18[0] = 96;
  emlrtAssign(&y, mxCreateStructArray(1, iv18, 0, NULL));
  for (i1 = 0; i1 < 96; i1++) {
    r0 = &u[i1];
    b_u = r0->context;
    b_y = NULL;
    m1 = mxCreateString(b_u);
    emlrtAssign(&b_y, m1);
    emlrtAddField(y, b_y, "context", i1);
    b_u = r0->name;
    c_y = NULL;
    m1 = mxCreateString(b_u);
    emlrtAssign(&c_y, m1);
    emlrtAddField(y, c_y, "name", i1);
    b_u = r0->dominantType;
    d_y = NULL;
    m1 = mxCreateString(b_u);
    emlrtAssign(&d_y, m1);
    emlrtAddField(y, d_y, "dominantType", i1);
    b_u = r0->resolved;
    e_y = NULL;
    m1 = mxCreateString(b_u);
    emlrtAssign(&e_y, m1);
    emlrtAddField(y, e_y, "resolved", i1);
    c_u = r0->fileTimeLo;
    f_y = NULL;
    m1 = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)mxGetData(m1) = c_u;
    emlrtAssign(&f_y, m1);
    emlrtAddField(y, f_y, "fileTimeLo", i1);
    c_u = r0->fileTimeHi;
    g_y = NULL;
    m1 = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)mxGetData(m1) = c_u;
    emlrtAssign(&g_y, m1);
    emlrtAddField(y, g_y, "fileTimeHi", i1);
    c_u = r0->mFileTimeLo;
    h_y = NULL;
    m1 = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)mxGetData(m1) = c_u;
    emlrtAssign(&h_y, m1);
    emlrtAddField(y, h_y, "mFileTimeLo", i1);
    c_u = r0->mFileTimeHi;
    i_y = NULL;
    m1 = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *(uint32_T *)mxGetData(m1) = c_u;
    emlrtAssign(&i_y, m1);
    emlrtAddField(y, i_y, "mFileTimeHi", i1);
  }

  return y;
}

static void info_helper(ResolvedFunctionInfo info[96])
{
  info[0].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[0].name = "getgravc";
  info[0].dominantType = "double";
  info[0].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m";
  info[0].fileTimeLo = 1373561244U;
  info[0].fileTimeHi = 0U;
  info[0].mFileTimeLo = 0U;
  info[0].mFileTimeHi = 0U;
  info[1].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m";
  info[1].name = "mtimes";
  info[1].dominantType = "double";
  info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[1].fileTimeLo = 1289541292U;
  info[1].fileTimeHi = 0U;
  info[1].mFileTimeLo = 0U;
  info[1].mFileTimeHi = 0U;
  info[2].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m";
  info[2].name = "mrdivide";
  info[2].dominantType = "double";
  info[2].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[2].fileTimeLo = 1357973148U;
  info[2].fileTimeHi = 0U;
  info[2].mFileTimeLo = 1319751566U;
  info[2].mFileTimeHi = 0U;
  info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[3].name = "rdivide";
  info[3].dominantType = "double";
  info[3].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  info[3].fileTimeLo = 1346531988U;
  info[3].fileTimeHi = 0U;
  info[3].mFileTimeLo = 0U;
  info[3].mFileTimeHi = 0U;
  info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  info[4].name = "eml_scalexp_compatible";
  info[4].dominantType = "double";
  info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  info[4].fileTimeLo = 1286840396U;
  info[4].fileTimeHi = 0U;
  info[4].mFileTimeLo = 0U;
  info[4].mFileTimeHi = 0U;
  info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  info[5].name = "eml_div";
  info[5].dominantType = "double";
  info[5].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  info[5].fileTimeLo = 1313369410U;
  info[5].fileTimeHi = 0U;
  info[5].mFileTimeLo = 0U;
  info[5].mFileTimeHi = 0U;
  info[6].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m";
  info[6].name = "sqrt";
  info[6].dominantType = "double";
  info[6].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  info[6].fileTimeLo = 1343851986U;
  info[6].fileTimeHi = 0U;
  info[6].mFileTimeLo = 0U;
  info[6].mFileTimeHi = 0U;
  info[7].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  info[7].name = "eml_error";
  info[7].dominantType = "char";
  info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  info[7].fileTimeLo = 1343851958U;
  info[7].fileTimeHi = 0U;
  info[7].mFileTimeLo = 0U;
  info[7].mFileTimeHi = 0U;
  info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  info[8].name = "eml_scalar_sqrt";
  info[8].dominantType = "double";
  info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  info[8].fileTimeLo = 1286840338U;
  info[8].fileTimeHi = 0U;
  info[8].mFileTimeLo = 0U;
  info[8].mFileTimeHi = 0U;
  info[9].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/getgravc.m";
  info[9].name = "fprintf";
  info[9].dominantType = "char";
  info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m";
  info[9].fileTimeLo = 1354862384U;
  info[9].fileTimeHi = 0U;
  info[9].mFileTimeLo = 0U;
  info[9].mFileTimeHi = 0U;
  info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m";
  info[10].name = "isequal";
  info[10].dominantType = "char";
  info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  info[10].fileTimeLo = 1286840358U;
  info[10].fileTimeHi = 0U;
  info[10].mFileTimeLo = 0U;
  info[10].mFileTimeHi = 0U;
  info[11].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  info[11].name = "eml_isequal_core";
  info[11].dominantType = "char";
  info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  info[11].fileTimeLo = 1286840386U;
  info[11].fileTimeHi = 0U;
  info[11].mFileTimeLo = 0U;
  info[11].mFileTimeHi = 0U;
  info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m!isequal_scalar";
  info[12].name = "isnan";
  info[12].dominantType = "char";
  info[12].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  info[12].fileTimeLo = 1286840360U;
  info[12].fileTimeHi = 0U;
  info[12].mFileTimeLo = 0U;
  info[12].mFileTimeHi = 0U;
  info[13].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m";
  info[13].name = "fprintf";
  info[13].dominantType = "double";
  info[13].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m";
  info[13].fileTimeLo = 1354862384U;
  info[13].fileTimeHi = 0U;
  info[13].mFileTimeLo = 0U;
  info[13].mFileTimeHi = 0U;
  info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m!validate_arguments";
  info[14].name = "coder.internal.assert";
  info[14].dominantType = "char";
  info[14].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m";
  info[14].fileTimeLo = 1334093538U;
  info[14].fileTimeHi = 0U;
  info[14].mFileTimeLo = 0U;
  info[14].mFileTimeHi = 0U;
  info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/lang/fprintf.m!check_type";
  info[15].name = "coder.internal.assert";
  info[15].dominantType = "char";
  info[15].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m";
  info[15].fileTimeLo = 1334093538U;
  info[15].fileTimeHi = 0U;
  info[15].mFileTimeLo = 0U;
  info[15].mFileTimeHi = 0U;
  info[16].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[16].name = "mrdivide";
  info[16].dominantType = "double";
  info[16].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[16].fileTimeLo = 1357973148U;
  info[16].fileTimeHi = 0U;
  info[16].mFileTimeLo = 1319751566U;
  info[16].mFileTimeHi = 0U;
  info[17].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[17].name = "mtimes";
  info[17].dominantType = "double";
  info[17].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[17].fileTimeLo = 1289541292U;
  info[17].fileTimeHi = 0U;
  info[17].mFileTimeLo = 0U;
  info[17].mFileTimeHi = 0U;
  info[18].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[18].name = "length";
  info[18].dominantType = "char";
  info[18].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  info[18].fileTimeLo = 1303167806U;
  info[18].fileTimeHi = 0U;
  info[18].mFileTimeLo = 0U;
  info[18].mFileTimeHi = 0U;
  info[19].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[19].name = "mpower";
  info[19].dominantType = "double";
  info[19].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  info[19].fileTimeLo = 1286840442U;
  info[19].fileTimeHi = 0U;
  info[19].mFileTimeLo = 0U;
  info[19].mFileTimeHi = 0U;
  info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  info[20].name = "power";
  info[20].dominantType = "double";
  info[20].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  info[20].fileTimeLo = 1348213530U;
  info[20].fileTimeHi = 0U;
  info[20].mFileTimeLo = 0U;
  info[20].mFileTimeHi = 0U;
  info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  info[21].name = "eml_scalar_eg";
  info[21].dominantType = "double";
  info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[21].fileTimeLo = 1286840396U;
  info[21].fileTimeHi = 0U;
  info[21].mFileTimeLo = 0U;
  info[21].mFileTimeHi = 0U;
  info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  info[22].name = "eml_scalexp_alloc";
  info[22].dominantType = "double";
  info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  info[22].fileTimeLo = 1352446460U;
  info[22].fileTimeHi = 0U;
  info[22].mFileTimeLo = 0U;
  info[22].mFileTimeHi = 0U;
  info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  info[23].name = "floor";
  info[23].dominantType = "double";
  info[23].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  info[23].fileTimeLo = 1343851980U;
  info[23].fileTimeHi = 0U;
  info[23].mFileTimeLo = 0U;
  info[23].mFileTimeHi = 0U;
  info[24].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  info[24].name = "eml_scalar_floor";
  info[24].dominantType = "double";
  info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  info[24].fileTimeLo = 1286840326U;
  info[24].fileTimeHi = 0U;
  info[24].mFileTimeLo = 0U;
  info[24].mFileTimeHi = 0U;
  info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  info[25].name = "eml_error";
  info[25].dominantType = "char";
  info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  info[25].fileTimeLo = 1343851958U;
  info[25].fileTimeHi = 0U;
  info[25].mFileTimeLo = 0U;
  info[25].mFileTimeHi = 0U;
  info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  info[26].name = "eml_scalar_eg";
  info[26].dominantType = "double";
  info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[26].fileTimeLo = 1286840396U;
  info[26].fileTimeHi = 0U;
  info[26].mFileTimeLo = 0U;
  info[26].mFileTimeHi = 0U;
  info[27].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[27].name = "days2mdh";
  info[27].dominantType = "double";
  info[27].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/days2mdh.m";
  info[27].fileTimeLo = 1373566350U;
  info[27].fileTimeHi = 0U;
  info[27].mFileTimeLo = 0U;
  info[27].mFileTimeHi = 0U;
  info[28].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/days2mdh.m";
  info[28].name = "mtimes";
  info[28].dominantType = "double";
  info[28].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[28].fileTimeLo = 1289541292U;
  info[28].fileTimeHi = 0U;
  info[28].mFileTimeLo = 0U;
  info[28].mFileTimeHi = 0U;
  info[29].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/days2mdh.m";
  info[29].name = "floor";
  info[29].dominantType = "double";
  info[29].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  info[29].fileTimeLo = 1343851980U;
  info[29].fileTimeHi = 0U;
  info[29].mFileTimeLo = 0U;
  info[29].mFileTimeHi = 0U;
  info[30].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/days2mdh.m";
  info[30].name = "rem";
  info[30].dominantType = "double";
  info[30].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m";
  info[30].fileTimeLo = 1343851984U;
  info[30].fileTimeHi = 0U;
  info[30].mFileTimeLo = 0U;
  info[30].mFileTimeHi = 0U;
  info[31].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m";
  info[31].name = "eml_scalar_eg";
  info[31].dominantType = "double";
  info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  info[31].fileTimeLo = 1286840396U;
  info[31].fileTimeHi = 0U;
  info[31].mFileTimeLo = 0U;
  info[31].mFileTimeHi = 0U;
  info[32].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m";
  info[32].name = "eml_scalexp_alloc";
  info[32].dominantType = "double";
  info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  info[32].fileTimeLo = 1352446460U;
  info[32].fileTimeHi = 0U;
  info[32].mFileTimeLo = 0U;
  info[32].mFileTimeHi = 0U;
  info[33].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/days2mdh.m";
  info[33].name = "fix";
  info[33].dominantType = "double";
  info[33].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/fix.m";
  info[33].fileTimeLo = 1343851980U;
  info[33].fileTimeHi = 0U;
  info[33].mFileTimeLo = 0U;
  info[33].mFileTimeHi = 0U;
  info[34].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/fix.m";
  info[34].name = "eml_scalar_fix";
  info[34].dominantType = "double";
  info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_fix.m";
  info[34].fileTimeLo = 1307672838U;
  info[34].fileTimeHi = 0U;
  info[34].mFileTimeLo = 0U;
  info[34].mFileTimeHi = 0U;
  info[35].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[35].name = "jday";
  info[35].dominantType = "double";
  info[35].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/jday.m";
  info[35].fileTimeLo = 1373385345U;
  info[35].fileTimeHi = 0U;
  info[35].mFileTimeLo = 0U;
  info[35].mFileTimeHi = 0U;
  info[36].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/jday.m";
  info[36].name = "mtimes";
  info[36].dominantType = "double";
  info[36].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[36].fileTimeLo = 1289541292U;
  info[36].fileTimeHi = 0U;
  info[36].mFileTimeLo = 0U;
  info[36].mFileTimeHi = 0U;
  info[37].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/jday.m";
  info[37].name = "mrdivide";
  info[37].dominantType = "double";
  info[37].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[37].fileTimeLo = 1357973148U;
  info[37].fileTimeHi = 0U;
  info[37].mFileTimeLo = 1319751566U;
  info[37].mFileTimeHi = 0U;
  info[38].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/jday.m";
  info[38].name = "floor";
  info[38].dominantType = "double";
  info[38].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  info[38].fileTimeLo = 1343851980U;
  info[38].fileTimeHi = 0U;
  info[38].mFileTimeLo = 0U;
  info[38].mFileTimeHi = 0U;
  info[39].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/SGP4_Setup.m";
  info[39].name = "sgp4init";
  info[39].dominantType = "double";
  info[39].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[39].fileTimeLo = 1373566727U;
  info[39].fileTimeHi = 0U;
  info[39].mFileTimeLo = 0U;
  info[39].mFileTimeHi = 0U;
  info[40].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[40].name = "mrdivide";
  info[40].dominantType = "double";
  info[40].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[40].fileTimeLo = 1357973148U;
  info[40].fileTimeHi = 0U;
  info[40].mFileTimeLo = 1319751566U;
  info[40].mFileTimeHi = 0U;
  info[41].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[41].name = "mpower";
  info[41].dominantType = "double";
  info[41].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  info[41].fileTimeLo = 1286840442U;
  info[41].fileTimeHi = 0U;
  info[41].mFileTimeLo = 0U;
  info[41].mFileTimeHi = 0U;
  info[42].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[42].name = "initl";
  info[42].dominantType = "double";
  info[42].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[42].fileTimeLo = 1373566628U;
  info[42].fileTimeHi = 0U;
  info[42].mFileTimeLo = 0U;
  info[42].mFileTimeHi = 0U;
  info[43].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[43].name = "mrdivide";
  info[43].dominantType = "double";
  info[43].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[43].fileTimeLo = 1357973148U;
  info[43].fileTimeHi = 0U;
  info[43].mFileTimeLo = 1319751566U;
  info[43].mFileTimeHi = 0U;
  info[44].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[44].name = "mtimes";
  info[44].dominantType = "double";
  info[44].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[44].fileTimeLo = 1289541292U;
  info[44].fileTimeHi = 0U;
  info[44].mFileTimeLo = 0U;
  info[44].mFileTimeHi = 0U;
  info[45].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[45].name = "sqrt";
  info[45].dominantType = "double";
  info[45].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  info[45].fileTimeLo = 1343851986U;
  info[45].fileTimeHi = 0U;
  info[45].mFileTimeLo = 0U;
  info[45].mFileTimeHi = 0U;
  info[46].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[46].name = "cos";
  info[46].dominantType = "double";
  info[46].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  info[46].fileTimeLo = 1343851972U;
  info[46].fileTimeHi = 0U;
  info[46].mFileTimeLo = 0U;
  info[46].mFileTimeHi = 0U;
  info[47].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  info[47].name = "eml_scalar_cos";
  info[47].dominantType = "double";
  info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  info[47].fileTimeLo = 1286840322U;
  info[47].fileTimeHi = 0U;
  info[47].mFileTimeLo = 0U;
  info[47].mFileTimeHi = 0U;
  info[48].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[48].name = "mpower";
  info[48].dominantType = "double";
  info[48].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  info[48].fileTimeLo = 1286840442U;
  info[48].fileTimeHi = 0U;
  info[48].mFileTimeLo = 0U;
  info[48].mFileTimeHi = 0U;
  info[49].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[49].name = "sin";
  info[49].dominantType = "double";
  info[49].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  info[49].fileTimeLo = 1343851986U;
  info[49].fileTimeHi = 0U;
  info[49].mFileTimeLo = 0U;
  info[49].mFileTimeHi = 0U;
  info[50].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  info[50].name = "eml_scalar_sin";
  info[50].dominantType = "double";
  info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  info[50].fileTimeLo = 1286840336U;
  info[50].fileTimeHi = 0U;
  info[50].mFileTimeLo = 0U;
  info[50].mFileTimeHi = 0U;
  info[51].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/initl.m";
  info[51].name = "gstime";
  info[51].dominantType = "double";
  info[51].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/gstime.m";
  info[51].fileTimeLo = 1373385345U;
  info[51].fileTimeHi = 0U;
  info[51].mFileTimeLo = 0U;
  info[51].mFileTimeHi = 0U;
  info[52].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/gstime.m";
  info[52].name = "mtimes";
  info[52].dominantType = "double";
  info[52].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[52].fileTimeLo = 1289541292U;
  info[52].fileTimeHi = 0U;
  info[52].mFileTimeLo = 0U;
  info[52].mFileTimeHi = 0U;
  info[53].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/gstime.m";
  info[53].name = "mrdivide";
  info[53].dominantType = "double";
  info[53].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[53].fileTimeLo = 1357973148U;
  info[53].fileTimeHi = 0U;
  info[53].mFileTimeLo = 1319751566U;
  info[53].mFileTimeHi = 0U;
  info[54].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/gstime.m";
  info[54].name = "rem";
  info[54].dominantType = "double";
  info[54].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m";
  info[54].fileTimeLo = 1343851984U;
  info[54].fileTimeHi = 0U;
  info[54].mFileTimeLo = 0U;
  info[54].mFileTimeHi = 0U;
  info[55].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[55].name = "mtimes";
  info[55].dominantType = "double";
  info[55].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[55].fileTimeLo = 1289541292U;
  info[55].fileTimeHi = 0U;
  info[55].mFileTimeLo = 0U;
  info[55].mFileTimeHi = 0U;
  info[56].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[56].name = "abs";
  info[56].dominantType = "double";
  info[56].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  info[56].fileTimeLo = 1343851966U;
  info[56].fileTimeHi = 0U;
  info[56].mFileTimeLo = 0U;
  info[56].mFileTimeHi = 0U;
  info[57].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  info[57].name = "eml_scalar_abs";
  info[57].dominantType = "double";
  info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  info[57].fileTimeLo = 1286840312U;
  info[57].fileTimeHi = 0U;
  info[57].mFileTimeLo = 0U;
  info[57].mFileTimeHi = 0U;
  info[58].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[58].name = "cos";
  info[58].dominantType = "double";
  info[58].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  info[58].fileTimeLo = 1343851972U;
  info[58].fileTimeHi = 0U;
  info[58].mFileTimeLo = 0U;
  info[58].mFileTimeHi = 0U;
  info[59].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[59].name = "sin";
  info[59].dominantType = "double";
  info[59].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m";
  info[59].fileTimeLo = 1343851986U;
  info[59].fileTimeHi = 0U;
  info[59].mFileTimeLo = 0U;
  info[59].mFileTimeHi = 0U;
  info[60].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4init.m";
  info[60].name = "sgp4";
  info[60].dominantType = "double";
  info[60].resolved =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[60].fileTimeLo = 1373567286U;
  info[60].fileTimeHi = 0U;
  info[60].mFileTimeLo = 0U;
  info[60].mFileTimeHi = 0U;
  info[61].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[61].name = "mtimes";
  info[61].dominantType = "double";
  info[61].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  info[61].fileTimeLo = 1289541292U;
  info[61].fileTimeHi = 0U;
  info[61].mFileTimeLo = 0U;
  info[61].mFileTimeHi = 0U;
  info[62].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[62].name = "mrdivide";
  info[62].dominantType = "double";
  info[62].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  info[62].fileTimeLo = 1357973148U;
  info[62].fileTimeHi = 0U;
  info[62].mFileTimeLo = 1319751566U;
  info[62].mFileTimeHi = 0U;
  info[63].context =
    "[E]//mtucifs2/home/Attitude Control Software/SGP4 Prop/mat/sgp4.m";
  info[63].name = "cos";
  info[63].dominantType = "double";
  info[63].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  info[63].fileTimeLo = 1343851972U;
  info[63].fileTimeHi = 0U;
  info[63].mFileTimeLo = 0U;
  info[63].mFileTimeHi = 0U;
}

static void k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[69])
{
  int32_T iv19[2];
  int32_T i2;
  for (i2 = 0; i2 < 2; i2++) {
    iv19[i2] = 1 + 68 * i2;
  }

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "char", FALSE, 2U,
    iv19);
  emlrtImportCharArray(src, ret, 69);
  emlrtDestroyArray(&src);
}

void SGP4_Setup_api(const mxArray * const prhs[2])
{
  char_T longstr1[69];
  char_T longstr2[69];
  const mxArray *tmp;

  /* Marshall function inputs */
  c_emlrt_marshallIn(emlrtAliasP(prhs[0]), "longstr1", longstr1);
  c_emlrt_marshallIn(emlrtAliasP(prhs[1]), "longstr2", longstr2);

  /* Marshall in global variables */
  tmp = mexGetVariable("global", "satrec");
  if (tmp) {
    e_emlrt_marshallIn(tmp, "satrec", &satrec);
    satrec_dirty = 0U;
  }

  tmp = mexGetVariable("global", "gravc");
  if (tmp) {
    gravc = h_emlrt_marshallIn(tmp, "gravc");
    gravc_dirty = 0U;
  }

  /* Invoke the target function */
  SGP4_Setup(longstr1, longstr2);

  /* Marshall out global variables */
  mexPutVariable("global", "satrec", b_emlrt_marshallOut(&satrec));
  mexPutVariable("global", "gravc", c_emlrt_marshallOut(gravc));
}

const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  ResolvedFunctionInfo info[96];
  nameCaptureInfo = NULL;
  info_helper(info);
  b_info_helper(info);
  emlrtAssign(&nameCaptureInfo, emlrt_marshallOut(info));
  emlrtNameCapturePostProcessR2012a(emlrtAlias(nameCaptureInfo));
  return nameCaptureInfo;
}

/* End of code generation (SGP4_Setup_api.c) */
