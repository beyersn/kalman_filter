/*
 * SGP4_Setup_mexutil.c
 *
 * Code generation for function 'SGP4_Setup_mexutil'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "SGP4_Setup_mexutil.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Function Declarations */
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, struct_T *y);
static char_T g_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static b_struct_T i_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId);
static real_T j_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);
static char_T l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);

/* Function Definitions */
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, struct_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[46] = { "error", "satnum", "epochyr",
    "epochdays", "ndot", "nddot", "bstar", "inclo", "nodeo", "ecco", "argpo",
    "mo", "no", "a", "alta", "altp", "jdsatepoch", "isimp", "method", "aycof",
    "con41", "cc1", "cc4", "cc5", "d2", "d3", "d4", "delmo", "eta", "argpdot",
    "omgcof", "sinmao", "t", "t2cof", "t3cof", "t4cof", "t5cof", "x1mth2",
    "x7thm1", "mdot", "nodedot", "xlcof", "xmcof", "nodecf", "init", "gsto" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 46, fieldNames, 0U, 0);
  thisId.fIdentifier = "error";
  y->error = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "error")), &thisId);
  thisId.fIdentifier = "satnum";
  y->satnum = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "satnum")), &thisId);
  thisId.fIdentifier = "epochyr";
  y->epochyr = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "epochyr")), &thisId);
  thisId.fIdentifier = "epochdays";
  y->epochdays = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "epochdays")), &thisId);
  thisId.fIdentifier = "ndot";
  y->ndot = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "ndot")), &thisId);
  thisId.fIdentifier = "nddot";
  y->nddot = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "nddot")), &thisId);
  thisId.fIdentifier = "bstar";
  y->bstar = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "bstar")), &thisId);
  thisId.fIdentifier = "inclo";
  y->inclo = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "inclo")), &thisId);
  thisId.fIdentifier = "nodeo";
  y->nodeo = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "nodeo")), &thisId);
  thisId.fIdentifier = "ecco";
  y->ecco = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "ecco")), &thisId);
  thisId.fIdentifier = "argpo";
  y->argpo = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "argpo")), &thisId);
  thisId.fIdentifier = "mo";
  y->mo = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "mo")), &thisId);
  thisId.fIdentifier = "no";
  y->no = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "no")), &thisId);
  thisId.fIdentifier = "a";
  y->a = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "a")), &thisId);
  thisId.fIdentifier = "alta";
  y->alta = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "alta")), &thisId);
  thisId.fIdentifier = "altp";
  y->altp = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "altp")), &thisId);
  thisId.fIdentifier = "jdsatepoch";
  y->jdsatepoch = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "jdsatepoch")), &thisId);
  thisId.fIdentifier = "isimp";
  y->isimp = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "isimp")), &thisId);
  thisId.fIdentifier = "method";
  y->method = g_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "method")), &thisId);
  thisId.fIdentifier = "aycof";
  y->aycof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "aycof")), &thisId);
  thisId.fIdentifier = "con41";
  y->con41 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "con41")), &thisId);
  thisId.fIdentifier = "cc1";
  y->cc1 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "cc1")), &thisId);
  thisId.fIdentifier = "cc4";
  y->cc4 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "cc4")), &thisId);
  thisId.fIdentifier = "cc5";
  y->cc5 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "cc5")), &thisId);
  thisId.fIdentifier = "d2";
  y->d2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "d2")), &thisId);
  thisId.fIdentifier = "d3";
  y->d3 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "d3")), &thisId);
  thisId.fIdentifier = "d4";
  y->d4 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "d4")), &thisId);
  thisId.fIdentifier = "delmo";
  y->delmo = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "delmo")), &thisId);
  thisId.fIdentifier = "eta";
  y->eta = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "eta")), &thisId);
  thisId.fIdentifier = "argpdot";
  y->argpdot = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "argpdot")), &thisId);
  thisId.fIdentifier = "omgcof";
  y->omgcof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "omgcof")), &thisId);
  thisId.fIdentifier = "sinmao";
  y->sinmao = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "sinmao")), &thisId);
  thisId.fIdentifier = "t";
  y->t = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "t")), &thisId);
  thisId.fIdentifier = "t2cof";
  y->t2cof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "t2cof")), &thisId);
  thisId.fIdentifier = "t3cof";
  y->t3cof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "t3cof")), &thisId);
  thisId.fIdentifier = "t4cof";
  y->t4cof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "t4cof")), &thisId);
  thisId.fIdentifier = "t5cof";
  y->t5cof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "t5cof")), &thisId);
  thisId.fIdentifier = "x1mth2";
  y->x1mth2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "x1mth2")), &thisId);
  thisId.fIdentifier = "x7thm1";
  y->x7thm1 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "x7thm1")), &thisId);
  thisId.fIdentifier = "mdot";
  y->mdot = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "mdot")), &thisId);
  thisId.fIdentifier = "nodedot";
  y->nodedot = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "nodedot")), &thisId);
  thisId.fIdentifier = "xlcof";
  y->xlcof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "xlcof")), &thisId);
  thisId.fIdentifier = "xmcof";
  y->xmcof = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "xmcof")), &thisId);
  thisId.fIdentifier = "nodecf";
  y->nodecf = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "nodecf")), &thisId);
  thisId.fIdentifier = "init";
  y->init = g_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "init")), &thisId);
  thisId.fIdentifier = "gsto";
  y->gsto = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "gsto")), &thisId);
  emlrtDestroyArray(&u);
}

static char_T g_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  char_T y;
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static b_struct_T i_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId)
{
  b_struct_T y;
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[8] = { "mu", "radiusearthkm", "xke", "tumin",
    "j2", "j3", "j4", "j3oj2" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 8, fieldNames, 0U, 0);
  thisId.fIdentifier = "mu";
  y.mu = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "mu")), &thisId);
  thisId.fIdentifier = "radiusearthkm";
  y.radiusearthkm = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a
    (emlrtRootTLSGlobal, u, 0, "radiusearthkm")), &thisId);
  thisId.fIdentifier = "xke";
  y.xke = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "xke")), &thisId);
  thisId.fIdentifier = "tumin";
  y.tumin = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "tumin")), &thisId);
  thisId.fIdentifier = "j2";
  y.j2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "j2")), &thisId);
  thisId.fIdentifier = "j3";
  y.j3 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "j3")), &thisId);
  thisId.fIdentifier = "j4";
  y.j4 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal, u,
    0, "j4")), &thisId);
  thisId.fIdentifier = "j3oj2";
  y.j3oj2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2013a(emlrtRootTLSGlobal,
    u, 0, "j3oj2")), &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T j_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", FALSE, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static char_T l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId)
{
  char_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "char", FALSE, 0U, 0);
  emlrtImportChar(src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

real_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = j_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

const mxArray *b_emlrt_marshallOut(const struct_T *u)
{
  const mxArray *y;
  const mxArray *b_y;
  const mxArray *m2;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *q_y;
  const mxArray *r_y;
  const mxArray *s_y;
  const mxArray *t_y;
  const mxArray *u_y;
  const mxArray *v_y;
  const mxArray *w_y;
  const mxArray *x_y;
  const mxArray *y_y;
  const mxArray *ab_y;
  const mxArray *bb_y;
  const mxArray *cb_y;
  const mxArray *db_y;
  const mxArray *eb_y;
  const mxArray *fb_y;
  const mxArray *gb_y;
  const mxArray *hb_y;
  const mxArray *ib_y;
  const mxArray *jb_y;
  const mxArray *kb_y;
  const mxArray *lb_y;
  const mxArray *mb_y;
  const mxArray *nb_y;
  const mxArray *ob_y;
  const mxArray *pb_y;
  const mxArray *qb_y;
  const mxArray *rb_y;
  const mxArray *sb_y;
  const mxArray *tb_y;
  const mxArray *ub_y;
  const mxArray *vb_y;
  y = NULL;
  emlrtAssign(&y, mxCreateStructMatrix(1, 1, 0, NULL));
  b_y = NULL;
  m2 = mxCreateDoubleScalar(u->error);
  emlrtAssign(&b_y, m2);
  emlrtAddField(y, b_y, "error", 0);
  c_y = NULL;
  m2 = mxCreateDoubleScalar(u->satnum);
  emlrtAssign(&c_y, m2);
  emlrtAddField(y, c_y, "satnum", 0);
  d_y = NULL;
  m2 = mxCreateDoubleScalar(u->epochyr);
  emlrtAssign(&d_y, m2);
  emlrtAddField(y, d_y, "epochyr", 0);
  e_y = NULL;
  m2 = mxCreateDoubleScalar(u->epochdays);
  emlrtAssign(&e_y, m2);
  emlrtAddField(y, e_y, "epochdays", 0);
  f_y = NULL;
  m2 = mxCreateDoubleScalar(u->ndot);
  emlrtAssign(&f_y, m2);
  emlrtAddField(y, f_y, "ndot", 0);
  g_y = NULL;
  m2 = mxCreateDoubleScalar(u->nddot);
  emlrtAssign(&g_y, m2);
  emlrtAddField(y, g_y, "nddot", 0);
  h_y = NULL;
  m2 = mxCreateDoubleScalar(u->bstar);
  emlrtAssign(&h_y, m2);
  emlrtAddField(y, h_y, "bstar", 0);
  i_y = NULL;
  m2 = mxCreateDoubleScalar(u->inclo);
  emlrtAssign(&i_y, m2);
  emlrtAddField(y, i_y, "inclo", 0);
  j_y = NULL;
  m2 = mxCreateDoubleScalar(u->nodeo);
  emlrtAssign(&j_y, m2);
  emlrtAddField(y, j_y, "nodeo", 0);
  k_y = NULL;
  m2 = mxCreateDoubleScalar(u->ecco);
  emlrtAssign(&k_y, m2);
  emlrtAddField(y, k_y, "ecco", 0);
  l_y = NULL;
  m2 = mxCreateDoubleScalar(u->argpo);
  emlrtAssign(&l_y, m2);
  emlrtAddField(y, l_y, "argpo", 0);
  m_y = NULL;
  m2 = mxCreateDoubleScalar(u->mo);
  emlrtAssign(&m_y, m2);
  emlrtAddField(y, m_y, "mo", 0);
  n_y = NULL;
  m2 = mxCreateDoubleScalar(u->no);
  emlrtAssign(&n_y, m2);
  emlrtAddField(y, n_y, "no", 0);
  o_y = NULL;
  m2 = mxCreateDoubleScalar(u->a);
  emlrtAssign(&o_y, m2);
  emlrtAddField(y, o_y, "a", 0);
  p_y = NULL;
  m2 = mxCreateDoubleScalar(u->alta);
  emlrtAssign(&p_y, m2);
  emlrtAddField(y, p_y, "alta", 0);
  q_y = NULL;
  m2 = mxCreateDoubleScalar(u->altp);
  emlrtAssign(&q_y, m2);
  emlrtAddField(y, q_y, "altp", 0);
  r_y = NULL;
  m2 = mxCreateDoubleScalar(u->jdsatepoch);
  emlrtAssign(&r_y, m2);
  emlrtAddField(y, r_y, "jdsatepoch", 0);
  s_y = NULL;
  m2 = mxCreateDoubleScalar(u->isimp);
  emlrtAssign(&s_y, m2);
  emlrtAddField(y, s_y, "isimp", 0);
  t_y = NULL;
  m2 = emlrtCreateString1(u->method);
  emlrtAssign(&t_y, m2);
  emlrtAddField(y, t_y, "method", 0);
  u_y = NULL;
  m2 = mxCreateDoubleScalar(u->aycof);
  emlrtAssign(&u_y, m2);
  emlrtAddField(y, u_y, "aycof", 0);
  v_y = NULL;
  m2 = mxCreateDoubleScalar(u->con41);
  emlrtAssign(&v_y, m2);
  emlrtAddField(y, v_y, "con41", 0);
  w_y = NULL;
  m2 = mxCreateDoubleScalar(u->cc1);
  emlrtAssign(&w_y, m2);
  emlrtAddField(y, w_y, "cc1", 0);
  x_y = NULL;
  m2 = mxCreateDoubleScalar(u->cc4);
  emlrtAssign(&x_y, m2);
  emlrtAddField(y, x_y, "cc4", 0);
  y_y = NULL;
  m2 = mxCreateDoubleScalar(u->cc5);
  emlrtAssign(&y_y, m2);
  emlrtAddField(y, y_y, "cc5", 0);
  ab_y = NULL;
  m2 = mxCreateDoubleScalar(u->d2);
  emlrtAssign(&ab_y, m2);
  emlrtAddField(y, ab_y, "d2", 0);
  bb_y = NULL;
  m2 = mxCreateDoubleScalar(u->d3);
  emlrtAssign(&bb_y, m2);
  emlrtAddField(y, bb_y, "d3", 0);
  cb_y = NULL;
  m2 = mxCreateDoubleScalar(u->d4);
  emlrtAssign(&cb_y, m2);
  emlrtAddField(y, cb_y, "d4", 0);
  db_y = NULL;
  m2 = mxCreateDoubleScalar(u->delmo);
  emlrtAssign(&db_y, m2);
  emlrtAddField(y, db_y, "delmo", 0);
  eb_y = NULL;
  m2 = mxCreateDoubleScalar(u->eta);
  emlrtAssign(&eb_y, m2);
  emlrtAddField(y, eb_y, "eta", 0);
  fb_y = NULL;
  m2 = mxCreateDoubleScalar(u->argpdot);
  emlrtAssign(&fb_y, m2);
  emlrtAddField(y, fb_y, "argpdot", 0);
  gb_y = NULL;
  m2 = mxCreateDoubleScalar(u->omgcof);
  emlrtAssign(&gb_y, m2);
  emlrtAddField(y, gb_y, "omgcof", 0);
  hb_y = NULL;
  m2 = mxCreateDoubleScalar(u->sinmao);
  emlrtAssign(&hb_y, m2);
  emlrtAddField(y, hb_y, "sinmao", 0);
  ib_y = NULL;
  m2 = mxCreateDoubleScalar(u->t);
  emlrtAssign(&ib_y, m2);
  emlrtAddField(y, ib_y, "t", 0);
  jb_y = NULL;
  m2 = mxCreateDoubleScalar(u->t2cof);
  emlrtAssign(&jb_y, m2);
  emlrtAddField(y, jb_y, "t2cof", 0);
  kb_y = NULL;
  m2 = mxCreateDoubleScalar(u->t3cof);
  emlrtAssign(&kb_y, m2);
  emlrtAddField(y, kb_y, "t3cof", 0);
  lb_y = NULL;
  m2 = mxCreateDoubleScalar(u->t4cof);
  emlrtAssign(&lb_y, m2);
  emlrtAddField(y, lb_y, "t4cof", 0);
  mb_y = NULL;
  m2 = mxCreateDoubleScalar(u->t5cof);
  emlrtAssign(&mb_y, m2);
  emlrtAddField(y, mb_y, "t5cof", 0);
  nb_y = NULL;
  m2 = mxCreateDoubleScalar(u->x1mth2);
  emlrtAssign(&nb_y, m2);
  emlrtAddField(y, nb_y, "x1mth2", 0);
  ob_y = NULL;
  m2 = mxCreateDoubleScalar(u->x7thm1);
  emlrtAssign(&ob_y, m2);
  emlrtAddField(y, ob_y, "x7thm1", 0);
  pb_y = NULL;
  m2 = mxCreateDoubleScalar(u->mdot);
  emlrtAssign(&pb_y, m2);
  emlrtAddField(y, pb_y, "mdot", 0);
  qb_y = NULL;
  m2 = mxCreateDoubleScalar(u->nodedot);
  emlrtAssign(&qb_y, m2);
  emlrtAddField(y, qb_y, "nodedot", 0);
  rb_y = NULL;
  m2 = mxCreateDoubleScalar(u->xlcof);
  emlrtAssign(&rb_y, m2);
  emlrtAddField(y, rb_y, "xlcof", 0);
  sb_y = NULL;
  m2 = mxCreateDoubleScalar(u->xmcof);
  emlrtAssign(&sb_y, m2);
  emlrtAddField(y, sb_y, "xmcof", 0);
  tb_y = NULL;
  m2 = mxCreateDoubleScalar(u->nodecf);
  emlrtAssign(&tb_y, m2);
  emlrtAddField(y, tb_y, "nodecf", 0);
  ub_y = NULL;
  m2 = emlrtCreateString1(u->init);
  emlrtAssign(&ub_y, m2);
  emlrtAddField(y, ub_y, "init", 0);
  vb_y = NULL;
  m2 = mxCreateDoubleScalar(u->gsto);
  emlrtAssign(&vb_y, m2);
  emlrtAddField(y, vb_y, "gsto", 0);
  return y;
}

const mxArray *c_emlrt_marshallOut(const b_struct_T u)
{
  const mxArray *y;
  const mxArray *b_y;
  const mxArray *m3;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  y = NULL;
  emlrtAssign(&y, mxCreateStructMatrix(1, 1, 0, NULL));
  b_y = NULL;
  m3 = mxCreateDoubleScalar(u.mu);
  emlrtAssign(&b_y, m3);
  emlrtAddField(y, b_y, "mu", 0);
  c_y = NULL;
  m3 = mxCreateDoubleScalar(u.radiusearthkm);
  emlrtAssign(&c_y, m3);
  emlrtAddField(y, c_y, "radiusearthkm", 0);
  d_y = NULL;
  m3 = mxCreateDoubleScalar(u.xke);
  emlrtAssign(&d_y, m3);
  emlrtAddField(y, d_y, "xke", 0);
  e_y = NULL;
  m3 = mxCreateDoubleScalar(u.tumin);
  emlrtAssign(&e_y, m3);
  emlrtAddField(y, e_y, "tumin", 0);
  f_y = NULL;
  m3 = mxCreateDoubleScalar(u.j2);
  emlrtAssign(&f_y, m3);
  emlrtAddField(y, f_y, "j2", 0);
  g_y = NULL;
  m3 = mxCreateDoubleScalar(u.j3);
  emlrtAssign(&g_y, m3);
  emlrtAddField(y, g_y, "j3", 0);
  h_y = NULL;
  m3 = mxCreateDoubleScalar(u.j4);
  emlrtAssign(&h_y, m3);
  emlrtAddField(y, h_y, "j4", 0);
  i_y = NULL;
  m3 = mxCreateDoubleScalar(u.j3oj2);
  emlrtAssign(&i_y, m3);
  emlrtAddField(y, i_y, "j3oj2", 0);
  return y;
}

void e_emlrt_marshallIn(const mxArray *c_satrec, const char_T *identifier,
  struct_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  f_emlrt_marshallIn(emlrtAlias(c_satrec), &thisId, y);
  emlrtDestroyArray(&c_satrec);
}

void emlrt_checkEscapedGlobals(void)
{
  satrec_dirty |= satrec_dirty >> 1U;
  gravc_dirty |= gravc_dirty >> 1U;
}

void emlrt_synchGlobalsFromML(void)
{
  e_emlrt_marshallIn(mexGetVariable("global", "satrec"), "satrec", &satrec);
  gravc = h_emlrt_marshallIn(mexGetVariable("global", "gravc"), "gravc");
}

void emlrt_synchGlobalsToML(void)
{
  if (satrec_dirty & 1U) {
    mexPutVariable("global", "satrec", b_emlrt_marshallOut(&satrec));
    satrec_dirty &= 2U;
  }

  if (gravc_dirty & 1U) {
    mexPutVariable("global", "gravc", c_emlrt_marshallOut(gravc));
    gravc_dirty &= 2U;
  }
}

b_struct_T h_emlrt_marshallIn(const mxArray *c_gravc, const char_T *identifier)
{
  b_struct_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = i_emlrt_marshallIn(emlrtAlias(c_gravc), &thisId);
  emlrtDestroyArray(&c_gravc);
  return y;
}

/* End of code generation (SGP4_Setup_mexutil.c) */
