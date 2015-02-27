/*
 * UKF_quaternion_withBias.c
 *
 * Code generation for function 'UKF_quaternion_withBias'
 *
 * C source code generated on: Thu Aug 15 19:39:27 2013
 *
 */

/* Include files */
#include "UKF_quaternion_withBias.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static real_T P_km1km1[81];
static real_T x_km1km1[10];
static boolean_T x_km1km1_not_empty;
static real_T R_k[36];
static real_T Q_k[81];
static real_T Lambda;
static real_T alpha;
static real_T Beta;
static real_T q_s_c[4];
static real_T I_c[3];

/* Function Declarations */
static void Kinematics(const real_T x[10], const real_T I[3], const real_T
  torque_c[3], real_T results[10]);
static void RK4(const real_T X_km1km1[190], const real_T b_I_c[3], const real_T
                Torque_c[3], real_T Ts, real_T X_kkm1[190]);
static void RotateVecCont2Sensor(const real_T vec[3], const real_T b_q_s_c[4],
  real_T rvec[3]);
static void RotateVecSensor2Cont(const real_T vec[3], const real_T b_q_s_c[4],
  real_T rvec[3]);
static void SensorModel(const real_T X_kkm1[190], const real_T B_ref[3], real_T
  Z_kkm1[114]);
static void b_power(const real_T a[4], real_T y[4]);
static void b_quatmultiply(const real_T q[76], const real_T r[4], real_T qres[76]);
static void b_rdivide(const real_T x[4], real_T y, real_T z[4]);
static void b_sqrt(real_T x[19]);
static real_T b_sum(const real_T x[4]);
static void c_power(const real_T a[57], real_T y[57]);
static void c_sum(const real_T x[57], real_T y[19]);
static void chol(real_T A[81]);
static void d_sum(const real_T x[190], real_T y[10]);
static void e_sum(const real_T x[171], real_T y[9]);
static void f_sum(const real_T x[114], real_T y[6]);
static void inv(const real_T x[9], real_T y[9]);
static void mrdivide(const real_T A[54], const real_T B[36], real_T y[54]);
static void power(const real_T a[3], real_T y[3]);
static void quatinv(const real_T qin[4], real_T qinv[4]);
static void quatmultiply(const real_T q[4], const real_T r[4], real_T qres[4]);
static void rdivide(const real_T x[3], real_T y, real_T z[3]);
static real_T sum(const real_T x[3]);

/* Function Definitions */
static void Kinematics(const real_T x[10], const real_T I[3], const real_T
  torque_c[3], real_T results[10])
{
  real_T b_I[9];
  int32_T j;
  real_T a[9];
  real_T c[9];
  real_T b_c[9];
  real_T dv14[9];
  real_T dv15[16];
  int32_T i6;
  real_T dv16[16];
  real_T dv17[9];
  real_T c_I[3];
  real_T dv18[3];
  real_T d0;
  real_T dv19[4];
  memset(&b_I[0], 0, 9U * sizeof(real_T));
  for (j = 0; j < 3; j++) {
    b_I[j + 3 * j] = I[j];
  }

  /*  Returns the skew symmetric matrix of the input vector */
  memcpy(&a[0], &b_I[0], 9U * sizeof(real_T));
  memcpy(&c[0], &a[0], 9U * sizeof(real_T));
  memcpy(&b_c[0], &c[0], 9U * sizeof(real_T));
  inv(b_c, c);

  /*  Returns the skew symmetric matrix of the input vector */
  dv14[0] = 0.0;
  dv14[3] = -x[6];
  dv14[6] = x[5];
  dv14[1] = x[6];
  dv14[4] = 0.0;
  dv14[7] = -x[4];
  dv14[2] = -x[5];
  dv14[5] = x[4];
  dv14[8] = 0.0;
  dv15[0] = 0.0;
  for (j = 0; j < 3; j++) {
    dv15[(j + 1) << 2] = -x[j + 4];
  }

  for (j = 0; j < 3; j++) {
    dv15[j + 1] = x[j + 4];
  }

  for (j = 0; j < 3; j++) {
    for (i6 = 0; i6 < 3; i6++) {
      dv15[(i6 + ((j + 1) << 2)) + 1] = -dv14[i6 + 3 * j];
    }
  }

  for (j = 0; j < 4; j++) {
    for (i6 = 0; i6 < 4; i6++) {
      dv16[i6 + (j << 2)] = 0.5 * dv15[i6 + (j << 2)];
    }
  }

  dv17[0] = 0.0;
  dv17[3] = -x[6];
  dv17[6] = x[5];
  dv17[1] = x[6];
  dv17[4] = 0.0;
  dv17[7] = -x[4];
  dv17[2] = -x[5];
  dv17[5] = x[4];
  dv17[8] = 0.0;
  for (j = 0; j < 3; j++) {
    for (i6 = 0; i6 < 3; i6++) {
      dv14[i6 + 3 * j] = -dv17[i6 + 3 * j];
    }

    c_I[j] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      c_I[j] += b_I[j + 3 * i6] * x[i6 + 4];
    }
  }

  for (j = 0; j < 3; j++) {
    d0 = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      d0 += dv14[j + 3 * i6] * c_I[i6];
    }

    dv18[j] = d0 + torque_c[j];
  }

  for (j = 0; j < 4; j++) {
    dv19[j] = 0.0;
    for (i6 = 0; i6 < 4; i6++) {
      dv19[j] += dv16[j + (i6 << 2)] * x[i6];
    }
  }

  for (j = 0; j < 3; j++) {
    c_I[j] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      c_I[j] += c[j + 3 * i6] * dv18[i6];
    }
  }

  for (j = 0; j < 4; j++) {
    results[j] = dv19[j];
  }

  for (j = 0; j < 3; j++) {
    results[j + 4] = c_I[j];
  }

  for (j = 0; j < 3; j++) {
    results[j + 7] = 0.0;
  }
}

static void RK4(const real_T X_km1km1[190], const real_T b_I_c[3], const real_T
                Torque_c[3], real_T Ts, real_T X_kkm1[190])
{
  int32_T idx;
  real_T k1[10];
  real_T b_X_km1km1[10];
  int32_T i5;
  real_T k2[10];
  real_T k3[10];
  real_T k4[10];

  /*  This function performs a fourth-order Runge-Kutta integration on the */
  /*  kinematic equation for the satellite to propogate the sigma points */
  /*  through it's current dynamic behavior to develop a prediction state */
  /*  */
  /*  X_km1km1 is the quaternion sigma points */
  /*  w is the angular velocities */
  /*  Ts is the descrete sample time */
  for (idx = 0; idx < 19; idx++) {
    Kinematics(*(real_T (*)[10])&X_km1km1[10 * idx], b_I_c, Torque_c, k1);
    for (i5 = 0; i5 < 10; i5++) {
      b_X_km1km1[i5] = X_km1km1[i5 + 10 * idx] + 0.5 * k1[i5] * Ts;
    }

    Kinematics(b_X_km1km1, b_I_c, Torque_c, k2);
    for (i5 = 0; i5 < 10; i5++) {
      b_X_km1km1[i5] = X_km1km1[i5 + 10 * idx] + 0.5 * k2[i5] * Ts;
    }

    Kinematics(b_X_km1km1, b_I_c, Torque_c, k3);
    for (i5 = 0; i5 < 10; i5++) {
      b_X_km1km1[i5] = X_km1km1[i5 + 10 * idx] + k3[i5] * Ts;
    }

    Kinematics(b_X_km1km1, b_I_c, Torque_c, k4);
    for (i5 = 0; i5 < 10; i5++) {
      X_kkm1[i5 + 10 * idx] = X_km1km1[i5 + 10 * idx] + (((k1[i5] + 2.0 * k2[i5])
        + 2.0 * k3[i5]) + k4[i5]) * Ts / 6.0;
    }
  }

  /*  End of for */
}

static void RotateVecCont2Sensor(const real_T vec[3], const real_T b_q_s_c[4],
  real_T rvec[3])
{
  int32_T i9;
  real_T r[4];
  real_T c_q_s_c[3];
  real_T d_q_s_c[3];
  real_T qres[4];
  for (i9 = 0; i9 < 3; i9++) {
    r[i9 + 1] = vec[i9];
  }

  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  c_q_s_c[0] = b_q_s_c[0] * r[1];
  c_q_s_c[1] = b_q_s_c[0] * r[2];
  c_q_s_c[2] = b_q_s_c[0] * r[3];
  d_q_s_c[0] = b_q_s_c[2] * r[3] - b_q_s_c[3] * r[2];
  d_q_s_c[1] = b_q_s_c[3] * r[1] - b_q_s_c[1] * r[3];
  d_q_s_c[2] = b_q_s_c[1] * r[2] - b_q_s_c[2] * r[1];
  r[0] = ((0.0 - b_q_s_c[1] * r[1]) - b_q_s_c[2] * r[2]) - b_q_s_c[3] * r[3];
  for (i9 = 0; i9 < 3; i9++) {
    r[i9 + 1] = c_q_s_c[i9] + d_q_s_c[i9];
  }

  for (i9 = 0; i9 < 4; i9++) {
    qres[i9] = r[i9];
  }

  quatinv(b_q_s_c, r);

  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  rvec[0] = (qres[0] * r[1] + r[0] * qres[1]) + (qres[2] * r[3] - qres[3] * r[2]);
  rvec[1] = (qres[0] * r[2] + r[0] * qres[2]) + (qres[3] * r[1] - qres[1] * r[3]);
  rvec[2] = (qres[0] * r[3] + r[0] * qres[3]) + (qres[1] * r[2] - qres[2] * r[1]);
}

static void RotateVecSensor2Cont(const real_T vec[3], const real_T b_q_s_c[4],
  real_T rvec[3])
{
  real_T q[4];
  int32_T i2;
  real_T r[4];
  real_T b_q[3];
  real_T c_q[3];
  quatinv(b_q_s_c, q);
  for (i2 = 0; i2 < 3; i2++) {
    r[i2 + 1] = vec[i2];
  }

  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  b_q[0] = q[0] * r[1];
  b_q[1] = q[0] * r[2];
  b_q[2] = q[0] * r[3];
  c_q[0] = q[2] * r[3] - q[3] * r[2];
  c_q[1] = q[3] * r[1] - q[1] * r[3];
  c_q[2] = q[1] * r[2] - q[2] * r[1];
  q[0] = ((0.0 - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  for (i2 = 0; i2 < 3; i2++) {
    q[i2 + 1] = b_q[i2] + c_q[i2];
  }

  for (i2 = 0; i2 < 4; i2++) {
    r[i2] = q[i2];
  }

  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  rvec[0] = (r[0] * b_q_s_c[1] + b_q_s_c[0] * r[1]) + (r[2] * b_q_s_c[3] - r[3] *
    b_q_s_c[2]);
  rvec[1] = (r[0] * b_q_s_c[2] + b_q_s_c[0] * r[2]) + (r[3] * b_q_s_c[1] - r[1] *
    b_q_s_c[3]);
  rvec[2] = (r[0] * b_q_s_c[3] + b_q_s_c[0] * r[3]) + (r[1] * b_q_s_c[2] - r[2] *
    b_q_s_c[1]);
}

static void SensorModel(const real_T X_kkm1[190], const real_T B_ref[3], real_T
  Z_kkm1[114])
{
  int32_T idx;
  real_T q[4];
  int32_T i7;
  real_T r[4];
  real_T b_q[3];
  real_T c_q[3];
  real_T qres[4];
  real_T b_qres[3];
  real_T b_X_kkm1[3];
  real_T c_qres[3];

  /*  This function propagates the sigma points through the sensor model to */
  /*  obtain the transformed sigma points (Predicted Measurements). */
  /*  */
  /*  x_kkm1 is the propagated sigma points */
  /*  B_ref is the reference frame magnetic field vector */
  memset(&Z_kkm1[0], 0, 114U * sizeof(real_T));
  for (idx = 0; idx < 19; idx++) {
    /*  Magnetic Field Prediction */
    quatinv(*(real_T (*)[4])&X_kkm1[10 * idx], q);
    for (i7 = 0; i7 < 3; i7++) {
      r[i7 + 1] = B_ref[i7];
    }

    /*  Calculate vector portion of quaternion product */
    /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
    /*  Calculate scalar portion of quaternion product */
    /*  scalar = s1*s2 - dot(v1,v2) */
    b_q[0] = q[0] * r[1];
    b_q[1] = q[0] * r[2];
    b_q[2] = q[0] * r[3];
    c_q[0] = q[2] * r[3] - q[3] * r[2];
    c_q[1] = q[3] * r[1] - q[1] * r[3];
    c_q[2] = q[1] * r[2] - q[2] * r[1];
    q[0] = ((0.0 - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
    for (i7 = 0; i7 < 3; i7++) {
      q[i7 + 1] = b_q[i7] + c_q[i7];
    }

    for (i7 = 0; i7 < 4; i7++) {
      qres[i7] = q[i7];
    }

    /*  Calculate vector portion of quaternion product */
    /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
    /*  Calculate scalar portion of quaternion product */
    /*  scalar = s1*s2 - dot(v1,v2) */
    b_qres[0] = qres[0] * X_kkm1[1 + 10 * idx];
    b_qres[1] = qres[0] * X_kkm1[2 + 10 * idx];
    b_qres[2] = qres[0] * X_kkm1[3 + 10 * idx];
    b_X_kkm1[0] = X_kkm1[10 * idx] * qres[1];
    b_X_kkm1[1] = X_kkm1[10 * idx] * qres[2];
    b_X_kkm1[2] = X_kkm1[10 * idx] * qres[3];
    c_qres[0] = qres[2] * X_kkm1[3 + 10 * idx] - qres[3] * X_kkm1[2 + 10 * idx];
    c_qres[1] = qres[3] * X_kkm1[1 + 10 * idx] - qres[1] * X_kkm1[3 + 10 * idx];
    c_qres[2] = qres[1] * X_kkm1[2 + 10 * idx] - qres[2] * X_kkm1[1 + 10 * idx];
    qres[0] = ((qres[0] * X_kkm1[10 * idx] - qres[1] * X_kkm1[1 + 10 * idx]) -
               qres[2] * X_kkm1[2 + 10 * idx]) - qres[3] * X_kkm1[3 + 10 * idx];
    for (i7 = 0; i7 < 3; i7++) {
      qres[i7 + 1] = (b_qres[i7] + b_X_kkm1[i7]) + c_qres[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      Z_kkm1[i7 + 6 * idx] = qres[1 + i7];
    }

    /*  Angular Velocity Prediction */
    for (i7 = 0; i7 < 3; i7++) {
      Z_kkm1[(i7 + 6 * idx) + 3] = X_kkm1[(i7 + 10 * idx) + 4] + X_kkm1[(i7 + 10
        * idx) + 7];
    }
  }
}

static void b_power(const real_T a[4], real_T y[4])
{
  int32_T k;
  real_T u0;
  for (k = 0; k < 4; k++) {
    u0 = a[k];
    u0 = pow(u0, 2.0);
    y[k] = u0;
  }
}

static void b_quatmultiply(const real_T q[76], const real_T r[4], real_T qres[76])
{
  real_T b_q[76];
  int32_T i3;
  int32_T i4;
  real_T c_q[57];
  real_T b_r[57];
  real_T d_q[57];
  for (i3 = 0; i3 < 4; i3++) {
    for (i4 = 0; i4 < 19; i4++) {
      b_q[i4 + 19 * i3] = q[i3 + (i4 << 2)];
    }
  }

  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  for (i3 = 0; i3 < 19; i3++) {
    c_q[i3] = b_q[i3] * r[1];
    c_q[19 + i3] = b_q[i3] * r[2];
    c_q[38 + i3] = b_q[i3] * r[3];
    b_r[i3] = r[0] * b_q[19 + i3];
    b_r[19 + i3] = r[0] * b_q[38 + i3];
    b_r[38 + i3] = r[0] * b_q[57 + i3];
    d_q[i3] = b_q[38 + i3] * r[3] - b_q[57 + i3] * r[2];
    d_q[19 + i3] = b_q[57 + i3] * r[1] - b_q[19 + i3] * r[3];
    d_q[38 + i3] = b_q[19 + i3] * r[2] - b_q[38 + i3] * r[1];
    qres[i3 << 2] = ((b_q[i3] * r[0] - b_q[19 + i3] * r[1]) - b_q[38 + i3] * r[2])
      - b_q[57 + i3] * r[3];
    for (i4 = 0; i4 < 3; i4++) {
      qres[(i4 + (i3 << 2)) + 1] = (c_q[i3 + 19 * i4] + b_r[i3 + 19 * i4]) +
        d_q[i3 + 19 * i4];
    }
  }
}

static void b_rdivide(const real_T x[4], real_T y, real_T z[4])
{
  int32_T i;
  for (i = 0; i < 4; i++) {
    z[i] = x[i] / y;
  }
}

static void b_sqrt(real_T x[19])
{
  int32_T k;
  for (k = 0; k < 19; k++) {
    x[k] = sqrt(x[k]);
  }
}

static real_T b_sum(const real_T x[4])
{
  real_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 3; k++) {
    y += x[k + 1];
  }

  return y;
}

static void c_power(const real_T a[57], real_T y[57])
{
  int32_T k;
  real_T u0;
  for (k = 0; k < 57; k++) {
    u0 = a[k];
    u0 = pow(u0, 2.0);
    y[k] = u0;
  }
}

static void c_sum(const real_T x[57], real_T y[19])
{
  int32_T ix;
  int32_T iy;
  int32_T i;
  int32_T ixstart;
  real_T s;
  ix = -1;
  iy = -1;
  for (i = 0; i < 19; i++) {
    ixstart = ix + 1;
    ix++;
    s = x[ixstart];
    for (ixstart = 0; ixstart < 2; ixstart++) {
      ix++;
      s += x[ix];
    }

    iy++;
    y[iy] = s;
  }
}

static void chol(real_T A[81])
{
  int32_T info;
  int32_T colj;
  int32_T j;
  boolean_T exitg1;
  int32_T jj;
  real_T ajj;
  int32_T ix;
  int32_T iy;
  int32_T jmax;
  int32_T i10;
  real_T c;
  int32_T i;
  int32_T ia;
  info = -1;
  colj = 0;
  j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (j + 1 < 10)) {
    jj = colj + j;
    ajj = 0.0;
    if (j < 1) {
    } else {
      ix = colj;
      iy = colj;
      for (jmax = 1; jmax <= j; jmax++) {
        ajj += A[ix] * A[iy];
        ix++;
        iy++;
      }
    }

    ajj = A[jj] - ajj;
    if (ajj > 0.0) {
      ajj = sqrt(ajj);
      A[jj] = ajj;
      if (j + 1 < 9) {
        if (j == 0) {
        } else {
          iy = jj + 9;
          i10 = (colj + 9 * (7 - j)) + 10;
          for (jmax = colj + 10; jmax <= i10; jmax += 9) {
            ix = colj;
            c = 0.0;
            i = (jmax + j) - 1;
            for (ia = jmax; ia <= i; ia++) {
              c += A[ia - 1] * A[ix];
              ix++;
            }

            A[iy] += -c;
            iy += 9;
          }
        }

        ajj = 1.0 / ajj;
        i10 = (jj + 9 * (7 - j)) + 10;
        for (jmax = jj + 9; jmax + 1 <= i10; jmax += 9) {
          A[jmax] *= ajj;
        }

        colj += 9;
      }

      j++;
    } else {
      A[jj] = ajj;
      info = j;
      exitg1 = TRUE;
    }
  }

  if (info + 1 == 0) {
    jmax = 9;
  } else {
    jmax = info;
  }

  for (j = 0; j + 1 <= jmax; j++) {
    for (i = j + 1; i + 1 <= jmax; i++) {
      A[i + 9 * j] = 0.0;
    }
  }
}

static void d_sum(const real_T x[190], real_T y[10])
{
  int32_T iy;
  int32_T ixstart;
  int32_T j;
  int32_T ix;
  real_T s;
  int32_T k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 10; j++) {
    ixstart++;
    ix = ixstart;
    s = x[ixstart];
    for (k = 0; k < 18; k++) {
      ix += 10;
      s += x[ix];
    }

    iy++;
    y[iy] = s;
  }
}

static void e_sum(const real_T x[171], real_T y[9])
{
  int32_T iy;
  int32_T ixstart;
  int32_T j;
  int32_T ix;
  real_T s;
  int32_T k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 9; j++) {
    ixstart++;
    ix = ixstart;
    s = x[ixstart];
    for (k = 0; k < 18; k++) {
      ix += 9;
      s += x[ix];
    }

    iy++;
    y[iy] = s;
  }
}

static void f_sum(const real_T x[114], real_T y[6])
{
  int32_T iy;
  int32_T ixstart;
  int32_T j;
  int32_T ix;
  real_T s;
  int32_T k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 6; j++) {
    ixstart++;
    ix = ixstart;
    s = x[ixstart];
    for (k = 0; k < 18; k++) {
      ix += 6;
      s += x[ix];
    }

    iy++;
    y[iy] = s;
  }
}

static void inv(const real_T x[9], real_T y[9])
{
  real_T b_x[9];
  int32_T p1;
  int32_T p2;
  int32_T p3;
  real_T absx11;
  real_T absx21;
  real_T absx31;
  int32_T itmp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(x[0]);
  absx21 = fabs(x[1]);
  absx31 = fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if (fabs(b_x[5]) > fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

static void mrdivide(const real_T A[54], const real_T B[36], real_T y[54])
{
  real_T b_A[36];
  int8_T ipiv[6];
  int32_T i8;
  int32_T iy;
  int32_T j;
  int32_T c;
  int32_T ix;
  real_T temp;
  int32_T k;
  real_T s;
  int32_T jy;
  int32_T ijA;
  real_T Y[54];
  for (i8 = 0; i8 < 6; i8++) {
    for (iy = 0; iy < 6; iy++) {
      b_A[iy + 6 * i8] = B[i8 + 6 * iy];
    }

    ipiv[i8] = (int8_T)(1 + i8);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 6;
          iy += 6;
        }
      }

      i8 = (c - j) + 6;
      for (jy = c + 1; jy + 1 <= i8; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (k = 1; k <= 5 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i8 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i8; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i8 = 0; i8 < 9; i8++) {
    for (iy = 0; iy < 6; iy++) {
      Y[iy + 6 * i8] = A[i8 + 9 * iy];
    }
  }

  for (jy = 0; jy < 6; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 9; j++) {
        temp = Y[jy + 6 * j];
        Y[jy + 6 * j] = Y[(ipiv[jy] + 6 * j) - 1];
        Y[(ipiv[jy] + 6 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 9; j++) {
    c = 6 * j;
    for (k = 0; k < 6; k++) {
      iy = 6 * k;
      if (Y[k + c] != 0.0) {
        for (jy = k + 2; jy < 7; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 9; j++) {
    c = 6 * j;
    for (k = 5; k > -1; k += -1) {
      iy = 6 * k;
      if (Y[k + c] != 0.0) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i8 = 0; i8 < 6; i8++) {
    for (iy = 0; iy < 9; iy++) {
      y[iy + 9 * i8] = Y[i8 + 6 * iy];
    }
  }
}

static void power(const real_T a[3], real_T y[3])
{
  int32_T k;
  real_T u0;
  for (k = 0; k < 3; k++) {
    u0 = a[k];
    u0 = pow(u0, 2.0);
    y[k] = u0;
  }
}

static void quatinv(const real_T qin[4], real_T qinv[4])
{
  real_T y[4];
  int32_T i;
  real_T b_y;
  for (i = 0; i < 4; i++) {
    b_y = qin[i];
    b_y = pow(b_y, 2.0);
    y[i] = b_y;
  }

  b_y = y[0];
  for (i = 0; i < 3; i++) {
    b_y += y[i + 1];
  }

  b_y = sqrt(b_y);
  qinv[0] = qin[0] / b_y;
  for (i = 0; i < 3; i++) {
    qinv[i + 1] = -qin[i + 1] / b_y;
  }
}

static void quatmultiply(const real_T q[4], const real_T r[4], real_T qres[4])
{
  /*  Calculate vector portion of quaternion product */
  /*  vec = s1*v2 + s2*v1 + cross(v1,v2) */
  /*  Calculate scalar portion of quaternion product */
  /*  scalar = s1*s2 - dot(v1,v2) */
  qres[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qres[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - q[3] * r[2]);
  qres[2] = (q[0] * r[2] + r[0] * q[2]) + (q[3] * r[1] - q[1] * r[3]);
  qres[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - q[2] * r[1]);
}

static void rdivide(const real_T x[3], real_T y, real_T z[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    z[i] = x[i] / y;
  }
}

static real_T sum(const real_T x[3])
{
  real_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  return y;
}

void UKF_quaternion_withBias(real_T B_ref[3], real_T B_sat[3], const real_T
  w_gyro[3], const real_T Torque_s[3], real_T Ts, real_T Attitude_sensor[4],
  real_T w_sensor[3], real_T w_bias_sensor[3])
{
  real_T b_B_sat[3];
  real_T w_control[3];
  int32_T i;
  static const int8_T iv0[4] = { 1, 0, 0, 0 };

  int32_T i0;
  static const real_T dv0[3] = { 3.4, 3.4, 1.9 };

  static const real_T dv1[81] = { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001 };

  real_T W0_c;
  static const real_T dv2[36] = { 5.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2E-5 };

  static const real_T dv3[81] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0E-6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8 };

  real_T DX_sigma[81];
  real_T dX_km1km1[171];
  int32_T i1;
  real_T b_dX_km1km1[57];
  real_T dv4[57];
  real_T dv5[19];
  real_T X_km1km1_temp[76];
  real_T b_X_km1km1_temp[76];
  real_T dv6[57];
  real_T X_km1km1[190];
  real_T b_X_km1km1[190];
  real_T dv7[3];
  real_T W0_m;
  real_T Wi_cm;
  real_T b_W0_m[190];
  real_T x_kkm1[10];
  real_T dv8[4];
  real_T b_x_kkm1[4];
  real_T c_X_km1km1[76];
  real_T Z_kkm1_control[114];
  real_T c_W0_m[171];
  real_T dx_kkm1[9];
  real_T b[9];
  real_T b_b[9];
  real_T dv9[3];
  real_T dv10[3];
  real_T d_W0_m[114];
  real_T z_kkm1_control[6];
  real_T P_zkzk[36];
  real_T P_xkzk[54];
  real_T Torque_c[6];
  real_T b_Torque_c[6];
  real_T b_P_xkzk[54];
  real_T c_Torque_c[3];
  real_T dv11[3];
  real_T dv12[3];
  real_T c_P_xkzk[54];
  real_T dv13[4];

  /*  UKF_quaternion */
  /*  Author: Brandon Jackson */
  /*  Contact: bajackso@mtu.edu */
  /*  Date: 22 May 2013 */
  /*  */
  /*  This function performs an Unscented Kalman Filter to determine the */
  /*  spacecraft attitude. */
  /*  */
  /*  The state vector is: [Error Quaternion (3,1); Angular_Velocity; Gyro Bias (3,1)] */
  /*  Vectors will be normalized, direction not magnitude, is what is important */
  /*  values closer to one will have less issues with roundoff. */
  for (i = 0; i < 3; i++) {
    b_B_sat[i] = B_sat[i];
    w_control[i] = B_ref[i];
  }

  /*  Declare persistent variables. These variables behave similiar to global */
  /*  variables although they are only present within this function */
  /* % Initialize filter if necessary */
  if (!x_km1km1_not_empty) {
    for (i = 0; i < 4; i++) {
      q_s_c[i] = (real_T)iv0[i];
    }

    for (i0 = 0; i0 < 3; i0++) {
      I_c[i0] = dv0[i0];
    }

    /*  Initialize the initial error covariance matrix */
    memcpy(&P_km1km1[0], &dv1[0], 81U * sizeof(real_T));

    /*  Initialize the state vector */
    memset(&x_km1km1[0], 0, 10U * sizeof(real_T));
    x_km1km1_not_empty = TRUE;
    for (i0 = 0; i0 < 4; i0++) {
      x_km1km1[i0] = (real_T)iv0[i0];
    }

    RotateVecSensor2Cont(w_gyro, q_s_c, *(real_T (*)[3])&x_km1km1[4]);
    for (i0 = 0; i0 < 3; i0++) {
      x_km1km1[7 + i0] = 0.0;
    }

    /*  Gyro Bias */
    /*  Definitions */
    /*  number of states (3 quaternions & 3 angular rates) */
    alpha = 3.0;
    alpha = sqrt(alpha);
    W0_c = pow(alpha, 2.0);
    Lambda = W0_c * 9.0 - 9.0;
    memcpy(&R_k[0], &dv2[0], 36U * sizeof(real_T));

    /*  Measurement Noise Covariance */
    memcpy(&Q_k[0], &dv3[0], 81U * sizeof(real_T));

    /*  Matrix for Process noise covariance */
    Beta = 2.0;
  } else {
    /*  already initiated, go to regular operating mode */
    /*  Allocate Returned Variable Sizes */
    /*     %% Sigma Points */
    /*  Calculate the error sigma points */
    for (i0 = 0; i0 < 81; i0++) {
      DX_sigma[i0] = (9.0 + Lambda) * P_km1km1[i0];
    }

    chol(DX_sigma);
    memset(&dX_km1km1[0], 0, 9U * sizeof(real_T));
    for (i0 = 0; i0 < 9; i0++) {
      for (i1 = 0; i1 < 9; i1++) {
        dX_km1km1[i1 + 9 * (i0 + 1)] = -DX_sigma[i1 + 9 * i0];
      }
    }

    for (i0 = 0; i0 < 9; i0++) {
      memcpy(&dX_km1km1[9 * (i0 + 10)], &DX_sigma[9 * i0], 9U * sizeof(real_T));
    }

    /*  Calculate the full sigma points */
    for (i0 = 0; i0 < 19; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        b_dX_km1km1[i1 + 3 * i0] = dX_km1km1[i1 + 9 * i0];
      }
    }

    c_power(b_dX_km1km1, dv4);
    c_sum(dv4, dv5);
    for (i0 = 0; i0 < 19; i0++) {
      dv5[i0] = 1.0 - dv5[i0];
    }

    b_sqrt(dv5);
    for (i0 = 0; i0 < 19; i0++) {
      X_km1km1_temp[i0 << 2] = dv5[i0];
      for (i1 = 0; i1 < 3; i1++) {
        X_km1km1_temp[(i1 + (i0 << 2)) + 1] = dX_km1km1[i1 + 9 * i0];
      }
    }

    /* 	% Quaternion */
    /*                 % Angular Velocity */
    memcpy(&b_X_km1km1_temp[0], &X_km1km1_temp[0], 76U * sizeof(real_T));
    b_quatmultiply(b_X_km1km1_temp, *(real_T (*)[4])&x_km1km1[0], X_km1km1_temp);
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 19; i1++) {
        b_dX_km1km1[i0 + 3 * i1] = x_km1km1[4 + i0] + dX_km1km1[(i0 + 9 * i1) +
          3];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 19; i1++) {
        dv6[i0 + 3 * i1] = x_km1km1[7 + i0] + dX_km1km1[(i0 + 9 * i1) + 6];
      }
    }

    for (i0 = 0; i0 < 19; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        X_km1km1[i1 + 10 * i0] = X_km1km1_temp[i1 + (i0 << 2)];
      }

      for (i1 = 0; i1 < 3; i1++) {
        X_km1km1[(i1 + 10 * i0) + 4] = b_dX_km1km1[i1 + 3 * i0];
      }

      for (i1 = 0; i1 < 3; i1++) {
        X_km1km1[(i1 + 10 * i0) + 7] = dv6[i1 + 3 * i0];
      }
    }

    /*  Gyro Bias */
    /*  Propogate the sigma points with the nonlinear system model and the input */
    /*  vector: */
    memcpy(&b_X_km1km1[0], &X_km1km1[0], 190U * sizeof(real_T));
    RotateVecSensor2Cont(Torque_s, q_s_c, dv7);
    RK4(b_X_km1km1, I_c, dv7, Ts, X_km1km1);

    /*  Calculate the priori state estimate */
    W0_m = Lambda / (9.0 + Lambda);
    W0_c = pow(alpha, 2.0);
    W0_c = Lambda / (9.0 + Lambda) + ((1.0 - W0_c) + Beta);
    Wi_cm = 1.0 / (2.0 * (9.0 + Lambda));
    for (i0 = 0; i0 < 10; i0++) {
      b_W0_m[i0] = W0_m * X_km1km1[i0];
    }

    for (i0 = 0; i0 < 18; i0++) {
      for (i1 = 0; i1 < 10; i1++) {
        b_W0_m[i1 + 10 * (i0 + 1)] = Wi_cm * X_km1km1[i1 + 10 * (1 + i0)];
      }
    }

    d_sum(b_W0_m, x_kkm1);

    /*  sum along row */
    b_power(*(real_T (*)[4])&x_kkm1[0], dv8);
    for (i0 = 0; i0 < 4; i0++) {
      b_x_kkm1[i0] = (*(real_T (*)[4])&x_kkm1[0])[i0];
    }

    b_rdivide(b_x_kkm1, sqrt(b_sum(dv8)), *(real_T (*)[4])&x_kkm1[0]);

    /*  Calculate full error state */
    quatinv(*(real_T (*)[4])&x_kkm1[0], dv8);
    for (i0 = 0; i0 < 19; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        c_X_km1km1[i1 + (i0 << 2)] = X_km1km1[i1 + 10 * i0];
      }
    }

    b_quatmultiply(c_X_km1km1, dv8, X_km1km1_temp);
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 19; i1++) {
        Z_kkm1_control[i0 + 6 * i1] = X_km1km1[(i0 + 10 * i1) + 4] - x_kkm1[4 +
          i0];
      }
    }

    for (i0 = 0; i0 < 19; i0++) {
      for (i1 = 0; i1 < 3; i1++) {
        dX_km1km1[i1 + 9 * i0] = X_km1km1_temp[(i1 + (i0 << 2)) + 1];
      }

      for (i1 = 0; i1 < 6; i1++) {
        dX_km1km1[(i1 + 9 * i0) + 3] = Z_kkm1_control[i1 + 6 * i0];
      }
    }

    /*  Calculate mean error state */
    for (i0 = 0; i0 < 9; i0++) {
      c_W0_m[i0] = W0_m * dX_km1km1[i0];
    }

    for (i0 = 0; i0 < 18; i0++) {
      for (i1 = 0; i1 < 9; i1++) {
        c_W0_m[i1 + 9 * (i0 + 1)] = Wi_cm * dX_km1km1[i1 + 9 * (1 + i0)];
      }
    }

    e_sum(c_W0_m, dx_kkm1);

    /*  sum along row */
    /*  Calculate the priori error covariance matrix */
    memset(&DX_sigma[0], 0, 81U * sizeof(real_T));
    for (i = 0; i < 19; i++) {
      if (1 + i == 1) {
        for (i0 = 0; i0 < 9; i0++) {
          b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
          b_b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
        }

        for (i0 = 0; i0 < 9; i0++) {
          for (i1 = 0; i1 < 9; i1++) {
            DX_sigma[i0 + 9 * i1] += W0_c * b[i0] * b_b[i1];
          }
        }
      } else {
        for (i0 = 0; i0 < 9; i0++) {
          b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
          b_b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
        }

        for (i0 = 0; i0 < 9; i0++) {
          for (i1 = 0; i1 < 9; i1++) {
            DX_sigma[i0 + 9 * i1] += Wi_cm * b[i0] * b_b[i1];
          }
        }
      }
    }

    for (i0 = 0; i0 < 81; i0++) {
      DX_sigma[i0] += Q_k[i0];
    }

    /*  Propogate the sigma points through the sensor model in order to obtain */
    /*  the transformed sigma points. Predicted measurements must be in the */
    /*  satelite reference frame. */
    power(B_ref, dv9);
    rdivide(w_control, sqrt(sum(dv9)), dv10);
    SensorModel(X_km1km1, dv10, Z_kkm1_control);
    for (i0 = 0; i0 < 6; i0++) {
      d_W0_m[i0] = W0_m * Z_kkm1_control[i0];
    }

    for (i0 = 0; i0 < 18; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        d_W0_m[i1 + 6 * (i0 + 1)] = Wi_cm * Z_kkm1_control[i1 + 6 * (1 + i0)];
      }
    }

    f_sum(d_W0_m, z_kkm1_control);

    /*  sum along row */
    /*  Calculate the posteriori state estimate using the measurement vector */
    /*  containing measurements obtained at k: */
    memset(&P_zkzk[0], 0, 36U * sizeof(real_T));
    memset(&P_xkzk[0], 0, 54U * sizeof(real_T));
    for (i = 0; i < 19; i++) {
      if (1 + i == 1) {
        for (i0 = 0; i0 < 6; i0++) {
          Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
          b_Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
        }

        for (i0 = 0; i0 < 6; i0++) {
          for (i1 = 0; i1 < 6; i1++) {
            P_zkzk[i0 + 6 * i1] += W0_c * Torque_c[i0] * b_Torque_c[i1];
          }
        }

        for (i0 = 0; i0 < 9; i0++) {
          b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
        }

        for (i0 = 0; i0 < 6; i0++) {
          b_Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
        }

        for (i0 = 0; i0 < 9; i0++) {
          for (i1 = 0; i1 < 6; i1++) {
            P_xkzk[i0 + 9 * i1] += W0_c * b[i0] * b_Torque_c[i1];
          }
        }
      } else {
        for (i0 = 0; i0 < 6; i0++) {
          Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
          b_Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
        }

        for (i0 = 0; i0 < 6; i0++) {
          for (i1 = 0; i1 < 6; i1++) {
            P_zkzk[i0 + 6 * i1] += Wi_cm * Torque_c[i0] * b_Torque_c[i1];
          }
        }

        for (i0 = 0; i0 < 9; i0++) {
          b[i0] = dX_km1km1[i0 + 9 * i] - dx_kkm1[i0];
        }

        for (i0 = 0; i0 < 6; i0++) {
          b_Torque_c[i0] = Z_kkm1_control[i0 + 6 * i] - z_kkm1_control[i0];
        }

        for (i0 = 0; i0 < 9; i0++) {
          for (i1 = 0; i1 < 6; i1++) {
            P_xkzk[i0 + 9 * i1] += Wi_cm * b[i0] * b_Torque_c[i1];
          }
        }
      }
    }

    for (i0 = 0; i0 < 36; i0++) {
      P_zkzk[i0] += R_k[i0];
    }

    /*  Calculate the Kalman gain */
    memcpy(&b_P_xkzk[0], &P_xkzk[0], 54U * sizeof(real_T));
    mrdivide(b_P_xkzk, P_zkzk, P_xkzk);

    /*  Normalize predicted maganetic field vector */
    power(*(real_T (*)[3])&z_kkm1_control[0], c_Torque_c);
    rdivide(*(real_T (*)[3])&z_kkm1_control[0], sqrt(sum(c_Torque_c)),
            c_Torque_c);
    for (i = 0; i < 3; i++) {
      z_kkm1_control[i] = c_Torque_c[i];
    }

    /*  normalize Z vector */
    /*  Rotate measurements from sensor to control reference frame. */
    power(B_sat, dv11);
    rdivide(b_B_sat, sqrt(sum(dv11)), dv12);
    RotateVecSensor2Cont(dv12, q_s_c, c_Torque_c);
    RotateVecSensor2Cont(w_gyro, q_s_c, w_control);
    for (i0 = 0; i0 < 3; i0++) {
      b_Torque_c[i0] = c_Torque_c[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_Torque_c[i0 + 3] = w_control[i0];
    }

    for (i0 = 0; i0 < 6; i0++) {
      Torque_c[i0] = b_Torque_c[i0] - z_kkm1_control[i0];
    }

    for (i0 = 0; i0 < 9; i0++) {
      dx_kkm1[i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        dx_kkm1[i0] += P_xkzk[i0 + 9 * i1] * Torque_c[i1];
      }
    }

    /*  Calculate the full state */
    power(*(real_T (*)[3])&dx_kkm1[0], c_Torque_c);
    b_x_kkm1[0] = sqrt(1.0 - sum(c_Torque_c));
    for (i = 0; i < 3; i++) {
      b_x_kkm1[i + 1] = dx_kkm1[i];
    }

    quatmultiply(b_x_kkm1, *(real_T (*)[4])&x_kkm1[0], dv8);
    for (i = 0; i < 4; i++) {
      x_km1km1[i] = dv8[i];
    }

    for (i = 0; i < 3; i++) {
      x_km1km1[i + 4] = x_kkm1[i + 4] + dx_kkm1[i + 3];
    }

    for (i = 0; i < 3; i++) {
      x_km1km1[i + 7] = x_kkm1[i + 7] + dx_kkm1[i + 6];
    }

    /*  Calculate the a posteriori error covariance */
    for (i0 = 0; i0 < 9; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        c_P_xkzk[i0 + 9 * i1] = 0.0;
        for (i = 0; i < 6; i++) {
          c_P_xkzk[i0 + 9 * i1] += P_xkzk[i0 + 9 * i] * P_zkzk[i + 6 * i1];
        }
      }
    }

    for (i0 = 0; i0 < 9; i0++) {
      for (i1 = 0; i1 < 9; i1++) {
        W0_c = 0.0;
        for (i = 0; i < 6; i++) {
          W0_c += c_P_xkzk[i0 + 9 * i] * P_xkzk[i1 + 9 * i];
        }

        P_km1km1[i0 + 9 * i1] = DX_sigma[i0 + 9 * i1] - W0_c;
      }
    }
  }

  /* % Store, Rotate, and Deliver */
  /*  Store states for next call */
  /*  Convert states from satellite controller frame to the reference frame */
  quatinv(q_s_c, dv13);
  quatmultiply(*(real_T (*)[4])&x_km1km1[0], dv13, Attitude_sensor);
  RotateVecCont2Sensor(*(real_T (*)[3])&x_km1km1[4], q_s_c, w_sensor);
  RotateVecCont2Sensor(*(real_T (*)[3])&x_km1km1[7], q_s_c, w_bias_sensor);
}

void UKF_quaternion_withBias_initialize(void)
{
  x_km1km1_not_empty = FALSE;
}

void UKF_quaternion_withBias_terminate(void)
{
  /* (no terminate code required) */
}

/* End of code generation (UKF_quaternion_withBias.c) */
