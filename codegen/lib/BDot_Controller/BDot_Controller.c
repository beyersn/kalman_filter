/*
 * BDot_Controller.c
 *
 * Code generation for function 'BDot_Controller'
 *
 * C source code generated on: Mon Oct 14 18:58:09 2013
 *
 */

/* Include files */
#include "BDot_Controller.h"

/* Variable Definitions */
static real_T Bint[3];
static real_T Bdot[3];
static boolean_T Bdot_not_empty;

/* Function Definitions */
void BDot_Controller(const real_T B_s[3], real32_T Command_Trigger, real_T Ts,
                     real_T Dipole_Command_s[3])
{
  int32_T i;
  int32_T i0;
  static const int32_T a[9] = { -21500000, 0, 0, 0, -21500000, 0, 0, 0,
    -19500000 };

  /*  BDot_Controller */
  /*  Author: Brandon Jackson */
  /*  Contact: bajackso@mtu.edu */
  /*  Date: 7 August 2013 */
  /*  */
  /*  This function is a Bdot based detumble controller. */
  if ((Command_Trigger == 1.0F) || (!Bdot_not_empty)) {
    /*  This is the measurement stage of the controller, */
    Bdot_not_empty = TRUE;
    for (i = 0; i < 3; i++) {
      Bdot[i] = 0.8 * (B_s[i] - Bint[i]);
      Bint[i] += Bdot[i] * Ts;
      Dipole_Command_s[i] = 0.0;
    }
  } else {
    for (i = 0; i < 3; i++) {
      Dipole_Command_s[i] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        Dipole_Command_s[i] += (real_T)a[i + 3 * i0] * Bdot[i0];
      }
    }
  }
}

void BDot_Controller_initialize(void)
{
  int32_T i;
  Bdot_not_empty = FALSE;
  for (i = 0; i < 3; i++) {
    Bint[i] = 0.0;
  }
}

void BDot_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/* End of code generation (BDot_Controller.c) */
