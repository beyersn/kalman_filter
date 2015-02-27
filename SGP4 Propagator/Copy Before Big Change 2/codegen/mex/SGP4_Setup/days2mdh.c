/*
 * days2mdh.c
 *
 * Code generation for function 'days2mdh'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "SGP4_Setup.h"
#include "days2mdh.h"
#include "SGP4_Setup_data.h"
#include <stdio.h>

/* Function Definitions */
void days2mdh(real_T year, real_T days, real_T *mon, real_T *day, real_T *hr,
              real_T *minute, real_T *sec)
{
  real_T lmonth[12];
  int32_T i;
  real_T dayofyr;
  real_T inttemp;

  /*  ------------------------------------------------------------------------------ */
  /*  */
  /*                            function days2mdh */
  /*  */
  /*   this function converts the day of the year, days, to the equivalent month */
  /*     day, hour, minute and second. */
  /*  */
  /*   author        : david vallado                  719-573-2600   22 jun 2002 */
  /*  */
  /*   revisions */
  /*                 - */
  /*  */
  /*   inputs          description                    range / units */
  /*     year        - year                           1900 .. 2100 */
  /*     days        - julian day of the year         0.0  .. 366.0 */
  /*  */
  /*   outputs       : */
  /*     mon         - month                          1 .. 12 */
  /*     day         - day                            1 .. 28,29,30,31 */
  /*     hr          - hour                           0 .. 23 */
  /*     minute      - minute                         0 .. 59 */
  /*     sec         - second                         0.0 .. 59.999 */
  /*  */
  /*   locals        : */
  /*     dayofyr     - day of year */
  /*     temp        - temporary extended values */
  /*     inttemp     - temporary integer value */
  /*     i           - index */
  /*     lmonth(12)  - integer array containing the number of days per month */
  /*  */
  /*   coupling      : */
  /*     none. */
  /*  */
  /*  [mon,day,hr,minute,sec] = days2mdh ( year,days); */
  /*  ----------------------------------------------------------------------------- */
  /*  Set up array of days in month */
  memset(&lmonth[0], 0, 12U * sizeof(real_T));
  for (i = 0; i < 12; i++) {
    if (1 + i == 2) {
      lmonth[i] = 28.0;
    }

    if ((1 + i == 4) || (1 + i == 6) || (1 + i == 9) || (1 + i == 11)) {
      lmonth[i] = 30.0;
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  dayofyr = muDoubleScalarFloor(days);

  /*  Find month and day of month */
  if (muDoubleScalarRem(year - 1900.0, 4.0) == 0.0) {
    lmonth[1] = 29.0;

    /*  Leap year */
  }

  i = 1;
  inttemp = 0.0;
  while ((dayofyr > inttemp + lmonth[i - 1]) && (i < 12)) {
    inttemp += lmonth[i - 1];
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, emlrtRootTLSGlobal);
  }

  *mon = i;
  *day = dayofyr - inttemp;

  /*  Find hours minutes and seconds */
  inttemp = (days - dayofyr) * 24.0;
  if (inttemp < 0.0) {
    *hr = muDoubleScalarCeil(inttemp);
  } else {
    *hr = muDoubleScalarFloor(inttemp);
  }

  inttemp = (inttemp - *hr) * 60.0;
  if (inttemp < 0.0) {
    *minute = muDoubleScalarCeil(inttemp);
  } else {
    *minute = muDoubleScalarFloor(inttemp);
  }

  *sec = (inttemp - *minute) * 60.0;
}

/* End of code generation (days2mdh.c) */
