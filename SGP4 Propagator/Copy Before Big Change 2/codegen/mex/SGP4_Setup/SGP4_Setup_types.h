/*
 * SGP4_Setup_types.h
 *
 * Code generation for function 'SGP4_Setup'
 *
 * C source code generated on: Thu Jul 11 14:30:47 2013
 *
 */

#ifndef __SGP4_SETUP_TYPES_H__
#define __SGP4_SETUP_TYPES_H__

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_ResolvedFunctionInfo
#define typedef_ResolvedFunctionInfo
typedef struct
{
    const char * context;
    const char * name;
    const char * dominantType;
    const char * resolved;
    uint32_T fileTimeLo;
    uint32_T fileTimeHi;
    uint32_T mFileTimeLo;
    uint32_T mFileTimeHi;
} ResolvedFunctionInfo;
#endif /*typedef_ResolvedFunctionInfo*/
#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct
{
    real_T mu;
    real_T radiusearthkm;
    real_T xke;
    real_T tumin;
    real_T j2;
    real_T j3;
    real_T j4;
    real_T j3oj2;
} b_struct_T;
#endif /*typedef_b_struct_T*/
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct
{
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
#endif /*typedef_struct_T*/

#endif
/* End of code generation (SGP4_Setup_types.h) */
