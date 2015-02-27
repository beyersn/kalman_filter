/* Include files */

#include "UKF_10hz_sfun.h"
#include "UKF_10hz_sfun_debug_macros.h"
#include "c2_UKF_10hz.h"
#include "c3_UKF_10hz.h"
#include "c5_UKF_10hz.h"
#include "c6_UKF_10hz.h"
#include "c7_UKF_10hz.h"
#include "c8_UKF_10hz.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _UKF_10hzMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void UKF_10hz_initializer(void)
{
}

void UKF_10hz_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_UKF_10hz_method_dispatcher(SimStruct *simstructPtr, unsigned int
  chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_UKF_10hz_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_UKF_10hz_process_check_sum_call( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3979433985U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3773173702U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(303049931U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2258659208U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1850665397U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1671674039U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2245016761U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(722547192U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c2_UKF_10hz_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c3_UKF_10hz_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c5_UKF_10hz_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c6_UKF_10hz_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c7_UKF_10hz_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_UKF_10hz_get_check_sum(mxArray *plhs[]);
          sf_c8_UKF_10hz_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1451661113U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2468675666U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1662977581U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3894208977U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_UKF_10hz_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "YPKCb8qgfXarf3Fu5LGaIH") == 0) {
          extern mxArray *sf_c2_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c2_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "PSK2gVEA0BRJ6t5sx5PCKG") == 0) {
          extern mxArray *sf_c3_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c3_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "Ns8c56m2y3Qy42TjZOseCF") == 0) {
          extern mxArray *sf_c5_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c5_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "xqJm2Ji5Spw9FYD9tXdHQH") == 0) {
          extern mxArray *sf_c6_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c6_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "B4RAqXnj7v1LZkByIwbGEC") == 0) {
          extern mxArray *sf_c7_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c7_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "7f9SDkw4ejBpvZV0gvCAVC") == 0) {
          extern mxArray *sf_c8_UKF_10hz_get_autoinheritance_info(void);
          plhs[0] = sf_c8_UKF_10hz_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_UKF_10hz_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray *sf_c2_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray *sf_c6_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray *sf_c7_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray *sf_c8_UKF_10hz_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_UKF_10hz_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_UKF_10hz_third_party_uses_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Vg1EQ99RHiPkvsFP0b3jkC") == 0) {
          extern mxArray *sf_c2_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c2_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "xTH5pp6cLaZ1k9AaPSyjuF") == 0) {
          extern mxArray *sf_c3_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c3_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "1gCFCPe2W3QtJkrHlArF8B") == 0) {
          extern mxArray *sf_c5_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c5_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "RK2sNpoxjzVOSxC01rGccG") == 0) {
          extern mxArray *sf_c6_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c6_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "RUBn5C8lxKYbHdAHK5PsCG") == 0) {
          extern mxArray *sf_c7_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c7_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "0jmSpcluQ3X4ttwcZlivwE") == 0) {
          extern mxArray *sf_c8_UKF_10hz_third_party_uses_info(void);
          plhs[0] = sf_c8_UKF_10hz_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_UKF_10hz_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Vg1EQ99RHiPkvsFP0b3jkC") == 0) {
          extern mxArray *sf_c2_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "xTH5pp6cLaZ1k9AaPSyjuF") == 0) {
          extern mxArray *sf_c3_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "1gCFCPe2W3QtJkrHlArF8B") == 0) {
          extern mxArray *sf_c5_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "RK2sNpoxjzVOSxC01rGccG") == 0) {
          extern mxArray *sf_c6_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "RUBn5C8lxKYbHdAHK5PsCG") == 0) {
          extern mxArray *sf_c7_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c7_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "0jmSpcluQ3X4ttwcZlivwE") == 0) {
          extern mxArray *sf_c8_UKF_10hz_updateBuildInfo_args_info(void);
          plhs[0] = sf_c8_UKF_10hz_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void UKF_10hz_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _UKF_10hzMachineNumber_ = sf_debug_initialize_machine(debugInstance,"UKF_10hz",
    "sfun",0,6,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_UKF_10hzMachineNumber_,0,
    0);
  sf_debug_set_machine_data_thresholds(debugInstance,_UKF_10hzMachineNumber_,0);
}

void UKF_10hz_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_UKF_10hz_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("UKF_10hz",
      "UKF_10hz");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_UKF_10hz_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
