@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2013a
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2013a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=SGP4_Setup_mex
set MEX_NAME=SGP4_Setup_mex
set MEX_EXT=.mexw64
call mexopts.bat
echo # Make settings for SGP4_Setup > SGP4_Setup_mex.mki
echo COMPILER=%COMPILER%>> SGP4_Setup_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> SGP4_Setup_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> SGP4_Setup_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> SGP4_Setup_mex.mki
echo LINKER=%LINKER%>> SGP4_Setup_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> SGP4_Setup_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> SGP4_Setup_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> SGP4_Setup_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> SGP4_Setup_mex.mki
echo BORLAND=%BORLAND%>> SGP4_Setup_mex.mki
echo OMPFLAGS= >> SGP4_Setup_mex.mki
echo OMPLINKFLAGS= >> SGP4_Setup_mex.mki
echo EMC_COMPILER=msvc100>> SGP4_Setup_mex.mki
echo EMC_CONFIG=optim>> SGP4_Setup_mex.mki
"C:\Program Files\MATLAB\R2013a\bin\win64\gmake" -B -f SGP4_Setup_mex.mk
