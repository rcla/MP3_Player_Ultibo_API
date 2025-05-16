SET OLDPATH=%PATH%
SET TL_PATH=C:\Ultibo\GCC\bin\
SET PATH=%TL_PATH%;C:\Ultibo\Core\fpc\3.2.2\bin\i386-win32\;%PATH%

arm-none-eabi-gcc -DUltibo -D_POSIX_THREADS -O2 -mabi=aapcs -marm -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard -D__DYNAMIC_REENT__ -c *.c -c imdct_l_arm.S
arm-none-eabi-ar rcs ../libmad.a *.o
rm -f *.o

SET PATH=%OLDPATH%
