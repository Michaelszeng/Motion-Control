@echo off
@echo cmd logfilename ps_name
copy %1.log %1_old.log/Y
adb logcat -c
rem start setprop.cmd 1
IF "%2"==""  (
	adb shell ps |findstr com.qualcomm.ftcrobotcontroller>ps.txt
) ELSE (
	adb shell ps |findstr %2 >ps.txt
)
set /p ftcrobot=<ps.txt
IF "%ftcrobot%"=="" (
	@echo "com.qualcomm.ftcrobotcontroller has died!!!"
	goto XX
)
echo %ftcrobot%

for /F "tokens=1,2 delims=: " %%a in ("%ftcrobot%") do (
   @rem echo %%a
   echo %%b
   set var=%%b
)
IF "%1"==""  (
	adb logcat -b all -v color --pid %var% 
	rem adb logcat -b all -v color |findstr %var%
) ELSE (
	adb logcat -b all  --pid %var% |wtee %1.log
	rem adb logcat -b all -v color |findstr %var% |wtee %1.log
)

:XX
