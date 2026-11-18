@echo off
rem Starts the motor demo on Windows with the native ROS 2 installation.
rem Usage: run.cmd                      start the demo
rem        run.cmd --check-connections  only check the four boards (no motor moves)
setlocal
if not defined ROS_WINDOWS_ROOT set "ROS_WINDOWS_ROOT=%USERPROFILE%\Desktop\ros2-windows\ros2-windows"
if not exist "%ROS_WINDOWS_ROOT%\start_native.cmd" (
  echo ERROR: Native ROS setup not found at "%ROS_WINDOWS_ROOT%". 1>&2
  echo Set ROS_WINDOWS_ROOT to the folder containing start_native.cmd. 1>&2
  exit /b 1
)
call "%ROS_WINDOWS_ROOT%\start_native.cmd" python -u "%~dp0motor_demo.py" %*
exit /b %errorlevel%
