ROS 2 Four-Motor Demo
=====================

One ROS 2 controller coordinates four motors. Every motor has its own
microcontroller, connected over USB:

  dc       XMC4700 Relax Kit + BTN9970LV shield    firmware/dc_motor/dc_motor.ino
  bldc     XMC1100 Boot Kit + TLE9879QXA40 shield  firmware/bldc_motor/bldc_motor.ino
  stepper  PSOC 6 board (MicroPython)               firmware/stepper/stepper.py
  servo    PSOC Edge board (MicroPython)            firmware/servo/servo.py

Files
-----
  motor_demo.py     ROS 2 program (controller + one bridge node per motor)
  run.sh            start script for Linux / Raspberry Pi
  run.cmd           start script for Windows (native ROS 2 installation)
  requirements.txt  Python package (pyserial) for Windows / pip
  firmware/         one program per microcontroller

1. Flash the boards
-------------------
  Switch off the motor supplies while wiring and flashing.
  Install the Infineon XMC board package for Arduino:
    https://github.com/Infineon/XMC-for-Arduino
  dc_motor.ino    Arduino IDE, board KIT_XMC47_RELAX, upload
  bldc_motor.ino  Arduino IDE, board KIT_XMC11_BOOT_001, upload

  Install compatible MicroPython firmware using the board instructions:
    PSOC 6:    https://ifx-micropython.readthedocs.io/en/latest/psoc6/installation.html
    PSOC Edge: https://ifx-micropython.readthedocs.io/en/latest/psoc-edge/installation.html
  In Thonny (https://thonny.org), connect one board at a time, select
  Tools > Options > Interpreter > MicroPython (generic) and its serial port.
  Open the corresponding file and choose File > Save as > MicroPython device:
    stepper.py -> main.py on the PSOC 6 board
    servo.py   -> main.py on the PSOC Edge board
  This overwrites an existing main.py. Reset the board, and close Thonny
  before starting the ROS program so the serial port is free.
  For the BLDC, switch on the shield's 12 V supply before resetting the XMC1100.

2. Linux / Raspberry Pi (Ubuntu)
--------------------------------
  Install ROS 2 Lyrical using the official Ubuntu instructions:
    https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html
  Install pyserial:
    sudo apt install python3-serial
  Allow access to the serial ports, then log out and in again:
    sudo usermod -aG dialout $USER
  Start:
    chmod +x run.sh
    ./run.sh --check-connections   (finds all four boards, motors do not move)
    ./run.sh

3. Windows
----------
  Install ROS 2 Lyrical using the official Windows instructions:
    https://docs.ros.org/en/lyrical/Get-Started/Installation/Windows-Install-Binary.html
  In a Command Prompt, activate the Pixi environment and source ROS 2 as
  described there. In this ROS directory, install pyserial and start:
    python -m pip install -r requirements.txt
    python -u motor_demo.py --check-connections
    python -u motor_demo.py

  Alternatively, run.cmd is for a custom native ROS 2 installation providing
  start_native.cmd. If it is not at the default path used by run.cmd, set
  ROS_WINDOWS_ROOT to its directory:
    run.cmd --check-connections
    run.cmd

Keys
----
  d = DC   b = BLDC   t = stepper   s = servo   (each runs for 2 s)
  SPACE = stop all   q = quit

More options
------------
  --list-ports         show all serial ports
  --port ROLE=PORT     skip automatic search, give all four, e.g.
                       --port dc=/dev/ttyACM0 --port bldc=/dev/ttyACM1 ...
  --no-keyboard        no terminal input; publish keys to /motor_demo/key

How it works
------------
  The script finds the boards by their USB IDs (J-Link and KitProg3) and asks
  each one "HELLO". The board answers with its role, so the USB order does
  not matter. Each motor gets the topics /motors/<role>/command and
  /motors/<role>/status. Only one motor runs at a time. Every board stops its
  motor by itself after 2 s.

  The BLDC shield only restarts reliably after a reset. After every run, the
  firmware resets it in the background (~2.5 s, status REARMING). During this
  time, the other three motors can still be used.

Safety
------
  STOP is a software stop, not an emergency stop. Keep a way to cut motor
  power within reach. If the script reports "STOP not confirmed", switch off
  the motor power. Missing confirmations, a lost USB connection (heartbeat)
  or an error from a board stop all motors and end the program.
