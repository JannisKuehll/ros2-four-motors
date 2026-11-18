# Four Motors with ROS 2

This demonstration uses ROS 2 to coordinate four different motors, each
controlled by a separate Infineon microcontroller. The ROS 2 program runs on
a Raspberry Pi or another computer connected to the boards over USB. A
keyboard command starts one motor for approximately two seconds; only one
motor runs at a time.

## Files

- `ROS/motor_demo.py`: ROS 2 controller and the four serial bridge nodes.
- `ROS/firmware/`: the programs for the four microcontrollers.
- `ROS/run.sh`, `ROS/run.cmd`, and `ROS/requirements.txt`: launch scripts and
  Python dependency.
- `3D_Printing/STL/`: the motor holder and the DC and BLDC rotation indicators
  as STL files.
- `3D_Printing/3MF/`: the corresponding supplied 3MF files.

See [ROS/README.txt](ROS/README.txt) for flashing, installation and running
instructions.

## Hardware and Wiring

Connect each microcontroller to the ROS 2 host over USB. A powered USB hub may
be necessary depending on the host and its power supply.

- **DC:** XMC4700 Relax Kit with the BTN9970LV shield. Plug the shield onto
  the board, connect the motor to OUT1 and OUT2, and connect the motor supply
  to the shield.
- **BLDC:** XMC1100 Boot Kit with the TLE9879QXA40 shield. Plug the shield
  onto the board and connect the motor supply. In this setup the red motor
  wire connects to L1, yellow to L2, and black to L3.
- **Stepper:** PSOC™ 6 and a TMC2209 driver. Connect P9_0 to STEP, P9_1
  to DIR, P9_2 to EN, 3V3 to the driver's VDD, and board GND to driver GND.
  Connect the motor supply to the driver's power input. In this setup the
  green motor wire connects to 1B, black to 1A, red to 2A, and blue to 2B.
- **Servo:** PSOC™ Edge E84. Connect the signal to P16_1, power to the 5 V
  solder pad on the back of the board, and servo ground to board GND.

Motor wire colours are specific to the motors used here. Check your own motor
and driver documentation before wiring. The demonstrated setup uses a separate
12 V, 1.25 A supply for each of the DC, BLDC and stepper drivers. The servo
uses 5 V from the board, not 12 V. Verify motor and driver ratings, polarity,
common grounds and the stepper driver's current limit before applying power.

## 3D Printing

The supplied STL files do not contain units; import them in millimetres. The
3MF files declare millimetres. Check the orientation and dimensions in your
slicer, and verify fit before operating the motors.

## Safety

Disconnect motor power while wiring or adjusting the mechanism. Secure the
motors and keep hands clear of moving parts. A software STOP is not an
emergency stop; keep a way to disconnect motor power within reach.

## License

No license file is included yet. Check the rights and select appropriate
licenses for the code and 3D files before publishing or sharing them.
