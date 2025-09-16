# ⚠️ **WARNING: PROJECT UNDER DEVELOPMENT, CHANGES WILL BE MADE**
<p align="center" style="display: flex; gap: 10px;">
  <img src="https://github.com/ManhTi3012/DuoDrive/blob/main/photos/schematic.png" width="24%" />
  <img src="https://github.com/ManhTi3012/DuoDrive/blob/main/photos/PCB.png" width="24%" />
  <img src="https://github.com/ManhTi3012/DuoDrive/blob/main/photos/render.png" width="24%" />
  <img src="https://github.com/ManhTi3012/DuoDrive/blob/main/photos/code.png" width="24%" />
</p>

# DuoDrive
DuoDrive is a closed-loop brushed motor driver design to offer simple and affordable open-source hardware and software:
* Running internal PID loop for driving up to 2 brushed motor.
* Increment-type and absolute-type encoder (not yet).
* UART interface and CAN Bus (no firmware support yet).
* Velocity and position control.

  An autonomous robot using DuoDrive's velocity control can still run straight without line or gyro correction.
  ![hi](https://github.com/ManhTi3012/DuoDrive/blob/main/photos/robot.gif)

# Hardware
DuoDrive is designed to handle high current within a small footprint:
* Voltage input: 12-24v, 30V absolute maximum
* Maximum continuous current: 25A
* Power traces are exposed for high current handling, beware of shorts
* Dimensions: 49mm x 75mm

Step file can be found in "3D" folder.
# Firmware
The firmware is expected to be stable for now, but further testing still needed for stability-related bugs.
Please open a new issues if you encounter any problem.
For detailed explanation and methods, check Wiki page.

# Documentation
For more detail, check the [WIKI Page](https://github.com/ManhTi3012/DuoDrive/wiki)

# Liability
* The software and hardware are still in development and may contain bugs, errors, or incomplete features.
* Users are encouraged to use this software and hardware responsibly and at their own risk.
