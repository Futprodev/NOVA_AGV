# NOVA_AGV Firmware Test Project

PlatformIO firmware for an Arduino Mega 2560 used to test NOVA AGV low-level control: motor drivers, hall odometry, and MPU6050 yaw. The Mega receives velocity commands over serial and streams odometry back to the host.

## Hardware used
- Arduino Mega 2560
- 2x DC motors + motor drivers (PWM, DIR, STOP)
- 2x hall sensors for wheel ticks
- MPU6050 IMU over I2C

## Firmware behavior
- Runs the control loop every `CONTROL_PERIOD_MS` (default 50 ms).
- Parses serial lines containing `V:<linear_mps> W:<angular_radps>`.
- Applies a 500 ms command timeout and stops both motors if no new command arrives.
- Updates IMU yaw, wheel odometry, and a PD wheel-speed controller.
- Streams odometry at 50 ms intervals as `O:<x> <y> <theta> <v> <w>`.

## Serial protocol
PC -> Mega (command):
- `V:0.20 W:0.00`
- `V:-0.15 W:0.20`

Mega -> PC (odometry):
- `O:<x> <y> <theta> <v> <w>`

Units:
- `V` in m/s
- `W` in rad/s
- `x, y` in meters
- `theta` in radians

## Configuration
Edit `include/Config.h` for:
- Wheel radius, base width, hall ticks per revolution
- Control loop period
- PD gains
- Pin assignments

## Build and upload (PlatformIO)
From the project root:

```bash
pio run
pio run -t upload --environment megaatmega2560
pio device monitor -b 115200
```

## Quick test
1. Open the serial monitor at 115200 baud.
2. Send `V:0.20 W:0.00` and verify odometry lines start streaming.
3. Stop commands to confirm the 500 ms timeout stops the motors.

## Project layout
- `src/main.cpp`: serial protocol, control loop, odometry output
- `include/`: motor driver, controller, IMU, and odometry modules
- `test/`: PlatformIO test scaffolding
