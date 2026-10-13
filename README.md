# Design of an Omnidirectional-Wheel Self-Balancing Robot

An underactuated two-wheel inverted pendulum robot driven by collinear Mecanum wheels. It runs on an ATmega328P (Arduino Uno) with hardware-timed, multi-rate cascade control, and is designed so that a third wheel can be added later for sideways motion.

[![YouTube](https://img.shields.io/badge/YouTube-Video%20Demo-red?logo=youtube)](https://www.youtube.com/shorts/cfTprMj1Mzs)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20Uno-blue.svg)](https://www.arduino.cc/)

[English](README.md) | [繁體中文](README_zh.md)

## Overview

| Item | Value |
|---|---|
| Controller | ATmega328P, 16 MHz (Arduino Uno) |
| Attitude sensor | MPU-6050 (DMP for pitch and yaw, gyro register read for tilt rate) |
| Motor driver | TB6612FNG, two DC gear motors with Hall encoders |
| Wheels | Two 65 mm Mecanum wheels on one axle |
| Size and mass | 19 cm wide, 17 cm high, 1.039 kg |
| Wireless link | HC-06 Bluetooth, 9600 baud |
| Power | 2S 18650 battery (7.4 V) for motors; 5 V power bank for the chip and sensor |

Status: the **two-wheel robot is built and tested**. The **three-wheel version (a central 90° omni wheel for sideways motion) and the Android phone remote are planned and not built yet.**

## Measured results

Numbers come from the logs of 10/11 to 10/13/2026. They are measured by the robot's own sensors at a 5 Hz log rate, with small sample sizes, and the firmware changed between sessions.

| Test | Result |
|---|---|
| Standing still | Tilt stays within ±1.2° of its mean (largest deviation 1.1°, σ = 0.42°; 35 s of standing logs, 10/12) |
| Longest standing time | More than 90 s (demo video) |
| Straight driving (firmware v34, 10/13 autotest) | Forward: mean 16.5 cm/s (command 18, 4 runs). Backward: mean 17.7 cm/s (command 20, 3 runs). No falls in these runs |
| Move commands `#M` (10/11, older firmware) | 26 of 31 finished; average error 1.1 cm, worst 2.6 cm |
| Turn commands `#T` (10/11, older firmware) | 32 turns; average error 1.3°, worst 2.4° |

In the 10/13 autotest, 1 of 43 runs ended in a fall. Distances use an encoder scale of 36.2 ticks/cm, calibrated by pushing the robot 50 cm.

## Control design

The interrupt (Timer2, CTC mode) runs the balance loop every **10.13 ms (98.7 Hz)**. The speed loop runs on every 4th pass (**40.5 ms**). The ISR is guarded and re-enables interrupts so the encoder counters keep running.

1. **Balance loop (PD).** `ub = Kp·(θref − θ) − Kd·q` with Kp = 1275 and Kd = 40. The tilt rate `q` is read directly from the gyro register. A back-EMF feedforward (1.8 PWM per speed unit above 8, max 50) is added at high speed.
2. **Speed loop (PI).** Wheel speed is filtered (time constant about 95 ms). The PI output (Kp = 2.8, Ki = 0.015) is a small tilt offset θoff, limited to ±3.5° (±3.0° while braking, ±4.0° with the overspeed brake). If speed is more than 8 units over the target, the P gain is multiplied by 1.6.
3. **Heading loop.** `ut = Kff·ωref + Kpy·(ψref − ψ) + Kdy·(ωref − ω)` with Kff = 30, Kpy = 38, Kdy = 7. It gives a turning torque difference between the two wheels: `uL = ub + ut`, `uR = ub − ut`.
4. **Motor output.** Dead-zone compensation (5 PWM) and a left-wheel trim of 1.037, then PWM clamped to ±255.

Safety states, checked first in every balance cycle:

| Condition | Action |
|---|---|
| Tilt error > 35° | `FALLEN`: motors off, states reset. Restarts when tilt error is below 3° for 20 cycles |
| Tilt error > 7° (exit below 4.5°) | `RESCUE`: strong PD only (Kp 1400, Kd 48), no turning |
| No command for 300 ms | Stop the motion command |
| No serial character for 1 s during navigation | Abort the navigation |

The robot model and axes follow Watson et al. [1].

## Commands

Commands are text lines sent over the Bluetooth serial port.

| Command | Meaning |
|---|---|
| `#D v ω` | Continuous drive: speed `v` from −20 to 20 units, turn rate `ω` up to ±0.6 rad/s while moving (±1.0 on the spot) |
| `#M dist [angle]` | Move straight `dist` cm (negative = backward), optionally holding a heading |
| `#T angle` | Turn to a heading in degrees (0 = start heading, positive = right) |
| `#C arc radius` | Drive an arc (positive radius = right turn) |
| `#S` or `#A` | Stop or abort |

Navigation replies: `NAV:START`, `NAV:DONE`, `NAV:TIMEOUT`, `NAV:ABORT`, `NAV:REJECT`. Telemetry is sent every 200 ms (tilt, speed, state, PWM, odometry X/Y).

## Wiring

| Function | Arduino pin |
|---|---|
| Motor A: IN1, IN2, PWM | D7, D6, D9 |
| Motor B: IN3, IN4, PWM | D13, D12, D10 |
| TB6612FNG STBY | D8 |
| Left encoder A / B | D2 (INT0) / A0 |
| Right encoder A / B | A1 (pin change) / D4 |
| MPU-6050 | I²C (A4 = SDA, A5 = SCL) |
| HC-06 | Hardware serial (D0 / D1), 9600 baud |

Encoders: 660 ticks per wheel turn (11-line Hall × 1:30 gearbox × 2 edges counted).

## Repository layout

| Folder | Content |
|---|---|
| `latest/` | Latest firmware (`repo.ino`, v34, with `nav.h`, `nav_cmd.h`, `rec.h`, `tuning.h` in the same folder) |
| `latest/python/` | PC tools: `drive.py` (drive), `nav.py` (routes), `autotest.py`, `analyze.py`, and more. See its README |
| `stable/` | Earlier stable version |
| `libraries/` | Arduino libraries used by the firmware |
| `Experiments/` | Encoder test sketches |
| `flow/` | Flow charts |
| `historyrepo/` | Older documents and versions |

## Build and run

1. Install the Arduino libraries from `libraries/`: I2Cdev, MPU6050 (with the DMP MotionApps 2.0 header) and PinChangeInterrupt.
2. Open `repo.ino` in the Arduino IDE. Keep `nav.h`, `nav_cmd.h`, `rec.h` and `tuning.h` in the same folder, or it will not compile.
3. Select **Arduino Uno** and upload.
4. Place the robot upright for the first seconds. It calibrates the gyro, then starts balancing.
5. Pair the HC-06 on the PC and run the tools in `python/`:
   ```
   python -m pip install pyserial
   python drive.py --port COM5
   ```

Notes: flash use is almost full (about 32.2 KB of 32.3 KB), so `ENABLE_TUNING` is 0 by default. The feature switches `ENABLE_EMF_FF`, `ENABLE_OVERSPEED_BRAKE` and `ENABLE_FAST_START` in `repo.ino` can be set to 0 to return to the v33 behavior.

## Roadmap (planned, not built)

- **Three-wheel robot:** left and right Mecanum wheels plus a central 90° omni wheel for sideways motion, with the balance loop unchanged. The command would grow from `#D v ω` to `#D vx vy ω`.
- **Android phone remote:** a ready-made Bluetooth terminal or joystick app to send `#D`, plus fixed routes.

## Reference

[1] M. T. Watson, D. T. Gladwin, T. J. Prescott and S. O. Conran, "Dual-mode model predictive control of an omnidirectional wheeled inverted pendulum," *IEEE/ASME Trans. Mechatronics*, vol. 24, no. 6, pp. 2964–2975, Dec. 2019, doi: 10.1109/TMECH.2019.2943708.
