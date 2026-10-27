# Design of an Omnidirectional-Wheel Self-Balancing Robot

An underactuated two-wheel inverted pendulum robot driven by collinear Mecanum wheels. It runs on an ATmega328P (Arduino Uno) with hardware-timed, multi-rate cascade control, and is designed so that a third wheel can be added later for sideways motion.

[![YouTube](https://img.shields.io/badge/YouTube-Video%20Demo-red?logo=youtube)](https://www.youtube.com/shorts/1qH6NnuWVuc)
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
| Firmware | v37g (`repo.ino` in the repository root) |
| PC tools | C++ (`cpp/`): `drive.exe` for driving, `nav.exe` for routes |

Status: the **two-wheel robot is built and tested**. The **three-wheel version (a central 90° omni wheel for sideways motion) is planned and not built yet.**

## Measured results

All numbers come from the robot's own sensors (5 Hz status log), with no outside measurement. Sample sizes are small, and the firmware changed between test sessions.

| Test | Result | Basis |
|---|---|---|
| Standing still | Tilt within ±1.2° of its mean (σ = 0.42°) | 35 s, 10/12 log |
| Longest balancing time | About 850 s without a fall, driving back and forth | 10/20 log, firmware v37e |
| Straight driving | 16.5 cm/s forward, 17.7 cm/s backward | 10/13 autotest, firmware v34 |
| Move commands `#M` | 26 of 31 finished; mean error 1.1 cm, worst 2.6 cm | 10/11 logs, odometry |
| Turn commands `#T` | 32 turns; mean error 1.3°, worst 2.4° | 10/11 logs, yaw sensor |
| Manual driving (W/X/A/D) | Keys pressed and the robot path, plotted on slide 16 | `log/drive.csv`, 10/27, v37g |
| Figure-eight (three `#C` arcs, r = 20 cm) | Ended about 2 cm from the start point (odometry) | `log/8pattern.csv`, 10/27, v37g |

Notes:
- 5 of the 31 moves did not finish (3 timeouts, 2 emergency stops). In the 10/13 autotest, 1 of 43 runs ended in a fall.
- Logs before 10/20 used an encoder scale of 36.2 ticks/cm (50 cm push test). A tape-measure check on 10/20 gave 33.6 ticks/cm (used from v37f on), so the older distances and speeds read about 8% low.

## Control design

The interrupt (Timer2, CTC mode, prescaler 64, OCR2A = 148) runs the balance loop every **10.13 ms (98.7 Hz)**. The speed loop runs on every 4th pass (**40.5 ms**). The ISR is guarded and re-enables interrupts so the encoder counters keep running.

1. **Balance loop (PD).** `ub = Kp·(θref − θ) − Kd·q` with Kp = 1275 and Kd = 40. The tilt rate `q` is read directly from the gyro register (the DMP gyro value was almost always 0). A back-EMF feedforward (1.8 PWM per speed unit above 8, max 50) is added at high speed.
2. **Speed loop (PI).** Wheel speed is filtered (`Vf = 0.7·Vf + 0.3·V`). The PI output (Kp = 2.8, Ki = 0.015) is a small tilt offset θoff, limited to ±3.5° (±3.0° while braking, ±4.0° with the overspeed brake). If speed is more than 8 units over the target, the P gain is multiplied by 1.6.
3. **Heading loop.** `ut = Kff·ωref + Kpy·(ψref − ψ) + Kdy·(ωref − ω)` with Kff = 30, Kpy = 38, Kdy = 7, limited to ±45 PWM and reduced when the tilt error is large (balance has priority). From v36/v37: a heading integral while driving straight, and a heading hold when standing (press `Y` to set the heading; it turns back if the error is over 4°, and stops within 1.5°).
4. **Motor output.** `uL = ub + ut`, `uR = ub − ut`, then dead-zone compensation (5 PWM) and a left-wheel trim of 1.037, clamped to ±255.

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
| `#D v ω` | Continuous drive: speed `v` from −20 to 20 units, turn rate `ω` up to ±0.6 rad/s while moving (±1.0 on the spot). Must be resent about 10 times a second |
| `#M dist [angle]` | Move straight `dist` cm (negative = backward), optionally holding a heading |
| `#T angle` | Turn to a heading in degrees (0 = start heading, positive = right) |
| `#C arc radius [speed]` | Drive an arc of length `arc` cm (positive radius = right turn). A new `#C` during an arc chains to it without stopping |
| `#F` | Finish the current arc and slow down |
| `#S` or `#A` | Stop or abort |

Navigation replies: `NAV:START`, `NAV:DONE`, `NAV:TIMEOUT`, `NAV:ABORT`, `NAV:REJECT`. Telemetry is sent every 200 ms (tilt, speed, state, PWM, odometry X/Y). The column list is in `log/README.md`.

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

Encoders: 660 ticks per wheel turn (11-line Hall × 1:30 gearbox × 2 edges counted). Distance scale: 33.6 ticks/cm (tape-measure check, 10/20).

## Repository layout

| Path | Content |
|---|---|
| `repo.ino`, `nav.h`, `nav_cmd.h`, `rec.h`, `tuning.h` | **Current firmware (v37g).** Keep all five files in one folder |
| `cpp/` | **PC tools in C++**: `drive.cpp` (keyboard driving), `nav.cpp` (routes), `link.h` (serial link and CSV log), `sim.h` (simulated robot). See `cpp/README.md` |
| `log/` | Logs behind the slide 16 plots: `drive.csv` (manual driving) and `8pattern.csv` (figure-eight) |
| `python/` | Earlier Python tools. `autotest.py` and `analyze.py` are still used for automatic tests and log analysis |
| `stable/` | Saved firmware versions (`v0.3` = v37g) |
| `latest/` | An older firmware version, kept for history (the current one is in the root) |
| `libraries/` | Arduino libraries used by the firmware |
| `Experiments/` | Encoder and sensor test sketches |
| `flow/` | Flow charts |
| `historyrepo/` | Older documents and versions |

## Build and run

**Firmware**
1. Install the Arduino libraries from `libraries/` (or the Library Manager): I2Cdev, MPU6050 (with the DMP MotionApps 2.0 header) and PinChangeInterrupt.
2. Open `repo.ino` (in the repository root) in the Arduino IDE. Keep `nav.h`, `nav_cmd.h`, `rec.h` and `tuning.h` in the same folder, or it will not compile.
3. Select **Arduino Uno** and upload.
4. Power on with the robot upright and still while the DMP starts. The serial port prints a start message and `v37g 11`, which confirms the firmware is loaded. The MPU-6050 offsets in `setup()` belong to this unit; calibrate again for another unit.

**PC tools (Windows, C++)**
1. Install g++ (MinGW-w64). In `cpp/`, run `build.bat`. It builds `drive.exe` and `nav.exe` (`g++ -std=c++17 -O2 -static`).
2. Pair the HC-06, then drive with the keyboard:
   ```
   drive.exe --port COM9
   ```
   Keys: `W`/`X` forward/back, `A`/`D` left/right, `S` stop, `C` centre the steering, `Z` reset the position, `Y` set the heading, `U`/`I` nudge the heading, `Q` quit.
3. Run a route, for example the figure-eight on slide 16:
   ```
   nav.exe "C62.8@-20@15 C125.7@20@15 C62.8@-20@15" --steps --port COM9
   ```
   Use `--dry-run` (preview) or `--sim` (simulated robot) first. `nav.exe --help` lists all options.
4. Both tools save a CSV log for every run in `controller/logs/<date>/`.

Notes: flash use is almost full, so `ENABLE_TUNING` is 0 by default. The feature switches (`ENABLE_EMF_FF`, `ENABLE_OVERSPEED_BRAKE`, `ENABLE_FAST_START`, `ENABLE_YAW_HOLD`, `ENABLE_YAW_I`) in `repo.ino` can be set to 0 to turn each feature off.

## Roadmap (planned, not built)

- **Three-wheel robot:** left and right Mecanum wheels plus a central 90° omni wheel for sideways motion, with the balance loop unchanged. With two wheels, sideways motion and turning always come together; the third wheel adds the missing constraint. The command would grow from `#D v ω` to `#D vx vy ω`.
- **Android phone remote:** a ready-made Bluetooth joystick app to send `#D`, plus fixed routes.

## Reference

[1] M. T. Watson, D. T. Gladwin, T. J. Prescott and S. O. Conran, "Dual-mode model predictive control of an omnidirectional wheeled inverted pendulum," *IEEE/ASME Trans. Mechatronics*, vol. 24, no. 6, pp. 2964–2975, Dec. 2019, doi: 10.1109/TMECH.2019.2943708.
