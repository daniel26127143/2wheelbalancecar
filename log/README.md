# log

Raw logs behind the two plots on slide 16 of the Stage I deck. Both were recorded on 10/27 with firmware v37g and the C++ PC tools. One row is one status line from the robot (every 200 ms).

| File | Test | Tool |
|---|---|---|
| `drive.csv` | Manual driving: W forward with turns, then X back to the start | `drive.exe --port COM9` |
| `8pattern.csv` | Figure-eight: three `#C` arcs with radius 20 cm | `nav.exe "C62.8@-20@15 C125.7@20@15 C62.8@-20@15" --steps --port COM9` |

## Main columns

| Column | Meaning |
|---|---|
| `Timestamp (s)` | Time since the tool started |
| `Active_Cmd` | Command sent by the PC: `D+18.0/-0.50` = `#D` with speed 18 and turn rate −0.50 rad/s, `IDLE` = no command, `NAV` = route running |
| `State` | `NORMAL`, `EMERGENCY` (rescue mode), `FALLEN`, or `EVENT` (a message line, text in `Raw_Message`) |
| `ANG` | Tilt angle (°) |
| `SA` | Lean offset from the speed loop (°) |
| `B`, `T` | Balance output and turn output (PWM) |
| `PWM_L`, `PWM_R` | PWM sent to each motor |
| `Y`, `YR` | Heading (rad) and turn rate (rad/s) |
| `V`, `TV` | Wheel speed and target speed (ticks per 40.5 ms) |
| `PX`, `PY` | Position from odometry (cm). PX forward, PY to the right; `Z` resets both to 0 |
| `NV`, `NR` | Navigation step and the distance (cm) or angle (°) left |
| `Raw_Message` | The original line from the robot |

Distances use 33.6 ticks/cm (tape-measure check, firmware v37f and later). Positions come from the robot's own encoders and heading, not from an outside measurement.
