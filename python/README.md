# PC tools (Python)

Host-side tools for the two-wheel Mecanum self-balancing robot. They talk to the Arduino firmware (`repo.ino`, v34) over the HC-06 Bluetooth serial link (9600 baud) and record telemetry to CSV.

Run them from this folder. Set the serial port with `--port COM5` (default is `COM_PORT` in `control.py`).

| File | What it does |
|---|---|
| `drive.py` | Drive the robot with continuous speed and turn rate (keyboard or USB gamepad). Sends `#D v ω`. |
| `autotest.py` | Repeatable automatic tests (fixed hold and rest times). Results go to `logs/autotest_results.csv`. |
| `nav.py` | Run a route of waypoints (`routes/*.txt`) with `#M` (move) and `#T` (turn). Has a `--sim` mode with no robot. |
| `control.py` | Keyboard remote control and telemetry logger. Other tools import it for the serial port and CSV logging. |
| `bal_rec.py` | Short high-rate recording for balance diagnosis. |
| `analyze.py` | Analyze telemetry CSV files and write a text report and plots. |
| `cruise_stats.py` | Speed and tilt-offset statistics for straight driving. |
| `organize_logs.py` | Move telemetry CSV files into `logs/YYYYMMDD/`. |
| `routes/` | Example routes (square, rectangle, zigzag, circles). |
| `logs/autotest_results.csv` | Results of the 10/13 autotest runs used in the Stage I slides. |

## Requirements

- Python 3
- `pyserial` (all tools that connect to the robot)
- `pandas` (`cruise_stats.py`), `matplotlib` (plots in `analyze.py` and `bal_rec.py`), `pygame` (only for `drive.py --pad`)

```
python -m pip install pyserial pandas matplotlib
```

Examples:

```
python drive.py --dry-run
python nav.py "0,0 1,1 1,-1 -1,-1" --sim
python analyze.py logs/20261012
```
