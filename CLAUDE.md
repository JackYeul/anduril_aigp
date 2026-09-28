# Drone Racing GNC — Claude Context

## Project Overview
Autonomous drone racing controller for the Anduril AI Grand Prix. Flies a cascaded PID
controller (position → velocity → attitude) through a gate course using ground-truth NED
position/velocity telemetry over MAVLink. Camera-based perception is scaffolded
(`vision_rx.py`) but intentionally not wired into control yet — it's relevant to later
stages of the competition that haven't been pursued.

## Stack
- Python 3.11
- NumPy, SciPy for math
- OpenCV for perception (scaffolded, not yet used by control)
- MAVLink (pymavlink) for communication with the sim

## Repo Structure
- `src/controller.py` — PID class + `Controller`: the position → velocity → attitude cascade, MAVLink attitude-target commands
- `src/config.py` — all PID gains and tunable params
- `src/mavlink_rx.py` — background thread parsing incoming MAVLink telemetry into the shared `data` dict
- `src/vision_rx.py` — background thread receiving camera frames (perception not yet wired into control)
- `src/timesync.py` — background thread sending periodic MAVLink timesync requests
- `src/setup.py` — wires up the MAVLink connection and all of the above components
- `src/main.py` — entry point: connect, arm, fly
- `src/data_logger.py` / `src/plot_run.py` — per-run CSV logging and commanded-vs-actual plotting
- `requirements.txt` — Python dependencies

## Conventions
- Type hints on all public functions
- PID gains and tunable params live in config.py only
