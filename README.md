# Anduril AI Grand Prix — Drone Racing Controller 🛩️

An autonomous drone racing controller built for the [Anduril AI Grand Prix](https://www.anduril.com/), a
competition where teams fly a simulated racing quadrotor through a gate course as fast as possible.
The drone is commanded entirely over MAVLink — no manual piloting.

**Status:** consistently completes the course.

## How it works

The controller (`src/controller.py`) is a cascaded PID loop, run every control tick:

1. **Position loop** — NED position error to the active gate → desired NED velocity setpoint.
2. **Velocity loop** — velocity error → desired roll/pitch angle + thrust. A multirotor
   accelerates by tilting: pitching nose-down accelerates it North, rolling accelerates it East.
3. **Attitude command** — roll/pitch/yaw are converted to a quaternion and sent as a single
   `SET_ATTITUDE_TARGET` MAVLink message; the vehicle's onboard controller closes the rate loop.

All PID gains and tunable constants live in `src/config.py`. Position and velocity feedback
comes from the sim's ground-truth `LOCAL_POSITION_NED` telemetry — this is not yet a
GPS-denied/vision-based solution. Camera frames are received and decoded
(`src/vision_rx.py`), but perception isn't wired into control yet; that's relevant to later
stages of the competition that haven't been pursued.

Every armed run is logged tick-by-tick to a CSV (`src/data_logger.py`) and can be visualized
with `src/plot_run.py`, which plots commanded vs. actual position/velocity/attitude/thrust —
this was the main tool used for tuning.

## Repo layout

```
src/
  main.py          entry point: connect, arm, fly
  setup.py         wires up the MAVLink connection and background components
  controller.py    the PID cascade + attitude commands
  config.py        all PID gains and tunable constants
  mavlink_rx.py     background thread parsing incoming MAVLink telemetry
  vision_rx.py      background thread receiving camera frames (unused by control)
  timesync.py       background thread sending MAVLink timesync requests
  data_logger.py    per-run CSV logging
  plot_run.py       plot a logged run (commanded vs. actual)
requirements.txt
```

## Running it

1. Install dependencies:
   ```
   pip install -r requirements.txt
   ```
2. Obtain and launch the AI Grand Prix simulator separately (not included in this repo —
   it's a large third-party binary distribution).
3. Start the controller:
   ```
   python src/main.py
   ```
   It waits for track data from the sim, then prompts before arming and flying the course.
4. After a run, plot it:
   ```
   python src/plot_run.py
   ```
   (defaults to the most recently logged run in `src/logs/`)
