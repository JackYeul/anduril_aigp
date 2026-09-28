import csv
import dataclasses
import os
import time
from dataclasses import dataclass

import config

NAN = float("nan")


@dataclass
class LogRow:
    """Commanded vs. actual value of every quantity in the PID cascade at one control tick."""
    t: float

    x_cmd: float
    x_act: float
    y_cmd: float
    y_act: float
    z_cmd: float
    z_act: float

    vx_cmd: float
    vx_act: float
    vy_cmd: float
    vy_act: float
    vz_cmd: float
    vz_act: float

    roll_cmd: float
    roll_act: float
    pitch_cmd: float
    pitch_act: float
    yaw_cmd: float
    yaw_act: float
    thrust_cmd: float

    roll_rate_act: float
    pitch_rate_act: float
    yaw_rate_act: float


class RunLogger:
    """Writes one CSV row per control tick for a single armed run."""

    def __init__(self, log_dir: str = config.LOG_DIR):
        os.makedirs(log_dir, exist_ok=True)
        filename = f"run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.path = os.path.join(log_dir, filename)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([f.name for f in dataclasses.fields(LogRow)])

    def log(self, row: LogRow) -> None:
        self._writer.writerow(dataclasses.astuple(row))
        self._file.flush()

    def close(self) -> None:
        self._file.close()
