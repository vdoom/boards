"""Parsing of the firmware's text protocol (see docs/PROTOCOL.md).

The same functions serve a live serial connection and a saved log file, so every
calibration can be run offline from a recording.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Defaults for the ICM-42688-P 20-bit FIFO format; overridden by the "# key=value"
# lines of the 'info' command when they are present in the stream or log.
DEFAULT_SCALES = {
    "gyro_dps_per_lsb": 2000.0 / 524288.0,
    "accel_g_per_lsb": 16.0 / 524288.0,
    "temp_lsb_per_c": 132.48,
    "temp_offset_c": 25.0,
    "odr_hz": 512.0,
}


def parse_info_line(line: str) -> dict[str, str]:
    """'# a=1 b=two' -> {'a': '1', 'b': 'two'}; anything else -> {}."""
    if not line.startswith("#"):
        return {}
    out = {}
    for token in line[1:].split():
        key, sep, value = token.partition("=")
        if sep and key:
            out[key] = value.rstrip(",")
    return out


def parse_cal_line(line: str) -> tuple[str, list[float]] | None:
    """'cal set name v0 v1 ...' -> (name, values)."""
    parts = line.split()
    if len(parts) < 4 or parts[0] != "cal" or parts[1] != "set":
        return None
    try:
        return parts[2], [float(v) for v in parts[3:]]
    except ValueError:
        return None


@dataclass
class Recording:
    """Sensor data collected from a stream, in physical units, uncalibrated.

    IMU rows are block means: `n` raw samples were summed by the firmware, starting
    at sample index `seq`. Time is seq / odr, so it is exact and gap-aware.
    """

    scales: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_SCALES))
    imu_seq: list[int] = field(default_factory=list)
    imu_n: list[int] = field(default_factory=list)
    gyro: list[tuple[float, float, float]] = field(default_factory=list)   # dps
    accel: list[tuple[float, float, float]] = field(default_factory=list)  # g
    temp: list[float] = field(default_factory=list)                        # deg C
    mag: list[tuple[float, float, float]] = field(default_factory=list)    # uT
    mag_imu_index: list[int] = field(default_factory=list)  # IMU rows received before each mag row
    lost_samples: int = 0
    calibration: dict[str, list[float]] = field(default_factory=dict)

    def feed(self, line: str) -> str | None:
        """Consume one protocol line. Returns the record type, or None."""
        line = line.strip()
        if line.startswith("$RAW,"):
            parts = line.split(",")
            if len(parts) != 10:
                return None
            try:
                values = [int(v) for v in parts[1:]]
            except ValueError:
                return None
            seq, n = values[0], values[1]
            if n <= 0:
                return None
            if self.imu_seq:
                expected = self.imu_seq[-1] + self.imu_n[-1]
                if seq > expected:
                    self.lost_samples += seq - expected
            s = self.scales
            self.imu_seq.append(seq)
            self.imu_n.append(n)
            self.gyro.append(tuple(v / n * s["gyro_dps_per_lsb"] for v in values[2:5]))
            self.accel.append(tuple(v / n * s["accel_g_per_lsb"] for v in values[5:8]))
            self.temp.append(values[8] / n / s["temp_lsb_per_c"] + s["temp_offset_c"])
            return "RAW"
        if line.startswith("$MAG,"):
            parts = line.split(",")
            if len(parts) != 5:
                return None
            try:
                self.mag.append(tuple(float(v) for v in parts[2:5]))
            except ValueError:
                return None
            self.mag_imu_index.append(len(self.imu_seq))
            return "MAG"
        if line.startswith("#"):
            for key, value in parse_info_line(line).items():
                if key in self.scales:
                    try:
                        self.scales[key] = float(value)
                    except ValueError:
                        pass
            return "INFO"
        cal = parse_cal_line(line)
        if cal:
            self.calibration[cal[0]] = cal[1]
            return "CAL"
        return None

    # ---- Array views -----------------------------------------------------------

    def imu_arrays(self):
        """(t [s], dt [s], gyro [dps], accel [g], temp [C]) as numpy arrays.

        t is the mid-time of each block and dt its duration.
        """
        seq = np.asarray(self.imu_seq, dtype=float)
        n = np.asarray(self.imu_n, dtype=float)
        odr = self.scales["odr_hz"]
        return ((seq + 0.5 * n) / odr, n / odr, np.asarray(self.gyro, dtype=float).reshape(-1, 3),
                np.asarray(self.accel, dtype=float).reshape(-1, 3), np.asarray(self.temp, dtype=float))

    def mag_with_imu(self):
        """Magnetometer rows paired with the IMU rows around them in the stream.

        Returns (mag [uT], accel [g], gyro [dps]). The firmware prints a mag sample a
        few milliseconds after it was taken, between two IMU blocks; the mean of
        those two blocks is the IMU reading at that moment to within a block length.
        """
        _, _, gyro, accel, _ = self.imu_arrays()
        mag = np.asarray(self.mag, dtype=float).reshape(-1, 3)
        index = np.asarray(self.mag_imu_index, dtype=int)
        keep = (index >= 1) & (index < len(accel))
        index = index[keep]
        return (mag[keep], 0.5 * (accel[index - 1] + accel[index]),
                0.5 * (gyro[index - 1] + gyro[index]))


def load_log(path: str) -> Recording:
    rec = Recording()
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            rec.feed(line)
    return rec
