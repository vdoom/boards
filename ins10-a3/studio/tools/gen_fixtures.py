"""Generates the calibration fixtures for the C++ tests (tests/fixtures/).

The Python calibration tool (../firmware/tools/ins10tool) is the reference.
For each scenario of its own test suite (tools/tests/test_calib.py) this writes the
input as the firmware would have streamed it (a .log in protocol format) or, where
the Python test feeds arrays directly, the arrays (in the .json), together with the
Python results and the true sensor errors the data was made from.

The files are committed, so the C++ tests do not need Python. Re-run after changing
the Python tool:

    python tools/gen_fixtures.py

Needs numpy and scipy. Only reads the firmware folder.
"""
from __future__ import annotations

import contextlib
import dataclasses
import io
import json
import os
import sys

sys.dont_write_bytecode = True  # leave no __pycache__ in the firmware folder

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.normpath(os.path.join(HERE, "..", "..", "firmware", "tools"))
OUT = os.path.normpath(os.path.join(HERE, "..", "tests", "fixtures"))
sys.path.insert(0, os.path.join(TOOLS, "tests"))
sys.path.insert(0, TOOLS)

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

import test_calib as tc  # noqa: E402
from ins10tool import calib, cli  # noqa: E402
from ins10tool.records import load_log  # noqa: E402


def plain(value):
    """Dataclasses, numpy arrays and scalars as JSON-ready Python values."""
    if dataclasses.is_dataclass(value):
        return {f.name: plain(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    return value


def write_log(name: str, lines) -> str:
    path = os.path.join(OUT, name + ".log")
    with open(path, "w", encoding="ascii", newline="\n") as handle:
        handle.write("\n".join(lines) + "\n")
    return path


def write_json(name: str, data) -> None:
    with open(os.path.join(OUT, name + ".json"), "w", encoding="utf-8", newline="\n") as handle:
        json.dump(plain(data), handle, indent=1)
        handle.write("\n")


def run_cli(*argv) -> dict:
    """Runs the tool offline and collects the 'cal set' lines it prints."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = cli.main(list(argv))
    fields = {}
    for line in out.getvalue().splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[0] == "cal" and parts[1] == "set":
            fields[parts[2]] = [float(v) for v in parts[3:]]
    return {"exit_code": code, "fields": fields}


def contiguity(rec):
    seq, n = np.asarray(rec.imu_seq), np.asarray(rec.imu_n)
    return np.append(seq[1:] == seq[:-1] + n[:-1], True)


def outcome(fn):
    """The result, or the CalibrationError message."""
    try:
        return {"result": fn()}
    except calib.CalibrationError as error:
        return {"error": str(error)}


# ---- Scenarios (the same data as test_calib.py) ---------------------------------------

def mag():
    rng = np.random.default_rng(1)
    t, w, att = tc.smooth_tumble(150.0, 55.0, seed=2)
    field_imu = att.inv().apply(tc.FIELD_NAV)
    up_imu = att.inv().apply(tc.UP_NAV)
    mount = Rotation.from_euler("zyx", [178.0, 2.0, -1.5], degrees=True).as_matrix()
    soft = np.array([[1.08, 0.03, -0.02], [0.03, 0.94, 0.04], [-0.02, 0.04, 1.01]])
    hard = np.array([23.0, -41.0, 12.0])
    raw_mag = (field_imu @ mount.T) @ soft.T + hard + rng.normal(0, 0.05, field_imu.shape)
    accel = up_imu + rng.normal(0, 0.002, up_imu.shape) + np.array([0.004, -0.003, 0.005])
    gyro = np.degrees(w) + rng.normal(0, 0.05, w.shape)
    path = write_log("mag", tc.to_lines(gyro, accel, np.full(len(t), 30.0), block=8, mag_ut=raw_mag))
    rec = load_log(path)
    m, a, g = rec.mag_with_imu()
    result = calib.calibrate_mag(m, a, g)
    # The true field at each logged magnetometer sample (every 10th IMU sample).
    write_json("mag", {"python": result, "cli": run_cli("magcal", "--from-log", path),
                       "truth": {"hard": hard, "soft": soft, "mount": mount, "field_ut": 50.0, "dip_deg": 66.0,
                                 "field_imu_at_mag": field_imu[::10][:len(rec.mag)]}})


def mag_ring():
    rng = np.random.default_rng(3)
    angle = np.linspace(0, 12 * np.pi, 3000)
    ring = np.column_stack([20 * np.cos(angle), 20 * np.sin(angle), np.full(len(angle), -45.0)])
    ring += rng.normal(0, 0.05, ring.shape) + np.array([10.0, 5.0, -3.0])
    path = write_log("mag_ring", [f"$MAG,{i},{m[0]:.3f},{m[1]:.3f},{m[2]:.3f}" for i, m in enumerate(ring)])
    rec = load_log(path)
    write_json("mag_ring", outcome(lambda: calib.calibrate_mag(np.asarray(rec.mag))))


def imu():
    rng = np.random.default_rng(4)
    w, att = tc.multi_position_motion(16, seed=5)
    up = att.inv().apply(tc.UP_NAV)
    a_bias = np.array([0.012, -0.018, 0.025])
    a_matrix = np.array([[1.004, 0.002, -0.001], [0.002, 0.997, 0.003], [-0.001, 0.003, 1.006]])
    g_bias = np.array([0.31, -0.22, 0.14])
    g_matrix = np.array([[1.004, 0.003, -0.002], [-0.004, 0.995, 0.001], [0.002, -0.003, 1.006]])
    accel = up @ np.linalg.inv(a_matrix).T + a_bias + rng.normal(0, 0.0011, up.shape)
    gyro = np.degrees(w) @ np.linalg.inv(g_matrix).T + g_bias + rng.normal(0, 0.045, w.shape)
    path = write_log("imu", tc.to_lines(gyro, accel, np.full(len(w), 31.0), block=2))
    rec = load_log(path)
    t, dt, gy, ac, te = rec.imu_arrays()
    intervals = calib.find_static_intervals(t, dt, gy, ac, te)
    result = calib.calibrate_imu(t, dt, gy, ac, te, contiguity(rec))
    write_json("imu", {"python": result, "cli": run_cli("imucal", "--from-log", path),
                       "intervals": [[iv.start, iv.stop] for iv in intervals],
                       "truth": {"accel_bias": a_bias, "accel_matrix": a_matrix, "gyro_bias": g_bias,
                                 "gyro_matrix": g_matrix, "temp_c": 31.0, "positions": 17}})


def accel_faces():
    a_bias = np.array([0.012, -0.018, 0.025])
    faces = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=float)
    means = faces @ np.linalg.inv(np.diag([1.004, 0.997, 1.006])).T + a_bias
    bias, matrix, after, before, orientations = calib.calibrate_accel(means)
    write_json("accel_faces", {"means": means, "python": {"bias": bias, "matrix": matrix, "after": after,
                                                          "before": before, "orientations": orientations},
                               "four": outcome(lambda: calib.calibrate_accel(means[:4])),
                               "truth": {"bias": a_bias, "scale": [1.004, 0.997, 1.006]}})


def imu_lost():
    rng = np.random.default_rng(6)
    w, att = tc.multi_position_motion(14, seed=7)
    up = att.inv().apply(tc.UP_NAV)
    accel = up + rng.normal(0, 0.0011, up.shape)
    gyro = np.degrees(w) * 1.005 + rng.normal(0, 0.045, w.shape)
    period = int(5.0 * tc.ODR)
    hole = (3 * period + int(3.6 * tc.ODR), 3 * period + int(4.4 * tc.ODR))
    path = write_log("imu_lost", tc.to_lines(gyro, accel, np.full(len(w), 25.0), block=2, drop=hole))
    rec = load_log(path)
    t, dt, gy, ac, te = rec.imu_arrays()
    result = calib.calibrate_imu(t, dt, gy, ac, te, contiguity(rec))
    write_json("imu_lost", {"python": result, "lost_samples": rec.lost_samples,
                            "truth": {"gyro_scale": 1 / 1.005, "hole": hole}})


def thermal():
    rng = np.random.default_rng(8)
    n = 2400
    temp = 22.0 + 26.0 * (1 - np.exp(-np.arange(n) / 700.0))
    d = temp - 35.0
    g_true = np.array([[0.30, 0.012, 2e-4], [-0.21, -0.008, -1e-4], [0.15, 0.015, 3e-4]])
    gyro = g_true[:, 0] + np.outer(d, g_true[:, 1]) + np.outer(d ** 2, g_true[:, 2])
    gyro += rng.normal(0, 0.003, gyro.shape)
    a_tc = np.array([1.5e-4, -1.0e-4, 2.5e-4])
    accel = np.array([0.02, -0.01, 1.003]) + np.outer(temp - 28.0, a_tc) + rng.normal(0, 7e-5, (n, 3))
    gyro[1000:1010] += 6.0
    result = calib.calibrate_thermal(temp, gyro, accel, accel_t_ref=28.0)
    flat_temp = np.full(100, 30.0) + rng.normal(0, 0.05, 100)
    write_json("thermal", {"temp": temp, "gyro": gyro, "accel": accel, "accel_t_ref": 28.0, "python": result,
                           "flat": {"temp": flat_temp, "gyro": gyro[:100], "accel": accel[:100],
                                    **outcome(lambda: calib.calibrate_thermal(flat_temp, gyro[:100], accel[:100]))},
                           "truth": {"g_true": g_true, "t_true_ref": 35.0, "a_tc": a_tc}})


def thermal_tool():
    rng = np.random.default_rng(14)
    n = 256 * 400
    temp = 20.0 + 25.0 * np.arange(n) / n
    gyro = np.array([0.3, -0.2, 0.1]) + np.outer(temp - 30.0, [0.012, -0.009, 0.015]) + rng.normal(0, 0.045, (n, 3))
    accel = np.array([0.0, 0.0, 1.0]) + np.outer(temp - 30.0, [1e-4, -2e-4, 3e-4]) + rng.normal(0, 0.0011, (n, 3))
    path = write_log("thermal_tool", tc.to_lines(gyro, accel, temp, block=256))
    rec = load_log(path)
    t, dt, gy, ac, te = rec.imu_arrays()
    result = calib.calibrate_thermal(te, gy, ac, None)
    write_json("thermal_tool", {"python": result, "cli": run_cli("thermalcal", "--from-log", path),
                                "truth": {"slopes": [0.012, -0.009, 0.015]}})


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    for scenario in (mag, mag_ring, imu, accel_faces, imu_lost, thermal, thermal_tool):
        print(f"{scenario.__name__} ...", flush=True)
        scenario()
    total = sum(os.path.getsize(os.path.join(OUT, f)) for f in os.listdir(OUT))
    print(f"fixtures in {OUT}: {total / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
