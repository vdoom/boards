"""Tests of the calibration algorithms and protocol parsing on synthetic sensors.

Each test builds raw readings from known sensor errors, runs the same code the tool
runs on a real recording (including the text protocol), and checks that the errors
are recovered. Run directly:  python tools/tests/test_calib.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from fake_board import FakeBoard  # noqa: E402
from ins10tool import calib, cli  # noqa: E402
from ins10tool.records import DEFAULT_SCALES, Recording  # noqa: E402

ODR = 512.0
FIELD_NAV = 50.0 * np.array([0.0, np.cos(np.radians(66.0)), -np.sin(np.radians(66.0))])  # ENU
UP_NAV = np.array([0.0, 0.0, 1.0])


def smooth_tumble(duration: float, rate_dps: float, seed: int, fs: float = ODR):
    """Body rates (rad/s) of a slow, smooth tumble and the resulting attitudes."""
    rng = np.random.default_rng(seed)
    t = np.arange(int(duration * fs)) / fs
    w = np.zeros((len(t), 3))
    for axis in range(3):
        for _ in range(4):
            f = rng.uniform(0.03, 0.25)
            w[:, axis] += rng.uniform(0.3, 1.0) * np.sin(2 * np.pi * f * t + rng.uniform(0, 6.28))
    w *= np.radians(rate_dps) / np.abs(w).max()
    return t, w, integrate(w, 1.0 / fs)


def integrate(w: np.ndarray, dt: float) -> Rotation:
    """Attitude (body -> nav) after each sample, for body rates w."""
    steps = Rotation.from_rotvec(w * dt)
    q = np.empty((len(w), 4))
    current = Rotation.identity()
    for i in range(len(w)):
        current = current * steps[i]
        q[i] = current.as_quat()
    return Rotation.from_quat(q)


def to_lines(gyro_dps, accel_g, temp_c, block: int, mag_ut=None, mag_every: int = 10,
             drop: tuple[int, int] | None = None) -> list[str]:
    """Encode samples as the firmware would print them ($RAW sums and $MAG)."""
    s = DEFAULT_SCALES
    g = np.round(gyro_dps / s["gyro_dps_per_lsb"]).astype(np.int64)
    a = np.round(accel_g / s["accel_g_per_lsb"]).astype(np.int64)
    tr = np.round((temp_c - s["temp_offset_c"]) * s["temp_lsb_per_c"]).astype(np.int64)
    lines = []
    for i0 in range(0, len(g) - block + 1, block):
        if mag_ut is not None:
            for k in range(i0, i0 + block):
                if k % mag_every == 0:
                    m = mag_ut[k]
                    lines.append(f"$MAG,{int(k / ODR * 1000)},{m[0]:.3f},{m[1]:.3f},{m[2]:.3f}")
        if drop and drop[0] <= i0 < drop[1]:
            continue  # lines lost on the way to the host
        sums = list(g[i0:i0 + block].sum(axis=0)) + list(a[i0:i0 + block].sum(axis=0)) + [tr[i0:i0 + block].sum()]
        lines.append(f"$RAW,{i0},{block}," + ",".join(str(int(v)) for v in sums))
    return lines


def record(lines) -> Recording:
    rec = Recording()
    for line in lines:
        rec.feed(line)
    return rec


def contiguity(rec: Recording) -> np.ndarray:
    seq = np.asarray(rec.imu_seq)
    n = np.asarray(rec.imu_n)
    return np.append(seq[1:] == seq[:-1] + n[:-1], True)


# ---- Magnetometer -------------------------------------------------------------------

def test_magnetometer():
    rng = np.random.default_rng(1)
    t, w, att = smooth_tumble(150.0, 55.0, seed=2)
    field_imu = att.inv().apply(FIELD_NAV)           # field in the IMU frame
    up_imu = att.inv().apply(UP_NAV)
    # The magnetometer chip is mounted rotated relative to the IMU, has soft iron
    # (gain, cross-coupling) and a large hard-iron offset from the carrier board.
    mount = Rotation.from_euler("zyx", [178.0, 2.0, -1.5], degrees=True).as_matrix()
    soft = np.array([[1.08, 0.03, -0.02], [0.03, 0.94, 0.04], [-0.02, 0.04, 1.01]])
    hard = np.array([23.0, -41.0, 12.0])
    raw_mag = (field_imu @ mount.T) @ soft.T + hard + rng.normal(0, 0.05, field_imu.shape)
    accel = up_imu + rng.normal(0, 0.002, up_imu.shape) + np.array([0.004, -0.003, 0.005])
    gyro = np.degrees(w) + rng.normal(0, 0.05, w.shape)

    rec = record(to_lines(gyro, accel, np.full(len(t), 30.0), block=8, mag_ut=raw_mag))
    mag, acc, gyr = rec.mag_with_imu()
    result = calib.calibrate_mag(mag, acc, gyr)
    assert result.aligned, result.warnings
    assert not result.warnings, result.warnings

    corrected = (raw_mag - result.bias) @ result.matrix.T
    norms = np.linalg.norm(corrected, axis=1)
    cosang = np.einsum("ij,ij->i", corrected, field_imu) / (norms * 50.0)
    angle = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
    print(f"  mag: bias error {np.abs(result.bias - hard).max():.3f} uT, direction error "
          f"rms {np.sqrt(np.mean(angle ** 2)):.3f} max {angle.max():.3f} deg, "
          f"|B| spread {norms.std() / norms.mean() * 100:.3f} %, dip {result.dip_deg:.2f} deg, "
          f"coverage {result.coverage * 100:.0f} %")
    assert np.abs(result.bias - hard).max() < 0.3
    assert angle.max() < 0.6
    assert norms.std() / norms.mean() < 0.004
    assert abs(result.dip_deg - 66.0) < 0.5
    # The soft-iron scale is fixed by convention (unit average gain), so the field
    # magnitude is right to within the determinant of the true soft-iron matrix.
    assert abs(result.field_ut / (50.0 * np.linalg.det(soft) ** (1 / 3)) - 1.0) < 0.01


MIRROR = np.diag([1.0, 1.0, -1.0])  # INS10-A3: the MMC5983MA's Z axis points into the board


def heading_error_deg(corrected: np.ndarray, field_imu: np.ndarray, up_imu: np.ndarray) -> np.ndarray:
    """Angle between the horizontal parts of the corrected and the true field."""
    def horizontal(v):
        h = v - np.einsum("ij,ij->i", v, up_imu)[:, None] * up_imu
        return h / np.linalg.norm(h, axis=1, keepdims=True)
    cosang = np.einsum("ij,ij->i", horizontal(corrected), horizontal(field_imu))
    return np.degrees(np.arccos(np.clip(cosang, -1, 1)))


def test_magnetometer_mirrored():
    """A magnetometer whose axes are a mirror image of the IMU's, in both hemispheres.

    Taking the alignment as a proper rotation turns such a field upside down: the dip
    comes out negative and the heading 180 deg out. That is what the INS10-A3 showed.
    """
    t, w, att = smooth_tumble(150.0, 55.0, seed=4)
    up_imu = att.inv().apply(UP_NAV)
    soft = np.array([[1.03, 0.02, 0.0], [0.02, 0.97, -0.01], [0.0, -0.01, 1.0]])
    hard = np.array([49.8, -5.8, 1.3])
    for southern, nav in ((False, FIELD_NAV), (True, FIELD_NAV * np.array([1.0, 1.0, -1.0]))):
        rng = np.random.default_rng(5)
        field_imu = att.inv().apply(nav)
        raw_mag = (field_imu @ MIRROR.T) @ soft.T + hard + rng.normal(0, 0.05, field_imu.shape)
        accel = up_imu + rng.normal(0, 0.002, up_imu.shape)
        rec = record(to_lines(np.degrees(w), accel, np.full(len(t), 30.0), block=8, mag_ut=raw_mag))
        result = calib.calibrate_mag(*rec.mag_with_imu(), southern=southern)
        assert result.aligned and result.mirrored, (southern, result.warnings)
        assert np.linalg.det(result.matrix) < 0
        corrected = (raw_mag - result.bias) @ result.matrix.T
        heading = heading_error_deg(corrected, field_imu, up_imu)
        print(f"  mirrored, {'south' if southern else 'north'}: dip {result.dip_deg:.2f} deg, "
              f"heading error max {heading.max():.3f} deg")
        assert abs(result.dip_deg - (-66.0 if southern else 66.0)) < 0.5  # positive when pointing down
        assert heading.max() < 1.5  # 0.6 deg in direction, over cos(dip)

    # The usual case is unchanged: right-handed axes still give a proper rotation.
    rng = np.random.default_rng(6)
    field_imu = att.inv().apply(FIELD_NAV)
    raw_mag = field_imu + hard + rng.normal(0, 0.05, field_imu.shape)
    rec = record(to_lines(np.degrees(w), up_imu, np.full(len(t), 30.0), block=8, mag_ut=raw_mag))
    result = calib.calibrate_mag(*rec.mag_with_imu())
    assert result.aligned and not result.mirrored and np.linalg.det(result.matrix) > 0


def test_magnetometer_near_equator():
    """With almost no vertical field the hemisphere cannot pick the sign: a warning."""
    t, w, att = smooth_tumble(150.0, 55.0, seed=7)
    up_imu = att.inv().apply(UP_NAV)
    nav = 40.0 * np.array([0.0, np.cos(np.radians(4.0)), -np.sin(np.radians(4.0))])
    rng = np.random.default_rng(8)
    field_imu = att.inv().apply(nav)
    raw_mag = field_imu + np.array([10.0, 20.0, -5.0]) + rng.normal(0, 0.05, field_imu.shape)
    rec = record(to_lines(np.degrees(w), up_imu, np.full(len(t), 30.0), block=8, mag_ut=raw_mag))
    result = calib.calibrate_mag(*rec.mag_with_imu())
    assert result.aligned and not result.mirrored
    assert abs(result.dip_deg - 4.0) < 0.5
    assert any("magnetic equator" in text for text in result.warnings), result.warnings


def test_magnetometer_rejects_poor_data():
    rng = np.random.default_rng(3)
    # Rotation about one axis only: a ring, not an ellipsoid.
    angle = np.linspace(0, 12 * np.pi, 3000)
    ring = np.column_stack([20 * np.cos(angle), 20 * np.sin(angle), np.full(len(angle), -45.0)])
    ring += rng.normal(0, 0.05, ring.shape)
    try:
        result = calib.calibrate_mag(ring + np.array([10.0, 5.0, -3.0]))
    except calib.CalibrationError as error:
        print(f"  single-axis data rejected: {error}")
        return
    assert result.warnings and result.coverage < 0.6, "poor coverage must at least be reported"
    print(f"  single-axis data flagged: coverage {result.coverage * 100:.0f} %")


# ---- Accelerometer and gyro ---------------------------------------------------------

def multi_position_motion(positions: int, seed: int):
    """Rest for 3.5 s, turn to a new orientation in 1.5 s, and so on."""
    rng = np.random.default_rng(seed)
    rest, turn = int(3.5 * ODR), int(1.5 * ODR)
    w = []
    for _ in range(positions):
        w.append(np.zeros((rest, 3)))
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(np.radians(70), np.radians(150))
        profile = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(turn) / turn)  # smooth start and stop
        w.append(np.outer(profile * angle / (profile.sum() / ODR), axis))
    w.append(np.zeros((rest, 3)))
    w = np.vstack(w)
    return w, integrate(w, 1.0 / ODR)


def test_imu():
    rng = np.random.default_rng(4)
    w, att = multi_position_motion(16, seed=5)
    up = att.inv().apply(UP_NAV)
    a_bias = np.array([0.012, -0.018, 0.025])
    a_matrix = np.array([[1.004, 0.002, -0.001], [0.002, 0.997, 0.003], [-0.001, 0.003, 1.006]])
    g_bias = np.array([0.31, -0.22, 0.14])
    g_matrix = np.array([[1.004, 0.003, -0.002], [-0.004, 0.995, 0.001], [0.002, -0.003, 1.006]])
    # corrected = M (raw - b)  =>  raw = M^-1 true + b
    accel = up @ np.linalg.inv(a_matrix).T + a_bias + rng.normal(0, 0.0011, up.shape)
    gyro = np.degrees(w) @ np.linalg.inv(g_matrix).T + g_bias + rng.normal(0, 0.045, w.shape)

    rec = record(to_lines(gyro, accel, np.full(len(w), 31.0), block=2))
    t, dt, gy, ac, te = rec.imu_arrays()
    result = calib.calibrate_imu(t, dt, gy, ac, te, contiguity(rec))
    print(f"  imu: {result.positions} positions, {result.orientations} orientations; accel residual "
          f"{result.accel_residual_before_mg:.2f} -> {result.accel_residual_mg:.3f} mg; gyro "
          f"({result.gyro_matrix_fitted}) residual {result.gyro_residual_before_deg:.3f} -> "
          f"{result.gyro_residual_deg:.3f} deg")
    print(f"       accel bias error {np.abs(result.accel_bias - a_bias).max() * 1000:.3f} mg, matrix error "
          f"{np.abs(result.accel_matrix - a_matrix).max():.5f}; gyro bias error "
          f"{np.abs(result.gyro_bias - g_bias).max():.4f} dps, matrix error "
          f"{np.abs(result.gyro_matrix - g_matrix).max():.5f}")
    assert result.positions == 17 and result.orientations >= 9
    assert np.abs(result.accel_bias - a_bias).max() < 0.0005
    assert np.abs(result.accel_matrix - a_matrix).max() < 0.0005
    assert np.abs(result.gyro_bias - g_bias).max() < 0.005
    assert result.gyro_matrix_fitted == "full"
    assert np.abs(result.gyro_matrix - g_matrix).max() < 0.0015
    assert result.gyro_residual_deg < 0.25 * result.gyro_residual_before_deg
    assert abs(result.temp_c - 31.0) < 0.05

    # Six faces only: bias and scale, no cross-axis terms, and it says so.
    faces = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=float)
    means = faces @ np.linalg.inv(np.diag([1.004, 0.997, 1.006])).T + a_bias
    bias, matrix, after, before, orientations = calib.calibrate_accel(means)
    assert orientations == 6 and np.abs(bias - a_bias).max() < 1e-6 and after < 1e-3 < before
    assert np.abs(np.diag(matrix) - [1.004, 0.997, 1.006]).max() < 1e-6

    # Too few orientations is an error, not a silent bad fit.
    try:
        calib.calibrate_accel(means[:4])
    except calib.CalibrationError:
        pass
    else:
        raise AssertionError("four orientations must be rejected")


def test_imu_with_lost_lines():
    """A turn with lost samples must be skipped, not integrated across the hole."""
    rng = np.random.default_rng(6)
    w, att = multi_position_motion(14, seed=7)
    up = att.inv().apply(UP_NAV)
    accel = up + rng.normal(0, 0.0011, up.shape)
    gyro = np.degrees(w) * 1.005 + rng.normal(0, 0.045, w.shape)
    period = int(5.0 * ODR)
    hole = (3 * period + int(3.6 * ODR), 3 * period + int(4.4 * ODR))  # inside the fourth turn
    rec = record(to_lines(gyro, accel, np.full(len(w), 25.0), block=2, drop=hole))
    assert abs(rec.lost_samples - (hole[1] - hole[0])) <= 2, rec.lost_samples
    t, dt, gy, ac, te = rec.imu_arrays()
    result = calib.calibrate_imu(t, dt, gy, ac, te, contiguity(rec))
    scale = np.diag(result.gyro_matrix)
    print(f"  imu with {rec.lost_samples} lost samples: gyro scale {scale.round(5)} (true 0.99502)")
    assert result.gyro_matrix_fitted != "none"
    assert np.abs(scale - 1 / 1.005).max() < 0.002


# ---- Temperature --------------------------------------------------------------------

def test_thermal():
    rng = np.random.default_rng(8)
    n = 2400
    temp = 22.0 + 26.0 * (1 - np.exp(-np.arange(n) / 700.0))
    d = temp - 35.0
    g_true = np.array([[0.30, 0.012, 2e-4], [-0.21, -0.008, -1e-4], [0.15, 0.015, 3e-4]])
    gyro = g_true[:, 0] + np.outer(d, g_true[:, 1]) + np.outer(d ** 2, g_true[:, 2])
    gyro += rng.normal(0, 0.003, gyro.shape)
    a_tc = np.array([1.5e-4, -1.0e-4, 2.5e-4])
    accel = np.array([0.02, -0.01, 1.003]) + np.outer(temp - 28.0, a_tc) + rng.normal(0, 7e-5, (n, 3))
    gyro[1000:1010] += 6.0  # someone bumped the table

    result = calib.calibrate_thermal(temp, gyro, accel, accel_t_ref=28.0)
    model = result.gyro_bias + sum(np.outer((temp - result.t_ref) ** (k + 1), result.gyro_tc[:, k]) for k in range(3))
    truth = g_true[:, 0] + np.outer(d, g_true[:, 1]) + np.outer(d ** 2, g_true[:, 2])
    print(f"  thermal: order {result.order}, {result.t_min:.1f}..{result.t_max:.1f} C, gyro drift "
          f"{result.gyro_drift_dps:.3f} dps -> residual {result.gyro_residual_dps:.4f} dps, model error "
          f"{np.abs(model - truth).max():.4f} dps; accel drift {result.accel_drift_mg:.2f} mg -> "
          f"{result.accel_residual_mg:.3f} mg")
    assert np.abs(model - truth).max() < 0.003
    assert np.abs(result.accel_tc[:, 0] - a_tc).max() < 2e-5
    assert result.gyro_drift_dps > 0.3 and result.gyro_residual_dps < 0.005
    assert any("moved" in w for w in result.warnings)

    try:
        calib.calibrate_thermal(np.full(100, 30.0) + rng.normal(0, 0.05, 100), gyro[:100], accel[:100])
    except calib.CalibrationError:
        pass
    else:
        raise AssertionError("a recording without a temperature change must be rejected")


# ---- Protocol -----------------------------------------------------------------------

def test_protocol():
    rec = Recording()
    assert rec.feed("# gyro_dps_per_lsb=0.5 accel_g_per_lsb=0.25 temp_lsb_per_c=100 temp_offset_c=20") == "INFO"
    assert rec.feed("# imu=ok clock=internal odr_hz=500.000 measured_hz=499.8") == "INFO"
    assert rec.feed("$RAW,10,2,4,-8,12,8,0,-4,600") == "RAW"
    assert rec.feed("$MAG,123,1.5,-2.5,3.5") == "MAG"
    assert rec.feed("$RAW,14,2,0,0,0,0,0,0,0") == "RAW"      # samples 12 and 13 never arrived
    assert rec.feed("cal set accel_bias 0.001 -0.002 0.003") == "CAL"
    assert rec.feed("$RAW,garbage") is None and rec.feed("OK") is None
    t, dt, gyro, accel, temp = rec.imu_arrays()
    assert np.allclose(gyro[0], [1.0, -2.0, 3.0]) and np.allclose(accel[0], [1.0, 0.0, -0.5])
    assert np.isclose(temp[0], 23.0) and np.isclose(dt[0], 2 / 500.0) and np.isclose(t[0], 11 / 500.0)
    assert rec.lost_samples == 2
    assert rec.calibration["accel_bias"] == [0.001, -0.002, 0.003]
    mag, acc, gyr = rec.mag_with_imu()
    assert np.allclose(mag[0], [1.5, -2.5, 3.5]) and np.allclose(acc[0], [0.5, 0.0, -0.25])


# ---- The command-line tool, end to end against a simulated board ---------------------

def run_tool(board: FakeBoard, *argv: str) -> int:
    """Run the real CLI with the board standing in for the serial port.

    The tool's own output is shown only if the test then fails.
    """
    import contextlib
    import io
    import serial
    original, output = serial.Serial, io.StringIO()
    serial.Serial = board
    try:
        with contextlib.redirect_stdout(output):
            code = cli.main(["--port", "FAKE", *argv])
    finally:
        serial.Serial = original
    run_tool.last_output = output.getvalue()
    return code


def test_tool_imucal():
    rng = np.random.default_rng(10)
    w, att = multi_position_motion(14, seed=11)
    up = att.inv().apply(UP_NAV)
    a_bias = np.array([0.010, 0.015, -0.020])
    accel = up + a_bias + rng.normal(0, 0.0011, up.shape)
    gyro = np.degrees(w) / 0.996 + np.array([0.2, -0.1, 0.3]) + rng.normal(0, 0.045, w.shape)
    board = FakeBoard(gyro, accel, np.full(len(w), 33.0))
    assert run_tool(board, "imucal", "--seconds", "3", "--yes") == 0
    saved = board.saved_cal
    assert saved is not None, "calibration was not saved"
    assert int(saved["flags"][0]) == 3, saved["flags"]              # accel + gyro
    assert np.abs(np.array(saved["accel_bias"]) - a_bias).max() < 0.0005
    assert np.abs(np.array(saved["gyro_bias"]) - [0.2, -0.1, 0.3]).max() < 0.005
    assert np.abs(np.diag(np.array(saved["gyro_matrix"]).reshape(3, 3)) - 0.996).max() < 0.002
    assert abs(saved["accel_tc_tref"][0] - 33.0) < 0.1
    # Streams are stopped and the previous attitude rate restored.
    assert board.raw_div == 0 and board.att_rate == 50, (board.raw_div, board.att_rate)
    print(f"  tool imucal: {len(board.commands)} commands, flags {int(saved['flags'][0])}")


def test_tool_magcal():
    rng = np.random.default_rng(12)
    t, w, att = smooth_tumble(150.0, 55.0, seed=13)
    field_imu = att.inv().apply(FIELD_NAV)
    mount = Rotation.from_euler("z", 180.0, degrees=True).as_matrix()
    hard = np.array([-15.0, 30.0, 8.0])
    raw_mag = field_imu @ mount.T + hard + rng.normal(0, 0.05, field_imu.shape)
    accel = att.inv().apply(UP_NAV) + rng.normal(0, 0.002, field_imu.shape)
    board = FakeBoard(np.degrees(w), accel, np.full(len(t), 30.0), mag_ut=raw_mag)
    board.cal["flags"] = [3.0]  # an IMU calibration is already there and must survive
    assert run_tool(board, "magcal", "--seconds", "3", "--yes") == 0
    saved = board.saved_cal
    assert saved is not None and int(saved["flags"][0]) == 7, saved and saved["flags"]
    matrix = np.array(saved["mag_matrix"]).reshape(3, 3)
    corrected = (raw_mag - np.array(saved["mag_bias"])) @ matrix.T
    angle = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", corrected, field_imu) /
                                         (np.linalg.norm(corrected, axis=1) * 50.0), -1, 1)))
    print(f"  tool magcal: direction error max {angle.max():.3f} deg, dip {saved['mag_dip'][0]:.1f} deg")
    assert angle.max() < 0.6 and abs(saved["mag_dip"][0] - 66.0) < 0.5

    # The same recording through --save-log and --from-log gives the same answer offline.
    import tempfile
    board2 = FakeBoard(np.degrees(w), accel, np.full(len(t), 30.0), mag_ut=raw_mag)
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "mag.log")
        assert run_tool(board2, "magcal", "--seconds", "3", "--yes", "--save-log", path) == 0
        offline = calib.calibrate_mag(*cli.load_log(path).mag_with_imu())
    assert np.allclose(offline.bias, board2.saved_cal["mag_bias"], atol=1e-3)


def test_tool_magcal_mirrored():
    """The INS10-A3 case through the tool: the saved calibration must point north."""
    rng = np.random.default_rng(15)
    t, w, att = smooth_tumble(150.0, 55.0, seed=16)
    field_imu = att.inv().apply(FIELD_NAV)
    up_imu = att.inv().apply(UP_NAV)
    raw_mag = field_imu @ MIRROR.T + np.array([49.8, -5.8, 1.3]) + rng.normal(0, 0.05, field_imu.shape)
    accel = up_imu + rng.normal(0, 0.002, field_imu.shape)
    board = FakeBoard(np.degrees(w), accel, np.full(len(t), 30.0), mag_ut=raw_mag)
    board.cal["flags"] = [3.0]
    assert run_tool(board, "magcal", "--seconds", "3", "--yes") == 0
    assert "mirror image" in run_tool.last_output, run_tool.last_output
    saved = board.saved_cal
    matrix = np.array(saved["mag_matrix"]).reshape(3, 3)
    corrected = (raw_mag - np.array(saved["mag_bias"])) @ matrix.T
    heading = heading_error_deg(corrected, field_imu, up_imu)
    print(f"  tool magcal, mirrored: det {np.linalg.det(matrix):+.3f}, dip {saved['mag_dip'][0]:.1f} deg, "
          f"heading error max {heading.max():.3f} deg")
    assert np.linalg.det(matrix) < 0 and abs(saved["mag_dip"][0] - 66.0) < 0.5 and heading.max() < 1.5


def test_tool_thermalcal_and_refusals():
    rng = np.random.default_rng(14)
    n = 256 * 400
    temp = 20.0 + 25.0 * np.arange(n) / n
    gyro = np.array([0.3, -0.2, 0.1]) + np.outer(temp - 30.0, [0.012, -0.009, 0.015]) + rng.normal(0, 0.045, (n, 3))
    accel = np.array([0.0, 0.0, 1.0]) + np.outer(temp - 30.0, [1e-4, -2e-4, 3e-4]) + rng.normal(0, 0.0011, (n, 3))
    board = FakeBoard(gyro, accel, temp)
    assert run_tool(board, "thermalcal", "--seconds", "3", "--yes") == 0
    saved = board.saved_cal
    assert saved is not None and int(saved["flags"][0]) == 24
    slopes = [saved[f"gyro_tc_{a}"][0] for a in "xyz"]
    print(f"  tool thermalcal: gyro slopes {np.round(slopes, 4)} dps/C")
    assert np.abs(np.array(slopes) - [0.012, -0.009, 0.015]).max() < 0.002

    # A recording with nothing usable in it is refused and writes nothing.
    still = FakeBoard(rng.normal(0, 0.045, (4000, 3)), np.array([0, 0, 1.0]) + rng.normal(0, 0.001, (4000, 3)),
                      np.full(4000, 25.0))
    assert run_tool(still, "imucal", "--seconds", "1", "--yes") == 1
    assert still.saved_cal is None and still.raw_div == 0


def test_tool_monitor():
    board = FakeBoard(np.zeros((8, 3)), np.zeros((8, 3)), np.full(8, 25.0))
    board.att_rate, board.att_lines, board.interrupt_after_att = 25, 3, True
    assert run_tool(board, "monitor", "--hz", "10") == 0
    assert "86.50" in run_tool.last_output and "aligned still heading" in run_tool.last_output
    # The rate the board had before is restored, so a later 'cal save' keeps it.
    assert board.att_rate == 25, board.att_rate
    print("  tool monitor: attitude line decoded, output rate restored")


TESTS = [test_protocol, test_magnetometer, test_magnetometer_mirrored, test_magnetometer_near_equator,
         test_magnetometer_rejects_poor_data, test_imu, test_imu_with_lost_lines, test_thermal,
         test_tool_imucal, test_tool_magcal, test_tool_magcal_mirrored,
         test_tool_thermalcal_and_refusals, test_tool_monitor]

if __name__ == "__main__":
    failed = 0
    for test in TESTS:
        print(f"[ RUN  ] {test.__name__}")
        try:
            test()
            print(f"[  OK  ] {test.__name__}")
        except Exception as error:  # noqa: BLE001 - report every failure, then exit non-zero
            failed += 1
            import traceback
            traceback.print_exc()
            print(getattr(run_tool, "last_output", ""))
            print(f"[ FAIL ] {test.__name__}: {error}")
    print(f"{len(TESTS)} test(s), {failed} failed")
    sys.exit(1 if failed else 0)
