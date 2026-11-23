"""Calibration algorithms. Pure numpy/scipy: no serial port, testable offline.

Every result is expressed in the firmware's calibration model

    corrected = matrix @ (raw - bias - tempco(T))

with raw in the sensor's unit (gyro dps, accel g, mag uT).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class CalibrationError(RuntimeError):
    pass


# ---------------------------------------------------------------------------------------
# Magnetometer: ellipsoid fit (hard and soft iron), then alignment to the IMU frame
# ---------------------------------------------------------------------------------------

@dataclass
class MagCalibration:
    bias: np.ndarray            # uT, hard iron
    matrix: np.ndarray          # 3x3: soft iron and rotation into the IMU frame
    field_ut: float             # local field magnitude implied by the fit
    dip_deg: float | None       # positive when the field points down; None if not aligned
    norm_residual_pct: float    # 1-sigma of |B| after calibration, percent of the field
    dip_residual_deg: float | None
    coverage: float             # fraction of the sphere of directions that was sampled
    samples: int
    aligned: bool
    warnings: list[str] = field(default_factory=list)
    mirrored: bool = False      # aligned with a reflection: det(matrix) < 0


# Below this |sin dip| (about 10 deg from the magnetic equator) the field's vertical
# component is too small to tell which way it points, so the hemisphere cannot pick
# the sign of the alignment. Same value as INS10 Studio's kMinSinDipForSign.
MIN_SIN_DIP_FOR_SIGN = 0.17


def _sym(p):
    return np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])


def fit_ellipsoid(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Algebraic least-squares ellipsoid through the points x (n, 3).

    Returns (center, A) with A symmetric positive definite and |A (x - center)| = 1.
    """
    c0 = x.mean(axis=0)
    scale = float(np.sqrt(((x - c0) ** 2).sum(axis=1).mean()))
    if not scale > 0:
        raise CalibrationError("magnetometer data has no spread")
    y = (x - c0) / scale
    design = np.column_stack([y[:, 0] ** 2, y[:, 1] ** 2, y[:, 2] ** 2,
                              2 * y[:, 0] * y[:, 1], 2 * y[:, 0] * y[:, 2], 2 * y[:, 1] * y[:, 2],
                              2 * y[:, 0], 2 * y[:, 1], 2 * y[:, 2], np.ones(len(y))])
    p = np.linalg.svd(design, full_matrices=False)[2][-1]
    q, u, k = _sym(p[:6]), p[6:9], p[9]
    if np.trace(q) < 0:
        q, u, k = -q, -u, -k
    if np.linalg.eigvalsh(q).min() <= 0:
        raise CalibrationError("the samples do not describe an ellipsoid: rotate the board "
                               "through many more orientations")
    center = -np.linalg.solve(q, u)
    gain = center @ q @ center - k
    if gain <= 0:
        raise CalibrationError("degenerate ellipsoid fit")
    w, v = np.linalg.eigh(q / gain)
    a = (v * np.sqrt(w)) @ v.T
    return c0 + scale * center, a / scale


def sphere_coverage(directions: np.ndarray, bands: int = 6, sectors: int = 12) -> float:
    """Fraction of equal-area cells of the unit sphere hit by the given directions."""
    d = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    band = np.clip(((d[:, 2] + 1) * 0.5 * bands).astype(int), 0, bands - 1)
    sector = ((np.arctan2(d[:, 1], d[:, 0]) + np.pi) / (2 * np.pi) * sectors).astype(int) % sectors
    return len(set(zip(band.tolist(), sector.tolist()))) / (bands * sectors)


def align_to_accel(mag_unit: np.ndarray, accel: np.ndarray,
                   southern: bool = False) -> tuple[np.ndarray, float, float, float]:
    """Orthogonal R taking calibrated field directions into the accelerometer frame.

    In every orientation the angle between gravity and the earth's field is the same,
    so  a_i . (R m_i) = const  over the whole recording. That fixes R up to a common
    sign of (R, const). The field's direction picks the sign: it points down in the
    northern hemisphere (const = a . B < 0) and up in the southern one. R is then a
    reflection (det -1) when the magnetometer's axes are a mirror image of the IMU's,
    as on the INS10-A3, where the MMC5983MA's Z axis points into the board. Choosing
    the proper rotation instead would turn the field upside down there and put the
    heading 180 deg out. Near the magnetic equator the sign of const says nothing, and
    the proper rotation is kept.

    Returns (R, sin_dip, residual_deg, conditioning) with sin_dip positive when the
    field points down.
    """
    ah = accel / np.linalg.norm(accel, axis=1, keepdims=True)
    n = len(ah)
    design = np.column_stack([(ah[:, :, None] * mag_unit[:, None, :]).reshape(n, 9), -np.ones(n)])
    p = np.linalg.svd(design, full_matrices=False)[2][-1]
    u, s, vt = np.linalg.svd(p[:9].reshape(3, 3))
    conditioning = float(s.min() / s.max())
    r0, const = u @ vt, p[9] / s.mean()
    if abs(const) < MIN_SIN_DIP_FOR_SIGN:
        flip = np.linalg.det(r0) < 0
    else:
        flip = (const > 0) != southern
    if flip:
        r0, const = -r0, -const

    def residual(params):
        r = r0 @ Rotation.from_rotvec(params[:3]).as_matrix()
        return np.einsum("ij,ij->i", ah, mag_unit @ r.T) - params[3]

    fit = least_squares(residual, np.array([0.0, 0.0, 0.0, const]), loss="soft_l1", f_scale=0.03)
    r = r0 @ Rotation.from_rotvec(fit.x[:3]).as_matrix()
    const = float(np.clip(fit.x[3], -1.0, 1.0))
    res = residual(fit.x)
    cos_dip = max(np.sqrt(1.0 - const * const), 1e-3)
    residual_deg = float(np.degrees(np.sqrt(np.median(res ** 2) * 2.198) / cos_dip))  # robust rms
    return r, -const, residual_deg, conditioning


def calibrate_mag(mag: np.ndarray, accel: np.ndarray | None = None,
                  gyro: np.ndarray | None = None, southern: bool = False) -> MagCalibration:
    """Hard iron, soft iron and (given accel) alignment to the IMU frame.

    mag (n, 3) in uT. accel (n, 3) in g and gyro (n, 3) in dps are the IMU readings at
    the same moments; only samples taken in slow, smooth motion are used for alignment.
    southern: the recording was made south of the magnetic equator (the field points
    up there); it decides the sign of the alignment, see align_to_accel.
    """
    mag = np.asarray(mag, dtype=float)
    if len(mag) < 200:
        raise CalibrationError(f"only {len(mag)} magnetometer samples; record at least 20 s")
    warnings: list[str] = []
    center, a = fit_ellipsoid(mag)
    field_ut = float(np.linalg.det(a) ** (-1.0 / 3.0))

    # Geometric refinement with a robust loss: the algebraic fit is biased by noise
    # and a few samples taken next to something magnetic should not bend the result.
    def unpack(params):
        return params[:3], _sym(params[3:9])

    def residual(params):
        c, m = unpack(params)
        return np.linalg.norm((mag - c) @ m.T, axis=1) - field_ut

    a0 = a * field_ut
    start = np.concatenate([center, [a0[0, 0], a0[1, 1], a0[2, 2], a0[0, 1], a0[0, 2], a0[1, 2]]])
    fit = least_squares(residual, start, loss="soft_l1", f_scale=0.02 * field_ut)
    center, a = unpack(fit.x)
    if np.linalg.eigvalsh(a).min() <= 0:
        raise CalibrationError("soft-iron matrix is not positive definite; record again")
    norm = np.linalg.det(a) ** (1.0 / 3.0)  # keep the average gain at one
    a, field_ut = a / norm, field_ut / norm
    calibrated = (mag - center) @ a.T
    radius = np.linalg.norm(calibrated, axis=1)
    norm_residual = float(np.sqrt(np.median((radius - field_ut) ** 2) * 2.198) / field_ut * 100.0)

    coverage = sphere_coverage(calibrated)
    if coverage < 0.6:
        warnings.append(f"only {coverage * 100:.0f} % of all directions were sampled; the "
                        "soft-iron estimate is weak. Rotate about all three axes, including "
                        "upside down.")
    if not 20.0 < field_ut < 70.0:
        warnings.append(f"field magnitude {field_ut:.1f} uT is outside the earth's range "
                        "(22-67 uT): strong magnetic material nearby?")
    if norm_residual > 2.0:
        warnings.append(f"residual {norm_residual:.1f} % is high: the field was not uniform "
                        "(moving near steel, cables, magnets) or the sensor is saturating.")

    matrix, dip, dip_residual, aligned, mirrored = a, None, None, False, False
    if accel is not None and len(accel) == len(mag):
        accel = np.asarray(accel, dtype=float)
        an = np.linalg.norm(accel, axis=1)
        keep = np.abs(an / np.median(an) - 1.0) < 0.03
        if gyro is not None:
            keep &= np.linalg.norm(np.asarray(gyro, dtype=float), axis=1) < 60.0
        keep &= np.abs(radius / field_ut - 1.0) < 0.05
        if keep.sum() < 150:
            warnings.append("too few samples in slow, smooth motion to align the magnetometer "
                            "to the IMU: turn the board more slowly.")
        else:
            r, sin_dip, dip_residual, conditioning = align_to_accel(calibrated[keep] / field_ut, accel[keep],
                                                                    southern)
            if conditioning < 0.6:
                warnings.append("the alignment to the IMU is poorly determined by this "
                                "recording; it was not applied.")
            else:
                matrix, aligned = r @ a, True
                mirrored = bool(np.linalg.det(r) < 0)
                dip = float(np.degrees(np.arcsin(np.clip(sin_dip, -1.0, 1.0))))
                if abs(sin_dip) < MIN_SIN_DIP_FOR_SIGN:
                    warnings.append(f"the dip angle is only {dip:.1f} deg, too close to the magnetic "
                                    "equator to tell which way the field points, so the alignment "
                                    "was taken as a rotation. On a board whose magnetometer axes are "
                                    "mirrored (the INS10-A3) the heading is then 180 deg out; check it "
                                    "against a compass.")
                if dip_residual > 2.0:
                    warnings.append(f"dip angle varies by {dip_residual:.1f} deg over the "
                                    "recording: motion too fast or field not uniform.")
    return MagCalibration(center, matrix, field_ut, dip, norm_residual, dip_residual, coverage,
                          len(mag), aligned, warnings, mirrored)


# ---------------------------------------------------------------------------------------
# Accelerometer and gyroscope: multi-position calibration
# ---------------------------------------------------------------------------------------

@dataclass
class StaticInterval:
    start: int          # first IMU row
    stop: int           # one past the last row
    duration: float     # s
    accel: np.ndarray   # mean, g
    gyro: np.ndarray    # mean, dps
    temp: float


def find_static_intervals(t, dt, gyro, accel, temp, window_s: float = 0.5,
                          gyro_std_dps: float = 0.3, accel_std_g: float = 0.006,
                          min_duration_s: float = 1.5) -> list[StaticInterval]:
    """Periods during which the board lay still, with the mean readings of each."""
    t = np.asarray(t, dtype=float)
    if len(t) < 8:
        return []
    edges = np.arange(t[0], t[-1] + window_s, window_s)
    index = np.searchsorted(t, edges)
    windows = []
    for i0, i1 in zip(index[:-1], index[1:]):
        if i1 - i0 < 3:
            windows.append((i0, i1, False))
            continue
        quiet = (gyro[i0:i1].std(axis=0).max() < gyro_std_dps and
                 accel[i0:i1].std(axis=0).max() < accel_std_g and
                 np.abs(gyro[i0:i1].mean(axis=0)).max() < 5.0)
        windows.append((i0, i1, bool(quiet)))
    intervals: list[StaticInterval] = []
    run: list[tuple[int, int]] = []

    def close_run():
        # Drop the first and last window: they contain the settling and the pick-up.
        if len(run) >= 4:
            i0, i1 = run[1][0], run[-2][1]
            duration = float(dt[i0:i1].sum())
            if duration >= min_duration_s:
                w = dt[i0:i1] / dt[i0:i1].sum()
                intervals.append(StaticInterval(int(i0), int(i1), duration, w @ accel[i0:i1],
                                                w @ gyro[i0:i1], float(w @ temp[i0:i1])))
        run.clear()

    for i0, i1, quiet in windows:
        if quiet and run and np.abs(accel[i0:i1].mean(axis=0) - accel[run[0][0]:run[0][1]].mean(axis=0)).max() > 0.01:
            close_run()  # still, but in a new orientation: a slow slide between two rests
        if quiet:
            run.append((i0, i1))
        else:
            close_run()
    close_run()
    return intervals


def count_orientations(directions: np.ndarray, min_angle_deg: float = 25.0) -> int:
    """Number of mutually distinct directions among the given vectors."""
    unit = directions / np.linalg.norm(directions, axis=1, keepdims=True)
    chosen: list[np.ndarray] = []
    limit = np.cos(np.radians(min_angle_deg))
    for d in unit:
        if all(d @ c < limit for c in chosen):
            chosen.append(d)
    return len(chosen)


@dataclass
class ImuCalibration:
    accel_bias: np.ndarray
    accel_matrix: np.ndarray
    accel_residual_mg: float        # 1-sigma of |a| - 1 g over the positions, after
    accel_residual_before_mg: float
    gyro_bias: np.ndarray           # dps
    gyro_matrix: np.ndarray
    gyro_residual_deg: float | None         # gravity-direction mismatch after the turns
    gyro_residual_before_deg: float | None
    gyro_matrix_fitted: str         # "full", "scale" or "none"
    positions: int
    orientations: int
    temp_c: float
    warnings: list[str] = field(default_factory=list)


def calibrate_accel(means: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float, int]:
    """Bias and scale/cross-axis matrix from the mean readings in several rest positions.

    At rest the corrected reading has magnitude 1 g in every orientation; with enough
    orientations that determines the bias and a symmetric matrix. Returns
    (bias, matrix, residual_mg, residual_before_mg, distinct_orientations).
    """
    means = np.asarray(means, dtype=float)
    orientations = count_orientations(means)
    if orientations < 6:
        raise CalibrationError(f"only {orientations} distinct orientations found; at least 6 "
                               "are needed (each face of the board up), 12 or more are better")
    full = orientations >= 9

    def unpack(params):
        m = np.diag(params[3:6])
        if full:
            m[0, 1] = m[1, 0] = params[6]
            m[0, 2] = m[2, 0] = params[7]
            m[1, 2] = m[2, 1] = params[8]
        return params[:3], m

    def residual(params):
        b, m = unpack(params)
        return np.linalg.norm((means - b) @ m.T, axis=1) - 1.0

    start = np.array([0, 0, 0, 1, 1, 1] + ([0, 0, 0] if full else []), dtype=float)
    before = float(np.sqrt(np.mean(residual(start) ** 2)) * 1000.0)
    fit = least_squares(residual, start)
    bias, matrix = unpack(fit.x)
    after = float(np.sqrt(np.mean(fit.fun ** 2)) * 1000.0)
    if np.abs(bias).max() > 0.15 or np.abs(np.diag(matrix) - 1.0).max() > 0.1:
        raise CalibrationError("accelerometer calibration is far outside the sensor's "
                               "tolerance; were the rest positions really at rest?")
    return bias, matrix, after, before, orientations


def _chain(rot: Rotation) -> Rotation:
    """Ordered product r0 * r1 * ... of a stack of rotations, by pairwise reduction."""
    while len(rot) > 1:
        n = len(rot)
        even = n - (n % 2)
        pairs = rot[0:even:2] * rot[1:even:2]
        rot = Rotation.concatenate([pairs, rot[even:n]]) if n % 2 else pairs
    return rot[0]


def calibrate_gyro(dt, gyro, intervals: list[StaticInterval], accel_bias, accel_matrix,
                   contiguous: np.ndarray | None = None):
    """Gyro bias from the rest positions; scale and misalignment from the turns between.

    After Tedaldi, Pretto and Menegatti (ICRA 2014). Between two rest positions the
    integrated, corrected gyro must turn the gravity direction measured at the first
    into the one measured at the second. The accelerometer is the reference, so the
    resulting gyro axes are aligned with the calibrated accelerometer axes.

    contiguous[i] is False where IMU rows i and i+1 are not consecutive (lost data);
    turns containing such a break are skipped.

    Returns (bias, matrix, residual_deg, residual_before_deg, mode).
    """
    weights = np.array([iv.duration for iv in intervals])
    bias = (weights[:, None] * np.array([iv.gyro for iv in intervals])).sum(axis=0) / weights.sum()
    up = np.array([(iv.accel - accel_bias) @ np.asarray(accel_matrix).T for iv in intervals])
    up /= np.linalg.norm(up, axis=1, keepdims=True)

    turns = []
    for k in range(len(intervals) - 1):
        # Integrate from the middle of one rest to the middle of the next: the gyro
        # reads only its bias there, so the end points do not matter.
        i0 = (intervals[k].start + intervals[k].stop) // 2
        i1 = (intervals[k + 1].start + intervals[k + 1].stop) // 2
        if contiguous is not None and not contiguous[i0:i1].all():
            continue
        angle = np.degrees(np.arccos(np.clip(up[k] @ up[k + 1], -1, 1)))
        if angle < 20.0:
            continue  # nothing to learn from a turn that barely moved gravity
        turns.append((i0, i1, up[k], up[k + 1]))
    if len(turns) < 6:
        return bias, np.eye(3), None, None, "none"

    def residual_for(matrix):
        out = []
        for i0, i1, u0, u1 in turns:
            rate = (gyro[i0:i1] - bias) @ matrix.T
            total = _chain(Rotation.from_rotvec(np.radians(rate) * dt[i0:i1, None]))
            out.append(total.inv().apply(u0) - u1)
        return np.concatenate(out)

    def rms_deg(res):
        return float(np.degrees(np.sqrt(np.mean(np.sum(res.reshape(-1, 3) ** 2, axis=1)))))

    before = rms_deg(residual_for(np.eye(3)))
    full = least_squares(lambda p: residual_for(p.reshape(3, 3)), np.eye(3).ravel())
    sv = np.linalg.svd(full.jac, compute_uv=False)
    matrix, mode, res = full.x.reshape(3, 3), "full", full.fun
    if len(turns) < 9 or sv.min() < 1e-3 * sv.max() or np.abs(matrix - np.eye(3)).max() > 0.05:
        # Not enough variety of rotation axes for nine parameters: scale factors only.
        diag = least_squares(lambda p: residual_for(np.diag(p)), np.ones(3))
        sv = np.linalg.svd(diag.jac, compute_uv=False)
        matrix, mode, res = np.diag(diag.x), "scale", diag.fun
        if sv.min() < 1e-2 * sv.max() or np.abs(diag.x - 1.0).max() > 0.05:
            return bias, np.eye(3), before, before, "none"
    return bias, matrix, rms_deg(res), before, mode


def calibrate_imu(t, dt, gyro, accel, temp, contiguous: np.ndarray | None = None) -> ImuCalibration:
    intervals = find_static_intervals(t, dt, gyro, accel, temp)
    if len(intervals) < 6:
        raise CalibrationError(f"found {len(intervals)} rest positions; at least 6 are needed. "
                               "Leave the board untouched for 3 s in each position.")
    accel_means = np.array([iv.accel for iv in intervals])
    a_bias, a_matrix, a_res, a_before, orientations = calibrate_accel(accel_means)
    g_bias, g_matrix, g_res, g_before, mode = calibrate_gyro(dt, gyro, intervals, a_bias, a_matrix, contiguous)
    weights = np.array([iv.duration for iv in intervals])
    temp_c = float(weights @ np.array([iv.temp for iv in intervals]) / weights.sum())
    warnings: list[str] = []
    if orientations < 9:
        warnings.append(f"{orientations} distinct orientations: accelerometer scale and bias "
                        "were fitted, cross-axis terms were not (needs 9 or more).")
    if mode == "none":
        warnings.append("gyro scale factors were not fitted: too few usable turns between "
                        "rest positions, or too little variety of rotation axes.")
    elif mode == "scale":
        warnings.append("gyro scale factors were fitted, misalignment was not (needs more "
                        "turns about different axes).")
    if a_res > 3.0:
        warnings.append(f"accelerometer residual {a_res:.1f} mg is high: was the board moving "
                        "or vibrating in some positions?")
    return ImuCalibration(a_bias, a_matrix, a_res, a_before, g_bias, g_matrix, g_res, g_before,
                          mode, len(intervals), orientations, temp_c, warnings)


# ---------------------------------------------------------------------------------------
# Temperature: bias drift from a stationary warm-up or cool-down recording
# ---------------------------------------------------------------------------------------

@dataclass
class ThermalCalibration:
    t_ref: float
    gyro_bias: np.ndarray       # dps at t_ref
    gyro_tc: np.ndarray         # (3 axes, 3 terms): dps per degC^(k+1)
    accel_tc: np.ndarray        # g per degC^(k+1)
    accel_t_ref: float
    order: int
    t_min: float
    t_max: float
    gyro_residual_dps: float
    gyro_drift_dps: float       # peak-to-peak over the recording, before compensation
    accel_residual_mg: float
    accel_drift_mg: float
    samples: int
    warnings: list[str] = field(default_factory=list)


def _polyfit_clipped(d: np.ndarray, y: np.ndarray, order: int):
    """Least-squares polynomial in d per column of y, with one round of 4-sigma clipping."""
    x = np.vander(d, order + 1, increasing=True)
    keep = np.ones(len(d), dtype=bool)
    for _ in range(2):
        coef = np.linalg.lstsq(x[keep], y[keep], rcond=None)[0]
        res = y - x @ coef
        sigma = np.sqrt(np.median(res[keep] ** 2, axis=0) * 2.198) + 1e-12
        keep = (np.abs(res) < 4.0 * sigma).all(axis=1)
    return coef, res[keep]


def calibrate_thermal(temp, gyro, accel, accel_t_ref: float | None = None) -> ThermalCalibration:
    """Bias-versus-temperature polynomials from a recording made with the board at rest.

    The gyro reads its bias directly. The accelerometer reads gravity plus its bias,
    and gravity is constant in a fixed orientation, so the change of the reading with
    temperature is the bias drift; it is expressed relative to accel_t_ref, the
    temperature at which the accelerometer bias was calibrated.
    """
    temp = np.asarray(temp, dtype=float)
    gyro = np.asarray(gyro, dtype=float)
    accel = np.asarray(accel, dtype=float)
    if len(temp) < 30:
        raise CalibrationError("recording too short")
    warnings: list[str] = []
    span = float(np.ptp(temp))
    if span < 3.0:
        raise CalibrationError(f"temperature changed by only {span:.1f} C; at least 3 C is "
                               "needed, 15 C or more is better (start from a cold board)")
    order = 1 if span < 8.0 else 2 if span < 20.0 else 3
    if order < 3:
        warnings.append(f"temperature span {span:.1f} C: fitted a polynomial of order {order}. "
                        "Outside the recorded range the compensation is an extrapolation.")
    if np.linalg.norm(np.abs(gyro - np.median(gyro, axis=0)), axis=1).max() > 2.0:
        warnings.append("the board moved during the recording; those samples were rejected.")
    t_ref = float(np.round(np.median(temp)))
    g_coef, g_res = _polyfit_clipped(temp - t_ref, gyro, order)
    if accel_t_ref is None:
        accel_t_ref = t_ref
    a_coef, a_res = _polyfit_clipped(temp - accel_t_ref, accel, order)

    def terms(coef):
        out = np.zeros((3, 3))
        out[:, :order] = coef[1:order + 1].T
        return out

    return ThermalCalibration(
        t_ref=t_ref, gyro_bias=g_coef[0], gyro_tc=terms(g_coef), accel_tc=terms(a_coef),
        accel_t_ref=float(accel_t_ref), order=order, t_min=float(temp.min()), t_max=float(temp.max()),
        gyro_residual_dps=float(np.sqrt(np.mean(g_res ** 2))),
        gyro_drift_dps=float(np.ptp(np.vander(temp - t_ref, order + 1, increasing=True) @ g_coef, axis=0).max()),
        accel_residual_mg=float(np.sqrt(np.mean(a_res ** 2)) * 1000.0),
        accel_drift_mg=float(np.ptp(np.vander(temp - accel_t_ref, order + 1, increasing=True) @ a_coef, axis=0).max() * 1000.0),
        samples=len(temp), warnings=warnings)
