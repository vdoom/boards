"""Command-line interface of the INS10 host tool.

    python tools/ins10.py --help

Calibration commands record from the board, compute, show a report and ask before
writing anything. With --from-log they run on a saved recording instead.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time

import numpy as np

from . import calib
from .device import (CAL_ACCEL, CAL_ACCEL_TC, CAL_GYRO, CAL_GYRO_TC, CAL_MAG, Device, DeviceError,
                     find_ports)
from .records import Recording, load_log

FLAG_NAMES = ["aligned", "coarse", "still", "heading", "acc-disturbed", "mag-disturbed",
              "mag-uncal", "saturated", "thermal", "recovery", "mag-stale"]


# ---- Helpers ------------------------------------------------------------------------

def open_device(args) -> Device:
    return Device(args.port)


def confirm(question: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def fmt(values, digits: int = 6) -> str:
    return " ".join(f"{float(v):.{digits}g}" for v in np.ravel(values))


def print_warnings(warnings) -> None:
    for text in warnings:
        print(f"  ! {text}")


def tempco(cal: dict, prefix: str, temp: np.ndarray) -> np.ndarray:
    """Evaluate the stored temperature polynomial of 'gyro' or 'accel' at temp."""
    t_ref = cal.get(f"{prefix}_tc_tref", [25.0])[0]
    d = np.asarray(temp, dtype=float) - t_ref
    out = np.zeros((len(d), 3))
    for axis, name in enumerate("xyz"):
        coef = cal.get(f"{prefix}_tc_{name}", [0.0, 0.0, 0.0])
        out[:, axis] = coef[0] * d + coef[1] * d ** 2 + coef[2] * d ** 3
    return out


def capture(dev: Device, raw_div: int, mag: bool, seconds: float | None, progress,
            save_log: str | None) -> Recording:
    """Stream raw data from the board until Enter is pressed or `seconds` elapse."""
    rec = Recording()
    header = dev.info_lines()
    for line in header:
        rec.feed(line)
    previous_att = dev.info().get("att_rate_hz", "0")
    log = open(save_log, "w", encoding="utf-8") if save_log else None
    if log:
        log.write("\n".join(header) + "\n")
    stop = threading.Event()
    if seconds is None:
        threading.Thread(target=lambda: (sys.stdin.readline(), stop.set()), daemon=True).start()
    dev.command("stream off")
    dev.stream_lines.clear()
    if mag:
        dev.command("stream mag 1")
    dev.command(f"stream raw {raw_div}")
    start = last = time.monotonic()
    try:
        while not stop.is_set():
            now = time.monotonic()
            if seconds is not None and now - start >= seconds:
                break
            pending, dev.stream_lines = dev.stream_lines, []
            line = dev.read_line()
            for text in pending + ([line] if line is not None else []):
                rec.feed(text)
                if log:
                    log.write(text + "\n")
            if progress and now - last >= 0.5:
                last = now
                print("\r  " + progress(rec, now - start).ljust(76), end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dev.command("stream off")
            if previous_att != "0":
                dev.command(f"stream att {previous_att}")
        finally:
            if log:
                log.close()
    print()
    if rec.lost_samples:
        print(f"  note: {rec.lost_samples} IMU samples were lost on the way to the PC")
    return rec


def contiguity(rec: Recording) -> np.ndarray:
    seq, n = np.asarray(rec.imu_seq), np.asarray(rec.imu_n)
    return np.append(seq[1:] == seq[:-1] + n[:-1], True)


def recording_for(args, raw_div: int, mag: bool, instructions: str, progress):
    """A Recording and the device calibration it should be interpreted with."""
    if args.from_log:
        rec = load_log(args.from_log)
        print(f"Loaded {len(rec.imu_seq)} IMU rows and {len(rec.mag)} magnetometer rows from {args.from_log}")
        return rec, None, {}
    dev = open_device(args)
    current = dev.read_calibration()
    print(instructions)
    if args.seconds is None:
        print("Press Enter when done.")
    rec = capture(dev, raw_div, mag, args.seconds, progress, args.save_log)
    return rec, dev, current


def apply_fields(args, dev: Device | None, fields: dict, flags: int) -> None:
    print("\nCalibration fields:")
    for name, values in fields.items():
        print(f"  cal set {name} {fmt(values, 9)}")
    if dev is None:
        print("(offline: nothing written; send these lines with 'cal import' or a terminal)")
        return
    if not confirm("\nWrite to the board and save to flash?", args.yes):
        print("Not written.")
        return
    dev.write_calibration({k: list(np.ravel(v)) for k, v in fields.items()}, set_flags=flags)
    print("Written and saved.")


# ---- Commands -----------------------------------------------------------------------

def cmd_ports(args) -> int:
    ports = find_ports()
    print("\n".join(ports) if ports else "no Raspberry Pi Pico serial port found")
    return 0


def cmd_passthrough(args) -> int:
    with open_device(args) as dev:
        print("\n".join(dev.command(args.command)))
    return 0


def cmd_monitor(args) -> int:
    with open_device(args) as dev:
        previous = dev.info().get("att_rate_hz", "0")
        dev.command(f"stream att {args.hz}")
        print("Ctrl+C to stop.   roll    pitch      yaw  heading | sigma tilt/hdg | bias dps | temp | flags")
        try:
            while True:
                line = dev.read_line()
                if not line or not line.startswith("$ATT,"):
                    continue
                p = line.split(",")
                if len(p) != 17:
                    continue
                flags = int(p[16], 16)
                names = [n for bit, n in enumerate(FLAG_NAMES) if flags >> bit & 1]
                text = (f"{float(p[6]):8.2f} {float(p[7]):8.2f} {float(p[8]):8.2f} {float(p[9]):8.2f} | "
                        f"{float(p[13]):5.2f} {float(p[14]):5.2f} | {float(p[10]):+.3f} {float(p[11]):+.3f} "
                        f"{float(p[12]):+.3f} | {float(p[15]):5.1f} | {' '.join(names)}")
                print("\r" + text.ljust(118), end="", flush=True)
        except KeyboardInterrupt:
            print()
        finally:
            dev.command(f"stream att {previous}")  # leave the board as it was found
    return 0


def cmd_log(args) -> int:
    with open_device(args) as dev:
        print(f"Recording to {args.file}." + ("" if args.seconds else " Press Enter to stop."))
        rec = capture(dev, args.raw, not args.no_mag, args.seconds,
                      lambda r, t: f"{t:6.1f} s  {len(r.imu_seq)} IMU rows, {len(r.mag)} mag rows",
                      args.file)
        print(f"{len(rec.imu_seq)} IMU rows, {len(rec.mag)} magnetometer rows.")
    return 0


def cmd_imucal(args) -> int:
    def progress(rec: Recording, elapsed: float) -> str:
        t, dt, gyro, accel, temp = rec.imu_arrays()
        intervals = calib.find_static_intervals(t, dt, gyro, accel, temp)
        distinct = calib.count_orientations(np.array([iv.accel for iv in intervals])) if intervals else 0
        return f"{elapsed:5.0f} s  rest positions: {len(intervals)}, distinct orientations: {distinct}"

    rec, dev, current = recording_for(
        args, raw_div=2, mag=False, progress=progress,
        instructions=(
            "Accelerometer and gyro calibration.\n"
            "  1. Put the board down in 12 or more clearly different orientations: each face and\n"
            "     edge down, and a few tilted ones. A small box to lean it against helps.\n"
            "  2. Leave it completely still for 3-4 s in each, then turn it to the next in one\n"
            "     smooth motion. The turns calibrate the gyro, so vary the direction of turning.\n"
            "Rest positions are counted below as they are recognised."))
    t, dt, gyro, accel, temp = rec.imu_arrays()
    flags = int(current.get("flags", [0])[0])
    # Bias refers to the reference temperature of an existing temperature calibration.
    if flags & CAL_ACCEL_TC:
        accel = accel - tempco(current, "accel", temp)
    if flags & CAL_GYRO_TC:
        gyro = gyro - tempco(current, "gyro", temp)
    result = calib.calibrate_imu(t, dt, gyro, accel, temp, contiguity(rec))

    print(f"\n{result.positions} rest positions, {result.orientations} distinct orientations, {result.temp_c:.1f} C")
    print(f"Accelerometer: |a| error over the positions {result.accel_residual_before_mg:.2f} mg -> "
          f"{result.accel_residual_mg:.2f} mg")
    print(f"  bias [mg]   {fmt(result.accel_bias * 1000, 4)}")
    print(f"  scale       {fmt(np.diag(result.accel_matrix), 6)}")
    print(f"Gyro: bias [dps] {fmt(result.gyro_bias, 4)}")
    if result.gyro_matrix_fitted != "none":
        print(f"  gravity mismatch after the turns {result.gyro_residual_before_deg:.3f} deg -> "
              f"{result.gyro_residual_deg:.3f} deg ({result.gyro_matrix_fitted} matrix)")
        print(f"  scale       {fmt(np.diag(result.gyro_matrix), 6)}")
    print_warnings(result.warnings)

    fields = {"accel_bias": result.accel_bias, "accel_matrix": result.accel_matrix,
              "gyro_bias": result.gyro_bias}
    new_flags = CAL_ACCEL
    if result.gyro_matrix_fitted != "none":
        fields["gyro_matrix"] = result.gyro_matrix
        new_flags |= CAL_GYRO
    if not flags & CAL_ACCEL_TC:
        fields["accel_tc_tref"] = [result.temp_c]
    if not flags & CAL_GYRO_TC:
        fields["gyro_tc_tref"] = [result.temp_c]
    apply_fields(args, dev, fields, new_flags)
    return 0


def cmd_magcal(args) -> int:
    def progress(rec: Recording, elapsed: float) -> str:
        if len(rec.mag) < 20:
            return f"{elapsed:5.0f} s  waiting for magnetometer data"
        m = np.asarray(rec.mag)
        coverage = calib.sphere_coverage(m - 0.5 * (m.max(axis=0) + m.min(axis=0)))
        return f"{elapsed:5.0f} s  {len(m)} samples, directions covered: {coverage * 100:3.0f} % (aim for 80+)"

    rec, dev, current = recording_for(
        args, raw_div=8, mag=True, progress=progress,
        instructions=(
            "Magnetometer calibration. Do this with the board mounted as it will be used\n"
            "(its carrier, wires and nearby parts are part of what is calibrated), away from\n"
            "steel furniture, speakers, monitors and power cables.\n"
            "  Turn the board slowly through every orientation: several full turns about each\n"
            "  of its three axes, including upside down. About 60-90 s; slower than one turn\n"
            "  in 6 s, without shaking."))
    mag, accel, gyro = rec.mag_with_imu()
    flags = int(current.get("flags", [0])[0])
    if flags & CAL_ACCEL:
        bias = np.array(current["accel_bias"])
        matrix = np.array(current["accel_matrix"]).reshape(3, 3)
        accel = (accel - bias) @ matrix.T
    result = calib.calibrate_mag(mag, accel, gyro, southern=args.southern)
    if dev is not None and not flags & CAL_ACCEL:
        result.warnings.append(
            "the accelerometer is not calibrated yet. The magnetometer is aligned to its "
            "axes, so its bias (up to about a degree) goes into the alignment. Run imucal "
            "first, then magcal again.")

    print(f"\n{result.samples} samples, {result.coverage * 100:.0f} % of directions covered")
    print(f"Hard iron [uT]  {fmt(result.bias, 5)}")
    print(f"Field magnitude {result.field_ut:.2f} uT, spread after calibration {result.norm_residual_pct:.2f} %")
    if result.aligned:
        print(f"Aligned to the IMU frame; dip angle {result.dip_deg:.1f} deg "
              f"(consistent to {result.dip_residual_deg:.2f} deg)")
        if result.mirrored:
            print("The magnetometer's axes are a mirror image of the IMU's, as on the INS10-A3,\n"
                  "so the alignment is a reflection; the calibration includes it. The heading is\n"
                  "right if the hemisphere is: " + ("southern (--southern)." if args.southern else
                                                    "northern (pass --southern south of the magnetic equator)."))
    print_warnings(result.warnings)
    if not result.aligned:
        print("The result was not aligned to the IMU frame and will not be written.")
        return 1
    fields = {"mag_bias": result.bias, "mag_matrix": result.matrix,
              "mag_field": [result.field_ut], "mag_dip": [result.dip_deg]}
    apply_fields(args, dev, fields, CAL_MAG)
    return 0


def cmd_thermalcal(args) -> int:
    def progress(rec: Recording, elapsed: float) -> str:
        if not rec.temp:
            return "waiting for data"
        temp = np.asarray(rec.temp)
        return (f"{elapsed / 60:5.1f} min  {temp[-1]:.1f} C now, {temp.min():.1f}..{temp.max():.1f} C "
                f"recorded (span {np.ptp(temp):.1f} C)")

    rec, dev, current = recording_for(
        args, raw_div=256, mag=False, progress=progress,
        instructions=(
            "Temperature calibration of the gyro and accelerometer bias.\n"
            "  1. Cool the board (unpowered, in a closed bag in a fridge for 20 min works).\n"
            "  2. Put it on a solid surface, plug it in and do not touch it or the table.\n"
            "  3. Let it warm up for 15-30 min; warming it gently further with a hair dryer from\n"
            "     a distance widens the range. More than 15 C of range is worth having."))
    t, dt, gyro, accel, temp = rec.imu_arrays()
    flags = int(current.get("flags", [0])[0])
    accel_t_ref = current.get("accel_tc_tref", [None])[0] if flags & CAL_ACCEL else None
    result = calib.calibrate_thermal(temp, gyro, accel, accel_t_ref)

    print(f"\n{result.samples} samples, {result.t_min:.1f}..{result.t_max:.1f} C, polynomial order {result.order}")
    print(f"Gyro bias drift over the range {result.gyro_drift_dps:.3f} dps -> residual {result.gyro_residual_dps:.4f} dps")
    print(f"  bias at {result.t_ref:.0f} C [dps]  {fmt(result.gyro_bias, 4)}")
    print(f"  slope [dps/C]        {fmt(result.gyro_tc[:, 0], 3)}")
    print(f"Accel bias drift over the range {result.accel_drift_mg:.2f} mg -> residual {result.accel_residual_mg:.3f} mg")
    print(f"  slope [mg/C]         {fmt(result.accel_tc[:, 0] * 1000, 3)}")
    print_warnings(result.warnings)
    fields = {"gyro_bias": result.gyro_bias, "gyro_tc_tref": [result.t_ref],
              "accel_tc_tref": [result.accel_t_ref]}
    for axis, name in enumerate("xyz"):
        fields[f"gyro_tc_{name}"] = result.gyro_tc[axis]
        fields[f"accel_tc_{name}"] = result.accel_tc[axis]
    apply_fields(args, dev, fields, CAL_GYRO_TC | CAL_ACCEL_TC)
    return 0


def cmd_cal(args) -> int:
    with open_device(args) as dev:
        if args.action == "show":
            for name, values in dev.read_calibration().items():
                print(f"cal set {name} {fmt(values, 9)}")
        elif args.action == "export":
            with open(args.file, "w", encoding="utf-8") as handle:
                json.dump(dev.read_calibration(), handle, indent=2)
            print(f"Calibration written to {args.file}")
        elif args.action == "import":
            with open(args.file, "r", encoding="utf-8") as handle:
                text = handle.read()
            if text.lstrip().startswith("{"):
                fields = json.loads(text)
            else:  # a saved 'cal show' listing
                rec = Recording()
                for line in text.splitlines():
                    rec.feed(line)
                fields = rec.calibration
            if not fields:
                print("no calibration found in the file")
                return 1
            if confirm(f"Write {len(fields)} fields to the board and save to flash?", args.yes):
                dev.write_calibration(fields)
                print("Written and saved.")
        elif args.action == "erase":
            if confirm("Erase all calibration, the thermal model and settings?", args.yes):
                dev.command("cal erase", timeout=10.0)
                print("Erased.")
    return 0


# ---- Entry point --------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ins10", description="INS10 AHRS host tool")
    parser.add_argument("--port", help="serial port (default: the only Pico found)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="list Pico serial ports").set_defaults(func=cmd_ports)
    for name in ("info", "status"):
        sub.add_parser(name, help=f"print the board's '{name}' report").set_defaults(func=cmd_passthrough, command=name)

    p = sub.add_parser("monitor", help="live attitude display")
    p.add_argument("--hz", type=int, default=10)
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("log", help="record raw sensor data to a file")
    p.add_argument("file")
    p.add_argument("--seconds", type=float)
    p.add_argument("--raw", type=int, default=1, help="IMU samples summed per line (default 1)")
    p.add_argument("--no-mag", action="store_true")
    p.set_defaults(func=cmd_log)

    for name, func, text in (("imucal", cmd_imucal, "calibrate accelerometer and gyro (multi-position)"),
                             ("magcal", cmd_magcal, "calibrate the magnetometer and align it to the IMU"),
                             ("thermalcal", cmd_thermalcal, "calibrate bias drift over temperature")):
        p = sub.add_parser(name, help=text)
        p.add_argument("--from-log", help="compute from a saved recording instead of the board")
        p.add_argument("--save-log", help="also save the recording to this file")
        p.add_argument("--seconds", type=float, help="record for a fixed time instead of until Enter")
        p.add_argument("--yes", action="store_true", help="write without asking")
        if name == "magcal":
            p.add_argument("--southern", action="store_true",
                           help="calibrating south of the magnetic equator (the field points up)")
        p.set_defaults(func=func)

    p = sub.add_parser("cal", help="show, export, import or erase the stored calibration")
    p.add_argument("action", choices=["show", "export", "import", "erase"])
    p.add_argument("file", nargs="?")
    p.add_argument("--yes", action="store_true")
    p.set_defaults(func=cmd_cal)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "cal" and args.action in ("export", "import") and not args.file:
        print("a file name is required")
        return 2
    try:
        return args.func(args)
    except (DeviceError, calib.CalibrationError) as error:
        print(f"error: {error}")
        return 1
    except OSError as error:  # serial port problems
        print(f"error: {error}")
        return 1
