"""Serial connection to the INS10 AHRS firmware."""
from __future__ import annotations

import time

from .records import parse_cal_line, parse_info_line

PICO_USB_VID = 0x2E8A

# Calibration flag bits (lib/ins10/include/ins10/calib.h).
CAL_GYRO, CAL_ACCEL, CAL_MAG, CAL_GYRO_TC, CAL_ACCEL_TC = 1, 2, 4, 8, 16


class DeviceError(RuntimeError):
    pass


def find_ports() -> list[str]:
    from serial.tools import list_ports
    return [p.device for p in list_ports.comports() if p.vid == PICO_USB_VID]


class Device:
    """Line-oriented access: commands with OK/ERR replies, and stream records."""

    def __init__(self, port: str | None = None, timeout: float = 0.2):
        import serial
        if port is None:
            ports = find_ports()
            if not ports:
                raise DeviceError("no Raspberry Pi Pico serial port found; pass --port")
            if len(ports) > 1:
                raise DeviceError(f"several Pico ports found ({', '.join(ports)}); pass --port")
            port = ports[0]
        self.port = port
        # DTR tells the firmware that a console is attached; the baud rate is ignored
        # by USB CDC.
        self.serial = serial.Serial(port, 115200, timeout=timeout, write_timeout=2.0)
        self.serial.dtr = True
        self._buffer = b""
        self.stream_lines: list[str] = []  # stream records seen while waiting for a reply
        time.sleep(0.1)
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def read_line(self) -> str | None:
        """One complete line without its terminator, or None on timeout."""
        while b"\n" not in self._buffer:
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if not chunk:
                return None
            self._buffer += chunk
        line, _, self._buffer = self._buffer.partition(b"\n")
        return line.decode("ascii", errors="replace").rstrip("\r")

    def command(self, text: str, timeout: float = 5.0) -> list[str]:
        """Send a command; return its reply lines (without the final OK)."""
        self.serial.write(text.encode("ascii") + b"\n")
        self.serial.flush()
        reply: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.read_line()
            if line is None:
                continue
            if line == "OK" or line.startswith("OK "):
                return reply
            if line.startswith("ERR"):
                raise DeviceError(f"'{text}': {line[3:].strip() or 'error'}")
            if line.startswith("$"):
                self.stream_lines.append(line)  # an output stream is running
            else:
                reply.append(line)
        raise DeviceError(f"'{text}': no reply within {timeout:.0f} s (is this the AHRS firmware?)")

    def info(self) -> dict[str, str]:
        out: dict[str, str] = {}
        for line in self.command("info"):
            out.update(parse_info_line(line))
        return out

    def info_lines(self) -> list[str]:
        return [line for line in self.command("info") if line.startswith("#")]

    def read_calibration(self) -> dict[str, list[float]]:
        cal: dict[str, list[float]] = {}
        for line in self.command("cal show"):
            parsed = parse_cal_line(line)
            if parsed:
                cal[parsed[0]] = parsed[1]
        return cal

    def write_calibration(self, fields: dict[str, list[float]], set_flags: int = 0,
                          save: bool = True) -> None:
        """Set calibration fields, OR `set_flags` into the flag word, optionally save."""
        for name, values in fields.items():
            if name == "flags":
                continue
            self.command("cal set " + name + " " + " ".join(f"{float(v):.9g}" for v in values))
        if "flags" in fields:
            self.command(f"cal set flags {int(fields['flags'][0])}")
        if set_flags:
            flags = int(self.read_calibration().get("flags", [0])[0]) | set_flags
            self.command(f"cal set flags {flags}")
        if save:
            self.command("cal save", timeout=10.0)
