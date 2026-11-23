"""A stand-in for the board's serial port, speaking the firmware's text protocol.

Lets the tests drive the real command-line tool end to end (serial framing, stream
capture, calibration upload) without hardware. It is installed in place of
serial.Serial; "sensor data" is whatever arrays the test supplies, delivered as fast
as the tool reads.
"""
from __future__ import annotations

import numpy as np

from ins10tool.records import DEFAULT_SCALES

DEFAULT_CAL = {
    "flags": [0.0],
    "gyro_bias": [0.0] * 3, "gyro_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
    "accel_bias": [0.0] * 3, "accel_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
    "mag_bias": [0.0] * 3, "mag_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
    "gyro_tc_tref": [25.0], "gyro_tc_x": [0.0] * 3, "gyro_tc_y": [0.0] * 3, "gyro_tc_z": [0.0] * 3,
    "accel_tc_tref": [25.0], "accel_tc_x": [0.0] * 3, "accel_tc_y": [0.0] * 3, "accel_tc_z": [0.0] * 3,
    "declination": [0.0], "mag_field": [0.0], "mag_dip": [0.0],
}


class FakeBoard:
    def __init__(self, gyro_dps, accel_g, temp_c, mag_ut=None, odr: float = 512.0, mag_every: int = 10):
        s = DEFAULT_SCALES
        self.gyro = np.round(np.asarray(gyro_dps) / s["gyro_dps_per_lsb"]).astype(np.int64)
        self.accel = np.round(np.asarray(accel_g) / s["accel_g_per_lsb"]).astype(np.int64)
        self.temp = np.round((np.asarray(temp_c) - s["temp_offset_c"]) * s["temp_lsb_per_c"]).astype(np.int64)
        self.mag = mag_ut
        self.odr, self.mag_every = odr, mag_every
        self.cal = {k: list(map(float, v)) for k, v in DEFAULT_CAL.items()}
        self.saved_cal = None
        self.raw_div, self.mag_stream, self.att_rate = 0, False, 50
        self.position = 0           # next sample to stream
        self.att_lines = 0          # $ATT lines still to send while the stream is on
        self.interrupt_after_att = False
        self.commands: list[str] = []
        self._rx = b""              # bytes typed by the host, not yet a full line
        self._tx = b""              # bytes waiting to be read by the host
        self.dtr = False

    # ---- serial.Serial interface used by the tool ------------------------------

    def __call__(self, *args, **kwargs):  # so an instance can replace the Serial class
        return self

    @property
    def in_waiting(self) -> int:
        return len(self._tx)

    def reset_input_buffer(self) -> None:
        self._tx = b""

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self._rx += data
        while b"\n" in self._rx:
            line, _, self._rx = self._rx.partition(b"\n")
            self._execute(line.decode("ascii").strip())
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if not self._tx:
            self._stream(40)
        out, self._tx = self._tx[:size], self._tx[size:]
        return out

    # ---- firmware behaviour ------------------------------------------------------

    def _say(self, *lines: str) -> None:
        self._tx += ("\n".join(lines) + "\n").encode("ascii")

    def _execute(self, text: str) -> None:
        self.commands.append(text)
        words = text.split()
        if text == "info":
            s = DEFAULT_SCALES
            self._say("# fw=ins10-ahrs version=test board=fake lib=test",
                      f"# imu=ok clock=external odr_hz={self.odr:.3f} measured_hz={self.odr:.3f}",
                      f"# gyro_dps_per_lsb={s['gyro_dps_per_lsb']:.9g} accel_g_per_lsb={s['accel_g_per_lsb']:.9g} "
                      f"temp_lsb_per_c={s['temp_lsb_per_c']:.6g} temp_offset_c={s['temp_offset_c']:.6g}",
                      f"# att_rate_hz={self.att_rate} raw_div={self.raw_div}", "OK")
        elif text == "cal show":
            self._say(*[f"cal set {k} " + " ".join(f"{v:.9g}" for v in vals) for k, vals in self.cal.items()], "OK")
        elif words[:2] == ["cal", "set"] and len(words) >= 4:
            name, values = words[2], [float(v) for v in words[3:]]
            if name not in self.cal or len(values) != len(self.cal[name]):
                self._say("ERR unknown field or wrong number of values")
            else:
                self.cal[name] = values
                self._say("OK")
        elif text == "cal save":
            self.saved_cal = {k: list(v) for k, v in self.cal.items()}
            self._say("OK")
        elif text == "stream off":
            self.raw_div, self.mag_stream, self.att_rate = 0, False, 0
            self._say("OK")
        elif words[:2] == ["stream", "raw"]:
            self.raw_div = int(words[2])
            self._say("OK")
        elif words[:2] == ["stream", "mag"]:
            self.mag_stream = words[2] == "1"
            self._say("OK")
        elif words[:2] == ["stream", "att"]:
            self.att_rate = int(words[2])
            self._say("OK")
        else:
            self._say("ERR unknown command; try 'help'")

    def _stream(self, blocks: int) -> None:
        if self.att_rate > 0 and self.att_lines > 0:
            self.att_lines -= 1
            self._say("$ATT,1000,1.000000,0.000000,0.000000,0.000000,1.50,-2.50,3.50,86.50,"
                      "0.2100,-0.1500,0.0900,0.03,0.21,31.40,00D")
        elif self.att_rate > 0 and self.interrupt_after_att:
            raise KeyboardInterrupt  # the user presses Ctrl+C
        if self.raw_div <= 0:
            return
        n = self.raw_div
        lines = []
        for _ in range(blocks):
            i0 = self.position
            if i0 + n > len(self.gyro):
                break
            if self.mag_stream and self.mag is not None:
                for k in range(i0, i0 + n):
                    if k % self.mag_every == 0:
                        m = self.mag[k]
                        lines.append(f"$MAG,{int(k / self.odr * 1000)},{m[0]:.3f},{m[1]:.3f},{m[2]:.3f}")
            sums = (list(self.gyro[i0:i0 + n].sum(axis=0)) + list(self.accel[i0:i0 + n].sum(axis=0)) +
                    [self.temp[i0:i0 + n].sum()])
            lines.append(f"$RAW,{i0},{n}," + ",".join(str(int(v)) for v in sums))
            self.position += n
        if lines:
            self._say(*lines)
