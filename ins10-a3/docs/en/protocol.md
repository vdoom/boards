# INS10 USB protocol (ins10_ahrs firmware 2.0)

The `ins10_ahrs` firmware talks to the PC in plain text lines over USB CDC (a virtual serial port). INS10 Studio, the Python tool `tools/ins10.py` and your own programs all use this protocol.

[Українська версія](../../firmware/docs/PROTOCOL.md) · [User guide](user-guide.md) · [Datasheet](datasheet.md)

## Basics

- The baud rate does not matter (it is USB). The terminal must assert **DTR**; most do by default, and pyserial does.
- A **command** is one line ending in `\n` or `\r`.
- A **reply** is zero or more lines, then exactly one line `OK` or `ERR <reason>`.
- Lines starting with `#` are information, usually as `key=value` pairs that are easy to parse.
- Lines starting with `$` are **stream records**. They may appear between the lines of a reply.
- If the PC does not read fast enough, the firmware drops output rather than slowing the sensor loop.
- When a terminal opens the port, the firmware prints a greeting: `# INS10 AHRS 2.0.0 (pico), library 2.0.0. Type 'help'.`

The firmware keeps an unfinished input line between port openings. If another program left half a command behind, send an empty line first.

## Commands

| Command | Action |
|---|---|
| `help` | List the commands |
| `info` | Version, board, sensor status, IMU clock source and rate, raw data scales, active streams, reason for the last reset (`last_reset=watchdog` means the main loop hung), calibration flags |
| `status` | Filter flags, angles, uncertainties, gyro bias, acceleration, field, temperature, barometer, sensor and filter health counters, processor load |
| `stream att <hz>` | `$ATT` rate, 0–200 Hz. 0 turns it off |
| `stream raw <n>` | `$RAW` records: sum of `n` IMU samples per line, 1–1024. 1 = every sample, 0 = off |
| `stream mag <0\|1>` | `$MAG` records on or off |
| `stream baro <0\|1>` | `$BARO` records on or off |
| `stream off` | All streams off (including `$ATT`) |
| `cal show` | The calibration as `cal set …` lines; saved output can be sent back unchanged |
| `cal set <field> <values…>` | Change one field in RAM. Takes effect at once without restarting the filter; goes to flash only with `cal save` |
| `cal save` | Write calibration, learned temperature model and settings to flash |
| `cal load` | Discard changes and reload from flash (`ERR nothing stored` if flash is empty) |
| `cal erase` | Factory values, clear the flash, restart the filter |
| `thermal show` | The learned gyro-bias-versus-temperature model |
| `thermal reset` | Forget the learned model; the filter restarts from initial alignment |
| `set mag <0\|1>` | Use the magnetometer or not. Stored with `cal save` |
| `reset` | Restart the filter from initial alignment |
| `reboot` | Restart the Pico |
| `bootsel` | Enter the USB bootloader for flashing |

`cal save` also stores the `$ATT` rate in force at that moment as the power-up rate (default 50 Hz). Programs that change the rate temporarily should restore it before `cal save`; INS10 Studio does this automatically.

## Streams

### `$ATT`: orientation

```text
$ATT,t_ms,qw,qx,qy,qz,roll,pitch,yaw,heading,bx,by,bz,tilt_sigma,heading_sigma,temp,flags
```

| Field | Unit | Meaning |
|---|---|---|
| `t_ms` | ms | Time since the Pico started |
| `qw … qz` | — | Unit quaternion, body frame → east-north-up |
| `roll`, `pitch`, `yaw` | ° | Euler angles, yaw-pitch-roll order |
| `heading` | ° | Compass heading of the body X axis, 0–360, clockwise from north |
| `bx … bz` | °/s | Estimated gyro bias |
| `tilt_sigma`, `heading_sigma` | ° | Filter 1σ for tilt (per axis) and heading |
| `temp` | °C | IMU die temperature, filtered |
| `flags` | hex | Status bits, see below |

**Frames.** The navigation frame is east-north-up (ENU). The body frame is the IMU's: X right, Y away from J1, Z up out of the component side ([datasheet](datasheet.md#axes)). Lying flat, components up: roll and pitch are 0 and the accelerometer reads +1 g on Z. Yaw is positive counter-clockwise seen from above and zero when X points east. `heading` is the usual compass heading. North is magnetic until you set `declination`.

**Flags**

| Bit | Mask | Name in `status` | Meaning |
|---:|---|---|---|
| 0 | 0x001 | `aligned` | Orientation initialised |
| 1 | 0x002 | `coarse` | Aligned while moving; tilt still converging |
| 2 | 0x004 | `still` | At rest: zero-rate updates active |
| 3 | 0x008 | `heading` | Heading locked to the magnetic field |
| 4 | 0x010 | `acc-disturbed` | Acceleration detected; accelerometer weight cut at least tenfold |
| 5 | 0x020 | `mag-disturbed` | Magnetic disturbance detected; magnetometer weight cut |
| 6 | 0x040 | `mag-uncalibrated` | Magnetometer ignored: not calibrated |
| 7 | 0x080 | `saturated` | Sensor saturation or lost samples in the last second |
| 8 | 0x100 | `thermal-transient` | Temperature transient in progress |
| 9 | 0x200 | `recovery` | Divergence watchdog fired in the last 5 s |
| 10 | 0x400 | `mag-stale` | No usable magnetometer sample for over a second |

### `$RAW`: raw IMU

```text
$RAW,seq,n,gx,gy,gz,ax,ay,az,temp
```

All fields are integers. Each line holds the **sums** of `n` consecutive samples in sensor counts; `seq` is the number of the first one. Every IMU sample is numbered whether or not the stream is on, so `seq` and `n` reveal lost lines, and the sample time is `seq / odr_hz`.

To physical units, with the factors from `info`:

```text
gyro, °/s         = gx / n · gyro_dps_per_lsb       (2000 / 2^19)
acceleration, g   = ax / n · accel_g_per_lsb        (16 / 2^19)
temperature, °C   = temp / n / temp_lsb_per_c + temp_offset_c
```

The data are uncalibrated. At `n = 1` and 512 Hz, the stream is about 30 kB/s.

### `$MAG`: raw magnetometer

```text
$MAG,t_ms,mx,my,mz
```

Field in µT in the MMC5983MA's own axes, with the bridge offset already removed by SET/RESET, uncalibrated. `t_ms` is the middle of the conversion. About 50 lines per second.

The MMC5983MA's Z axis points into the board, so this frame is a mirror image of the body frame: in body axes the field is `(mx, my, −mz)` before calibration. The magnetometer calibration matrix includes this.

### `$BARO`: barometer

```text
$BARO,t_ms,pressure_hPa,temp_C,altitude_m
```

Altitude from the international barometric formula, relative to 1013.25 hPa. 25 lines per second.

## Calibration fields

Format: `cal set <field> <values…>`. Each sensor is corrected as `corrected = matrix · (raw − bias − polynomial(T − T_ref))`.

| Field | Count | Unit | Meaning |
|---|---:|---|---|
| `flags` | 1 | — | Bits: 1 gyro, 2 accelerometer, 4 magnetometer, 8 gyro temperature, 16 accelerometer temperature |
| `gyro_bias` | 3 | °/s | Gyro bias at `gyro_tc_tref` |
| `gyro_matrix` | 9 | — | Scale and non-orthogonality, row by row |
| `accel_bias` | 3 | g | Accelerometer bias |
| `accel_matrix` | 9 | — | Scale and cross-axis terms |
| `mag_bias` | 3 | µT | Hard iron |
| `mag_matrix` | 9 | — | Soft iron and the mapping into IMU axes (may be a reflection, see `$MAG`) |
| `gyro_tc_tref`, `accel_tc_tref` | 1 | °C | Reference temperature of the polynomial |
| `gyro_tc_x`, `_y`, `_z` | 3 | °/s per °C, °C², °C³ | Polynomial coefficients per axis |
| `accel_tc_x`, `_y`, `_z` | 3 | g per °C, °C², °C³ | The same for the accelerometer |
| `declination` | 1 | ° | Magnetic declination, east positive. 0 = magnetic north |
| `mag_field`, `mag_dip` | 1 | µT, ° | Field and dip found by calibration; for reference |

The filter uses the magnetometer only when bit 4 is set in `flags`.

## Example session

```text
> info
# fw=ins10-ahrs version=2.0.0 board=pico lib=2.0.0
# imu=ok clock=external odr_hz=512.000 measured_hz=511.987
# gyro_dps_per_lsb=0.00381469727 accel_g_per_lsb=3.05175781e-05 temp_lsb_per_c=132.48 temp_offset_c=25
# mag=ok mag_enabled=1 baro=ok
# att_rate_hz=50 raw_div=0
# last_reset=normal
# cal_flags=7 stored=1 thermal_count=1840
OK
> stream att 2
OK
$ATT,81234,0.999990,0.001200,-0.003100,0.002900,0.14,-0.36,0.33,89.67,0.2104,-0.1532,0.0871,0.03,0.21,31.42,00D
```

The numbers in the example are illustrative.
