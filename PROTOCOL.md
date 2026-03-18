# MotionCal Serial Wire Protocol

This document describes the serial communication protocol used between an IMU device
(running Adafruit imucal firmware or compatible) and a calibration host such as
[PJRC MotionCal](https://github.com/PaulStoffregen/MotionCal).

The protocol originated in Paul Stoffregen's [MotionCal](https://github.com/PaulStoffregen/MotionCal)
and was extended by Adafruit's [SensorLab imucal](https://github.com/adafruit/Adafruit_SensorLab)
firmware.  This document is the first formal specification; prior implementations are the
authoritative reference where this document is ambiguous.

---

## Overview

Communication is half-duplex over a serial connection (typically USB-CDC at 115200 baud,
though the protocol is baud-independent).

- **Device → Host**: newline-terminated ASCII lines at ~100 Hz, carrying raw sensor data
  and calibration echo.
- **Host → Device**: a single 68-byte binary calibration packet, sent on demand.

---

## Device → Host: ASCII Line Protocol

All lines are terminated with `\r\n` (CR LF).  Fields within a line are separated by
commas with no spaces.  Lines that do not match a known prefix are silently ignored by
the host.

### `Raw:` — Scaled integer sensor data

Emitted every update cycle (~100 Hz).  Nine comma-separated **signed 16-bit integers**
encoding accelerometer, gyroscope, and magnetometer readings:

```
Raw:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>\r\n
```

| Field | Sensor | Encoding | Decode to physical units |
|-------|--------|----------|--------------------------|
| ax, ay, az | Accelerometer | g × 8192 → int16 | value / 8192 → g |
| gx, gy, gz | Gyroscope     | deg/s × 16 → int16 | value / 16 → deg/s |
| mx, my, mz | Magnetometer  | µT × 10 → int16 | value / 10 → µT |

This format is compatible with the original PJRC MotionCal tool.  Values are lossy due to
the fixed-point scaling; use `Uni:` when full precision is needed.

**Example:**
```
Raw:-58,-815,8362,76,-121,-95,-375,-159,-24\r\n
```

### `Uni:` — Floating-point sensor data (SI units)

Emitted every update cycle (~100 Hz), immediately after `Raw:`.  Nine comma-separated
**IEEE 754 single-precision floats**:

```
Uni:<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>\r\n
```

| Field | Sensor | Units |
|-------|--------|-------|
| ax, ay, az | Accelerometer | m/s² |
| gx, gy, gz | Gyroscope     | rad/s |
| mx, my, mz | Magnetometer  | µT |

> **Unit mismatch note:** `Raw:` and `Uni:` use *different* unit conventions for
> accelerometer (g vs m/s²) and gyroscope (deg/s vs rad/s).  This is an inherited quirk
> of the MotionCal-compatible `Raw:` encoding.  A compliant host must normalize both line
> types to a common internal representation before use.

Hosts that support both line types should prefer `Uni:` for its higher precision.
The original MotionCal tool only understands `Raw:`; extended hosts understand both.

**Example:**
```
Uni:-0.07,-0.98,10.00,0.0832,-0.1327,-0.1046,-37.50,-15.93,-2.50\r\n
```

### `Cal1:` — Calibration echo (scalar and hard iron fields)

Emitted periodically (suggested: every 50 update cycles) to allow the host to confirm that
a previously sent calibration packet was received and applied correctly.  Ten
comma-separated floats:

```
Cal1:<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<mag_hix>,<mag_hiy>,<mag_hiz>,<mag_field>\r\n
```

| Fields 0–2  | Accelerometer zero-g offsets (g) |
|-------------|----------------------------------|
| Fields 3–5  | Gyroscope zero-rate offsets (deg/s) |
| Fields 6–8  | Magnetometer hard iron offsets (µT) |
| Field 9     | Magnetometer reference field strength B (µT) |

### `Cal2:` — Calibration echo (soft iron matrix)

Emitted periodically (suggested: every 100 update cycles).  Nine comma-separated floats
encoding the 3×3 magnetometer soft iron correction matrix in **row-major order**:

```
Cal2:<XX>,<XY>,<XZ>,<YX>,<YY>,<YZ>,<ZX>,<ZY>,<ZZ>\r\n
```

The matrix is symmetric (XY == YX, XZ == ZX, YZ == ZY), so the nine echoed values
contain only six unique numbers.  The row-major echo order differs from the upper-triangle
transmission order used in the binary calibration packet (see below); a confirming host
must account for this when comparing echoed values to what it sent.  The host uses `Cal1:`
and `Cal2:` together to verify delivery: after sending a calibration packet it compares
the echoed values against what it sent and reports confirmation when both match within
tolerance.

### `Name:` — Device friendly name  *(planned extension)*

Emitted once at startup and periodically thereafter.  A human-readable UTF-8 string
identifying the device, firmware, or sensor configuration:

```
Name:<name>\r\n
```

`<name>` is a free-form UTF-8 string, maximum 63 bytes, with no embedded commas or
newlines.  Hosts should display this string in their device selector UI.

**Example:**
```
Name:Adafruit imucal/Feather ESP32-S3\r\n
```

---

## Host → Device: Binary Calibration Packet

The host sends a **68-byte little-endian binary packet** to deliver calibration
coefficients to the device.  All multi-byte fields are little-endian.

### Packet layout

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 1 | uint8 | Signature byte 0: `0x75` |
| 1 | 1 | uint8 | Signature byte 1: `0x54` |
| 2 | 4 | float32 | Accelerometer zero-g offset X (g) |
| 6 | 4 | float32 | Accelerometer zero-g offset Y (g) |
| 10 | 4 | float32 | Accelerometer zero-g offset Z (g) |
| 14 | 4 | float32 | Gyroscope zero-rate offset X (deg/s) |
| 18 | 4 | float32 | Gyroscope zero-rate offset Y (deg/s) |
| 22 | 4 | float32 | Gyroscope zero-rate offset Z (deg/s) |
| 26 | 4 | float32 | Magnetometer hard iron offset X (µT) |
| 30 | 4 | float32 | Magnetometer hard iron offset Y (µT) |
| 34 | 4 | float32 | Magnetometer hard iron offset Z (µT) |
| 38 | 4 | float32 | Magnetometer reference field strength B (µT) |
| 42 | 4 | float32 | Soft iron invW[0][0] |
| 46 | 4 | float32 | Soft iron invW[1][1] |
| 50 | 4 | float32 | Soft iron invW[2][2] |
| 54 | 4 | float32 | Soft iron invW[0][1] (== invW[1][0], matrix is symmetric) |
| 58 | 4 | float32 | Soft iron invW[0][2] (== invW[2][0]) |
| 62 | 4 | float32 | Soft iron invW[1][2] (== invW[2][1]) |
| 66 | 1 | uint8 | CRC16 low byte |
| 67 | 1 | uint8 | CRC16 high byte |

Total: **68 bytes**.

### Soft iron matrix

`invW` is the inverse of the soft iron distortion matrix W — the 3×3 matrix that
corrects ellipsoidal magnetometer distortion back to a sphere.  Only the six unique
elements of the symmetric matrix are transmitted (upper triangle plus diagonal).
The device reconstructs the full matrix by mirroring: `invW[r][c] = invW[c][r]`.

### CRC

The CRC covers bytes 0–65 (the full packet excluding the two CRC bytes themselves).

- **Algorithm:** CRC-16/IBM, also known as CRC-16/ARC
- **Polynomial:** 0x8005 reflected → 0xA001
- **Initial value:** 0xFFFF
- **Input/output reflection:** yes (reflected algorithm)
- **Final XOR:** none

The receiver computes the CRC over all 68 bytes (including the two CRC bytes) using the
same algorithm.  A correct packet yields a residual of **0x0000**.

**C reference implementation:**
```c
uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= data;  // XOR data byte into low byte of CRC before processing
    for (int i = 0; i < 8; ++i) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}
// Usage: init crc = 0xFFFF; feed each byte; correct packet residual == 0x0000
```

> **Implementation note:** The `data` byte is XOR'd into the low byte of `crc` *before*
> the shift loop — not inside it.  This matches `Adafruit_Sensor_Calibration::crc16_update()`
> and the original MotionCal `serialdata.c`.  The full send-side call pattern is:
> ```c
> uint16_t crc = 0xFFFF;
> for (int i = 0; i < 66; ++i)
>     crc = crc16_update(crc, buf[i]);
> buf[66] = (uint8_t)(crc);
> buf[67] = (uint8_t)(crc >> 8);
> ```

### Current limitations

The current MotionCal implementation sends **0.0** for all accelerometer and gyroscope
offsets.  These fields are structurally present and the device stores them, but no
accel/gyro calibration state machines exist yet.  Future versions will populate these
fields with real calibration results.

---

## Planned Extensions

The following extensions are not yet implemented in MotionCal or Adafruit imucal.  They
are tracked in [SensorCal](https://github.com/bruceoberg/SensorCal), a newer
MotionCal-compatible host that extends this protocol, and are documented here to reserve
field positions and establish intent.

### Extended binary packet with accel/gyro calibration  *(planned)*

When accelerometer and gyroscope calibration state machines are added to libcalib, the
binary packet will carry real offset vectors for those sensors.  The existing packet
layout already has the right fields at offsets 2–25; the extension is purely in the
firmware that populates and applies them.

### `Cal3:` / `Cal4:` echo lines  *(planned)*

Corresponding device→host echo lines for accelerometer and gyroscope calibration,
analogous to `Cal1:`/`Cal2:`.  Exact field layout TBD when the calibration state machines
are designed.

### `Name:` line  *(planned — see above)*

Already described in the Device→Host section.

---

## Compatibility

| Feature | PJRC MotionCal | Adafruit imucal | SensorCal¹ |
|---------|:--------------:|:---------------:|:----------:|
| Send `Raw:` | — | ✓ | — |
| Send `Uni:` | — | ✓ | — |
| Send `Cal1:`/`Cal2:` | — | ✓ | — |
| Receive `Raw:` | ✓ | — | ✓ |
| Receive `Uni:` | — | — | ✓ (preferred) |
| Receive `Cal1:`/`Cal2:` | — | — | ✓ (confirmation) |
| Send binary packet | ✓ | — | ✓ |
| Receive binary packet | — | ✓ | — |

¹ SensorCal is a newer, MotionCal-compatible host that extends this protocol.

---

## References

- [PJRC MotionCal source](https://github.com/PaulStoffregen/MotionCal) — `imuread.c`,
  `rawdata.c`, `serialdata.c`
- [Adafruit SensorLab imucal firmware](https://github.com/adafruit/Adafruit_SensorLab/tree/master/examples/calibration/imucal)
- [Adafruit Sensor Calibration library](https://github.com/adafruit/Adafruit_Sensor_Calibration) — `crc16_update()`
- [Adafruit SensorLab Magnetometer Calibration guide](https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration)
- [SensorCal](https://github.com/bruceoberg/SensorCal) — newer MotionCal-compatible host;
  protocol extensions originated here
