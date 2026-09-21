# BMI160 head IMU

The corertheta tilting head carries a Bosch BMI160: a three-axis
accelerometer and a three-axis gyroscope in one package, on one bus,
sharing one FIFO and one sample clock. It is the only sensor this fork
supports for B homing and calibration - [Accel_B_Homing.md](Accel_B_Homing.md),
`[accel_b_homing]` - where B is measured against gravity on an angle
fused from both sensors.

This document covers the sensor itself: wiring it to the machine,
bringing it up, how the driver (`klippy/extras/bmi160.py`,
`src/sensor_bmi160.c`) presents it, and its calibration. The option
reference is `[bmi160]` in [Config_Reference.md](Config_Reference.md#bmi160).

**Status.** The driver, the fused B measurement and the motion gate are
implemented and host-tested. Beyond a first `G28 B`,
none of it has been exercised on the machine: every number below is a
datasheet number or arithmetic on one, not a measurement.

## Why an IMU rather than an accelerometer

An accelerometer at rest reads gravity, and gravity in the head's frame
is the head's tilt - but **an accelerometer cannot tell a tilted head
from an accelerating one**. A head rocking slowly on its belts barely
moves the sample deviation and still ruins the angle. The gyroscope is
what closes that gap, two ways:

1. **A fused angle.** The two sensors fail in opposite directions: the
   accelerometer is absolute but only once the head has stopped moving,
   the gyroscope is exact over short times but cannot say where the head
   started and drifts. A complementary filter crosses them over at
   `fusion_tau`, so the angle is absolute *and* tracks the head through
   the ringing after a move. B is only ever homed, checked or calibrated
   on this angle.
2. **A motion gate that observes motion.** A head at rest reads zero
   rotation rate, so `max_rotation_rate` tests directly for the thing a
   deviation threshold can only infer.

The two sensors are **simultaneous by construction**: one FIFO frame
carries both, sampled on the same clock. A fusion needs an acceleration
and a rate measured at the same instant, and here that is a property of
the hardware rather than something the host has to arrange.

## The chip, on the numbers

From the BMI160 datasheet:

| | Accelerometer | Gyroscope |
| --- | --- | --- |
| Resolution | 16 bit; 0.061 mg/LSB at +/-2 g, 0.49 mg/LSB at +/-16 g | 16 bit; 3.8 m deg/s per LSB at +/-125 deg/s |
| Range | +/-2, 4, 8 or 16 g | +/-125 to +/-2000 deg/s |
| Max output data rate | 1600 Hz | 3200 Hz (1600 Hz here - see below) |
| Noise | 180 ug/sqrt(Hz) typical, 300 max | 0.007 deg/s/sqrt(Hz) |
| Offset | +/-40 mg at board level, +/-150 mg over life | +/-3 deg/s zero-rate offset, uncalibrated |
| On-chip offset compensation | to 3.9 mg, against a declared pose | to zero rate, at any pose |

Two consequences to know:

* **1600 Hz is the ceiling for both sensors.** The driver reads the FIFO
  in headerless mode, which requires every enabled sensor to share an
  output data rate, and the accelerometer stops at 1600 Hz. For
  `[resonance_tester]` that is an 800 Hz Nyquist frequency, well above
  where input shaping works.
* **Range trades resolution for headroom.** At +/-2 g a count is
  0.061 mg, about 0.0035 degrees of tilt, which is what a complementary
  filter wants from each sample. `[resonance_tester]` on a hard-driven
  head may want more headroom, which is what `accel_range` is for.

## On this machine: SPI0 of the Pi Zero W

The BMI160 is wired to **SPI0 of the Raspberry Pi Zero W that runs
klippy**, and read through klipper_mcu, the Linux-process host MCU.

### Topology

```
  BMI160 --SPI0--> Pi Zero W kernel (spi-bcm2835, /dev/spidev0.0)
                       |
                   klipper_mcu        Linux-process MCU, [mcu rpi],
                       |              SCHED_FIFO realtime priority
                       | pty /tmp/klipper_host_mcu
                       v
                    klippy            same Pi, same single ARMv6 core,
                       |              shared with Moonraker
                       | USB
                       v
                    LPC1769           [mcu], all the steppers
```

| Signal | Pi Zero W | Header pin | BMI160 breakout label |
| --- | --- | --- | --- |
| SCLK | GPIO11 | 23 | SCL |
| MOSI | GPIO10 | 19 | SDA |
| MISO | GPIO9 | 21 | SA0 |
| CS | GPIO8 (CE0) | 24 | CS |
| 3.3 V | - | 1 or 17 | 3V3 |
| GND | - | 6, 9, 20 or 25 | GND |

Most breakouts print the I2C names. Use SCL and SDA, not SCX and SDX,
which are the auxiliary magnetometer bus. The kernel drives CE0 itself,
which is why the config says `cs_pin: rpi:None`.

The chip powers up speaking I2C and switches to SPI on the first rising
edge of chip select, staying there until power is lost. The driver
provides that edge with a dummy read of register 0x7F before every chip
id check, so there is nothing to configure - but a hand-written test
script has to do the same (see [Bringing it up](#bringing-it-up)).

### Why SPI

Klipper's Linux I2C driver ignores `i2c_speed`, a Pi's I2C1 defaults to
100 kHz, and draining one 48-byte FIFO block costs about 510 bits on the
wire - so 1600 Hz needed twice a stock bus. On SPI the same block is one
3-byte length read and one 49-byte data read, 416 bits, clocked at
`spi_speed`, which the Linux SPI driver does honour (the BCM2835 rounds
it down to 125 MHz over an even divisor, so 1 MHz becomes 992 kHz):

| `rate` | blocks/s | wire time at 1 MHz | at 4 MHz |
| --- | --- | --- | --- |
| 1600 Hz | 400 | 17 % | 4 % |
| 800 Hz | 200 | 8 % | 2 % |
| 400 Hz | 100 | 4 % | 1 % |
| 200 Hz | 50 | 2 % | 0.5 % |

These are wire-time lower bounds, and on SPI they are not the binding
cost. Each block is still two `ioctl` system calls from klipper_mcu on a
single ARMv6 core, and that overhead - measurable, not computable - is
what limits the rate. The driver's 1 MHz default already takes the wire
out of the picture, and a slower clock is kinder to the long unshielded
run out to the head.

### Choosing `rate`

**B wants `rate: 400`.** With the chip's normal-mode filter, bandwidth
scales with the data rate, so the noise on a mean over a window of length
T is set by the noise density and T alone - about `density / sqrt(2 T)`.
At 180 ug/sqrt(Hz) over the default half-second window that is about
0.18 mg, roughly 0.01 degrees of tilt, **at any rate**. The lower rate
buys margin where the Pi is short:

* **FIFO headroom.** 85 frames is 212 ms at 400 Hz against 53 ms at
  1600 Hz - four times as long for klipper_mcu to be late before frames
  are lost.
* **Host CPU.** A quarter of the bulk messages into klippy and of the
  system calls in klipper_mcu, on the core that also plans motion.
* **The gate's noise floor.** `max_rotation_rate` tests a mean of
  per-sample magnitudes, which does depend on bandwidth: estimating the
  gyroscope's bandwidth at 0.4 x ODR, a parked head reads about
  0.14 deg/s at 400 Hz and 0.28 deg/s at 1600 Hz. Both are under the
  1.0 default.
* **Fusion.** A 2.5 ms step is still eighty samples per 0.2 s time
  constant.

Raise it only for `[resonance_tester]`, whose band of interest needs
the higher Nyquist frequency.

### What a bad link does

SPI has no acknowledge, so a wiring fault does not shut the printer down
- an unplugged chip, a broken MISO wire or a flipped bit all read back as
data. Only a transfer the kernel refuses is a shutdown ("Unable to issue
spi ioctl"), and that means a missing `/dev/spidev0.0`, not a bad wire.
The checks therefore live in the driver:

* **Every measurement starts with the chip id.** An absent chip or an
  open MISO reads `0x00` or `0xff` instead of `0xd1`, and the command
  fails with "Invalid bmi160 id".
* **Every configuration write is read back.** A register that does not
  hold its value fails the measurement the same way.
* **A corrupted FIFO length shows up as invalid frames.** Reading past
  the end of the FIFO returns a fixed 0x80 pattern, which the driver
  drops and counts in each batch's `errors`. Nothing refuses a
  measurement on that count yet.
* **A flipped bit inside a sample is not detected.** At +/-2 g the top
  two bits are worth 2 g and 1 g, and a spike that size pushes the
  sample deviation past `max_sample_deviation`, so the measurement is
  refused. A 0.5 g spike in a 200-sample window moves the mean by
  2.5 mg, about 0.14 degrees, and is invisible. Repeating `B_MEASURE`
  on a parked head is how to look for it.

The chip is read only while a client is measuring, so a print that never
measures never exercises the link. Keep the run to the head short and
away from motor leads, run a ground alongside SCLK, and treat an
intermittent "Invalid bmi160 id" as wiring before software. A breakout
with level-shifting transistors on SDA and SCL may not pass 1 MHz; drop
`spi_speed` to 400000 to tell.

## Bringing it up

1. **Enable SPI0.** On Raspbian bookworm add `dtparam=spi=on` to
   `/boot/firmware/config.txt` and reboot. `ls /dev/spidev0.*` should list
   `spidev0.0` and `spidev0.1`.

2. **Read the chip id, before Klipper is involved.** `sudo apt install
   python3-spidev`, then:

   ```
   python3 -c '
   import spidev
   spi = spidev.SpiDev()
   spi.open(0, 0)
   spi.max_speed_hz = 1000000
   spi.mode = 0
   spi.xfer2([0xff, 0x00])                 # dummy read: the CS edge selects SPI
   print(hex(spi.xfer2([0x80, 0x00])[1]))  # chip id
   '
   ```

   `0xd1` is a BMI160 on SPI. `0x0` or `0xff` is a wiring fault - MISO,
   power or chip select - and anything else is usually SCLK and MOSI
   swapped.

3. **Build klipper_mcu from this tree.** Its `config_bmi160` takes a
   `bytes_per_frame` argument that stock builds do not know, and a
   mismatched build is refused at connect. SPI and BMI160 support are on
   by default in the Linux process build. Use a separate config and
   output directory so the LPC1769's `.config` is left alone:

   ```
   cd ~/klipper
   make KCONFIG_CONFIG=config.rpi OUT=out_rpi/ menuconfig    # Linux process
   sudo systemctl stop klipper
   make KCONFIG_CONFIG=config.rpi OUT=out_rpi/ flash
   sudo cp scripts/klipper-mcu.service /etc/systemd/system/
   sudo systemctl enable --now klipper-mcu
   sudo systemctl start klipper
   ```

   The LPC1769 does not need reflashing.

4. **Configure it:**

   ```ini
   [mcu rpi]
   serial: /tmp/klipper_host_mcu

   [bmi160]
   cs_pin: rpi:None
   spi_bus: spidev0.0
   rate: 400
   ```

5. **Check it.** `BMI160_QUERY` should report about 9800 mm/s^2 on one
   axis and near-zero rotation. Then `BMI160_CALIBRATE GYRO=1` with the
   head parked, and `B_MEASURE` a few times: a "possible fifo overflows"
   refusal at `rate: 400` means the Pi, not the wire, is too loaded.

From here the homing documents take over: derive `zero_vector` and
`positive_vector` for `[accel_b_homing]` (see
[Accel_B_Homing.md](Accel_B_Homing.md#declaring-the-zero)), then follow
its commissioning order.

## Axes

### `axes_map`, and why the gyroscope needs care

One chip means one `axes_map`, applied to both sensors - but not
identically. **An angular rate is a pseudovector.** Under a map that
*reflects* the frame rather than rotating it - a swap without a matching
negation, such as `x, z, y`, or a lone negation such as `x, -y, z` - a
pseudovector picks up a sign a vector does not. `bmi160.py` multiplies
the mapped gyroscope by the map's determinant for exactly that reason.
Nothing reading only |omega| would notice; a fusion would integrate the
rate backwards. A map that drops or repeats an axis cannot be corrected
and is refused while the gyroscope is enabled.

`[accel_b_homing]` does not need any particular `axes_map`: its
`zero_vector` and `positive_vector` are declared in whatever frame the
map produces. If the chip is replaced or remounted, derive the map and
both vectors again by looking, rather than carrying the old values.

### The gyroscope's sign is derived, not configured

Fusing a rate into B needs to know which gyroscope axis carries dB/dt,
and with which sign. That follows from `zero_vector` and
`positive_vector` alone. B is the angle of world-up in the sensor frame,
measured from `zero_vector` toward `positive_vector`. Turning the sensor
at omega makes world-fixed vectors appear to turn at *minus* omega, and
the zero-to-positive sense is a positive rotation about `w_hat x u_hat`,
so

    dB/dt = -omega . (w_hat x u_hat)

With both vectors axis-aligned this is one signed component. For the
example config (`zero_vector: +z`, `positive_vector: +x`) the rotation
axis is y and `dB/dt = -omega_y`. The host tests confirm the derivation
recovers the true rate for all twenty-four axis-aligned mountings.

**What checks it on the machine.** `[accel_b_homing]` compares the fused
angle against an accelerometer-only average over the end of the same
window, and `max_fusion_disagreement` turns a large difference into an
error. That catches an inverted, mis-scaled or mis-axed gyroscope only
while the head is turning: the divergence is about `2 * tau * rate`, so
at the default 0.2 s and 5 degrees the head must turn at roughly
12 deg/s. **On a parked head a wrong sign is invisible.** Check it once
by hand during commissioning: with the motors off, run `BMI160_QUERY`
while turning the head steadily toward +B - the direction in which the
reading along `positive_vector` grows. The gyroscope axis that
`[accel_b_homing]`'s status names as `rotation_axis` should read a rate
with the same sign as its `rotation_axis_sign`.

## Offset calibration

`BMI160_CALIBRATE` runs the chip's fast offset compensation (FOC): it
measures both sensors against a declared target and writes the negated
result into the chip's offset registers, which it then applies to every
sample. It takes at most 250 ms.

* **Gyroscope - do this.** The target is zero rate at any pose, so it
  needs nothing but a stationary head. It turns a +/-3 deg/s offset into
  one small enough for a sub-degree-per-second motion gate, and it
  matters for B: the fused angle carries a fixed bias of roughly
  offset x `fusion_tau`. Run `BMI160_CALIBRATE GYRO=1` after every power
  cycle, before `G28 B`.
* **Accelerometer - do not, with `[accel_b_homing]`.** FOC needs a pose
  declared exact (`X`/`Y`/`Z` = -1, 0 or +1 g), so a head a few degrees
  off vertical is booked as vertical, and the result is lost on power
  cycle - silently changing the raw readings the B sensor fit was made
  on. `B_SENSOR_CALIBRATE` fits the offsets and gain ratio in software
  from a sweep that needs no exact pose, and `SAVE_CONFIG` keeps them.

The offset registers have an NVM backup rated for at most 14 writes. The
driver writes only the volatile image and never programs NVM, so every
calibration is lost on power cycle by design.

## The driver

### One FIFO, two sensors, one timebase

In headerless mode the FIFO stores sensor data in data-register order,
and the gyroscope registers (0x0C) precede the accelerometer's (0x12).
With both enabled one frame is twelve bytes:

    gx_lo gx_hi gy_lo gy_hi gz_lo gz_hi ax_lo ax_hi ay_lo ay_hi az_lo az_hi

six little-endian signed 16-bit values, gyroscope first. The host
converts each frame once and presents views of the stream:

| Client | Samples | Used by |
| --- | --- | --- |
| `start_internal_imu_client()` | `(time, gx, gy, gz, ax, ay, az)` | the fused B measurement |
| `start_internal_client()` | `(time, ax, ay, az)` | `ACCELEROMETER_*` commands, `[resonance_tester]`, `B_MEASURE FUSION=0`, `B_SENSOR_CALIBRATE` |
| `start_internal_gyro_client()` | `(time, gx, gy, gz)` | the motion gate on an unfused measurement |

The accelerometer and gyroscope views are also dump endpoints
(`bmi160/dump_bmi160`, `bmi160/dump_bmi160_gyro`). `gyro: False` removes
the gyroscope from the FIFO, giving six-byte frames; `[accel_b_homing]`
refuses a chip configured that way.

### Block arithmetic

Klipper's `FixedFreqReader` timestamps bulk samples by *counting* them,
so every bulk message must carry exactly `MAX_BULK_MSG_SIZE // frame`
frames. `MAX_BULK_MSG_SIZE` is 51: 51 // 12 = 4 frames in combined mode,
51 // 6 = 8 in accelerometer-only mode. Both are 48 bytes, so
`BYTES_PER_BLOCK` in the firmware is one constant; the frame size is
passed at config time as `bytes_per_frame`. The firmware polls the FIFO
every four sample periods and reads a block only once a whole block is
pending, so a read never straddles a partial frame or reads the 0x80
over-read pattern.

### Lost frames are reported

In headerless mode the chip overwrites old frames without any other
indication - its skip frame exists only in header mode - and a FIFO byte
count cannot exceed the FIFO size. `sensor_bmi160.c` therefore reports a
possible overflow whenever the FIFO is too full to take another whole
frame. Because timestamps come from counting, lost frames also leave the
survivors mistimed, which a fused angle would integrate. `[accel_b_homing]`
refuses a measurement during which the overflow count rose, naming
`rate` as the thing to lower.

## Commands

* `BMI160_QUERY [CHIP=<name>]` - the latest acceleration and rotation
  rate. Use it to derive `axes_map` and `[accel_b_homing]`'s vectors.
* `BMI160_CALIBRATE [CHIP=<name>] [GYRO=0|1] [X=] [Y=] [Z=]` - fast
  offset compensation, above.
* `ACCELEROMETER_QUERY`, `ACCELEROMETER_MEASURE` and the other standard
  accelerometer commands work on the accelerometer view, so
  `[resonance_tester]` can use `accel_chip: bmi160`.

## Testing

`test/multi_axis/test_bmi160.py` runs on any host with Python: the scale
factors against the datasheet tables, register values for each range and
rate, frame decoding, `axes_map` on both streams with the determinant
correction, the stream views, and the invariant that the frames per
message fill `BYTES_PER_BLOCK`. The fusion and sign convention are
covered in `test_accel_b_homing.py`.

Nothing on the host can check the wiring, the real noise floor and
zero-rate offset, the right `fusion_tau`, or any threshold. Those are
machine measurements.

## Open questions

* Does klipper_mcu keep the FIFO drained at 400 Hz while klippy is
  busy? The overflow check answers it.
* Does the SPI run to the head survive the stepper cables? The symptom
  is an intermittent "Invalid bmi160 id", or `B_MEASURE` results that
  scatter more than the noise floor predicts.
* Should a measurement be refused when the chip's `errors` count rises,
  as it is for overflows? On SPI that count is the only sign of a
  corrupted FIFO length read.
* How large is the gyroscope's zero-rate offset after FOC, and how far
  does it drift over a print's temperature change? It sets
  `max_rotation_rate` and the fused angle's bias.
