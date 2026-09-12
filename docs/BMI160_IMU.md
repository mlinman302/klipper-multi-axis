# BMI160: an IMU for B homing and Z homing

**Status: architecture.** The sensor layer described below is
implemented. The consumers on top of it - the gyro motion gate in
`[accel_b_homing]`, `B_GYRO_CALIBRATE`, and the tap detector of
[Accel_Z_Tap.md](Accel_Z_Tap.md) - land in the phases at the end of this
document. Nothing here has run on the machine yet; every number quoted
from the datasheet is a datasheet number, not a measurement.

The BMI160 replaces the ADXL345 as the head sensor for
[Accel_B_Homing.md](Accel_B_Homing.md) and
[Accel_Z_Tap.md](Accel_Z_Tap.md). It is a six-axis IMU: the same
three-axis accelerometer role the ADXL345 played, plus a three-axis
gyroscope in the same package, on the same bus, in the same FIFO, sampled
on the same clock.

The accelerometer half is an improvement. The gyroscope half is the
point.

## Why a gyroscope belongs on a tilting head

`[accel_b_homing]` measures B by reading the gravity vector in the head's
own frame. That works, and it is absolute, but it has one structural
weakness: **an accelerometer cannot distinguish a tilted head from an
accelerating one.** The module's defence today is indirect - it looks at
the per-sample standard deviation of the accelerometer and calls the head
"still" if that number is small enough (`max_sample_deviation`). A head
rocking slowly on its belts at 2 Hz barely moves that statistic and
comfortably ruins the angle.

A gyroscope measures rotation rate *directly*. Four things follow, and
they are the whole case for the chip.

### 1. A motion gate that actually tests for motion

A head at rest reads |omega| = 0. Not "small deviation" - zero, to within
the zero-rate offset, and the zero-rate offset is a constant the chip can
calibrate away by itself (see [Offset calibration](#offset-calibration)).
Gating a B measurement on measured rotation rate replaces an inference
with an observation, which is the same upgrade the accelerometer itself
made over counting steps.

### 2. Rotation sense and degrees-per-step, measured rather than assumed

The rotation sense of B on this machine is still unverified - see
[Accel_B_Homing.md](Accel_B_Homing.md) on `positive_vector` being
*declared* rather than fitted. Integrating the gyro's on-axis rate over a
commanded move answers it directly:

    command B +10 degrees, integrate omega over the move

If the integral is +10.0, sense and scale are both confirmed. If it is
-10.0, the sense is inverted. If it is +9.2, `rotation_distance` on the B
stepper is wrong by 8 %. This measurement does not depend on the
accelerometer, on the zero reference, on where B was homed, or on the
head being level - it is a pure observation of the move that just
happened. The accelerometer cannot do it: over a 10 degree move near
B = 0 the gravity vector barely changes, which is exactly where a
single-axis accel reading goes flat.

### 3. A gravity-free channel for tap detection

[Accel_Z_Tap.md](Accel_Z_Tap.md) spends most of its length on one
problem: the accelerometer's contact signal sits on top of a 1 g dc term
whose distribution across the axes is a function of B. That document's
answer is a high-pass filter. The gyroscope needs no such answer -
**there is no gravity term in a rate signal at all.** A nozzle strike is
applied some distance from the head's rotation axis, so it delivers an
angular impulse as well as a linear one, and that impulse appears in the
gyro against a background that is genuinely zero-mean.

Whether the angular channel is *better* than the filtered linear one is a
measurement, not an argument, and phase 4 below captures both. The point
here is that the BMI160 makes it a free comparison: the two channels
arrive interleaved in one FIFO, on one timebase, from one chip.

### 4. Settling without waiting for it

`[accel_b_homing]` currently pays `settle_time` (0.25 s) plus
`sample_time` (0.5 s) plus `batch_margin` (0.3 s) on every measurement,
almost all of it waiting for a head on belts to stop ringing. A
complementary filter - accelerometer for the absolute low-frequency
angle, gyroscope for the clean short-term rate - tracks the angle
*through* the ringing instead of waiting it out. This is the least urgent
of the four and the most work, so it is last in the phase list, but it is
the one that would make measuring B cheap enough to do routinely rather
than once per home.

## The accelerometer half, on the numbers

Replacing a working sensor needs the accelerometer half to be at least
neutral. It is not neutral; it is better on every axis that matters here
except sample rate.

| | ADXL345 | BMI160 |
| --- | --- | --- |
| Accel resolution | 13 bit, 3.9 mg/LSB (full-res, any range) | 16 bit, 0.061 mg/LSB at +/-2 g; 0.49 mg/LSB at +/-16 g |
| Accel range | +/-2 to +/-16 g | +/-2 to +/-16 g |
| Accel max ODR | 3200 Hz | 1600 Hz |
| Accel noise | 0.75 LSB rms X/Y, 1.1 LSB rms Z at 100 Hz | 180 ug/sqrt(Hz) typ, 300 max |
| Zero-g offset | +/-150 mg over life; manual trim at 15.6 mg/LSB | +/-40 mg at board level, +/-150 mg over life; **hardware FOC to 3.9 mg** |
| Gyroscope | none | 16 bit, +/-125 to +/-2000 deg/s |
| Gyro resolution | - | 3.8 m deg/s per LSB at +/-125 deg/s |
| Gyro noise | - | 0.007 deg/s/sqrt(Hz) |
| Gyro zero-rate offset | - | +/-3 deg/s uncalibrated; **hardware FOC** |

Two rows in that table are the actual trade and deserve reading
carefully.

**Sample rate goes down.** The BMI160's accelerometer tops out at
1600 Hz against the ADXL345's 3200 Hz, and in the combined
accel-plus-gyro FIFO mode this driver uses, the two sensors are
*required* to share an output data rate (see
[One FIFO, two sensors](#one-fifo-two-sensors-one-timebase)), so 1600 Hz
is the operating point for both. For `[accel_b_homing]`, which averages
over half a second, this is invisible - it halves the sample count of an
already enormous average. For `[resonance_tester]` it halves the Nyquist
frequency, from 1600 Hz to 800 Hz; input shaper work lives well below
200 Hz so this is unlikely to matter, but it is a real reduction and it
should be stated rather than discovered. For the Z tap detector it is the
open question: if the contact transient lives above 800 Hz, this chip
cannot see it in this mode.

**Amplitude resolution goes up by a lot.** At the +/-2 g range this
driver defaults to, one count is 0.061 mg against the ADXL345's 3.9 mg -
64 times finer. For tilt measurement that is the number that matters: a
gravity vector resolved to 0.061 mg is an angle resolved to about
0.0035 degrees, where 3.9 mg is about 0.22 degrees. A half-second average
buries both figures under the noise floor, but it means the
*instantaneous* reading is usable, which is what a complementary filter
needs.

Range is a config option (`accel_range`) rather than a constant, because
the two uses want different answers: tilt wants +/-2 g for resolution,
tap and resonance testing want +/-16 g for headroom against impact. The
previous BMI160 driver hard-coded +/-16 g.

## Layering

Four layers, and the seams between them are the architecture:

```
  src/sensor_bmi160.c         drains the chip FIFO into bulk messages;
                              optionally decodes one channel per frame
                              and feeds trigger_analog          (MCU)
        |                            |
        | sensor_bulk_data           | trsync_do_trigger
        v                            v
  klippy/extras/bmi160.py      klippy/extras/trigger_analog.py
    - bus setup, chip config     - sos filter, threshold, endstop
    - combined sample stream
    - accel view    gyro view
        |               |                    |
        v               v                    v
  [accel_b_homing]  (motion gate,        [accel_z_tap]      (phase 5)
  [resonance_tester] B_GYRO_CALIBRATE)
```

The rule that shapes all of it: **the chip object knows about the chip
and nothing else.** It does not know what B is, it does not know what a
probe is, and it never moves the machine. `[accel_b_homing]` looks it up
by name and reads it through the same small interface
`[resonance_tester]` uses, exactly as it does today with the ADXL345 -
which is why swapping the chip is a config change rather than a code
change for that module.

## The sensor layer

### One FIFO, two sensors, one timebase

The BMI160's FIFO in *headerless* mode stores only sensor data, in the
same order as the data registers. Those put the gyroscope at 0x0C-0x11
and the accelerometer at 0x12-0x17, so with `fifo_gyr_en` and
`fifo_acc_en` both set, one frame is twelve bytes:

    gx_lo gx_hi gy_lo gy_hi gz_lo gz_hi ax_lo ax_hi ay_lo ay_hi az_lo az_hi

six little-endian signed 16-bit values, gyroscope first. Headerless mode
requires the output data rates of all enabled sensors to be identical,
which is the constraint behind the shared 1600 Hz noted above.

This is why the gyroscope costs almost nothing to add. The samples are
not two streams that have to be correlated after the fact - they are one
stream, captured by one chip on one sample clock, delivered through one
FIFO. The accelerometer sample and the gyroscope sample in a frame are
simultaneous by construction. Any fusion built on top gets that for free;
an ADXL345 plus a separate gyro would have had to earn it.

### Block arithmetic

Klipper's `FixedFreqReader` assigns timestamps by *counting* samples: it
multiplies a message sequence number by a fixed samples-per-block and
interpolates. Two consequences bind the driver:

* Every bulk message must carry exactly `MAX_BULK_MSG_SIZE // frame_size`
  frames, or the timestamps drift. `MAX_BULK_MSG_SIZE` is 51.
* 51 // 12 = 4 frames of 48 bytes in combined mode; 51 // 6 = 8 frames of
  48 bytes in accel-only mode.

Both land on 48 bytes, which is why `BYTES_PER_BLOCK` in the firmware is
one constant for both modes, and why enabling the gyroscope needed no
change to the block-transfer path at all. The FIFO length register counts
whole frames' worth of bytes, so a block read that waits for 48 pending
bytes never straddles a partial frame and never reads the 0x80 magic that
marks an over-read.

The firmware needs to know the frame size for one thing only - the
trigger seam below - and it is told at config time.

### The trigger seam

A tap has to be detected on the MCU that owns the chip; the reasoning is
in [Accel_Z_Tap.md](Accel_Z_Tap.md) and the short version is that the
host is two orders of magnitude too slow. The firmware therefore exposes:

    bmi160_attach_trigger_analog oid=%c trigger_analog_oid=%c
                                 frame_offset=%c bytes_per_frame=%c

`frame_offset` is the byte offset, within a frame, of the 16-bit channel
to watch: 10 for accel Z in combined mode, 4 for gyro Z, 4 for accel Z in
accel-only mode. One channel, chosen at config time, decoded in the block
read loop and handed to `trigger_analog_update()`. Everything downstream
of that call - the second-order-section filter, the threshold, the trsync
dispatch, the sensor-quiet monitor - already exists and is shared with
the load cell and eddy current probes.

This is a seam, not a feature: it is the interface `[accel_z_tap]` plugs
into in phase 5. No host code calls it yet.

### Offset calibration

The BMI160 can calibrate its own offsets, which the ADXL345 cannot. A
`start_foc` command measures both sensors against a declared target and
writes the negated result into the OFFSET registers, from where the chip
applies it to every sample. Accelerometer accuracy is 3.9 mg; the whole
procedure takes at most 250 ms.

The gyroscope case is the easy and valuable one: **the target is always
zero**, in every axis, at any orientation. Calibrating zero-rate offset
needs nothing but a stationary head, so it can be done whenever wanted,
and it turns the +/-3 deg/s uncalibrated spec into a number small enough
that a motion gate at a fraction of a degree per second is meaningful.

The accelerometer case needs a *pose*: `foc_acc_x/y/z` each declare
whether that axis should read -1 g, 0 g or +1 g, so the head has to be at
a known attitude - which on this machine means B = 0, with the axes named
by `[accel_b_homing]`'s `zero_vector`. That makes accelerometer FOC a
deliberate calibration command rather than something to do at startup,
and it is what finally answers the "phase two offset and gain fit" that
[Accel_B_Homing.md](Accel_B_Homing.md) defers: a large part of it is done
in hardware.

Note the chip's own warning: the offset registers have an NVM backup with
a lifetime of at most 14 write cycles. This driver writes the volatile
image registers and never programs NVM, so a calibration is lost on power
cycle and has to be redone - which is the right trade for something that
takes 250 ms.

## The B homing path

`[accel_b_homing]` needs one code change to use the gyroscope: the motion
gate. The measurement itself is unchanged, because it is already
chip-agnostic - it looks up `accel_chip` by name and calls
`start_internal_client()`.

    measure():
        wait_moves, dwell(settle)
        average accel over sample_time    ->  B = atan2(u, w)   (as today)
        average |omega| over sample_time  ->  reject if > max_rotation_rate

The gate is additive and degrades cleanly: a chip with no gyroscope (an
ADXL345, a LIS2DW) does not expose a gyro client, the gate is skipped,
and the existing `max_sample_deviation` check remains the only defence.
That keeps `[accel_b_homing]` working with every chip it works with
today, which matters because the module is useful to people who do not
have a BMI160.

`B_GYRO_CALIBRATE` - job 2 above - is new work, not a change to an
existing path: start a gyro client, run a commanded B move, integrate the
on-axis rate across it, report swept angle against commanded angle. It
moves the machine, which nothing in `[accel_b_homing]` does today, so it
arrives with the usual homing-state and interlock questions.

## The Z homing path

Unchanged in design from [Accel_Z_Tap.md](Accel_Z_Tap.md); the BMI160
changes three things about it.

* **Path A is dead, and its wiring experiment is moot.** That document's
  fallback was the ADXL345's built-in tap engine, gated on whether INT1
  is routed to a GPIO on the accelerometer board. The BMI160's equivalent
  (`INT_TAP`, 0x63-0x64) has the same dc-coupling and interrupt-routing
  problems, and the BMI160's FIFO data is 16-bit rather than 13-bit, so
  the one argument in Path A's favour - that the chip's internal path
  sees detail the stream does not - is much weaker. Path B, the
  `trigger_analog` detector, is the only path.
* **There is a second channel to try.** Angular rate, per job 3.
* **The sample rate is 1600 Hz, not 3200 Hz.** Per the trade above, this
  is the one place the change could genuinely hurt, and phase 4 measures
  it rather than guessing.

## Migrating from the ADXL345

The BMI160 is a drop-in at the config level. Everything that reads an
accelerometer by name - `[accel_b_homing]`, `[resonance_tester]` - takes
`bmi160` where it took `adxl345`.

```ini
# before
[adxl345]
cs_pin: adxl:gpio9
spi_software_sclk_pin: adxl:gpio10
spi_software_mosi_pin: adxl:gpio11
spi_software_miso_pin: adxl:gpio12

# after
[bmi160]
cs_pin: imu:gpio9
spi_software_sclk_pin: imu:gpio10
spi_software_mosi_pin: imu:gpio11
spi_software_miso_pin: imu:gpio12
```

Three things do change, and are worth knowing before the swap:

* **`axes_map` is almost certainly different.** The BMI160's package axes
  are not the ADXL345's, and on a breakout board the orientation is
  whatever the board vendor chose. Re-derive `axes_map`, `zero_vector`
  and `positive_vector` by looking - park the head, run
  `ACCELEROMETER_QUERY` - exactly as they were derived the first time.
  Do not carry the old values across.
* **`[accel_b_homing]`'s `accel_chip` now defaults to `bmi160`.**
  Configurations still using an ADXL345 must say `accel_chip: adxl345`
  explicitly.
* **Input shaper results do not transfer.** Different chip, different
  mounting, half the sample rate: re-run `SHAPER_CALIBRATE`.

The ADXL345 support is untouched. `[adxl345]` still exists, still works,
and remains the right choice for a machine that only wants resonance
measurement at 3200 Hz.

## Phases

1. **Sensor layer.** Combined accel+gyro FIFO, both streams exposed to
   the host, configurable range and rate, FOC commands, the
   `trigger_analog` attach point. Host tests for frame decoding and
   scaling. *This is what is implemented.*
2. **The motion gate.** `max_rotation_rate` in `[accel_b_homing]`, fed by
   a gyro client, skipped when the chip has no gyroscope.
3. **`B_GYRO_CALIBRATE`.** Integrate rate across a commanded move; report
   sense and scale. Settles the rotation-sense question in
   [Accel_B_Homing.md](Accel_B_Homing.md).
4. **Z tap phase 0.** Capture contact signatures on both channels, at
   several speeds, with the machine also captured moving in air. The
   deliverable is the contact-to-background ratio, the band that
   maximises it, and a verdict on whether 1600 Hz is enough.
5. **`[accel_z_tap]`.** The detector, the endstop, the probe session.
6. **Complementary filtering.** Job 4 - measure B without waiting for the
   head to settle.

## Testing

Same rule as [Accel_B_Homing.md](Accel_B_Homing.md): host tests run
against a stubbed printer and synthetic data, so they run anywhere Python
does. (klippy does not run natively on the Windows development host, so
anything needing a live printer is a machine test, not a CI test.)

What the host tests cover honestly: frame decoding, the scale factors
against the datasheet's sensitivity tables, `axes_map` application to
both streams at once, register-value construction for each range and
rate, and the block-arithmetic invariant that
`MAX_BULK_MSG_SIZE // frame_size` frames fit in `BYTES_PER_BLOCK`.

What they cannot cover: whether the chip is wired correctly, the real
noise floor, the real zero-rate offset, and every threshold number in the
config. Those are machine measurements.

## Open questions

* Is the contact transient visible at 1600 Hz? (Phase 4. This is the
  question the whole Z path rests on.)
* Angular or linear channel for tap detection - or both, ANDed?
  (Phase 4.)
* How large is the gyro's zero-rate offset after FOC, and how much does
  it drift over a print's worth of temperature change? This sets
  `max_rotation_rate`, and it decides whether FOC belongs at connect time
  or before each measurement.
* Does +/-2 g leave enough headroom on a moving head, or does a normal
  acceleration move clip it? (+/-2 g is 1 g of gravity plus 1 g of
  margin, and 1 g is about 9800 mm/s^2 - comfortably above any print
  move, but not above a crash.) If clipping shows up, +/-4 g is the
  answer and costs one bit.
* Is the BMI160's 16-bit accelerometer at 1600 Hz actually better than
  the ADXL345's 13-bit at 3200 Hz for `[resonance_tester]`? Worth one
  side-by-side `SHAPER_CALIBRATE` rather than an assumption.
