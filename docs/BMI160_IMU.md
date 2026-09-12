# BMI160: an IMU for B homing and Z homing

**Status: architecture.** The sensor layer, the fused B measurement and
the motion gate are implemented. `B_GYRO_CALIBRATE` and the tap detector
of [Accel_Z_Tap.md](Accel_Z_Tap.md) land in the phases at the end of
this document. Nothing here has run on the machine yet; every number
quoted from the datasheet is a datasheet number, not a measurement, and
`fusion_tau` in particular is a starting point rather than a result.

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

A gyroscope measures rotation rate *directly*, and the two sensors fail
in opposite directions - the accelerometer is absolute but only over
long times, the gyroscope is exact but only over short ones. Fusing them
is therefore the point, not a refinement of it.

### 1. A fused angle: absolute *and* fast

Neither sensor alone can measure a moving head's tilt. The accelerometer
sees gravity plus whatever the head's own acceleration adds, so it is
only trustworthy once the head has stopped ringing. The gyroscope sees
the rotation exactly, but it cannot say where the head started and its
zero-rate offset makes an integrated angle drift without bound.

A complementary filter crosses them over at one time constant, `tau`:

    predicted = angle + rate * dt          (gyroscope, short term)
    angle     = predicted + (1 - alpha) * (accel_angle - predicted)
    alpha     = tau / (tau + dt)

Above `tau` the accelerometer wins, so the answer is absolute and does
not drift. Below `tau` the gyroscope wins, so the answer tracks the head
*through* the ringing that follows a move rather than waiting for it to
stop. `[accel_b_homing]` runs this over the whole capture - settle dwell
included - and reports the filter's final state, which turns
`settle_time` from "wait for the head to stop" into "give the filter
time to converge".

Two details are load-bearing enough to state here.

**The blend is written as a correction, not as an average.** Blending
two angles arithmetically puts a measurement that straddles the +/-180
wrap on the opposite side of the circle. The implementation adds
`(1 - alpha)` of the *wrapped difference* to the prediction instead, so
the wrap is a non-event.

**The two sensors are simultaneous by construction.** A fused sample
needs an acceleration and a rotation rate measured at the same instant.
Here they arrive in the same twelve-byte FIFO frame, on the same sample
clock - so `[accel_b_homing]` reads the combined stream directly rather
than pairing up the accelerometer and gyroscope views and hoping they
line up. An ADXL345 plus a separate gyro would have had to earn that
guarantee; this chip gives it away.

### 2. A motion gate that actually tests for motion

A head at rest reads |omega| = 0. Not "small deviation" - zero, to within
the zero-rate offset, and the zero-rate offset is a constant the chip can
calibrate away by itself (see [Offset calibration](#offset-calibration)).
Gating a B measurement on measured rotation rate replaces an inference
with an observation, which is the same upgrade the accelerometer itself
made over counting steps.

### 3. Rotation sense and degrees-per-step, measured rather than assumed

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

### 4. A gravity-free channel for tap detection

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

## The gyroscope's sign is not a free parameter

This is the part worth reading twice, because it is the part most likely
to be wrong on a machine and the part that is cheapest to get right.

Fusing a rate into an angle needs to know **which gyroscope axis carries
dB/dt, and with which sign**. That looks like a thing to add a config
option for. It is not: it follows from `zero_vector` and
`positive_vector`, which `[accel_b_homing]` already declares.

B is the angle of the world-up vector in the sensor frame, measured from
`zero_vector` toward `positive_vector`. Turning the sensor at omega
makes world-fixed vectors appear to turn at *minus* omega in the sensor
frame, and the zero-to-positive sense is a positive rotation about
`w_hat x u_hat`, so

    dB/dt = -omega . (w_hat x u_hat)

Both vectors are axis aligned, so `w_hat x u_hat` is plus or minus the
third basis vector and the whole thing collapses to one signed
component. For the reference mounting in the example config
(`zero_vector: +z`, `positive_vector: +x`) the rotation axis is y and
`dB/dt = -omega_y`.

There is one caveat, and it lives in the chip module rather than in the
homing module. **An angular rate is a pseudovector.** Under an `axes_map`
that *reflects* the frame rather than merely rotating it - any swap
without a matching negation, such as `x, z, y`, and also a lone negation
such as `x, -y, z` - a pseudovector picks up a sign that a vector does
not. `bmi160.py` therefore multiplies the mapped gyroscope by the map's
determinant. Nothing that reads only `|omega|` notices the difference;
anything that fuses the two does, and would integrate the rate backwards
without it. A degenerate `axes_map` that drops or repeats an axis is
refused outright when the gyroscope is enabled.

### What checks this on the machine

`[accel_b_homing]` compares the fused angle against a trailing
accelerometer average over the end of the same window. Both estimate
"the angle now", so they agree when the gyroscope is wired up correctly
and diverge when its sign, scale or axis is wrong;
`max_fusion_disagreement` turns a large divergence into an error rather
than a silent bias, and `B_MEASURE` prints the difference either way.

Be clear about the limits of that check, because it is easy to
over-trust:

* On a **parked head** it proves nothing. There is no rotation to
  integrate, so an inverted gyroscope produces exactly the same answer
  as a correct one. The test suite states this as a test rather than
  leaving it implied.
* On a **moving head** the divergence from an inverted gyroscope is
  about `2 * tau * rate`. At the default `tau` of 0.2 s that is 0.4
  degrees per deg/s, so catching it at the default 5 degree tolerance
  needs the head to be turning at something like 12 deg/s.

The definitive check is therefore a *deliberate* move - turn B by a
known amount and confirm the fused angle tracks it - which is exactly
what `B_GYRO_CALIBRATE` automates in phase 3. Until that lands, do it by
hand once during commissioning.

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

`[accel_b_homing]` now produces two estimates of the angle from one
capture, and reports both:

    measure():
        wait_moves, start the combined imu stream
        dwell(settle + sample_time + batch_margin)

        accel_angle  = atan2(u, w) of the mean acceleration over the
                       window                          (as before)
        fused_angle  = complementary filter over settle+window, ending
                       at the window's end             (new, authoritative)
        rotation_rate = mean |omega| over the window   -> motion gate
        disagreement  = fused_angle - trailing accel angle -> sanity check

`angle` - the number `CHECK=1` compares against the commanded B, and the
number in `get_status()` - is the fused one when a gyroscope is present.
`accel_angle` is kept alongside it, unchanged in meaning, so the old and
new estimators can be compared on the machine rather than trusted.

The filter deliberately stops at the **end of the measurement window**
rather than the end of the capture. The trailing `batch_margin` is
delivery slack that the user did not ask to measure, and running the
filter through it would leave the fused angle and the accelerometer tail
it is checked against describing different instants.

Everything degrades cleanly without a gyroscope. An `[adxl345]` or a
`[lis2dw]` exposes no gyroscope client, so fusion and the gate are both
skipped, `angle` is `accel_angle`, and the module behaves exactly as it
did before. That matters: the module is useful to people who do not have
a BMI160.

### What this does not yet buy

Fusion makes a *shorter* `settle_time` safe in principle - that is the
whole point of the gyroscope carrying the short timescales - but the
defaults are unchanged, because the right `settle_time` is a machine
measurement. The commissioning path is: leave the defaults, run
`B_MEASURE`, watch the fused and accelerometer-only angles agree on a
parked head, then lower `settle_time` and watch where they start to
disagree. `max_sample_deviation` and `max_rotation_rate` both reject a
moving head by default and will need raising or disabling first - they
answer "was this a static tilt?", which is a different question from
"what is the angle?".

`B_GYRO_CALIBRATE` - job 3 above - remains new work: start the stream,
run a commanded B move, integrate the on-axis rate across it, report
swept angle against commanded angle. It moves the machine, which nothing
in `[accel_b_homing]` does today, so it arrives with the usual
homing-state and interlock questions. It is also what finally verifies
the gyroscope's sign, per the limits noted above.

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

1. **Sensor layer.** Combined accel+gyro FIFO, all three streams
   exposed to the host (accelerometer, gyroscope, and the combined one
   fusion reads), configurable range and rate, pseudovector-correct
   `axes_map`, FOC commands, the `trigger_analog` attach point.
   *Implemented.*
2. **The fused measurement and the motion gate.** The complementary
   filter, the derived gyroscope axis and sign, `max_rotation_rate` and
   `max_fusion_disagreement` in `[accel_b_homing]`. All skipped when the
   chip has no gyroscope. *Implemented.*
3. **`B_GYRO_CALIBRATE`.** Integrate rate across a commanded move;
   report sense and scale. Settles the rotation-sense question in
   [Accel_B_Homing.md](Accel_B_Homing.md), and is the definitive check
   on the gyroscope's sign.
4. **Commissioning `fusion_tau` and `settle_time`.** Measure the head's
   ringing, pick `tau`, and find how far `settle_time` can come down.
   This is the phase that actually collects the benefit of phase 2.
5. **Z tap phase 0.** Capture contact signatures on both channels, at
   several speeds, with the machine also captured moving in air. The
   deliverable is the contact-to-background ratio, the band that
   maximises it, and a verdict on whether 1600 Hz is enough.
6. **`[accel_z_tap]`.** The detector, the endstop, the probe session.

## Testing

Same rule as [Accel_B_Homing.md](Accel_B_Homing.md): host tests run
against a stubbed printer and synthetic data, so they run anywhere Python
does. (klippy does not run natively on the Windows development host, so
anything needing a live printer is a machine test, not a CI test.)

What the host tests cover honestly: frame decoding, the scale factors
against the datasheet's sensitivity tables, `axes_map` application to
both streams at once and the determinant correction on the gyroscope,
register-value construction for each range and rate, and the
block-arithmetic invariant that `MAX_BULK_MSG_SIZE // frame_size` frames
fit in `BYTES_PER_BLOCK`.

For the fusion: the filter's behaviour against synthetic streams - that
it holds a steady angle, converges from a bad seed, follows the
gyroscope over short timescales and the accelerometer over long ones,
tracks a consistent ramp without lag, and crosses the +/-180 wrap
correctly. And the sign convention, checked by rotating a synthetic head
about its own B axis and confirming that the derived coefficient
recovers the true rate for **all twenty-four** axis-aligned mountings -
which is the cheapest possible insurance against the one number nobody
can eyeball.

What they cannot cover: whether the chip is wired correctly, the real
noise floor, the real zero-rate offset, the right `fusion_tau`, and every
threshold number in the config. Those are machine measurements.

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
* What is the head's actual ringing frequency and decay, and therefore
  what should `fusion_tau` be? The default of 0.2 s assumes the
  interesting motion is faster than about 5 Hz, which is a guess.
  Phase 4.
* How far can `settle_time` come down before the fused and
  accelerometer-only angles part company? That difference is the
  measurement, and `B_MEASURE` already prints it.
* Does the gyroscope's zero-rate offset drift enough over a print for a
  fixed `tau` to matter? A complementary filter passes offset straight
  through at `rate * tau`, so 0.1 deg/s of residual offset is 0.02
  degrees of bias at the default `tau` - small, but it scales with
  `tau` and is worth knowing before raising it.
