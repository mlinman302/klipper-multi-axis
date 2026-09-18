# B axis homing and calibration

`[accel_b_homing]` (`klippy/extras/accel_b_homing.py`) measures the
corertheta tilting head's B angle against gravity with the BMI160 bolted
to the head, and builds three things on that measurement:

* **`G28 B`** - B has no endstop. The head is measured, the measurement is
  booked as B, and the head is turned to B = 0, measured again after each
  move until it is there.
* **`B_SENSOR_CALIBRATE`** - fits the sensor's offsets and gain ratio, so
  B = 0 is gravity's vertical rather than the sensor's.
* **`B_STEP_CALIBRATE`** - measures the drive ratio, `b_coupling_ratio`.

plus `B_MEASURE`, which reports the measurement and can compare it with
the commanded angle.

The BMI160 is the only supported sensor, and its gyroscope must be
enabled. Wiring, bring-up and the driver are in
[BMI160_IMU.md](BMI160_IMU.md); the option reference is
[Config_Reference.md](Config_Reference.md#accel_b_homing).

**Status.** Everything here is implemented and host-tested
(`test/multi_axis/test_accel_b_homing.py`). Only the first `G28 B` has
run on the machine - it homed about 7 degrees from a known vertical,
which an uncalibrated accelerometer offset explains, and
`B_SENSOR_CALIBRATE` has not yet been run to remove it. Whether a
positive B really tilts the nozzle outboard is still unverified.

## Why measure B

B is the awkward axis on this machine:

* **It has nothing to home against.** Its range ends are where the
  filament tube starts to kink, not hard stops. It used to home on a
  StallGuard virtual endstop shared with the R gantry pair, where a home
  issued too soon after another move triggered instantly, the axis did
  not move, and B was still reported homed. `[rtcp]` turns every B error
  into an X/Z error at the nozzle, so a silent wrong home is expensive.
* **Its zero is a physical claim** - the nozzle points straight down -
  that a switch or a stall threshold only approximates. `[rtcp]` and the
  probe's `b_offset` are both built on it.
* **Its drive ratio cannot be measured with a ruler.** Both gantry motors
  turn for a B move, through a differential, and `b_coupling_ratio` is a
  nominal 1.0 until it is measured.

An accelerometer at rest reads the gravity vector, and gravity in the
head's own frame *is* the head's tilt - absolutely, whatever the axis
has done since it was last homed. That makes homing an observation and
the drive ratio something measurable.

## Mounting

* Rigid to the **rotating** part of the head, not the carriage. Any
  compliance in the mount is measured as tilt.
* **Square**: one sensor axis parallel to the B rotation axis, the other
  two spanning the plane the head tilts in. Which is which is declared,
  below. A sensor rotated a degree about the B axis reads a degree out,
  and no calibration here can see it.
* Route the cable so the 145-degree swing does not tug it. A cable that
  pulls on the head is a systematic angle error.

The chip can be shared with `[resonance_tester]`, and no particular
`axes_map` is required.

## Declaring the zero

Two signed sensor axes, restricted to the six axis-aligned directions,
say which way is B = 0:

    zero_vector      the sensor axis that reads +1 g at B = 0
    positive_vector  the sensor axis that reads +1 g at B = +90

An accelerometer at rest reads the specific force, which points *up*, so
both name "the sensor axis pointing straight up" at their angle. Find
them by looking: park the head at B = 0, run `BMI160_QUERY`, and name the
axis reading about +9800 mm/s^2 (the negated name if it reads -9800).
Turn the head toward +90 and repeat for `positive_vector`. With `w` the
reading along `zero_vector` and `u` the reading along `positive_vector`,

    B = atan2(u, w)

which is 0 at B = 0 and +90 at B = +90 by construction - no offset angle
and no inversion flag. Using both in-plane axes keeps the resolution
uniform across the range, where a single axis through `asin` goes flat
near +/-90 degrees. The third axis is the rotation axis, by elimination;
it should barely change as B turns, and it is reported as a health check.

`positive_vector` also fixes the rotation sense the sensor believes in.
It must agree with the direction the *motors* call +B, or `G28 B` stops
on its first move (see below). `invert_b_direction` in `[printer]` stays
the one place the machine's own rotation sense is set.

## The measurement

`B_MEASURE` and everything that moves B read the head through one
primitive:

```
measure(settle, window)
  1. wait for moves; start the BMI160's combined stream
     dwell(settle + window + batch_margin)
  2. refuse if the chip reported possible FIFO overflows
  3. correct every sample by offset_u, offset_w and gain_ratio
  4. over the window (settle .. settle + window):
       refuse if no samples, or fewer than half those expected
       refuse if any axis's sample deviation > max_sample_deviation
       refuse if | |mean| - 1 g | > max_magnitude_error
       refuse if mean |omega| > max_rotation_rate
     accel_angle = atan2(u, w) of the mean acceleration
  5. over the capture, settle included, to the window's end:
       fused_angle = complementary filter of the accelerometer angle
                     and the gyroscope rate about the rotation axis
       refuse if the capture is shorter than 3 x fusion_tau
       refuse if fused_angle differs from the accelerometer average over
                  the last fifth of the window by > max_fusion_disagreement
```

### The fused angle is the only one acted on

The accelerometer cannot tell a tilted head from an accelerating one; the
gyroscope sees rotation exactly but cannot say where the head started,
and its offset makes an integral drift. A complementary filter crosses
them over at `fusion_tau`:

    predicted = angle + rate * dt
    angle     = predicted + (1 - alpha) * (accel_angle - predicted)
    alpha     = tau / (tau + dt)

Above `tau` the accelerometer wins, so the angle is absolute and does not
drift. Below it the gyroscope wins, so the angle tracks the head through
the ringing after a move. The blend is written as a correction to the
prediction, not an average of two angles, so a measurement straddling
+/-180 does not land on the opposite side of the circle. Which gyroscope
axis carries dB/dt, and its sign, is derived from the two vectors - see
[BMI160_IMU.md](BMI160_IMU.md#the-gyroscopes-sign-is-derived-not-configured).

**`G28 B`, the endstop direction and verification, `B_MEASURE CHECK=1`
and `B_STEP_CALIBRATE` act only on the fused angle**, and `measured_b`
in the status is only ever a fused angle. The accelerometer-only angle is a diagnostic: `B_MEASURE` prints
it beside the fused one, `FUSION=0` measures with it alone, and the
status reports it as `accel_b`. `B_SENSOR_CALIBRATE` also reads it,
because it fits the averaged vector at parked stations, not an angle.
Fusion cannot be turned off in the config.

### Details that matter

* **Settle, window and margin are separate.** The head hangs on belts and
  rings after a move, so averaging starts after `settle_time`. The fused
  angle runs through the settle too, which turns the settle from "wait
  for the head to stop" into "let the filter converge". `batch_margin`
  is never sampled: the BMI160 delivers in 0.1 s batches, later still
  through klipper_mcu, and the margin lets the batch carrying the end of
  the window arrive before it is read.
* **Rejection is loud.** A reading taken while the head drifts silently
  poisons a home, so every gate raises an error naming the setting to
  change rather than returning a number.
* **No transform has to be off to measure.** The sensor reads a physical
  angle. `[rtcp]` moves X and Z, never B. `[b_projection]` changes what
  the *commanded* B means, so `B_MEASURE` converts the commanded angle
  with `project_pos()` before comparing. Everything that *moves* B does
  need both transforms off.

### What limits accuracy

| Source | Size | Angle effect | Handled by |
| --- | --- | --- | --- |
| Quantisation | 0.061 mg/LSB at +/-2 g | 0.0035 deg per sample | nothing needed |
| Noise | 180 ug/sqrt(Hz) | ~0.01 deg over a 0.5 s window | averaging, fusion |
| **Accelerometer offset** | **+/-40 mg board, +/-150 mg life** | **up to ~8.5 deg near B = 0** | **`B_SENSOR_CALIBRATE` - required** |
| Gain mismatch between axes | small | angle dependent | `B_SENSOR_CALIBRATE` |
| Gyroscope zero-rate offset | +/-3 deg/s uncalibrated | ~offset x `fusion_tau` of bias | `BMI160_CALIBRATE GYRO=1` |
| Head still moving | - | unbounded | fusion; settle; the gates |
| Sensor rotated on its mount | - | direct | nothing - a physical reference |
| Frame not level with gravity | - | direct | nothing |

The row that decides the design is the accelerometer offset. Near B = 0
the angle is almost entirely `u / w`, so an offset along
`positive_vector` moves the zero directly: 100 mg is 5.7 degrees.
Repeatability is a different matter and is already good - a parked head
re-measures to a few hundredths of a degree - which is why the gates are
tight while `check_tolerance` is loose.

## `G28 B`

With no `endstop_pin` in `[stepper_tilt]` (the default), `G28 B` is a
measured home (`AccelBHoming.home_axis()`):

```
G28 B
  1. refuse unless RTCP and the B projection are off
  2. energise both gantry motors - a limp head would be measured where it
     droops, then jump when the motors took hold
  3. B = measure_vertical(); book it as B (B is now homed)
     report it, and say so if it is outside the soft limits
  4. while |B| > zero_tolerance:
       refuse after max_homing_moves moves ("not converging")
       until the direction is confirmed:
           target = B moved toward 0 by at most direction_check_move,
                    clamped into [position_min, position_max]
       afterwards:
           target = 0
       move to target; B' = measure_vertical()
       on the first move of at least 1 deg, check (B' - B) / (target - B):
           <= -0.5     refuse: positive_vector and the motors disagree
           < 0.5       refuse: the head is not following the motors
           > 2         refuse: b_coupling_ratio is far off
       B = B'; book it as B
  5. finish on a commanded B = 0
```

Booking the measurement as B is exact whatever the drive ratio, so the
loop tolerates a wrong `b_coupling_ratio`: a 10 % error converges from
60 degrees out in three moves after the check move. A loop that does not
converge means the ratio is badly wrong, and the error says so.

The capped first move is the safety half. Until the head has been seen
to follow the motors, "toward zero" is an assumption: with
`positive_vector` or `invert_b_direction` wrong, a straight move to zero
from B = 80 drives the head to 160, far past the soft limit. Capping the
first move bounds that mistake to `direction_check_move` degrees and turns
it into an error. Soft limits apply to commanded angles only, so a head
measured beyond one has its first move end *on* the limit - a move longer
than the cap, still checked.

On any error `G28` turns the motors off and leaves B unhomed.
`[stepper_tilt]`'s `position_min` and `position_max` are soft limits, and
`homing_speed` is the speed the head is turned at.

B = 0 is only as vertical as the sensor makes it: uncalibrated, it is the
sensor's zero, not gravity's. Run `B_SENSOR_CALIBRATE` once.

### With an endstop

A `[stepper_tilt]` that has an `endstop_pin` sweeps into it as usual, and
the measurement steers and checks the sweep. With `homing_positive_dir`
unset the head is measured first: below `position_endstop` it homes
positive, above it negative, and the sweep is the measured distance plus
`homing_tolerance`. Within `homing_tolerance` of the endstop the side
cannot be told; an endstop at a range limit is then homed toward that
limit, and one inside the range is refused. After the home the head is
measured again (`verify_home`), and B is left unhomed if it is not within
`homing_tolerance` of `position_endstop` - which catches a sensorless home
that triggered without moving. Setting `homing_positive_dir` bypasses the
direction measurement.

### How the home is wired in

`PrinterHoming.cmd_G28` ends with `ea.home()` for each extra axis.
`[accel_b_homing]` registers itself on the B axis at connect with
`BaseRotaryAxis.set_homing_source()`; a rail without an endstop then
calls `source.home_axis(axis)`. The source drives the axis through
`get_range()`, `set_measured_position(angle)`, `move_axis(angle)` and
`get_drive_steppers()` (both gantry motors on corertheta). The rail is
built with `need_endstop=False`, so `[stepper_tilt]` without an
`endstop_pin` refuses `position_endstop` and the other endstop-only
options rather than ignoring them. Without `[accel_b_homing]`, such an
axis can only be placed with `SET_ROTARY_AXIS AXIS=B SET_POSITION=`.

## Sensor calibration: `B_SENSOR_CALIBRATE`

Every sample is corrected before anything else sees it:

    u' = u - offset_u
    w' = (w - offset_w) / gain_ratio
    B  = atan2(u', w')

The three numbers are fitted, not declared:

```
B_SENSOR_CALIBRATE [START=] [END=] [STEPS=13] [SETTLE=1] [SAMPLE_TIME=1] [GAIN=1]
  preconditions: B homed, RTCP and the B projection off,
                 START and END inside the soft limits, at least 60 deg apart
  for each of STEPS stations from START to END (default: the soft limits):
      move B, measure the accelerometer mean, undo the loaded calibration
  refuse if an axis other than the rotation axis varied least
         (the two vectors name the wrong pair)
  refuse if the measured angle ran opposite to commanded B
  fit the ellipse the in-plane readings trace,
      (u - offset_u)^2 + (w - offset_w)^2 / gain_ratio^2 = A^2
  as a linear least squares problem
  refuse, changing nothing, if an offset exceeds 0.3 g, gain_ratio leaves
         0.8 .. 1.25, or the radius A leaves 0.8 .. 1.2 g
  apply the result, set it for SAVE_CONFIG, report every station,
  return to B = 0, and home B again on the new calibration
```

The model depends on the arc the head was *measured* to cover, not the
arc commanded:

| Measured arc | Fitted |
| --- | --- |
| 120 deg or more, 5+ stations | both offsets and the gain ratio |
| 60 to 120 deg, 4+ stations (or `GAIN=0`) | both offsets; `gain_ratio` held at 1 |
| less than 60 deg | refused |

The fit needs only that gravity is the same size in every pose - never
the true angle of any station - so neither a wrong `b_coupling_ratio` nor
a slightly rotated sensor leaks into it. The same property means it
cannot *see* a rotated sensor or a frame out of level: both keep the
circle a circle. The corertheta range of -45 to 100 degrees supports the
full fit.

The report ends with how far the head that measured B = 0 before now is
from B = 0 - how far the next home will turn it. An endstop rail is not
homed again, since the switch decided its position.

## Drive ratio calibration: `B_STEP_CALIBRATE`

The question is how far the head really turns per commanded degree:

```
B_STEP_CALIBRATE [START=] [END=] [STEPS=13] [SETTLE=1] [SAMPLE_TIME=1] [RETURN=0]
  preconditions, all before the first move: RTCP and the B projection off,
      B homed, the sensor calibrated (not the identity), START and END
      inside the soft limits and at least 20 deg apart, and room for 5 deg
      of over-travel beyond them
  1. move to START - 5 deg, so every station is approached from the same
     side and backlash stays out of the scale
  2. at each station: move; t = axis.get_step_position() (the angle the
     integer step counters imply); phi = measure_vertical()
  3. with RETURN=1: move to END + 5 deg and visit the stations in reverse
  4. unwrap phi, then fit one scale shared by both legs and one offset
     per leg:   phi = scale * t + offset[leg]
  5. refuse, changing nothing, a scale outside 0.5 .. 2
  6. report every residual, their rms and max, and with RETURN=1 the
     backlash (the difference between the two offsets)
  7. set the corrected ratio for SAVE_CONFIG; park at B = 0
```

START and END default to 5 degrees inside the soft limits. A failure
after the first move parks the head at B = 0 before the error is raised.
`MODE=QUICK [ANGLE=90]` is a two-station spot check from B = 0 that
reports the scale and writes nothing.

The sensor calibration is required, not recommended: an uncorrected
offset bends the measured angle - worth degrees near B = 0 and almost
nothing near B = 90 - and a straight-line fit books that curvature as a
ratio error. The zero of the sensor and that of the step counters both
land in the offsets and do not matter.

### Counting steps

The fit is against the integer step counters, not the commanded
position, so step rounding is included and the scale is dimensionless.
On corertheta the two gantry motors are

    stepper_r     p_minus = b * ratio - radius
    stepper_tilt  p_plus  = b * ratio + radius

so their **sum depends only on B**: `0.5 * (p_plus + p_minus) = b * ratio`.
`CoreRThetaKinematics.get_axis_step_position('b')` is that sum, divided by
the signed coupling coefficient so `invert_b_direction` is folded in, and
the radius cancels exactly - any radial motion during the sweep is
invisible to the calibration. Each counter is scaled by its step distance
directly, keeping one arbitrary zero for the session; only differences
are used.

### What gets written

* **corertheta:** `b_coupling_ratio` in `[printer]`, belt millimetres per
  degree: `ratio_new = ratio_old / scale`. The degree-valued limits of
  `[stepper_tilt]` are independent of it, and the radius is derived from
  the *difference* of the gantry positions, so R is unaffected.
* **A dedicated `[stepper_b]`:** `rotation_distance`, degrees per motor
  revolution: `rotation_distance_new = rotation_distance_old * scale`,
  for every stepper in the rail.

The ratio is baked into the step solvers at startup, so it takes effect
on the restart `SAVE_CONFIG` performs, and B must be homed again after it.

### How accurate it is

The ratio error is roughly the angle noise divided by the arc. At
0.03 degrees of measurement noise: about 2 parts in 10 000 over the full
145 degrees, 3 over 90 degrees, 15 over 20 degrees. Sweep the largest arc
the machine allows, and prefer the multi-station fit to `MODE=QUICK` -
the extra stations buy residuals, the only way to tell a wrong ratio from
a nonlinear drive.

### Reading the residuals

| Residual shape | Means |
| --- | --- |
| flat, small | the ratio is right |
| a smooth bow | the sensor calibration is off - re-run `B_SENSOR_CALIBRATE` |
| one cycle per pulley revolution | pulley eccentricity or a bent shaft |
| a step at the direction reversal | backlash - the `RETURN=1` number |
| growing toward one end | belt tension or a binding mount |
| random and large | the head is not settling; raise `SETTLE` |

A residual rms above 0.2 degrees adds a warning to the report.

## Commands

| Command | Does |
| --- | --- |
| `B_MEASURE [SETTLE=] [SAMPLE_TIME=] [FUSION=0]` | Report the fused angle from vertical, the accelerometer-only angle and their difference, the rotation rate, the mean vector and its magnitude, the in-plane and out-of-plane components, the sample deviation, whether the sensor is calibrated, and - with B homed - the commanded angle and the error. `FUSION=0` measures with the accelerometer alone, labelled as not fused. Moves nothing. |
| `B_MEASURE CHECK=1 [TOLERANCE=]` | As above, and raise an error if the fused angle is further than `TOLERANCE` (default `check_tolerance`) from commanded B. Needs B homed; refuses `FUSION=0`. For `PRINT_START`: it catches belt slip and a failed home. |
| `G28 B` | The measured home above. |
| `B_SENSOR_CALIBRATE [START=] [END=] [STEPS=] [SETTLE=] [SAMPLE_TIME=] [GAIN=0]` | Fit `offset_u`, `offset_w`, `gain_ratio`; apply; re-home. `SAVE_CONFIG` keeps them. |
| `B_STEP_CALIBRATE [START=] [END=] [STEPS=] [SETTLE=] [SAMPLE_TIME=] [RETURN=1]` | Fit the drive ratio and set it for `SAVE_CONFIG`. |
| `B_STEP_CALIBRATE MODE=QUICK [ANGLE=90]` | Two-point spot check; writes nothing. |

The status object carries `measured_b` (the last fused angle), `accel_b`,
`rotation_rate`, `fusion_disagreement`, the declared vectors with the
derived `rotation_axis` and `rotation_axis_sign`, `fusion_tau`, and the
calibration.

## Commissioning

In order, with the head clear of the bed at every angle it will visit:

1. **Bring up the BMI160** - [BMI160_IMU.md](BMI160_IMU.md#bringing-it-up).
   `BMI160_QUERY` shows a 1 g vector.
2. **Zero the gyroscope:** `BMI160_CALIBRATE GYRO=1` with the head still.
   Repeat after every power cycle.
3. **Declare the vectors** by looking, as above.
4. **Check them without homing.** With RTCP and the B projection off,
   park the head near vertical, `SET_ROTARY_AXIS AXIS=B SET_POSITION=0`,
   then `B_MEASURE`: the angle is within a few degrees of zero
   and the out-of-plane component is small. A small `G1 B` move toward
   +B, then `B_MEASURE` again: the angle moved the same way. If not, fix
   `positive_vector` first, then `invert_b_direction`. Check the
   gyroscope's sign by hand as described in
   [BMI160_IMU.md](BMI160_IMU.md#the-gyroscopes-sign-is-derived-not-configured).
5. **Set the gate.** `B_MEASURE` on a parked head reports the rotation
   rate - that is the noise floor; `max_rotation_rate` belongs just above
   it.
6. **`G28 B`** from several resting angles on both sides of zero. Each
   should finish within `zero_tolerance` in a similar number of moves.
7. **`B_SENSOR_CALIBRATE`** over the full range, then `SAVE_CONFIG`. The
   in-plane radius should be within a few percent of 1 g.
8. **`B_MEASURE` ten times without moving** - the spread should be well
   under 0.05 degrees.
9. **`B_STEP_CALIBRATE RETURN=1`**, then `SAVE_CONFIG` and `G28 B` again.
   Check the residual shape against the table above.
10. **Tune `fusion_tau` and `settle_time`.** `B_MEASURE` prints the fused
    and accelerometer-only angles side by side: they agree on a parked
    head, and lowering `settle_time` until they start to disagree shows
    how much settle the fusion really needs. `max_sample_deviation` and
    `max_rotation_rate` reject a moving head and must be relaxed to
    explore this.
11. **Add `B_MEASURE CHECK=1` to `PRINT_START`.**

## Safety

Turning B swings a tool hanging ~69 mm below the pivot through up to 145
degrees. `G28 B` runs before Z is homed, so it cannot lift: the head must
clear the bed at every angle between where it rests and B = 0. Its soft
limits and capped first move bound a wrong direction, and it finishes on
a commanded B = 0; on error the motors are turned off and B is unhomed.

`B_SENSOR_CALIBRATE` and `B_STEP_CALIBRATE` swing the head through the
whole range and do not lift Z first. They range-check `START` and `END`
(and the over-travel) before the first move, and leave the head at B = 0,
including on the error path. Raise the carriage before running them.

`B_MEASURE` moves nothing.

## Troubleshooting

| Error or symptom | Cause and fix |
| --- | --- |
| `accel_chip must name a [bmi160] section` | Only the BMI160 is supported. |
| `has its gyroscope disabled (gyro: False)` | Remove `gyro: False` from `[bmi160]`. |
| `no accelerometer samples in the measurement window` | The chip is not responding - try `BMI160_QUERY`, check wiring. |
| `only N of an expected M accelerometer samples` | The link is dropping data, or `batch_margin` needs raising. |
| `possible fifo overflows` | klipper_mcu cannot keep up - lower `rate`, or unload the Pi. |
| `the head was still moving` / `the head was turning` | Raise `settle_time`, or raise the gate if it sits on the noise floor. |
| `measured N mm/s^2 where gravity is 9807` | The head is accelerating, or the chip is misreporting. |
| `the capture is shorter than 3 x fusion_tau` | Raise `settle_time` or `sample_time`, or lower `fusion_tau`. |
| `the fused angle is ... where the accelerometer alone reads ...` | The head was moving, or the gyroscope is inverted or on the wrong axis - check the vectors and `axes_map`. |
| `B was commanded to turn +X deg and the head turned -Y` | `positive_vector` and the motors disagree about +B. |
| `the head is not following the motors` | The motors are not driving the head, or the sensor is not on its rotating part. |
| `still at B = X after N moves` | `b_coupling_ratio` is badly wrong - `B_STEP_CALIBRATE`. |
| `B homed, but the head measures X where the endstop is at Y` | Endstop rails: a sensorless home triggered without moving (`G4 P2000` first), or `positive_vector` is wrong. |
| `the sensor is uncalibrated` from `B_STEP_CALIBRATE` | Run `B_SENSOR_CALIBRATE` first. |
| `the X axis varied least ... they do not name the two axes the head turns through` | `zero_vector`/`positive_vector` name the wrong pair. |
| `the sensor fit is not believable` | The head moved during the sweep, or the mounting is not the declared one. |
| B = 0 is visibly off vertical after calibration | The sensor is rotated on its mount or the frame is out of level - neither is fitted. |
| Measured B drifts between prints | Thermal drift of the offsets; re-run `B_SENSOR_CALIBRATE` after heat soak. |

## Code and tests

    klippy/extras/accel_b_homing.py          the module
    klippy/extras/bmi160.py                  the sensor
    klippy/kinematics/rotary_axis.py         set_homing_source(), home()
    klippy/kinematics/corertheta.py          B rail without an endstop, step hooks
    klippy/stepper.py                        need_endstop=False rails
    test/multi_axis/test_accel_b_homing.py   host tests

The module is split so the mathematics can be tested on a host that
cannot run klippy: pure functions (`measure_angle()`,
`gyro_axis_coefficient()`, `ComplementaryFilter`, `fit_ellipse()`,
`fit_drive_sweep()`, ...), the measurement primitive, and the routines.
Run the tests with `python test/multi_axis/test_accel_b_homing.py`. They
drive the real module against a stubbed printer and a synthetic BMI160
whose accelerometer and gyroscope agree about how the head is turning,
covering config validation, every gate, the filter and sign convention
for all 24 mountings, `B_MEASURE`, `G28 B` against a simulated head
whose drive is exact, short, reversed, stalled or wildly long, both fits
against known offsets, gain, scale and backlash, and the kinematics'
step hooks.

## Not implemented

* **A zero against a physical reference.** Nothing takes out a sensor
  rotated about the B axis or a frame out of level with gravity. A
  `B_SET_ZERO` - declare B = 0 with the nozzle squared to the bed or the
  probe pin hanging vertical - would, but which reference to standardise
  on is not settled.
* **Automatic checks.** `B_MEASURE CHECK=1` has to be added to
  `PRINT_START` by hand; an opt-in config option would be harder to
  forget.
* **Backlash compensation.** `RETURN=1` measures it; acting on it belongs
  in the kinematics.
* **A lift before calibration sweeps.** Raise the carriage by hand, or in
  a macro.
