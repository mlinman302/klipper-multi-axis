# Gravity-referenced B axis: homing and step calibration

> **The sensor is now a BMI160 IMU**, and the angle is now *fused*.
> `[accel_b_homing]`'s `accel_chip` defaults to `bmi160`, and the
> gyroscope is combined with the accelerometer through a complementary
> filter: the accelerometer keeps the angle absolute over long
> timescales, the gyroscope carries it through the ringing that follows
> a move. The fused angle is the authoritative one; the
> accelerometer-only angle this document describes is still computed
> and still reported beside it. The same gyroscope supplies the motion
> gate (`max_rotation_rate`) - a direct test that the head was at rest,
> where `max_sample_deviation` below can only infer it.
>
> Everything below about mounting and the zero reference is unchanged
> and chip-independent - and it now does more work, because which
> gyroscope axis carries dB/dt, and with which sign, is *derived* from
> `zero_vector` and `positive_vector` rather than separately configured.
> The BMI160 also changes the accuracy argument below: its fast offset
> compensation trims the zero-g offset in hardware to 3.9 mg, so a large
> part of the phase two offset fit is done by the chip. See
> [BMI160_IMU.md](BMI160_IMU.md).

**Status: phase one is implemented** - `[accel_b_homing]` and the
`B_MEASURE` command (`klippy/extras/accel_b_homing.py`). Phases two to
five below are still design.

This document describes how an ADXL345 mounted on the tilting head can

  1. **home the B axis** by *measuring* its absolute angle against gravity
     instead of sweeping it into an endstop, and
  2. **calibrate the B drive ratio** - the motor steps that correspond to
     one degree of head rotation - by commanding known rotations and
     measuring what actually happened.

The two features share one primitive (read the head's tilt) and one
calibration (the sensor's own mounting and offsets), so they belong in a
single module.

## Why bother

B on the corertheta machine is the awkward axis:

* It homes on a **TMC StallGuard virtual endstop** shared with the R
  gantry pair. A home issued too soon after another move triggers
  instantly, the axis does not move, no error is raised, and B is still
  reported homed - see the `HOME_B` macro's `G4 P2000` in
  `config/example-corertheta.cfg`. A silent wrong home on B is expensive:
  `[rtcp]` turns every B error into an X/Z error at the nozzle.
* Its zero is a *physical* claim - "the nozzle points straight down" -
  that a switch or a stall threshold only approximates. `[rtcp]` and
  `[bltouch] b_offset` are both built on that claim.
* Its drive ratio is not directly measurable with a ruler. B is coupled:
  both gantry motors turn for a B move, through a differential, and the
  belt travel per degree is the `b_coupling_ratio` option of `[printer]`.
  It is currently a nominal `1.0`, unverified against the machine.

An accelerometer at rest reads the gravity vector. A gravity vector in
the head's own frame *is* the head's tilt, absolutely, with no reference
to where the axis has travelled. That makes homing an observation rather
than a move, gives an independent check on the stall home, and turns the
drive ratio into something measurable.

## Hardware

The board in use is a **Fly-ADXL345-USB**: an ADXL345 plus an onboard
RP2040 that runs Klipper firmware and enumerates as a USB serial device.
There is nothing to wire - the SPI link to the sensor is internal to the
board, between the RP2040 and the ADXL345.

That makes it a **secondary Klipper MCU**, not a USB sensor with a driver
of its own. Three consequences:

* **Flash it from this tree.** The RP2040 runs Klipper firmware built
  for `rp2040`, and host and MCU should be the same version. Building it
  from this fork rather than from stock keeps them in step.
* **It gets an `[mcu]` section**, and `[adxl345]` addresses its pins
  through that MCU's name. The vendor's configuration uses **software
  SPI**, not `spi_bus`:

  ```ini
  [mcu adxl]
  serial: /dev/serial/by-id/usb-Klipper_rp2040_XXXXXXXXXXXX-if00

  [adxl345]
  cs_pin: adxl:gpio9
  spi_software_sclk_pin: adxl:gpio10
  spi_software_mosi_pin: adxl:gpio11
  spi_software_miso_pin: adxl:gpio12
  ```

* **The link adds latency**, which is the one thing in this module that
  had to change for it - see `batch_margin` below.

None of this reaches `accel_b_homing.py`. The module looks the chip up by
name and calls `start_internal_client()`; it never touches a bus, a pin
or an MCU. A USB board, a CAN toolhead and a chip wired straight to the
mainboard's SPI are all the same to it, and so is a different chip
entirely - `[lis2dw]`, `[mpu9250]` and `[lis3dh]` all expose the same
interface and the same `data_rate` attribute.

### What the USB link does change: `batch_margin`

The bulk sensor helpers deliver samples to the host in batches (0.100 s
in `adxl345.py`), and a secondary MCU adds a link's worth of latency on
top. `finish_measurements()` waits for the *moves* to finish, not for the
sensor batches to arrive, so the batch carrying the tail of the averaging
window has usually not been delivered when the samples are asked for.

The fix is a trailing dwell - `batch_margin`, 0.3 s by default - after
the averaging window and before `finish_measurements()`, so the window is
comfortably in the past by the time it is read. Losing the tail is
harmless on a long window, but a short one on a laggy link loses enough
of itself to trip the routine's own data-loss check: with a 0.2 s window
and 0.15 s of delivery lag, three quarters of the window is missing. On a
chip wired directly to the mainboard the margin costs 0.3 s and changes
nothing else; it is not worth making the distinction in config.

Mounting:

* Rigid to the **rotating** part of the head, not the carriage. Any
  compliance in the mount is measured as head tilt.
* Mount it **square**: the B rotation axis parallel to one sensor axis,
   and the other two spanning the plane the head tilts in. Which axis is
   which does not matter - that is declared, see below - but phase one
   has no way to take out a skewed mounting, so a sensor glued on two
   degrees out reads two degrees out.
* Route the cable so a 145-degree swing does not tug it. A cable that
  pulls on the head is a systematic angle error, and a USB cable is
  stiffer than the ribbon a directly-wired sensor would use - this
  matters more here than it would on a bench.
* The board is bus powered, so there is no separate supply to route.

The chip can be shared with `[resonance_tester]`; this module does not
require any particular `axes_map`, so input shaping keeps whatever
mapping it needs.

## The physics, and the error budget

With the head at rest the sensor reads `-g` in head coordinates. Pick the
two sensor axes that span the tilt plane and call them `u` and `w`; the
third, `v`, is the out-of-plane axis parallel to the B rotation axis. As
B turns, `(u, w)` traces a circle of radius 1 g and `v` stays constant.
The head angle is

    phi = atan2(u - u0, w - w0)          (corrected for offsets/gains)
    B   = sign * (phi - phi_zero)

That is the general form. Phase one runs it with the corrections at their
identity values and the zero fixed to an axis direction, which is what
the next section describes.

Using **both** in-plane axes through `atan2` - rather than one axis
through `asin` - is what makes the resolution uniform across the whole
range. A single-axis reading goes flat near +/-90 degrees; `atan2` does
not.

`v` is not wasted: it is the health check. It must stay constant across
the sweep, and `u^2 + w^2 + v^2` must stay at 1 g.

### Declaring the zero, phase one

The mounting is **declared, not fitted**, and is restricted to the six
axis-aligned directions. Two signed sensor axes say everything the
measurement needs:

    zero_vector      the sensor axis that reads +1 g at B = 0
    positive_vector  the sensor axis that reads +1 g at B = +90

An accelerometer at rest reads the specific force, which points *up*, so
both of these are "the sensor axis pointing straight up" at their
respective angles - and both are found by looking rather than by
measuring anything. Park the head, run `ACCELEROMETER_QUERY`, note which
axis reads about +9800 mm/s^2, and use the negated name if it reads about
-9800.

Then, with `w` the reading along `zero_vector` and `u` the reading along
`positive_vector`,

    B = atan2(u, w)

which is 0 at B = 0 and +90 at B = +90 by construction. That is the whole
zero reference: no `zero_angle` offset, and no separate inversion flag -
the sign convention falls out of which direction `positive_vector` names.
The third axis is the rotation axis, by elimination.

The cost is that the zero is quantised to the six axis directions, so a
sensor glued on a couple of degrees out is a couple of degrees out. That
is a phase two problem: the fine offset is one more number on top of this
reference, not a replacement for it.

### What limits accuracy

| Source | Raw magnitude | Angle effect | Handled by |
| --- | --- | --- | --- |
| Quantisation (3.9 mg/LSB, full-res) | 1 LSB | ~0.22 deg per sample | averaging |
| Broadband noise at 3200 Hz ODR | ~10-20 mg rms | ~0.9 deg per sample | averaging (~0.02-0.05 deg over 0.5 s) |
| **Zero-g offset** | **up to +/-150 mg** | **up to ~8.6 deg** | **ellipse fit - mandatory** |
| Inter-axis gain mismatch | up to ~10 % | several deg, angle dependent | ellipse fit |
| Cross-axis sensitivity | ~1 % | ~0.5 deg | optional 2x2 fit |
| Offset drift with temperature | order 1 mg/degC | tenths of a deg | re-zero after heat soak |
| Residual head motion / ringing | - | unbounded | settle dwell + reject on stddev |
| Machine frame not level with gravity | - | direct bias | `level_offset` |

The row that decides the architecture is the zero-g offset. Uncorrected,
an ADXL345 can be **eight degrees** wrong about which way is down. Raw
`atan2` on raw counts is not usable for this job; a per-sensor offset and
gain calibration is not a refinement, it is a precondition. Everything
else is comfortably sub-tenth-degree with half a second of averaging -
which is already better than a belt-driven differential's mechanical
repeatability.

## Where the code lives

    klippy/extras/accel_b_homing.py          new - the whole feature
    klippy/kinematics/rotary_axis.py         small hook: home override
    config/example-corertheta.cfg            new section, HOME_B rewrite
    docs/Config_Reference.md                 new section
    docs/Multi_Axis.md                       cross-reference
    test/multi_axis/test_accel_b_homing.py   new - host tests

Config section name `[accel_b_homing]`; the printer object is the sensor,
and the commands it registers are named for the axis (`B_*`).

The module splits into three layers, deliberately:

1. **Pure functions** (module level, no printer dependency):
   `parse_signed_axis()`, `out_of_plane_index()`, `measure_angle()`,
   `summarize()` today, joined by `fit_gravity_ellipse()`, `unwrap()` and
   `fit_deg_per_step()` in the later phases. All the mathematics lives
   here so it can be unit tested on a host that cannot run klippy -
   which, on the Windows dev box, is all of them.
2. **The measurement primitive**: `measure()` - dwell, sample, filter,
   average, validate, return a `TiltReading`.
3. **The routines**: homing, sensor calibration, step calibration, and
   the g-code commands that drive them.

### The hook into `G28 B`

`PrinterHoming.cmd_G28` already ends with `for ea in extra_axes:
ea.home()`, so the cleanest integration is a one-method override point on
`BaseRotaryAxis`:

```python
# rotary_axis.py
def set_home_override(self, cb):      # called by accel_b_homing at connect
    self.home_override = cb
def home(self):
    if self.home_override is not None:
        return self.home_override()
    ... existing endstop sweep ...
```

That is about ten lines of core change, no monkeypatching, and `G28 B`,
`M400`, the homed-state bookkeeping and the error path all keep working
unchanged. `B_HOME` is registered as well, so the routine is reachable
without going through `G28`.

## Config surface

Phase one, as implemented:

```ini
[accel_b_homing]
zero_vector: +z               # sensor axis reading +1 g at B = 0
positive_vector: +x           # sensor axis reading +1 g at B = +90
#accel_chip: adxl345          # any chip exposing start_internal_client()
#settle_time: 0.250           # dwell before sampling, s
#sample_time: 0.500           # averaging window, s
#batch_margin: 0.300          # trailing dwell for late batches, s
#max_sample_deviation: 500    # mm/s^2; above this the head was moving
#max_magnitude_error: 1500    # mm/s^2; |a| must be 1 g within this
#check_tolerance: 5.0         # deg; default for B_MEASURE CHECK=1
```

Only the two vectors are required. The two gates are in mm/s^2, the units
the chips report, and both are disabled by setting them to zero.

`max_magnitude_error` defaults deliberately loose. With no gain
calibration yet, a chip inside its +/-10 % sensitivity spec legitimately
reads 0.9 to 1.1 g; the gate is there to catch a head that is
accelerating or a chip that is not reporting properly, not to grade the
sensor. `check_tolerance` is loose for the same reason - see *What phase
one does not do* below.

Phases two to five add:

```ini
# --- sensor calibration (written by B_SENSOR_CALIBRATE / SAVE_CONFIG) ---
zero_offset: 0.0              # deg; fine offset on top of zero_vector
offset_u: 0.0                 # zero-g offset, mm/s^2
offset_w: 0.0
gain_ratio: 1.0               # amplitude(w) / amplitude(u)
level_offset: 0.0             # frame tilt vs gravity, deg (usually 0)

# --- homing ---
home_method: accelerometer    # accelerometer | endstop | endstop_then_accel
home_tolerance: 0.10          # deg; refine loop convergence
max_home_iterations: 3
verify_move: 5.0              # deg; 0 disables the liveness check
endstop_agreement: 3.0        # deg; endstop_then_accel disagreement limit

# --- safety ---
min_safe_z: 40.0              # refuse to swing B below this Z
```

`zero_offset`, the offsets and `gain_ratio` are outputs of the
calibration routines, written back through `configfile.set()` and
`SAVE_CONFIG`, exactly as `PROBE_CALIBRATE` and `delta_calibrate` do.
`zero_vector` and `positive_vector` are not: they are the coarse frame
the fitted numbers sit on top of, and they stay hand-declared.

### What phase one does not do

The reading is **uncorrected**. An ADXL345 has a zero-g offset of up to
+/-150 mg and an inter-axis gain tolerance of about +/-10 %, which
together are worth several degrees of absolute error. Until the offset
and gain fit of phase two lands, treat `B_MEASURE` as a diagnostic, not
as a calibrated angle.

Repeatability is a different matter, and is already good: a stationary
head re-measures to a few hundredths of a degree. That is why the noise
gates are tight while the absolute tolerances are loose, and it is what
makes `B_MEASURE CHECK=1` useful as a gross-error detector - a home that
silently did not move is tens of degrees out, not tenths.

## G-code surface

| Command | Does |
| --- | --- |
| `B_MEASURE [SETTLE=] [SAMPLE_TIME=]` | *(implemented)* Report measured B, the raw vector, the in-plane and out-of-plane components, the noise stats, and the error against commanded B. Read-only, safe any time the head is still. |
| `B_MEASURE CHECK=1 [TOLERANCE=]` | *(implemented)* As above, but raise an error if measured and commanded B disagree. For `PRINT_START` and layer macros - it catches belt slip and a silently failed home. Requires B homed, since an unhomed B has no commanded angle to compare against. |
| `B_HOME` | Set B from the measurement (also what `G28 B` calls). |
| `B_SET_ZERO` | Declare the current physical pose to be B = 0. Run it with the head referenced mechanically - square against the bed, or with the probe pin hanging vertical. |
| `B_SENSOR_CALIBRATE [START=] [END=] [STEPS=]` | Sweep the arc, fit offsets and gain, report conditioning. |
| `B_STEP_CALIBRATE [START=] [END=] [STEPS=] [RETURN=1]` | The drive ratio routine. `RETURN=1` sweeps back to measure backlash. |
| `B_STEP_CALIBRATE MODE=QUICK ANGLE=90` | Two-point spot check. Requires a valid sensor calibration. |

## Algorithm: the measurement primitive

```
measure(settle, window) -> TiltReading
  1. toolhead.wait_moves()
  2. client = chip.start_internal_client()
     t0 = toolhead.get_last_move_time()
     toolhead.dwell(settle + window + batch_margin)
     client.finish_measurements()
  3. keep samples with t0 + settle <= t <= t0 + settle + window
  4. reject if: no samples, count < 0.5 * expected,
                stddev(any axis) > max_sample_deviation,
                | |a| - 1g | > max_magnitude_error
  5. mean -> (u, w, v); B = atan2(u, w)
     (phase two: apply offsets, gain_ratio, zero_offset, level_offset)
  6. return B, raw mean, stddev, in-plane and out-of-plane parts, count
```

Three details that matter:

* **Neither transform has to be off.** The design started out refusing to
  measure with `[rtcp]` or `[b_projection]` enabled, following what
  `homing.py` does. That turned out to be unnecessary and it cost the
  most useful application - a mid-print sanity check. `[rtcp]` moves x
  and z, never B, so it does not affect this at all. `[b_projection]`
  does change the meaning of the commanded B, so the comparison converts:
  `project_pos()` maps the commanded bed-frame angle onto the plane the
  head can tilt in, which is exactly the angle the sensor sees. The
  measurement itself never needed either transform off - it reads a
  physical head angle - and only the comparison did.

* **The settle window is separate from the averaging window.** The head
  hangs on belts through a differential; it rings after a move. Sampling
  starts only after `settle_time`. `AccelQueryHelper` trims samples to
  the request window on its own, but not to a settle offset, so the
  filtering in step 3 is explicit.
* **And so is the trailing margin**, which is not sampled at all - it
  exists only to let the last batches arrive. See `batch_margin` above.
* **Rejection is loud.** A reading taken while the head is drifting is
  worse than no reading, because it silently poisons a home. The stddev
  and magnitude gates exist to convert "head still moving" into an error
  rather than a wrong number.

## Algorithm: homing

```
B_HOME
  1. refuse unless the transforms are off (the same check G28 already runs)
  2. ensure the B motors are energised - a head that can droop unpowered
     must be held before it is measured
  3. optional liveness check (verify_move != 0):
       measure -> B0 ; move B by +verify_move ; measure -> B1
       require |(B1 - B0) - verify_move| < 2 deg, else raise
       "the accelerometer is not responding to B motion"
  4. measure -> B_meas
  5. range check against the axis pos_min / pos_max
  6. axis.set_position(B_meas); is_homed = True
  7. optional refinement (home_tolerance > 0):
       repeat up to max_home_iterations:
         move to B = 0; measure; if |B| <= tolerance: done
         else set_position(measured) and go again
```

Step 7 is what makes homing tolerant of a wrong `b_coupling_ratio`:
setting the position is exact regardless of the ratio, and the refine
loop closes the gap between "where I said to go" and "where I ended up".
Two iterations converge for any ratio error under ~10 %. It is also a
diagnostic - if the loop does not converge, the ratio is badly wrong, and
the message should say so and point at `B_STEP_CALIBRATE`.

**`home_method: endstop_then_accel`** runs the existing stall-detect home
first and then measures. If the measurement disagrees with
`position_endstop` by more than `endstop_agreement`, the endstop home is
declared failed - which is precisely the silent-no-op failure mode the
StallGuard pair has today. This is the recommended mode while the sensor
is being trusted, because it keeps the mechanical hard reference and adds
detection of the known bug. `accelerometer` mode, once trusted, is
strictly better: no swing, no stall threshold, no `G4 P2000`, and an
absolute rather than a relative zero.

## Algorithm: sensor calibration

The offsets and gains cannot come from the datasheet - they are
per-device and up to 150 mg. They are fitted from data.

```
B_SENSOR_CALIBRATE START=-40 END=90 STEPS=14
  for each station:
      move B, settle, measure raw mean (u, w, v)
  fit the axis-aligned ellipse
      (u - u0)^2 / Au^2 + (w - w0)^2 / Aw^2 = 1
  as the linear least squares problem
      u^2 + B*w^2 + C*u + D*w + E = 0
  giving u0 = -C/2, w0 = -D/(2B), Au^2 = u0^2 + B*w0^2 - E, Aw = Au/sqrt(B)
  write offset_u = u0, offset_w = w0, gain_ratio = Aw / Au
```

Conditioning depends on the arc swept, so the fit is tiered and the
routine picks the model the data can support:

| Arc | Model | Fitted |
| --- | --- | --- |
| >= 120 deg | full | both offsets + gain ratio |
| 60-120 deg | reduced | both offsets, gain ratio forced to 1 |
| < 60 deg | zero only | `zero_offset` only; offsets left alone |

The corertheta B range (-45 to 100, ie 145 degrees) supports the full
fit, which is a happy accident worth stating: this machine can
self-calibrate its own sensor.

**Bench alternative.** The cleanest calibration is a full 360-degree
rotation, which the head cannot do but a hand can. Recording the sensor
while slowly turning it through a complete turn *before* mounting gives
an unconditionally well-posed fit. If that is done, the on-machine
routine only has to find `zero_offset`, and `B_SET_ZERO` is the whole
procedure.

The sweep also **verifies** `zero_vector` and `positive_vector`, which
stay hand-declared: it reports which sensor axis pair actually varied
(the in-plane pair), which stayed put (`v`), and the sign of the change
against commanded B. Getting those two flags wrong is the most likely
setup mistake, so the routine should say plainly when the data disagrees
with what was declared.

## Algorithm: B step calibration

The question is *degrees of real head rotation per motor step*. The
routine answers it by commanding a rotation, counting the steps the MCU
actually issued, and measuring the rotation that resulted.

```
B_STEP_CALIBRATE START=-40 END=90 STEPS=14 RETURN=1
  0. preconditions: transforms off, Z above min_safe_z, motors on,
     sensor calibration valid, START/END inside pos_min/pos_max
  1. move to START, then over-travel and come back, so every station in
     the sweep is approached from the same direction (backlash out of
     the primary fit)
  2. for each station i:
       move to the commanded B
       record n_i = the integer MCU step counters (see below)
       settle, measure -> phi_i (corrected angle, not commanded B)
  3. unwrap phi, then least squares  phi_i = alpha * n_i + beta
       alpha = degrees per step        <- the answer
  4. residuals: report RMS and max
  5. if RETURN=1, sweep back and report the offset between the two fits
     at matching stations - that is the backlash, in degrees
  6. convert alpha to the config parameter the machine actually has,
     and offer it through SAVE_CONFIG
```

### Counting the steps actually applied

The user-visible question is "how many steps did that take", and the
honest answer is the integer step counter, not the commanded float
position - it includes step rounding, and it is what the ratio is
ultimately *about*. Klipper exposes it per stepper:

```python
mcu_pos = mcu_stepper.get_mcu_position()                    # integer steps
pos_mm  = mcu_stepper.mcu_to_commanded_position(mcu_pos)    # signed, in mm
```

`mcu_to_commanded_position()` already folds in `dir_pin` inversion and
step distance, so it is the safe converter. (`rotary_axis.py` uses the
same pair in `find_past_position()`.)

**The coupled case has a property worth exploiting.** On corertheta the
two gantry solvers are

    stepper_r     p_minus = b * ratio - radius
    stepper_tilt  p_plus  = b * ratio + radius

so their **sum depends only on B**:

    b_belt_mm = 0.5 * (p_plus + p_minus) = b * ratio

The radius cancels exactly. The calibration therefore reads both gantry
step counters, sums them, and is immune to any radial motion during the
sweep - including whatever the arm does while getting out of the way.
That is much better than trying to hold R perfectly still.

### What gets written

* **corertheta / coupled B.** The parameter is `b_coupling_ratio` in
  `[printer]` - belt millimetres per degree:

      ratio_new = ratio_old * (B_commanded_span / B_measured_span)

  Nothing else needs touching. The degree-valued `position_endstop`,
  `position_min` and `position_max` of `[stepper_tilt]` are independent
  of the ratio, and `calc_position()` derives the radius from the
  *difference* of the two gantry positions, so R is unaffected. Write it
  with `configfile.set('printer', 'b_coupling_ratio', ...)`.

* **A dedicated `[stepper_b]`.** The parameter is `rotation_distance`
  (degrees per motor revolution):

      rotation_distance_new = rotation_distance_old
                              * (B_measured_span / B_commanded_span)

### How accurate is it

The ratio error is roughly the angle measurement error divided by the arc
swept. At 0.03 degrees of measurement noise:

* over the full 145-degree range: ~2 parts in 10 000 (0.02 %)
* over 90 degrees: ~3 parts in 10 000
* over 20 degrees: ~15 parts in 10 000 (0.15 %)

So **sweep the largest arc the machine allows**, and prefer the
multi-station fit over a two-point measurement - the extra stations buy
residuals, which are the only way to tell a wrong ratio from a nonlinear
drive.

### Reading the residuals

The residual pattern is the real diagnostic value of this routine:

| Residual shape | Means |
| --- | --- |
| flat, small | ratio is right |
| linear trend not removed by the fit | wrong zero reference, not a ratio problem |
| one cycle per pulley revolution | pulley eccentricity or a bent shaft |
| step at the direction reversal | backlash - the `RETURN=1` number |
| growing toward one end | belt tension or a binding mount |
| random and large | the head is not settling; raise `settle_time` |

## Interaction with the rest of the fork

* **The measured B is the machine B** - the angle the head is really
  turned to - which is exactly the quantity `[rtcp]` consumes and the
  quantity `[bltouch] b_offset` is expressed in.
* **Measuring needs no transform off.** `B_MEASURE` reads a physical
  angle, so it works in any mode; the *comparison* against the commanded
  angle converts through `b_projection.project_pos()`. See "the
  measurement primitive" above.
* **Every routine that moves B does need both off**, for exactly the
  reasons `homing.py` already documents: with RTCP on, a B move is also
  an X/Z move, and with the projection on, a commanded B is scaled by
  whatever bed angle the arm is over. Those routines reuse
  `check_disabled()` on both objects and produce the same class of error
  message.
* **`invert_b_direction` stays the one place rotation sense is set.** If
  the sensor disagrees with the machine's sign convention, that is
  recorded by negating `positive_vector` here - and phase two's sweep
  reports when the two disagree. The kinematics' own inversion is
  untouched.
* **This closes an open verification item.** Whether a positive B really
  tilts the nozzle outboard is still unverified on the machine.
  `B_MEASURE` answers it directly, without RTCP in the loop: command a
  positive B with the transforms off and read which way the head actually
  went.  This is available now.

## Safety

Rotating B swings a tool that hangs ~69 mm below the pivot through an arc
of up to 145 degrees. Every routine that moves B:

* requires Z homed and lifts to `min_safe_z` first, or refuses -
  `[rtcp_probe]` already does this with `orient_lift_z`, and the same
  approach applies;
* range checks `START` and `END` against the axis limits before the first
  move, not station by station;
* leaves the head at a defined angle on exit, including on the error
  path.

`B_MEASURE` moves nothing and needs none of this.

## Failure modes and what the user should see

| Symptom | Message should say |
| --- | --- |
| No samples | the chip is not responding; try `ACCELEROMETER_QUERY` |
| Fewer samples than expected | the link is dropping data, or is slow enough that `batch_margin` needs raising |
| High stddev | the head was still moving; raise `settle_time` |
| Magnitude far from 1 g | the head is moving, or the chip is misconfigured |
| `v` varies across the sweep | `zero_vector`/`positive_vector` are wrong - here is the pair that did vary |
| In-plane radius well below 1 g | the B axis is not in the fitted plane - remount or re-run the sweep |
| Refine loop will not converge | `b_coupling_ratio` is wrong - run `B_STEP_CALIBRATE` |
| Endstop and sensor disagree | the stall home did not move the axis (the known `G4 P2000` failure) |
| Measured B drifts between prints | thermal offset drift; re-run `B_SET_ZERO` after heat soak |

## Testing

The dev box cannot run klippy, so the test split follows the code split:

* **Host, `test/multi_axis/test_accel_b_homing.py`** (32 tests, run it
  with `python test/multi_axis/test_accel_b_homing.py`).  It drives the
  *real* module against a stubbed printer and a synthetic accelerometer,
  following `test_rtcp_probe.py`, so the whole of phase one is covered on
  a host that cannot run klippy - config validation, the sample window
  and its settle offset, every rejection path, the angle at the reference
  poses, noise averaging, and the `B_MEASURE` report and its
  `b_projection`-aware comparison.
  Later phases add the fits: synthesise `(u, w)` points with known
  offsets, gain mismatch and gaussian noise and assert the ellipse fit
  recovers them; assert the angle unwrap survives a sweep through
  +/-180; assert `fit_deg_per_step` recovers a known slope.
* **`test/klippy/`** - a `multi_axis_accel_b.cfg` plus `.test` for config
  parsing through the real config machinery, and for the refusals the
  later phases add. Sensor data cannot be simulated in that harness, so
  the measurement path itself stays covered by the host test.
* **On the machine** - the acceptance runs, in order:
  1. `ACCELEROMETER_QUERY` returns a plausible 1 g vector.
  2. `B_MEASURE` with the head parked at B = 0 reads near zero, and the
     out-of-plane component is small - if not, the two vector flags are
     wrong, and the reported vector says what they should be.
  3. `B_SENSOR_CALIBRATE` over the full range; the reported in-plane
     radius is within a few percent of 1 g and `v` is flat.
  4. `B_MEASURE` repeated ten times without moving - spread under 0.05
     degrees.
  5. `B_MEASURE` after commanding several angles - measured tracks
     commanded to within the ratio error.
  6. `B_STEP_CALIBRATE`; compare the fitted ratio against the nominal
     1.0, and check the residual shape against the table above.
  7. `home_method: endstop_then_accel` for a while, watching for the
     disagreement error - it should fire on exactly the back-to-back
     homes that fail today.

## Implementation order

Each phase is independently useful and independently shippable.

1. **Read-only** - *done*. `[accel_b_homing]`, the measurement
   primitive, `B_MEASURE`. Nothing moves; the head can be turned by hand.
   This alone answers the outstanding rotation-sense question.
2. **Sensor calibration.** `B_SENSOR_CALIBRATE`, `B_SET_ZERO`, the
   ellipse fit, `SAVE_CONFIG` write-back. After this the measurement is
   trustworthy in absolute terms.
3. **Step calibration.** `B_STEP_CALIBRATE`, step counting, the residual
   report, the ratio write-back. Uses only phases 1-2 and the existing
   endstop home.
4. **Homing.** The `rotary_axis` hook, `B_HOME`, `home_method`, the
   refine loop, `endstop_then_accel`. Last, because it is the only phase
   that changes existing behaviour, and because phases 1-3 are what make
   it safe to trust.
5. **Guard rail.** `B_MEASURE CHECK=1` in `PRINT_START`, and optionally a
   periodic check between layers.

## Open questions

* **Is the frame level with the bed?** The sensor measures against
  gravity; `[rtcp]` cares about the machine Z. `level_offset` exists to
  hold the difference, but nothing currently measures it. The bed mesh
  already knows the bed plane in machine coordinates - deriving
  `level_offset` from a mesh is possible and would remove a manual step.
  Left out of this design deliberately; worth revisiting once phase 2 is
  on the machine.
* **Should `B_MEASURE CHECK=1` be automatic?** A cheap check before every
  print start is attractive. It costs about a second and it catches a
  failure that currently ruins prints silently. That argues for an opt-in
  config option (`check_before_print: True`) rather than a macro the user
  has to remember.
* **Backlash compensation, or just reporting?** The `RETURN=1` sweep
  measures it. Acting on it is a separate feature and probably belongs in
  the kinematics, not here.
* **A finer zero than the six axis directions.** Phase one quantises the
  zero reference to +/-x/y/z, so a sensor glued on a degree or two out is
  a degree or two out. `zero_offset` in phase two is the fix, but it
  needs a physical reference to be set against - a machinist's square on
  the nozzle face, or the probe pin hanging vertical. Which of those to
  standardise on is not settled.
