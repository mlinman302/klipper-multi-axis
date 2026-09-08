# Gravity-referenced B axis: homing and step calibration

**Status: design outline. No code is implemented on this branch yet.**

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

The linked board is a generic ADXL345 breakout. Two things to verify
before wiring:

* **Klipper's `[adxl345]` is SPI only** (`bus.MCU_SPI_from_config` in
  `klippy/extras/adxl345.py`). The board's I2C mode cannot be used. Wire
  CS / SCLK / SDI (MOSI) / SDO (MISO) plus power and ground.
* **Logic level.** Confirm the board's logic is 3.3 V, or that it carries
  level shifting, before connecting it to a 5 V MCU.

Mounting:

* Rigid to the **rotating** part of the head, not the carriage. Any
  compliance in the mount is measured as head tilt.
* The sensor's package axes need not line up with anything. The module
  learns the mapping (see *Sensor calibration*). What matters is that
  **two of the three sensor axes span the plane the head tilts in** - ie
  the B rotation axis should be roughly parallel to one sensor axis. A
  45-degree skew still works but costs sensitivity.
* Route the cable so a 145-degree swing does not tug it. A cable that
  pulls on the head is a systematic angle error.

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

Using **both** in-plane axes through `atan2` - rather than one axis
through `asin` - is what makes the resolution uniform across the whole
range. A single-axis reading goes flat near +/-90 degrees; `atan2` does
not.

`v` is not wasted: it is the health check. It must stay constant across
the sweep, and `u^2 + w^2 + v^2` must stay at 1 g.

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

    klippy/extras/b_inclinometer.py          new - the whole feature
    klippy/kinematics/rotary_axis.py         small hook: home override
    config/example-corertheta.cfg            new section, HOME_B rewrite
    docs/Config_Reference.md                 new section
    docs/Multi_Axis.md                       cross-reference
    test/multi_axis/test_b_inclinometer.py   new - pure-math unit tests

Config section name `[b_inclinometer]`; the printer object is the sensor,
and the commands it registers are named for the axis (`B_*`).

The module splits into three layers, deliberately:

1. **Pure functions** (module level, no printer dependency):
   `fit_gravity_ellipse()`, `angle_from_vector()`, `unwrap()`,
   `fit_deg_per_step()`. All the mathematics lives here so it can be unit
   tested on a host that cannot run klippy - which, on the Windows dev
   box, is all of them.
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
def set_home_override(self, cb):      # called by b_inclinometer at connect
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

```ini
[b_inclinometer]
accel_chip: adxl345           # any chip exposing start_internal_client()
axis: b                       # which rotary axis this measures

# --- sensor mounting (written by B_SENSOR_CALIBRATE / SAVE_CONFIG) ---
plane_axes: x, z              # the two sensor axes spanning the tilt plane
invert: False                 # True if +B decreases the measured angle
zero_angle: 0.0               # raw phi (deg) at true B = 0
offset_u: 0.0                 # zero-g offset, in g
offset_w: 0.0
gain_ratio: 1.0               # amplitude(w) / amplitude(u)
level_offset: 0.0             # frame tilt vs gravity, deg (usually 0)

# --- measurement ---
settle_time: 0.250            # dwell before sampling, s
sample_time: 0.500            # averaging window, s
max_sample_stddev: 0.030      # g; above this the head was still moving
max_magnitude_error: 0.050    # g; |a| must be 1 g within this

# --- homing ---
home_method: accelerometer    # accelerometer | endstop | endstop_then_accel
home_tolerance: 0.10          # deg; refine loop convergence
max_home_iterations: 3
verify_move: 5.0              # deg; 0 disables the liveness check
endstop_agreement: 3.0        # deg; endstop_then_accel disagreement limit

# --- safety ---
min_safe_z: 40.0              # refuse to swing B below this Z
```

`plane_axes`, `invert`, `zero_angle`, the offsets and `gain_ratio` are all
outputs of the calibration routines, written back through
`configfile.set()` and `SAVE_CONFIG`, exactly as `PROBE_CALIBRATE` and
`delta_calibrate` do.

## G-code surface

| Command | Does |
| --- | --- |
| `B_MEASURE` | Report measured B, the raw vector, the noise stats, and the error against commanded B. Read-only, safe any time the head is still. |
| `B_MEASURE CHECK=1 TOLERANCE=0.5` | As above, but raise an error if measured and commanded B disagree. For `PRINT_START` and layer macros - it catches belt slip and a silently failed home. |
| `B_HOME` | Set B from the measurement (also what `G28 B` calls). |
| `B_SET_ZERO` | Declare the current physical pose to be B = 0. Run it with the head referenced mechanically - square against the bed, or with the probe pin hanging vertical. |
| `B_SENSOR_CALIBRATE [START=] [END=] [STEPS=]` | Sweep the arc, fit offsets and gain, report conditioning. |
| `B_STEP_CALIBRATE [START=] [END=] [STEPS=] [RETURN=1]` | The drive ratio routine. `RETURN=1` sweeps back to measure backlash. |
| `B_STEP_CALIBRATE MODE=QUICK ANGLE=90` | Two-point spot check. Requires a valid sensor calibration. |

## Algorithm: the measurement primitive

```
measure(settle, window) -> TiltReading
  1. rtcp.check_disabled("B measurement")
     b_projection.check_disabled("B measurement")
  2. toolhead.wait_moves()
  3. t0 = toolhead.get_last_move_time()
     client = chip.start_internal_client()
     toolhead.dwell(settle + window)
     client.finish_measurements()
  4. keep samples with t0 + settle <= t <= t0 + settle + window
  5. reject if: no samples, count < 0.5 * expected,
                stddev(any axis) > max_sample_stddev,
                | |a| - 1g | > max_magnitude_error
  6. mean -> (u, w, v); apply offsets and gain_ratio;
     phi = atan2(...); B = sign * (phi - zero_angle) - level_offset
  7. return B, phi, raw mean, stddev, sample count
```

Two details that matter:

* **The settle window is separate from the averaging window.** The head
  hangs on belts through a differential; it rings after a move. Sampling
  starts only after `settle_time`. `AccelQueryHelper` trims samples to
  the request window on its own, but not to a settle offset, so the
  filtering in step 4 is explicit.
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
| < 60 deg | zero only | `zero_angle` only; offsets left alone |

The corertheta B range (-45 to 100, ie 145 degrees) supports the full
fit, which is a happy accident worth stating: this machine can
self-calibrate its own inclinometer.

**Bench alternative.** The cleanest calibration is a full 360-degree
rotation, which the head cannot do but a hand can. Recording the sensor
while slowly turning it through a complete turn *before* mounting gives
an unconditionally well-posed fit. If that is done, the on-machine
routine only has to find `zero_angle`, and `B_SET_ZERO` is the whole
procedure.

`plane_axes` and `invert` are also determined here rather than
configured: the routine sweeps, and reports which sensor axis pair varied
(the in-plane pair), which stayed put (`v`), and the sign of the change
against commanded B. Getting these wrong by hand is the most likely setup
mistake, so the routine should just work them out and print them.

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
| linear trend not removed by the fit | wrong `zero_angle`, not a ratio problem |
| one cycle per pulley revolution | pulley eccentricity or a bent shaft |
| step at the direction reversal | backlash - the `RETURN=1` number |
| growing toward one end | belt tension or a binding mount |
| random and large | the head is not settling; raise `settle_time` |

## Interaction with the rest of the fork

* **`[rtcp]` and `[b_projection]` must both be off** for every routine
  here, for exactly the reasons `homing.py` already documents: with RTCP
  on, a B move is also an X/Z move, and with the projection on, a
  commanded B is scaled by whatever bed angle the arm is over. The module
  reuses `check_disabled()` on both objects and produces the same class
  of error message.
* **The measured B is the machine B** - the angle the head is really
  turned to - which is exactly the quantity `[rtcp]` consumes and the
  quantity `[bltouch] b_offset` is expressed in. No frame conversion is
  needed anywhere.
* **`invert_b_direction` stays the one place rotation sense is set.** If
  the sensor disagrees with the machine's sign convention, that is what
  `invert` in this section records - and the calibration routine reports
  it rather than asking. The kinematics' own inversion is untouched.
* **This closes an open verification item.** Whether a positive B really
  tilts the nozzle outboard is still unverified on the machine.
  `B_MEASURE` answers it directly, without RTCP in the loop: command a
  positive B with the transforms off and read which way the head actually
  went.

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
| High stddev | the head was still moving; raise `settle_time` |
| Magnitude far from 1 g | the head is moving, or the chip is misconfigured |
| `v` varies across the sweep | `plane_axes` is wrong - here is the pair that did vary |
| In-plane radius well below 1 g | the B axis is not in the fitted plane - remount or re-run the sweep |
| Refine loop will not converge | `b_coupling_ratio` is wrong - run `B_STEP_CALIBRATE` |
| Endstop and sensor disagree | the stall home did not move the axis (the known `G4 P2000` failure) |
| Measured B drifts between prints | thermal offset drift; re-run `B_SET_ZERO` after heat soak |

## Testing

The dev box cannot run klippy, so the test split follows the code split:

* **Host, pytest, `test/multi_axis/test_b_inclinometer.py`** - everything
  in layer 1. Synthesise `(u, w)` points from a known angle sequence,
  inject known offsets, gain mismatch and gaussian noise, and assert the
  ellipse fit recovers them; assert the angle unwrap survives a sweep
  through +/-180; assert `fit_deg_per_step` recovers a known slope and
  that the ratio conversions are self-inverse. This is where the real
  test coverage is, and none of it needs a printer.
* **`test/klippy/`** - a `multi_axis_b_incl.cfg` plus `.test` for config
  parsing, command registration, and the refusals (transforms on, axis
  out of range, Z too low). Sensor data cannot be simulated in that
  harness, so the measurement path itself is not covered there.
* **On the machine** - the acceptance runs, in order:
  1. `ACCELEROMETER_QUERY` returns a plausible 1 g vector.
  2. `B_SENSOR_CALIBRATE` over the full range; the reported in-plane
     radius is within a few percent of 1 g and `v` is flat.
  3. `B_MEASURE` repeated ten times without moving - spread under 0.05
     degrees.
  4. `B_MEASURE` after commanding several angles - measured tracks
     commanded to within the ratio error.
  5. `B_STEP_CALIBRATE`; compare the fitted ratio against the nominal
     1.0, and check the residual shape against the table above.
  6. `home_method: endstop_then_accel` for a while, watching for the
     disagreement error - it should fire on exactly the back-to-back
     homes that fail today.

## Implementation order

Each phase is independently useful and independently shippable.

1. **Read-only.** `[b_inclinometer]`, the measurement primitive,
   `B_MEASURE`. Nothing moves; the head can be turned by hand. This alone
   answers the outstanding rotation-sense question.
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
* **Naming.** `[b_inclinometer]` describes the sensor honestly, but the
  section also owns the step calibration, which is not an inclinometer
  concern. `[b_accel]` and `[b_calibrate]` are the alternatives.
