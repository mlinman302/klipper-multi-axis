# Accelerometer Z homing: tapping the bed with the nozzle

> **The sensor is now a BMI160 IMU**, not the ADXL345 this document
> was written against. Three things change: Path A below is dead (see
> [BMI160_IMU.md](BMI160_IMU.md), "The Z homing path"), the gyroscope
> adds a second, gravity-free detection channel worth capturing in
> phase 0, and the sample rate is 1600 Hz rather than 3200 Hz. The
> `trigger_analog` plumbing of phase 1 is implemented in
> `src/sensor_bmi160.c`. Path B, and everything else below, stands.
>
> **The latency budget below assumed a USB board's RP2040.** On the
> corertheta machine the BMI160 is on the Raspberry Pi Zero W's SPI0,
> so the detector runs in klipper_mcu and the trigger is relayed to the
> LPC1769. Detection delay - dominated by the poll interval, which
> shrinks with `rate` up to 1600 Hz, as far as the Pi can keep up -
> biases the recorded Z; the relay only adds overshoot. See [BMI160_IMU.md](BMI160_IMU.md), "On this machine: the
> detector runs in a Linux process".

**Status: design outline only.** Nothing in this document is
implemented. It is the same kind of document
[Accel_B_Homing.md](Accel_B_Homing.md) started as, and it assumes that
module's hardware and mounting work as read.

The idea is short: drive Z down until the nozzle touches the bed, detect
the contact as a mechanical shock in the ADXL345 already bolted to the
tilting head, and stop the move from the accelerometer board itself.
That makes the nozzle the Z reference, and it makes Z homing possible
without deploying a probe or turning the head to a probe angle.

The long part is *how the machine decides that a shock was contact*, and
that is what most of this document is about. There are two candidate
detectors - the ADXL345's own tap engine, and one of our own built on
Klipper's existing `trigger_analog` helper - and they are not close in
quality. The recommendation below is our own detector; the chip's tap
engine is kept as a cheap experiment because it costs almost nothing to
try.

## Why bother on this machine

* **Z homing currently needs a whole dance.** Z homes on the BLTouch
  (`endstop_pin: probe:z_virtual_endstop`,
  [config/example-corertheta.cfg:80](../config/example-corertheta.cfg)),
  so the `HOME_Z` macro has to turn both transforms off, orient the head
  to the probe angle with `RTCP_PROBE_ORIENT`, and move the arm to a
  radius where the probe is actually over the bed - all before Z is
  homed and therefore all at a carriage height nobody has measured yet.
  A nozzle tap at B = 0 needs none of it: the nozzle is already the
  lowest point of the head, and it is over the bed wherever the arm is.

* **The Z datum becomes the nozzle.** Today the homed Z follows from the
  BLTouch's `z_offset` alone - the macro's own comment says so. Every
  probe offset in that chain (`z_offset`, `b_offset`, the arm-frame x/y
  offsets of `[rtcp_probe]`) is a number the first layer depends on. Tap
  homing takes them all out of the *home*: contact is nozzle-to-bed, so
  the homed position is the physical thing the first layer cares about.

* **It gives the probe geometry an independent check.** Tap Z at a
  point, then probe the same point with the BLTouch, and the difference
  *is* `z_offset` - measured, not trusted. On a machine where the probe
  hangs off a rotating head and its offsets were established by hand,
  that is a calibration there is currently no second opinion on.

* **The hardware is already there.** The Fly-ADXL345-USB is on the head
  for `[accel_b_homing]`, and the mounting requirements are the same
  ones that module already imposes.

It also has a real cost, paid on every home: the nozzle is driven into
the bed. See "What tapping costs" below - that section is not optional
reading.

## The one hard constraint: the trigger must fire on the MCU

This constraint decides the whole design, so it comes first.

`[accel_b_homing]` reads the accelerometer on the host: it starts a
bulk-sensor client, dwells, and averages what arrives. That is fine for
measuring a *stationary* head. It is useless for stopping a *moving*
one. The delivery path is:

* the chip's FIFO, drained by the RP2040 on a timer
  (`rest_ticks = 4/data_rate`,
  [klippy/extras/adxl345.py:292](../klippy/extras/adxl345.py)),
* batched into bulk sensor messages of `BATCH_UPDATES = 0.100` seconds
  ([klippy/extras/adxl345.py:199](../klippy/extras/adxl345.py)),
* over a USB serial link to the host, which `[accel_b_homing]` already
  has to allow 0.3 s of slack for (`batch_margin` - see
  [Accel_B_Homing.md](Accel_B_Homing.md)),
* into a Python callback scheduled by the reactor.

That is 100 ms at absolute best and a few hundred milliseconds in
practice. At a 5 mm/s probing speed, 100 ms is 500 um of continued
descent after contact, with the nozzle already hard against the bed.
Host-side detection is not slow, it is off by two orders of magnitude.

So: **the RP2040 on the accelerometer board must be the thing that
decides, and it must call `trsync_do_trigger()`.** Klipper's homing
machinery is fine with this - an endstop living on a secondary MCU is
ordinary (`[probe]` on a toolhead board is the same shape), and
`trsync` exists precisely so a trigger on one MCU can stop steppers on
another. Nothing about the multi-MCU part is new work.

What is new is teaching that MCU what contact looks like.

## Path A - the ADXL345's own tap engine

The chip can detect taps by itself and assert an interrupt pin. From
the datasheet (Rev. E, register definitions and the Tap Detection
section):

| Register | Address | Meaning | Scale |
| --- | --- | --- | --- |
| `THRESH_TAP` | 0x1D | tap threshold, unsigned | 62.5 mg/LSB (0xFF = 16 g) |
| `DUR` | 0x21 | max time above threshold to still count as a tap | 625 us/LSB; 0 disables tap |
| `Latent` | 0x22 | wait after tap 1 before the double-tap window | 1.25 ms/LSB |
| `Window` | 0x23 | length of the double-tap window | 1.25 ms/LSB |
| `TAP_AXES` | 0x2A | D3 suppress, D2/D1/D0 enable X/Y/Z | - |
| `ACT_TAP_STATUS` | 0x2B | which axis crossed first | overwritten, never cleared |
| `INT_ENABLE` | 0x2E | D6 single tap, D5 double tap | - |
| `INT_MAP` | 0x2F | same bit layout; 0 to INT1, 1 to INT2 | - |
| `INT_SOURCE` | 0x30 | which function fired | cleared by reading it |
| `DATA_FORMAT` | 0x31 | D5 `INT_INVERT` makes interrupts active low | - |

Single tap fires when the acceleration goes back *below* the threshold
having been above it for no longer than `DUR`. Only single tap is of
interest here - `Latent` and `Window` exist for the double-tap gesture
and would only be set to zero.

Everything needed to try this exists already: `[adxl345]` registers
`ACCELEROMETER_DEBUG_READ REG=...` and
`ACCELEROMETER_DEBUG_WRITE REG=... VAL=...`
([klippy/extras/adxl345.py:179](../klippy/extras/adxl345.py)), so the
whole configuration can be poked in from the console with no code at
all.

### Four problems with it

1. **The interrupt pin may not go anywhere.** The Fly-ADXL345-USB
   documentation covers exactly four pins - `gpio9` CS, `gpio10` SCLK,
   `gpio11` MOSI, `gpio12` MISO, all software SPI - and says nothing
   about INT1 or INT2. If those pads are not routed to an RP2040 GPIO,
   Path A is dead on this board short of soldering to a QFN package.
   This is answerable in ten minutes, see the experiment below.

2. **Gravity is in the way, and gravity moves with B.** Tap detection is
   dc-coupled: it compares the acceleration on the enabled axes against
   `THRESH_TAP` with no reference subtraction (only the *activity*
   function has ac/dc coupling bits, and only *inactivity* uses filtered
   data). With the head at B = 0 the head-down sensor axis sits at a
   steady 1 g - above any useful threshold, permanently, so it never
   falls back below and never fires a single tap. Two ways out, both
   bad:

   * Set `THRESH_TAP` above 1 g - at least 16 LSB, realistically 24 LSB
     (1.5 g) for margin. That demands a tap hard enough to add half a g
     to a head that weighs what a toolhead weighs. On the bed.
   * Enable only the axes that read near zero g in the probing pose -
     but those are the axes across the tap direction, which see the
     contact only through cross-axis coupling and head ringing.

   And because the head tilts, which axis carries the gravity changes
   with B, so any threshold chosen this way is a function of B. The
   module would have to reprogram the chip for each probing angle.

3. **62.5 mg per count is coarse.** The whole interesting range of a
   gentle nozzle tap is a few hundred mg. Path A resolves that range in
   perhaps three or four steps - and, per problem 2, cannot use the
   bottom sixteen of them anyway.

4. **You cannot check its work.** Tap detection runs on *undecimated*
   data - the datasheet's Threshold section is explicit that the high-g,
   high-frequency content driving tap detection may not be present if
   the accelerometer's output is examined. So the chip can fire on
   something that does not appear in the stream Klipper records, and a
   tap test that captures samples around the trigger cannot confirm or
   refute what caused it. For a detector whose failure mode is crashing
   the nozzle into the bed, un-auditable is a serious mark against it.

Point 4 also cuts the other way and should be said honestly: the
undecimated path *sees* the sharp leading edge of a real impact, which
the decimated stream at any configured ODR partly filters away. If the
contact signature turns out to live above 1600 Hz, Path A is looking at
the only place it exists.

### The wiring experiment (do this early, it is cheap)

No code, no compilation:

1. `ACCELEROMETER_DEBUG_WRITE REG=0x1D VAL=0x30` (3 g threshold, the
   datasheet's own suggested starting point), `REG=0x21 VAL=0x20`
   (20 ms), `REG=0x22 VAL=0x00`, `REG=0x23 VAL=0x00` (no double tap),
   `REG=0x2A VAL=0x07` (all three axes), `REG=0x2F VAL=0x00`
   (everything to INT1), `REG=0x2E VAL=0x40` (enable single tap).
2. Rap the head with a knuckle.
3. `ACCELEROMETER_DEBUG_READ REG=0x30` - bit 6 set means the chip
   detected it. That alone confirms the tap engine works and gives a
   feel for what threshold a real tap needs, independent of any wiring.
4. For the wiring: add `[gcode_button]` sections on candidate free
   RP2040 pins on the `adxl` MCU, tap, and `QUERY_BUTTON` each one. The
   single-tap interrupt latches until `INT_SOURCE` is read, so the
   sequence "tap, QUERY_BUTTON says pressed, read REG=0x30,
   QUERY_BUTTON says released" identifies the pin unambiguously. If no
   pin behaves that way, INT1 is not routed and Path A ends here.

Do step 3 regardless of how step 4 turns out - it is a free measurement
of how big a hand tap is on this head, which calibrates intuition for
Path B.

**Verdict: not the primary design.** Try the experiment because it is
half an hour and it answers a hardware question worth having answered;
keep it as a fallback if the streamed signal turns out to be too soft to
work with.

## Path B - our own detector, on the `trigger_analog` helper

This is the recommendation. The load-cell probe work landed a
general-purpose MCU-side "a sensor stream crossed a threshold, fire
trsync" helper, and a nozzle tap is exactly the shape of problem it
solves.

### What already exists

* [src/trigger_analog.c](../src/trigger_analog.c) - takes raw samples
  through `trigger_analog_update()`, runs them through a second-order
  section filter, checks a trigger condition, and calls
  `trsync_do_trigger()`. It also range-checks the raw value, watches for
  the sensor going quiet (`monitor`), and reports both as distinct
  trsync error codes, so a stalled or glitching sensor aborts the home
  instead of silently never triggering.
* Three trigger types, already implemented: `abs_ge` (magnitude at or
  above a value), `gt` (signed, above a value), and `diff_peak_gt`
  (dropped by more than a value from the running peak).
* An await-homing flag with a `homing_clock`, so the detector is armed
  at a print time of the host's choosing rather than at the start of the
  move - which is how the acceleration transient at the start of the
  probing move is kept out of the detector.
* [klippy/extras/trigger_analog.py](../klippy/extras/trigger_analog.py) -
  `MCU_trigger_analog` (line 271) is already an endstop in Klipper's
  sense: `home_start()`, `home_wait()`, `get_steppers()`, a trsync
  dispatch, and error-code translation.
* Filter design in Python via scipy, with pre-generated coefficients as
  a fallback (`DigitalFilter`, line 73): high pass, low pass, notch and
  derivative sections are all available.
* The sensor side of the contract is small and is already implemented
  three times over (hx71x, ads1220, ads131m0x): a
  `setup_trigger_analog(oid)` that sends an attach command,
  `get_samples_per_second()`, and `lookup_sensor_error()` - see
  [klippy/extras/ads1220.py:105](../klippy/extras/ads1220.py) and, on
  the firmware side, the single `trigger_analog_update()` call in
  [src/sensor_ads1220.c:100](../src/sensor_ads1220.c) plus its
  `attach_trigger_analog` command.

### What has to be written

Much less than it looks like.

* **`src/sensor_adxl345.c`**: a `struct trigger_analog *ta` field, an
  `adxl345_attach_trigger_analog oid=%c trigger_analog_oid=%c` command,
  and one `trigger_analog_update(ax->ta, value)` call per decoded sample
  in `adxl_query()`. The value is one axis' 13-bit signed reading, or a
  fixed combination of axes chosen at config time - see "Which axis"
  below. Perhaps twenty lines.
* **`klippy/extras/adxl345.py`**: `setup_trigger_analog()`,
  `get_samples_per_second()` (it is `self.data_rate`) and
  `lookup_sensor_error()`. Perhaps fifteen lines, and shaped exactly
  like the three existing implementations.
* **`klippy/extras/accel_z_tap.py`**: the new module. Config, filter
  design, threshold, the endstop wrapper, the probe session and the
  commands. This is the only genuinely new file.

### Why this fixes what Path A got wrong

* **Gravity disappears.** A high-pass section removes the dc term
  entirely, so the detector sees only the transient, and the threshold
  stops being a function of B. The head can tap at any tilt with one
  number.
* **The threshold can be small.** In full resolution mode one count is
  3.9 mg. The datasheet's noise figures are 0.75 LSB rms (X/Y) and
  1.1 LSB rms (Z) at 100 Hz ODR, rising by roughly sqrt(2) per doubling
  of the data rate - so at 3200 Hz, about 4.2 and 6.2 LSB rms, i.e.
  ~17 and ~24 mg rms per sample, and less than that inside a narrow
  band once filtered. A five-sigma trigger sits near 0.12 g. Path A's
  floor was above 1 g.
* **The band is choosable.** Contact is a step in force; ringing at the
  head's own resonances follows. A band pass that keeps the impact and
  rejects both dc and the frequencies the motion system produces by
  itself is the entire trick, and `DigitalFilter` already builds one.
* **It is auditable.** The detector runs on the same samples that get
  streamed to the host, so a captured CSV around a trigger explains the
  trigger. Compare Path A's problem 4.

### Trigger type

`abs_ge` on the band-passed signal is the obvious first choice: contact
is a symmetric transient and its sign depends on mounting.
`diff_peak_gt` is worth keeping in mind as the alternative if the
machine turns out to produce a slow rise the filter cannot separate from
contact, since it triggers on the *fall* from a peak rather than on the
crossing.

### Latency budget

Real, quantifiable, and not zero:

* ODR 3200 Hz gives 0.31 ms per sample.
* The firmware polls the FIFO every four sample periods (1.25 ms) and
  drains it, so a sample waits up to ~1.25 ms before it is looked at.
  Dropping `rest_ticks` to one sample period while a tap is armed is an
  obvious knob and probably worth having.
* Software SPI on the Fly board for a nine-byte transfer: order 0.1 ms,
  worth measuring rather than assuming.
* Filter group delay: a few samples, by design.

Call it ~2 ms worst case, so ~10 um at 5 mm/s and ~40 um at 20 mm/s.
That is a *systematic* offset at constant speed, not a random error, so
it calibrates out into the probe offset - on the strict condition that
the probing speed used in calibration is the probing speed used
afterwards. This is the same rule every probe on every printer already
lives under; it just needs saying because the magnitude is bigger than a
microswitch's.

The trigger *time* recorded by trsync is when the MCU decided, so the
recorded trigger position is not corrupted by whatever the head does
afterwards. The head still keeps moving into the bed for those two
milliseconds plus the deceleration ramp, which is a mechanical question,
not a measurement one, and it belongs in "What tapping costs".

### Which axis

Open question, to be answered by phase 0 rather than argued. The
candidates:

* The head's own "down" axis - the one `[accel_b_homing]` calls
  `zero_vector`. Aligned with the impact, but also the axis carrying
  gravity, which only matters if the high pass is imperfect.
* The sum of squares of all three - orientation independent, but the
  squaring is arithmetic in an interrupt-adjacent path and the noise of
  three axes adds.
* A fixed single axis chosen by measurement.

Reusing `[accel_b_homing]`'s declared `zero_vector` / `positive_vector`
would be elegant, and would mean a tap at a nonzero B could pick the
axis pointing along the machine's Z. Whether that is needed depends on
whether tapping is ever wanted anywhere but B = 0.

## Phase 0: does the signal even exist?

**Do this before writing any code.** Everything above assumes a nozzle
touching a bed at a safe speed produces a shock that stands out of the
noise on a sensor bolted to the far side of the head from the nozzle.
That is an assumption, not a measurement, and it is cheap to test:

1. `ACCELEROMETER_MEASURE` (start), a slow `G1 Z...` descent onto the
   bed from a known height, `ACCELEROMETER_MEASURE` (stop).
2. Repeat at 1, 2, 5, 10 and 20 mm/s; on the bare bed and on a sheet of
   PEI; near the middle of the bed and out at the arm's reach, where the
   structure is softest.
3. Also capture the machine doing *nothing but move* at those speeds, in
   the air. That is the false-trigger background and it sets the floor
   for any threshold.

Then a small analysis script (`scripts/`, alongside the existing
helpers) that reads the CSVs, high passes them, and reports peak
amplitude at contact against peak amplitude in the moving-but-not-
touching case. The deliverable is a number - the ratio between them -
and the filter band that maximises it.

If that ratio is comfortably above 10, Path B is straightforward
engineering. If it is near 2, the design needs the tap to be harder or
the head to be stiffer, and that is much better to know before writing a
firmware patch. If contact is invisible in the streamed data at every
speed, then Path A's undecimated data is the only remaining option and
the wiring experiment stops being optional.

## Module sketch

```ini
[accel_z_tap]
accel_chip: adxl345          # as [accel_b_homing]; may be shared
#axis: zero_vector           # or an explicit sensor axis
#highpass: 50                # Hz, removes gravity and slow structure
#lowpass: 800                # Hz, keeps the impact, rejects sensor noise
#trigger_threshold: 0.15     # g, on the filtered signal
#probe_speed: 5.0            # mm/s; part of the calibration, see above
#retract_dist: 2.0
#samples: 3
#samples_tolerance: 0.010
#samples_tolerance_retries: 2
#arm_delay: 0.050            # s after the move starts before arming
#min_trigger_travel: 1.0     # mm; a trigger sooner than this is an error
```

The module would:

* build the filter with `DigitalFilter`, hand it to an `MCU_SosFilter`,
  and wrap an `MCU_trigger_analog` - all existing classes;
* register a pin chip of its own so `[stepper_z]` can say
  `endstop_pin: accel_tap:z_virtual_endstop`. Note this is a *second*
  virtual endstop on a machine that already has
  `probe:z_virtual_endstop` from the BLTouch; `[probe]` claims the chip
  name `probe` ([klippy/extras/probe.py:237](../klippy/extras/probe.py))
  and the `probe` object, so the tap module must claim neither and must
  not `add_object('probe', ...)` the way `[load_cell_probe]` and
  `[probe_eddy_current]` do. The two coexist; the config chooses which
  one homes Z, and the other stays available for meshing;
* offer a probe-session interface so `PROBE`, `PROBE_ACCURACY` and mesh
  calibration can use it too, once homing is trusted;
* register `ACCEL_TAP_QUERY`, `ACCEL_TAP_TEST` (n taps, report the
  scatter and the captured waveform), and `ACCEL_TAP_CALIBRATE` (sweep
  the threshold, find the range where every tap fires and none fires
  early, and report the middle of it).

### Interaction with RTCP and B

Tap homing does not need the head turned to a probe angle, but it does
need to know which way the nozzle points:

* B must be homed and at B = 0 (nozzle down) before a tap, or the nozzle
  is not the lowest point of the head and something else touches first.
  `[accel_b_homing]` is the natural way to *verify* that rather than
  assume it - measure B, confirm it is within a degree of zero, refuse
  otherwise. The two modules earn their keep together.
* Probing runs with RTCP off, as it does today. With RTCP on, a Z move
  is not a pure carriage move and the trigger position would need the
  transform applied; keep the existing "probing is refused with RTCP on"
  rule.
* If tapping at nonzero B is ever wanted (for measuring the head's
  rotation geometry, which is the interesting case), the contact
  direction is still machine Z but the sensor frame has rotated - hence
  the "which axis" question above.

## What tapping costs

Stated plainly, because it is the reason this is a design document and
not a patch:

* **Every home marks the bed.** A nozzle stopped by a trsync still
  travels for the detection latency plus the deceleration ramp, and the
  force during that time is whatever the Z drive can produce. On a
  smooth PEI sheet a repeated tap in one spot will eventually show.
  Mitigations: tap at different spots, tap slowly, and use the smallest
  usable `retract_dist` between samples.
* **A hot, oozing nozzle taps early and softly.** The measurement is
  then of the plastic, not the bed. Any real implementation needs a
  temperature policy - tap cold, or tap after a wipe - and probably a
  refusal above some extruder temperature.
* **A false trigger in mid-air homes Z at the wrong height**, and the
  next move drives the nozzle into the bed with no detection at all.
  This is the failure that matters. Defences, in order: arm the detector
  only after the move is at constant speed; require the trigger to occur
  within a plausible Z window (`min_trigger_travel` above); take
  multiple samples and require them to agree (`samples_tolerance`, which
  `[probe]` already implements); and treat a trigger in the first
  millimetre of travel as an error rather than a result.
* **A missed trigger is a crash.** `position_min` on `[stepper_z]`
  bounds it in the ordinary way, as it does for BLTouch probing today,
  and `trigger_analog`'s sensor monitor catches the "accelerometer went
  quiet" case. Neither helps if the threshold is simply set too high,
  which is what `ACCEL_TAP_CALIBRATE` is for.

## Phases

1. **Phase 0 - measure.** No code beyond an analysis script. Capture
   contact signatures, decide whether the signal exists and in what
   band. Also run the Path A wiring experiment. *Deliverable: numbers,
   and a go/no-go on Path B.*
2. **Phase 1 - plumbing.** `trigger_analog` support in
   `src/sensor_adxl345.c` and `klippy/extras/adxl345.py`. No homing yet;
   the test is a trigger fired by a hand tap, with the print time it
   fired at reported back.
3. **Phase 2 - a probe.** `accel_z_tap.py` as an endstop and a probe
   session. `ACCEL_TAP_CALIBRATE`. Homing Z on it, with the BLTouch
   still configured and available.
4. **Phase 3 - cross-check.** Tap and BLTouch the same points; report
   the difference as a measured `z_offset`; feed that back into
   `[bltouch]` and `[rtcp_probe]`. This is the phase that pays for the
   whole exercise on this machine.
5. **Phase 4 - tapping at angle**, if phase 3 shows it is needed for the
   head geometry.

## Testing

Following `test/multi_axis/test_accel_b_homing.py`: a host test that
drives the real module against a stubbed printer and a *synthetic*
sample stream, so it runs anywhere Python does - no MCU, no
`c_helper.so`, no serial port. (Relevant here: klippy does not run
natively on the Windows development host, so anything needing a live
printer is a machine test, not a CI test.)

What the host test can cover honestly:

* filter design - a Python model of the same sos filter, fed a synthetic
  step-plus-ringing, and checked against the threshold arithmetic;
* the arming logic, and the "trigger too early" and "trigger outside the
  plausible window" refusals;
* sample agreement and retry logic;
* config validation and the error messages.

What it cannot cover, and which needs the machine: latency, the actual
contact signature, and every threshold number in the config. Phase 0's
captured CSVs are the right fixtures for the parts in between - a replay
test against real recorded taps is worth more than any synthetic one.

## Open questions

* Are INT1/INT2 routed to RP2040 GPIOs on the Fly-ADXL345-USB? (Path A
  lives or dies on this; the experiment above settles it.)
* Is the contact transient visible in the decimated 3200 Hz stream at a
  probing speed that does not mark the bed? (Phase 0.)
* Which axis, or combination, gives the best contact-to-background
  ratio? (Phase 0.)
* How repeatable is a tap, really - and how much of the scatter is the
  detector rather than the machine's own Z repeatability? A useful
  control: tap ten times without moving in X or Y, then tap ten times
  returning to the same point from different directions.
* Does the trigger latency vary with `rest_ticks` the way the budget
  above predicts? Measurable by tapping at several speeds and looking at
  the slope of trigger height against speed - the intercept is the
  geometry, the slope is the latency.
* Should the tap module refuse to run unless `[accel_b_homing]` confirms
  B is at zero, or merely warn?
