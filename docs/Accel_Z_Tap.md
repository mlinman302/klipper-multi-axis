# Z homing by nozzle tap

`[accel_z_tap]` (`klippy/extras/accel_z_tap.py`) homes Z by driving the
nozzle down until it touches the bed, seeing the contact as a shock in
the BMI160 on the tilting head, and stopping the steppers from the MCU
that reads the BMI160. The nozzle becomes the Z reference: no probe is
deployed and the head is not turned to a probe angle.

The BMI160 is the only supported sensor. Wiring and bring-up are in
[BMI160_IMU.md](BMI160_IMU.md); the option reference is
[Config_Reference.md](Config_Reference.md#accel_z_tap).

**Status.** Implemented and host-tested
(`test/multi_axis/test_accel_z_tap.py`); it has never touched a bed.
Every threshold and filter number is a starting point, and whether a
contact stands out of the background on this head at all is the first
thing to measure - see [Commissioning](#commissioning). The example
config still homes Z on the BLTouch.

## Why tap

* **Z homing currently needs a whole dance.** Z homes on the BLTouch, so
  the `HOME_Z` macro turns both transforms off, orients the head to the
  probe angle and moves the arm so the probe is over the bed - all before
  Z is homed, at a carriage height nobody has measured. A tap at B = 0
  needs none of it: the nozzle is already the lowest point of the head,
  over the bed wherever the arm is.
* **The Z datum becomes the nozzle.** The homed Z currently follows from
  the BLTouch's `z_offset`, and its `b_offset` and arm-frame offsets are
  all numbers the first layer depends on. A tap is nozzle-to-bed.
* **It checks the probe geometry.** Tap a point, probe the same point with
  the BLTouch, and the difference *is* `z_offset` - measured rather than
  trusted.
* **The hardware is already there** for `[accel_b_homing]`.

It has a real cost, paid on every home: the nozzle is driven into the
bed. See [What tapping costs](#what-tapping-costs).

## How it works

### The detector runs on the MCU

Detecting on the host is hopeless for stopping a moving nozzle. Samples
reach klippy in 0.1 s bulk batches; at 5 mm/s, 100 ms is half a
millimetre of continued descent into the bed. So the decision is made
where the samples are read: `src/sensor_bmi160.c` feeds one channel of
every FIFO frame to Klipper's `trigger_analog`, which filters it, tests
it against a threshold and calls `trsync_do_trigger()`. `trsync` exists
so a trigger on one MCU can stop steppers on another, exactly as for a
probe on a toolhead board. `[accel_z_tap]` configures that detector,
decides when it arms, and checks the result.

On this machine the BMI160 is on the Pi's SPI0, so the detector runs in
klipper_mcu and the trigger is relayed to the LPC1769 - see
[Latency](#latency).

### What the detector sees

```
tap_channel (raw counts)
  -> subtract the first sample after arming    removes gravity
  -> x 1024 (Q10)
  -> high pass (default 50 Hz, 2nd order)      removes drift and slow structure
  -> low pass  (default off)                   removes sensor noise
  -> |value| >= trigger_threshold  ->  trsync_do_trigger()
```

* **Gravity is removed by offset.** A Z move does not tilt the head, so
  subtracting the first sample the detector processes after arming (the
  sos filter's `auto_offset`) starts the filter from rest at any B.
  Without it a high pass fed a 1 g step at arming would ring straight
  through the threshold. The high pass remains for drift and slow
  structure, and can be turned off.
* **Magnitude, not sign** (`abs_ge`): the sign of a contact depends on how
  the chip is mounted.
* **The filter runs on raw counts in Q10.** A full-scale swing is 2^26,
  leaving 32x of int32 headroom for overshoot, while the rounding in the
  recursion - which a low high-pass cutoff amplifies - stays far below a
  count. An earlier version in Q16 of g drifted 12 mg from the float
  model on a 5 Hz fourth-order high pass. `trigger_threshold` is
  configured in the channel's unit (g, or deg/s) and converted.
* **Butterworth sections are computed without SciPy** - a dozen lines of
  bilinear transform, so a Pi Zero W needs nothing extra. The tests check
  them against `scipy.signal.butter` where SciPy is installed.

### Which channel

`tap_channel` in `[bmi160]` picks one of `accel_x/y/z` or `gyro_x/y/z`,
fixed at config time because it is a byte offset into the FIFO frame.
The default is `accel_z`. An accelerometer axis aligned with the impact
sees it directly; a gyroscope axis sees the angular impulse of a strike
landing off the head's rotation axis, against a background with no
gravity term at all. Which gives the better contact-to-background ratio
on this head is a measurement - `ACCEL_TAP_CALIBRATE` reports the ratio
for whichever channel is configured. A gyroscope channel has no default
threshold.

### It is an endstop, not a probe

The section registers a pin chip, `accel_tap`, so `[stepper_z]` can say
`endstop_pin: accel_tap:z_virtual_endstop`. It is deliberately not the
`probe` chip and creates no `probe` object: the `[bltouch]` stays
configured for meshing, and whichever pin `[stepper_z]` names homes Z.
The endstop has no `get_position_endstop()`, so `G28 Z` is an ordinary
rail home using `[stepper_z]`'s `position_endstop`. `ACCEL_TAP_PROBE`
taps the Z steppers whichever pin `[stepper_z]` uses.

## Arming and the blind distance

The start of a move is a jolt, so the detector must not be armed through
it. The trsync starts with the move, but the detector arms at

    move start + HOMING_START_DELAY + speed / accel + arm_delay

with `accel = min(max_accel, max_z_accel)` - once the move is at constant
speed. The distance covered before that is **blind**:

    blind = speed^2 / (2 accel) + speed * (HOMING_START_DELAY + arm_delay)

On this machine (`max_z_accel: 30`) that is 0.67 mm at 5 mm/s. A nozzle
that starts closer to the bed than the blind distance is driven into it
undetected. `ACCEL_TAP_QUERY` reports it, and every retract distance is
checked against it.

## The guards

In order of importance - a false trigger in mid-air homes Z at the wrong
height, and the next move drives the nozzle into the bed with no
detection at all:

* **Arming after acceleration**, above.
* **`min_trigger_travel`.** A trigger within 1 mm of the start of the move
  is an error, not a result.
* **Two taps must agree.** With a `homing_retract_dist`, `G28 Z` taps
  twice, and the two trigger positions must agree within
  `samples_tolerance` or Z is not homed. The retract must exceed both the
  blind distance at `second_homing_speed` and `min_trigger_travel`.
  `ACCEL_TAP_PROBE` samples and retries the way `[probe]` does.
* **Lost frames refuse the result.** A FIFO overflow during a tap means
  the detector may have missed the contact.
* **B must be homed and within `b_tolerance` of 0**, since only then is
  the nozzle the lowest point of the head. This is checked on the
  commanded angle; `measure_b: True` also measures the head's fused
  angle with `[accel_b_homing]` before `G28 Z`, `ACCEL_TAP_PROBE` and
  `ACCEL_TAP_CALIBRATE`, at the cost of about a second.
* **A hot nozzle is refused** above `max_extruder_temp` (150 C, current or
  target). It oozes, and taps early and softly on its own plastic.
* **Speed is capped** at `max_speed` for `homing_speed`,
  `second_homing_speed` and the probing speed.
* **The sensor monitor** in `trigger_analog` aborts the move if the chip
  delivers no samples for about `sensor_timeout`. Its tick is the larger
  of one FIFO poll and `sensor_timeout / 5`, since the BMI160 delivers
  four frames at a time and klipper_mcu adds Linux jitter.
* **A missed trigger is a crash**, bounded only by `[stepper_z]`'s
  `position_min` (for `G28 Z`) or `max_overtravel` below Z = 0 (for
  `ACCEL_TAP_PROBE`). Nothing helps if the threshold is simply too high -
  which is what `ACCEL_TAP_CALIBRATE` is for.

## Latency

Everything between the nozzle touching the bed and the steppers stopping
splits into two parts that do very different damage.

**Detection delay biases the recorded Z.** `trigger_analog_update()`
stamps a trigger with the MCU clock at the moment it *processes* the
sample, and homing reads the stepper position back at that time. So the
frame's age in the FIFO (up to one poll interval of four frames), the SPI
read and the chip's own filter delay all shift the recorded height. The
constant part calibrates out into `position_endstop`, as for any probe at
a fixed speed - **so calibrate at the speeds you home at**. The variable
part, the frame's age, is roughly uniform over the poll interval and goes
straight into repeatability:

| `rate` | poll interval | sample-age scatter (1 sigma) | at 2 mm/s | at 5 mm/s |
| --- | --- | --- | --- | --- |
| 400 Hz | 10 ms | 2.9 ms | 6 um | 14 um |
| 1600 Hz | 2.5 ms | 0.7 ms | 1.4 um | 3.6 um |

At 400 Hz the chip's own filter also narrows to about 160 Hz, which may
soften the contact enough to make detection later and less consistent.

**Relay delay only causes overshoot.** The trigger travels klipper_mcu ->
pty -> klippy's serial thread (forwarded in C by `trdispatch`, no Python)
-> USB -> LPC1769. That does not move the recorded position; it drives the
nozzle further into the bed. At a slow tap speed it is harmless as a
distance. Its real risk is Klipper's 25 ms trsync liveness timeout: a
loaded single core that delays klipper_mcu's status by that long aborts
the home with "Communication timeout during homing".

The levers, cheapest first:

* **Tap slowly.** A normal `homing_speed` for the first tap and a slow
  `second_homing_speed` for the one that counts scales both scatter and
  overshoot down.
* **Raise `rate` to 1600.** SPI makes this a config change (17 % of a
  1 MHz wire); the cost is four times the work for the Pi, on the core
  whose lateness trips the trsync timeout. `[accel_b_homing]` loses
  nothing by it but margin.
* **Back-date the trigger** to the sample time of the frame that fired,
  which the firmware could compute from the frames still queued behind
  it. That would remove most of the scatter above, but it is a change to
  the `trigger_analog` interface and is not implemented.
* **Move the chip to the LPC1769**, avoiding the relay, at the price of a
  long SPI run to the mainboard. Only if the Pi cannot keep up at 1600 Hz.

## Configuration

```ini
[bmi160]
cs_pin: rpi:None
spi_bus: spidev0.0
rate: 1600
#tap_channel: accel_z

[accel_z_tap]
#trigger_threshold: 0.15
#highpass: 50
#samples_tolerance: 0.05

[stepper_z]
endstop_pin: accel_tap:z_virtual_endstop
# The Z at which the nozzle touches the bed, measured at the homing
# speeds below - the detection delay biases it in proportion to speed
position_endstop: 0
homing_speed: 5
second_homing_speed: 2
homing_retract_dist: 3
```

`G28 Z` does not know how high the nozzle starts, so it must already be
more than the blind distance above the bed. The example config's
`HOME_Z_TAP` macro turns the transforms off, puts the head at B = 0 and
off the bed's centre, and runs `G28 Z`; it expects the nozzle to clear
the bed by a few millimetres and to be cold.

## Commands

| Command | Does |
| --- | --- |
| `ACCEL_TAP_QUERY [TIME=1]` | Capture the tap channel with the machine still, run it through the detector's filter on the host, and report the output's rms and peak, how far above that peak `trigger_threshold` sits, and the blind distance at the probing speed. |
| `ACCEL_TAP_TEST [TAPS=3] [TIMEOUT=30]` | Arm the detector with no movement and wait for taps on the head by hand, reporting each one's print time and peak. |
| `ACCEL_TAP_PROBE [PROBE_SPEED=] [LIFT_SPEED=] [SAMPLES=] [SAMPLE_RETRACT_DIST=] [SAMPLES_TOLERANCE=] [SAMPLES_TOLERANCE_RETRIES=] [SAMPLES_RESULT=]` | Tap the bed at the current X/Y and report the Z of contact, lifting after every tap. Needs Z homed, RTCP and the B projection off. |
| `ACCEL_TAP_CALIBRATE [DISTANCE=5] [TAPS=3] [PROBE_SPEED=]` | Descend `DISTANCE` through the air at the probing speed and filter the capture on the host (the background), then tap the bed `TAPS` times at the current threshold. Report the background peak, each contact's peak and their ratio, warn below 3, and set the geometric mean of the background and the weakest contact as `trigger_threshold` for `SAVE_CONFIG`. Needs Z homed and the nozzle at least `DISTANCE` + `sample_retract_dist` above the bed. |

`ACCEL_TAP_CALIBRATE` measures rather than sweeps: sweeping the threshold
would mean crashing into the bed at thresholds known to be too high. Its
contacts are taken at the current threshold, so start from one that
detects them.

## Commissioning

1. **Bring up the BMI160** and home B - [BMI160_IMU.md](BMI160_IMU.md),
   [Accel_B_Homing.md](Accel_B_Homing.md). Set `rate: 1600` if the Pi
   keeps up without FIFO overflows.
2. **`ACCEL_TAP_TEST`** - tap the head by hand. The detector fires and the
   plumbing from klipper_mcu to trsync works.
3. **`ACCEL_TAP_QUERY`** with the machine still, and again with the
   steppers energised - the background floor.
4. **With Z homed on the BLTouch and the nozzle cold, `ACCEL_TAP_CALIBRATE`**
   near the bed's middle and out at the arm's reach, where the structure
   is softest. This is the go/no-go: a contact-to-background ratio
   comfortably above 10 is straightforward; near 2 the tap needs to be
   harder, the band different, or the other kind of channel. Try another
   `highpass`/`lowpass` band and a gyroscope `tap_channel` before giving
   up.
5. **`ACCEL_TAP_PROBE SAMPLES=10`** at one point - the scatter. Repeat at
   a second speed: the slope of contact height against speed is the
   detection delay, the scatter's growth with speed its jitter.
6. **Cross-check the probe.** `ACCEL_TAP_PROBE` and `PROBE` at the same
   point; the difference is the BLTouch's `z_offset`.
7. **Switch `[stepper_z]`** to `accel_tap:z_virtual_endstop`. Set
   `position_endstop` to the contact Z that `ACCEL_TAP_PROBE` reports at
   `PROBE_SPEED` equal to `second_homing_speed` - the speed of the tap
   `G28 Z` keeps - and use `HOME_Z_TAP` in place of `HOME_Z`.

## What tapping costs

* **Every home marks the bed.** A stopped nozzle still travels for the
  detection and relay delay plus the deceleration, pushing with whatever
  the Z drive can produce. Tap slowly, vary the spot, keep retracts short.
* **A hot nozzle taps on its ooze.** Hence `max_extruder_temp`: tap cold,
  or after a wipe.
* **A false trigger is the failure that matters** - see
  [The guards](#the-guards).

## Interaction with B and RTCP

* The tap needs B = 0: only then is the nozzle the lowest point of the
  head. Home B first, and use `measure_b: True` to have the sensor confirm
  it rather than trusting the commanded angle.
* Probing runs with RTCP and the B projection off, as for the BLTouch;
  `ACCEL_TAP_PROBE` and `ACCEL_TAP_CALIBRATE` refuse otherwise.
* Tapping at a nonzero B, to measure the head's rotation geometry, is not
  supported.

## Tests

`python test/multi_axis/test_accel_z_tap.py` drives the real module
against a stubbed printer, MCU and a synthetic BMI160 sample stream: the
Butterworth sections against SciPy, a Python copy of the firmware's
fixed-point `sos_filter_apply()` against the float model, the commands
sent to the MCU, arming and the blind distance, every guard, two-tap
agreement in `G28 Z`, probe sampling and retries, and the three
commissioning commands. Latency, the real contact signature and every
threshold need the machine.

## Not implemented

* Back-dating the trigger to the sample time (see [Latency](#latency)).
* A command for the BLTouch cross-check - `ACCEL_TAP_PROBE` then `PROBE`
  at the same point does it by hand.
* A CSV dump of a tap capture. `ACCELEROMETER_MEASURE` around a slow
  descent captures the accelerometer channels for offline analysis.
* Tapping at a nonzero B.
