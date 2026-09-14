# BMI160: an IMU for B homing and Z homing

**Status: architecture.** The sensor layer, the fused B measurement and
the motion gate are implemented. `B_GYRO_CHECK` and the tap detector
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

### 3. Degrees-per-step from the fused angle, and a check on the gyroscope

**The drive ratio is calibrated on the fused angle from vertical, and on
nothing else.** `B_STEP_CALIBRATE` in
[Accel_B_Homing.md](Accel_B_Homing.md) commands a sweep of stations,
counts the steps the MCU issued, and at each station reads the head
through `measure_vertical()` - the same fused measurement `G28 B` homes
on. `b_coupling_ratio` (or `rotation_distance` on a dedicated B stepper)
is fitted from those angles. The gyroscope's contribution to the
calibration is the fusion itself: it carries each station reading
through the ringing after the move, and its motion gate says the head
was at rest.

Integrating the gyroscope's on-axis rate across a commanded move is
*not* used for the ratio, even though it looks like the direct answer:

    command B +10 degrees, integrate omega over the move

That integral is relative rather than referenced to vertical, and it
carries the gyroscope's own sensitivity error and its zero-rate offset
multiplied by the length of the move - neither of which a fit against
step counts can separate from a ratio error. Fusion bounds both, because
the accelerometer pins the angle to gravity at every station.

What the integral *is* good for is the one question fusion cannot answer
on a parked head: whether the gyroscope's sign is right (see "What checks
this on the machine" below). `B_GYRO_CHECK` runs it as a check - +10.0
confirms sense and scale, -10.0 means the sign is inverted - and writes
nothing to the config. The rotation sense of B itself is still
unverified on this machine; `G28 B`'s capped check move already refuses
a head that turns the wrong way.

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
measurement, not an argument, and phase 5 below captures both. The point
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
what `B_GYRO_CHECK` automates in phase 3. Until that lands, do it by
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
                              and feeds trigger_analog          (MCU -
                              klipper_mcu on the Pi, on this machine)
        |                            |
        | sensor_bulk_data           | trsync_do_trigger
        v                            v
  klippy/extras/bmi160.py      klippy/extras/trigger_analog.py
    - bus setup, chip config     - sos filter, threshold, endstop
    - combined sample stream
    - accel view    gyro view
        |               |                    |
        v               v                    v
  [accel_b_homing]  (fused angle,        [accel_z_tap]      (phase 5)
  [resonance_tester] motion gate)
```

The rule that shapes all of it: **the chip object knows about the chip
and nothing else.** It does not know what B is, it does not know what a
probe is, and it never moves the machine. `[accel_b_homing]` looks it up
by name and reads it through the same small interface
`[resonance_tester]` uses, exactly as it does today with the ADXL345 -
which is why swapping the chip is a config change rather than a code
change for that module.

## This machine: the BMI160 on the Pi Zero W's SPI0

Everything above is chip architecture. This section is what it means on
the corertheta machine, where the BMI160 is wired to **SPI0 of the
Raspberry Pi Zero W that runs klippy** - not to a USB sensor board, which
is what earlier revisions of these documents assumed, and no longer to
the Pi's I2C1, which the first bring-up plan used.

### The topology

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

The breakout labels are the I2C names most modules print, and the
[Config_Reference](Config_Reference.md#bmi160) note on ambiguous labels
applies (SCL, not SCX; SDA, not SDX). The kernel drives CE0 itself, which
is why the config says `cs_pin: rpi:None`. The host facts - Pi Zero W
Rev 1.1, one core, ~430 MB, Raspbian bookworm, LPC1769 main MCU, no
`[mcu rpi]` configured yet - were read from the machine's Moonraker API
on 2026-09-12.

The chip powers up speaking I2C and switches to SPI on the first rising
edge of its chip select, staying there until it loses power. The driver
provides that edge with a dummy read of register 0x7F before it checks
the chip id, at the start of every measurement, so there is nothing to
configure - but it is why a hand-written test script has to do the same
(see "Bringing it up").

`sensor_bmi160.c` needs no port for this: it is ordinary Klipper MCU
code, it already has an SPI path, and the Linux process target already
provides `/dev/spidev`. What changes is the budget.

### Why SPI rather than I2C

On I2C1 the bus was the bottleneck. Klipper's Linux I2C driver ignores
`i2c_speed`, a Pi defaults to 100 kHz, and draining one 48-byte FIFO
block costs about 510 bits on the wire, so the driver's default 1600 Hz
needed 204 % of a stock bus and 51 % of one raised to 400 kHz. That made
`rate: 400` a ceiling rather than a choice, and the Z tap's
repeatability - which scales with the poll interval - inherited the
ceiling.

On SPI the same block is one 3-byte length read and one 49-byte data
read: 416 bits, clocked at `spi_speed`, which the Linux SPI driver *does*
honour (the BCM2835 rounds it down to 125 MHz over an even divisor, so
1 MHz becomes 992 kHz):

| `rate` | blocks/s | wire time at 1 MHz | at 4 MHz | I2C at 400 kHz, for comparison |
| --- | --- | --- | --- | --- |
| 1600 Hz | 400 | 17 % | 4 % | 51 % |
| 800 Hz | 200 | 8 % | 2 % | 26 % |
| 400 Hz | 100 | 4 % | 1 % | 13 % |
| 200 Hz | 50 | 2 % | 0.5 % | 6 % |

Those are wire-time lower bounds, and on SPI they are no longer the
binding cost. Each block is still two `ioctl` system calls from
klipper_mcu on a single ARMv6 core, and that overhead - which cannot be
computed, only measured - is now what limits the rate. The driver's
1 MHz default is left alone: it already takes the wire out of the
picture, and a slower clock is more forgiving of the long, unshielded
run out to the head.

### `rate: 400` costs the B measurement nothing

SPI makes 1600 Hz affordable, but this machine still runs `rate: 400`,
because B does not want more.

It is tempting to read a lower rate as a less precise measurement. For a
window average it is not. With the chip's normal-mode digital filter,
bandwidth scales with the output data rate, so the samples stay
band-limited below Nyquist and the noise on a mean over a window of
length T is set by the noise density and T alone - about
`density / sqrt(2 T)`. At the datasheet's 180 ug/sqrt(Hz) over the
default half second that is roughly 0.18 mg, or about 0.01 degrees of
tilt, **at any rate**. More samples of a narrower-band signal are not
more information.

And the lower rate buys margin wherever the Pi is short:

* **FIFO headroom.** 85 frames is 53 ms at 1600 Hz and 212 ms at 400 Hz:
  four times as long for klipper_mcu to be late before frames are lost.
* **Host CPU.** 100 bulk messages a second into klippy instead of 400,
  and a quarter of the system calls in klipper_mcu, on the one core that
  is also planning motion.
* **The gyro gate's noise floor.** `max_rotation_rate` tests a mean of
  per-sample magnitudes, which does depend on bandwidth. Estimating the
  gyro's normal-mode bandwidth at about 0.4 x ODR, a parked head reads
  roughly 0.14 deg/s at 400 Hz against 0.28 deg/s at 1600 Hz - both under
  the 1.0 default, but the lower rate leaves the gate more room.
* **Fusion.** `fusion_tau` is 0.2 s; a 2.5 ms step is still eighty
  samples per time constant.

So this machine runs `rate: 400` - 4 % of a 1 MHz wire - and 1600 Hz is
left to the Z tap, where it is a question of detection jitter rather than
of what the bus allows.

### Lost frames are now loud

The overflow check the driver inherited could never fire. It compared the
FIFO's byte count against 1024, but the counter saturates at the FIFO
size rather than exceeding it, and the skip frame the chip uses to report
lost frames exists only in header mode, which this driver does not use.
A link that could not keep up would therefore have silently overwritten
frames - and because Klipper timestamps bulk samples by *counting* them,
the frames that survived would have been mistimed too, which a fused
angle integrates directly into its answer.

`sensor_bmi160.c` now reports a possible overflow whenever the FIFO is
too full to accept another whole frame, and `[accel_b_homing]` refuses a
measurement during which the overflow count rose, naming `rate` as the
thing to change. The refusal reads the overflow count every Klipper
accelerometer puts in its batches, so it protects an `[adxl345]` too.

### What a bad link does

SPI has no acknowledge, and that changes how a wiring fault shows up.
On I2C a chip that did not answer was a NACK, and a NACK on klipper_mcu
was a shutdown of the whole printer. On SPI the bus cannot tell: an
unplugged chip, a broken MISO wire or a flipped bit all just read back as
data. Only a transfer the kernel itself refuses is a shutdown ("Unable to
issue spi ioctl"), and that means a missing `/dev/spidev0.0`, not a bad
wire.

So the checks move from the bus into the driver:

* **Every measurement starts with the chip id.** An absent chip or an
  open MISO reads as `0x00` or `0xff` rather than `0xd1`, and the
  measurement fails with "Invalid bmi160 id" - an error on the command,
  not a printer shutdown.
* **Every configuration write is read back.** A register that does not
  hold its value fails the measurement the same way.
* **A corrupted FIFO length shows up as invalid frames.** Reading past
  the end of the FIFO returns a fixed 0x80 pattern, which the driver
  drops and counts in each batch's `errors`. `[accel_b_homing]` does not
  yet refuse on that count - see "Open questions".
* **A flipped bit inside a sample is not detected by anything** - nor
  was it on I2C, whose acknowledge confirms that a byte was clocked, not
  that it arrived intact. At
  +/-2 g the top two bits are worth 2 g and 1 g, and a spike that size
  in a 200-sample window pushes the deviation past `max_sample_deviation`'s
  500 mm/s^2 default, so the measurement is refused. The next bit down
  is not caught: a 0.5 g spike moves a 200-sample mean by 2.5 mg, about
  0.14 degrees - far above the noise floor above, and invisible. Phase 0
  should look for it by repeating `B_MEASURE` on a parked head.

The exposure is also limited to measurements: the chip is read only
while a client is measuring, so a print that never calls `B_MEASURE`
never exercises the link.

The wiring is six conductors rather than four, and SPI's push-pull edges
are sharper than I2C's pulled-up ones - better against cable
capacitance, worse for crosstalk. If the BMI160 is on the moving head and
the Pi is not, keep the run short and away from motor leads, run a ground
alongside SCLK, and treat an intermittent "Invalid bmi160 id" as wiring
before software. A breakout built for 5 V I2C, with level-shifting
transistors on SDA and SCL, may not pass a 1 MHz clock; drop `spi_speed`
to 400000 to tell.

### Bringing it up

Nothing here has been done on the machine yet. In order:

1. **Enable SPI0.** On bookworm the file is `/boot/firmware/config.txt`:

   ```
   dtparam=spi=on
   ```

   and reboot. `ls /dev/spidev0.*` should list `spidev0.0` and
   `spidev0.1`. I2C1 is not needed for the IMU, so there is no reason to
   enable it.

2. **Find the chip, before Klipper is involved.** SPI has no
   `i2cdetect`, so read the chip id directly. `sudo apt install
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

3. **Build klipper_mcu from this branch.** The host MCU must understand
   this branch's `config_bmi160`, which gained a `bytes_per_frame`
   argument; a klipper_mcu built from anything older is refused at
   connect. SPI and BMI160 support are both on by default in the Linux
   process build - leave them on. Use a separate config and output
   directory, so the LPC1769 build configuration in `.config` is left
   alone:

   ```
   cd ~/klipper
   make KCONFIG_CONFIG=config.rpi OUT=out_rpi/ menuconfig    # Linux process
   sudo systemctl stop klipper
   make KCONFIG_CONFIG=config.rpi OUT=out_rpi/ flash
   sudo cp scripts/klipper-mcu.service /etc/systemd/system/
   sudo systemctl enable --now klipper-mcu
   sudo systemctl start klipper
   ```

   Expect the build to be slow on an ARMv6 core. The LPC1769 does not
   need reflashing for any of this.

4. **Configure it** - see the `[mcu rpi]` and `[bmi160]` sections of
   `config/example-corertheta.cfg`.

5. **Check the budget before trusting a reading.** `BMI160_QUERY`, then
   `B_MEASURE` a few times with the head parked. Any "possible fifo
   overflows" refusal at `rate: 400` over SPI means the Pi, not the wire,
   is too loaded; lower `rate` before anything else.

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
deliberate calibration command rather than something to do at startup -
and on the B axis, not the right one. It trims every axis against a pose
declared exact, so a head a few degrees off vertical is booked as
vertical, and it is lost on power cycle. `B_SENSOR_CALIBRATE` fits the
offsets and gain ratio in software instead, from a sweep that needs no
exact pose, and `SAVE_CONFIG` keeps them (see
[Accel_B_Homing.md](Accel_B_Homing.md)). The two must not be combined:
accelerometer FOC changes the raw readings that fit was made on.

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

The reading carries the two under their own names and has no field
meaning "whichever was available". **Everything that acts on B reads
`measure_vertical()`**, which returns only `fused_angle` and refuses a
chip without a gyroscope: `G28 B`, the endstop direction and
verification, `CHECK=1`, and the drive ratio calibration. `measured_b`
in `get_status()` is only ever a fused angle. `accel_angle` is kept
alongside - reported by `B_MEASURE` and as `accel_b` in the status - so
the two estimators can be compared on the machine rather than trusted,
but nothing acts on it.

The filter deliberately stops at the **end of the measurement window**
rather than the end of the capture. The trailing `batch_margin` is
delivery slack that the user did not ask to measure, and running the
filter through it would leave the fused angle and the accelerometer tail
it is checked against describing different instants.

Without a gyroscope the module is a diagnostic only. An `[adxl345]` or
a `[lis2dw]` exposes no gyroscope client, so fusion and the gate are both
skipped and `B_MEASURE` reports the accelerometer-only angle, labelled as
not fused - but `G28 B` and `CHECK=1` refuse, since there is no fused
angle to act on. There is no config option to turn fusion off, for the
same reason; `B_MEASURE FUSION=0` does it for one diagnostic reading.

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

`B_GYRO_CHECK` - job 3 above - remains new work: start the stream, run
a commanded B move, integrate the on-axis rate across it, report swept
angle against commanded angle. It is a check, not a calibration: it
writes nothing, and the drive ratio comes from fused station angles in
`B_STEP_CALIBRATE`. Everything in `[accel_b_homing]` that moves the
machine measures a parked head before and after each move, so it
checks the fused angle's sense of rotation but not the gyroscope's sign
on its own. It is `B_GYRO_CHECK` that finally verifies that sign, per
the limits noted above.

The fused angle also inherits the gyroscope's zero-rate offset as a
fixed bias of offset x `fusion_tau` (see "Open questions"), which
`max_fusion_disagreement` is far too loose to notice. Since `G28 B` now
acts on that angle, run `BMI160_CALIBRATE GYRO=1` after every power
cycle before homing.

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
  is the one place the change could genuinely hurt, and phase 5 measures
  it rather than guessing.

### On this machine: the detector runs in a Linux process

[Accel_Z_Tap.md](Accel_Z_Tap.md) put the detector on a USB board's
RP2040. Here it is klipper_mcu on the Pi Zero W, and the Z steppers are on
the LPC1769. That splits the latency into two parts that do very
different damage, and the distinction is the whole analysis.

**Detection delay biases the Z datum.** `trigger_analog_update()` stamps
a trigger with the MCU clock at the moment it *processes* the sample,
and homing reads the stepper position back at that time. So everything
between the chip sampling the contact and klipper_mcu processing that
sample shifts the recorded Z: the frame's age in the FIFO (up to one
poll interval, four frames), the SPI read, and the chip's own filter
delay. The constant part calibrates out, as for any probe at a fixed
speed. The *variable* part is the frame's age, roughly uniform over the
poll interval, and it goes straight into repeatability:

| `rate` | poll interval | sample-age scatter (1 sigma) | at 2 mm/s | at 5 mm/s |
| --- | --- | --- | --- | --- |
| 400 Hz | 10 ms | 2.9 ms | 6 um | 14 um |
| 1600 Hz | 2.5 ms | 0.7 ms | 1.4 um | 3.6 um |

On top of that, at 400 Hz the chip's own filter narrows to roughly
160 Hz, which may soften the contact transient enough to make detection
later and less consistent. Phase 5 has to measure that; it cannot be
computed.

**Relay delay only causes overshoot.** The trigger then travels
klipper_mcu -> pty -> klippy's serial thread, which forwards it in C
(`trdispatch`, no Python) -> USB -> LPC1769. That time does not move the
recorded position; it moves the nozzle further into the bed. At a slow
final tap speed it is harmless as a distance. Its real risk is Klipper's
25 ms trsync liveness timeout: a single loaded core that delays
klipper_mcu's periodic status by that long aborts the home with
"Communication timeout during homing".

So the Z tap is feasible on this hardware. On I2C the bus capped `rate`
at 400 Hz and with it the tap's repeatability; on SPI the tap can run at
1600 Hz while B stays at 400, and what is left to decide is whether the
Pi keeps up. The levers, cheapest first:

* **Tap slowly.** A fast approach and a slow final tap scales both the
  scatter and the overshoot down with speed.
* **Back-date the trigger.** The firmware knows how many frames were
  still queued behind the one that triggered, so it could stamp the
  trigger with that frame's sample time instead of the processing time,
  removing most of the scatter in the table above. This is a change to
  the `trigger_analog` interface, not a tuning knob.
* **Raise `rate` to 1600 Hz.** SPI0 has made this a config change - 17 %
  of a 1 MHz wire - rather than the hardware change it was on I2C. The
  cost is the Pi's: four times the system calls in klipper_mcu and four
  times the bulk messages into klippy, on the core whose lateness is what
  trips the trsync timeout above. Phase 5 measures whether it can afford
  that.

Moving the chip to the LPC1769, which would avoid the cross-MCU relay,
is possible on SPI where it was not on I2C (Klipper's LPC176x I2C driver
is fixed at 100 kHz), but it trades the Pi's load for a long SPI run to
the mainboard. It is the lever to reach for only if phase 5 shows the Pi
cannot keep up at 1600 Hz.

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

# after - on SPI, for example on a USB sensor board
[bmi160]
cs_pin: imu:gpio9
spi_software_sclk_pin: imu:gpio10
spi_software_mosi_pin: imu:gpio11
spi_software_miso_pin: imu:gpio12

# after - on this machine: SPI0 of the Raspberry Pi running klippy
[mcu rpi]
serial: /tmp/klipper_host_mcu

[bmi160]
cs_pin: rpi:None
spi_bus: spidev0.0
rate: 400
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

0. **Bring-up on the Pi.** The five steps under "Bringing it up": SPI0
   enabled, chip id read, klipper_mcu from this branch, config, and a
   clean run of `B_MEASURE` with no overflow refusals.
1. **Sensor layer.** Combined accel+gyro FIFO, all three streams
   exposed to the host (accelerometer, gyroscope, and the combined one
   fusion reads), configurable range and rate, pseudovector-correct
   `axes_map`, FOC commands, the `trigger_analog` attach point, and
   overflow detection that works in headerless mode. *Implemented.*
2. **The fused measurement and the motion gate.** The complementary
   filter, the derived gyroscope axis and sign, `max_rotation_rate` and
   `max_fusion_disagreement` in `[accel_b_homing]`. All skipped when the
   chip has no gyroscope. *Implemented.*
3. **`B_GYRO_CHECK`.** Integrate rate across a commanded move; report
   sense and scale. The definitive check on the gyroscope's sign, which
   the fused angle depends on. It writes nothing: the drive ratio is
   calibrated on fused station angles by `B_STEP_CALIBRATE`
   ([Accel_B_Homing.md](Accel_B_Homing.md)).
4. **Commissioning `fusion_tau` and `settle_time`.** Measure the head's
   ringing, pick `tau`, and find how far `settle_time` can come down.
   This is the phase that actually collects the benefit of phase 2.
5. **Z tap phase 0.** Capture contact signatures on both channels, at
   several speeds, with the machine also captured moving in air. The
   deliverable is the contact-to-background ratio, the band that
   maximises it, and - on this machine - a verdict on whether the tap
   needs 1600 Hz and whether the Pi can serve it. Measure the tap scatter
   at two speeds: its slope is the detection jitter above.
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

* Is the contact transient visible at 1600 Hz - and at 400 Hz, if the Pi
  cannot keep up with 1600? (Phase 5. This is the question the whole Z
  path rests on.)
* Does klipper_mcu keep the FIFO drained over SPI while klippy is busy,
  at 400 Hz and at 1600 Hz? The wire is not the limit; the single core
  is. Phase 0 answers it for 400 Hz with the overflow check, phase 5 for
  1600 Hz.
* Does the SPI run between the head and the Pi survive the stepper
  cables? SPI has no acknowledge, so the symptom is not a shutdown but an
  intermittent "Invalid bmi160 id", or `B_MEASURE` results that scatter
  more than the noise floor predicts.
* Should `[accel_b_homing]` refuse a measurement during which the chip's
  `errors` count rose, as it does for overflows? On SPI that count is the
  one sign of a corrupted FIFO length read.
* Angular or linear channel for tap detection - or both, ANDed?
  (Phase 5.)
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
