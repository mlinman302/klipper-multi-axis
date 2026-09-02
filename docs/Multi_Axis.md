# Multi-Axis Support (A/B/C rotational axes)

This document describes the design of the "additional axes" support in
this fork of Klipper.  It is intended to be read alongside
[Code_Overview.md](Code_Overview.md), which describes the stock Klipper
motion pipeline.

A machine may declare extra *rotational* axes:

| G-Code word | Rotation about | Config letter |
| ----------- | -------------- | ------------- |
| `A`         | X axis         | `a`           |
| `B`         | Y axis         | `b`           |
| `C`         | Z axis         | `c`           |

Axis positions are expressed in **degrees**.

## The central design decision: one motion space

Stock Klipper's motion queue (`trapq`) carries three coordinates.  Extra
axes such as the extruder get a *separate* queue of their own.

That is not sufficient here, for two reasons:

* **Coupled drives.**  A core r-theta stage has two motors on one belt,
  where each motor position is a linear combination of the radial
  position *and* the end-effector rotation.  A stepper kinematic can only
  read one queue, so if X and C live in different queues no single
  stepper can be a function of both.
* **RTCP.**  Keeping the tool tip stationary while the head tilts means
  evaluating linear and rotational position *at the same instant of the
  same move*.  Separate queues have independent move boundaries, so there
  is no common time base to sample.

So this fork widens the motion space itself from three axes to **six**:

```c
// klippy/chelper/trapq.h
#define KIN_AXES 6

struct coord {
    union {
        struct { double x, y, z, a, b, c; };
        double axis[KIN_AXES];
    };
};
```

All six travel in the toolhead's single `trapq`.  Any stepper kinematic
callback can therefore read any combination of them at one `move_time`.
Machines with no rotational axes simply leave `a`/`b`/`c` at zero, and
the existing 3-axis kinematics are unaffected (they read `.x/.y/.z` and
never see the difference).

The extra widening costs nothing measurable: Klipper builds the C helper
with `-flto -fwhole-program`, so a callback that reads one component has
the other five eliminated at link time.  A before/after benchmark of
`cartesian_stepper_alloc('x')` over 20M samples showed 2.7 ns/call
before and 2.5 ns/call after (i.e. within noise).

## Configuration

The set of extra axes is declared in the `[printer]` section:

```
[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
additional_axes: a, b
```

`additional_axes` accepts the letters `a`, `b` and `c` in any order, with
`,`, whitespace or nothing as a separator (`abc`, `a b c` and `a,b,c` are
equivalent).

Each declared letter must then be driven in one of two ways.

### 1. A dedicated stepper (uncoupled)

One motor drives the axis directly.  Declare `[stepper_a]` /
`[stepper_b]` / `[stepper_c]`:

```
[stepper_a]
step_pin: PF0
dir_pin: PF1
enable_pin: !PD7
microsteps: 16
rotation_distance: 360      # degrees of axis travel per motor rotation
gear_ratio: 50:1
endstop_pin: ^PJ1           # optional - enables "G28 A"
position_endstop: 0
position_min: 0
position_max: 360
homing_speed: 30
axis_max_velocity: 120      # optional, deg/s
axis_max_accel: 1000        # optional, deg/s^2
```

See [sample-multi-axis.cfg](../config/sample-multi-axis.cfg).

### 2. A kinematics carriage (coupled)

Several motors together drive the axis.  Declare it as a carriage of
`[generic_cartesian]` and give each stepper a coefficient on it.  A core
r-theta stage looks like this:

```
[carriage carriage_c]
axis: c
position_endstop: 0
position_min: 0
position_max: 360
endstop_pin: ^PD2

[stepper rtheta_1]
carriages: 0.5*carriage_x + 0.5*carriage_c
...

[stepper rtheta_2]
carriages: 0.5*carriage_x - 0.5*carriage_c
...

[printer]
kinematics: generic_cartesian
additional_axes: c
```

Moving X alone turns both motors the same way; moving C alone turns them
in opposition.  This is exactly the CoreXY idiom, applied to a mix of a
linear and a rotational axis — which only works because both coordinates
are in one queue.  A full example is
[test/klippy/multi_axis_rtheta.cfg](../test/klippy/multi_axis_rtheta.cfg).

When `additional_axes` names a letter with no `[stepper_<letter>]`
section, the axis is assumed to be coupled and is resolved against the
kinematics carriage of the same letter.

## Where the code lives

```
klippy/chelper/trapq.{c,h}         the six-axis coordinate and motion queue
klippy/chelper/itersolve.{c,h}     per-axis active flags, set_position
klippy/chelper/kin_generic.c       linear combination of all six axes
                                     (this is what expresses core r-theta)
klippy/chelper/kin_rotary_axis.c   the uncoupled rotational stepper
klippy/chelper/kin_rtcp.c          the RTCP transform (wraps a solver)
klippy/chelper/kin_bproject.c      the bed-frame B projection (wraps a solver)
klippy/chelper/__init__.py         build + cffi declarations
klippy/stepper.py                  kin_coords() - the position gather
klippy/toolhead.py                 position vector, Move, trapq_append
klippy/kinematics/rotary_axis.py   RotaryAxis / CoupledRotaryAxis
klippy/kinematics/generic_cartesian.py  a/b/c carriages
klippy/kinematics/kinematic_stepper.py  six-coefficient parsing
klippy/extras/gcode_move.py        A/B/C g-code words
klippy/extras/homing.py            homing across six axes
klippy/extras/motion_report.py     six-axis trapq dumps
klippy/extras/rtcp.py              installs RTCP, reach checks
klippy/extras/b_projection.py      installs the B projection, speed limits
klippy/extras/rtcp_probe.py        probe geometry on the tilting head
```

## Position vector layout

The toolhead position vector is:

```
index:   0  1  2  3  4  5  6   7+
        [x, y, z, e, a, b, c, ...additional gcode axes]
```

The extruder deliberately stays at index 3.  Moving it would touch dozens
of `extras/` modules (`Coord.e`, `axes_d[3]`, `_fill_coord`,
firmware_retraction, exclude_object, …) and would conflict with every
future upstream merge.

The consequence is that the six kinematic axes are **not contiguous** in
that vector.  `stepper.kin_coords()` performs the gather:

```python
KIN_AXIS_INDEXES = (0, 1, 2, 4, 5, 6)   # x, y, z, a, b, c
```

It always gathers (filling missing trailing entries with zero) rather
than padding, which means short single-dimension vectors such as the
extruder's `[pos, 0., 0.]` gather correctly too.  That is why no
kinematics module needed changing: they still receive and forward the
toolhead position vector unmodified.

The `a`/`b`/`c` slots are always allocated, declared or not, so the
indexes are fixed.  Undeclared letters hold a `DummyRotaryAxis` that
registers no g-code word and rejects motion.

## Data flow of a `G1 X10 A45` command

```
GCodeDispatch._process_commands()        klippy/gcode.py
    splits the line into params {'X': '10', 'A': '45'}
        |
GCodeMove.cmd_G1()                       klippy/extras/gcode_move.py
    self.axis_map maps 'A' -> position index 4
        |
ToolHead.move(newpos, speed)             klippy/toolhead.py
    Move() computes axes_d/axes_r for every index, but move_d only from
    indices 0..2, so the A component does not change the feedrate
        |
    RotaryAxis.check_move()  -> range check (+ optional speed limits)
    LookAheadQueue           -> junction planning (X/Y/Z geometry only)
        |
ToolHead._process_lookahead()
    trapq_append(... sp[0],sp[1],sp[2], sp[4],sp[5],sp[6],
                     ar[0],ar[1],ar[2], ar[4],ar[5],ar[6] ...)
    -> ONE queue entry carrying both the linear and rotational motion
        |
itersolve_generate_steps()               klippy/chelper/itersolve.c
    each stepper's callback samples the coordinates it needs at move_time
        |
stepcompress -> MCU step queue
```

### `needs_trapq`

`Move.is_kinematic_move` is false when X/Y/Z do not move.  Upstream that
also means "skip the trapq", which would silently drop a rotation-only
move now that rotation lives there.  The two ideas are therefore split:

* `is_kinematic_move` — X/Y/Z moved.  Still gates `kin.check_move()` and
  the junction geometry.
* `needs_trapq` — *any* of the six kinematic axes moved.  Gates
  `trapq_append`.

An extrude-only move sets neither, exactly as before.

## G-Code surface

* `G0`/`G1` — `A`/`B`/`C` words, honouring `G90`/`G91`.
* `G92` — accepts `A`/`B`/`C`.
* `G28` — `G28 A` homes the named rotational axes.  A bare `G28` homes
  X/Y/Z only, so existing start g-code keeps working.
* `M114`, `GET_POSITION` — report the rotational axes.
* `SET_GCODE_OFFSET` — accepts `A`/`B`/`C` and `A_ADJUST` etc.
* `SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE` — save and restore them.
* `SET_ROTARY_AXIS AXIS=A [SET_POSITION=<deg>] [ENABLE=0|1]`.

## Status fields

`toolhead.extra_axes` maps an axis name to its position index, e.g.
`{'extruder': 3, 'rotary_axis a': 4}`.  Each axis also publishes:

```
{'position': 45.0, 'homed': True, 'gcode_axis': 'A', 'rotates_about': 'x',
 'position_min': 0.0, 'position_max': 360.0,
 'max_velocity': 120.0, 'max_accel': 1000.0}
```

Because the rotational axes ride in the toolhead queue,
`DUMP_TRAPQ NAME=toolhead` and the `motion_report/dump_trapq` API now
report six-component positions and ratios.  `scripts/motan` can plot
`trapq:toolhead:a`, `:b`, `:c` (and their velocity/accel variants)
alongside the linear axes.

## Tests

| Test | Runs on | Covers |
| ---- | ------- | ------ |
| `test/multi_axis/run_c_tests.sh` | any host with a C compiler | Shared time base, core r-theta coefficients, RTCP geometry, the bed-frame B projection, 3-axis regression, benchmark |
| `test/multi_axis/test_gcode_pipeline.py` | any host with Python + cffi | Real `gcode.py`, `gcode_move.py`, `Move`, `LookAheadQueue`, `RotaryAxis` |
| `test/multi_axis/test_rtcp_probe.py` | any host with Python + cffi | Tilting-head probe geometry, the radial probe transform, its config checks |
| `test/klippy/multi_axis.test` | Linux (`scripts/test_klippy.py`) | Uncoupled A/C axes: config load, homing, step generation |
| `test/klippy/multi_axis_rtheta.test` | Linux (`scripts/test_klippy.py`) | Coupled core r-theta stage |
| `test/klippy/multi_axis_rtcp.test` | Linux (`scripts/test_klippy.py`) | RTCP on a B axis tilting head |
| `test/klippy/multi_axis_rtcp_probe.test` | Linux (`scripts/test_klippy.py`) | Probing and round-bed mesh with the probe on the tilting head |
| `test/klippy/multi_axis_bproject.test` | Linux (`scripts/test_klippy.py`) | Bed-frame B projection: held leans through a bed turn, the taper band, `SET_B_PROJECTION` |

```bash
bash test/multi_axis/run_c_tests.sh && python test/multi_axis/test_gcode_pipeline.py     && python test/multi_axis/test_rtcp_probe.py
```

## RTCP (Rotational Tool Center Point)

The machine has a head that tilts about **B** (an axis parallel to Y).
`[rtcp]` describes the tool on that head and switches the compensation on
and off.

```
[rtcp]
tool_vertical_offset: 40.0     # mm the tool tip sits below the B pivot
#tool_horizontal_offset: 0.0   # mm it sits inboard of the pivot
#horizontal_frame:             # radial or cartesian; follows kinematics
#enable: True
```

### The two conventions

Everything else follows from two rules, and neither is configurable:

* **At `B = 0` the nozzle points straight down**, exactly as on a printer
  with no tilting head.  This holds whether compensation is on or off, so
  machine coordinates equal commanded coordinates at `B = 0` and homing,
  the bed mesh, Z offsets and every existing config value stay in the
  frame they already use.
* **A positive B tilts the nozzle outboard** — the tip swings away from
  the centre of the bed and rises.  A machine that turns the other way
  inverts B in its kinematics (`invert_b_direction` in `[printer]` /
  `[corertheta]`), never in `[rtcp]`.  Keeping the rotation sense in one
  place is the point: it is the one thing that cannot be derived, and it
  used to be settable in two.

The tool is then two numbers, each positive in the direction the tool
actually sits: `tool_vertical_offset` (how far the tip is *below* the
pivot) and `tool_horizontal_offset` (how far it is *inboard* of the
pivot, towards the centre of the bed).

### The toggle

| | `SET_RTCP ENABLE=1` | `SET_RTCP ENABLE=0` |
| --- | --- | --- |
| g-code commands | the tool tip | the carriage |
| a B move | swings the tip, so the carriages move to cancel it | just turns the head |
| used for | printing | homing, probing, bed mesh |

Toggling does not move the machine: `SET_RTCP` converts the reported
position between the two frames so the carriages stay where they are.  At
`B = 0` the two frames coincide and the conversion is a no-op, which is
why the example macros toggle there.

The right hand column is enforced rather than advised.  `G28` refuses to
home anything at all with compensation on, and `PROBE`, `G28 Z` through
`probe:z_virtual_endstop` and `BED_MESH_CALIBRATE` refuse as well — see
"Why homing and probing run with RTCP off" below.  `[b_projection]`,
where it is configured, is off for exactly the same operations and
refuses them the same way, so the two are toggled together.

### The transform

Writing `h` for the horizontal offset and `v` for the vertical one, the
tip sits at `(-h, -v)` relative to the pivot at `B = 0`.  Rotating that
by `b` and subtracting the `b = 0` value gives the carriage displacement:

```
dh(b) = h*(cos b - 1) - v*sin b
dz(b) = h*sin b       + v*(cos b - 1)
```

Sanity check at `b = 90` with `h = 0`: `dh = -v`, `dz = -v`.  The nozzle
has tilted a quarter turn outboard, so the tip is now level with the
pivot and `v` further out; the carriage retreats by `v` and drops by `v`
to leave the tip alone.  That is what `test_kin_6axis.c` and
`test_gcode_pipeline.py` both assert, with the same numbers.

### Which direction is "horizontal"

The tip swings in the direction the tool leans, which is not the same
axis on every machine:

* **`cartesian`** — the tip swings along +X.  `machine_x = x + dh`.
  For a cartesian, corexy or generic_cartesian gantry.
* **`radial`** — the tip swings along the arm, away from the centre of
  the bed.  For a polar machine such as corertheta, where x/y are *bed*
  coordinates and the arm travels in radius.  The correction scales x and
  y together, so it changes the arm radius and leaves the bed angle
  alone.

The default follows the kinematics — radial for `corertheta` and `polar`,
cartesian otherwise — and `horizontal_frame` overrides it.  Getting this
wrong on a polar machine is not a small error: a cartesian correction is
only right at bed angle zero, and swings the arm sideways everywhere
else.

`dz` is applied to Z in both frames.

### How it is applied

`kin_rtcp.c` *wraps* each kinematic stepper rather than replacing it, the
same way `kin_idex.c` does, so it composes with cartesian, corexy,
generic_cartesian or corertheta underneath: the corrected coordinate is
handed to the original solver.  `[rtcp]` installs the wrappers at connect
time exactly as `[input_shaper]` does.

Two details matter:

* **The correction is evaluated at every sample time**, out of the shared
  six-axis queue.  It is therefore continuous *through* a move, not just
  correct at the endpoints — this is the whole reason the rotational axes
  had to move into the toolhead trapq.
* **A stepper driven by a linear axis becomes active on B.**
  `rtcp_set_sk()` ORs `AF_B` into the wrapped stepper's `active_flags`.
  Without that, a rotation-only move would be considered irrelevant to
  those steppers, no steps would be generated, and the tip would swing
  away instead of staying put.  In the cartesian frame that means X and
  Z; in the radial frame Y joins them, since the correction scales x and
  y together.

### Reported positions

`kin.calc_position()` works in machine (carriage) coordinates.  With RTCP
active those differ from the tip coordinates g-code uses, so the inverse
transform is applied when reading positions back out of the steppers —
in `HomingMove.calc_toolhead_pos()` and in `GET_POSITION`.  Everything
the user sees stays in the tip frame.

`rtcp.py` repeats the transform in Python rather than calling the C
through cffi, so a position can be converted on a host with no compiled
`c_helper.so`.  The two implementations have to be kept in step; the
tests pin both to the same numbers.

### Reach checking

The kinematics check the *tip* position against the axis limits, but the
carriages go somewhere else.  `[rtcp]` therefore registers a
`toolhead.register_move_check()` callback that maps both move endpoints
into machine coordinates and compares them against
`axis_minimum`/`axis_maximum`.  Checking the endpoints is sufficient
because B varies monotonically within a move, so the offset does too.

In the radial frame it additionally rejects a move whose corrected arm
radius would go negative.  The transform would happily produce a point on
the far side of the bed centre at the same bed angle — which is within
the machine's x/y bounds, and is not somewhere the arm can be.  It is
also the one case the inverse transform cannot recover, since `hypot()`
has already lost the sign.

This check runs for any move that reaches the motion queue, **including
rotation-only moves** — under RTCP a bare `G1 B45` moves the carriages.

Plan the rail travel for it: with a 40 mm tool offset and B limited to
±45°, the horizontal carriage swings ±28.3 mm beyond the tip position and
the Z carriage dips 11.7 mm below it.

### Commands

`SET_RTCP [ENABLE=0|1] [VERTICAL_OFFSET=<mm>] [HORIZONTAL_OFFSET=<mm>]`
toggles or retunes the compensation at runtime.

### Scope

Only B is compensated — that is the machine this fork targets.  A and C
are carried through the transform untouched.  Extending to a full
three-rotation head would mean composing three rotations (and fixing an
order convention), and belongs in `rtcp_calc_position()`.

## Bed-frame B on a rotating bed (`[b_projection]`)

On the core r-theta machine the head tilts about the machine's Y axis,
so the tool can only lean within the machine's XZ plane.  The bed,
however, turns underneath it.  A tool orientation expressed in the *bed*
frame — which is the frame the G-code X/Y words already use — is
therefore not reachable in general, and the best the machine can do is
the projection of it onto the plane it can tilt in.

`[b_projection]` makes that the meaning of `B`.  A commanded `B` is a
lean toward the bed's +X direction, and the machine is driven to

```
b_machine = b * cos(theta),   theta = atan2(y, x)   (the bed angle)
```

so holding `B10` through a full turn of the bed sweeps the machine's B
over 10 → 0 → −10 → 0 → 10.  `cos(theta)` is `x/|xy|`, so no
trigonometry is involved.

Whether the machine reads `B` this way at all is one setting:
`enable` in `[b_projection]`.  With it `False`, `B` is a plain
machine tilt — the head leans along the arm by exactly the angle
commanded, at every bed position — which is also the frame homing
and probing want, so nothing has to be toggled for them.  The
macros turn the transform off for those operations and then
`SET_B_PROJECTION RESTORE=1` rather than forcing it on, so the
config setting is what they come back to and a machine configured
`False` stays that way without any macro being edited.

### Why the B axis owns this, and not the bed

The obvious alternative is to make the bed axis (`[stepper_c]`) own the
compensation, since the correction is a function of the bed angle.  It
cannot:

* **The bed motor's own position does not change.**  It still follows
  `atan2(y, x)`.  The bed only *supplies* the angle.
* **A stepper kinematic computes its own stepper's position and nothing
  else.**  There is no way for the bed solver to reach across and rewrite
  the B another solver will read.

B, on the other hand, is *consumed* in three places — both gantry motors
of the differential and the RTCP tip correction — so the remap has to sit
ahead of all three.  It is installed as a wrapping
`stepper_kinematics` (the `kin_shaper.c` / `kin_rtcp.c` idiom), *outside*
the RTCP wrapper, so the whole chain below it sees one consistent B: the
angle the head is really turned to.  Getting that ordering wrong would
make RTCP hold the tip still for a tilt the head is not actually making.

The ordering is established once, by the order of the two `klippy:connect`
handlers, and both modules then find the wrapper they installed by
stepper name.  That last part matters: a module that instead recognised
its own wrapper by reading the stepper's outermost kinematic back would,
once something had wrapped outside it, fail to recognise anything and
wrap a *second* time.  `SET_RTCP` would retune that new outer copy while
the original went on compensating underneath — the compensation
reported off and still applied, with the projection sandwiched between
two RTCP transforms.  `test_gcode_pipeline.py`'s `TestStepperWrapping`
pins the chain.

It also has to live in C rather than in a Python move transform, for the
same reason RTCP does: the bed angle changes continuously *within* a
move, so a per-move transform would only be right at the endpoints.

### Why the scaling applies at every angle

It used to stop above `max_angle`.  Not every `B` is a print angle:
`RTCP_PROBE_ORIENT MODE=PROBE` swings the head to the probe's `b_offset`
(45 on this machine) to point the pin down, and `G28 B` parks it at -90;
both are *machine* angles and have to arrive unscaled.  So angles at or
beyond `max_angle + taper_range` passed straight through, smoothstepped
over the taper band so the machine's B stayed continuous across the
switch, and klippy refused to start if the probe's `b_offset` was not
outside the band.

Continuity was the wrong thing to aim for.  Everything the projection
held back below the threshold - `(1 - cos(theta)) * max_angle` - had to
be paid out inside the few degrees of taper above it, so the band was not
a smoothed step but a very steep ramp, and its slope grew as the bed
angle approached square-on.  At a bed angle of 76 degrees on the example
machine, `G1 B40` -> `B50` came out as **40.6 degrees of head travel for
a 10 degree command**, peaking at a gain of 10x; `[rtcp]` then turned
that into 42 mm of arm and 24 mm of Z.  `max_b_velocity` can only slow
such a move down - the excursion is the same however slowly it is taken.

Nor could the band be re-tuned around it.  Keeping print moves out of it
needs `max_angle` above the largest print angle, while the probe check
needs `max_angle + taper_range` at or below `|b_offset|`.  With a probe
at 45 degrees and print angles to 50, no value satisfies both.

So the band is gone.  The scaling now applies at every `B`, the ratio at
a given X/Y is the same at `B5` as at `B50`, and orientation angles take
an explicit route instead: **homing, probing and `RTCP_PROBE_ORIENT` run
with the projection off**, exactly as they already run with RTCP off.
`check_disabled()` refuses each of them otherwise, and the macros in
`config/example-corertheta.cfg` toggle the two transforms together.
`max_angle` and `taper_range` are refused as config options rather than
ignored, since a config that still sets them meant something else.

`_check_move()` still limits `max_b_velocity`, for the case it was really
needed for: a move that swings the bed while `B` is held changes the
machine's B without the g-code asking for any B travel at all.

### The band, and why it is tapered

Not every `B` is a print angle.  `RTCP_PROBE_ORIENT MODE=PROBE` swings
the head to the probe's `b_offset` (45 on this machine) to point the pin
down, and `G28 B` parks it at −90; both have to reach the machine
untouched.  Angles at or beyond `max_angle + taper_range` therefore pass
straight through, and klippy refuses to start if the probe's `b_offset`
is not one of them.

Switching at a hard threshold would put a discontinuity of up to
`max_angle` degrees into the machine's B — at a bed angle of 90°, `B39.9`
maps to ≈0 while `B40.1` maps to 40.1 — which arrives at the step
compressor as thousands of steps in a few microseconds ("Internal error
in stepcompress").  The correction is instead faded out with a smoothstep
over `taper_range` degrees above `max_angle`, and `_check_move()` slows
any move whose machine B moves faster than the commanded B — crossing the
band, or swinging the bed while B is held — down to `max_b_velocity`.

### Reported positions

`calc_position()` resolves the *machine* B out of the gantry motors, so
`calc_toolhead_pos()` maps it back through `machine_to_commanded()` after
the RTCP inverse.  With no band the projection is a plain scaling, so the
inverse is a plain division by `cos(theta)`.  The one place it fails is a
bed angle square to the tilt plane — every commanded B maps to a machine
B of zero there — and in that case the B the toolhead already believes it
is at is kept, which is correct whenever B did not move.  In practice
homing runs with the projection off, where the map is the identity and
the inverse is exact.

### Scope

The transform is specific to a rotating bed and is rejected on any
kinematics other than `corertheta`.  A and C are not touched.

## Probing and mesh bed levelling on a tilting head

The probe is bolted to the tilting head, a fixed angle around the B pivot
from the nozzle.  The temptation is to model that as geometry — two
concentric circles about the pivot, the probe's offset from the nozzle a
function of B — and an earlier version of this fork did.  It is the wrong
shape for the problem.

The probe is used in **exactly one orientation**: the B angle at which
its pin hangs vertically.  In that orientation, and with RTCP
compensation off, it is an ordinary probe at a fixed offset from the
toolhead.  So it owns four measured numbers, all in its own config
section (`[bltouch]` here):

```
[bltouch]
b_offset: 45     # B angle at which the pin is vertical
x_offset: 0      # where the probe sits relative to the carriage
y_offset: 0      #   at that angle, with RTCP off
z_offset: -0.1   # how far the trigger point is below the nozzle
```

`b_offset` is also the angular distance from the nozzle round to the
probe, because the nozzle is vertical at `B = 0` by definition.

`z_offset` keeps its stock Klipper meaning and is calibrated the stock
way.  Its geometric meaning here is the difference between the probe's
and the nozzle's radius about the B pivot — but nothing computes it from
that, and nothing should: it is measured.

### Why homing and probing run with RTCP off

Homing has to, on two counts, and neither of them announces itself:

* **A B home would drive the linear axes, unchecked.**  With RTCP on,
  turning B *is* an X/Z move, and a B home sweeps the head across 1.5
  times its range looking for the endstop — so the carriages travel by up
  to the whole tool offset, typically before either of them is homed.
  Nothing catches it, either: a rotation-only move has
  `is_kinematic_move = False`, so `kin.check_move()` is skipped, and
  `drip_move()` — which is what a homing move is — never runs the
  registered move checks, so the reach check above does not fire.
* **A linear home would be booked in the wrong frame.**  What `G28` sets
  is a carriage position, but with compensation on it is read back as a
  tool tip position.  The sweep itself still covers the right distance,
  since B is constant through it and `dh` with it, so the axis simply
  ends up homed `dh` away from where it thinks it is, and nothing raises
  an error.

Probing has three reasons of its own, and they are why its model
collapses to four constants.

1. With RTCP on, turning B *is* an X/Z move, so `G28 B` cannot be
   followed by a probe orientation until X, Y and Z are all homed — and Z
   is exactly what is being homed.  With RTCP off a B move touches no
   linear axis and works with nothing homed.
2. With RTCP off the toolhead position *is* the carriage, so the probe's
   offset from it does not depend on B at all once the head is at the
   probing angle.  There is no geometry left to get wrong.
3. `horizontal_move_z` goes back to being an ordinary carriage height,
   the arm radius is simply the bed radius being probed, and the mesh z
   values are the reported toolhead z at each trigger.

So it is not a recommendation, it is enforced.  `G28` is refused outright
with compensation on, whatever axis it names; `PROBE`, `G28 Z` through
`probe:z_virtual_endstop` and `BED_MESH_CALIBRATE` are refused with it on
*and* refused with B away from the probe's `b_offset`.
`PrinterHoming._check_transforms_disabled()`, `rtcp.check_disabled()` and
`rtcp_probe.check_probe_ready()` do the refusing, and all say what to run
instead.  The homing macros in `config/example-corertheta.cfg` each turn
compensation off for themselves, so they work whatever state the machine
was left in.

`[b_projection]` is refused by the same three call sites, and by
`RTCP_PROBE_ORIENT`.  Its reason is different but points the same way:
the endstop sweep, the park angle and the angle the probe pin hangs
vertical at are all *machine* angles, and the projection would scale
every one of them by whatever bed angle happened to be under the arm —
by nearly zero with the arm square to the tilt plane, so a B home would
never reach its endstop.

The catch is point 3's other half: with RTCP off the arm radius *is* the
bed radius, so probing the centre of the bed drives the arm to radius
zero, where a polar machine's bed angle is undefined.  Home Z off centre,
and if the mesh's centre point misbehaves, either set
`max_angular_velocity` in `[printer]` to slow moves near the middle or
declare a small `faulty_region` around it so `[bed_mesh]` substitutes
neighbouring points.

### The one thing that is not stock

On a polar machine the toolhead's x/y are *bed* coordinates while the
probe is displaced along the arm.  A fixed pair of bed-frame offsets
would only be right at bed angle zero.  So the x/y pair is applied in the
machine's own frame at the toolhead — `x_offset` along the arm (positive
outboard), `y_offset` across it — which on a cartesian machine is just
x and y, and the arithmetic reduces to the stock subtraction.

That is all `klippy/extras/rtcp_probe.py` does with the offsets, plus
turning the head to and from the probing orientation.  It registers
itself as the printer object `probe_transform`, which `probe.py` consults
at three points:

| Site | Without a transform | With one |
| --- | --- | --- |
| `ProbePointsHelper.start_probe` | — | `check_probe_ready()`, before the first move |
| `DescendToEndstopHelper.descend_until_trigger` | `manual_probe.create_probe_result` | `check_probe_ready()`, then `create_probe_result()` |
| `ProbePointsHelper._move_next` | subtract x/y offsets | `bed_to_tool()` |

The z offset is deliberately *not* the transform's business: it keeps its
ordinary meaning, so the `horizontal_move_z` check and the probing
descent limit are stock code.

`G28 Z` needs no hook of its own: `homing.py` derives the homed z from
the probe result's `bed_z`, so the transform reaches it through
`create_probe_result()`.  `HomingViaProbeHelper.get_position_endstop()`
deliberately stays static — it is read while the rails are still being
built, before any transform exists.

Everything downstream — `[bed_mesh]`, `[z_tilt]`, `[bed_tilt]`,
`PROBE_ACCURACY` — consumes `ProbeResult` and needs no change.  Manual
probing (`METHOD=manual`) ignores the transform: there the nozzle does
the touching.

### Consequences worth knowing

* **`G28 Z` needs the probe facing the bed**, with RTCP off so that
  turning B does not become an X/Z move before Z is homed, and — since Z
  is not homed yet — the head already clear of the bed.  The `HOME_Z`
  macro in `config/example-corertheta.cfg` does all three.
  `RTCP_PROBE_ORIENT` refuses to turn B with RTCP on and X/Y/Z unhomed,
  and says to run `SET_RTCP ENABLE=0`.
* **An outboard probe can put the bed centre out of reach.**  The arm
  radius cannot go negative, so a probe with a positive `x_offset` can
  never be brought over the middle of the bed.  Set `bed_radius` in
  `[rtcp_probe]` and klippy checks at startup.
* **The mesh turns with the bed.**  The g-code x/y frame of a polar
  machine is fixed to the bed, and `[stepper_c]` does not home, so the
  angular origin is wherever the bed happened to be at startup.  A saved
  `BED_MESH_PROFILE` is therefore meaningless after a restart — the mesh
  has to be recalibrated before each print.
* **B is not in `homed_axes`.**  A rotational axis is a rotary axis
  object, not one of the kinematics' linear axes, so
  `toolhead.get_status()['homed_axes']` only ever reports x/y/z.  Its
  homed flag is on the axis object itself, reached through
  `toolhead.get_extra_axes()` — which is what `[rtcp_probe]` keeps a
  reference to.  Testing `'b' in homed_axes` always reads false.
* **`PROBE_CALIBRATE` works normally**, since `z_offset` is now an
  ordinary z offset — run it with RTCP off and the probe oriented, as
  everything else in this section.

## Deliberate limitations (current stage)

* **Rotation does not affect the feedrate.**  `G1 X10 A360` takes exactly
  as long as `G1 X10`; the A axis is commanded to cover 360 degrees in
  whatever time the linear move takes.  There is a regression test for
  this (`test_xyz_timing_unaffected_by_rotation`).
* Consequently `axis_max_velocity`, `axis_max_accel` and
  `instantaneous_corner_velocity` all default to **unset**.  Setting any
  of them is the one way a rotational axis can influence planning: it
  then calls `move.limit_speed()`, slowing the move as a whole.
* RTCP compensates B only; A and C rotations do not move the linear
  axes.
* Rotational axes are not part of a kinematics class' linear limits, so
  `kinematics.axis_minimum/maximum` still describes X/Y/Z only.  Probing
  and bed mesh do account for the B angle - see "Probing and mesh bed
  levelling on a tilting head" above - but only through the probe
  geometry, not through the reach checks.

## Next

* **Fold rotation into the planner.**  RTCP makes rotation produce real
  linear motion, so a fast B rotation can in principle command the linear
  axes faster than they can move.  On this machine the rotational axes
  are very slow relative to XYZ, so the stage-1 simplification holds and
  no speed or acceleration checking is done.  If that stops being true,
  the fix is to include the RTCP-induced linear displacement in
  `Move.move_d` and the junction planner.
* **Rotational limits in the kinematics classes**, so `axis_minimum` /
  `axis_maximum` and the front-end status describe the rotational axes
  too.
* **Multi-rotation RTCP** (A and C as well as B), if a future head needs
  it — see "Scope" above.
