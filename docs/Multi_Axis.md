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
| `test/multi_axis/run_c_tests.sh` | any host with a C compiler | Shared time base, core r-theta coefficients, RTCP geometry, 3-axis regression, benchmark |
| `test/multi_axis/test_gcode_pipeline.py` | any host with Python + cffi | Real `gcode.py`, `gcode_move.py`, `Move`, `LookAheadQueue`, `RotaryAxis` |
| `test/multi_axis/test_rtcp_probe.py` | any host with Python + cffi | Tilting-head probe geometry, the radial probe transform, its config checks |
| `test/klippy/multi_axis.test` | Linux (`scripts/test_klippy.py`) | Uncoupled A/C axes: config load, homing, step generation |
| `test/klippy/multi_axis_rtheta.test` | Linux (`scripts/test_klippy.py`) | Coupled core r-theta stage |
| `test/klippy/multi_axis_rtcp.test` | Linux (`scripts/test_klippy.py`) | RTCP on a B axis tilting head |
| `test/klippy/multi_axis_rtcp_probe.test` | Linux (`scripts/test_klippy.py`) | Probing and round-bed mesh with the probe on the tilting head |

```bash
bash test/multi_axis/run_c_tests.sh && python test/multi_axis/test_gcode_pipeline.py     && python test/multi_axis/test_rtcp_probe.py
```

## RTCP (Rotational Tool Center Point)

The machine has a head that tilts about **B** (an axis parallel to Y).
With `[rtcp]` configured, g-code commands where the **tool tip** should
be; as the head tilts, the tip swings about the pivot, so the linear
carriages move to hold the tip where it was asked to be.

```
[rtcp]
pivot_length: 40.0     # mm from the tool tip to the B pivot, at B=0
#pivot_x_offset: 0.0   # X offset from tip to pivot at B=0, if any
#enable: True
```

### The transform

With the pivot at offset `(px, pz)` from the tip when `B = 0`:

```
machine_x = x + px*(cos b - 1) + pz*sin b
machine_y = y
machine_z = z - px*sin b       + pz*(cos b - 1)
```

The `-1` terms normalise the transform so that machine coordinates equal
the commanded tip position at `B = 0`.  That keeps homing, bed mesh, Z
offsets and every existing config value in the frame they already use —
`B = 0` behaves exactly like a machine without RTCP.

Sanity check at `B = 90` with `px = 0`: the tool now points along −X from
the pivot, so the carriage must move `+pz` in X and `−pz` in Z to leave
the tip alone.  That is what the transform gives, and what
`test_kin_6axis.c` asserts.

### How it is applied

`kin_rtcp.c` *wraps* each kinematic stepper rather than replacing it, the
same way `kin_idex.c` does, so it composes with cartesian, corexy or
generic_cartesian underneath: the corrected coordinate is handed to the
original solver.  `[rtcp]` installs the wrappers at connect time exactly
as `[input_shaper]` does.

Two details matter:

* **The correction is evaluated at every sample time**, out of the shared
  six-axis queue.  It is therefore continuous *through* a move, not just
  correct at the endpoints — this is the whole reason the rotational axes
  had to move into the toolhead trapq.
* **A stepper driven by X or Z becomes active on B.**  `rtcp_set_sk()`
  ORs `AF_B` into the wrapped stepper's `active_flags`.  Without that, a
  rotation-only move would be considered irrelevant to the X and Z
  steppers, no steps would be generated for them, and the tip would swing
  away instead of staying put.

### Reported positions

`kin.calc_position()` works in machine (carriage) coordinates.  With RTCP
active those differ from the tip coordinates g-code uses, so the inverse
transform is applied when reading positions back out of the steppers —
in `HomingMove.calc_toolhead_pos()` and in `GET_POSITION`.  Everything
the user sees stays in the tip frame.

### Reach checking

The kinematics check the *tip* position against the axis limits, but the
carriages go somewhere else.  `[rtcp]` therefore registers a
`toolhead.register_move_check()` callback that maps both move endpoints
into machine coordinates and compares them against
`axis_minimum`/`axis_maximum`.  Checking the endpoints is sufficient
because B varies monotonically within a move, so the offset does too.

This check runs for any move that reaches the motion queue, **including
rotation-only moves** — under RTCP a bare `G1 B45` moves the carriages.

Plan the rail travel for it: with a 40 mm pivot and B limited to ±45°,
the X carriage swings ±28.3 mm beyond the tip position and the Z carriage
dips 11.7 mm below it.

### Commands

`SET_RTCP [ENABLE=0|1] [PIVOT_LENGTH=<mm>] [PIVOT_X_OFFSET=<mm>]` toggles
or retunes the compensation at runtime.  Changing it re-syncs the
toolhead position, since the same tip position now maps to a different
machine position.

### Scope

Only B is compensated — that is the machine this fork targets.  A and C
are carried through the transform untouched.  Extending to a full
three-rotation head would mean composing three rotations (and fixing an
order convention), and belongs in `rtcp_calc_position()`.

## Probing and mesh bed levelling on a tilting head

A probe bolted to the tilting head is not at a fixed offset from the
nozzle.  It swings with the head, and on the core r-theta machine the
offset it ends up at is **radial** - along the arm - because the bed
angle the probe sits at is the same as the nozzle's.  `[probe]`
`x_offset`/`y_offset`, which are fixed cartesian numbers, cannot express
that.

`klippy/extras/rtcp_probe.py` supplies the geometry instead.  The model
is two concentric circles about the B pivot: the nozzle tip rides one of
radius `L = hypot(pivot_x_offset, pivot_length)`, the probe point rides
one of radius `L + z_offset`, and `probe_b_offset` is the angle between
them.  A negative probe `z_offset` - "the probe reads the bed low" - is
exactly a probe circle that much smaller, so `z_offset` is *consumed by
the geometry* and is not subtracted a second time as it is on a normal
printer.

With `psi` the rotation away from the probing angle
(`B - probe_b_position`) and `theta = psi - probe_b_offset`, the probe
sits at

```
radial offset = L*sin(theta) - (L+z_offset)*sin(psi)
z offset      = L*cos(theta) - (L+z_offset)*cos(psi)
```

from the nozzle tip - which, with `[rtcp]` enabled, is the position the
toolhead reports.

### How it is applied

`probe.py` looks up an optional printer object named `probe_transform`
and, when one is present, uses it in place of the configured x/y/z
offsets at four points:

| Site | Without a transform | With one |
| --- | --- | --- |
| `DescendToEndstopHelper.descend_until_trigger` (descent target) | the z axis `position_min` | `descend_limit_z()`, so it is the *probe* that stops at `position_min` |
| `DescendToEndstopHelper.descend_until_trigger` (result) | `manual_probe.create_probe_result` | `create_probe_result()` |
| `ProbePointsHelper._move_next` | subtract x/y offsets | `bed_to_tool()` |
| `ProbePointsHelper.start_probe` (`horizontal_move_z` check) | the probe `z_offset` | `get_z_endstop_position()` - how far the probe reaches below the nozzle |

`G28 Z` needs no hook of its own: `homing.py` derives the homed z from
the probe result's `bed_z`, so the transform reaches it through
`create_probe_result()`.  `HomingViaProbeHelper.get_position_endstop()`
deliberately stays static - it is read while the rails are still being
built, before any transform exists.

Everything downstream - `[bed_mesh]`, `[z_tilt]`, `[bed_tilt]`,
`PROBE_ACCURACY` - consumes `ProbeResult` and needs no change.  Manual
probing (`METHOD=manual`) ignores the transform: there the nozzle does
the touching.

### Consequences worth knowing

* **`horizontal_move_z` is a nozzle height.**  While probing, the probe
  hangs `-z offset` below the nozzle - about 20mm on the reference
  machine - so `horizontal_move_z` must exceed that or the probe is
  dragged through the bed.  `probe.py` refuses the calibration if it
  does not.  `RTCP_PROBE_INFO` prints the figure.
* **`G28 Z` needs the probe facing the bed** and, since Z is not homed
  yet, the head has to already be above `orient_lift_z`.  The `HOME_Z`
  macro in `config/example-corertheta.cfg` runs `RTCP_PROBE_ORIENT
  MODE=PROBE` first.
* **The sign of `probe_b_offset` decides whether the bed centre is
  reachable.**  The arm radius cannot go negative, so a probe that sits
  *outboard* of the nozzle can never be brought over the middle of the
  bed.  Set `bed_radius` and klippy checks this at startup.
* **The mesh turns with the bed.**  The g-code x/y frame of a polar
  machine is fixed to the bed, and `[stepper_c]` does not home, so the
  angular origin is wherever the bed happened to be at startup.  A saved
  `BED_MESH_PROFILE` is therefore meaningless after a restart - the mesh
  has to be recalibrated before each print.
* **`PROBE_CALIBRATE` and `Z_OFFSET_APPLY_PROBE` do not apply.**  They
  assume the probe is a fixed distance below the nozzle.  Calibrate
  `z_offset` as the difference between the two circle radii instead.

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
