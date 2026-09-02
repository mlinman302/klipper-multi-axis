# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

# A feedrate low enough to stand in for "as slow as this axis can go".
# Only reached within microns of the centre of the bed.
MIN_CENTRE_VELOCITY = 0.01
# Passed to move.limit_speed() where only the velocity is being limited
NO_ACCEL_LIMIT = 999999999.9
# |u| / |offset| at which the bed's angular acceleration peaks - see
# path_geometry() below
ANGULAR_ACCEL_PEAK_U = 1. / math.sqrt(3.)


######################################################################
# Bed centre geometry
######################################################################

# On a rotating bed machine the bed angle is not a commanded axis - it is
# derived, theta = atan2(y, x) - so it has no value at all on the line
# x = y = 0, and its derivatives have already run away on the approach.
# A move is a straight line, so all of that behaviour follows from two
# numbers: the perpendicular offset of the line of travel from the centre,
#
#     offset = x0*uy - y0*ux        (u the unit direction of travel)
#
# which is constant along the move, and the arclength u measured from the
# foot of that perpendicular.  In terms of those,
#
#     r(u)^2     = offset^2 + u^2
#     theta_dot  = v * offset / r^2
#     theta_ddot = -2 * v^2 * offset * u / r^4  +  a * offset / r^2
#
# theta_dot peaks where r is smallest, at v*offset/r_min^2.  The geometric
# part of theta_ddot peaks a little further out, at |u| = |offset|/sqrt(3),
# where it reaches 0.65 * v^2 / offset^2; the second part is the move's own
# acceleration carried through the same 1/r^2.
#
# So the bed's angular velocity diverges as 1/r and its acceleration as
# 1/r^2 - which is why a near miss that passes a feedrate check can still
# overrun the step compressor on the bed queue, and why both limits land on
# the feedrate rather than on the move's acceleration.
#
# klippy/extras/polar_singularity.py turns these into limits and refuses
# the moves no feedrate can rescue.

def path_geometry(start_pos, end_pos):
    # Bed centre geometry of the xy segment start_pos -> end_pos: the
    # signed perpendicular offset of the path from the centre, the closest
    # the path comes to the centre, and the arclengths of the two endpoints
    # from the foot of the perpendicular.
    x0, y0 = start_pos[0], start_pos[1]
    dx, dy = end_pos[0] - x0, end_pos[1] - y0
    length = math.sqrt(dx*dx + dy*dy)
    if not length:
        return 0., math.sqrt(x0*x0 + y0*y0), 0., 0.
    ux, uy = dx / length, dy / length
    offset = x0 * uy - y0 * ux
    u_start = x0 * ux + y0 * uy
    u_end = u_start + length
    if u_start * u_end <= 0.:
        # The path crosses the foot of the perpendicular, so its closest
        # approach really is reached within the move
        r_min = abs(offset)
    else:
        r_min = math.sqrt(offset*offset
                          + min(u_start*u_start, u_end*u_end))
    return offset, r_min, u_start, u_end

def swept_angle(start_pos, end_pos):
    # Signed change in bed angle over an xy segment, in radians.  A
    # straight line can never sweep more than half a turn, so this is just
    # the angle between the two position vectors and needs no unwrapping.
    # An endpoint sitting on the centre has no angle of its own and
    # contributes nothing, which is what makes a move that departs from
    # [0, 0, N] along a ray a zero sweep rather than an undefined one.
    x0, y0 = start_pos[0], start_pos[1]
    x1, y1 = end_pos[0], end_pos[1]
    cross = x0 * y1 - y0 * x1
    dot = x0 * x1 + y0 * y1
    if not cross and not dot:
        return 0.
    return math.atan2(cross, dot)

def peak_angular_velocity(velocity, offset, r_min):
    # Largest |theta_dot| the bed sees over the move.  A radial move -
    # including one that departs from or arrives at the centre - never
    # turns the bed at all, and a non-zero offset keeps r_min away from
    # zero, so this never divides by zero.
    if not offset:
        return 0.
    return abs(velocity * offset) / (r_min * r_min)

def peak_angular_accel(velocity, offset, u_start, u_end):
    # Largest |theta_ddot| the geometry alone contributes over the move.
    # Its magnitude rises with |u| to a peak at |offset|/sqrt(3) and falls
    # away after it, so the worst point of this move is that peak clamped
    # into the range of |u| the move actually covers.
    if not offset:
        return 0.
    if u_start * u_end <= 0.:
        u_lo = 0.
    else:
        u_lo = min(abs(u_start), abs(u_end))
    u_hi = max(abs(u_start), abs(u_end))
    u = min(max(abs(offset) * ANGULAR_ACCEL_PEAK_U, u_lo), u_hi)
    r2 = offset*offset + u*u
    return 2. * velocity * velocity * abs(offset) * u / (r2 * r2)

def limits_for_angular_rates(offset, r_min, u_start, u_end,
                             max_angular_v, max_angular_a):
    # Feedrate and acceleration limits that keep the bed inside its own
    # angular velocity and acceleration limits over this move.  Both bed
    # limits land mostly on the feedrate: theta_dot scales with v, and the
    # geometric part of theta_ddot with v squared.  The move's own
    # acceleration contributes a further a*offset/r^2, which is what the
    # second value bounds.  The two theta_ddot terms peak a little way
    # apart along the path (at r_min and at about 1.15*r_min), so each is
    # given the whole budget rather than half of it - close enough for a
    # limit whose constant is empirical anyway.
    #
    # Either value is None where that limit does not bind.
    if not offset:
        return None, None
    v_limit = a_limit = None
    if max_angular_v:
        v_limit = max_angular_v * r_min * r_min / abs(offset)
    if max_angular_a:
        a_limit = max_angular_a * r_min * r_min / abs(offset)
        shape = peak_angular_accel(1., offset, u_start, u_end)
        if shape:
            v_accel = math.sqrt(max_angular_a / shape)
            if v_limit is None or v_accel < v_limit:
                v_limit = v_accel
    return v_limit, a_limit

def limit_centre_speed(move, max_angular_v, max_angular_a=0.):
    # Apply the bed's angular limits to a move as a feedrate limit.  Left
    # here rather than in the two kinematics so that both apply the same
    # arithmetic.
    offset, r_min, u_start, u_end = path_geometry(move.start_pos,
                                                  move.end_pos)
    v_limit, a_limit = limits_for_angular_rates(
        offset, r_min, u_start, u_end, max_angular_v, max_angular_a)
    if v_limit is None and a_limit is None:
        return
    # A path that passes near enough to the centre asks for a feedrate of
    # zero.  Clamping to a floor rather than returning early is deliberate:
    # the move that most needs limiting used to be the one move that
    # escaped this check altogether.  [polar_singularity] is what turns
    # such a move into an error rather than a crawl.
    if v_limit is None:
        v_limit = NO_ACCEL_LIMIT
    move.limit_speed(max(v_limit, MIN_CENTRE_VELOCITY),
                     NO_ACCEL_LIMIT if a_limit is None else a_limit)


# distance_to_center() used to live here.  It returned the closest a
# segment came to the centre, which is now the second value of
# path_geometry() above - computed the same way, alongside the two other
# numbers a limit needs.


class PolarKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        rail_arm = stepper.LookupRail(config.getsection('stepper_arm'))
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        stepper_bed.setup_itersolve('polar_stepper_alloc', b'a')
        rail_arm.setup_itersolve('polar_stepper_alloc', b'r')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails = [rail_arm, rail_z]
        self.steppers = [stepper_bed] + [ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity, above=0.,
            maxval=self.max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
        self.v_rad_max = config.getfloat(
            'max_angular_velocity', above=0., default=0)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord((-max_xy, -max_xy, min_z))
        self.axes_max = toolhead.Coord((max_xy, max_xy, max_z))
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        arm_pos = stepper_positions[self.rails[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        return [math.cos(bed_angle) * arm_pos, math.sin(bed_angle) * arm_pos,
                z_pos]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if "z" in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if "x" in homing_axes and "y" in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1]**2
    def clear_homing_state(self, clear_axes):
        if "x" in clear_axes or "y" in clear_axes:
            # X and Y cannot be cleared separately
            self.limit_xy2 = -1.
        if "z" in clear_axes:
            self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        # Do actual homing
        if home_xy:
            self._home_axis(homing_state, 0, self.rails[0])
        if home_z:
            self._home_axis(homing_state, 2, self.rails[1])
    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
        # Slow down near center.  A move whose closest approach to the
        # centre was zero used to return here without being limited at
        # all - the one move that most needs the limit was the one move
        # that escaped it.  See the geometry notes at the top of this
        # file; [polar_singularity] is what refuses the moves no feedrate
        # can rescue.
        if self.v_rad_max and (move.axes_d[0] or move.axes_d[1]):
            limit_centre_speed(move, self.v_rad_max)

    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarKinematics(toolhead, config)
