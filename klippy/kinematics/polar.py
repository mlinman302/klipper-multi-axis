# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper
from .bed_centre import path_geometry, limits_for_angular_rates

# A feedrate low enough to stand in for "as slow as this axis can go".
# Only reached within microns of the centre of the bed.
MIN_CENTRE_VELOCITY = 0.01
# Passed to move.limit_speed() where only the velocity is being limited
NO_ACCEL_LIMIT = 999999999.9


# The bed angle is derived from x/y, so it is singular at the centre and
# its rates diverge on the approach.  The geometry is in bed_centre.py;
# this applies it to a move.
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
# bed_centre.path_geometry() - computed the same way, alongside the two
# other numbers a limit needs.


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
        # that escaped it.  See the geometry notes in bed_centre.py;
        # [polar_singularity] is what refuses the moves no feedrate can
        # rescue.
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
