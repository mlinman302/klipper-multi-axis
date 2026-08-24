# Code for handling the kinematics of core r-theta polar robots
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The machine is a traditional polar printer - a rotating bed plus a
# radial arm - whose arm carriage additionally carries a tool that can
# rotate about the y axis (the B axis of docs/Multi_Axis.md):
#
#   [stepper_c]     the rotating bed.  As on any polar printer its angle
#                   is derived from the commanded cartesian x/y and is
#                   not itself a g-code axis.
#   [stepper_x]     first gantry motor.  Carries the X (arm radius)
#                   endstop and position_min/position_max.
#   [stepper_tilt]  second gantry motor.  Carries the B (tool rotation)
#                   endstop and position_min/position_max, in degrees.
#   [stepper_z]     leadscrew raising the gantry.
#
# The two gantry motors are coupled through a differential: driving them
# in the same direction produces B rotation alone, driving them in
# opposition produces linear X motion alone.  This is the CoreXY idiom
# applied to one linear and one rotational coordinate, which works only
# because both coordinates travel in the single six-axis motion queue -
# see docs/Multi_Axis.md.
#
# B is therefore a coupled axis: it must be listed in the
# 'additional_axes' option of [printer] and must NOT have a [stepper_b]
# section of its own.  rotary_axis.CoupledRotaryAxis resolves it through
# get_axis_rail() below.
import logging, math
import stepper
from .polar import distance_to_center
from .rotary_axis import parse_additional_axes


class CoreRThetaKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # The B axis is driven by the kinematics, so it can only exist if
        # the machine declared it
        printer_config = config.getsection('printer')
        if 'b' not in parse_additional_axes(printer_config):
            raise config.error(
                "The corertheta kinematics drives a B axis - add 'b' to the"
                " 'additional_axes' option of the [printer] config section")
        if config.has_section('stepper_b'):
            raise config.error(
                "The corertheta B axis is driven by [stepper_x] and"
                " [stepper_tilt] together - remove the [stepper_b] section")
        # Belt travel (in [stepper_x] units) per degree of B rotation.  The
        # default of 1. makes the differential a plain CoreXY-style sum.
        self.b_ratio = config.getfloat('b_coupling_ratio', 1., above=0.)
        # Which way the gantry motors have to turn for a positive B.  The
        # differential fixes the two axis directions relative to each
        # other, not absolutely: swapping the '+' and '-' solvers below
        # negates X alone, and inverting both motors' dir_pin negates X
        # and B together, so no combination of the two can invert B by
        # itself.  This option is the missing degree of freedom - set it
        # if B rotates the wrong way while X is already correct.
        if config.getboolean('invert_b_direction', False):
            self.b_coeff = -self.b_ratio
        else:
            self.b_coeff = self.b_ratio
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_c'),
                                             units_in_radians=True)
        rail_x = stepper.LookupMultiRail(config.getsection('stepper_x'))
        rail_b = stepper.LookupMultiRail(config.getsection('stepper_tilt'))
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        # Either gantry motor moves both X and B, so each axis' endstop
        # has to watch both of them
        for s in rail_b.get_steppers():
            rail_x.get_endstops()[0][0].add_stepper(s)
        for s in rail_x.get_steppers():
            rail_b.get_endstops()[0][0].add_stepper(s)
        stepper_bed.setup_itersolve('corertheta_stepper_alloc', b'c',
                                    self.b_coeff)
        rail_x.setup_itersolve('corertheta_stepper_alloc', b'-', self.b_coeff)
        rail_b.setup_itersolve('corertheta_stepper_alloc', b'+', self.b_coeff)
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rail_x, self.rail_b, self.rail_z = rail_x, rail_b, rail_z
        self.stepper_bed = stepper_bed
        self.steppers = [stepper_bed] + [s for r in [rail_x, rail_b, rail_z]
                                         for s in r.get_steppers()]
        for s in self.steppers:
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
        max_xy = rail_x.get_range()[1]
        min_z, max_z = rail_z.get_range()
        self.axes_min = toolhead.Coord((-max_xy, -max_xy, min_z))
        self.axes_max = toolhead.Coord((max_xy, max_xy, max_z))
    def get_steppers(self):
        return list(self.steppers)
    def get_axis_rail(self, axis_name):
        # Called by rotary_axis.CoupledRotaryAxis to find the range and
        # endstop of the B axis, which has no stepper of its own
        if axis_name == 'b':
            return self.rail_b
        return None
    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.stepper_bed.get_name()]
        # stepper_x runs the '-' solver (b*ratio - radius) and
        # stepper_tilt the '+' one (b*ratio + radius) - see the
        # setup_itersolve() calls above
        p_minus = stepper_positions[self.rail_x.get_name()]
        p_plus = stepper_positions[self.rail_b.get_name()]
        z_pos = stepper_positions[self.rail_z.get_name()]
        radius = .5 * (p_plus - p_minus)
        b_pos = .5 * (p_plus + p_minus) / self.b_coeff
        # Kinematic axis order is x, y, z, a, b, c.  The bed angle is not
        # a commanded axis, so leave the a and c slots alone.
        return [math.cos(bed_angle) * radius, math.sin(bed_angle) * radius,
                z_pos, None, b_pos, None]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if "z" in homing_axes:
            self.limit_z = self.rail_z.get_range()
        if "x" in homing_axes and "y" in homing_axes:
            self.limit_xy2 = self.rail_x.get_range()[1]**2
    def clear_homing_state(self, clear_axes):
        if "x" in clear_axes or "y" in clear_axes:
            # X and Y cannot be cleared separately
            self.limit_xy2 = -1.
        if "z" in clear_axes:
            self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        toolhead = self.printer.lookup_object('toolhead')
        num_pos = len(toolhead.get_position())
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None] * num_pos
        homepos[axis] = hi.position_endstop
        if axis == 0:
            # The arm homes along +X, which fixes the bed angle at zero
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        if axis == 0:
            # X is the arm radius and the homing sweep runs along y == 0 at
            # a fixed bed angle, so it must not cross the centre.  In polar
            # coordinates a straight line through x == 0 is a half turn of
            # the bed, commanded in the instant the sign of x flips, and it
            # promptly overruns the step compressor ("Internal error in
            # stepcompress" on the stepper_c queue).  Crossing the centre
            # also drives the arm the wrong way first, because the radius
            # the gantry motors follow is |x| - a position_min of -30 backs
            # the arm 30mm away from the endstop before it turns around.
            #
            # A negative position_min is still a legal print coordinate: it
            # is the far side of the bed, reached by turning the bed rather
            # than by driving the arm through the middle.  Only the homing
            # sweep has to stay on one side, and sweeping from a radius of
            # zero still covers the arm's whole radial travel.
            if hi.position_endstop < 0.:
                raise self.printer.config_error(
                    "corertheta position_endstop for stepper_x is the arm"
                    " radius at the endstop and cannot be negative")
            forcepos[axis] = max(forcepos[axis], 0.)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY together.  B is homed by its rotary axis object,
        # which drives rail_b through homing_state directly.
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
            self._home_axis(homing_state, 0, self.rail_x)
        if home_z:
            self._home_axis(homing_state, 2, self.rail_z)
    def check_move(self, move):
        end_pos = move.end_pos
        # Only range check the radius on moves that actually change x or
        # y.  Without this guard a z-only or b-only move (including the
        # homing moves for those axes) is rejected while x/y are still
        # unhomed, which makes it impossible to home anything before x.
        # cartesian.py applies the same axes_d guard in _check_endstops.
        if move.axes_d[0] or move.axes_d[1]:
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
        # Slow down near center
        if move.axes_d[0] or move.axes_d[1]:
            if self.v_rad_max == 0:
                return
            min_dist = distance_to_center(move.start_pos[0:2],
                                          move.end_pos[0:2])
            if min_dist == 0:
                return
            v_angular = math.sqrt(move.max_cruise_v2) / min_dist
            if self.v_rad_max < v_angular:
                scale_radius = self.v_rad_max / v_angular
                move.limit_speed(self.max_velocity * scale_radius,
                                 self.max_accel * scale_radius)
    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CoreRThetaKinematics(toolhead, config)
