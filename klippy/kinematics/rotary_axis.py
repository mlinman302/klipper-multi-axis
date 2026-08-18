# Support for additional rotational axes (A/B/C) on multi-axis machines
#
# Copyright (C) 2025  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# See docs/Multi_Axis.md for the overall design.  In short: a rotational
# axis is a first class kinematic axis.  Its position travels in the
# *toolhead* trapq alongside x/y/z (see 'struct coord' in
# chelper/trapq.h), which is what lets a single stepper kinematic read a
# linear and a rotational coordinate at the same instant - the
# requirement for coupled drives (core r-theta) and for RTCP.
#
# There are two ways an axis gets driven:
#
#   RotaryAxis         one motor drives the axis directly, declared with a
#                      [stepper_a] / [stepper_b] / [stepper_c] section.
#   CoupledRotaryAxis  the motion comes out of the machine kinematics,
#                      where several motors each mix this rotation with
#                      other axes (a core r-theta stage).  Declared as a
#                      carriage in [generic_cartesian] instead.
#
# Either way the object is registered in toolhead.extra_axes so that the
# g-code layer picks up its A/B/C word and range checking has a home; it
# never owns a motion queue.
import logging
import stepper

# Config letter -> (g-code word, cartesian axis rotated about)
ROTARY_AXES = {'a': ('A', 'x'), 'b': ('B', 'y'), 'c': ('C', 'z')}

# Parse the [printer] additional_axes option into a list of letters.
# Accepts "abc", "a b c", "a,b,c" and any mixture of those.
def parse_additional_axes(config):
    raw = config.get('additional_axes', '')
    letters = []
    for ch in raw.lower():
        if ch in ' \t,;':
            continue
        if ch not in ROTARY_AXES:
            raise config.error(
                "Invalid additional_axes entry '%s' in section '%s'"
                " (must be a combination of 'a', 'b' and 'c')"
                % (ch, config.get_name()))
        if ch in letters:
            raise config.error("Duplicate additional_axes entry '%s'" % (ch,))
        letters.append(ch)
    return letters


# Placeholder occupying the slot of a rotational axis that was not
# declared in additional_axes.  It registers no g-code word, so the axis
# is simply unreachable, and it rejects any attempt to move it.
class DummyRotaryAxis:
    def __init__(self, printer, axis_letter):
        self.printer = printer
        self.axis_letter = axis_letter
        self.gcode_id = ROTARY_AXES[axis_letter][0]
    def get_name(self):
        return ""
    def get_axis_letter(self):
        return self.axis_letter
    def get_axis_gcode_id(self):
        return None
    def get_trapq(self):
        return None
    def check_move(self, move, ea_index):
        raise self.printer.command_error(
            "Rotational axis %s is not configured\n"
            "See the 'additional_axes' option of the [printer] config"
            " section" % (self.gcode_id,))
    def calc_junction(self, prev_move, move, ea_index):
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        pass
    def find_past_position(self, print_time):
        return 0.


# Behaviour shared by both kinds of driven rotational axis.  Subclasses
# set: printer, axis_letter, gcode_id, rotates_about, name, trapq, rail,
# steppers, pos_min, pos_max, can_home, is_homed, and the three optional
# limits (max_velocity, max_accel, instant_corner_v).
class BaseRotaryAxis:
    def _register(self):
        self.commanded_pos = 0.
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('SET_ROTARY_AXIS', 'AXIS',
                                   self.gcode_id, self.cmd_SET_ROTARY_AXIS,
                                   desc=self.cmd_SET_ROTARY_AXIS_help)

    ######################################################################
    # Extra axis interface (called by toolhead.py)
    ######################################################################
    def get_name(self):
        return self.name
    def get_axis_letter(self):
        return self.axis_letter
    def get_axis_gcode_id(self):
        return self.gcode_id
    def get_trapq(self):
        return self.trapq
    def get_steppers(self):
        return list(self.steppers)
    def check_move(self, move, ea_index):
        if not self.is_homed:
            raise self.printer.command_error(
                "Must home rotary axis %s first" % (self.gcode_id,))
        movepos = move.end_pos[ea_index]
        if ((self.pos_min is not None and movepos < self.pos_min)
            or (self.pos_max is not None and movepos > self.pos_max)):
            raise move.move_error("Rotary axis %s move out of range"
                                  % (self.gcode_id,))
        if self.max_velocity is None and self.max_accel is None:
            return
        # Slow the move down if the rotation would exceed this axis' limits.
        # The rotation never adds to move.move_d, so all this can do is
        # reduce the speed of the move as a whole.
        axis_d = abs(move.axes_d[ea_index])
        if not axis_d:
            return
        axis_ratio = move.move_d / axis_d
        limit_v = limit_a = 999999999.9
        if self.max_velocity is not None:
            limit_v = self.max_velocity * axis_ratio
        if self.max_accel is not None:
            limit_a = self.max_accel * axis_ratio
        move.limit_speed(limit_v, limit_a)
    def calc_junction(self, prev_move, move, ea_index):
        if self.instant_corner_v is None:
            # Rotation does not take part in junction planning
            return move.max_cruise_v2
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        # Nothing to queue: the toolhead already placed this axis into the
        # shared trapq as part of the move itself.
        self.commanded_pos = move.end_pos[ea_index]
    def find_past_position(self, print_time):
        if not self.steppers:
            return 0.
        mcu_stepper = self.steppers[0]
        mcu_pos = mcu_stepper.get_past_mcu_position(print_time)
        return mcu_stepper.mcu_to_commanded_position(mcu_pos)
    def get_status(self, eventtime=None):
        return {'position': self.commanded_pos,
                'homed': self.is_homed,
                'gcode_axis': self.gcode_id,
                'rotates_about': self.rotates_about,
                'position_min': self.pos_min,
                'position_max': self.pos_max,
                'max_velocity': self.max_velocity,
                'max_accel': self.max_accel}

    ######################################################################
    # Position and homing
    ######################################################################
    def get_position_index(self):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_extra_axes().index(self)
    def set_position(self, newpos):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[self.get_position_index()] = newpos
        toolhead.set_position(pos)
        self.commanded_pos = newpos
    def home(self):
        # The axis is part of the main kinematic space, so homing is an
        # ordinary toolhead homing move along this axis - no private drip
        # move is involved.
        if not self.can_home:
            raise self.printer.command_error(
                "Rotary axis %s has no endstop - cannot home"
                % (self.gcode_id,))
        from extras import homing
        toolhead = self.printer.lookup_object('toolhead')
        pos_index = self.get_position_index()
        hi = self.rail.get_homing_info()
        homepos = [None] * len(toolhead.get_position())
        homepos[pos_index] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[pos_index] -= 1.5 * (hi.position_endstop - self.pos_min)
        else:
            forcepos[pos_index] += 1.5 * (self.pos_max - hi.position_endstop)
        self.is_homed = False
        homing_state = homing.Homing(self.printer)
        homing_state.set_axes([pos_index])
        homing_state.home_rails([self.rail], forcepos, homepos)
        self.is_homed = True
        self.commanded_pos = toolhead.get_position()[pos_index]

    ######################################################################
    # Commands
    ######################################################################
    cmd_SET_ROTARY_AXIS_help = "Set rotary axis position or enable state"
    def cmd_SET_ROTARY_AXIS(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            stepper_enable = self.printer.lookup_object('stepper_enable')
            stepper_enable.set_motors_enable(
                [s.get_name() for s in self.steppers], enable)
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.set_position(setpos)
            self.is_homed = True
        gcmd.respond_info("Rotary axis %s at %.6f (homed=%s)"
                          % (self.gcode_id, self.commanded_pos, self.is_homed))


# A rotational axis driven by its own dedicated stepper(s)
class RotaryAxis(BaseRotaryAxis):
    def __init__(self, config, axis_letter):
        self.printer = config.get_printer()
        self.axis_letter = axis_letter
        self.gcode_id, self.rotates_about = ROTARY_AXES[axis_letter]
        self.name = 'rotary_axis %s' % (axis_letter,)
        # Optional motion limits (deg/s, deg/s^2) and junction limit.  All
        # three are unset by default so that rotation has no influence at
        # all on the toolhead velocity/accel planning - see
        # docs/Multi_Axis.md ("Deliberate limitations").  Setting any of
        # them lets this axis slow a move down.
        self.max_velocity = config.getfloat('axis_max_velocity', None,
                                            above=0.)
        self.max_accel = config.getfloat('axis_max_accel', None, above=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', None, above=0.)
        # Setup stepper(s).  An endstop_pin turns this into a homeable
        # rail, which then also requires position_min/position_max.
        self.can_home = config.get('endstop_pin', None) is not None
        if self.can_home:
            self.rail = stepper.LookupMultiRail(config)
            self.pos_min, self.pos_max = self.rail.get_range()
            self.steppers = self.rail.get_steppers()
        else:
            self.rail = stepper.PrinterStepper(config)
            self.pos_min = config.getfloat('position_min', None)
            self.pos_max = config.getfloat('position_max', None,
                                           above=self.pos_min)
            self.steppers = [self.rail]
        # This stepper reads its own component out of the shared motion
        # space.  (A coupled drive uses generic_cartesian steppers, which
        # take a coefficient on every axis including this one.)
        self.rail.setup_itersolve('rotary_axis_stepper_alloc',
                                  axis_letter.encode())
        # The motion lives in the toolhead trapq, not a private one
        toolhead = self.printer.lookup_object('toolhead')
        self.trapq = toolhead.get_trapq()
        self.rail.set_trapq(self.trapq)
        self.is_homed = not self.can_home
        self._register()


# A rotational axis driven by the machine kinematics (eg, core r-theta)
class CoupledRotaryAxis(BaseRotaryAxis):
    def __init__(self, printer, axis_letter):
        self.printer = printer
        self.axis_letter = axis_letter
        self.gcode_id, self.rotates_about = ROTARY_AXES[axis_letter]
        self.name = 'rotary_axis %s' % (axis_letter,)
        # Speed limiting for a coupled axis belongs to the kinematics,
        # which already limits the motors that drive it
        self.max_velocity = self.max_accel = self.instant_corner_v = None
        toolhead = printer.lookup_object('toolhead')
        self.trapq = toolhead.get_trapq()
        kin = toolhead.get_kinematics()
        get_axis_rail = getattr(kin, 'get_axis_rail', None)
        self.rail = None
        if get_axis_rail is not None:
            self.rail = get_axis_rail(axis_letter)
        if self.rail is None:
            raise printer.config_error(
                "Additional axis '%s' needs either a [stepper_%s] config"
                " section or a carriage on axis '%s' in the printer"
                " kinematics" % (axis_letter, axis_letter, axis_letter))
        self.steppers = self.rail.get_steppers()
        self.pos_min, self.pos_max = self.rail.get_range()
        self.can_home = bool(self.rail.get_endstops())
        self.is_homed = not self.can_home
        self._register()


# Called from toolhead.add_printer_objects() once the toolhead exists
def add_printer_objects(config):
    printer = config.get_printer()
    toolhead = printer.lookup_object('toolhead')
    printer_config = config.getsection('printer')
    for axis_letter in parse_additional_axes(printer_config):
        section = 'stepper_%s' % (axis_letter,)
        if config.has_section(section):
            ra = RotaryAxis(config.getsection(section), axis_letter)
            driven_by = "stepper %s" % (section,)
        else:
            # No dedicated stepper - the axis must be a carriage of the
            # machine kinematics (a coupled drive such as core r-theta)
            ra = CoupledRotaryAxis(printer, axis_letter)
            driven_by = "the printer kinematics"
        printer.add_object(ra.get_name(), ra)
        toolhead.set_rotary_axis(ra)
        logging.info("Added rotational axis %s (about %s), driven by %s",
                     ra.get_axis_gcode_id(), ra.rotates_about, driven_by)
