#!/usr/bin/env python
# Host test of the tilting-head probe geometry (klippy/extras/rtcp_probe.py).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* extras/rtcp_probe.py with a stubbed printer, so
# it runs anywhere Python + cffi are available (it does not need a
# compiled c_helper.so, a serial port, or a Linux host).
#
# Run with:  python test/multi_axis/test_rtcp_probe.py
#
# The end-to-end probing and mesh run is covered separately by
# test/klippy/multi_axis_rtcp_probe.test (Linux only).
import math, os, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# stepper.py imports mcu, which needs pyserial; nothing under test uses it
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

from extras import rtcp as rtcp_mod
from extras import rtcp_probe as rtcp_probe_mod


######################################################################
# Stubbed printer environment
######################################################################

class ConfigError(Exception):
    pass

class FakeReactor:
    def monotonic(self):
        return 0.


class FakeRail:
    def __init__(self, position_min, position_max):
        self.range = (position_min, position_max)
    def get_range(self):
        return self.range

class FakeKinematics:
    def __init__(self, rails):
        self.rails = rails
    def get_axis_rail(self, axis_name):
        return self.rails.get(axis_name)

class FakeRotaryAxis:
    # B is a rotary axis object, not one of the kinematics' linear axes:
    # its homed flag lives here, never in toolhead homed_axes
    def __init__(self, gcode_id='B', is_homed=True):
        self.gcode_id, self.is_homed = gcode_id, is_homed
    def get_axis_gcode_id(self):
        return self.gcode_id
    def get_status(self, eventtime=None):
        return {'position': 0., 'homed': self.is_homed,
                'gcode_axis': self.gcode_id}

class FakeExtruder:
    # Sits at index 0 of get_extra_axes() and has no gcode axis id
    def get_name(self):
        return 'extruder'

class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin
        self.position = [0.] * 7
        self.moves = []
        # Deliberately NOT including 'b' - the real corertheta kinematics
        # only ever reports x/y/z here
        self.homed = "xyz"
        self.b_axis = FakeRotaryAxis()
        self.extra_axes = [FakeExtruder(), None, self.b_axis]
    def get_extra_axes(self):
        return list(self.extra_axes)
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        return list(self.position)
    def get_status(self, eventtime):
        return {'homed_axes': self.homed}
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

class FakeRTCP:
    # Stands in for [rtcp]: rtcp_probe only reads the frame and the
    # enable flag off it, and asks it to enforce the latter
    def __init__(self, frame, enabled=True):
        self.frame, self.enabled = frame, enabled
    def check_disabled(self, what):
        if self.enabled:
            raise ConfigError("%s must run with RTCP compensation off"
                              % (what,))

class FakeProbe:
    def __init__(self, offsets, b_offset):
        self.offsets, self.b_offset = offsets, b_offset
    def get_offsets(self, gcmd=None):
        return tuple(self.offsets)
    def get_b_offset(self):
        return self.b_offset

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func

class FakePrinter:
    config_error = ConfigError
    command_error = ConfigError
    def __init__(self):
        self.objects = {'gcode': FakeGCode()}
        self.event_handlers = {}
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=Ellipsis):
        if name in self.objects:
            return self.objects[name]
        if default is not Ellipsis:
            return default
        raise ConfigError("Unknown config object '%s'" % (name,))
    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

class FakeConfig:
    error = ConfigError
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'rtcp_probe'
    def get(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return self.values[option]
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return default
    def getfloat(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return float(self.values[option])
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return None if default is None else float(default)
    def getboolean(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return bool(self.values[option])
        return default
    def has_value(self, option):
        return option in self.values

class FakeGCmd:
    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []
    error = ConfigError
    def get(self, name, default=Ellipsis):
        return self.params.get(name, default)
    def get_float(self, name, default=Ellipsis, **kwargs):
        if name in self.params:
            return float(self.params[name])
        if default is Ellipsis:
            raise ConfigError("Missing %s" % (name,))
        return default
    def respond_info(self, msg):
        self.responses.append(msg)


# The reference machine: config/example-corertheta.cfg.  The nozzle
# points straight down at B=0 by definition, and the BLTouch pin does so
# at B=45 - that angle is the probe's b_offset.  At that angle the pin
# hangs straight below the pivot, which is straight above the carriage,
# so the probe has no x/y offset from the toolhead and triggers 0.1mm
# above it.
PROBE_B_OFFSET = 45.
PROBE_OFFSETS = (0., 0., -0.1)
BED_RADIUS = 50.


def build(config_values=None, offsets=PROBE_OFFSETS,
          b_offset=PROBE_B_OFFSET, frame=rtcp_mod.FRAME_RADIAL,
          b_range=(-45., 100.), r_range=(0., 149.21), connect=True,
          bed_radius=BED_RADIUS):
    printer = FakePrinter()
    printer.add_object('rtcp', FakeRTCP(frame))
    printer.add_object('probe', FakeProbe(offsets, b_offset))
    kin = FakeKinematics({'b': FakeRail(*b_range), 'r': FakeRail(*r_range)})
    printer.add_object('toolhead', FakeToolhead(kin))
    values = {'bed_radius': bed_radius}
    if config_values is not None:
        values.update(config_values)
    values = {k: v for k, v in values.items() if v is not None}
    obj = rtcp_probe_mod.load_config(FakeConfig(printer, values))
    if connect:
        obj._connect()
        # Park B where the probe faces the bed, as the orient command
        # does, and drop into the carriage frame that probing runs in
        obj.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = b_offset
        printer.lookup_object('rtcp').enabled = False
    return printer, obj


######################################################################
# The angles
######################################################################

class TestAngles(unittest.TestCase):
    def setUp(self):
        self.printer, self.rp = build()

    def test_the_nozzle_points_down_at_b_zero(self):
        # Not derived from anything - it is the definition
        self.assertEqual(self.rp.get_nozzle_b_position(), 0.)

    def test_the_probing_angle_is_the_probes_b_offset(self):
        self.assertAlmostEqual(self.rp.get_probe_b_position(),
                               PROBE_B_OFFSET, places=9)

    def test_a_probe_b_offset_outside_the_b_range_is_rejected(self):
        with self.assertRaises(ConfigError) as cm:
            build(b_offset=120.)
        self.assertIn("b_offset", str(cm.exception))

    def test_the_removed_options_name_their_replacements(self):
        for old in ('probe_b_offset', 'probe_b_position',
                    'invert_b_direction'):
            with self.assertRaises(ConfigError) as cm:
                build({old: 1.})
            self.assertIn(old, str(cm.exception))


######################################################################
# The radial frame
######################################################################

class TestRadialFrame(unittest.TestCase):
    def test_zero_offsets_are_the_identity(self):
        _, rp = build()
        for pt in ((0., 0.), (25., 0.), (-30., 12.), (0., -50.)):
            self.assertAlmostEqual(rp.bed_to_tool(pt)[0], pt[0], places=9)
            self.assertAlmostEqual(rp.bed_to_tool(pt)[1], pt[1], places=9)

    def test_a_radial_offset_only_changes_the_arm_radius(self):
        # x_offset is along the arm, so the bed angle is untouched.
        # bed_radius is left out: an outboard probe cannot reach the
        # centre of the bed, which is the next test.
        _, rp = build(offsets=(8., 0., -0.1), bed_radius=None)
        tool = rp.bed_to_tool((0., 40.))
        self.assertAlmostEqual(math.hypot(*tool), 32., places=9)
        self.assertAlmostEqual(math.atan2(tool[1], tool[0]), math.pi / 2.,
                               places=9)

    def test_bed_to_tool_and_back_round_trips(self):
        for offsets in ((0., 0., -0.1), (8., 0., -0.1), (-6., 3.5, 0.2),
                        (0., -4., 0.)):
            _, rp = build(offsets=offsets, bed_radius=None)
            for pt in ((25., 0.), (-30., 12.), (0., -45.), (18., -22.)):
                tool = rp.bed_to_tool(pt)
                back = rp.tool_to_bed(tool[0], tool[1])
                self.assertAlmostEqual(back[0], pt[0], places=8, msg=str(pt))
                self.assertAlmostEqual(back[1], pt[1], places=8, msg=str(pt))

    def test_a_point_inside_the_tangential_offset_is_unreachable(self):
        # No bed angle can swing a probe 5mm off the arm onto a point 2mm
        # from the centre
        _, rp = build(offsets=(0., 5., -0.1))
        with self.assertRaises(ConfigError):
            rp.bed_to_tool((2., 0.))

    def test_an_outboard_probe_that_cannot_reach_the_centre_is_rejected(self):
        with self.assertRaises(ConfigError) as cm:
            build(offsets=(20., 0., -0.1))
        self.assertIn("x_offset", str(cm.exception))

    def test_an_inboard_probe_reaching_the_whole_bed_is_accepted(self):
        build(offsets=(-20., 0., -0.1))


######################################################################
# The cartesian frame
######################################################################

class TestCartesianFrame(unittest.TestCase):
    def test_offsets_are_plain_subtraction(self):
        _, rp = build(offsets=(3., -2., -0.1),
                      frame=rtcp_mod.FRAME_CARTESIAN)
        self.assertEqual(rp.bed_to_tool((25., 10.)), [22., 12.])
        self.assertEqual(rp.tool_to_bed(22., 12.), (25., 10.))


######################################################################
# Probe results
######################################################################

class TestProbeResult(unittest.TestCase):
    def test_z_offset_keeps_its_stock_meaning(self):
        _, rp = build(offsets=(0., 0., -0.1))
        res = rp.create_probe_result((25., 0., 3.5))
        self.assertAlmostEqual(res.bed_z, 3.6, places=9)
        self.assertAlmostEqual(res.test_z, 3.5, places=9)

    def test_the_result_names_where_the_probe_touched(self):
        _, rp = build(offsets=(8., 0., -0.1), bed_radius=None)
        res = rp.create_probe_result((32., 0., 1.))
        self.assertAlmostEqual(res.bed_x, 40., places=9)
        self.assertAlmostEqual(res.bed_y, 0., places=9)
        self.assertAlmostEqual(res.test_x, 32., places=9)


######################################################################
# The guards
######################################################################

class TestGuards(unittest.TestCase):
    def test_probing_with_rtcp_on_is_refused(self):
        printer, rp = build()
        printer.lookup_object('rtcp').enabled = True
        with self.assertRaises(ConfigError) as cm:
            rp.check_probe_ready()
        self.assertIn("RTCP", str(cm.exception))

    def test_probing_at_the_probing_angle_is_allowed(self):
        _, rp = build()
        rp.check_probe_ready()

    def test_probing_away_from_the_probing_angle_is_refused(self):
        _, rp = build()
        rp.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = 0.
        with self.assertRaises(ConfigError) as cm:
            rp.check_probe_ready()
        self.assertIn("RTCP_PROBE_ORIENT", str(cm.exception))

    def test_the_angle_check_can_be_turned_off(self):
        _, rp = build({'check_probe_b_angle': False})
        rp.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = 0.
        rp.check_probe_ready()

    def test_oriented_needs_both_rtcp_off_and_the_right_angle(self):
        printer, rp = build()
        self.assertTrue(rp.get_status()['oriented'])
        printer.lookup_object('rtcp').enabled = True
        self.assertFalse(rp.get_status()['oriented'])
        printer.lookup_object('rtcp').enabled = False
        rp.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = 0.
        self.assertFalse(rp.get_status()['oriented'])


######################################################################
# Orienting the head
######################################################################

class TestOrient(unittest.TestCase):
    def setUp(self):
        self.printer, self.rp = build()

    def _orient(self, mode):
        gcmd = FakeGCmd({'MODE': mode})
        self.rp.cmd_RTCP_PROBE_ORIENT(gcmd)
        return gcmd

    def test_mode_probe_turns_b_to_the_probing_angle(self):
        self.rp.toolhead.moves = []
        self._orient('PROBE')
        turn = self.rp.toolhead.moves[-1]
        self.assertEqual(turn[0][rtcp_probe_mod.B_POS_INDEX],
                         PROBE_B_OFFSET)

    def test_mode_tool_turns_b_to_zero(self):
        self.rp.toolhead.moves = []
        self._orient('TOOL')
        turn = self.rp.toolhead.moves[-1]
        self.assertEqual(turn[0][rtcp_probe_mod.B_POS_INDEX], 0.)

    def test_orienting_needs_b_homed(self):
        self.rp.toolhead.b_axis.is_homed = False
        with self.assertRaises(ConfigError):
            self._orient('PROBE')

    def test_orienting_with_rtcp_on_and_unhomed_axes_is_rejected(self):
        self.printer.lookup_object('rtcp').enabled = True
        self.rp.toolhead.homed = "y"
        with self.assertRaises(ConfigError) as cm:
            self._orient('PROBE')
        self.assertIn("SET_RTCP ENABLE=0", str(cm.exception))

    def test_orienting_with_rtcp_off_needs_only_b_homed(self):
        self.rp.toolhead.homed = ""
        self.rp.toolhead.moves = []
        self._orient('PROBE')
        self.assertEqual(
            self.rp.toolhead.get_position()[rtcp_probe_mod.B_POS_INDEX],
            PROBE_B_OFFSET)


if __name__ == '__main__':
    unittest.main()
