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
    def __init__(self, pivot_x, pivot_z, enabled=True):
        self.pivot_x, self.pivot_z = pivot_x, pivot_z
        self.enabled = enabled

class FakeProbe:
    def __init__(self, z_offset):
        self.z_offset = z_offset
    def get_offsets(self, gcmd=None):
        return (0., 0., self.z_offset)

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
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'rtcp_probe'
    def getfloat(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return float(self.values[option])
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return None if default is None else float(default)
    def getboolean(self, option, default=Ellipsis):
        if option in self.values:
            return bool(self.values[option])
        return default

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


# The reference machine: config/example-corertheta.cfg
PIVOT_X, PIVOT_Z = 21.94, 65.6
Z_OFFSET = -0.1
PROBE_B_OFFSET = 45.
PROBE_B_POSITION = -26.5
TOOL_RADIUS = math.hypot(PIVOT_X, PIVOT_Z)


def build(config_values=None, pivot=(PIVOT_X, PIVOT_Z), z_offset=Z_OFFSET,
          b_range=(-48., 100.), r_range=(0., 149.21), connect=True):
    printer = FakePrinter()
    printer.add_object('rtcp', FakeRTCP(pivot[0], pivot[1]))
    printer.add_object('probe', FakeProbe(z_offset))
    kin = FakeKinematics({'b': FakeRail(*b_range), 'r': FakeRail(*r_range)})
    printer.add_object('toolhead', FakeToolhead(kin))
    values = {'probe_b_offset': PROBE_B_OFFSET,
              'probe_b_position': PROBE_B_POSITION,
              'bed_radius': 50.}
    if config_values is not None:
        values.update(config_values)
    values = {k: v for k, v in values.items() if v is not None}
    obj = rtcp_probe_mod.load_config(FakeConfig(printer, values))
    if connect:
        obj._connect()
        # Park B where the probe faces the bed, as the orient command does
        obj.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = \
            obj.probe_b_position
    return printer, obj


######################################################################
# Tests
######################################################################

class TestGeometry(unittest.TestCase):
    def setUp(self):
        self.printer, self.rp = build()
    def test_radii(self):
        # The probe rides a circle concentric with the nozzle's, shorter
        # by the probe z_offset
        self.assertAlmostEqual(self.rp.tool_radius, TOOL_RADIUS, places=9)
        self.assertAlmostEqual(self.rp.probe_radius, TOOL_RADIUS + Z_OFFSET,
                               places=9)
    def test_probe_faces_the_bed_at_probe_b_position(self):
        # The probe is straight below the pivot there, so its offset from
        # the pivot is purely vertical
        off_r, off_z = self.rp.get_offsets(self.rp.probe_b_position)
        expect_r = -TOOL_RADIUS * math.sin(math.radians(PROBE_B_OFFSET))
        expect_z = (TOOL_RADIUS * math.cos(math.radians(PROBE_B_OFFSET))
                    - (TOOL_RADIUS + Z_OFFSET))
        self.assertAlmostEqual(off_r, expect_r, places=9)
        self.assertAlmostEqual(off_z, expect_z, places=9)
    def test_probe_is_inboard_and_below_while_probing(self):
        off_r, off_z = self.rp.get_offsets(self.rp.probe_b_position)
        self.assertLess(off_r, 0.)      # towards the centre of the bed
        self.assertLess(off_z, 0.)      # below the nozzle
    def test_nozzle_faces_the_bed_a_probe_b_offset_away(self):
        # Turning B by probe_b_offset swaps which of the two faces down,
        # so there the nozzle is the lower of the pair
        b_tool = self.rp.probe_b_position + PROBE_B_OFFSET
        off_z = self.rp.get_offsets(b_tool)[1]
        self.assertGreater(off_z, 0.)
    def test_z_endstop_is_where_the_nozzle_sits_at_a_trigger(self):
        off_z = self.rp.get_offsets(self.rp.probe_b_position)[1]
        self.assertAlmostEqual(self.rp.get_z_endstop_position(), -off_z,
                               places=9)
        self.assertGreater(self.rp.get_z_endstop_position(), 20.)
    def test_descend_limit_stops_the_probe_not_the_nozzle(self):
        # The nozzle must stop short so it is the probe that reaches the
        # z axis position_min
        limit = self.rp.descend_limit_z(-8.)
        off_z = self.rp.get_offsets(self.rp.probe_b_position)[1]
        self.assertAlmostEqual(limit + off_z, -8., places=9)
        self.assertGreater(limit, 0.)
    def test_derived_probe_b_position(self):
        # Left out of the config it follows from the pivot offsets: the
        # nozzle faces the bed at -atan2(pivot_x, pivot_z)
        _, rp = build({'probe_b_position': None}, pivot=(-PIVOT_X, PIVOT_Z))
        nozzle_b = math.degrees(math.atan2(PIVOT_X, PIVOT_Z))
        self.assertAlmostEqual(rp.probe_b_position,
                               nozzle_b - PROBE_B_OFFSET, places=9)


class TestTransform(unittest.TestCase):
    def setUp(self):
        self.printer, self.rp = build()
        self.off_r = self.rp.get_offsets(self.rp.probe_b_position)[0]
    def _round_trip(self, bed_x, bed_y):
        tool = self.rp.bed_to_tool((bed_x, bed_y))
        pos = [tool[0], tool[1], 12.5, 0., 0., 0., 0.]
        return tool, self.rp.create_probe_result(pos)
    def test_round_trip(self):
        for bed_x, bed_y in [(0., 0.), (50., 0.), (0., -50.), (-25., 10.),
                             (35.355, 35.355), (3., -4.)]:
            tool, res = self._round_trip(bed_x, bed_y)
            self.assertAlmostEqual(res.bed_x, bed_x, places=9)
            self.assertAlmostEqual(res.bed_y, bed_y, places=9)
    def test_offset_is_purely_radial(self):
        # The probe shares the nozzle's bed angle, so only the radius
        # changes - the bed point and the tool point are colinear with
        # the centre and on the same side of it
        for bed_x, bed_y in [(50., 0.), (-25., 10.), (3., -4.)]:
            tool = self.rp.bed_to_tool((bed_x, bed_y))
            self.assertAlmostEqual(bed_x * tool[1] - bed_y * tool[0], 0.,
                                   places=9)
            self.assertGreater(bed_x * tool[0] + bed_y * tool[1], 0.)
            self.assertAlmostEqual(math.hypot(*tool),
                                   math.hypot(bed_x, bed_y) - self.off_r,
                                   places=9)
    def test_bed_centre_is_reached_from_a_safe_radius(self):
        # The whole point of mounting the probe inboard: the arm never has
        # to approach the polar singularity at the centre of the bed
        tool = self.rp.bed_to_tool((0., 0.))
        self.assertAlmostEqual(math.hypot(*tool), -self.off_r, places=9)
        self.assertGreater(math.hypot(*tool), 40.)
    def test_arm_stays_within_its_travel_over_the_whole_bed(self):
        radii = [math.hypot(*self.rp.bed_to_tool((r, 0.)))
                 for r in [0., 12.5, 25., 37.5, 50.]]
        self.assertGreater(min(radii), 0.)
        self.assertLess(max(radii), 149.21)
    def test_bed_z_is_the_nozzle_z_plus_the_probe_offset(self):
        off_z = self.rp.get_offsets(self.rp.probe_b_position)[1]
        pos = [60., 0., 21.5, 0., 0., 0., 0.]
        res = self.rp.create_probe_result(pos)
        self.assertAlmostEqual(res.bed_z, 21.5 + off_z, places=9)
        self.assertEqual((res.test_x, res.test_y, res.test_z),
                         (60., 0., 21.5))
    def test_a_flat_bed_at_the_nominal_height_reads_zero(self):
        # Descending until the probe triggers leaves the nozzle at the z
        # endstop position, which must map back to a bed z of zero
        pos = [60., 0., self.rp.get_z_endstop_position(), 0., 0., 0., 0.]
        self.assertAlmostEqual(self.rp.create_probe_result(pos).bed_z, 0.,
                               places=9)


class TestChecks(unittest.TestCase):
    def test_probing_b_outside_the_axis_range_is_rejected(self):
        # This is what a sign error in probe_b_offset looks like
        with self.assertRaises(ConfigError) as cm:
            build({'probe_b_position': None})
        self.assertIn("outside the", str(cm.exception))
    def test_a_probe_outboard_of_the_nozzle_cannot_reach_the_centre(self):
        # The arm radius cannot go negative, so the middle of the bed is
        # out of reach however far the arm is driven back
        with self.assertRaises(ConfigError) as cm:
            build({'probe_b_offset': -PROBE_B_OFFSET,
                   'probe_b_position': 26.5})
        self.assertIn("only reach bed radii", str(cm.exception))
    def test_probing_with_the_wrong_b_is_rejected(self):
        _, rp = build()
        rp.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = \
            rp.probe_b_position + 10.
        with self.assertRaises(ConfigError) as cm:
            rp.descend_limit_z(-8.)
        self.assertIn("RTCP_PROBE_ORIENT", str(cm.exception))
    def test_the_check_can_be_turned_off(self):
        _, rp = build({'check_probe_b_angle': False})
        rp.toolhead.position[rtcp_probe_mod.B_POS_INDEX] = 0.
        rp.descend_limit_z(-8.)
    def test_probing_with_rtcp_disabled_is_rejected(self):
        printer, rp = build()
        printer.lookup_object('rtcp').enabled = False
        with self.assertRaises(ConfigError):
            rp.get_z_endstop_position()
    def test_a_probe_z_offset_larger_than_the_pivot_is_rejected(self):
        with self.assertRaises(ConfigError):
            build(z_offset=-100.)


class TestCommands(unittest.TestCase):
    def setUp(self):
        self.printer, self.rp = build()
        self.gcode = self.printer.lookup_object('gcode')
    def test_orient_lifts_before_turning_b(self):
        self.rp.toolhead.position[2] = 1.
        gcmd = FakeGCmd({'MODE': 'PROBE'})
        self.gcode.commands['RTCP_PROBE_ORIENT'](gcmd)
        lift, turn = self.rp.toolhead.moves[-2:]
        self.assertEqual(lift[0][2], self.rp.orient_lift_z)
        self.assertEqual(turn[0][rtcp_probe_mod.B_POS_INDEX],
                         self.rp.probe_b_position)
    def test_orient_tool_faces_the_nozzle_at_the_bed(self):
        gcmd = FakeGCmd({'MODE': 'TOOL'})
        self.gcode.commands['RTCP_PROBE_ORIENT'](gcmd)
        self.assertAlmostEqual(
            self.rp.toolhead.moves[-1][0][rtcp_probe_mod.B_POS_INDEX],
            self.rp.probe_b_position + PROBE_B_OFFSET, places=9)
    def test_probe_move_puts_the_probe_over_the_bed_point(self):
        gcmd = FakeGCmd({'X': 40., 'Y': 0.})
        self.gcode.commands['RTCP_PROBE_MOVE'](gcmd)
        pos = self.rp.toolhead.get_position()
        res = self.rp.create_probe_result([pos[0], pos[1], 0.])
        self.assertAlmostEqual(res.bed_x, 40., places=9)
        self.assertAlmostEqual(res.bed_y, 0., places=9)
    def test_orient_needs_b_homed(self):
        # B's homed flag is on the rotary axis object - it never shows up
        # in toolhead homed_axes, so reading it from there always failed
        self.rp.b_axis.is_homed = False
        with self.assertRaises(ConfigError) as cm:
            self.gcode.commands['RTCP_PROBE_ORIENT'](FakeGCmd({}))
        self.assertIn("Must home B", str(cm.exception))
    def test_orient_works_with_b_homed_but_not_in_homed_axes(self):
        self.assertNotIn('b', self.rp.toolhead.homed)
        self.gcode.commands['RTCP_PROBE_ORIENT'](FakeGCmd({}))
        self.assertAlmostEqual(
            self.rp.toolhead.get_position()[rtcp_probe_mod.B_POS_INDEX],
            self.rp.probe_b_position, places=9)
    def test_status(self):
        status = self.rp.get_status()
        self.assertTrue(status['oriented'])
        self.assertAlmostEqual(status['probe_b_position'],
                               PROBE_B_POSITION, places=9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
