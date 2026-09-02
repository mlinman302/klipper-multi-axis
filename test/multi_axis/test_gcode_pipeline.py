#!/usr/bin/env python
# Host test of the multi-axis (A/B/C) g-code pipeline.
#
# Copyright (C) 2025  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* gcode.py, gcode_move.py, toolhead.Move,
# toolhead.LookAheadQueue and kinematics/rotary_axis.py code with stubbed
# hardware, so it runs anywhere Python + cffi are available (it does not
# need a compiled c_helper.so, a serial port, or a Linux host).
#
# Run with:  python test/multi_axis/test_gcode_pipeline.py
#
# The C side of the six-axis motion space is covered separately by
# test/multi_axis/run_c_tests.sh, and the full firmware regression test
# (Linux only) is test/klippy/multi_axis.test.
import math, os, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# toolhead.py imports mcu, which needs pyserial; stub it out since none
# of the code under test touches the mcu.
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

import gcode as gcode_mod
import stepper as stepper_mod
import toolhead as toolhead_mod
from extras import gcode_move as gcode_move_mod
from kinematics import rotary_axis as rotary_axis_mod
from extras import rtcp as rtcp_mod
from extras import b_projection as bproject_mod


######################################################################
# Stubbed printer environment
######################################################################

class FakeMutex:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        return False

class FakeReactor:
    def mutex(self):
        return FakeMutex()

class FakePrinter:
    config_error = gcode_mod.CommandError
    command_error = gcode_mod.CommandError
    def __init__(self):
        self.objects = {}
        self.event_handlers = {}
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, event, cb):
        self.event_handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.event_handlers.get(event, [])]
    def invoke_shutdown(self, msg):
        raise AssertionError("invoke_shutdown: %s" % (msg,))
    def is_shutdown(self):
        return False
    def request_exit(self, result):
        pass

class FakeConfig:
    def __init__(self, printer):
        self.printer = printer
    def get_printer(self):
        return self.printer


class FakeExtruder:
    # Mirrors kinematics.extruder.DummyExtruder's extra-axis interface
    def get_name(self):
        return "extruder"
    def get_axis_gcode_id(self):
        return 'E'
    def check_move(self, move, ea_index):
        pass
    def calc_junction(self, prev_move, move, ea_index):
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        pass
    def find_past_position(self, print_time):
        return 0.


def make_rotary_axis(printer, axis_letter):
    # Build a RotaryAxis without touching config/stepper/mcu code, then
    # exercise its real motion methods.
    ra = rotary_axis_mod.RotaryAxis.__new__(rotary_axis_mod.RotaryAxis)
    ra.printer = printer
    ra.axis_letter = axis_letter
    ra.gcode_id, ra.rotates_about = rotary_axis_mod.ROTARY_AXES[axis_letter]
    ra.name = 'rotary_axis %s' % (axis_letter,)
    ra.max_velocity = None   # unset by default, as in config
    ra.max_accel = None
    ra.instant_corner_v = None
    ra.can_home = False
    ra.pos_min = -360.
    ra.pos_max = 360.
    ra.commanded_pos = 0.
    ra.is_homed = True
    ra.trapq = 'toolhead_trapq'
    ra.steppers = []
    return ra


class FakeKinematics:
    def __init__(self):
        self.checked = []
        self.status = {}
    def check_move(self, move):
        self.checked.append(move)
    def set_position(self, newpos, homing_axes):
        pass
    def get_status(self, eventtime=None):
        return self.status


class FakeToolHead:
    # A minimal stand-in for toolhead.ToolHead that keeps the parts under
    # test real: Move, LookAheadQueue and the extra-axis dispatch.
    def __init__(self, printer, rotary_letters=('a',)):
        self.printer = printer
        self.max_velocity = 300.
        self.max_accel = 3000.
        self.min_cruise_ratio = 0.5
        self.square_corner_velocity = 5.
        self.junction_deviation = self.mcr_pseudo_accel = 0.
        self._calc_junction_deviation()
        self.Coord = gcode_mod.Coord
        self.kin = FakeKinematics()
        self.lookahead = toolhead_mod.LookAheadQueue()
        self.print_time = 0.
        self.trapq_log = []   # (print_time, start_pos6, axes_r6) per move
        self.commanded_pos = [0.] * toolhead_mod.BASE_POS_LEN
        # extra_axes[i] <-> position index i+3, with fixed rotary slots
        self.extra_axes = [FakeExtruder()] + [
            rotary_axis_mod.DummyRotaryAxis(printer, letter)
            for letter in toolhead_mod.ROTARY_LETTERS]
        self.rotary_axes = {}
        for letter in rotary_letters:
            ra = make_rotary_axis(printer, letter)
            self.rotary_axes[ra.gcode_id] = ra
            index = toolhead_mod.ROTARY_LETTERS.index(letter)
            self.extra_axes[1 + index] = ra
        self.extra_axes_status = {}
        self._build_extra_axes_status()
        self.move_checks = []
    def register_move_check(self, callback):
        self.move_checks.append(callback)
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.mcr_pseudo_accel = self.max_accel * (1. - self.min_cruise_ratio)
    def _build_extra_axes_status(self):
        enames = [ea.get_name() for ea in self.extra_axes]
        self.extra_axes_status = {n: e_index + 3
                                  for e_index, n in enumerate(enames) if n}
    def get_position(self):
        return list(self.commanded_pos)
    def set_position(self, newpos, homing_axes=""):
        self.commanded_pos[:] = list(newpos)
    def get_extra_axes(self):
        return [None, None, None] + self.extra_axes
    def get_rotary_axes(self):
        return [(i, self.extra_axes[i - 3]) for i in toolhead_mod.ROTARY_POS
                if self.extra_axes[i - 3].get_axis_gcode_id() is not None]
    def get_kinematics(self):
        return self.kin
    def move(self, newpos, speed):
        move = toolhead_mod.Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.needs_trapq:
            for check in self.move_checks:
                check(move)
        for e_index, ea in enumerate(self.extra_axes):
            if move.axes_d[e_index + 3]:
                ea.check_move(move, e_index + 3)
        self.commanded_pos[:] = move.end_pos
        self.lookahead.add_move(move)
    def flush(self):
        # Mirrors ToolHead._process_lookahead()
        moves = self.lookahead.flush()
        next_move_time = self.print_time
        for move in moves:
            if move.needs_trapq:
                sp, ar = move.start_pos, move.axes_r
                self.trapq_log.append(
                    (next_move_time,
                     (sp[0], sp[1], sp[2], sp[4], sp[5], sp[6]),
                     (ar[0], ar[1], ar[2], ar[4], ar[5], ar[6]),
                     move))
            for e_index, ea in enumerate(self.extra_axes):
                if move.axes_d[e_index + 3]:
                    ea.process_move(next_move_time, move, e_index + 3)
            next_move_time += move.accel_t + move.cruise_t + move.decel_t
        self.print_time = next_move_time
        return moves


def build_env(rotary_letters=('a',)):
    printer = FakePrinter()
    gcode = gcode_mod.GCodeDispatch(printer)
    printer.add_object('gcode', gcode)
    toolhead = FakeToolHead(printer, rotary_letters)
    printer.add_object('toolhead', toolhead)
    gmove = gcode_move_mod.GCodeMove(FakeConfig(printer))
    printer.add_object('gcode_move', gmove)
    responses = []
    gcode.register_output_handler(responses.append)
    printer.send_event("klippy:ready")
    gcode._handle_ready()
    gmove._update_extra_axes()
    return printer, gcode, gmove, toolhead, responses


######################################################################
# Tests
######################################################################

class TestAdditionalAxesParsing(unittest.TestCase):
    class Cfg:
        error = gcode_mod.CommandError
        def __init__(self, value):
            self.value = value
        def get(self, name, default=None):
            return self.value
        def get_name(self):
            return 'printer'
    def parse(self, value):
        return rotary_axis_mod.parse_additional_axes(self.Cfg(value))
    def test_empty(self):
        self.assertEqual(self.parse(''), [])
    def test_separators(self):
        for text in ['abc', 'a b c', 'a,b,c', 'A, B, C', 'a;b;c', ' a b,c ']:
            self.assertEqual(self.parse(text), ['a', 'b', 'c'], text)
    def test_subset_and_order(self):
        self.assertEqual(self.parse('c,a'), ['c', 'a'])
    def test_invalid_letter(self):
        self.assertRaises(gcode_mod.CommandError, self.parse, 'a,d')
    def test_duplicate(self):
        self.assertRaises(gcode_mod.CommandError, self.parse, 'aa')


class TestPositionLayout(unittest.TestCase):
    def test_rotary_slots_are_fixed(self):
        # A/B/C always occupy indexes 4/5/6 whether or not configured, so
        # that the kinematic gather in stepper.kin_coords is constant
        printer, gcode, gmove, th, resp = build_env(('b',))
        self.assertEqual(len(th.commanded_pos), 7)
        self.assertEqual(gmove.axis_map, {'X': 0, 'Y': 1, 'Z': 2, 'E': 3,
                                          'B': 5})
        self.assertEqual(th.extra_axes_status,
                         {'extruder': 3, 'rotary_axis b': 5})

    def test_kin_coords_gather(self):
        # [x, y, z, e, a, b, c] -> [x, y, z, a, b, c]
        pos = [1., 2., 3., 4., 5., 6., 7.]
        self.assertEqual(stepper_mod.kin_coords(pos), [1., 2., 3., 5., 6., 7.])
        # Short single-dimension vectors gather to zeros
        self.assertEqual(stepper_mod.kin_coords([9., 0., 0.]),
                         [9., 0., 0., 0., 0., 0.])
        self.assertEqual(stepper_mod.kin_coords([9., 0., 0., 0.]),
                         [9., 0., 0., 0., 0., 0.])

    def test_unconfigured_axis_has_no_gcode_word(self):
        printer, gcode, gmove, th, resp = build_env(('a',))
        self.assertNotIn('B', gmove.axis_map)
        self.assertNotIn('C', gmove.axis_map)

    def test_unconfigured_axis_rejects_motion(self):
        printer, gcode, gmove, th, resp = build_env(('a',))
        dummy = th.extra_axes[2]   # the 'b' placeholder
        move = toolhead_mod.Move(th, [0.] * 7, [1., 0., 0., 0., 0., 5., 0.],
                                 100.)
        self.assertRaises(gcode_mod.CommandError, dummy.check_move, move, 5)


class TestGCodeParsing(unittest.TestCase):
    def test_axis_map_includes_rotary(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'b', 'c'))
        self.assertEqual(gmove.axis_map,
                         {'X': 0, 'Y': 1, 'Z': 2, 'E': 3,
                          'A': 4, 'B': 5, 'C': 6})

    def test_g1_sets_rotary_position(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'b'))
        gcode.run_script("G1 X10 Y5 A45 B-30 F1200")
        self.assertAlmostEqual(gmove.last_position[4], 45.)
        self.assertAlmostEqual(gmove.last_position[5], -30.)
        self.assertAlmostEqual(th.commanded_pos[4], 45.)
        self.assertAlmostEqual(th.commanded_pos[5], -30.)

    def test_rotary_does_not_change_move_d(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 X30 Y40 A90 F1200")
        move = th.kin.checked[-1]
        self.assertAlmostEqual(move.move_d, 50.)     # sqrt(30^2 + 40^2)
        self.assertAlmostEqual(move.axes_d[4], 90.)
        self.assertAlmostEqual(move.axes_r[4], 90. / 50.)

    def test_relative_and_absolute(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 A45\nG91\nG1 A10\nG90\nG1 A5")
        self.assertAlmostEqual(gmove.last_position[4], 5.)

    def test_g92_sets_rotary_origin(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 A90\nG92 A0")
        self.assertAlmostEqual(gmove.base_position[4], 90.)
        gcode.run_script("G1 A10")
        self.assertAlmostEqual(gmove.last_position[4], 100.)

    def test_m114_reports_rotary(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'c'))
        gcode.run_script("G1 X1 A45 C90")
        del resp[:]
        gcode.run_script("M114")
        self.assertIn("A:45.000", resp[-1])
        self.assertIn("C:90.000", resp[-1])

    def test_set_gcode_offset_rotary(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("SET_GCODE_OFFSET A=5")
        self.assertAlmostEqual(gmove.homing_position[4], 5.)
        gcode.run_script("G1 A10")
        self.assertAlmostEqual(gmove.last_position[4], 15.)

    def test_save_restore_gcode_state(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 A45\nSAVE_GCODE_STATE NAME=t\nG1 A90")
        gcode.run_script("RESTORE_GCODE_STATE NAME=t MOVE=1")
        self.assertAlmostEqual(gmove.last_position[4], 45.)


class TestSharedMotionQueue(unittest.TestCase):
    # The point of stage 2: rotation rides in the toolhead trapq
    def test_rotation_reaches_the_toolhead_trapq(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'c'))
        gcode.run_script("G1 X100 A50 C-20 F600")
        th.flush()
        self.assertEqual(len(th.trapq_log), 1)
        print_time, start_pos, axes_r, move = th.trapq_log[0]
        # start_pos/axes_r are the six kinematic components in one queue
        self.assertEqual(len(start_pos), 6)
        self.assertEqual(len(axes_r), 6)
        self.assertAlmostEqual(axes_r[0], move.axes_r[0])   # x
        self.assertAlmostEqual(axes_r[3], move.axes_r[4])   # a
        self.assertAlmostEqual(axes_r[5], move.axes_r[6])   # c
        # The rotational ratios are relative to the same move distance as
        # the linear ones, which is what puts them on a common time base
        self.assertAlmostEqual(axes_r[3] * move.move_d, 50.)
        self.assertAlmostEqual(axes_r[5] * move.move_d, -20.)

    def test_rotation_only_move_still_reaches_the_trapq(self):
        # Before the queues were shared this move was dropped entirely:
        # is_kinematic_move is False when x/y/z do not move
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 A180 F600")
        move = th.lookahead.queue[-1]
        self.assertFalse(move.is_kinematic_move)
        self.assertTrue(move.needs_trapq)
        th.flush()
        self.assertEqual(len(th.trapq_log), 1)
        print_time, start_pos, axes_r, m = th.trapq_log[0]
        self.assertAlmostEqual(axes_r[3] * m.move_d, 180.)

    def test_extrude_only_move_does_not_set_needs_trapq_via_rotation(self):
        printer, gcode, gmove, th, resp = build_env()
        gcode.run_script("G1 E5 F600")
        move = th.lookahead.queue[-1]
        self.assertFalse(move.is_kinematic_move)
        self.assertFalse(move.needs_trapq)
        th.flush()
        self.assertEqual(th.trapq_log, [])

    def test_rotary_process_move_does_not_queue_separately(self):
        # The axis object no longer owns a queue - the toolhead already
        # placed the rotation into the shared one
        printer, gcode, gmove, th, resp = build_env()
        ra = th.rotary_axes['A']
        self.assertEqual(ra.get_trapq(), 'toolhead_trapq')
        gcode.run_script("G1 X10 A45 F600")
        th.flush()
        self.assertAlmostEqual(ra.commanded_pos, 45.)
        self.assertEqual(len(th.trapq_log), 1)


class TestRotaryLimits(unittest.TestCase):
    def test_axis_velocity_limit_slows_move(self):
        printer, gcode, gmove, th, resp = build_env()
        ra = th.rotary_axes['A']
        ra.max_velocity = 10.       # deg/s
        ra.max_accel = 1000.        # deg/s^2
        # 10mm of travel with 100 degrees of rotation: the axis needs
        # 10 seconds, so the move is limited to 1 mm/s
        gcode.run_script("G1 X10 A100 F18000")
        move = th.kin.checked[-1]
        self.assertAlmostEqual(math.sqrt(move.max_cruise_v2), 1.0, places=9)

    def test_out_of_range_rejected(self):
        printer, gcode, gmove, th, resp = build_env()
        th.rotary_axes['A'].pos_max = 90.
        del resp[:]
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, "G1 X1 A120")
        self.assertTrue(any('out of range' in r for r in resp), resp)

    def test_unhomed_axis_rejected(self):
        printer, gcode, gmove, th, resp = build_env()
        th.rotary_axes['A'].is_homed = False
        del resp[:]
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, "G1 X1 A10")
        self.assertTrue(any('Must home rotary axis A' in r for r in resp), resp)

    def test_calc_junction_default_is_no_limit(self):
        printer, gcode, gmove, th, resp = build_env()
        ra = th.rotary_axes['A']
        gcode.run_script("G1 X10 A0 F600\nG1 X20 A90 F600")
        moves = th.lookahead.queue
        self.assertAlmostEqual(ra.calc_junction(moves[0], moves[1], 4),
                               moves[1].max_cruise_v2)
        ra.instant_corner_v = 1.
        v2 = ra.calc_junction(moves[0], moves[1], 4)
        diff_r = moves[1].axes_r[4] - moves[0].axes_r[4]
        self.assertAlmostEqual(v2, (ra.instant_corner_v / abs(diff_r))**2)

    def test_status(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'b', 'c'))
        for gid, about in (('A', 'x'), ('B', 'y'), ('C', 'z')):
            sts = th.rotary_axes[gid].get_status(0.)
            self.assertEqual(sts['gcode_axis'], gid)
            self.assertEqual(sts['rotates_about'], about)


def make_rtcp(printer, toolhead, tool_v=40., tool_h=0.,
              frame=rtcp_mod.FRAME_CARTESIAN):
    # Build an RTCP object without config/chelper, then exercise its real
    # transform and range-check logic
    r = rtcp_mod.RTCP.__new__(rtcp_mod.RTCP)
    r.printer = printer
    r.toolhead = toolhead
    r.tool_v = tool_v
    r.tool_h = tool_h
    r.frame = frame
    r.enabled = True
    r.orig_stepper_kinematics = []
    r.rtcp_stepper_kinematics = {}
    return r


class TestRTCP(unittest.TestCase):
    # The B position index must be 5 for the transforms to read the right
    # component out of a toolhead position vector
    def test_b_index(self):
        self.assertEqual(rtcp_mod.B_POS_INDEX, 5)

    def _pos(self, x=0., y=0., z=0., b=0.):
        return [x, y, z, 0., 0., b, 0.]

    def test_identity_at_zero_tilt(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th)
        m = r.tool_to_machine(self._pos(10., 20., 30., 0.))
        self.assertAlmostEqual(m[0], 10.)
        self.assertAlmostEqual(m[1], 20.)
        self.assertAlmostEqual(m[2], 30.)

    def test_ninety_degrees(self):
        # A positive B tilts the nozzle outboard, so at B=90 the tool
        # points along +X from the pivot and the carriage must retreat by
        # L in X and drop by L in Z to hold the tip.  Same numbers as the
        # kin_rtcp.c check in test_kin_6axis.c - the two implementations
        # have to agree.
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.)
        m = r.tool_to_machine(self._pos(10., 5., 30., 90.))
        self.assertAlmostEqual(m[0], -30.)
        self.assertAlmostEqual(m[1], 5.)     # Y is untouched
        self.assertAlmostEqual(m[2], -10.)

    def test_radial_frame_moves_the_arm_radius_not_the_bed_angle(self):
        # On a polar machine the tip swings along the arm, so the
        # correction scales x and y together
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.,
                      frame=rtcp_mod.FRAME_RADIAL)
        m = r.tool_to_machine(self._pos(0., 50., 0., 90.))
        self.assertAlmostEqual(m[0], 0.)
        self.assertAlmostEqual(m[1], 10.)
        self.assertAlmostEqual(m[2], -40.)

    def test_round_trip(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        for frame in (rtcp_mod.FRAME_CARTESIAN, rtcp_mod.FRAME_RADIAL):
            r = make_rtcp(printer, th, tool_v=37.5, tool_h=2.5,
                          frame=frame)
            for b in (-90., -33.3, 0., 12.7, 45., 180.):
                # Far enough out that the radial correction never drives
                # the arm radius through zero, which is not invertible -
                # _check_move rejects those moves instead
                p = self._pos(110., 22., 33., b)
                back = r.machine_to_tool(r.tool_to_machine(p))
                for i in range(3):
                    self.assertAlmostEqual(back[i], p[i], places=9)

    def test_a_radius_through_the_centre_is_rejected(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.,
                      frame=rtcp_mod.FRAME_RADIAL)
        th.kin.status = {'axis_minimum': [-200., -200., -200.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        # At B=90 the carriage must sit 40mm inboard of the tip, which
        # from a radius of 10 is on the far side of the bed
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, "G1 X10 Y0 B90 F600")

    def test_disabled_is_identity(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th)
        r.enabled = False
        p = self._pos(10., 20., 30., 90.)
        self.assertEqual(r.tool_to_machine(p), list(p))
        self.assertEqual(r.machine_to_tool(p), list(p))

    def test_check_move_rejects_unreachable_machine_position(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.)
        th.kin.status = {'axis_minimum': [0., 0., 0.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        # Tip at X=10 is fine on its own, but tilting to B=90 puts the
        # carriage at -30, past the end of the rail
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, "G1 X10 B90 F600")

    def test_check_move_allows_reachable(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.)
        th.kin.status = {'axis_minimum': [-100., 0., -100.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        gcode.run_script("G1 X100 Z50 B90 F600")
        self.assertAlmostEqual(th.commanded_pos[5], 90.)

    def test_check_move_catches_an_interior_radius_dip(self):
        # A chord that does not pass through the bed centre has its
        # smallest arm radius in the middle, not at either end: from
        # (40, -40) to (40, 40) the radius is 56.6 at both ends and dips
        # to 40 between them.  With the tip 50 mm below the pivot and the
        # head at B90 the correction is -50, so the ends clear the centre
        # by 6.6 mm while the middle is 10 mm through it.  An
        # endpoint-only check waved this through.
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=50.,
                      frame=rtcp_mod.FRAME_RADIAL)
        th.kin.status = {'axis_minimum': [-200., -200., -200.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        gcode.run_script('G1 X40 Y-40 B90 F600')
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, 'G1 X40 Y40 F600')
        # ...while the same chord kept clear of the centre is fine
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=50.,
                      frame=rtcp_mod.FRAME_RADIAL)
        th.kin.status = {'axis_minimum': [-200., -200., -200.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        gcode.run_script('G1 X80 Y-40 B90 F600')
        gcode.run_script('G1 X80 Y40 F600')

    def test_rotation_only_move_is_range_checked(self):
        # A B-only move still moves the carriages under RTCP, so it must
        # go through the machine-position check
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.)
        th.kin.status = {'axis_minimum': [0., 0., 0.],
                         'axis_maximum': [200., 200., 200.]}
        th.register_move_check(r._check_move)
        self.assertRaises(gcode_mod.CommandError,
                          gcode.run_script, "G1 B90 F600")


class TestMultipleMoves(unittest.TestCase):
    def test_sequence_reaches_final_angles(self):
        printer, gcode, gmove, th, resp = build_env(('a', 'b'))
        gcode.run_script("G1 X10 Y10 A30 B15 F1200\n"
                         "G1 X20 Y10 A60 B15 F1200\n"
                         "G1 X20 Y20 A60 B45 F1200")
        th.flush()
        self.assertAlmostEqual(th.commanded_pos[4], 60.)
        self.assertAlmostEqual(th.commanded_pos[5], 45.)
        self.assertEqual(len(th.trapq_log), 3)

    def test_xyz_timing_unaffected_by_rotation(self):
        # Stage-1 simplification still holds: the same XYZ program takes
        # the same time with and without rotation
        def run(script):
            printer, gcode, gmove, th, resp = build_env()
            gcode.run_script(script)
            th.flush()
            return th.print_time
        t_plain = run("G1 X10 F600\nG1 X20 F600\nG1 X30 F600")
        t_rot = run("G1 X10 A20 F600\nG1 X20 A40 F600\nG1 X30 A60 F600")
        self.assertAlmostEqual(t_plain, t_rot, places=9)



######################################################################
# corertheta check_move
######################################################################

class _CheckMoveError(Exception):
    pass


class _FakeMove:
    def __init__(self, axes_d, end_pos, move_d=1.):
        self.axes_d = axes_d
        self.end_pos = end_pos
        self.start_pos = [0.] * len(end_pos)
        self.move_d = move_d
        self.max_cruise_v2 = 100.
        self.limited = None
    def move_error(self, msg="Move out of range"):
        return _CheckMoveError(msg)
    def limit_speed(self, v, a):
        self.limited = (v, a)


class TestCoreRThetaCheckMove(unittest.TestCase):
    # check_move must range check each axis only on moves that actually
    # touch it.  Two regressions live here: rejecting z/b-only moves while
    # x/y are unhomed (which made it impossible to home anything before R),
    # and reading end_pos on a path where it had not been assigned.
    def _kin(self, limit_xy2=-1., limit_z=(0., 250.)):
        from kinematics import corertheta
        kin = object.__new__(corertheta.CoreRThetaKinematics)
        kin.limit_xy2 = limit_xy2
        kin.limit_z = limit_z
        kin.v_rad_max = 0.
        kin.max_velocity = kin.max_accel = 300.
        kin.max_z_velocity = 5.
        kin.max_z_accel = 100.
        return kin
    def test_z_only_move_allowed_while_xy_unhomed(self):
        kin = self._kin()
        m = _FakeMove([0., 0., 5.], [0., 0., 10.], move_d=5.)
        kin.check_move(m)          # must not raise (NameError regression)
        self.assertIsNotNone(m.limited)
    def test_b_only_move_allowed_while_xy_unhomed(self):
        kin = self._kin()
        m = _FakeMove([0., 0., 0.], [0., 0., 0.])
        kin.check_move(m)          # must not raise
    def test_xy_move_rejected_while_unhomed(self):
        kin = self._kin()
        m = _FakeMove([10., 0., 0.], [10., 0., 0.], move_d=10.)
        with self.assertRaises(_CheckMoveError):
            kin.check_move(m)
    def test_z_move_rejected_while_z_unhomed(self):
        kin = self._kin(limit_xy2=40000., limit_z=(1., -1.))
        m = _FakeMove([0., 0., 5.], [0., 0., 10.], move_d=5.)
        with self.assertRaises(_CheckMoveError):
            kin.check_move(m)
    def test_xy_move_allowed_within_radius(self):
        kin = self._kin(limit_xy2=40000.)
        m = _FakeMove([10., 0., 0.], [10., 0., 0.], move_d=10.)
        kin.check_move(m)


######################################################################
# corertheta R homing sweep
######################################################################

class _FakeHomingInfo:
    def __init__(self, position_endstop, positive_dir=True):
        self.position_endstop = position_endstop
        self.positive_dir = positive_dir
        self.speed = 40.
        self.retract_dist = 0.
        self.retract_speed = 40.
        self.second_homing_speed = 20.


class _FakeRail:
    def __init__(self, rng, hi):
        self._range = rng
        self._hi = hi
    def get_range(self):
        return self._range
    def get_homing_info(self):
        return self._hi


class _FakeToolhead:
    def get_position(self):
        return [0.] * 7


class _FakeHomingState:
    def __init__(self):
        self.calls = []
    def home_rails(self, rails, forcepos, homepos):
        self.calls.append((rails, forcepos, homepos))


class _FakePrinter:
    class config_error(Exception):
        pass
    def lookup_object(self, name):
        assert name == 'toolhead'
        return _FakeToolhead()


class TestCoreRThetaHomeR(unittest.TestCase):
    # R is the arm radius, so the homing sweep has to stay on one side of
    # the centre.  Sweeping through r == 0 is a half turn of the bed in
    # polar coordinates, commanded the instant the sign of x flips, which
    # shuts klippy down with "Internal error in stepcompress" on the
    # stepper_c queue - and it runs the arm inward before it turns around,
    # since the radius the gantry motors follow is |x|.
    def _kin(self):
        from kinematics import corertheta
        kin = object.__new__(corertheta.CoreRThetaKinematics)
        kin.printer = _FakePrinter()
        return kin
    def _forcepos(self, rng, position_endstop, positive_dir=True):
        kin = self._kin()
        hs = _FakeHomingState()
        rail = _FakeRail(rng, _FakeHomingInfo(position_endstop, positive_dir))
        kin._home_axis(hs, 0, rail)
        rails, forcepos, homepos = hs.calls[0]
        return forcepos, homepos
    def test_negative_position_min_does_not_cross_the_centre(self):
        # The reported machine: position_min -30, endstop/max 200.  The
        # sweep used to start at x = -30 and blow up 30mm in, as the
        # commanded x crossed zero.
        forcepos, homepos = self._forcepos((-30., 200.), 200.)
        self.assertEqual(forcepos[0], 0.)
        self.assertEqual(forcepos[1], 0.)
        self.assertEqual(homepos[0], 200.)
        self.assertEqual(homepos[1], 0.)
    def test_non_negative_position_min_is_untouched(self):
        forcepos, homepos = self._forcepos((20., 200.), 200.)
        self.assertEqual(forcepos[0], 20.)
        self.assertEqual(homepos[0], 200.)
    def test_position_min_of_zero_is_untouched(self):
        forcepos, homepos = self._forcepos((0., 200.), 200.)
        self.assertEqual(forcepos[0], 0.)
    def test_homing_toward_the_centre_also_stays_positive(self):
        # Endstop at the inner end: the sweep runs position_max -> endstop
        forcepos, homepos = self._forcepos((-30., 200.), 5.,
                                           positive_dir=False)
        self.assertGreaterEqual(forcepos[0], 0.)
        self.assertEqual(homepos[0], 5.)
    def test_negative_position_endstop_is_a_config_error(self):
        with self.assertRaises(_FakePrinter.config_error):
            self._forcepos((-30., 200.), -10.)
    def test_y_is_pinned_to_zero_for_the_whole_sweep(self):
        # The bed angle is atan2(y, x); a non-zero y would turn the bed
        forcepos, homepos = self._forcepos((-30., 200.), 200.)
        self.assertEqual(forcepos[1], homepos[1])


######################################################################
# corertheta calc_position
######################################################################

class _NamedRail:
    def __init__(self, name):
        self.name = name
    def get_name(self):
        return self.name


class TestCoreRThetaCalcPosition(unittest.TestCase):
    # calc_position has to invert the same formulas kin_corertheta.c
    # applies: stepper_r runs the '-' solver and stepper_tilt the '+' one.
    # When the two were swapped in setup_itersolve() without updating this
    # inverse, the radius came back negated - so the first thing that
    # recomputed the toolhead position from the steppers (homing Z, after
    # R was already homed) flipped the sign of X.
    B_RATIO = 2.5
    def _kin(self, b_coeff=None):
        from kinematics import corertheta
        kin = object.__new__(corertheta.CoreRThetaKinematics)
        kin.stepper_bed = _NamedRail('stepper_c')
        kin.rail_r = _NamedRail('stepper_r')
        kin.rail_b = _NamedRail('stepper_tilt')
        kin.rail_z = _NamedRail('stepper_z')
        kin.b_ratio = self.B_RATIO
        kin.b_coeff = self.B_RATIO if b_coeff is None else b_coeff
        return kin
    def _stepper_positions(self, x, y, z, b, b_coeff=None):
        # The forward kinematics of kin_corertheta.c
        if b_coeff is None:
            b_coeff = self.B_RATIO
        radius = math.sqrt(x*x + y*y)
        return {'stepper_c': math.atan2(y, x),
                'stepper_r': b_coeff * b - radius,
                'stepper_tilt': b_coeff * b + radius,
                'stepper_z': z}
    def _check(self, x, y, z, b):
        kin = self._kin()
        pos = kin.calc_position(self._stepper_positions(x, y, z, b))
        self.assertAlmostEqual(pos[0], x, places=9)
        self.assertAlmostEqual(pos[1], y, places=9)
        self.assertAlmostEqual(pos[2], z, places=9)
        self.assertAlmostEqual(pos[4], b, places=9)
    def test_round_trip_on_the_r_axis(self):
        # The position right after homing R: bed angle zero, arm out
        self._check(200., 0., 12.5, 0.)
    def test_round_trip_off_axis(self):
        self._check(-30., 45., 100., -17.)
    def test_round_trip_with_b_rotation(self):
        self._check(120., 0., 0., 30.)
    def test_inverted_b_round_trips_through_the_signed_coefficient(self):
        # invert_b_direction negates the b term of both gantry solvers;
        # calc_position has to divide by the same signed coefficient, or
        # B would read back inverted while X stayed correct.
        coeff = -self.B_RATIO
        kin = self._kin(b_coeff=coeff)
        sp = self._stepper_positions(120., 0., 0., 30., b_coeff=coeff)
        pos = kin.calc_position(sp)
        self.assertAlmostEqual(pos[0], 120., places=9)
        self.assertAlmostEqual(pos[4], 30., places=9)
    def test_inverting_b_leaves_x_alone(self):
        # The whole point of the option: it is the one degree of freedom
        # the dir_pins and the '+'/'-' solver assignment cannot express
        kin = self._kin(b_coeff=-self.B_RATIO)
        sp = self._stepper_positions(200., 0., 0., 0.)
        self.assertAlmostEqual(kin.calc_position(sp)[0], 200., places=9)
    def test_x_keeps_its_sign_when_z_is_homed_after_r(self):
        # Homing R leaves the toolhead at (200, 0); homing Z afterwards runs
        # calc_position over the unchanged gantry steppers, and must not
        # come back with x = -200.
        kin = self._kin()
        sp = self._stepper_positions(200., 0., 0., 0.)
        sp['stepper_z'] = 10.       # only z moved
        self.assertGreater(kin.calc_position(sp)[0], 0.)


######################################################################
# Bed-frame B projection
######################################################################

# Pure-Python mirror of bproject_project_b() in
# klippy/chelper/kin_bproject.c.  The C function is what the steppers
# actually run and is covered by test/multi_axis/run_c_tests.sh; this
# stands in for it here so the host-side logic built on top of it - the
# inverse and the speed limiting - can be tested without a compiled
# c_helper.so.
def _project_b(b, x, y, max_angle, taper_range):
    ab = abs(b)
    if max_angle <= 0. or ab >= max_angle + taper_range:
        return b
    r2 = x * x + y * y
    cos_t = x / math.sqrt(r2) if r2 >= 0.010**2 else 1.
    w = 1.
    if ab > max_angle:
        t = (ab - max_angle) / taper_range
        w = 1. - t * t * (3. - 2. * t)
    return b * (1. + w * (cos_t - 1.))


class FakeSK:
    # Stands in for a struct stepper_kinematics.  'kind' is which wrapper
    # allocated it (or None for the solver underneath), 'wrapped' is what
    # it sits on top of, and 'params' is whatever was last set on it.
    def __init__(self, kind, wrapped=None):
        self.kind = kind
        self.wrapped = wrapped
        self.params = None
    def chain(self):
        # Outermost first, down to the bare solver
        node, res = self, []
        while node is not None:
            res.append(node.kind)
            node = node.wrapped
        return res


class FakeFFILib:
    @staticmethod
    def bproject_project_b(b, x, y, max_angle, taper_range):
        return _project_b(b, x, y, max_angle, taper_range)
    @staticmethod
    def rtcp_alloc():
        return FakeSK('rtcp')
    @staticmethod
    def rtcp_set_sk(sk, orig_sk):
        sk.wrapped = orig_sk
        return 0
    @staticmethod
    def rtcp_set_tool(sk, tool_h, tool_v, frame):
        sk.params = (tool_h, tool_v, frame)
    @staticmethod
    def bproject_alloc():
        return FakeSK('bproject')
    @staticmethod
    def bproject_set_sk(sk, orig_sk):
        sk.wrapped = orig_sk
        return 0
    @staticmethod
    def bproject_set_params(sk, max_angle, taper_range):
        sk.params = (max_angle, taper_range)
    @staticmethod
    def free(sk):
        pass


class FakeFFIMain:
    @staticmethod
    def gc(obj, destructor):
        return obj


class FakeChelper:
    @staticmethod
    def get_ffi():
        return FakeFFIMain, FakeFFILib


class FakeWrappedStepper:
    # Just enough of an MCU_stepper for the wrapping code
    def __init__(self, name):
        self.name = name
        self.sk = FakeSK(None)
    def get_name(self):
        return self.name
    def get_trapq(self):
        return object()
    def get_stepper_kinematics(self):
        return self.sk
    def set_stepper_kinematics(self, sk):
        self.sk = sk


class FakeWrappedKin:
    def __init__(self, steppers):
        self.steppers = steppers
    def get_steppers(self):
        return list(self.steppers)


class FakeWrappingToolhead:
    def __init__(self, kin):
        self.kin = kin
    def get_kinematics(self):
        return self.kin
    def flush_step_generation(self):
        pass


class FakeMotionQueuing:
    def check_step_generation_scan_windows(self):
        pass


class TestStepperWrapping(unittest.TestCase):
    # [rtcp] and [b_projection] both wrap each kinematic stepper, rtcp
    # innermost.  Retuning either one must retune the wrapper it already
    # installed rather than adding another.
    def setUp(self):
        self._real = (rtcp_mod.chelper, bproject_mod.chelper)
        rtcp_mod.chelper = bproject_mod.chelper = FakeChelper

    def tearDown(self):
        rtcp_mod.chelper, bproject_mod.chelper = self._real

    def _build(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        printer.add_object('motion_queuing', FakeMotionQueuing())
        steppers = [FakeWrappedStepper('stepper_x'),
                    FakeWrappedStepper('stepper_z')]
        wth = FakeWrappingToolhead(FakeWrappedKin(steppers))
        r = make_rtcp(printer, wth, tool_v=40.)
        bp = make_bprojection(printer, wth)
        # Connect order: [rtcp] wraps first, [b_projection] outside it
        r._update_kinematics()
        bp._update_kinematics()
        return r, bp, steppers

    def test_connect_order_puts_rtcp_inside_the_projection(self):
        r, bp, steppers = self._build()
        for s in steppers:
            self.assertEqual(s.get_stepper_kinematics().chain(),
                             ['bproject', 'rtcp', None])

    def test_set_rtcp_retunes_in_place_instead_of_re_wrapping(self):
        # The bug this pins: with [b_projection] loaded, the stepper's
        # outermost kinematic is the projection, so an RTCP that
        # recognised only its own wrapper would wrap a second time.  The
        # new outer copy took the "disabled" zero tool while the inner
        # one kept the real offsets, so SET_RTCP ENABLE=0 reported the
        # compensation off and the steppers went on applying it.
        r, bp, steppers = self._build()
        r.enabled = False
        r._update_kinematics()
        for s in steppers:
            self.assertEqual(s.get_stepper_kinematics().chain(),
                             ['bproject', 'rtcp', None])
            rtcp_sk = s.get_stepper_kinematics().wrapped
            self.assertEqual(rtcp_sk.params[:2], (0., 0.))
        # ...and back on
        r.enabled = True
        r._update_kinematics()
        for s in steppers:
            self.assertEqual(s.get_stepper_kinematics().chain(),
                             ['bproject', 'rtcp', None])
            self.assertEqual(
                s.get_stepper_kinematics().wrapped.params[:2], (0., 40.))

    def test_repeated_toggling_adds_no_layers(self):
        r, bp, steppers = self._build()
        for _ in range(5):
            r.enabled = not r.enabled
            r._update_kinematics()
            bp._update_kinematics()
        for s in steppers:
            self.assertEqual(s.get_stepper_kinematics().chain(),
                             ['bproject', 'rtcp', None])


def make_bprojection(printer, toolhead, max_b_velocity=60.,
                     config_enabled=True):
    # Build the object without config or chelper, then exercise its real
    # transform, inverse and move-check logic
    bp = bproject_mod.BAxisProjection.__new__(bproject_mod.BAxisProjection)
    bp.printer = printer
    bp.toolhead = toolhead
    bp.config_enabled = config_enabled
    bp.enabled = config_enabled
    bp.max_b_velocity = max_b_velocity
    bp.max_b_accel = None
    bp.orig_stepper_kinematics = []
    bp.bproject_stepper_kinematics = {}
    return bp


class TestBProjection(unittest.TestCase):
    def setUp(self):
        self._real_chelper = bproject_mod.chelper
        bproject_mod.chelper = FakeChelper

    def tearDown(self):
        bproject_mod.chelper = self._real_chelper

    def _pos(self, x=0., y=0., b=0.):
        return [x, y, 0., 0., 0., b, 0.]

    def test_b_index(self):
        self.assertEqual(bproject_mod.B_POS_INDEX, 5)

    def test_sweep_over_a_bed_turn(self):
        # A held B of 10 sweeps the machine over 10 -> 0 -> -10 -> 0
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        for (x, y), want in (((100., 0.), 10.), ((0., 100.), 0.),
                             ((-100., 0.), -10.), ((0., -100.), 0.)):
            self.assertAlmostEqual(bp.project_pos(self._pos(x, y, 10.)),
                                   want, places=9)
        s = 100. / math.sqrt(2.)
        self.assertAlmostEqual(bp.project_pos(self._pos(s, s, 10.)),
                               10. / math.sqrt(2.), places=9)

    def test_scaling_is_uniform_over_the_whole_b_range(self):
        # The bug this pins.  There used to be a pass-through band above
        # max_angle, so that orientation angles reached the machine
        # untouched.  Everything the projection held back below the
        # threshold had to be paid out inside the taper above it: at a
        # bed angle of 76 degrees, B40 -> B50 became 40.6 degrees of head
        # travel for a 10 degree command.  The ratio must not depend on B.
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        x, y = 1.85, 7.65      # the position it was reported from
        cos_t = x / math.hypot(x, y)
        for b in (5., 10., 25., 39.9, 40., 40.1, 45., 50., 90.):
            for sign in (1., -1.):
                self.assertAlmostEqual(
                    bp.project_pos(self._pos(x, y, sign * b)),
                    sign * b * cos_t, places=9)

    def test_no_gain_cliff_anywhere_in_the_b_range(self):
        # Equivalently: d(machine B)/d(commanded B) is cos(theta)
        # everywhere, so no small g-code move can become a large one
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        x, y = 1.85, 7.65
        cos_t = x / math.hypot(x, y)
        b, step = -90., 0.25
        prev = bp.project_pos(self._pos(x, y, b))
        while b < 90.:
            b += step
            cur = bp.project_pos(self._pos(x, y, b))
            self.assertAlmostEqual((cur - prev) / step, cos_t, places=9)
            prev = cur

    def test_disabled_is_identity(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        bp.enabled = False
        p = self._pos(0., 100., 10.)
        self.assertEqual(bp.project_pos(p), 10.)
        self.assertEqual(bp.machine_to_commanded(p), list(p))

    def test_inverse_round_trips(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        for b in (-90., -45., -37.5, -10., 0., 10., 25., 41.5, 44., 45., 60.):
            for deg in range(0, 360, 17):
                th_rad = math.radians(deg)
                x, y = 60. * math.cos(th_rad), 60. * math.sin(th_rad)
                if abs(math.cos(th_rad)) < 0.2:
                    continue    # not invertible - covered below
                machine = self._pos(x, y, bp.project_pos(self._pos(x, y, b)))
                back = bp.machine_to_commanded(machine, self._pos(x, y, b))
                self.assertAlmostEqual(back[5], b, places=6)

    def test_inverse_falls_back_where_not_invertible(self):
        # At a bed angle of 90 every commanded B flattens to the same
        # machine B, so the B the toolhead already has is kept
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        machine = self._pos(0., 100., bp.project_pos(self._pos(0., 100., 25.)))
        back = bp.machine_to_commanded(machine, self._pos(0., 100., 25.))
        self.assertAlmostEqual(back[5], 25., places=1)

    def test_held_b_through_a_bed_move_is_speed_limited(self):
        # B does not change, but the bed swings a quarter turn under it,
        # so the machine's B runs from 10 to 0 - the move has to be slowed
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th, max_b_velocity=1.)
        th.register_move_check(bp._check_move)
        gcode.run_script("G1 X60 B10 F600")
        gcode.run_script("G1 X0 Y60 F600")
        move = th.lookahead.queue[-1]
        # ~10 degrees of machine B over the move, at 1 deg/s
        self.assertLess(math.sqrt(move.max_cruise_v2), move.move_d / 9.)

    def test_chord_off_the_bed_centre_is_speed_limited(self):
        # The bug this pins.  A chord that does not pass through the bed
        # centre has the same bed angle cosine at both ends, so an
        # endpoint-only check saw no machine B travel at all and left the
        # move at full speed - while the machine's B really ran from
        # 10*cos(72) out to the full 10 in the middle and back.
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th, max_b_velocity=1.)
        th.register_move_check(bp._check_move)
        gcode.run_script('G1 X10 Y-30 B10 F600')
        gcode.run_script('G1 X10 Y30 F600')
        move = th.lookahead.queue[-1]
        self.assertAlmostEqual(bp.project_pos(move.start_pos),
                               bp.project_pos(move.end_pos), places=9)
        # 6.84 degrees of machine B over a 60 mm move, at 1 deg/s
        self.assertLess(math.sqrt(move.max_cruise_v2), 9.)
        self.assertGreater(math.sqrt(move.max_cruise_v2), 8.5)

    def test_move_with_no_machine_b_change_is_untouched(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th, max_b_velocity=1.)
        th.register_move_check(bp._check_move)
        gcode.run_script("G1 X60 F600")
        gcode.run_script("G1 X20 F600")
        move = th.lookahead.queue[-1]
        self.assertAlmostEqual(math.sqrt(move.max_cruise_v2), 10.)

    def test_check_disabled(self):
        # Homing, probing and orienting command machine angles, which
        # the projection would scale by whatever bed angle is under the
        # arm - so they are refused rather than quietly mis-aimed
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = make_bprojection(printer, th)
        self.assertRaises(printer.command_error, bp.check_disabled, 'Homing')
        bp.enabled = False
        bp.check_disabled('Homing')

    def test_toggling_does_not_turn_the_head(self):
        # The head stays at the angle it is at; what changes is the
        # commanded B that names that angle.  Toggling at a non-zero B
        # used to leave the number alone, so the head turned on the
        # next move.
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = self._bp_with_gcode(printer, gcode, th)
        x, y, b = 1.85, 7.65, 20.
        cos_t = x / math.hypot(x, y)
        th.set_position(self._pos(x, y, b))
        machine_before = bp.project_pos(th.get_position())
        self.assertAlmostEqual(machine_before, b * cos_t, places=9)
        gcode.run_script('SET_B_PROJECTION ENABLE=0')
        # Same physical angle, now named directly
        self.assertAlmostEqual(th.get_position()[5], b * cos_t, places=9)
        self.assertAlmostEqual(bp.project_pos(th.get_position()),
                               machine_before, places=9)
        gcode.run_script('SET_B_PROJECTION ENABLE=1')
        self.assertAlmostEqual(th.get_position()[5], b, places=6)
        self.assertAlmostEqual(bp.project_pos(th.get_position()),
                               machine_before, places=6)

    def _bp_with_gcode(self, printer, gcode, th, config_enabled=True):
        bp = make_bprojection(printer, th, config_enabled=config_enabled)
        printer.add_object('b_projection', bp)
        gcode.register_command('SET_B_PROJECTION', bp.cmd_SET_B_PROJECTION)
        bp._update_kinematics = lambda: None
        return bp

    def test_restore_returns_to_the_configured_setting(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = self._bp_with_gcode(printer, gcode, th)
        gcode.run_script('SET_B_PROJECTION ENABLE=0')
        self.assertFalse(bp.enabled)
        gcode.run_script('SET_B_PROJECTION RESTORE=1')
        self.assertTrue(bp.enabled)

    def test_a_machine_configured_off_stays_off_through_the_macros(self):
        # 'enable: False' in printer.cfg is the machine-level switch.
        # The homing macros turn the projection off and then RESTORE
        # it, so it must come back off - forcing ENABLE=1 there would
        # switch the feature on behind the operator.
        printer, gcode, gmove, th, resp = build_env(('b',))
        bp = self._bp_with_gcode(printer, gcode, th, config_enabled=False)
        self.assertFalse(bp.enabled)
        gcode.run_script('SET_B_PROJECTION ENABLE=0')
        gcode.run_script('SET_B_PROJECTION RESTORE=1')
        self.assertFalse(bp.enabled)
        # ...and it is still reachable by hand for a deliberate test
        gcode.run_script('SET_B_PROJECTION ENABLE=1')
        self.assertTrue(bp.enabled)

    def test_enable_and_restore_together_are_refused(self):
        printer, gcode, gmove, th, resp = build_env(('b',))
        self._bp_with_gcode(printer, gcode, th)
        self.assertRaises(gcode_mod.CommandError, gcode.run_script,
                          'SET_B_PROJECTION ENABLE=1 RESTORE=1')

    def test_removed_band_options_are_refused(self):
        # The band was removed rather than re-tuned, so a config that
        # still sets max_angle means something different now
        self.assertIn('max_angle', bproject_mod.REMOVED_OPTIONS)
        self.assertIn('taper_range', bproject_mod.REMOVED_OPTIONS)

    def test_rtcp_uses_the_projected_angle(self):
        # RTCP holds the tip still for the tilt the head really makes.
        # At a bed angle of 90 the projection flattens B away, so there
        # is nothing for RTCP to correct.
        printer, gcode, gmove, th, resp = build_env(('b',))
        r = make_rtcp(printer, th, tool_v=40.,
                      frame=rtcp_mod.FRAME_RADIAL)
        r.b_project = make_bprojection(printer, th)
        m = r.tool_to_machine(self._pos(0., 100., 10.))
        self.assertAlmostEqual(m[0], 0., places=9)
        self.assertAlmostEqual(m[1], 100., places=9)
        # Along the bed's x axis the full tilt survives
        m = r.tool_to_machine(self._pos(100., 0., 10.))
        self.assertAlmostEqual(m[0], 100. - 40. * math.sin(math.radians(10.)),
                               places=9)
        # ...and with the projection removed the commanded B is used
        r.b_project = None
        m = r.tool_to_machine(self._pos(0., 100., 10.))
        self.assertAlmostEqual(m[1], 100. - 40. * math.sin(math.radians(10.)),
                               places=9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
