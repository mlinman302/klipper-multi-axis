#!/usr/bin/env python
# Host test of the bed centre move check and feedrate limit
# (klippy/extras/polar_singularity.py and limit_centre_speed() in
# klippy/kinematics/polar.py).  The geometry underneath both is covered by
# test_bed_centre.py.
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* modules with a stubbed printer and a stubbed
# Move, so it runs anywhere Python is available - it needs no compiled
# c_helper.so, no serial port and no Linux host.
#
# Run with:  python test/multi_axis/test_polar_singularity.py
#
# The end-to-end runs are covered separately by
# test/klippy/polar_singularity.test and
# test/klippy/polar_singularity_refuse.test (Linux only).
import math, os, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# stepper.py imports mcu, which needs pyserial; nothing under test uses it
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

from kinematics import bed_centre, centre_path, polar
from extras import polar_singularity as ps
import toolhead as toolhead_mod


######################################################################
# Stubs
######################################################################

class CommandError(Exception):
    pass

class FakeMove:
    # Just enough of toolhead.Move for a move check: the two endpoints,
    # the per-axis deltas, the cruise velocity and the two limit hooks.
    def __init__(self, start_pos, end_pos, velocity):
        self.start_pos = tuple(start_pos) + (0.,) * (4 - len(start_pos))
        self.end_pos = tuple(end_pos) + (0.,) * (4 - len(end_pos))
        self.axes_d = [e - s for s, e in zip(self.start_pos, self.end_pos)]
        self.move_d = math.sqrt(sum([d*d for d in self.axes_d[:3]]))
        self.max_cruise_v2 = velocity ** 2
        self.accel = 3000.
        self.branch = 1
        self.branch_flip = False
    def get_end_branch(self):
        return -self.branch if self.branch_flip else self.branch
    def limit_speed(self, speed, accel):
        if speed ** 2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed ** 2
        self.accel = min(self.accel, accel)
    def move_error(self, msg="Move out of range"):
        return CommandError(msg)
    def cruise_velocity(self):
        return math.sqrt(self.max_cruise_v2)


def build_checker(max_angular_v=5., max_angular_a=0., min_velocity=0.5):
    # The real PolarSingularity without its config or its printer
    chk = ps.PolarSingularity.__new__(ps.PolarSingularity)
    chk.max_angular_v = max_angular_v
    chk.max_angular_a = max_angular_a
    chk.min_velocity = min_velocity
    chk.planner = centre_path.CentrePlanner(max_angular_v, max_angular_a,
                                            min_velocity)
    chk.last_radius = chk.last_swept = chk.last_velocity_limit = 0.
    chk.toolhead = None
    chk.held = chk.hold_timer = None
    return chk


######################################################################
# The move check
######################################################################

class TestMoveCheck(unittest.TestCase):
    def setUp(self):
        self.chk = build_checker(max_angular_v=5., max_angular_a=50.,
                                 min_velocity=0.5)

    def check(self, start, end, velocity=100.):
        move = FakeMove(start, end, velocity)
        self.chk._check_move(move)
        return move

    def refused(self, start, end, velocity=100.):
        move = FakeMove(start, end, velocity)
        try:
            self.chk._check_move(move)
        except CommandError as e:
            return str(e)
        return None

    # -- the moves that are legal on the axis ----------------------------
    def test_pure_z_on_the_axis(self):
        # The N in [0, 0, N] - the radius is zero for the whole move and
        # the bed never turns
        move = self.check((0., 0., 50.), (0., 0., 10.))
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    def test_departing_the_axis_along_a_ray(self):
        move = self.check((0., 0.), (10., 0.))
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    def test_arriving_at_the_axis_along_a_ray(self):
        move = self.check((50., 0.), (0., 0.))
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    def test_a_radial_move_is_never_limited(self):
        move = self.check((10., 10.), (80., 80.))
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    # -- the moves that are refused --------------------------------------
    def test_a_chord_across_the_axis_is_refused(self):
        msg = self.refused((10., 0.), (-10., 0.))
        self.assertIsNotNone(msg)
        self.assertIn("crosses the centre", msg)

    def test_a_diagonal_across_the_axis_is_refused(self):
        self.assertIsNotNone(self.refused((50., 50.), (-50., -50.)))

    def test_a_chord_too_close_to_slow_down_for_is_refused(self):
        # 0.05mm from the centre needs 0.25mm/s, below the 0.5mm/s floor
        msg = self.refused((0.05, -20.), (0.05, 20.))
        self.assertIsNotNone(msg)
        self.assertIn("from the centre", msg)

    def test_a_move_already_that_slow_is_not_refused(self):
        # 0.05mm out needs 0.25mm/s - below the floor, but a move asked to
        # run at 0.2mm/s is already slow enough for the bed
        self.assertIsNone(self.refused((0.05, -20.), (0.05, 20.),
                                       velocity=.2))
        self.assertIsNotNone(self.refused((0.05, -20.), (0.05, 20.),
                                          velocity=.3))

    def test_the_floor_is_where_the_refusal_starts(self):
        # 0.1mm needs exactly the 0.5mm/s floor and is allowed; a hair
        # closer is not
        self.assertIsNone(self.refused((0.1, -20.), (0.1, 20.)))
        self.assertIsNotNone(self.refused((0.099, -20.), (0.099, 20.)))

    # -- the clamp --------------------------------------------------------
    def test_a_near_miss_is_slowed_to_the_bed_limit(self):
        for r in (0.15, 1., 5., 20.):
            move = self.check((r, -30.), (r, 30.))
            offset, r_min, u_start, u_end = bed_centre.path_geometry(
                move.start_pos, move.end_pos)
            velocity = move.cruise_velocity()
            self.assertAlmostEqual(
                bed_centre.peak_angular_velocity(velocity, offset, r_min), 5.,
                msg="r_min=%s" % (r_min,))
            self.assertLessEqual(
                bed_centre.peak_angular_accel(velocity, offset, u_start, u_end),
                50. + 1e-6)

    def test_a_wide_path_is_left_alone(self):
        move = self.check((40., -30.), (40., 30.))
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    def test_a_slow_move_is_not_sped_up(self):
        move = self.check((40., -30.), (40., 30.), velocity=10.)
        self.assertAlmostEqual(move.cruise_velocity(), 10.)

    def test_rotation_only_at_the_centre_is_not_checked(self):
        # No xy travel, so the bed is not asked to turn.  axes_d[0:3] are
        # zeroed by Move for a rotation only move.
        move = FakeMove((0., 0., 10.), (0., 0., 10.), 100.)
        self.chk._check_move(move)
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    # -- diagnostics ------------------------------------------------------
    def test_status_reports_the_last_approach(self):
        self.check((5., -20.), (5., 20.))
        status = self.chk.get_status(0.)
        self.assertAlmostEqual(status['last_radius'], 5.)
        self.assertAlmostEqual(abs(status['last_swept_angle']),
                               math.degrees(abs(bed_centre.swept_angle(
                                   (5., -20.), (5., 20.)))))
        self.assertAlmostEqual(status['last_velocity_limit'], 25.)
        self.assertEqual(status['travel_policy'], 'bypass')
        self.assertEqual(status['print_policy'], 'error')
        self.assertEqual(status['reorient_radius'], .125)


######################################################################
# The limiter the kinematics apply
######################################################################

class TestKinematicsLimit(unittest.TestCase):
    def test_a_path_through_the_centre_is_no_longer_unlimited(self):
        # This used to return early on a closest approach of zero - the
        # one move that most needed the limit was the one move that
        # escaped it.  There is no rate to hold here (the perpendicular
        # offset is zero, so the bed turns at no rate at all until it
        # steps), but the call must at least not fall over.
        move = FakeMove((10., 0.), (-10., 0.), 100.)
        polar.limit_centre_speed(move, 5.)
        self.assertGreater(move.cruise_velocity(), 0.)

    def test_the_limit_matches_the_module(self):
        move = FakeMove((5., -20.), (5., 20.), 100.)
        polar.limit_centre_speed(move, 5.)
        self.assertAlmostEqual(move.cruise_velocity(), 25.)

    def test_a_radial_move_is_untouched(self):
        move = FakeMove((0., 0.), (80., 0.), 100.)
        polar.limit_centre_speed(move, 5.)
        self.assertAlmostEqual(move.cruise_velocity(), 100.)


######################################################################
# The position names the bed angle
######################################################################

PARK = 1e-4


class TestCentreRest(unittest.TestCase):
    # A move may only come to rest in the dead zone on the ray it arrived
    # along, and only leave it along the line the bed faces
    def setUp(self):
        self.chk = build_checker()

    def refused(self, start, end, branch=1, flip=False):
        move = FakeMove(start, end, 100.)
        move.branch, move.branch_flip = branch, flip
        try:
            self.chk._check_move(move)
        except CommandError as e:
            return str(e)
        return None

    def test_leaving_the_bare_centre_off_the_line_it_faces(self):
        # Standing on (0, 0) names a bed angle of zero
        self.assertIn("leaves the centre", self.refused((0., 0.), (0., 10.)))
        self.assertIn("leaves the centre",
                      self.refused((0., 0.), (-10., 0.)))
        self.assertIsNone(self.refused((0., 0.), (10., 0.)))

    def test_leaving_a_park_point_off_its_ray(self):
        self.assertIsNotNone(self.refused((0., PARK), (10., PARK)))
        self.assertIsNone(self.refused((0., PARK), (0., 10.)))

    def test_coming_to_rest_on_the_bare_centre_off_its_ray(self):
        self.assertIn("comes to rest", self.refused((0., 40.), (0., 0.)))
        self.assertIsNone(self.refused((40., 0.), (0., 0.)))
        self.assertIsNone(self.refused((0., 40.), (0., PARK)))

    def test_a_small_turn_is_left_to_the_step_generator(self):
        # Off the line by well under ANGLE_TOLERANCE
        self.assertIsNone(self.refused((PARK, 0.), (40., 40e-6)))


class TestCrossingMoves(unittest.TestCase):
    def setUp(self):
        self.chk = build_checker(max_angular_v=5., max_angular_a=50.)

    def check(self, start, end, branch=1, flip=False):
        move = FakeMove(start, end, 100.)
        move.branch, move.branch_flip = branch, flip
        self.chk._check_move(move)
        return move

    def test_straight_through_on_the_flip(self):
        # Off the line by rounding alone, as a planned move is.  The bed
        # does not turn, so it is neither refused nor slowed.
        move = self.check((PARK, 1e-12), (-40., 0.), flip=True)
        self.assertAlmostEqual(move.cruise_velocity(), 100.)
        self.assertEqual(self.chk.last_swept, 0.)
        # ...and back
        move = self.check((-PARK, 0.), (40., 0.), branch=-1, flip=True)
        self.assertAlmostEqual(move.cruise_velocity(), 100.)

    def test_the_same_move_without_it_is_refused(self):
        self.assertRaises(CommandError, self.check, (PARK, 1e-12),
                          (-40., 0.))

    def test_a_flip_off_the_line_the_bed_faces_is_refused(self):
        self.assertRaises(CommandError, self.check, (PARK, 0.), (-40., 5.),
                          1, True)

    def test_a_flip_that_does_not_reach_the_far_side(self):
        # Marked, but stopping short of the centre: the last instant would
        # hand the bed the far side's angle
        self.assertRaises(CommandError, self.check, (PARK, 0.),
                          (PARK / 2., 0.), 1, True)


class FakeKinematics:
    def __init__(self, can_cross):
        self.can_cross = can_cross
    def can_cross_centre(self):
        return self.can_cross


class FakeToolhead:
    def __init__(self, can_cross=False, position=(0., 0., 0., 0.),
                 branch=1):
        self.kin = FakeKinematics(can_cross)
        self.position = list(position)
        self.branch = branch
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        return list(self.position)
    def get_branch(self):
        return self.branch


class TestConfigAtConnect(unittest.TestCase):
    def build(self, travel_policy='bypass', print_policy='error'):
        chk = build_checker()
        chk.name = 'polar_singularity'
        chk.travel_policy, chk.print_policy = travel_policy, print_policy
        chk.reorient_radius = None
        chk.blend_tolerance, chk.blend_radius = .05, 2.
        chk.held = chk.hold_timer = None
        return chk

    def test_cross_needs_the_kinematics_to_allow_it(self):
        chk = self.build(travel_policy='cross')
        self.assertRaises(CommandError, chk._make_planner, False,
                          CommandError)
        planner = chk._make_planner(True, CommandError)
        self.assertEqual(planner.travel_policy, 'cross')
        self.assertEqual(planner.blend_tolerance, .05)

    def test_the_defaults_follow_what_the_arm_can_do(self):
        chk = self.build(travel_policy=None, print_policy=None)
        planner = chk._make_planner(True, CommandError)
        self.assertEqual((planner.travel_policy, planner.print_policy),
                         ('cross', 'cross'))
        planner = chk._make_planner(False, CommandError)
        self.assertEqual((planner.travel_policy, planner.print_policy),
                         ('bypass', 'error'))
        # One set, the other defaulted
        chk.print_policy = 'error'
        planner = chk._make_planner(True, CommandError)
        self.assertEqual((planner.travel_policy, planner.print_policy),
                         ('cross', 'error'))

    def test_bad_blend_values_are_config_errors(self):
        chk = self.build()
        chk.blend_radius = .05
        self.assertRaises(CommandError, chk._make_planner, True,
                          CommandError)

    def test_status_reports_where_the_tool_stands(self):
        chk = self.build()
        chk.toolhead = FakeToolhead(position=(PARK, 0., 0., 0.), branch=-1)
        status = chk.get_status(0.)
        self.assertTrue(status['at_centre'])
        self.assertEqual(status['branch'], -1)
        self.assertFalse(status['holding'])
        self.assertEqual(status['blend_tolerance'], .05)
        chk.toolhead.position = [40., 0., 0., 0.]
        self.assertFalse(chk.get_status(0.)['at_centre'])



######################################################################
# The toolhead gives up a held move before anything else sees it
######################################################################

class TestToolheadReleasesHeldMoves(unittest.TestCase):
    # [polar_singularity] holds the tail of a move onto the centre back
    # for the next move.  Every toolhead operation that must see all the
    # moves made so far has to release it first - before it reads the
    # position, the queue or the time.
    def build(self):
        th = toolhead_mod.ToolHead.__new__(toolhead_mod.ToolHead)
        th.held_move_releases = []
        calls = []
        th.register_held_moves(lambda: calls.append('release'))
        def internal(name, result=None):
            def f(*args, **kw):
                calls.append(name)
                return result
            return f
        for name in ('_flush_lookahead', '_process_lookahead',
                     '_calc_print_time', '_advance_move_time',
                     '_check_pause', 'set_branch'):
            setattr(th, name, internal(name))
        th.special_queuing_state = 'NeedPrime'
        th.print_time = 0.
        th.motion_queuing = types.SimpleNamespace(
            flush_all_steps=internal('flush_all_steps'),
            get_kin_flush_delay=internal('get_kin_flush_delay', 0.))
        th.lookahead = types.SimpleNamespace(
            get_last=internal('get_last'))
        th.commanded_pos = [0.] * 7
        return th, calls

    def assert_released_first(self, call):
        th, calls = self.build()
        try:
            call(th)
        except Exception:
            # What follows the release is not what is under test
            pass
        self.assertTrue(calls, msg=calls)
        self.assertEqual(calls[0], 'release', msg=calls)

    def test_every_operation_releases_first(self):
        operations = [
            lambda th: th.move([1., 0., 0., 0., 0., 0., 0.], 10.),
            lambda th: th.manual_move([1., None], 10.),
            lambda th: th.dwell(1.),
            lambda th: th.wait_moves(),
            lambda th: th.get_last_move_time(),
            lambda th: th.flush_step_generation(),
            lambda th: th.set_position([0.] * 7),
            lambda th: th.drip_move([1., 0., 0.], 10., None),
            lambda th: th.register_lookahead_callback(lambda t: None),
        ]
        for call in operations:
            self.assert_released_first(call)

    def test_releasing_with_nothing_held_does_nothing(self):
        chk = build_checker()
        chk.release()
        self.assertIsNone(chk.held)


if __name__ == '__main__':
    unittest.main()
