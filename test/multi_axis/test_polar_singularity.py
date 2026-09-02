#!/usr/bin/env python
# Host test of the bed centre singularity geometry and limits
# (klippy/kinematics/polar.py and klippy/extras/polar_singularity.py).
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

from kinematics import polar
from extras import polar_singularity as ps

# The coefficient the angular acceleration peaks at, 9 / (8 * sqrt(3))
ALPHA_PEAK = 9. / (8. * math.sqrt(3.))


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
    chk.last_radius = chk.last_swept = chk.last_velocity_limit = 0.
    return chk


######################################################################
# Geometry
######################################################################

class TestPathGeometry(unittest.TestCase):
    def test_chord_reaches_its_closest_approach(self):
        # A chord from (40, -30) to (40, 30) has a radius of 50 at both
        # ends and dips to 40 in the middle
        offset, r_min, u_start, u_end = polar.path_geometry((40., -30.),
                                                            (40., 30.))
        self.assertAlmostEqual(abs(offset), 40.)
        self.assertAlmostEqual(r_min, 40.)
        # The endpoints straddle the foot of the perpendicular
        self.assertLessEqual(u_start * u_end, 0.)

    def test_closest_approach_at_an_endpoint(self):
        # An outbound move never comes closer than where it started
        offset, r_min, u_start, u_end = polar.path_geometry((10., 0.),
                                                            (30., 0.1))
        self.assertAlmostEqual(r_min, 10., places=4)
        self.assertGreater(u_start * u_end, 0.)

    def test_radial_paths_have_no_offset(self):
        # A radial move, a move departing the centre and a move arriving
        # at it all leave the bed angle alone
        for start, end in (((10., 0.), (30., 0.)),
                           ((0., 0.), (10., 0.)),
                           ((50., 50.), (0., 0.)),
                           ((15., 15.), (20., 20.))):
            offset = polar.path_geometry(start, end)[0]
            self.assertAlmostEqual(offset, 0.,
                                   msg="%s -> %s" % (start, end))

    def test_zero_length_path(self):
        offset, r_min, u_start, u_end = polar.path_geometry((3., 4.),
                                                            (3., 4.))
        self.assertAlmostEqual(offset, 0.)
        self.assertAlmostEqual(r_min, 5.)


class TestSweptAngle(unittest.TestCase):
    def test_straight_across_the_centre_is_half_a_turn(self):
        self.assertAlmostEqual(abs(polar.swept_angle((10., 0.), (-10., 0.))),
                               math.pi)
        self.assertAlmostEqual(abs(polar.swept_angle((30., 30.),
                                                     (-30., -30.))),
                               math.pi)

    def test_quarter_turn_and_its_sign(self):
        self.assertAlmostEqual(polar.swept_angle((10., 0.), (0., 10.)),
                               math.pi / 2.)
        self.assertAlmostEqual(polar.swept_angle((0., 10.), (10., 0.)),
                               -math.pi / 2.)

    def test_an_endpoint_on_the_centre_sweeps_nothing(self):
        # The centre has no angle of its own, so departing along a ray and
        # arriving along one are zero sweeps rather than undefined ones
        self.assertAlmostEqual(polar.swept_angle((0., 0.), (10., 0.)), 0.)
        self.assertAlmostEqual(polar.swept_angle((50., 50.), (0., 0.)), 0.)


class TestAngularRates(unittest.TestCase):
    def test_peak_velocity_is_v_over_r_min(self):
        offset, r_min, u_start, u_end = polar.path_geometry((40., -30.),
                                                            (40., 30.))
        self.assertAlmostEqual(polar.peak_angular_velocity(100., offset,
                                                           r_min),
                               100. / 40.)

    def test_peak_accel_matches_the_closed_form(self):
        offset, r_min, u_start, u_end = polar.path_geometry((40., -30.),
                                                            (40., 30.))
        self.assertAlmostEqual(
            polar.peak_angular_accel(100., offset, u_start, u_end),
            ALPHA_PEAK * 100. ** 2 / 40. ** 2)

    def test_a_radial_path_turns_the_bed_at_no_rate(self):
        offset, r_min, u_start, u_end = polar.path_geometry((0., 0.),
                                                            (10., 0.))
        self.assertEqual(polar.peak_angular_velocity(100., offset, r_min), 0.)
        self.assertEqual(polar.peak_angular_accel(100., offset,
                                                  u_start, u_end), 0.)

    def test_rates_diverge_as_the_path_tightens(self):
        # theta_dot as 1/r and theta_ddot as 1/r^2 - the whole reason a
        # velocity limit alone is not enough
        rates = []
        for r in (40., 20., 10., 5., 2.5):
            offset, r_min, u_start, u_end = polar.path_geometry((r, -30.),
                                                                (r, 30.))
            rates.append((polar.peak_angular_velocity(100., offset, r_min),
                          polar.peak_angular_accel(100., offset,
                                                   u_start, u_end)))
        for (w0, a0), (w1, a1) in zip(rates, rates[1:]):
            self.assertAlmostEqual(w1 / w0, 2.)
            self.assertAlmostEqual(a1 / a0, 4., delta=1e-6)


class TestRateLimits(unittest.TestCase):
    def test_velocity_limit_meets_the_angular_velocity_exactly(self):
        offset, r_min, u_start, u_end = polar.path_geometry((40., -30.),
                                                            (40., 30.))
        v_limit, a_limit = polar.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 5., 0.)
        self.assertAlmostEqual(
            polar.peak_angular_velocity(v_limit, offset, r_min), 5.)
        self.assertIsNone(a_limit)

    def test_velocity_limit_meets_the_angular_accel_exactly(self):
        offset, r_min, u_start, u_end = polar.path_geometry((40., -30.),
                                                            (40., 30.))
        v_limit, a_limit = polar.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 0., 200.)
        self.assertAlmostEqual(
            polar.peak_angular_accel(v_limit, offset, u_start, u_end), 200.)
        # The move's own acceleration is bounded through a*offset/r^2
        self.assertAlmostEqual(a_limit * abs(offset) / (r_min * r_min), 200.)

    def test_a_radial_path_is_not_limited(self):
        offset, r_min, u_start, u_end = polar.path_geometry((0., 0.),
                                                            (10., 0.))
        self.assertEqual(polar.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 5., 200.), (None, None))

    def test_the_velocity_limit_holds_the_accel_below_a_fixed_figure(self):
        # Wherever the velocity limit binds, the feedrate it leaves is
        # max_angular_velocity * r_min, and the angular acceleration at
        # that feedrate is 0.65 * max_angular_velocity^2 whatever the
        # radius.  So a max_angular_accel above that figure never limits a
        # feedrate - which is worth knowing before setting one.
        for r in (0.5, 5., 20., 40.):
            offset, r_min, u_start, u_end = polar.path_geometry((r, -30.),
                                                                (r, 30.))
            v_limit = polar.limits_for_angular_rates(
                offset, r_min, u_start, u_end, 5., 0.)[0]
            self.assertAlmostEqual(
                polar.peak_angular_accel(v_limit, offset, u_start, u_end),
                ALPHA_PEAK * 5. ** 2, delta=1e-6)


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

    def test_the_floor_is_where_the_refusal_starts(self):
        # 0.1mm needs exactly the 0.5mm/s floor and is allowed; a hair
        # closer is not
        self.assertIsNone(self.refused((0.1, -20.), (0.1, 20.)))
        self.assertIsNotNone(self.refused((0.099, -20.), (0.099, 20.)))

    # -- the clamp --------------------------------------------------------
    def test_a_near_miss_is_slowed_to_the_bed_limit(self):
        for r in (0.15, 1., 5., 20.):
            move = self.check((r, -30.), (r, 30.))
            offset, r_min, u_start, u_end = polar.path_geometry(
                move.start_pos, move.end_pos)
            velocity = move.cruise_velocity()
            self.assertAlmostEqual(
                polar.peak_angular_velocity(velocity, offset, r_min), 5.,
                msg="r_min=%s" % (r_min,))
            self.assertLessEqual(
                polar.peak_angular_accel(velocity, offset, u_start, u_end),
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
                               math.degrees(abs(polar.swept_angle(
                                   (5., -20.), (5., 20.)))))
        self.assertAlmostEqual(status['last_velocity_limit'], 25.)


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


if __name__ == '__main__':
    unittest.main()
