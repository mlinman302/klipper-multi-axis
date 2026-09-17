#!/usr/bin/env python
# Host test of the bed centre geometry (klippy/kinematics/bed_centre.py)
# and of its C mirror (klippy/chelper/bed_centre.h).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# bed_centre.py imports nothing but math, so this needs nothing but
# Python - no compiled c_helper.so, no cffi, no Linux host.
#
# Run with:  python test/multi_axis/test_bed_centre.py
#
# The C side of the dead zone rule is checked against the same table by
# test_bed_centre() in test/multi_axis/test_kin_6axis.c, which
# test/multi_axis/run_c_tests.sh builds and runs.
import math, os, re, sys, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.normpath(os.path.join(HERE, '..', '..'))
KLIPPY_DIR = os.path.join(REPO_DIR, 'klippy')
sys.path.insert(0, KLIPPY_DIR)

from kinematics import bed_centre

HEADER = os.path.join(KLIPPY_DIR, 'chelper', 'bed_centre.h')
PEAK = bed_centre.ANGULAR_ACCEL_PEAK


######################################################################
# The C mirror
######################################################################

def header_defines():
    defines = {}
    with open(HEADER) as f:
        for line in f:
            m = re.match(r'\s*#define\s+(BED_CENTRE_\w+)\s+(\S+)', line)
            if m:
                defines[m.group(1)] = float(m.group(2))
    return defines

# Every number the header defines, and the Python name it mirrors
MIRRORED = {
    'BED_CENTRE_RADIUS': 'CENTRE_RADIUS',
    'BED_CENTRE_EPSILON': 'CENTRE_EPSILON',
}

# The names the copies used to go by.  Each was once a separate
# definition of something bed_centre now owns.
RETIRED = re.compile(r'\b(BED_MIN_RADIUS|RADIAL_EPSILON'
                     r'|bproject_cos_bed_angle)\b')


class TestMirror(unittest.TestCase):
    def test_header_matches_python(self):
        defines = header_defines()
        for c_name, py_name in MIRRORED.items():
            self.assertIn(c_name, defines)
            self.assertEqual(defines[c_name], getattr(bed_centre, py_name),
                             "%s in bed_centre.h does not match %s in"
                             " bed_centre.py" % (c_name, py_name))

    def test_header_defines_nothing_unmirrored(self):
        # A number added to the header must be added to bed_centre.py and
        # to MIRRORED above, or nothing here guards it
        self.assertEqual(sorted(header_defines()), sorted(MIRRORED))

    def test_no_copies_left_behind(self):
        found = []
        for top in ('klippy', 'test', 'config'):
            for dirpath, dirnames, filenames in os.walk(
                    os.path.join(REPO_DIR, top)):
                dirnames[:] = [d for d in dirnames if d != '__pycache__']
                for fn in filenames:
                    if not fn.endswith(('.py', '.c', '.h', '.cfg')):
                        continue
                    path = os.path.join(dirpath, fn)
                    if os.path.samefile(path, __file__):
                        continue
                    with open(path, errors='replace') as f:
                        for lineno, line in enumerate(f, 1):
                            if RETIRED.search(line):
                                found.append("%s:%d" % (
                                    os.path.relpath(path, REPO_DIR),
                                    lineno))
        self.assertEqual(found, [])


######################################################################
# The bed angle at a sample
######################################################################

# The same table as test_bed_centre() in test_kin_6axis.c:
# (what, x, y, travel x, travel y, angle)
HALF_PI = math.pi / 2.
BED_CENTRE_CASES = [
    ("outside, static, +x",          50.,    0.,     0.,  0., 0.),
    ("outside, static, +y",          0.,     50.,    0.,  0., HALF_PI),
    ("inside, static, off centre",   -0.005, 0.,     0.,  0., math.pi),
    ("on the centre, static",        0.,     0.,     0.,  0., 0.),
    ("inside, heading in along +x",  -0.005, 0.,     1.,  0., math.pi),
    ("inside, heading out along +x", 0.005,  0.,     1.,  0., 0.),
    ("on the centre, along +x",      0.,     0.,     1.,  0., 0.),
    ("on the centre, along -x",      0.,     0.,     -1., 0., math.pi),
    ("inside, heading in along -x",  0.005,  0.,     -1., 0., 0.),
    ("inside, heading out along +y", 0.003,  0.004,  0.,  1., HALF_PI),
    ("outside, moving, +x",          0.02,   0.,     0.,  1., 0.),
    ("inside, heading in along +y",  0.,     -0.005, 0.,  1., -HALF_PI),
]


class TestBedAngle(unittest.TestCase):
    def test_the_shared_table(self):
        for what, x, y, rx, ry, angle in BED_CENTRE_CASES:
            travel = (rx, ry)
            self.assertAlmostEqual(bed_centre.bed_angle(x, y, travel),
                                   angle, places=12, msg=what)
            self.assertAlmostEqual(bed_centre.cos_bed_angle(x, y, travel),
                                   math.cos(angle), places=12, msg=what)

    def test_no_travel_is_the_same_as_standing_still(self):
        for x, y in ((50., 0.), (-0.005, 0.), (0., 0.), (0.003, -0.004)):
            self.assertEqual(bed_centre.bed_angle(x, y),
                             bed_centre.bed_angle(x, y, (0., 0.)))
            self.assertEqual(bed_centre.cos_bed_angle(x, y),
                             bed_centre.cos_bed_angle(x, y, (0., 0.)))

    def test_cosine_agrees_with_the_angle_everywhere(self):
        # The cosine form skips the atan2 on the common path, so check it
        # over a grid spanning both sides of the zone boundary
        r = bed_centre.CENTRE_RADIUS
        coords = [-3. * r, -r, -.5 * r, -1e-6, 0., 1e-6, .5 * r, r, 3. * r]
        travels = [None, (1., 0.), (-1., 0.), (0., 1.), (.6, -.8)]
        for x in coords:
            for y in coords:
                for travel in travels:
                    self.assertAlmostEqual(
                        bed_centre.cos_bed_angle(x, y, travel),
                        math.cos(bed_centre.bed_angle(x, y, travel)),
                        places=12, msg="%s %s %s" % (x, y, travel))

    def test_nothing_turns_at_the_zone_boundary(self):
        # Heading out along a ray, the travel angle just inside the zone is
        # the position angle just outside it
        r = bed_centre.CENTRE_RADIUS
        for angle in (0., 1., -2., math.pi):
            ux, uy = math.cos(angle), math.sin(angle)
            inside = bed_centre.bed_angle(.99 * r * ux, .99 * r * uy,
                                          (ux, uy))
            outside = bed_centre.bed_angle(1.01 * r * ux, 1.01 * r * uy,
                                           (ux, uy))
            self.assertAlmostEqual(math.cos(inside), math.cos(outside))
            self.assertAlmostEqual(math.sin(inside), math.sin(outside))

    def test_half_turn(self):
        self.assertEqual(bed_centre.half_turn(0.), math.pi)
        self.assertEqual(bed_centre.half_turn(math.pi), 0.)
        self.assertAlmostEqual(bed_centre.half_turn(-HALF_PI), HALF_PI)
        for angle in (-3., -1., 0., 1., 3., math.pi):
            turned = bed_centre.half_turn(angle)
            self.assertGreater(turned, -math.pi - 1e-15)
            self.assertLessEqual(turned, math.pi)
            self.assertAlmostEqual(math.cos(turned), -math.cos(angle))

    def test_unwrap_angle(self):
        self.assertAlmostEqual(bed_centre.unwrap_angle(-3., 3.),
                               -3. + 2. * math.pi)
        self.assertAlmostEqual(bed_centre.unwrap_angle(3., -3.),
                               3. - 2. * math.pi)
        self.assertEqual(bed_centre.unwrap_angle(1., 0.), 1.)


######################################################################
# A move in two numbers
######################################################################

class TestPathGeometry(unittest.TestCase):
    def test_chord_reaches_its_closest_approach(self):
        # A chord from (40, -30) to (40, 30) has a radius of 50 at both
        # ends and dips to 40 in the middle
        offset, r_min, u_start, u_end = bed_centre.path_geometry(
            (40., -30.), (40., 30.))
        self.assertAlmostEqual(abs(offset), 40.)
        self.assertAlmostEqual(r_min, 40.)
        # The endpoints straddle the foot of the perpendicular
        self.assertLessEqual(u_start * u_end, 0.)

    def test_closest_approach_at_an_endpoint(self):
        # An outbound move never comes closer than where it started
        offset, r_min, u_start, u_end = bed_centre.path_geometry(
            (10., 0.), (30., 0.1))
        self.assertAlmostEqual(r_min, 10., places=4)
        self.assertGreater(u_start * u_end, 0.)

    def test_radial_paths_have_no_offset(self):
        # A radial move, a move departing the centre and a move arriving
        # at it all leave the bed angle alone
        for start, end in (((10., 0.), (30., 0.)),
                           ((0., 0.), (10., 0.)),
                           ((50., 50.), (0., 0.)),
                           ((15., 15.), (20., 20.))):
            offset = bed_centre.path_geometry(start, end)[0]
            self.assertAlmostEqual(offset, 0.,
                                   msg="%s -> %s" % (start, end))

    def test_zero_length_path(self):
        offset, r_min, u_start, u_end = bed_centre.path_geometry(
            (3., 4.), (3., 4.))
        self.assertAlmostEqual(offset, 0.)
        self.assertAlmostEqual(r_min, 5.)


class TestSweptAngle(unittest.TestCase):
    def test_straight_across_the_centre_is_half_a_turn(self):
        self.assertAlmostEqual(
            abs(bed_centre.swept_angle((10., 0.), (-10., 0.))), math.pi)
        self.assertAlmostEqual(
            abs(bed_centre.swept_angle((30., 30.), (-30., -30.))), math.pi)

    def test_quarter_turn_and_its_sign(self):
        self.assertAlmostEqual(
            bed_centre.swept_angle((10., 0.), (0., 10.)), HALF_PI)
        self.assertAlmostEqual(
            bed_centre.swept_angle((0., 10.), (10., 0.)), -HALF_PI)

    def test_an_endpoint_on_the_centre_sweeps_nothing(self):
        # The centre has no angle of its own, so departing along a ray and
        # arriving along one are zero sweeps rather than undefined ones
        self.assertAlmostEqual(
            bed_centre.swept_angle((0., 0.), (10., 0.)), 0.)
        self.assertAlmostEqual(
            bed_centre.swept_angle((50., 50.), (0., 0.)), 0.)


class TestCrossesCentre(unittest.TestCase):
    def crosses(self, start, end):
        r_min = bed_centre.path_geometry(start, end)[1]
        return bed_centre.crosses_centre(
            r_min, bed_centre.swept_angle(start, end))

    def test_through_the_centre(self):
        self.assertTrue(self.crosses((10., 0.), (-10., 0.)))
        self.assertTrue(self.crosses((30., 30.), (-30., -30.)))
        # Inside the dead zone counts as through it
        self.assertTrue(self.crosses((10., .005), (-10., .005)))

    def test_on_the_axis_without_turning(self):
        self.assertFalse(self.crosses((0., 0.), (0., 0.)))
        self.assertFalse(self.crosses((0., 0.), (10., 0.)))
        self.assertFalse(self.crosses((50., 50.), (0., 0.)))

    def test_near_misses_are_left_to_the_rate_limits(self):
        self.assertFalse(self.crosses((10., .02), (-10., .02)))
        self.assertFalse(self.crosses((40., -30.), (40., 30.)))


######################################################################
# Where along a move the centre matters
######################################################################

class TestFractions(unittest.TestCase):
    def test_interpolate_carries_every_coordinate(self):
        self.assertEqual(
            bed_centre.interpolate((0., 0., 0., 0., 10.),
                                   (10., 20., 30., 1., 30.), .5),
            [5., 10., 15., .5, 20.])

    def test_closest_approach_fraction(self):
        fraction = bed_centre.closest_approach_fraction
        self.assertAlmostEqual(fraction((40., -30.), (40., 30.)), .5)
        self.assertAlmostEqual(fraction((10., -10.), (10., 30.)), .25)
        # At an end, or no xy travel at all - nothing to add
        self.assertIsNone(fraction((10., 0.), (30., 0.)))
        self.assertIsNone(fraction((50., 0.), (0., 0.)))
        self.assertIsNone(fraction((5., 5.), (5., 5.)))

    def test_closest_approach_agrees_with_path_geometry(self):
        for start, end in (((40., -30.), (40., 30.)),
                           ((10., -10.), (10., 30.)),
                           ((-7., 3.), (12., -1.))):
            t = bed_centre.closest_approach_fraction(start, end)
            x, y = bed_centre.interpolate(start, end, t)
            self.assertAlmostEqual(math.hypot(x, y),
                                   bed_centre.path_geometry(start, end)[1])

    def test_axis_crossing_fraction(self):
        fraction = bed_centre.axis_crossing_fraction
        self.assertAlmostEqual(fraction((40., -30.), (40., 10.), 1), .75)
        self.assertAlmostEqual(fraction((-10., 5.), (30., 5.), 0), .25)
        # No crossing, or one at an end
        self.assertIsNone(fraction((40., 10.), (40., 30.), 1))
        self.assertIsNone(fraction((40., 0.), (40., 30.), 1))
        self.assertIsNone(fraction((40., -30.), (40., 0.), 1))
        self.assertIsNone(fraction((40., 0.), (40., 0.), 1))

    def test_zone_crossing(self):
        crossing = bed_centre.zone_crossing
        # Straight through: the middle tenth of a 20mm chord is within 1mm
        t_in, t_out = crossing((-10., 0.), (10., 0.), 1.)
        self.assertAlmostEqual(t_in, .45)
        self.assertAlmostEqual(t_out, .55)
        # A chord 0.6mm off centre spends 2 * 0.8mm inside a 1mm disc
        t_in, t_out = crossing((-10., .6), (10., .6), 1.)
        self.assertAlmostEqual((t_out - t_in) * 20., 1.6)
        # Starting inside: clipped to the start of the move
        t_in, t_out = crossing((0., 0.), (10., 0.), 1.)
        self.assertEqual(t_in, 0.)
        self.assertAlmostEqual(t_out, .1)
        # Standing still inside
        self.assertEqual(crossing((.1, .1), (.1, .1), 1.), (0., 1.))
        # Missing it, and grazing it
        self.assertIsNone(crossing((-10., 2.), (10., 2.), 1.))
        self.assertIsNone(crossing((-10., 1.), (10., 1.), 1.))

    def test_zone_crossing_ends_on_the_circle(self):
        start, end = (-7., 3.), (12., -1.)
        for t in bed_centre.zone_crossing(start, end, 5.):
            x, y = bed_centre.interpolate(start, end, t)
            self.assertAlmostEqual(math.hypot(x, y), 5.)


######################################################################
# Angular rates and the limits that follow from them
######################################################################

def chord(r):
    # A 60mm chord passing r from the centre, and its geometry
    return bed_centre.path_geometry((r, -30.), (r, 30.))


class TestAngularRates(unittest.TestCase):
    def test_peak_velocity_is_v_over_r_min(self):
        offset, r_min, u_start, u_end = chord(40.)
        self.assertAlmostEqual(
            bed_centre.peak_angular_velocity(100., offset, r_min),
            100. / 40.)

    def test_peak_accel_matches_the_closed_form(self):
        offset, r_min, u_start, u_end = chord(40.)
        self.assertAlmostEqual(
            bed_centre.peak_angular_accel(100., offset, u_start, u_end),
            PEAK * 100. ** 2 / 40. ** 2)

    def test_a_radial_path_turns_the_bed_at_no_rate(self):
        offset, r_min, u_start, u_end = bed_centre.path_geometry(
            (0., 0.), (10., 0.))
        self.assertEqual(
            bed_centre.peak_angular_velocity(100., offset, r_min), 0.)
        self.assertEqual(
            bed_centre.peak_angular_accel(100., offset, u_start, u_end), 0.)

    def test_rates_diverge_as_the_path_tightens(self):
        # theta_dot as 1/r and theta_ddot as 1/r^2 - the whole reason a
        # velocity limit alone is not enough
        rates = []
        for r in (40., 20., 10., 5., 2.5):
            offset, r_min, u_start, u_end = chord(r)
            rates.append((
                bed_centre.peak_angular_velocity(100., offset, r_min),
                bed_centre.peak_angular_accel(100., offset,
                                              u_start, u_end)))
        for (w0, a0), (w1, a1) in zip(rates, rates[1:]):
            self.assertAlmostEqual(w1 / w0, 2.)
            self.assertAlmostEqual(a1 / a0, 4., delta=1e-6)


class TestRateLimits(unittest.TestCase):
    def test_velocity_limit_meets_the_angular_velocity_exactly(self):
        offset, r_min, u_start, u_end = chord(40.)
        v_limit, a_limit = bed_centre.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 5., 0.)
        self.assertAlmostEqual(
            bed_centre.peak_angular_velocity(v_limit, offset, r_min), 5.)
        self.assertIsNone(a_limit)

    def test_velocity_limit_meets_the_angular_accel_exactly(self):
        offset, r_min, u_start, u_end = chord(40.)
        v_limit, a_limit = bed_centre.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 0., 200.)
        self.assertAlmostEqual(
            bed_centre.peak_angular_accel(v_limit, offset, u_start, u_end),
            200.)
        # The move's own acceleration is bounded through a*offset/r^2
        self.assertAlmostEqual(a_limit * abs(offset) / (r_min * r_min),
                               200.)

    def test_a_radial_path_is_not_limited(self):
        offset, r_min, u_start, u_end = bed_centre.path_geometry(
            (0., 0.), (10., 0.))
        self.assertEqual(bed_centre.limits_for_angular_rates(
            offset, r_min, u_start, u_end, 5., 200.), (None, None))

    def test_the_velocity_limit_holds_the_accel_below_a_fixed_figure(self):
        # Wherever the velocity limit binds, the feedrate it leaves is
        # max_angular_velocity * r_min, and the angular acceleration at
        # that feedrate is 0.65 * max_angular_velocity^2 whatever the
        # radius.  So a max_angular_accel above that figure never limits a
        # feedrate - which is worth knowing before setting one.
        for r in (0.5, 5., 20., 40.):
            offset, r_min, u_start, u_end = chord(r)
            v_limit = bed_centre.limits_for_angular_rates(
                offset, r_min, u_start, u_end, 5., 0.)[0]
            self.assertAlmostEqual(
                bed_centre.peak_angular_accel(v_limit, offset,
                                              u_start, u_end),
                PEAK * 5. ** 2, delta=1e-6)


######################################################################
# Signed radius
######################################################################

class TestSignedRadius(unittest.TestCase):
    def test_round_trip_on_both_branches(self):
        for x, y in ((30., 40.), (-30., 40.), (-30., -40.), (30., -40.),
                     (5., 0.), (-5., 0.), (0., 5.), (0., -5.)):
            for branch in (1, -1):
                r, theta = bed_centre.polar_position(x, y, branch)
                self.assertEqual(r < 0., branch < 0)
                self.assertGreater(theta, -math.pi - 1e-15)
                self.assertLessEqual(theta, math.pi)
                back_x, back_y = bed_centre.cartesian_position(r, theta)
                self.assertAlmostEqual(back_x, x)
                self.assertAlmostEqual(back_y, y)
                self.assertEqual(
                    bed_centre.branch_for_angle(x, y, theta), branch)
                self.assertAlmostEqual(
                    bed_centre.signed_radius(x, y, theta), r)

    def test_the_two_branches_are_half_a_turn_apart(self):
        r_pos, t_pos = bed_centre.polar_position(30., 40., 1)
        r_neg, t_neg = bed_centre.polar_position(30., 40., -1)
        self.assertAlmostEqual(r_neg, -r_pos)
        self.assertAlmostEqual(abs(t_pos - t_neg), math.pi)

    def test_the_centre(self):
        self.assertEqual(bed_centre.polar_position(0., 0., 1), (0., 0.))
        r, theta = bed_centre.polar_position(0., 0., -1)
        self.assertEqual((r, theta), (0., math.pi))

    def test_a_line_through_the_centre_is_radial_with_the_angle_held(self):
        # The point of the second branch.  Held at the angle the line
        # starts on, the radius runs straight through zero and the bed
        # angle never moves - where on the usual branch it steps by pi.
        start, end = (30., 40.), (-30., -40.)
        theta = bed_centre.polar_position(start[0], start[1])[1]
        radii, branches = [], []
        for i in range(11):
            x, y = bed_centre.interpolate(start, end, i / 10.)
            radius = bed_centre.signed_radius(x, y, theta)
            # Holding theta reaches every point on the line
            back_x, back_y = bed_centre.cartesian_position(radius, theta)
            self.assertAlmostEqual(back_x, x)
            self.assertAlmostEqual(back_y, y)
            radii.append(radius)
            branches.append(bed_centre.branch_for_angle(x, y, theta))
        for a, b in zip(radii, radii[1:]):
            self.assertAlmostEqual(a - b, 10.)
        self.assertAlmostEqual(radii[0], 50.)
        self.assertAlmostEqual(radii[5], 0.)
        self.assertAlmostEqual(radii[-1], -50.)
        self.assertEqual(branches[:5], [1] * 5)
        self.assertEqual(branches[6:], [-1] * 5)


if __name__ == '__main__':
    unittest.main()
