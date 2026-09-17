#!/usr/bin/env python
# Host test of path planning at the bed centre
# (klippy/kinematics/centre_path.py) and of the g-code transform that
# applies it (klippy/extras/polar_singularity.py).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Runs anywhere Python is available - no compiled c_helper.so, no MCU.
#
# The central check is bed_trace() below.  It replays a sequence of moves
# through bed_centre.bed_angle(), the Python mirror of the rule the step
# generator runs (pinned to the C by test_bed_centre.py), and reports the
# largest jump in the bed angle it commands.  A jump is what the step
# compressor cannot follow; a planned sequence must not contain one, and
# the same g-code sent straight through does.
#
# The replay is stricter than the step generator in two places: it
# samples the very last instant of every move, which the step generator
# stops short of, and it samples the bed on moves with no x/y travel,
# which the step generator skips unless RTCP has made the bed live on B.
# Passing it is therefore sufficient.  test_kin_6axis.c runs the same
# planned sequences through the real step compressor.
#
# Run with:  python test/multi_axis/test_centre_path.py
import math, os, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# stepper.py imports mcu, which needs pyserial; nothing under test uses it
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

from kinematics import bed_centre, centre_path
from extras import polar_singularity as ps

MAX_V = 5.
MAX_A = 50.
MIN_V = .5
# The largest bed angle change between neighbouring samples that is a
# rate rather than a step
STEP = .01


def make_planner(**kw):
    args = dict(max_angular_v=MAX_V, max_angular_a=MAX_A,
                min_velocity=MIN_V)
    args.update(kw)
    return centre_path.CentrePlanner(**args)

def pos(x, y, z=10., e=0.):
    return [x, y, z, e]

def xy_of(moves):
    return [(p[0], p[1]) for p, speed in moves]


######################################################################
# Replaying moves through the step generator's bed angle rule
######################################################################

def _samples(start, end):
    offset, r_min, u_start, u_end = bed_centre.path_geometry(start, end)
    length = u_end - u_start
    if not offset or not length:
        # Radial, or still: the angle can only change at the ends, where
        # the samples always are
        return 64
    # Near the closest approach a sample spacing of ds turns the bed by
    # about ds / r_min, so keep that well under the step threshold
    return int(min(max(64, math.ceil(2. * length / (r_min * STEP))),
                   200000))

def bed_trace(start_xy, targets):
    # The bed angles the step generator would command, at rest and
    # through each move in turn
    angles = [bed_centre.bed_angle(*start_xy)]
    here = start_xy
    for target in targets:
        dx, dy = target[0] - here[0], target[1] - here[1]
        travel = (dx, dy) if (dx or dy) else None
        n = _samples(here, target)
        for i in range(n + 1):
            t = float(i) / n
            angles.append(bed_centre.bed_angle(
                here[0] + t * dx, here[1] + t * dy, travel))
        here = (target[0], target[1])
        angles.append(bed_centre.bed_angle(*here))
    return angles

def largest_step(angles):
    return max([abs(centre_path.wrap_angle(b - a))
                for a, b in zip(angles, angles[1:])] + [0.])


class Transform:
    # The planner applied the way the g-code transform applies it, with
    # the toolhead reduced to the position it was last sent
    def __init__(self, planner, start):
        self.planner = planner
        self.machine = list(start)
        self.last = list(start)
        self.sent = []
    def move(self, newpos, speed=100.):
        for p, s in self.planner.plan(self.machine[:2], self.last, newpos,
                                      speed):
            self.sent.append((list(p), s))
            self.machine = list(p)
        self.last = list(newpos)


######################################################################
# Arriving, staying and departing
######################################################################

class TestArrive(unittest.TestCase):
    def test_stops_short_on_the_ray_it_arrived_along(self):
        moves = make_planner().plan((30., 40.), pos(30., 40.),
                                    pos(0., 0., 5., 2.), 50.)
        self.assertEqual(len(moves), 1)
        p, speed = moves[0]
        self.assertAlmostEqual(math.hypot(p[0], p[1]),
                               centre_path.PARK_RADIUS)
        self.assertAlmostEqual(math.atan2(p[1], p[0]), math.atan2(40., 30.))
        # Everything else arrives as asked
        self.assertEqual(p[2:], [5., 2.])
        self.assertEqual(speed, 50.)

    def test_a_target_just_off_the_centre_is_the_centre(self):
        p = make_planner().plan((30., 0.), pos(30., 0.),
                                pos(0.004, -0.003), 50.)[0][0]
        self.assertAlmostEqual(math.hypot(p[0], p[1]),
                               centre_path.PARK_RADIUS)
        self.assertAlmostEqual(p[1], 0.)

    def test_staying_on_the_axis_holds_the_park_point(self):
        park = (0., centre_path.PARK_RADIUS)
        moves = make_planner().plan(park, pos(0., 0., 10.),
                                    pos(0., 0., 30., 1.), 5.)
        self.assertEqual(moves, [([park[0], park[1], 30., 1.], 5.)])


class TestDepart(unittest.TestCase):
    def setUp(self):
        self.planner = make_planner()
        self.park = (centre_path.PARK_RADIUS, 0.)

    def test_along_the_ray_the_bed_faces_is_one_move(self):
        moves = self.planner.plan(self.park, pos(0., 0.),
                                  pos(40., 0., 20., 3.), 50.)
        self.assertEqual(moves, [(pos(40., 0., 20., 3.), 50.)])

    def test_to_a_new_ray_turns_the_bed_on_an_arc_first(self):
        radius = self.planner.reorient_radius
        moves = self.planner.plan(self.park, pos(0., 0., 10., 1.),
                                  pos(0., 40., 20., 3.), 50.)
        points = xy_of(moves)
        # Out along +x, a quarter turn in 10 degree chords, then up +y
        self.assertAlmostEqual(points[0][0], radius)
        self.assertAlmostEqual(points[0][1], 0.)
        self.assertEqual(len(moves), 1 + 9 + 1)
        for x, y in points[1:-1]:
            self.assertAlmostEqual(math.hypot(x, y), radius)
        self.assertAlmostEqual(points[-2][0], 0.)
        self.assertAlmostEqual(points[-2][1], radius)
        self.assertEqual(moves[-1], (pos(0., 40., 20., 3.), 50.))
        # z, e and the rest are held until the tool is on its way
        for p, speed in moves[:-1]:
            self.assertEqual(p[2:], [10., 1.])

    def test_the_arc_is_held_to_the_bed_limit(self):
        moves = self.planner.plan(self.park, pos(0., 0.), pos(-40., 0.), 50.)
        radius = self.planner.reorient_radius
        for p, speed in moves[1:-1]:
            self.assertAlmostEqual(
                speed, MAX_V * radius * centre_path.ARC_CHORD_DIP)
        # A half turn is 18 chords
        self.assertEqual(len(moves), 1 + 18 + 1)

    def test_a_slow_request_is_not_sped_up_on_the_arc(self):
        moves = self.planner.plan(self.park, pos(0., 0.), pos(0., 40.), .1)
        self.assertEqual(set(speed for p, speed in moves), {.1})

    def test_without_a_limit_the_arc_limits_itself(self):
        planner = make_planner(max_angular_v=0., max_angular_a=0.)
        moves = planner.plan(self.park, pos(0., 0.), pos(0., 40.), 50.)
        for p, speed in moves[1:-1]:
            self.assertAlmostEqual(
                speed, centre_path.DEFAULT_REORIENT_VELOCITY
                * planner.reorient_radius * centre_path.ARC_CHORD_DIP)

    def test_from_the_exact_centre_the_bed_faces_zero(self):
        # What set_position leaves behind, eg after G92 X0 Y0
        moves = self.planner.plan((0., 0.), pos(0., 0.), pos(0., 40.), 50.)
        self.assertAlmostEqual(moves[0][0][1], 0.)
        self.assertGreater(moves[0][0][0], 0.)
        self.assertEqual(len(moves), 1 + 9 + 1)


######################################################################
# Moves that do not start or end on the centre
######################################################################

class TestThrough(unittest.TestCase):
    def test_far_from_the_centre_nothing_changes(self):
        moves = make_planner().plan((40., -30.), pos(40., -30.),
                                    pos(40., 30.), 100.)
        self.assertEqual(moves, [(pos(40., 30.), 100.)])

    def test_radial_moves_pass_through(self):
        moves = make_planner().plan((10., 0.), pos(10., 0.),
                                    pos(80., 0.), 100.)
        self.assertEqual(moves, [(pos(80., 0.), 100.)])

    def test_a_crossing_travel_move_is_routed_through_the_centre(self):
        planner = make_planner()
        start, end = pos(-40., 0., 10., 0.), pos(0., 40., 30., 0.)
        # A chord that passes 0.004mm from the centre, inside the disc
        start, end = pos(-40., 0.004, 10.), pos(40., 0.004, 30.)
        moves = planner.plan(start[:2], start, end, 50.)
        points = xy_of(moves)
        # Arrive and park on the -x side, turn half a turn, leave on +x
        self.assertAlmostEqual(math.hypot(*points[0]),
                               centre_path.PARK_RADIUS)
        self.assertLess(points[0][0], 0.)
        self.assertEqual(moves[-1][0], end)
        # z is interpolated at the closest approach, and held for the turn
        for p, speed in moves[:-1]:
            self.assertAlmostEqual(p[2], 20.)
        # Nothing strays further from the commanded line than the arc
        for x, y in points:
            self.assertLessEqual(abs(y - 0.004),
                                 planner.reorient_radius + 1e-9)

    def test_a_crossing_print_move_is_left_for_the_check(self):
        start, end = pos(-40., 0., 10., 0.), pos(40., 0., 10., 5.)
        moves = make_planner().plan(start[:2], start, end, 50.)
        self.assertEqual(moves, [(end, 50.)])

    def test_the_policies_can_be_swapped(self):
        planner = make_planner(travel_policy='error', print_policy='bypass')
        travel = planner.plan((-40., 0.), pos(-40., 0.), pos(40., 0.), 50.)
        self.assertEqual(len(travel), 1)
        printing = planner.plan((-40., 0.), pos(-40., 0., 10., 0.),
                                pos(40., 0., 10., 5.), 50.)
        self.assertGreater(len(printing), 1)
        # A retraction is not printing
        self.assertEqual(planner.policy(pos(0., 0., 0., 5.),
                                        pos(1., 0., 0., 4.)), 'error')

    def test_a_near_miss_too_tight_to_slow_for_is_routed_too(self):
        # 0.05mm out needs 0.25mm/s, below the 0.5mm/s floor
        moves = make_planner().plan((-20., .05), pos(-20., .05),
                                    pos(20., .05), 50.)
        self.assertGreater(len(moves), 1)
        self.assertAlmostEqual(math.hypot(*xy_of(moves)[0]),
                               centre_path.PARK_RADIUS)

    def test_a_move_already_that_slow_is_left_alone(self):
        moves = make_planner().plan((-20., .05), pos(-20., .05),
                                    pos(20., .05), .2)
        self.assertEqual(moves, [(pos(20., .05), .2)])


class TestSplitSlow(unittest.TestCase):
    def split(self, start, end, speed=100.):
        return make_planner().plan(start[:2], start, end, speed)

    def test_only_the_close_part_runs_slowly(self):
        start, end = pos(1., -30., 0., 0.), pos(1., 30., 0., 6.)
        moves = self.split(start, end)
        self.assertGreater(len(moves), 1)
        self.assertEqual(moves[-1][0], end)
        # Collinear, in order, and carrying e along the line
        ys = [p[1] for p, speed in moves]
        self.assertEqual(ys, sorted(ys))
        for p, speed in moves:
            self.assertAlmostEqual(p[0], 1.)
            self.assertAlmostEqual(p[3], (p[1] + 30.) / 10.)
        # Each piece is only held to the limit at its own inner end, so
        # the outermost ones run at the full feedrate
        here = start
        limits = []
        for p, speed in moves:
            offset, r_min, u0, u1 = bed_centre.path_geometry(here, p)
            v_limit = bed_centre.limits_for_angular_rates(
                offset, r_min, u0, u1, MAX_V, 0.)[0]
            limits.append(v_limit)
            here = p
        self.assertAlmostEqual(limits[0], 100.)
        self.assertAlmostEqual(limits[-1], 100.)
        self.assertAlmostEqual(min(limits), MAX_V * 1.)
        # Piece by piece the limit doubles, then quadruples - the radius
        # doubles each time
        self.assertLess(len(moves), 2 * centre_path.MAX_SLOW_SEGMENTS + 1)

    def test_a_one_sided_approach_is_split_on_one_side(self):
        # Closest at the start, and moving away
        start, end = pos(1., 0.), pos(1., 30.)
        moves = self.split(start, end)
        self.assertGreater(len(moves), 1)
        ys = [p[1] for p, speed in moves]
        self.assertEqual(ys, sorted(ys))
        self.assertGreater(ys[0], 0.)

    def test_not_split_when_the_limit_does_not_bind(self):
        start, end = pos(1., -30.), pos(1., 30.)
        self.assertEqual(len(self.split(start, end, speed=4.)), 1)


class TestConfig(unittest.TestCase):
    def test_default_radius_clears_the_floor(self):
        planner = make_planner()
        self.assertGreaterEqual(
            MAX_V * planner.reorient_radius * centre_path.ARC_CHORD_DIP,
            MIN_V)
        self.assertGreaterEqual(planner.reorient_radius,
                                centre_path.MIN_REORIENT_RADIUS)
        # A generous bed gets the smallest radius the dead zone allows
        self.assertEqual(make_planner(max_angular_v=100.).reorient_radius,
                         centre_path.MIN_REORIENT_RADIUS)

    def test_bad_values_are_refused(self):
        with self.assertRaises(ValueError):
            make_planner(travel_policy='route')
        with self.assertRaises(ValueError):
            make_planner(reorient_radius=.02)
        with self.assertRaises(ValueError):
            # Held to 5 * 0.05 = 0.25mm/s, below the floor
            make_planner(reorient_radius=.05)


######################################################################
# The properties that matter
######################################################################

ANGLES = [0., .3, 1., math.pi / 2., 2.5, math.pi, -2.5, -math.pi / 2., -1.]

def around(angle, radius, z=10., e=0.):
    return pos(radius * math.cos(angle), radius * math.sin(angle), z, e)


class TestNoBedSteps(unittest.TestCase):
    def sequences(self):
        # Arrive from one ray, work on the axis, leave along another
        for a in ANGLES:
            for b in ANGLES:
                yield [around(a, 40.), pos(0., 0.), pos(0., 0., 30.),
                       around(b, 40., 30.)]
        # Straight across the centre, from every direction
        for a in ANGLES:
            yield [around(a, 40.), around(a + math.pi, 40.)]
        # Across it and back, and through it twice in a row
        yield [pos(40., 0.), pos(-40., 0.), pos(40., 0.)]
        yield [pos(40., .003), pos(-40., -.003), pos(0., 40.),
               pos(0., -40.)]
        # To the centre from a spot already on it
        yield [pos(0., 0.), pos(0.002, 0.), pos(0., 0.), pos(20., 20.)]

    def plan(self, targets):
        transform = Transform(make_planner(), targets[0])
        for target in targets[1:]:
            transform.move(target)
        return transform

    def test_a_planned_sequence_never_steps_the_bed(self):
        for targets in self.sequences():
            transform = self.plan(targets)
            trace = bed_trace(targets[0][:2],
                              [p for p, speed in transform.sent])
            self.assertLess(largest_step(trace), STEP, msg=targets)

    def test_the_same_g_code_sent_straight_does(self):
        # The bug being fixed, made visible: arrive on the centre along
        # -x, then leave along +y
        trace = bed_trace((40., 0.), [pos(0., 0.), pos(0., 40.)])
        self.assertGreater(largest_step(trace), 1.)
        trace = bed_trace((40., 0.), [pos(-40., 0.)])
        self.assertGreater(largest_step(trace), 3.)

    def test_every_planned_move_gets_past_the_check(self):
        chk = ps.PolarSingularity.__new__(ps.PolarSingularity)
        chk.max_angular_v, chk.max_angular_a = MAX_V, MAX_A
        chk.min_velocity = MIN_V
        chk.last_radius = chk.last_swept = chk.last_velocity_limit = 0.
        for targets in self.sequences():
            here = targets[0]
            for p, speed in self.plan(targets).sent:
                move = FakeMove(here, p, speed)
                if move.axes_d[0] or move.axes_d[1]:
                    chk._check_move(move)
                    # And once held, the bed stays inside its limit
                    offset, r_min, u0, u1 = bed_centre.path_geometry(here, p)
                    self.assertLessEqual(bed_centre.peak_angular_velocity(
                        move.cruise(), offset, r_min), MAX_V + 1e-9)
                here = p

    def test_the_toolhead_ends_where_it_was_asked_to(self):
        for targets in self.sequences():
            transform = self.plan(targets)
            final = targets[-1]
            if centre_path.at_centre(final):
                self.assertTrue(centre_path.at_centre(transform.machine))
            else:
                self.assertEqual(transform.machine, final)


class FakeMove:
    def __init__(self, start_pos, end_pos, velocity):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.axes_d = [e - s for s, e in zip(self.start_pos, self.end_pos)]
        self.max_cruise_v2 = velocity ** 2
    def limit_speed(self, speed, accel):
        self.max_cruise_v2 = min(self.max_cruise_v2, speed ** 2)
    def move_error(self, msg="Move out of range"):
        return AssertionError("refused %s -> %s: %s"
                              % (self.start_pos, self.end_pos, msg))
    def cruise(self):
        return math.sqrt(self.max_cruise_v2)


class TestMatchesTheCTest(unittest.TestCase):
    # test_centre_path_step_generation() in test_kin_6axis.c builds these
    # sequences by hand and runs them through the real step generator and
    # step compressor.  They have to be what the planner really emits.
    def c_departure(self, from_angle, to_angle, target_r):
        radius = .125
        turn = math.atan2(math.sin(to_angle - from_angle),
                          math.cos(to_angle - from_angle))
        chords = int(math.ceil(abs(turn) / (math.pi / 18.) - 1e-9))
        points = [(radius * math.cos(from_angle),
                   radius * math.sin(from_angle))]
        for i in range(1, chords + 1):
            angle = from_angle + turn * i / chords
            points.append((radius * math.cos(angle),
                           radius * math.sin(angle)))
        points.append((target_r * math.cos(to_angle),
                       target_r * math.sin(to_angle)))
        return points

    def check_same(self, sent, expected):
        self.assertEqual(len(sent), len(expected))
        for (x, y), (ex, ey) in zip(sent, expected):
            self.assertAlmostEqual(x, ex, places=12)
            self.assertAlmostEqual(y, ey, places=12)

    def test_the_constants(self):
        planner = make_planner()
        self.assertEqual(centre_path.PARK_RADIUS, 1e-4)
        self.assertEqual(planner.reorient_radius, .125)
        self.assertEqual(centre_path.MAX_ARC_CHORD, math.pi / 18.)

    def test_leaving_on_a_new_ray(self):
        transform = Transform(make_planner(), pos(0., 40.))
        transform.move(pos(0., 0.))
        transform.move(pos(40., 0.))
        self.check_same(xy_of(transform.sent),
                        [(0., 1e-4)]
                        + self.c_departure(math.pi / 2., 0., 40.))

    def test_straight_across(self):
        transform = Transform(make_planner(), pos(40., 0.))
        transform.move(pos(-40., 0.))
        self.check_same(xy_of(transform.sent),
                        [(1e-4, 0.)]
                        + self.c_departure(0., math.pi, 40.))


######################################################################
# The g-code transform
######################################################################

class FakeToolhead:
    def __init__(self, position):
        self.position = list(position)
        self.sent = []
    def get_position(self):
        return list(self.position)
    def move(self, newpos, speed):
        self.sent.append((list(newpos), speed))
        self.position = list(newpos)


class TestTransform(unittest.TestCase):
    def build(self, position):
        obj = ps.PolarSingularity.__new__(ps.PolarSingularity)
        obj.planner = make_planner()
        obj.toolhead = obj.next_transform = FakeToolhead(position)
        obj.last_position = [0., 0., 0., 0.]
        obj.get_position()
        return obj

    def test_a_parked_tool_reports_the_centre(self):
        obj = self.build(pos(50., 0.))
        obj.move(pos(0., 0., 10., 0.), 50.)
        machine = obj.toolhead.get_position()
        self.assertNotEqual(machine[:2], [0., 0.])
        self.assertEqual(obj.get_position(), pos(0., 0., 10., 0.))

    def test_elsewhere_the_position_is_passed_through(self):
        obj = self.build(pos(50., 0.))
        self.assertEqual(obj.get_position(), pos(50., 0.))
        obj.move(pos(20., 30., 5., 1.), 50.)
        self.assertEqual(obj.get_position(), pos(20., 30., 5., 1.))

    def test_planning_follows_the_toolhead_not_the_g_code(self):
        # Parked on the +y ray: leaving along +y needs no turn
        obj = self.build(pos(0., centre_path.PARK_RADIUS))
        obj.move(pos(0., 40.), 50.)
        self.assertEqual(obj.toolhead.sent, [(pos(0., 40.), 50.)])
        self.assertEqual(obj.last_position, pos(0., 40.))

    def test_a_whole_arrival_and_departure(self):
        obj = self.build(pos(40., 0.))
        obj.move(pos(0., 0.), 50.)
        obj.move(pos(0., 0., 30.), 10.)
        obj.move(pos(0., 40., 30.), 50.)
        sent = [p for p, speed in obj.toolhead.sent]
        self.assertEqual(sent[-1], pos(0., 40., 30.))
        self.assertLess(largest_step(bed_trace((40., 0.), sent)), STEP)


if __name__ == '__main__':
    unittest.main()
