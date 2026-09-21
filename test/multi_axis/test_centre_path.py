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
    # Blending is off unless a test asks for it, so the bed turns on the
    # arc - the plans test_kin_6axis.c builds by hand
    args = dict(max_angular_v=MAX_V, max_angular_a=MAX_A,
                min_velocity=MIN_V, blend_tolerance=0.)
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
        self.branch = 1
        self.branch_flip = False
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
    def get_branch(self):
        return 1
    def move(self, newpos, speed):
        self.sent.append((list(newpos), speed))
        self.position = list(newpos)


class TestTransform(unittest.TestCase):
    def build(self, position):
        obj = ps.PolarSingularity.__new__(ps.PolarSingularity)
        obj.planner = make_planner()
        obj.toolhead = obj.next_transform = FakeToolhead(position)
        obj.last_position = [0., 0., 0., 0.]
        obj.pending, obj.reactor, obj.hold_timer = [], None, None
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


######################################################################
# Crossing on an arm that can, and standing the head up
######################################################################

class Machine:
    # The toolhead and the corertheta kinematics reduced to what a plan
    # relies on: where the tool is, the branch it is on, and the flip the
    # kinematics marks a move with when it carries on through the centre
    def __init__(self, planner, start, can_cross=True, branch=1):
        self.planner = planner
        self.pos = list(start)
        self.last = list(start)
        self.branch = branch
        self.can_cross = can_cross
        self.moves = []
    def send(self, p, speed):
        moving = p[0] != self.pos[0] or p[1] != self.pos[1]
        flip = (self.can_cross and moving and bed_centre.flips_through_centre(
            self.pos, p, self.branch))
        self.moves.append((list(self.pos), list(p), self.branch, flip, speed))
        self.branch = bed_centre.end_branch(self.branch, flip)
        self.pos = list(p)
    def move(self, newpos, speed=100.):
        for p, s in self.planner.plan(self.pos[:2], self.last, newpos, speed,
                                      self.branch):
            self.send(p, s)
        self.last = list(newpos)

def branch_trace(moves):
    # The bed angles the step generator would command through a sequence
    # of (start, end, branch, flip, speed) moves, at rest between them
    angles = []
    for start, end, branch, flip, speed in moves:
        dx, dy = end[0] - start[0], end[1] - start[1]
        travel = (dx, dy) if (dx or dy) else None
        angles.append(bed_centre.bed_angle(start[0], start[1], None, branch))
        n = _samples(start, end)
        for i in range(n + 1):
            t = float(i) / n
            x, y = start[0] + t * dx, start[1] + t * dy
            angles.append(bed_centre.bed_angle(
                x, y, travel,
                bed_centre.sample_branch(x, y, travel, branch, flip)))
        angles.append(bed_centre.bed_angle(
            end[0], end[1], None, bed_centre.end_branch(branch, flip)))
    return angles

def cross_planner(**kw):
    args = dict(travel_policy='cross', print_policy='cross', can_cross=True)
    args.update(kw)
    return make_planner(**args)

def distance_to_line(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / math.hypot(dx, dy)


class TestCross(unittest.TestCase):
    def run_moves(self, planner, targets, can_cross=True):
        machine = Machine(planner, targets[0], can_cross)
        for target in targets[1:]:
            machine.move(target)
        return machine

    def test_only_on_an_arm_that_can(self):
        with self.assertRaises(ValueError):
            make_planner(travel_policy='cross')
        with self.assertRaises(ValueError):
            make_planner(print_policy='cross')
        cross_planner()

    def test_straight_through_keeps_its_path_and_the_bed_still(self):
        for a in ANGLES:
            start, end = around(a, 40.), around(a + math.pi, 40.)
            machine = self.run_moves(cross_planner(), [start, end])
            # Stop short of the centre and carry straight on - no arc
            self.assertEqual(len(machine.moves), 2, msg=a)
            for s, e, branch, flip, speed in machine.moves:
                self.assertLess(distance_to_line(e, start, end), 1e-9)
                self.assertEqual(speed, 100.)
            self.assertEqual([m[3] for m in machine.moves], [False, True])
            self.assertEqual(machine.branch, -1)
            trace = branch_trace(machine.moves)
            facing = trace[0]
            for angle in trace:
                self.assertLess(abs(centre_path.wrap_angle(angle - facing)),
                                1e-6, msg=a)

    def test_there_and_back(self):
        machine = self.run_moves(cross_planner(),
                                 [pos(40., 0.), pos(-40., 0.), pos(40., 0.)])
        self.assertEqual(machine.branch, 1)
        self.assertLess(largest_step(branch_trace(machine.moves)), 1e-6)

    def test_printing_straight_through(self):
        machine = self.run_moves(cross_planner(),
                                 [pos(40., 0., 10., 0.),
                                  pos(-40., 0., 10., 4.)])
        self.assertEqual(len(machine.moves), 2)
        # Extruding the whole way, in proportion
        e_park = machine.moves[0][1][3]
        self.assertAlmostEqual(e_park, 2., places=4)
        self.assertEqual(machine.moves[1][1][3], 4.)

    def test_a_near_miss_turns_the_bed_a_little(self):
        start, end = pos(-40., .005), pos(40., .005)
        machine = self.run_moves(cross_planner(), [start, end])
        self.assertLess(largest_step(branch_trace(machine.moves)), STEP)
        self.assertEqual(machine.branch, -1)
        # A single chord of arc, at the reorientation radius
        radius = machine.planner.reorient_radius
        for s, e, branch, flip, speed in machine.moves:
            self.assertLessEqual(distance_to_line(e, start, end),
                                 radius + 1e-9)
        self.assertEqual(len(machine.moves), 4)

    def test_leaving_the_centre_the_nearer_way(self):
        # Facing 0, for a target at 100 degrees: a 100 degree turn on the
        # usual branch, or 80 the other way with the arm on the far side
        planner = cross_planner()
        machine = Machine(planner, pos(centre_path.PARK_RADIUS, 0.))
        machine.last = pos(0., 0.)
        machine.move(around(math.radians(100.), 40.))
        self.assertEqual(machine.branch, -1)
        self.assertLess(largest_step(branch_trace(machine.moves)), STEP)
        # Eight chords, not ten
        self.assertEqual(len(machine.moves), 1 + 8 + 1)
        # And out on the far side of the centre from where it stood
        self.assertLess(machine.moves[0][1][0], 0.)

    def test_a_crossing_arm_can_still_be_told_to_bypass(self):
        planner = make_planner(can_cross=True)
        machine = self.run_moves(planner, [pos(40., 0.), pos(-40., 0.)])
        self.assertEqual(machine.branch, 1)
        self.assertFalse(any(m[3] for m in machine.moves))
        self.assertLess(largest_step(branch_trace(machine.moves)), STEP)

    def test_every_crossing_move_gets_past_the_check(self):
        chk = ps.PolarSingularity.__new__(ps.PolarSingularity)
        chk.max_angular_v, chk.max_angular_a = MAX_V, MAX_A
        chk.min_velocity = MIN_V
        chk.last_radius = chk.last_swept = chk.last_velocity_limit = 0.
        sequences = [[around(a, 40.), around(a + math.pi, 40.),
                      pos(0., 0.), around(b, 40.)]
                     for a in ANGLES for b in ANGLES[::2]]
        sequences.append([pos(-40., .005), pos(40., .005), pos(0., 30.)])
        for targets in sequences:
            machine = self.run_moves(cross_planner(), targets)
            self.assertLess(largest_step(branch_trace(machine.moves)),
                            STEP, msg=targets)
            for s, e, branch, flip, speed in machine.moves:
                move = FakeMove(s, e, speed)
                move.branch, move.branch_flip = branch, flip
                if move.axes_d[0] or move.axes_d[1]:
                    chk._check_move(move)


def pos5(x, y, z=10., e=0., b=0.):
    return [x, y, z, e, b]


class TestTiltIsNeverTouched(unittest.TestCase):
    # Nothing the planner does changes the head's tilt: B goes wherever
    # the g-code sends it, interpolated along the way like z and e
    def plan(self, planner, start, end):
        return planner.plan(start[:2], start, end, 50.)

    def test_a_tilted_bypass_keeps_its_tilt(self):
        moves = self.plan(make_planner(), pos5(-40., 0., b=10.),
                          pos5(40., 0., b=10.))
        self.assertGreater(len(moves), 2)
        self.assertTrue(all(p[4] == 10. for p, speed in moves))

    def test_a_changing_tilt_is_interpolated_not_stood_up(self):
        start, end = pos5(-40., 0., b=10.), pos5(40., 0., b=-6.)
        moves = self.plan(make_planner(), start, end)
        tilts = [p[4] for p, speed in moves]
        # Halfway at the centre, held there while the bed turns, then on
        self.assertAlmostEqual(tilts[0], 2.)
        self.assertEqual(tilts, sorted(tilts, reverse=True))
        self.assertEqual(moves[-1][0], end)
        self.assertNotIn(0., tilts)

    def test_a_tilted_print_bypass_is_planned_like_any_other(self):
        planner = make_planner(print_policy='bypass')
        tilted = self.plan(planner, pos5(-40., 0., e=0., b=10.),
                           pos5(40., 0., e=2., b=10.))
        upright = self.plan(planner, pos5(-40., 0., e=0.),
                            pos5(40., 0., e=2.))
        self.assertEqual(xy_of(tilted), xy_of(upright))
        self.assertTrue(all(p[4] == 10. for p, speed in tilted))

    def test_crossing_holds_the_tilt(self):
        moves = self.plan(cross_planner(), pos5(-40., 0., e=0., b=10.),
                          pos5(40., 0., e=2., b=10.))
        self.assertEqual(len(moves), 2)
        self.assertTrue(all(p[4] == 10. for p, speed in moves))


class FakeCrossingToolhead(FakeToolhead):
    # corertheta's part: marking a move that carries on through the centre.
    # Records each move as (start, end, branch, flip, speed).
    def __init__(self, position, can_cross=True):
        FakeToolhead.__init__(self, position)
        self.branch = 1
        self.can_cross = can_cross
        self.moves = []
    def get_branch(self):
        return self.branch
    def move(self, newpos, speed):
        moving = (newpos[0] != self.position[0]
                  or newpos[1] != self.position[1])
        flip = bool(self.can_cross and moving
                    and bed_centre.flips_through_centre(
                        self.position, newpos, self.branch))
        self.moves.append((list(self.position), list(newpos), self.branch,
                           flip, speed))
        self.branch = bed_centre.end_branch(self.branch, flip)
        FakeToolhead.move(self, newpos, speed)


class FakePrinter:
    class command_error(Exception):
        pass


def build_transform(planner, position, can_cross=True):
    obj = ps.PolarSingularity.__new__(ps.PolarSingularity)
    obj.printer = FakePrinter()
    obj.planner = planner
    obj.toolhead = obj.next_transform = FakeCrossingToolhead(position,
                                                             can_cross)
    obj.last_position = [0., 0., 0., 0.]
    obj.pending, obj.reactor, obj.hold_timer = [], None, None
    obj.get_position()
    return obj


class TestCrossingTransform(unittest.TestCase):
    def test_the_toolhead_branch_reaches_the_planner(self):
        obj = build_transform(cross_planner(), pos(40., 0.))
        obj.move(pos(-40., 0.), 50.)
        self.assertEqual(obj.toolhead.branch, -1)
        # Back through the centre from the far side, still in a line
        obj.move(pos(40., 0.), 50.)
        self.assertEqual(obj.toolhead.branch, 1)
        self.assertEqual(len(obj.toolhead.sent), 4)


######################################################################
# Turning the bed on the move, and looking ahead to do it
######################################################################

TOLERANCE = centre_path.DEFAULT_BLEND_TOLERANCE

def blend_planner(**kw):
    # With the bed's angular acceleration unchecked, as the example config
    # ships it until it has been measured
    args = dict(travel_policy='cross', print_policy='cross', can_cross=True,
                blend_tolerance=TOLERANCE, max_angular_a=0.)
    args.update(kw)
    return make_planner(**args)

def _segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx*dx + dy*dy
    t = 0.
    if length2:
        t = min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2,
                    0.), 1.)
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)

def deviation(moves, commanded):
    # The furthest the path the moves describe strays from the commanded
    # polyline, sampled along each move
    worst = 0.
    for start, end, branch, flip, speed in moves:
        for i in range(33):
            t = i / 32.
            p = (start[0] + t * (end[0] - start[0]),
                 start[1] + t * (end[1] - start[1]))
            worst = max(worst, min(_segment_distance(p, a, b) for a, b
                                   in zip(commanded, commanded[1:])))
    return worst

def make_checker(max_a):
    chk = ps.PolarSingularity.__new__(ps.PolarSingularity)
    chk.max_angular_v, chk.max_angular_a = MAX_V, max_a
    chk.min_velocity = MIN_V
    chk.last_radius = chk.last_swept = chk.last_velocity_limit = 0.
    return chk

def check_all(test, moves, max_a=0., exact=True):
    # Every move gets past the move check and never steps the bed.  With
    # 'exact', every move already runs at the speed the check allows it:
    # the plan held each chord to the bed's limits itself.
    chk = make_checker(max_a)
    for start, end, branch, flip, speed in moves:
        move = FakeMove(start, end, speed)
        move.branch, move.branch_flip = branch, flip
        if move.axes_d[0] or move.axes_d[1]:
            chk._check_move(move)
            if exact:
                test.assertAlmostEqual(move.cruise(), speed,
                                       delta=1e-6 * speed, msg=(start, end))
    test.assertLess(largest_step(branch_trace(moves)), STEP)

def corner(deflection, reach=20.):
    # In along -x onto the centre, out again turned 'deflection' radians
    # from carrying straight on - so pi is straight back the way it came
    out = math.pi - deflection
    return (pos(reach, 0., 10., 0.), pos(0., 0., 10., 2.),
            pos(reach * math.cos(out), reach * math.sin(out), 10., 4.))

def run_corner(planner, deflection, can_cross=True, speed=60.):
    start, middle, end = corner(deflection)
    obj = build_transform(planner, start, can_cross)
    obj.move(middle, speed)
    obj.move(end, speed)
    obj.release()
    return obj, [start[:2], middle[:2], end[:2]]


class TestBlend(unittest.TestCase):
    def test_a_corner_through_the_centre_keeps_moving(self):
        for degrees in (3., 10., 25., 45., 70., 85., 89.):
            obj, commanded = run_corner(blend_planner(),
                                        math.radians(degrees))
            moves = obj.toolhead.moves
            msg = '%g degrees' % (degrees,)
            check_all(self, moves)
            self.assertLessEqual(deviation(moves, commanded), TOLERANCE,
                                 msg=msg)
            # Onto the far branch, and extruding on every move - never
            # standing still with e held, which is where a blob comes from
            self.assertEqual(obj.toolhead.branch, -1, msg=msg)
            for start, end, branch, flip, speed in moves:
                self.assertGreater(end[3], start[3], msg=msg)
                self.assertGreaterEqual(speed, MIN_V, msg=msg)
            self.assertEqual(moves[-1][1], corner(math.radians(degrees))[2])

    def test_a_shallow_corner_runs_at_full_speed(self):
        obj, commanded = run_corner(blend_planner(), math.radians(10.))
        self.assertEqual(set(m[4] for m in obj.toolhead.moves), {60.})

    def test_sharper_corners_run_slower(self):
        slowest = []
        for degrees in (20., 40., 80.):
            obj, commanded = run_corner(blend_planner(),
                                        math.radians(degrees))
            slowest.append(min(m[4] for m in obj.toolhead.moves))
        self.assertEqual(slowest, sorted(slowest, reverse=True))

    def test_the_tilt_is_what_was_commanded(self):
        # b falls linearly with e along the commanded path, so wherever
        # the blend puts the tool the two must still agree
        start, middle, end = (pos5(20., 0., e=0., b=10.),
                              pos5(0., 0., e=2., b=4.),
                              pos5(-14.14, 14.14, e=4., b=-2.))
        obj = build_transform(blend_planner(), start)
        obj.move(middle, 60.)
        obj.move(end, 60.)
        for s, e, branch, flip, speed in obj.toolhead.moves:
            self.assertAlmostEqual(e[4], 10. - 3. * e[3], places=9)

    def test_a_reversal_blends_on_an_arm_that_cannot_cross(self):
        # Back out 20 degrees off the way it came: the bed turns 20
        # degrees and the tool never leaves the near branch
        planner = blend_planner(travel_policy='bypass', print_policy='error',
                                can_cross=False)
        obj, commanded = run_corner(planner, math.radians(160.),
                                    can_cross=False)
        moves = obj.toolhead.moves
        check_all(self, moves)
        self.assertFalse(any(m[3] for m in moves))
        self.assertLessEqual(deviation(moves, commanded), TOLERANCE)
        for start, end, branch, flip, speed in moves:
            self.assertGreater(end[3], start[3])

    def test_too_sharp_to_blend_turns_on_the_arc(self):
        planner = blend_planner(blend_tolerance=.002)
        obj, commanded = run_corner(planner, math.radians(85.))
        moves = obj.toolhead.moves
        check_all(self, moves)
        radius = planner.reorient_radius
        arc = [m for m in moves
               if abs(math.hypot(m[1][0], m[1][1]) - radius) < 1e-9]
        self.assertGreater(len(arc), 5)
        # The arc holds e, as it always has
        self.assertEqual(len(set(m[1][3] for m in arc)), 1)


class TestLookahead(unittest.TestCase):
    def test_the_move_onto_the_centre_is_held_back(self):
        obj = build_transform(blend_planner(), pos(20., 0.))
        obj.move(pos(0., 0., 10., 2.), 60.)
        # Sent as far as blend_radius short of the centre
        here = obj.toolhead.position
        self.assertAlmostEqual(here[0], centre_path.DEFAULT_BLEND_RADIUS)
        self.assertAlmostEqual(here[3], 1.8)
        self.assertTrue(obj.pending)
        self.assertEqual(obj.last_position, pos(0., 0., 10., 2.))

    def test_a_short_move_is_held_whole(self):
        obj = build_transform(blend_planner(), pos(1., 0.))
        obj.move(pos(0., 0.), 60.)
        self.assertEqual(obj.toolhead.sent, [])

    def test_a_release_parks_the_held_tail(self):
        obj = build_transform(blend_planner(), pos(20., 0.))
        obj.move(pos(0., 0., 10., 2.), 60.)
        obj.release()
        self.assertEqual(obj.pending, [])
        self.assertAlmostEqual(obj.toolhead.position[0],
                               centre_path.PARK_RADIUS)
        self.assertEqual(obj.toolhead.position[2:], [10., 2.])
        # And a second release has nothing to send
        count = len(obj.toolhead.sent)
        obj.release()
        self.assertEqual(len(obj.toolhead.sent), count)

    def test_reading_the_position_releases_it(self):
        obj = build_transform(blend_planner(), pos(20., 0.))
        obj.move(pos(0., 0., 10., 2.), 60.)
        self.assertEqual(obj.get_position(), pos(0., 0., 10., 2.))
        self.assertEqual(obj.pending, [])

    def test_the_timer_releases_it(self):
        obj = build_transform(blend_planner(), pos(20., 0.))
        obj.move(pos(0., 0.), 60.)
        class Reactor:
            NEVER = 9999999999999999.
        obj.reactor = Reactor()
        self.assertEqual(obj._hold_timeout(1.), Reactor.NEVER)
        self.assertEqual(obj.pending, [])

    def test_staying_on_the_centre_releases_then_stays(self):
        obj = build_transform(blend_planner(), pos(20., 0.))
        obj.move(pos(0., 0., 10., 2.), 60.)
        obj.move(pos(0., 0., 12., 2.), 5.)
        self.assertEqual(obj.pending, [])
        self.assertAlmostEqual(obj.toolhead.position[0],
                               centre_path.PARK_RADIUS)
        self.assertEqual(obj.toolhead.position[2], 12.)

    def test_after_a_release_the_way_out_blends_alone(self):
        # Stopped on the centre - a layer change, say - then out along a
        # new line: the bed turns as the tool leaves, still extruding
        planner = blend_planner()
        obj = build_transform(planner, pos(20., 0., 10., 0.))
        obj.move(pos(0., 0., 10., 2.), 60.)
        obj.move(pos(0., 0., 10.2, 2.), 5.)
        mark = len(obj.toolhead.moves)
        out = math.radians(30.)
        end = pos(20. * math.cos(out), 20. * math.sin(out), 10.2, 4.)
        obj.move(end, 60.)
        moves = obj.toolhead.moves
        check_all(self, moves)
        leaving = moves[mark:]
        self.assertGreater(len(leaving), 3)
        for start, stop, branch, flip, speed in leaving:
            self.assertGreater(stop[3], start[3])
        self.assertLessEqual(
            deviation(leaving, [(0., 0.), (end[0], end[1])]), TOLERANCE)

    def test_blending_off_holds_nothing(self):
        planner = blend_planner(blend_tolerance=0.)
        self.assertIsNone(planner.hold_point(pos(20., 0.),
                                             pos(0., 0.)))
        obj = build_transform(planner, pos(20., 0.))
        obj.move(pos(0., 0.), 60.)
        self.assertEqual(obj.pending, [])
        self.assertAlmostEqual(obj.toolhead.position[0],
                               centre_path.PARK_RADIUS)


class TestNearMissThroughTheCentre(unittest.TestCase):
    def test_a_close_print_move_goes_through_without_stopping(self):
        # Too close to slow for (5 * 0.02 = 0.1 mm/s is below the floor),
        # so routed through the centre - blended, where it used to stop
        # there and turn a fraction of a degree on the circle
        start, end = pos(-20., .02, 10., 0.), pos(20., .02, 10., 4.)
        obj = build_transform(blend_planner(), start)
        obj.move(end, 60.)
        moves = obj.toolhead.moves
        check_all(self, moves)
        # Onto the far branch at full speed, extruding the whole way
        self.assertEqual(obj.toolhead.branch, -1)
        self.assertEqual(set(m[4] for m in moves), {60.})
        self.assertLessEqual(deviation(moves, [start[:2], end[:2]]),
                             TOLERANCE)
        self.assertEqual(moves[-1][1], end)

    def test_a_slowed_move_within_tolerance_goes_through_too(self):
        # A 0.2 mm tolerance takes in a miss the bed's limit would only
        # slow, to 5 * 0.15 = 0.75 mm/s, rather than refuse
        planner = blend_planner(blend_tolerance=.2)
        start, end = pos(-20., .15, 10., 0.), pos(20., .15, 10., 4.)
        obj = build_transform(planner, start)
        obj.move(end, 60.)
        moves = obj.toolhead.moves
        check_all(self, moves)
        self.assertEqual(obj.toolhead.branch, -1)
        self.assertGreater(min(m[4] for m in moves), 10.)
        self.assertLessEqual(deviation(moves, [start[:2], end[:2]]), .2)

    def test_further_out_it_is_only_slowed(self):
        start, end = pos(-20., .2, 10., 0.), pos(20., .2, 10., 4.)
        obj = build_transform(blend_planner(), start)
        obj.move(end, 60.)
        self.assertEqual(obj.toolhead.branch, 1)
        for s, e, branch, flip, speed in obj.toolhead.moves:
            self.assertAlmostEqual(e[1], .2)

    def test_not_without_cross(self):
        planner = blend_planner(travel_policy='bypass', print_policy='error')
        moves = planner.plan((-20., .02), pos(-20., .02, 10., 0.),
                             pos(20., .02, 10., 4.), 60.)
        for p, speed in moves:
            self.assertAlmostEqual(p[1], .02)


class TestNoBedStepsBlending(unittest.TestCase):
    # TestNoBedSteps again, through the transform, with the lookahead -
    # and with an angular acceleration limit, which the plans do not
    # predict exactly but must still get past
    def test_every_sequence(self):
        for can_cross in (True, False):
            for max_a in (0., MAX_A):
                self.run_sequences(can_cross, max_a)

    def run_sequences(self, can_cross, max_a):
        kw = dict(max_angular_a=max_a)
        if not can_cross:
            kw.update(travel_policy='bypass', print_policy='error',
                      can_cross=False)
        planner = blend_planner(**kw)
        for targets in TestNoBedSteps().sequences():
            obj = build_transform(planner, targets[0], can_cross)
            for target in targets[1:]:
                obj.move(target, 100.)
            obj.release()
            check_all(self, obj.toolhead.moves, max_a, exact=not max_a)
            final = targets[-1]
            if centre_path.at_centre(final):
                self.assertTrue(centre_path.at_centre(
                    obj.toolhead.position))
            else:
                self.assertEqual(obj.toolhead.position, final)


class TestBlendConfig(unittest.TestCase):
    def test_bad_values_are_refused(self):
        with self.assertRaises(ValueError):
            blend_planner(blend_tolerance=-1.)
        with self.assertRaises(ValueError):
            blend_planner(blend_radius=.05)

######################################################################
# An arm that reaches only a little way past the centre
######################################################################

REACH = 5.

def reach_planner(**kw):
    kw.setdefault('far_reach', REACH)
    return blend_planner(**kw)

def check_reach(test, moves, reach=REACH):
    # Nothing is sent to the far side further out than the arm goes
    for start, end, branch, flip, speed in moves:
        for pos, br in ((start, branch),
                        (end, bed_centre.end_branch(branch, flip))):
            if br < 0:
                test.assertLessEqual(math.hypot(pos[0], pos[1]),
                                     reach + 1e-9, msg=(start, end))

def run_moves(planner, start, targets, release=True):
    obj = build_transform(planner, start)
    for target, speed in targets:
        obj.move(target, speed)
    if release:
        obj.release()
    return obj

def e_held(moves):
    # Moves in x/y that carry no extrusion
    return [m for m in moves if m[1][3] == m[0][3]
            and (m[1][0] != m[0][0] or m[1][1] != m[0][1])]


class TestShortReach(unittest.TestCase):
    def test_a_line_across_the_bed_turns_the_bed_on_the_centre(self):
        # The far side cannot hold a tool 40mm out, so the bed makes its
        # half turn on the centre, as on an arm that cannot cross
        obj = run_moves(reach_planner(), pos(40., 0., 10., 0.),
                        [(pos(-40., 0., 10., 4.), 60.)])
        moves = obj.toolhead.moves
        check_all(self, moves)
        check_reach(self, moves)
        self.assertFalse(any(m[3] for m in moves))
        self.assertEqual(obj.toolhead.position, pos(-40., 0., 10., 4.))

    def test_a_short_excursion_crosses_and_comes_back(self):
        planner = reach_planner()
        obj = run_moves(planner, pos(40., 0., 10., 0.),
                        [(pos(-3., 0., 10., 2.), 60.)], release=False)
        # Held until it is known the tool comes back
        self.assertEqual(obj.toolhead.moves, [])
        self.assertTrue(obj.pending)
        obj.move(pos(40., 0., 10., 4.), 60.)
        self.assertEqual(obj.pending, [])
        moves = obj.toolhead.moves
        check_all(self, moves)
        check_reach(self, moves)
        # Across and back on the far side, the bed never turning, at full
        # speed and extruding throughout
        self.assertEqual([m[3] for m in moves].count(True), 2)
        self.assertEqual(obj.toolhead.branch, 1)
        trace = branch_trace(moves)
        self.assertLess(max(abs(centre_path.wrap_angle(a - trace[0]))
                            for a in trace), 1e-6)
        self.assertEqual(set(m[4] for m in moves), {60.})
        self.assertEqual(e_held(moves), [])

    def test_a_print_that_would_be_stranded_stays_on_the_near_side(self):
        # Across to 3mm past the centre, then on along the far side out
        # of reach: the far side would strand it, so the crossing is
        # planned again with a half turn on the centre instead
        obj = run_moves(reach_planner(), pos(40., 0., 10., 0.),
                        [(pos(-3., 0., 10., 2.), 60.),
                         (pos(-3., 10., 10., 4.), 60.)], release=False)
        self.assertEqual(obj.pending, [])
        moves = obj.toolhead.moves
        # Passing a few mm from the centre on the near side is
        # slowed by the move check, as any such move is
        check_all(self, moves, exact=False)
        check_reach(self, moves)
        self.assertFalse(any(m[3] for m in moves))
        # And the second line is printed where it was asked for
        last = [m for m in moves if m[1][3] > 2.]
        for start, end, branch, flip, speed in last:
            self.assertAlmostEqual(end[0], -3.)

    def test_a_stranded_travel_detours_back_through_the_centre(self):
        obj = run_moves(reach_planner(), pos(40., 0.),
                        [(pos(-3., 0.), 100.), (pos(-3., 10.), 100.)],
                        release=False)
        moves = obj.toolhead.moves
        check_all(self, moves)
        check_reach(self, moves)
        # Onto the far side and back, with no half turn of the bed
        self.assertEqual([m[3] for m in moves].count(True), 2)
        self.assertEqual(obj.toolhead.branch, 1)
        self.assertEqual(obj.toolhead.position, pos(-3., 10.))

    def test_a_release_gives_up_the_far_side(self):
        obj = run_moves(reach_planner(), pos(40., 0., 10., 0.),
                        [(pos(-3., 0., 10., 2.), 60.)])
        moves = obj.toolhead.moves
        check_all(self, moves)
        self.assertFalse(any(m[3] for m in moves))
        self.assertEqual(obj.toolhead.position, pos(-3., 0., 10., 2.))

    def test_a_bend_through_the_centre_that_comes_back(self):
        # In along -x, out 7 degrees off, 4mm past the centre, and back
        # through it: the far side carries both bends
        out = math.radians(180. - 7.)
        far = pos(4. * math.cos(out), 4. * math.sin(out), 10., 3.)
        back = pos(-20. * math.cos(out), -20. * math.sin(out), 10., 5.)
        obj = run_moves(reach_planner(), pos(20., 0., 10., 0.),
                        [(pos(0., 0., 10., 2.), 60.), (far, 60.),
                         (back, 60.)], release=False)
        moves = obj.toolhead.moves
        check_all(self, moves)
        check_reach(self, moves)
        self.assertEqual(obj.toolhead.branch, 1)
        self.assertEqual(e_held(moves), [])
        self.assertTrue(any(m[3] for m in moves))

    def test_a_long_chain_on_the_far_side_is_given_up(self):
        targets = [(pos(-2., 0., 10., 1.), 60.)]
        e = 1.
        for i in range(centre_path.MAX_FAR_CHAIN + 2):
            e += .1
            targets.append((pos(-2., .5 * (i % 2), 10., e), 60.))
        obj = run_moves(reach_planner(), pos(40., 0., 10., 0.), targets,
                        release=False)
        moves = obj.toolhead.moves
        # Passing a few mm from the centre on the near side is
        # slowed by the move check, as any such move is
        check_all(self, moves, exact=False)
        check_reach(self, moves)
        self.assertFalse(any(m[3] for m in moves))
        self.assertEqual(obj.pending, [])

    def test_every_sequence_stays_in_reach(self):
        for targets in TestNoBedSteps().sequences():
            obj = build_transform(reach_planner(), targets[0])
            for target in targets[1:]:
                obj.move(target, 100.)
            obj.release()
            check_all(self, obj.toolhead.moves)
            check_reach(self, obj.toolhead.moves)
            self.assertEqual(obj.toolhead.branch, 1)

    def test_the_reach_must_hold_the_arc(self):
        with self.assertRaises(ValueError):
            reach_planner(far_reach=.1)

######################################################################
# The plans test_kin_6axis.c runs through the real step compressor
######################################################################

C_TABLES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'centre_blend_plans.h')

def blend_plans():
    # (what, the moves a plan sends) for each case the C test runs
    plans = []
    obj, commanded = run_corner(blend_planner(), math.radians(30.))
    plans.append(("30 degree print corner onto the far branch",
                  obj.toolhead.moves))
    obj, commanded = run_corner(blend_planner(), math.radians(80.))
    plans.append(("80 degree print corner onto the far branch",
                  obj.toolhead.moves))
    obj, commanded = run_corner(
        blend_planner(travel_policy='bypass', print_policy='error',
                      can_cross=False), math.radians(160.), can_cross=False)
    plans.append(("back out 20 degrees off the way in", obj.toolhead.moves))
    obj = build_transform(blend_planner(), pos(20., 0., 10., 0.))
    obj.move(pos(0., 0., 10., 2.), 60.)
    obj.release()
    mark = len(obj.toolhead.moves)
    out = math.radians(60.)
    obj.move(pos(20. * math.cos(out), 20. * math.sin(out), 10., 4.), 60.)
    plans.append(("out along a new line from rest",
                  obj.toolhead.moves[mark:]))
    obj = build_transform(blend_planner(), pos(-20., .02, 10., 0.))
    obj.move(pos(20., .02, 10., 4.), 60.)
    plans.append(("a near miss taken through the centre",
                  obj.toolhead.moves))
    obj = run_moves(reach_planner(), pos(40., 0., 10., 0.),
                    [(pos(-3., 0., 10., 2.), 60.),
                     (pos(40., 0., 10., 4.), 60.)])
    plans.append(("3mm past the centre and back", obj.toolhead.moves))
    return plans

def render_c_tables():
    def num(value):
        return '%.17g' % (value,)
    lines = [
        "// The plans test_centre_blend_step_generation() in",
        "// test_kin_6axis.c runs through the real step compressor.",
        "// Generated by TestTheCTables in test_centre_path.py - do not",
        "// edit.  Regenerate with CENTRE_BLEND_REGENERATE=1.",
        ""]
    plans = blend_plans()
    longest = max(len(moves) for what, moves in plans) + 1
    lines.append("#define BLEND_PLAN_MAX %d" % (longest,))
    for i, (what, moves) in enumerate(plans):
        first = moves[0]
        lines.append("")
        lines.append("static const struct branch_pt blend_plan_%d[] = {"
                     % (i,))
        lines.append("    {%s, %s," % (num(first[0][0]), num(first[0][1])))
        lines.append("     0, %d, 0, 0}," % (first[2],))
        for start, end, branch, flip, speed in moves:
            lines.append("    {%s, %s," % (num(end[0]), num(end[1])))
            lines.append("     0, %d, %d, %s}," % (branch, int(flip),
                                                 num(speed)))
        lines.append("};")
    lines += ["",
              "static const struct {",
              "    const char *what;",
              "    const struct branch_pt *pts;",
              "    int n;",
              "} blend_plans[] = {"]
    for i, (what, moves) in enumerate(plans):
        lines.append('    {"%s", blend_plan_%d, %d},' % (what, i, len(moves)))
    lines.append("};")
    return "\n".join(lines) + "\n"


class TestTheCTables(unittest.TestCase):
    def test_the_c_tables_are_what_the_planner_emits(self):
        text = render_c_tables()
        if os.environ.get('CENTRE_BLEND_REGENERATE'):
            with open(C_TABLES, 'w') as f:
                f.write(text)
        with open(C_TABLES) as f:
            current = f.read().replace('\r\n', '\n')
        self.assertEqual(current, text,
                         msg="centre_blend_plans.h is stale - run with"
                         " CENTRE_BLEND_REGENERATE=1")

    def test_every_plan_gets_past_the_check(self):
        for what, moves in blend_plans():
            self.assertGreater(len(moves), 3, msg=what)
            check_all(self, moves)
            check_reach(self, moves, REACH if 'past' in what else 1e9)


if __name__ == '__main__':
    unittest.main()
