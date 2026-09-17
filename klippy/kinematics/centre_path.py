# Planning paths at the bed centre on rotating-bed kinematics
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# WHAT THIS IS FOR
#
# On a rotating-bed machine the bed angle is derived from x/y, and within
# a small disc at the centre it stops being determined by position at all.
# There it becomes a free degree of freedom, and the step generator can
# only choose it one sample at a time - which it does, in bed_centre.h, by
# taking it from the direction of travel.  A single sample cannot tell a
# move that is arriving at the centre from one that is leaving it, so that
# rule steps the bed whenever a move leaves the centre along any ray but
# the one the bed faces - carrying straight on through it included - and,
# because on the bare centre it falls back to atan2 of wherever the
# carriage is, whenever RTCP shifts the carriage there with a B move.
#
# The host can see the moves on either side, so the host schedules the
# angle.  This file turns one commanded move into the list of moves that
# carries it out without ever asking the bed to step.  It is pure - no
# printer, no toolhead - so the g-code transform in [polar_singularity]
# uses it, and so can anything else that moves the toolhead.
#
# THE THREE MOVES IT MAKES
#
# Arriving.  A move whose target is on the centre is stopped PARK_RADIUS
# short of it, on the ray it arrived along.  At rest the bed angle is then
# the angle of that park point - the one the bed arrived facing - rather
# than whatever atan2 makes of the bare centre, and it stays so under a
# B move with RTCP on, whose correction scales the carriage along the
# park ray.  It also means the bed angle can be read back off the
# toolhead position, which is what departing needs.  The park point is a
# tenth of a micron from the centre: far below anything the arm can
# resolve, and far above rounding.
#
# Departing.  A move from the centre to a point on a different ray needs
# the bed to turn first, and the bed can only turn at a finite rate while
# the tool is outside the disc.  So the tool steps radially out to
# reorient_radius along the ray the bed is already facing, follows that
# circle round to the new ray at no more than the bed's angular velocity
# limit, and then carries on radially to the target.  The arc is the only
# place the tool leaves the commanded path, by reorient_radius at most,
# and z, e and any rotary axis are held while it runs.
#
# Crossing.  A move that passes through the disc - or so close to it that
# no feedrate within min_velocity would hold the bed's limits - is either
# left alone for the move check to refuse ('error'), or split at its
# closest approach into an arrival and a departure ('bypass').  Bypass
# stops on the centre while the bed turns up to half a turn, which is
# harmless on a travel move and leaves a blob on a print move, so the two
# have separate policies and printing defaults to 'error'.
#
# Everything else passes through.  A near miss that the bed's limits slow
# down is split around its closest approach, into segments whose inner
# radii double outward, so that only the part of the move that really is
# close to the centre runs slowly.
#
# WHAT IT DOES NOT DO YET
#
# It does not stand the head up.  A tilted head at the centre puts the arm
# on the far side of the middle as soon as RTCP swings the tip, and the
# RTCP reach check refuses that - so centre work is for B at zero, or at
# least at a B whose swing fits inside the arc.
import math
from . import bed_centre

POLICIES = ('error', 'bypass')

# A target this close to the centre is on it.  The same disc the step
# generator treats specially, so that nothing is ever left standing inside
# it except a park point.
SNAP_RADIUS = bed_centre.CENTRE_RADIUS
# How far short of the centre an arriving move stops, in mm
PARK_RADIUS = 1e-4
# A bed angle change this small (in radians) is left to the step
# generator - well under a microstep on any bed this is likely to drive
ANGLE_TOLERANCE = 1e-4
# The most one chord of a reorientation arc turns the bed through.  Each
# chord dips inside the circle by ARC_CHORD_DIP at its middle.
MAX_ARC_CHORD = math.radians(10.)
ARC_CHORD_DIP = math.cos(MAX_ARC_CHORD / 2.)
# The inner radii of the slow segments around a near miss grow by this
# factor, and there are at most this many of them on each side
SLOW_SEGMENT_RATIO = 2.
MAX_SLOW_SEGMENTS = 6
# The bed's angular velocity (rad/s) for a reorientation when nothing
# configures one.  With no limit configured the move check will not slow
# the arc, so the arc has to slow itself.
DEFAULT_REORIENT_VELOCITY = 1.
# The smallest reorient_radius: comfortably outside the step generator's
# dead zone, whose rule is exactly what the arc exists to avoid
MIN_REORIENT_RADIUS = 4. * bed_centre.CENTRE_RADIUS


def default_reorient_radius(max_angular_v, min_velocity):
    # The arc is held to max_angular_v * radius by the move check, so it
    # must be large enough for that to clear the min_velocity floor - with
    # room for the dip of its chords
    radius = MIN_REORIENT_RADIUS
    if max_angular_v:
        radius = max(radius, 1.25 * min_velocity / max_angular_v)
    return radius

def at_centre(pos):
    return pos[0]*pos[0] + pos[1]*pos[1] <= SNAP_RADIUS * SNAP_RADIUS

def with_xy(pos, x, y):
    res = list(pos)
    res[0], res[1] = x, y
    return res

def wrap_angle(angle):
    # Into (-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

def is_extruding(start, end):
    return len(start) > 3 and len(end) > 3 and end[3] > start[3]


class CentrePlanner:
    def __init__(self, max_angular_v=0., max_angular_a=0., min_velocity=.5,
                 reorient_radius=None, travel_policy='bypass',
                 print_policy='error'):
        for policy in (travel_policy, print_policy):
            if policy not in POLICIES:
                raise ValueError("Unknown centre policy '%s'" % (policy,))
        if reorient_radius is None:
            reorient_radius = default_reorient_radius(max_angular_v,
                                                      min_velocity)
        if reorient_radius < MIN_REORIENT_RADIUS:
            raise ValueError(
                "reorient_radius must be at least %.3f mm - four times the"
                " step generator's dead zone" % (MIN_REORIENT_RADIUS,))
        if (max_angular_v and max_angular_v * reorient_radius * ARC_CHORD_DIP
                < min_velocity):
            raise ValueError(
                "reorient_radius %.3f mm is too small: the bed's angular"
                " velocity limit holds an arc that tight below min_velocity."
                "  Use at least %.3f mm"
                % (reorient_radius,
                   min_velocity / (max_angular_v * ARC_CHORD_DIP)))
        self.max_angular_v = max_angular_v
        self.max_angular_a = max_angular_a
        self.min_velocity = min_velocity
        self.reorient_radius = reorient_radius
        self.reorient_v = max_angular_v or DEFAULT_REORIENT_VELOCITY
        self.travel_policy = travel_policy
        self.print_policy = print_policy

    def policy(self, start, end):
        if is_extruding(start, end):
            return self.print_policy
        return self.travel_policy

    def plan(self, machine_xy, start, end, speed):
        # The moves that carry out start -> end.  'start' and 'end' are
        # full position vectors in the frame of the caller; 'machine_xy'
        # is where the toolhead really is, which on the centre is a park
        # point rather than the (0, 0) the caller asked for.  Returns a
        # list of (position, speed).
        from_centre = at_centre(machine_xy)
        if at_centre(end):
            if from_centre:
                # Staying on the axis: hold the park point, so that z,
                # e and rotation happen without the bed moving
                return [(with_xy(end, machine_xy[0], machine_xy[1]),
                         speed)]
            return self._arrive(machine_xy, end, speed)
        if from_centre:
            return self._depart(machine_xy, start, end, speed)
        return self._through(start, end, speed)

    ######################################################################
    # Arriving and departing
    ######################################################################
    def _arrive(self, machine_xy, end, speed):
        # Stop short, on the ray the tool arrived along
        x, y = machine_xy[0], machine_xy[1]
        scale = PARK_RADIUS / math.sqrt(x*x + y*y)
        return [(with_xy(end, x * scale, y * scale), speed)]

    def _depart(self, machine_xy, start, end, speed):
        angle_from = bed_centre.bed_angle(machine_xy[0], machine_xy[1])
        angle_to = math.atan2(end[1], end[0])
        turn = wrap_angle(angle_to - angle_from)
        if abs(turn) <= ANGLE_TOLERANCE:
            # Already facing the right way - the move is radial
            return [(end, speed)]
        radius = self.reorient_radius
        def on_arc(angle):
            return with_xy(start, radius * math.cos(angle),
                           radius * math.sin(angle))
        # Out along the ray the bed already faces.  It starts inside the
        # dead zone heading outward, so the bed holds its angle.
        moves = [(on_arc(angle_from), speed)]
        # Round to the new ray, outside the dead zone the whole way
        # A quarter turn is nine chords, not ten because of rounding
        count = int(math.ceil(abs(turn) / MAX_ARC_CHORD - 1e-9))
        arc_speed = min(speed, self.reorient_v * radius * ARC_CHORD_DIP)
        for i in range(1, count + 1):
            moves.append((on_arc(angle_from + turn * i / count), arc_speed))
        # And radially on to the target, carrying everything else with it
        moves.append((end, speed))
        return moves

    ######################################################################
    # Everything else
    ######################################################################
    def _through(self, start, end, speed):
        if start[0] == end[0] and start[1] == end[1]:
            return [(end, speed)]
        offset, r_min, u_start, u_end = bed_centre.path_geometry(start, end)
        swept = bed_centre.swept_angle(start, end)
        v_limit, a_limit = bed_centre.limits_for_angular_rates(
            offset, r_min, u_start, u_end,
            self.max_angular_v, self.max_angular_a)
        # Too tight when holding the limit would take the move below the
        # floor - a move already asked to run that slowly is left alone
        too_tight = (v_limit is not None and v_limit < self.min_velocity
                     and v_limit < speed)
        if bed_centre.crosses_centre(r_min, swept) or too_tight:
            if self.policy(start, end) == 'error':
                # Leave the refusal to the move check, which says why
                return [(end, speed)]
            return self._bypass(start, end, speed, u_start, u_end)
        if v_limit is not None and v_limit < speed and self.max_angular_v:
            return self._split_slow(start, end, speed, offset,
                                    u_start, u_end)
        return [(end, speed)]

    def _bypass(self, start, end, speed, u_start, u_end):
        # Visit the centre at the closest approach, interpolating
        # everything else there, and turn the bed while stopped on it
        length = u_end - u_start
        t = min(max(-u_start / length, 0.), 1.)
        middle = with_xy(bed_centre.interpolate(start, end, t), 0., 0.)
        moves = self._arrive(start, middle, speed)
        park = moves[-1][0]
        return moves + self._depart(park, middle, end, speed)

    def _split_slow(self, start, end, speed, offset, u_start, u_end):
        # Past r_fast the bed's velocity limit no longer binds at 'speed'.
        # Inside it, cut the move where the radius doubles, so each piece
        # is only held to the limit at its own inner end.
        r_min = bed_centre.path_geometry(start, end)[1]
        r_fast = math.sqrt(speed * abs(offset) / self.max_angular_v)
        radii = []
        radius = r_min * SLOW_SEGMENT_RATIO
        while radius < r_fast and len(radii) < MAX_SLOW_SEGMENTS - 1:
            radii.append(radius)
            radius *= SLOW_SEGMENT_RATIO
        radii.append(r_fast)
        cuts = []
        for radius in radii:
            u = math.sqrt(max(radius*radius - offset*offset, 0.))
            for candidate in (-u, u):
                if u_start < candidate < u_end:
                    cuts.append(candidate)
        length = u_end - u_start
        moves = [(bed_centre.interpolate(start, end,
                                         (u - u_start) / length), speed)
                 for u in sorted(cuts)]
        moves.append((end, speed))
        return moves
