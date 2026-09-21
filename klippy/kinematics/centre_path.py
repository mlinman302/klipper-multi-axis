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
# the one the bed faces - carrying straight on through it included.
#
# The host can see the moves on either side, so the host schedules the
# angle.  This file turns commanded moves into the list of moves that
# carries them out without ever asking the bed to step.  It is pure - no
# printer, no toolhead - so the g-code transform in [polar_singularity]
# uses it, and so can anything else that moves the toolhead.
#
# THE MOVES IT MAKES
#
# Arriving.  A move whose target is on the centre is stopped PARK_RADIUS
# short of it, on the ray it arrived along.  At rest the bed angle is then
# the angle of that park point - the one the bed arrived facing - rather
# than whatever atan2 makes of the bare centre.  That is what lets the
# bed angle be read back off the toolhead position, which departing
# needs, and which setting the toolhead position relies on (see "The
# position names the bed angle" in bed_centre.py).  The park point is a
# tenth of a micron from the centre: far below anything the arm can
# resolve, and far above rounding.
#
# Departing.  A move from the centre to a point off the line the bed faces
# needs the bed to turn first, and the bed can only turn at a finite rate
# while the tool is outside the disc.  Where the arm can travel through
# the centre (arm_crosses_centre) and the move's policy is 'cross', the
# tool may leave along either half of the line the bed faces - carrying
# on through the centre onto the other branch if that half is the nearer
# - so the bed never turns more than a quarter turn.  How the turn is
# made is below, under "Turning the bed on the move".
#
# Crossing.  A move that passes through the disc - or so close to it that
# no feedrate within min_velocity would hold the bed's limits - is left
# alone for the move check to refuse ('error'), or split at its closest
# approach into an arrival and a departure.  With 'bypass' the departure
# turns the bed up to half a turn, which is harmless on a travel move and
# leaves a blob on a print move.  With 'cross' it carries straight on
# through the centre with the bed held still and the arm travelling
# through zero radius: a move dead through the middle keeps its exact
# path and its speed.  Printing defaults to 'error', since only 'cross'
# is fit for it.
#
# Everything else passes through.  A near miss that the bed's limits slow
# down is split around its closest approach, into segments whose inner
# radii double outward, so that only the part of the move that really is
# close to the centre runs slowly.
#
# THE HEAD'S TILT IS NEVER TOUCHED
#
# B is carried along every planned move exactly as the g-code commands
# it, interpolated like z and e.  Nothing here stands the head up or
# otherwise changes it to make a transit easier.  With the head tilted,
# turning the bed swings the machine's B through [b_projection] and the
# arm through [rtcp] - on an arm that cannot travel through the middle,
# through the middle, which [rtcp]'s own move check refuses, so a tilted
# transit on such an arm fails with that check's message rather than
# quietly printing a different angle.  'cross' holds the bed still and
# keeps the tilt with no cost at all.
#
# TURNING THE BED ON THE MOVE
#
# The simplest way to turn the bed at the centre is to stop on it and
# walk the tool round a tiny circle - reorient_radius, 0.125 mm by
# default - while the bed turns under it, with z, e and rotation held.
# On a travel move that costs nothing.  On a print move it is a blob:
# the nozzle comes to a stop on the part and dwells there, full of
# pressure and with nothing extruding, for as long as the bed takes to
# turn.
#
# So the turn is blended into the moves either side of the centre
# instead.  A position on the bed is (rho, phi): an arm radius, signed on
# the negative branch, and the angle the bed faces.  The blend walks rho
# through the centre at the tool's own speed while phi ramps from the
# angle the bed faces on the way in to the one it has to face on the way
# out, spread over a distance R either side:
#
#     lambda   distance along the commanded path from the centre,
#              negative on the way in
#     rho      = branch_in * |lambda|  or  branch_out * lambda
#     phi      = phi_in + turn * sigma(lambda)
#
# sigma rises linearly from 0 at -R_in to 1 at +R_out, flat across a
# small core either side of the centre so the tool passes through it
# straight, along the line the bed faces - through the park point, and
# onto the other branch there if it carries on to the far side.  Every
# other axis - z, e, B - takes the value the commanded path has at the
# same lambda, so extrusion goes on at the commanded rate per millimetre
# and the tilt is exactly what was commanded.
#
# The path leaves the commanded one by lambda times the angle phi still
# has to make up, which peaks at about |turn| * R / 8 for a symmetric
# blend, and the bed turns at v * |turn| / (R_in + R_out).  So a blend
# within blend_tolerance of the path has R at most 8 * tolerance /
# |turn|, and runs at up to the bed's angular limit times
# (R_in + R_out) / |turn|.  With the defaults and a 5 rad/s bed a
# 10 degree corner through the centre is blended at full print speed, a
# 45 degree one at a few mm/s, and the tool is moving and extruding the
# whole time.  A turn too sharp to blend at min_velocity or better falls
# back to the arc.
#
# The blend is drawn as chords whose radii grow geometrically outward
# from the core, BLEND_CHORD_RATIO apart, since a chord turns the bed
# fastest at its inner end, by about that ratio over the rate the blend
# was designed for.  Each chord is held to the bed's limits here, at
# exactly the speed the move check would allow it.
#
# LOOKING AHEAD
#
# A symmetric blend needs the move after the centre before the move onto
# it has finished.  So hold_point() says how much of an arriving move to
# send now - all but the last blend_radius of it - and plan_held() plans
# the rest together with the next move once it is known: a blend through
# the centre if the next move leaves it, or the usual park point if it
# does not.  [polar_singularity] holds the tail in between, and gives it
# up to the toolhead on its own (release()) before anything else touches
# the toolhead.  Starting from rest on the centre - after a z move there,
# say - only the departing half is left, which blends too, over a
# distance four times shorter for the same tolerance.
#
# A near miss uses the same machinery.  One so close that holding the
# bed's limits would take it below min_velocity is routed through the
# centre by its policy, and there it is now blended where it used to stop
# and turn on the circle - under 'cross' the bend is a fraction of a
# degree, so it runs at full speed.  And under 'cross', a move that is
# merely slowed - crawling past at the bed's angular limit, which takes
# about pi / max_angular_velocity however near it passes - is routed the
# same way when it passes within blend_tolerance of the centre.  With the
# defaults every near miss that close is already too tight to slow for;
# the second case matters for a larger tolerance or a faster bed.
import math
from . import bed_centre

POLICIES = ('error', 'bypass', 'cross')

# A target this close to the centre is on it.  The same disc the step
# generator treats specially, so that nothing is ever left standing inside
# it except a park point.
SNAP_RADIUS = bed_centre.CENTRE_RADIUS
# How far short of the centre an arriving move stops, in mm
PARK_RADIUS = 1e-4
# A bed angle change this small (in radians) is left to the step
# generator - the same tolerance the move check holds a departure to
ANGLE_TOLERANCE = bed_centre.ANGLE_TOLERANCE
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
# The straight core of a blend: within this radius of the centre the tool
# runs along the line the bed faces, outside the dead zone either side
BLEND_CORE_RADIUS = MIN_REORIENT_RADIUS
# Successive radii of a blend's chords grow by at most this factor.  A
# chord turns the bed fastest at its inner end, by about this much over
# the blend's own rate, so the blend is designed for the bed's limit
# divided by it.
BLEND_CHORD_RATIO = 1.25
# The share of blend_tolerance the smooth curve is designed to; the chords
# drawn through it take some of the rest
BLEND_DESIGN_MARGIN = .9
# How far the blended path may leave the commanded one (mm), and how far
# from the centre a blend may reach (mm), when nothing configures them
DEFAULT_BLEND_TOLERANCE = .05
DEFAULT_BLEND_RADIUS = 2.


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

wrap_angle = bed_centre.wrap_angle

def is_extruding(start, end):
    return len(start) > 3 and len(end) > 3 and end[3] > start[3]

def _geometric_radii(r_from, r_to):
    # Radii from r_from to r_to inclusive, either way round, each within
    # BLEND_CHORD_RATIO of the last
    n = int(math.ceil(abs(math.log(r_from / r_to))
                      / math.log(BLEND_CHORD_RATIO) - 1e-9))
    n = max(n, 1)
    return [r_from * (r_to / r_from) ** (float(i) / n) for i in range(n + 1)]

def _ramp_error(share, radius, core):
    # The furthest a blend side of this radius strays from the commanded
    # path per radian of turn, when it makes 'share' of the turn: the
    # largest lambda * (angle still to make) over the ramp, which the
    # straight core bounds from below
    peak = max(radius / 2., core)
    return share * peak * (radius - peak) / (radius - core)


class CentrePlanner:
    def __init__(self, max_angular_v=0., max_angular_a=0., min_velocity=.5,
                 reorient_radius=None, travel_policy='bypass',
                 print_policy='error', can_cross=False,
                 blend_tolerance=DEFAULT_BLEND_TOLERANCE,
                 blend_radius=DEFAULT_BLEND_RADIUS):
        # 'can_cross' says the arm can travel through the centre to a
        # negative radius.  A blend_tolerance of zero turns the bed on the
        # arc alone and never holds a move back.
        for policy in (travel_policy, print_policy):
            if policy not in POLICIES:
                raise ValueError("Unknown centre policy '%s'" % (policy,))
            if policy == 'cross' and not can_cross:
                raise ValueError(
                    "The 'cross' policy carries the arm through the centre"
                    " of the bed to a negative radius, and this machine"
                    " does not allow that - see arm_crosses_centre")
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
        if blend_tolerance < 0.:
            raise ValueError("blend_tolerance must not be negative")
        min_blend = 2. * BLEND_CORE_RADIUS * BLEND_CHORD_RATIO
        if blend_radius < min_blend:
            raise ValueError(
                "blend_radius must be at least %.3f mm - room for the"
                " straight core of a blend and a chord either side of it"
                % (min_blend,))
        self.max_angular_v = max_angular_v
        self.max_angular_a = max_angular_a
        self.min_velocity = min_velocity
        self.reorient_radius = reorient_radius
        self.reorient_v = max_angular_v or DEFAULT_REORIENT_VELOCITY
        self.travel_policy = travel_policy
        self.print_policy = print_policy
        self.can_cross = can_cross
        self.blend_tolerance = blend_tolerance
        self.blend_radius = blend_radius

    def policy(self, start, end):
        if is_extruding(start, end):
            return self.print_policy
        return self.travel_policy

    def blending(self):
        return self.blend_tolerance > 0.

    def plan(self, machine_xy, start, end, speed, branch=1):
        # The moves that carry out start -> end.  'start' and 'end' are
        # full position vectors in the frame of the caller; 'machine_xy'
        # is where the toolhead really is, which on the centre is a park
        # point rather than the (0, 0) the caller asked for, and 'branch'
        # the branch it is on (see bed_centre.py).  Returns a list of
        # (position, speed).  A move that has no plan is returned as it
        # is, for the move check to refuse.
        from_centre = at_centre(machine_xy)
        if at_centre(end):
            if from_centre:
                # Staying on the axis: hold the park point, so that z,
                # e and rotation happen without the bed moving
                return [(with_xy(end, machine_xy[0], machine_xy[1]),
                         speed)]
            return self._arrive(machine_xy, end, speed)
        if from_centre:
            return self._depart(machine_xy, start, end, speed, branch,
                                self.policy(start, end) == 'cross')
        return self._through(start, end, speed, branch)

    ######################################################################
    # Looking ahead across the centre
    ######################################################################
    def hold_point(self, machine_xy, start, end):
        # For a move onto the centre, the point to send it to now - the
        # rest is held back to be planned with whatever follows, see
        # plan_held().  None for a move that is not held: one that does
        # not arrive on the centre, or with blending off.  The point is
        # the start itself when the whole move is within blend_radius.
        if not self.blending() or not at_centre(end) or at_centre(machine_xy):
            return None
        length = math.sqrt(machine_xy[0]**2 + machine_xy[1]**2)
        if length <= self.blend_radius:
            return list(start)
        centre = with_xy(end, 0., 0.)
        return bed_centre.interpolate(start, centre,
                                      1. - self.blend_radius / length)

    def plan_held(self, held, end, speed, branch=1):
        # Plan the held tail of an arrival, 'held' = (hold point, the
        # target on the centre, its speed), together with the move from
        # the centre to 'end'.  The tool is standing at the hold point.
        hold, centre, held_speed = held
        if not at_centre(end):
            moves = self._blend(hold, centre, held_speed, end, speed, branch,
                                self.policy(centre, end) == 'cross',
                                self.blend_tolerance)
            if moves is not None:
                return moves
        moves = self.release(held)
        return moves + self.plan(moves[-1][0][:2], centre, end, speed,
                                 branch)

    def release(self, held):
        # The held tail of an arrival on its own, when nothing follows it
        # to plan it with: on to the park point
        hold, centre, held_speed = held
        return self._arrive(hold, centre, held_speed)

    ######################################################################
    # Arriving and departing
    ######################################################################
    def _arrive(self, machine_xy, end, speed):
        # Stop short, on the ray the tool arrived along
        x, y = machine_xy[0], machine_xy[1]
        scale = PARK_RADIUS / math.sqrt(x*x + y*y)
        return [(with_xy(end, x * scale, y * scale), speed)]

    def _departure(self, machine_xy, end, branch, may_cross):
        # How far the bed has to turn before the tool can leave the centre
        # for 'end' in a straight line, and the branch it leaves on: the
        # usual one, or - where the arm may cross the centre - the other,
        # if that turns the bed less.  'machine_xy' is any point on the
        # line the bed faces, on the side the tool is.
        facing = bed_centre.bed_angle(machine_xy[0], machine_xy[1], None,
                                      branch)
        ray = math.atan2(end[1], end[0])
        best = None
        for new_branch in ((1, -1) if may_cross else (1,)):
            target = ray if new_branch > 0 else bed_centre.half_turn(ray)
            turn = wrap_angle(target - facing)
            if best is None or abs(turn) < abs(best[0]) - ANGLE_TOLERANCE:
                best = (turn, new_branch)
        return best

    def _depart(self, machine_xy, start, end, speed, branch=1,
                may_cross=False):
        turn, new_branch = self._departure(machine_xy, end, branch,
                                           may_cross)
        if abs(turn) <= ANGLE_TOLERANCE:
            # Already facing along the line to the target - the move is
            # radial.  If the target is on the far side of the centre the
            # kinematics carries it through onto the other branch.
            return [(end, speed)]
        if self.blending():
            # Turn on the way out, moving, rather than on the arc
            moves = self._blend(None, start, speed, end, speed, branch,
                                may_cross, self.blend_tolerance, machine_xy)
            if moves is not None:
                return moves
        angle_from = bed_centre.bed_angle(machine_xy[0], machine_xy[1], None,
                                          branch)
        # The arc runs at this arm radius: on the far side of the centre,
        # negative, when the tool leaves on the other branch
        radius = new_branch * self.reorient_radius
        def on_arc(angle):
            return with_xy(start, radius * math.cos(angle),
                           radius * math.sin(angle))
        # Out along the line the bed already faces.  It starts inside the
        # dead zone heading straight along that line, so the bed holds its
        # angle - on the far side, by carrying on through the centre.
        moves = [(on_arc(angle_from), speed)]
        # Round to the new ray, outside the dead zone the whole way
        # A quarter turn is nine chords, not ten because of rounding
        count = int(math.ceil(abs(turn) / MAX_ARC_CHORD - 1e-9))
        arc_speed = min(speed,
                        self.reorient_v * self.reorient_radius * ARC_CHORD_DIP)
        for i in range(1, count + 1):
            moves.append((on_arc(angle_from + turn * i / count), arc_speed))
        # And radially on to the target, carrying everything else with it
        moves.append((end, speed))
        return moves

    ######################################################################
    # Turning the bed on the move
    ######################################################################
    def _blend(self, start, centre, speed_in, end, speed_out, branch,
               may_cross, tolerance, machine_xy=None):
        # The moves from 'start', on a straight line in to 'centre', on
        # through the centre and out to 'end', turning the bed on the way
        # rather than stopped on the centre - see "Turning the bed on the
        # move" above.  With 'start' None the tool is standing on the
        # centre, parked at 'machine_xy', and only the way out is blended.
        # Returns None when there is no blend within 'tolerance' of the
        # path that runs at min_velocity or better.
        core = BLEND_CORE_RADIUS
        centre = with_xy(centre, 0., 0.)
        facing_xy = start if start is not None else machine_xy
        turn, new_branch = self._departure(facing_xy, end, branch, may_cross)
        if abs(turn) <= ANGLE_TOLERANCE:
            return None
        len_in = 0.
        if start is not None:
            len_in = math.sqrt(start[0]**2 + start[1]**2)
        len_out = math.sqrt(end[0]**2 + end[1]**2)
        cap_in = min(len_in, self.blend_radius)
        cap_out = min(len_out, self.blend_radius)
        shortest = core * BLEND_CHORD_RATIO
        if cap_out < shortest or (start is not None and cap_in < shortest):
            return None
        design = tolerance * BLEND_DESIGN_MARGIN
        def sides(reach):
            return (min(cap_in, reach) if start is not None else 0.,
                    min(cap_out, reach))
        def core_share(r_in, r_out):
            # The share of the turn made by the time the tool reaches the
            # core, chosen so the bed turns at one rate on both sides
            if not r_in:
                return 0.
            return (r_in - core) / (r_in + r_out - 2. * core)
        def error(reach):
            r_in, r_out = sides(reach)
            share = core_share(r_in, r_out)
            err = _ramp_error(1. - share, r_out, core)
            if r_in:
                err = max(err, _ramp_error(share, r_in, core))
            return abs(turn) * err
        # The widest blend within tolerance: its error grows with reach
        lo, hi = shortest, max(cap_in, cap_out)
        if error(lo) > design:
            return None
        if error(hi) <= design:
            lo = hi
        else:
            for i in range(40):
                mid = .5 * (lo + hi)
                if error(mid) <= design:
                    lo = mid
                else:
                    hi = mid
        r_in, r_out = sides(lo)
        share = core_share(r_in, r_out)
        span = r_out - core + (r_in - core if r_in else 0.)
        omega = self.reorient_v / BLEND_CHORD_RATIO
        v_blend = omega * span / abs(turn)
        if v_blend < self.min_velocity:
            return None
        phi_in = bed_centre.bed_angle(facing_xy[0], facing_xy[1], None,
                                      branch)
        def sigma(lam):
            if lam < 0.:
                return share * (r_in + lam) / (r_in - core)
            return share + (1. - share) * (lam - core) / (r_out - core)
        def point(lam, s):
            if lam < 0.:
                cmd = bed_centre.interpolate(start, centre,
                                             1. + lam / len_in)
                rho = branch * -lam
            else:
                cmd = bed_centre.interpolate(centre, end, lam / len_out)
                rho = new_branch * lam
            phi = phi_in + turn * s
            return with_xy(cmd, rho * math.cos(phi), rho * math.sin(phi))
        # (position, requested speed, whether it is a chord of the blend)
        plan = []
        if start is not None:
            if r_in < len_in:
                # In along the commanded line until the blend begins
                plan.append((point(-r_in, 0.), speed_in, False))
            for radius in _geometric_radii(r_in, core)[1:]:
                plan.append((point(-radius, sigma(-radius)),
                             min(speed_in, v_blend), True))
            # Through the park point, on the line the bed faces
            plan.append((point(-PARK_RADIUS, share),
                         min(speed_in, v_blend), False))
        # On through the centre, or back out of it, along that line
        plan.append((point(core, share), min(speed_out, v_blend), False))
        for radius in _geometric_radii(core, r_out)[1:]:
            plan.append((point(radius, sigma(radius)),
                         min(speed_out, v_blend), True))
        if r_out < len_out:
            plan.append((list(end), speed_out, False))
        # Hold each chord to the bed's limits, as the move check would
        moves = []
        here = start if start is not None else machine_xy
        for pos, speed, chord in plan:
            if chord:
                offset, r_min, u_start, u_end = bed_centre.path_geometry(
                    here, pos)
                v_limit = bed_centre.limits_for_angular_rates(
                    offset, r_min, u_start, u_end,
                    self.max_angular_v, self.max_angular_a)[0]
                if v_limit is not None:
                    if v_limit < self.min_velocity:
                        return None
                    speed = min(speed, v_limit)
            moves.append((pos, speed))
            here = pos
        return moves

    ######################################################################
    # Everything else
    ######################################################################
    def _through(self, start, end, speed, branch=1):
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
            policy = self.policy(start, end)
            if policy == 'error':
                # Leave the refusal to the move check, which says why
                return [(end, speed)]
            return self._via_centre(start, end, speed, branch,
                                    policy == 'cross', u_start, u_end, r_min)
        if v_limit is not None and v_limit < speed and self.max_angular_v:
            if (self.policy(start, end) == 'cross' and u_start * u_end < 0.
                    and r_min < self.blend_tolerance):
                # Close enough to go through the centre instead of
                # crawling past it
                moves = self._blend(
                    start, self._closest(start, end, u_start, u_end), speed,
                    end, speed, branch, True, self.blend_tolerance - r_min)
                if moves is not None:
                    return moves
            return self._split_slow(start, end, speed, offset,
                                    u_start, u_end)
        return [(end, speed)]

    def _closest(self, start, end, u_start, u_end):
        # The commanded position where the move comes closest to the
        # centre, moved onto the centre
        length = u_end - u_start
        t = min(max(-u_start / length, 0.), 1.)
        return with_xy(bed_centre.interpolate(start, end, t), 0., 0.)

    def _via_centre(self, start, end, speed, branch, may_cross,
                    u_start, u_end, r_min):
        # Visit the centre at the closest approach, interpolating
        # everything else there, and leave it for the target - turning
        # the bed on the way through where that stays within tolerance,
        # else stopped on the centre, or carrying straight on through
        middle = self._closest(start, end, u_start, u_end)
        if r_min < self.blend_tolerance:
            moves = self._blend(start, middle, speed, end, speed, branch,
                                may_cross, self.blend_tolerance - r_min)
            if moves is not None:
                return moves
        park = self._arrive(start, middle, speed)[0][0]
        return ([(park, speed)]
                + self._depart(park, middle, end, speed, branch, may_cross))

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
