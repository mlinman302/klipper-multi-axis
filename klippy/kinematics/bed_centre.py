# Geometry of the bed centre on rotating-bed kinematics
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# THE SINGULARITY
#
# On a polar or core r-theta machine the bed angle is not a commanded axis.
# It is derived - theta = atan2(y, x) - so it has no value at all on the
# line x = y = 0, and its derivatives have already run away on the
# approach.  A tool tip travelling through [0, 0, N] asks the bed for a
# half turn in the instant the sign flips.
#
# This file is the one place that geometry is written down.  It imports
# nothing but math, so any layer can use it: the kinematics that limit a
# move's feedrate near the centre, the [polar_singularity] move check that
# refuses what no feedrate can rescue, and the [rtcp] and [b_projection]
# checks that need to know where along a move the centre matters.
#
# THE MIRROR
#
# The step generators need the same numbers and the same dead zone rule,
# and they are C.  klippy/chelper/bed_centre.h carries them, and
# test/multi_axis/test_bed_centre.py fails if the two ever disagree.  A
# change to a constant here is a change to that header too.
#
# A MOVE IN TWO NUMBERS
#
# A move is a straight line, so its whole relationship with the centre
# follows from two numbers: the perpendicular offset of the line of travel
# from the centre,
#
#     offset = x0*uy - y0*ux        (u the unit direction of travel)
#
# which is constant along the move, and the arclength u measured from the
# foot of that perpendicular.  In terms of those,
#
#     r(u)^2     = offset^2 + u^2
#     theta_dot  = v * offset / r^2
#     theta_ddot = -2 * v^2 * offset * u / r^4  +  a * offset / r^2
#
# theta_dot peaks where r is smallest, at v*offset/r_min^2.  The geometric
# part of theta_ddot peaks a little further out, at |u| = |offset|/sqrt(3),
# where it reaches 0.65 * v^2 / offset^2; the second part is the move's own
# acceleration carried through the same 1/r^2.  So the bed's angular
# velocity diverges as 1/r and its acceleration as 1/r^2, and both limits
# land on the feedrate rather than on the move's acceleration.
#
# An offset of zero is the radial move - straight down the axis, departing
# from the centre along a ray, arriving along one - and it makes every
# rate zero with no special case.  It is also the chord dead through the
# centre, whose rates are all zero while the bed still has half a turn to
# make.  That one is caught by the swept angle, never by a rate.
#
# TWO BRANCHES
#
# Every x/y position has two polar names, (r, theta) and (-r, theta + pi).
# A move's branch says which one it is solved on: the usual one, with the
# arm at a positive radius, or - on a machine whose arm can travel through
# the centre - the other, with the arm at a negative radius and the bed
# turned the other half turn.  On the negative branch a straight line
# through the centre is a plain radial move with the bed held still, so a
# move marked branch_flip changes over to the other branch where it passes
# the centre.  That is the only way the branch ever changes: at the
# centre, where the arm radius is zero on both.
#
# THE POSITION NAMES THE BED ANGLE
#
# Standing still, the bed faces atan2(y, x) - the other half turn on the
# negative branch, and zero on the centre itself - and that is what
# setting a position tells the bed motor it is at.  Inside the dead zone a
# moving tool's bed angle comes from its direction of travel instead, so
# a move that ends there, or starts there, can leave the bed facing
# somewhere its position does not name.  The next position set would then
# quietly redefine the bed angle, and everything printed afterwards would
# be turned by the difference.  centre_turns() measures that mismatch and
# [polar_singularity] refuses any move that has one: a move may only come
# to rest in the dead zone on the ray it arrived along, and only leave it
# along the line the bed faces.
import math


######################################################################
# Constants - mirrored in klippy/chelper/bed_centre.h
######################################################################

# Within this radius (in mm) of the centre the bed angle is not
# meaningfully defined: an arbitrarily small change in x/y swings atan2 by
# up to pi.  Inside it, while the tool is moving in x/y, the bed angle is
# taken from the direction of travel instead - see bed_angle() below.
CENTRE_RADIUS = 0.010

# A sweep smaller than this is not worth refusing a move over
MIN_SWEPT_ANGLE = math.radians(1.)
# A step in the bed angle this small (in radians), made in a single
# sample, is left to the step generator - well under a microstep on any
# bed this is likely to drive.  Anything larger where a move leaves or
# comes to rest at the centre is refused (see centre_turns()).
ANGLE_TOLERANCE = 1e-4
# |u| / |offset| at which the geometric part of theta_ddot peaks, and the
# peak itself as a multiple of v^2 / offset^2
ANGULAR_ACCEL_PEAK_U = 1. / math.sqrt(3.)
ANGULAR_ACCEL_PEAK = 9. / (8. * math.sqrt(3.))


######################################################################
# The bed angle at a sample - mirrors move_get_branch() in
# klippy/chelper/trapq.c and bed_centre_angle(), bed_centre_facing(),
# bed_centre_cos() and bed_centre_radius() in klippy/chelper/bed_centre.h
######################################################################

# 'travel' is the move's direction in x/y (its axes_r.x and axes_r.y), or
# None for a position that is not moving.  Outside the dead zone, or when
# not moving in x/y, the bed angle is plain atan2 of the position.  Inside
# it, while moving, the angle is the one the path has where it leaves the
# zone - or, while the tool is still heading inward, where it entered.
# The bed is then already at the right angle by the time the radius means
# something again, so nothing has to turn at the boundary.  On the
# negative branch the bed is turned the other half turn.
#
# This is the rule the step generators actually run, which is why it is
# mirrored here rather than improved.  It is right for the homing sweep
# it was written for, which departs from the centre along +x.  It is not
# right in general: a move that leaves the centre along any ray but the
# line the bed already faces steps the bed at its first sample - the real
# step generator confirms it (see test_centre_path_step_generation() in
# test/multi_axis/test_kin_6axis.c) - and a move that comes to rest in the
# dead zone off the ray it arrived along leaves the bed facing somewhere
# its position does not name.  centre_turns() below measures both.
#
# The rule also hands the very last instant of a move that arrives on the
# centre the angle of travel rather than the angle arrived at, but the
# step generator stops short of that instant, so arriving is harmless.
#
# Scheduling the bed angle while the tool is on the axis is the job of the
# layer above this one (centre_path.py); nothing at the level of a single
# sample can do it, because a sample cannot tell a move that is arriving
# from one that is leaving.

def half_turn(angle):
    # angle + pi, folded back into (-pi, pi] - the same half turn the C
    # applies, so both sides agree to the bit
    return angle + (-math.pi if angle > 0. else math.pi)

def wrap_angle(angle):
    # Into (-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

def _moving(travel):
    return travel is not None and bool(travel[0] or travel[1])

def _in_zone(x, y, travel):
    return (x*x + y*y < CENTRE_RADIUS * CENTRE_RADIUS) and _moving(travel)

def _heading_inward(x, y, travel):
    return x * travel[0] + y * travel[1] < 0.

def sample_branch(x, y, travel, branch, branch_flip=False):
    # The branch a sample of a move is solved on: -1 or 1.  A move marked
    # branch_flip changes over from the point where it stops heading
    # towards the centre.
    negative = branch < 0
    if (branch_flip and travel is not None
            and x * travel[0] + y * travel[1] >= 0.):
        negative = not negative
    return -1 if negative else 1

def bed_angle(x, y, travel=None, branch=1):
    # The bed angle for a sample on the given branch (already resolved
    # with sample_branch()), in (-pi, pi], before unwrapping
    if not _in_zone(x, y, travel):
        angle = math.atan2(y, x)
    else:
        angle = math.atan2(travel[1], travel[0])
        if _heading_inward(x, y, travel):
            angle = half_turn(angle)
    if branch < 0:
        return half_turn(angle)
    return angle

def facing(x, y, travel=None, branch=1):
    # The unit vector bed_angle() points along - the direction of
    # increasing arm radius, in bed coordinates - without the atan2
    if not _in_zone(x, y, travel):
        r = math.sqrt(x*x + y*y)
        if r > 0.:
            fx, fy = x / r, y / r
        else:
            # atan2(0, 0) is zero
            fx, fy = 1., 0.
    else:
        length = math.sqrt(travel[0]**2 + travel[1]**2)
        fx, fy = travel[0] / length, travel[1] / length
        if _heading_inward(x, y, travel):
            fx, fy = -fx, -fy
    if branch < 0:
        return -fx, -fy
    return fx, fy

def cos_bed_angle(x, y, travel=None, branch=1):
    # cos(bed_angle())
    return facing(x, y, travel, branch)[0]

def arm_radius(x, y, branch=1):
    # The arm radius at a position - negative on the negative branch
    return branch * math.sqrt(x*x + y*y)

def unwrap_angle(angle, reference):
    # Bring an atan2 result to within half a turn of where the bed already
    # is, the way the bed solver does.  A single step either way, since
    # both values are already within a turn of zero.
    if angle - reference > math.pi:
        return angle - 2. * math.pi
    if angle - reference < -math.pi:
        return angle + 2. * math.pi
    return angle


######################################################################
# Where along a move the centre matters
######################################################################

# Every coordinate moves linearly along a move, so a fraction t of the way
# along it is exactly the linear interpolation of the two endpoints, for
# the whole position vector.

def interpolate(start_pos, end_pos, t):
    return [s + t * (e - s) for s, e in zip(start_pos, end_pos)]

def path_geometry(start_pos, end_pos):
    # The centre geometry of the xy segment start_pos -> end_pos: the
    # signed perpendicular offset of the path from the centre, the closest
    # the path comes to the centre, and the arclengths of the two endpoints
    # from the foot of the perpendicular.
    x0, y0 = start_pos[0], start_pos[1]
    dx, dy = end_pos[0] - x0, end_pos[1] - y0
    length = math.sqrt(dx*dx + dy*dy)
    if not length:
        return 0., math.sqrt(x0*x0 + y0*y0), 0., 0.
    ux, uy = dx / length, dy / length
    offset = x0 * uy - y0 * ux
    u_start = x0 * ux + y0 * uy
    u_end = u_start + length
    if u_start * u_end <= 0.:
        # The path crosses the foot of the perpendicular, so its closest
        # approach really is reached within the move
        r_min = abs(offset)
    else:
        r_min = math.sqrt(offset*offset
                          + min(u_start*u_start, u_end*u_end))
    return offset, r_min, u_start, u_end

def closest_approach_fraction(start_pos, end_pos):
    # How far along the move (0..1) the xy path comes closest to the
    # centre, or None when that is one of its ends - where it adds nothing
    # to checking the ends themselves.
    x0, y0 = start_pos[0], start_pos[1]
    dx, dy = end_pos[0] - x0, end_pos[1] - y0
    d2 = dx*dx + dy*dy
    if not d2:
        return None
    t = -(x0*dx + y0*dy) / d2
    if not 0. < t < 1.:
        return None
    return t

def axis_crossing_fraction(start_pos, end_pos, index):
    # How far along the move (0..1) coordinate 'index' changes sign, or
    # None if it does not do so strictly inside the move.  For index 1
    # that is where the path crosses the bed's x axis - where cos(bed
    # angle) is at its extreme, and which a path that is not radial
    # reaches only in its interior.
    s, e = start_pos[index], end_pos[index]
    if (s < 0.) == (e < 0.):
        return None
    t = s / (s - e)
    if not 0. < t < 1.:
        return None
    return t

def zone_crossing(start_pos, end_pos, radius):
    # The fractions (t_in, t_out) of the move spent strictly inside the
    # disc of the given radius about the centre, clipped to the move, or
    # None if the move never enters it.
    offset, r_min, u_start, u_end = path_geometry(start_pos, end_pos)
    if r_min >= radius:
        return None
    length = u_end - u_start
    if not length:
        # Not moving in x/y, and sitting inside the disc
        return 0., 1.
    # r_min >= |offset|, so this is the square root of a positive number
    half = math.sqrt(radius*radius - offset*offset)
    t_in = max(0., (-half - u_start) / length)
    t_out = min(1., (half - u_start) / length)
    return t_in, t_out

def swept_angle(start_pos, end_pos):
    # Signed change in bed angle over an xy segment, in radians.  A
    # straight line can never sweep more than half a turn, so this is just
    # the angle between the two position vectors and needs no unwrapping.
    # An endpoint sitting on the centre has no angle of its own and
    # contributes nothing, which is what makes a move that departs from
    # [0, 0, N] along a ray a zero sweep rather than an undefined one.
    x0, y0 = start_pos[0], start_pos[1]
    x1, y1 = end_pos[0], end_pos[1]
    cross = x0 * y1 - y0 * x1
    dot = x0 * x1 + y0 * y1
    if not cross and not dot:
        return 0.
    return math.atan2(cross, dot)

def crosses_centre(r_min, swept):
    # Whether a move with this closest approach and sweep crosses the axis
    # itself - where the bed angle steps rather than turns, and no
    # feedrate makes a step take longer
    return r_min <= CENTRE_RADIUS and abs(swept) >= MIN_SWEPT_ANGLE


######################################################################
# Angular rates and the limits that follow from them
######################################################################

def peak_angular_velocity(velocity, offset, r_min):
    # Largest |theta_dot| the bed sees over the move.  A radial move never
    # turns the bed at all, and a non-zero offset keeps r_min away from
    # zero, so this never divides by zero.
    if not offset:
        return 0.
    return abs(velocity * offset) / (r_min * r_min)

def peak_angular_accel(velocity, offset, u_start, u_end):
    # Largest |theta_ddot| the geometry alone contributes over the move.
    # Its magnitude rises with |u| to a peak at |offset|/sqrt(3) and falls
    # away after it, so the worst point of this move is that peak clamped
    # into the range of |u| the move actually covers.
    if not offset:
        return 0.
    if u_start * u_end <= 0.:
        u_lo = 0.
    else:
        u_lo = min(abs(u_start), abs(u_end))
    u_hi = max(abs(u_start), abs(u_end))
    u = min(max(abs(offset) * ANGULAR_ACCEL_PEAK_U, u_lo), u_hi)
    r2 = offset*offset + u*u
    return 2. * velocity * velocity * abs(offset) * u / (r2 * r2)

def limits_for_angular_rates(offset, r_min, u_start, u_end,
                             max_angular_v, max_angular_a):
    # Feedrate and acceleration limits that keep the bed inside its own
    # angular velocity and acceleration limits over this move.  Both bed
    # limits land mostly on the feedrate: theta_dot scales with v, and the
    # geometric part of theta_ddot with v squared.  The move's own
    # acceleration contributes a further a*offset/r^2, which is what the
    # second value bounds.  The two theta_ddot terms peak a little way
    # apart along the path (at r_min and at about 1.15*r_min), so each is
    # given the whole budget rather than half of it - close enough for a
    # limit whose constant is empirical anyway.
    #
    # Either value is None where that limit does not bind.
    if not offset:
        return None, None
    v_limit = a_limit = None
    if max_angular_v:
        v_limit = max_angular_v * r_min * r_min / abs(offset)
    if max_angular_a:
        a_limit = max_angular_a * r_min * r_min / abs(offset)
        shape = peak_angular_accel(1., offset, u_start, u_end)
        if shape:
            v_accel = math.sqrt(max_angular_a / shape)
            if v_limit is None or v_accel < v_limit:
                v_limit = v_accel
    return v_limit, a_limit


######################################################################
# Leaving rest and coming to rest at the centre
######################################################################

def _travel(start_pos, end_pos):
    dx, dy = end_pos[0] - start_pos[0], end_pos[1] - start_pos[1]
    if not dx and not dy:
        return None
    return dx, dy

def end_branch(branch, branch_flip):
    # The branch a move leaves the tool standing on
    return -branch if branch_flip else branch

def _last_sample_angle(x, y, travel, branch, branch_flip):
    # The bed angle a move approaches as it comes to rest at x/y: the
    # limit of the dead zone rule from before the move's end.  Only the
    # very last instant can differ from it, and the step generator stops
    # short of that instant - a move that ends on the centre was still
    # heading in.
    along = x * travel[0] + y * travel[1]
    negative = branch < 0
    if branch_flip and along > 0.:
        negative = not negative
    if x*x + y*y < CENTRE_RADIUS * CENTRE_RADIUS:
        angle = math.atan2(travel[1], travel[0])
        if along <= 0.:
            angle = half_turn(angle)
    else:
        angle = math.atan2(y, x)
    if negative:
        return half_turn(angle)
    return angle

def centre_turns(start_pos, end_pos, branch=1, branch_flip=False):
    # How far the bed angle steps, in a single sample, where a move leaves
    # rest at its start and where it comes back to rest at its end: the
    # difference between the angle the position names standing still and
    # the one the dead zone rule drives the bed to there.  Both are zero
    # for a move that starts and ends outside the dead zone, and for one
    # that does not move in x/y.  A non-zero turn at the start is a step
    # the step compressor has to swallow; one at the end leaves the bed
    # facing somewhere the toolhead position does not name.
    travel = _travel(start_pos, end_pos)
    if travel is None:
        return 0., 0.
    sx, sy = start_pos[0], start_pos[1]
    ex, ey = end_pos[0], end_pos[1]
    at_rest = bed_angle(sx, sy, None, branch)
    first = bed_angle(sx, sy, travel,
                      sample_branch(sx, sy, travel, branch, branch_flip))
    last = _last_sample_angle(ex, ey, travel, branch, branch_flip)
    to_rest = bed_angle(ex, ey, None, end_branch(branch, branch_flip))
    return wrap_angle(first - at_rest), wrap_angle(last - to_rest)

def flips_through_centre(start_pos, end_pos, branch=1):
    # Whether a move that starts in the dead zone ends on the far side of
    # the centre along the line the bed faces.  With the bed held still it
    # can only get there by changing over to the other branch where it
    # passes the centre, with the arm travelling through zero radius.
    sx, sy = start_pos[0], start_pos[1]
    if sx*sx + sy*sy >= CENTRE_RADIUS * CENTRE_RADIUS:
        return False
    fx, fy = facing(sx, sy, None, branch)
    return branch * (end_pos[0] * fx + end_pos[1] * fy) < 0.
