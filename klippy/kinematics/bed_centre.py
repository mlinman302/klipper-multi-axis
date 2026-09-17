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
import math


######################################################################
# Constants - mirrored in klippy/chelper/bed_centre.h
######################################################################

# Within this radius (in mm) of the centre the bed angle is not
# meaningfully defined: an arbitrarily small change in x/y swings atan2 by
# up to pi.  Inside it, while the tool is moving in x/y, the bed angle is
# taken from the direction of travel instead - see bed_angle() below.
CENTRE_RADIUS = 0.010
# A position this close to the centre line is on it, to within rounding.
# The radial RTCP correction scales x and y about the centre, which has no
# direction to scale along here.
#
# This is not CENTRE_RADIUS, and the gap between them is deliberate but
# worth knowing: between the two, the RTCP correction is applied along the
# position's own angle while the bed is driven to the angle of travel.
# The two agree on every path that stays radial inside the zone - the only
# paths [polar_singularity] lets in there - and disagree on the paths it
# refuses.
CENTRE_EPSILON = 1e-9

# A sweep smaller than this is not worth refusing a move over
MIN_SWEPT_ANGLE = math.radians(1.)
# |u| / |offset| at which the geometric part of theta_ddot peaks, and the
# peak itself as a multiple of v^2 / offset^2
ANGULAR_ACCEL_PEAK_U = 1. / math.sqrt(3.)
ANGULAR_ACCEL_PEAK = 9. / (8. * math.sqrt(3.))


######################################################################
# The bed angle at a sample - mirrors bed_centre_angle() and
# bed_centre_cos() in klippy/chelper/bed_centre.h
######################################################################

# 'travel' is the move's direction in x/y (its axes_r.x and axes_r.y), or
# None for a position that is not moving.  Outside the dead zone, or when
# not moving in x/y, the bed angle is plain atan2 of the position.  Inside
# it, while moving, the angle is the one the path has where it leaves the
# zone - or, while the tool is still heading inward, where it entered.
# The bed is then already at the right angle by the time the radius means
# something again, so nothing has to turn at the boundary.
#
# This is the rule the step generators actually run, which is why it is
# mirrored here rather than improved.  It is right for the homing sweep
# it was written for, which departs from the centre along +x.  It is not
# right in general: a move that ends exactly on the centre is "not heading
# inward" at its last sample and so flips the bed by pi there, and a move
# that starts from the centre along a new ray steps the bed at its first.
# Scheduling the bed angle while the tool is on the axis is the job of the
# layer above this one; nothing at the level of a single sample can do it,
# because a sample cannot tell a move that is arriving from one that is
# leaving.

def half_turn(angle):
    # angle + pi, folded back into (-pi, pi] - the same half turn the C
    # applies, so both sides agree to the bit
    return angle + (-math.pi if angle > 0. else math.pi)

def _moving(travel):
    return travel is not None and bool(travel[0] or travel[1])

def _in_zone(x, y, travel):
    return (x*x + y*y < CENTRE_RADIUS * CENTRE_RADIUS) and _moving(travel)

def _heading_inward(x, y, travel):
    return x * travel[0] + y * travel[1] < 0.

def bed_angle(x, y, travel=None):
    # The bed angle for a sample, in (-pi, pi], before unwrapping
    if not _in_zone(x, y, travel):
        return math.atan2(y, x)
    angle = math.atan2(travel[1], travel[0])
    if _heading_inward(x, y, travel):
        return half_turn(angle)
    return angle

def cos_bed_angle(x, y, travel=None):
    # cos(bed_angle()), computed without the atan2 on the common path
    if not _in_zone(x, y, travel):
        r2 = x*x + y*y
        if r2 <= 0.:
            # atan2(0, 0) is zero
            return 1.
        return x / math.sqrt(r2)
    cos_t = travel[0] / math.sqrt(travel[0]**2 + travel[1]**2)
    if _heading_inward(x, y, travel):
        return -cos_t
    return cos_t

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
# Signed radius
######################################################################

# Every x/y position has two polar names: (r, theta) and (-r, theta + pi).
# The kinematics only ever use the first, which is why a straight line
# through the centre is a half turn of the bed.  On the second branch the
# same line is a plain radial move through r = 0 with theta held - no
# rotation at all - provided the arm can travel to a negative radius.
#
# Nothing drives a signed radius yet.  These are the conversions a move
# carrying its branch will need, kept here so that the arithmetic has one
# definition from the start.  The angle is kept in (-pi, pi] on both
# branches by half_turn(), the same half turn the dead zone applies.

def polar_position(x, y, branch=1):
    # (r, theta) for a position on the given branch: +1 is the usual
    # non-negative radius, -1 the negative one
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    if branch < 0:
        return -r, half_turn(theta)
    return r, theta

def cartesian_position(r, theta):
    # The inverse of polar_position(), for either branch
    return r * math.cos(theta), r * math.sin(theta)

def signed_radius(x, y, theta):
    # The radius at which bed angle theta puts the tool at x/y - negative
    # when the position lies behind theta.  Exact only for positions on
    # the line through the centre at that angle, which are the only ones a
    # branch-holding move ever visits.
    return x * math.cos(theta) + y * math.sin(theta)

def branch_for_angle(x, y, theta):
    # Which branch names x/y at bed angle theta
    if signed_radius(x, y, theta) < 0.:
        return -1
    return 1
