#ifndef BED_CENTRE_H
#define BED_CENTRE_H

// Geometry of the bed centre on rotating-bed kinematics
//
// On a polar or core r-theta machine the bed angle is derived from the
// commanded position, theta = atan2(y, x), and has no value at all on the
// line x = y = 0.  Every step generator that needs the bed angle takes it
// from here, so that the bed motor and everything that depends on the
// angle it is being driven to - the B projection, for one - agree on it
// sample by sample.
//
// klippy/kinematics/bed_centre.py owns these numbers and this rule, and
// explains them.  It mirrors this header line for line, and
// test/multi_axis/test_bed_centre.py fails if the two ever disagree.

#include <math.h> // atan2, sqrt
#include "trapq.h" // struct move

// Within this radius (in mm) of the centre the bed angle is not
// meaningfully defined, and while the tool is moving in x/y it is taken
// from the direction of travel instead.  CENTRE_RADIUS in bed_centre.py.
#define BED_CENTRE_RADIUS 0.010
// A position this close to the centre line is on it, to within rounding -
// the radial RTCP correction has no direction to scale along there.
// CENTRE_EPSILON in bed_centre.py, which also notes the gap between the
// two.
#define BED_CENTRE_EPSILON 1e-9

static inline int
bed_centre_in_zone(struct move *m, struct coord *c)
{
    return (c->x * c->x + c->y * c->y
            < BED_CENTRE_RADIUS * BED_CENTRE_RADIUS)
        && (m->axes_r.x || m->axes_r.y);
}

static inline int
bed_centre_heading_inward(struct move *m, struct coord *c)
{
    return c->x * m->axes_r.x + c->y * m->axes_r.y < 0.;
}

// angle + pi, folded back into (-pi, pi]
static inline double
bed_centre_half_turn(double angle)
{
    return angle + (angle > 0. ? -M_PI : M_PI);
}

// The bed angle for a sample, in (-pi, pi], before unwrapping.  Outside
// the dead zone, or when not moving in x/y, it is atan2 of the position.
// Inside it, while moving, it is the angle the path has where it leaves
// the zone - or, while still heading inward, where it entered - so the
// bed is already there by the time the radius means something again.
//
// This has to stay a pure function of (move, sample).
// itersolve_set_position() runs the bed solver over a zeroed move, so a
// rule that held on to where the bed already was would make setting the
// position at the centre a no-op - which is what homing R does with a
// position_min of zero.
//
// Note what it cannot do: a sample cannot tell a move that is arriving at
// the centre from one that is leaving it.  A move that leaves the centre
// along any ray but the one the bed faces steps the bed at its first
// sample, and a sample on the bare centre while the bed is live - which
// RTCP makes it on a B move - falls back to atan2 of wherever the
// carriage is.  Scheduling the angle on the axis is a job for the host,
// which sees the moves on either side; see
// klippy/kinematics/centre_path.py.
static inline double
bed_centre_angle(struct move *m, struct coord *c)
{
    if (!bed_centre_in_zone(m, c))
        return atan2(c->y, c->x);
    double angle = atan2(m->axes_r.y, m->axes_r.x);
    if (bed_centre_heading_inward(m, c))
        return bed_centre_half_turn(angle);
    return angle;
}

// cos(bed_centre_angle()), without the atan2 on the common path
static inline double
bed_centre_cos(struct move *m, struct coord *c)
{
    if (!bed_centre_in_zone(m, c)) {
        double r2 = c->x * c->x + c->y * c->y;
        if (r2 <= 0.)
            // atan2(0, 0) is zero
            return 1.;
        return c->x / sqrt(r2);
    }
    double rx = m->axes_r.x, ry = m->axes_r.y;
    double cos_t = rx / sqrt(rx * rx + ry * ry);
    if (bed_centre_heading_inward(m, c))
        return -cos_t;
    return cos_t;
}

#endif // bed_centre.h
