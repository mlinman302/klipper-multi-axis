#ifndef BED_CENTRE_H
#define BED_CENTRE_H

// Geometry of the bed centre on rotating-bed kinematics
//
// On a polar or core r-theta machine the bed angle is derived from the
// commanded position, theta = atan2(y, x), and has no value at all on the
// line x = y = 0.  Every step generator that needs the bed angle or the
// arm radius takes it from here, so that the bed motor, the gantry
// motors and everything that depends on the angle the bed is driven to -
// the B projection and the RTCP arm correction - agree on it sample by
// sample.
//
// klippy/kinematics/bed_centre.py owns these numbers and these rules, and
// explains them.  It mirrors this header line for line, and
// test/multi_axis/test_bed_centre.py fails if the two ever disagree.
//
// THE TWO BRANCHES
//
// Every x/y position has two polar names, (r, theta) and (-r, theta+pi).
// A move's 'branch' says which one it is solved on: the usual one, with
// the arm at a positive radius, or - on a machine whose arm can travel
// through the centre - the other, with the arm at a negative radius and
// the bed turned the other half turn.  A move marked 'branch_flip'
// changes over where it passes the centre, which is how the tool
// crosses the middle of the bed along a straight line with the bed held
// still.  move_get_branch() in trapq.c is that rule.

#include <math.h> // atan2, sqrt
#include "trapq.h" // struct move, move_get_branch

// Within this radius (in mm) of the centre the bed angle is not
// meaningfully defined, and while the tool is moving in x/y it is taken
// from the direction of travel instead.  CENTRE_RADIUS in bed_centre.py.
#define BED_CENTRE_RADIUS 0.010

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
// bed is already there by the time the radius means something again.  On
// the negative branch the bed is turned the other half turn.
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
// sample.  Scheduling the angle on the axis is a job for the host, which
// sees the moves on either side; see klippy/kinematics/centre_path.py.
static inline double
bed_centre_angle(struct move *m, struct coord *c)
{
    double angle;
    if (!bed_centre_in_zone(m, c)) {
        angle = atan2(c->y, c->x);
    } else {
        angle = atan2(m->axes_r.y, m->axes_r.x);
        if (bed_centre_heading_inward(m, c))
            angle = bed_centre_half_turn(angle);
    }
    if (move_get_branch(m, c) < 0)
        return bed_centre_half_turn(angle);
    return angle;
}

// The unit vector bed_centre_angle() points along - the direction of
// increasing arm radius, in bed coordinates - without the atan2
static inline void
bed_centre_facing(struct move *m, struct coord *c, double *fx, double *fy)
{
    double x, y;
    if (!bed_centre_in_zone(m, c)) {
        double r = sqrt(c->x * c->x + c->y * c->y);
        if (r > 0.) {
            x = c->x / r;
            y = c->y / r;
        } else {
            // atan2(0, 0) is zero
            x = 1.;
            y = 0.;
        }
    } else {
        double rx = m->axes_r.x, ry = m->axes_r.y;
        double len = sqrt(rx * rx + ry * ry);
        x = rx / len;
        y = ry / len;
        if (bed_centre_heading_inward(m, c)) {
            x = -x;
            y = -y;
        }
    }
    if (move_get_branch(m, c) < 0) {
        x = -x;
        y = -y;
    }
    *fx = x;
    *fy = y;
}

// cos(bed_centre_angle())
static inline double
bed_centre_cos(struct move *m, struct coord *c)
{
    double fx, fy;
    bed_centre_facing(m, c, &fx, &fy);
    return fx;
}

// The arm radius for a sample - negative on the negative branch
static inline double
bed_centre_radius(struct move *m, struct coord *c)
{
    return move_get_branch(m, c) * sqrt(c->x * c->x + c->y * c->y);
}

#endif // bed_centre.h
