// Bed-frame B axis projection for a rotating-bed (r-theta) machine
//
// Copyright (C) 2026  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// On a core r-theta machine the head tilts about the machine's y axis, so
// the tool can only ever lean within the machine's xz plane.  The bed,
// however, turns underneath it, so a tool orientation expressed in the
// *bed* frame - which is the frame g-code x/y already uses - is in
// general not reachable.
//
// This transform interprets the commanded B as a lean toward the bed's +x
// direction and hands the machine the projection of that lean onto the
// plane it can actually tilt in.  With the bed at angle theta the bed's +x
// direction makes an angle theta with the machine's xz plane, so the lean
// that survives the projection is scaled by cos(theta):
//
//     b_machine = b * cos(theta),      theta = atan2(y, x)
//
// Holding B at 10 degrees through a full turn of the bed therefore sweeps
// the machine's B over 10 -> 0 -> -10 -> 0 -> 10.  cos(theta) is x/|xy|,
// so no trigonometry is needed.
//
// The projection is only wanted for print moves.  Angles beyond max_angle
// are orientation commands - swinging the probe down, parking the head -
// and have to reach the machine untouched.  A hard switch there would be
// a discontinuity of up to max_angle degrees on the machine's B, which the
// step compressor cannot absorb, so the correction is faded out with a
// smoothstep over taper_range degrees above max_angle instead.
//
// Like kin_rtcp.c and kin_shaper.c this wraps another stepper_kinematics
// rather than replacing it, and it is installed *outside* the RTCP
// wrapper: everything downstream - the RTCP tip correction as much as the
// two gantry motors - then sees one consistent B, the one the head is
// really turned to.

#include <math.h> // sqrt, fabs
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

#define DUMMY_T 500.0

// Below this radius (in mm) the bed angle is not meaningfully defined.
// Kept identical to kin_corertheta.c, whose dead zone handling this
// mirrors so that the bed angle used here is the one the bed motor is
// actually being driven to.
#define BED_MIN_RADIUS 0.010

struct bproject_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    double max_angle, taper_range;
};

// cos() of the bed angle at a sampled position.  Inside the dead zone the
// angle comes from the direction of travel, exactly as the bed solver in
// kin_corertheta.c resolves it, so the two never disagree.
static double
bproject_cos_bed_angle(struct move *m, struct coord *c)
{
    double r2 = c->x * c->x + c->y * c->y;
    if (r2 >= BED_MIN_RADIUS * BED_MIN_RADIUS)
        return c->x / sqrt(r2);
    double rx = m->axes_r.x, ry = m->axes_r.y;
    double rn2 = rx * rx + ry * ry;
    if (rn2 <= 0.)
        // Not moving in xy and sitting on the centre - the bed angle has
        // no effect on anything, so leave B alone
        return 1.;
    double cos_t = rx / sqrt(rn2);
    if (c->x * rx + c->y * ry < 0.)
        // Heading inward - the zone was entered from the far side, which
        // is the bed angle turned by pi
        cos_t = -cos_t;
    return cos_t;
}

// The projection itself.  Exposed so that the host code can apply the
// same mapping to a single position without going through a stepper.
double __visible
bproject_project_b(double b, double x, double y
                   , double max_angle, double taper_range)
{
    double ab = fabs(b);
    double none_angle = max_angle + taper_range;
    if (max_angle <= 0. || ab >= none_angle)
        return b;
    struct move m;
    memset(&m, 0, sizeof(m));
    struct coord c = { .x = x, .y = y };
    double cos_t = bproject_cos_bed_angle(&m, &c);
    double w = 1.;
    if (ab > max_angle) {
        // Smoothstep the correction away over the taper band, so the
        // machine's B stays continuous across the switch to pass-through
        double t = (ab - max_angle) / taper_range;
        w = 1. - t * t * (3. - 2. * t);
    }
    return b * (1. + w * (cos_t - 1.));
}

static double
bproject_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct bproject_stepper *bs = container_of(
        sk, struct bproject_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    // The wrapped solver may resolve its position relative to where it
    // already is (the corertheta bed unwraps atan2 that way), so it has
    // to see this stepper's commanded position, not a stale zero
    bs->orig_sk->commanded_pos = sk->commanded_pos;
    double ab = fabs(pos.b);
    double none_angle = bs->max_angle + bs->taper_range;
    if (bs->max_angle > 0. && ab < none_angle) {
        double cos_t = bproject_cos_bed_angle(m, &pos);
        double w = 1.;
        if (ab > bs->max_angle) {
            double t = (ab - bs->max_angle) / bs->taper_range;
            w = 1. - t * t * (3. - 2. * t);
        }
        pos.b *= 1. + w * (cos_t - 1.);
    }
    bs->m.start_pos = pos;
    // Carry the direction of travel through as well - the bed solver
    // needs it to resolve the angle inside the dead zone
    bs->m.axes_r = m->axes_r;
    return bs->orig_sk->calc_position_cb(bs->orig_sk, &bs->m, DUMMY_T);
}

// Forward post_cb (and the commanded_pos it works on) to the wrapped
// solver, for the same reason kin_rtcp.c and kin_shaper.c do
static void
bproject_commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct bproject_stepper *bs = container_of(
        sk, struct bproject_stepper, sk);
    bs->orig_sk->commanded_pos = sk->commanded_pos;
    bs->orig_sk->post_cb(bs->orig_sk);
    sk->commanded_pos = bs->orig_sk->commanded_pos;
}

// A stepper that reads B now reads it through the bed angle, so it gains
// a dependency on x and y.  A move no axis claims generates no steps at
// all, which is why this has to be kept in step with the parameters.
static void
bproject_update_active_flags(struct bproject_stepper *bs)
{
    int af = bs->orig_sk->active_flags;
    bs->sk.active_flags = af;
    if (bs->max_angle > 0. && (af & AF_B))
        bs->sk.active_flags |= AF_X | AF_Y;
}

int __visible
bproject_set_sk(struct stepper_kinematics *sk
                , struct stepper_kinematics *orig_sk)
{
    struct bproject_stepper *bs = container_of(
        sk, struct bproject_stepper, sk);
    if (orig_sk == NULL || orig_sk->calc_position_cb == NULL)
        return -1;
    bs->sk.calc_position_cb = bproject_calc_position;
    bs->orig_sk = orig_sk;
    bs->sk.commanded_pos = orig_sk->commanded_pos;
    if (orig_sk->post_cb)
        bs->sk.post_cb = bproject_commanded_pos_post_fixup;
    bs->sk.gen_steps_pre_active = orig_sk->gen_steps_pre_active;
    bs->sk.gen_steps_post_active = orig_sk->gen_steps_post_active;
    bproject_update_active_flags(bs);
    return 0;
}

// Largest |B| that is fully projected, and the width of the band above it
// over which the projection fades out.  A max_angle of zero disables the
// transform (and drops the x/y dependency again).
void __visible
bproject_set_params(struct stepper_kinematics *sk
                    , double max_angle, double taper_range)
{
    struct bproject_stepper *bs = container_of(
        sk, struct bproject_stepper, sk);
    bs->max_angle = max_angle;
    bs->taper_range = taper_range;
    bproject_update_active_flags(bs);
}

struct stepper_kinematics * __visible
bproject_alloc(void)
{
    struct bproject_stepper *bs = malloc(sizeof(*bs));
    memset(bs, 0, sizeof(*bs));
    bs->m.move_t = 2. * DUMMY_T;
    return &bs->sk;
}
