// Core r-theta kinematics stepper pulse time generation
//
// Copyright (C) 2025  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The machine is a polar (rotating bed) printer whose arm carriage also
// carries a rotating tool:
//
//   * A bed motor turns the build plate.  As on a traditional polar
//     printer this angle is not a commanded axis - it is derived from the
//     cartesian x/y position, together with the arm radius.
//   * Two motors act on the x gantry through a differential.  Turning
//     both the same way rotates the tool about y (the B axis); turning
//     them in opposition moves the arm radially (the X axis).
//   * A leadscrew motor raises the gantry (the Z axis).
//
// The differential is the CoreXY idiom applied to one linear and one
// rotational coordinate, which is only expressible because both travel in
// the same six-axis motion queue - see 'struct coord' in trapq.h.  The
// b_ratio converts a degree of B rotation into the motor travel it costs,
// so that the two terms of the sum share the units of the belt.

#include <math.h> // sqrt, atan2
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct corertheta_stepper {
    struct stepper_kinematics sk;
    double b_ratio;
};

// Below this radius (in mm) the polar angle is not meaningfully defined:
// an arbitrarily small change in x/y swings atan2 by up to pi, and at
// exactly x==y==0 it is degenerate.  Bed rotation also has no effect on
// the tool position there, so holding the last angle is both safe and
// physically faithful.
#define BED_MIN_RADIUS 0.010

// Bed rotation - the polar angle of the commanded cartesian position
static double
corertheta_stepper_bed_calc_position(struct stepper_kinematics *sk
                                     , struct move *m, double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    if (c.x*c.x + c.y*c.y < BED_MIN_RADIUS * BED_MIN_RADIUS)
        // At the centre the angle is indeterminate - hold the previous
        // one rather than let atan2 flip, which would command the bed to
        // make an instantaneous half turn and overrun the step compressor
        return sk->commanded_pos;
    double angle = atan2(c.y, c.x);
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

static void
corertheta_stepper_bed_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}

// First gantry motor: B rotation plus arm radius
static double
corertheta_stepper_plus_calc_position(struct stepper_kinematics *sk
                                      , struct move *m, double move_time)
{
    struct corertheta_stepper *cs = container_of(
        sk, struct corertheta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    return cs->b_ratio * c.b + sqrt(c.x*c.x + c.y*c.y);
}

// Second gantry motor: B rotation minus arm radius
static double
corertheta_stepper_minus_calc_position(struct stepper_kinematics *sk
                                       , struct move *m, double move_time)
{
    struct corertheta_stepper *cs = container_of(
        sk, struct corertheta_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    return cs->b_ratio * c.b - sqrt(c.x*c.x + c.y*c.y);
}

struct stepper_kinematics * __visible
corertheta_stepper_alloc(char type, double b_ratio)
{
    struct corertheta_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    cs->b_ratio = b_ratio;
    if (type == 'c') {
        cs->sk.calc_position_cb = corertheta_stepper_bed_calc_position;
        cs->sk.post_cb = corertheta_stepper_bed_post_fixup;
        cs->sk.active_flags = AF_X | AF_Y;
    } else if (type == '+') {
        cs->sk.calc_position_cb = corertheta_stepper_plus_calc_position;
        cs->sk.active_flags = AF_X | AF_Y | AF_B;
    } else if (type == '-') {
        cs->sk.calc_position_cb = corertheta_stepper_minus_calc_position;
        cs->sk.active_flags = AF_X | AF_Y | AF_B;
    } else {
        errorf("Invalid corertheta stepper type '%c'", type);
        free(cs);
        return NULL;
    }
    return &cs->sk;
}
