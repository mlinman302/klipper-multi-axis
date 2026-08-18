// Generic cartesian kinematics stepper position calculation
//
// Copyright (C) 2024  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct generic_cartesian_stepper {
    struct stepper_kinematics sk;
    struct coord a;
};

// The stepper position is a linear combination of all six motion axes.
// Mixing a linear and a rotational coefficient is what expresses a
// coupled drive: a core r-theta stage, where two motors share one belt to
// set both the radial position and the end-effector rotation, is just
// "0.5*x + 0.5*c" on one motor and "0.5*x - 0.5*c" on the other - the
// same idiom CoreXY already uses for two linear axes.
static double
generic_cartesian_stepper_calc_position(struct stepper_kinematics *sk
                                        , struct move *m, double move_time)
{
    struct generic_cartesian_stepper *cs = container_of(
            sk, struct generic_cartesian_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    return (cs->a.x * c.x + cs->a.y * c.y + cs->a.z * c.z
            + cs->a.a * c.a + cs->a.b * c.b + cs->a.c * c.c);
}

void __visible
generic_cartesian_stepper_set_coeffs(struct stepper_kinematics *sk
                                     , double a_x, double a_y, double a_z
                                     , double a_a, double a_b, double a_c)
{
    struct generic_cartesian_stepper *cs = container_of(
            sk, struct generic_cartesian_stepper, sk);
    cs->a.x = a_x;
    cs->a.y = a_y;
    cs->a.z = a_z;
    cs->a.a = a_a;
    cs->a.b = a_b;
    cs->a.c = a_c;
    cs->sk.active_flags = 0;
    if (a_x) cs->sk.active_flags |= AF_X;
    if (a_y) cs->sk.active_flags |= AF_Y;
    if (a_z) cs->sk.active_flags |= AF_Z;
    if (a_a) cs->sk.active_flags |= AF_A;
    if (a_b) cs->sk.active_flags |= AF_B;
    if (a_c) cs->sk.active_flags |= AF_C;
}

struct stepper_kinematics * __visible
generic_cartesian_stepper_alloc(double a_x, double a_y, double a_z
                                , double a_a, double a_b, double a_c)
{
    struct generic_cartesian_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    cs->sk.calc_position_cb = generic_cartesian_stepper_calc_position;
    generic_cartesian_stepper_set_coeffs(&cs->sk, a_x, a_y, a_z,
                                         a_a, a_b, a_c);
    return &cs->sk;
}
