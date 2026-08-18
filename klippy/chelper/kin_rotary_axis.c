// Rotational (A/B/C) axis stepper pulse time generation
//
// Copyright (C) 2025  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// This is the *uncoupled* rotational stepper: one motor drives one
// rotational axis directly.  It reads its angle (in degrees) out of the
// shared six-axis motion space - see 'struct coord' in trapq.h - rather
// than from a private queue, so that its motion is on the same time base
// as the linear axes.
//
// Drives where a motor position depends on several axes at once (eg, a
// core r-theta stage) use generic_cartesian_stepper_alloc() instead,
// which takes a coefficient per axis.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

// Index of the a/b/c components within struct coord
#define ROTARY_AXIS_OFFSET 3

struct rotary_axis_stepper {
    struct stepper_kinematics sk;
    int axis_index; // index into struct coord.axis[]
};

static double
rotary_axis_calc_position(struct stepper_kinematics *sk, struct move *m
                          , double move_time)
{
    struct rotary_axis_stepper *rs = container_of(
        sk, struct rotary_axis_stepper, sk);
    return move_get_coord(m, move_time).axis[rs->axis_index];
}

struct stepper_kinematics * __visible
rotary_axis_stepper_alloc(char axis)
{
    if (axis != 'a' && axis != 'b' && axis != 'c') {
        errorf("Invalid rotary axis '%c'", axis);
        return NULL;
    }
    struct rotary_axis_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->axis_index = ROTARY_AXIS_OFFSET + (axis - 'a');
    rs->sk.calc_position_cb = rotary_axis_calc_position;
    rs->sk.active_flags = AF_A << (axis - 'a');
    return &rs->sk;
}

// Return the cartesian axis that this rotary axis rotates about
// ('x' for A, 'y' for B, 'z' for C)
char __visible
rotary_axis_get_rotation_axis(struct stepper_kinematics *sk)
{
    struct rotary_axis_stepper *rs = container_of(
        sk, struct rotary_axis_stepper, sk);
    return 'x' + (rs->axis_index - ROTARY_AXIS_OFFSET);
}
