// Rotational Tool Center Point (RTCP) compensation for a tilting head
//
// Copyright (C) 2025  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The machine has a head that tilts about B (an axis parallel to Y).
// The g-code commands where the *tool tip* should be; as the head tilts,
// the tip swings about the pivot, so the linear carriages must move to
// hold the tip where it was asked to be.
//
// Let the pivot sit at offset (px, pz) from the tool tip when B = 0, so
// the tool vector (tip relative to pivot) is (-px, -pz).  Rotating the
// head by b about Y turns that vector, and the pivot must go to
//
//     P = T - R_y(b) * (-px, -py, -pz)
//
// Subtracting the b = 0 offset so that machine coordinates agree with the
// commanded tip position at b = 0 (which keeps homing, bed mesh and Z
// offsets in the frame everything else already uses) gives:
//
//     machine_x = x + px*(cos b - 1) + pz*sin b
//     machine_y = y
//     machine_z = z - px*sin b       + pz*(cos b - 1)
//
// This wraps another stepper_kinematics rather than replacing it, in the
// same way kin_idex.c does, so it composes with cartesian, corexy or
// generic_cartesian underneath: the corrected coordinate is handed to the
// original solver.  Because the correction is evaluated at every sample
// time out of the shared six-axis queue (see trapq.h), the compensation
// is continuous through a move rather than only correct at its endpoints.

#include <math.h> // sin, cos
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

#define DUMMY_T 500.0
#define DEG_TO_RAD (M_PI / 180.)

struct rtcp_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    double pivot_x, pivot_z;
};

static double
rtcp_calc_position(struct stepper_kinematics *sk, struct move *m
                   , double move_time)
{
    struct rtcp_stepper *rs = container_of(sk, struct rtcp_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    // The wrapped solver may resolve its position relative to where it
    // already is (the corertheta bed unwraps atan2 that way), so it has
    // to see this stepper's commanded position, not a stale zero
    rs->orig_sk->commanded_pos = sk->commanded_pos;
    double b = pos.b * DEG_TO_RAD;
    double sin_b = sin(b), cos_b_1 = cos(b) - 1.;
    rs->m.start_pos.x = pos.x + rs->pivot_x * cos_b_1 + rs->pivot_z * sin_b;
    rs->m.start_pos.y = pos.y;
    rs->m.start_pos.z = pos.z - rs->pivot_x * sin_b + rs->pivot_z * cos_b_1;
    // Pass the rotations through untouched so a wrapped solver that reads
    // them (eg, a coupled r-theta motor) still sees the right values
    rs->m.start_pos.a = pos.a;
    rs->m.start_pos.b = pos.b;
    rs->m.start_pos.c = pos.c;
    return rs->orig_sk->calc_position_cb(rs->orig_sk, &rs->m, DUMMY_T);
}

// Forward post_cb (and the commanded_pos it works on) to the wrapped
// solver - kin_shaper.c does the same for the same reason.  A solver
// that keeps state in commanded_pos (eg, the corertheta bed angle, whose
// atan2 result has to be unwrapped against where the bed already is)
// otherwise never sees its own position advance.
static void
rtcp_commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct rtcp_stepper *rs = container_of(sk, struct rtcp_stepper, sk);
    rs->orig_sk->commanded_pos = sk->commanded_pos;
    rs->orig_sk->post_cb(rs->orig_sk);
    sk->commanded_pos = rs->orig_sk->commanded_pos;
}

// Recompute which axes this stepper responds to.  The important part is
// that a stepper driven by X or Z must also become active on B: tilting
// the head moves those carriages even when x/y/z are not commanded to
// change, and a move that no axis claims generates no steps at all.
static void
rtcp_update_active_flags(struct rtcp_stepper *rs)
{
    int af = rs->orig_sk->active_flags;
    rs->sk.active_flags = af;
    if ((rs->pivot_x || rs->pivot_z) && (af & (AF_X | AF_Z)))
        rs->sk.active_flags |= AF_B;
}

int __visible
rtcp_set_sk(struct stepper_kinematics *sk, struct stepper_kinematics *orig_sk)
{
    struct rtcp_stepper *rs = container_of(sk, struct rtcp_stepper, sk);
    if (orig_sk == NULL || orig_sk->calc_position_cb == NULL)
        return -1;
    rs->sk.calc_position_cb = rtcp_calc_position;
    rs->orig_sk = orig_sk;
    rs->sk.commanded_pos = orig_sk->commanded_pos;
    if (orig_sk->post_cb)
        rs->sk.post_cb = rtcp_commanded_pos_post_fixup;
    rs->sk.gen_steps_pre_active = orig_sk->gen_steps_pre_active;
    rs->sk.gen_steps_post_active = orig_sk->gen_steps_post_active;
    rtcp_update_active_flags(rs);
    return 0;
}

// Offsets from the tool tip to the pivot at B=0, in mm.  Setting both to
// zero disables the correction (and drops the B dependency again).
void __visible
rtcp_set_pivot(struct stepper_kinematics *sk, double pivot_x, double pivot_z)
{
    struct rtcp_stepper *rs = container_of(sk, struct rtcp_stepper, sk);
    rs->pivot_x = pivot_x;
    rs->pivot_z = pivot_z;
    rtcp_update_active_flags(rs);
}

struct stepper_kinematics * __visible
rtcp_alloc(void)
{
    struct rtcp_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->m.move_t = 2. * DUMMY_T;
    return &rs->sk;
}

// Forward transform, for callers that need the machine position of a
// given tool tip position without going through a stepper
void __visible
rtcp_tip_to_machine(double pivot_x, double pivot_z, double b
                    , double *pos_xz)
{
    double br = b * DEG_TO_RAD;
    double sin_b = sin(br), cos_b_1 = cos(br) - 1.;
    double x = pos_xz[0], z = pos_xz[1];
    pos_xz[0] = x + pivot_x * cos_b_1 + pivot_z * sin_b;
    pos_xz[1] = z - pivot_x * sin_b + pivot_z * cos_b_1;
}

// Inverse transform: recover the tool tip position from a machine
// position.  Used when reading positions back out of the steppers (eg,
// after homing) so they are reported in the frame the g-code uses.
void __visible
rtcp_machine_to_tip(double pivot_x, double pivot_z, double b
                    , double *pos_xz)
{
    double br = b * DEG_TO_RAD;
    double sin_b = sin(br), cos_b_1 = cos(br) - 1.;
    double x = pos_xz[0], z = pos_xz[1];
    pos_xz[0] = x - pivot_x * cos_b_1 - pivot_z * sin_b;
    pos_xz[1] = z + pivot_x * sin_b - pivot_z * cos_b_1;
}
