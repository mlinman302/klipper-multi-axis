// Rotational Tool Center Point (RTCP) compensation for a tilting head
//
// Copyright (C) 2026  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// GEOMETRY
//
// The head tilts about B.  Two conventions fix the whole model:
//
//   * At B = 0 the nozzle points straight down, exactly as on a printer
//     with no tilting head.  This holds whether or not compensation is
//     switched on.
//   * A positive B tilts the nozzle *outboard* - the tip swings away
//     from the centre of the bed and rises.  A machine that turns the
//     other way inverts B in its kinematics (invert_b_direction), not
//     here.
//
// The tool tip is described relative to the B pivot at B = 0 by two
// offsets, both positive in the direction the tool actually sits:
//
//   tool_vertical    how far the tip is *below* the pivot
//   tool_horizontal  how far the tip is *inboard* of the pivot, ie
//                    closer to the centre of the bed
//
// So the tip vector (tip relative to pivot) at B = 0 is
// (-tool_horizontal, -tool_vertical) in (horizontal, vertical).
// Rotating it by b in the sense above - which carries straight-down
// towards outboard - gives
//
//     tip(b) = (-h*cos b + v*sin b,  -h*sin b - v*cos b)
//
// With compensation ON g-code commands the tip, so the carriages have to
// take up the difference.  Machine coordinates are anchored to agree
// with the commanded position at B = 0, which keeps homing, probing, the
// bed mesh and Z offsets in the frame everything else already uses:
//
//     dh(b) = -(tip_h(b) - tip_h(0)) = h*(cos b - 1) - v*sin b
//     dz(b) = -(tip_v(b) - tip_v(0)) = h*sin b + v*(cos b - 1)
//
//     machine_horizontal = commanded_horizontal + dh(b)
//     machine_vertical   = commanded_vertical   + dz(b)
//
// With compensation OFF both deltas are forced to zero: a B move then
// just turns the head and the carriages hold still.
//
// FRAMES
//
// "Horizontal" is the direction the tip swings in, which is not the same
// axis on every machine:
//
//   RTCP_FRAME_CARTESIAN  the tip swings along +X.  For a cartesian,
//                         corexy, ... gantry.
//   RTCP_FRAME_RADIAL     the tip swings along the arm, in the direction
//                         of increasing arm radius.  For a polar machine
//                         such as corertheta, where x/y are bed
//                         coordinates and the arm travels in radius.  The
//                         arm points the way the bed faces, so the
//                         correction moves the position along that
//                         direction: it changes the arm radius and leaves
//                         the bed angle alone.  Outboard is +r on either
//                         branch (see bed_centre.h) - away from the
//                         centre on the usual branch, towards it on the
//                         negative one - and a large enough correction
//                         carries the arm through the centre onto the
//                         other branch.
//
//                         Since the bed angle does not change, a stepper
//                         that follows the bed angle alone gains nothing
//                         from being wrapped.  [rtcp] leaves the
//                         corertheta bed motor unwrapped, so it goes on
//                         seeing the tool tip the bed angle is defined by
//                         rather than a carriage position that may be
//                         anywhere relative to the centre.
//
// This wraps another stepper_kinematics rather than replacing it, in the
// same way kin_idex.c does, so it composes with cartesian, corexy,
// generic_cartesian or corertheta underneath: the corrected coordinate
// is handed to the original solver.  Because the correction is evaluated
// at every sample time out of the shared six-axis queue (see trapq.h),
// the compensation is continuous through a move rather than only correct
// at its endpoints.

#include <math.h> // sin, cos
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "bed_centre.h" // bed_centre_facing
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "kin_rtcp.h" // RTCP_FRAME_RADIAL
#include "trapq.h" // struct move

#define DUMMY_T 500.0
#define DEG_TO_RAD (M_PI / 180.)

struct rtcp_stepper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    double tool_h, tool_v;
    int frame;
};

// The carriage displacement a tilt of b degrees calls for, in
// (horizontal, vertical).  Zero at b = 0 by construction.
static inline void
rtcp_deltas(double tool_h, double tool_v, double b, double *dh, double *dz)
{
    double br = b * DEG_TO_RAD;
    double sin_b = sin(br), cos_b_1 = cos(br) - 1.;
    *dh = tool_h * cos_b_1 - tool_v * sin_b;
    *dz = tool_h * sin_b + tool_v * cos_b_1;
}

// Apply a (horizontal, vertical) displacement to a cartesian position.
// In the radial frame the horizontal part is taken along (fx, fy), the
// unit vector the bed faces, and the branch the displaced position lies
// on is returned: the arm radius there is the position's own radius
// along the arm plus dh, which is negative once dh has carried it
// through the centre.  The cartesian frame has no branches and hands
// 'branch' back unchanged.
static inline int
rtcp_apply(int frame, double fx, double fy, int branch
           , double dh, double dz, double *x, double *y, double *z)
{
    *z += dz;
    if (frame != RTCP_FRAME_RADIAL) {
        *x += dh;
        return branch;
    }
    double r_machine = *x * fx + *y * fy + dh;
    *x += dh * fx;
    *y += dh * fy;
    if (r_machine < 0.)
        return -1;
    if (r_machine > 0.)
        return 1;
    // On the centre itself: the branch whose position there - standing
    // still, atan2(0, 0) = 0 - faces the same way along x
    return fx < 0. ? -1 : 1;
}

// The direction the bed faces at a position the tool is standing still
// at, on the given branch
static inline void
rtcp_static_facing(double x, double y, int branch, double *fx, double *fy)
{
    struct move m;
    memset(&m, 0, sizeof(m));
    m.branch = branch;
    struct coord c = { .x = x, .y = y };
    bed_centre_facing(&m, &c, fx, fy);
}

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
    double dh, dz, fx = 1., fy = 0.;
    rtcp_deltas(rs->tool_h, rs->tool_v, pos.b, &dh, &dz);
    int branch = move_get_branch(m, &pos);
    if (rs->frame == RTCP_FRAME_RADIAL)
        bed_centre_facing(m, &pos, &fx, &fy);
    // The wrapped solver is handed a position standing still on a known
    // branch, so it needs no flip of its own
    rs->m.branch = rtcp_apply(rs->frame, fx, fy, branch
                              , dh, dz, &pos.x, &pos.y, &pos.z);
    rs->m.branch_flip = 0;
    rs->m.start_pos.x = pos.x;
    rs->m.start_pos.y = pos.y;
    rs->m.start_pos.z = pos.z;
    // Pass the rotations through untouched so a wrapped solver that reads
    // them (eg, a coupled r-theta motor) still sees the right values
    rs->m.start_pos.a = pos.a;
    rs->m.start_pos.b = pos.b;
    rs->m.start_pos.c = pos.c;
    // Carry the direction of travel through - a wrapped solver may need
    // it (the corertheta bed resolves its dead zone from it)
    rs->m.axes_r = m->axes_r;
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
// that a stepper driven by a linear axis must also become active on B:
// tilting the head moves the carriages even when x/y/z are not commanded
// to change, and a move that no axis claims generates no steps at all.
// In the radial frame the correction moves x and y together, so a
// stepper reading either of them picks up the B dependency.
static void
rtcp_update_active_flags(struct rtcp_stepper *rs)
{
    int af = rs->orig_sk->active_flags;
    rs->sk.active_flags = af;
    // In the cartesian frame the correction only touches x and z; in the
    // radial one it moves x and y together, so y joins them
    int linear = AF_X | AF_Z;
    if (rs->frame == RTCP_FRAME_RADIAL)
        linear |= AF_Y;
    if ((rs->tool_h || rs->tool_v) && (af & linear))
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

// Set the tool geometry, in mm: how far the tip sits inboard of the
// pivot and how far below it, both measured at B=0.  Setting both to
// zero disables the correction (and drops the B dependency again), which
// is how [rtcp] implements its off state.
void __visible
rtcp_set_tool(struct stepper_kinematics *sk, double tool_h, double tool_v
              , int frame)
{
    struct rtcp_stepper *rs = container_of(sk, struct rtcp_stepper, sk);
    rs->tool_h = tool_h;
    rs->tool_v = tool_v;
    rs->frame = frame;
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
// tool tip position standing still on the given branch, without going
// through a stepper.  pos_xyz is updated in place and the branch the
// machine position is on is returned.  klippy/extras/rtcp.py repeats
// this pair in Python - it has to convert positions on hosts with no
// compiled c_helper.so - so the two must be kept in step.
int __visible
rtcp_tool_to_machine(double tool_h, double tool_v, int frame, double b
                     , int branch, double *pos_xyz)
{
    double dh, dz, fx, fy;
    rtcp_deltas(tool_h, tool_v, b, &dh, &dz);
    rtcp_static_facing(pos_xyz[0], pos_xyz[1], branch, &fx, &fy);
    return rtcp_apply(frame, fx, fy, branch
                      , dh, dz, &pos_xyz[0], &pos_xyz[1], &pos_xyz[2]);
}

// Inverse transform: recover the tool tip position from a machine
// position on the given branch, and return the branch the tip is on.
// Used when reading positions back out of the steppers (eg, after
// homing) so they are reported in the frame the g-code uses.
int __visible
rtcp_machine_to_tool(double tool_h, double tool_v, int frame, double b
                     , int branch, double *pos_xyz)
{
    double dh, dz, fx, fy;
    rtcp_deltas(tool_h, tool_v, b, &dh, &dz);
    rtcp_static_facing(pos_xyz[0], pos_xyz[1], branch, &fx, &fy);
    return rtcp_apply(frame, fx, fy, branch
                      , -dh, -dz, &pos_xyz[0], &pos_xyz[1], &pos_xyz[2]);
}
