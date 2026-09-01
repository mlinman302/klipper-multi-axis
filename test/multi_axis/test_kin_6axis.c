// Host test of the six-axis (X Y Z A B C) motion space.
//
// Copyright (C) 2025  Klipper multi-axis contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Covers the three things the widening of 'struct coord' was for:
//   1. rotational axes are sampled from the same trapq, on the same time
//      base, as the linear axes;
//   2. a coupled drive (core r-theta) can be expressed as a linear
//      combination of a linear and a rotational axis;
//   3. the existing 3-axis kinematics are bit-for-bit unaffected.
//
// Build and run with test/multi_axis/run_c_tests.sh

#include <math.h>
#include <stddef.h> // offsetof
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "itersolve.h"
#include "list.h"
#include "stepcompress.h"
#include "trapq.h"

extern struct stepper_kinematics *rotary_axis_stepper_alloc(char axis);
extern char rotary_axis_get_rotation_axis(struct stepper_kinematics *sk);
extern struct stepper_kinematics *cartesian_stepper_alloc(char axis);
extern struct stepper_kinematics *corexy_stepper_alloc(char type);
extern struct stepper_kinematics *generic_cartesian_stepper_alloc(
    double a_x, double a_y, double a_z, double a_a, double a_b, double a_c);
extern struct stepper_kinematics *corertheta_stepper_alloc(char type,
                                                           double b_ratio);
extern struct stepper_kinematics *bproject_alloc(void);
extern int bproject_set_sk(struct stepper_kinematics *sk,
                           struct stepper_kinematics *orig_sk);
extern void bproject_set_params(struct stepper_kinematics *sk,
                                double max_angle, double taper_range);
extern double bproject_project_b(double b, double x, double y,
                                 double max_angle, double taper_range);
extern struct stepper_kinematics *rtcp_alloc(void);
extern int rtcp_set_sk(struct stepper_kinematics *sk,
                       struct stepper_kinematics *orig_sk);
extern void rtcp_set_pivot(struct stepper_kinematics *sk,
                           double pivot_x, double pivot_z);

#define NEVER_TIME 9999999999999999.9

static int failures = 0;

static void
check(const char *what, double got, double want, double tol)
{
    if (!(fabs(got - want) <= tol)) {
        printf("FAIL %-46s got %.9f want %.9f\n", what, got, want);
        failures++;
    } else {
        printf("ok   %-46s %.6f\n", what, got);
    }
}

// Sample a stepper at an absolute print_time by walking the trapq.
// Sentinel moves are skipped and the time is clamped into the chosen
// move so a request landing exactly on a boundary is not lost to
// floating point rounding.
static double
sample(struct trapq *tq, struct stepper_kinematics *sk, double print_time)
{
    struct move *m, *best = NULL;
    list_for_each_entry(m, &tq->moves, node) {
        if (m->print_time < 0. || m->move_t <= 0. || m->move_t >= NEVER_TIME)
            continue;
        if (m->print_time <= print_time || best == NULL)
            best = m;
    }
    if (best == NULL) {
        printf("FAIL no move covers print_time %.6f\n", print_time);
        failures++;
        return 0. / 0.;
    }
    double mt = print_time - best->print_time;
    if (mt < 0.)
        mt = 0.;
    if (mt > best->move_t)
        mt = best->move_t;
    return sk->calc_position_cb(sk, best, mt);
}

// Queue a single constant-velocity move covering the given deltas in
// move_t seconds.  Mirrors what ToolHead._process_lookahead() emits.
static struct trapq *
queue_move(double t0, double move_t, const double start[KIN_AXES],
           const double delta[KIN_AXES])
{
    // The trapq stores a scalar distance plus a unit-ish ratio per axis.
    // Use a nominal distance of 1 so that axes_r carries the deltas.
    double v = 1. / move_t;
    struct trapq *tq = trapq_alloc();
    trapq_append(tq, t0, 0., move_t, 0.,
                 start[0], start[1], start[2],
                 start[3], start[4], start[5],
                 delta[0], delta[1], delta[2],
                 delta[3], delta[4], delta[5],
                 v, v, 0.);
    return tq;
}

static void
test_rotary_shares_time_base(void)
{
    printf("\n-- rotational axes share the linear time base --\n");
    struct stepper_kinematics *skx = cartesian_stepper_alloc('x');
    struct stepper_kinematics *ska = rotary_axis_stepper_alloc('a');
    struct stepper_kinematics *skb = rotary_axis_stepper_alloc('b');
    struct stepper_kinematics *skc = rotary_axis_stepper_alloc('c');

    check("A rotates about x", rotary_axis_get_rotation_axis(ska), 'x', 0.);
    check("B rotates about y", rotary_axis_get_rotation_axis(skb), 'y', 0.);
    check("C rotates about z", rotary_axis_get_rotation_axis(skc), 'z', 0.);
    check("A active flag", ska->active_flags, AF_A, 0.);
    check("B active flag", skb->active_flags, AF_B, 0.);
    check("C active flag", skc->active_flags, AF_C, 0.);
    if (rotary_axis_stepper_alloc('q') != NULL) {
        printf("FAIL invalid rotary axis accepted\n");
        failures++;
    } else {
        printf("ok   invalid rotary axis rejected\n");
    }

    // X 10 -> 110 while C 0 -> 90, in one move of 2 seconds
    double t0 = 0.100, move_t = 2.0;
    double start[KIN_AXES] = {10., 0., 0., 0., 0., 0.};
    double delta[KIN_AXES] = {100., 0., 0., 0., 0., 90.};
    struct trapq *tq = queue_move(t0, move_t, start, delta);

    check("X at t=0", sample(tq, skx, t0), 10., 1e-9);
    check("C at t=0", sample(tq, skc, t0), 0., 1e-9);
    check("X at t=end", sample(tq, skx, t0 + move_t), 110., 1e-9);
    check("C at t=end", sample(tq, skc, t0 + move_t), 90., 1e-9);
    // The whole point: at any shared instant both axes are at the same
    // fraction of their travel
    for (int i = 1; i < 8; i++) {
        double frac = i / 8.;
        double t = t0 + move_t * frac;
        double x = sample(tq, skx, t), c = sample(tq, skc, t);
        double x_frac = (x - 10.) / 100., c_frac = c / 90.;
        if (fabs(x_frac - c_frac) > 1e-9) {
            printf("FAIL X and C out of sync at frac %.3f: %.9f vs %.9f\n",
                   frac, x_frac, c_frac);
            failures++;
        }
    }
    printf("ok   X and C stay in sync across the move\n");
    // An axis not involved in the move stays put
    check("B unaffected", sample(tq, skb, t0 + move_t), 0., 1e-9);
    trapq_free(tq);
}

static void
test_core_r_theta(void)
{
    printf("\n-- coupled drive: core r-theta --\n");
    // Two motors on one belt: motor1 = 0.5*x + 0.5*c, motor2 = 0.5*x - 0.5*c
    struct stepper_kinematics *m1 = generic_cartesian_stepper_alloc(
        0.5, 0., 0., 0., 0., 0.5);
    struct stepper_kinematics *m2 = generic_cartesian_stepper_alloc(
        0.5, 0., 0., 0., 0., -0.5);
    check("motor1 active flags", m1->active_flags, AF_X | AF_C, 0.);
    check("motor2 active flags", m2->active_flags, AF_X | AF_C, 0.);

    double t0 = 0.1, move_t = 1.0;
    double zero[KIN_AXES] = {0., 0., 0., 0., 0., 0.};

    // Pure radial move: both motors turn together
    double d_x[KIN_AXES] = {100., 0., 0., 0., 0., 0.};
    struct trapq *tq_x = queue_move(t0, move_t, zero, d_x);
    check("pure X: motor1", sample(tq_x, m1, t0 + move_t), 50., 1e-9);
    check("pure X: motor2", sample(tq_x, m2, t0 + move_t), 50., 1e-9);

    // Pure rotation: motors turn in opposition
    double d_c[KIN_AXES] = {0., 0., 0., 0., 0., 90.};
    struct trapq *tq_c = queue_move(t0, move_t, zero, d_c);
    check("pure C: motor1", sample(tq_c, m1, t0 + move_t), 45., 1e-9);
    check("pure C: motor2", sample(tq_c, m2, t0 + move_t), -45., 1e-9);

    // Combined move: superposition, evaluated at one shared instant
    double d_xc[KIN_AXES] = {100., 0., 0., 0., 0., 90.};
    struct trapq *tq_xc = queue_move(t0, move_t, zero, d_xc);
    check("X+C: motor1", sample(tq_xc, m1, t0 + move_t), 95., 1e-9);
    check("X+C: motor2", sample(tq_xc, m2, t0 + move_t), 5., 1e-9);
    check("X+C: motor1 midway", sample(tq_xc, m1, t0 + move_t / 2.),
          47.5, 1e-9);
    check("X+C: motor2 midway", sample(tq_xc, m2, t0 + move_t / 2.),
          2.5, 1e-9);

    // A rotation-only move must still drive the coupled motors - this is
    // the case that silently did nothing before the trapq was shared
    if (sample(tq_c, m1, t0 + move_t) == 0.) {
        printf("FAIL rotation-only move produced no motor motion\n");
        failures++;
    } else {
        printf("ok   rotation-only move drives the coupled motors\n");
    }
    trapq_free(tq_x);
    trapq_free(tq_c);
    trapq_free(tq_xc);
}

// Mirrors BED_MIN_RADIUS in kin_corertheta.c
#define BED_HOLD_R 0.010

static void
test_corertheta(void)
{
    printf("\n-- corertheta: polar bed plus a B differential --\n");
    double b_ratio = 2.;
    struct stepper_kinematics *bed = corertheta_stepper_alloc('c', b_ratio);
    struct stepper_kinematics *mp = corertheta_stepper_alloc('+', b_ratio);
    struct stepper_kinematics *mm = corertheta_stepper_alloc('-', b_ratio);
    check("bed active flags", bed->active_flags, AF_X | AF_Y, 0.);
    check("motor+ active flags", mp->active_flags, AF_X | AF_Y | AF_B, 0.);
    check("motor- active flags", mm->active_flags, AF_X | AF_Y | AF_B, 0.);
    if (corertheta_stepper_alloc('q', 1.) != NULL) {
        printf("FAIL invalid corertheta type accepted\n");
        failures++;
    } else {
        printf("ok   invalid corertheta type rejected\n");
    }

    double t0 = 0.1, move_t = 1.0;
    double zero[KIN_AXES] = {0., 0., 0., 0., 0., 0.};

    // Pure radial move along +X: the motors turn in opposition and the
    // bed angle stays at zero
    double d_x[KIN_AXES] = {100., 0., 0., 0., 0., 0.};
    struct trapq *tq_x = queue_move(t0, move_t, zero, d_x);
    check("pure X: motor+", sample(tq_x, mp, t0 + move_t), 100., 1e-9);
    check("pure X: motor-", sample(tq_x, mm, t0 + move_t), -100., 1e-9);
    check("pure X: bed angle", sample(tq_x, bed, t0 + move_t), 0., 1e-9);

    // Pure B rotation: the motors turn together and the radius is unchanged
    double start_r[KIN_AXES] = {50., 0., 0., 0., 0., 0.};
    double d_b[KIN_AXES] = {0., 0., 0., 0., 30., 0.};
    struct trapq *tq_b = queue_move(t0, move_t, start_r, d_b);
    check("pure B: motor+", sample(tq_b, mp, t0 + move_t),
          b_ratio * 30. + 50., 1e-9);
    check("pure B: motor-", sample(tq_b, mm, t0 + move_t),
          b_ratio * 30. - 50., 1e-9);
    // Solve the differential back: same direction is B, opposition is r
    double pp = sample(tq_b, mp, t0 + move_t);
    double pm = sample(tq_b, mm, t0 + move_t);
    check("pure B: recovered radius", .5 * (pp - pm), 50., 1e-9);
    check("pure B: recovered B", .5 * (pp + pm) / b_ratio, 30., 1e-9);

    // The bed carries the polar angle of the commanded cartesian position
    double d_y[KIN_AXES] = {-50., 50., 0., 0., 0., 0.};
    struct trapq *tq_y = queue_move(t0, move_t, start_r, d_y);
    check("bed angle at +Y", sample(tq_y, bed, t0 + move_t), M_PI / 2., 1e-9);
    check("radius held at +Y", .5 * (sample(tq_y, mp, t0 + move_t)
                                     - sample(tq_y, mm, t0 + move_t)),
          50., 1e-9);

    // A B-only move must still drive the gantry motors
    if (sample(tq_b, mp, t0 + move_t) == sample(tq_b, mp, t0)) {
        printf("FAIL B-only move produced no motor motion\n");
        failures++;
    } else {
        printf("ok   B-only move drives the gantry motors\n");
    }
    // Centre singularity.  The polar angle is indeterminate at r=0, and
    // near it an arbitrarily small x/y change swings atan2 by up to pi,
    // which would command the bed to make an instantaneous half turn and
    // overrun the step compressor ("Internal error in stepcompress" on the
    // bed queue).  Inside the dead radius the angle is therefore taken
    // from the direction of travel, so that it already agrees with atan2
    // where the path leaves the zone and nothing turns at the boundary.
    //
    // Crucially it stays a pure function of the move - it must NOT hold
    // sk->commanded_pos.  itersolve_set_position() runs this callback over
    // a zeroed move, so a stateful hold made "set the position to the
    // centre" a no-op.  That is exactly what homing R does when
    // stepper_r's position_min is 0, and the stale angle then tore through
    // the step compressor as soon as the homing move left the dead zone.
    //
    // The sweep below runs x from -1 to +1, so a fraction f of move_t sits
    // at x = 2f - 1.
    double thru[KIN_AXES] = {-1., 0., 0., 0., 0., 0.};
    double d_thru[KIN_AXES] = {2., 0., 0., 0., 0., 0.};
    struct trapq *tq_o = queue_move(t0, move_t, thru, d_thru);
    double t_before = t0 + move_t * (1. - 2. * BED_HOLD_R) / 2.; // x=-2*hold
    double t_in = t0 + move_t * (1. - BED_HOLD_R / 2.) / 2.;     // x=-hold/2
    double t_out = t0 + move_t * (1. + BED_HOLD_R / 2.) / 2.;    // x=+hold/2
    bed->commanded_pos = M_PI;
    check("outside the zone: tracks atan2",
          sample(tq_o, bed, t_before), M_PI, 1e-9);
    check("centre: inbound half keeps the entry angle",
          sample(tq_o, bed, t_in), M_PI, 1e-9);
    bed->commanded_pos = 0.;
    check("centre: outbound half sits at the exit angle",
          sample(tq_o, bed, t_out), 0., 1e-9);
    // ...and the held angle does not depend on where the bed happens to be
    bed->commanded_pos = -M_PI / 2.;
    check("centre: hold ignores the previous angle",
          sample(tq_o, bed, t_out), 0., 1e-9);
    // Outside the hold radius the angle tracks atan2 again
    bed->commanded_pos = 0.;
    check("outside hold radius: tracks atan2",
          sample(tq_o, bed, t0 + move_t), 0., 1e-9);
    // The gantry motors are unaffected by the hold - radius is |x|
    check("centre: motor+ still tracks radius",
          sample(tq_o, mp, t0 + move_t), 1., 1e-9);
    // Regression: setting the position at the centre must reset the bed
    // angle rather than leave the previous one in place
    itersolve_set_position(bed, cos(2.) * 100., sin(2.) * 100.,
                           0., 0., 0., 0.);
    check("set_position away from the centre", bed->commanded_pos, 2., 1e-9);
    itersolve_set_position(bed, 0., 0., 0., 0., 0., 0.);
    check("set_position at the centre is not a stale hold",
          bed->commanded_pos, 0., 1e-9);

    trapq_free(tq_x);
    trapq_free(tq_b);
    trapq_free(tq_y);
    trapq_free(tq_o);
}

static void
test_existing_kinematics_unchanged(void)
{
    printf("\n-- regression: 3-axis kinematics unaffected --\n");
    struct stepper_kinematics *skx = cartesian_stepper_alloc('x');
    struct stepper_kinematics *sky = cartesian_stepper_alloc('y');
    struct stepper_kinematics *skz = cartesian_stepper_alloc('z');
    struct stepper_kinematics *cxp = corexy_stepper_alloc('+');
    struct stepper_kinematics *cxm = corexy_stepper_alloc('-');
    check("cartesian x flags", skx->active_flags, AF_X, 0.);
    check("corexy flags", cxp->active_flags, AF_X | AF_Y, 0.);

    double t0 = 0.1, move_t = 1.0;
    double start[KIN_AXES] = {5., 7., 11., 0., 0., 0.};
    // Include rotation in the move to prove it does not leak into the
    // linear solutions
    double delta[KIN_AXES] = {30., 40., 8., 15., 25., 35.};
    struct trapq *tq = queue_move(t0, move_t, start, delta);
    double te = t0 + move_t;
    check("cartesian x", sample(tq, skx, te), 35., 1e-9);
    check("cartesian y", sample(tq, sky, te), 47., 1e-9);
    check("cartesian z", sample(tq, skz, te), 19., 1e-9);
    check("corexy +", sample(tq, cxp, te), 35. + 47., 1e-9);
    check("corexy -", sample(tq, cxm, te), 35. - 47., 1e-9);
    trapq_free(tq);
}

static void
test_set_position_and_active_axis(void)
{
    printf("\n-- set_position / is_active_axis --\n");
    struct stepper_kinematics *ska = rotary_axis_stepper_alloc('a');
    struct stepper_kinematics *skx = cartesian_stepper_alloc('x');
    itersolve_set_position(ska, 1., 2., 3., 44., 55., 66.);
    check("rotary A set_position", itersolve_get_commanded_pos(ska), 44., 1e-9);
    itersolve_set_position(skx, 1., 2., 3., 44., 55., 66.);
    check("cartesian X set_position", itersolve_get_commanded_pos(skx),
          1., 1e-9);
    check("A is_active_axis('a')", itersolve_is_active_axis(ska, 'a'), 1, 0.);
    check("A is_active_axis('b')", itersolve_is_active_axis(ska, 'b'), 0, 0.);
    check("A is_active_axis('x')", itersolve_is_active_axis(ska, 'x'), 0, 0.);
    check("X is_active_axis('x')", itersolve_is_active_axis(skx, 'x'), 1, 0.);
    check("X is_active_axis('a')", itersolve_is_active_axis(skx, 'a'), 0, 0.);
}

// The itersolve hot loop calls calc_position_cb many times per step, and
// move_get_coord now computes six components instead of three.  Time a
// large number of samples so a regression here is visible.
static void
benchmark(void)
{
    printf("\n-- benchmark (informational) --\n");
    struct stepper_kinematics *skx = cartesian_stepper_alloc('x');
    double t0 = 0.1, move_t = 1.0;
    double start[KIN_AXES] = {0., 0., 0., 0., 0., 0.};
    double delta[KIN_AXES] = {100., 50., 25., 10., 20., 30.};
    struct trapq *tq = queue_move(t0, move_t, start, delta);
    struct move *m = NULL, *it;
    list_for_each_entry(it, &tq->moves, node)
        if (it->print_time >= 0. && it->move_t > 0. && it->move_t < NEVER_TIME)
            m = it;
    const long N = 20000000;
    double acc = 0.;
    clock_t t_start = clock();
    for (long i = 0; i < N; i++)
        acc += skx->calc_position_cb(skx, m, (i % 1000) * 0.001);
    double secs = (double)(clock() - t_start) / CLOCKS_PER_SEC;
    printf("     %ld calc_position calls in %.3fs (%.1f ns/call), acc=%.1f\n",
           N, secs, secs * 1e9 / N, acc);
    trapq_free(tq);
}

// End to end regression for the reported "G28 X causes a stepcompress
// error" on a corertheta machine.  G28 X is the R home - the radial
// rail is [stepper_r], but the g-code words stay cartesian, so the
// HOME_R macro issues G28 X.  Everything above checks the kinematic
// function itself; this drives the real step generator and the real step
// compressor over the move that homing actually issues, because that is
// where the fault surfaced - the bed had to cover a whole stale angle in
// the microseconds it took the radius to cross the dead zone, which the
// compressor rejects with "Invalid sequence" (an interval of zero ticks).
static void
test_corertheta_home_r_step_generation(void)
{
    printf("\n-- corertheta: G28 X drives the step compressor --\n");
    const double mcu_freq = 16000000.;
    // 200 full steps, 16 microsteps, 4:1 bed reduction
    const double bed_step_dist = 2. * M_PI / (200. * 16. * 4.);

    struct list_head msg_queue;
    list_init(&msg_queue);
    struct stepcompress *sc = stepcompress_alloc(&msg_queue);
    stepcompress_fill(sc, 1, (uint32_t)(0.000025 * mcu_freq), 1, 2);
    stepcompress_set_time(sc, 0., mcu_freq);

    struct stepper_kinematics *bed = corertheta_stepper_alloc('c', 1.);
    struct trapq *tq = trapq_alloc();
    itersolve_set_trapq(bed, tq, bed_step_dist);

    // An earlier print move left the bed at 2 rad, then G28 X forced the
    // toolhead to (position_min, 0) - the centre, for a position_min of 0
    itersolve_set_position(bed, cos(2.) * 100., sin(2.) * 100.,
                           0., 0., 0., 0.);
    itersolve_set_position(bed, 0., 0., 0., 0., 0., 0.);
    check("G28 X: forcepos clears the stale bed angle",
          itersolve_get_commanded_pos(bed), 0., 1e-9);

    // The homing drip move: radius 0 -> 200mm along y == 0 at 50mm/s
    double t0 = 0.1, move_t = 200. / 50.;
    double start[KIN_AXES] = {0., 0., 0., 0., 0., 0.};
    double delta[KIN_AXES] = {200., 0., 0., 0., 0., 0.};
    struct trapq *tq_h = queue_move(t0, move_t, start, delta);
    itersolve_set_trapq(bed, tq_h, bed_step_dist);

    check("G28 X: itersolve_generate_steps",
          itersolve_generate_steps(bed, sc, t0 + move_t), 0., 0.);
    check("G28 X: stepcompress_flush",
          stepcompress_flush(sc, (uint64_t)((t0 + move_t + 1.) * mcu_freq)),
          0., 0.);
    // Homing straight out along +X leaves the bed where it started
    check("G28 X: bed angle after the homing move",
          itersolve_get_commanded_pos(bed), 0., 1e-9);

    stepcompress_free(sc);
    trapq_free(tq);
    trapq_free(tq_h);
}

// RTCP: the head tilts about B, and the linear carriages must move to
// hold the commanded tool tip in place.
static void
test_rtcp(void)
{
    printf("\n-- RTCP compensation for a B axis head --\n");
    const double L = 40.;   // pivot sits 40mm above the tip at B=0
    struct stepper_kinematics *rx = rtcp_alloc();
    struct stepper_kinematics *rz = rtcp_alloc();
    struct stepper_kinematics *ry = rtcp_alloc();
    check("rtcp_set_sk on x", rtcp_set_sk(rx, cartesian_stepper_alloc('x')),
          0, 0.);
    check("rtcp_set_sk on z", rtcp_set_sk(rz, cartesian_stepper_alloc('z')),
          0, 0.);
    check("rtcp_set_sk on y", rtcp_set_sk(ry, cartesian_stepper_alloc('y')),
          0, 0.);
    check("rtcp_set_sk rejects NULL", rtcp_set_sk(rtcp_alloc(), NULL), -1, 0.);
    rtcp_set_pivot(rx, 0., L);
    rtcp_set_pivot(rz, 0., L);
    rtcp_set_pivot(ry, 0., L);

    // X and Z must now respond to B - otherwise a rotation-only move
    // generates no steps and the tip is not held
    check("x stepper is active on B", (rx->active_flags & AF_B) != 0, 1, 0.);
    check("z stepper is active on B", (rz->active_flags & AF_B) != 0, 1, 0.);
    check("y stepper is NOT active on B", (ry->active_flags & AF_B) != 0,
          0, 0.);

    double t0 = 0.1, move_t = 1.0;
    double zero[KIN_AXES] = {0., 0., 0., 0., 0., 0.};

    // B = 0: machine coordinates equal the commanded tip coordinates
    struct trapq *tq0 = queue_move(t0, move_t, zero, zero);
    check("B=0: x", sample(tq0, rx, t0), 0., 1e-9);
    check("B=0: z", sample(tq0, rz, t0), 0., 1e-9);

    // Tilt to B=90 holding the tip still.  The tool now points along -X
    // from the pivot, so the carriage must go +L in X and -L in Z.
    double d_b90[KIN_AXES] = {0., 0., 0., 0., 90., 0.};
    struct trapq *tq90 = queue_move(t0, move_t, zero, d_b90);
    check("B=90: x carriage", sample(tq90, rx, t0 + move_t), L, 1e-9);
    check("B=90: z carriage", sample(tq90, rz, t0 + move_t), -L, 1e-9);
    check("B=90: y carriage", sample(tq90, ry, t0 + move_t), 0., 1e-9);
    // B=-90 mirrors it
    double d_bm90[KIN_AXES] = {0., 0., 0., 0., -90., 0.};
    struct trapq *tqm90 = queue_move(t0, move_t, zero, d_bm90);
    check("B=-90: x carriage", sample(tqm90, rx, t0 + move_t), -L, 1e-9);
    check("B=-90: z carriage", sample(tqm90, rz, t0 + move_t), -L, 1e-9);

    // Mid-rotation the compensation must already be applied - this is
    // the part that only works because B shares the move's time base
    double b_mid = 45.;
    double x_want = L * sin(b_mid * M_PI / 180.);
    double z_want = L * (cos(b_mid * M_PI / 180.) - 1.);
    check("B=45 (midway): x carriage", sample(tq90, rx, t0 + move_t / 2.),
          x_want, 1e-9);
    check("B=45 (midway): z carriage", sample(tq90, rz, t0 + move_t / 2.),
          z_want, 1e-9);

    // Combined: the tip travels in X while the head tilts
    double start_xz[KIN_AXES] = {10., 0., 5., 0., 0., 0.};
    double d_xb[KIN_AXES] = {100., 0., 0., 0., 90., 0.};
    struct trapq *tqxb = queue_move(t0, move_t, start_xz, d_xb);
    check("tip X move + tilt: x carriage", sample(tqxb, rx, t0 + move_t),
          110. + L, 1e-9);
    check("tip X move + tilt: z carriage", sample(tqxb, rz, t0 + move_t),
          5. - L, 1e-9);

    // A pivot offset in X shifts where the tip sits under the pivot
    struct stepper_kinematics *ox = rtcp_alloc();
    rtcp_set_sk(ox, cartesian_stepper_alloc('x'));
    rtcp_set_pivot(ox, 3., L);
    check("px offset at B=0", sample(tq0, ox, t0), 0., 1e-9);
    check("px offset at B=90", sample(tq90, ox, t0 + move_t),
          3. * (cos(M_PI / 2.) - 1.) + L * sin(M_PI / 2.), 1e-9);

    // Disabling (zero pivot) restores the identity and drops the B link
    rtcp_set_pivot(rx, 0., 0.);
    check("disabled: x carriage", sample(tq90, rx, t0 + move_t), 0., 1e-9);
    check("disabled: not active on B", (rx->active_flags & AF_B) != 0, 0, 0.);

    trapq_free(tq0);
    trapq_free(tq90);
    trapq_free(tqm90);
    trapq_free(tqxb);
}

// Regression for "Internal error in stepcompress" on the stepper_c queue
// part way into a print of a full circle centred on the bed.
//
// The bed solver derives its angle from atan2, which jumps by 2*pi as the
// path crosses the -X axis, so it unwraps the result against
// sk->commanded_pos.  [rtcp] wraps every kinematic stepper, including the
// bed, and itersolve only ever advances the *wrapper's* commanded_pos -
// so unless the wrapper forwards that position (and post_cb) inward, the
// bed solver compares against a stale zero, the unwrap can never fire,
// and the branch cut asks for half a bed revolution in one step interval.
//
// The points below are the real toolpath, lifted from the trapq dump of
// a shutdown, and the geometry is the machine's: a 65.6mm pivot 21.94mm
// off the tip, a 16000 step bed, 20mm/s.
static void
test_corertheta_rtcp_branch_cut(void)
{
    printf("\n-- corertheta: bed crosses theta = +-pi under [rtcp] --\n");
    const double mcu_freq = 120000000.;
    // 200 full steps, 16 microsteps, 80:16 gear reduction
    const double bed_step_dist = 2. * M_PI / 16000.;
    const double vel = 20.;
    // x, y, z, b - a spiral crossing the -X axis at (-22.99, 0)
    static const double pts[][4] = {
        {-22.242,  5.701, 11.599, 1.319}, {-22.403,  4.999, 11.638, 1.338},
        {-22.551,  4.294, 11.676, 1.358}, {-22.679,  3.586, 11.716, 1.377},
        {-22.786,  2.873, 11.755, 1.397}, {-22.874,  2.158, 11.795, 1.416},
        {-22.921,  1.439, 11.836, 1.435}, {-22.956,  0.720, 11.878, 1.455},
        {-22.990, -0.000, 11.919, 1.474}, {-22.961, -0.720, 11.962, 1.494},
        {-22.931, -1.440, 12.005, 1.513}, {-22.893, -2.159, 12.048, 1.532},
        {-22.809, -2.875, 12.093, 1.552}, {-22.696, -3.586, 12.138, 1.571},
        {-22.576, -4.296, 12.183, 1.591},
    };
    const int npts = (int)(sizeof(pts) / sizeof(pts[0]));

    struct list_head msg_queue;
    list_init(&msg_queue);
    struct stepcompress *sc = stepcompress_alloc(&msg_queue);
    stepcompress_fill(sc, (uint32_t)(0.000002 * mcu_freq),
                      (uint32_t)(0.000025 * mcu_freq), 1, 2);
    stepcompress_set_time(sc, 0., mcu_freq);

    struct stepper_kinematics *raw = corertheta_stepper_alloc('c', .25);
    struct stepper_kinematics *bed = rtcp_alloc();
    check("branch cut: rtcp_set_sk on the bed", rtcp_set_sk(bed, raw), 0, 0.);
    rtcp_set_pivot(bed, 21.94, 65.6);

    struct trapq *tq = trapq_alloc();
    itersolve_set_trapq(bed, tq, bed_step_dist);
    itersolve_set_position(bed, pts[0][0], pts[0][1], pts[0][2],
                           0., pts[0][3], 0.);
    double start_angle = itersolve_get_commanded_pos(bed);

    double t = 0.1;
    for (int i = 1; i < npts; i++) {
        double dx = pts[i][0] - pts[i-1][0], dy = pts[i][1] - pts[i-1][1];
        double dz = pts[i][2] - pts[i-1][2], db = pts[i][3] - pts[i-1][3];
        double d = sqrt(dx*dx + dy*dy + dz*dz), mt = d / vel;
        trapq_append(tq, t, 0., mt, 0.,
                     pts[i-1][0], pts[i-1][1], pts[i-1][2],
                     0., pts[i-1][3], 0.,
                     dx/d, dy/d, dz/d, 0., db/d, 0., vel, vel, 0.);
        t += mt;
    }

    check("branch cut: itersolve_generate_steps",
          itersolve_generate_steps(bed, sc, t), 0., 0.);
    check("branch cut: stepcompress_flush",
          stepcompress_flush(sc, (uint64_t)((t + 1.) * mcu_freq)), 0., 0.);
    // The wrapper must have kept the inner solver's position in step with
    // its own - that is the whole reason the unwrap can work
    check("branch cut: wrapped solver tracks the wrapper",
          itersolve_get_commanded_pos(raw),
          itersolve_get_commanded_pos(bed), 1e-12);
    // The bed turns the short way through pi, not the long way back
    // around.  atan2 of the rtcp-corrected endpoints puts the sweep at
    // 0.472308 rad; the tolerance is one bed half step (0.000196).
    double end_angle = itersolve_get_commanded_pos(bed);
    double swept = end_angle - start_angle;
    if (swept < -M_PI)
        swept += 2. * M_PI;
    check("branch cut: bed sweep across the cut", swept, 0.472308, 0.0004);

    stepcompress_free(sc);
    trapq_free(tq);
}

// [b_projection]: a commanded B is a lean toward the *bed's* +x, so what
// the machine can reach is that lean projected onto its own xz plane -
// scaled by cos of the bed angle.
static void
test_b_projection(void)
{
    printf("\n-- bed-frame B projection --\n");
    const double MA = 40., TR = 5.;
    const double S = 70.710678118654755;  // 100/sqrt(2)

    // The pure mapping.  Holding B at 10 degrees through a turn of the
    // bed sweeps the machine's B over 10 -> 0 -> -10 -> 0.
    check("proj: bed angle 0 leaves B alone",
          bproject_project_b(10., 100., 0., MA, TR), 10., 1e-12);
    check("proj: bed angle 90 flattens B",
          bproject_project_b(10., 0., 100., MA, TR), 0., 1e-12);
    check("proj: bed angle 180 negates B",
          bproject_project_b(10., -100., 0., MA, TR), -10., 1e-12);
    check("proj: bed angle 45 scales B by cos",
          bproject_project_b(10., S, S, MA, TR), 7.0710678, 1e-6);
    check("proj: bed angle 270 flattens B",
          bproject_project_b(10., 0., -100., MA, TR), 0., 1e-12);
    check("proj: B=0 is a fixed point",
          bproject_project_b(0., 0., 100., MA, TR), 0., 1e-12);

    // Orientation angles beyond the band reach the machine untouched -
    // that is what lets RTCP_PROBE_ORIENT and G28 B mean what they say
    check("proj: probe angle passes through",
          bproject_project_b(45., 0., 100., MA, TR), 45., 1e-12);
    check("proj: park angle passes through",
          bproject_project_b(-90., 0., 100., MA, TR), -90., 1e-12);

    // ...and the taper across the band keeps the machine's B continuous,
    // instead of jumping by up to max_angle in one step
    check("proj: mid-taper is half corrected",
          bproject_project_b(42.5, 0., 100., MA, TR), 21.25, 1e-9);
    double just_in = bproject_project_b(44.999, 0., 100., MA, TR);
    double just_out = bproject_project_b(45.001, 0., 100., MA, TR);
    check("proj: continuous at the top of the band",
          just_out - just_in, 0., 0.01);
    double below = bproject_project_b(39.999, 0., 100., MA, TR);
    double above = bproject_project_b(40.001, 0., 100., MA, TR);
    check("proj: continuous at the bottom of the band",
          above - below, 0., 0.01);
    check("proj: a zero max_angle is the identity",
          bproject_project_b(10., 0., 100., 0., TR), 10., 1e-12);

    // Wrapped around a corertheta gantry motor: the '+' solver is
    // b_ratio*B + radius, and it must see the projected B
    struct stepper_kinematics *plus = bproject_alloc();
    check("bproject_set_sk on the + gantry motor",
          bproject_set_sk(plus, corertheta_stepper_alloc('+', 1.)), 0, 0.);
    check("bproject_set_sk rejects NULL",
          bproject_set_sk(bproject_alloc(), NULL), -1, 0.);
    bproject_set_params(plus, MA, TR);
    check("wrapped: B reads through at bed angle 0",
          itersolve_calc_position_from_coord(plus, 100., 0., 0., 0., 10., 0.),
          110., 1e-9);
    check("wrapped: B is flattened at bed angle 90",
          itersolve_calc_position_from_coord(plus, 0., 100., 0., 0., 10., 0.),
          100., 1e-9);
    check("wrapped: park angle still reaches the motor",
          itersolve_calc_position_from_coord(plus, 0., 100., 0., 0., -90., 0.),
          10., 1e-9);
    // A B-only move now depends on where the bed is, so the stepper has
    // to become active on x and y as well - a move no axis claims
    // generates no steps at all
    check("wrapped: active on b", itersolve_is_active_axis(plus, 'b'), 1, 0.);
    check("wrapped: active on x", itersolve_is_active_axis(plus, 'x'), 1, 0.);
    check("wrapped: active on y", itersolve_is_active_axis(plus, 'y'), 1, 0.);

    // Outside RTCP, so the tip correction uses the angle the head is
    // really turned to.  At bed angle 90 the projected B is zero, so the
    // head is upright and the tip is not swung at all.
    const double L = 40.;
    struct stepper_kinematics *cx = bproject_alloc();
    struct stepper_kinematics *inner = rtcp_alloc();
    check("chain: rtcp over cartesian x",
          rtcp_set_sk(inner, cartesian_stepper_alloc('x')), 0, 0.);
    rtcp_set_pivot(inner, 0., L);
    check("chain: bproject over rtcp", bproject_set_sk(cx, inner), 0, 0.);
    bproject_set_params(cx, MA, TR);
    check("chain: upright head is not corrected",
          itersolve_calc_position_from_coord(cx, 0., 100., 0., 0., 10., 0.),
          0., 1e-9);
    // Along the bed's x axis the projection is the identity, so the full
    // RTCP swing of L*sin(B) is applied
    check("chain: full tilt gets the full tip correction",
          itersolve_calc_position_from_coord(cx, 5., 0., 0., 0., 10., 0.),
          5. + L * sin(10. * M_PI / 180.), 1e-9);
}

int
main(void)
{
    test_rotary_shares_time_base();
    test_core_r_theta();
    test_corertheta();
    test_corertheta_home_r_step_generation();
    test_corertheta_rtcp_branch_cut();
    test_existing_kinematics_unchanged();
    test_set_position_and_active_axis();
    test_rtcp();
    test_b_projection();
    benchmark();
    printf("\n%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures != 0;
}
