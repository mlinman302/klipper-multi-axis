#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

// The motion space is six dimensional: three linear axes (x, y, z) and
// three rotational axes (a, b, c - rotations about x, y and z).  All six
// share a single trapq so that a stepper kinematic can sample linear and
// rotational position at the same instant.  That common time base is what
// coupled drives (eg, core r-theta, where one motor position depends on
// both a linear and a rotational axis) and RTCP compensation require.
// Machines with no rotational axes simply leave a/b/c at zero.
#define KIN_AXES 6

struct coord {
    union {
        struct {
            double x, y, z, a, b, c;
        };
        double axis[KIN_AXES];
    };
};

struct move {
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    // Which of the two solutions of a kinematics with a redundant
    // representation the move is solved on, and whether it changes over
    // to the other one where it passes through the centre - see
    // bed_centre.h.  Both are zero on every other machine.
    int branch, branch_flip;

    struct list_node node;
};

struct trapq {
    struct list_head moves, history;
    // Stamped onto each move trapq_append() adds - see trapq_set_branch()
    int branch, branch_flip;
};

struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_x, start_y, start_z, start_a, start_b, start_c;
    double x_r, y_r, z_r, a_r, b_r, c_r;
};

struct move *move_alloc(void);
double move_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);
int move_get_branch(struct move *m, struct coord *c);
struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_set_branch(struct trapq *tq, int branch, int branch_flip);
void trapq_append(struct trapq *tq, double print_time
                  , double accel_t, double cruise_t, double decel_t
                  , double start_pos_x, double start_pos_y, double start_pos_z
                  , double start_pos_a, double start_pos_b, double start_pos_c
                  , double axes_r_x, double axes_r_y, double axes_r_z
                  , double axes_r_a, double axes_r_b, double axes_r_c
                  , double start_v, double cruise_v, double accel);
void trapq_finalize_moves(struct trapq *tq, double print_time
                          , double clear_history_time);
void trapq_set_position(struct trapq *tq, double print_time
                        , double pos_x, double pos_y, double pos_z
                        , double pos_a, double pos_b, double pos_c);
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                      , double start_time, double end_time);

#endif // trapq.h
