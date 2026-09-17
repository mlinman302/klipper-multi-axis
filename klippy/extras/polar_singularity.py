# Bed centre singularity handling for rotating bed kinematics
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# THE SINGULARITY
#
# On a polar or core r-theta machine the bed angle is not a commanded
# axis.  It is derived - theta = atan2(y, x), evaluated per sample in
# klippy/chelper/kin_corertheta.c - and that derivation has no value at
# all on the line x = y = 0.  The tool tip travelling through [0, 0, N]
# is therefore a singularity: an arbitrarily small step across the centre
# is a half turn of the bed, asked for in the instant the sign flips.
#
# It is not an edge case that only bites exactly on the axis.  The bed's
# angular velocity diverges as 1/r on the approach and its angular
# acceleration as 1/r^2, so a near miss is the same problem with a finite
# but still unreachable number attached to it.  The geometry, and the
# limits that follow from it, are derived in
# klippy/kinematics/bed_centre.py; this file only decides what to do
# with them.
#
# WHAT COUNTS AS SINGULAR
#
# The test is on the *g-code* x/y, not on the machine position.  With
# [rtcp] on, the tool tip is what g-code commands and the carriage takes
# up the difference - but in the radial frame that correction scales x and
# y together, which moves the arm radius and leaves the bed angle exactly
# where it was.  So the bed angle, and with it everything in this file,
# depends only on the commanded tip position.  What does live in the
# machine frame is the arm's own travel near the centre, where a small
# change of tip position becomes a large change of radius; [rtcp] already
# range checks that in its own move check.
#
# The classification keys on the swept angle, not on the radius alone.
# Two moves sit at a closest approach of zero and are perfectly legal:
#
#   G1 X0 Y0 Z50 -> Z10   straight down the axis.  The radius is zero for
#                         the whole move and the bed never turns - this is
#                         the N in [0, 0, N].
#   G1 X0 Y0 -> X10 Y0    departing along a ray.  The radius is zero at
#                         one endpoint, and the bed still never turns.
#
# Both have a perpendicular offset of zero, which is what path_geometry()
# reports and what makes them cost nothing here.
#
# WHAT THIS DOES, AND WHAT IT DOES NOT DO YET
#
# This is the first stage: it makes the machine refuse what it cannot do,
# and says why.  It does not reshape a path to avoid the centre.  A move
# that crosses the axis is rejected rather than routed around it, because
# routing it around costs either a path deviation or a signed arm radius,
# and both are decisions the machine's owner has to make rather than
# something to do silently underneath them.
import math
from kinematics.bed_centre import (
    CENTRE_RADIUS, path_geometry, swept_angle, crosses_centre,
    peak_angular_velocity, peak_angular_accel, limits_for_angular_rates)
from kinematics.polar import NO_ACCEL_LIMIT


class PolarSingularity:
    def __init__(self, config):
        self.printer = config.get_printer()
        # The bed's angular velocity limit already has a home in the
        # [printer] section, where the kinematics reads it.  Default to it
        # so that a machine states the figure once.
        printer_config = config.getsection('printer')
        default_v = printer_config.getfloat('max_angular_velocity', 0.,
                                            minval=0.)
        self.max_angular_v = config.getfloat('max_angular_velocity',
                                             default_v, minval=0.)
        # Nothing in the tree records this today.  It is the limit that
        # bites first - theta_ddot goes as 1/r^2 where theta_dot goes as
        # 1/r - so leaving it unset leaves the sharper of the two edges
        # unguarded.
        #
        # Note what it does and does not reach.  Wherever the velocity
        # limit above is the binding one, the feedrate it leaves is
        # v = max_angular_velocity * r_min, and the geometric part of
        # theta_ddot at that feedrate is 0.65 * max_angular_velocity^2
        # whatever the radius - so a max_angular_accel above that figure
        # never limits a feedrate at all.  What it always does is bound
        # the move's own acceleration, through the a*offset/r^2 term,
        # and near the centre that is the term that moves the bed.
        self.max_angular_a = config.getfloat('max_angular_accel', 0.,
                                             minval=0.)
        # Below this feedrate, slowing down has stopped being an answer
        self.min_velocity = config.getfloat('min_velocity', 0.5, above=0.)
        # Diagnostics for the last move that came near the centre.  Most
        # of the failures this replaces present as an "Internal error in
        # stepcompress" with no indication of where the machine was.
        self.last_radius = 0.
        self.last_swept = 0.
        self.last_velocity_limit = 0.
        self.printer.register_event_handler("klippy:connect", self._connect)
    def _connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_move_check(self._check_move)

    ######################################################################
    # Move checking
    ######################################################################
    def _check_move(self, move):
        if not move.axes_d[0] and not move.axes_d[1]:
            # Nothing in the xy plane, so the bed is not asked to turn.
            # This is the pure z move along the axis, and the rotation
            # only move at the centre.
            return
        offset, r_min, u_start, u_end = path_geometry(move.start_pos,
                                                      move.end_pos)
        swept = swept_angle(move.start_pos, move.end_pos)
        self.last_radius = r_min
        self.last_swept = swept
        self.last_velocity_limit = 0.
        if crosses_centre(r_min, swept):
            # The path crosses the axis itself.  There is no rate here to
            # slow down - the bed angle steps, and no feedrate makes a step
            # take longer.  A dead straight line across the centre is the
            # common way to arrive here: its perpendicular offset is
            # exactly zero, so every rate this file computes is zero while
            # the bed still has half a turn to make.
            raise move.move_error(
                "Move crosses the centre of the bed (closest approach"
                " %.4f mm, %.1f degrees of bed rotation).  The bed angle is"
                " derived from x/y, so crossing the axis asks for that"
                " rotation in no time at all.  Route the move so it stays"
                " more than %.3f mm from the centre"
                % (r_min, math.degrees(abs(swept)), CENTRE_RADIUS))
        v_limit, a_limit = limits_for_angular_rates(
            offset, r_min, u_start, u_end,
            self.max_angular_v, self.max_angular_a)
        if v_limit is None and a_limit is None:
            return
        if v_limit is not None:
            if v_limit < self.min_velocity:
                velocity = math.sqrt(move.max_cruise_v2)
                raise move.move_error(
                    "Move passes %.4f mm from the centre of the bed and"
                    " would turn it at %.1f rad/s (%.0f rad/s^2).  Holding"
                    " the bed's limits needs a feedrate of %.3f mm/s, below"
                    " the %.2f mm/s floor.  Route the move further from the"
                    " centre, or raise max_angular_velocity /"
                    " max_angular_accel if the bed can really do it"
                    % (r_min,
                       peak_angular_velocity(velocity, offset, r_min),
                       peak_angular_accel(velocity, offset, u_start, u_end),
                       v_limit, self.min_velocity))
            self.last_velocity_limit = v_limit
        move.limit_speed(NO_ACCEL_LIMIT if v_limit is None else v_limit,
                         NO_ACCEL_LIMIT if a_limit is None else a_limit)

    def get_status(self, eventtime):
        return {
            'max_angular_velocity': self.max_angular_v,
            'max_angular_accel': self.max_angular_a,
            'last_radius': self.last_radius,
            'last_swept_angle': math.degrees(self.last_swept),
            'last_velocity_limit': self.last_velocity_limit,
        }


def load_config(config):
    return PolarSingularity(config)
