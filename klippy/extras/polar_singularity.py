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
# WHAT THIS DOES
#
# Two things, at two different altitudes.
#
# A move check, which every move reaching the toolhead passes through -
# including the ones no g-code transform ever sees.
# It slows a move that passes near the centre to the bed's angular limits,
# and refuses one that crosses the axis or that would have to crawl below
# min_velocity to stay inside them.  It also refuses a move that would
# leave the centre, or come to rest there, with the bed facing somewhere
# the toolhead position does not name - see "The position names the bed
# angle" in bed_centre.py.
#
# A g-code move transform, which plans the path around the centre before
# the moves are made: it parks the tool on the ray it arrived along, turns
# the bed on the way through the centre - blended into the moves either
# side while the tool keeps moving, or on a small arc where that cannot
# stay within blend_tolerance - routes a crossing move through the centre
# instead of refusing it, and splits a near miss so only its close part
# runs slowly.  The head's tilt is never changed to make any of that
# easier.  The planning itself lives in klippy/kinematics/centre_path.py,
# which explains each of those.  The transform only schedules what the
# move check would allow; anything it passes through unchanged still has
# to get past the check.
#
# HOLDING A MOVE BACK
#
# Blending a turn through the centre needs the move after it.  So the
# transform sends a move onto the centre only as far as blend_radius
# short of it, and holds the rest until the next g-code move arrives to
# plan it with.  Nothing else may see the toolhead in between: the
# toolhead calls release() before any operation that would - another
# move, a dwell, M400, a timed callback such as a fan or heater change,
# setting a position - and a timer releases it if no command follows at
# all.  A release sends the tail on to the park point, exactly as if
# there had been no hold.  While a tail is held no move is being emitted
# anywhere, so a release is safe from any context.
#
# The transform is registered at connect time, after every transform that
# insists on being first ([bed_mesh], [bed_tilt]) and before the ones that
# stack on top ([skew_correction]), so the x/y it plans in are the x/y the
# toolhead receives.  [bed_mesh] and [bed_tilt] only change z, and
# [bed_mesh] cuts a move into pieces along the same straight line, so
# neither disturbs a plan.
#
# CROSSING ON AN ARM THAT CAN
#
# On a corertheta machine whose arm travels through the centre
# (arm_crosses_centre), a move that carries straight on through the
# middle changes over to the other branch there - the arm passes through
# zero radius and the bed is held still - and the kinematics marks it so
# (branch_flip).  The bed does not turn on such a move, so the crossing
# refusal and the angular limits do not apply to it; the check only makes
# sure it runs straight along the line the bed faces.
#
# WHAT IT DOES NOT DO
#
# It only plans g-code moves: anything that moves the toolhead itself is
# seen by the move check alone, and has to plan its own moves with
# kinematics/centre_path.py if they touch the centre.  [input_shaper]
# smooths the path across neighbouring moves, which at the centre is on
# the scale of the park point and the reorientation arc; the plans here
# assume the path the moves describe.
import math
from kinematics import centre_path
from kinematics.bed_centre import (
    CENTRE_RADIUS, ANGLE_TOLERANCE, path_geometry, swept_angle,
    crosses_centre, centre_turns, peak_angular_velocity, peak_angular_accel,
    limits_for_angular_rates)
from kinematics.polar import NO_ACCEL_LIMIT

# How long (in seconds) a held tail may wait for the next command before
# it is sent on its own
HOLD_TIMEOUT = .1


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
        # What to do with a move that crosses the centre: refuse it, turn
        # the bed half a turn on the centre, or - on an arm that can -
        # carry straight on through it.  Turning is harmless on a travel
        # move and leaves a blob on a print move.
        self.travel_policy = config.getchoice(
            'travel_policy', list(centre_path.POLICIES), 'bypass')
        self.print_policy = config.getchoice(
            'print_policy', list(centre_path.POLICIES), 'error')
        # The circle the tool follows while the bed turns at the centre
        self.reorient_radius = config.getfloat('reorient_radius', None,
                                               above=0.)
        # How far a turn blended through the centre may take the tool off
        # its path, and how far from the centre the blend may reach.  A
        # blend_tolerance of zero turns the bed on the arc alone.
        self.blend_tolerance = config.getfloat(
            'blend_tolerance', centre_path.DEFAULT_BLEND_TOLERANCE, minval=0.)
        self.blend_radius = config.getfloat(
            'blend_radius', centre_path.DEFAULT_BLEND_RADIUS, above=0.)
        self.name = config.get_name()
        # Whether 'cross' is possible depends on the kinematics, which is
        # not there to ask until connect time - so check everything else
        # now, and build the planner for real then
        self.planner = self._make_planner(can_cross=True, error=config.error)
        self.toolhead = self.next_transform = None
        self.last_position = [0., 0., 0., 0.]
        # The tail of a move onto the centre, held for the next move (see
        # "Holding a move back" above), and the timer that gives up on it
        self.held = None
        self.reactor = self.hold_timer = None
        # Diagnostics for the last move that came near the centre.  Most
        # of the failures this replaces present as an "Internal error in
        # stepcompress" with no indication of where the machine was.
        self.last_radius = 0.
        self.last_swept = 0.
        self.last_velocity_limit = 0.
        self.printer.register_event_handler("klippy:connect", self._connect)
    def _make_planner(self, can_cross, error):
        try:
            return centre_path.CentrePlanner(
                self.max_angular_v, self.max_angular_a, self.min_velocity,
                self.reorient_radius, self.travel_policy, self.print_policy,
                can_cross, self.blend_tolerance, self.blend_radius)
        except ValueError as e:
            raise error("[%s] %s" % (self.name, e))

    def _connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        can_cross = getattr(kin, 'can_cross_centre', lambda: False)()
        self.planner = self._make_planner(can_cross,
                                          self.printer.config_error)
        self.toolhead.register_move_check(self._check_move)
        self.toolhead.register_held_moves(self.release)
        self.reactor = self.printer.get_reactor()
        self.hold_timer = self.reactor.register_timer(self._hold_timeout)
        gcode_move = self.printer.lookup_object('gcode_move')
        self.next_transform = gcode_move.set_move_transform(self, force=True)

    ######################################################################
    # Move transform
    ######################################################################
    def get_position(self):
        # A tool parked on the axis stands a tenth of a micron off it, on
        # the ray the bed faces; the caller asked for the centre itself.
        # Anything asking where the toolhead is gets the answer with no
        # tail held back.
        self.release()
        pos = self.next_transform.get_position()
        if centre_path.at_centre(pos):
            pos[0] = pos[1] = 0.
        self.last_position[:] = pos
        return list(pos)
    def move(self, newpos, speed):
        held, self.held = self.held, None
        self._set_hold_timer(None)
        # The toolhead's own x/y is what the bed angle follows, and on the
        # centre it is a park point rather than the position asked for
        machine_xy = self.toolhead.get_position()[:2]
        branch = self.toolhead.get_branch()
        hold = None
        if held is not None:
            moves = self.planner.plan_held(held, newpos, speed, branch)
        else:
            hold = self.planner.hold_point(machine_xy, self.last_position,
                                           newpos)
            if hold is not None:
                moves = []
                if hold[:2] != list(machine_xy):
                    moves.append((hold, speed))
            else:
                moves = self.planner.plan(machine_xy, self.last_position,
                                          newpos, speed, branch)
        for pos, move_speed in moves:
            self.next_transform.move(pos, move_speed)
        self.last_position[:] = newpos
        if hold is not None:
            self.held = (hold, list(newpos), speed)
            if self.reactor is not None:
                self._set_hold_timer(self.reactor.monotonic() + HOLD_TIMEOUT)
    def release(self):
        # Send a held tail on to the park point, as if it had never been
        # held.  The toolhead calls this before anything that must see
        # every move made so far.
        held, self.held = self.held, None
        if held is None:
            return
        self._set_hold_timer(None)
        for pos, move_speed in self.planner.release(held):
            self.next_transform.move(pos, move_speed)
    def _set_hold_timer(self, waketime):
        if self.hold_timer is not None:
            self.reactor.update_timer(
                self.hold_timer,
                self.reactor.NEVER if waketime is None else waketime)
    def _hold_timeout(self, eventtime):
        # No command followed the move onto the centre
        self.release()
        return self.reactor.NEVER

    ######################################################################
    # Move checking
    ######################################################################
    def _check_move(self, move):
        if not move.axes_d[0] and not move.axes_d[1]:
            # Nothing in the xy plane, so the bed is not asked to turn.
            # This is the pure z move along the axis, and the rotation
            # only move at the centre.
            return
        # Leaving rest and coming to rest must not step the bed, and must
        # leave it facing the way the position says it does
        turn_start, turn_end = centre_turns(move.start_pos, move.end_pos,
                                            move.branch, move.branch_flip)
        if abs(turn_start) > ANGLE_TOLERANCE:
            raise move.move_error(
                "Move leaves the centre of the bed %.2f degrees off the line"
                " the bed faces, which would turn the bed that far at once."
                "  Leave the centre along the line of the ray the tool"
                " arrived on - the [polar_singularity] g-code transform"
                " plans that" % (math.degrees(abs(turn_start)),))
        if abs(turn_end) > ANGLE_TOLERANCE:
            raise move.move_error(
                "Move comes to rest at the centre of the bed %.2f degrees"
                " off the ray it arrived along, where the position no"
                " longer says which way the bed faces.  Stop short of the"
                " centre on the ray of arrival - the [polar_singularity]"
                " g-code transform plans that"
                % (math.degrees(abs(turn_end)),))
        offset, r_min, u_start, u_end = path_geometry(move.start_pos,
                                                      move.end_pos)
        self.last_radius = r_min
        self.last_velocity_limit = 0.
        if move.branch_flip:
            # Straight through the centre onto the other branch, with the
            # bed held still: the turns above keep it on the line the bed
            # faces, so over the whole move the bed turns by no more than
            # they allow.  Neither the crossing refusal nor the rates -
            # which assume the bed follows atan2 - apply.
            self.last_swept = 0.
            return
        swept = swept_angle(move.start_pos, move.end_pos)
        self.last_swept = swept
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
            velocity = math.sqrt(move.max_cruise_v2)
            # A move already asked to run below the floor, and slowly
            # enough for the bed, is not one this needs to refuse
            if v_limit < self.min_velocity and v_limit < velocity:
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
        status = {
            'max_angular_velocity': self.max_angular_v,
            'max_angular_accel': self.max_angular_a,
            'last_radius': self.last_radius,
            'last_swept_angle': math.degrees(self.last_swept),
            'last_velocity_limit': self.last_velocity_limit,
            'reorient_radius': self.planner.reorient_radius,
            'travel_policy': self.planner.travel_policy,
            'print_policy': self.planner.print_policy,
            'blend_tolerance': self.planner.blend_tolerance,
            'blend_radius': self.planner.blend_radius,
            'can_cross': self.planner.can_cross,
            # Whether the tail of a move onto the centre is waiting for
            # the move after it
            'holding': self.held is not None,
        }
        if self.toolhead is not None:
            # Whether the tool is standing on the centre, and on which
            # branch - negative once it has crossed onto the far side
            status['at_centre'] = centre_path.at_centre(
                self.toolhead.get_position())
            status['branch'] = self.toolhead.get_branch()
        return status


def load_config(config):
    return PolarSingularity(config)
