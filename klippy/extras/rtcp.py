# Rotational Tool Center Point (RTCP) compensation for a tilting head
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# THE MODEL
#
# The head tilts about B, and two conventions fix everything else:
#
#   * At B = 0 the nozzle points straight down, just as on a printer with
#     no tilting head.  This is true whether compensation is on or off.
#   * A positive B tilts the nozzle outboard - the tip swings away from
#     the centre of the bed and rises.  A machine that turns the other
#     way inverts B in its kinematics (the invert_b_direction option of
#     [printer] / [corertheta]), never here.
#
# The tool is then described by two offsets, measured at B = 0:
#
#   tool_vertical_offset    how far the tip is *below* the B pivot
#   tool_horizontal_offset  how far the tip is *inboard* of the B pivot,
#                           ie closer to the centre of the bed
#
# THE TOGGLE
#
#   RTCP on   g-code commands the *tool tip*.  A B move swings the tip
#             about the pivot, so the carriages move to cancel that swing
#             and hold the tip where it was asked to be.
#   RTCP off  g-code commands the *carriage*.  A B move just turns the
#             head; nothing else moves.
#
# Off is the frame homing, probing and bed meshing run in - see
# check_disabled() below and docs/Multi_Axis.md.  On is for printing.
#
# The transform that generates the steps lives in
# klippy/chelper/kin_rtcp.c, which this module wraps around each
# kinematic stepper the same way [input_shaper] does.  The position
# conversions below repeat the same arithmetic in Python so that a
# position can be converted without a compiled c_helper.so; kin_rtcp.c
# carries the derivation both follow.
#
# Rotation is assumed to be slow relative to linear motion, so - as
# elsewhere in this fork - the rotational component does not take part in
# the velocity or acceleration planning.
import math
import chelper, stepper
from kinematics import bed_centre

# Only a B axis head (tilting about an axis parallel to Y) is modelled.
RTCP_AXIS_GCODE_ID = 'B'
# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]

# Must match kin_rtcp.h
FRAME_CARTESIAN = 0
FRAME_RADIAL = 1
FRAME_NAMES = {'cartesian': FRAME_CARTESIAN, 'radial': FRAME_RADIAL}
# Kinematics whose x/y are bed coordinates and whose arm travels in
# radius, so the tip swings along the arm rather than along +X
RADIAL_KINEMATICS = ('corertheta', 'polar')

# Options that were replaced when the geometry was redefined.  The sign
# conventions changed with them, so an old config must not be silently
# reinterpreted.
RENAMED_OPTIONS = {
    'pivot_length': ('tool_vertical_offset',
                     "the distance from the B pivot down to the tool tip;"
                     " the value is the same"),
    'pivot_x_offset': ('tool_horizontal_offset',
                       "how far the tool tip sits inboard of the B pivot,"
                       " ie towards the centre of the bed - check the sign"),
}


def lookup_frame(config):
    # Which direction the tool tip swings in as the head tilts
    frame = config.get('horizontal_frame', None)
    if frame is not None:
        if frame not in FRAME_NAMES:
            raise config.error(
                "Invalid horizontal_frame '%s' in section '%s' - must be"
                " 'radial' or 'cartesian'" % (frame, config.get_name()))
        return FRAME_NAMES[frame]
    kin = config.getsection('printer').get('kinematics', None,
                                           note_valid=False)
    if kin in RADIAL_KINEMATICS:
        return FRAME_RADIAL
    return FRAME_CARTESIAN


class RTCP:
    # Optional bed-frame B projection - see klippy/extras/b_projection.py.
    # A class attribute so that code building an RTCP without running
    # __init__ (the host tests do) still finds it.
    b_project = None

    def __init__(self, config):
        self.printer = config.get_printer()
        self.toolhead = None
        for old, (new, hint) in RENAMED_OPTIONS.items():
            if config.get(old, None, note_valid=False) is not None:
                raise config.error(
                    "The [rtcp] option '%s' is now '%s' - %s.  See the RTCP"
                    " section of docs/Multi_Axis.md" % (old, new, hint))
        # Tool tip position relative to the B pivot at B=0
        self.tool_v = config.getfloat('tool_vertical_offset')
        self.tool_h = config.getfloat('tool_horizontal_offset', 0.)
        self.frame = lookup_frame(config)
        self.enabled = config.getboolean('enable', True)
        self.orig_stepper_kinematics = []
        # The wrapper installed on each stepper, keyed by stepper name
        self.rtcp_stepper_kinematics = {}
        self.printer.register_event_handler("klippy:connect", self._connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_RTCP', self.cmd_SET_RTCP,
                               desc=self.cmd_SET_RTCP_help)

    def _connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        # The B axis must exist, otherwise there is nothing to compensate
        extra_axes = self.toolhead.get_extra_axes()
        have_b = any(ea is not None
                     and ea.get_axis_gcode_id() == RTCP_AXIS_GCODE_ID
                     for ea in extra_axes)
        if not have_b:
            raise self.printer.config_error(
                "[rtcp] requires a B axis - add 'b' to the"
                " 'additional_axes' option of the [printer] section")
        self.b_project = self.printer.lookup_object('b_projection', None)
        self.toolhead.register_move_check(self._check_move)
        self._update_kinematics()

    ######################################################################
    # Stepper wrapping
    ######################################################################
    def _get_rtcp_stepper_kinematics(self, s):
        # The wrapper is installed once, at connect time, and only
        # retuned afterwards.  It has to be found by stepper name rather
        # than by reading s.get_stepper_kinematics() back: [b_projection]
        # wraps these same steppers *outside* this one, so once it has
        # done so the outermost kinematic is its wrapper, not ours.
        # Recognising only our own would then wrap a second time, leaving
        # rtcp -> b_projection -> rtcp -> solver: SET_RTCP would retune
        # the new outer copy and report the compensation off while the
        # inner one went on applying it, with the projection sandwiched
        # the wrong way round between them.
        rtcp_sk = self.rtcp_stepper_kinematics.get(s.get_name())
        if rtcp_sk is not None:
            return rtcp_sk
        sk = s.get_stepper_kinematics()
        ffi_main, ffi_lib = chelper.get_ffi()
        rtcp_sk = ffi_main.gc(ffi_lib.rtcp_alloc(), ffi_lib.free)
        s.set_stepper_kinematics(rtcp_sk)
        if ffi_lib.rtcp_set_sk(rtcp_sk, sk) < 0:
            s.set_stepper_kinematics(sk)
            return None
        self.orig_stepper_kinematics.append(sk)
        self.rtcp_stepper_kinematics[s.get_name()] = rtcp_sk
        return rtcp_sk

    def _update_kinematics(self):
        if self.toolhead is None:
            return
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        tool_h, tool_v = self.get_tool_offsets()
        kin = self.toolhead.get_kinematics()
        unwrapped = set()
        if self.frame == FRAME_RADIAL and hasattr(kin,
                                                  'get_bed_angle_steppers'):
            # The radial correction moves the arm and never turns the bed,
            # so the bed motor is left seeing the tool tip - which is what
            # the bed angle is defined by.  Wrapped, it would see a
            # carriage position that can sit on, or across, the centre
            # while the tip is well away from it.
            unwrapped = {s.get_name() for s in kin.get_bed_angle_steppers()}
        for s in kin.get_steppers():
            if s.get_trapq() is None or s.get_name() in unwrapped:
                continue
            rtcp_sk = self._get_rtcp_stepper_kinematics(s)
            if rtcp_sk is None:
                continue
            ffi_lib.rtcp_set_tool(rtcp_sk, tool_h, tool_v, self.frame)
        motion_queuing = self.printer.lookup_object('motion_queuing')
        motion_queuing.check_step_generation_scan_windows()

    def get_tool_offsets(self):
        # A disabled RTCP is a zero tool vector, which makes the transform
        # the identity and drops the steppers' dependency on B
        if not self.enabled:
            return 0., 0.
        return self.tool_h, self.tool_v

    def _can_cross_centre(self):
        kin = self.toolhead.get_kinematics()
        return getattr(kin, 'can_cross_centre', lambda: False)()

    ######################################################################
    # Coordinate transforms
    ######################################################################
    # These mirror rtcp_deltas()/rtcp_apply() in kin_rtcp.c, which is what
    # actually generates the steps.  They are repeated here rather than
    # called through cffi so that positions can be converted without a
    # compiled c_helper.so (and so the host tests can exercise them); the
    # C file carries the derivation.
    def _deltas(self, b_angle, offsets=None):
        b_rad = math.radians(b_angle)
        sin_b, cos_b_1 = math.sin(b_rad), math.cos(b_rad) - 1.
        if offsets is None:
            offsets = self.get_tool_offsets()
        tool_h, tool_v = offsets
        return (tool_h * cos_b_1 - tool_v * sin_b,
                tool_h * sin_b + tool_v * cos_b_1)

    def get_machine_b(self, pos, branch=1):
        # The angle the head is really turned to for a commanded toolhead
        # position.  Normally the commanded B itself, but [b_projection]
        # re-interprets B in the bed frame, and it is the projected angle
        # that swings the tip.
        if self.b_project is None:
            return pos[B_POS_INDEX]
        return self.b_project.project_pos(pos, branch)

    def _apply(self, pos, dh, dz, branch, direction=None):
        # rtcp_apply() in kin_rtcp.c: move a position by (dh, dz).  In the
        # radial frame dh is taken along the arm, which points the way the
        # bed faces - 'direction' where the caller knows it, otherwise the
        # way it faces standing still at 'pos' on 'branch'.  Returns the
        # new position and the branch it is on: the arm radius there is
        # the position's own plus dh, and a large enough dh carries it
        # through the centre.
        res = list(pos)
        res[2] += dz
        if self.frame != FRAME_RADIAL:
            res[0] += dh
            return res, branch
        if direction is None:
            direction = bed_centre.facing(res[0], res[1], None, branch)
        fx, fy = direction
        r_machine = res[0] * fx + res[1] * fy + dh
        res[0] += dh * fx
        res[1] += dh * fy
        if r_machine < 0.:
            return res, -1
        if r_machine > 0.:
            return res, 1
        # On the centre itself: the branch whose position there, standing
        # still, faces the same way along x
        return res, (-1 if fx < 0. else 1)

    def tool_to_machine_branch(self, pos, branch=1, direction=None):
        # Machine (carriage) position for a toolhead position vector on
        # the given branch, and the branch the carriage is on.  The tip
        # swings by the angle the head is really turned to, which
        # [b_projection] can make differ from the commanded B.
        dh, dz = self._deltas(self.get_machine_b(pos, branch))
        return self._apply(pos, dh, dz, branch, direction)

    def tool_to_machine(self, pos, branch=1):
        return self.tool_to_machine_branch(pos, branch)[0]

    def machine_to_tool_branch(self, pos, branch=1, direction=None):
        # Inverse of the above - used when reading positions back out of
        # the steppers so they are reported in the frame g-code uses.  The
        # B in a machine position is already the angle the head is turned
        # to, so it is used as it stands.  A carriage standing exactly on
        # the centre does not say which way the bed faces, so a caller
        # that knows should pass it as 'direction'.
        dh, dz = self._deltas(pos[B_POS_INDEX])
        return self._apply(pos, -dh, -dz, branch, direction)

    def machine_to_tool(self, pos, branch=1, direction=None):
        return self.machine_to_tool_branch(pos, branch, direction)[0]

    def change_offsets(self, pos, branch, old_offsets, new_offsets):
        # The tool position that names the same carriage position under
        # new tool offsets, and its branch.  The bed does not move, so the
        # conversion runs along the way it already faces rather than
        # through a carriage position that may sit on the centre and no
        # longer say which way that is.
        machine_b = self.get_machine_b(pos, branch)
        dh_old, dz_old = self._deltas(machine_b, old_offsets)
        dh_new, dz_new = self._deltas(machine_b, new_offsets)
        direction = bed_centre.facing(pos[0], pos[1], None, branch)
        res, new_branch = self._apply(pos, dh_old - dh_new, dz_old - dz_new,
                                      branch, direction)
        if (self.frame == FRAME_RADIAL and not (res[0] or res[1])
                and bed_centre.facing(0., 0., None, new_branch) != direction):
            # Exactly on the centre, where no position standing still
            # names the way this bed faces
            raise self.printer.command_error(
                "Changing the RTCP offsets here would put the tool exactly"
                " on the centre of the bed, where its position cannot say"
                " which way the bed faces.  Move off the centre first")
        return res, new_branch

    ######################################################################
    # Range checking
    ######################################################################
    def _interior_points(self, move):
        # Positions inside a move that can be worse than either end.  The
        # commanded B changes monotonically along a move, so on its own it
        # needs no interior check - but the geometry a move sweeps out on
        # a polar machine is not monotonic unless the path is radial:
        #
        #  * where the path crosses the bed's x axis, [b_projection] is at
        #    |cos(bed angle)| = 1, so the machine's B - and with it the
        #    swing of the tip - is at its largest.
        #  * where the path passes closest to the centre of the bed the
        #    arm radius is at its smallest, which is what the radial check
        #    below is about.
        #
        # A chord from (40, -30) to (40, 30) has a radius of 50 at both
        # ends and dips to 40 in the middle; the same chord held at B10
        # projects to the same machine B at both ends and runs out to the
        # full 10 between them.  Every coordinate moves linearly along a
        # move, so interpolating the whole position vector is exact.
        sp, ep = move.start_pos, move.end_pos
        ts = []
        if self.b_project is not None:
            ts.append(bed_centre.axis_crossing_fraction(sp, ep, 1))
        if self.frame == FRAME_RADIAL:
            ts.append(bed_centre.closest_approach_fraction(sp, ep))
        return [bed_centre.interpolate(sp, ep, t)
                for t in ts if t is not None]

    def _check_move(self, move):
        # The kinematics checked the *tip* position; make sure the machine
        # position it maps to is also reachable.  Both endpoints are
        # checked, along with any interior extreme - see _interior_points().
        tool_h, tool_v = self.get_tool_offsets()
        if not tool_h and not tool_v:
            return
        kin = self.toolhead.get_kinematics()
        status = kin.get_status(None)
        axis_min = status.get('axis_minimum')
        axis_max = status.get('axis_maximum')
        if axis_min is None or axis_max is None:
            return
        sp, ep = move.start_pos, move.end_pos
        travel = (ep[0] - sp[0], ep[1] - sp[1])
        if not travel[0] and not travel[1]:
            travel = None
        for endpoint in [sp, ep] + self._interior_points(move):
            # Solved the way the step generator solves it: on the branch
            # the move is on there, along the way the bed is driven to face
            branch = bed_centre.sample_branch(
                endpoint[0], endpoint[1], travel, move.branch,
                move.branch_flip)
            direction = bed_centre.facing(endpoint[0], endpoint[1], travel,
                                          branch)
            pos, machine_branch = self.tool_to_machine_branch(
                endpoint, branch, direction)
            if self.frame == FRAME_RADIAL:
                # The arm radius the carriage needs.  The x/y bounds below
                # are a square around the centre, which is not the shape
                # the arm can reach, so check the radius itself - and
                # below zero, the far side of the centre, which only an
                # arm that can travel through the middle can reach.
                radius = machine_branch * math.hypot(pos[0], pos[1])
                r_max = axis_max[0]
                r_min = -r_max if self._can_cross_centre() else 0.
                if radius < r_min - 0.000000001:
                    raise move.move_error(
                        "RTCP move at B=%.3f needs an arm radius of %.3f,"
                        " which is through the centre of the bed"
                        % (endpoint[B_POS_INDEX], radius))
                if radius > r_max + 0.000000001:
                    raise move.move_error(
                        "RTCP move at B=%.3f needs an arm radius of %.3f,"
                        " beyond the %.3f the arm can reach"
                        % (endpoint[B_POS_INDEX], radius, r_max))
            for i, name in ((0, 'X'), (1, 'Y'), (2, 'Z')):
                if pos[i] < axis_min[i] - 0.000000001 \
                        or pos[i] > axis_max[i] + 0.000000001:
                    raise move.move_error(
                        "RTCP move puts the %s carriage at %.3f, outside"
                        " %.3f..%.3f" % (name, pos[i], axis_min[i],
                                         axis_max[i]))

    ######################################################################
    # Enforcement
    ######################################################################
    def check_disabled(self, what):
        # Homing, probing and bed meshing all work in the carriage frame:
        # they need B moves that do not disturb X/Z (which may not even be
        # homed yet), and their results are measured against the carriage.
        # Rather than quietly switching frames underneath the caller, say
        # so.
        if self.enabled:
            raise self.printer.command_error(
                "%s must run with RTCP compensation off - run"
                " SET_RTCP ENABLE=0 first (and SET_RTCP ENABLE=1 afterwards"
                " to print)" % (what,))

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        frame = ('radial' if self.frame == FRAME_RADIAL else 'cartesian')
        return {'enabled': self.enabled,
                'tool_vertical_offset': self.tool_v,
                'tool_horizontal_offset': self.tool_h,
                'horizontal_frame': frame,
                'axis': RTCP_AXIS_GCODE_ID}

    cmd_SET_RTCP_help = "Enable/disable or retune RTCP compensation"
    def cmd_SET_RTCP(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        tool_v = gcmd.get_float('VERTICAL_OFFSET', None)
        tool_h = gcmd.get_float('HORIZONTAL_OFFSET', None)
        if enable is not None or tool_v is not None or tool_h is not None:
            # The machine does not move, but what the reported position
            # *means* does, so convert it across before changing anything
            # and hand the result back as the new toolhead position.  With
            # B at zero the two frames coincide and this is a no-op.
            pos = self.toolhead.get_position()
            branch = self.toolhead.get_branch()
            new_enabled = self.enabled if enable is None else bool(enable)
            new_v = self.tool_v if tool_v is None else tool_v
            new_h = self.tool_h if tool_h is None else tool_h
            new_offsets = (new_h, new_v) if new_enabled else (0., 0.)
            # Raises, before anything has changed, if the new position
            # cannot be expressed
            new_pos, new_branch = self.change_offsets(
                pos, branch, self.get_tool_offsets(), new_offsets)
            self.enabled, self.tool_v, self.tool_h = new_enabled, new_v, new_h
            self._update_kinematics()
            self.toolhead.set_position(new_pos, branch=new_branch)
        gcmd.respond_info(
            "rtcp: enabled=%s tool_vertical_offset=%.6f"
            " tool_horizontal_offset=%.6f"
            % (self.enabled, self.tool_v, self.tool_h))


def load_config(config):
    return RTCP(config)
