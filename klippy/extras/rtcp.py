# Rotational Tool Center Point (RTCP) compensation for a tilting head
#
# Copyright (C) 2025  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# With RTCP enabled the g-code commands where the *tool tip* should be.
# As the B axis tilts the head, the tip swings about the pivot, so the
# linear carriages are moved to hold the tip where it was asked to be.
#
# The geometry and the transform live in klippy/chelper/kin_rtcp.c; this
# module wraps each kinematic stepper with it (the same way
# [input_shaper] wraps them), keeps the reported positions in the tip
# frame, and range checks the machine positions the tip positions map to.
#
# Rotation is assumed to be slow relative to linear motion, so - as
# elsewhere in this fork - the rotational component does not take part in
# the velocity or acceleration planning.  See docs/Multi_Axis.md.
import math
import chelper, stepper

# Only a B axis head (tilting about an axis parallel to Y) is modelled.
RTCP_AXIS = 'b'
RTCP_AXIS_GCODE_ID = 'B'
# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]


class RTCP:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.toolhead = None
        # Offsets from the tool tip to the pivot, measured at B=0.
        # pivot_length is the usual one: how far the pivot sits above the
        # tip.  pivot_x_offset covers a head whose tip is not directly
        # below the pivot.
        self.pivot_z = config.getfloat('pivot_length')
        self.pivot_x = config.getfloat('pivot_x_offset', 0.)
        self.enabled = config.getboolean('enable', True)
        self.orig_stepper_kinematics = []
        self.rtcp_stepper_kinematics = []
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
        self.toolhead.register_move_check(self._check_move)
        self._update_kinematics()

    ######################################################################
    # Stepper wrapping
    ######################################################################
    def _get_rtcp_stepper_kinematics(self, s):
        sk = s.get_stepper_kinematics()
        if sk in self.rtcp_stepper_kinematics:
            return sk
        ffi_main, ffi_lib = chelper.get_ffi()
        rtcp_sk = ffi_main.gc(ffi_lib.rtcp_alloc(), ffi_lib.free)
        s.set_stepper_kinematics(rtcp_sk)
        if ffi_lib.rtcp_set_sk(rtcp_sk, sk) < 0:
            s.set_stepper_kinematics(sk)
            return None
        self.orig_stepper_kinematics.append(sk)
        self.rtcp_stepper_kinematics.append(rtcp_sk)
        return rtcp_sk

    def _update_kinematics(self):
        if self.toolhead is None:
            return
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        px, pz = self.get_pivot()
        kin = self.toolhead.get_kinematics()
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            rtcp_sk = self._get_rtcp_stepper_kinematics(s)
            if rtcp_sk is None:
                continue
            ffi_lib.rtcp_set_pivot(rtcp_sk, px, pz)
        motion_queuing = self.printer.lookup_object('motion_queuing')
        motion_queuing.check_step_generation_scan_windows()

    def get_pivot(self):
        # A disabled RTCP is a zero pivot, which makes the transform the
        # identity and drops the steppers' dependency on B
        if not self.enabled:
            return 0., 0.
        return self.pivot_x, self.pivot_z

    ######################################################################
    # Coordinate transforms
    ######################################################################
    def tip_to_machine(self, pos):
        # Machine (carriage) position for a toolhead position vector
        px, pz = self.get_pivot()
        if not px and not pz:
            return list(pos)
        b_rad = math.radians(pos[B_POS_INDEX])
        sin_b, cos_b_1 = math.sin(b_rad), math.cos(b_rad) - 1.
        res = list(pos)
        res[0] = pos[0] + px * cos_b_1 + pz * sin_b
        res[2] = pos[2] - px * sin_b + pz * cos_b_1
        return res

    def machine_to_tip(self, pos):
        # Inverse of the above - used when reading positions back out of
        # the steppers so they are reported in the frame g-code uses
        px, pz = self.get_pivot()
        if not px and not pz:
            return list(pos)
        b_rad = math.radians(pos[B_POS_INDEX])
        sin_b, cos_b_1 = math.sin(b_rad), math.cos(b_rad) - 1.
        res = list(pos)
        res[0] = pos[0] - px * cos_b_1 - pz * sin_b
        res[2] = pos[2] + px * sin_b - pz * cos_b_1
        return res

    ######################################################################
    # Range checking
    ######################################################################
    def _check_move(self, move):
        # The kinematics checked the *tip* position; make sure the machine
        # position it maps to is also reachable.  Both endpoints are
        # checked - the B angle changes monotonically within a move, so
        # the RTCP offset does too, and no interior point is further out
        # than the ends.
        px, pz = self.get_pivot()
        if not px and not pz:
            return
        kin = self.toolhead.get_kinematics()
        status = kin.get_status(None)
        axis_min = status.get('axis_minimum')
        axis_max = status.get('axis_maximum')
        if axis_min is None or axis_max is None:
            return
        for pos in (self.tip_to_machine(move.start_pos),
                    self.tip_to_machine(move.end_pos)):
            for i, name in ((0, 'X'), (2, 'Z')):
                if pos[i] < axis_min[i] - 0.000000001 \
                        or pos[i] > axis_max[i] + 0.000000001:
                    raise move.move_error(
                        "RTCP move puts %s carriage at %.3f, outside"
                        " %.3f..%.3f" % (name, pos[i], axis_min[i],
                                         axis_max[i]))

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        return {'enabled': self.enabled,
                'pivot_length': self.pivot_z,
                'pivot_x_offset': self.pivot_x,
                'axis': RTCP_AXIS_GCODE_ID}

    cmd_SET_RTCP_help = "Enable/disable or retune RTCP compensation"
    def cmd_SET_RTCP(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        pivot_length = gcmd.get_float('PIVOT_LENGTH', None)
        pivot_x_offset = gcmd.get_float('PIVOT_X_OFFSET', None)
        if enable is not None:
            self.enabled = bool(enable)
        if pivot_length is not None:
            self.pivot_z = pivot_length
        if pivot_x_offset is not None:
            self.pivot_x = pivot_x_offset
        if (enable is not None or pivot_length is not None
            or pivot_x_offset is not None):
            # Changing the transform moves the machine under a fixed tip
            # position, so resync the toolhead to where it now is
            self._update_kinematics()
            self.toolhead.set_position(self.toolhead.get_position())
        gcmd.respond_info(
            "rtcp: enabled=%s pivot_length=%.6f pivot_x_offset=%.6f"
            % (self.enabled, self.pivot_z, self.pivot_x))


def load_config(config):
    return RTCP(config)
