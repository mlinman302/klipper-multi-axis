# Bed-frame interpretation of the B axis on a rotating-bed machine
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# On a core r-theta machine the head tilts about the machine's y axis, so
# the tool can only lean within the machine's xz plane, while the bed
# turns underneath it.  A tool orientation expressed in the *bed* frame -
# the frame the g-code x/y words already use - is therefore not reachable
# in general.
#
# With this module loaded a commanded B is read as a lean toward the bed's
# +x direction, and the machine is given the projection of that lean onto
# the plane it can tilt in:
#
#     b_machine = b * cos(theta),   theta = atan2(y, x)  (the bed angle)
#
# So at bed angle 0 a commanded B reaches the machine untouched; directly
# across the bed it arrives inverted; and at the two bed angles square to
# those it collapses to zero, because there the lean being asked for is
# perpendicular to the only plane the head can tilt in.  Holding B at 10
# degrees through a full turn of the bed sweeps the machine's B over
# 10 -> 0 -> -10 -> 0 -> 10.
#
# THE SCALING APPLIES AT EVERY ANGLE
#
# It used to stop above a 'max_angle', so that orientation commands -
# swinging the probe down, parking the head - could reach the machine
# untouched.  A threshold on |B| turned out to be the wrong place to draw
# that line.  Everything the projection held back below the threshold,
# (1 - cos theta) * max_angle, had to be paid out inside the few degrees
# of taper above it; near the square-on bed angles that is almost the
# whole commanded angle, so a ten degree g-code move became forty degrees
# of head travel - and, with [rtcp] turning that into carriage motion,
# tens of millimetres of arm and z with it.
#
# Orientation angles take an explicit route instead: homing, probing and
# turning the head all run with the projection off, exactly as they
# already run with RTCP off.  See check_disabled() below and
# docs/Multi_Axis.md.
#
# Ownership note: the transform belongs to the B axis, not to the bed.
# The bed motor's own position is unchanged - it still follows
# atan2(y, x) - it only supplies the angle.  B, on the other hand, is
# consumed by both gantry motors and by the RTCP tip correction, so the
# remap has to happen ahead of all three.  It is installed as a wrapping
# stepper_kinematics outside [rtcp] (the kin_shaper/kin_rtcp idiom), which
# is what makes every consumer see one consistent B - the angle the head
# is really turned to - and lets the projection be re-evaluated at every
# sample time, since the bed angle changes continuously *within* a move.
import math
import chelper, stepper

# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]

# kin_bproject.c switches to pass-through at max_angle + taper_range, and
# reads a max_angle of zero as "transform disabled".  The projection now
# applies at every angle, so the C is handed a threshold that no reachable
# B can meet; the taper width is then never read.
NO_BAND_ANGLE = 1e30
NO_BAND_TAPER = 1.

# Below this radius (in mm) the bed angle is not meaningfully defined.
# Kept identical to kin_bproject.c and kin_corertheta.c.
BED_MIN_RADIUS = 0.010
# Below this |cos(theta)| the projection has no inverse: every commanded B
# maps onto a machine B of nearly zero.
COS_EPSILON = 1e-6

# Options that configured the pass-through band.  It was removed rather
# than re-tuned - see the note at the top of this file - so a config that
# still sets them means something different now.
REMOVED_OPTIONS = {
    'max_angle': "the projection applies at every B",
    'taper_range': "there is no band left to taper",
}


class BAxisProjection:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.toolhead = None
        printer_config = config.getsection('printer')
        kin_name = printer_config.get('kinematics')
        if kin_name != 'corertheta':
            raise config.error(
                "[b_projection] models a B axis on a rotating bed and only"
                " applies to the 'corertheta' kinematics (this printer uses"
                " '%s')" % (kin_name,))
        for name, why in REMOVED_OPTIONS.items():
            if config.get(name, None, note_valid=False) is not None:
                raise config.error(
                    "The [b_projection] option '%s' no longer exists - %s,"
                    " and homing, probing and RTCP_PROBE_ORIENT run with"
                    " SET_B_PROJECTION ENABLE=0 instead.  See the"
                    " [b_projection] section of docs/Multi_Axis.md"
                    % (name, why))
        # Whether this machine uses the bed-frame interpretation of B at
        # all.  This is the machine-level switch: SET_B_PROJECTION turns
        # the transform off and back on around homing and probing, and
        # RESTORE returns it to the value set here, so a config that says
        # False stays False through the macros rather than being switched
        # back on by the first HOME_ALL.
        self.config_enabled = config.getboolean('enable', True)
        self.enabled = self.config_enabled
        # Ceiling on how fast the *machine* B may be driven.  A move that
        # swings the bed while B is held changes the machine's B without
        # the g-code asking for any B travel at all, so it has to be
        # slowed down.  The default converts the gantry's linear speed
        # limit through the coupling ratio, which is where the belt travel
        # actually comes from.
        self.max_b_velocity = config.getfloat('max_b_velocity', None,
                                              above=0.)
        self.max_b_accel = config.getfloat('max_b_accel', None, above=0.)
        self.orig_stepper_kinematics = []
        # The wrapper installed on each stepper, keyed by stepper name
        self.bproject_stepper_kinematics = {}
        # [rtcp] wraps the steppers in its own klippy:connect handler and
        # this wrapper has to end up outside it, so make sure the rtcp
        # object - and therefore its handler - is created first
        if config.has_section('rtcp'):
            self.printer.load_object(config, 'rtcp')
        self.printer.register_event_handler("klippy:connect", self._connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_B_PROJECTION', self.cmd_SET_B_PROJECTION,
                               desc=self.cmd_SET_B_PROJECTION_help)

    def _connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        have_b = any(ea is not None and ea.get_axis_gcode_id() == 'B'
                     for ea in self.toolhead.get_extra_axes())
        if not have_b:
            raise self.printer.config_error(
                "[b_projection] requires a B axis - add 'b' to the"
                " 'additional_axes' option of the [printer] section")
        if self.max_b_velocity is None:
            kin = self.toolhead.get_kinematics()
            b_ratio = abs(getattr(kin, 'b_coeff', 1.)) or 1.
            self.max_b_velocity = self.toolhead.max_velocity / b_ratio
        self.toolhead.register_move_check(self._check_move)
        self._update_kinematics()

    ######################################################################
    # Stepper wrapping
    ######################################################################
    def _get_bproject_stepper_kinematics(self, s):
        # Found by stepper name, not by reading the stepper's outermost
        # kinematic back - see the matching note in klippy/extras/rtcp.py.
        # Nothing currently wraps outside this module, but recognising
        # only our own wrapper would silently stack a second projection
        # the moment something did.
        bp_sk = self.bproject_stepper_kinematics.get(s.get_name())
        if bp_sk is not None:
            return bp_sk
        sk = s.get_stepper_kinematics()
        ffi_main, ffi_lib = chelper.get_ffi()
        bp_sk = ffi_main.gc(ffi_lib.bproject_alloc(), ffi_lib.free)
        s.set_stepper_kinematics(bp_sk)
        if ffi_lib.bproject_set_sk(bp_sk, sk) < 0:
            s.set_stepper_kinematics(sk)
            return None
        self.orig_stepper_kinematics.append(sk)
        self.bproject_stepper_kinematics[s.get_name()] = bp_sk
        return bp_sk

    def _update_kinematics(self):
        if self.toolhead is None:
            return
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        max_angle, taper_range = self.get_params()
        kin = self.toolhead.get_kinematics()
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            bp_sk = self._get_bproject_stepper_kinematics(s)
            if bp_sk is None:
                continue
            ffi_lib.bproject_set_params(bp_sk, max_angle, taper_range)
        motion_queuing = self.printer.lookup_object('motion_queuing')
        motion_queuing.check_step_generation_scan_windows()

    def get_params(self):
        # What kin_bproject.c is handed.  A disabled projection is a zero
        # max_angle, which makes the transform the identity and drops the
        # steppers' dependency on x/y.
        if not self.enabled:
            return 0., NO_BAND_TAPER
        return NO_BAND_ANGLE, NO_BAND_TAPER

    ######################################################################
    # Coordinate transforms
    ######################################################################
    def cos_bed_angle(self, x, y):
        # cos(theta) at a position, matching bproject_cos_bed_angle() in
        # kin_bproject.c for a position that is not moving.  Inside the
        # dead zone at the centre the C resolves the angle from the
        # direction of travel; a static position has none, and there the
        # bed angle has no effect on anything, so leave B alone.
        r2 = x * x + y * y
        if r2 < BED_MIN_RADIUS * BED_MIN_RADIUS:
            return 1.
        return x / math.sqrt(r2)

    def project(self, b, x, y):
        # Machine B for a commanded B at bed position x/y.  The C helper
        # is the one the steppers run, so calling it keeps the host side
        # and the step generation in exact agreement.
        if not self.enabled:
            return b
        ffi_main, ffi_lib = chelper.get_ffi()
        max_angle, taper_range = self.get_params()
        return ffi_lib.bproject_project_b(b, x, y, max_angle, taper_range)

    def project_pos(self, pos):
        # Machine B for a toolhead position vector
        return self.project(pos[B_POS_INDEX], pos[0], pos[1])

    def commanded_to_machine(self, pos):
        res = list(pos)
        res[B_POS_INDEX] = self.project_pos(pos)
        return res

    def machine_to_commanded(self, pos, fallback_pos=None):
        # Inverse of the above - used when reading a position back out of
        # the steppers (after homing) so B is reported in the frame g-code
        # uses.  The projection is a plain scaling by cos(theta), so the
        # inverse is a division by it.  The one place that fails is a bed
        # angle square to the machine's tilt plane, where every commanded
        # B maps onto a machine B of zero and there is nothing to recover;
        # keep what the toolhead already believes B is there.  Homing runs
        # with the projection off, so in practice this is the identity.
        res = list(pos)
        if not self.enabled:
            return res
        cos_t = self.cos_bed_angle(pos[0], pos[1])
        if abs(cos_t) < COS_EPSILON:
            if fallback_pos is not None:
                res[B_POS_INDEX] = fallback_pos[B_POS_INDEX]
            return res
        res[B_POS_INDEX] = pos[B_POS_INDEX] / cos_t
        return res

    ######################################################################
    # Move checking
    ######################################################################
    def _check_move(self, move):
        # The commanded B and the machine B do not move at the same rate:
        # a move that swings the bed while B is held changes the machine's
        # B without the g-code asking for any B travel at all.  Slow such
        # a move down so the gantry can follow.
        if not self.enabled or not move.move_d:
            return
        b_start = self.project_pos(move.start_pos)
        b_end = self.project_pos(move.end_pos)
        axis_d = abs(b_end - b_start)
        if not axis_d:
            return
        axis_ratio = move.move_d / axis_d
        limit_v = self.max_b_velocity * axis_ratio
        limit_a = 999999999.9
        if self.max_b_accel is not None:
            limit_a = self.max_b_accel * axis_ratio
        move.limit_speed(limit_v, limit_a)

    ######################################################################
    # Enforcement
    ######################################################################
    def check_disabled(self, what):
        # Homing, probing and turning the head all command a *machine* B -
        # the endstop sweep, the angle the probe pin hangs vertical at,
        # the park angle - and with the projection on, every one of them
        # would be scaled by whatever bed angle happened to be under the
        # arm.  They run in the machine frame, the same way they already
        # run with RTCP off.
        if self.enabled:
            raise self.printer.command_error(
                "%s must run with the bed-frame B projection off - run"
                " SET_B_PROJECTION ENABLE=0 first (and SET_B_PROJECTION"
                " ENABLE=1 afterwards to print)" % (what,))

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        return {'enabled': self.enabled,
                'configured_enable': self.config_enabled,
                'max_b_velocity': self.max_b_velocity,
                'axis': 'B'}

    def set_enabled(self, enabled):
        if enabled == self.enabled:
            return
        # Toggling must not turn the head.  It stays at the angle it is
        # at, but the commanded B that names that angle changes, so
        # convert it across before switching and hand the result back as
        # the new toolhead position - the same thing SET_RTCP does with
        # the carriage position.  commanded_to_machine() runs in the old
        # mode and machine_to_commanded() in the new one.  At B=0 the two
        # frames coincide and this is a no-op, which is why the example
        # macros toggle there.
        pos = self.toolhead.get_position()
        machine_pos = self.commanded_to_machine(pos)
        self.enabled = enabled
        self._update_kinematics()
        self.toolhead.set_position(self.machine_to_commanded(machine_pos, pos))

    cmd_SET_B_PROJECTION_help = \
        "Turn the bed-frame B projection off for homing or probing" \
        " (ENABLE=0), or back to the [b_projection] enable setting" \
        " (RESTORE=1)"
    def cmd_SET_B_PROJECTION(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        restore = gcmd.get_int('RESTORE', None, minval=0, maxval=1)
        if enable is not None and restore:
            raise gcmd.error(
                "SET_B_PROJECTION takes ENABLE or RESTORE, not both")
        if restore:
            # Back to whatever the config asked for.  A macro that turned
            # the projection off for a home or a probe uses this rather
            # than ENABLE=1, so that a machine configured with
            # 'enable: False' is not switched on behind the operator.
            enable = self.config_enabled
        if enable is not None:
            self.set_enabled(bool(enable))
        gcmd.respond_info(
            "b_projection: enabled=%s (config enable: %s)"
            " max_b_velocity=%.3f"
            % (self.enabled, self.config_enabled, self.max_b_velocity))


def load_config(config):
    return BAxisProjection(config)
