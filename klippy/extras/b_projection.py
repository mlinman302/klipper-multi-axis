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
# so holding B at 10 degrees through a full turn of the bed sweeps the
# machine's B over 10 -> 0 -> -10 -> 0 -> 10.  Angles beyond 'max_angle'
# are orientation commands rather than print moves (swinging the probe
# down, parking the head) and reach the machine untouched; the correction
# fades out over 'taper_range' degrees above max_angle so that the
# machine's B stays continuous across the switch.
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
import chelper, stepper

# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]


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
        self.enabled = config.getboolean('enable', True)
        # Largest |B| that is fully projected, and the band above it over
        # which the projection fades out
        self.max_angle = config.getfloat('max_angle', 40., minval=0.)
        self.taper_range = config.getfloat('taper_range', 5., above=0.)
        # Ceiling on how fast the *machine* B may be driven.  Crossing the
        # taper band changes the machine's B far faster than the commanded
        # B changes, so a move that does it has to be slowed down.  The
        # default converts the gantry's linear speed limit through the
        # coupling ratio, which is where the belt travel actually comes
        # from.
        self.max_b_velocity = config.getfloat('max_b_velocity', None,
                                              above=0.)
        self.max_b_accel = config.getfloat('max_b_accel', None, above=0.)
        self.orig_stepper_kinematics = []
        self.bproject_stepper_kinematics = []
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
        sk = s.get_stepper_kinematics()
        if sk in self.bproject_stepper_kinematics:
            return sk
        ffi_main, ffi_lib = chelper.get_ffi()
        bp_sk = ffi_main.gc(ffi_lib.bproject_alloc(), ffi_lib.free)
        s.set_stepper_kinematics(bp_sk)
        if ffi_lib.bproject_set_sk(bp_sk, sk) < 0:
            s.set_stepper_kinematics(sk)
            return None
        self.orig_stepper_kinematics.append(sk)
        self.bproject_stepper_kinematics.append(bp_sk)
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
        # A disabled projection is a zero max_angle, which makes the
        # transform the identity and drops the steppers' dependency on x/y
        if not self.enabled:
            return 0., self.taper_range
        return self.max_angle, self.taper_range

    ######################################################################
    # Coordinate transforms
    ######################################################################
    def project(self, b, x, y):
        # Machine B for a commanded B at bed position x/y.  The C helper
        # is the one the steppers run, so calling it keeps the host side
        # and the step generation in exact agreement.
        max_angle, taper_range = self.get_params()
        if max_angle <= 0.:
            return b
        ffi_main, ffi_lib = chelper.get_ffi()
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
        # uses.  The mapping is not one to one: where the bed angle puts
        # cos(theta) near zero every commanded B collapses onto nearly the
        # same machine B, and inside the taper band a negative cos(theta)
        # gives two preimages.  So collect every solution and take the one
        # nearest where the toolhead already believes B is - which is the
        # right answer whenever B did not move, and B only moves on a G28
        # B, which happens at the park angle where the map is the
        # identity.
        res = list(pos)
        max_angle, taper_range = self.get_params()
        if max_angle <= 0.:
            return res
        b_m, x, y = pos[B_POS_INDEX], pos[0], pos[1]
        none_angle = max_angle + taper_range
        if abs(b_m) >= none_angle:
            # Outside the band the transform is the identity
            return res
        ref = b_m if fallback_pos is None else fallback_pos[B_POS_INDEX]
        roots = []
        steps = 400
        prev_b = -none_angle
        prev_h = self.project(prev_b, x, y) - b_m
        for i in range(1, steps + 1):
            cand = -none_angle + 2. * none_angle * i / steps
            h = self.project(cand, x, y) - b_m
            if h == 0.:
                roots.append(cand)
            elif (prev_h < 0.) != (h < 0.):
                lo, hi, h_lo = prev_b, cand, prev_h
                for _ in range(60):
                    mid = .5 * (lo + hi)
                    h_mid = self.project(mid, x, y) - b_m
                    if (h_mid < 0.) == (h_lo < 0.):
                        lo, h_lo = mid, h_mid
                    else:
                        hi = mid
                roots.append(.5 * (lo + hi))
            prev_b, prev_h = cand, h
        if not roots:
            # Only reachable if the machine B is not one this transform can
            # produce at all - keep what the toolhead already has
            if fallback_pos is not None:
                res[B_POS_INDEX] = fallback_pos[B_POS_INDEX]
            return res
        res[B_POS_INDEX] = min(roots, key=lambda c: abs(c - ref))
        return res

    ######################################################################
    # Move checking
    ######################################################################
    def _check_move(self, move):
        # The commanded B and the machine B do not move at the same rate:
        # a move that crosses the taper band, or that swings the bed while
        # B is held, can change the machine's B far more than the g-code
        # asked for.  Slow such a move down so the gantry can follow.
        max_angle, _ = self.get_params()
        if max_angle <= 0. or not move.move_d:
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
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        return {'enabled': self.enabled,
                'max_angle': self.max_angle,
                'taper_range': self.taper_range,
                'max_b_velocity': self.max_b_velocity,
                'axis': 'B'}

    cmd_SET_B_PROJECTION_help = "Enable/disable or retune the B projection"
    def cmd_SET_B_PROJECTION(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        max_angle = gcmd.get_float('MAX_ANGLE', None, minval=0.)
        taper_range = gcmd.get_float('TAPER_RANGE', None, above=0.)
        if enable is not None:
            self.enabled = bool(enable)
        if max_angle is not None:
            self.max_angle = max_angle
        if taper_range is not None:
            self.taper_range = taper_range
        if (enable is not None or max_angle is not None
                or taper_range is not None):
            # Changing the transform turns the head under a fixed
            # commanded B, so resync the toolhead to where it now is
            self._update_kinematics()
            self.toolhead.set_position(self.toolhead.get_position())
        gcmd.respond_info(
            "b_projection: enabled=%s max_angle=%.6f taper_range=%.6f"
            % (self.enabled, self.max_angle, self.taper_range))


def load_config(config):
    return BAxisProjection(config)
