# Probing with a probe carried on an RTCP (B axis) tilting head
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The probe is bolted to the tilting head, a fixed angle around the B
# pivot from the nozzle.  It is used in exactly one orientation - the B
# angle at which its pin hangs vertically - and in that orientation it is
# an ordinary probe at a fixed offset from the toolhead.  So the probe
# owns four measured numbers, all of them in its own config section (eg
# [bltouch]):
#
#   b_offset  the B angle at which the pin is vertical.  The nozzle is
#             vertical at B=0, so this is the angular distance from the
#             nozzle round to the probe.
#   z_offset  the usual Klipper meaning - how far the trigger point sits
#             below the nozzle.  (Geometrically it is the difference
#             between the probe's and the nozzle's radius about the B
#             pivot, but it is calibrated the ordinary way and nothing
#             here recomputes it.)
#   x_offset  where the probe sits relative to the toolhead while it is
#   y_offset  in that vertical orientation.
#
# None of this involves the RTCP transform, and it must not: probing runs
# with compensation OFF, where the toolhead position *is* the carriage
# and a B move disturbs nothing else.  That is what lets G28 Z run before
# X and Z are homed, and it is what makes the four offsets above constant
# rather than functions of B.  This module enforces it - see
# check_probe_ready().
#
# The one thing that does not follow from stock Klipper is the frame the
# x/y offsets live in.  On a polar machine (corertheta) the toolhead's
# x/y are *bed* coordinates while the probe is displaced along the arm,
# so a fixed pair of bed-frame offsets would only be right at bed angle
# zero.  The offsets are therefore applied in the machine's own frame at
# the toolhead: x_offset outboard along the arm, y_offset tangential to
# it.  On a cartesian machine that frame is just x/y and the arithmetic
# below reduces to the stock subtraction.
#
# The module registers itself as the printer object 'probe_transform',
# which klippy/extras/probe.py consults in place of the fixed offsets.
# See docs/Multi_Axis.md.
import math
import stepper
from . import manual_probe, rtcp

# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]
RTCP_AXIS_GCODE_ID = 'B'

# Options that described the old, derived geometry.  Everything they said
# is now either a convention (the nozzle is vertical at B=0, a positive B
# tilts it outboard) or a measured probe offset.
REMOVED_OPTIONS = {
    'probe_b_offset':
        "set 'b_offset' in the probe section (eg [bltouch]) to the B angle"
        " at which the probe pin is vertical - the nozzle is vertical at"
        " B=0, so that angle is the offset from the nozzle to the probe",
    'probe_b_position':
        "this is now the probe section's 'b_offset', and it is no longer"
        " derived from anything",
    'invert_b_direction':
        "the head's rotation sense is fixed: a positive B tilts the nozzle"
        " outboard.  A machine that turns the other way sets"
        " invert_b_direction in [printer] / [corertheta]",
}


class RTCPProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        for old, hint in REMOVED_OPTIONS.items():
            if config.get(old, None, note_valid=False) is not None:
                raise config.error(
                    "The [%s] option '%s' has been removed - %s.  See the"
                    " probing section of docs/Multi_Axis.md"
                    % (self.name, old, hint))
        # Probing away from the probe's b_offset is geometric nonsense
        # (the pin is not pointing at the bed), so it is rejected unless
        # the check is turned off.
        self.check_b_angle = config.getboolean('check_probe_b_angle', True)
        self.b_angle_tolerance = config.getfloat('probe_b_angle_tolerance',
                                                 1., above=0.)
        # Largest bed radius the probe has to reach.  Purely a startup
        # sanity check on the offsets - [bed_mesh] mesh_radius is what
        # actually bounds the generated points.
        self.bed_radius = config.getfloat('bed_radius', None, above=0.)
        # Toolhead height the head is raised to before B is rotated, so
        # the probe does not sweep through the bed on its way round.
        self.orient_lift_z = config.getfloat('orient_lift_z', 40., above=0.)
        self.orient_speed = config.getfloat('orient_speed', 20., above=0.)
        self.lift_speed = config.getfloat('orient_lift_speed', 5., above=0.)
        # Resolved at connect time.  The frame the x/y offsets live in is
        # the same one the tool tip swings in, so [rtcp] owns it.
        self.frame = rtcp.FRAME_CARTESIAN
        self.rtcp = None
        self.toolhead = None
        self.probe = None
        self.b_axis = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('RTCP_PROBE_ORIENT', self.cmd_RTCP_PROBE_ORIENT,
                               desc=self.cmd_RTCP_PROBE_ORIENT_help)
        gcode.register_command('RTCP_PROBE_MOVE', self.cmd_RTCP_PROBE_MOVE,
                               desc=self.cmd_RTCP_PROBE_MOVE_help)
        gcode.register_command('RTCP_PROBE_INFO', self.cmd_RTCP_PROBE_INFO,
                               desc=self.cmd_RTCP_PROBE_INFO_help)

    def _connect(self):
        config_error = self.printer.config_error
        self.toolhead = self.printer.lookup_object('toolhead')
        self.rtcp = self.printer.lookup_object('rtcp', None)
        if self.rtcp is None:
            raise config_error("[%s] requires an [rtcp] section - it is what"
                               " tells the machine a B axis carries the tool"
                               % (self.name,))
        self.frame = self.rtcp.frame
        self.probe = self.printer.lookup_object('probe', None)
        if self.probe is None:
            raise config_error("[%s] requires a probe (eg [bltouch])"
                               % (self.name,))
        # B is a rotary axis object, not one of the kinematics' linear
        # axes: it does NOT appear in toolhead.get_status()['homed_axes'],
        # which only ever reports x/y/z.  Its homed flag lives on the axis
        # itself, so keep hold of it.
        for extra_axis in self.toolhead.get_extra_axes():
            get_id = getattr(extra_axis, 'get_axis_gcode_id', None)
            if get_id is not None and get_id() == RTCP_AXIS_GCODE_ID:
                self.b_axis = extra_axis
                break
        else:
            raise config_error(
                "[%s] requires a B axis - add 'b' to the 'additional_axes'"
                " option of the [printer] section" % (self.name,))
        # Check the probing angle is reachable
        b_angle = self.get_probe_b_position()
        kin = self.toolhead.get_kinematics()
        get_rail = getattr(kin, 'get_axis_rail', lambda n: None)
        rail_b = get_rail('b')
        if rail_b is not None:
            b_min, b_max = rail_b.get_range()
            if not b_min <= b_angle <= b_max:
                raise config_error(
                    "The probe pin is vertical at B=%.2f, outside the"
                    " %.2f..%.2f range of the B axis.  Check the sign of"
                    " b_offset in the probe config section"
                    % (b_angle, b_min, b_max))
        # On a polar machine the arm has to be able to put the probe over
        # the whole bed, which a large outboard x_offset can prevent
        rail_r = get_rail('r')
        if (self.frame == rtcp.FRAME_RADIAL and rail_r is not None
                and self.bed_radius is not None):
            x_off = self.probe.get_offsets()[0]
            r_min, r_max = rail_r.get_range()
            reach_min = max(0., max(0., r_min) + x_off)
            reach_max = r_max + x_off
            if reach_min > 0. or reach_max < self.bed_radius:
                raise config_error(
                    "[%s] the probe sits %.2f mm %sboard of the toolhead, so"
                    " it only reaches bed radii %.2f..%.2f, not 0..%.2f."
                    "  Check the sign of x_offset in the probe config section"
                    % (self.name, abs(x_off),
                       "out" if x_off > 0. else "in",
                       reach_min, reach_max, self.bed_radius))

    ######################################################################
    # Geometry
    ######################################################################
    def get_probe_b_position(self):
        # The B angle at which the probe pin is vertical.  The nozzle is
        # vertical at B=0, so the probe's angular offset from the nozzle
        # is that angle.
        return self.probe.get_b_offset()

    def get_nozzle_b_position(self):
        return 0.

    def _xy_offsets(self):
        x_offset, y_offset = self.probe.get_offsets()[:2]
        return x_offset, y_offset

    def bed_to_tool(self, coord):
        # Toolhead x/y that puts the probe point over bed position coord
        x_off, y_off = self._xy_offsets()
        bed_x, bed_y = coord[0], coord[1]
        if self.frame != rtcp.FRAME_RADIAL:
            return [bed_x - x_off, bed_y - y_off]
        # Polar: the offsets are along and across the arm, so solve for
        # the arm radius and the bed angle that put the probe on the point
        bed_radius = math.hypot(bed_x, bed_y)
        if bed_radius < abs(y_off):
            raise self.printer.command_error(
                "Probe point %.2f,%.2f is %.2f mm from the centre of the bed,"
                " closer in than the %.2f mm the probe sits across the arm -"
                " no bed angle can put the probe on it"
                % (bed_x, bed_y, bed_radius, abs(y_off)))
        tool_radius = math.sqrt(bed_radius**2 - y_off**2) - x_off
        if tool_radius < 0.:
            raise self.printer.command_error(
                "Probe point %.2f,%.2f is %.2f mm from the centre of the bed,"
                " inside the %.2f mm the probe sits outboard of the toolhead -"
                " the arm cannot reach it"
                % (bed_x, bed_y, bed_radius, x_off))
        if bed_radius < 1e-9:
            # The bed centre lies on every radius; approach it along +x
            return [tool_radius, 0.]
        bed_angle = math.atan2(bed_y, bed_x)
        tool_angle = bed_angle - math.asin(y_off / bed_radius)
        return [tool_radius * math.cos(tool_angle),
                tool_radius * math.sin(tool_angle)]

    def tool_to_bed(self, x, y):
        # Where the probe point is, for a toolhead at x/y
        x_off, y_off = self._xy_offsets()
        if self.frame != rtcp.FRAME_RADIAL:
            return x + x_off, y + y_off
        tool_radius = math.hypot(x, y)
        tool_angle = math.atan2(y, x) if tool_radius >= 1e-9 else 0.
        # Radial unit vector along the arm, and the tangent across it
        cos_a, sin_a = math.cos(tool_angle), math.sin(tool_angle)
        radial = tool_radius + x_off
        return (radial * cos_a - y_off * sin_a,
                radial * sin_a + y_off * cos_a)

    ######################################################################
    # probe_transform interface (see klippy/extras/probe.py)
    ######################################################################
    def check_probe_ready(self):
        # Probing works in the carriage frame and at one B angle only
        self.rtcp.check_disabled("Probing")
        if not self.check_b_angle:
            return
        b_angle = self._current_b()
        probe_b = self.get_probe_b_position()
        if abs(b_angle - probe_b) > self.b_angle_tolerance:
            raise self.printer.command_error(
                "Probing with B at %.2f, but the probe pin is only vertical"
                " at B=%.2f - run RTCP_PROBE_ORIENT MODE=PROBE first"
                % (b_angle, probe_b))

    def create_probe_result(self, test_pos):
        # Where the probe touched, given the toolhead position at trigger
        z_offset = self.probe.get_offsets()[2]
        bed_x, bed_y = self.tool_to_bed(test_pos[0], test_pos[1])
        return manual_probe.ProbeResult(bed_x, bed_y, test_pos[2] - z_offset,
                                        test_pos[0], test_pos[1], test_pos[2])

    ######################################################################
    # Status and commands
    ######################################################################
    def _current_b(self):
        return self.toolhead.get_position()[B_POS_INDEX]

    def _b_is_homed(self):
        return bool(self.b_axis.get_status().get('homed'))

    def get_status(self, eventtime=None):
        if self.toolhead is None:
            return {}
        b_angle = self._current_b()
        probe_b = self.get_probe_b_position()
        x_off, y_off = self._xy_offsets()
        return {'probe_b_position': probe_b,
                'nozzle_b_position': self.get_nozzle_b_position(),
                'rtcp_enabled': self.rtcp.enabled,
                'x_offset': x_off,
                'y_offset': y_off,
                'z_offset': self.probe.get_offsets()[2],
                'horizontal_frame': ('radial'
                                     if self.frame == rtcp.FRAME_RADIAL
                                     else 'cartesian'),
                'oriented': (not self.rtcp.enabled
                             and abs(b_angle - probe_b)
                             <= self.b_angle_tolerance)}

    def _move_b(self, b_angle, gcmd):
        lift_z = gcmd.get_float('LIFT_Z', self.orient_lift_z)
        curtime = self.printer.get_reactor().monotonic()
        homed = self.toolhead.get_status(curtime)['homed_axes']
        if 'z' in homed:
            if self.toolhead.get_position()[2] < lift_z:
                # Rotating B swings the probe through a large arc, so get
                # the head out of the way of the bed first
                self.toolhead.manual_move([None, None, lift_z],
                                          self.lift_speed)
        else:
            gcmd.respond_info(
                "rtcp_probe: z is not homed - make sure the head is at least"
                " %.1f mm above the bed before rotating B" % (lift_z,))
        newpos = [None] * len(self.toolhead.get_position())
        newpos[B_POS_INDEX] = b_angle
        self.toolhead.manual_move(newpos, self.orient_speed)

    cmd_RTCP_PROBE_ORIENT_help = \
        "Rotate B so the probe (MODE=PROBE) or the nozzle (MODE=TOOL)" \
        " points straight down"
    def cmd_RTCP_PROBE_ORIENT(self, gcmd):
        mode = gcmd.get('MODE', 'PROBE').upper()
        if mode == 'PROBE':
            b_angle = self.get_probe_b_position()
        elif mode == 'TOOL':
            b_angle = self.get_nozzle_b_position()
        elif mode == 'B':
            b_angle = gcmd.get_float('B')
        else:
            raise gcmd.error("MODE must be PROBE, TOOL or B")
        if not self._b_is_homed():
            raise gcmd.error("Must home B before orienting the head")
        if self.rtcp.enabled:
            # With RTCP on a B move is also an X/Z move, so it needs those
            # axes homed - and orienting is something that happens before
            # they are.  Probing does not need the compensation at all.
            curtime = self.printer.get_reactor().monotonic()
            homed = self.toolhead.get_status(curtime)['homed_axes']
            missing = [a for a in 'xyz' if a not in homed]
            if missing:
                raise gcmd.error(
                    "Turning B with RTCP on is an X/Z move, and %s %s not"
                    " homed yet.  Run SET_RTCP ENABLE=0 first - probing does"
                    " not need the compensation."
                    % (", ".join(missing).upper(),
                       "is" if len(missing) == 1 else "are"))
        self._move_b(b_angle, gcmd)
        gcmd.respond_info("rtcp_probe: B moved to %.3f" % (b_angle,))

    cmd_RTCP_PROBE_MOVE_help = (
        "Move the head so the probe is over the given bed position")
    def cmd_RTCP_PROBE_MOVE(self, gcmd):
        # A g-code macro evaluates its whole template before running a
        # line of it, so a macro cannot work this out for itself
        curtime = self.printer.get_reactor().monotonic()
        homed = self.toolhead.get_status(curtime)['homed_axes']
        if 'x' not in homed or 'y' not in homed:
            raise gcmd.error("Must home the arm before RTCP_PROBE_MOVE")
        bed_x = gcmd.get_float('X', 0.)
        bed_y = gcmd.get_float('Y', 0.)
        speed = gcmd.get_float('F', self.orient_speed, above=0.)
        toolpos = self.bed_to_tool((bed_x, bed_y))
        self.toolhead.manual_move(toolpos, speed)
        gcmd.respond_info(
            "rtcp_probe: probe over bed %.3f,%.3f - head at %.3f,%.3f"
            % (bed_x, bed_y, toolpos[0], toolpos[1]))

    cmd_RTCP_PROBE_INFO_help = "Report the probe geometry on the tilting head"
    def cmd_RTCP_PROBE_INFO(self, gcmd):
        x_off, y_off = self._xy_offsets()
        z_off = self.probe.get_offsets()[2]
        probe_b = self.get_probe_b_position()
        frame = ("along and across the arm"
                 if self.frame == rtcp.FRAME_RADIAL else "along x and y")
        gcmd.respond_info(
            "rtcp_probe: the nozzle points down at B=%.3f, the probe pin at"
            " B=%.3f\n"
            "at that angle the probe sits %.4f, %.4f from the toolhead (%s)"
            " and %.4f below it\n"
            "B is currently %.3f, RTCP compensation is %s"
            % (self.get_nozzle_b_position(), probe_b, x_off, y_off, frame,
               z_off, self._current_b(),
               "on" if self.rtcp.enabled else "off"))


def load_config(config):
    rtcp_probe = RTCPProbe(config)
    config.get_printer().add_object('probe_transform', rtcp_probe)
    return rtcp_probe
