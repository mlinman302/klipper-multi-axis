# Probe geometry for a probe carried on an RTCP (B axis) tilting head
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# On a normal printer the probe sits at a fixed x/y/z offset from the
# nozzle, which is what [probe] x_offset/y_offset/z_offset describe.  On
# the corertheta machine it does not: the probe is bolted to the tilting
# head, a fixed angle around the B pivot from the nozzle, so its offset
# from the nozzle depends on the B angle - and because the machine is
# polar, that offset is *radial* (along the arm) rather than along a
# fixed cartesian axis.
#
# The model is the one the machine is built to: the nozzle tip and the
# probe point ride two concentric circles about the B pivot.
#
#              . - ~ ~ ~ - .
#          ,                 .        L  = pivot -> nozzle tip
#        ,      pivot         ,       Lp = pivot -> probe point
#       .         o           .            = L + z_offset
#       .        / \          .       probe_b_offset = the angle
#       .   Lp  /   \  L      .            between them, measured
#        .     /     \       ,             in the +B direction
#          .  P       T    ,
#            ' - , _ _ , -
#
# Because the two radii differ by only z_offset (a fraction of a mm),
# [probe] z_offset keeps its usual calibrated meaning - "the probe reads
# the bed this much low" - and no separate z_offset subtraction is done
# on top of the geometry: the shortened probe radius *is* that offset.
#
# The module registers itself as the printer object 'probe_transform',
# which klippy/extras/probe.py consults in place of the fixed x/y/z
# offsets.  See docs/Multi_Axis.md.
import math
import stepper
from . import manual_probe

# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]
RTCP_AXIS_GCODE_ID = 'B'


class RTCPProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        # The angle from the nozzle direction to the probe direction,
        # measured in the direction of increasing B.  On this machine the
        # probe is mounted a quarter turn around from the nozzle, so this
        # is +/- 45.  A positive value puts the probe *inboard* of the
        # nozzle (at a smaller arm radius) when it points at the bed.
        self.probe_b_offset = config.getfloat('probe_b_offset')
        # Which way round the pivot a positive B turns the head.  The two
        # angles above fix where the nozzle and the probe point, but not
        # which side of the pivot each is on while it points there - that
        # depends on the physical B direction, and the [rtcp] pivot
        # offsets are not a reliable guide to it on a machine whose B zero
        # is set by an endstop.  Flip this if the probe ends up on the
        # wrong side of the nozzle; it mirrors the radial offset and
        # leaves both angles, and the z offset, untouched.
        self.invert_b = config.getboolean('invert_b_direction', False)
        # The B angle at which the probe points straight down.  If it is
        # not given it is derived from the [rtcp] pivot offsets, which fix
        # the B angle at which the *nozzle* points straight down.
        self.cfg_probe_b_position = config.getfloat('probe_b_position', None)
        self.probe_b_position = self.cfg_probe_b_position
        # Probing away from probe_b_position is legal - the geometry below
        # is exact for any B - but it is nearly always a mistake, so it is
        # rejected unless the check is turned off.
        self.check_b_angle = config.getboolean('check_probe_b_angle', True)
        self.b_angle_tolerance = config.getfloat('probe_b_angle_tolerance',
                                                 1., above=0.)
        # Largest bed radius the mesh may ask for.  Purely a startup sanity
        # check on the geometry - [bed_mesh] mesh_radius is what actually
        # bounds the generated points.
        self.bed_radius = config.getfloat('bed_radius', None, above=0.)
        # Nozzle height the head is raised to before B is rotated, so the
        # probe does not sweep through the bed on its way round.
        self.orient_lift_z = config.getfloat('orient_lift_z', 40., above=0.)
        self.orient_speed = config.getfloat('orient_speed', 20., above=0.)
        self.lift_speed = config.getfloat('orient_lift_speed', 5., above=0.)
        # Resolved at connect time
        self.rtcp = None
        self.toolhead = None
        self.b_axis = None
        self.tool_radius = 0.
        self.probe_radius = 0.
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
            raise config_error("[%s] requires an [rtcp] section - the probe"
                               " offsets are measured from the B pivot"
                               % (self.name,))
        pprobe = self.printer.lookup_object('probe', None)
        if pprobe is None:
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
        # Radius of the nozzle tip about the B pivot, and of the probe
        # point, which rides the same circle shortened by z_offset
        px, pz = self.rtcp.pivot_x, self.rtcp.pivot_z
        self.tool_radius = math.hypot(px, pz)
        if self.tool_radius < 1e-6:
            raise config_error("[%s] needs a non-zero [rtcp] pivot_length /"
                               " pivot_x_offset" % (self.name,))
        z_offset = pprobe.get_offsets()[2]
        self.probe_radius = self.tool_radius + z_offset
        if self.probe_radius <= 0.:
            raise config_error("[%s] probe z_offset of %.3f is larger than the"
                               " %.3f mm pivot radius"
                               % (self.name, z_offset, self.tool_radius))
        # Derive the probing B angle if it was not configured.  The tip
        # hangs straight below the pivot - the printing orientation - at
        # B = -nozzle_angle, and the probe is probe_b_offset further
        # round.  This is only as good as the [rtcp] offsets: they have to
        # be expressed in the same B frame the endstop sets, which is
        # easiest to arrange by making the printing orientation B=0.
        nozzle_angle = math.degrees(math.atan2(px, pz))
        if self.probe_b_position is None:
            self.probe_b_position = -nozzle_angle - self.probe_b_offset
        # Check the probing angle is reachable
        kin = self.toolhead.get_kinematics()
        get_rail = getattr(kin, 'get_axis_rail', lambda n: None)
        rail_b = get_rail('b')
        if rail_b is not None:
            b_min, b_max = rail_b.get_range()
            if not b_min <= self.probe_b_position <= b_max:
                raise config_error(
                    "[%s] the probe points at the bed at B=%.2f, outside the"
                    " %.2f..%.2f range of the B axis.  Check the sign of"
                    " probe_b_offset, or set probe_b_position to the B angle"
                    " at which the probe pin is vertical"
                    % (self.name, self.probe_b_position, b_min, b_max))
        # Check the bed is reachable.  The arm radius cannot go negative,
        # so with the probe outboard of the nozzle the middle of the bed
        # is unreachable however far the arm is driven back.
        off_r = self.get_offsets(self.probe_b_position)[0]
        rail_r = get_rail('r')
        if rail_r is not None and self.bed_radius is not None:
            r_min, r_max = rail_r.get_range()
            reach_min = max(0., max(0., r_min) + off_r)
            reach_max = r_max + off_r
            if reach_min > 0. or reach_max < self.bed_radius:
                raise config_error(
                    "[%s] with the probe %.2f mm %sboard of the nozzle it can"
                    " only reach bed radii %.2f..%.2f, not 0..%.2f.  If the"
                    " probe is really on the other side of the nozzle, flip"
                    " invert_b_direction - that keeps probe_b_position and"
                    " probe_b_offset as they are"
                    % (self.name, abs(off_r),
                       "out" if off_r > 0. else "in",
                       reach_min, reach_max, self.bed_radius))

    ######################################################################
    # Geometry
    ######################################################################
    def get_offsets(self, b_angle):
        # Offset of the probe point from the nozzle tip at a given B
        # angle, as (radial, z).  Radial is along the arm, positive
        # outward; both are in the frame the toolhead position uses,
        # which - with [rtcp] enabled - is the nozzle tip.
        psi = math.radians(b_angle - self.probe_b_position)
        theta = psi - math.radians(self.probe_b_offset)
        lp, lt = self.probe_radius, self.tool_radius
        # Mirroring the head about the z axis negates the radial component
        # and leaves the z one alone
        radial_sign = -1. if self.invert_b else 1.
        return (radial_sign * (lt * math.sin(theta) - lp * math.sin(psi)),
                lt * math.cos(theta) - lp * math.cos(psi))

    def _current_b(self):
        return self.toolhead.get_position()[B_POS_INDEX]

    def _current_offsets(self):
        if not self.rtcp.enabled:
            raise self.printer.command_error(
                "[%s] needs RTCP compensation enabled - the probe offsets are"
                " measured from the nozzle tip" % (self.name,))
        return self.get_offsets(self._current_b())

    def _b_is_homed(self):
        return bool(self.b_axis.get_status().get('homed'))

    def _check_b_angle(self):
        if not self.check_b_angle:
            return
        b_angle = self._current_b()
        if abs(b_angle - self.probe_b_position) > self.b_angle_tolerance:
            raise self.printer.command_error(
                "Probing with B at %.2f, but the probe only points at the bed"
                " at B=%.2f - run RTCP_PROBE_ORIENT MODE=PROBE first"
                % (b_angle, self.probe_b_position))

    ######################################################################
    # probe_transform interface (see klippy/extras/probe.py)
    ######################################################################
    def bed_to_tool(self, coord):
        # Nozzle x/y that puts the probe point over bed position coord.
        # The offset is radial, so only the radius changes.
        off_r = self._current_offsets()[0]
        x, y = coord[0], coord[1]
        radius = math.hypot(x, y)
        tool_radius = radius - off_r
        if tool_radius < 0.:
            raise self.printer.command_error(
                "Probe point %.2f,%.2f is %.2f mm from the centre of the bed,"
                " inside the %.2f mm the probe sits outboard of the nozzle -"
                " the arm cannot reach it" % (x, y, radius, off_r))
        if radius < 1e-9:
            # The bed centre lies on every radius; approach it along +x
            return [tool_radius, 0.]
        scale = tool_radius / radius
        return [x * scale, y * scale]

    def create_probe_result(self, test_pos):
        # Where the probe actually touched, given the nozzle position at
        # the trigger.  The probe rides the same bed angle as the nozzle,
        # so again only the radius changes.
        off_r, off_z = self._current_offsets()
        x, y = test_pos[0], test_pos[1]
        radius = math.hypot(x, y)
        bed_radius = radius + off_r
        if radius < 1e-9:
            bed_x, bed_y = bed_radius, 0.
        else:
            scale = bed_radius / radius
            bed_x, bed_y = x * scale, y * scale
        return manual_probe.ProbeResult(bed_x, bed_y, test_pos[2] + off_z,
                                        test_pos[0], test_pos[1], test_pos[2])

    def get_z_endstop_position(self):
        # Nozzle z at which the probe touches a bed at z == 0
        return -self._current_offsets()[1]

    def descend_limit_z(self, z_min_position):
        # Lowest nozzle z a probing move may descend to.  Without this the
        # probing move drives the *nozzle* down to the z axis position_min,
        # which - with the probe hanging well below it - buries the probe.
        self._check_b_angle()
        return z_min_position - self._current_offsets()[1]

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        if self.toolhead is None:
            return {}
        b_angle = self._current_b()
        off_r, off_z = self.get_offsets(b_angle)
        return {'probe_b_position': self.probe_b_position,
                'probe_b_offset': self.probe_b_offset,
                'nozzle_b_position': (self.probe_b_position
                                      + self.probe_b_offset),
                'invert_b_direction': self.invert_b,
                'pivot_radius': self.tool_radius,
                'probe_radius': self.probe_radius,
                'radial_offset': off_r,
                'z_offset': off_z,
                'oriented': (abs(b_angle - self.probe_b_position)
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
        " faces the bed"
    def cmd_RTCP_PROBE_ORIENT(self, gcmd):
        mode = gcmd.get('MODE', 'PROBE').upper()
        if mode == 'PROBE':
            b_angle = self.probe_b_position
        elif mode == 'TOOL':
            b_angle = self.probe_b_position + self.probe_b_offset
        elif mode == 'B':
            b_angle = gcmd.get_float('B')
        else:
            raise gcmd.error("MODE must be PROBE, TOOL or B")
        if not self._b_is_homed():
            raise gcmd.error("Must home B before orienting the probe")
        self._move_b(b_angle, gcmd)
        gcmd.respond_info("rtcp_probe: B moved to %.3f" % (b_angle,))

    cmd_RTCP_PROBE_MOVE_help = (
        "Move the head so the probe is over the given bed position")
    def cmd_RTCP_PROBE_MOVE(self, gcmd):
        # The offsets depend on the current B, and a g-code macro
        # evaluates its whole template before running a line of it, so a
        # macro cannot compute this itself after orienting the head
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
        b_angle = gcmd.get_float('B', self._current_b())
        off_r, off_z = self.get_offsets(b_angle)
        pd_r, pd_z = self.get_offsets(self.probe_b_position)
        gcmd.respond_info(
            "rtcp_probe: pivot radius %.4f, probe radius %.4f\n"
            "probe faces the bed at B=%.3f%s, the nozzle at B=%.3f\n"
            "at B=%.3f the probe is %.4f radial, %.4f z from the nozzle\n"
            "while probing it is %.4f %sboard of the nozzle and %.4f below"
            " it, so the nozzle sits at z=%.4f when the probe triggers on a"
            " flat bed"
            % (self.tool_radius, self.probe_radius,
               self.probe_b_position,
               "" if self.cfg_probe_b_position is not None else " (derived)",
               self.probe_b_position + self.probe_b_offset,
               b_angle, off_r, off_z,
               abs(pd_r), "out" if pd_r > 0. else "in", abs(pd_z), -pd_z))


def load_config(config):
    rtcp_probe = RTCPProbe(config)
    config.get_printer().add_object('probe_transform', rtcp_probe)
    return rtcp_probe
