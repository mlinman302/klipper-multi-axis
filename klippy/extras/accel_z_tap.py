# Z homing by tapping the bed with the nozzle, detected by the head IMU
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Drive Z down until the nozzle touches the bed, see the contact as a
# mechanical shock in the IMU bolted to the head, and stop the steppers
# from the MCU that reads the IMU.  The nozzle becomes the Z reference,
# with no probe to deploy and no head turned to a probe angle.  The
# design, and the reasons behind every guard below, are in
# docs/Accel_Z_Tap.md; this is its "Path B".
#
# THE DETECTOR RUNS ON THE MCU.  Host-side detection is two orders of
# magnitude too slow, so the chip's firmware feeds one channel of every
# fifo frame to trigger_analog (src/sensor_bmi160.c), which filters it
# and calls trsync_do_trigger().  This module only configures that
# detector - the filter, the threshold, when it arms - and checks the
# result.  On the corertheta machine the chip is on the Pi's SPI0, so the
# detector runs in klipper_mcu and the trigger is relayed to the LPC1769
# (docs/BMI160_IMU.md, "On this machine").
#
# WHAT THE DETECTOR SEES.  The filter input is offset by the first sample
# processed after arming (the sos filter's auto_offset), which removes
# gravity - a Z move does not tilt the head - so the filter starts from
# rest and the threshold is not a function of B.  An optional high pass
# then removes slow structure and drift, an optional low pass removes
# sensor noise, and "abs_ge" fires on the magnitude, since the sign of
# a contact depends on how the chip is mounted.  The filter runs on raw
# counts in Q10: a full-scale swing is 2^26, leaving the int32 arithmetic
# 32x of headroom for filter overshoot, while the rounding in its
# recursion - which a low high pass amplifies - stays far below a count.
# The threshold is configured in the channel's unit: g for an
# accelerometer channel, deg/s for a gyroscope one.
#
# THE ENDSTOP.  [stepper_z] says endstop_pin: accel_tap:z_virtual_endstop
# and gives a position_endstop, and G28 Z is an ordinary endstop home.
# This is deliberately *not* the "probe" chip and not the 'probe' object:
# a [bltouch] can stay configured alongside for meshing, and whichever
# section [stepper_z] names homes Z.
#
# THE GUARDS, in the order the design document ranks them:
#
#   * The detector arms only once the move is at constant speed, plus
#     arm_delay, so the start of the move is not taken for a contact.
#     The distance covered before that is "blind": a nozzle that starts
#     closer to the bed than that is driven into it undetected.
#   * A trigger within min_trigger_travel of the start of the move is an
#     error, not a result.
#   * A G28 Z with a homing_retract_dist taps twice, and the two contacts
#     must agree within samples_tolerance.  ACCEL_TAP_PROBE takes
#     samples and checks them the way [probe] does.
#   * A lost fifo frame during a tap is an error: the detector may have
#     missed the samples.
#   * B must be homed and at B=0, or the nozzle is not the lowest point of
#     the head.  A hot nozzle is refused - it taps on its own ooze.
#   * trigger_analog's sensor monitor aborts the move if the chip goes
#     quiet.
#
# WHAT IS NOT MEASURED YET.  Every threshold and filter number below is a
# starting point.  Phase 0 of docs/Accel_Z_Tap.md - does the contact
# stand out of the background at all, and in which band - is what
# ACCEL_TAP_QUERY and ACCEL_TAP_CALIBRATE capture.
import logging, math
import mcu, pins, stepper
from . import homing, probe, trigger_analog

# The firmware filter works in raw counts, in Q10
FILTER_UNITS_PER_COUNT = 1024
# High pass and low pass, each up to fourth order
MAX_FILTER_SECTIONS = 4
FILTER_ORDERS = [2, 4]
# A BMI160 sample is a signed 16-bit count
RAW_MIN, RAW_MAX = -32768, 32767
# Every Klipper bulk block of a BMI160 is 48 bytes of frames
BYTES_PER_BLOCK = 48
# Samples taken this long either side of a trigger are the contact, for
# ACCEL_TAP_CALIBRATE's peak
CONTACT_WINDOW = .010
# How long ACCEL_TAP_TEST waits after a tap before re-arming, so the hand
# that tapped has moved away
TEST_REARM_DELAY = .5
# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]
PIN_NAME = 'z_virtual_endstop'
CHIP_NAME = 'accel_tap'


######################################################################
# Filter design
######################################################################

# Butterworth second-order sections by the bilinear transform with
# prewarping - what scipy.signal.butter(..., output='sos') computes, up
# to how the gain is spread across the sections.  Written out here so a
# Raspberry Pi host does not need SciPy.  Each row is
# [b0, b1, b2, 1, a1, a2].
def butterworth_sections(btype, order, cutoff, sample_rate):
    if order % 2 or order < 2:
        raise ValueError("Butterworth order must be even, got %s" % (order,))
    if not 0. < cutoff < sample_rate / 2.:
        raise ValueError("cutoff %.1f Hz is not below the Nyquist frequency"
                         " of %.1f Hz" % (cutoff, sample_rate / 2.))
    k = math.tan(math.pi * cutoff / sample_rate)
    sections = []
    for i in range(order // 2):
        # The analog prototype's pole pairs, as biquad quality factors
        q = 1. / (2. * math.cos((2. * i + 1.) * math.pi / (2. * order)))
        norm = 1. / (1. + k / q + k * k)
        a1 = 2. * (k * k - 1.) * norm
        a2 = (1. - k / q + k * k) * norm
        if btype == 'lowpass':
            b0 = k * k * norm
            b1, b2 = 2. * b0, b0
        elif btype == 'highpass':
            b0 = norm
            b1, b2 = -2. * b0, b0
        else:
            raise ValueError("unknown filter type '%s'" % (btype,))
        sections.append([b0, b1, b2, 1., a1, a2])
    return sections

# The same filter the MCU runs (direct form II transposed, as
# sos_filter_apply()), in floating point, for looking at captured
# samples on the host
def sos_filter(sections, values):
    state = [[0., 0.] for s in sections]
    out = []
    for x in values:
        for (b0, b1, b2, a0, a1, a2), st in zip(sections, state):
            y = b0 * x + st[0]
            st[0] = b1 * x - a1 * y + st[1]
            st[1] = b2 * x - a2 * y
            x = y
        out.append(x)
    return out

# What the detector would have computed from a run of raw counts: offset
# by the first sample (auto_offset), scaled to the channel's unit, and
# filtered.  Returns values in the channel's unit.
def detector_response(sections, raw_counts, counts_per_unit):
    if not raw_counts:
        return []
    first = raw_counts[0]
    return sos_filter(sections, [(c - first) / counts_per_unit
                                 for c in raw_counts])

# The container MCU_SosFilter reads a design from
class TapFilterDesign:
    def __init__(self, sections):
        self.sections = sections
    def get_filter_sections(self):
        return self.sections
    def get_initial_state(self):
        # The input starts at zero by construction (auto_offset)
        return [[0., 0.] for s in self.sections]

# Distance a move of speed/accel covers before the detector arms
def blind_distance(speed, accel, arm_delay):
    accel_dist = 0.
    if accel:
        accel_dist = speed * speed / (2. * accel)
    return accel_dist + speed * (homing.HOMING_START_DELAY + arm_delay)


######################################################################
# MCU plumbing
######################################################################

# trigger_analog's sensor monitor counts ticks of 1/samples-per-second
# between samples, but a BMI160 delivers samples in whole fifo blocks
# every few sample periods, and on a Linux host MCU with jitter on top.
# This presents the chip to MCU_trigger_analog with a tick sized to the
# timeout this module wants.  The filter is designed on the chip's real
# rate, never on this one.
class TapSensor:
    def __init__(self, chip, sensor_timeout):
        self.chip = chip
        rate = chip.get_samples_per_second()
        frames_per_block = BYTES_PER_BLOCK // chip.get_bytes_per_frame()
        poll_interval = frames_per_block / float(rate)
        # The monitor fires after MONITOR_MAX + 2 ticks with no sample
        ticks = trigger_analog.MCU_trigger_analog.MONITOR_MAX + 2
        self.monitor_tick = max(poll_interval, sensor_timeout / ticks)
    def get_mcu(self):
        return self.chip.get_mcu()
    def get_samples_per_second(self):
        return 1. / self.monitor_tick
    def setup_trigger_analog(self, trigger_analog_oid):
        self.chip.setup_trigger_analog(trigger_analog_oid)
    def lookup_sensor_error(self, error_code):
        return self.chip.lookup_sensor_error(error_code)

# MCU_trigger_analog arms the detector at the print time the move starts.
# A tap must arm later - once the move is at speed - while the trsync
# still starts with the move, so this splits the two times.
class TapTriggerAnalog(trigger_analog.MCU_trigger_analog):
    def home_start_armed(self, print_time, arm_time):
        self._last_trigger_time = 0.
        self._reset_filter()
        trigger_completion = self._dispatch.start(print_time)
        arm_clock = self._mcu.print_time_to_clock(max(print_time, arm_time))
        sensor_update = 1. / self._sensor.get_samples_per_second()
        sm_ticks = self._mcu.seconds_to_clock(sensor_update)
        self._home_cmd.send([self._oid, self._dispatch.get_oid(),
            mcu.MCU_trsync.REASON_ENDSTOP_HIT,
            self.REASON_TRIGGER_ANALOG, arm_clock, sm_ticks,
            self.MONITOR_MAX], reqclock=arm_clock)
        return trigger_completion

# The z_virtual_endstop pin.  An ordinary endstop as far as homing.py is
# concerned - it has no get_position_endstop(), so G28 Z runs the normal
# rail home rather than the probe one.
class TapEndstop:
    def __init__(self, tap):
        self.tap = tap
        self.stepper_names = set()
    # MCU_endstop interface
    def get_mcu(self):
        return self.tap.mcu_trigger.get_mcu()
    def add_stepper(self, stepper):
        # The rail adds the Z steppers when [stepper_z] uses this pin, and
        # LookupZSteppers adds them for ACCEL_TAP_PROBE when it does not
        if stepper.get_name() in self.stepper_names:
            return
        self.stepper_names.add(stepper.get_name())
        self.tap.mcu_trigger.get_dispatch().add_stepper(stepper)
    def get_steppers(self):
        return self.tap.mcu_trigger.get_steppers()
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        return self.tap.home_start(print_time, rest_time)
    def home_wait(self, home_end_time):
        return self.tap.home_wait(home_end_time)
    def query_endstop(self, print_time):
        return 0
    # Printer pins module setup_pin() interface
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != PIN_NAME:
            raise pins.error(
                "%s:%s is only useful as an endstop pin"
                % (CHIP_NAME, PIN_NAME))
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error(
                "Can not pullup/invert the %s:%s virtual endstop"
                % (CHIP_NAME, PIN_NAME))
        return self


######################################################################
# The tap
######################################################################

class TapCapture:
    # One tap's worth of host-side samples and what became of it
    def __init__(self, client):
        self.client = client
        self.arm_time = None
        self.trigger_time = 0.
        self.travel = None
        self.trigger_mcu_pos = None

class AccelZTap:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        # The chip.  It has to exist at config time: the detector is
        # attached to it in the MCU's config commands.
        self.chip_name = config.get('accel_chip', 'bmi160')
        if not config.has_section(self.chip_name):
            raise config.error("[%s] accel_chip '%s' is not configured"
                               % (self.name, self.chip_name))
        chip = self.printer.load_object(config, self.chip_name)
        if not hasattr(chip, 'get_trigger_channel_info'):
            raise config.error(
                "[%s] '%s' cannot run a tap detector - it needs an"
                " accelerometer with an mcu-side trigger, such as a [bmi160]"
                % (self.name, self.chip_name))
        self.chip = chip
        self.sample_rate = chip.get_samples_per_second()
        self.channel, self.unit, self.counts_per_unit = \
            chip.get_trigger_channel_info()
        nyquist = self.sample_rate / 2.
        # The filter
        self.highpass = config.getfloat('highpass', 50., minval=0.,
                                        below=nyquist)
        self.highpass_order = config.getchoice('highpass_order',
                                               FILTER_ORDERS, 2)
        self.lowpass = config.getfloat('lowpass', 0., minval=0.,
                                       below=nyquist)
        self.lowpass_order = config.getchoice('lowpass_order',
                                              FILTER_ORDERS, 2)
        if self.highpass and self.lowpass and self.lowpass <= self.highpass:
            raise config.error(
                "[%s] lowpass (%.1f Hz) must be above highpass (%.1f Hz)"
                % (self.name, self.lowpass, self.highpass))
        self.filter_sections = []
        if self.highpass:
            self.filter_sections.extend(butterworth_sections(
                'highpass', self.highpass_order, self.highpass,
                self.sample_rate))
        if self.lowpass:
            self.filter_sections.extend(butterworth_sections(
                'lowpass', self.lowpass_order, self.lowpass,
                self.sample_rate))
        # The threshold, in the channel's unit.  An accelerometer channel
        # gets the design document's starting point; a gyroscope channel
        # has no such number yet, so it must be given.
        if self.unit == 'g':
            self.threshold = config.getfloat('trigger_threshold', .15,
                                             above=0.)
        else:
            self.threshold = config.getfloat('trigger_threshold', above=0.)
        self.trigger_value = int(round(
            self.threshold * self.counts_per_unit * FILTER_UNITS_PER_COUNT))
        if self.trigger_value >= 2**31:
            raise config.error("[%s] trigger_threshold is too large"
                               % (self.name,))
        # Arming and the early-trigger guard
        self.arm_delay = config.getfloat('arm_delay', .050, minval=0.)
        self.min_trigger_travel = config.getfloat('min_trigger_travel', 1.,
                                                  minval=0.)
        self.max_speed = config.getfloat('max_speed', 10., above=0.)
        self.sensor_timeout = config.getfloat('sensor_timeout', .050,
                                              above=0.)
        # A bulk link delivers the last samples after the move ends; the
        # capture waits this long for them before it is checked
        self.batch_margin = config.getfloat('batch_margin', .3, minval=0.)
        # Machine state a tap needs
        self.b_tolerance = config.getfloat('b_tolerance', 1., above=0.)
        self.measure_b = config.getboolean('measure_b', False)
        self.max_extruder_temp = config.getfloat('max_extruder_temp', 150.,
                                                 minval=0.)
        # How far below Z=0 ACCEL_TAP_PROBE may drive looking for the bed
        self.max_overtravel = config.getfloat('max_overtravel', 2., above=0.)
        self.z_min_position = probe.lookup_minimum_z(config)
        # speed, lift_speed, samples, sample_retract_dist, samples_result,
        # samples_tolerance and samples_tolerance_retries, as [probe]
        self.param_helper = probe.ProbeParameterHelper(config)
        # The MCU detector
        self.mcu_sensor = TapSensor(chip, self.sensor_timeout)
        self.mcu_trigger = TapTriggerAnalog(self.mcu_sensor)
        cmd_queue = self.mcu_trigger.get_dispatch().get_command_queue()
        sos = trigger_analog.MCU_SosFilter(chip.get_mcu(), cmd_queue,
                                           MAX_FILTER_SECTIONS)
        sos.set_filter_design(TapFilterDesign(self.filter_sections))
        sos.set_offset_scale(0, FILTER_UNITS_PER_COUNT, auto_offset=True)
        self.mcu_trigger.setup_sos_filter(sos)
        self.mcu_trigger.set_raw_range(RAW_MIN, RAW_MAX)
        self.mcu_trigger.set_trigger('abs_ge', self.trigger_value)
        # The endstop
        self.endstop = TapEndstop(self)
        self.printer.lookup_object('pins').register_chip(CHIP_NAME,
                                                         self.endstop)
        probe.LookupZSteppers(config, self.endstop.add_stepper)
        # State
        self.toolhead = None
        self.capture = None
        self.rail_taps = None
        self.last_capture = None
        self.last_z_result = None
        self.last_peak = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        gcode = self.printer.lookup_object('gcode')
        for cmd in ('ACCEL_TAP_QUERY', 'ACCEL_TAP_TEST', 'ACCEL_TAP_PROBE',
                    'ACCEL_TAP_CALIBRATE'):
            gcode.register_command(cmd, getattr(self, 'cmd_' + cmd),
                                   desc=getattr(self, 'cmd_%s_help' % cmd))
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    ######################################################################
    # Machine state
    ######################################################################
    def get_accel(self):
        # The acceleration a Z-only move runs at
        accels = [getattr(self.toolhead, 'max_accel', None),
                  getattr(self.toolhead.get_kinematics(), 'max_z_accel',
                          None)]
        accels = [a for a in accels if a]
        return min(accels) if accels else None
    def get_blind_distance(self, speed):
        return blind_distance(speed, self.get_accel(), self.arm_delay)
    def _find_b_axis(self):
        for ea in self.toolhead.get_extra_axes():
            if ea is not None and getattr(ea, 'get_axis_gcode_id', None):
                if ea.get_axis_gcode_id() == 'B':
                    return ea
        return None
    def check_b(self):
        # The nozzle is the lowest point of the head only at B=0
        b_homing = self.printer.lookup_object('accel_b_homing', None)
        b_axis = self._find_b_axis()
        if b_homing is None and b_axis is None:
            return
        if b_homing is not None:
            homed = b_homing.is_b_homed()
            b = b_homing.get_commanded_b()
        else:
            homed = bool(b_axis.get_status().get('homed'))
            b = self.toolhead.get_position()[B_POS_INDEX]
        if not homed:
            raise self.printer.command_error(
                "%s: B must be homed before a tap - run G28 B.  Only at B=0"
                " is the nozzle the lowest point of the head" % (self.name,))
        if abs(b) > self.b_tolerance:
            raise self.printer.command_error(
                "%s: B is at %.2f deg - a tap needs the nozzle pointing down,"
                " within %.2f deg of B=0.  Turn the head to B=0 first"
                % (self.name, b, self.b_tolerance))
    def measure_b_now(self):
        if not self.measure_b:
            return
        b_homing = self.printer.lookup_object('accel_b_homing', None)
        if b_homing is None:
            raise self.printer.command_error(
                "%s: measure_b needs [accel_b_homing]" % (self.name,))
        b = b_homing.measure_vertical()
        if abs(b) > self.b_tolerance:
            raise self.printer.command_error(
                "%s: the head measures %.2f deg from vertical - a tap needs"
                " it within %.2f deg.  Re-home B with G28 B"
                % (self.name, b, self.b_tolerance))
    def check_extruder(self):
        # A hot nozzle oozes, and taps early and softly on its own plastic
        if not self.max_extruder_temp:
            return
        extruder = self.printer.lookup_object('extruder', None)
        if extruder is None:
            return
        eventtime = self.printer.get_reactor().monotonic()
        status = extruder.get_heater().get_status(eventtime)
        hottest = max(status['temperature'], status['target'])
        if hottest > self.max_extruder_temp:
            raise self.printer.command_error(
                "%s: the extruder is at %.0fC (target %.0fC) - tapping is"
                " refused above %.0fC, where the nozzle oozes and taps on its"
                " own plastic.  Cool it first, or raise max_extruder_temp"
                % (self.name, status['temperature'], status['target'],
                   self.max_extruder_temp))
    def check_transforms_disabled(self, what):
        for name in ('rtcp', 'b_projection'):
            obj = self.printer.lookup_object(name, None)
            if obj is not None:
                obj.check_disabled(what)
    def check_speed(self, speed, what):
        if speed > self.max_speed:
            raise self.printer.command_error(
                "%s: %s at %.2f mm/s is faster than max_speed (%.2f mm/s).  A"
                " tap keeps driving into the bed for the detection delay and"
                " the deceleration, and its detection delay biases the"
                " contact height by an amount proportional to speed"
                % (self.name, what, speed, self.max_speed))
    def check_approach(self, distance, speed, what):
        # The nozzle must start further from the bed than the detector is
        # blind for, or it reaches the bed before the detector is armed
        blind = self.get_blind_distance(speed)
        needed = max(blind, self.min_trigger_travel)
        if distance <= needed:
            raise self.printer.command_error(
                "%s: %s of %.3f mm is not enough - at %.2f mm/s the detector"
                " is blind for the first %.3f mm of a move and refuses a"
                " contact inside min_trigger_travel (%.3f mm).  Use more than"
                " %.3f mm" % (self.name, what, distance, speed, blind,
                              self.min_trigger_travel, needed))
    def _check_ready(self):
        self.check_extruder()
        self.check_b()

    ######################################################################
    # The capture
    ######################################################################
    def _start_capture(self):
        if self.capture is not None:
            self._stop_capture()
        self.capture = TapCapture(self.chip.start_internal_tap_client())
        return self.capture
    def _stop_capture(self):
        capture, self.capture = self.capture, None
        if capture is not None and not capture.client.is_finished:
            capture.client.finish_measurements()
        return capture
    def _finish_capture(self):
        # Let the batches covering the end of the move arrive
        capture = self.capture
        if capture is None:
            return None
        if self.batch_margin:
            self.toolhead.dwell(self.batch_margin)
        self._stop_capture()
        self.last_capture = capture
        return capture
    def _overflows(self, capture):
        counts = [m.get('overflows', 0) for m in capture.client.msgs]
        if len(counts) < 2:
            return 0
        return counts[-1] - counts[0]
    def capture_response(self, capture, start_time=None, end_time=None):
        # (time, detector output) over a capture, as the MCU filter would
        # have seen it from start_time (default: when it armed)
        if start_time is None:
            start_time = capture.arm_time
        times, counts = [], []
        for sample in capture.client.get_samples():
            if start_time is not None and sample[0] < start_time:
                continue
            if end_time is not None and sample[0] > end_time:
                break
            raw = self.chip.raw_trigger_channel(sample)
            if raw is None:
                return []
            times.append(sample[0])
            counts.append(raw)
        values = detector_response(self.filter_sections, counts,
                                   self.counts_per_unit)
        return list(zip(times, values))
    def _peak(self, response):
        if not response:
            return None
        return max([abs(v) for t, v in response])

    ######################################################################
    # Homing and probing moves (called by homing.py)
    ######################################################################
    def _is_ours(self, hmove):
        return self.endstop in hmove.get_mcu_endstops()
    def _handle_homing_move_begin(self, hmove):
        if not self._is_ours(hmove):
            return
        self._check_ready()
        self._start_capture()
    def home_start(self, print_time, rest_time):
        capture = self.capture
        try:
            if capture is None:
                raise self.printer.command_error(
                    "%s: internal error - a tap move started without a"
                    " capture" % (self.name,))
            steppers = self.endstop.get_steppers()
            speed = accel_time = 0.
            if steppers and rest_time > 0.:
                speed = steppers[0].get_step_dist() / rest_time
                self.check_speed(speed, "the tap move")
                accel = self.get_accel()
                if accel:
                    accel_time = speed / accel
            capture.arm_time = (print_time + homing.HOMING_START_DELAY
                                + accel_time + self.arm_delay)
            return self.mcu_trigger.home_start_armed(print_time,
                                                     capture.arm_time)
        except:
            self._stop_capture()
            raise
    def home_wait(self, home_end_time):
        trigger_time = self.mcu_trigger.home_wait(home_end_time)
        if self.capture is not None:
            self.capture.trigger_time = trigger_time
        return trigger_time
    def _handle_homing_move_end(self, hmove):
        if not self._is_ours(hmove):
            return
        capture = self._finish_capture()
        if capture is None:
            return
        lost = self._overflows(capture)
        if lost:
            raise self.printer.command_error(
                "%s: '%s' reported %d possible fifo overflows during the tap"
                " - the detector may have missed the contact, so the result"
                " is refused.  Lower the chip's rate, or unload the host"
                % (self.name, self.chip_name, lost))
        names = set([s.get_name() for s in self.endstop.get_steppers()])
        positions = [sp for sp in hmove.stepper_positions
                     if sp.stepper_name in names]
        if not capture.trigger_time or not positions:
            return
        sp = positions[0]
        capture.trigger_mcu_pos = sp.trig_pos
        capture.travel = (abs(sp.trig_pos - sp.start_pos)
                          * sp.stepper.get_step_dist())
        if self.rail_taps is not None:
            self.rail_taps.append(capture)
        if capture.travel < self.min_trigger_travel:
            raise self.printer.command_error(
                "%s: triggered after only %.3f mm of travel, inside"
                " min_trigger_travel (%.3f mm).  A contact this soon is more"
                " likely the start of the move than the bed, so it is refused"
                " - start the tap further above the bed, or check the"
                " threshold with ACCEL_TAP_QUERY"
                % (self.name, capture.travel, self.min_trigger_travel))
    def _handle_home_rails_begin(self, homing_state, rails):
        if not any([es is self.endstop for rail in rails
                    for es, name in rail.get_endstops()]):
            return
        self.rail_taps = []
        info = rails[0].get_homing_info()
        self.check_speed(info.speed, "homing_speed")
        if info.retract_dist:
            self.check_speed(info.second_homing_speed, "second_homing_speed")
            self.check_approach(info.retract_dist, info.second_homing_speed,
                                "homing_retract_dist")
        self._check_ready()
        self.measure_b_now()
    def _handle_home_rails_end(self, homing_state, rails):
        taps, self.rail_taps = self.rail_taps, None
        if not taps or len(taps) < 2:
            return
        params = self.param_helper.get_probe_params()
        step_dist = self.endstop.get_steppers()[0].get_step_dist()
        positions = [t.trigger_mcu_pos * step_dist for t in taps]
        spread = max(positions) - min(positions)
        if spread > params['samples_tolerance']:
            raise self.printer.command_error(
                "%s: the homing taps disagree by %.3f mm, more than"
                " samples_tolerance (%.3f mm).  One of them was probably not"
                " the bed - Z is not homed"
                % (self.name, spread, params['samples_tolerance']))
        logging.info("%s: homing taps agree within %.4f mm",
                     self.name, spread)
    def _handle_command_error(self):
        self.rail_taps = None
        try:
            self._stop_capture()
        except:
            logging.exception("%s: stopping the capture", self.name)

    ######################################################################
    # Probing
    ######################################################################
    def _is_z_homed(self):
        curtime = self.printer.get_reactor().monotonic()
        return 'z' in self.toolhead.get_status(curtime)['homed_axes']
    def _check_probe_ready(self, what):
        if not self._is_z_homed():
            raise self.printer.command_error(
                "%s: home Z before %s" % (self.name, what))
        self.check_transforms_disabled(what)
    def tap_once(self, speed):
        # One descent to the bed; returns the contact z
        pos = self.toolhead.get_position()
        pos[2] = max(self.z_min_position, -self.max_overtravel)
        if pos[2] >= self.toolhead.get_position()[2]:
            raise self.printer.command_error(
                "%s: the nozzle is already below the lowest tap position"
                " (z=%.3f)" % (self.name, pos[2]))
        phoming = self.printer.lookup_object('homing')
        epos = phoming.probing_move(self.endstop, pos, speed)
        return epos[2]
    def probe_z(self, gcmd):
        # Tap until the samples agree, as [probe] does
        params = self.param_helper.get_probe_params(gcmd)
        speed = params['probe_speed']
        retract = params['sample_retract_dist']
        self.check_speed(speed, "PROBE_SPEED")
        self.check_approach(retract, speed, "SAMPLE_RETRACT_DIST")
        self.measure_b_now()
        start = self.toolhead.get_position()
        retries = 0
        positions = []
        while len(positions) < params['samples']:
            positions.append(self.tap_once(speed))
            if max(positions) - min(positions) > params['samples_tolerance']:
                if retries >= params['samples_tolerance_retries']:
                    raise gcmd.error("%s: tap samples exceed samples_tolerance"
                                     % (self.name,))
                gcmd.respond_info("Tap samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Lift off the bed after every tap
            lift = self.toolhead.get_position()
            lift[2] += retract
            self.toolhead.manual_move(start[:2] + [lift[2]],
                                      params['lift_speed'])
        if params['samples_result'] == 'median':
            ordered = sorted(positions)
            middle = len(ordered) // 2
            if len(ordered) % 2:
                z = ordered[middle]
            else:
                z = .5 * (ordered[middle - 1] + ordered[middle])
        else:
            z = sum(positions) / len(positions)
        return z, positions

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        speed = self.param_helper.get_probe_params()['probe_speed']
        capture = self.last_capture
        return {'accel_chip': self.chip_name, 'channel': self.channel,
                'unit': self.unit, 'sample_rate': self.sample_rate,
                'trigger_threshold': self.threshold,
                'highpass': self.highpass, 'lowpass': self.lowpass,
                'blind_distance': (None if self.toolhead is None
                                   else self.get_blind_distance(speed)),
                'last_trigger_time': (None if capture is None
                                      else capture.trigger_time),
                'last_travel': None if capture is None else capture.travel,
                'last_z_result': self.last_z_result,
                'last_peak': self.last_peak}
    cmd_ACCEL_TAP_QUERY_help = ("Measure the tap detector's background with"
                                " the machine still")
    def cmd_ACCEL_TAP_QUERY(self, gcmd):
        duration = gcmd.get_float('TIME', 1., minval=.1, maxval=30.)
        self.toolhead.wait_moves()
        capture = self._start_capture()
        capture.arm_time = self.toolhead.get_last_move_time()
        self.toolhead.dwell(duration)
        self._finish_capture()
        response = self.capture_response(capture)
        if not response:
            raise gcmd.error("%s: no samples of %s were captured"
                             % (self.name, self.channel))
        peak = self._peak(response)
        rms = math.sqrt(sum([v * v for t, v in response]) / len(response))
        self.last_peak = peak
        speed = self.param_helper.get_probe_params(gcmd)['probe_speed']
        lines = [
            "%s: %s at %d Hz, %d samples over %.2f s"
            % (self.name, self.channel, self.sample_rate, len(response),
               duration),
            "  detector output: rms %.4f %s, peak %.4f %s"
            % (rms, self.unit, peak, self.unit),
            "  trigger_threshold %.4f %s is %.1fx the still peak"
            % (self.threshold, self.unit,
               self.threshold / peak if peak else float('inf')),
            "  at %.2f mm/s the detector is blind for the first %.3f mm of"
            " a tap" % (speed, self.get_blind_distance(speed))]
        if self._overflows(capture):
            lines.append("  WARNING: the chip reported fifo overflows")
        gcmd.respond_info("\n".join(lines))
    cmd_ACCEL_TAP_TEST_help = ("Arm the tap detector with no movement and"
                               " report taps on the head")
    def cmd_ACCEL_TAP_TEST(self, gcmd):
        taps = gcmd.get_int('TAPS', 3, minval=1, maxval=10)
        timeout = gcmd.get_float('TIMEOUT', 30., minval=1., maxval=120.)
        reactor = self.printer.get_reactor()
        gcmd.respond_info("%s: tap the head %d times" % (self.name, taps))
        for i in range(taps):
            self.toolhead.wait_moves()
            capture = self._start_capture()
            print_time = self.toolhead.get_last_move_time()
            capture.arm_time = print_time + self.arm_delay
            try:
                self.mcu_trigger.home_start_armed(print_time,
                                                  capture.arm_time)
                trigger_time = self.home_wait(print_time + timeout)
                # Make sure the capture runs past the contact
                late = (trigger_time + CONTACT_WINDOW
                        - self.toolhead.get_last_move_time())
                if trigger_time and late > 0.:
                    self.toolhead.dwell(late)
            finally:
                self._finish_capture()
            if not trigger_time:
                raise gcmd.error("%s: no tap detected within %.0f s"
                                 % (self.name, timeout))
            peak = self._peak(self.capture_response(
                capture, end_time=trigger_time + CONTACT_WINDOW))
            self.last_peak = peak
            msg = "Tap %d detected at print time %.4f" % (i + 1, trigger_time)
            if peak is not None:
                msg += ", peak %.4f %s" % (peak, self.unit)
            gcmd.respond_info(msg)
            reactor.pause(reactor.monotonic() + TEST_REARM_DELAY)
        gcmd.respond_info("%s: %d taps detected" % (self.name, taps))
    cmd_ACCEL_TAP_PROBE_help = "Find the bed height by tapping the nozzle"
    def cmd_ACCEL_TAP_PROBE(self, gcmd):
        self._check_probe_ready("ACCEL_TAP_PROBE")
        z, positions = self.probe_z(gcmd)
        self.last_z_result = z
        gcmd.respond_info(
            "%s: nozzle contacts the bed at z=%.4f (%d taps, range %.4f)"
            % (self.name, z, len(positions),
               max(positions) - min(positions)))
    cmd_ACCEL_TAP_CALIBRATE_help = ("Compare the tap detector's background in"
                                    " a moving head with real contacts")
    def cmd_ACCEL_TAP_CALIBRATE(self, gcmd):
        self._check_probe_ready("ACCEL_TAP_CALIBRATE")
        params = self.param_helper.get_probe_params(gcmd)
        speed = params['probe_speed']
        distance = gcmd.get_float('DISTANCE', 5., above=0.)
        taps = gcmd.get_int('TAPS', 3, minval=1, maxval=20)
        self.check_speed(speed, "PROBE_SPEED")
        self.check_approach(params['sample_retract_dist'], speed,
                            "SAMPLE_RETRACT_DIST")
        self._check_ready()
        self.measure_b_now()
        # 1. The background: the same descent, in the air.  Nothing is
        # armed; the capture is filtered on the host exactly as the MCU
        # would have filtered it.
        start = self.toolhead.get_position()
        clearance = max(self.min_trigger_travel,
                        params['sample_retract_dist'])
        if start[2] - distance < clearance:
            raise gcmd.error(
                "%s: raise the nozzle first - the background descent of"
                " %.1f mm must end at least %.3f mm above z=0"
                % (self.name, distance, clearance))
        self.toolhead.wait_moves()
        capture = self._start_capture()
        move_start = self.toolhead.get_last_move_time()
        accel = self.get_accel()
        capture.arm_time = (move_start + self.arm_delay
                            + (speed / accel if accel else 0.))
        low = list(start)
        low[2] -= distance
        self.toolhead.manual_move(low, speed)
        move_end = self.toolhead.get_last_move_time()
        self._finish_capture()
        background = self._peak(self.capture_response(
            capture, end_time=move_end))
        self.toolhead.manual_move(start, params['lift_speed'])
        if background is None:
            raise gcmd.error("%s: no samples of %s were captured in the"
                             " background descent" % (self.name, self.channel))
        # 2. Real contacts, at the current threshold
        contacts = []
        for i in range(taps):
            self.tap_once(speed)
            capture = self.last_capture
            contacts.append(self._peak(self.capture_response(
                capture, start_time=capture.arm_time,
                end_time=capture.trigger_time + CONTACT_WINDOW)))
            lift = self.toolhead.get_position()
            lift[2] += params['sample_retract_dist']
            self.toolhead.manual_move(lift, params['lift_speed'])
        self.toolhead.manual_move(start, params['lift_speed'])
        contacts = [c for c in contacts if c is not None]
        if not contacts:
            raise gcmd.error("%s: no contact samples were captured"
                             % (self.name,))
        weakest = min(contacts)
        if not background:
            raise gcmd.error("%s: the background descent measured exactly"
                             " zero - check the filter settings"
                             % (self.name,))
        ratio = weakest / background
        suggestion = math.sqrt(background * weakest)
        lines = [
            "%s: background peak %.4f %s moving at %.2f mm/s"
            % (self.name, background, self.unit, speed),
            "  contact peaks: %s %s"
            % (", ".join(["%.4f" % c for c in contacts]), self.unit),
            "  weakest contact is %.1fx the background" % (ratio,)]
        if ratio < 3.:
            lines.append(
                "  WARNING: the contacts barely stand out of the background."
                " A threshold between them will false-trigger or miss - try"
                " another highpass/lowpass band, a slower speed, or a"
                " gyroscope tap_channel before trusting tap homing.")
        # The contacts were taken at the current threshold, so they are
        # bounded below by it; the suggestion is the geometric middle of
        # the gap that was actually observed.
        lines.append(
            "  trigger_threshold: %.4f (current %.4f)\nThe SAVE_CONFIG command"
            " will update the printer config file and restart the printer."
            % (suggestion, self.threshold))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'trigger_threshold', "%.4f" % (suggestion,))
        self.last_peak = weakest
        gcmd.respond_info("\n".join(lines))

def load_config(config):
    return AccelZTap(config)
