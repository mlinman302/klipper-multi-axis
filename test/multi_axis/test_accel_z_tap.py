#!/usr/bin/env python
# Host test of Z homing by tapping the bed (klippy/extras/accel_z_tap.py).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* extras/accel_z_tap.py, and the real
# extras/trigger_analog.py under it, against a stubbed printer, mcu and
# chip, so it runs anywhere Python is available (no c_helper.so, serial
# port, IMU or Linux host).
#
# What it can check honestly: the filter design against SciPy, the
# fixed-point filter the MCU runs against the float model the host uses,
# the arming arithmetic, every refusal, sample agreement, and the
# commands sent to the MCU.
#
# What it cannot check: latency, the real contact signature, and every
# threshold number in the config.  Those are machine measurements - see
# "Commissioning" in docs/Accel_Z_Tap.md.
#
# Run with:  python test/multi_axis/test_accel_z_tap.py
import math, os, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# trigger_analog.py reads its trsync reasons from mcu at import time, and
# needs a TriggerDispatch; the real ones need pyserial and c_helper.so
class FakeTrsync:
    REASON_ENDSTOP_HIT = 1
    REASON_HOST_REQUEST = 2
    REASON_PAST_END_TIME = 3
    REASON_COMMS_TIMEOUT = 4

class FakeDispatch:
    def __init__(self, mcu):
        self.mcu = mcu
        self.steppers = []
        self.started = []
        self.wait_ends = []
        self.stop_reason = FakeTrsync.REASON_ENDSTOP_HIT
    def get_oid(self):
        return 77
    def get_command_queue(self):
        return None
    def add_stepper(self, stepper):
        self.steppers.append(stepper)
    def get_steppers(self):
        return list(self.steppers)
    def start(self, print_time):
        self.started.append(print_time)
        return 'completion'
    def wait_end(self, end_time):
        self.wait_ends.append(end_time)
    def stop(self):
        return self.stop_reason

fake_mcu = types.ModuleType('mcu')
fake_mcu.MCU_trsync = FakeTrsync
fake_mcu.TriggerDispatch = FakeDispatch
sys.modules['mcu'] = fake_mcu

import pins
from extras import accel_z_tap as azt, trigger_analog

try:
    import scipy.signal
except ImportError:
    scipy = None


######################################################################
# Stubbed printer environment
######################################################################

class CommandError(Exception):
    pass

class FakeCommand:
    def __init__(self, fmt, log):
        self.name = fmt.split()[0]
        self.log = log
    def send(self, args, reqclock=None):
        self.log.append((self.name, list(args), reqclock))

class FakeQueryCommand:
    def __init__(self, mcu):
        self.mcu = mcu
    def send(self, args):
        return {'homing_clock': self.mcu.trigger_clock}

class FakeMCU:
    CLOCK_FREQ = 1000000.
    def __init__(self, printer):
        self.printer = printer
        self.next_oid = 0
        self.config_cmds = []
        self.config_callbacks = []
        self.sent = []
        self.trigger_clock = 0
    def get_printer(self):
        return self.printer
    def create_oid(self):
        self.next_oid += 1
        return self.next_oid - 1
    def add_config_cmd(self, cmd, **kwargs):
        self.config_cmds.append(cmd)
    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)
    def build_config(self):
        for cb in self.config_callbacks:
            cb()
    def lookup_command(self, fmt, cq=None):
        return FakeCommand(fmt, self.sent)
    def lookup_query_command(self, fmt, resp, oid=None, cq=None):
        return FakeQueryCommand(self)
    def get_enumerations(self):
        return {'trigger_analog_error:': {'RAW_RANGE': 0, 'OVERFLOW': 1,
                                          'MONITOR': 2, 'SENSOR_SPECIFIC': 3}}
    def print_time_to_clock(self, print_time):
        return int(print_time * self.CLOCK_FREQ)
    def clock_to_print_time(self, clock):
        return clock / self.CLOCK_FREQ
    def clock32_to_clock64(self, clock):
        return clock
    def seconds_to_clock(self, secs):
        return int(secs * self.CLOCK_FREQ)
    def is_fileoutput(self):
        return False
    def sent_named(self, name):
        return [s for s in self.sent if s[0] == name]

class FakeClient:
    def __init__(self, chip, start):
        self.chip = chip
        self.start = start
        self.end = None
        self.is_finished = False
        self.msgs = [{'overflows': 0}, {'overflows': chip.overflows}]
    def finish_measurements(self):
        self.end = self.chip.toolhead.get_last_move_time()
        self.is_finished = True
    def get_samples(self):
        chip = self.chip
        if chip.signal is not None:
            # Samples on the chip's clock, generated from a signal in g
            first = int(math.ceil(self.start * chip.rate))
            last = int(math.floor(self.end * chip.rate))
            return [(i / float(chip.rate), 0., 0., 0., 0., 0.,
                     chip.signal(i / float(chip.rate)))
                    for i in range(first, last + 1)]
        return [s for s in chip.samples if self.start <= s[0] <= self.end]

# A BMI160-shaped chip whose samples are whatever the test puts in
# `samples`: (time, gx, gy, gz, ax, ay, az), with raw counts recovered
# from the accel z column at 16384 counts per g
class FakeChip:
    def __init__(self, printer, rate=1600, channel=('accel_z', 'g', 16384.),
                 bytes_per_frame=12):
        self.printer = printer
        self.mcu = FakeMCU(printer)
        self.rate = rate
        self.channel = channel
        self.bytes_per_frame = bytes_per_frame
        self.attached = []
        self.samples = []
        self.signal = None
        self.overflows = 0
        self.clients = []
        self.toolhead = None
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.rate
    def get_bytes_per_frame(self):
        return self.bytes_per_frame
    def get_trigger_channel_info(self):
        return self.channel
    def setup_trigger_analog(self, oid):
        self.attached.append(oid)
    def lookup_sensor_error(self, code):
        return "bmi160 error %d" % (code,)
    def raw_trigger_channel(self, sample):
        return sample[6] * self.channel[2]
    def start_internal_tap_client(self):
        client = FakeClient(self, self.toolhead.get_last_move_time())
        self.clients.append(client)
        return client

class FakeStepper:
    def __init__(self, name='stepper_z', step_dist=.01):
        self.name, self.step_dist = name, step_dist
    def get_name(self):
        return self.name
    def get_step_dist(self):
        return self.step_dist

class FakeKinematics:
    max_z_accel = 50.

class FakeRotaryAxis:
    def __init__(self, homed=True):
        self.homed = homed
    def get_axis_gcode_id(self):
        return 'B'
    def get_status(self, eventtime=None):
        return {'homed': self.homed}

class FakeToolhead:
    max_accel = 3000.
    def __init__(self):
        self.position = [0., 0., 10., 0., 0., 0., 0.]
        self.print_time = 100.
        self.kin = FakeKinematics()
        self.b_axis = FakeRotaryAxis()
        self.extra_axes = [None, None, self.b_axis]
        self.homed_axes = 'xyz'
        self.moves = []
        self.dwells = []
    def get_kinematics(self):
        return self.kin
    def get_extra_axes(self):
        return list(self.extra_axes)
    def get_position(self):
        return list(self.position)
    def get_last_move_time(self):
        return self.print_time
    def get_status(self, eventtime):
        return {'homed_axes': self.homed_axes}
    def wait_moves(self):
        pass
    def dwell(self, delay):
        self.dwells.append(delay)
        self.print_time += delay
    def manual_move(self, coord, speed):
        for i, c in enumerate(coord):
            if c is not None:
                self.position[i] = c
        self.moves.append((list(coord), speed))
        self.print_time += 1.

class FakeHeater:
    def __init__(self, temperature=25., target=0.):
        self.temperature, self.target = temperature, target
    def get_status(self, eventtime):
        return {'temperature': self.temperature, 'target': self.target}

class FakeExtruder:
    def __init__(self, heater):
        self.heater = heater
    def get_heater(self):
        return self.heater

class FakeReactor:
    def monotonic(self):
        return 0.
    def pause(self, waketime):
        pass

class FakeGCodeCommand:
    error = CommandError
    def __init__(self, params=None):
        self.params = dict(params or {})
        self.responses = []
    def get_float(self, name, default=Ellipsis, **kwargs):
        if name in self.params:
            return float(self.params[name])
        return default
    def get_int(self, name, default=Ellipsis, **kwargs):
        if name in self.params:
            return int(self.params[name])
        return default
    def get(self, name, default=Ellipsis):
        return self.params.get(name, default)
    def respond_info(self, msg):
        self.responses.append(msg)

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func
    def create_gcode_command(self, cmd, line, params):
        return FakeGCodeCommand(params)

class FakeConfigfile:
    def __init__(self):
        self.saved = {}
    def set(self, section, option, value):
        self.saved[(section, option)] = value

class FakePrinter:
    config_error = CommandError
    command_error = CommandError
    def __init__(self):
        self.objects = {'gcode': FakeGCode(), 'pins': pins.PrinterPins(),
                        'configfile': FakeConfigfile()}
        self.handlers = {}
        self.chip = None
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=Ellipsis):
        if name in self.objects:
            return self.objects[name]
        if default is not Ellipsis:
            return default
        raise CommandError("Unknown object '%s'" % (name,))
    def load_object(self, config, name):
        return self.objects[name]
    def get_reactor(self):
        return FakeReactor()
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *args):
        for cb in self.handlers.get(event, []):
            cb(*args)

class FakeConfig:
    error = CommandError
    def __init__(self, printer, values, sections=('bmi160',)):
        self.printer = printer
        self.values = values
        self.sections = sections
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'accel_z_tap'
    def has_section(self, name):
        return name in self.sections
    def get_prefix_sections(self, prefix):
        return []
    def getsection(self, name):
        # [stepper_z] and [printer], for probe.lookup_minimum_z()
        return FakeConfig(self.printer, {'position_min': -8.,
                                         'minimum_z_position': -8.})
    def _lookup(self, option, default):
        if option in self.values:
            return self.values[option]
        if default is Ellipsis:
            raise CommandError("Option '%s' is required" % (option,))
        return default
    def get(self, option, default=Ellipsis, **kwargs):
        return self._lookup(option, default)
    def getfloat(self, option, default=Ellipsis, minval=None, maxval=None,
                 above=None, below=None, **kwargs):
        val = self._lookup(option, default)
        if val is None:
            return None
        val = float(val)
        if minval is not None and val < minval:
            raise CommandError("%s must have minimum of %s" % (option, minval))
        if above is not None and val <= above:
            raise CommandError("%s must be above %s" % (option, above))
        if below is not None and val >= below:
            raise CommandError("%s must be below %s" % (option, below))
        return val
    def getint(self, option, default=Ellipsis, **kwargs):
        val = self._lookup(option, default)
        return val if val is None else int(val)
    def getboolean(self, option, default=Ellipsis, **kwargs):
        return bool(self._lookup(option, default))
    def getchoice(self, option, choices, default=Ellipsis, **kwargs):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        c = self._lookup(option, default)
        if type(list(choices.keys())[0]) == int:
            c = int(c)
        if c not in choices:
            raise CommandError("Choice '%s' for '%s' is not valid"
                               % (c, option))
        return choices[c]

def build(values=None, chip_kwargs=None,
          sections=('bmi160', 'stepper_z')):
    printer = FakePrinter()
    chip = FakeChip(printer, **(chip_kwargs or {}))
    printer.add_object('bmi160', chip)
    toolhead = FakeToolhead()
    chip.toolhead = toolhead
    printer.add_object('toolhead', toolhead)
    tap = azt.AccelZTap(FakeConfig(printer, dict(values or {}), sections))
    chip.mcu.build_config()
    printer.send_event("klippy:connect")
    stepper = FakeStepper()
    tap.endstop.add_stepper(stepper)
    return tap


######################################################################
# Filter design
######################################################################

def magnitude(sections, freq, rate):
    w = 2. * math.pi * freq / rate
    z1 = complex(math.cos(-w), math.sin(-w))
    z2 = z1 * z1
    res = 1.
    for b0, b1, b2, a0, a1, a2 in sections:
        res *= (b0 + b1 * z1 + b2 * z2) / (a0 + a1 * z1 + a2 * z2)
    return abs(res)

class TestFilterDesign(unittest.TestCase):
    CASES = [('highpass', 2, 50., 1600.), ('highpass', 4, 50., 1600.),
             ('lowpass', 2, 400., 1600.), ('lowpass', 4, 150., 400.),
             ('highpass', 2, 5., 400.)]
    def test_the_cutoff_is_three_db_down(self):
        for btype, order, cutoff, rate in self.CASES:
            sections = azt.butterworth_sections(btype, order, cutoff, rate)
            self.assertAlmostEqual(magnitude(sections, cutoff, rate),
                                   math.sqrt(.5), places=9)
    def test_pass_and_stop_bands(self):
        hp = azt.butterworth_sections('highpass', 2, 50., 1600.)
        self.assertAlmostEqual(magnitude(hp, 0., 1600.), 0., places=12)
        self.assertAlmostEqual(magnitude(hp, 800., 1600.), 1., places=12)
        lp = azt.butterworth_sections('lowpass', 4, 400., 1600.)
        self.assertAlmostEqual(magnitude(lp, 0., 1600.), 1., places=12)
        self.assertAlmostEqual(magnitude(lp, 800., 1600.), 0., places=12)
    @unittest.skipIf(scipy is None, "SciPy is not installed")
    def test_matches_scipy_butter(self):
        for btype, order, cutoff, rate in self.CASES:
            ours = azt.butterworth_sections(btype, order, cutoff, rate)
            theirs = scipy.signal.butter(order, Wn=cutoff, btype=btype,
                                         fs=rate, output='sos')
            for f in (0., cutoff / 3., cutoff, cutoff * 1.7, rate * .45):
                self.assertAlmostEqual(magnitude(ours, f, rate),
                                       magnitude(theirs, f, rate), places=9)
            # Same poles, in any order of sections
            ours_a = sorted([tuple(round(c, 9) for c in s[4:]) for s in ours])
            theirs_a = sorted([tuple(round(c, 9) for c in s[4:])
                               for s in theirs])
            self.assertEqual(ours_a, theirs_a)
    @unittest.skipIf(scipy is None, "SciPy is not installed")
    def test_the_host_model_is_scipy_sosfilt(self):
        sections = azt.butterworth_sections('highpass', 4, 60., 1600.)
        values = [math.sin(i * .3) + .01 * i for i in range(300)]
        ours = azt.sos_filter(sections, values)
        theirs = scipy.signal.sosfilt(sections, values)
        for a, b in zip(ours, theirs):
            self.assertAlmostEqual(a, b, places=9)
    def test_bad_designs_are_refused(self):
        with self.assertRaises(ValueError):
            azt.butterworth_sections('highpass', 3, 50., 1600.)
        with self.assertRaises(ValueError):
            azt.butterworth_sections('lowpass', 2, 800., 1600.)
        with self.assertRaises(ValueError):
            azt.butterworth_sections('bandpass', 2, 80., 1600.)


######################################################################
# The MCU's fixed-point filter against the host's float model
######################################################################

# sos_filter_apply() and fixed_mul() from src/sos_filter.c, in Python
def fixed_mul(coeff, value, frac_bits):
    result = coeff * value
    if frac_bits:
        result += 1 << (frac_bits - 1)
        result >>= frac_bits
    if not -2**31 <= result < 2**31:
        raise OverflowError("int32 overflow")
    return result

class FirmwareFilter:
    def __init__(self, sos):
        self.coeffs = sos._convert_filter()
        self.cfb = sos._coeff_frac_bits
        self.scale_frac_bits = trigger_analog.calc_frac_bits([sos._scale])
        self.scale = trigger_analog.to_fixed_32(sos._scale,
                                                self.scale_frac_bits)
        self.auto_offset = sos._auto_offset
        self.offset = sos._offset
        self.state = [[0, 0] for c in self.coeffs]
    def apply(self, raw):
        if self.auto_offset:
            self.offset = -raw
            self.auto_offset = False
        cur = fixed_mul(self.scale, raw + self.offset, self.scale_frac_bits)
        for (c0, c1, c2, c3, c4), st in zip(self.coeffs, self.state):
            nxt = fixed_mul(c0, cur, self.cfb) + st[0]
            st0 = fixed_mul(c1, cur, self.cfb) - fixed_mul(c3, nxt, self.cfb)
            st1 = fixed_mul(c2, cur, self.cfb) - fixed_mul(c4, nxt, self.cfb)
            st[0], st[1] = st0 + st[1], st1
            cur = nxt
        return cur

def contact_counts(rate, counts_per_g, gravity_g=1., contact_g=.6,
                   contact_at=.1, ring_hz=350., decay=.004, length=.2):
    # A parked-looking head carrying gravity, then a contact: a damped
    # ring at the head's resonance, starting at contact_at
    res = []
    for i in range(int(length * rate)):
        t = i / float(rate)
        g = gravity_g
        if t >= contact_at:
            dt = t - contact_at
            g += (contact_g * math.exp(-dt / decay)
                  * math.sin(2. * math.pi * ring_hz * dt))
        res.append(int(round(g * counts_per_g)))
    return res

class TestFixedPoint(unittest.TestCase):
    def _run(self, values, counts):
        tap = build(values)
        sos = tap.mcu_trigger.get_sos_filter()
        fw = FirmwareFilter(sos)
        scale = azt.FILTER_UNITS_PER_COUNT * tap.counts_per_unit
        fixed = [fw.apply(c) / scale for c in counts]
        model = azt.detector_response(tap.filter_sections, counts,
                                      tap.counts_per_unit)
        return tap, fixed, model
    def test_gravity_does_not_trigger_and_a_contact_does(self):
        counts = contact_counts(1600, 16384.)
        tap, fixed, model = self._run({}, counts)
        scale = azt.FILTER_UNITS_PER_COUNT * tap.counts_per_unit
        first = [i for i, v in enumerate(fixed)
                 if round(abs(v) * scale) >= tap.trigger_value]
        self.assertTrue(first)
        self.assertGreaterEqual(first[0], int(.1 * 1600))
        self.assertLess(first[0], int(.1 * 1600) + 4)
    def test_the_fixed_point_filter_tracks_the_float_model(self):
        # A low, steep high pass puts its poles close to the unit circle,
        # where the recursion amplifies rounding - the case that decided
        # the filter's Q10 units.  In Q16 of g it drifted by 12 mg.
        for values in ({}, {'highpass': 20., 'highpass_order': 4,
                            'lowpass': 500.}, {'highpass': 0.},
                       {'highpass': 5., 'highpass_order': 4}):
            counts = contact_counts(1600, 16384., length=2.)
            tap, fixed, model = self._run(values, counts)
            for a, b in zip(fixed, model):
                self.assertAlmostEqual(a, b, delta=1e-4)
    def test_sixteen_g_range_and_a_hard_hit_do_not_overflow(self):
        counts = [max(-32768, min(32767, c)) for c in
                  contact_counts(1600, 2048., contact_g=15.)]
        tap, fixed, model = self._run({}, counts)
        for a, b in zip(fixed, model):
            self.assertAlmostEqual(a, b, delta=1e-4)
    def test_a_gyroscope_channel_in_deg_per_s(self):
        tap = build({'trigger_threshold': 20.},
                    {'channel': ('gyro_y', 'deg/s', 131.2)})
        fw = FirmwareFilter(tap.mcu_trigger.get_sos_filter())
        # A full-scale swing from rest, which is the worst case for the
        # offset and scale arithmetic
        counts = [0] * 10 + [32767] * 10 + [-32768] * 10
        model = azt.detector_response(tap.filter_sections, counts, 131.2)
        scale = azt.FILTER_UNITS_PER_COUNT * 131.2
        for c, m in zip(counts, model):
            self.assertAlmostEqual(fw.apply(c) / scale, m, delta=1e-3)
        # 20 deg/s in raw Q10 counts
        self.assertEqual(tap.trigger_value, int(round(20. * 131.2 * 1024)))


######################################################################
# Configuration and MCU setup
######################################################################

class TestConfig(unittest.TestCase):
    def test_defaults_and_mcu_setup(self):
        tap = build()
        self.assertEqual(tap.threshold, .15)
        self.assertEqual(len(tap.filter_sections), 1)
        mcu = tap.chip.mcu
        self.assertEqual(tap.chip.attached, [tap.mcu_trigger.get_oid()])
        sos = tap.mcu_trigger.get_sos_filter()
        self.assertIn("config_sos_filter oid=%d max_sections=4"
                      % (sos.get_oid(),), mcu.config_cmds)
        tap.mcu_trigger._reset_filter()
        (_, args, _), = mcu.sent_named('trigger_analog_set_trigger')
        self.assertEqual(args[1:], ['abs_ge', int(round(.15 * 16384 * 1024))])
        (_, args, _), = mcu.sent_named('trigger_analog_set_raw_range')
        self.assertEqual(args[1:], [-32768, 32767])
        (_, args, _), = mcu.sent_named('sos_filter_set_offset_scale')
        # Q10 counts, auto_offset on
        self.assertEqual(args[1:], [0, 1024, 0, True])
        (_, args, _), = mcu.sent_named('sos_filter_set_active')
        self.assertEqual(args[1], 1)
    def test_a_gyroscope_channel_needs_a_threshold(self):
        with self.assertRaises(CommandError) as cm:
            build(chip_kwargs={'channel': ('gyro_x', 'deg/s', 131.2)})
        self.assertIn("trigger_threshold", str(cm.exception))
    def test_filter_bands_are_checked(self):
        with self.assertRaises(CommandError):
            build({'highpass': 800.})
        with self.assertRaises(CommandError) as cm:
            build({'highpass': 100., 'lowpass': 80.})
        self.assertIn("must be above highpass", str(cm.exception))
        with self.assertRaises(CommandError):
            build({'highpass_order': 3})
    def test_the_chip_must_be_a_configured_bmi160(self):
        with self.assertRaises(CommandError) as cm:
            build(sections=())
        self.assertIn("is not configured", str(cm.exception))
        for name in ('adxl345', 'lis2dw head'):
            printer = FakePrinter()
            with self.assertRaises(CommandError) as cm:
                azt.AccelZTap(FakeConfig(printer, {'accel_chip': name}))
            self.assertIn("must name a [bmi160] section", str(cm.exception))
    def test_the_virtual_endstop_pin(self):
        tap = build()
        ppins = tap.printer.lookup_object('pins')
        params = ppins.parse_pin('accel_tap:z_virtual_endstop', True, True)
        self.assertIs(params['chip'].setup_pin('endstop', params),
                      tap.endstop)
        for desc in ('!accel_tap:z_virtual_endstop',
                     '^accel_tap:z_virtual_endstop'):
            params = ppins.parse_pin(desc, True, True)
            with self.assertRaises(pins.error):
                params['chip'].setup_pin('endstop', params)
        # Not the probe's chip, and no 'probe' object - a BLTouch coexists
        self.assertNotIn('probe', ppins.chips)
        self.assertNotIn('probe', tap.printer.objects)
        self.assertFalse(hasattr(tap.endstop, 'get_position_endstop'))
    def test_a_stepper_is_only_added_once(self):
        tap = build()
        tap.endstop.add_stepper(FakeStepper())
        self.assertEqual(len(tap.endstop.get_steppers()), 1)
    def test_all_commands_are_registered(self):
        gcode = build().printer.lookup_object('gcode')
        for cmd in ('ACCEL_TAP_QUERY', 'ACCEL_TAP_TEST', 'ACCEL_TAP_PROBE',
                    'ACCEL_TAP_CALIBRATE'):
            self.assertIn(cmd, gcode.commands)

class TestSensorMonitor(unittest.TestCase):
    def test_the_monitor_tick_is_at_least_a_fifo_poll(self):
        # 1600 Hz, 4 combined frames per block: a 2.5 ms poll, but the
        # default 50 ms timeout over 5 ticks is 10 ms
        tap = build()
        self.assertAlmostEqual(tap.mcu_sensor.monitor_tick, .010)
        # 200 Hz, 8 accel-only frames per block: a 40 ms poll wins
        tap = build(chip_kwargs={'rate': 200, 'bytes_per_frame': 6})
        self.assertAlmostEqual(tap.mcu_sensor.monitor_tick, .040)


######################################################################
# Arming, and the checks around a tap move
######################################################################

class FakeStepperPosition:
    def __init__(self, stepper, start_pos, trig_pos):
        self.stepper = stepper
        self.stepper_name = stepper.get_name()
        self.start_pos, self.trig_pos = start_pos, trig_pos

class FakeHomingMove:
    def __init__(self, endstop, start_pos=0, trig_pos=-500):
        self.endstop = endstop
        self.stepper_positions = [
            FakeStepperPosition(s, start_pos, trig_pos)
            for s in endstop.get_steppers()]
    def get_mcu_endstops(self):
        return [self.endstop]

def run_tap(tap, trig_pos=-500, trigger_time=None, rest_time=None,
            start_pos=0):
    # What homing.HomingMove.homing_move() does with an endstop
    hmove = FakeHomingMove(tap.endstop, start_pos, trig_pos)
    printer, toolhead = tap.printer, tap.toolhead
    printer.send_event("homing:homing_move_begin", hmove)
    print_time = toolhead.get_last_move_time()
    if rest_time is None:
        rest_time = .01 / 5.   # 5 mm/s on a 0.01 mm step
    tap.endstop.home_start(print_time, 0., 0, rest_time)
    toolhead.dwell(1.)
    if trigger_time is None:
        trigger_time = print_time + .5
    tap.chip.mcu.trigger_clock = tap.chip.mcu.print_time_to_clock(
        trigger_time)
    error = None
    try:
        tap.endstop.home_wait(toolhead.get_last_move_time())
    except CommandError as e:
        error = e
    try:
        printer.send_event("homing:homing_move_end", hmove)
    except CommandError as e:
        error = error or e
    if error is not None:
        raise error
    return print_time

class TestArming(unittest.TestCase):
    def test_the_detector_arms_after_the_move_reaches_speed(self):
        tap = build()
        print_time = run_tap(tap)
        mcu = tap.chip.mcu
        self.assertEqual(tap.mcu_trigger.get_dispatch().started, [print_time])
        # The second trigger_analog_home is home_wait() disarming it
        (_, args, reqclock), disarm = mcu.sent_named('trigger_analog_home')
        self.assertEqual(disarm[1][5], 0)
        # 5 mm/s at min(3000, 50) mm/s^2 is 0.1 s of acceleration
        expected = print_time + .001 + .1 + .05
        self.assertEqual(args[4], mcu.print_time_to_clock(expected))
        self.assertEqual(reqclock, args[4])
        self.assertAlmostEqual(tap.last_capture.arm_time, expected)
        # The monitor tick is sent in clock ticks
        self.assertEqual(args[5], mcu.seconds_to_clock(.010))
    def test_blind_distance(self):
        # 5 mm/s: 0.25 mm accelerating at 50 mm/s^2, then 5 * 0.051 mm
        self.assertAlmostEqual(azt.blind_distance(5., 50., .05),
                               .25 + 5. * .051)
        self.assertAlmostEqual(build().get_blind_distance(5.), .505)
    def test_the_result_is_recorded(self):
        tap = build()
        print_time = run_tap(tap, trig_pos=-300)
        capture = tap.last_capture
        self.assertAlmostEqual(capture.travel, 3.)
        self.assertAlmostEqual(capture.trigger_time, print_time + .5)
        self.assertTrue(capture.client.is_finished)
        self.assertIsNone(tap.capture)
        self.assertIn(tap.batch_margin, tap.toolhead.dwells)
    def test_a_trigger_inside_min_trigger_travel_is_refused(self):
        tap = build()
        with self.assertRaises(CommandError) as cm:
            run_tap(tap, trig_pos=-50)
        self.assertIn("inside min_trigger_travel", str(cm.exception))
    def test_a_move_that_is_too_fast_is_refused_before_it_starts(self):
        tap = build()
        with self.assertRaises(CommandError) as cm:
            run_tap(tap, rest_time=.01 / 20.)
        self.assertIn("faster than max_speed", str(cm.exception))
        self.assertEqual(tap.mcu_trigger.get_dispatch().started, [])
        self.assertTrue(tap.chip.clients[-1].is_finished)
    def test_lost_frames_refuse_the_tap(self):
        tap = build()
        tap.chip.overflows = 3
        with self.assertRaises(CommandError) as cm:
            run_tap(tap)
        self.assertIn("3 possible fifo overflows", str(cm.exception))
    def test_a_quiet_sensor_is_reported_by_name(self):
        tap = build()
        dispatch = tap.mcu_trigger.get_dispatch()
        # REASON_TRIGGER_ANALOG + MONITOR
        dispatch.stop_reason = FakeTrsync.REASON_COMMS_TIMEOUT + 1 + 2
        with self.assertRaises(CommandError) as cm:
            run_tap(tap)
        self.assertIn("MONITOR", str(cm.exception))
        self.assertTrue(tap.last_capture.client.is_finished)
    def test_other_endstops_are_left_alone(self):
        tap = build()
        other = FakeHomingMove(tap.endstop)
        other.get_mcu_endstops = lambda: [object()]
        tap.printer.send_event("homing:homing_move_begin", other)
        self.assertIsNone(tap.capture)

class TestMachineState(unittest.TestCase):
    def test_b_must_be_homed(self):
        tap = build()
        tap.toolhead.b_axis.homed = False
        with self.assertRaises(CommandError) as cm:
            run_tap(tap)
        self.assertIn("G28 B", str(cm.exception))
        self.assertEqual(tap.chip.clients, [])
    def test_b_must_be_at_zero(self):
        tap = build()
        tap.toolhead.position[azt.B_POS_INDEX] = 1.5
        with self.assertRaises(CommandError) as cm:
            run_tap(tap)
        self.assertIn("within 1.00 deg of B=0", str(cm.exception))
        tap.toolhead.position[azt.B_POS_INDEX] = .5
        run_tap(tap)
    def test_accel_b_homing_answers_for_b_when_present(self):
        class BHoming:
            def is_b_homed(self):
                return True
            def get_commanded_b(self):
                return -4.
            def measure_vertical(self):
                return 3.
        tap = build({'measure_b': True})
        tap.printer.add_object('accel_b_homing', BHoming())
        with self.assertRaises(CommandError) as cm:
            tap.check_b()
        self.assertIn("-4.00 deg", str(cm.exception))
        with self.assertRaises(CommandError) as cm:
            tap.measure_b_now()
        self.assertIn("measures 3.00 deg", str(cm.exception))
    def test_a_machine_without_b_is_not_checked(self):
        tap = build()
        tap.toolhead.extra_axes = [None]
        tap.toolhead.position[azt.B_POS_INDEX] = 45.
        run_tap(tap)
    def test_a_hot_nozzle_is_refused(self):
        tap = build()
        heater = FakeHeater(temperature=40., target=200.)
        tap.printer.add_object('extruder', FakeExtruder(heater))
        with self.assertRaises(CommandError) as cm:
            run_tap(tap)
        self.assertIn("refused above 150C", str(cm.exception))
        heater.target = 0.
        run_tap(tap)
        tap.max_extruder_temp = 0.
        heater.temperature = 240.
        run_tap(tap)


######################################################################
# G28 Z: two taps that must agree
######################################################################

class FakeHomingInfo:
    def __init__(self, speed=5., second_speed=2., retract_dist=3.):
        self.speed = speed
        self.second_homing_speed = second_speed
        self.retract_dist = retract_dist

class FakeRail:
    def __init__(self, endstop, info):
        self.endstop, self.info = endstop, info
    def get_endstops(self):
        return [(self.endstop, 'stepper_z')]
    def get_homing_info(self):
        return self.info

def home_z(tap, trig_positions, info=None, values=None):
    rails = [FakeRail(tap.endstop, info or FakeHomingInfo())]
    tap.printer.send_event("homing:home_rails_begin", None, rails)
    for trig in trig_positions:
        run_tap(tap, trig_pos=trig, start_pos=trig + 300)
    tap.printer.send_event("homing:home_rails_end", None, rails)

class TestHomeRails(unittest.TestCase):
    def test_agreeing_taps_home(self):
        tap = build()
        # samples_tolerance defaults to 0.1 mm; 5 steps is 0.05 mm
        home_z(tap, [-1000, -1005])
        self.assertIsNone(tap.rail_taps)
    def test_disagreeing_taps_are_refused(self):
        tap = build()
        with self.assertRaises(CommandError) as cm:
            home_z(tap, [-1000, -1200])
        self.assertIn("disagree by 2.000 mm", str(cm.exception))
    def test_the_tolerance_is_configurable(self):
        tap = build({'samples_tolerance': .01})
        with self.assertRaises(CommandError):
            home_z(tap, [-1000, -1005])
    def test_a_retract_inside_the_blind_distance_is_refused(self):
        tap = build()
        with self.assertRaises(CommandError) as cm:
            home_z(tap, [], FakeHomingInfo(retract_dist=.4, second_speed=5.))
        self.assertIn("homing_retract_dist of 0.400 mm", str(cm.exception))
    def test_a_fast_homing_speed_is_refused(self):
        tap = build()
        with self.assertRaises(CommandError) as cm:
            home_z(tap, [], FakeHomingInfo(speed=25.))
        self.assertIn("homing_speed", str(cm.exception))
    def test_a_command_error_clears_the_rail_state(self):
        tap = build()
        rails = [FakeRail(tap.endstop, FakeHomingInfo())]
        tap.printer.send_event("homing:home_rails_begin", None, rails)
        tap.capture = tap._start_capture()
        tap.printer.send_event("gcode:command_error")
        self.assertIsNone(tap.rail_taps)
        self.assertIsNone(tap.capture)


######################################################################
# ACCEL_TAP_PROBE
######################################################################

class FakePhoming:
    def __init__(self, tap, contacts):
        self.tap = tap
        self.contacts = list(contacts)
        self.calls = []
    def probing_move(self, endstop, pos, speed):
        self.calls.append((list(pos), speed))
        z = self.contacts.pop(0)
        self.tap.toolhead.position[2] = z
        return [pos[0], pos[1], z]

class TestProbe(unittest.TestCase):
    def _build(self, contacts, values=None):
        tap = build(values)
        phoming = FakePhoming(tap, contacts)
        tap.printer.add_object('homing', phoming)
        return tap, phoming
    def test_samples_are_combined(self):
        tap, phoming = self._build([.02, .01, .03],
                                   {'samples': 3, 'samples_result': 'median'})
        gcmd = FakeGCodeCommand()
        tap.cmd_ACCEL_TAP_PROBE(gcmd)
        self.assertAlmostEqual(tap.last_z_result, .02)
        self.assertIn("z=0.0200 (3 taps", gcmd.responses[-1])
        # Never below -max_overtravel, even with position_min further down
        self.assertEqual([c[0][2] for c in phoming.calls], [-2.] * 3)
        # The nozzle is lifted off the bed after every tap
        self.assertEqual(len(tap.toolhead.moves), 3)
    def test_disagreeing_samples_retry_then_fail(self):
        tap, phoming = self._build([0., .5, 0., .5],
                                   {'samples': 2,
                                    'samples_tolerance_retries': 1})
        gcmd = FakeGCodeCommand()
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_PROBE(gcmd)
        self.assertIn("exceed samples_tolerance", str(cm.exception))
        self.assertIn("Retrying", gcmd.responses[0])
    def test_z_must_be_homed(self):
        tap, phoming = self._build([0.])
        tap.toolhead.homed_axes = 'xy'
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_PROBE(FakeGCodeCommand())
        self.assertIn("home Z", str(cm.exception))
        self.assertEqual(phoming.calls, [])
    def test_rtcp_must_be_off(self):
        class Rtcp:
            def check_disabled(self, what):
                raise CommandError("%s must run with RTCP compensation off"
                                   % (what,))
        tap, phoming = self._build([0.])
        tap.printer.add_object('rtcp', Rtcp())
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_PROBE(FakeGCodeCommand())
        self.assertIn("ACCEL_TAP_PROBE must run", str(cm.exception))
    def test_a_short_retract_is_refused(self):
        tap, phoming = self._build([0.])
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_PROBE(FakeGCodeCommand(
                {'SAMPLE_RETRACT_DIST': '.5'}))
        self.assertIn("SAMPLE_RETRACT_DIST", str(cm.exception))
        self.assertEqual(phoming.calls, [])


######################################################################
# ACCEL_TAP_QUERY: the host model over a captured background
######################################################################

class TestQuery(unittest.TestCase):
    def test_the_still_background_is_reported(self):
        tap = build()
        start = tap.toolhead.get_last_move_time()
        # 1 g of gravity with a small alternating ripple on top
        tap.chip.samples = [
            (start + i / 1600., 0., 0., 0., 0., 0.,
             1. + (.002 if i % 2 else -.002)) for i in range(1600)]
        gcmd = FakeGCodeCommand({'TIME': '1.'})
        tap.cmd_ACCEL_TAP_QUERY(gcmd)
        msg = gcmd.responses[-1]
        self.assertIn("1600 samples", msg)
        # Gravity is gone; the 800 Hz ripple passes the 50 Hz high pass
        self.assertLess(tap.last_peak, .01)
        self.assertGreater(tap.last_peak, .002)
        self.assertIn("blind for the first 0.505 mm", msg)


######################################################################
# ACCEL_TAP_TEST and ACCEL_TAP_CALIBRATE, end to end
######################################################################

# A still head: gravity, with a small 800 Hz ripple the high pass passes
# as the background, plus a damped ring at every contact time
class HeadSignal:
    def __init__(self, ripple=.002, contact_g=.6):
        self.ripple, self.contact_g = ripple, contact_g
        self.contacts = []
    def __call__(self, t):
        g = 1. + (self.ripple if int(round(t * 1600.)) % 2 else -self.ripple)
        for c in self.contacts:
            if t >= c:
                dt = t - c
                g += (self.contact_g * math.exp(-dt / .004)
                      * math.sin(2. * math.pi * 350. * dt))
        return g

# Runs a probing move through the real homing events, with the contact
# landing shortly after the detector arms
class TappingPhoming:
    def __init__(self, tap, signal):
        self.tap, self.signal = tap, signal
        self.moves = 0
    def probing_move(self, endstop, pos, speed):
        tap, toolhead = self.tap, self.tap.toolhead
        hmove = FakeHomingMove(endstop, 0, -500)
        tap.printer.send_event("homing:homing_move_begin", hmove)
        print_time = toolhead.get_last_move_time()
        step_dist = endstop.get_steppers()[0].get_step_dist()
        endstop.home_start(print_time, 0., 0, step_dist / speed)
        contact = tap.capture.arm_time + .05
        self.signal.contacts.append(contact)
        tap.chip.mcu.trigger_clock = tap.chip.mcu.print_time_to_clock(
            contact + 1. / 1600.)
        toolhead.dwell(1.)
        endstop.home_wait(toolhead.get_last_move_time())
        tap.printer.send_event("homing:homing_move_end", hmove)
        self.moves += 1
        toolhead.position[2] = 0.
        return [pos[0], pos[1], 0.]

class TestCommands(unittest.TestCase):
    def test_a_hand_tap_is_detected_without_moving(self):
        tap = build()
        signal = HeadSignal()
        tap.chip.signal = signal
        print_time = tap.toolhead.get_last_move_time()
        signal.contacts.append(print_time + 2.)
        tap.chip.mcu.trigger_clock = tap.chip.mcu.print_time_to_clock(
            print_time + 2.001)
        gcmd = FakeGCodeCommand({'TAPS': '1'})
        tap.cmd_ACCEL_TAP_TEST(gcmd)
        self.assertIn("Tap 1 detected at print time 102.0010",
                      gcmd.responses[1])
        self.assertGreater(tap.last_peak, .3)
        self.assertEqual(tap.mcu_trigger.get_dispatch().started, [print_time])
        self.assertEqual(tap.toolhead.moves, [])
    def test_no_tap_is_an_error(self):
        tap = build()
        tap.chip.signal = HeadSignal()
        dispatch = tap.mcu_trigger.get_dispatch()
        dispatch.stop_reason = FakeTrsync.REASON_PAST_END_TIME
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_TEST(FakeGCodeCommand({'TAPS': '1',
                                                    'TIMEOUT': '5'}))
        self.assertIn("no tap detected within 5 s", str(cm.exception))
        self.assertIsNone(tap.capture)
    def test_calibrate_compares_contacts_with_the_moving_background(self):
        tap = build()
        signal = HeadSignal()
        tap.chip.signal = signal
        phoming = TappingPhoming(tap, signal)
        tap.printer.add_object('homing', phoming)
        gcmd = FakeGCodeCommand({'TAPS': '2'})
        tap.cmd_ACCEL_TAP_CALIBRATE(gcmd)
        self.assertEqual(phoming.moves, 2)
        msg = gcmd.responses[-1]
        self.assertNotIn("WARNING", msg)
        saved = float(tap.printer.lookup_object('configfile').saved[
            ('accel_z_tap', 'trigger_threshold')])
        # Between the background ripple and the contact ring
        self.assertGreater(saved, .004)
        self.assertLess(saved, .3)
        # Back where it started
        self.assertEqual(tap.toolhead.position[2], 10.)
    def test_calibrate_warns_when_contacts_do_not_stand_out(self):
        tap = build()
        signal = HeadSignal(ripple=.1, contact_g=.2)
        tap.chip.signal = signal
        tap.printer.add_object('homing', TappingPhoming(tap, signal))
        gcmd = FakeGCodeCommand({'TAPS': '1'})
        tap.cmd_ACCEL_TAP_CALIBRATE(gcmd)
        self.assertIn("WARNING", gcmd.responses[-1])
    def test_calibrate_needs_room_for_the_background_descent(self):
        tap = build()
        tap.toolhead.position[2] = 4.
        with self.assertRaises(CommandError) as cm:
            tap.cmd_ACCEL_TAP_CALIBRATE(FakeGCodeCommand())
        self.assertIn("raise the nozzle first", str(cm.exception))
        self.assertEqual(tap.toolhead.moves, [])


if __name__ == '__main__':
    unittest.main()
