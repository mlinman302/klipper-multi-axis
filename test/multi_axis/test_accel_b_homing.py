#!/usr/bin/env python
# Host test of the gravity-referenced B measurement
# (klippy/extras/accel_b_homing.py).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* extras/accel_b_homing.py with a stubbed printer
# and a synthetic accelerometer, so it runs anywhere Python is available
# (it does not need a compiled c_helper.so, a serial port, an
# accelerometer, or a Linux host).
#
# Run with:  python test/multi_axis/test_accel_b_homing.py
import math, os, random, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# stepper.py imports mcu, which needs pyserial; nothing under test uses it
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

from extras import accel_b_homing as abh

G = abh.FREEFALL_ACCEL


######################################################################
# Stubbed printer environment
######################################################################

class ConfigError(Exception):
    pass

class FakeRotaryAxis:
    def __init__(self, gcode_id='B', is_homed=True):
        self.gcode_id, self.is_homed = gcode_id, is_homed
    def get_axis_gcode_id(self):
        return self.gcode_id
    def get_status(self, eventtime=None):
        return {'position': 0., 'homed': self.is_homed}

class FakeExtruder:
    # Sits at index 0 of get_extra_axes() and has no gcode axis id
    def get_name(self):
        return 'extruder'

class FakeToolhead:
    def __init__(self):
        self.position = [0.] * 7
        self.print_time = 100.
        self.dwells = []
        self.b_axis = FakeRotaryAxis()
        self.extra_axes = [FakeExtruder(), None, self.b_axis]
    def get_extra_axes(self):
        return list(self.extra_axes)
    def get_position(self):
        return list(self.position)
    def wait_moves(self):
        pass
    def get_last_move_time(self):
        return self.print_time
    def dwell(self, delay):
        self.dwells.append(delay)
        self.print_time += delay

# An accelerometer that reports a head sitting at a known angle.  The
# sensor frame here is the one the tests configure: +z reads 1 g at B=0,
# +x reads 1 g at B=+90, so y is the rotation axis.
class FakeAccelChip:
    data_rate = 3200
    def __init__(self, angle=0., noise=0., gain=(1., 1., 1.),
                 offset=(0., 0., 0.), y_bias=0., delivery_lag=0.,
                 overflows=0):
        self.angle, self.noise = angle, noise
        # Possible fifo overflows the chip reports during a capture, as
        # the running count carried in each bulk batch
        self.overflows = overflows
        self.gain, self.offset, self.y_bias = gain, offset, y_bias
        self.toolhead = None
        self.dropped = 0.
        # Samples this far back from the end of the dwell have not been
        # delivered yet - the batching and link latency of a chip on a
        # secondary mcu, such as a USB accelerometer board
        self.delivery_lag = delivery_lag
        self.rng = random.Random(1234)
    def start_internal_client(self):
        return FakeAccelClient(self)
    def angle_at(self, t, start):
        # A head that is not turning.  FakeIMUChip overrides this.
        return self.angle
    def samples_over(self, start, end):
        end -= self.delivery_lag
        res = []
        total = int(round((end - start) * self.data_rate))
        # Losses are spread evenly across the span rather than truncating
        # it, so that a lossy link still delivers samples inside the
        # measurement window
        keep = int(round((1. - self.dropped) * 100.))
        for i in range(total):
            if i % 100 >= keep:
                continue
            t = start + (i + .5) / self.data_rate
            rad = math.radians(self.angle_at(t, start))
            base = (G * math.sin(rad), self.y_bias, G * math.cos(rad))
            vec = []
            for axis in range(3):
                v = base[axis] * self.gain[axis] + self.offset[axis]
                if self.noise:
                    v += self.rng.gauss(0., self.noise)
                vec.append(v)
            res.append((t, vec[0], vec[1], vec[2]))
        return res

class FakeAccelClient:
    def __init__(self, chip):
        self.chip = chip
        self.start = chip.toolhead.get_last_move_time()
        self.end = None
    def finish_measurements(self):
        self.end = self.chip.toolhead.get_last_move_time()
    @property
    def msgs(self):
        return [{'overflows': 0}, {'overflows': self.chip.overflows}]
    def get_samples(self):
        return self.chip.samples_over(self.start, self.end)

# A BMI160-shaped chip, and a physically self-consistent one: if b_rate
# is non-zero the head really is turning, so the accelerometer angle
# sweeps at that rate *and* the gyroscope reports the rotation that
# produces it.  Fusing an accelerometer with a gyroscope that disagrees
# with it would prove nothing.
#
# These tests mount the chip as zero=+z, positive=+x, so the B rotation
# axis is w_hat x u_hat = z_hat x x_hat = +y, and the sensor's own rate
# about that axis is -dB/dt (see accel_b_homing.gyro_axis_coefficient).
# That minus sign is the whole convention under test: if the module got
# it backwards, the fused angle would run away from the accelerometer
# instead of tracking it.
#
# off_axis_rate spins the chip about its zero_vector axis instead.  That
# is real motion the gate's magnitude sees, but it does not tilt the
# head and so must not move the measured angle.
class FakeIMUChip(FakeAccelChip):
    def __init__(self, b_rate=0., off_axis_rate=0., invert_gyro=False,
                 gyro_lag=0., **kwargs):
        FakeAccelChip.__init__(self, **kwargs)
        self.b_rate = b_rate
        self.off_axis_rate = off_axis_rate
        self.invert_gyro = invert_gyro
        self.gyro_lag = gyro_lag
    def has_gyro(self):
        return True
    def angle_at(self, t, start):
        return self.angle + self.b_rate * (t - start)
    def gyro_vector(self):
        rate = [0., -self.b_rate, self.off_axis_rate]
        if self.invert_gyro:
            rate = [-r for r in rate]
        return tuple(rate)
    def start_internal_gyro_client(self):
        return FakeGyroClient(self)
    def start_internal_imu_client(self):
        return FakeIMUClient(self)
    def gyro_samples_over(self, start, end):
        end -= self.delivery_lag + self.gyro_lag
        total = int(round((end - start) * self.data_rate))
        return [(start + (i + .5) / self.data_rate,) + self.gyro_vector()
                for i in range(max(0, total))]
    def imu_samples_over(self, start, end):
        gyro = self.gyro_vector()
        return [(t,) + gyro + (ax, ay, az)
                for t, ax, ay, az in self.samples_over(start, end)]

class FakeGyroClient(FakeAccelClient):
    def get_samples(self):
        return self.chip.gyro_samples_over(self.start, self.end)

class FakeIMUClient(FakeAccelClient):
    def get_samples(self):
        return self.chip.imu_samples_over(self.start, self.end)

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func

class FakePrinter:
    config_error = ConfigError
    command_error = ConfigError
    def __init__(self):
        self.objects = {'gcode': FakeGCode()}
        self.event_handlers = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=Ellipsis):
        if name in self.objects:
            return self.objects[name]
        if default is not Ellipsis:
            return default
        raise ConfigError("Unknown config object '%s'" % (name,))
    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)
    def send_event(self, event):
        for cb in self.event_handlers.get(event, []):
            cb()

class FakeConfig:
    error = ConfigError
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'accel_b_homing'
    def get(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return self.values[option]
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return default
    def getfloat(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return float(self.values[option])
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return None if default is None else float(default)
    def getboolean(self, option, default=Ellipsis, **kwargs):
        if option in self.values:
            return bool(self.values[option])
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return default

class FakeGCmd:
    error = ConfigError
    def __init__(self, params=None):
        self.params = params or {}
        self.responses = []
    def get_float(self, name, default=Ellipsis, **kwargs):
        if name in self.params:
            return float(self.params[name])
        if default is Ellipsis:
            raise ConfigError("Missing %s" % (name,))
        return default
    def get_int(self, name, default=Ellipsis, **kwargs):
        if name in self.params:
            return int(self.params[name])
        if default is Ellipsis:
            raise ConfigError("Missing %s" % (name,))
        return default
    def respond_info(self, msg):
        self.responses.append(msg)


# The reference mounting used by most tests below
BASE_CONFIG = {'zero_vector': '+z', 'positive_vector': '+x'}

def build(config_values=None, chip=None, b_projection=None):
    printer = FakePrinter()
    toolhead = FakeToolhead()
    printer.add_object('toolhead', toolhead)
    chip = chip if chip is not None else FakeAccelChip()
    chip.toolhead = toolhead
    printer.add_object('bmi160', chip)
    if b_projection is not None:
        printer.add_object('b_projection', b_projection)
    values = dict(BASE_CONFIG)
    values.update(config_values or {})
    obj = abh.AccelBHoming(FakeConfig(printer, values))
    printer.send_event("klippy:connect")
    return obj


######################################################################
# The pure helpers
######################################################################

class TestHelpers(unittest.TestCase):
    def test_signed_axis_round_trip(self):
        for name in ('+x', '-x', '+y', '-y', '+z', '-z'):
            axis = abh.parse_signed_axis(name)
            self.assertEqual(abh.signed_axis_name(axis), name)
    def test_unsigned_axis_means_positive(self):
        self.assertEqual(abh.parse_signed_axis('Z '), (2, 1.))
    def test_bad_axis_rejected(self):
        for name in ('w', '+w', 'xy', '', '2z'):
            with self.assertRaises(ValueError):
                abh.parse_signed_axis(name)
    def test_out_of_plane_index(self):
        z, x = abh.parse_signed_axis('+z'), abh.parse_signed_axis('-x')
        self.assertEqual(abh.out_of_plane_index(z, x), 1)
        with self.assertRaises(ValueError):
            abh.out_of_plane_index(z, abh.parse_signed_axis('-z'))
    def test_angle_at_the_two_reference_poses(self):
        z, x = abh.parse_signed_axis('+z'), abh.parse_signed_axis('+x')
        self.assertAlmostEqual(abh.measure_angle((0., 0., G), z, x), 0.)
        self.assertAlmostEqual(abh.measure_angle((G, 0., 0.), z, x), 90.)
        self.assertAlmostEqual(abh.measure_angle((-G, 0., 0.), z, x), -90.)
        self.assertAlmostEqual(abh.measure_angle((0., 0., -G), z, x), 180.)
    def test_angle_recovers_an_arbitrary_pose(self):
        z, x = abh.parse_signed_axis('+z'), abh.parse_signed_axis('+x')
        for deg in (-89.5, -45., -0.25, 12.5, 60., 144.):
            rad = math.radians(deg)
            vec = (G * math.sin(rad), 0., G * math.cos(rad))
            self.assertAlmostEqual(abh.measure_angle(vec, z, x), deg, places=9)
    def test_a_negated_mounting_negates_the_angle(self):
        z = abh.parse_signed_axis('+z')
        plus = abh.parse_signed_axis('+x')
        minus = abh.parse_signed_axis('-x')
        rad = math.radians(30.)
        vec = (G * math.sin(rad), 0., G * math.cos(rad))
        self.assertAlmostEqual(abh.measure_angle(vec, z, plus), 30.)
        self.assertAlmostEqual(abh.measure_angle(vec, z, minus), -30.)
    def test_summarize(self):
        means, devs = abh.summarize([(1., 10., 0.), (3., 10., 0.)])
        self.assertEqual(means, (2., 10., 0.))
        self.assertAlmostEqual(devs[0], math.sqrt(2.))
        self.assertEqual(devs[1], 0.)
        means, devs = abh.summarize([(1., 2., 3.)])
        self.assertEqual((means, devs), ((1., 2., 3.), (0., 0., 0.)))
        with self.assertRaises(ValueError):
            abh.summarize([])


######################################################################
# Configuration
######################################################################

class TestConfig(unittest.TestCase):
    def test_rotation_axis_is_derived(self):
        obj = build()
        status = obj.get_status()
        self.assertEqual(status['zero_vector'], '+z')
        self.assertEqual(status['positive_vector'], '+x')
        self.assertEqual(status['rotation_axis'], 'y')
        self.assertIsNone(status['measured_b'])
    def test_shared_axis_is_a_config_error(self):
        with self.assertRaises(ConfigError) as cm:
            build({'positive_vector': '-z'})
        self.assertIn("must name different", str(cm.exception))
    def test_invalid_axis_names_the_valid_ones(self):
        with self.assertRaises(ConfigError) as cm:
            build({'zero_vector': 'up'})
        self.assertIn("+x, -x, +y, -y, +z or -z", str(cm.exception))
    def test_missing_chip_is_reported(self):
        with self.assertRaises(ConfigError) as cm:
            build({'accel_chip': 'adxl345 head'})
        self.assertIn("is not configured", str(cm.exception))
    def test_a_non_accelerometer_is_reported(self):
        printer = FakePrinter()
        toolhead = FakeToolhead()
        printer.add_object('toolhead', toolhead)
        printer.add_object('bmi160', object())
        abh.AccelBHoming(FakeConfig(printer, dict(BASE_CONFIG)))
        with self.assertRaises(ConfigError) as cm:
            printer.send_event("klippy:connect")
        self.assertIn("is not an accelerometer", str(cm.exception))
    def test_a_machine_without_b_is_reported(self):
        printer = FakePrinter()
        toolhead = FakeToolhead()
        toolhead.extra_axes = [FakeExtruder(), None, None]
        printer.add_object('toolhead', toolhead)
        chip = FakeAccelChip()
        chip.toolhead = toolhead
        printer.add_object('bmi160', chip)
        abh.AccelBHoming(FakeConfig(printer, dict(BASE_CONFIG)))
        with self.assertRaises(ConfigError) as cm:
            printer.send_event("klippy:connect")
        self.assertIn("additional_axes", str(cm.exception))
    def test_the_command_is_registered(self):
        obj = build()
        gcode = obj.printer.lookup_object('gcode')
        self.assertIn('B_MEASURE', gcode.commands)


######################################################################
# The measurement primitive
######################################################################

class TestMeasure(unittest.TestCase):
    def test_a_clean_reading(self):
        obj = build(chip=FakeAccelChip(angle=-32.5))
        reading = obj.measure()
        self.assertAlmostEqual(reading.angle, -32.5, places=9)
        self.assertAlmostEqual(reading.magnitude, G, places=6)
        self.assertEqual(reading.count, int(3200 * .5))
        self.assertAlmostEqual(math.hypot(reading.u, reading.w), G, places=6)
        self.assertAlmostEqual(reading.out_of_plane, 0.)
    def test_the_settle_dwell_is_excluded_from_the_average(self):
        obj = build({'settle_time': .25, 'sample_time': .5})
        toolhead = obj.printer.lookup_object('toolhead')
        obj.measure()
        # One dwell covering settle + window + batch margin, but only the
        # window is averaged
        self.assertEqual(toolhead.dwells, [.25 + .5 + .3])
        self.assertEqual(obj.last_reading.count, int(3200 * .5))
    def test_overridden_sample_time_changes_the_count(self):
        obj = build()
        self.assertEqual(obj.measure(0., .1).count, 320)
    def test_noise_averages_out(self):
        # 500 mm/s^2 per sample is well above a real ADXL345 (120-180 is
        # typical at 3200 Hz).  Averaged over 1600 samples that is
        # 12.5 mm/s^2 on the mean, or about 0.07 degrees - so a tenth of
        # a degree is a safe bound, and this is the whole reason the
        # routine averages rather than taking one sample.
        chip = FakeAccelChip(angle=20., noise=500.)
        obj = build({'max_sample_deviation': 0.}, chip=chip)
        self.assertAlmostEqual(obj.measure().angle, 20., delta=.1)
    def test_a_moving_head_is_rejected(self):
        chip = FakeAccelChip(angle=0., noise=900.)
        obj = build(chip=chip)
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("still moving", str(cm.exception))
        self.assertIn("settle_time", str(cm.exception))
    def test_a_bad_magnitude_is_rejected(self):
        chip = FakeAccelChip(angle=0., gain=(1., 1., .5))
        obj = build(chip=chip)
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("where gravity is", str(cm.exception))
    def test_a_chip_inside_its_gain_spec_is_accepted(self):
        # +/-10 % sensitivity is within the ADXL345 datasheet spec and
        # must not trip the magnitude gate - there is no gain
        # calibration in this phase
        for gain in (.9, 1.1):
            chip = FakeAccelChip(angle=0., gain=(gain, gain, gain))
            build(chip=chip).measure()
    def test_silence_is_reported_as_a_wiring_problem(self):
        chip = FakeAccelChip()
        chip.dropped = 1.
        obj = build(chip=chip)
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("ACCELEROMETER_QUERY", str(cm.exception))
    def test_heavy_data_loss_is_reported(self):
        chip = FakeAccelChip()
        chip.dropped = .8
        obj = build(chip=chip)
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("dropping data", str(cm.exception))
    def test_late_batches_still_cover_the_window(self):
        # A chip on a USB or CAN attached mcu delivers its last batch
        # after the moves have finished.  The trailing batch_margin dwell
        # is what keeps the averaging window fully covered.
        obj = build(chip=FakeAccelChip(angle=8., delivery_lag=.2))
        self.assertEqual(obj.measure().count, int(3200 * .5))
    def test_without_the_margin_the_tail_of_the_window_is_lost(self):
        obj = build({'batch_margin': 0.},
                    chip=FakeAccelChip(angle=8., delivery_lag=.2))
        # 0.2 s of the 0.5 s window never arrives
        self.assertEqual(obj.measure().count, int(3200 * .3))
    def test_a_short_window_without_the_margin_trips_the_gate(self):
        # The failure the margin exists to prevent: on a laggy link a
        # short window loses enough of itself to look like data loss
        chip = FakeAccelChip(angle=8., delivery_lag=.15)
        obj = build({'sample_time': .2, 'batch_margin': 0.}, chip=chip)
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("batch_margin", str(cm.exception))
        # ...and with the margin restored the same window is complete
        obj = build({'sample_time': .2}, chip=chip)
        self.assertEqual(obj.measure().count, int(3200 * .2))
    def test_the_out_of_plane_axis_is_reported(self):
        chip = FakeAccelChip(angle=0., y_bias=250.)
        obj = build(chip=chip)
        self.assertAlmostEqual(obj.measure().out_of_plane, 250.)
    def test_status_carries_the_last_reading(self):
        obj = build(chip=FakeAccelChip(angle=15.))
        obj.measure()
        self.assertAlmostEqual(obj.get_status()['measured_b'], 15., places=9)


######################################################################
# B_MEASURE
######################################################################

class FakeBProjection:
    # [b_projection] scales a commanded (bed frame) B onto the plane the
    # head can tilt in.  Only project_pos() is used here.
    def __init__(self, scale=1.):
        self.scale = scale
    def project_pos(self, pos):
        return pos[abh.B_POS_INDEX] * self.scale

class TestCommand(unittest.TestCase):
    def _run(self, obj, params=None):
        gcmd = FakeGCmd(params)
        obj.printer.lookup_object('gcode').commands['B_MEASURE'](gcmd)
        return "\n".join(gcmd.responses)
    def test_report_contents(self):
        obj = build(chip=FakeAccelChip(angle=-12.))
        toolhead = obj.printer.lookup_object('toolhead')
        toolhead.position[abh.B_POS_INDEX] = -12.5
        out = self._run(obj)
        self.assertIn("B = -12.000 deg", out)
        self.assertIn("out of plane (y)", out)
        self.assertIn("commanded B = -12.500 deg", out)
        self.assertIn("error = +0.500 deg", out)
    def test_unhomed_b_reports_no_comparison(self):
        obj = build()
        obj.printer.lookup_object('toolhead').b_axis.is_homed = False
        out = self._run(obj)
        self.assertIn("B is not homed", out)
        self.assertNotIn("commanded B", out)
    def test_check_on_an_unhomed_b_is_an_error(self):
        obj = build(chip=FakeAccelChip(angle=7.))
        obj.printer.lookup_object('toolhead').b_axis.is_homed = False
        gcmd = FakeGCmd({'CHECK': 1})
        with self.assertRaises(ConfigError) as cm:
            obj.printer.lookup_object('gcode').commands['B_MEASURE'](gcmd)
        self.assertIn("while B is unhomed", str(cm.exception))
        # The measurement is still reported - it is the useful part of
        # the answer even when there is nothing to compare it against
        self.assertIn("B = 7.000 deg", "\n".join(gcmd.responses))
    def test_check_passes_within_tolerance(self):
        obj = build(chip=FakeAccelChip(angle=30.4))
        obj.printer.lookup_object('toolhead').position[abh.B_POS_INDEX] = 30.
        self._run(obj, {'CHECK': 1, 'TOLERANCE': .5})
    def test_check_fails_outside_tolerance(self):
        obj = build(chip=FakeAccelChip(angle=34.))
        obj.printer.lookup_object('toolhead').position[abh.B_POS_INDEX] = 30.
        with self.assertRaises(ConfigError) as cm:
            self._run(obj, {'CHECK': 1, 'TOLERANCE': .5})
        self.assertIn("commanded to 30.000", str(cm.exception))
    def test_the_comparison_follows_b_projection(self):
        # With the projection on, the toolhead's B is a bed-frame angle
        # and the head is really at its projection - which is what the
        # sensor sees.  Comparing against the raw commanded B here would
        # report a 30 degree error that does not exist.
        obj = build(chip=FakeAccelChip(angle=30.),
                    b_projection=FakeBProjection(scale=.5))
        obj.printer.lookup_object('toolhead').position[abh.B_POS_INDEX] = 60.
        out = self._run(obj, {'CHECK': 1, 'TOLERANCE': .5})
        self.assertIn("commanded B = 30.000 deg", out)


######################################################################
# The gyroscope motion gate
######################################################################

class TestOverflows(unittest.TestCase):
    def test_overflows_during_reads_the_increase(self):
        class Client:
            msgs = [{'overflows': 3}, {'overflows': 3}, {'overflows': 5}]
        self.assertEqual(abh.overflows_during(Client()), 2)
        Client.msgs = [{'overflows': 4}]
        self.assertEqual(abh.overflows_during(Client()), 0)
        self.assertEqual(abh.overflows_during(object()), 0)
    def test_a_measurement_that_lost_frames_is_refused(self):
        for chip in (FakeAccelChip(overflows=2),
                     FakeIMUChip(overflows=2)):
            obj = build(chip=chip)
            with self.assertRaises(ConfigError) as cm:
                obj.measure()
            self.assertIn("2 possible fifo overflows", str(cm.exception))
            self.assertIn("i2c_arm_baudrate", str(cm.exception))
    def test_the_unfused_gyroscope_stream_is_checked_too(self):
        obj = build({'fusion': False}, chip=FakeIMUChip(overflows=1))
        with self.assertRaises(ConfigError):
            obj.measure()


class TestMotionGate(unittest.TestCase):
    def test_a_chip_without_a_gyroscope_skips_the_gate(self):
        obj = build()
        self.assertFalse(obj.has_gyro)
        self.assertIsNone(obj.measure().rotation_rate)
    def test_a_head_at_rest_passes_and_reports_its_rate(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        self.assertTrue(obj.has_gyro)
        reading = obj.measure()
        self.assertAlmostEqual(reading.angle, 12., places=4)
        self.assertAlmostEqual(reading.rotation_rate, 0.)
    def test_a_turning_head_is_rejected(self):
        obj = build(chip=FakeIMUChip(b_rate=4.))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("turning during the measurement", str(cm.exception))
    def test_the_gate_uses_the_magnitude_not_one_axis(self):
        # 3-4-5: the head turns about B at 4 deg/s while the whole chip
        # also spins at 3 deg/s about the axis pointing up, which tilts
        # nothing.  The gate should see 5.
        obj = build({'max_sample_deviation': 0.},
                    chip=FakeIMUChip(b_rate=4., off_axis_rate=3.))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("5.000 deg/s", str(cm.exception))
    def test_the_gate_can_be_raised(self):
        obj = build({'max_rotation_rate': 10., 'max_sample_deviation': 0.},
                    chip=FakeIMUChip(b_rate=4.))
        self.assertAlmostEqual(obj.measure().rotation_rate, 4.)
    def test_zero_disables_the_gate_entirely(self):
        obj = build({'max_rotation_rate': 0., 'max_sample_deviation': 0.,
                     'max_fusion_disagreement': 0.},
                    chip=FakeIMUChip(b_rate=99.))
        self.assertIsNone(obj.measure().rotation_rate)
    def test_a_silent_gyroscope_is_reported(self):
        # The accelerometer arrives but the gyroscope stream does not.
        # Fusion reads one combined stream, so this is the unfused path.
        obj = build({'fusion': False}, chip=FakeIMUChip(gyro_lag=5.))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("no gyroscope samples", str(cm.exception))
    def test_the_rate_reaches_get_status(self):
        obj = build({'max_sample_deviation': 0.},
                    chip=FakeIMUChip(b_rate=.25))
        self.assertTrue(obj.get_status()['has_gyro'])
        self.assertIsNone(obj.get_status()['rotation_rate'])
        obj.measure()
        self.assertAlmostEqual(obj.get_status()['rotation_rate'], .25)
    def test_the_rate_is_reported_by_b_measure(self):
        obj = build({'max_sample_deviation': 0.},
                    chip=FakeIMUChip(b_rate=.25))
        gcmd = FakeGCmd()
        obj.cmd_B_MEASURE(gcmd)
        self.assertIn("rotation rate = 0.2500 deg/s", gcmd.responses[0])


######################################################################
# The complementary filter
######################################################################

class TestComplementaryFilter(unittest.TestCase):
    def test_wrap180(self):
        for raw, want in ((0., 0.), (180., 180.), (-180., 180.),
                          (181., -179.), (-181., 179.), (540., 180.),
                          (359., -1.), (-359., 1.)):
            self.assertAlmostEqual(abh.wrap180(raw), want, places=9)
    def test_a_steady_accelerometer_and_no_rate_holds_the_angle(self):
        filt = abh.ComplementaryFilter(.2)
        for _ in range(1000):
            filt.update(30., 0., .001)
        self.assertAlmostEqual(filt.angle, 30., places=6)
    def test_it_converges_on_the_accelerometer_from_a_bad_seed(self):
        filt = abh.ComplementaryFilter(.05)
        filt.update(0., 0., 0.)          # seeded at 0
        for _ in range(2000):
            filt.update(45., 0., .001)   # accelerometer says 45
        self.assertAlmostEqual(filt.angle, 45., places=4)
    def test_the_gyroscope_carries_the_short_timescales(self):
        # A rate the accelerometer does not see at all: the filter
        # follows it for about tau and is then pulled back
        filt = abh.ComplementaryFilter(.2)
        filt.update(0., 0., 0.)
        for _ in range(50):
            filt.update(0., 10., .001)   # 10 deg/s for 50 ms
        self.assertGreater(filt.angle, .3)
        for _ in range(5000):
            filt.update(0., 0., .001)
        self.assertAlmostEqual(filt.angle, 0., places=4)
    def test_a_consistent_ramp_is_tracked_without_lag(self):
        # Accelerometer and gyroscope agreeing on a 20 deg/s sweep: the
        # fused angle should sit on the accelerometer's value, not
        # behind it
        filt = abh.ComplementaryFilter(.2)
        dt, rate = .001, 20.
        angle = 0.
        for i in range(3000):
            angle = rate * i * dt
            filt.update(angle, rate, dt if i else 0.)
        self.assertAlmostEqual(filt.angle, angle, places=3)
    def test_it_survives_the_180_degree_wrap(self):
        filt = abh.ComplementaryFilter(.2)
        dt, rate = .001, 60.
        angle = 170.
        for i in range(1000):
            angle = abh.wrap180(170. + rate * i * dt)
            filt.update(angle, rate, dt if i else 0.)
        # the sweep crosses +180 partway through; the filter should come
        # out on the far side of the wrap, tracking the last angle fed in
        self.assertLess(filt.angle, 0.)
        self.assertAlmostEqual(filt.angle, angle, places=3)
    def test_a_non_positive_tau_is_refused(self):
        for tau in (0., -1.):
            with self.assertRaises(ValueError):
                abh.ComplementaryFilter(tau)


######################################################################
# The gyroscope axis and sign, which are derived rather than configured
######################################################################

class TestGyroAxis(unittest.TestCase):
    def test_the_reference_mounting(self):
        # zero=+z, positive=+x  =>  rotation axis y, and dB/dt = -w_y
        z, x = abh.parse_signed_axis('+z'), abh.parse_signed_axis('+x')
        self.assertEqual(abh.gyro_axis_coefficient(z, x), (-1., 1))
    def test_negating_positive_vector_flips_the_sign(self):
        z = abh.parse_signed_axis('+z')
        self.assertEqual(abh.gyro_axis_coefficient(
            z, abh.parse_signed_axis('-x')), (1., 1))
    def test_negating_zero_vector_flips_the_sign(self):
        x = abh.parse_signed_axis('+x')
        self.assertEqual(abh.gyro_axis_coefficient(
            abh.parse_signed_axis('-z'), x), (1., 1))
    def test_swapping_the_two_flips_the_sign(self):
        z, x = abh.parse_signed_axis('+z'), abh.parse_signed_axis('+x')
        a, _ = abh.gyro_axis_coefficient(z, x)
        b, _ = abh.gyro_axis_coefficient(x, z)
        self.assertEqual(a, -b)
    def test_every_mounting_recovers_the_true_rate(self):
        # Rotate a synthetic head about its own B axis and check that
        # the derived coefficient turns the sensor's rate vector back
        # into +10 deg/s, for all 24 axis-aligned mountings.
        for zname in ('+x', '-x', '+y', '-y', '+z', '-z'):
            for pname in ('+x', '-x', '+y', '-y', '+z', '-z'):
                zero = abh.parse_signed_axis(zname)
                positive = abh.parse_signed_axis(pname)
                if zero[0] == positive[0]:
                    continue
                w = [0.] * 3
                w[zero[0]] = zero[1]
                u = [0.] * 3
                u[positive[0]] = positive[1]
                # the machine's B axis in sensor coords is w x u, and
                # the sensor's own rate about it is -dB/dt
                n = [w[1]*u[2]-w[2]*u[1], w[2]*u[0]-w[0]*u[2],
                     w[0]*u[1]-w[1]*u[0]]
                omega = [-10. * c for c in n]
                coeff, index = abh.gyro_axis_coefficient(zero, positive)
                self.assertAlmostEqual(coeff * omega[index], 10., places=9,
                                       msg="%s / %s" % (zname, pname))


######################################################################
# The fused measurement
######################################################################

class TestFusion(unittest.TestCase):
    def test_a_chip_without_a_gyroscope_does_not_fuse(self):
        obj = build(chip=FakeAccelChip(angle=7.))
        reading = obj.measure()
        self.assertIsNone(reading.fused_angle)
        self.assertAlmostEqual(reading.angle, reading.accel_angle)
        self.assertFalse(obj.get_status()['fusion'])
    def test_a_stationary_head_fuses_to_the_same_answer(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        reading = obj.measure()
        self.assertIsNotNone(reading.fused_angle)
        self.assertAlmostEqual(reading.fused_angle, 12., places=4)
        self.assertAlmostEqual(reading.accel_angle, 12., places=4)
        self.assertAlmostEqual(reading.disagreement, 0., places=4)
    def test_the_fused_angle_leads_the_accelerometer_on_a_moving_head(self):
        # Sweeping at 4 deg/s from 0: the window runs 0.25 to 0.75 s, so
        # the accelerometer average lands at its midpoint (2.0 deg) while
        # the fused angle tracks to the window's end (3.0 deg).
        obj = build({'max_rotation_rate': 0., 'max_sample_deviation': 0.},
                    chip=FakeIMUChip(b_rate=4.))
        reading = obj.measure()
        self.assertAlmostEqual(reading.accel_angle, 2.0, delta=.02)
        self.assertAlmostEqual(reading.fused_angle, 3.0, delta=.02)
        # and the authoritative angle is the fused one
        self.assertEqual(reading.angle, reading.fused_angle)
    def test_the_fused_angle_follows_the_sign_of_the_rotation(self):
        for b_rate in (4., -4.):
            obj = build({'max_rotation_rate': 0., 'max_sample_deviation': 0.},
                        chip=FakeIMUChip(b_rate=b_rate))
            reading = obj.measure()
            # fused sits ahead of the accelerometer average in the
            # direction the head is actually turning
            self.assertGreater(
                (reading.fused_angle - reading.accel_angle) * b_rate, 0.)
    def test_an_off_axis_spin_does_not_move_the_angle(self):
        obj = build({'max_rotation_rate': 0.},
                    chip=FakeIMUChip(angle=12., off_axis_rate=30.))
        reading = obj.measure()
        self.assertAlmostEqual(reading.fused_angle, 12., places=4)
    def test_an_inverted_gyroscope_is_caught_when_the_head_moves_enough(self):
        obj = build({'max_rotation_rate': 0., 'max_sample_deviation': 0.,
                     'max_magnitude_error': 0.},
                    chip=FakeIMUChip(b_rate=40., invert_gyro=True))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("gyroscope is inverted", str(cm.exception))
    def test_an_inverted_gyroscope_is_invisible_on_a_parked_head(self):
        # Stated as a test because it is a real limitation: with nothing
        # to integrate, the sign cannot be checked.  That needs a
        # deliberate move (B_GYRO_CALIBRATE, phase 3).
        obj = build(chip=FakeIMUChip(angle=12., invert_gyro=True))
        self.assertAlmostEqual(obj.measure().fused_angle, 12., places=4)
    def test_fusion_can_be_turned_off_in_config(self):
        obj = build({'fusion': False}, chip=FakeIMUChip(angle=12.))
        reading = obj.measure()
        self.assertIsNone(reading.fused_angle)
        self.assertAlmostEqual(reading.angle, reading.accel_angle)
    def test_fusion_can_be_turned_off_per_command(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        self.assertIsNone(obj.measure(fusion=False).fused_angle)
        self.assertIsNotNone(obj.measure(fusion=True).fused_angle)
    def test_b_measure_reports_both_estimates(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        gcmd = FakeGCmd()
        obj.cmd_B_MEASURE(gcmd)
        self.assertIn("fused 12.000 deg", gcmd.responses[0])
        self.assertIn("accelerometer alone 12.000", gcmd.responses[0])
    def test_b_measure_can_ask_for_the_unfused_angle(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        gcmd = FakeGCmd({'FUSION': 0})
        obj.cmd_B_MEASURE(gcmd)
        self.assertNotIn("fused", gcmd.responses[0])
    def test_a_capture_shorter_than_the_filter_needs_is_refused(self):
        obj = build({'settle_time': 0., 'sample_time': .05,
                     'fusion_tau': .5}, chip=FakeIMUChip(angle=12.))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("fusion_tau", str(cm.exception))
    def test_the_fusion_state_reaches_get_status(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        status = obj.get_status()
        self.assertTrue(status['fusion'])
        self.assertAlmostEqual(status['fusion_tau'], .2)
        self.assertEqual(status['rotation_axis_sign'], -1.)
        self.assertIsNone(status['fusion_disagreement'])
        obj.measure()
        self.assertIsNotNone(obj.get_status()['fusion_disagreement'])


if __name__ == '__main__':
    unittest.main()
