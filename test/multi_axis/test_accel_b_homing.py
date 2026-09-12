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
                 offset=(0., 0., 0.), y_bias=0., delivery_lag=0.):
        self.angle, self.noise = angle, noise
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
    def samples_over(self, start, end):
        end -= self.delivery_lag
        rad = math.radians(self.angle)
        base = (G * math.sin(rad), self.y_bias, G * math.cos(rad))
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
    def get_samples(self):
        return self.chip.samples_over(self.start, self.end)

# A BMI160-shaped chip: the same accelerometer, plus a gyroscope stream
# turning at whatever rate the test declares.  rate is in deg/s in the
# sensor frame, so (0, 0, 0) is a head genuinely at rest.
class FakeIMUChip(FakeAccelChip):
    def __init__(self, rate=(0., 0., 0.), gyro_lag=0., **kwargs):
        FakeAccelChip.__init__(self, **kwargs)
        self.rate = rate
        self.gyro_lag = gyro_lag
    def has_gyro(self):
        return True
    def start_internal_gyro_client(self):
        return FakeGyroClient(self)
    def gyro_samples_over(self, start, end):
        end -= self.delivery_lag + self.gyro_lag
        total = int(round((end - start) * self.data_rate))
        return [(start + (i + .5) / self.data_rate,) + tuple(self.rate)
                for i in range(max(0, total))]

class FakeGyroClient(FakeAccelClient):
    def get_samples(self):
        return self.chip.gyro_samples_over(self.start, self.end)

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

class TestMotionGate(unittest.TestCase):
    def test_a_chip_without_a_gyroscope_skips_the_gate(self):
        obj = build()
        self.assertFalse(obj.has_gyro)
        self.assertIsNone(obj.measure().rotation_rate)
    def test_a_head_at_rest_passes_and_reports_its_rate(self):
        obj = build(chip=FakeIMUChip(angle=12.))
        self.assertTrue(obj.has_gyro)
        reading = obj.measure()
        self.assertAlmostEqual(reading.angle, 12., places=6)
        self.assertAlmostEqual(reading.rotation_rate, 0.)
    def test_a_turning_head_is_rejected(self):
        obj = build(chip=FakeIMUChip(rate=(0., 4., 0.)))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("turning during the measurement", str(cm.exception))
    def test_the_gate_uses_the_magnitude_not_one_axis(self):
        # 3-4-5: no single axis exceeds the default 1 deg/s gate by much,
        # but the head is turning at 5 deg/s
        obj = build(chip=FakeIMUChip(rate=(3., 4., 0.)))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("5.000 deg/s", str(cm.exception))
    def test_the_gate_can_be_raised(self):
        obj = build({'max_rotation_rate': 10.},
                    chip=FakeIMUChip(rate=(0., 4., 0.)))
        self.assertAlmostEqual(obj.measure().rotation_rate, 4.)
    def test_zero_disables_the_gate_entirely(self):
        obj = build({'max_rotation_rate': 0.},
                    chip=FakeIMUChip(rate=(0., 99., 0.)))
        self.assertIsNone(obj.measure().rotation_rate)
    def test_a_silent_gyroscope_is_reported(self):
        # The accelerometer arrives but the gyroscope stream does not
        obj = build(chip=FakeIMUChip(gyro_lag=5.))
        with self.assertRaises(ConfigError) as cm:
            obj.measure()
        self.assertIn("no gyroscope samples", str(cm.exception))
    def test_the_rate_reaches_get_status(self):
        obj = build(chip=FakeIMUChip(rate=(0., .25, 0.)))
        self.assertTrue(obj.get_status()['has_gyro'])
        self.assertIsNone(obj.get_status()['rotation_rate'])
        obj.measure()
        self.assertAlmostEqual(obj.get_status()['rotation_rate'], .25)
    def test_the_rate_is_reported_by_b_measure(self):
        obj = build(chip=FakeIMUChip(rate=(0., .25, 0.)))
        gcmd = FakeGCmd()
        obj.cmd_B_MEASURE(gcmd)
        self.assertIn("rotation rate = 0.2500 deg/s", gcmd.responses[0])


if __name__ == '__main__':
    unittest.main()
