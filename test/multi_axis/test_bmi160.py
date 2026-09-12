#!/usr/bin/env python
# Host test of the BMI160 IMU driver (klippy/extras/bmi160.py).
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This drives the *real* extras/bmi160.py against a stubbed printer, mcu
# and bus, so it runs anywhere Python is available (it does not need a
# compiled c_helper.so, a serial port, or a BMI160).
#
# What it can check honestly: the register values the driver builds for
# each range and rate against the datasheet's own tables, the frame
# geometry that the firmware and the host have to agree on, the
# accelerometer and gyroscope views of one combined stream, axes_map
# applied to both at once, and the config errors.
#
# What it cannot check: whether the chip is wired correctly, the real
# noise floor, or any threshold number.  Those are machine measurements.
#
# Run with:  python test/multi_axis/test_bmi160.py
import os, struct, sys, types, unittest

KLIPPY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'klippy')
sys.path.insert(0, os.path.normpath(KLIPPY_DIR))

# extras/bus.py imports mcu, which needs pyserial; nothing under test
# reaches the real thing
sys.modules.setdefault('mcu', types.ModuleType('mcu'))

from extras import bmi160, bulk_sensor

G = bmi160.FREEFALL_ACCEL


######################################################################
# Stubbed printer environment
######################################################################

class ConfigError(Exception):
    pass

class FakeGCode:
    def __init__(self):
        self.mux = {}
    def register_mux_command(self, cmd, key, value, func, desc=None):
        if (cmd, value) in self.mux:
            raise ConfigError("%s %s=%s already registered" % (cmd, key, value))
        self.mux[(cmd, value)] = func

class FakeWebhooks:
    def __init__(self):
        self.endpoints = {}
    def register_mux_endpoint(self, path, key, value, callback):
        self.endpoints[(path, value)] = callback

class FakePrinter:
    config_error = ConfigError
    command_error = ConfigError
    def __init__(self):
        self.objects = {'gcode': FakeGCode(), 'webhooks': FakeWebhooks()}
    def get_reactor(self):
        return FakeReactor()
    def lookup_object(self, name, default=Ellipsis):
        if name in self.objects:
            return self.objects[name]
        if default is not Ellipsis:
            return default
        raise ConfigError("Unknown config object '%s'" % (name,))

class FakeReactor:
    def pause(self, delay):
        pass

class FakeMCU:
    def __init__(self):
        self.config_cmds = []
        self.next_oid = 0
    def create_oid(self):
        self.next_oid += 1
        return self.next_oid - 1
    def add_config_cmd(self, cmd, **kwargs):
        self.config_cmds.append(cmd)
    def register_config_callback(self, cb):
        pass
    def seconds_to_clock(self, secs):
        return int(secs * 1000000.)
    def is_fileoutput(self):
        return True

class FakeBus:
    def __init__(self, mcu):
        self.mcu = mcu
        self.writes = []
    def get_mcu(self):
        return self.mcu
    def get_oid(self):
        return 99
    def get_command_queue(self):
        return None
    def spi_transfer(self, data, minclock=0):
        self.writes.append(tuple(data))
        return {'response': bytearray([0, 0])}
    def i2c_write(self, data, minclock=0):
        self.writes.append(tuple(data))
    def i2c_read(self, regs, count):
        return {'response': bytearray([0] * count)}

class FakeConfig:
    error = ConfigError
    def __init__(self, printer, values, name='bmi160'):
        self.printer = printer
        self.values = values
        self.name = name
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def _lookup(self, option, default):
        if option in self.values:
            return self.values[option]
        if default is Ellipsis:
            raise ConfigError("Option '%s' is not valid" % (option,))
        return default
    def get(self, option, default=Ellipsis, **kwargs):
        val = self._lookup(option, default)
        return val if val is None else str(val)
    def getint(self, option, default=Ellipsis, **kwargs):
        val = self._lookup(option, default)
        return val if val is None else int(val)
    def getfloat(self, option, default=Ellipsis, **kwargs):
        val = self._lookup(option, default)
        return val if val is None else float(val)
    def getboolean(self, option, default=Ellipsis, **kwargs):
        val = self._lookup(option, default)
        return val if val is None else bool(val)
    def getchoice(self, option, choices, default=Ellipsis, **kwargs):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default)
        else:
            c = self.get(option, default)
        if c not in choices:
            raise ConfigError("Choice '%s' for option '%s' is not valid"
                              % (c, option))
        return choices[c]
    def getlist(self, option, default=Ellipsis, count=None, **kwargs):
        val = self._lookup(option, default)
        if isinstance(val, str):
            val = [v.strip() for v in val.split(',')]
        return val


def build(values=None, name='bmi160'):
    printer = FakePrinter()
    mcu = FakeMCU()
    vals = {'cs_pin': 'imu:gpio9'}
    vals.update(values or {})
    config = FakeConfig(printer, vals, name)
    orig_spi, orig_i2c = bmi160.bus.MCU_SPI_from_config, \
        bmi160.bus.MCU_I2C_from_config
    bmi160.bus.MCU_SPI_from_config = lambda *a, **k: FakeBus(mcu)
    bmi160.bus.MCU_I2C_from_config = lambda *a, **k: FakeBus(mcu)
    try:
        return bmi160.BMI160(config)
    finally:
        bmi160.bus.MCU_SPI_from_config = orig_spi
        bmi160.bus.MCU_I2C_from_config = orig_i2c


######################################################################
# The datasheet tables
######################################################################

class TestDatasheetTables(unittest.TestCase):
    def test_accel_sensitivity(self):
        # BMI160 datasheet, "OUTPUT SIGNAL ACCELEROMETER": the typical
        # sensitivity halves with every doubling of the range
        self.assertEqual(bmi160.ACCEL_RANGES[2], (0x03, 16384.))
        self.assertEqual(bmi160.ACCEL_RANGES[4], (0x05, 8192.))
        self.assertEqual(bmi160.ACCEL_RANGES[8], (0x08, 4096.))
        self.assertEqual(bmi160.ACCEL_RANGES[16], (0x0C, 2048.))
    def test_gyro_sensitivity(self):
        # Datasheet 2.11.14, Register (0x43) GYR_RANGE
        self.assertEqual(bmi160.GYRO_RANGES[2000], (0x00, 16.4))
        self.assertEqual(bmi160.GYRO_RANGES[1000], (0x01, 32.8))
        self.assertEqual(bmi160.GYRO_RANGES[500], (0x02, 65.6))
        self.assertEqual(bmi160.GYRO_RANGES[250], (0x03, 131.2))
        self.assertEqual(bmi160.GYRO_RANGES[125], (0x04, 262.4))
    def test_output_data_rate_codes(self):
        # ODR = 100 / 2**(8 - code) Hz, and the accelerometer is not
        # permitted above 1600 Hz (code 0b1100)
        for rate, code in bmi160.DATA_RATES.items():
            self.assertEqual(rate, 100. / 2 ** (8 - code))
        self.assertEqual(max(bmi160.DATA_RATES), 1600)
    def test_the_invalid_frame_magic(self):
        # An over-read returns 0x80 in every data byte
        raw = struct.unpack("<h", bytes(bytearray([0x80, 0x80])))[0]
        self.assertEqual(raw, bmi160.INVALID_FRAME)


######################################################################
# Frame geometry - the host and the firmware have to agree on this
######################################################################

class TestFrameGeometry(unittest.TestCase):
    def test_combined_frames_are_twelve_bytes(self):
        self.assertEqual(build().get_bytes_per_frame(), 12)
    def test_accel_only_frames_are_six_bytes(self):
        self.assertEqual(build({'gyro': False}).get_bytes_per_frame(), 6)
    def test_a_whole_number_of_frames_fits_a_bulk_message(self):
        # FixedFreqReader timestamps by counting, so every bulk message
        # must hold exactly MAX_BULK_MSG_SIZE // frame_size whole frames,
        # and the firmware sends a fixed 48-byte block for both layouts
        for gyro, frame in ((True, 12), (False, 6)):
            per_block = bulk_sensor.MAX_BULK_MSG_SIZE // frame
            self.assertEqual(per_block * frame, 48)
            fmt = "<hhhhhh" if gyro else "<hhh"
            self.assertEqual(struct.calcsize(fmt), frame)
    def test_trigger_offsets_in_a_combined_frame(self):
        # Gyroscope first, then accelerometer, x/y/z two bytes apart
        for channel, offset in (('gyro_x', 0), ('gyro_y', 2), ('gyro_z', 4),
                                ('accel_x', 6), ('accel_y', 8),
                                ('accel_z', 10)):
            chip = build({'tap_channel': channel})
            self.assertEqual(chip.get_trigger_frame_offset(), offset)
    def test_trigger_offsets_without_the_gyroscope(self):
        for channel, offset in (('accel_x', 0), ('accel_y', 2),
                                ('accel_z', 4)):
            chip = build({'gyro': False, 'tap_channel': channel})
            self.assertEqual(chip.get_trigger_frame_offset(), offset)
    def test_the_firmware_is_told_the_frame_size(self):
        chip = build()
        cfg = [c for c in chip.mcu.config_cmds if c.startswith('config_bmi160')]
        self.assertEqual(len(cfg), 1)
        self.assertIn("bytes_per_frame=12", cfg[0])
        self.assertIn("bytes_per_frame=6",
                      build({'gyro': False}).mcu.config_cmds[0])
    def test_attaching_a_trigger_passes_the_offset(self):
        chip = build({'tap_channel': 'gyro_y'})
        chip.setup_trigger_analog(7)
        attach = [c for c in chip.mcu.config_cmds
                  if c.startswith('bmi160_attach_trigger_analog')]
        self.assertEqual(len(attach), 1)
        self.assertIn("trigger_analog_oid=7", attach[0])
        self.assertIn("frame_offset=2", attach[0])


######################################################################
# Register construction
######################################################################

class TestRegisters(unittest.TestCase):
    def test_the_default_accel_conf_matches_the_old_driver(self):
        # 0x2C: acc_bwp=0b010 (normal mode), acc_odr=0b1100 (1600 Hz)
        chip = build()
        self.assertEqual(bmi160.BWP_NORMAL | chip.odr_code, 0x2C)
    def test_gyr_conf_shares_the_rate(self):
        # Headerless fifo mode requires identical rates, so the two
        # config registers carry the same bandwidth and rate bits
        chip = build({'rate': 800})
        self.assertEqual(bmi160.BWP_NORMAL | chip.odr_code, 0x2B)
    def test_range_codes(self):
        self.assertEqual(build().accel_range_code, 0x03)         # +/-2 g
        self.assertEqual(build().gyro_range_code, 0x03)          # +/-250
        self.assertEqual(build({'accel_range': 16}).accel_range_code, 0x0C)
        self.assertEqual(build({'gyro_range': 2000}).gyro_range_code, 0x00)
    def test_fifo_config_enables_both_sensors(self):
        self.assertEqual(bmi160.FIFO_ACC_EN | bmi160.FIFO_GYR_EN, 0xC0)
        self.assertEqual(bmi160.FIFO_ACC_EN, 0x40)
    def test_foc_accel_targets(self):
        # Datasheet 2.11.29: 0b01 is +1 g, 0b10 is -1 g, 0b11 is 0 g
        targets = bmi160.BMI160.FOC_ACC_TARGETS
        self.assertEqual((targets[1], targets[-1], targets[0]), (1, 2, 3))


######################################################################
# Sample conversion
######################################################################

def convert(chip, raw_samples):
    samples = list(raw_samples)
    chip._convert_samples(samples)
    return samples

class TestConversion(unittest.TestCase):
    def test_combined_frames_split_into_gyro_then_accel(self):
        chip = build()
        # 16384 LSB/g at +/-2 g, 131.2 LSB per deg/s at +/-250
        out = convert(chip, [(1., 1312, 0, -656, 0, 0, 16384)])
        self.assertEqual(len(out), 1)
        t, gx, gy, gz, ax, ay, az = out[0]
        self.assertEqual(t, 1.)
        self.assertAlmostEqual(gx, 10., places=3)
        self.assertAlmostEqual(gy, 0.)
        self.assertAlmostEqual(gz, -5., places=3)
        self.assertAlmostEqual(ax, 0.)
        self.assertAlmostEqual(az, G, places=2)
    def test_accel_only_frames_keep_the_old_shape(self):
        chip = build({'gyro': False})
        out = convert(chip, [(1., 0, 0, 16384)])
        self.assertEqual(len(out[0]), 4)
        self.assertAlmostEqual(out[0][3], G, places=2)
    def test_axes_map_applies_to_both_sensors(self):
        # One physical chip, so one map - but two unit scales
        chip = build({'axes_map': 'z, -y, x'})
        out = convert(chip, [(1., 1312, 0, 0, 0, 0, 16384)])
        _, gx, gy, gz, ax, ay, az = out[0]
        # gyro x=10 deg/s about the chip's x lands on the mapped z
        self.assertAlmostEqual(gz, 10., places=3)
        self.assertAlmostEqual(gx, 0.)
        # accel 1 g about the chip's z lands on the mapped x
        self.assertAlmostEqual(ax, G, places=2)
        self.assertAlmostEqual(az, 0.)
    def test_a_negated_axis_is_negated_in_both(self):
        chip = build({'axes_map': 'x, -y, z'})
        out = convert(chip, [(1., 0, 1312, 0, 0, 16384, 0)])
        self.assertAlmostEqual(out[0][2], -10., places=3)
        self.assertAlmostEqual(out[0][5], -G, places=2)
    def test_the_range_sets_the_scale(self):
        chip = build({'accel_range': 16, 'gyro_range': 2000})
        out = convert(chip, [(1., 164, 0, 0, 0, 0, 2048)])
        self.assertAlmostEqual(out[0][1], 10., places=3)
        self.assertAlmostEqual(out[0][6], G, places=2)
    def test_an_over_read_frame_is_dropped_and_counted(self):
        chip = build()
        bad = bmi160.INVALID_FRAME
        out = convert(chip, [(1., 0, 0, 0, 0, 0, 16384),
                             (2., bad, bad, bad, bad, bad, bad),
                             (3., 0, 0, 0, 0, 0, 16384)])
        self.assertEqual([s[0] for s in out], [1., 3.])
        self.assertEqual(chip.last_error_count, 1)


######################################################################
# The two views of one stream
######################################################################

class FakeBatchBulk:
    def __init__(self):
        self.clients = []
    def add_client(self, cb):
        self.clients.append(cb)
    def send(self, msg):
        return [cb(msg) for cb in self.clients]

class TestStreamViews(unittest.TestCase):
    def test_a_view_projects_its_three_columns(self):
        printer = FakePrinter()
        batch = FakeBatchBulk()
        gyro = bmi160.SampleStreamView(printer, batch, 1)
        accel = bmi160.SampleStreamView(printer, batch, 4)
        seen = {}
        gyro.add_client(lambda m: seen.setdefault('gyro', m['data']))
        accel.add_client(lambda m: seen.setdefault('accel', m['data']))
        batch.send({'data': [(1., 10., 11., 12., 20., 21., 22.)],
                    'errors': 0})
        self.assertEqual(seen['gyro'], [(1., 10., 11., 12.)])
        self.assertEqual(seen['accel'], [(1., 20., 21., 22.)])
    def test_a_view_leaves_the_rest_of_the_message_alone(self):
        printer = FakePrinter()
        batch = FakeBatchBulk()
        view = bmi160.SampleStreamView(printer, batch, 4)
        seen = []
        view.add_client(lambda m: seen.append(m) or True)
        batch.send({'data': [(1., 0., 0., 0., 0., 0., 0.)], 'errors': 3,
                    'overflows': 7})
        self.assertEqual(seen[0]['errors'], 3)
        self.assertEqual(seen[0]['overflows'], 7)
    def test_both_dump_endpoints_are_registered(self):
        chip = build()
        wh = chip.printer.lookup_object('webhooks')
        self.assertIn(("bmi160/dump_bmi160", 'bmi160'), wh.endpoints)
        self.assertIn(("bmi160/dump_bmi160_gyro", 'bmi160'), wh.endpoints)
    def test_no_gyro_endpoint_without_a_gyroscope(self):
        chip = build({'gyro': False})
        wh = chip.printer.lookup_object('webhooks')
        self.assertIn(("bmi160/dump_bmi160", 'bmi160'), wh.endpoints)
        self.assertNotIn(("bmi160/dump_bmi160_gyro", 'bmi160'), wh.endpoints)
    def test_a_gyro_client_is_refused_when_disabled(self):
        chip = build({'gyro': False})
        self.assertFalse(chip.has_gyro())
        with self.assertRaises(ConfigError):
            chip.start_internal_gyro_client()


######################################################################
# Configuration
######################################################################

class TestConfig(unittest.TestCase):
    def test_defaults(self):
        chip = build()
        self.assertEqual(chip.data_rate, 1600)
        self.assertEqual(chip.accel_range, 2)
        self.assertEqual(chip.gyro_range, 250)
        self.assertEqual(chip.tap_channel, 'accel_z')
        self.assertTrue(chip.has_gyro())
    def test_an_unsupported_rate_is_refused(self):
        with self.assertRaises(ConfigError):
            build({'rate': 3200})
    def test_an_unsupported_range_is_refused(self):
        with self.assertRaises(ConfigError):
            build({'accel_range': 3})
        with self.assertRaises(ConfigError):
            build({'gyro_range': 100})
    def test_a_gyro_tap_channel_needs_the_gyroscope(self):
        with self.assertRaises(ConfigError) as cm:
            build({'gyro': False, 'tap_channel': 'gyro_z'})
        self.assertIn("needs the gyroscope", str(cm.exception))
    def test_the_commands_are_registered(self):
        chip = build()
        gcode = chip.printer.lookup_object('gcode')
        # Both the named form and the CHIP-less default form
        self.assertIn(('BMI160_QUERY', 'bmi160'), gcode.mux)
        self.assertIn(('BMI160_QUERY', None), gcode.mux)
        self.assertIn(('BMI160_CALIBRATE', 'bmi160'), gcode.mux)
    def test_a_named_section_gets_no_default_command(self):
        chip = build(name='bmi160 head')
        gcode = chip.printer.lookup_object('gcode')
        self.assertIn(('BMI160_QUERY', 'head'), gcode.mux)
        self.assertNotIn(('BMI160_QUERY', None), gcode.mux)
    def test_get_status(self):
        status = build({'accel_range': 8}).get_status()
        self.assertEqual(status['data_rate'], 1600)
        self.assertEqual(status['accel_range'], 8)
        self.assertEqual(status['gyro_range'], 250)
        self.assertTrue(status['gyro'])
        self.assertIsNone(build({'gyro': False}).get_status()['gyro_range'])


if __name__ == '__main__':
    unittest.main()
