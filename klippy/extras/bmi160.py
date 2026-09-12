# Support for the BMI160 six-axis IMU (accelerometer + gyroscope)
#
# Copyright (C) 2025  Francisco Stephens <francisco.stephens.g@gmail.com>
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The BMI160 carries an accelerometer and a gyroscope in one package, on
# one bus, sharing one FIFO and one sample clock.  This driver reads them
# together: with fifo_gyr_en and fifo_acc_en both set and the FIFO in
# headerless mode, each frame is twelve bytes -
#
#   gx_lo gx_hi gy_lo gy_hi gz_lo gz_hi ax_lo ax_hi ay_lo ay_hi az_lo az_hi
#
# six little-endian signed 16-bit values, gyroscope first (the FIFO
# stores sensor data in data-register order, and the gyro registers at
# 0x0C precede the accel registers at 0x12).  The two halves of a frame
# are therefore simultaneous by construction, which is the whole reason
# to put both sensors in one chip - see docs/BMI160_IMU.md.
#
# Headerless mode requires every enabled sensor to run at the same output
# data rate, and the accelerometer stops at 1600 Hz, so 1600 Hz bounds
# both.  Running without the gyroscope (gyro: False) gives the older
# accel-only behaviour and six-byte frames.
import logging
import collections, multiprocessing, os
from . import bus, adxl345, bulk_sensor

# BMI160 registers
REG_CHIPID          = 0x00
# The FIFO stores sensor data in data-register order, so these two
# addresses are what put the gyroscope first in a combined frame
REG_GYR_DATA_START  = 0x0C
REG_ACC_DATA_START  = 0x12
REG_ACC_CONF        = 0x40
REG_ACC_RANGE       = 0x41
REG_GYR_CONF        = 0x42
REG_GYR_RANGE       = 0x43
REG_FIFO_DOWNS      = 0x45
REG_FIFO_CONFIG_1   = 0x47
REG_FOC_CONF        = 0x69
REG_OFFSET_6        = 0x77
REG_CMD             = 0x7E

REG_MOD_READ        = 0x80

# BMI160 commands for the CMD register
CMD_START_FOC       = 0x03
CMD_ACC_PM_SUSPEND  = 0x10
CMD_ACC_PM_NORMAL   = 0x11
CMD_GYR_PM_SUSPEND  = 0x14
CMD_GYR_PM_NORMAL   = 0x15
CMD_FIFO_FLUSH      = 0xB0

BMI160_DEV_ID       = 0xD1

# Output data rates.  ODR = 100 / 2**(8 - code) Hz; the accelerometer
# is not permitted above 1600 Hz, and headerless FIFO mode requires the
# gyroscope to match it.
DATA_RATES = {100: 0x08, 200: 0x09, 400: 0x0A, 800: 0x0B, 1600: 0x0C}
# Normal-mode digital filter, for both sensors (acc_bwp / gyr_bwp = 0b010)
BWP_NORMAL = 0x20

# Accelerometer full scale in g -> (acc_range code, LSB/g)
ACCEL_RANGES = {2: (0x03, 16384.), 4: (0x05, 8192.),
                8: (0x08, 4096.), 16: (0x0C, 2048.)}
# Gyroscope full scale in deg/s -> (gyr_range code, LSB per deg/s)
GYRO_RANGES = {125: (0x04, 262.4), 250: (0x03, 131.2), 500: (0x02, 65.6),
               1000: (0x01, 32.8), 2000: (0x00, 16.4)}

# FIFO_CONFIG_1 bits.  Header mode is deliberately off: it costs a byte
# per frame and buys flexibility this driver does not want.
FIFO_ACC_EN         = 0x40
FIFO_GYR_EN         = 0x80
# No FIFO downsampling
SET_FIFO_DOWNS      = 0x00

# OFFSET[6] enable bits
OFFSET_GYR_EN       = 0x80
OFFSET_ACC_EN       = 0x40

# 1 g in the units Klipper's accelerometer clients expect (mm/s^2)
FREEFALL_ACCEL = adxl345.FREEFALL_ACCEL

# Over-reading the FIFO returns the fixed magic 0x80 in every data byte,
# which decodes to this value.  The firmware only reads a block once a
# whole block is pending, so seeing it means something went wrong.
INVALID_FRAME = -0x7f80

BATCH_UPDATES = 0.100

BMI_I2C_ADDR = 0x69

# The gyroscope takes 55 ms typical and 80 ms maximum to reach normal
# mode from suspend; the accelerometer takes 3.8 ms.
GYRO_STARTUP_TIME = .100
ACCEL_STARTUP_TIME = .010
# Fast offset compensation takes at most 250 ms
FOC_TIME = .300

Gyro_Measurement = collections.namedtuple(
    'Gyro_Measurement', ('time', 'gyro_x', 'gyro_y', 'gyro_z'))

# Which 16-bit channel of a frame the mcu-side tap detector watches.  The
# value is a byte offset into the frame, so it depends on whether the
# gyroscope is in the FIFO - see get_trigger_frame_offset().
TAP_CHANNELS = ('accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z')


######################################################################
# Sample stream views
######################################################################

# A three-column view of the chip's combined sample stream, presented as
# an independent bulk sensor stream.  The chip converts each frame once,
# into one (time, ...) tuple; a view hands its clients the three columns
# they asked for and drops the rest.  This is what lets one FIFO feed
# both an accelerometer client and a gyroscope client without either of
# them knowing the other exists.
class SampleStreamView:
    def __init__(self, printer, batch_bulk, first_column):
        self.printer = printer
        self.batch_bulk = batch_bulk
        self.first_column = first_column
    def _project(self, client_cb):
        c = self.first_column
        def handle_batch(msg):
            view = dict(msg)
            view['data'] = [(s[0], s[c], s[c+1], s[c+2]) for s in msg['data']]
            return client_cb(view)
        return handle_batch
    def add_client(self, client_cb):
        self.batch_bulk.add_client(self._project(client_cb))
    def add_mux_endpoint(self, path, key, value, header):
        start_resp = {'header': header}
        def add_api_client(web_request):
            whbatch = bulk_sensor.BatchWebhooksClient(web_request)
            self.add_client(whbatch.handle_batch)
            web_request.send(start_resp)
        wh = self.printer.lookup_object('webhooks')
        wh.register_mux_endpoint(path, key, value, add_api_client)

# The accelerometer helper's batching and windowing, with the gyroscope's
# names.  The names only matter in the CSV header, but a capture of tap
# data labelled "accel_x" when it is angular rate is worse than useless.
class GyroQueryHelper(adxl345.AccelQueryHelper):
    def get_samples(self):
        if not self.msgs:
            return self.samples
        total = sum([len(m['data']) for m in self.msgs])
        count = 0
        self.samples = samples = [None] * total
        for msg in self.msgs:
            for samp_time, x, y, z in msg['data']:
                if samp_time < self.request_start_time:
                    continue
                if samp_time > self.request_end_time:
                    break
                samples[count] = Gyro_Measurement(samp_time, x, y, z)
                count += 1
        del samples[count:]
        return self.samples
    def write_to_file(self, filename):
        def write_impl():
            try:
                # Try to re-nice writing process
                os.nice(20)
            except:
                pass
            f = open(filename, "w")
            f.write("#time,gyro_x,gyro_y,gyro_z\n")
            samples = self.samples or self.get_samples()
            for t, gyro_x, gyro_y, gyro_z in samples:
                f.write("%.6f,%.6f,%.6f,%.6f\n" % (t, gyro_x, gyro_y, gyro_z))
            f.close()
        write_proc = multiprocessing.Process(target=write_impl)
        write_proc.daemon = True
        write_proc.start()


######################################################################
# The chip
######################################################################

class BMI160:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        adxl345.AccelCommandHelper(config, self)
        # Sampling.  The gyroscope cannot be in the FIFO at a different
        # rate than the accelerometer, so there is one data_rate.
        self.enable_gyro = config.getboolean('gyro', True)
        self.data_rate = config.getchoice('rate', sorted(DATA_RATES), 1600)
        self.odr_code = DATA_RATES[self.data_rate]
        accel_g = config.getchoice('accel_range', sorted(ACCEL_RANGES), 2)
        self.accel_range_code, accel_lsb_per_g = ACCEL_RANGES[accel_g]
        gyro_dps = config.getchoice('gyro_range', sorted(GYRO_RANGES), 250)
        self.gyro_range_code, gyro_lsb_per_dps = GYRO_RANGES[gyro_dps]
        self.accel_range = accel_g
        self.gyro_range = gyro_dps
        # One physical chip, so one axes_map - but the two sensors carry
        # different units, so the map is read twice with different scales
        accel_scale = FREEFALL_ACCEL / accel_lsb_per_g
        self.axes_map = adxl345.read_axes_map(config, accel_scale,
                                              accel_scale, accel_scale)
        gyro_scale = 1. / gyro_lsb_per_dps
        self.gyro_axes_map = adxl345.read_axes_map(config, gyro_scale,
                                                   gyro_scale, gyro_scale)
        # The mcu-side tap detector watches one channel, named here
        # because the frame layout is this section's business
        self.tap_channel = config.getchoice('tap_channel',
                                            list(TAP_CHANNELS), 'accel_z')
        if not self.enable_gyro and self.tap_channel.startswith('gyro'):
            raise config.error(
                "[%s] tap_channel '%s' needs the gyroscope, but gyro is"
                " disabled in this section"
                % (config.get_name(), self.tap_channel))
        # Bus.  cs_pin selects SPI; without it this is an I2C chip.
        if config.get('cs_pin', None) is not None:
            self.bus = bus.MCU_SPI_from_config(config, 0, default_speed=1000000)
            self.bus_type = 'spi'
        else:
            self.bus = bus.MCU_I2C_from_config(config,
                                default_addr=BMI_I2C_ADDR, default_speed=400000)
            self.bus_type = 'i2c'
        self.mcu = mcu = self.bus.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_bmi160_cmd = None
        mcu.add_config_cmd(
            "config_bmi160 oid=%d bus_oid=%d bus_oid_type=%s"
            " bytes_per_frame=%d" % (oid, self.bus.get_oid(), self.bus_type,
                                     self.get_bytes_per_frame()))
        mcu.add_config_cmd("query_bmi160 oid=%d rest_ticks=0"
                           % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        # Bulk sample message reading.  One reader for the whole frame;
        # the views below split it.
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        unpack_fmt = "<hhhhhh" if self.enable_gyro else "<hhh"
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth,
                                                    unpack_fmt)
        self.last_error_count = 0
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        # Views.  In combined frames the gyroscope occupies columns 1-3
        # and the accelerometer 4-6; without the gyroscope the
        # accelerometer is back at columns 1-3.
        accel_col = 4 if self.enable_gyro else 1
        self.accel_stream = SampleStreamView(self.printer, self.batch_bulk,
                                             accel_col)
        self.accel_stream.add_mux_endpoint(
            "bmi160/dump_bmi160", "sensor", self.name,
            ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration'))
        self.gyro_stream = None
        if self.enable_gyro:
            self.gyro_stream = SampleStreamView(self.printer, self.batch_bulk,
                                                1)
            self.gyro_stream.add_mux_endpoint(
                "bmi160/dump_bmi160_gyro", "sensor", self.name,
                ('time', 'x_rate', 'y_rate', 'z_rate'))
        self._register_commands(config)
    def _build_config(self):
        cmdqueue = self.bus.get_command_queue()
        self.query_bmi160_cmd = self.mcu.lookup_command(
            "query_bmi160 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.ffreader.setup_query_command("query_bmi160_status oid=%c",
                                          oid=self.oid, cq=cmdqueue)

    ######################################################################
    # Frame layout
    ######################################################################
    def get_bytes_per_frame(self):
        return 12 if self.enable_gyro else 6
    def get_trigger_frame_offset(self):
        # Byte offset of self.tap_channel within a frame.  Gyroscope
        # first, then accelerometer, x/y/z within each, two bytes apart.
        sensor, axis = self.tap_channel.split('_')
        base = 0
        if sensor == 'accel' and self.enable_gyro:
            base = 6
        return base + 2 * 'xyz'.index(axis)

    ######################################################################
    # Register access
    ######################################################################
    def read_reg(self, reg):
        if self.mcu.is_fileoutput():
            return 0
        if self.bus_type == 'spi':
            params = self.bus.spi_transfer([reg | REG_MOD_READ, 0x00])
            response = bytearray(params['response'])
            return response[1]
        params = self.bus.i2c_read([reg], 1)
        return bytearray(params['response'])[0]
    def set_reg(self, reg, val, minclock=0):
        if self.bus_type == 'spi':
            # spi_transfer to ensure command completes on mcu before
            # continuing
            self.bus.spi_transfer([reg, val & 0xFF], minclock=minclock)
        else:
            # I2C already waits for completion by default
            self.bus.i2c_write([reg, val & 0xFF], minclock=minclock)
        # Small delay between register writes for stability
        self.reactor.pause(0.002)
        # The CMD register latches an opcode and reads back as something
        # else; registers below 0x40 are data, status or read-only.
        if reg >= 0x40 and reg != REG_CMD:
            stored_val = self.read_reg(reg)
            if stored_val != val and not self.mcu.is_fileoutput():
                raise self.printer.command_error(
                    "Failed to set BMI160 register [0x%x] to 0x%x: got 0x%x."
                    " This is generally indicative of connection problems"
                    " (e.g. faulty wiring) or a faulty bmi160 chip."
                    % (reg, val, stored_val))
    def _check_id(self):
        if self.bus_type == 'spi':
            # A rising edge on CSB puts the chip in SPI mode; a dummy
            # read of an unused register is the documented way to make
            # one happen before the first real transfer.
            self.read_reg(0x7F)
            self.reactor.pause(0.010)
        dev_id = self.read_reg(REG_CHIPID)
        if dev_id != BMI160_DEV_ID and not self.mcu.is_fileoutput():
            raise self.printer.command_error(
                "Invalid bmi160 id (got %x vs %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty bmi160 chip."
                % (dev_id, BMI160_DEV_ID))

    ######################################################################
    # Power and configuration
    ######################################################################
    def _wake_sensors(self):
        self.set_reg(REG_CMD, CMD_ACC_PM_NORMAL)
        self.reactor.pause(ACCEL_STARTUP_TIME)
        if self.enable_gyro:
            self.set_reg(REG_CMD, CMD_GYR_PM_NORMAL)
            self.reactor.pause(GYRO_STARTUP_TIME)
    def _suspend_sensors(self):
        if self.enable_gyro:
            self.set_reg(REG_CMD, CMD_GYR_PM_SUSPEND)
            self.reactor.pause(0.002)
        self.set_reg(REG_CMD, CMD_ACC_PM_SUSPEND)
    def _configure_sensors(self):
        # Both sensors run the normal-mode filter at the shared rate;
        # headerless FIFO mode requires the rates to be identical.
        self.set_reg(REG_ACC_CONF, BWP_NORMAL | self.odr_code)
        self.set_reg(REG_ACC_RANGE, self.accel_range_code)
        if self.enable_gyro:
            self.set_reg(REG_GYR_CONF, BWP_NORMAL | self.odr_code)
            self.set_reg(REG_GYR_RANGE, self.gyro_range_code)
        self.set_reg(REG_FIFO_DOWNS, SET_FIFO_DOWNS)
        fifo_config = FIFO_ACC_EN
        if self.enable_gyro:
            fifo_config |= FIFO_GYR_EN
        self.set_reg(REG_FIFO_CONFIG_1, fifo_config)
    def _flush_fifo(self):
        self.set_reg(REG_CMD, CMD_FIFO_FLUSH)
        self.reactor.pause(0.010)

    ######################################################################
    # Offset calibration (fast offset compensation)
    ######################################################################
    # The chip measures both sensors against a declared target and writes
    # the negated result into its OFFSET registers, applying it to every
    # subsequent sample.  The gyroscope's target is always zero rate, so
    # calibrating it needs nothing but a stationary head; the
    # accelerometer's target is a pose, so each axis has to be told what
    # it should be reading.  Only the volatile image registers are
    # written - the NVM behind them tolerates at most 14 write cycles, so
    # a calibration is deliberately lost on power cycle.
    FOC_ACC_TARGETS = {0: 0b11, 1: 0b01, -1: 0b10}
    def run_foc(self, calibrate_gyro=True, accel_targets=None):
        was_started = self.batch_bulk.is_started
        if was_started:
            raise self.printer.command_error(
                "bmi160: cannot calibrate '%s' while a measurement is in"
                " progress" % (self.name,))
        self._check_id()
        self._wake_sensors()
        self._configure_sensors()
        foc_conf = 0
        if calibrate_gyro:
            if not self.enable_gyro:
                raise self.printer.command_error(
                    "bmi160: the gyroscope is disabled in section '%s'"
                    % (self.name,))
            foc_conf |= 0x40
        if accel_targets is not None:
            x, y, z = accel_targets
            foc_conf |= (self.FOC_ACC_TARGETS[x] << 4
                         | self.FOC_ACC_TARGETS[y] << 2
                         | self.FOC_ACC_TARGETS[z])
        self.set_reg(REG_FOC_CONF, foc_conf)
        self.set_reg(REG_CMD, CMD_START_FOC)
        self.reactor.pause(FOC_TIME)
        # Enabling a sensor's offset is what actually applies the result
        enable = self.read_reg(REG_OFFSET_6) & 0x3f
        if calibrate_gyro:
            enable |= OFFSET_GYR_EN
        if accel_targets is not None:
            enable |= OFFSET_ACC_EN
        self.set_reg(REG_OFFSET_6, enable)
        self._suspend_sensors()

    ######################################################################
    # Client interfaces
    ######################################################################
    def start_internal_client(self):
        aqh = adxl345.AccelQueryHelper(self.printer)
        self.accel_stream.add_client(aqh.handle_batch)
        return aqh
    def start_internal_gyro_client(self):
        if self.gyro_stream is None:
            raise self.printer.command_error(
                "bmi160: the gyroscope is disabled in section '%s'"
                % (self.name,))
        gqh = GyroQueryHelper(self.printer)
        self.gyro_stream.add_client(gqh.handle_batch)
        return gqh
    def has_gyro(self):
        return self.enable_gyro
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.data_rate
    def setup_trigger_analog(self, trigger_analog_oid):
        # The mcu-side seam for accel_z_tap - see docs/BMI160_IMU.md
        self.mcu.add_config_cmd(
            "bmi160_attach_trigger_analog oid=%d trigger_analog_oid=%d"
            " frame_offset=%d" % (self.oid, trigger_analog_oid,
                                  self.get_trigger_frame_offset()),
            is_init=True)
    def lookup_sensor_error(self, error_code):
        return "Unknown bmi160 error %d" % (error_code,)

    ######################################################################
    # Measurement
    ######################################################################
    def _convert_samples(self, samples):
        (ax_pos, ax_scale), (ay_pos, ay_scale), (az_pos, az_scale) = \
            self.axes_map
        if not self.enable_gyro:
            count = 0
            for ptime, rx, ry, rz in samples:
                raw = (rx, ry, rz)
                if rx == ry == rz == INVALID_FRAME:
                    self.last_error_count += 1
                    continue
                samples[count] = (round(ptime, 6),
                                  round(raw[ax_pos] * ax_scale, 6),
                                  round(raw[ay_pos] * ay_scale, 6),
                                  round(raw[az_pos] * az_scale, 6))
                count += 1
            del samples[count:]
            return
        (gx_pos, gx_scale), (gy_pos, gy_scale), (gz_pos, gz_scale) = \
            self.gyro_axes_map
        count = 0
        for ptime, rgx, rgy, rgz, rax, ray, raz in samples:
            if rax == ray == raz == INVALID_FRAME:
                # An over-read of the FIFO; the firmware is meant to make
                # this impossible, so count it rather than hide it.
                self.last_error_count += 1
                continue
            gyro = (rgx, rgy, rgz)
            accel = (rax, ray, raz)
            samples[count] = (round(ptime, 6),
                              round(gyro[gx_pos] * gx_scale, 6),
                              round(gyro[gy_pos] * gy_scale, 6),
                              round(gyro[gz_pos] * gz_scale, 6),
                              round(accel[ax_pos] * ax_scale, 6),
                              round(accel[ay_pos] * ay_scale, 6),
                              round(accel[az_pos] * az_scale, 6))
            count += 1
        del samples[count:]
    def _start_measurements(self):
        self._check_id()
        self._wake_sensors()
        self._configure_sensors()
        self._flush_fifo()
        # Drain the FIFO roughly every four sample periods
        rest_ticks = self.mcu.seconds_to_clock(4. / self.data_rate)
        self.query_bmi160_cmd.send([self.oid, rest_ticks])
        logging.info("BMI160 starting '%s' measurements", self.name)
        self.ffreader.note_start()
        self.last_error_count = 0
    def _finish_measurements(self):
        self.query_bmi160_cmd.send_wait_ack([self.oid, 0])
        self._suspend_sensors()
        self.ffreader.note_end()
        logging.info("BMI160 finished '%s' measurements", self.name)
    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        return {'data_rate': self.data_rate, 'gyro': self.enable_gyro,
                'accel_range': self.accel_range,
                'gyro_range': self.gyro_range if self.enable_gyro else None,
                'tap_channel': self.tap_channel}
    def _register_commands(self, config):
        # As AccelCommandHelper does: the named form always, plus the
        # CHIP-less default form when this section has no explicit name.
        self._register_mux_commands(self.name)
        if len(config.get_name().split()) == 1:
            self._register_mux_commands(None)
    def _register_mux_commands(self, name):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("BMI160_QUERY", "CHIP", name,
                                   self.cmd_BMI160_QUERY,
                                   desc=self.cmd_BMI160_QUERY_help)
        gcode.register_mux_command("BMI160_CALIBRATE", "CHIP", name,
                                   self.cmd_BMI160_CALIBRATE,
                                   desc=self.cmd_BMI160_CALIBRATE_help)
    cmd_BMI160_QUERY_help = "Query the BMI160 for its current values"
    def cmd_BMI160_QUERY(self, gcmd):
        aclient = self.start_internal_client()
        gclient = None
        if self.enable_gyro:
            gclient = self.start_internal_gyro_client()
        self.printer.lookup_object('toolhead').dwell(1.)
        aclient.finish_measurements()
        avalues = aclient.get_samples()
        if not avalues:
            raise gcmd.error("No bmi160 measurements found")
        _, ax, ay, az = avalues[-1]
        lines = ["bmi160 %s: acceleration x/y/z = %.3f / %.3f / %.3f mm/s^2"
                 % (self.name, ax, ay, az)]
        if gclient is not None:
            gclient.finish_measurements()
            gvalues = gclient.get_samples()
            if gvalues:
                _, gx, gy, gz = gvalues[-1]
                lines.append("  rotation x/y/z = %.4f / %.4f / %.4f deg/s"
                             % (gx, gy, gz))
        gcmd.respond_info("\n".join(lines))
    cmd_BMI160_CALIBRATE_help = "Run BMI160 fast offset compensation"
    def cmd_BMI160_CALIBRATE(self, gcmd):
        # The gyroscope's target is always zero rate, so GYRO=1 needs
        # nothing but a stationary chip.  The accelerometer's target is a
        # pose: each axis is told whether it should read -1, 0 or +1 g,
        # so the head has to be parked somewhere known first.
        do_gyro = gcmd.get_int('GYRO', 1, minval=0, maxval=1)
        targets = None
        if any([gcmd.get_int(a, None) is not None for a in "XYZ"]):
            targets = tuple([gcmd.get_int(a, 0, minval=-1, maxval=1)
                             for a in "XYZ"])
            if sorted([abs(t) for t in targets]) != [0, 0, 1]:
                raise gcmd.error(
                    "BMI160_CALIBRATE: X, Y and Z declare which way the chip"
                    " is facing, so exactly one of them must be +1 or -1 (the"
                    " axis pointing up or down) and the others 0 - got"
                    " %s" % (targets,))
        if not do_gyro and targets is None:
            raise gcmd.error(
                "BMI160_CALIBRATE: nothing to calibrate - pass GYRO=1, or"
                " X/Y/Z to calibrate the accelerometer against a known pose")
        self.printer.lookup_object('toolhead').wait_moves()
        self.run_foc(calibrate_gyro=bool(do_gyro), accel_targets=targets)
        what = []
        if do_gyro:
            what.append("gyroscope zero-rate offset")
        if targets is not None:
            what.append("accelerometer offset against x/y/z = %d/%d/%d g"
                        % targets)
        gcmd.respond_info(
            "bmi160 %s: calibrated %s.\nThe result lives in the chip's"
            " volatile offset registers and is lost on power cycle."
            % (self.name, " and ".join(what)))


def load_config(config):
    return BMI160(config)

def load_config_prefix(config):
    return BMI160(config)
