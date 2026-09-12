# Gravity-referenced measurement of the B (tool tilt) axis
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# An accelerometer bolted to the *rotating* part of a tilting head reads
# the gravity vector in the head's own frame, and a gravity vector in the
# head's frame is the head's tilt - absolutely, with no reference to
# where the axis has travelled since it was last homed.  That turns
# measuring B into an observation rather than a move.
#
# This module is phase one of docs/Accel_B_Homing.md: the measurement
# primitive and the B_MEASURE command that reports it.  Nothing here
# moves the machine, and nothing here homes anything yet.
#
# THE ZERO REFERENCE
#
# Which way is "B = 0" is declared, not fitted.  Two signed sensor axes
# say everything the measurement needs:
#
#   zero_vector      the sensor axis that reads +1 g at B = 0
#   positive_vector  the sensor axis that reads +1 g at B = +90
#
# An accelerometer at rest reads the specific force, which points *up*,
# so both of these are "the sensor axis pointing straight up" at their
# respective angles.  Both are found by looking: park the head, run
# ACCELEROMETER_QUERY, and note which axis reads about +9800 (use the
# negated name if it reads about -9800).
#
# Writing the two components as
#
#   w = the reading along zero_vector      (1 g at B = 0)
#   u = the reading along positive_vector  (1 g at B = +90)
#
# the head angle is just
#
#   B = atan2(u, w)
#
# which is 0 at B = 0 and +90 at B = +90 by construction, so no separate
# offset or inversion flag is needed.  Using *both* in-plane axes through
# atan2 - rather than one axis through asin - is what keeps the
# resolution uniform across the whole range; a single-axis reading goes
# flat near +/-90 degrees.
#
# The third sensor axis is the one parallel to the B rotation axis.  It
# is not used in the angle, but it is reported, because it is the health
# check: it should barely change as B turns, and the in-plane radius
# hypot(u, w) should stay at 1 g.
#
# THE MOTION GATE
#
# An accelerometer cannot tell a tilted head from an accelerating one,
# so a measurement is only a tilt if the head was actually at rest.  The
# accelerometer's own evidence for that is indirect - a small per-sample
# deviation - and a head rocking slowly on its belts barely moves that
# statistic.  When the chip is an IMU (a [bmi160]), the gyroscope
# answers the question directly: a head at rest reads zero rate.  The
# gate is skipped for chips with no gyroscope, so an [adxl345] keeps
# working exactly as before.  See docs/BMI160_IMU.md.
#
# WHAT THIS PHASE DOES NOT DO
#
# The reading is uncorrected.  An ADXL345 has a zero-g offset of up to
# +/-150 mg and an inter-axis gain tolerance of about +/-10 %, which
# together are worth several degrees of absolute error - so treat the
# number as a diagnostic, not as a calibrated angle, until the offset and
# gain fit of phase two lands.  Repeatability is much better than
# accuracy: a stationary head re-measures to a few hundredths of a
# degree, which is why the noise gates below are tight even though the
# absolute tolerances are loose.
import collections, math
import stepper

# 1 g in the units the accelerometer chips report (mm/s^2).  Matches
# adxl345.FREEFALL_ACCEL.
FREEFALL_ACCEL = 9.80665 * 1000.

# The bulk sensor helpers deliver samples to the host in batches - 0.100 s
# in adxl345.py - and a chip on a secondary mcu (a USB accelerometer board
# such as the Fly-ADXL345-USB, or a CAN toolhead) adds a link's worth of
# latency on top of that.  finish_measurements() waits for the moves to
# finish, not for the sensor batches to arrive, so without a trailing
# dwell the batch carrying the tail of the averaging window has usually
# not been delivered yet.  Losing the tail is harmless on a long window
# but a short one on a laggy link can drop enough samples to trip the
# "dropping data" check.  Dwell past the window instead, so it is
# comfortably in the past before the samples are asked for.
DEFAULT_BATCH_MARGIN = .3

# Index of the B coordinate within a toolhead position vector
B_POS_INDEX = stepper.KIN_AXIS_INDEXES[4]

AXIS_NAMES = ('x', 'y', 'z')
SIGNED_AXES = {
    'x': (0, 1.), '+x': (0, 1.), '-x': (0, -1.),
    'y': (1, 1.), '+y': (1, 1.), '-y': (1, -1.),
    'z': (2, 1.), '+z': (2, 1.), '-z': (2, -1.),
}
VALID_AXES = "+x, -x, +y, -y, +z or -z"

TiltReading = collections.namedtuple('TiltReading', (
    'angle', 'vector', 'deviation', 'magnitude', 'u', 'w', 'out_of_plane',
    'count', 'rotation_rate'))


######################################################################
# Pure helpers (no printer dependency - unit tested directly)
######################################################################

def parse_signed_axis(value):
    # A signed sensor axis, restricted to the six axis-aligned
    # directions.  An arbitrary mounting angle is a phase two problem.
    key = str(value).strip().lower()
    if key not in SIGNED_AXES:
        raise ValueError("invalid sensor axis '%s'" % (value,))
    return SIGNED_AXES[key]

def signed_axis_name(axis):
    index, sign = axis
    return ('+' if sign > 0. else '-') + AXIS_NAMES[index]

def out_of_plane_index(zero_axis, positive_axis):
    # The remaining axis - the one parallel to the B rotation axis
    if zero_axis[0] == positive_axis[0]:
        raise ValueError("zero_vector and positive_vector share an axis")
    used = (zero_axis[0], positive_axis[0])
    return [i for i in range(3) if i not in used][0]

def project(vector, axis):
    index, sign = axis
    return sign * vector[index]

def measure_angle(vector, zero_axis, positive_axis):
    # The head angle, in degrees, from a gravity vector in sensor axes
    w = project(vector, zero_axis)
    u = project(vector, positive_axis)
    return math.degrees(math.atan2(u, w))

def summarize(vectors):
    # Per-axis mean and sample standard deviation of a list of triples
    count = len(vectors)
    if not count:
        raise ValueError("no samples to summarize")
    means = tuple(sum([v[i] for v in vectors]) / count for i in range(3))
    if count < 2:
        return means, (0., 0., 0.)
    devs = []
    for i in range(3):
        mean = means[i]
        var = sum([(v[i] - mean)**2 for v in vectors]) / (count - 1)
        devs.append(math.sqrt(var))
    return means, tuple(devs)

def magnitude(vector):
    return math.sqrt(sum([c * c for c in vector]))


######################################################################
# The printer object
######################################################################

class AccelBHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.chip_name = config.get('accel_chip', 'bmi160')
        self.zero_axis = self._get_axis(config, 'zero_vector')
        self.positive_axis = self._get_axis(config, 'positive_vector')
        if self.zero_axis[0] == self.positive_axis[0]:
            raise config.error(
                "[%s] zero_vector and positive_vector must name different"
                " sensor axes - '%s' and '%s' are the same axis.  They are"
                " the direction the sensor points up in at B=0 and at"
                " B=+90, which cannot be the same direction."
                % (self.name, config.get('zero_vector'),
                   config.get('positive_vector')))
        self.oop_index = out_of_plane_index(self.zero_axis, self.positive_axis)
        # Sampling.  The head hangs on belts and rings after a move, so
        # the settle dwell is separate from the averaging window.
        self.settle_time = config.getfloat('settle_time', .25, minval=0.)
        self.sample_time = config.getfloat('sample_time', .5, minval=.05)
        self.batch_margin = config.getfloat('batch_margin',
                                            DEFAULT_BATCH_MARGIN, minval=0.)
        # A stationary ADXL345 at 3200 Hz shows roughly 120-180 mm/s^2 of
        # per-sample noise, so anything much above that is the head still
        # moving.  Zero disables the check.
        self.max_deviation = config.getfloat('max_sample_deviation', 500.,
                                             minval=0.)
        # The magnitude gate is deliberately loose: with no gain
        # calibration yet, a chip within its +/-10 % sensitivity spec can
        # legitimately read 0.9 to 1.1 g.  It is here to catch a moving
        # head or a misconfigured chip, not to grade the sensor.
        self.max_magnitude_error = config.getfloat('max_magnitude_error',
                                                   1500., minval=0.)
        # The gyroscope gate, when the chip has a gyroscope.  The
        # default is provisional: it is meant to sit well above the
        # sensor's own noise floor and well below any real motion, and
        # the right value is whatever B_MEASURE reports on a parked head
        # plus a margin.  Zero disables the gate.
        self.max_rotation_rate = config.getfloat('max_rotation_rate', 1.,
                                                 minval=0.)
        # Generous by default, because the reading is uncalibrated - see
        # the header comment.
        self.check_tolerance = config.getfloat('check_tolerance', 5.,
                                               above=0.)
        self.chip = self.toolhead = self.b_axis = None
        self.has_gyro = False
        self.b_projection = None
        self.last_reading = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('B_MEASURE', self.cmd_B_MEASURE,
                               desc=self.cmd_B_MEASURE_help)
    def _get_axis(self, config, option):
        try:
            return parse_signed_axis(config.get(option))
        except ValueError as e:
            raise config.error("[%s] %s - use one of %s"
                               % (self.name, e, VALID_AXES))
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.b_projection = self.printer.lookup_object('b_projection', None)
        chip = self.printer.lookup_object(self.chip_name, None)
        if chip is None:
            raise self.printer.config_error(
                "[%s] accel_chip '%s' is not configured"
                % (self.name, self.chip_name))
        if not hasattr(chip, 'start_internal_client'):
            raise self.printer.config_error(
                "[%s] '%s' is not an accelerometer"
                % (self.name, self.chip_name))
        self.chip = chip
        # An IMU offers a second stream of angular rate; a plain
        # accelerometer does not, and the gate below is then skipped
        self.has_gyro = (hasattr(chip, 'start_internal_gyro_client')
                         and getattr(chip, 'has_gyro', lambda: True)())
        # The B axis is a rotary axis object, so its homed flag lives
        # there rather than in the toolhead's homed_axes
        for ea in self.toolhead.get_extra_axes():
            if ea is not None and getattr(ea, 'get_axis_gcode_id', None):
                if ea.get_axis_gcode_id() == 'B':
                    self.b_axis = ea
                    break
        if self.b_axis is None:
            raise self.printer.config_error(
                "[%s] the printer has no B axis - add 'b' to the"
                " 'additional_axes' option of [printer]" % (self.name,))

    ######################################################################
    # The measurement primitive
    ######################################################################
    def measure(self, settle_time=None, sample_time=None):
        settle = self.settle_time if settle_time is None else settle_time
        window = self.sample_time if sample_time is None else sample_time
        toolhead = self.toolhead
        toolhead.wait_moves()
        client = self.chip.start_internal_client()
        gyro_client = None
        if self.has_gyro and self.max_rotation_rate:
            gyro_client = self.chip.start_internal_gyro_client()
        start_time = toolhead.get_last_move_time()
        # The averaging window is settle..settle+window; the extra margin
        # is only there to let the batches carrying it arrive
        toolhead.dwell(settle + window + self.batch_margin)
        client.finish_measurements()
        if gyro_client is not None:
            gyro_client.finish_measurements()
        # AccelQueryHelper trims to the request window but knows nothing
        # about the settle dwell, so drop that part here.  Samples are
        # (print_time, x, y, z); index rather than name the fields, so
        # any chip exposing start_internal_client() works.
        first = start_time + settle
        last = first + window
        vectors = [(s[1], s[2], s[3]) for s in client.get_samples()
                   if first <= s[0] <= last]
        if not vectors:
            raise self.printer.command_error(
                "%s: no accelerometer samples in the measurement window -"
                " check that '%s' is responding (try ACCELEROMETER_QUERY)"
                % (self.name, self.chip_name))
        rate = getattr(self.chip, 'data_rate', None)
        if rate and len(vectors) < .5 * rate * window:
            raise self.printer.command_error(
                "%s: only %d of an expected %d accelerometer samples -"
                " the connection to '%s' is dropping data, or is slow"
                " enough that batch_margin needs raising"
                % (self.name, len(vectors), int(rate * window),
                   self.chip_name))
        mean, dev = summarize(vectors)
        if self.max_deviation and max(dev) > self.max_deviation:
            raise self.printer.command_error(
                "%s: the head was still moving during the measurement"
                " (sample deviation %.0f > %.0f mm/s^2) - increase"
                " settle_time" % (self.name, max(dev), self.max_deviation))
        mag = magnitude(mean)
        if (self.max_magnitude_error
            and abs(mag - FREEFALL_ACCEL) > self.max_magnitude_error):
            raise self.printer.command_error(
                "%s: measured %.0f mm/s^2 where gravity is %.0f - the head"
                " is accelerating, or '%s' is not reporting correctly"
                % (self.name, mag, FREEFALL_ACCEL, self.chip_name))
        # The gyroscope gate.  This is a direct observation that the
        # head was not turning, where everything above it is an
        # inference from how steady the gravity vector looked.
        rotation_rate = None
        if gyro_client is not None:
            rates = [magnitude((s[1], s[2], s[3]))
                     for s in gyro_client.get_samples()
                     if first <= s[0] <= last]
            if not rates:
                raise self.printer.command_error(
                    "%s: no gyroscope samples in the measurement window -"
                    " check that '%s' is responding (try BMI160_QUERY)"
                    % (self.name, self.chip_name))
            rotation_rate = sum(rates) / len(rates)
            if rotation_rate > self.max_rotation_rate:
                raise self.printer.command_error(
                    "%s: the head was turning during the measurement"
                    " (%.3f deg/s > %.3f) - increase settle_time, or raise"
                    " max_rotation_rate if this is the sensor's noise floor"
                    % (self.name, rotation_rate, self.max_rotation_rate))
        reading = TiltReading(
            angle=measure_angle(mean, self.zero_axis, self.positive_axis),
            vector=mean, deviation=dev, magnitude=mag,
            u=project(mean, self.positive_axis),
            w=project(mean, self.zero_axis),
            out_of_plane=mean[self.oop_index], count=len(vectors),
            rotation_rate=rotation_rate)
        self.last_reading = reading
        return reading

    ######################################################################
    # Comparison against the commanded angle
    ######################################################################
    def is_b_homed(self):
        return bool(self.b_axis.get_status().get('homed'))
    def get_commanded_b(self):
        # The angle the machine believes the head is turned to.  With
        # [b_projection] enabled the toolhead's B is a bed-frame angle,
        # and the head is really at its projection onto the plane the
        # head can tilt in - which is what the sensor measures.  [rtcp]
        # needs no such conversion: it moves x/z, never B.
        pos = self.toolhead.get_position()
        if self.b_projection is not None:
            return self.b_projection.project_pos(pos)
        return pos[B_POS_INDEX]

    ######################################################################
    # Status and commands
    ######################################################################
    def get_status(self, eventtime=None):
        res = {'zero_vector': signed_axis_name(self.zero_axis),
               'positive_vector': signed_axis_name(self.positive_axis),
               'rotation_axis': AXIS_NAMES[self.oop_index],
               'accel_chip': self.chip_name}
        res['has_gyro'] = self.has_gyro
        reading = self.last_reading
        res['measured_b'] = None if reading is None else reading.angle
        res['rotation_rate'] = (None if reading is None
                                else reading.rotation_rate)
        return res
    cmd_B_MEASURE_help = "Measure the B axis angle against gravity"
    def cmd_B_MEASURE(self, gcmd):
        settle = gcmd.get_float('SETTLE', self.settle_time, minval=0.)
        window = gcmd.get_float('SAMPLE_TIME', self.sample_time, minval=.05)
        reading = self.measure(settle, window)
        x, y, z = reading.vector
        dx, dy, dz = reading.deviation
        lines = [
            "%s: B = %.3f deg (%d samples)"
            % (self.name, reading.angle, reading.count),
            "  vector x/y/z = %.1f / %.1f / %.1f mm/s^2,"
            " |a| = %.1f (%.4f g)"
            % (x, y, z, reading.magnitude, reading.magnitude / FREEFALL_ACCEL),
            "  in plane u/w = %.1f / %.1f, radius = %.1f,"
            " out of plane (%s) = %.1f"
            % (reading.u, reading.w, math.hypot(reading.u, reading.w),
               AXIS_NAMES[self.oop_index], reading.out_of_plane),
            "  sample deviation x/y/z = %.1f / %.1f / %.1f mm/s^2"
            % (dx, dy, dz)]
        if reading.rotation_rate is not None:
            lines.append("  rotation rate = %.4f deg/s (gate %.3f)"
                         % (reading.rotation_rate, self.max_rotation_rate))
        check = gcmd.get_int('CHECK', 0, minval=0, maxval=1)
        if not self.is_b_homed():
            # Report the measurement either way - it is the useful part
            # of the answer, and the whole point of the command
            lines.append("  B is not homed - no commanded angle to compare")
            gcmd.respond_info("\n".join(lines))
            if check:
                raise gcmd.error("%s: cannot CHECK B against a commanded"
                                 " angle while B is unhomed" % (self.name,))
            return
        commanded = self.get_commanded_b()
        error = reading.angle - commanded
        lines.append("  commanded B = %.3f deg, error = %+.3f deg"
                     % (commanded, error))
        gcmd.respond_info("\n".join(lines))
        if check:
            tolerance = gcmd.get_float('TOLERANCE', self.check_tolerance,
                                       above=0.)
            if abs(error) > tolerance:
                raise gcmd.error(
                    "%s: the head is at %.3f deg but B is commanded to"
                    " %.3f deg (error %+.3f, tolerance %.3f)"
                    % (self.name, reading.angle, commanded, error, tolerance))


def load_config(config):
    return AccelBHoming(config)
