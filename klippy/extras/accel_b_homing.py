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
# moves the machine.  It does steer the endstop home: see "WHICH WAY TO
# HOME" below.
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
# THE FUSED MEASUREMENT
#
# An accelerometer cannot tell a tilted head from an accelerating one.
# A gyroscope can see the head turn but cannot say where it started, and
# its zero-rate offset makes an integrated angle drift.  The two fail in
# opposite directions, which is what makes them worth fusing: the
# accelerometer is absolute but only over long times, the gyroscope is
# exact but only over short ones.
#
# A complementary filter crosses them over at one time constant, tau:
#
#   predicted = angle + rate * dt          (gyroscope, short term)
#   angle     = predicted + (1 - alpha) * (accel_angle - predicted)
#   alpha     = tau / (tau + dt)           (accelerometer, long term)
#
# Above tau the accelerometer wins, so the result is absolute and does
# not drift.  Below tau the gyroscope wins, so the result tracks the
# head *through* the ringing that follows a move rather than waiting for
# it to stop.
#
# THE GYROSCOPE'S SIGN IS NOT A FREE PARAMETER
#
# This is the part worth reading twice.  Which gyroscope axis carries
# dB/dt, and with which sign, follows from zero_vector and
# positive_vector alone - there is nothing extra to declare and nothing
# to guess.  B is the angle of the world-up vector in the sensor frame,
# measured from zero_vector toward positive_vector.  Turning the sensor
# at omega makes world-fixed vectors appear to turn at -omega in the
# sensor frame, and the zero->positive sense is a positive rotation
# about (w_hat x u_hat), so
#
#   dB/dt = -omega . (w_hat x u_hat)
#
# With both vectors axis aligned, w_hat x u_hat is plus or minus the
# third basis vector and this collapses to one signed component - which
# is what gyro_axis_coefficient() below returns.
#
# One caveat lives in the chip module rather than here: an angular rate
# is a pseudovector, so an axes_map that reflects the frame flips it.
# bmi160.py applies the map's determinant for exactly this reason.
#
# THE MOTION GATE
#
# Fusion says what the angle is; the gate says whether to trust it as a
# *static* tilt.  A head at rest reads zero rate, so max_rotation_rate
# tests directly for the thing max_sample_deviation can only infer.
# Both are skipped for chips with no gyroscope, so an [adxl345] keeps
# working exactly as before.  See docs/BMI160_IMU.md.
#
# WHICH WAY TO HOME
#
# The corertheta B rail does not guess its homing direction from where
# position_endstop sits in the range (see stepper.py's infer_homing_dir).
# With homing_positive_dir unset, G28 B asks this module instead: the
# head is measured where it rests, and homes positive if that is below
# position_endstop, negative if above.  Within homing_tolerance of the
# endstop the measurement cannot tell the sides apart, so an endstop at
# a range limit is homed toward that limit and one inside the range is
# refused.  After the home the head is measured again, and a head that
# is not at the endstop - a sensorless home that triggered instantly, or
# a positive_vector that disagrees with the motors about which way is
# +B - leaves B unhomed.
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
import collections, logging, math
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

# The filter is seeded from the accelerometer and converges with time
# constant tau, so a capture much shorter than a few tau is reporting
# its seed rather than a fused answer.
MIN_FUSION_SPANS = 3.

# Fraction of the measurement window used for the trailing accelerometer
# average that the fused angle is checked against.  Both estimate "the
# angle now", so they agree when the gyroscope is wired up correctly and
# diverge when it is not - see the disagreement check in measure().
FUSION_TAIL_FRACTION = .2

# Levi-Civita symbol, for the cross product in gyro_axis_coefficient()
LEVI_CIVITA = {(0, 1, 2): 1., (1, 2, 0): 1., (2, 0, 1): 1.,
               (0, 2, 1): -1., (2, 1, 0): -1., (1, 0, 2): -1.}

AXIS_NAMES = ('x', 'y', 'z')
SIGNED_AXES = {
    'x': (0, 1.), '+x': (0, 1.), '-x': (0, -1.),
    'y': (1, 1.), '+y': (1, 1.), '-y': (1, -1.),
    'z': (2, 1.), '+z': (2, 1.), '-z': (2, -1.),
}
VALID_AXES = "+x, -x, +y, -y, +z or -z"

TiltReading = collections.namedtuple('TiltReading', (
    'angle', 'accel_angle', 'fused_angle', 'disagreement', 'vector',
    'deviation', 'magnitude', 'u', 'w', 'out_of_plane', 'count',
    'rotation_rate'))


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

def overflows_during(client):
    # Every Klipper accelerometer reports a running count of possible
    # fifo overflows in each batch it delivers.  An increase across the
    # batches a client received means frames were lost while it was
    # listening - and because the bulk sensor helpers timestamp samples
    # by *counting* them, lost frames also leave the surviving ones
    # mistimed, which a fused angle integrates straight into its answer.
    counts = [m.get('overflows', 0) for m in getattr(client, 'msgs', [])]
    if len(counts) < 2:
        return 0
    return counts[-1] - counts[0]

def wrap180(angle):
    # An angle difference, brought into (-180, 180]
    return -((180. - angle) % 360. - 180.)

def gyro_axis_coefficient(zero_axis, positive_axis):
    # dB/dt from the raw gyroscope, as (coefficient, sensor axis index),
    # so that  dB/dt = coefficient * omega[index].  See the header.
    (zero_index, zero_sign) = zero_axis
    (positive_index, positive_sign) = positive_axis
    rotation_index = out_of_plane_index(zero_axis, positive_axis)
    eps = LEVI_CIVITA[(zero_index, positive_index, rotation_index)]
    coefficient = -(math.copysign(1., zero_sign)
                    * math.copysign(1., positive_sign) * eps)
    return coefficient, rotation_index

# Crosses an absolute-but-noisy angle over with a clean-but-drifting
# rate at one time constant.  Written as a correction to the predicted
# angle rather than as a weighted mean of two angles, so that a
# measurement straddling the +/-180 wrap does not average to the
# opposite side of the circle.
class ComplementaryFilter:
    def __init__(self, tau):
        if tau <= 0.:
            raise ValueError("fusion time constant must be positive")
        self.tau = tau
        self.angle = None
    def update(self, accel_angle, rate, dt):
        if self.angle is None or dt <= 0.:
            # Seed from the accelerometer.  The filter converges from
            # any starting point; starting at approximately the right
            # answer just saves a time constant of settling.
            self.angle = wrap180(accel_angle)
            return self.angle
        alpha = self.tau / (self.tau + dt)
        predicted = self.angle + rate * dt
        self.angle = wrap180(predicted
                             + (1. - alpha) * wrap180(accel_angle - predicted))
        return self.angle

def fuse_samples(samples, zero_axis, positive_axis, gyro_coefficient,
                 gyro_index, tau):
    # samples are (time, gx, gy, gz, ax, ay, az) - one instant per
    # sample, which is what makes fusing them legitimate
    filt = ComplementaryFilter(tau)
    last_time = None
    for sample in samples:
        accel_angle = measure_angle(tuple(sample[4:7]), zero_axis,
                                    positive_axis)
        rate = gyro_coefficient * sample[1 + gyro_index]
        dt = 0. if last_time is None else sample[0] - last_time
        filt.update(accel_angle, rate, dt)
        last_time = sample[0]
    return filt.angle


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
        # Fusion.  tau is the crossover: shorter trusts the gyroscope
        # further, which tracks a moving head better but lets the
        # gyroscope's zero-rate offset through; longer trusts the
        # accelerometer further.  It is the knob worth tuning on the
        # machine, and the default is a starting point, not a result.
        self.fusion = config.getboolean('fusion', True)
        self.fusion_tau = config.getfloat('fusion_tau', .2, above=0.)
        # How far the fused angle may sit from a trailing accelerometer
        # average before the measurement is refused.  Both estimate the
        # angle *now*, so this catches a gyroscope that is inverted,
        # mis-scaled or on the wrong axis - failures that would
        # otherwise bias every measurement silently.
        self.max_fusion_disagreement = config.getfloat(
            'max_fusion_disagreement', 5., minval=0.)
        self.gyro_coefficient, self.gyro_index = gyro_axis_coefficient(
            self.zero_axis, self.positive_axis)
        # Generous by default, because the reading is uncalibrated - see
        # the header comment.
        self.check_tolerance = config.getfloat('check_tolerance', 5.,
                                               above=0.)
        # Picking the B homing direction.  The tolerance is the band
        # around position_endstop inside which the uncalibrated reading
        # cannot say which side the head is on, the extra sweep past the
        # measured distance, and the allowed error of the post-home check.
        self.homing_tolerance = config.getfloat('homing_tolerance', 5.,
                                                above=0.)
        self.verify_home_enabled = config.getboolean('verify_home', True)
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
        set_source = getattr(self.b_axis, 'set_homing_direction_source', None)
        if set_source is not None:
            set_source(self)

    ######################################################################
    # The measurement primitive
    ######################################################################
    def measure(self, settle_time=None, sample_time=None, fusion=None):
        settle = self.settle_time if settle_time is None else settle_time
        window = self.sample_time if sample_time is None else sample_time
        toolhead = self.toolhead
        fuse = self.fusion if fusion is None else fusion
        fuse = bool(fuse) and self.has_gyro
        toolhead.wait_moves()
        # Fusing reads the combined stream, where each sample carries an
        # acceleration and a rotation rate the chip measured at the same
        # instant.  Without fusion the two split views are enough, and
        # the gyroscope one is only needed for the gate.
        imu_client = client = gyro_client = None
        if fuse:
            imu_client = self.chip.start_internal_imu_client()
        else:
            client = self.chip.start_internal_client()
            if self.has_gyro and self.max_rotation_rate:
                gyro_client = self.chip.start_internal_gyro_client()
        start_time = toolhead.get_last_move_time()
        # The averaging window is settle..settle+window; the extra margin
        # is only there to let the batches carrying it arrive
        toolhead.dwell(settle + window + self.batch_margin)
        clients = [c for c in (imu_client, client, gyro_client)
                   if c is not None]
        for c in clients:
            c.finish_measurements()
        lost = max([overflows_during(c) for c in clients])
        if lost:
            raise self.printer.command_error(
                "%s: '%s' reported %d possible fifo overflows during the"
                " measurement - the link to the chip cannot keep up with"
                " its data rate, so samples were lost and the rest are"
                " mistimed.  Lower the chip's rate, or on i2c raise the bus"
                " speed (on a Raspberry Pi host that is"
                " dtparam=i2c_arm_baudrate in config.txt; i2c_speed is"
                " ignored there)" % (self.name, self.chip_name, lost))
        # AccelQueryHelper trims to the request window but knows nothing
        # about the settle dwell, so drop that part here.  Samples are
        # (print_time, x, y, z); index rather than name the fields, so
        # any chip exposing start_internal_client() works.
        first = start_time + settle
        last = first + window
        if fuse:
            imu_samples = imu_client.get_samples()
            accel_samples = [(s[0], s[4], s[5], s[6]) for s in imu_samples]
            gyro_samples = [(s[0], s[1], s[2], s[3]) for s in imu_samples]
        else:
            imu_samples = None
            accel_samples = client.get_samples()
            gyro_samples = (None if gyro_client is None
                            else gyro_client.get_samples())
        vectors = [(s[1], s[2], s[3]) for s in accel_samples
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
        if gyro_samples is not None and self.max_rotation_rate:
            rates = [magnitude((s[1], s[2], s[3])) for s in gyro_samples
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
        accel_angle = measure_angle(mean, self.zero_axis, self.positive_axis)
        # The fused angle runs over the whole capture, settle included -
        # with the gyroscope carrying the short timescales, the settle
        # dwell stops being "wait for the head to stop" and becomes
        # "give the filter time to converge".
        fused_angle = disagreement = None
        if fuse:
            # ...but it stops at the end of the window, not the end of
            # the capture: the trailing batch_margin is delivery slack
            # that the user did not ask to measure, and running past it
            # would put the fused angle and the accelerometer tail it is
            # checked against at different instants.
            imu_samples = [s for s in imu_samples if s[0] <= last]
            if len(imu_samples) < 2:
                raise self.printer.command_error(
                    "%s: no combined imu samples in the measurement"
                    " window - check that '%s' is responding (try"
                    " BMI160_QUERY)" % (self.name, self.chip_name))
            span = imu_samples[-1][0] - imu_samples[0][0]
            if span < MIN_FUSION_SPANS * self.fusion_tau:
                raise self.printer.command_error(
                    "%s: the %.3f s capture is shorter than %g x fusion_tau"
                    " (%.3f s), so the complementary filter would report"
                    " its own starting value - raise settle_time or"
                    " sample_time, or lower fusion_tau"
                    % (self.name, span, MIN_FUSION_SPANS,
                       MIN_FUSION_SPANS * self.fusion_tau))
            fused_angle = fuse_samples(
                imu_samples, self.zero_axis, self.positive_axis,
                self.gyro_coefficient, self.gyro_index, self.fusion_tau)
            # Check it against the accelerometer's own view of the *end*
            # of the window rather than the whole of it: both estimate
            # the angle now, so they agree when the gyroscope is right
            # and diverge when its sign, scale or axis is wrong.  A
            # stationary head has nothing to integrate and so proves
            # nothing either way - that needs a deliberate move.
            tail_start = last - FUSION_TAIL_FRACTION * window
            tail = [(s[1], s[2], s[3]) for s in accel_samples
                    if tail_start <= s[0] <= last]
            if tail:
                tail_mean, _ = summarize(tail)
                tail_angle = measure_angle(tail_mean, self.zero_axis,
                                           self.positive_axis)
                disagreement = wrap180(fused_angle - tail_angle)
                if (self.max_fusion_disagreement
                    and abs(disagreement) > self.max_fusion_disagreement):
                    raise self.printer.command_error(
                        "%s: the fused angle is %.3f deg where the"
                        " accelerometer alone reads %.3f (difference"
                        " %+.3f, tolerance %.3f).  Either the head was"
                        " still moving, or the gyroscope is inverted,"
                        " mis-scaled or on the wrong axis - check"
                        " zero_vector, positive_vector and the chip's"
                        " axes_map."
                        % (self.name, fused_angle, tail_angle, disagreement,
                           self.max_fusion_disagreement))
        reading = TiltReading(
            angle=accel_angle if fused_angle is None else fused_angle,
            accel_angle=accel_angle, fused_angle=fused_angle,
            disagreement=disagreement,
            vector=mean, deviation=dev, magnitude=mag,
            u=project(mean, self.positive_axis),
            w=project(mean, self.zero_axis),
            out_of_plane=mean[self.oop_index], count=len(vectors),
            rotation_rate=rotation_rate)
        self.last_reading = reading
        return reading

    ######################################################################
    # Homing direction (called by rotary_axis.BaseRotaryAxis.home)
    ######################################################################
    def _respond(self, msg):
        logging.info(msg)
        self.printer.lookup_object('gcode').respond_info(msg)
    def choose_homing_direction(self, position_endstop, position_min,
                                position_max):
        reading = self.measure()
        angle, tol = reading.angle, self.homing_tolerance
        distance = position_endstop - angle
        if abs(distance) > tol:
            positive_dir = distance > 0.
        elif position_endstop >= position_max:
            positive_dir = True
        elif position_endstop <= position_min:
            positive_dir = False
        else:
            raise self.printer.command_error(
                "%s: the head measures B = %.2f deg, within %.2f of the"
                " endstop at %.2f, so it cannot tell which side of the"
                " endstop it is on.  Turn the head away from the endstop"
                " by hand (M84 first) and home again, or set"
                " homing_positive_dir in [stepper_tilt]"
                % (self.name, angle, tol, position_endstop))
        self._respond("%s: head at B = %.2f deg, homing %s toward the"
                      " endstop at %.2f"
                      % (self.name, angle,
                         "positive" if positive_dir else "negative",
                         position_endstop))
        return positive_dir, abs(distance) + tol
    def verify_home(self, position_endstop):
        if not self.verify_home_enabled:
            return
        reading = self.measure()
        error = wrap180(reading.angle - position_endstop)
        if abs(error) > self.homing_tolerance:
            raise self.printer.command_error(
                "%s: B homed, but the head measures %.2f deg where the"
                " endstop is at %.2f (error %+.2f, tolerance %.2f).  Either"
                " the home triggered before the head moved (a sensorless"
                " endstop needs G4 P2000 before G28 B), or positive_vector"
                " and the motors disagree about which way is +B - check"
                " positive_vector, then invert_b_direction in [printer]"
                % (self.name, reading.angle, position_endstop, error,
                   self.homing_tolerance))

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
        res['fusion'] = self.fusion and self.has_gyro
        res['fusion_tau'] = self.fusion_tau
        res['rotation_axis_sign'] = self.gyro_coefficient
        reading = self.last_reading
        res['measured_b'] = None if reading is None else reading.angle
        res['rotation_rate'] = (None if reading is None
                                else reading.rotation_rate)
        res['fusion_disagreement'] = (None if reading is None
                                      else reading.disagreement)
        return res
    cmd_B_MEASURE_help = "Measure the B axis angle against gravity"
    def cmd_B_MEASURE(self, gcmd):
        settle = gcmd.get_float('SETTLE', self.settle_time, minval=0.)
        window = gcmd.get_float('SAMPLE_TIME', self.sample_time, minval=.05)
        fusion = gcmd.get_int('FUSION', None, minval=0, maxval=1)
        reading = self.measure(settle, window,
                               None if fusion is None else bool(fusion))
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
        if reading.fused_angle is not None:
            lines.append(
                "  fused %.3f deg, accelerometer alone %.3f, tail"
                " difference %+.3f (tau %.3f s)"
                % (reading.fused_angle, reading.accel_angle,
                   reading.disagreement
                   if reading.disagreement is not None else float('nan'),
                   self.fusion_tau))
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
