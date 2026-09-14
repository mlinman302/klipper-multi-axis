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
# This module is the measurement primitive of docs/Accel_B_Homing.md,
# the B_MEASURE command that reports it, and the G28 B home built on it:
# see "HOMING" below.  B_MEASURE never moves the machine.
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
# See docs/BMI160_IMU.md.
#
# WHICH ANGLE IS ACTED ON
#
# measure() produces two angles from one capture, and a TiltReading
# carries both under their own names - there is deliberately no field
# that means "whichever one was available".  Everything that acts on B -
# G28 B, the endstop direction and its verification, B_MEASURE CHECK=1
# and the drive ratio calibration - reads the head through
# measure_vertical(), which only ever returns the fused angle from
# vertical and refuses when there is none.  The accelerometer-only angle
# is a diagnostic that B_MEASURE prints beside it: it cannot tell a
# tilted head from an accelerating one, so acting on it would book a
# head still ringing on its belts as a static tilt.  A chip without a
# gyroscope can still run B_MEASURE, but cannot home B.
#
# HOMING
#
# The corertheta B axis has no endstop by default.  Its position_min and
# position_max are soft limits - there to keep the filament tube from
# kinking, not physical stops - and G28 B asks this module to home it
# (see home_axis()): energise the gantry motors, measure the head's
# fused angle from vertical, book it as B, and drive it to B=0 (nozzle
# vertical), measuring and correcting until it is within zero_tolerance.
# The first move is
# capped at direction_check_move and checked, so a positive_vector that
# disagrees with the motors about which way is +B stops the home a few
# degrees in instead of driving the head through a soft limit.
#
# A [stepper_tilt] with an endstop_pin still homes to it.  The rail does
# not guess the direction from where position_endstop sits in the range
# (see stepper.py's infer_homing_dir): with homing_positive_dir unset the
# head is measured where it rests, and homes positive if that is below
# position_endstop, negative if above.  Within homing_tolerance of the
# endstop the measurement cannot tell the sides apart, so an endstop at
# a range limit is homed toward that limit and one inside the range is
# refused.  After the home the head is measured again, and a head that
# is not at the endstop leaves B unhomed.
#
# SENSOR CALIBRATION
#
# An accelerometer's zero-g offset is up to +/-150 mg and its axes'
# sensitivities differ by several percent.  Near B = 0 the angle is
# almost entirely u / w, so an offset along u moves the zero directly:
# 100 mg is 5.7 degrees.  Every sample is therefore corrected before
# anything else sees it:
#
#   u' = u - offset_u
#   w' = (w - offset_w) / gain_ratio
#
# and B = atan2(u', w').  The three numbers are fitted, not declared, by
# B_SENSOR_CALIBRATE: it turns the head through as much of its range as
# it has, measures the raw mean at each station, and fits the ellipse
# the in-plane readings trace,
#
#   (u - offset_u)^2 + (w - offset_w)^2 / gain_ratio^2 = A^2
#
# as a linear least squares problem.  The fit needs only that gravity is
# the same size in every pose - never the true angle of any station - so
# neither the drive ratio nor a sensor glued on slightly rotated leaks
# into it.  That also means it cannot see a rotated sensor: a rotation
# keeps the circle a circle.  The offset along the rotation axis does
# not enter the angle and is not fitted.
#
# How much the data can support depends on the arc it covers, so the
# model is tiered: an arc of FULL_FIT_ARC or more fits both offsets and
# the gain ratio, one of REDUCED_FIT_ARC or more fits the offsets with
# the gain ratio held at 1, and anything shorter is refused.
# Repeatability was always much better than accuracy - a stationary head
# re-measures to a few hundredths of a degree - and the calibration is
# what closes the gap.
#
# DRIVE RATIO CALIBRATION
#
# B_STEP_CALIBRATE answers how far the head really turns per step.  It
# sweeps B through a row of stations and at each one records two angles:
# the one the integer step counters imply (axis.get_step_position(), on
# corertheta the sum of both gantry motors, so the radius cancels) and
# the fused angle from vertical.  A drive ratio that is off by a factor
# makes the head turn that factor per commanded degree, so the fit is a
# straight line,
#
#   measured = scale * step_angle + offset
#
# and scale is the correction: b_coupling_ratio (belt travel per degree)
# is divided by it, a dedicated stepper's rotation_distance (degrees per
# revolution) multiplied.  Neither the zero of the sensor nor that of the
# step counters matters - both land in the offset.  The sensor's gain and
# offsets do matter, since they bend the angle non-linearly, so the
# sensor must be calibrated first.
#
# Every station is approached from the same side, after an over-travel
# of STEP_TAKEUP, so backlash is a constant in the offset rather than
# noise in the scale.  RETURN=1 sweeps back the other way too, and the two
# legs are fitted with one common scale and an offset each: the
# difference of those offsets is the backlash.
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

# Homing without an endstop.  A move shorter than this says nothing
# reliable about which way the head went, so it neither confirms the
# direction nor refuses it.
MIN_DIRECTION_CHECK = 1.
# How far the head's response to that first move may stray from what was
# commanded before it is refused - measured / commanded outside these
# bounds is not a b_coupling_ratio error, it is a head that is not
# following the motors.
MIN_RESPONSE_RATIO = .5
MAX_RESPONSE_RATIO = 2.

# Sensor calibration.  The arc a sweep must measurably cover before each
# model is fitted - see "SENSOR CALIBRATION" in the header - and the
# fewest stations each needs to leave at least one degree of freedom
# over its unknowns.
FULL_FIT_ARC = 120.
REDUCED_FIT_ARC = 60.
MIN_FULL_FIT_STATIONS = 5
MIN_REDUCED_FIT_STATIONS = 4
# Bounds on a fit worth believing.  The datasheets allow 150 mg of
# offset and 10 % of sensitivity error, so a fit well outside these is
# describing a head that moved, or a mounting that is not the declared
# one, rather than the sensor.
MAX_FIT_OFFSET = .3 * FREEFALL_ACCEL
MIN_GAIN_RATIO = .8
MAX_GAIN_RATIO = 1.25
MIN_FIT_RADIUS = .8 * FREEFALL_ACCEL
MAX_FIT_RADIUS = 1.2 * FREEFALL_ACCEL
# B_SENSOR_CALIBRATE measures longer and more patiently than a home: the
# offsets it fits are worth degrees, so the noise on them should not be
CALIBRATE_SETTLE_TIME = 1.
CALIBRATE_SAMPLE_TIME = 1.

# Drive ratio calibration.  The over-travel before each leg of a sweep,
# in degrees - far more than belt backlash - so every station is
# approached from the same side.
STEP_TAKEUP = 5.
# The ratio error is roughly the angle noise divided by the arc, so a
# short arc is refused rather than fitted
MIN_STEP_ARC = 20.
MIN_STEP_STATIONS = 3
# Above this the residuals are worth a warning - see "Reading the
# residuals" in docs/Accel_B_Homing.md
STEP_RESIDUAL_WARNING = .2

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

# fused_angle is None when the reading was not fused.  Nothing that acts
# on B may fall back to accel_angle - see measure_vertical().
TiltReading = collections.namedtuple('TiltReading', (
    'accel_angle', 'fused_angle', 'disagreement', 'vector',
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

# The sensor calibration as (offset_u, offset_w, gain_ratio).  Both
# directions work on sensor-frame vectors, so everything downstream of
# the correction - the angle, the gates, the report - is unchanged.
IDENTITY_CALIBRATION = (0., 0., 1.)

def correct_vector(vector, zero_axis, positive_axis, calibration):
    offset_u, offset_w, gain_ratio = calibration
    (w_index, w_sign), (u_index, u_sign) = zero_axis, positive_axis
    res = list(vector)
    res[u_index] = vector[u_index] - u_sign * offset_u
    res[w_index] = (vector[w_index] - w_sign * offset_w) / gain_ratio
    return tuple(res)

def raw_vector(vector, zero_axis, positive_axis, calibration):
    # The inverse of correct_vector().  It is linear, so it recovers a raw
    # mean from a corrected one exactly.
    offset_u, offset_w, gain_ratio = calibration
    (w_index, w_sign), (u_index, u_sign) = zero_axis, positive_axis
    res = list(vector)
    res[u_index] = vector[u_index] + u_sign * offset_u
    res[w_index] = vector[w_index] * gain_ratio + w_sign * offset_w
    return tuple(res)

def solve_linear(matrix, rhs):
    # Gaussian elimination with partial pivoting, for the few unknowns of
    # the fits below - numpy is not guaranteed on the host
    n = len(rhs)
    rows = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-12:
            raise ValueError("singular system")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        for r in range(col + 1, n):
            factor = rows[r][col] / rows[col][col]
            for c in range(col, n + 1):
                rows[r][c] -= factor * rows[col][c]
    res = [0.] * n
    for r in range(n - 1, -1, -1):
        known = sum([rows[r][c] * res[c] for c in range(r + 1, n)])
        res[r] = (rows[r][n] - known) / rows[r][r]
    return res

def least_squares(design, rhs):
    # Through the normal equations - well enough conditioned here, where
    # the columns are scaled to 1 g and there are at most four of them
    n = len(design[0])
    ata = [[sum([row[i] * row[j] for row in design]) for j in range(n)]
           for i in range(n)]
    atb = [sum([row[i] * b for row, b in zip(design, rhs)])
           for i in range(n)]
    return solve_linear(ata, atb)

def fit_ellipse(points, fit_gain):
    # Fits (u - u0)^2 + (w - w0)^2 / g^2 = A^2 to raw (u, w) points.
    # With fit_gain the linear model is  u^2 + b w^2 + c u + d w + e = 0,
    # without it b is held at 1.  Completing the squares gives
    #   u0 = -c / 2,  w0 = -d / 2b,  A^2 = u0^2 + b w0^2 - e,  g = 1/sqrt(b)
    # Returns (u0, w0, g, A) in the units of the points.
    scale = FREEFALL_ACCEL
    pts = [(u / scale, w / scale) for u, w in points]
    if fit_gain:
        b, c, d, e = least_squares([(w * w, u, w, 1.) for u, w in pts],
                                   [-u * u for u, w in pts])
        if b <= 0.:
            raise ValueError("the points do not lie on an ellipse")
    else:
        b = 1.
        c, d, e = least_squares([(u, w, 1.) for u, w in pts],
                                [-(u * u + w * w) for u, w in pts])
    u0, w0 = -c / 2., -d / (2. * b)
    radius_sq = u0 * u0 + b * w0 * w0 - e
    if radius_sq <= 0.:
        raise ValueError("the points do not lie on an ellipse")
    return (u0 * scale, w0 * scale, 1. / math.sqrt(b),
            math.sqrt(radius_sq) * scale)

def fit_drive_sweep(points):
    # points are (step_angle, measured_angle, leg), leg 0 for the outbound
    # sweep and 1 for the return.  Fits
    #   measured = scale * step_angle + offsets[leg]
    # with one scale shared by every leg.  Returns (scale, offsets,
    # residuals), residuals in the order of the points.
    legs = sorted(set([leg for t, phi, leg in points]))
    column = dict((leg, i) for i, leg in enumerate(legs))
    design = []
    for t, phi, leg in points:
        row = [t] + [0.] * len(legs)
        row[1 + column[leg]] = 1.
        design.append(row)
    if len(points) < len(design[0]):
        raise ValueError("too few stations to fit")
    solution = least_squares(design, [phi for t, phi, leg in points])
    scale = solution[0]
    offsets = dict((leg, solution[1 + column[leg]]) for leg in legs)
    residuals = [phi - (scale * t + offsets[leg]) for t, phi, leg in points]
    return scale, offsets, residuals

def unwrap_angles(angles):
    # Angles in visiting order, unwrapped so a sweep through +/-180 stays
    # continuous
    res = []
    for angle in angles:
        res.append(angle if not res else res[-1] + wrap180(angle - res[-1]))
    return res

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
        # There is no option to turn fusion off: the fused angle is the
        # only one B is homed, checked or calibrated on.
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
        # Homing without an endstop (the default).  The head is driven to
        # B=0 and re-measured until it is within zero_tolerance, using at
        # most max_homing_moves moves.  The first of them is no longer
        # than direction_check_move, because until the head has been seen
        # to follow the motors a move toward zero might be a move away
        # from it - toward a soft limit and a kinked filament tube.
        self.zero_tolerance = config.getfloat('zero_tolerance', .25,
                                              above=0.)
        self.max_homing_moves = config.getint('max_homing_moves', 5,
                                              minval=1)
        self.direction_check_move = config.getfloat(
            'direction_check_move', 5., minval=MIN_DIRECTION_CHECK)
        # Homing to an endstop, when [stepper_tilt] has one.  The
        # tolerance is the band around position_endstop inside which the
        # uncalibrated reading cannot say which side the head is on, the
        # extra sweep past the measured distance, and the allowed error of
        # the post-home check.
        self.homing_tolerance = config.getfloat('homing_tolerance', 5.,
                                                above=0.)
        self.verify_home_enabled = config.getboolean('verify_home', True)
        # The sensor calibration, written by B_SENSOR_CALIBRATE.  The
        # identity is the uncalibrated sensor.
        self.calibration = (
            config.getfloat('offset_u', 0.),
            config.getfloat('offset_w', 0.),
            config.getfloat('gain_ratio', 1., minval=MIN_GAIN_RATIO,
                            maxval=MAX_GAIN_RATIO))
        self.chip = self.toolhead = self.b_axis = None
        self.has_gyro = False
        self.b_projection = None
        self.last_reading = None
        self.last_fused_angle = None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('B_MEASURE', self.cmd_B_MEASURE,
                               desc=self.cmd_B_MEASURE_help)
        gcode.register_command('B_SENSOR_CALIBRATE',
                               self.cmd_B_SENSOR_CALIBRATE,
                               desc=self.cmd_B_SENSOR_CALIBRATE_help)
        gcode.register_command('B_STEP_CALIBRATE',
                               self.cmd_B_STEP_CALIBRATE,
                               desc=self.cmd_B_STEP_CALIBRATE_help)
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
        set_source = getattr(self.b_axis, 'set_homing_source', None)
        if set_source is not None:
            set_source(self)

    ######################################################################
    # The measurement primitive
    ######################################################################
    def measure(self, settle_time=None, sample_time=None, fusion=True):
        # fusion=False is B_MEASURE FUSION=0, the diagnostic that compares
        # the two estimators.  Nothing that acts on B passes it.
        settle = self.settle_time if settle_time is None else settle_time
        window = self.sample_time if sample_time is None else sample_time
        toolhead = self.toolhead
        fuse = bool(fusion) and self.has_gyro
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
                " mistimed.  Lower the chip's rate.  On spi the wire is"
                " rarely the limit, so look for a loaded host; on i2c"
                " also check the bus speed (on a Raspberry Pi host that"
                " is dtparam=i2c_arm_baudrate in config.txt; i2c_speed is"
                " ignored there)" % (self.name, self.chip_name, lost))
        # AccelQueryHelper trims to the request window but knows nothing
        # about the settle dwell, so drop that part here.  Samples are
        # (print_time, x, y, z); index rather than name the fields, so
        # any chip exposing start_internal_client() works.
        first = start_time + settle
        last = first + window
        # The sensor calibration is applied to every sample, before the
        # gates, the average and the fusion see any of them
        calibration = self.calibration
        if calibration == IDENTITY_CALIBRATION:
            correct = None
        else:
            correct = lambda v: correct_vector(v, self.zero_axis,
                                               self.positive_axis,
                                               calibration)
        if fuse:
            imu_samples = imu_client.get_samples()
            if correct is not None:
                imu_samples = [tuple(s[:4]) + correct(s[4:7])
                               for s in imu_samples]
            accel_samples = [(s[0], s[4], s[5], s[6]) for s in imu_samples]
            gyro_samples = [(s[0], s[1], s[2], s[3]) for s in imu_samples]
        else:
            imu_samples = None
            accel_samples = client.get_samples()
            if correct is not None:
                accel_samples = [(s[0],) + correct(s[1:4])
                                 for s in accel_samples]
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
            accel_angle=accel_angle, fused_angle=fused_angle,
            disagreement=disagreement,
            vector=mean, deviation=dev, magnitude=mag,
            u=project(mean, self.positive_axis),
            w=project(mean, self.zero_axis),
            out_of_plane=mean[self.oop_index], count=len(vectors),
            rotation_rate=rotation_rate)
        self.last_reading = reading
        if fused_angle is not None:
            self.last_fused_angle = fused_angle
        return reading

    ######################################################################
    # The angle from vertical
    ######################################################################
    def measure_vertical(self, settle_time=None, sample_time=None):
        # The head's angle from vertical, in degrees - always the fused
        # one.  This is the only measurement G28 B, the endstop checks,
        # CHECK=1 and the drive ratio calibration act on; see "WHICH
        # ANGLE IS ACTED ON" in the header.
        if not self.has_gyro:
            raise self.printer.command_error(
                "%s: '%s' has no gyroscope, so there is no fused angle to"
                " measure B from vertical with.  B is only homed, checked"
                " and calibrated on the fused angle - use an IMU such as a"
                " [bmi160] with its gyroscope enabled.  B_MEASURE still"
                " reports the accelerometer-only angle"
                % (self.name, self.chip_name))
        reading = self.measure(settle_time, sample_time, fusion=True)
        if reading.fused_angle is None:
            raise self.printer.command_error(
                "%s: the measurement was not fused, so it cannot be used"
                " as B" % (self.name,))
        return reading.fused_angle

    ######################################################################
    # Homing without an endstop (called by rotary_axis.BaseRotaryAxis.home)
    ######################################################################
    def _respond(self, msg):
        logging.info(msg)
        self.printer.lookup_object('gcode').respond_info(msg)
    def _energise(self, axis):
        # A head left limp would be measured where it droops and then
        # jump when the motors took hold, so hold it before measuring.
        # The settle dwell of the measurement covers the jump.
        stepper_enable = self.printer.lookup_object('stepper_enable', None)
        if stepper_enable is None:
            return
        stepper_enable.set_motors_enable(
            [s.get_name() for s in axis.get_drive_steppers()], True)
    def _check_response(self, before, after, commanded):
        moved = wrap180(after - before)
        ratio = moved / commanded
        if MIN_RESPONSE_RATIO <= ratio <= MAX_RESPONSE_RATIO:
            return
        if ratio <= -MIN_RESPONSE_RATIO:
            hint = ("positive_vector and the motors disagree about which way"
                    " is +B - check positive_vector, then invert_b_direction"
                    " in [printer]")
        elif ratio > MAX_RESPONSE_RATIO:
            hint = ("the head turned much further than commanded - check"
                    " b_coupling_ratio in [printer]")
        else:
            hint = ("the head is not following the motors - check that they"
                    " are driving it, and that the sensor is on the rotating"
                    " part of the head")
        raise self.printer.command_error(
            "%s: B was commanded to turn %+.2f deg and the head turned %+.2f"
            " (from %.2f to %.2f), so homing stopped before driving it any"
            " further: %s" % (self.name, commanded, moved, before, after,
                              hint))
    def home_axis(self, axis):
        # Measure the head, book the measurement as B, and drive to B=0.
        # A wrong b_coupling_ratio lands the move short or long, so
        # measure again and repeat until the head is within
        # zero_tolerance.  The first move is capped at
        # direction_check_move and its result checked, so a head that
        # turns the wrong way is caught a few degrees in rather than
        # after it has been driven through a soft limit.
        self._energise(axis)
        pos_min, pos_max = axis.get_range()
        angle = self.measure_vertical()
        axis.set_measured_position(angle)
        note = ""
        if angle < pos_min or angle > pos_max:
            note = (" - outside the soft limits of %.2f to %.2f, so check the"
                    " filament tube" % (pos_min, pos_max))
        self._respond("%s: head at B = %.2f deg, homing to B = 0%s"
                      % (self.name, angle, note))
        confirmed = False
        moves = 0
        while abs(angle) > self.zero_tolerance:
            if moves >= self.max_homing_moves:
                raise self.printer.command_error(
                    "%s: the head is still at B = %.2f deg after %d moves"
                    " toward B = 0 (tolerance %.2f) - the moves are not"
                    " converging, so check b_coupling_ratio in [printer],"
                    " or raise zero_tolerance if the head is settling"
                    " somewhere slightly different each time"
                    % (self.name, angle, moves, self.zero_tolerance))
            if confirmed:
                target = 0.
            else:
                step = min(abs(angle), self.direction_check_move)
                target = angle - math.copysign(step, angle)
                # A head measured beyond a soft limit cannot be commanded
                # to stay beyond it, so its first move goes to that limit
                target = min(max(target, pos_min), pos_max)
            axis.move_axis(target)
            moves += 1
            new_angle = self.measure_vertical()
            commanded = target - angle
            if not confirmed and abs(commanded) >= MIN_DIRECTION_CHECK:
                self._check_response(angle, new_angle, commanded)
                confirmed = True
            angle = new_angle
            axis.set_measured_position(angle)
        # B now reads the measured angle, within zero_tolerance of zero.
        # Finish on a commanded zero, so the transforms switched on after
        # homing see the nozzle vertical.
        if angle:
            axis.move_axis(0.)
        self._respond("%s: B homed, head measured at %.2f deg after %d"
                      " moves" % (self.name, angle, moves))

    ######################################################################
    # Homing to an endstop (called by rotary_axis.BaseRotaryAxis.home)
    ######################################################################
    def choose_homing_direction(self, position_endstop, position_min,
                                position_max):
        angle, tol = self.measure_vertical(), self.homing_tolerance
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
        angle = self.measure_vertical()
        error = wrap180(angle - position_endstop)
        if abs(error) > self.homing_tolerance:
            raise self.printer.command_error(
                "%s: B homed, but the head measures %.2f deg where the"
                " endstop is at %.2f (error %+.2f, tolerance %.2f).  Either"
                " the home triggered before the head moved (a sensorless"
                " endstop needs G4 P2000 before G28 B), or positive_vector"
                " and the motors disagree about which way is +B - check"
                " positive_vector, then invert_b_direction in [printer]"
                % (self.name, angle, position_endstop, error,
                   self.homing_tolerance))

    ######################################################################
    # Sensor calibration
    ######################################################################
    def sweep_sensor(self, axis, stations, settle_time, sample_time):
        # The raw mean vector at each commanded station, in visiting
        # order.  Unfused: the head is parked, and it is the average that
        # is fitted.  The measurement is corrected by whatever calibration
        # is loaded, so undo that - exactly, since it is linear.
        res = []
        for commanded in stations:
            axis.move_axis(commanded)
            reading = self.measure(settle_time, sample_time, fusion=False)
            res.append((commanded, raw_vector(
                reading.vector, self.zero_axis, self.positive_axis,
                self.calibration)))
        return res
    def fit_sweep(self, sweep, fit_gain=True):
        # Fits the sensor calibration to a sweep, refusing one that does
        # not describe the declared mounting.  Returns (model, calibration,
        # radius, residuals, arc).
        error = self.printer.command_error
        commanded = [c for c, v in sweep]
        vectors = [v for c, v in sweep]
        # The rotation axis is the one that should stay put.  If another
        # axis varied less, the two vectors name the wrong pair, and an
        # ellipse fitted to them would be fitted to nothing.
        ranges = [max([v[i] for v in vectors]) - min([v[i] for v in vectors])
                  for i in range(3)]
        steadiest = ranges.index(min(ranges))
        if steadiest != self.oop_index:
            raise error(
                "%s: over the sweep the %s axis varied least (%.0f mm/s^2)"
                " but zero_vector and positive_vector leave %s as the"
                " rotation axis (varied %.0f) - they do not name the two"
                " axes the head turns through.  Axis ranges x/y/z ="
                " %.0f / %.0f / %.0f"
                % (self.name, AXIS_NAMES[steadiest], ranges[steadiest],
                   AXIS_NAMES[self.oop_index], ranges[self.oop_index],
                   ranges[0], ranges[1], ranges[2]))
        angles = unwrap_angles([measure_angle(v, self.zero_axis,
                                              self.positive_axis)
                                for v in vectors])
        turned = angles[-1] - angles[0]
        if turned * (commanded[-1] - commanded[0]) < 0.:
            raise error(
                "%s: B was swept from %.1f to %.1f deg but the head turned"
                " %+.1f - positive_vector and the motors disagree about"
                " which way is +B" % (self.name, commanded[0], commanded[-1],
                                      turned))
        arc = max(angles) - min(angles)
        if (fit_gain and arc >= FULL_FIT_ARC
            and len(sweep) >= MIN_FULL_FIT_STATIONS):
            model = 'full'
        elif arc >= REDUCED_FIT_ARC and len(sweep) >= MIN_REDUCED_FIT_STATIONS:
            model = 'offsets only'
        else:
            raise error(
                "%s: the sweep covered %.1f deg over %d stations, and fitting"
                " the offsets needs at least %.0f deg and %d stations -"
                " widen START/END or add STEPS"
                % (self.name, arc, len(sweep), REDUCED_FIT_ARC,
                   MIN_REDUCED_FIT_STATIONS))
        points = [(project(v, self.positive_axis), project(v, self.zero_axis))
                  for v in vectors]
        try:
            offset_u, offset_w, gain_ratio, radius = fit_ellipse(
                points, model == 'full')
        except ValueError as e:
            raise error("%s: the sensor fit failed: %s - the head was likely"
                        " moving during the sweep" % (self.name, e))
        problems = []
        if max(abs(offset_u), abs(offset_w)) > MAX_FIT_OFFSET:
            problems.append("offsets u/w of %.0f / %.0f mm/s^2 (limit %.0f)"
                            % (offset_u, offset_w, MAX_FIT_OFFSET))
        if not MIN_GAIN_RATIO <= gain_ratio <= MAX_GAIN_RATIO:
            problems.append("a gain ratio of %.3f (limits %.2f to %.2f)"
                            % (gain_ratio, MIN_GAIN_RATIO, MAX_GAIN_RATIO))
        if not MIN_FIT_RADIUS <= radius <= MAX_FIT_RADIUS:
            problems.append("an in-plane radius of %.0f mm/s^2 (limits %.0f"
                            " to %.0f) - is the B rotation axis horizontal?"
                            % (radius, MIN_FIT_RADIUS, MAX_FIT_RADIUS))
        if problems:
            raise error("%s: the sensor fit is not believable - it found %s."
                        "  Nothing was changed" % (self.name,
                                                   ", and ".join(problems)))
        calibration = (offset_u, offset_w, gain_ratio)
        residuals = [math.hypot(u - offset_u, (w - offset_w) / gain_ratio)
                     - radius for u, w in points]
        return model, calibration, radius, residuals, arc
    def calibrate_sensor(self, axis, start, end, steps, settle_time,
                         sample_time, fit_gain=True):
        pos_min, pos_max = axis.get_range()
        for name, value in (('START', start), ('END', end)):
            if not pos_min <= value <= pos_max:
                raise self.printer.command_error(
                    "%s: %s=%.2f is outside the soft limits of %.2f to %.2f"
                    % (self.name, name, value, pos_min, pos_max))
        if abs(end - start) < REDUCED_FIT_ARC:
            raise self.printer.command_error(
                "%s: a sweep from %.2f to %.2f cannot be fitted - it needs to"
                " cover at least %.0f deg" % (self.name, start, end,
                                              REDUCED_FIT_ARC))
        stations = [start + (end - start) * i / (steps - 1.)
                    for i in range(steps)]
        self._energise(axis)
        sweep = self.sweep_sensor(axis, stations, settle_time, sample_time)
        try:
            model, calibration, radius, residuals, arc = self.fit_sweep(
                sweep, fit_gain)
        except self.printer.command_error:
            axis.move_axis(0.)
            raise
        old = self.calibration
        # How far the head that measured B = 0 before is from B = 0 now,
        # which is how far the next home will turn it
        vertical = [0., 0., 0.]
        vertical[self.zero_axis[0]] = self.zero_axis[1] * FREEFALL_ACCEL
        zero_shift = measure_angle(
            correct_vector(raw_vector(vertical, self.zero_axis,
                                      self.positive_axis, old),
                           self.zero_axis, self.positive_axis, calibration),
            self.zero_axis, self.positive_axis)
        lines = ["%s: sensor calibration from B = %.1f to %.1f, %d stations"
                 " covering %.1f deg - %s fit"
                 % (self.name, start, end, steps, arc, model)]
        for (commanded, vector), residual in zip(sweep, residuals):
            before = measure_angle(
                correct_vector(vector, self.zero_axis, self.positive_axis,
                               old), self.zero_axis, self.positive_axis)
            after = measure_angle(
                correct_vector(vector, self.zero_axis, self.positive_axis,
                               calibration),
                self.zero_axis, self.positive_axis)
            lines.append(
                "  B %7.2f: raw u/w = %.1f / %.1f, out of plane %.1f,"
                " measured %.2f -> %.2f deg, radius residual %+.1f"
                % (commanded, project(vector, self.positive_axis),
                   project(vector, self.zero_axis), vector[self.oop_index],
                   before, after, residual))
        rms = math.sqrt(sum([r * r for r in residuals]) / len(residuals))
        lines += [
            "  offset_u = %.1f mm/s^2 (was %.1f), offset_w = %.1f (was %.1f),"
            " gain_ratio = %.5f (was %.5f)"
            % (calibration[0], old[0], calibration[1], old[1],
               calibration[2], old[2]),
            "  in-plane radius %.1f mm/s^2 (%.4f g), residual rms %.1f,"
            " max %.1f mm/s^2"
            % (radius, radius / FREEFALL_ACCEL, rms,
               max([abs(r) for r in residuals])),
            "  the head that measured B = 0 before now measures %+.2f deg"
            % (zero_shift,)]
        self.calibration = calibration
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'offset_u', "%.1f" % (calibration[0],))
        configfile.set(self.name, 'offset_w', "%.1f" % (calibration[1],))
        configfile.set(self.name, 'gain_ratio', "%.5f" % (calibration[2],))
        lines.append("  The calibration is in use now; run SAVE_CONFIG to"
                     " keep it.")
        self._respond("\n".join(lines))
        axis.move_axis(0.)
        if not getattr(axis, 'has_endstop', False):
            # B was booked on the old calibration, so book it again.  (An
            # endstop decides where B is, so a rail with one is still
            # right - only its verification reads the sensor.)
            self._respond("%s: homing B again on the new calibration"
                          % (self.name,))
            axis.home()
        return calibration

    ######################################################################
    # Drive ratio calibration
    ######################################################################
    def _return_to_zero(self, axis):
        # Leaves the head at a defined angle on an error path, without
        # letting a failure to get there hide the error that got here
        try:
            axis.move_axis(0.)
        except self.printer.command_error:
            logging.exception("%s: could not return B to 0", self.name)
    def _check_drive_scale(self, scale):
        if MIN_RESPONSE_RATIO <= scale <= MAX_RESPONSE_RATIO:
            return
        if scale <= -MIN_RESPONSE_RATIO:
            hint = ("positive_vector and the motors disagree about which way"
                    " is +B - check positive_vector, then invert_b_direction"
                    " in [printer]")
        elif scale > MAX_RESPONSE_RATIO:
            hint = ("that is too far off to be fitted safely - correct the"
                    " drive ratio by hand first")
        else:
            hint = ("the head is not following the motors - check that they"
                    " are driving it, and that the sensor is on the rotating"
                    " part of the head")
        raise self.printer.command_error(
            "%s: the head turned %.3f deg per commanded degree over the"
            " sweep, so nothing was changed: %s" % (self.name, scale, hint))
    def calibrate_drive(self, axis, start, end, steps, settle_time,
                        sample_time, sweep_back=False, save=True):
        # Sweeps B from start to end (and back, with sweep_back), fits the
        # measured head angle against the angle the step counters imply,
        # and writes the corrected drive ratio for SAVE_CONFIG.  With
        # save=False it is a spot check that changes nothing.  Returns
        # the scale: degrees the head turned per commanded degree.
        error = self.printer.command_error
        options = axis.get_drive_ratio_options()
        if axis.get_step_position() is None or not options:
            raise error("%s: the B drive does not report its step counters"
                        " or its drive ratio, so it cannot be calibrated"
                        % (self.name,))
        if self.calibration == IDENTITY_CALIBRATION:
            raise error(
                "%s: the sensor is uncalibrated.  Its offsets bend the"
                " measured angle, which a drive ratio fit would book as"
                " ratio error - run B_SENSOR_CALIBRATE first" % (self.name,))
        pos_min, pos_max = axis.get_range()
        for name, value in (('START', start), ('END', end)):
            if not pos_min <= value <= pos_max:
                raise error(
                    "%s: %s=%.2f is outside the soft limits of %.2f to %.2f"
                    % (self.name, name, value, pos_min, pos_max))
        if abs(end - start) < MIN_STEP_ARC:
            raise error(
                "%s: a sweep from %.2f to %.2f is too short to fit a drive"
                " ratio on - it needs to cover at least %.0f deg"
                % (self.name, start, end, MIN_STEP_ARC))
        if steps < 2:
            raise error("%s: a drive ratio needs at least 2 stations"
                        % (self.name,))
        # Over-travel before each leg, so every station on it is approached
        # from the same side and backlash stays out of the scale
        direction = 1. if end > start else -1.
        approach = start - direction * STEP_TAKEUP
        overshoot = end + direction * STEP_TAKEUP
        turns = [('START', start, approach)]
        if sweep_back:
            turns.append(('END', end, overshoot))
        for name, value, turn in turns:
            if not pos_min <= turn <= pos_max:
                raise error(
                    "%s: every station is approached from the same side,"
                    " after %.0f deg of over-travel, and from %s=%.2f that"
                    " reaches %.2f - outside the soft limits of %.2f to %.2f."
                    "  Move %s at least %.0f deg inside the limit"
                    % (self.name, STEP_TAKEUP, name, value, turn, pos_min,
                       pos_max, name, STEP_TAKEUP))
        stations = [start + (end - start) * i / (steps - 1.)
                    for i in range(steps)]
        plan = [(approach, None)] + [(s, 0) for s in stations]
        if sweep_back:
            plan += [(overshoot, None)] + [(s, 1) for s in reversed(stations)]
        self._energise(axis)
        records = []
        try:
            for commanded, leg in plan:
                axis.move_axis(commanded)
                if leg is None:
                    continue
                step_angle = axis.get_step_position()
                measured = self.measure_vertical(settle_time, sample_time)
                records.append((commanded, leg, step_angle, measured))
            angles = unwrap_angles([r[3] for r in records])
            try:
                scale, offsets, residuals = fit_drive_sweep(
                    [(r[2], phi, r[1]) for r, phi in zip(records, angles)])
            except ValueError as e:
                raise error("%s: the drive fit failed: %s - the head was"
                            " likely moving during the sweep" % (self.name, e))
            self._check_drive_scale(scale)
        except error:
            self._return_to_zero(axis)
            raise
        lines = ["%s: drive calibration from B = %.1f to %.1f, %d stations%s"
                 % (self.name, start, end, steps,
                    " and back" if sweep_back else "")]
        for (commanded, leg, step_angle, measured), phi, residual in zip(
                records, angles, residuals):
            lines.append("  B %7.2f %s: measured %8.3f deg, residual %+.3f"
                         % (commanded, "back" if leg else "out ", phi,
                            residual))
        rms = math.sqrt(sum([r * r for r in residuals]) / len(residuals))
        lines.append("  the head turns %.5f deg per commanded degree (%+.3f %%)"
                     % (scale, 100. * (scale - 1.)))
        if sweep_back:
            # Each leg lags behind the motors in its own direction, so the
            # return leg reads further back along the sweep
            backlash = direction * (offsets[1] - offsets[0])
            lines.append("  backlash %.3f deg, from the offset between the"
                         " outbound and return sweeps" % (backlash,))
        lines.append("  residual rms %.3f deg, max %.3f"
                     % (rms, max([abs(r) for r in residuals])))
        if rms > STEP_RESIDUAL_WARNING:
            lines.append("  The residuals are large for a head measured at"
                         " rest: look at their shape before trusting the"
                         " ratio (see docs/Accel_B_Homing.md)")
        configfile = self.printer.lookup_object('configfile')
        for section, option, value, per_degree in options:
            new_value = value / scale if per_degree else value * scale
            lines.append("  %s = %.6f in [%s] (was %.6f)"
                         % (option, new_value, section, value))
            if save:
                configfile.set(section, option, "%.6f" % (new_value,))
        if save:
            lines.append("  Run SAVE_CONFIG to keep it.  The drive does not"
                         " change until the restart that follows, and B must"
                         " be homed again after it.")
        else:
            lines.append("  A spot check - nothing was changed.  Run"
                         " B_STEP_CALIBRATE without MODE=QUICK to fit and"
                         " save the ratio.")
        self._respond("\n".join(lines))
        axis.move_axis(0.)
        return scale

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
        res['fusion_tau'] = self.fusion_tau
        res['rotation_axis_sign'] = self.gyro_coefficient
        # measured_b is only ever a fused angle from vertical: the last
        # one measured, even if a B_MEASURE FUSION=0 has run since.  The
        # accelerometer-only angle of the latest reading is accel_b.
        res['measured_b'] = self.last_fused_angle
        reading = self.last_reading
        res['accel_b'] = None if reading is None else reading.accel_angle
        res['rotation_rate'] = (None if reading is None
                                else reading.rotation_rate)
        res['fusion_disagreement'] = (None if reading is None
                                      else reading.disagreement)
        res['offset_u'], res['offset_w'], res['gain_ratio'] = self.calibration
        return res
    cmd_B_SENSOR_CALIBRATE_help = ("Sweep B and fit the sensor's offsets and"
                                   " gain ratio")
    def cmd_B_SENSOR_CALIBRATE(self, gcmd):
        # Moves B, in the machine frame, through the stations
        for name in ('rtcp', 'b_projection'):
            obj = self.printer.lookup_object(name, None)
            if obj is not None:
                obj.check_disabled("B_SENSOR_CALIBRATE")
        if not self.is_b_homed():
            raise gcmd.error("%s: B_SENSOR_CALIBRATE turns the head, so B"
                             " must be homed first (G28 B)" % (self.name,))
        pos_min, pos_max = self.b_axis.get_range()
        start = gcmd.get_float('START', pos_min)
        end = gcmd.get_float('END', pos_max)
        steps = gcmd.get_int('STEPS', 13, minval=MIN_REDUCED_FIT_STATIONS)
        settle = gcmd.get_float('SETTLE', CALIBRATE_SETTLE_TIME, minval=0.)
        window = gcmd.get_float('SAMPLE_TIME', CALIBRATE_SAMPLE_TIME,
                                minval=.05)
        fit_gain = gcmd.get_int('GAIN', 1, minval=0, maxval=1)
        self.calibrate_sensor(self.b_axis, start, end, steps, settle, window,
                              bool(fit_gain))
    cmd_B_STEP_CALIBRATE_help = ("Sweep B and fit the drive ratio against the"
                                 " measured head angle")
    def cmd_B_STEP_CALIBRATE(self, gcmd):
        # Moves B, in the machine frame, through the stations
        for name in ('rtcp', 'b_projection'):
            obj = self.printer.lookup_object(name, None)
            if obj is not None:
                obj.check_disabled("B_STEP_CALIBRATE")
        if not self.is_b_homed():
            raise gcmd.error("%s: B_STEP_CALIBRATE turns the head, so B must"
                             " be homed first (G28 B)" % (self.name,))
        settle = gcmd.get_float('SETTLE', CALIBRATE_SETTLE_TIME, minval=0.)
        window = gcmd.get_float('SAMPLE_TIME', CALIBRATE_SAMPLE_TIME,
                                minval=.05)
        mode = gcmd.get('MODE', 'FULL').upper()
        if mode == 'QUICK':
            angle = gcmd.get_float('ANGLE', 90.)
            self.calibrate_drive(self.b_axis, 0., angle, 2, settle, window,
                                 save=False)
            return
        if mode != 'FULL':
            raise gcmd.error("%s: MODE must be FULL or QUICK, not '%s'"
                             % (self.name, mode))
        pos_min, pos_max = self.b_axis.get_range()
        start = gcmd.get_float('START', pos_min + STEP_TAKEUP)
        end = gcmd.get_float('END', pos_max - STEP_TAKEUP)
        steps = gcmd.get_int('STEPS', 13, minval=MIN_STEP_STATIONS)
        sweep_back = gcmd.get_int('RETURN', 0, minval=0, maxval=1)
        self.calibrate_drive(self.b_axis, start, end, steps, settle, window,
                             bool(sweep_back))
    cmd_B_MEASURE_help = "Measure the B axis angle against gravity"
    def cmd_B_MEASURE(self, gcmd):
        settle = gcmd.get_float('SETTLE', self.settle_time, minval=0.)
        window = gcmd.get_float('SAMPLE_TIME', self.sample_time, minval=.05)
        fusion = gcmd.get_int('FUSION', 1, minval=0, maxval=1)
        check = gcmd.get_int('CHECK', 0, minval=0, maxval=1)
        if check and not fusion:
            raise gcmd.error("%s: CHECK=1 compares the fused angle, so it"
                             " cannot be combined with FUSION=0"
                             % (self.name,))
        if check:
            # Refuses a chip with no gyroscope before sampling anything
            self.measure_vertical(settle, window)
            reading = self.last_reading
        else:
            reading = self.measure(settle, window, bool(fusion))
        x, y, z = reading.vector
        dx, dy, dz = reading.deviation
        if reading.fused_angle is not None:
            angle = reading.fused_angle
            lines = ["%s: B = %.3f deg from vertical, fused (%d samples)"
                     % (self.name, angle, reading.count)]
        else:
            angle = reading.accel_angle
            lines = ["%s: B = %.3f deg, accelerometer only (%d samples) -"
                     " not fused, so not an angle G28 B or CHECK=1 acts on"
                     % (self.name, angle, reading.count)]
        lines += [
            "  vector x/y/z = %.1f / %.1f / %.1f mm/s^2,"
            " |a| = %.1f (%.4f g)"
            % (x, y, z, reading.magnitude, reading.magnitude / FREEFALL_ACCEL),
            "  in plane u/w = %.1f / %.1f, radius = %.1f,"
            " out of plane (%s) = %.1f"
            % (reading.u, reading.w, math.hypot(reading.u, reading.w),
               AXIS_NAMES[self.oop_index], reading.out_of_plane),
            "  sample deviation x/y/z = %.1f / %.1f / %.1f mm/s^2"
            % (dx, dy, dz)]
        if self.calibration == IDENTITY_CALIBRATION:
            lines.append("  sensor uncalibrated - run B_SENSOR_CALIBRATE")
        else:
            lines.append("  corrected by offset_u/offset_w = %.1f / %.1f"
                         " mm/s^2, gain_ratio = %.5f" % self.calibration)
        if reading.rotation_rate is not None:
            lines.append("  rotation rate = %.4f deg/s (gate %.3f)"
                         % (reading.rotation_rate, self.max_rotation_rate))
        if reading.fused_angle is not None:
            lines.append(
                "  accelerometer alone %.3f deg, tail difference %+.3f"
                " (tau %.3f s)"
                % (reading.accel_angle,
                   reading.disagreement
                   if reading.disagreement is not None else float('nan'),
                   self.fusion_tau))
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
        error = angle - commanded
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
                    % (self.name, angle, commanded, error, tolerance))


def load_config(config):
    return AccelBHoming(config)
