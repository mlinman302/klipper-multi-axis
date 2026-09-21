#!/usr/bin/env python
# Check a g-code file against the reach of an RTCP tilting head
#
# Copyright (C) 2026  Klipper multi-axis contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# [rtcp] holds the tool *tip* where the g-code asks for it, so a B move
# has to be paid for in carriage travel.  On a polar machine (corertheta)
# that travel is arm radius, and the arm cannot go past the centre of the
# bed - see the radial check in klippy/extras/rtcp.py.  A file that asks
# for a tilted nozzle too close to the centre therefore aborts the print
# part way through, which is an expensive way to find out.
#
# This walks the file with the same arithmetic klippy uses and reports
# every move that would be refused, so a slicer's output can be checked
# in a second instead of in an hour of printing.
import argparse, math, sys

# Kept identical to klippy/extras/b_projection.py
BED_MIN_RADIUS = 0.010


def deltas(b_angle, tool_h, tool_v):
    # rtcp.RTCP._deltas() - how far the carriage must move to hold the tip
    b_rad = math.radians(b_angle)
    sin_b, cos_b_1 = math.sin(b_rad), math.cos(b_rad) - 1.
    return (tool_h * cos_b_1 - tool_v * sin_b,
            tool_h * sin_b + tool_v * cos_b_1)


def min_tip_radius(b_angle, tool_h, tool_v):
    # The closest the tip can come to the centre of the bed at this angle:
    # the arm bottoms out at radius zero, and the tip is -dh outboard of it
    return max(0., -deltas(b_angle, tool_h, tool_v)[0])


def project_b(b_angle, x, y, projection):
    # b_projection.BAxisProjection.project() - a commanded B is a lean
    # toward the bed's +x, and the machine gets its component in the plane
    # the head can actually tilt in
    if not projection:
        return b_angle
    r2 = x * x + y * y
    if r2 < BED_MIN_RADIUS * BED_MIN_RADIUS:
        return b_angle
    return b_angle * x / math.sqrt(r2)


def interior_points(sp, ep, projection):
    # rtcp.RTCP._interior_points() - the places inside a move that can be
    # worse than either end
    ts = []
    if projection and (sp[1] < 0.) != (ep[1] < 0.):
        ts.append(sp[1] / (sp[1] - ep[1]))
    dx, dy = ep[0] - sp[0], ep[1] - sp[1]
    d2 = dx * dx + dy * dy
    if d2 > 0.:
        ts.append(-(sp[0] * dx + sp[1] * dy) / d2)
    return [[s + t * (e - s) for s, e in zip(sp, ep)]
            for t in ts if 0. < t < 1.]


def shortfall(pos, tool_h, tool_v, projection):
    # How far inside the forbidden zone around the centre this point is
    x, y = pos[0], pos[1]
    machine_b = project_b(pos[3], x, y, projection)
    radius = math.hypot(x, y)
    return min_tip_radius(machine_b, tool_h, tool_v) - radius, machine_b


def iter_moves(fname):
    # Absolute G0/G1 only - which is what a non-planar slicer emits
    pos = [0., 0., 0., 0.]
    with open(fname) as f:
        for lineno, line in enumerate(f, 1):
            line = line.split(';')[0].strip()
            if not line:
                continue
            parts = line.split()
            if parts[0].upper() not in ('G0', 'G1'):
                continue
            start = list(pos)
            for word in parts[1:]:
                axis = word[0].upper()
                if axis not in 'XYZB':
                    continue
                try:
                    value = float(word[1:])
                except ValueError:
                    continue
                pos['XYZB'.index(axis)] = value
            yield lineno, line, start, list(pos)


def main():
    parser = argparse.ArgumentParser(description=(
        "Check a g-code file against the reach of an [rtcp] tilting head"
        " on a polar machine"))
    parser.add_argument('gcode', help="the g-code file to check")
    parser.add_argument('-v', '--tool-vertical-offset', type=float,
                        required=True, metavar='MM',
                        help="[rtcp] tool_vertical_offset")
    parser.add_argument('-o', '--tool-horizontal-offset', type=float,
                        default=0., metavar='MM',
                        help="[rtcp] tool_horizontal_offset")
    parser.add_argument('--no-projection', action='store_true',
                        help="the machine has no [b_projection]")
    parser.add_argument('-n', '--report', type=int, default=5, metavar='N',
                        help="how many offending moves to list")
    args = parser.parse_args()
    tool_h, tool_v = args.tool_horizontal_offset, args.tool_vertical_offset
    projection = not args.no_projection

    nmoves = 0
    bad = []
    for lineno, line, start, end in iter_moves(args.gcode):
        nmoves += 1
        worst = None
        for pos in [start, end] + interior_points(start, end, projection):
            short, machine_b = shortfall(pos, tool_h, tool_v, projection)
            if worst is None or short > worst[0]:
                worst = (short, machine_b, pos)
        if worst[0] > 1e-9:
            bad.append((lineno, line, worst))

    print("%s: %d moves, %d unreachable" % (args.gcode, nmoves, len(bad)))
    if not bad:
        print("The whole file stays outside the head's forbidden zone.")
        return 0
    print("\nMinimum tip radius by machine B angle"
          " (tool_vertical_offset=%g, tool_horizontal_offset=%g):"
          % (tool_v, tool_h))
    for b in (5, 10, 15, 30, 45, 60, 90):
        print("    B=%3d deg  ->  the tip cannot come within %6.2f mm"
              " of the centre" % (b, min_tip_radius(b, tool_h, tool_v)))
    print("\nFirst offending move - this is where the print stops:")
    listing = [bad[0]] + sorted(bad, key=lambda e: -e[2][0])[:args.report]
    seen = set()
    for i, (lineno, line, (short, machine_b, pos)) in enumerate(listing):
        if lineno in seen:
            continue
        seen.add(lineno)
        if i == 1:
            print("\nWorst offenders:")
        print("  line %d: %s" % (lineno, line))
        print("    tip at radius %.2f mm, B %.2f -> machine B %.2f,"
              " which needs radius %.2f: short by %.2f mm"
              % (math.hypot(pos[0], pos[1]), pos[3], machine_b,
                 math.hypot(pos[0], pos[1]) + short, short))
    return 1


if __name__ == '__main__':
    sys.exit(main())
