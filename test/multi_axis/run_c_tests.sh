#!/bin/bash
# Build and run the host tests for the six-axis motion space.
#
# These link the real klippy/chelper kinematics against a small driver so
# the motion math can be checked without a printer, an MCU, or a full
# c_helper.so build.  pyhelper.c is Linux-only (prctl/pthread), so the one
# symbol the kinematics need from it - errorf() - is stubbed below.
#
# Usage: test/multi_axis/run_c_tests.sh
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
CHELPER="$HERE/../../klippy/chelper"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CC="${CC:-gcc}"
# On this Windows checkout the MinGW cc1 needs its own bin dir on PATH
if [ -x /c/MinGW/bin/gcc.exe ] && ! command -v gcc >/dev/null 2>&1; then
    export PATH="/c/MinGW/bin:$PATH"
fi

cat > "$OUT/errorf_stub.c" <<'STUB'
#include <stdarg.h>
#include <stdio.h>
void errorf(const char *fmt, ...);
void
errorf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "errorf: ");
    vfprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    va_end(args);
}
STUB

echo "Building test_kin_6axis..."
$CC -Wall -O2 -I"$CHELPER" -o "$OUT/test_kin_6axis" \
    "$HERE/test_kin_6axis.c" "$OUT/errorf_stub.c" \
    "$CHELPER/trapq.c" "$CHELPER/itersolve.c" \
    "$CHELPER/kin_cartesian.c" "$CHELPER/kin_corexy.c" \
    "$CHELPER/kin_generic.c" "$CHELPER/kin_rotary_axis.c" \
    "$CHELPER/kin_rtcp.c" "$CHELPER/kin_corertheta.c" \
    "$CHELPER/kin_bproject.c" \
    "$CHELPER/stepcompress.c" "$CHELPER/msgblock.c" -lm

"$OUT/test_kin_6axis"
