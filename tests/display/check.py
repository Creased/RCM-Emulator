#!/usr/bin/env python3
"""Compare the emulator's frame snapshot with what a display scenario drew.

Usage: check.py <scenario> <last_fb.rgba> <last_fb.meta>

last_fb.rgba is the host view, ARGB8888 little-endian, as last_fb.meta's
"W H" gives it. The expected picture follows tests/display/payload.c.
"""
import sys
from array import array


def pat(x, y):
    return 0xFF000000 | ((x * 7) & 0xFF) << 16 | ((y * 3) & 0xFF) << 8 | ((x ^ y) & 0xFF)


def patq(x, y):
    return 0xFF000000 | ((x * 5 + y) & 0xFF) << 16 | 0x4000 | ((y * 9) & 0xFF)


def blend(s, d, k1):
    # BLEND_MATCH_SELECT SRC K1, DST NEG_K1 (1 - K1), per channel, opaque.
    out = 0xFF000000
    for sh in (0, 8, 16):
        c = (((s >> sh) & 0xFF) * k1 + ((d >> sh) & 0xFF) * (255 - k1)) // 255
        out |= min(c, 255) << sh
    return out


def expected(scenario, w, h):
    if scenario == 1:
        return (720, 1280), lambda x, y: pat(x, y)
    if scenario in (2, 3, 4, 5):
        # The picture was drawn landscape and turned on its way to the
        # panel; the view turns it back.
        return (1280, 720), lambda x, y: pat(x, y)
    if scenario == 6:
        def f(x, y):
            a = pat(x, 1279 - y)
            if 100 <= x < 300 and 200 <= y < 500:
                return blend(patq(x - 100, y - 200), a, 128)
            return a
        return (720, 1280), f
    raise SystemExit("unknown scenario %d" % scenario)


def main():
    scenario = int(sys.argv[1])
    w, h = map(int, open(sys.argv[3]).read().split())
    frame = array("I")
    with open(sys.argv[2], "rb") as fh:
        frame.frombytes(fh.read())
    (ew, eh), f = expected(scenario, w, h)
    if (w, h) != (ew, eh):
        print("  FAIL  scenario %d: view is %dx%d, expected %dx%d" % (scenario, w, h, ew, eh))
        return 1
    bad = 0
    first = None
    for y in range(h):
        row = y * w
        for x in range(w):
            want = f(x, y)
            if frame[row + x] != want:
                bad += 1
                if first is None:
                    first = (x, y, frame[row + x], want)
    if bad:
        x, y, got, want = first
        print("  FAIL  scenario %d: %d of %d pixels differ; first at (%d,%d): %08X, expected %08X"
              % (scenario, bad, w * h, x, y, got, want))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
