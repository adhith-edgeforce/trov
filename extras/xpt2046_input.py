#!/usr/bin/env python3
"""
XPT2046 Touch Driver — Degree-4 Polynomial + Aggressive Jitter Filter
======================================================================
Two-stage filtering:
  Stage 1: Hardware debounce — collect 8 raw readings, take median
  Stage 2: Output smoothing — reject if jump > JUMP_THRESH pixels from last output,
           apply exponential moving average on stable readings

Calibration: degree-4 bivariate polynomial (border-only, mean~11px)
"""
import spidev, time, struct, os, fcntl
from collections import deque

# ── Calibration coefficients ─────────────────────────────────────────────────
RAW_X_MID   = 1800
RAW_X_SCALE = 700
RAW_Y_MID   = 1800
RAW_Y_SCALE = 700

W_X = [
     297.891463,
     849.272371,
      -6.189015,
    -185.699953,
     494.821740,
     656.046052,
    -375.008427,
    -904.048753,
    -275.171414,
    -469.931365,
     534.238284,
    -522.254171,
     929.281443,
      14.885770,
       2.872548,
]

W_Y = [
     419.229301,
       4.493708,
    -672.188949,
     540.005067,
      -2.904129,
    -415.486520,
    -102.217143,
     120.047180,
    -242.547452,
     747.744660,
     387.555250,
     466.440503,
   -1273.172806,
     -26.686358,
     176.620223,
]

SCREEN_W, SCREEN_H = 1024, 600

# ── Valid raw range ───────────────────────────────────────────────────────────
VALID_X_MIN, VALID_X_MAX = 1200, 2600
VALID_Y_MIN, VALID_Y_MAX = 1100, 2700
SPIKES = {511, 512, 1023, 1024, 2048, 2560, 3072, 3584, 4095}

# ── Stage 1: Hardware median filter ──────────────────────────────────────────
# Collect this many raw SPI readings per sample, take median
# Higher = smoother but more latency. 8 @ 3ms each = ~24ms per sample
MEDIAN_WINDOW = 8

# ── Stage 2: Output jump filter ──────────────────────────────────────────────
# Reject screen-coordinate jumps larger than this (pixels)
# Prevents cursor teleporting on noise spikes
JUMP_THRESH = 60   # px — increase if legitimate fast swipes get dropped

# Exponential moving average factor (0=no smoothing, 1=no update)
# 0.4 means new_pos = 0.6*new + 0.4*old — light smoothing
EMA_ALPHA = 0.6

# ── No-touch detection ───────────────────────────────────────────────────────
# How many consecutive failed reads before we declare finger lifted
NO_TOUCH_COUNT = 4

# ── uinput constants ──────────────────────────────────────────────────────────
UI_SET_EVBIT   = 0x40045564
UI_SET_ABSBIT  = 0x40045567
UI_SET_KEYBIT  = 0x40045565
UI_SET_PROPBIT = 0x40045569
UI_DEV_CREATE  = 0x5501
UI_DEV_DESTROY = 0x5502

EV_SYN = 0; EV_KEY = 1; EV_ABS = 3
ABS_X = 0; ABS_Y = 1; ABS_PRESSURE = 24
BTN_TOUCH = 0x14a
INPUT_PROP_DIRECT = 1


def emit(fd, etype, code, value):
    os.write(fd, struct.pack('llHHi', 0, 0, etype, code, value))


def setup_uinput():
    fd = os.open('/dev/uinput', os.O_WRONLY | os.O_NONBLOCK)
    fcntl.ioctl(fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT)
    fcntl.ioctl(fd, UI_SET_EVBIT,  EV_KEY)
    fcntl.ioctl(fd, UI_SET_EVBIT,  EV_ABS)
    fcntl.ioctl(fd, UI_SET_KEYBIT, BTN_TOUCH)
    fcntl.ioctl(fd, UI_SET_ABSBIT, ABS_X)
    fcntl.ioctl(fd, UI_SET_ABSBIT, ABS_Y)
    fcntl.ioctl(fd, UI_SET_ABSBIT, ABS_PRESSURE)
    name = b'XPT2046 Touchscreen\x00' + b'\x00' * 61
    abs_arrays = [0] * 256
    abs_arrays[0]  = SCREEN_W
    abs_arrays[1]  = SCREEN_H
    abs_arrays[24] = 255
    uud  = struct.pack('80sHHHHI', name, 3, 0x1234, 0x5678, 1, 0)
    uud += struct.pack('256i', *abs_arrays)
    os.write(fd, uud)
    fcntl.ioctl(fd, UI_DEV_CREATE)
    time.sleep(0.5)
    return fd


def read_raw(spi):
    rx = spi.xfer2([0x90, 0x00, 0x00])
    ry = spi.xfer2([0xD0, 0x00, 0x00])
    x = ((rx[1] << 8) | rx[2]) >> 3
    y = ((ry[1] << 8) | ry[2]) >> 3
    return x, y


def is_valid(x, y):
    if x in SPIKES or y in SPIKES:
        return False
    return (VALID_X_MIN < x < VALID_X_MAX) and (VALID_Y_MIN < y < VALID_Y_MAX)


def read_median(spi):
    """
    Stage 1: Collect MEDIAN_WINDOW valid raw readings, return their median.
    Returns (None, None) if we can't get enough valid readings in time.
    """
    xs = []; ys = []
    attempts = 0
    max_attempts = MEDIAN_WINDOW * 4  # allow some invalid reads

    while len(xs) < MEDIAN_WINDOW and attempts < max_attempts:
        x, y = read_raw(spi)
        attempts += 1
        if is_valid(x, y):
            xs.append(x)
            ys.append(y)
        else:
            # Got an invalid reading — if we had some valid ones, they might
            # be a real touch that just ended. Stop collecting.
            if len(xs) >= 2:
                break
        time.sleep(0.002)

    if len(xs) < 2:
        return None, None

    xs.sort(); ys.sort()
    mid = len(xs) // 2
    return xs[mid], ys[mid]


def poly_map(raw_x, raw_y):
    """Degree-4 bivariate polynomial mapping."""
    x = (raw_x - RAW_X_MID) / RAW_X_SCALE
    y = (raw_y - RAW_Y_MID) / RAW_Y_SCALE
    basis = [
        1,
        x,   y,
        x*y,
        x*x, y*y,
        x*x*y, x*y*y,
        x*x*x, y*y*y,
        x*x*x*y, x*x*y*y, x*y*y*y,
        x*x*x*x, y*y*y*y,
    ]
    sx = sum(W_X[i] * basis[i] for i in range(15))
    sy = sum(W_Y[i] * basis[i] for i in range(15))
    return max(0, min(SCREEN_W, int(sx))), max(0, min(SCREEN_H, int(sy)))


# ── Main ──────────────────────────────────────────────────────────────────────
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

ufd = setup_uinput()
print("XPT2046 | deg4 poly | median-8 + jump-filter + EMA")

touching       = False
no_touch_count = 0

# Stage 2 state
last_sx = None
last_sy = None

try:
    while True:
        raw_x, raw_y = read_median(spi)

        if raw_x is not None:
            sx, sy = poly_map(raw_x, raw_y)

            # Stage 2a: Jump filter — reject teleports
            if last_sx is not None:
                jump = ((sx - last_sx)**2 + (sy - last_sy)**2) ** 0.5
                if jump > JUMP_THRESH:
                    # Large jump — could be real fast swipe or noise spike
                    # Allow it only if we get 2 consecutive readings agreeing
                    raw_x2, raw_y2 = read_median(spi)
                    if raw_x2 is not None:
                        sx2, sy2 = poly_map(raw_x2, raw_y2)
                        jump2 = ((sx2 - last_sx)**2 + (sy2 - last_sy)**2) ** 0.5
                        if jump2 > JUMP_THRESH * 0.7:
                            # Two consecutive large jumps = real movement
                            sx, sy = sx2, sy2
                        else:
                            # Inconsistent — was a noise spike, skip
                            no_touch_count = 0
                            if not touching:
                                emit(ufd, EV_KEY, BTN_TOUCH, 1)
                                touching = True
                            continue
                    else:
                        no_touch_count = 0
                        continue

            # Stage 2b: EMA smoothing
            if last_sx is None:
                smooth_x = sx
                smooth_y = sy
            else:
                smooth_x = int(EMA_ALPHA * sx + (1 - EMA_ALPHA) * last_sx)
                smooth_y = int(EMA_ALPHA * sy + (1 - EMA_ALPHA) * last_sy)

            last_sx = smooth_x
            last_sy = smooth_y
            no_touch_count = 0

            if not touching:
                emit(ufd, EV_KEY, BTN_TOUCH, 1)
                touching = True

            emit(ufd, EV_ABS, ABS_X,        smooth_x)
            emit(ufd, EV_ABS, ABS_Y,        smooth_y)
            emit(ufd, EV_ABS, ABS_PRESSURE, 200)
            emit(ufd, EV_SYN, 0,            0)

        else:
            no_touch_count += 1
            if no_touch_count >= NO_TOUCH_COUNT and touching:
                emit(ufd, EV_KEY, BTN_TOUCH,    0)
                emit(ufd, EV_ABS, ABS_PRESSURE, 0)
                emit(ufd, EV_SYN, 0,            0)
                touching  = False
                last_sx   = None
                last_sy   = None

except KeyboardInterrupt:
    pass
finally:
    fcntl.ioctl(ufd, UI_DEV_DESTROY)
    os.close(ufd)
    spi.close()
