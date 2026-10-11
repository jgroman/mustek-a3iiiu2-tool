import struct

# PI2 is the driver's exact double pi/2 (0x400921fb4d12d84a / 2), loaded at
PI2 = 1.570796300000000034
ANGLE_512 = 512.0
ANGLE_256 = 256.0
VALID_DPIS = (300, 0x32, 0x4B, 0x96, 600, 0x4B0, 0x960)


def _ftol(x):
    return int(x)


def _fast(i, count, period, divisor):
    return _ftol(period + (count - period) * (0.04 ** (i * PI2 / divisor)))


def _slow(i, count, period, divisor):
    return _ftol(count - (count - period) * (0.5 ** (i * PI2 / divisor)))


def _slow_hardcoded(i, divisor):
    return _ftol(4500.0 - 2500.0 * (0.5 ** (i * PI2 / divisor)))


def build_phase_tables(dst, count, period, steps):
    for i in range(0x200):
        v = _fast(i, count, period, ANGLE_512)
        for off in (0x000, 0x1000, 0x1800):
            struct.pack_into("<H", dst, off + i * 2, v)
    for i in range(0xFF):
        v = _slow(i, count, period, ANGLE_256)
        for off in (0x400, 0xC00, 0x1400, 0x1C00):
            struct.pack_into("<H", dst, off + i * 2, v)
    if steps == 0:
        struct.pack_into("<H", dst, 0x800, period)
    else:
        for i in range(steps - 1, -1, -1):
            v = _fast(i, count, period, float(steps + 1))
            struct.pack_into("<H", dst, 0x800 + i * 2, v)
        struct.pack_into("<H", dst, 0x800 + steps * 2, period)


def build_home_motor_table(dst, count, period, steps, flags, dpi):
    for i in range(0x200):
        v = _fast(i, count, period, ANGLE_256)
        struct.pack_into("<H", dst, 0x000 + i * 2, v)
        struct.pack_into("<H", dst, 0x1800 + i * 2, v)
    for i in range(0x100):
        v = _slow_hardcoded(i, ANGLE_256)
        struct.pack_into("<H", dst, 0x400 + i * 2, v)
        struct.pack_into("<H", dst, 0x1C00 + i * 2, v)
    if dpi not in VALID_DPIS:
        return
    if count - period <= 0:
        ramp = [period] * steps
    elif steps == 0:
        ramp = []
    else:
        #   (pow(0.04, i*PI2/steps) - pow(0.04, (steps-1)*PI2/steps))
        # Two pow calls (the driver's exponent uses i and (steps-1) with the
        # same steps divisor); the difference normalizes the ramp so its last
        # entry (i=steps-1) is exactly period.
        c = 0.04 ** ((steps - 1) * PI2 / steps)
        ramp = [int(period + (count - period) * (0.04 ** (i * PI2 / steps) - c))
                for i in range(steps)]
    for i in range(steps):
        struct.pack_into("<H", dst, 0x800 + i * 2, ramp[i])
        struct.pack_into("<H", dst, 0x1000 + i * 2, ramp[i])
    for i in range(steps, 0x200):
        struct.pack_into("<H", dst, 0x800 + i * 2, period)
        struct.pack_into("<H", dst, 0x1000 + i * 2, period)
    if count - period <= 0:
        frmp = [count] * flags
    else:
        frmp = [_slow(i, count, period, float(flags)) for i in range(flags)]
    for i in range(flags):
        struct.pack_into("<H", dst, 0xC00 + i * 2, frmp[i])
        struct.pack_into("<H", dst, 0x1400 + i * 2, frmp[i])
    for i in range(flags, 0x100):
        struct.pack_into("<H", dst, 0xC00 + i * 2, count)
        struct.pack_into("<H", dst, 0x1400 + i * 2, count)
