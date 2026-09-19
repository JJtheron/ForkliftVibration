#!/usr/bin/env python3
"""
Bring-up checks for the forklift WBV logger.

Run this before the logger, in this order. Each check prints PASS, FAIL
or WARN with a specific hint, and later checks assume earlier ones
passed. Nothing here writes to the database or moves the truck.

    python3 bringup.py              # everything, including tap tests
    python3 bringup.py --quick      # skip anything needing you to move it
    python3 bringup.py --only i2c,fifo,int1

Checks:
    i2c      bus present, chip answers, DEVID correct
    accel    gravity reads ~9.81, axes sane, noise floor measured
    orient   which way is up - catches a sideways seat pad mount
    rate     actual sample rate vs configured (catches slow I2C)
    fifo     stream mode fills and drains correctly
    int1     watermark interrupt on GPIO17 actually fires
    int2     activity interrupt on GPIO27 (needs --hw-gate wiring)
    serial   UART free, console disabled
    gps      NMEA flowing, fix status
    db       MySQL reachable, schema present, durability setting
"""

import argparse
import os
import sys
import time
import lgpio

I2C_BUS = 1
ADDR_PRIMARY = 0x53      # SDO to GND
ADDR_ALT = 0x1D          # SDO to 3V3
INT1_GPIO = 17
INT2_GPIO = 27
G = 9.80665
LSB_MS2 = 0.0039 * G

REG_DEVID, REG_BW_RATE, REG_POWER_CTL = 0x00, 0x2C, 0x2D
REG_INT_ENABLE, REG_INT_MAP, REG_INT_SOURCE = 0x2E, 0x2F, 0x30
REG_DATA_FORMAT, REG_DATAX0 = 0x31, 0x32
REG_FIFO_CTL, REG_FIFO_STATUS = 0x38, 0x39
REG_THRESH_ACT, REG_THRESH_INACT = 0x24, 0x25
REG_TIME_INACT, REG_ACT_INACT_CTL = 0x26, 0x27

results = []


def report(name, ok, detail, hint=None):
    """ok: True pass, False fail, None warn, 'skip' not run.

    A check skipped because an earlier one failed is not itself a
    failure - showing ten FAILs for one unplugged chip buries the
    actual problem.
    """
    tag = {True: "PASS", False: "FAIL", None: "WARN", "skip": "SKIP"}[ok]
    print(f"[{tag}] {name}: {detail}")
    if hint and ok is not True and ok != "skip":
        for line in hint.strip().splitlines():
            print(f"       {line.strip()}")
    results.append((name, ok))
    return ok


# ---------------------------------------------------------------------

def check_i2c(st):
    try:
        from smbus2 import SMBus
    except ImportError:
        return report("i2c", False, "smbus2 not installed",
                      "sudo apt install python3-smbus2")
    if not os.path.exists(f"/dev/i2c-{I2C_BUS}"):
        return report("i2c", False, f"/dev/i2c-{I2C_BUS} missing",
                      "sudo raspi-config -> Interface Options -> I2C -> Yes,"
                      " then reboot")
    found = []
    try:
        bus = SMBus(I2C_BUS)
    except Exception as exc:
        return report("i2c", False, f"cannot open bus: {exc}",
                      "add your user to the i2c group: "
                      "sudo usermod -aG i2c $USER, then log out and back in")
    for addr in (ADDR_PRIMARY, ADDR_ALT):
        try:
            if bus.read_byte_data(addr, REG_DEVID) == 0xE5:
                found.append(addr)
        except Exception:
            pass
    if not found:
        bus.close()
        return report("i2c", False, "no ADXL343 responding",
                      """
                      Check 3V3 (not 5V), GND, SDA->GPIO2, SCL->GPIO3.
                      Run: i2cdetect -y 1
                      Expect 53 (SDO grounded) or 1d (SDO high).
                      Nothing at all usually means power or SDA/SCL swapped.
                      """)
    st["bus"] = bus
    st["addr"] = found[0]
    return report("i2c", True,
                  f"ADXL343 at 0x{found[0]:02X}, DEVID 0xE5 correct")


def _w(st, r, v):
    st["bus"].write_byte_data(st["addr"], r, v)


def _r(st, r):
    return st["bus"].read_byte_data(st["addr"], r)


def _sample(st):
    import struct
    # FIX: Add | 0x80 to enable multi-byte auto-increment on the ADXL343
    d = st["bus"].read_i2c_block_data(st["addr"], REG_DATAX0 | 0x80, 6)
    x, y, z = struct.unpack("<3h", bytes(d))
    return x * LSB_MS2, y * LSB_MS2, z * LSB_MS2


def check_accel(st):
    if "bus" not in st:
        return report("accel", "skip", "no chip from the i2c check")
    _w(st, REG_POWER_CTL, 0x00)
    _w(st, REG_BW_RATE, 0x0D)          # 800 Hz
    _w(st, REG_DATA_FORMAT, 0x08 | 0x01)   # full res, +/-4g
    _w(st, REG_FIFO_CTL, 0x00)
    _w(st, REG_POWER_CTL, 0x08)
    time.sleep(0.2)

    n = 400
    xs, ys, zs = [], [], []
    for _ in range(n):
        a, b, c = _sample(st)
        xs.append(a)
        ys.append(b)
        zs.append(c)
        time.sleep(0.002)
    mx, my, mz = (sum(v) / n for v in (xs, ys, zs))
    mag = (mx * mx + my * my + mz * mz) ** 0.5

    def sd(v, m):
        return (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5
    noise = max(sd(xs, mx), sd(ys, my), sd(zs, mz))
    st["mean"] = (mx, my, mz)
    st["noise"] = noise

    if not (9.0 < mag < 10.6):
        return report("accel", False,
                      f"vector magnitude {mag:.2f} m/s^2, expected ~9.81",
                      """
                      If it reads ~0, the chip is in standby or the range
                      bits are wrong. If it reads roughly double, the
                      FULL_RES bit is not set. If it is wildly noisy, check
                      the ground connection first.
                      """)
    ok = True if noise < 0.15 else None
    return report("accel", ok,
                  f"gravity {mag:.3f} m/s^2, noise floor {noise:.4f} m/s^2 rms",
                  None if ok else """
                  Noise above ~0.15 m/s^2 will swamp the 0.3 m/s^2 floor.
                  Usual causes: long unshielded I2C leads, sharing a supply
                  with something switching, or the sensor not rigidly
                  mounted. Retest with the truck switched off.
                  """)


def check_orient(st):
    if "mean" not in st:
        return report("orient", "skip", "no readings from the accel check")
    mx, my, mz = st["mean"]
    axes = {"x": mx, "y": my, "z": mz}
    up = max(axes, key=lambda k: abs(axes[k]))
    frac = abs(axes[up]) / ((mx * mx + my * my + mz * mz) ** 0.5)
    tilt = __import__("math").degrees(__import__("math").acos(min(1.0, frac)))
    detail = (f"gravity is on {up} ({axes[up]:+.2f} m/s^2), "
              f"tilt {tilt:.1f} deg off vertical")
    if up != "z":
        return report("orient", False, detail,
                      """
                      The code applies Wk (vertical weighting) to z and Wd
                      (horizontal) to x and y. With the sensor on its side
                      every number is weighted by the wrong curve.
                      Remount so z points up, or swap the axis-to-curve
                      mapping in AxisProcessor.
                      """)
    if tilt > 15:
        return report("orient", None, detail,
                      "More than 15 degrees of tilt puts real vertical "
                      "vibration onto the horizontal axes. Shim the pad flat.")
    return report("orient", True, detail)


def check_rate(st):
    """Measure the real sample rate. Catches I2C too slow for 800 Hz."""
    if "bus" not in st:
        return report("rate", "skip", "no chip from the i2c check")
    _w(st, REG_POWER_CTL, 0x00)
    _w(st, REG_BW_RATE, 0x0D)
    _w(st, REG_FIFO_CTL, 0x00)
    _w(st, REG_FIFO_CTL, 0x80 | 24)     # stream, watermark 24
    _w(st, REG_POWER_CTL, 0x08)
    time.sleep(0.1)
    _w(st, REG_FIFO_CTL, 0x00)          # flush
    _w(st, REG_FIFO_CTL, 0x80 | 24)

    t0 = time.time()
    got = 0
    while time.time() - t0 < 3.0:
        n = _r(st, REG_FIFO_STATUS) & 0x3F
        for _ in range(n):
            _sample(st)
            got += 1
        time.sleep(0.005)
    dur = time.time() - t0
    rate = got / dur
    st["rate"] = rate
    pct = 100 * rate / 800.0
    ok = True if pct > 97 else (None if pct > 85 else False)
    return report("rate", ok,
                  f"{rate:.0f} samples/s measured against 800 configured "
                  f"({pct:.0f}%)",
                  None if ok is True else """
                  The Pi is not draining the FIFO fast enough. Raise the
                  I2C clock: add dtparam=i2c_arm_baudrate=400000 to
                  /boot/firmware/config.txt and reboot. If it is still
                  short, drop to --odr 400 and accept the band-edge error,
                  or move the chip to SPI.
                  """)


def check_fifo(st):
    if "bus" not in st:
        return report("fifo", "skip", "no chip from the i2c check")
    _w(st, REG_POWER_CTL, 0x00)
    _w(st, REG_FIFO_CTL, 0x00)
    _w(st, REG_BW_RATE, 0x0A)           # 100 Hz, easy to time
    _w(st, REG_FIFO_CTL, 0x80 | 16)
    _w(st, REG_POWER_CTL, 0x08)
    time.sleep(0.05)
    _w(st, REG_FIFO_CTL, 0x00)
    _w(st, REG_FIFO_CTL, 0x80 | 16)
    time.sleep(0.25)                    # ~25 samples at 100 Hz -> should cap
    depth = _r(st, REG_FIFO_STATUS) & 0x3F
    if depth < 5:
        return report("fifo", False, f"FIFO only reached {depth} entries",
                      "Stream mode is not filling. Check POWER_CTL measure "
                      "bit and that FIFO_CTL was written after the flush.")
    before = depth
    for _ in range(min(depth, 10)):
        _sample(st)
    after = _r(st, REG_FIFO_STATUS) & 0x3F
    drained = before - after
    ok = drained >= 5
    return report("fifo", ok if ok else False,
                  f"filled to {before}, 10 reads dropped it to {after}",
                  None if ok else """
                  Each 6-byte burst from DATAX0 should pop exactly one
                  entry. If the depth is not falling, you are probably
                  reading single bytes rather than a 6-byte block.
                  """)



def check_int1(st):
    if "bus" not in st:
        return report("int1", "skip", "no chip from the i2c check")

    _w(st, REG_POWER_CTL, 0x00)
    _w(st, REG_FIFO_CTL, 0x00)
    _r(st, REG_INT_SOURCE)

    init_depth = _r(st, REG_FIFO_STATUS) & 0x3F
    for _ in range(init_depth):
        _sample(st)

    try:
        handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(handle, INT1_GPIO, lgpio.SET_PULL_DOWN)
    except Exception as exc:
        return report("int1", False, f"cannot open GPIO{INT1_GPIO}: {exc}",
                      "pip install lgpio")

    count = 0
    previous = lgpio.gpio_read(handle, INT1_GPIO)
    print(f"INT1 initial pin state: {previous}")
    _w(st, REG_BW_RATE, 0x0A)
    _w(st, REG_FIFO_CTL, 0x50)
    _w(st, REG_INT_MAP, 0x00)
    _w(st, REG_INT_ENABLE, 0x02)
    _w(st, REG_POWER_CTL, 0x08)
    time.sleep(1.0)
    t0 = time.time()
    while time.time() - t0 < 10.0:
        n = _r(st, REG_FIFO_STATUS) & 0x3F
        for _ in range(n):
            _sample(st)
        current = lgpio.gpio_read(handle, INT1_GPIO)
        print(f"INT1 pin state: {current}")
        if current != previous:
            if current == 1:
                count += 1
                _r(st, REG_INT_SOURCE)  # clear latched flag
            previous = current
        time.sleep(0.04)

    dur = time.time() - t0
    _w(st, REG_INT_ENABLE, 0x00)
    lgpio.gpiochip_close(handle)

    per_s = count / dur
    if count == 0:
        return report("int1", False,
                      f"no edges on GPIO{INT1_GPIO} in {dur:.0f}s "
                      f"(line reads {previous})",
                      f"""
                      The chip is producing data but the Pi never saw an
                      edge. Check INT1 is wired to physical pin 11
                      (GPIO{INT1_GPIO}) and not INT2.
                      The logger still works without it: use --poll.
                      """)
    return report("int1", True,
                  f"{count} interrupts in {dur:.0f}s "
                  f"({per_s:.1f}/s, expected ~6)")


def check_int2(st, interactive):
    if "bus" not in st:
        return report("int2", "skip", "no chip from the i2c check")
    if not interactive:
        return report("int2", "skip", "needs you to tap the sensor; omit --quick")
    try:
        pin = _gpio(INT2_GPIO)
    except Exception as exc:
        return report("int2", None, f"cannot open GPIO{INT2_GPIO}: {exc}",
                      "Only needed for --hw-gate. Safe to skip.")
    hits = {"n": 0}
    pin.when_activated = lambda: hits.__setitem__("n", hits["n"] + 1)

    _w(st, REG_POWER_CTL, 0x00)
    _w(st, REG_BW_RATE, 0x0A)
    _w(st, REG_THRESH_ACT, 2)           # 2 x 62.5 mg
    _w(st, REG_THRESH_INACT, 1)
    _w(st, REG_TIME_INACT, 5)
    _w(st, REG_ACT_INACT_CTL, 0xFF)     # ac-coupled, all axes
    _w(st, REG_INT_MAP, 0x18)           # activity+inactivity -> INT2
    _w(st, REG_INT_ENABLE, 0x10 | 0x08)
    _r(st, REG_INT_SOURCE)
    _w(st, REG_POWER_CTL, 0x08)

    print("       tap the sensor a few times... (6 seconds)")
    t0 = time.time()
    while time.time() - t0 < 6.0:
        _r(st, REG_INT_SOURCE)
        time.sleep(0.05)
    _w(st, REG_INT_ENABLE, 0x00)
    pin.close()

    if hits["n"] == 0:
        return report("int2", None, "no activity edges seen",
                      f"""
                      Either INT2 is not wired to physical pin 13
                      (GPIO{INT2_GPIO}), or you did not tap hard enough.
                      This is optional - without it the software motion
                      gate still works. Just omit --hw-gate.
                      """)
    return report("int2", True, f"{hits['n']} activity interrupts seen")


def check_serial(st):
    port = "/dev/serial0"
    if not os.path.exists(port):
        return report("serial", False, f"{port} missing",
                      "sudo raspi-config -> Interface Options -> Serial Port:"
                      " login shell NO, hardware YES. Then reboot.")
    hint = None
    for cmdline in ("/boot/firmware/cmdline.txt", "/boot/cmdline.txt"):
        if os.path.exists(cmdline):
            txt = open(cmdline).read()
            if "console=serial0" in txt or "console=ttyAMA0" in txt:
                return report("serial", False,
                              "serial console still enabled in " + cmdline,
                              """
                              The kernel is using the UART as a terminal, so
                              NMEA will be mangled. Disable the login shell
                              over serial in raspi-config and reboot.
                              """)
            break
    return report("serial", True, f"{port} present, console not on it", hint)


def check_gps(st, interactive):
    if not os.path.exists("/dev/serial0"):
        return report("gps", "skip", "no serial port from the serial check")
    try:
        import serial
    except ImportError:
        return report("gps", False, "pyserial not installed",
                      "sudo apt install python3-serial")
    lines, sentences, valid = 0, 0, 0
    try:
        ser = serial.Serial("/dev/serial0", 9600, timeout=1)
        t0 = time.time()
        while time.time() - t0 < 6.0:
            raw = ser.readline().decode("ascii", "ignore").strip()
            lines += 1
            if raw.startswith("$"):
                sentences += 1
                p = raw.split(",")
                if p[0].endswith("RMC") and len(p) > 2 and p[2] == "A":
                    valid += 1
        ser.close()
    except Exception as exc:
        return report("gps", False, f"serial read failed: {exc}")

    if sentences == 0:
        return report("gps", False, f"no NMEA in 6s ({lines} reads)",
                      """
                      Nothing coming out of the HAT. Check it is seated,
                      and that its power LED is on. If you previously set
                      57600 baud it will stay there across a warm reboot -
                      try python3 gps.py --dump which tries both rates.
                      """)
    if valid == 0:
        return report("gps", None,
                      f"{sentences} sentences but no valid fix yet",
                      """
                      The receiver is alive and talking - it just has no
                      lock. Cold start outdoors takes 30s to several
                      minutes. The fix LED blinks once every 15s when
                      locked. Indoors it will never lock.
                      """)
    return report("gps", True,
                  f"{sentences} sentences in 6s, {valid} with a valid fix")


def check_db(st):
    try:
        import pymysql
    except ImportError:
        return report("db", False, "PyMySQL not installed",
                      "sudo apt install python3-pymysql")
    cnf = os.path.expanduser("~/.my.cnf")
    kw = {"database": "wbv", "charset": "utf8mb4"}
    if os.path.exists(cnf):
        kw["read_default_file"] = cnf
    sock = "/var/run/mysqld/mysqld.sock"
    if os.path.exists(sock):
        kw["unix_socket"] = sock
    else:
        kw["host"] = "localhost"
    try:
        c = pymysql.connect(**kw)
    except Exception as exc:
        return report("db", False, f"cannot connect: {exc}",
                      """
                      Put credentials in ~/.my.cnf (chmod 600):
                        [client]
                        user=wbv
                        password=...
                      and load the schema: sudo mysql < schema.sql
                      """)
    want = {"session", "gps_fix", "shock_event", "exposure_interval",
            "logger_status", "command"}
    with c.cursor() as cur:
        cur.execute("SHOW TABLES")
        have = {r[0] for r in cur.fetchall()}
        cur.execute("SELECT @@innodb_flush_log_at_trx_commit, @@version")
        flush, ver = cur.fetchone()
    c.close()
    missing = want - have
    if missing:
        return report("db", False, f"missing tables: {sorted(missing)}",
                      "sudo mysql < schema.sql")
    if flush != 1:
        return report("db", None,
                      f"MySQL {ver}, all tables present, but "
                      f"innodb_flush_log_at_trx_commit={flush}",
                      """
                      Committed events can be lost when the truck key cuts
                      power. Set it to 1 in your server config unless you
                      have a UPS and a clean shutdown.
                      """)
    return report("db", True, f"MySQL {ver}, all 6 tables, durable commits")


CHECKS = ["i2c", "accel", "orient", "rate", "fifo", "int1", "int2",
          "serial", "gps", "db"]


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--quick", action="store_true",
                   help="skip checks needing you to move the sensor")
    p.add_argument("--only", help="comma-separated subset of: "
                                  + ",".join(CHECKS))
    a = p.parse_args()
    want = ([s.strip() for s in a.only.split(",")] if a.only else CHECKS)
    interactive = not a.quick

    st = {}
    print("Forklift WBV logger - bring-up checks\n")
    fns = {
        "i2c": lambda: check_i2c(st),
        "accel": lambda: check_accel(st),
        "orient": lambda: check_orient(st),
        "rate": lambda: check_rate(st),
        "fifo": lambda: check_fifo(st),
        "int1": lambda: check_int1(st),
        "int2": lambda: check_int2(st, interactive),
        "serial": lambda: check_serial(st),
        "gps": lambda: check_gps(st, interactive),
        "db": lambda: check_db(st),
    }
    for name in want:
        if name not in fns:
            print(f"unknown check {name!r}")
            continue
        try:
            fns[name]()
        except Exception as exc:
            report(name, False, f"check crashed: {exc}")

    if "bus" in st:
        try:
            st["bus"].write_byte_data(st["addr"], REG_INT_ENABLE, 0x00)
            st["bus"].write_byte_data(st["addr"], REG_POWER_CTL, 0x00)
            st["bus"].close()
        except Exception:
            pass

    bad = [n for n, ok in results if ok is False]
    warn = [n for n, ok in results if ok is None]
    skip = [n for n, ok in results if ok == "skip"]
    good = len(results) - len(bad) - len(warn) - len(skip)
    print(f"\n{good} passed, {len(warn)} warnings, {len(bad)} failed"
          + (f", {len(skip)} skipped" if skip else ""))
    if bad:
        print(f"  must fix: {', '.join(bad)}")
    if warn:
        print(f"  look at : {', '.join(warn)}")
    if skip:
        print(f"  not run : {', '.join(skip)} (fix the failures above first)")
    if not bad:
        print("\nReady. Next: python3 wbv_shock_logger.py --selftest")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
