#!/usr/bin/env python3
"""
Adafruit Ultimate GPS HAT (MTK3339) reader and position interpolator.

The HAT is on the Pi's UART, so before this works:
    sudo raspi-config    -> Interface Options -> Serial Port
                            login shell over serial: NO
                            serial port hardware:    YES
    reboot
Device is /dev/serial0. Do not use /dev/ttyAMA0 directly on a Pi 3/4/5 -
that is the Bluetooth UART on those boards.

Shock events are resolved to 1.25 ms; GPS updates at 1-10 Hz. Positions
are therefore interpolated between bracketing fixes, and every event
records how far it had to reach (pos_error_s) and by what method.

    python3 gps.py --selftest          # parser + interpolator, no hardware
    python3 gps.py --dump              # live fixes from the HAT
"""

import argparse
import math
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone, timedelta

DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 9600
TARGET_BAUD = 57600        # 9600 cannot carry RMC+GGA at 5 Hz
TARGET_RATE_MS = 200       # 5 Hz. 100 = 10 Hz, 1000 = 1 Hz

TRACK_SECONDS = 600        # how much fix history to keep for interpolation
MAX_INTERP_GAP_S = 3.0     # beyond this, do not interpolate across the hole
MAX_NEAREST_GAP_S = 10.0   # beyond this, report no position at all


# ---------------------------------------------------------------------
# NMEA
# ---------------------------------------------------------------------

def nmea_checksum(body):
    """XOR of everything between $ and *."""
    c = 0
    for ch in body:
        c ^= ord(ch)
    return c


def pmtk(command):
    """Wrap a PMTK command with its checksum and terminator."""
    return f"${command}*{nmea_checksum(command):02X}\r\n".encode("ascii")


def _dm_to_deg(value, hemi):
    """NMEA ddmm.mmmm -> signed decimal degrees."""
    if not value or not hemi:
        return None
    dot = value.find(".")
    if dot < 3:
        return None
    deg = int(value[:dot - 2])
    minutes = float(value[dot - 2:])
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out


def _f(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _i(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


class Fix:
    __slots__ = ("t", "lat", "lon", "alt", "speed", "course",
                 "quality", "sats", "hdop")

    def __init__(self, t, lat, lon, alt=None, speed=None, course=None,
                 quality=None, sats=None, hdop=None):
        self.t = t                  # POSIX seconds, UTC, from the sentence
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.speed = speed          # m/s
        self.course = course        # degrees true
        self.quality = quality
        self.sats = sats
        self.hdop = hdop

    def __repr__(self):
        return (f"Fix({datetime.fromtimestamp(self.t, timezone.utc):%H:%M:%S.%f}"
                f" {self.lat:.6f},{self.lon:.6f} "
                f"{'?' if self.speed is None else f'{self.speed:.1f}'} m/s)")


KNOTS_TO_MPS = 0.514444


def parse_sentence(line):
    """Parse one NMEA sentence.

    Returns ('rmc'|'gga', dict) or None. Talker ID is ignored so GP/GN/GL
    prefixes all work - the HAT emits GP, but multi-constellation
    receivers emit GN and the field layout is identical.
    """
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, given = line[1:].partition("*")
    try:
        if int(given[:2], 16) != nmea_checksum(body):
            return None
    except ValueError:
        return None

    parts = body.split(",")
    kind = parts[0][2:] if len(parts[0]) >= 5 else parts[0]

    if kind == "RMC" and len(parts) >= 10:
        if parts[2] != "A":                 # V = warning, fix not valid
            return ("rmc", {"valid": False})
        t = _nmea_datetime(parts[9], parts[1])
        speed = _f(parts[7])
        return ("rmc", {
            "valid": True,
            "t": t,
            "lat": _dm_to_deg(parts[3], parts[4]),
            "lon": _dm_to_deg(parts[5], parts[6]),
            "speed": None if speed is None else speed * KNOTS_TO_MPS,
            "course": _f(parts[8]),
        })

    if kind == "GGA" and len(parts) >= 10:
        return ("gga", {
            "quality": _i(parts[6]),
            "sats": _i(parts[7]),
            "hdop": _f(parts[8]),
            "alt": _f(parts[9]),
        })

    return None


def _nmea_datetime(ddmmyy, hhmmss):
    """Combine the RMC date and time fields into POSIX seconds UTC."""
    if not ddmmyy or not hhmmss or len(ddmmyy) < 6 or len(hhmmss) < 6:
        return None
    try:
        day, mon, yr = int(ddmmyy[0:2]), int(ddmmyy[2:4]), int(ddmmyy[4:6])
        hh, mm = int(hhmmss[0:2]), int(hhmmss[2:4])
        ss = float(hhmmss[4:])
    except ValueError:
        return None
    dt = datetime(2000 + yr, mon, day, hh, mm, int(ss),
                  int(round((ss % 1) * 1e6)), tzinfo=timezone.utc)
    return dt.timestamp()


# ---------------------------------------------------------------------
# Track: fix history + interpolation
# ---------------------------------------------------------------------

class Track:
    """Thread-safe fix history that can be queried at an arbitrary time."""

    def __init__(self, keep_s=TRACK_SECONDS):
        self.keep_s = keep_s
        self._fixes = deque()
        self._lock = threading.Lock()
        self.count = 0

    def add(self, fix):
        with self._lock:
            # sentences can arrive out of order after a buffer hiccup
            if self._fixes and fix.t <= self._fixes[-1].t:
                return False
            self._fixes.append(fix)
            self.count += 1
            cutoff = fix.t - self.keep_s
            while self._fixes and self._fixes[0].t < cutoff:
                self._fixes.popleft()
            return True

    def latest(self):
        with self._lock:
            return self._fixes[-1] if self._fixes else None

    def snapshot(self):
        with self._lock:
            return list(self._fixes)

    def at(self, t):
        """Position at time t.

        Returns (fix, source, error_seconds) where source is one of
        'interpolated', 'nearest', 'stale' or 'none'. error_seconds is the
        gap to the nearest real fix, so downstream can weight by it.
        """
        with self._lock:
            fixes = list(self._fixes)
        if not fixes:
            return None, "none", None

        # bracketing pair
        before = after = None
        for f in fixes:
            if f.t <= t:
                before = f
            elif after is None:
                after = f
                break

        if before is not None and after is not None:
            gap = after.t - before.t
            err = min(t - before.t, after.t - t)
            if gap <= MAX_INTERP_GAP_S:
                return _interp(before, after, t), "interpolated", err
            # hole in the track: fall back to whichever edge is closer
            near = before if (t - before.t) <= (after.t - t) else after
            return near, "nearest", err

        near = before or after
        err = abs(t - near.t)
        if err <= MAX_INTERP_GAP_S:
            return near, "nearest", err
        if err <= MAX_NEAREST_GAP_S:
            return near, "stale", err
        return None, "none", err


def _interp(a, b, t):
    span = b.t - a.t
    if span <= 0:
        return a
    w = (t - a.t) / span

    lat = a.lat + (b.lat - a.lat) * w
    # longitude: take the short way round, in case of a meridian crossing
    dlon = (b.lon - a.lon + 180.0) % 360.0 - 180.0
    lon = (a.lon + dlon * w + 180.0) % 360.0 - 180.0

    def lerp(x, y):
        return None if x is None or y is None else x + (y - x) * w

    course = None
    if a.course is not None and b.course is not None:
        d = (b.course - a.course + 180.0) % 360.0 - 180.0
        course = (a.course + d * w) % 360.0

    return Fix(t, lat, lon,
               alt=lerp(a.alt, b.alt),
               speed=lerp(a.speed, b.speed),
               course=course,
               quality=b.quality if w > 0.5 else a.quality,
               sats=b.sats if w > 0.5 else a.sats,
               hdop=lerp(a.hdop, b.hdop))


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


# ---------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------

class GPSReader(threading.Thread):
    """Background NMEA reader. Feeds a Track; never blocks the caller."""

    daemon = True

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD,
                 target_baud=TARGET_BAUD, rate_ms=TARGET_RATE_MS,
                 track=None, on_fix=None):
        super().__init__(name="gps")
        self.port = port
        self.baud = baud
        self.target_baud = target_baud
        self.rate_ms = rate_ms
        self.track = track if track is not None else Track()
        self.on_fix = on_fix
        self.stop_event = threading.Event()
        self.ser = None
        self.sentences = 0
        self.bad_checksums = 0
        self.last_error = None
        self._pending = {}       # GGA fields waiting for their RMC

    def open(self):
        import serial
        # The HAT keeps its baud across a warm reboot but resets to 9600 on
        # power loss, so try the target first and fall back.
        for baud in (self.target_baud, self.baud):
            ser = serial.Serial(self.port, baud, timeout=1)
            time.sleep(0.2)
            ser.reset_input_buffer()
            if self._talking(ser):
                self.ser = ser
                break
            ser.close()
        else:
            raise RuntimeError(
                f"no NMEA on {self.port} at {self.target_baud} or {self.baud} "
                f"(serial console still enabled?)")

        if self.ser.baudrate != self.target_baud:
            self.ser.write(pmtk(f"PMTK251,{self.target_baud}"))
            self.ser.flush()
            time.sleep(0.3)
            self.ser.baudrate = self.target_baud
            time.sleep(0.2)
            self.ser.reset_input_buffer()

        # Enable SBAS/WAAS corrections when a compatible correction source is
        # visible. This usually improves open-sky accuracy modestly.
        self.ser.write(pmtk("PMTK313,1"))
        time.sleep(0.1)
        self.ser.write(pmtk("PMTK301,2"))
        time.sleep(0.1)

        # RMC + GGA only; everything else wastes bandwidth
        self.ser.write(pmtk("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"))
        time.sleep(0.1)
        self.ser.write(pmtk(f"PMTK220,{self.rate_ms}"))
        time.sleep(0.1)
        return self

    @staticmethod
    def _talking(ser, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = ser.readline().decode("ascii", "ignore")
            except Exception:
                return False
            if line.startswith("$"):
                return True
        return False

    def run(self):
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline().decode("ascii", "ignore")
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.5)
                continue
            if not raw:
                continue
            self.sentences += 1
            parsed = parse_sentence(raw)
            if parsed is None:
                if raw.startswith("$"):
                    self.bad_checksums += 1
                continue
            kind, data = parsed
            if kind == "gga":
                self._pending = data
            elif kind == "rmc" and data.get("valid"):
                if data["t"] is None or data["lat"] is None:
                    continue
                fix = Fix(data["t"], data["lat"], data["lon"],
                          alt=self._pending.get("alt"),
                          speed=data["speed"], course=data["course"],
                          quality=self._pending.get("quality"),
                          sats=self._pending.get("sats"),
                          hdop=self._pending.get("hdop"))
                if self.track.add(fix) and self.on_fix:
                    try:
                        self.on_fix(fix)
                    except Exception as exc:
                        self.last_error = f"on_fix: {exc}"

    def stop(self):
        self.stop_event.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


# ---------------------------------------------------------------------
# Self-test and live dump
# ---------------------------------------------------------------------

def _sentence(body):
    """Build a valid sentence from a body, computing its checksum."""
    return f"${body}*{nmea_checksum(body):02X}"


# Bodies only - checksums are computed, never hand-written.
SAMPLE_BODIES = [
    "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
    "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",
    "GPRMC,123519,V,,,,,,,230394,,",
    "GNRMC,123520,A,4807.050,N,01131.020,E,022.4,084.4,230394,003.1,W",
]


def selftest():
    ok = True
    print("Checksum against the canonical NMEA examples:")
    for body, expect in (
        ("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W", 0x6A),
        ("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", 0x47),
    ):
        got = nmea_checksum(body)
        good = got == expect
        ok &= good
        print(f"  {body.split(',')[0]}  computed {got:02X}, "
              f"published {expect:02X}  {'ok' if good else 'MISMATCH'}")

    print("\nPMTK framing:")
    for c in ("PMTK220,200", "PMTK251,57600"):
        print(f"  {pmtk(c).decode().strip()}")

    print("\nParsing:")
    seen = set()
    for body in SAMPLE_BODIES:
        line = _sentence(body)
        r = parse_sentence(line)
        tag = body.split(",")[0]
        if r:
            seen.add(r[0])
        print(f"  {tag:9s} -> {r if r is None else r[0]}"
              f"{'' if r is None else ' ' + str(r[1])[:66]}")
    ok &= {"rmc", "gga"} <= seen

    print("\nCorrupt sentences must be rejected:")
    good = _sentence(SAMPLE_BODIES[1])
    for label, bad in (("bad checksum", good[:-2] + "00"),
                       ("truncated", good[:20]),
                       ("no checksum", good.split("*")[0]),
                       ("garbage", "not a sentence")):
        r = parse_sentence(bad)
        print(f"  {label:14s} -> {r}")
        ok &= r is None

    print("\nInvalid-fix RMC (status V) must not yield a position:")
    r = parse_sentence(_sentence(SAMPLE_BODIES[2]))
    print(f"  {r}")
    ok &= r is not None and r[1].get("valid") is False

    print("\nCoordinate conversion:")
    lat = _dm_to_deg("4807.038", "N")
    lon = _dm_to_deg("01131.000", "E")
    print(f"  4807.038,N -> {lat:.6f}   (expect 48.117300)")
    print(f"  01131.000,E -> {lon:.6f}   (expect 11.516667)")
    ok &= abs(lat - 48.1173) < 1e-6 and abs(lon - 11.5166667) < 1e-6
    print(f"  01131.000,W -> {_dm_to_deg('01131.000', 'W'):.6f}  (sign flip)")

    print("\nInterpolation, 5 Hz track, event between fixes:")
    tr = Track()
    t0 = 1_700_000_000.0
    for i in range(25):
        tr.add(Fix(t0 + i * 0.2, 53.4000 + i * 0.000018, -2.9000 + i * 0.000010,
                   speed=3.0, course=45.0, hdop=0.9, sats=9, quality=1))
    for dt, label in ((2.10, "mid-interval"), (2.00, "exactly on a fix"),
                      (5.50, "past the end"), (30.0, "way past the end")):
        f, src, err = tr.at(t0 + dt)
        pos = "-" if f is None else f"{f.lat:.6f},{f.lon:.6f}"
        print(f"  t0+{dt:5.2f}s  {src:13s} err {err:6.3f}s  {pos}")
        if label == "mid-interval":
            mid = tr.at(t0 + 2.10)[0]
            a = tr.at(t0 + 2.00)[0]
            b = tr.at(t0 + 2.20)[0]
            expect = (a.lat + b.lat) / 2
            print(f"                 midpoint check: {mid.lat:.9f} vs "
                  f"{expect:.9f}")
            ok &= abs(mid.lat - expect) < 1e-9

    print("\nGap handling (3 s hole in the track):")
    tr2 = Track()
    tr2.add(Fix(t0, 53.4, -2.9, speed=3.0))
    tr2.add(Fix(t0 + 4.0, 53.41, -2.91, speed=3.0))
    f, src, err = tr2.at(t0 + 2.0)
    print(f"  event mid-hole -> {src}, err {err:.2f}s "
          f"(not interpolated across a {4.0:.0f}s gap)")
    ok &= src == "nearest"

    print("\nDistance sanity:")
    d = haversine_m(53.4000, -2.9000, 53.4000, -2.8990)
    print(f"  0.001 deg lon at 53.4N = {d:.1f} m (expect ~66 m)")
    ok &= 60 < d < 72

    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


def dump(args):
    reader = GPSReader(args.port, args.baud, args.target_baud,
                       args.rate_ms).open()
    reader.start()
    print(f"Reading {args.port}. Cold start can take minutes; the fix LED "
          f"drops to one blink per 15 s once locked.\n")
    try:
        while True:
            time.sleep(1.0)
            f = reader.track.latest()
            if f is None:
                print(f"\r  no fix yet - {reader.sentences} sentences, "
                      f"{reader.bad_checksums} bad", end="", flush=True)
            else:
                age = time.time() - f.t
                print(f"\r  {f.lat:.6f},{f.lon:.6f}  "
                      f"{f.speed or 0:.1f} m/s  {f.sats or 0} sats  "
                      f"hdop {f.hdop or 0:.1f}  age {age:+.1f}s   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        reader.stop()
        print()
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--dump", action="store_true")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--target-baud", type=int, default=TARGET_BAUD)
    p.add_argument("--rate-ms", type=int, default=TARGET_RATE_MS)
    args = p.parse_args()
    if args.selftest:
        return selftest()
    if args.dump:
        return dump(args)
    p.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
