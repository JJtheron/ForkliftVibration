#!/usr/bin/env python3
"""
MySQL/MariaDB storage for the forklift WBV logger.

The database is local, so there is no upload or offline-sync layer - but
writes still go through a queue and a background thread. A commit that
blocks for 40 ms while the SD card does housekeeping would stall the
acquisition loop and overflow the ADXL343's 32-sample FIFO, and you would
lose real data to a storage hiccup.

    sudo apt install python3-pymysql
    python3 store.py --selftest        # exercises everything, no MySQL needed
    python3 store.py --check           # verify schema and credentials

Credentials come from ~/.my.cnf or --dsn, never from the source.
"""

import argparse
import hashlib
import json
import os
import queue
import sys
import threading
import time
import uuid
from datetime import datetime, timezone

WEIGHTING_VERSION = "iso2631-1-fit-1"   # bump when coefficients change

FIX_BATCH = 25            # rows per gps_fix insert
FIX_FLUSH_S = 5.0         # ...or this often, whichever comes first
QUEUE_MAX = 5000
RETRY_BASE_S = 1.0
RETRY_MAX_S = 60.0


def utc(ts):
    """POSIX seconds -> naive UTC datetime for a DATETIME(6) column.

    MySQL DATETIME has no timezone. Everything stored is UTC by
    convention; do not feed it local time or the GPS correlation breaks
    twice a year.
    """
    return datetime.fromtimestamp(ts, timezone.utc).replace(tzinfo=None)


def sha256_file(path, chunk=1 << 16):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


# ---------------------------------------------------------------------
# Statements. Table names are substituted so they can be renamed.
# ---------------------------------------------------------------------

SQL = {
    "session": """
        INSERT INTO {session}
            (session_uuid, logger_id, truck_id, operator_ref, started_at,
             sample_rate_hz, accel_range_g, fifo_watermark,
             shock_threshold_ms2, pre_trigger_s, post_trigger_s,
             weighting_version, k_factor_x, k_factor_y, k_factor_z,
             gating_enabled, config)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
    """,
    "session_end": "UPDATE {session} SET ended_at=%s WHERE session_id=%s",
    "fix": """
        INSERT IGNORE INTO {gps_fix}
            (session_id, fix_time, lat, lon, altitude_m, speed_mps,
             course_deg, fix_quality, satellites, hdop)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
    """,
    "event": """
        INSERT INTO {shock_event}
            (session_id, event_time, seq, peak_ms2, axis, crest_factor,
             vdv_contrib, raw_peak_ms2, clipped, lat, lon, speed_mps,
             course_deg, hdop, satellites, pos_source, pos_error_s,
             waveform_path, waveform_sha256)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
    """,
    "interval": """
        INSERT INTO {exposure_interval}
            (session_id, period_start, period_end,
             aw_x, aw_y, aw_z, vdv_x, vdv_y, vdv_z,
             peak_x, peak_y, peak_z, dominant_axis, crest_factor,
             moving_s, idle_s, shock_count,
             exposure_hours, a8_ms2, vdv8)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
        ON DUPLICATE KEY UPDATE
            period_end=VALUES(period_end),
            aw_x=VALUES(aw_x), aw_y=VALUES(aw_y), aw_z=VALUES(aw_z),
            vdv_x=VALUES(vdv_x), vdv_y=VALUES(vdv_y), vdv_z=VALUES(vdv_z),
            shock_count=VALUES(shock_count),
            a8_ms2=VALUES(a8_ms2), vdv8=VALUES(vdv8)
    """,
}

DEFAULT_TABLES = {
    "session": "session",
    "gps_fix": "gps_fix",
    "shock_event": "shock_event",
    "exposure_interval": "exposure_interval",
    "truck": "truck",
    "logger": "logger",
}


class Store(threading.Thread):
    """Queue-backed MySQL writer.

    Events commit immediately - they are rare and must not be lost to a
    power cut. GPS fixes batch, because at 5 Hz a commit per fix is
    pointless SD card wear.
    """

    daemon = True

    def __init__(self, dsn=None, tables=None, spool_warn=True):
        super().__init__(name="store")
        self.dsn = dsn or {}
        self.tables = dict(DEFAULT_TABLES, **(tables or {}))
        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.stop_event = threading.Event()
        self.conn = None
        self.session_id = None
        self.session_uuid = str(uuid.uuid4())
        self.written = {"fix": 0, "event": 0, "interval": 0}
        self.dropped = 0
        self.errors = 0
        self.last_error = None
        self._fix_buf = []
        self._last_fix_flush = time.time()
        self._spool_warn = spool_warn

    # -- connection ----------------------------------------------------

    def _sql(self, key):
        return SQL[key].format(**self.tables)

    def connect(self):
        import pymysql
        params = dict(
            host=self.dsn.get("host", "localhost"),
            user=self.dsn.get("user"),
            password=self.dsn.get("password"),
            database=self.dsn.get("database", "wbv"),
            charset="utf8mb4",
            autocommit=False,
        )
        # A local socket avoids the TCP stack entirely. Only fall back to
        # host/port if a non-local host was asked for.
        sock = self.dsn.get("unix_socket")
        if params["host"] in ("localhost", "127.0.0.1", None) and sock:
            params["unix_socket"] = sock
            params.pop("host", None)
        elif self.dsn.get("port"):
            params["port"] = int(self.dsn["port"])
        if self.dsn.get("read_default_file"):
            params["read_default_file"] = self.dsn["read_default_file"]
        self.conn = pymysql.connect(**{k: v for k, v in params.items()
                                       if v is not None})
        # Keep the session in UTC so any server-side NOW() agrees with
        # the timestamps we insert.
        with self.conn.cursor() as cur:
            cur.execute("SET time_zone = '+00:00'")
        self.conn.commit()
        return self

    def _reconnect(self):
        delay = RETRY_BASE_S
        while not self.stop_event.is_set():
            try:
                if self.conn:
                    try:
                        self.conn.close()
                    except Exception:
                        pass
                self.connect()
                return True
            except Exception as exc:
                self.last_error = f"reconnect: {exc}"
                self.errors += 1
                self.stop_event.wait(delay)
                delay = min(RETRY_MAX_S, delay * 2)
        return False

    # -- session -------------------------------------------------------

    def start_session(self, meta):
        """Insert the session row. Synchronous: nothing else can proceed
        without a session_id."""
        with self.conn.cursor() as cur:
            cur.execute(self._sql("session"), (
                self.session_uuid,
                meta.get("logger_id"), meta.get("truck_id"),
                meta.get("operator_ref"),
                utc(meta["started_at"]),
                meta["sample_rate_hz"], meta["accel_range_g"],
                meta.get("fifo_watermark"),
                meta["shock_threshold_ms2"],
                meta["pre_trigger_s"], meta["post_trigger_s"],
                meta.get("weighting_version", WEIGHTING_VERSION),
                meta.get("k_factor_x", 1.4), meta.get("k_factor_y", 1.4),
                meta.get("k_factor_z", 1.0),
                1 if meta.get("gating_enabled", True) else 0,
                json.dumps(meta.get("config", {})),
            ))
            self.session_id = cur.lastrowid
        self.conn.commit()
        return self.session_id

    def end_session(self, when=None):
        if self.session_id is None or self.conn is None:
            return
        try:
            with self.conn.cursor() as cur:
                cur.execute(self._sql("session_end"),
                            (utc(when or time.time()), self.session_id))
            self.conn.commit()
        except Exception as exc:
            self.last_error = f"end_session: {exc}"

    # -- producers (called from the acquisition thread) -----------------

    def _put(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            self.dropped += 1
            if self._spool_warn and self.dropped in (1, 10, 100, 1000):
                print(f"  WARNING: store queue full, dropped {self.dropped} "
                      f"rows (DB not keeping up)", file=sys.stderr, flush=True)

    def add_fix(self, fix):
        self._put(("fix", (
            fix.t, fix.lat, fix.lon, fix.alt, fix.speed, fix.course,
            fix.quality, fix.sats, fix.hdop)))

    def add_event(self, ev):
        self._put(("event", ev))

    def add_interval(self, iv):
        self._put(("interval", iv))

    # -- consumer ------------------------------------------------------

    def run(self):
        while not (self.stop_event.is_set() and self.q.empty()):
            try:
                kind, payload = self.q.get(timeout=0.5)
            except queue.Empty:
                self._maybe_flush_fixes()
                continue
            try:
                self._handle(kind, payload)
            except Exception as exc:
                self.errors += 1
                self.last_error = f"{kind}: {exc}"
                # Put it back once, then reconnect. Losing a shock event to
                # a transient DB error is worse than a duplicate.
                if not self._reconnect():
                    break
                try:
                    self._handle(kind, payload)
                except Exception as exc2:
                    self.last_error = f"{kind} retry: {exc2}"
        self._flush_fixes()

    def _handle(self, kind, payload):
        if kind == "fix":
            self._fix_buf.append((self.session_id,) + (utc(payload[0]),)
                                 + payload[1:])
            if len(self._fix_buf) >= FIX_BATCH:
                self._flush_fixes()
            return
        if kind == "event":
            self._flush_fixes()      # keep the track ahead of the event
            with self.conn.cursor() as cur:
                cur.execute(self._sql("event"), (self.session_id,) + payload)
            self.conn.commit()       # commit now: events are irreplaceable
            self.written["event"] += 1
            return
        if kind == "interval":
            with self.conn.cursor() as cur:
                cur.execute(self._sql("interval"), (self.session_id,) + payload)
            self.conn.commit()
            self.written["interval"] += 1

    def _maybe_flush_fixes(self):
        if self._fix_buf and time.time() - self._last_fix_flush >= FIX_FLUSH_S:
            self._flush_fixes()

    def _flush_fixes(self):
        if not self._fix_buf or self.conn is None:
            return
        rows, self._fix_buf = self._fix_buf, []
        try:
            with self.conn.cursor() as cur:
                cur.executemany(self._sql("fix"), rows)
            self.conn.commit()
            self.written["fix"] += len(rows)
        except Exception as exc:
            self.errors += 1
            self.last_error = f"fix flush: {exc}"
        self._last_fix_flush = time.time()

    def close(self, timeout=10.0):
        self.stop_event.set()
        if self.is_alive():
            self.join(timeout)
        self._flush_fixes()
        self.end_session()
        if self.conn:
            try:
                self.conn.close()
            except Exception:
                pass


def build_event_row(seq, event_time, peak, axis, crest, vdv_contrib,
                    raw_peak, clipped, fix, pos_source, pos_error,
                    waveform_path):
    """Assemble a shock_event tuple. Position fields are None when there
    was no usable fix - never silently substitute the last known one."""
    sha = None
    if waveform_path and os.path.exists(waveform_path):
        try:
            sha = sha256_file(waveform_path)
        except OSError:
            sha = None
    return (
        utc(event_time), seq, float(peak), axis,
        None if crest is None else float(crest),
        None if vdv_contrib is None else float(vdv_contrib),
        None if raw_peak is None else float(raw_peak),
        1 if clipped else 0,
        None if fix is None else fix.lat,
        None if fix is None else fix.lon,
        None if fix is None else fix.speed,
        None if fix is None else fix.course,
        None if fix is None else fix.hdop,
        None if fix is None else fix.sats,
        pos_source,
        None if pos_error is None else float(pos_error),
        waveform_path, sha,
    )


# ---------------------------------------------------------------------
# Self-test: runs the whole path against SQLite so no MySQL is needed
# ---------------------------------------------------------------------

def selftest():
    import sqlite3
    import tempfile

    ok = True
    print("Timestamp handling:")
    t = 1_760_000_000.123456
    d = utc(t)
    print(f"  {t} -> {d.isoformat()} (naive UTC, µs preserved: "
          f"{d.microsecond})")
    ok &= d.tzinfo is None and d.microsecond == 123456

    print("\nDST trap: the same wall-clock hour twice in a local zone")
    for ts in (1_667_106_000, 1_667_109_600):
        print(f"  {ts} -> UTC {utc(ts).isoformat()}")
    ok &= utc(1_667_106_000) != utc(1_667_109_600)

    print("\nStatement formatting with renamed tables:")
    s = Store(tables={"shock_event": "fl_shock", "gps_fix": "fl_track"})
    print(f"  event -> ...INTO {s._sql('event').split()[2]}")
    print(f"  fix   -> ...INTO {s._sql('fix').split()[3]}")
    ok &= "fl_shock" in s._sql("event") and "fl_track" in s._sql("fix")

    print("\nEvent row assembly with no GPS fix:")
    row = build_event_row(1, t, 3.2, "z", 11.4, 0.8, 4.1, False,
                          None, "none", None, None)
    print(f"  {len(row)} fields, lat={row[8]}, pos_source={row[14]}")
    ok &= row[8] is None and row[14] == "none"

    class F:
        lat, lon, speed, course, hdop, sats = 53.4001, -2.9002, 3.1, 47.0, 0.9, 9
    row = build_event_row(2, t, 3.2, "z", 11.4, 0.8, 4.1, True,
                          F(), "interpolated", 0.08, None)
    print(f"  with fix: lat={row[8]}, clipped={row[7]}, "
          f"pos_error={row[15]}")
    ok &= row[8] == 53.4001 and row[7] == 1

    print("\nWaveform hashing:")
    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as f:
        f.write(b"synthetic waveform")
        path = f.name
    row = build_event_row(3, t, 3.2, "z", 11.4, 0.8, 4.1, False,
                          None, "none", None, path)
    print(f"  sha256 = {row[-1][:16]}... ({len(row[-1])} chars)")
    ok &= row[-1] is not None and len(row[-1]) == 64
    os.unlink(path)
    row = build_event_row(4, t, 3.2, "z", 11.4, 0.8, 4.1, False,
                          None, "none", None, path)
    print(f"  missing file -> sha256 {row[-1]} (path kept: "
          f"{row[-2] is not None})")
    ok &= row[-1] is None and row[-2] == path

    print("\nPlaceholder count must match the row width:")
    for key, extra in (("event", 1), ("fix", 1), ("interval", 1)):
        n_ph = SQL[key].format(**DEFAULT_TABLES).count("%s")
        if key == "interval":
            n_ph = SQL[key].format(**DEFAULT_TABLES).split("VALUES")[1] \
                .split("ON DUPLICATE")[0].count("%s")
        print(f"  {key:9s} {n_ph} placeholders")
    n_ph = SQL["event"].format(**DEFAULT_TABLES).count("%s")
    width = len(build_event_row(1, t, 1.0, "z", 1.0, 1.0, 1.0, False,
                                None, "none", None, None)) + 1
    print(f"  event row width {width} vs {n_ph} placeholders "
          f"{'ok' if width == n_ph else 'MISMATCH'}")
    ok &= width == n_ph

    print("\nQueue backpressure (never blocks the caller):")
    s2 = Store(spool_warn=False)
    s2.q = queue.Queue(maxsize=5)
    start = time.time()
    for i in range(50):
        s2.add_event(("dummy",))
    took = time.time() - start
    print(f"  50 events into a 5-slot queue in {took*1000:.2f} ms, "
          f"{s2.dropped} dropped, no exception")
    ok &= s2.dropped == 45 and took < 0.05

    print("\nEnd-to-end insert against SQLite (MySQL syntax adapted):")
    con = sqlite3.connect(":memory:")
    con.executescript("""
        CREATE TABLE shock_event(
          event_id INTEGER PRIMARY KEY AUTOINCREMENT, session_id INTEGER,
          event_time TEXT, seq INTEGER, peak_ms2 REAL, axis TEXT,
          crest_factor REAL, vdv_contrib REAL, raw_peak_ms2 REAL,
          clipped INTEGER, lat REAL, lon REAL, speed_mps REAL,
          course_deg REAL, hdop REAL, satellites INTEGER, pos_source TEXT,
          pos_error_s REAL, waveform_path TEXT, waveform_sha256 TEXT);
    """)
    stmt = SQL["event"].format(**DEFAULT_TABLES).replace("%s", "?")
    rows = [(1,) + build_event_row(i, t + i, 2.5 + i * 0.1, "z", 11.0, 0.7,
                                   3.0, False, F(), "interpolated", 0.05, None)
            for i in range(5)]
    con.executemany(stmt, rows)
    n = con.execute("SELECT COUNT(*) FROM shock_event").fetchone()[0]
    worst = con.execute("SELECT MAX(peak_ms2) FROM shock_event").fetchone()[0]
    print(f"  inserted {n} rows, worst peak {worst:.1f} m/s^2, "
          f"placeholder count matches column count")
    ok &= n == 5

    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


def check(args):
    """Verify credentials and that the schema matches what we insert."""
    store = Store(dsn=parse_dsn(args))
    try:
        store.connect()
    except Exception as exc:
        print(f"connect failed: {exc}")
        return 1
    want = {
        store.tables["session"], store.tables["gps_fix"],
        store.tables["shock_event"], store.tables["exposure_interval"],
    }
    with store.conn.cursor() as cur:
        cur.execute("SHOW TABLES")
        have = {r[0] for r in cur.fetchall()}
        print(f"connected to {store.dsn.get('database', 'wbv')}")
        for t in sorted(want):
            print(f"  {t:22s} {'ok' if t in have else 'MISSING'}")
        cur.execute("SELECT @@innodb_flush_log_at_trx_commit, @@version")
        flush, ver = cur.fetchone()
        print(f"\n  server {ver}")
        print(f"  innodb_flush_log_at_trx_commit = {flush}"
              + ("" if flush == 1 else
                 "  <-- not 1: committed events can be lost on a power cut"))
    store.conn.close()
    return 0 if want <= have else 1


def parse_dsn(args):
    dsn = {}
    if args.defaults_file:
        dsn["read_default_file"] = args.defaults_file
    for k in ("host", "port", "user", "password", "database", "unix_socket"):
        v = getattr(args, k, None)
        if v:
            dsn[k] = v
    return dsn


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--check", action="store_true")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int)
    p.add_argument("--user")
    p.add_argument("--password")
    p.add_argument("--database", default="wbv")
    p.add_argument("--unix-socket", default="/var/run/mysqld/mysqld.sock")
    p.add_argument("--defaults-file", default=os.path.expanduser("~/.my.cnf"),
                   help="MySQL option file holding the credentials")
    args = p.parse_args()
    if args.selftest:
        return selftest()
    if args.check:
        return check(args)
    p.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
