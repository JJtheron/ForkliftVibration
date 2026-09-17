#!/usr/bin/env python3
"""Web API and process supervisor for the forklift logger."""

import csv
import io
import os
import shlex
import signal
import subprocess
import threading
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

import pymysql
from flask import Flask, Response, jsonify, request

from Local_screen import open_display, show

app = Flask(__name__)
PACIFIC_TZ = ZoneInfo("America/Los_Angeles")

DB = {
    "host": os.environ.get("DB_HOST", "mysql-server"),
    "port": int(os.environ.get("DB_PORT", "3306")),
    "database": os.environ.get("DB_NAME", "wbv"),
    "user": os.environ.get("DB_USER"),
    "password": os.environ.get("DB_PASSWORD"),
    "cursorclass": pymysql.cursors.DictCursor,
    "connect_timeout": 5,
}


def db_connection():
    return pymysql.connect(**DB)


def query(sql, params=()):
    with db_connection() as db:
        with db.cursor() as cursor:
            cursor.execute(sql, params)
            return cursor.fetchall()


def to_pacific(value):
    if value is None:
        return None
    if isinstance(value, str):
        try:
            value = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return value
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    return value.astimezone(PACIFIC_TZ)


def serialize_rows(rows):
    out = []
    for row in rows:
        converted = {}
        for key, value in row.items():
            if isinstance(value, datetime):
                converted[key] = to_pacific(value).isoformat(timespec="seconds")
            else:
                converted[key] = value
        out.append(converted)
    return out


def latest_readings():
    sessions = query(
        "SELECT session_id, started_at, ended_at FROM session "
        "ORDER BY session_id DESC LIMIT 1"
    )
    if not sessions:
        return None
    session_id = sessions[0]["session_id"]
    fixes = query(
        "SELECT fix_time, lat, lon, speed_mps, satellites, hdop "
        "FROM gps_fix WHERE session_id=%s ORDER BY fix_time DESC LIMIT 1",
        (session_id,),
    )
    events = query(
        "SELECT event_time, peak_ms2, axis, lat, lon, pos_source "
        "FROM shock_event WHERE session_id=%s ORDER BY event_time DESC LIMIT 1",
        (session_id,),
    )
    result = {"session": sessions[0], "gps": fixes[0] if fixes else None,
              "last_shock": events[0] if events else None}
    session = result["session"]
    if session.get("started_at") is not None:
        session["started_at"] = to_pacific(session["started_at"]).isoformat(timespec="seconds")
    if session.get("ended_at") is not None:
        session["ended_at"] = to_pacific(session["ended_at"]).isoformat(timespec="seconds")
    if result["gps"] is not None:
        result["gps"]["fix_time"] = to_pacific(result["gps"]["fix_time"]).isoformat(timespec="seconds")
    if result["last_shock"] is not None:
        result["last_shock"]["event_time"] = to_pacific(result["last_shock"]["event_time"]).isoformat(timespec="seconds")
    return result


@app.get("/api/health")
def health():
    try:
        query("SELECT 1")
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 503


@app.get("/api/status")
def status():
    try:
        return jsonify(latest_readings() or {})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 503


@app.get("/api/events")
def events():
    limit = min(max(request.args.get("limit", 100, type=int), 1), 1000)
    try:
        rows = query(
            "SELECT event_id, event_time, peak_ms2, axis, crest_factor, "
            "lat, lon, speed_mps, pos_source, clipped "
            "FROM shock_event ORDER BY event_time DESC LIMIT %s", (limit,)
        )
        return jsonify(serialize_rows(rows))
    except Exception as exc:
        return jsonify({"error": str(exc)}), 503


@app.get("/api/events.csv")
def events_csv():
    """Download shock values and GPS coordinates for mapping."""
    day = request.args.get("date")
    if day:
        where = "WHERE DATE(CONVERT_TZ(event_time, '+00:00', '-08:00')) = %s"
        params = (day,)
    else:
        where = ""
        params = ()
    try:
        rows = query(
            "SELECT event_time, peak_ms2, axis, crest_factor, lat, lon, "
            "speed_mps, pos_source, pos_error_s, clipped "
            f"FROM shock_event {where} ORDER BY event_time", params
        )
    except Exception as exc:
        return jsonify({"error": str(exc)}), 503
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["pst", "shock_ms2", "axis", "crest_factor", "latitude",
                     "longitude", "speed_mps", "position_source",
                     "position_error_s", "clipped"])
    for row in rows:
        pst = to_pacific(row["event_time"]).isoformat(timespec="seconds")
        writer.writerow([pst, row["peak_ms2"], row["axis"],
                         row["crest_factor"], row["lat"], row["lon"],
                         row["speed_mps"], row["pos_source"],
                         row["pos_error_s"], row["clipped"]])
    return Response(output.getvalue(), mimetype="text/csv", headers={
        "Content-Disposition": "attachment; filename=forklift-shocks.csv"
    })


def display_loop(stop_event):
    display = open_display()
    while not stop_event.wait(2.0):
        try:
            reading = latest_readings()
            gps = reading and reading["gps"]
            shock = reading and reading["last_shock"]
            line_one = "GPS: no fix" if not gps else \
                f"{gps['lat']:.5f},{gps['lon']:.5f}"
            line_two = "Shock: none" if not shock else \
                f"Shock {shock['peak_ms2']:.1f}{shock['axis']}"
            show(display, line_one, line_two)
        except Exception as exc:
            print(f"Display data unavailable: {exc}")
    show(display, "Logger stopped", "")


def start_logger():
    command = ["python", "Accelerometer_int.py", "--poll", "--gps", "--db",
               "--db-host", DB["host"], "--db-port", str(DB["port"]),
               "--db-name", DB["database"]]
    if DB["user"]:
        command.extend(["--db-user", DB["user"]])
    command.extend(shlex.split(os.environ.get("ACCELERATOR_ARGS", "")))
    env = dict(os.environ, PYTHONUNBUFFERED="1",
               WBV_OUT_DIR=os.environ.get("WBV_OUT_DIR", "/data/wbv_events"),
               PYTHONPATH="/app")
    return subprocess.Popen(command, env=env)


def main():
    logger = start_logger()
    stop_event = threading.Event()
    threading.Thread(target=display_loop, args=(stop_event,), daemon=True).start()

    def shutdown(signum, _frame):
        stop_event.set()
        if logger.poll() is None:
            logger.send_signal(signum)
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        stop_event.set()
        if logger.poll() is None:
            logger.terminate()
            try:
                logger.wait(timeout=15)
            except subprocess.TimeoutExpired:
                logger.kill()


if __name__ == "__main__":
    main()
