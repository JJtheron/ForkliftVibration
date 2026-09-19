import csv
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

import app


def test_read_local_events_csv(tmp_path):
    csv_path = tmp_path / "events.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["utc", "event", "peak_ms2", "axis", "crest", "file"])
        writer.writerow([
            "2026-09-16T19:54:19.318003+00:00",
            1,
            "4.490",
            "z",
            "1.50",
            "shock_20260916T195419_00001.npz",
        ])

    rows = app.read_local_events(csv_path)

    assert len(rows) == 1
    assert rows[0]["peak_ms2"] == 4.49
    assert rows[0]["axis"] == "z"
    assert rows[0]["event_time"].startswith("2026-09-16T19:54:19")


def test_events_endpoint_falls_back_to_csv(tmp_path, monkeypatch):
    csv_path = tmp_path / "events.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["utc", "event", "peak_ms2", "axis", "crest", "file"])
        writer.writerow([
            "2026-09-16T19:54:19.318003+00:00",
            1,
            "4.490",
            "z",
            "1.50",
            "shock_20260916T195419_00001.npz",
        ])

    monkeypatch.setattr(app, "query", lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("db down")))
    monkeypatch.setattr(app, "resolve_event_csv_path", lambda: str(csv_path))

    client = app.app.test_client()
    response = client.get("/api/events?limit=10")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload[0]["axis"] == "z"
    assert payload[0]["peak_ms2"] == 4.49
