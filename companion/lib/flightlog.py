"""Lightweight per-run logger: a CSV of every control-loop cycle plus a
parallel event log (STATUSTEXT, ACKs, arm/mode/land transitions). Used by
ctbr_acro_rc.py so a flight can be reconstructed afterward instead of relying
on whatever scrolled past in a terminal - which is exactly what made
diagnosing the 2026-08-21 altitude-ramp incident slower than it needed to be:
the 1Hz console print was enough that time, but only barely.

One instance per run. Files are timestamped so nothing overwrites a previous
flight's data:
    <log_dir>/<name>_<YYYYMMDD_HHMMSS>.csv
    <log_dir>/<name>_<YYYYMMDD_HHMMSS>_events.log
"""
import csv
import datetime
import os
import time


class FlightLogger:
    def __init__(self, name, columns, log_dir=None):
        self.log_dir = log_dir or os.path.join(
            os.path.expanduser("~"), "drone-ops", "companion", "logs")
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"{name}_{stamp}.csv")
        self.event_path = os.path.join(self.log_dir, f"{name}_{stamp}_events.log")

        self._csv_f = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_f)
        self._writer.writerow(["t_wall", "t_rel"] + list(columns))
        self._event_f = open(self.event_path, "w")

        self.t0 = time.time()
        print(f"[log] cycle data : {self.csv_path}")
        print(f"[log] events      : {self.event_path}")
        self.event("log opened")

    def row(self, *values):
        now = time.time()
        self._writer.writerow([f"{now:.3f}", f"{now - self.t0:.3f}", *values])

    def event(self, text):
        now = time.time()
        line = f"{now - self.t0:8.3f}s  {text}"
        print(f"    [log] {line}")
        self._event_f.write(line + "\n")
        self._event_f.flush()

    def close(self, reason="normal exit"):
        self.event(f"log closed ({reason})")
        self._csv_f.close()
        self._event_f.close()
