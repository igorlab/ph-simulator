#!/usr/bin/env python3
"""Log every live reading of the calibrator to a CSV file.

The page's own "Download CSV" keeps only time, pH and S. A measurement also needs E, the
board input, the leakage and the calibration line. All of them are in /data, the JSON the
page itself polls, so this script polls it too and writes every field it finds.

    python3 tools/calibrator_log.py -o run.csv                 # until Ctrl+C
    python3 tools/calibrator_log.py -o noise.csv -d 3600       # one hour

While it runs, type a short note and press Enter ("Rt 1G in", "E +100"): it goes into the
"mark" column of every following row, so the phases of an experiment can be told apart.

Only GET /data is sent. The calibrator is not reset and its settings are not touched.
Fields of /data: e - E, mV; s - S, V; vin - board input, mV; il - leakage, nA; noise - noise
of S, mV; gain, zero - the calibration line; r - series resistor, MΩ; i - board current, mA.
"""
import argparse
import csv
import json
import os
import select
import socket
import sys
import time
import urllib.request
from datetime import datetime

ap = argparse.ArgumentParser(description="Log the calibrator's /data to CSV.")
ap.add_argument("-o", "--out", required=True,
                help="CSV file to create, folders included (never overwritten)")
ap.add_argument("-H", "--host", default="calibrator.local",
                help="name or IP, optionally :port (calibrator.local)")
ap.add_argument("-i", "--interval", type=float, default=1.0, help="seconds between readings (1)")
ap.add_argument("-d", "--duration", type=float, default=0, help="stop after this many seconds")
ap.add_argument("-m", "--mark", default="", help="initial text of the mark column")
args = ap.parse_args()

host, _, port = args.host.partition(":")
ip = socket.gethostbyname(host)               # resolve once: mDNS lookups are slow
url = f"http://{ip}{':' + port if port else ''}/data"
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
try:
    f = open(args.out, "x", newline="")
except FileExistsError:
    sys.exit(f"{args.out} already exists: choose another name")
print(f"logging {url} to {args.out} every {args.interval:g} s; type a note + Enter to mark, "
      "Ctrl+C to stop", file=sys.stderr)
w = None
keys = []
mark = args.mark
t0 = time.monotonic()
next_t = t0
rows = fails = 0
stdin_open = True
try:
    while not args.duration or time.monotonic() - t0 < args.duration:
        wait = max(0.0, next_t - time.monotonic())
        if stdin_open:
            ready = select.select([sys.stdin], [], [], wait)[0]
        else:
            time.sleep(wait)
            ready = []
        if ready:
            line = sys.stdin.readline()
            if line:
                mark = line.strip()
                print(f"mark: {mark}", file=sys.stderr)
            else:
                stdin_open = False                 # stdin closed: keep logging without marks
            continue
        next_t = max(next_t + args.interval, time.monotonic() - args.interval)  # no burst after an outage
        try:
            with urllib.request.urlopen(url, timeout=max(2.0, args.interval * 2)) as r:
                d = json.load(r)
        except (OSError, ValueError) as e:
            fails += 1
            print(f"no reading ({e})", file=sys.stderr)
            continue
        if w is None:
            keys = list(d)
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["time", "t_s", "mark"] + keys)
        now = time.monotonic()
        w.writerow([datetime.now().isoformat(timespec="milliseconds"), f"{now - t0:.2f}", mark]
                   + [d.get(k, "") for k in keys])
        f.flush()
        rows += 1
        if rows % 10 == 1:
            print(f"{now - t0:7.0f} s  E {d.get('e')} mV  S {d.get('s')} V  Vin {d.get('vin')} mV  "
                  f"I {d.get('il')} nA  [{mark}]", file=sys.stderr)
except KeyboardInterrupt:
    pass
finally:
    f.close()
    if not rows:
        os.remove(args.out)                        # nothing logged: leave no empty file
    print(f"{rows} rows written to {args.out}, {fails} failed readings", file=sys.stderr)
