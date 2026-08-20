#!/usr/bin/env python3
"""
Read-only MAVLink link check. Connects, waits for a HEARTBEAT, prints basic
status, and exits. Does NOT arm, does NOT change mode, does NOT send RC
overrides or any command/param-set - it only listens. Safe to run with
propellers attached.

Usage:
    python3 check_fc_link.py --port /dev/ttyAMA10
    python3 check_fc_link.py --port /dev/ttyAMA10 --baud 921600   # skip the baud scan
"""
import argparse
import sys

from pymavlink import mavutil

COMMON_BAUDS = [57600, 921600, 115200, 460800, 500000, 1500000]


def try_connect(port, baud, timeout):
    print(f"[i] Trying {port} @ {baud} baud ...")
    m = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        m.close()
        return None
    return m


def main():
    ap = argparse.ArgumentParser(description="Read-only MAVLink link check (no arm, no mode change, no RC override)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyAMA10")
    ap.add_argument("--baud", type=int, default=None, help="omit to scan common bauds")
    ap.add_argument("--timeout", type=float, default=4.0, help="seconds to wait for a heartbeat per baud tried")
    args = ap.parse_args()

    bauds = [args.baud] if args.baud else COMMON_BAUDS
    m = None
    for baud in bauds:
        try:
            m = try_connect(args.port, baud, args.timeout)
        except Exception as e:
            print(f"[!] {baud}: {e}")
            m = None
        if m is not None:
            print(f"[i] Heartbeat received at {baud} baud.")
            break

    if m is None:
        print("[!] No heartbeat received on any baud tried. Link is NOT working (or wrong port/baud).")
        sys.exit(1)

    print(f"[i] target_system={m.target_system} target_component={m.target_component}")

    hb = m.messages.get("HEARTBEAT")
    if hb is not None:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        apname = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(hb.autopilot)
        typename = mavutil.mavlink.enums["MAV_TYPE"].get(hb.type)
        print(f"[i] autopilot={apname.name if apname else hb.autopilot} "
              f"type={typename.name if typename else hb.type} "
              f"armed={armed}")
        if armed:
            print("[!] WARNING: vehicle reports ARMED right now.")

    # Listen a couple more seconds purely passively to confirm the stream is
    # alive (not just a single stray packet), still no transmission from us.
    print("[i] Listening for a few more seconds to confirm a steady stream (read-only) ...")
    count = 0
    import time
    t_end = time.time() + 3.0
    while time.time() < t_end:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is not None:
            count += 1
    print(f"[i] Received {count} additional messages over 3s.")
    print("[i] Link check complete - nothing was sent to the vehicle except this app appearing as a MAVLink node.")
    m.close()


if __name__ == "__main__":
    main()
