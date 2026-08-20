#!/usr/bin/env python3
"""
Read-only dump of every SERIALx_PROTOCOL/BAUD/OPTIONS parameter, to figure
out which SERIALx corresponds to a physically-wired port and how it's
configured. No writes, no arm, no mode change, no RC override.

Usage:
    python3 check_serial_ports.py --connect /dev/ttyACM0 --baud 115200
"""
import argparse
import time

from pymavlink import mavutil


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t = time.time()
    while time.time() - t < timeout:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if p is not None and p.param_id.strip("\x00") == name:
            return p.param_value
    return None


PROTOCOL_NAMES = {
    0: "None", 1: "MAVLink1", 2: "MAVLink2", 5: "GPS", 23: "RCIN",
    28: "MAVLink High Latency", 32: "DisplayPort",
}


def main():
    ap = argparse.ArgumentParser(description="Read-only SERIALx_* dump (no writes)")
    ap.add_argument("--connect", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print(f"[i] Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    print(f"[i] Heartbeat ok.\n")

    for i in range(0, 8):
        proto = get_param(m, f"SERIAL{i}_PROTOCOL")
        if proto is None:
            continue
        baud = get_param(m, f"SERIAL{i}_BAUD")
        opts = get_param(m, f"SERIAL{i}_OPTIONS")
        pname = PROTOCOL_NAMES.get(int(proto), f"other({int(proto)})")
        opts_int = int(opts) if opts is not None else None
        opts_bin = f"0b{opts_int:08b}" if opts_int is not None else "?"
        print(f"SERIAL{i}: protocol={pname:22s} baud={baud} options={opts_int} ({opts_bin})")

    m.close()


if __name__ == "__main__":
    main()
