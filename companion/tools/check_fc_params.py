#!/usr/bin/env python3
"""
Read-only inspection of the exact parameters ctbr_acro_rc.py relies on, plus
board/version info. Does NOT arm, does NOT change mode, does NOT send RC
overrides, and does NOT set/write any parameter - only PARAM_REQUEST_READ
and a request for AUTOPILOT_VERSION, both purely informational. Safe to run
with propellers attached.

Usage:
    python3 check_fc_params.py --connect /dev/ttyAMA0 --baud 57600
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


def main():
    ap = argparse.ArgumentParser(description="Read-only param + board check (no arm, no writes)")
    ap.add_argument("--connect", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    args = ap.parse_args()

    print(f"[i] Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    print(f"[i] Heartbeat: target_system={m.target_system} target_component={m.target_component}")

    hb = m.messages.get("HEARTBEAT")
    if hb is not None:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[i] armed={armed}")
        if armed:
            print("[!] WARNING: vehicle reports ARMED right now.")

    # ---- board / firmware identity (read-only request) ----
    print("\n[i] Requesting AUTOPILOT_VERSION ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                             mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                             0, 0, 0, 0, 0, 0)
    av = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5.0)
    if av is None:
        print("[!] No AUTOPILOT_VERSION reply (some boards don't populate this).")
    else:
        def ver(v):
            return f"{(v>>24)&0xff}.{(v>>16)&0xff}.{(v>>8)&0xff}"
        print(f"    flight_sw_version = {ver(av.flight_sw_version)} (raw {av.flight_sw_version})")
        print(f"    board_version     = {av.board_version}")
        print(f"    vendor_id/product_id = {av.vendor_id}/{av.product_id}")
        print(f"    uid2 (board id, if set) = {bytes(av.uid2).hex() if hasattr(av, 'uid2') else 'n/a'}")
        print("    (MAVLink doesn't expose the MCU part number like 'F745' directly - that's a "
              "hwdef-level detail. board_version/vendor_id/product_id above is what's actually "
              "advertised; the physical board silkscreen or `dmesg`/bootlog on the FC itself, not "
              "reachable from here, would confirm the exact chip.)")

    # ---- the exact params ctbr_acro_rc.py reads ----
    print("\n[i] Reading the parameters ctbr_acro_rc.py depends on ...")
    names = ["ACRO_RP_RATE", "ACRO_Y_RATE", "ACRO_RP_EXPO", "ACRO_Y_EXPO", "ACRO_TRAINER",
             "MOT_THST_HOVER", "THR_MID",
             "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"]
    values = {}
    for n in names:
        v = get_param(m, n)
        values[n] = v
        status = "OK" if v is not None else "NO REPLY (script would fall back to a hardcoded default)"
        print(f"    {n:16s} = {v}   [{status}]")

    cmap = {"ROLL": int(values.get("RCMAP_ROLL") or 1), "PITCH": int(values.get("RCMAP_PITCH") or 2),
            "THROTTLE": int(values.get("RCMAP_THROTTLE") or 3), "YAW": int(values.get("RCMAP_YAW") or 4)}
    print(f"\n[i] Channel mapping: roll=RC{cmap['ROLL']} pitch=RC{cmap['PITCH']} "
          f"throttle=RC{cmap['THROTTLE']} yaw=RC{cmap['YAW']}")

    print("\n[i] Reading RCx_MIN/TRIM/MAX for those channels ...")
    for role, ch in cmap.items():
        mn = get_param(m, f"RC{ch}_MIN")
        tr = get_param(m, f"RC{ch}_TRIM")
        mx = get_param(m, f"RC{ch}_MAX")
        ok = "OK" if None not in (mn, tr, mx) else "MISSING VALUE(S)"
        print(f"    {role:9s} RC{ch}: min={mn} trim={tr} max={mx}   [{ok}]")

    missing = [n for n, v in values.items() if v is None]
    print()
    if missing:
        print(f"[!] {len(missing)} param(s) did not reply: {missing}")
        print("[!] ctbr_acro_rc.py would silently use its hardcoded fallback defaults for these.")
    else:
        print("[i] All checked parameters replied with real values from the vehicle.")
    print("[i] Done - no arm, no mode change, no RC override, no param writes were sent.")
    m.close()


if __name__ == "__main__":
    main()
