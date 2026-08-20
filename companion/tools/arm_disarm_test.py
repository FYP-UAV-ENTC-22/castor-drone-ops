#!/usr/bin/env python3
"""
Minimal arm -> disarm bench test. Sends MAV_CMD_COMPONENT_ARM_DISARM
(force-arm, magic 21196, to bypass prearm checks since this is a deliberate
bench test) then immediately disarms, checking status in between. Also
retries one param read afterward to see if the TX-not-reaching-FC issue
affects commands too.

SAFETY: this WILL arm the vehicle. Only run with propellers physically
removed from every motor - not enforced by this script, must be verified
by the operator before running.

Usage:
    python3 arm_disarm_test.py --connect /dev/ttyAMA0 --baud 57600
"""
import argparse
import time

from pymavlink import mavutil


def wait_ack(m, cmd, timeout=5.0):
    t = time.time()
    while time.time() - t < timeout:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
        if ack is not None and ack.command == cmd:
            r = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
            return r.name if r else str(ack.result)
    return None


def is_armed(m, timeout=3.0):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def main():
    ap = argparse.ArgumentParser(description="Minimal arm->disarm bench test. PROPS MUST BE REMOVED.")
    ap.add_argument("--connect", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    args = ap.parse_args()

    print("[!] This WILL arm the vehicle. Propellers must already be removed from ALL motors.")
    print(f"[i] Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    print(f"[i] Heartbeat ok. armed(before)={is_armed(m)}")

    print("[i] Sending ARM (force) ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                             1, 21196, 0, 0, 0, 0, 0)
    result = wait_ack(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print(f"[i] ARM ack: {result}")
    print(f"[i] armed(after arm cmd)={is_armed(m)}")

    print("[i] Sending DISARM ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                             0, 21196, 0, 0, 0, 0, 0)
    result = wait_ack(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print(f"[i] DISARM ack: {result}")
    print(f"[i] armed(final)={is_armed(m)}")

    print("\n[i] Re-testing a param read now that we've sent arm/disarm commands ...")
    m.mav.param_request_read_send(m.target_system, m.target_component, b"ACRO_RP_RATE", -1)
    p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3.0)
    print(f"    ACRO_RP_RATE = {p.param_value if p else 'NO REPLY'}")

    m.close()


if __name__ == "__main__":
    main()
