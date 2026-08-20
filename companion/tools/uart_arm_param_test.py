#!/usr/bin/env python3
"""
End-to-end UART check: read every parameter ctbr_acro_rc.py needs, arm, hold
5 s, disarm. Nothing is written to the FC and no RC override is ever sent -
this only proves the link can do what the flight script requires.

SAFETY: this DOES arm the vehicle and motors WILL spin up to MOT_SPIN_ARM
idle. Run only with propellers physically removed from every motor.

Usage:
    python3 uart_arm_param_test.py                       # /dev/ttyAMA0 @ 57600
    python3 uart_arm_param_test.py --connect /dev/ttyACM0 --baud 115200
    python3 uart_arm_param_test.py --dwell 10
    python3 uart_arm_param_test.py --no-arm               # params only, never arms
"""
import argparse
import time

from pymavlink import mavutil

ARM_CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
FORCE_MAGIC = 21196


def gcs_heartbeat(m):
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def get_param(m, name, timeout=1.0, tries=8):
    """Same retry+heartbeat strategy as ctbr_acro_rc.get_param()."""
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t = time.time()
        while time.time() - t < timeout:
            p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if p is not None and p.param_id.strip("\x00") == name:
                return p.param_value
        gcs_heartbeat(m)
    return None


def wait_ack(m, cmd, timeout=5.0):
    t = time.time()
    while time.time() - t < timeout:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True,
                           timeout=max(0.1, timeout - (time.time() - t)))
        if ack is not None and ack.command == cmd:
            r = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
            return r.name if r else str(ack.result)
    return None


def wait_armed_state(m, want, timeout=6.0):
    """Poll HEARTBEATs until armed state matches, avoiding stale single reads."""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb is None:
            continue
        last = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if last == want:
            return True, last
    return False, last


def main():
    ap = argparse.ArgumentParser(description="UART param + arm/disarm check. PROPS MUST BE OFF.")
    ap.add_argument("--connect", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--dwell", type=float, default=5.0, help="seconds to stay armed")
    ap.add_argument("--no-arm", action="store_true", help="read parameters only, never arm")
    args = ap.parse_args()

    if not args.no_arm:
        print("[!] This WILL arm the vehicle and motors WILL spin.")
        print("[!] Propellers must already be removed from ALL motors.\n")

    print(f"[i] Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    if not m.wait_heartbeat(timeout=15):
        print("[!] No heartbeat - link dead. Aborting.")
        return 1
    print(f"[i] Heartbeat ok (sys {m.target_system}).")
    for _ in range(3):
        gcs_heartbeat(m)
        time.sleep(0.1)

    # ---- every parameter the flight script depends on ----
    names = ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW",
             "ACRO_RP_RATE", "ACRO_Y_RATE", "ACRO_RP_EXPO", "ACRO_Y_EXPO",
             "ACRO_TRAINER", "MOT_THST_HOVER", "MOT_SPIN_ARM",
             "MOT_PWM_MIN", "MOT_PWM_MAX"]

    print(f"\n[i] Reading {len(names)} core parameters over the link ...")
    t0 = time.time()
    vals, failed = {}, []
    for n in names:
        v = get_param(m, n)
        vals[n] = v
        if v is None:
            failed.append(n)
        print(f"    {n:16s} = {'FAILED' if v is None else v}")

    # RC calibration for the four mapped channels
    chans = {}
    for role in ("ROLL", "PITCH", "THROTTLE", "YAW"):
        rc = vals.get(f"RCMAP_{role}")
        if rc is None:
            continue
        ch = int(rc)
        chans[role] = ch
        print(f"\n[i] RC{ch} ({role.lower()}) calibration:")
        for suffix in ("MIN", "MAX", "TRIM", "DZ", "REVERSED"):
            pn = f"RC{ch}_{suffix}"
            v = get_param(m, pn)
            vals[pn] = v
            if v is None:
                failed.append(pn)
            print(f"    {pn:16s} = {'FAILED' if v is None else v}")

    total = len(vals)
    elapsed = time.time() - t0
    print(f"\n[i] {total - len(failed)}/{total} parameters read in {elapsed:.1f}s")
    if failed:
        print(f"[!] FAILED: {failed}")
        print("[!] ctbr_acro_rc.py would refuse to run with these missing.")
    else:
        print("[i] All parameters read successfully - the flight script would proceed.")

    if args.no_arm:
        print("\n[i] --no-arm given; not arming. Done.")
        return 0 if not failed else 1

    if vals.get("ACRO_TRAINER") not in (None, 0.0):
        print(f"\n[i] Note: ACRO_TRAINER = {vals['ACRO_TRAINER']:g}. The flight script needs 0 "
              "(or --set-trainer). Not relevant to this arm test.")

    # ---- arm / dwell / disarm ----
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                            200000, 0, 0, 0, 0, 0)
    time.sleep(0.3)

    _, state = wait_armed_state(m, False, timeout=3.0)
    print(f"\n[i] armed(before) = {state}")

    print("[i] Sending ARM (force) ...")
    m.mav.command_long_send(m.target_system, m.target_component, ARM_CMD, 0,
                            1, FORCE_MAGIC, 0, 0, 0, 0, 0)
    print(f"[i] ARM ack: {wait_ack(m, ARM_CMD)}")
    armed_ok, state = wait_armed_state(m, True, timeout=6.0)
    print(f"[i] armed(after) = {state}" + ("" if armed_ok else "   [!] never reported armed"))

    peak = None
    if armed_ok:
        print(f"\n[i] Holding armed for {args.dwell:.1f}s ...")
        t_end = time.time() + args.dwell
        while time.time() < t_end:
            out = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1.0)
            if out is not None:
                pwms = [out.servo1_raw, out.servo2_raw, out.servo3_raw, out.servo4_raw]
                peak = max(peak or 0, max(pwms))
                print(f"    motor PWM: {pwms}")
            gcs_heartbeat(m)          # keep the link identified while armed
        print(f"[i] peak motor PWM while armed: {peak}")

    print("\n[i] Sending DISARM ...")
    m.mav.command_long_send(m.target_system, m.target_component, ARM_CMD, 0,
                            0, FORCE_MAGIC, 0, 0, 0, 0, 0)
    print(f"[i] DISARM ack: {wait_ack(m, ARM_CMD)}")
    disarmed_ok, final = wait_armed_state(m, False, timeout=8.0)
    print(f"[i] armed(final) = {final}" +
          ("" if disarmed_ok else "   [!] STILL ARMED - DISARM MANUALLY NOW"))

    print("\n=== Summary ===")
    print(f"link          : {args.connect} @ {args.baud}")
    print(f"parameters    : {total - len(failed)}/{total} read in {elapsed:.1f}s"
          + (f"  FAILED={failed}" if failed else ""))
    print(f"arm           : {'OK' if armed_ok else 'FAILED'}")
    if peak is not None:
        print(f"motor output  : peaked at {peak}us while armed")
    print(f"disarm        : {'OK - vehicle is disarmed' if disarmed_ok else 'FAILED - CHECK VEHICLE'}")
    return 0 if (not failed and armed_ok and disarmed_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
