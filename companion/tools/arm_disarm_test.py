#!/usr/bin/env python3
"""
Arm -> hold -> disarm bench test with motor-output diagnostics.

Answers the question "it says armed, but nothing physically happened - why?"
by watching the FC's actual motor outputs (SERVO_OUTPUT_RAW) while armed,
and reporting the parameters that decide whether motors spin at all.

SAFETY: this WILL arm the vehicle and motors MAY spin. Only run with
propellers physically removed from every motor. Not enforced by this
script - the operator must verify.

Usage:
    python3 arm_disarm_test.py --connect /dev/ttyACM0 --baud 115200
    python3 arm_disarm_test.py --dwell 5          # stay armed longer
    python3 arm_disarm_test.py --normal-arm       # respect prearm checks
"""
import argparse
import time

from pymavlink import mavutil

FORCE_MAGIC = 21196          # bypasses prearm checks (bench use)
ARM_CMD = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t = time.time()
    while time.time() - t < timeout:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if p is not None and p.param_id.strip("\x00") == name:
            return p.param_value
    return None


def wait_ack(m, cmd, timeout=5.0):
    """Wait for a COMMAND_ACK matching `cmd`, ignoring acks for other commands."""
    t = time.time()
    while time.time() - t < timeout:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=max(0.1, timeout - (time.time() - t)))
        if ack is not None and ack.command == cmd:
            r = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result)
            return r.name if r else str(ack.result)
    return None


def wait_armed_state(m, want, timeout=6.0):
    """Poll HEARTBEATs until armed state == want, or timeout.

    The old version read a single heartbeat, which could be one buffered from
    BEFORE the arm command was processed (ArduPilot only heartbeats at ~1 Hz),
    so it reported stale state. Polling until the state matches fixes that.
    Returns (matched, last_observed_state).
    """
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


def read_outputs(m, timeout=2.0):
    """Latest SERVO_OUTPUT_RAW as a list of the first 4 channel PWMs."""
    msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]


def main():
    ap = argparse.ArgumentParser(description="Arm/hold/disarm bench test. PROPS MUST BE REMOVED.")
    ap.add_argument("--connect", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dwell", type=float, default=3.0,
                    help="seconds to stay armed so motor behaviour is observable (default 3)")
    ap.add_argument("--normal-arm", action="store_true",
                    help="arm without the force magic, i.e. respect prearm checks")
    args = ap.parse_args()

    print("[!] This WILL arm the vehicle and motors MAY spin.")
    print("[!] Propellers must already be removed from ALL motors.\n")

    print(f"[i] Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    print(f"[i] Heartbeat ok (sys {m.target_system}).")

    # ---- params that decide whether motors physically spin when armed ----
    print("\n[i] Reading motor/safety parameters ...")
    spin_arm = get_param(m, "MOT_SPIN_ARM")
    pwm_min = get_param(m, "MOT_PWM_MIN")
    pwm_max = get_param(m, "MOT_PWM_MAX")
    safety_dflt = get_param(m, "BRD_SAFETY_DEFLT")
    arming_check = get_param(m, "ARMING_CHECK")
    for n, v in (("MOT_SPIN_ARM", spin_arm), ("MOT_PWM_MIN", pwm_min),
                 ("MOT_PWM_MAX", pwm_max), ("BRD_SAFETY_DEFLT", safety_dflt),
                 ("ARMING_CHECK", arming_check)):
        print(f"    {n:18s} = {v}")

    expected_pwm = None
    if None not in (spin_arm, pwm_min, pwm_max):
        expected_pwm = pwm_min + spin_arm * (pwm_max - pwm_min)
        print(f"    -> expected idle PWM when armed ~= {expected_pwm:.0f}us")
        if spin_arm == 0:
            print("    -> MOT_SPIN_ARM is 0: motors are NOT meant to spin on arm. "
                  "Silent motors would be correct behaviour, not a fault.")

    # ---- stream motor outputs so we can see what the FC actually drives ----
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                            200000, 0, 0, 0, 0, 0)          # 5 Hz
    time.sleep(0.3)

    matched, state = wait_armed_state(m, False, timeout=3.0)
    print(f"\n[i] armed(before) = {state}")
    before = read_outputs(m)
    print(f"[i] motor outputs (before arm): {before}")

    # ---- arm ----
    p2 = 0 if args.normal_arm else FORCE_MAGIC
    print(f"\n[i] Sending ARM ({'normal, prearm checks apply' if args.normal_arm else 'force'}) ...")
    m.mav.command_long_send(m.target_system, m.target_component, ARM_CMD, 0, 1, p2, 0, 0, 0, 0, 0)
    print(f"[i] ARM ack: {wait_ack(m, ARM_CMD)}")

    matched, state = wait_armed_state(m, True, timeout=6.0)
    print(f"[i] armed(after arm) = {state}" + ("" if matched else "   [!] never reported armed"))

    # ---- hold armed, watching outputs, so behaviour is physically observable ----
    if matched:
        print(f"\n[i] Holding armed for {args.dwell:.1f}s - watch/listen to the motors now.")
        samples = []
        t_end = time.time() + args.dwell
        while time.time() < t_end:
            out = read_outputs(m, timeout=1.0)
            if out is not None:
                samples.append(out)
                print(f"    motor PWM: {out}")
        if samples:
            peak = max(max(s) for s in samples)
            print(f"[i] peak motor PWM while armed: {peak}us")
        else:
            peak = None
            print("[!] No SERVO_OUTPUT_RAW received - cannot tell what outputs did.")
    else:
        peak = None

    # ---- disarm ----
    print("\n[i] Sending DISARM ...")
    m.mav.command_long_send(m.target_system, m.target_component, ARM_CMD, 0, 0, FORCE_MAGIC, 0, 0, 0, 0, 0)
    print(f"[i] DISARM ack: {wait_ack(m, ARM_CMD)}")
    matched_d, state_d = wait_armed_state(m, False, timeout=6.0)
    print(f"[i] armed(final) = {state_d}" + ("" if matched_d else "   [!] STILL ARMED - disarm manually!"))

    # ---- interpretation ----
    print("\n=== Summary ===")
    if peak is not None and expected_pwm is not None:
        if peak <= (pwm_min or 0) + 5:
            print(f"Motor outputs stayed at ~{peak}us (idle floor) despite being armed.")
            print("That means outputs were NOT live. Most likely a hardware safety switch is")
            print("engaged (BRD_SAFETY_DEFLT=1 means outputs stay disabled until the safety")
            print("button is pressed), or MOT_SPIN_ARM is 0 so no idle spin is commanded.")
        else:
            print(f"Motor outputs rose to {peak}us while armed (expected ~{expected_pwm:.0f}us),")
            print("so the FC really was driving the motors - the arm was physically real.")
    print("Vehicle is disarmed." if state_d is False else "[!] CHECK VEHICLE STATE MANUALLY.")
    m.close()


if __name__ == "__main__":
    main()
