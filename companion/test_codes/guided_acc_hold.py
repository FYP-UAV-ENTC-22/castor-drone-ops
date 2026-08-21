#!/usr/bin/env python3
"""
Position hold at altitude using ArduPilot's OWN GUIDED-mode acceleration
target - not the RC-override/ACRO trick ctbr_acro_rc.py and accbr_acro_rc.py
use. We stream a desired (north, east, up) acceleration over MAVLink;
ArduPilot's own AC_PosControl and attitude controller (AC_PID_2D, with its
own input filtering - see the comparison this design is based on) turn that
into lean angle, body rate, and thrust internally. We never touch RC
channels, never compute PWM, never fight ArduPilot's controller for
authority - we ask for an acceleration, ArduPilot delivers it.

This is deliberately NOT run at 100Hz. RC_CHANNELS_OVERRIDE fakes stick
input and needs updating fast enough to look continuous to ArduPilot's own
inner loops; a GUIDED acceleration target does not - it is a setpoint
ArduPilot tracks with its own control loops running onboard at their native
rate, same as a waypoint. The relevant number is ArduPilot's own timeout:

    GUID_TIMEOUT (default 3.0s) - if no new position/velocity/acceleration
    target arrives within this window, ArduPilot zeroes the commanded
    velocity AND acceleration and lets the position controller decelerate
    to a stop (ArduCopter/mode_guided.cpp: ModeGuided::velaccel_control_run,
    "set velocity to zero ... if no updates received"). This is a real,
    source-verified fail-safe distinct from RC_OVERRIDE_TIME - if this
    script dies mid-flight, the aircraft does not keep accelerating in a
    stale direction; it safely stops translating. Confirmed ModeGuided::
    set_accel() refreshes the exact same update_time_ms this timeout reads.

Default streaming rate here is 10Hz - roughly 30x margin below the 3s
timeout, smooth enough for a static hold, nowhere near 100Hz.

MAVLink mechanics (verified against ArduCopter/GCS_Mavlink.cpp's
handle_message_set_position_target_local_ned, Copter-4.6.2):
    - Requires GUIDED mode; silently ignored otherwise.
    - frame = MAV_FRAME_LOCAL_NED (matches LOCAL_POSITION_NED's own frame -
      no yaw rotation needed, unlike ctbr/accbr's RC-override math).
    - type_mask: position AND velocity bits set (ignored), acceleration
      bits clear (used) -> routes to ModeGuided::set_accel(), the
      acceleration-only path, not a mixed pos/vel/accel path.
    - afx/afy are north/east acceleration in m/s2, sent as-is.
    - afz is NED (DOWN-positive) by MAVLink convention - ArduPilot's own
      handler negates it back to up-positive internally
      (accel_vector.z = -packet.afz), so we send afz = -a_up ourselves.

Takeoff uses MAV_CMD_NAV_TAKEOFF - ArduPilot's own native climb, not a
ramp we compute. No altitude-setpoint-discontinuity class of bug is
possible here because we are not writing that logic at all.

The horizontal/vertical acceleration PID (KP_POS/KD_POS/KI_POS, alt_pid
gains, the position measurement low-pass filter) is the SAME architecture
verified for accbr_acro_rc.py - same gains, same --pos-filter-tc default.
What's different is everything downstream of "desired acceleration":
accbr computes attitude and PWM itself; this script hands the acceleration
to ArduPilot and stops there.

No parameters are ever written by this script - not even ACRO_TRAINER,
which does not apply here since we are not using ACRO mode.

CAUTION: this script arms the vehicle and commands takeoff.
Bench-test with propellers OFF before ever running it against real hardware.
"""
import argparse
import atexit
import math
import os
import sys
import time

from pymavlink import mavutil

from ctbr_acro_rc import (
    PID, RateMonitor, State,
    clamp, drain, gcs_heartbeat, set_mode_confirm, set_msg_interval,
)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib"))
from flightlog import FlightLogger  # noqa: E402

G = 9.80665

# type_mask bits, MAVLink SET_POSITION_TARGET_LOCAL_NED (common.xml)
POS_IGNORE = 0b0000_0000_0111       # bits 0-2: x,y,z position
VEL_IGNORE = 0b0000_0011_1000       # bits 3-5: vx,vy,vz
YAW_IGNORE = 0b0100_0000_0000       # bit 10
YAW_RATE_IGNORE = 0b1000_0000_0000  # bit 11
ACCEL_ONLY_MASK = POS_IGNORE | VEL_IGNORE | YAW_IGNORE | YAW_RATE_IGNORE


def send_accel_target(m, a_north, a_east, a_up):
    """One SET_POSITION_TARGET_LOCAL_NED, acceleration-only, LOCAL_NED frame."""
    m.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms, informational
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        ACCEL_ONLY_MASK,
        0, 0, 0,        # x, y, z (ignored)
        0, 0, 0,        # vx, vy, vz (ignored)
        a_north, a_east, -a_up,   # afx, afy, afz - NED down-positive, hence -a_up
        0, 0,            # yaw, yaw_rate (ignored)
    )


def wait_alt(m, st, mon, log, target_alt, tolerance=0.3, timeout=30.0):
    """Wait for altitude to approach the takeoff target. Returns True/False."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        drain(m, st, mon, log)
        if abs(st.altitude - target_alt) < tolerance:
            return True
        time.sleep(0.05)
    return False


def main():
    ap = argparse.ArgumentParser(
        description="Position hold via ArduPilot's own GUIDED-mode acceleration target")
    ap.add_argument("--connect", default="/dev/ttyAMA0",
                    help="MAVLink endpoint (default /dev/ttyAMA0, the GPIO UART)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--alt", type=float, default=3.0, help="hold altitude [m]")
    ap.add_argument("--hold", type=float, default=20.0, help="seconds to hold after reaching altitude")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="acceleration-target streaming rate [Hz] - see the module "
                         "docstring for why 10Hz is nowhere near the 3s GUID_TIMEOUT "
                         "this needs to stay under")
    ap.add_argument("--pos-filter-tc", type=float, default=0.3,
                    help="low-pass time constant [s] on x/y position measurement - "
                         "same verified default as accbr_acro_rc.py. 0 disables.")
    ap.add_argument("--force-arm", action="store_true")
    ap.add_argument("--log-dir", default=None)
    args = ap.parse_args()

    log = FlightLogger(
        "guided_acc_hold",
        ["armed", "mode", "alt", "alt_sp", "climb_rate", "a_up", "a_north", "a_east",
         "x", "y", "x_filt", "y_filt", "dxy", "vx", "vy", "yaw_deg",
         "dt_ms", "sched_slip_ms", "att_age_ms", "pos_age_ms"],
        log_dir=args.log_dir,
    )
    log.event(f"args: {vars(args)}")
    atexit.register(log.close)   # closes on every exit path: early return, Ctrl-C, or crash

    print(f"[i] Connecting to {args.connect} (baud={args.baud} if serial) ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    if not m.wait_heartbeat(timeout=15):
        log.event("[!] No heartbeat - aborting.")
        return
    log.event(f"Heartbeat sys {m.target_system}")
    for _ in range(3):
        gcs_heartbeat(m)
        time.sleep(0.1)

    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20.0)
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20.0)

    st = State(); mon = RateMonitor()
    print("[i] Waiting for ATTITUDE + LOCAL_POSITION_NED ...")
    wait_t0 = time.time()
    while not (st.have_att and st.have_pos):
        drain(m, st, mon, log)
        if time.time() - wait_t0 > 15.0:
            log.event(f"[!] No state after 15s (have_att={st.have_att} "
                      f"have_pos={st.have_pos}). Likely no GPS fix - aborting.")
            return
        time.sleep(0.005)
    x_sp, y_sp = st.x, st.y
    x_filt, y_filt = st.x, st.y
    log.event(f"State ok. alt={st.altitude:+.2f} yaw={math.degrees(st.yaw):+.1f} deg "
              f"start_pos=({st.x:.2f},{st.y:.2f})")

    if not set_mode_confirm(m, st, mon, "GUIDED", log=log):
        return

    log.event(f"Arming (GUIDED, force={args.force_arm}) ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 21196 if args.force_arm else 0, 0, 0, 0, 0, 0)
    t_arm = time.time()
    while time.time() - t_arm < 8.0 and not st.armed:
        drain(m, st, mon, log); time.sleep(0.05)
    if not st.armed:
        log.event("[!] NOT ARMED - see [AP] above. Try --force-arm."); return
    log.event("Armed.")

    log.event(f"Commanding MAV_CMD_NAV_TAKEOFF to {args.alt}m ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, args.alt)
    # No ground_alt offset needed: st.altitude (-LOCAL_POSITION_NED.z) and
    # MAV_CMD_NAV_TAKEOFF's target are both alt-above-home/EKF-origin already.

    if not wait_alt(m, st, mon, log, args.alt, tolerance=0.3, timeout=30.0):
        log.event(f"[!] Did not reach {args.alt}m within 30s (currently "
                  f"alt={st.altitude:.2f}) - landing instead of holding.")
        # fall through to the finally-equivalent landing below
        set_mode_confirm(m, st, mon, "LAND", timeout=3.0, log=log)
        t_l = time.time()
        while time.time() - t_l < 20.0 and st.armed:
            drain(m, st, mon, log); time.sleep(0.1)
        log.event("Done." if not st.armed else "Done (still armed).")
        return
    log.event(f"Reached target altitude: alt={st.altitude:.2f}")

    # ---- position PID: same architecture/gains as accbr_acro_rc.py ----
    A_UP_MIN, A_UP_MAX = -4.0, 6.0
    alt_pid = PID(kp=4.0, ki=1.0, kd=3.0, i_limit=2.0, out_lo=A_UP_MIN, out_hi=A_UP_MAX)
    KP_POS, KD_POS, KI_POS = 1.2, 1.8, 0.2
    XY_I_LIM = 3.0
    TILT_MAX = math.radians(12.0)   # sanity bound on OUR request; ArduPilot's own
                                     # ANGLE_MAX independently limits the real lean
                                     # angle regardless - this is defense in depth,
                                     # not the only thing standing between us and a
                                     # large lean angle.
    xin = yin = 0.0

    T = 1.0 / args.rate
    t0 = time.perf_counter(); t_start = t0
    next_t = t0; last_report = t0; last_loop = t0

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_loop; last_loop = now
            if dt <= 0:
                dt = T
            drain(m, st, mon, log)

            hold_t = now - t_start
            if hold_t > args.hold:
                break

            a_up = alt_pid.step(args.alt - st.altitude, st.climb_rate, dt)

            if args.pos_filter_tc > 0:
                x_filt += (st.x - x_filt) * (dt / args.pos_filter_tc)
                y_filt += (st.y - y_filt) * (dt / args.pos_filter_tc)
            else:
                x_filt, y_filt = st.x, st.y

            xin = clamp(xin + (x_sp - x_filt) * dt, -XY_I_LIM, XY_I_LIM)
            yin = clamp(yin + (y_sp - y_filt) * dt, -XY_I_LIM, XY_I_LIM)
            a_north = KP_POS * (x_sp - x_filt) - KD_POS * st.vx + KI_POS * xin
            a_east = KP_POS * (y_sp - y_filt) - KD_POS * st.vy + KI_POS * yin

            a_h_max = (a_up + G) * math.tan(TILT_MAX)
            a_h = math.hypot(a_north, a_east)
            if a_h > a_h_max > 0.0:
                a_north *= a_h_max / a_h
                a_east *= a_h_max / a_h

            send_accel_target(m, a_north, a_east, a_up)

            dxy = math.hypot(x_sp - st.x, y_sp - st.y)
            sched_slip = now - (next_t - T)
            att_age = (now - st.att_time) if st.att_time is not None else float("nan")
            pos_age = (now - st.pos_time) if st.pos_time is not None else float("nan")

            log.row(st.armed, st.custom_mode, f"{st.altitude:.3f}", f"{args.alt:.3f}",
                    f"{st.climb_rate:.3f}", f"{a_up:.3f}", f"{a_north:.3f}", f"{a_east:.3f}",
                    f"{st.x:.3f}", f"{st.y:.3f}", f"{x_filt:.3f}", f"{y_filt:.3f}", f"{dxy:.3f}",
                    f"{st.vx:.3f}", f"{st.vy:.3f}", f"{math.degrees(st.yaw):.2f}",
                    f"{dt*1000:.2f}", f"{sched_slip*1000:.2f}",
                    f"{att_age*1000:.2f}", f"{pos_age*1000:.2f}")

            if now - last_report >= 1.0:
                print(f"alt={st.altitude:+.2f}(sp{args.alt:+.2f}) dxy={dxy:4.2f} "
                      f"a_up={a_up:+5.2f} a_n={a_north:+5.2f} a_e={a_east:+5.2f} m/s2 "
                      f"att_age={att_age*1e3:4.1f}ms pos_age={pos_age*1e3:4.1f}ms")
                last_report = now

            next_t += T
            s = next_t - time.perf_counter()
            if s > 0:
                time.sleep(s)
            else:
                next_t = time.perf_counter()

    except KeyboardInterrupt:
        log.event("Interrupted (Ctrl-C).")
    except Exception as e:
        log.event(f"[!] Unhandled exception: {type(e).__name__}: {e}")
        raise
    finally:
        log.event("Landing (LAND mode) ...")
        # No RC override to release here (this script never sends one), so
        # there is no equivalent of ctbr/accbr's "confirm LAND before
        # releasing override" ordering concern - just switch mode.
        landed_mode = False
        for attempt in range(3):
            if set_mode_confirm(m, st, mon, "LAND", timeout=3.0, log=log):
                landed_mode = True
                break
            log.event(f"[!] LAND not confirmed (attempt {attempt + 1}/3), retrying ...")
        if not landed_mode:
            log.event("[!] LAND NOT CONFIRMED. TAKE MANUAL CONTROL NOW.")
        t_l = time.time()
        while time.time() - t_l < 20.0 and st.armed:
            drain(m, st, mon, log); time.sleep(0.1)
        log.event("Done." if not st.armed else "Done (still armed).")


if __name__ == "__main__":
    main()
