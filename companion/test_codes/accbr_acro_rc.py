#!/usr/bin/env python3
"""
ACCBR hover in ACRO via RC_CHANNELS_OVERRIDE - acceleration + body rates.

Same transport as ctbr_acro_rc.py (ArduCopter ACRO driven over
RC_CHANNELS_OVERRIDE, which can take off from the ground), but the outer loop
speaks a different language:

    ctbr_acro_rc.py   PID -> normalised thrust   -> PWM
    accbr_acro_rc.py  PID -> ACCELERATION [m/s2] -> normalised thrust -> PWM

The entire thrust mapping is one line, and it needs no vehicle mass:

    T_n = MOT_THST_HOVER * a_total / g

MOT_THST_HOVER is the value ArduPilot LEARNED in flight - by definition the
normalised thrust that produced exactly 1 g on this airframe. It therefore
already carries mass, thrust-to-weight, and the motor/prop/ESC curve at that
one point. Scaling by a_total/g is all that is left. Nothing here needs to know
what the aircraft weighs: change the battery or bolt on a payload, let the FC
relearn MOT_THST_HOVER, and this mapping follows it automatically.

Why the outer loop is in m/s2 and not in thrust units:
    ctbr_acro_rc.py's alt_pid gains are thrust-per-metre, so their physical
    meaning is divided by MOT_THST_HOVER - kp=0.35 is 1.00 g/m at hover 0.35
    but 0.84 g/m at hover 0.42. MOT_HOVER_LEARN defaults to LEARN_AND_SAVE, so
    that parameter changes on disarm and the altitude tuning silently moves
    with it. Here the gains are m/s2 per metre and mean the same thing forever.

Accuracy of the mapping, measured against ArduPilot's own thrust curve
(Thrust_Linearization::apply_thrust_curve_and_volt_scaling inverting
MOT_THST_EXPO), with MOT_THST_EXPO wrong by +/-0.15 - a large error:
    at hover      EXACT, by construction: H *is* the measured 1 g point
    +/-0.2 g      within 0.03 g
    +/-0.5 g      within 0.07 g
    +1.0 g        within 0.16 g
A smooth monotonic gain error, which the altitude integrator absorbs. It is
NOT accurate enough for open-loop feedforward with no correction.

BECAUSE H IS THE WHOLE SCALE FACTOR, this script refuses to fly when
MOT_THST_HOVER is still ArduPilot's 0.35 default - that means it was never
learned and the acceleration command would be wrong by however far the true
hover point is from 0.35. Note that ACRO can never learn it:
Copter::update_throttle_hover() returns early for any mode whose
has_manual_throttle() is true, and ModeAcro's returns true (so does Stabilize).
Learn it by hovering in ALT_HOLD / LOITER / GUIDED, level and not climbing, for
~30 s (the filter constant is AP_MOTORS_THST_HOVER_TC = 10 s), then disarm to
save. Flying THIS script will never improve it.

Body rates are mapped exactly as in ctbr_acro_rc.py, and the verified pieces
are imported from it rather than copied - euler->body kinematics, the
ArduPilot-matching circular stick limit, expo inversion and RC calibration.

Setup (defaults to the USB link, the one confirmed bidirectional on drone1):
    python3 accbr_acro_rc.py --alt 1
    python3 accbr_acro_rc.py --connect tcp:127.0.0.1:5762 --alt 1     # SITL

CAUTION: this script arms the vehicle and commands takeoff (see --force-arm).
Bench-test with propellers OFF before ever running it against real hardware.
"""

import argparse
import atexit
import math
import os
import sys
import time

from pymavlink import mavutil

# Verified primitives shared with the CTBR script - imported, not duplicated,
# so a fix to the RC/expo/kinematics maths cannot silently miss one of them.
from ctbr_acro_rc import (
    Chan, PID, RateMonitor, State,
    clamp, drain, euler_rates_to_body, expo_inverse, gcs_heartbeat, get_param,
    send_rc, set_mode_confirm, set_msg_interval, set_param, thrust_to_pwm,
    wrap_pi,
)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib"))
from flightlog import FlightLogger  # noqa: E402

G = 9.80665

# ArduPilot's own never-learned default (AP_MOTORS_THST_HOVER_DEFAULT).
HOVER_NEVER_LEARNED = 0.35


def specific_thrust_along_body_z(roll, pitch, a_fwd, a_rgt, a_up_plus_g):
    """Project the desired specific-thrust vector onto the ACTUAL body z axis.

    The propellers can only push along body -z, so the useful part of a desired
    thrust vector is its projection on that axis (the standard geometric /
    Mellinger projection). Using the MEASURED attitude rather than the commanded
    one means attitude lag is corrected for automatically, and it subsumes the
    1/cos(tilt) angle boost that ACRO does not apply for us
    (mode_acro.cpp: set_throttle_out(thr, false, ...)).

    Inputs are the desired accelerations already rotated into the yaw frame
    (forward / right) plus the vertical term (a_up + g). Returns m/s2 along the
    thrust axis, floored at 0 - if the aircraft is inverted the right answer is
    no thrust, not negative thrust.
    """
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    return max(sr * a_rgt - cr * sp * a_fwd + a_up_plus_g * cr * cp, 0.0)


def rate_to_norm(rate_rads, rate_max_degs, expo):
    """Desired body rate -> the normalised stick input ArduPilot needs for it."""
    n = clamp(math.degrees(rate_rads) / rate_max_degs, -1.0, 1.0)
    return expo_inverse(n, expo)


def circular_limit(n_roll, n_pitch):
    """ArduPilot limits the roll/pitch stick PAIR to the unit circle BEFORE expo
    (mode_acro.cpp get_pilot_desired_angle_rates). Letting that fire on the FC
    would rescale inputs we have already expo-inverted, corrupting both the
    magnitude and the roll:pitch ratio. Applying it here makes the FC's a no-op."""
    total = math.hypot(n_roll, n_pitch)
    if total > 1.0:
        return n_roll / total, n_pitch / total
    return n_roll, n_pitch


def main():
    ap = argparse.ArgumentParser(
        description="ACCBR hover in ACRO via RC override: PID outputs acceleration")
    ap.add_argument("--connect", default="/dev/ttyACM0",
                    help="MAVLink endpoint (default /dev/ttyACM0, the USB link; "
                         "/dev/ttyAMA0 for the GPIO UART, tcp:127.0.0.1:5762 for SITL)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud for serial targets: USB 115200, GPIO UART 57600")
    ap.add_argument("--alt", type=float, default=1.0, help="hover height [m]")
    ap.add_argument("--climb-rate", type=float, default=0.3,
                    help="altitude setpoint ramp rate [m/s] - see the note above RAMP_RATE "
                         "for why a slower ramp, not an output slew limit, is how takeoff "
                         "gentleness is tuned (ported from ctbr_acro_rc.py's fix)")
    ap.add_argument("--freq", type=float, default=100.0)
    ap.add_argument("--hold", type=float, default=20.0)
    ap.add_argument("--force-arm", action="store_true")
    ap.add_argument("--set-trainer", action="store_true",
                    help="temporarily write ACRO_TRAINER=0 and restore on exit - the "
                         "only parameter this script will ever write")
    ap.add_argument("--accept-default-hover", action="store_true",
                    help="fly even though MOT_THST_HOVER is still the 0.35 default. "
                         "That value was never learned, so every acceleration command "
                         "is scaled by a guess. Do not use this in the air.")
    ap.add_argument("--log-dir", default=None,
                    help="where to write flight logs (default ~/drone-ops/companion/logs)")
    args = ap.parse_args()

    log = FlightLogger(
        "accbr_acro_rc",
        ["armed", "mode", "alt", "alt_sp", "climb_rate", "a_up", "a_total", "a_cmd", "thrust",
         "x", "y", "dxy", "vx", "vy", "roll_deg", "pitch_deg", "yaw_deg",
         "roll_ref_deg", "pitch_ref_deg", "roll_rate", "pitch_rate", "yaw_rate",
         "pwm_roll", "pwm_pitch", "pwm_yaw", "pwm_thr",
         "dt_ms", "sched_slip_ms", "att_age_ms", "pos_age_ms"],
        log_dir=args.log_dir,
    )
    log.event(f"args: {vars(args)}")
    atexit.register(log.close)   # closes on every exit path: early return, Ctrl-C, or crash

    print(f"[i] Connecting to {args.connect} (baud={args.baud} if serial) ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    log.event(f"Heartbeat sys {m.target_system}")
    for _ in range(3):                      # identify as a GCS before param traffic
        gcs_heartbeat(m)
        time.sleep(0.1)

    missing = []

    def require(name):
        v = get_param(m, name)
        if v is None:
            missing.append(name)
            return 0.0
        return float(v)

    cmap = {k: int(require(f"RCMAP_{k}")) for k in ("ROLL", "PITCH", "THROTTLE", "YAW")}
    if missing:
        log.event(f"[!] Could not read the RC channel map from the FC: {missing}")
        log.event("[!] Refusing to run - the maths would silently use wrong channels.")
        return

    # ---- ACRO_TRAINER: the one thing that cannot be compensated in software ----
    if args.set_trainer:
        trainer_original = get_param(m, "ACRO_TRAINER")
        if trainer_original is None:
            log.event("[!] Could not read ACRO_TRAINER - refusing to overwrite it blindly.")
            return
        if abs(trainer_original) > 1e-6:
            def restore_trainer(value=trainer_original):
                for _ in range(5):
                    set_param(m, "ACRO_TRAINER", value)
                    back = get_param(m, "ACRO_TRAINER")
                    if back is not None and abs(back - value) < 1e-6:
                        log.event(f"ACRO_TRAINER restored to {value:g}.")
                        return
                log.event(f"[!] FAILED to restore ACRO_TRAINER. Set it back to {value:g} "
                          f"manually before flying with a transmitter.")
            atexit.register(restore_trainer)   # registered AFTER log.close so it runs first (LIFO)
            set_param(m, "ACRO_TRAINER", 0)
            confirmed = get_param(m, "ACRO_TRAINER")
            if confirmed is None or abs(confirmed) > 1e-6:
                log.event("[!] ACRO_TRAINER write was not confirmed by the FC - aborting.")
                return
            log.event(f"ACRO_TRAINER temporarily 0 (was {trainer_original:g}).")

    trainer = get_param(m, "ACRO_TRAINER")
    if trainer is None or abs(trainer) > 1e-6:
        log.event(f"[!] ACRO_TRAINER = {trainer}, must be 0. ArduPilot would add its own "
                  "earth-frame levelling rate that no RC value can cancel.")
        return

    # ---- RC calibration + ACRO rate model ----
    def chan(ch):
        return Chan(require(f"RC{ch}_MIN"), require(f"RC{ch}_MAX"), require(f"RC{ch}_TRIM"),
                    require(f"RC{ch}_DZ"), require(f"RC{ch}_REVERSED"))
    ch_roll, ch_pitch, ch_thr, ch_yaw = (chan(cmap["ROLL"]), chan(cmap["PITCH"]),
                                         chan(cmap["THROTTLE"]), chan(cmap["YAW"]))
    acro_rp_rate = require("ACRO_RP_RATE")
    acro_y_rate = require("ACRO_Y_RATE")
    acro_rp_expo = require("ACRO_RP_EXPO")
    acro_y_expo = require("ACRO_Y_EXPO")
    thr_hover = require("MOT_THST_HOVER")            # THE scale factor for everything
    mid_stick = float(get_param(m, "THR_MID") or 500.0)

    if missing:
        log.event(f"[!] {len(missing)} flight-critical parameter(s) unreadable after retries: {missing}")
        log.event("[!] Refusing to run - defaulting these means flying on a guess.")
        return
    if acro_rp_rate <= 0 or acro_y_rate <= 0:
        log.event(f"[!] Implausible rate limits (rp={acro_rp_rate}, y={acro_y_rate}) - aborting.")
        return

    # ---- read-only preflight: everything the thrust mapping depends on ----
    log.event(f"Thrust mapping inputs: MOT_THST_HOVER={thr_hover:.4f} (the entire accel scale)")
    for name, note in (("MOT_THST_EXPO", "motor curve ArduPilot inverts"),
                       ("MOT_HOVER_LEARN", "2 = learn and save on disarm"),
                       ("MOT_SPIN_ARM", "idle, below which thrust is not proportional"),
                       ("MOT_SPIN_MIN", ""),
                       ("MOT_BAT_VOLT_MAX", "0 = voltage compensation DISABLED"),
                       ("MOT_BAT_VOLT_MIN", ""),
                       ("ACRO_OPTIONS", "bit0 = pure rate loop; 0 = attitude loop on top"),
                       ("ACRO_RP_RATE_TC", "0 = no lag on the rate command"),
                       ("ACRO_Y_RATE_TC", "")):
        v = get_param(m, name)
        shown = "unreadable" if v is None else f"{v:g}"
        log.event(f"    {name:<17}= {shown:<10} {note}")

    if abs(thr_hover - HOVER_NEVER_LEARNED) < 1e-6:
        log.event(f"[!] MOT_THST_HOVER is exactly {HOVER_NEVER_LEARNED} - ArduPilot's "
                  "never-learned default. Every acceleration command would be scaled "
                  "by a guess. ACRO cannot learn it - hover ~30s in ALT_HOLD/LOITER/"
                  "GUIDED, level and not climbing, then disarm to save it.")
        if not args.accept_default_hover:
            log.event("[!] Refusing to run. Override with --accept-default-hover (props off).")
            return
        log.event("[!] --accept-default-hover given: continuing on an UNCALIBRATED scale.")

    log.event(f"ACRO_RP_RATE={acro_rp_rate:.0f} deg/s  ACRO_Y_RATE={acro_y_rate:.0f}  "
              f"expo rp={acro_rp_expo:.2f} y={acro_y_expo:.2f} (compensated in software)")
    log.event(f"RCMAP r/p/t/y={cmap['ROLL']}/{cmap['PITCH']}/{cmap['THROTTLE']}/{cmap['YAW']}"
              f"  thr min/dz/trim/max={ch_thr.min}/{ch_thr.dz}/{ch_thr.trim}/{ch_thr.max}")
    log.event(f"Thrust map: T_n = {thr_hover:.4f} * a_total / {G:.3f}   "
              f"(1 g -> T_n={thr_hover:.3f}, no vehicle mass involved)")

    def build_pwm(roll_rate, pitch_rate, yaw_rate, thrust):
        """Body rates [rad/s] + normalised thrust [0..1] -> the PWM that asks for them."""
        n_roll, n_pitch = circular_limit(
            rate_to_norm(roll_rate, acro_rp_rate, acro_rp_expo),
            rate_to_norm(pitch_rate, acro_rp_rate, acro_rp_expo))
        return {
            cmap["ROLL"]:     ch_roll.pwm_from_norm(n_roll),
            cmap["PITCH"]:    ch_pitch.pwm_from_norm(n_pitch),
            cmap["YAW"]:      ch_yaw.pwm_from_norm(
                                  rate_to_norm(yaw_rate, acro_y_rate, acro_y_expo)),
            cmap["THROTTLE"]: thrust_to_pwm(thrust, thr_hover, mid_stick, ch_thr),
        }

    # ---- streams ----
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, args.freq)
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, args.freq)

    st = State(); mon = RateMonitor()
    print("[i] Waiting for ATTITUDE + LOCAL_POSITION_NED ...")
    wait_t0 = time.time()
    while not (st.have_att and st.have_pos):
        drain(m, st, mon, log)
        if time.time() - wait_t0 > 15.0:
            log.event(f"[!] No state after 15s (have_att={st.have_att} "
                      f"have_pos={st.have_pos}). Likely no GPS fix - aborting "
                      "rather than hanging forever.")
            return
        time.sleep(0.005)
    yaw_sp = st.yaw
    x_sp, y_sp = st.x, st.y
    log.event(f"State ok. alt={st.altitude:+.2f} yaw={math.degrees(st.yaw):+.1f} deg "
              f"start_pos=({st.x:.2f},{st.y:.2f})")

    # ---- controllers: OUTPUTS ARE ACCELERATIONS [m/s2], not thrust ----
    # These gains are physical and stay meaningful whatever MOT_THST_HOVER
    # becomes. They are NOT a translation of ctbr_acro_rc.py's gains (whose
    # effective strength was ~1 g per metre) - re-tune on the bench.
    A_UP_MIN, A_UP_MAX = -4.0, 6.0                  # vertical accel authority [m/s2]
    alt_pid = PID(kp=4.0, ki=1.0, kd=3.0, i_limit=2.0, out_lo=A_UP_MIN, out_hi=A_UP_MAX)
    KP_POS, KD_POS, KI_POS = 1.2, 1.8, 0.2          # -> horizontal accel [m/s2]
    XY_I_LIM = 3.0
    TILT_MAX = math.radians(12.0)
    KP_ATT = 6.0; RATE_LIM = 3.0
    KP_YAW = 2.5; YAW_LIM = 1.5
    THRUST_MAX = 0.95
    xin = yin = 0.0

    # ---- ACRO + arm (throttle at min so the arming check passes) ----
    if not set_mode_confirm(m, st, mon, "ACRO", log=log):
        return
    idle = {cmap["ROLL"]: ch_roll.trim, cmap["PITCH"]: ch_pitch.trim,
            cmap["YAW"]: ch_yaw.trim, cmap["THROTTLE"]: ch_thr.min}
    for _ in range(30):
        send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, idle, mon)
        drain(m, st, mon, log); time.sleep(0.01)

    log.event(f"Arming (ACRO, throttle min, force={args.force_arm}) ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 21196 if args.force_arm else 0, 0, 0, 0, 0, 0)
    t_arm = time.time()
    while time.time() - t_arm < 8.0 and not st.armed:
        send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, idle, mon)
        drain(m, st, mon, log); time.sleep(0.01)
    if not st.armed:
        log.event("[!] NOT ARMED - see [AP] above. Try --force-arm."); return
    log.event("Armed. Taking off with ACCBR (throttle-up) ...")

    ground_alt = st.altitude
    T = 1.0 / args.freq
    t0 = time.perf_counter(); t_start = t0
    next_t = t0; last_report = t0; last_loop = t0; jit = 0.0; was_stale = False
    # Altitude setpoint ramps at --climb-rate (m/s), no branch, no snap to the
    # target - ported from ctbr_acro_rc.py's fix. The old form here (ramp for
    # a fixed 2s, then jump straight to --alt) was only invisible at --alt 1;
    # for anything larger it stepped the setpoint discontinuously while still
    # on the ground, driving near-max commanded thrust into a large
    # attitude/position excursion (see CLAUDE.md, 2026-08-21).
    RAMP_RATE = args.climb_rate

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_loop; last_loop = now
            if dt <= 0:
                dt = T
            drain(m, st, mon, log)

            climb_t = now - t_start
            alt_sp = ground_alt + min(args.alt, RAMP_RATE * climb_t)
            if climb_t > args.hold:
                break

            # ---------- outer loops produce ACCELERATION, in m/s2 ----------
            a_up = alt_pid.step(alt_sp - st.altitude, st.climb_rate, dt)

            xin = clamp(xin + (x_sp - st.x) * dt, -XY_I_LIM, XY_I_LIM)
            yin = clamp(yin + (y_sp - st.y) * dt, -XY_I_LIM, XY_I_LIM)
            a_n = KP_POS * (x_sp - st.x) - KD_POS * st.vx + KI_POS * xin
            a_e = KP_POS * (y_sp - st.y) - KD_POS * st.vy + KI_POS * yin

            # Tilt limit applied to the HORIZONTAL acceleration, not to the
            # angles: scaling a_n/a_e together preserves the commanded
            # direction, whereas clamping roll_ref and pitch_ref separately
            # would rotate it. Vertical accel is never sacrificed - altitude
            # keeps priority over position.
            a_h_max = (a_up + G) * math.tan(TILT_MAX)
            a_h = math.hypot(a_n, a_e)
            if a_h > a_h_max > 0.0:
                a_n *= a_h_max / a_h
                a_e *= a_h_max / a_h

            # rotate the horizontal command into the yaw frame
            cy, sy = math.cos(st.yaw), math.sin(st.yaw)
            a_fwd = a_n * cy + a_e * sy
            a_rgt = -a_n * sy + a_e * cy

            # total specific thrust the propellers must produce: THE acceleration
            # command, magnitude only, exactly what a CTBR/ACCBR interface takes
            a_total = math.sqrt(a_fwd * a_fwd + a_rgt * a_rgt + (a_up + G) ** 2)

            # ---------- the whole mapping. no mass, no thrust table ----------
            a_cmd = specific_thrust_along_body_z(st.roll, st.pitch, a_fwd, a_rgt, a_up + G)
            thrust = clamp(thr_hover * a_cmd / G, 0.0, THRUST_MAX)

            # attitude reference = direction of the desired thrust vector
            roll_ref = math.asin(clamp(a_rgt / a_total, -1.0, 1.0))
            pitch_ref = math.asin(clamp(-a_fwd / max(a_total * math.cos(roll_ref), 1e-6),
                                        -1.0, 1.0))

            # attitude P -> EULER rates, then convert: ACRO wants BODY rates
            roll_dot = clamp(KP_ATT * (roll_ref - st.roll), -RATE_LIM, RATE_LIM)
            pitch_dot = clamp(KP_ATT * (pitch_ref - st.pitch), -RATE_LIM, RATE_LIM)
            yaw_dot = clamp(KP_YAW * wrap_pi(yaw_sp - st.yaw), -YAW_LIM, YAW_LIM)
            roll_rate, pitch_rate, yaw_rate = euler_rates_to_body(
                st.roll, st.pitch, roll_dot, pitch_dot, yaw_dot)
            roll_rate = clamp(roll_rate, -RATE_LIM, RATE_LIM)
            pitch_rate = clamp(pitch_rate, -RATE_LIM, RATE_LIM)
            yaw_rate = clamp(yaw_rate, -YAW_LIM, YAW_LIM)

            pwm = build_pwm(roll_rate, pitch_rate, yaw_rate, thrust)
            send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, pwm, mon)

            dxy = math.hypot(x_sp - st.x, y_sp - st.y)
            # Latency diagnostics - same split as ctbr_acro_rc.py: sched_slip
            # shows whether THIS SCRIPT's own loop is keeping up with its
            # 100Hz schedule (Pi-side); att_age/pos_age show how stale the
            # state being acted on is, independent of loop timing - i.e.
            # whether the FC/link is delivering fresh state every cycle
            # (FC/link-side). Added per request, to answer the "is it the FC
            # executing slowly or the Pi processing slowly" question with
            # actual data on the next flight instead of a guess.
            sched_slip = now - (next_t - T)
            att_age = (now - st.att_time) if st.att_time is not None else float("nan")
            pos_age = (now - st.pos_time) if st.pos_time is not None else float("nan")

            log.row(st.armed, st.custom_mode, f"{st.altitude:.3f}", f"{alt_sp:.3f}",
                    f"{st.climb_rate:.3f}", f"{a_up:.3f}", f"{a_total:.3f}", f"{a_cmd:.3f}",
                    f"{thrust:.3f}", f"{st.x:.3f}", f"{st.y:.3f}", f"{dxy:.3f}",
                    f"{st.vx:.3f}", f"{st.vy:.3f}", f"{math.degrees(st.roll):.2f}",
                    f"{math.degrees(st.pitch):.2f}", f"{math.degrees(st.yaw):.2f}",
                    f"{math.degrees(roll_ref):.2f}", f"{math.degrees(pitch_ref):.2f}",
                    f"{roll_rate:.3f}", f"{pitch_rate:.3f}", f"{yaw_rate:.3f}",
                    pwm[cmap["ROLL"]], pwm[cmap["PITCH"]], pwm[cmap["YAW"]], pwm[cmap["THROTTLE"]],
                    f"{dt*1000:.2f}", f"{sched_slip*1000:.2f}",
                    f"{att_age*1000:.2f}", f"{pos_age*1000:.2f}")

            STALE_MS = 30.0
            is_stale = att_age * 1000 > STALE_MS or pos_age * 1000 > STALE_MS
            if is_stale and not was_stale:
                log.event(f"[!] state went stale: att_age={att_age*1000:.0f}ms "
                          f"pos_age={pos_age*1000:.0f}ms (>{STALE_MS:.0f}ms threshold) "
                          f"at alt={st.altitude:+.2f} dxy={dxy:.2f}")
            elif was_stale and not is_stale:
                log.event(f"state fresh again: att_age={att_age*1000:.0f}ms "
                          f"pos_age={pos_age*1000:.0f}ms")
            was_stale = is_stale

            jit = max(jit, abs(sched_slip))
            if now - last_report >= 1.0:
                print(f"alt={st.altitude:+.2f}(sp{alt_sp:+.2f}) dxy={dxy:4.2f} "
                      f"a_up={a_up:+5.2f} a_tot={a_total:5.2f} a_prj={a_cmd:5.2f} m/s2 "
                      f"T_n={thrust:.3f} pwm={pwm[cmap['THROTTLE']]} "
                      f"r={math.degrees(st.roll):+5.1f} p={math.degrees(st.pitch):+5.1f} "
                      f"| {mon.report()} | jit<={jit*1e3:4.1f}ms "
                      f"att_age={att_age*1e3:4.1f}ms pos_age={pos_age*1e3:4.1f}ms")
                last_report = now; jit = 0.0

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
        # Confirm LAND *before* releasing the override: releasing first hands
        # throttle back to the RC receiver while still in ACRO, and if the mode
        # change is then dropped that is zero throttle in mid-air.
        landed_mode = False
        for attempt in range(3):
            if set_mode_confirm(m, st, mon, "LAND", timeout=3.0, log=log):
                landed_mode = True
                break
            log.event(f"[!] LAND not confirmed (attempt {attempt + 1}/3), retrying ...")
        if landed_mode:
            m.mav.rc_channels_override_send(m.target_system, 1, *([0] * 8))
        else:
            log.event("[!] LAND NOT CONFIRMED - not releasing the override, so throttle "
                      "is not actively handed back mid-air. TAKE MANUAL CONTROL NOW.")
        t_l = time.time()
        while time.time() - t_l < 20.0 and st.armed:
            drain(m, st, mon, log); time.sleep(0.1)
        log.event("Done." if not st.armed else "Done (still armed).")


if __name__ == "__main__":
    main()
