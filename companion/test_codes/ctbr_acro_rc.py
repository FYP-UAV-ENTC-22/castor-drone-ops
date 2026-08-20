#!/usr/bin/env python3
"""
CTBR hover in ACRO via RC_CHANNELS_OVERRIDE — full control INCLUDING takeoff.

ArduCopter's ACRO mode is a pure body-rate + manual-collective-throttle mode, and it
takes off from the ground on throttle-up (no GUIDED land/spool lock). Driving its RC
channels over MAVLink therefore gives a genuine CTBR interface that can lift off:

    ch(roll)  -> roll  body rate       ch(throttle) -> collective thrust
    ch(pitch) -> pitch body rate        ch(yaw)      -> yaw   body rate

RC override sends PWM (1000..2000), so we invert ArduCopter's own stick->command maths
using parameters READ FROM THE VEHICLE. Nothing is written to the FC - whatever expo
and deadzone are configured is compensated for in software instead:
    rate:      rate_degs = ACRO_*_RATE * input_expo(norm_input, ACRO_*_EXPO)
               inverted via expo_inverse(), then norm_input -> PWM through
               RC_Channel::norm_input_dz() inverted with RCx_MIN/MAX/TRIM/DZ/REVERSED
    throttle:  cubic expo around MOT_THST_HOVER (inverted numerically), then
               0..1000 -> PWM through RC_Channel::pwm_to_range_dz() inverted,
               which measures from (RC3_MIN + RC3_DZ)

The one thing that CANNOT be handled in software is ACRO_TRAINER: when enabled,
ArduPilot adds its own earth-frame levelling rate computed from the attitude
target, not from stick input, so no RC value can cancel it. By default the
script only verifies ACRO_TRAINER == 0 and refuses to run otherwise. Pass
--set-trainer to have it temporarily write 0 and restore your original value on
exit; that is the only parameter write in this script.

Control (all done here; ArduPilot only runs the inner rate loop):
    altitude PID          -> collective thrust
    x/y position PID -> tilt ref -> attitude P -> roll/pitch body rate
    yaw hold P            -> yaw body rate

Setup:
    Real hardware (drone1) over the USB link - the one confirmed bidirectional:
        python3 ctbr_acro_rc.py --alt 1
        (defaults to --connect /dev/ttyACM0 --baud 115200)

    Over the GPIO UART instead (NOTE: on drone1 this link is currently
    receive-only - the FC's heartbeats arrive but nothing we send reaches it,
    so this will hang at wait_heartbeat's first param write):
        python3 ctbr_acro_rc.py --connect /dev/ttyAMA0 --baud 57600 --alt 1

    SITL - connect DIRECTLY, not through MAVProxy, so streams hit 100 Hz:
        python3 ctbr_acro_rc.py --connect tcp:127.0.0.1:5762 --alt 1

CAUTION: this script arms the vehicle and commands takeoff (see --force-arm).
Bench-test with propellers OFF before ever running this against real hardware.
"""

import argparse
import atexit
import math
import time
from collections import defaultdict

from pymavlink import mavutil


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PID:
    def __init__(self, kp, ki, kd, i_limit, out_lo, out_hi):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_lo, self.out_hi = i_limit, out_lo, out_hi
        self.integ = 0.0

    def step(self, error, meas_rate, dt, ff=0.0):
        self.integ = clamp(self.integ + error * dt, -self.i_limit, self.i_limit)
        out = ff + self.kp * error + self.ki * self.integ - self.kd * meas_rate
        return clamp(out, self.out_lo, self.out_hi)


class RateMonitor:
    def __init__(self):
        self.count = defaultdict(int)
        self.fresh = defaultdict(int)
        self.last_boot = {}
        self.t0 = time.perf_counter()

    def rx(self, name, boot=None):
        self.count[name] += 1
        if boot is not None:
            prev = self.last_boot.get(name)
            if prev is None or boot > prev:
                self.fresh[name] += 1
            self.last_boot[name] = boot

    def tx(self):
        self.count["TX"] += 1

    def report(self):
        dt = time.perf_counter() - self.t0
        parts = []
        for n in sorted(self.count):
            hz = self.count[n] / dt
            parts.append(f"{n}={hz:5.1f}" + (f"(f{self.fresh[n]/dt:4.1f})" if n in self.fresh else ""))
        self.count.clear(); self.fresh.clear(); self.t0 = time.perf_counter()
        return " ".join(parts)


class State:
    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.armed = False
        self.custom_mode = None
        self.have_att = self.have_pos = False

    @property
    def altitude(self):
        return -self.z

    @property
    def climb_rate(self):
        return -self.vz


def expo_inverse(n_out, expo):
    """Inverse of ArduPilot's input_expo() (AP_Math/control.cpp, Copter-4.6):

        out = (1 - expo) * in / (1 - expo * |in|)      when expo < 0.95
        out = in                                       otherwise

    Solving the first case for `in` gives:

        in = out / (1 - expo + expo * |out|)

    so we can ask for a rate and get the stick position that produces it,
    whatever ACRO_*_EXPO happens to be set to on the vehicle.
    """
    n_out = clamp(n_out, -1.0, 1.0)
    if expo >= 0.95:                      # matches AP's own branch
        return n_out
    den = 1.0 - expo + expo * abs(n_out)
    if den <= 1e-6:                       # unreachable for expo < 0.95, guard anyway
        return n_out
    return clamp(n_out / den, -1.0, 1.0)


class Chan:
    """One RC channel's calibration, used to invert ArduPilot's PWM -> input maths.

    Mirrors RC_Channel::norm_input_dz() (angle channels: roll/pitch/yaw) and
    RC_Channel::pwm_to_range_dz() (range channels: throttle), including the
    channel's deadzone and reverse flag, so no RCx_DZ has to be zeroed on the FC.
    """
    def __init__(self, mn, mx, tr, dz=0, rev=0):
        self.min, self.max, self.trim = int(mn), int(mx), int(tr)
        self.dz = int(dz)
        self.rev = bool(rev)

    def pwm_from_norm(self, n):
        """Inverse of norm_input_dz(): norm_input in [-1,1] -> PWM."""
        n = clamp(n, -1.0, 1.0)
        if self.rev:
            n = -n                        # undo reverse_mul
        dz_min, dz_max = self.trim - self.dz, self.trim + self.dz
        if n > 0:
            pwm = dz_max + n * (self.max - dz_max)
        elif n < 0:
            pwm = dz_min + n * (dz_min - self.min)
        else:
            pwm = self.trim               # anywhere in the deadzone reads as 0
        return int(clamp(pwm, self.min, self.max))

    def pwm_from_range(self, control_in, high_in=1000.0):
        """Inverse of pwm_to_range_dz(): control_in in 0..high_in -> PWM.

        Forward is  control_in = high_in * (r - (min + dz)) / (max - (min + dz)),
        with r mirrored about the range first when the channel is reversed.
        """
        c = clamp(control_in, 0.0, high_in)
        low = self.min + self.dz
        r = low + c * (self.max - low) / high_in
        if self.rev:
            r = self.max + self.min - r   # undo the mirroring AP applies first
        return int(clamp(r, self.min, self.max))


# --------------------------------------------------------------- param helpers
def gcs_heartbeat(m):
    """Announce ourselves as a GCS. ArduPilot answers a link it has seen a
    heartbeat on far more reliably; without this, param reads over the 57600
    UART are dropped often enough to matter."""
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def get_param(m, name, timeout=1.0, tries=8):
    """Read one parameter, retrying, with a heartbeat between attempts.

    A single-shot request is NOT reliable on the 57600 UART while telemetry is
    streaming - measured 5 of 16 reads failing. Silently accepting that and
    falling back to a default is how you fly with the wrong RC calibration, so
    callers must treat None as fatal (see require() in main).
    """
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t = time.time()
        while time.time() - t < timeout:
            p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if p is not None and p.param_id.strip("\x00") == name:
                return p.param_value
        gcs_heartbeat(m)
    return None


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """Write one parameter.

    Deliberately used for ACRO_TRAINER and nothing else, only under
    --set-trainer, and always paired with an atexit restore. Everything else
    this script needs is compensated for in software instead - do not add
    further callers.
    """
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value), ptype)
    time.sleep(0.05)


def set_msg_interval(m, msg_id, hz):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, int(1e6 / hz) if hz > 0 else 0, 0, 0, 0, 0, 0)


def set_mode_confirm(m, st, mon, name, timeout=5.0):
    if name not in m.mode_mapping():
        print(f"[!] mode {name} unavailable"); return False
    target = m.mode_mapping()[name]
    m.set_mode(target)
    t = time.time()
    while time.time() - t < timeout:
        drain(m, st, mon)
        if st.custom_mode == target:
            print(f"[i] Mode confirmed: {name}"); return True
        time.sleep(0.02)
    print(f"[!] mode {name} not confirmed (is {st.custom_mode})"); return False


# ------------------------------------------------------------- throttle mapping
def throttle_forward(throttle_control, thr_mid, mid_stick):
    """ArduCopter get_pilot_desired_throttle: 0..1000 stick -> 0..1 throttle."""
    tc = clamp(throttle_control, 0, 1000)
    if tc < mid_stick:
        thr_in = tc * 0.5 / mid_stick
    else:
        thr_in = 0.5 + (tc - mid_stick) * 0.5 / (1000 - mid_stick)
    expo = clamp(-(thr_mid - 0.5) / 0.375, -0.5, 1.0)
    return thr_in * (1.0 - expo) + expo * thr_in ** 3


def thrust_to_pwm(thrust, thr_mid, mid_stick, ch3):
    """Invert throttle_forward numerically, then map 0..1000 -> PWM on ch3."""
    thrust = clamp(thrust, 0.0, 1.0)
    lo, hi = 0.0, 1000.0
    for _ in range(30):                      # binary search (monotonic)
        mid = 0.5 * (lo + hi)
        if throttle_forward(mid, thr_mid, mid_stick) < thrust:
            lo = mid
        else:
            hi = mid
    tc = 0.5 * (lo + hi)
    # throttle is a RANGE channel: control_in comes from pwm_to_range_dz(), which
    # measures from (min + RC3_DZ), not from min. Ignoring that biases the whole
    # thrust feedforward low.
    return ch3.pwm_from_range(tc)


# --------------------------------------------------------------------- drain
def drain(m, st, mon):
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            return
        t = msg.get_type()
        if t == "HEARTBEAT":
            st.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            st.custom_mode = msg.custom_mode
        elif t == "ATTITUDE":
            st.roll, st.pitch, st.yaw = msg.roll, msg.pitch, msg.yaw
            st.have_att = True
            mon.rx("ATT", msg.time_boot_ms)
        elif t == "LOCAL_POSITION_NED":
            st.x, st.y, st.z = msg.x, msg.y, msg.z
            st.vx, st.vy, st.vz = msg.vx, msg.vy, msg.vz
            st.have_pos = True
            mon.rx("POS", msg.time_boot_ms)
        elif t == "STATUSTEXT":
            print(f"    [AP] {msg.text}")
        elif t == "COMMAND_ACK":
            r = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result)
            print(f"    [ACK] cmd={msg.command} {r.name if r else msg.result}")


def send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, pwm_map, mon):
    """Send RC_CHANNELS_OVERRIDE. pwm_map: dict channel_number(1..8) -> pwm.
    Channels not in pwm_map are released (0)."""
    vals = [pwm_map.get(i, 0) for i in range(1, 9)]     # 8 base channels
    m.mav.rc_channels_override_send(m.target_system, 1, *vals)
    mon.tx()


def main():
    ap = argparse.ArgumentParser(description="CTBR hover in ACRO via RC override (takeoff included)")
    ap.add_argument("--connect", default="/dev/ttyAMA0",
                     help="MAVLink endpoint - serial device path (default /dev/ttyAMA0, the "
                          "GPIO UART; /dev/ttyACM0 for the USB link) or a URL like "
                          "tcp:127.0.0.1:5762 for SITL")
    ap.add_argument("--baud", type=int, default=57600,
                     help="baud rate for serial --connect targets (ignored for tcp:/udp: URLs); "
                          "the GPIO UART /dev/ttyAMA0 uses 57600, USB /dev/ttyACM0 uses 115200")
    ap.add_argument("--alt", type=float, default=1.0, help="hover height [m]")
    ap.add_argument("--freq", type=float, default=100.0)
    ap.add_argument("--hold", type=float, default=20.0)
    ap.add_argument("--force-arm", action="store_true")
    ap.add_argument("--set-trainer", action="store_true",
                    help="temporarily write ACRO_TRAINER=0 and restore the original value "
                         "on exit. The ONLY parameter this script will ever write, and it "
                         "cannot survive a power cut or SIGKILL - if the restore is missed, "
                         "the script says so and you must set it back manually.")
    args = ap.parse_args()

    print(f"[i] Connecting to {args.connect} (baud={args.baud} if serial) ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    m.wait_heartbeat()
    print(f"[i] Heartbeat sys {m.target_system}")
    for _ in range(3):            # identify as a GCS before any param traffic
        gcs_heartbeat(m)
        time.sleep(0.1)

    # ---- VERIFY (never modify) the vehicle config this script's maths assumes ----
    # This script deliberately writes no parameters. The stick->command inversion
    # below is only valid when expo and deadzone are zero and ACRO is pure rate,
    # so those are checked and the run is refused if they don't match - rather
    # than silently reconfiguring the aircraft and leaving it that way.
    # Every value below is flight-critical: a wrong RC calibration or channel
    # map silently produces wrong PWM. Missing reads are collected and treated
    # as fatal rather than defaulted.
    missing = []

    def require(name):
        v = get_param(m, name)
        if v is None:
            missing.append(name)
            return 0.0
        return float(v)

    # channel map (which RC channel is roll/pitch/thr/yaw)
    cmap = {k: int(require(f"RCMAP_{k}"))
            for k in ("ROLL", "PITCH", "THROTTLE", "YAW")}
    if missing:
        # bail now: without the channel map every later read targets RC0_*
        print(f"[!] Could not read the RC channel map from the FC: {missing}")
        print("[!] Refusing to run - the maths would silently use wrong channels.")
        return

    # Expo and deadzone are compensated for in software (see expo_inverse and
    # Chan), so they may be set to anything. ACRO_TRAINER is different: with it
    # enabled ArduPilot ADDS its own earth-frame levelling rate, derived from the
    # attitude target rather than from stick input (ArduCopter/mode_acro.cpp),
    # so no choice of RC input can cancel it. It has to be off on the vehicle.
    # --set-trainer: the single permitted parameter write. Registered with atexit
    # BEFORE the write, so an early return, an exception or Ctrl-C all still
    # restore it. A power cut or SIGKILL cannot be covered - hence the warning.
    if args.set_trainer:
        trainer_original = get_param(m, "ACRO_TRAINER")
        if trainer_original is None:
            print("[!] Could not read ACRO_TRAINER - refusing to overwrite it blindly.")
            return
        if abs(trainer_original) > 1e-6:
            def restore_trainer(value=trainer_original):
                for _ in range(5):
                    set_param(m, "ACRO_TRAINER", value)
                    back = get_param(m, "ACRO_TRAINER")
                    if back is not None and abs(back - value) < 1e-6:
                        print(f"[i] ACRO_TRAINER restored to {value:g}.")
                        return
                print(f"[!] FAILED to restore ACRO_TRAINER. Set it back to {value:g} "
                      f"manually in Mission Planner before flying with a transmitter.")
            atexit.register(restore_trainer)

            set_param(m, "ACRO_TRAINER", 0)
            confirmed = get_param(m, "ACRO_TRAINER")
            if confirmed is None or abs(confirmed) > 1e-6:
                print("[!] ACRO_TRAINER write was not confirmed by the FC - aborting.")
                return
            print(f"[i] ACRO_TRAINER temporarily set to 0 (was {trainer_original:g}); "
                  "it will be restored on exit.")

    required = [("ACRO_TRAINER", 0.0)]

    problems = []
    for pname, want in required:
        got = get_param(m, pname)
        if got is None:
            problems.append(f"      {pname}: no reply from FC (could not verify)")
        elif abs(got - want) > 1e-6:
            problems.append(f"      {pname} = {got:g}, must be {want:g}")
    if problems:
        print("[!] Vehicle config does not match what this script's maths assumes:")
        for p in problems:
            print(p)
        print("[!] This script does not write parameters. Set these on the FC "
              "yourself (Mission Planner / QGroundControl), then re-run.")
        return
    print("[i] Config verified: ACRO trainer/expo and rate-channel deadzones are zero.")

    def chan(ch):
        return Chan(require(f"RC{ch}_MIN"), require(f"RC{ch}_MAX"),
                    require(f"RC{ch}_TRIM"), require(f"RC{ch}_DZ"),
                    require(f"RC{ch}_REVERSED"))
    ch_roll, ch_pitch, ch_thr, ch_yaw = (chan(cmap["ROLL"]), chan(cmap["PITCH"]),
                                         chan(cmap["THROTTLE"]), chan(cmap["YAW"]))
    acro_rp_rate = require("ACRO_RP_RATE")                        # deg/s at full stick
    acro_y_rate = require("ACRO_Y_RATE")
    acro_rp_expo = require("ACRO_RP_EXPO")                        # compensated, not forced
    acro_y_expo = require("ACRO_Y_EXPO")
    thr_mid = require("MOT_THST_HOVER")                           # hover throttle 0..1
    # THR_MID no longer exists on Copter 4.x; ArduPilot itself falls back to 500
    # (Mode::get_pilot_desired_throttle), so this fallback matches the firmware.
    mid_stick = float(get_param(m, "THR_MID") or 500.0)           # mid-stick 0..1000
    print(f"[i] ACRO_RP_RATE={acro_rp_rate:.0f} deg/s  ACRO_Y_RATE={acro_y_rate:.0f}  "
          f"MOT_THST_HOVER={thr_mid:.2f}  THR_MID={mid_stick:.0f}")
    print(f"[i] expo compensated in software: rp={acro_rp_expo:.2f} y={acro_y_expo:.2f}")
    print(f"[i] RCMAP r/p/t/y={cmap['ROLL']}/{cmap['PITCH']}/{cmap['THROTTLE']}/{cmap['YAW']}  "
          f"thr ch min/dz/trim/max={ch_thr.min}/{ch_thr.dz}/{ch_thr.trim}/{ch_thr.max}")
    print(f"[i] deadzones compensated in software: roll={ch_roll.dz} pitch={ch_pitch.dz} "
          f"yaw={ch_yaw.dz} thr={ch_thr.dz}")

    if missing:
        print(f"[!] {len(missing)} flight-critical parameter(s) could not be read "
              f"from the FC after retries:")
        for name in missing:
            print(f"      {name}")
        print("[!] Refusing to run. Falling back to defaults here would mean flying")
        print("[!] with the wrong RC calibration - e.g. a zero deadzone default")
        print("[!] makes every small attitude correction map inside the real")
        print("[!] deadzone and do nothing at all. Check the link and re-run.")
        return
    if acro_rp_rate <= 0 or acro_y_rate <= 0:
        print(f"[!] Implausible rate limits from FC (rp={acro_rp_rate}, y={acro_y_rate}) - aborting.")
        return

    def rate_to_pwm(rate_rads, rate_max_degs, ch, expo):
        # AP computes rate = ACRO_*_RATE * input_expo(norm_in, expo), so undo the
        # expo to find the norm_in that yields the rate we actually want.
        n = clamp(math.degrees(rate_rads) / rate_max_degs, -1.0, 1.0)
        return ch.pwm_from_norm(expo_inverse(n, expo))

    def build_pwm(roll_rate, pitch_rate, yaw_rate, thrust):
        return {
            cmap["ROLL"]:     rate_to_pwm(roll_rate, acro_rp_rate, ch_roll, acro_rp_expo),
            cmap["PITCH"]:    rate_to_pwm(pitch_rate, acro_rp_rate, ch_pitch, acro_rp_expo),
            cmap["YAW"]:      rate_to_pwm(yaw_rate, acro_y_rate, ch_yaw, acro_y_expo),
            cmap["THROTTLE"]: thrust_to_pwm(thrust, thr_mid, mid_stick, ch_thr),
        }

    # ---- streams ----
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, args.freq)
    set_msg_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, args.freq)

    st = State(); mon = RateMonitor()
    print("[i] Waiting for ATTITUDE + LOCAL_POSITION_NED ...")
    while not (st.have_att and st.have_pos):
        drain(m, st, mon); time.sleep(0.005)
    yaw_sp = st.yaw
    x_sp, y_sp = st.x, st.y
    print(f"[i] State ok. alt={st.altitude:+.2f} yaw={math.degrees(st.yaw):+.1f} deg")

    # ---- controllers ----
    alt_pid = PID(kp=0.35, ki=0.15, kd=0.30, i_limit=0.4, out_lo=0.0, out_hi=0.95)
    KP_ATT = 6.0; RATE_LIM = 3.0
    KP_YAW = 2.5; YAW_LIM = 1.5
    KP_POS = 0.5; KD_POS = 1.0; KI_POS = 0.15; TILT_MAX = math.radians(12.0); G = 9.81
    xin = xie = 0.0
    XY_I = 1.5

    # ---- ACRO + arm (throttle at min so the arming check passes) ----
    if not set_mode_confirm(m, st, mon, "ACRO"):
        return
    thr_min_pwm = ch_thr.min
    idle = {cmap["ROLL"]: ch_roll.trim, cmap["PITCH"]: ch_pitch.trim,
            cmap["YAW"]: ch_yaw.trim, cmap["THROTTLE"]: thr_min_pwm}
    for _ in range(30):
        send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, idle, mon)
        drain(m, st, mon); time.sleep(0.01)

    print("[i] Arming (ACRO, throttle min) ...")
    arm_p2 = 21196 if args.force_arm else 0
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, arm_p2, 0, 0, 0, 0, 0)
    t_arm = time.time()
    while time.time() - t_arm < 8.0 and not st.armed:
        send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw, idle, mon)
        drain(m, st, mon); time.sleep(0.01)
    if not st.armed:
        print("[!] NOT ARMED — see [AP]. Try --force-arm."); return
    print("[i] Armed. Taking off with CTBR (throttle-up) ...")

    ground_alt = st.altitude
    T = 1.0 / args.freq
    t0 = time.perf_counter(); t_start = t0
    next_t = t0; last_report = t0; last_loop = t0; jit = 0.0

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_loop; last_loop = now
            if dt <= 0:
                dt = T
            drain(m, st, mon)

            # altitude setpoint: ramp up from ground over 2 s, then hold
            climb_t = now - t_start
            alt_sp = ground_alt + min(args.alt, 0.5 * climb_t) if climb_t < 2.0 else ground_alt + args.alt
            if climb_t > args.hold:
                break

            # altitude PID -> thrust (feedforward = hover throttle)
            thrust = alt_pid.step(alt_sp - st.altitude, st.climb_rate, dt, ff=thr_mid)

            # position PD(+I) -> desired accel -> tilt ref
            xin = clamp(xin + (x_sp - st.x) * dt, -XY_I / max(KI_POS, 1e-6), XY_I / max(KI_POS, 1e-6))
            xie = clamp(xie + (y_sp - st.y) * dt, -XY_I / max(KI_POS, 1e-6), XY_I / max(KI_POS, 1e-6))
            a_n = KP_POS * (x_sp - st.x) - KD_POS * st.vx + KI_POS * xin
            a_e = KP_POS * (y_sp - st.y) - KD_POS * st.vy + KI_POS * xie
            cy, sy = math.cos(st.yaw), math.sin(st.yaw)
            a_fwd = a_n * cy + a_e * sy
            a_rgt = -a_n * sy + a_e * cy
            roll_ref = clamp(a_rgt / G, -TILT_MAX, TILT_MAX)
            pitch_ref = clamp(-a_fwd / G, -TILT_MAX, TILT_MAX)

            # attitude P -> body rates
            roll_rate = clamp(KP_ATT * (roll_ref - st.roll), -RATE_LIM, RATE_LIM)
            pitch_rate = clamp(KP_ATT * (pitch_ref - st.pitch), -RATE_LIM, RATE_LIM)
            yaw_rate = clamp(KP_YAW * wrap_pi(yaw_sp - st.yaw), -YAW_LIM, YAW_LIM)

            send_rc(m, ch_roll, ch_pitch, ch_thr, ch_yaw,
                    build_pwm(roll_rate, pitch_rate, yaw_rate, thrust), mon)

            jit = max(jit, abs(now - (next_t - T)))
            if now - last_report >= 1.0:
                dxy = math.hypot(x_sp - st.x, y_sp - st.y)
                pwm = build_pwm(roll_rate, pitch_rate, yaw_rate, thrust)
                print(f"alt={st.altitude:+.2f}(sp{alt_sp:+.2f}) dxy={dxy:4.2f} thr={thrust:.2f} "
                      f"thrPWM={pwm[cmap['THROTTLE']]} r={math.degrees(st.roll):+5.1f} "
                      f"p={math.degrees(st.pitch):+5.1f} | {mon.report()} | jit<={jit*1e3:4.1f}ms")
                last_report = now; jit = 0.0

            next_t += T
            s = next_t - time.perf_counter()
            if s > 0:
                time.sleep(s)
            else:
                next_t = time.perf_counter()

    except KeyboardInterrupt:
        print("\n[i] Interrupted.")
    finally:
        print("[i] Landing (LAND mode) ...")
        # Confirm LAND *before* releasing the override. Releasing first hands
        # throttle back to the RC receiver while still in ACRO - if the mode
        # change is then dropped, that is zero throttle in mid-air.
        landed_mode = False
        for attempt in range(3):
            if set_mode_confirm(m, st, mon, "LAND", timeout=3.0):
                landed_mode = True
                break
            print(f"[!] LAND not confirmed (attempt {attempt + 1}/3), retrying ...")
        if landed_mode:
            # safe now: LAND is active and controls the throttle itself
            m.mav.rc_channels_override_send(m.target_system, 1, *([0] * 8))
        else:
            print("[!] LAND NOT CONFIRMED - not releasing the override, so throttle "
                  "is not actively handed back mid-air. TAKE MANUAL CONTROL NOW.")
        t_l = time.time()
        while time.time() - t_l < 20.0 and st.armed:
            drain(m, st, mon); time.sleep(0.1)
        print("[i] Done." if not st.armed else "[i] Done (still armed).")


if __name__ == "__main__":
    main()
