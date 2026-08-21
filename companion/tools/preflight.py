#!/usr/bin/env python3
"""
One-shot GO / NO-GO preflight for ctbr_acro_rc.py, for use in the field.

Read-only apart from a brief RC-override probe, which is sent while the
vehicle is DISARMED and therefore cannot move a motor. It writes no
parameters and never arms.

Usage:
    python3 preflight.py                       # UART, /dev/ttyAMA0 @ 57600
    python3 preflight.py --connect /dev/ttyACM0 --baud 115200
    python3 preflight.py --skip-override       # don't probe RC override
"""
import argparse
import time

from pymavlink import mavutil

FIX = {0: "no GPS", 1: "NO FIX", 2: "2D", 3: "3D", 4: "DGPS",
       5: "RTK float", 6: "RTK fixed"}

MIN_SATS = 8
MAX_HDOP = 2.0

results = []


def check(name, ok, detail, fatal=True):
    results.append((name, ok, detail, fatal))
    mark = "PASS" if ok else ("FAIL" if fatal else "WARN")
    print(f"  [{mark}] {name}: {detail}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Field preflight check (read-only, never arms)")
    ap.add_argument("--connect", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--skip-override", action="store_true")
    args = ap.parse_args()

    print(f"Connecting to {args.connect} @ {args.baud} ...")
    m = mavutil.mavlink_connection(args.connect, baud=args.baud, source_system=255)
    if not m.wait_heartbeat(timeout=15):
        print("\n[FAIL] no heartbeat - check wiring/power. NO-GO.")
        return 1

    def beat():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    for _ in range(3):
        beat(); time.sleep(0.1)

    def gp(n, tries=8):
        for _ in range(tries):
            m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
            t = time.time()
            while time.time() - t < 1.0:
                p = m.recv_match(type="PARAM_VALUE", blocking=False)
                if p and p.param_id.strip("\x00") == n:
                    return p.param_value
                time.sleep(0.005)
            beat()
        return None

    for mid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5),
                    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),
                    (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10),
                    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mid, int(1e6 / hz), 0, 0, 0, 0, 0)
    time.sleep(1.5)

    latest = {}
    t = time.time()
    while time.time() - t < 3.0:
        msg = m.recv_match(blocking=False)
        if msg:
            latest[msg.get_type()] = msg
        else:
            time.sleep(0.01)

    print("\n=== LINK ===")
    t0 = time.time()
    names = ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW",
             "ACRO_RP_RATE", "ACRO_Y_RATE", "ACRO_RP_EXPO", "ACRO_Y_EXPO",
             "MOT_THST_HOVER"]
    vals = {n: gp(n) for n in names}
    for ch in (1, 2, 3, 4):
        for s in ("MIN", "MAX", "TRIM", "DZ", "REVERSED"):
            vals[f"RC{ch}_{s}"] = gp(f"RC{ch}_{s}")
    bad = [k for k, v in vals.items() if v is None]
    dt = time.time() - t0
    check("parameter reads", not bad,
          f"{len(vals)-len(bad)}/{len(vals)} in {dt:.1f}s" + (f", FAILED={bad}" if bad else ""))

    print("\n=== STATE ===")
    hb = latest.get("HEARTBEAT")
    if hb:
        inv = {v: k for k, v in m.mode_mapping().items()}
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        check("disarmed", not armed, f"mode={inv.get(hb.custom_mode,'?')} armed={armed}")
    bat = latest.get("SYS_STATUS")
    if bat and bat.voltage_battery not in (0, 65535):
        v = bat.voltage_battery / 1000.0
        check("battery", v > 10.5, f"{v:.2f} V", fatal=False)

    print("\n=== POSITION (needed to arm: fence requires it) ===")
    g = latest.get("GPS_RAW_INT")
    if g is None:
        check("GPS", False, "no GPS_RAW_INT")
    else:
        hdop = g.eph / 100.0 if g.eph != 65535 else 99.9
        check("GPS fix", g.fix_type >= 3, f"fix_type={g.fix_type} ({FIX.get(g.fix_type,'?')})")
        check("satellites", g.satellites_visible >= MIN_SATS,
              f"{g.satellites_visible} (want >= {MIN_SATS})")
        check("HDOP", hdop <= MAX_HDOP, f"{hdop:.2f} (want <= {MAX_HDOP})")
    lp = latest.get("LOCAL_POSITION_NED")
    check("LOCAL_POSITION_NED", lp is not None,
          f"x={lp.x:.2f} y={lp.y:.2f} z={lp.z:.2f}" if lp else "NOT AVAILABLE - script would hang")
    ek = latest.get("EKF_STATUS_REPORT")
    if ek:
        ok = bool(ek.flags & 16) and bool(ek.flags & 1)      # POS_HORIZ_ABS + ATTITUDE
        check("EKF", ok, f"flags=0x{ek.flags:x} pos_var={ek.pos_horiz_variance:.2f}")

    print("\n=== SAFETY CONFIG ===")
    tr = gp("ACRO_TRAINER")
    check("ACRO_TRAINER", tr == 0.0,
          f"{tr} - flight script needs 0, or run it with --set-trainer", fatal=False)
    fen = gp("FENCE_ENABLE")
    check("fence", True, f"FENCE_ENABLE={fen} (needs position to arm)", fatal=False)

    rc = latest.get("RC_CHANNELS")
    if rc:
        print(f"  [INFO] RC ch1-8: " +
              " ".join(str(getattr(rc, f"chan{i}_raw")) for i in range(1, 9)))

    if not args.skip_override:
        print("\n=== RC OVERRIDE (probe, vehicle disarmed) ===")
        TEST = 1600
        seen = []
        t0 = time.time()
        while time.time() - t0 < 4.0:
            m.mav.rc_channels_override_send(m.target_system, 1, TEST, 0, 0, 0, 0, 0, 0, 0)
            t = time.time()
            while time.time() - t < 0.25:
                r = m.recv_match(type="RC_CHANNELS", blocking=False)
                if r:
                    seen.append(r.chan1_raw)
                else:
                    time.sleep(0.01)
        took = any(abs(v - TEST) <= 2 for v in seen)
        for _ in range(5):
            m.mav.rc_channels_override_send(m.target_system, 1, 0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(0.1)
        check("RC override accepted", took,
              "ch1 followed the override" if took
              else "REJECTED - check the RC_OVERRIDE_ENABLE switch (RC7) is HIGH")

    fails = [r for r in results if not r[1] and r[3]]
    warns = [r for r in results if not r[1] and not r[3]]
    print("\n" + "=" * 52)
    if fails:
        print(f"NO-GO - {len(fails)} blocking issue(s):")
        for n, _, d, _ in fails:
            print(f"   - {n}: {d}")
    else:
        print("GO - all blocking checks passed.")
    if warns:
        print(f"warnings ({len(warns)}):")
        for n, _, d, _ in warns:
            print(f"   - {n}: {d}")
    print("=" * 52)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
