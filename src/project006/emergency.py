"""
Emergency return tools.

  Ground PC - set where the drone goes on RTL / failsafe (a rally point). This
  lives in the FLIGHT CONTROLLER, so it still works if the Pi or your laptop dies:
      python -m project006.emergency rally --lat 28.60210 --lon -81.20010 --alt 15
      python -m project006.emergency rally --x 20 --y -5 --alt 15     # metres from home

  Ground PC - panic button (any time, from any terminal):
      python -m project006.emergency panic             # RTL
      python -m project006.emergency panic --mode LAND

  Onboard Pi - run the anomaly guard (systemd service p006-guard):
      python -m project006.emergency guard
"""

from __future__ import annotations

import argparse
import sys
import time

from . import config as cfg
from .geometry import offset_latlon


def _vehicle(conf, args, onboard=False):
    from pymavlink import mavutil
    from .mav import Vehicle
    # Guard uses component 192 so it never collides with the vision process (191).
    comp = getattr(mavutil.mavlink, "MAV_COMP_ID_ONBOARD_COMPUTER2", 192) if onboard \
        else mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
    src = conf["vehicle"]["drone_id"] if onboard else conf["vehicle"]["source_system"]
    return Vehicle(args.connect or conf["vehicle"]["connection"], source_system=src, source_component=comp,
                   expected_sysid=conf["vehicle"]["drone_id"], heartbeat=not onboard).connect()


def cmd_rally(conf, a):
    v = _vehicle(conf, a)
    if a.lat is not None and a.lon is not None:
        lat, lon = a.lat, a.lon
    elif a.x is not None and a.y is not None:
        hlat, hlon, _ = v.home()
        lat, lon = offset_latlon(hlat, hlon, a.x, a.y)
    else:
        print("give --lat/--lon or --x/--y", file=sys.stderr)
        return 2
    v.upload_rally_points([(lat, lon, a.alt)])
    if not a.keep_home:
        v.set_param("RALLY_INCL_HOME", 0)    # RTL goes to the rally point, not home
    print(f"Emergency return point set: {lat:.7f}, {lon:.7f} at {a.alt} m "
          f"({'home also allowed' if a.keep_home else 'home excluded'}). "
          "RALLY_LIMIT_KM must be larger than the distance to it.")
    v.close()
    return 0


def cmd_panic(conf, a):
    v = _vehicle(conf, a)
    v.set_mode(a.mode)
    print(f"Drone {conf['vehicle']['drone_id']} -> {a.mode}")
    v.close()
    return 0


def cmd_guard(conf, a):
    from .guard import AnomalyDetector
    v = _vehicle(conf, a, onboard=True)
    det = AnomalyDetector(conf["guard"])
    print("[guard] running. Watching GPS health, position jumps, home changes, GCS link.")

    pending = []   # (kind, detail) - acted on in the main loop, not inside the callback

    def on_msg(msg):
        from_ap = (msg.get_srcSystem(), msg.get_srcComponent()) == (v.m.target_system, v.m.target_component)
        pending.extend(det.feed(msg.get_type(), msg, time.time(), from_ap))

    v.on_message(on_msg)
    # HOME_POSITION is not streamed by default; ask for it at 0.5 Hz
    from pymavlink import mavutil
    v.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 0.5)
    try:
        while True:
            v.pump(0.2)
            while pending:
                kind, detail = pending.pop(0)
                action = det.action_for(kind)
                print(f"[guard] !!! {kind}: {detail} -> action {action}", flush=True)
                v.send("statustext_send", 2, f"P006 guard: {kind} -> {action}"[:50].encode())  # CRITICAL
                if action != "NONE" and v.armed and v.mode != action:
                    try:
                        v.set_mode(action)
                    except Exception as e:
                        print(f"[guard] mode change failed: {e}", flush=True)
    except KeyboardInterrupt:
        return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config")
    ap.add_argument("--connect")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("rally", help="set the emergency return point")
    r.add_argument("--lat", type=float)
    r.add_argument("--lon", type=float)
    r.add_argument("--x", type=float, help="metres north of home")
    r.add_argument("--y", type=float, help="metres east of home")
    r.add_argument("--alt", type=float, default=15.0, help="metres above home")
    r.add_argument("--keep-home", action="store_true", help="RTL may still choose home if it is closer")
    p = sub.add_parser("panic", help="switch the drone to RTL/LAND/BRAKE now")
    p.add_argument("--mode", choices=["RTL", "LAND", "BRAKE", "SMART_RTL"], default="RTL")
    sub.add_parser("guard", help="onboard anomaly guard")
    a = ap.parse_args(argv)
    conf = cfg.load(a.config)
    return {"rally": cmd_rally, "panic": cmd_panic, "guard": cmd_guard}[a.cmd](conf, a)


if __name__ == "__main__":
    raise SystemExit(main())
