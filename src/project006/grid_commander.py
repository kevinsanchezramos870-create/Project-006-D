"""
Fly a 3D-printer-style G-code mission (see project006/gcode.py for the language).

    # check a mission without a drone (always do this first)
    python -m project006.grid_commander missions/delivery_demo.gcode --dry-run

    # fly it (ArduPilot SITL or the real drone)
    python -m project006.grid_commander missions/delivery_demo.gcode --connect udpin:127.0.0.1:14551

X/Y are measured from the exact spot where M17 took off, so (0, 0) is always
the takeoff pad even if the EKF origin was set somewhere else.

Ctrl+C at any point switches the drone to RTL (or LAND with --abort-mode LAND).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from . import config as cfg
from .gcode import GcodeError, describe, parse
from .geometry import Fence


def run(steps, vehicle, safety, log=print):
    from .mav import MavError  # imported here so --dry-run works without pymavlink

    origin = None              # local NED of the takeoff point
    tol = safety["arrival_tolerance_m"]

    def at_target(n, e, alt):
        p = vehicle.local_ned()
        return p and math.dist((p[0], p[1], -p[2]), (n, e, alt)) <= tol

    for s in steps:
        log(f"> line {s.line_no}: {s.source.strip()}")
        vehicle.pump()
        # Once airborne, never run another step if the pilot or a failsafe has
        # taken over - e.g. a payload release while the pilot is flying.
        if origin is not None and s.kind not in ("rtl", "land") and vehicle.mode != "GUIDED":
            raise RuntimeError(f"drone left GUIDED (now {vehicle.mode}) - pilot or failsafe took over; stopping")
        if s.kind == "takeoff":
            vehicle.set_mode("GUIDED")
            try:
                vehicle.gripper(release=False)       # make sure the hook is locked before lifting
            except MavError as e:
                log(f"  (warning) gripper not available: {e}. Is GRIP_ENABLE=1?")
            vehicle.arm()
            vehicle.pump(0.5)
            p = vehicle.local_ned() or (0.0, 0.0, 0.0)
            origin = (p[0], p[1])
            vehicle.takeoff(s.z)
        elif s.kind == "move":
            if vehicle.mode != "GUIDED":
                raise RuntimeError(f"drone left GUIDED (now {vehicle.mode}) - pilot or failsafe took over; stopping")
            if s.speed:
                vehicle.set_speed(s.speed)
            n, e = origin[0] + s.x, origin[1] + s.y
            vehicle.goto_local(n, e, s.z, s.yaw_deg)
            vehicle.wait_for(lambda: at_target(n, e, s.z) or vehicle.mode != "GUIDED",
                             safety["move_timeout_s"], f"arrival at X={s.x} Y={s.y} Z={s.z}")
        elif s.kind == "dwell":
            vehicle.pump(s.seconds)
        elif s.kind == "release":
            vehicle.gripper(release=True)
            vehicle.pump(1.0)
        elif s.kind == "grab":
            vehicle.gripper(release=False)
        elif s.kind == "report":
            p = vehicle.local_ned()
            if p and origin:
                log(f"  M114: X={p[0] - origin[0]:.2f} Y={p[1] - origin[1]:.2f} Z={-p[2]:.2f} "
                    f"mode={vehicle.mode}")
        elif s.kind == "pause":
            input(f"  M0 PAUSE: {s.text or 'drone is holding position'} - press Enter to continue ")
        elif s.kind == "rtl":
            vehicle.set_mode("RTL")
            vehicle.wait_for(lambda: not vehicle.armed, 180, "RTL landing + disarm")
        elif s.kind == "land":
            vehicle.set_mode("LAND")
            vehicle.wait_for(lambda: not vehicle.armed, 120, "landing + disarm")
    log("Mission complete.")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission", help="path to a .gcode mission file")
    ap.add_argument("--config", help="TOML config (default: config/drone.toml if present)")
    ap.add_argument("--connect", help="override vehicle.connection, e.g. udpin:127.0.0.1:14551")
    ap.add_argument("--dry-run", action="store_true", help="parse + validate only; no drone needed")
    ap.add_argument("--abort-mode", choices=["RTL", "LAND", "BRAKE"], default="RTL")
    ap.add_argument("--yes", action="store_true", help="skip the 'type FLY' confirmation (SITL only!)")
    a = ap.parse_args(argv)

    conf = cfg.load(a.config)
    s = conf["safety"]
    fence = Fence(s["fence_radius_m"], s["min_alt_m"], s["max_alt_m"])
    try:
        with open(a.mission) as f:
            steps = parse(f.read(), fence, s["max_speed_m_s"])
    except (OSError, GcodeError) as e:
        print(f"Mission rejected: {e}", file=sys.stderr)
        return 2

    print(f"Mission OK ({len(steps)} steps, fence {fence.radius_m:g} m / {fence.max_alt_m:g} m):")
    print(describe(steps))
    if a.dry_run:
        return 0

    from .mav import MavError, Vehicle
    v = Vehicle(a.connect or conf["vehicle"]["connection"],
                source_system=conf["vehicle"]["source_system"],
                expected_sysid=conf["vehicle"]["drone_id"]).connect()
    if not a.yes and input("\nProps area clear and pilot on the RC sticks? Type FLY to start: ").strip() != "FLY":
        print("Aborted before arming.")
        return 1
    try:
        run(steps, v, s)
        return 0
    except (KeyboardInterrupt, MavError, RuntimeError) as e:
        print(f"\n!! Mission aborted: {e or 'Ctrl+C'} -> switching to {a.abort_mode}", file=sys.stderr)
        try:
            if v.armed:
                v.set_mode(a.abort_mode)
        except Exception as e2:  # last resort: tell the pilot loudly
            print(f"!! Could not change mode ({e2}). TAKE MANUAL CONTROL.", file=sys.stderr)
        return 1
    finally:
        time.sleep(0.2)
        v.close()


if __name__ == "__main__":
    raise SystemExit(main())
