"""
Ground-station swarm coordinator (Linux / Windows / macOS).

    export P006_SWARM_KEY=<same secret as on every drone>
    python -m project006.swarm --config config/drone.toml --dry-run   # check the formation
    python -m project006.swarm --config config/drone.toml

What it does:
  1. Connects to every drone listed in [swarm].drones (one UDP port each).
  2. Uses drone 1's HOME as the shared grid origin (0, 0) for everyone. Each
     drone's own LOCAL_NED grid starts where *it* took off, so local
     coordinates are not shared between drones - GPS coordinates are.
  3. Puts every drone on its own altitude layer (alt_m) so formation changes
     never put two drones at the same height, and checks the minimum 3D
     separation of every slot before sending anything.
  4. Takes off one drone at a time, flies a patrol, and converges the
     formation over any target reported (and HMAC-verified) by the drones'
     cameras. Reports from all drones are fused with a median.
  5. Brings every drone back to above its own home *on its own layer*, then
     lands them one at a time. Ctrl+C triggers that same safe return; a second
     Ctrl+C sends every drone to RTL.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

from . import config as cfg
from .geometry import Fence, latlon_to_ne, min_pairwise_separation, offset_latlon

PATROL = [(0.0, 0.0), (15.0, 0.0), (15.0, 15.0), (0.0, 15.0), (0.0, 0.0)]


def formation_slots(centroid_ne, drones):
    cn, ce = centroid_ne
    return {d["id"]: (cn + d["offset_ne"][0], ce + d["offset_ne"][1], d["alt_m"]) for d in drones}


def check_formation(drones, safety, fence):
    """Raise ValueError if the formation is unsafe anywhere on the patrol."""
    sep, pair = min_pairwise_separation(formation_slots((0, 0), drones))
    if sep < safety["min_separation_m"]:
        raise ValueError(f"drones {pair} are only {sep:.1f} m apart in formation "
                         f"(minimum {safety['min_separation_m']} m)")
    alts = sorted(d["alt_m"] for d in drones)
    if any(b - a < 2.0 for a, b in zip(alts, alts[1:])):
        raise ValueError(f"altitude layers {alts} must be at least 2 m apart")
    for c in PATROL:
        for did, (n, e, alt) in formation_slots(c, drones).items():
            v = fence.violation(n, e, alt)
            if v:
                raise ValueError(f"drone {did} at patrol point {c}: {v}")
    return sep


class Swarm:
    def __init__(self, conf, log=print):
        from .mav import Vehicle
        from .swarm_protocol import TargetFusion, Verifier, load_key
        self.conf, self.log = conf, log
        self.safety = conf["safety"]
        self.fence = Fence(self.safety["fence_radius_m"], self.safety["min_alt_m"], self.safety["max_alt_m"])
        self.drones = conf["swarm"]["drones"]
        self.v = {}
        for d in self.drones:
            self.v[d["id"]] = Vehicle(d["connection"], source_system=255, expected_sysid=d["id"],
                                      log=lambda m, i=d["id"]: log(f"[D{i}] {m}"))
        self.verifier = Verifier(load_key(conf["swarm"]["key_env"]), conf["swarm"]["max_packet_age_s"])
        self.fusion = TargetFusion()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", conf["swarm"]["udp_port"]))
        self.sock.setblocking(False)
        self.origin = None
        self.homes = {}

    # ------------------------------------------------------------------ #
    def connect(self):
        for did, veh in self.v.items():
            veh.connect()
            self.homes[did] = veh.home()
            self.log(f"[D{did}] home {self.homes[did][0]:.6f}, {self.homes[did][1]:.6f}")
        self.origin = self.homes[self.drones[0]["id"]][:2]
        for did, (lat, lon, _) in self.homes.items():
            n, e = latlon_to_ne(*self.origin, lat, lon)
            self.log(f"[D{did}] home is {n:+.1f} N {e:+.1f} E of the shared origin")

    def pump_all(self, seconds=0.0):
        end = time.time() + seconds
        while True:
            for veh in self.v.values():
                veh.pump()
            self._poll_targets()
            if time.time() >= end:
                return
            time.sleep(0.05)

    def _poll_targets(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                self.fusion.add(self.verifier.decode(data))
            except ValueError as e:
                self.log(f"[swarm] dropped packet: {e}")

    def _check_modes(self):
        for did, veh in self.v.items():
            if veh.mode != "GUIDED":
                raise RuntimeError(f"drone {did} is in {veh.mode} (pilot or failsafe took over)")

    def takeoff_all(self):
        for d in self.drones:
            veh = self.v[d["id"]]
            veh.set_mode("GUIDED")
            veh.arm()
            veh.takeoff(d["alt_m"])
            self.pump_all(1.0)

    def move_formation(self, centroid_ne, timeout=None):
        timeout = timeout or self.safety["move_timeout_s"]
        slots = formation_slots(centroid_ne, self.drones)
        targets = {}
        for did, (n, e, alt) in slots.items():
            v = self.fence.violation(n, e, alt)
            if v:
                raise RuntimeError(f"drone {did}: {v}")
            lat, lon = offset_latlon(*self.origin, n, e)
            targets[did] = (lat, lon, alt)
            self.v[did].goto_global(lat, lon, alt)
        tol = self.safety["arrival_tolerance_m"] * 2
        end = time.time() + timeout
        while time.time() < end:
            self.pump_all(0.2)
            self._check_modes()
            done = True
            for did, (lat, lon, alt) in targets.items():
                p = self.v[did].global_pos()
                if not p:
                    done = False
                    continue
                n, e = latlon_to_ne(lat, lon, p[0], p[1])
                if (n * n + e * e + (p[2] - alt) ** 2) ** 0.5 > tol:
                    done = False
            if done:
                return True
        self.log("(warning) formation move timed out; continuing")
        return False

    def patrol(self):
        for c in PATROL:
            est = self.fusion.estimate()
            if est:
                tlat, tlon, n_obs = est
                tn, te = latlon_to_ne(*self.origin, tlat, tlon)
                r = (tn * tn + te * te) ** 0.5
                limit = self.fence.radius_m - 10
                if r > limit:   # stay inside the fence
                    tn, te = tn * limit / r, te * limit / r
                self.log(f"[swarm] target seen by {n_obs} drone(s) -> formation over {tn:+.1f} N {te:+.1f} E")
                self.move_formation((tn, te))
                self.pump_all(5.0)
            else:
                self.log(f"[swarm] patrol -> {c}")
                self.move_formation(c)
                self.pump_all(2.0)

    def return_and_land(self):
        """Each drone flies above its OWN home on its OWN layer, then lands one by one."""
        for d in self.drones:
            veh = self.v[d["id"]]
            if veh.armed:
                if veh.mode != "GUIDED":
                    veh.set_mode("GUIDED")
                lat, lon, _ = self.homes[d["id"]]
                veh.goto_global(lat, lon, d["alt_m"])
        self.pump_all(1.0)
        for d in self.drones:
            veh = self.v[d["id"]]
            if not veh.armed:
                continue
            lat, lon, _ = self.homes[d["id"]]
            try:
                veh.wait_for(lambda: veh.global_pos() and
                             sum(x * x for x in latlon_to_ne(lat, lon, *veh.global_pos()[:2])) ** 0.5 < 2.0,
                             90, f"drone {d['id']} above home")
            except Exception as e:
                self.log(f"[D{d['id']}] {e}")
            veh.set_mode("LAND")
            veh.wait_for(lambda: not veh.armed, 120, f"drone {d['id']} landed")
            self.log(f"[D{d['id']}] landed")

    def rtl_all(self):
        for did, veh in self.v.items():
            try:
                veh.set_mode("RTL")
            except Exception as e:
                print(f"!! D{did}: could not set RTL ({e}) - TAKE MANUAL CONTROL", file=sys.stderr)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config")
    ap.add_argument("--dry-run", action="store_true", help="only check the formation geometry")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt (SITL only!)")
    a = ap.parse_args(argv)
    conf = cfg.load(a.config)
    s = conf["safety"]
    fence = Fence(s["fence_radius_m"], s["min_alt_m"], s["max_alt_m"])
    try:
        sep = check_formation(conf["swarm"]["drones"], s, fence)
    except ValueError as e:
        print(f"Formation rejected: {e}", file=sys.stderr)
        return 2
    print(f"Formation OK: {len(conf['swarm']['drones'])} drones, minimum separation {sep:.1f} m")
    if a.dry_run:
        return 0

    swarm = Swarm(conf)
    swarm.connect()
    if not a.yes and input("Every pilot ready on their RC? Type FLY: ").strip() != "FLY":
        return 1
    try:
        swarm.takeoff_all()
        swarm.patrol()
        swarm.return_and_land()
        return 0
    except (KeyboardInterrupt, RuntimeError, Exception) as e:
        print(f"\n!! Swarm abort: {e or 'Ctrl+C'} -> layered return (Ctrl+C again = RTL all)", file=sys.stderr)
        try:
            swarm.return_and_land()
        except BaseException:
            swarm.rtl_all()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
