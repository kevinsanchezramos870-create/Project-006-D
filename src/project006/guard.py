"""
Anomaly detection for the onboard guard (pure logic, unit tested).

What this can and cannot do - read before relying on it:
  * It CANNOT stop someone from sending commands to the flight controller.
    MAVLink 2 message signing on the flight controller does that (see
    docs/BUILD_GUIDE.md section 6.4). The old draft's "unknown SYSID" check
    could never see packets addressed to the autopilot, so it was removed.
  * It CAN notice symptoms of jamming, spoofing or hijacking that the
    firmware failsafes do not cover, and react:
      - GPS degraded (fix lost / too few satellites) while armed
      - position "teleport" faster than the drone can physically fly
        (classic GPS-spoofing signature)
      - HOME position moved while armed (someone re-pointing RTL)
      - ground-station heartbeat lost (normally left to FS_GCS_ENABLE)
"""

from __future__ import annotations

import math

from .geometry import latlon_to_ne


class AnomalyDetector:
    def __init__(self, guard_conf: dict):
        self.c = guard_conf
        self.armed = False
        self.last_pos = None          # (t, lat, lon)
        self.home = None              # (lat, lon) captured at/after arming
        self.last_gcs = None
        self.gps_bad_since = None
        self.fired: set[str] = set()

    def _event(self, kind, detail):
        if kind in self.fired:
            return []
        self.fired.add(kind)
        return [(kind, detail)]

    def feed(self, mtype: str, msg, now: float, from_autopilot: bool):
        ev = []
        if mtype == "HEARTBEAT":
            if from_autopilot:
                was = self.armed
                self.armed = bool(msg.base_mode & 128)   # MAV_MODE_FLAG_SAFETY_ARMED
                if self.armed and not was:
                    self.fired.clear()
                    self.home = None
                    self.last_pos = None
            elif msg.type == 6:                           # MAV_TYPE_GCS
                self.last_gcs = now
            return ev
        if not from_autopilot:
            return ev

        if mtype == "GPS_RAW_INT" and self.armed:
            bad = msg.fix_type < 3 or msg.satellites_visible < self.c["min_satellites"]
            if bad:
                self.gps_bad_since = self.gps_bad_since or now
                if now - self.gps_bad_since >= 2.0:
                    ev += self._event("gps_degraded", f"fix_type={msg.fix_type} sats={msg.satellites_visible}")
            else:
                self.gps_bad_since = None

        elif mtype == "GLOBAL_POSITION_INT" and self.armed:
            lat, lon = msg.lat / 1e7, msg.lon / 1e7
            if self.last_pos:
                t0, lat0, lon0 = self.last_pos
                dt = now - t0
                if dt >= 0.05:
                    n, e = latlon_to_ne(lat0, lon0, lat, lon)
                    speed = math.hypot(n, e) / dt
                    if speed > self.c["max_position_jump_m"]:
                        ev += self._event("position_jump", f"position moved at {speed:.0f} m/s")
            self.last_pos = (now, lat, lon)

        elif mtype == "HOME_POSITION" and self.armed:
            lat, lon = msg.latitude / 1e7, msg.longitude / 1e7
            if self.home is None:
                self.home = (lat, lon)
            else:
                n, e = latlon_to_ne(*self.home, lat, lon)
                if math.hypot(n, e) > self.c["home_move_limit_m"]:
                    ev += self._event("home_changed", f"home moved {math.hypot(n, e):.0f} m while armed")

        if self.armed and self.last_gcs is not None and now - self.last_gcs > self.c["gcs_timeout_s"]:
            ev += self._event("gcs_lost", f"no ground-station heartbeat for {now - self.last_gcs:.1f} s")
        return ev

    def action_for(self, kind: str) -> str:
        return {
            "gps_degraded": self.c["on_gps_anomaly"],
            "position_jump": self.c["on_gps_anomaly"],
            "home_changed": self.c["on_home_changed"],
            "gcs_lost": self.c["on_gcs_lost"],
        }.get(kind, "NONE")
