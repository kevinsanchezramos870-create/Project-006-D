"""
Thin, ArduPilot-Copter-specific wrapper around pymavlink.

Why not MAVSDK? MAVSDK's offboard API targets PX4. On ArduPilot the native way
to fly a companion-computer mission is GUIDED mode + SET_POSITION_TARGET_*,
which is what this module does. Every command waits for COMMAND_ACK, so a
refused arm/takeoff is reported instead of silently ignored.
"""

from __future__ import annotations

import math
import threading
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

# type_mask bits: 1 = ignore that field
_IGNORE_VEL = 0b111 << 3
_IGNORE_ACC = 0b111 << 6
_IGNORE_YAW = 1 << 10
_IGNORE_YAW_RATE = 1 << 11
POS_ONLY = _IGNORE_VEL | _IGNORE_ACC | _IGNORE_YAW_RATE          # position + yaw
POS_NO_YAW = POS_ONLY | _IGNORE_YAW
VEL_ONLY = 0b111 | _IGNORE_ACC | _IGNORE_YAW | _IGNORE_YAW_RATE

COPTER_MODES = dict(mavutil.mode_mapping_acm)               # number -> name
COPTER_MODE_IDS = {name: num for num, name in COPTER_MODES.items()}


class MavError(RuntimeError):
    pass


def signing_key(env_var: str = "P006_MAVLINK_PASSPHRASE"):
    """32-byte MAVLink signing key from a passphrase, or None if unset."""
    import hashlib
    import os
    phrase = os.environ.get(env_var)
    return hashlib.sha256(phrase.encode()).digest() if phrase else None


class Vehicle:
    def __init__(self, connection: str, source_system: int = 255,
                 source_component: int = mavlink.MAV_COMP_ID_MISSIONPLANNER,
                 expected_sysid: int | None = None, heartbeat: bool = True, log=print):
        self.log = log
        self.expected_sysid = expected_sysid
        self.m = mavutil.mavlink_connection(
            connection, source_system=source_system, source_component=source_component,
            autoreconnect=True, dialect="ardupilotmega")
        key = signing_key()
        if key:
            # Same key derivation as Mission Planner / MAVProxy: SHA-256 of the passphrase.
            self.m.setup_signing(key, sign_outgoing=True)
            log("MAVLink 2 signing enabled for this link")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.latest: dict[str, object] = {}
        self.acks: dict[int, object] = {}
        self.mode = None
        self.armed = False
        self._listeners = []
        self._hb_thread = None
        self._want_heartbeat = heartbeat
        self._is_gcs = source_component == mavlink.MAV_COMP_ID_MISSIONPLANNER

    # ------------------------------------------------------------------ io --
    def send(self, fn_name: str, *args):
        with self._lock:
            getattr(self.m.mav, fn_name)(*args)

    def on_message(self, callback):
        """callback(msg) for every message from the autopilot."""
        self._listeners.append(callback)

    def _heartbeat_loop(self):
        mav_type = mavlink.MAV_TYPE_GCS if self._is_gcs else mavlink.MAV_TYPE_ONBOARD_CONTROLLER
        while not self._stop.wait(1.0):
            self.send("heartbeat_send", mav_type, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE)

    def pump(self, timeout: float = 0.0):
        """Read every waiting message and update self.latest. Returns count."""
        n = 0
        deadline = time.time() + timeout
        while True:
            msg = self.m.recv_msg()
            if msg is None:
                if time.time() >= deadline:
                    return n
                time.sleep(0.005)
                continue
            n += 1
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            for cb in self._listeners:      # listeners see everything (e.g. GCS heartbeats)
                cb(msg)
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (self.m.target_system, self.m.target_component):
                continue                    # state below only tracks *our* autopilot
            if t == "HEARTBEAT":
                self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))
            elif t == "COMMAND_ACK":
                self.acks[msg.command] = msg
            elif t == "STATUSTEXT":
                self.log(f"[FC] {msg.text}")
            self.latest[t] = msg

    def wait_for(self, predicate, timeout: float, what: str, poll: float = 0.05):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump(poll)
            if predicate():
                return
        raise MavError(f"timed out after {timeout:.0f} s waiting for {what}")

    # ------------------------------------------------------------- connect --
    def connect(self, timeout: float = 30.0):
        """Wait for the *autopilot* heartbeat (not a GCS or camera)."""
        self.log(f"Waiting for autopilot heartbeat on {self.m.address} ...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb is None or hb.autopilot == mavlink.MAV_AUTOPILOT_INVALID or hb.type == mavlink.MAV_TYPE_GCS:
                continue
            if self.expected_sysid is not None and hb.get_srcSystem() != self.expected_sysid:
                continue
            self.m.target_system = hb.get_srcSystem()
            self.m.target_component = hb.get_srcComponent()
            self.mode = COPTER_MODES.get(hb.custom_mode, str(hb.custom_mode))
            self.armed = bool(hb.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.log(f"Connected to system {self.m.target_system} (mode {self.mode}, "
                     f"{'ARMED' if self.armed else 'disarmed'})")
            break
        else:
            raise MavError(f"no autopilot heartbeat within {timeout:.0f} s")
        if self._want_heartbeat and self._hb_thread is None:
            self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._hb_thread.start()
        for msg_id, hz in ((mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
                           (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                           (mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
                           (mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
                           (mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)):
            self.set_message_rate(msg_id, hz)
        return self

    def close(self):
        self._stop.set()
        self.m.close()

    # ------------------------------------------------------------ commands --
    def command(self, cmd: int, *params: float, timeout: float = 3.0, retries: int = 2):
        p = list(params) + [0.0] * (7 - len(params))
        for attempt in range(retries + 1):
            self.acks.pop(cmd, None)
            self.send("command_long_send", self.m.target_system, self.m.target_component, cmd, attempt, *p)
            try:
                self.wait_for(lambda: cmd in self.acks, timeout, f"ACK for command {cmd}")
            except MavError:
                if attempt == retries:
                    raise
                continue
            res = self.acks[cmd].result
            if res in (mavlink.MAV_RESULT_ACCEPTED, mavlink.MAV_RESULT_IN_PROGRESS):
                return res
            name = mavlink.enums["MAV_RESULT"][res].name if res in mavlink.enums["MAV_RESULT"] else res
            raise MavError(f"command {cmd} rejected: {name}")

    def set_message_rate(self, msg_id: int, hz: float):
        try:
            self.command(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz, timeout=1.5, retries=1)
        except MavError as e:
            self.log(f"(warning) could not set rate for message {msg_id}: {e}")

    def set_mode(self, name: str, timeout: float = 5.0):
        if name not in COPTER_MODE_IDS:
            raise MavError(f"unknown Copter mode {name}")
        self.command(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE_IDS[name])
        self.wait_for(lambda: self.mode == name, timeout, f"mode {name}")
        self.log(f"Mode -> {name}")

    def arm(self, timeout: float = 10.0):
        self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.wait_for(lambda: self.armed, timeout, "arming")
        self.log("ARMED")

    def disarm(self):
        self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def takeoff(self, alt_m: float, timeout: float = 45.0):
        self.command(mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
        self.wait_for(lambda: self.rel_alt() >= alt_m * 0.92, timeout, f"climb to {alt_m} m")
        self.log(f"Takeoff complete at {self.rel_alt():.1f} m")

    def set_speed(self, m_s: float):
        self.command(mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, m_s, -1)

    def gripper(self, release: bool):
        # MAV_CMD_DO_GRIPPER: param1 = gripper instance, param2 = 0 release / 1 grab
        self.command(mavlink.MAV_CMD_DO_GRIPPER, 1, 0 if release else 1)
        self.log("Payload RELEASED" if release else "Payload hook LOCKED")

    def set_param(self, name: str, value: float, timeout: float = 3.0):
        self.latest.pop("PARAM_VALUE", None)
        self.send("param_set_send", self.m.target_system, self.m.target_component,
                  name.encode(), float(value), mavlink.MAV_PARAM_TYPE_REAL32)
        self.wait_for(lambda: (pv := self.latest.get("PARAM_VALUE")) is not None and pv.param_id == name,
                      timeout, f"param {name}")

    # ------------------------------------------------------------- targets --
    def goto_local(self, north: float, east: float, alt_up: float, yaw_deg: float | None = None):
        mask = POS_NO_YAW if yaw_deg is None else POS_ONLY
        yaw = math.radians(yaw_deg or 0.0)
        self.send("set_position_target_local_ned_send", 0, self.m.target_system, self.m.target_component,
                  mavlink.MAV_FRAME_LOCAL_NED, mask, north, east, -alt_up, 0, 0, 0, 0, 0, 0, yaw, 0)

    def goto_global(self, lat: float, lon: float, alt_rel: float, yaw_deg: float | None = None):
        mask = POS_NO_YAW if yaw_deg is None else POS_ONLY
        self.send("set_position_target_global_int_send", 0, self.m.target_system, self.m.target_component,
                  mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, int(lat * 1e7), int(lon * 1e7), alt_rel,
                  0, 0, 0, 0, 0, 0, math.radians(yaw_deg or 0.0), 0)

    def velocity_ned(self, vn: float, ve: float, vd: float = 0.0):
        """ArduPilot stops the drone if these stop arriving for GUID_TIMEOUT s."""
        self.send("set_position_target_local_ned_send", 0, self.m.target_system, self.m.target_component,
                  mavlink.MAV_FRAME_LOCAL_NED, VEL_ONLY, 0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)

    # ------------------------------------------------------------- telemetry --
    def local_ned(self):
        p = self.latest.get("LOCAL_POSITION_NED")
        return (p.x, p.y, p.z) if p else None

    def rel_alt(self) -> float:
        g = self.latest.get("GLOBAL_POSITION_INT")
        return g.relative_alt / 1000.0 if g else 0.0

    def global_pos(self):
        g = self.latest.get("GLOBAL_POSITION_INT")
        return (g.lat / 1e7, g.lon / 1e7, g.relative_alt / 1000.0) if g else None

    def attitude(self):
        a = self.latest.get("ATTITUDE")
        return (a.roll, a.pitch, a.yaw) if a else (0.0, 0.0, 0.0)

    def home(self, timeout: float = 5.0):
        """(lat, lon, alt_msl_m) of the home position."""
        self.latest.pop("HOME_POSITION", None)
        self.command(mavlink.MAV_CMD_REQUEST_MESSAGE, mavlink.MAVLINK_MSG_ID_HOME_POSITION)
        self.wait_for(lambda: "HOME_POSITION" in self.latest, timeout, "HOME_POSITION")
        h = self.latest["HOME_POSITION"]
        return h.latitude / 1e7, h.longitude / 1e7, h.altitude / 1000.0

    # --------------------------------------------------------------- rally --
    def upload_rally_points(self, points, timeout: float = 5.0):
        """points: list of (lat, lon, alt_rel_m). RTL flies to the nearest one."""
        mt = mavlink.MAV_MISSION_TYPE_RALLY
        self.send("mission_count_send", self.m.target_system, self.m.target_component, len(points), mt)
        sent = set()
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                    blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type != mavlink.MAV_MISSION_ACCEPTED:
                    raise MavError(f"rally upload rejected (code {msg.type})")
                self.log(f"Uploaded {len(points)} rally point(s)")
                return
            lat, lon, alt = points[msg.seq]
            self.send("mission_item_int_send", self.m.target_system, self.m.target_component, msg.seq,
                      mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavlink.MAV_CMD_NAV_RALLY_POINT,
                      0, 0, 0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt, mt)
            sent.add(msg.seq)
        raise MavError(f"rally upload timed out (sent {sorted(sent)})")
