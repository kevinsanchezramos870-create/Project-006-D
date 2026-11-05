"""Loads config/drone.toml (or a path you pass) on top of safe defaults."""

from __future__ import annotations

import copy
import os
import tomllib
from pathlib import Path

DEFAULTS = {
    "vehicle": {
        # Onboard (via mavlink-router): tcp:127.0.0.1:5760
        # SITL with MAVProxy --out:     udpin:127.0.0.1:14551
        "connection": "tcp:127.0.0.1:5760",
        "drone_id": 1,                 # must equal SYSID_THISMAV on the flight controller
        "source_system": 255,          # 255 = acts as the GCS (keeps FS_GCS failsafe fed)
    },
    "safety": {
        "fence_radius_m": 100.0,
        "min_alt_m": 2.0,
        "max_alt_m": 60.0,
        "max_speed_m_s": 8.0,
        "arrival_tolerance_m": 0.75,
        "move_timeout_s": 60.0,
        "min_separation_m": 4.0,      # swarm: minimum 3D distance between slots
    },
    "camera": {
        "width": 640, "height": 360,   # tracking stream; recording is 1920x1080
        "hfov_deg": 66.0, "vfov_deg": 41.0,
        "tilt_from_nadir_deg": 0.0,
        "record_dir": "~/flights",
    },
    "tracking": {
        # HSV ranges (OpenCV: H 0-179). Two ranges because red wraps around 0.
        "hsv_ranges": [[[0, 120, 70], [10, 255, 255]], [[170, 120, 70], [179, 255, 255]]],
        "min_area_px": 150,
        "follow_gain": 0.5,            # m/s of velocity per metre of ground error
        "follow_max_speed_m_s": 2.0,
        "follow_min_alt_m": 4.0,
    },
    "swarm": {
        "udp_port": 5005,
        "key_env": "P006_SWARM_KEY",
        "max_packet_age_s": 2.0,
        "drones": [
            {"id": 1, "connection": "udpin:0.0.0.0:14551", "offset_ne": [0.0, 0.0], "alt_m": 8.0},
            {"id": 2, "connection": "udpin:0.0.0.0:14552", "offset_ne": [-5.0, -5.0], "alt_m": 11.0},
            {"id": 3, "connection": "udpin:0.0.0.0:14553", "offset_ne": [-5.0, 5.0], "alt_m": 14.0},
        ],
    },
    "guard": {
        "min_satellites": 8,
        "max_position_jump_m": 15.0,   # per second, beyond what the drone can fly
        "home_move_limit_m": 5.0,
        "gcs_timeout_s": 5.0,
        "on_gps_anomaly": "LAND",      # RTL needs trustworthy GPS - LAND does not
        "on_home_changed": "LAND",
        "on_gcs_lost": "NONE",         # firmware FS_GCS_ENABLE already handles this
    },
}


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path: str | os.PathLike | None = None) -> dict:
    candidates = [path] if path else [os.environ.get("P006_CONFIG"), "config/drone.toml",
                                      Path.home() / ".config/project006/drone.toml"]
    for c in candidates:
        if c and Path(c).expanduser().is_file():
            with open(Path(c).expanduser(), "rb") as f:
                return _merge(DEFAULTS, tomllib.load(f))
    if path:
        raise FileNotFoundError(path)
    return copy.deepcopy(DEFAULTS)
