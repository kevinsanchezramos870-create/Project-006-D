"""
Signed swarm messages (target sightings) and multi-drone target fusion.

The original design broadcast plain JSON, so anyone on the Wi-Fi could inject
fake targets and steer the whole swarm. Every packet is now signed with
HMAC-SHA256 using a shared key, timestamped, and sequence-numbered so stale or
replayed packets are rejected.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import statistics
import time

PROTOCOL_VERSION = 1


def load_key(env_var: str = "P006_SWARM_KEY") -> bytes:
    key = os.environ.get(env_var, "")
    if len(key) < 16:
        raise RuntimeError(
            f"Set {env_var} to a shared secret of at least 16 characters on every drone "
            f"and the ground PC, e.g.  export {env_var}=$(python3 -c 'import secrets;print(secrets.token_hex(16))')")
    return key.encode()


def _canonical(body: dict) -> bytes:
    return json.dumps(body, sort_keys=True, separators=(",", ":")).encode()


def encode(body: dict, key: bytes) -> bytes:
    body = dict(body, v=PROTOCOL_VERSION)
    sig = hmac.new(key, _canonical(body), hashlib.sha256).hexdigest()
    return _canonical(dict(body, sig=sig))


class Verifier:
    """Checks signature, age and replay. One per receiver."""

    def __init__(self, key: bytes, max_age_s: float = 2.0, clock=time.time):
        self.key = key
        self.max_age_s = max_age_s
        self.clock = clock
        self._last_seq: dict[int, int] = {}

    def decode(self, data: bytes):
        """Returns the message dict, or raises ValueError with the reason."""
        try:
            msg = json.loads(data)
            sig = msg.pop("sig")
        except (ValueError, KeyError, TypeError) as e:
            raise ValueError(f"malformed packet: {e}") from None
        expected = hmac.new(self.key, _canonical(msg), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(sig, expected):
            raise ValueError("bad signature")
        if msg.get("v") != PROTOCOL_VERSION:
            raise ValueError("protocol version mismatch")
        age = self.clock() - float(msg["t"])
        if abs(age) > self.max_age_s:
            raise ValueError(f"stale packet ({age:.1f} s old) - check clocks (chrony/NTP)")
        drone, seq = int(msg["drone"]), int(msg["seq"])
        if seq <= self._last_seq.get(drone, -1):
            raise ValueError(f"replayed packet from drone {drone} (seq {seq})")
        self._last_seq[drone] = seq
        return msg


def target_report(drone_id: int, seq: int, lat: float, lon: float, area_px: float,
                  drone_lat: float, drone_lon: float, drone_alt: float) -> dict:
    return {
        "type": "target", "drone": drone_id, "seq": seq, "t": time.time(),
        "target": [round(lat, 7), round(lon, 7)], "area_px": round(area_px),
        "pos": [round(drone_lat, 7), round(drone_lon, 7), round(drone_alt, 2)],
    }


class TargetFusion:
    """Keeps the newest sighting per drone and fuses them with a median, so a
    single drone with a bad estimate cannot drag the swarm away."""

    def __init__(self, max_age_s: float = 3.0, clock=time.time):
        self.max_age_s = max_age_s
        self.clock = clock
        self._latest: dict[int, tuple[float, float, float]] = {}

    def add(self, msg: dict):
        lat, lon = msg["target"]
        self._latest[int(msg["drone"])] = (float(msg["t"]), lat, lon)

    def estimate(self):
        now = self.clock()
        fresh = [(lat, lon) for (t, lat, lon) in self._latest.values() if now - t <= self.max_age_s]
        if not fresh:
            return None
        return (statistics.median(p[0] for p in fresh),
                statistics.median(p[1] for p in fresh),
                len(fresh))
