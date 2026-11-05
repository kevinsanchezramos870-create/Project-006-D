"""
Pure-math helpers: camera ray -> ground position, local NED <-> GPS, geofence
and swarm separation checks. No hardware or pymavlink imports, so everything
here is unit-tested in tests/.

Frames used throughout the project:
  * NED  - North, East, Down in metres. Altitude "up" is -Down.
  * Body - x forward, y right, z down (aircraft frame).
  * Camera - x right, y down (image axes), z out of the lens.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6378137.0  # WGS-84 equatorial radius


# --------------------------------------------------------------------------- #
# Rotations
# --------------------------------------------------------------------------- #
def _mat_vec(m, v):
    return tuple(sum(m[r][c] * v[c] for c in range(3)) for r in range(3))


def body_to_ned_matrix(roll: float, pitch: float, yaw: float):
    """Standard aerospace ZYX rotation (radians), body -> NED."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


# --------------------------------------------------------------------------- #
# Camera geometry
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CameraModel:
    width_px: int = 640
    height_px: int = 360
    hfov_deg: float = 66.0    # Raspberry Pi Camera Module 3 (standard lens)
    vfov_deg: float = 41.0
    # Mount tilt measured from straight down: 0 = nadir, 30 = tilted 30 deg
    # toward the nose. The top edge of the image must face the nose.
    tilt_from_nadir_deg: float = 0.0

    @property
    def fx(self) -> float:
        return (self.width_px / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def fy(self) -> float:
        return (self.height_px / 2.0) / math.tan(math.radians(self.vfov_deg) / 2.0)

    def pixel_to_body_ray(self, u: float, v: float):
        """Unit-free direction (body frame) of the ray through pixel (u, v)."""
        xc = (u - self.width_px / 2.0) / self.fx   # right
        yc = (v - self.height_px / 2.0) / self.fy  # down in the image
        # Nadir mount: lens axis = body down, image right = body right,
        # image down = body backwards.
        bx, by, bz = -yc, xc, 1.0
        # Tilt the lens forward by theta (rotation about body y axis).
        t = math.radians(self.tilt_from_nadir_deg)
        return (bx * math.cos(t) + bz * math.sin(t), by, -bx * math.sin(t) + bz * math.cos(t))


def pixel_to_ground_offset_ned(
    cam: CameraModel, u: float, v: float, height_m: float,
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
):
    """
    Where on flat ground the pixel (u, v) is, as (north_m, east_m) relative to
    the point directly below the drone. Returns None if the ray points at or
    above the horizon (no ground intersection).
    """
    if height_m <= 0:
        return None
    ray_b = cam.pixel_to_body_ray(u, v)
    n, e, d = _mat_vec(body_to_ned_matrix(roll, pitch, yaw), ray_b)
    if d <= 1e-6:
        return None
    scale = height_m / d
    return n * scale, e * scale


# --------------------------------------------------------------------------- #
# Local NED <-> GPS (flat-earth, accurate to cm over a few km)
# --------------------------------------------------------------------------- #
def offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float):
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def latlon_to_ne(origin_lat: float, origin_lon: float, lat: float, lon: float):
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north, east


# --------------------------------------------------------------------------- #
# Safety envelopes
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Fence:
    """Cylinder around the takeoff point. Mirrors the FENCE_* params, but is
    checked *before* a command is sent so bad missions never start."""
    radius_m: float = 100.0
    min_alt_m: float = 2.0
    max_alt_m: float = 60.0

    def violation(self, north: float, east: float, alt: float):
        r = math.hypot(north, east)
        if r > self.radius_m:
            return f"{r:.1f} m from home exceeds fence radius {self.radius_m:.0f} m"
        if alt > self.max_alt_m:
            return f"altitude {alt:.1f} m exceeds max {self.max_alt_m:.0f} m"
        if alt < self.min_alt_m:
            return f"altitude {alt:.1f} m is below the {self.min_alt_m:.0f} m minimum for flight moves"
        return None


def min_pairwise_separation(points):
    """Smallest 3D distance between any two (n, e, alt) points, and the pair."""
    best = (math.inf, None)
    pts = list(points.items()) if isinstance(points, dict) else list(enumerate(points))
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            (ka, a), (kb, b) = pts[i], pts[j]
            dist = math.dist(a, b)
            if dist < best[0]:
                best = (dist, (ka, kb))
    return best
