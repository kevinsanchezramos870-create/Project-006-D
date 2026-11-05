import math
from types import SimpleNamespace

import pytest

from project006 import config, gcode, geometry, sizing, swarm_protocol
from project006.guard import AnomalyDetector
from project006.swarm import check_formation


# ----------------------------------------------------------------- sizing --
def test_default_build_meets_half_pound_requirement():
    r = sizing.analyze(sizing.Design())
    assert r["payload_lb"] == pytest.approx(0.5, abs=0.01)
    assert r["loaded"]["thrust_to_weight"] >= 2.0
    assert r["loaded"]["hover_minutes"] > 8
    assert r["empty"]["hover_minutes"] > r["loaded"]["hover_minutes"]


def test_hover_power_scales_with_mass_to_1_5():
    d = sizing.Design(avionics_w=0)
    ratio = sizing.hover_power_w(2000, d) / sizing.hover_power_w(1000, d)
    assert ratio == pytest.approx(2 ** 1.5, rel=1e-6)


def test_underpowered_design_warns():
    r = sizing.analyze(sizing.Design(max_thrust_per_motor_g=300))
    assert any("below 2:1" in w for w in r["warnings"])


# --------------------------------------------------------------- geometry --
def test_center_pixel_is_straight_below_nadir_camera():
    cam = geometry.CameraModel()
    n, e = geometry.pixel_to_ground_offset_ned(cam, 320, 180, 10.0)
    assert n == pytest.approx(0, abs=1e-9) and e == pytest.approx(0, abs=1e-9)


def test_right_edge_of_image_matches_half_hfov():
    cam = geometry.CameraModel()
    n, e = geometry.pixel_to_ground_offset_ned(cam, 640, 180, 10.0)
    assert e == pytest.approx(10 * math.tan(math.radians(33)), rel=1e-6)
    assert n == pytest.approx(0, abs=1e-9)


def test_top_of_image_is_forward_and_rotates_with_heading():
    cam = geometry.CameraModel()
    n, e = geometry.pixel_to_ground_offset_ned(cam, 320, 0, 10.0)            # facing north
    assert n > 0 and e == pytest.approx(0, abs=1e-9)
    n2, e2 = geometry.pixel_to_ground_offset_ned(cam, 320, 0, 10.0, yaw=math.radians(90))  # facing east
    assert e2 == pytest.approx(n, rel=1e-6) and n2 == pytest.approx(0, abs=1e-9)


def test_tilted_camera_center_is_ahead():
    cam = geometry.CameraModel(tilt_from_nadir_deg=30)
    n, e = geometry.pixel_to_ground_offset_ned(cam, 320, 180, 10.0)
    assert n == pytest.approx(10 * math.tan(math.radians(30)), rel=1e-6)


def test_ray_above_horizon_returns_none():
    cam = geometry.CameraModel(tilt_from_nadir_deg=85)
    assert geometry.pixel_to_ground_offset_ned(cam, 320, 0, 10.0) is None


def test_latlon_round_trip():
    lat, lon = geometry.offset_latlon(28.6, -81.2, 120.0, -45.0)
    n, e = geometry.latlon_to_ne(28.6, -81.2, lat, lon)
    assert n == pytest.approx(120.0, abs=0.01) and e == pytest.approx(-45.0, abs=0.01)


def test_fence():
    f = geometry.Fence(radius_m=50, min_alt_m=2, max_alt_m=30)
    assert f.violation(10, 10, 5) is None
    assert "radius" in f.violation(40, 40, 5)
    assert "exceeds max" in f.violation(0, 0, 31)
    assert "minimum" in f.violation(0, 0, 1)


# ------------------------------------------------------------------ gcode --
def test_parse_demo_mission():
    with open("missions/delivery_demo.gcode") as fh:
        steps = gcode.parse(fh.read())
    kinds = [s.kind for s in steps]
    assert kinds[0] == "takeoff" and kinds[-1] == "rtl" and "release" in kinds
    move = [s for s in steps if s.kind == "move"][1]
    assert (move.x, move.y, move.z, move.yaw_deg) == (15, 10, 5, 90)


def test_parse_square_relative_mission():
    with open("missions/square_survey.gcode") as fh:
        steps = gcode.parse(fh.read())
    moves = [(s.x, s.y) for s in steps if s.kind == "move"]
    assert moves[:4] == [(20, 0), (20, 20), (0, 20), (0, 0)]


@pytest.mark.parametrize("text,needle", [
    ("G0 X5 Z5\nG28", "before takeoff"),
    ("M17 Z5\nG0 X500\nG28", "fence radius"),
    ("M17 Z5\nG0 Z100\nG28", "exceeds max"),
    ("M17 Z5\nG0 X1", "ends in the air"),
    ("M17 Z5\nG0 X1 F50\nG28", "speed"),
    ("M17 Z5\nG77\nG28", "unsupported"),
    ("M100", "on the ground"),
])
def test_parse_rejects_unsafe(text, needle):
    with pytest.raises(gcode.GcodeError, match=needle):
        gcode.parse(text)


def test_comments_are_ignored():
    steps = gcode.parse("; hello\nM17 Z4 ( take off )\nG28 ; home")
    assert [s.kind for s in steps] == ["takeoff", "rtl"]


# ------------------------------------------------------------------ swarm --
KEY = b"0123456789abcdef0123"


def _report(seq, t=None, lat=28.6, lon=-81.2, drone=1):
    r = swarm_protocol.target_report(drone, seq, lat, lon, 500, 28.6, -81.2, 10)
    if t is not None:
        r["t"] = t
    return r


def test_signed_packet_round_trip():
    v = swarm_protocol.Verifier(KEY)
    msg = v.decode(swarm_protocol.encode(_report(1), KEY))
    assert msg["target"] == [28.6, -81.2]


def test_tampered_or_wrong_key_rejected():
    pkt = swarm_protocol.encode(_report(1), KEY)
    with pytest.raises(ValueError, match="signature"):
        swarm_protocol.Verifier(b"another-key-entirely").decode(pkt)
    with pytest.raises(ValueError, match="signature"):
        swarm_protocol.Verifier(KEY).decode(pkt.replace(b"28.6", b"29.6"))


def test_replay_and_stale_rejected():
    v = swarm_protocol.Verifier(KEY)
    pkt = swarm_protocol.encode(_report(5), KEY)
    v.decode(pkt)
    with pytest.raises(ValueError, match="replayed"):
        v.decode(pkt)
    with pytest.raises(ValueError, match="stale"):
        v.decode(swarm_protocol.encode(_report(6, t=0.0), KEY))


def test_fusion_median_ignores_one_outlier():
    f = swarm_protocol.TargetFusion(clock=lambda: 100.0)
    f.add({"drone": 1, "t": 100.0, "target": [10.0, 20.0]})
    f.add({"drone": 2, "t": 100.0, "target": [10.0002, 20.0001]})
    f.add({"drone": 3, "t": 100.0, "target": [55.0, 99.0]})
    lat, lon, n = f.estimate()
    assert n == 3 and lat == pytest.approx(10.0002) and lon == pytest.approx(20.0001)


def test_default_formation_is_safe_and_crowded_one_is_not():
    conf = config.load(None)
    s, f = conf["safety"], geometry.Fence()
    assert check_formation(conf["swarm"]["drones"], s, f) >= s["min_separation_m"]
    crowded = [{"id": 1, "offset_ne": [0, 0], "alt_m": 8}, {"id": 2, "offset_ne": [1, 0], "alt_m": 8.5}]
    with pytest.raises(ValueError):
        check_formation(crowded, s, f)


# ------------------------------------------------------------------ guard --
def _hb(armed, gcs=False):
    return SimpleNamespace(base_mode=128 if armed else 0, type=6 if gcs else 2)


def test_guard_detects_gps_teleport_and_home_move():
    d = AnomalyDetector(config.DEFAULTS["guard"])
    d.feed("HEARTBEAT", _hb(True), 0.0, True)
    d.feed("GLOBAL_POSITION_INT", SimpleNamespace(lat=int(28.6e7), lon=int(-81.2e7)), 1.0, True)
    ev = d.feed("GLOBAL_POSITION_INT", SimpleNamespace(lat=int(28.601e7), lon=int(-81.2e7)), 1.1, True)
    assert ev and ev[0][0] == "position_jump"      # 111 m in 0.1 s
    assert d.action_for("position_jump") == "LAND"
    d.feed("HOME_POSITION", SimpleNamespace(latitude=int(28.6e7), longitude=int(-81.2e7)), 2.0, True)
    ev = d.feed("HOME_POSITION", SimpleNamespace(latitude=int(28.6005e7), longitude=int(-81.2e7)), 3.0, True)
    assert ev[0][0] == "home_changed"


def test_guard_quiet_in_normal_flight_and_when_disarmed():
    d = AnomalyDetector(config.DEFAULTS["guard"])
    gps_bad = SimpleNamespace(fix_type=1, satellites_visible=3)
    assert d.feed("GPS_RAW_INT", gps_bad, 0.0, True) == []        # disarmed: ignore
    d.feed("HEARTBEAT", _hb(True), 0.0, True)
    for i in range(20):                                           # 5 m/s flight
        lat = 28.6 + (5 * i * 0.1) / 111320
        assert d.feed("GLOBAL_POSITION_INT", SimpleNamespace(lat=int(lat * 1e7), lon=int(-81.2e7)),
                      i * 0.1, True) == []
    assert d.feed("GPS_RAW_INT", gps_bad, 3.0, True) == []        # bad for <2 s: wait
    assert d.feed("GPS_RAW_INT", gps_bad, 5.5, True)[0][0] == "gps_degraded"
