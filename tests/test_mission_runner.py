"""Runs grid_commander.run() against a fake drone (no simulator needed)."""

import pytest

pytest.importorskip("pymavlink")

from project006 import config, gcode  # noqa: E402
from project006.grid_commander import run  # noqa: E402


class FakeDrone:
    """Teleports to every target; records what the mission asked for."""

    def __init__(self, start=(3.0, -2.0), takeover_after_moves=None):
        self.mode, self.armed = "LOITER", False
        self.pos = [start[0], start[1], 0.0]   # NED (EKF origin is NOT the pad)
        self.calls = []
        self.moves = 0
        self.takeover_after_moves = takeover_after_moves

    # telemetry
    def pump(self, timeout=0.0):
        pass

    def local_ned(self):
        return tuple(self.pos)

    def rel_alt(self):
        return -self.pos[2]

    def wait_for(self, predicate, timeout, what, poll=0.05):
        assert predicate(), f"would time out waiting for {what}"

    # commands
    def set_mode(self, m):
        self.calls.append(("mode", m))
        self.mode = m
        if m in ("RTL", "LAND"):
            self.armed = False

    def arm(self):
        self.calls.append(("arm",))
        self.armed = True

    def takeoff(self, alt):
        self.calls.append(("takeoff", alt))
        self.pos[2] = -alt

    def gripper(self, release):
        self.calls.append(("release" if release else "grab",))

    def set_speed(self, s):
        self.calls.append(("speed", s))

    def goto_local(self, n, e, alt, yaw=None):
        self.calls.append(("goto", round(n, 2), round(e, 2), alt, yaw))
        self.pos = [n, e, -alt]
        self.moves += 1
        if self.takeover_after_moves and self.moves >= self.takeover_after_moves:
            self.mode = "LOITER"   # pilot flips the mode switch


def _demo_steps():
    with open("missions/delivery_demo.gcode") as f:
        return gcode.parse(f.read())


def test_demo_mission_flies_in_order_relative_to_takeoff_pad():
    d = FakeDrone()
    run(_demo_steps(), d, config.DEFAULTS["safety"], log=lambda *_: None)
    kinds = [c[0] for c in d.calls]
    assert kinds[:4] == ["mode", "grab", "arm", "takeoff"]
    assert ("goto", 18.0, -2.0, 5.0, None) in d.calls          # X15 from a pad at (3, -2)
    assert ("goto", 18.0, 8.0, 3.0, 90.0) in d.calls            # drop point
    assert kinds.index("release") > kinds.index("goto")
    assert d.calls[-1] == ("mode", "RTL") and not d.armed


def test_pilot_takeover_stops_mission_before_payload_release():
    d = FakeDrone(takeover_after_moves=3)    # pilot takes over at the drop point, before M100
    with pytest.raises(RuntimeError, match="left GUIDED"):
        run(_demo_steps(), d, config.DEFAULTS["safety"], log=lambda *_: None)
    assert ("release",) not in d.calls
